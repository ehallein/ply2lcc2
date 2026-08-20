#include "lcc2_writer.hpp"
#include "lod_generator.hpp"
#include "platform.hpp"
#include "splat_buffer.hpp"

#include <fstream>
#include <array>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <functional>
#include <random>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace ply2lcc {

namespace {

struct SpzHeaderInfo {
    uint32_t point_count;
    uint8_t sh_degree;
};

uint32_t read_u32_le(const uint8_t* bytes) {
    return static_cast<uint32_t>(bytes[0]) |
           (static_cast<uint32_t>(bytes[1]) << 8) |
           (static_cast<uint32_t>(bytes[2]) << 16) |
           (static_cast<uint32_t>(bytes[3]) << 24);
}

SpzHeaderInfo inspect_spz_v4(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    std::array<uint8_t, 32> header{};
    file.read(reinterpret_cast<char*>(header.data()), static_cast<std::streamsize>(header.size()));
    if (file.gcount() != static_cast<std::streamsize>(header.size())) {
        throw std::runtime_error("SPZ file is too short: " + path.u8string());
    }

    constexpr uint32_t NGSP_MAGIC = 0x5053474e;
    const uint32_t magic = read_u32_le(header.data());
    const uint32_t version = read_u32_le(header.data() + 4);
    const uint32_t point_count = read_u32_le(header.data() + 8);
    const uint8_t sh_degree = header[12];
    const uint8_t stream_count = header[15];
    const uint32_t toc_offset = read_u32_le(header.data() + 16);

    if (magic != NGSP_MAGIC || version != 4) {
        throw std::runtime_error("Unsupported SPZ payload (expected NGSP version 4): " + path.u8string());
    }
    if (point_count == 0 || sh_degree > 4 || stream_count == 0 || toc_offset < header.size()) {
        throw std::runtime_error("Invalid SPZ v4 header: " + path.u8string());
    }
    return {point_count, sh_degree};
}

std::string lowercase_extension(const fs::path& path) {
    std::string extension = path.extension().u8string();
    for (char& c : extension) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return extension;
}

// LCC/LCC2 payload coordinates have an additional +90 degree X-axis source
// transform compared with ordinary PLY/SPZ coordinates. Pre-rotate payload
// data by -90 degrees so loading the resulting LCC2 reproduces the input pose.
void transform_splat_to_lcc2(Splat& splat) {
    splat.pos = Vec3f(splat.pos.x, splat.pos.z, -splat.pos.y);
    splat.normal = Vec3f(splat.normal.x, splat.normal.z, -splat.normal.y);

    // q' = rotation_x(-90 degrees) * q, with quaternions stored as w,x,y,z.
    constexpr float c = 0.7071067811865475244f;
    const float w = splat.rot[0];
    const float x = splat.rot[1];
    const float y = splat.rot[2];
    const float z = splat.rot[3];
    splat.rot[0] = c * (w + x);
    splat.rot[1] = c * (x - w);
    splat.rot[2] = c * (y + z);
    splat.rot[3] = c * (z - y);
}

void write_transformed_ply(const fs::path& source, const fs::path& destination) {
    SplatBuffer buffer;
    if (!buffer.initialize(source)) {
        throw std::runtime_error("Failed to transform " + source.u8string() + ": " + buffer.error());
    }
    std::vector<Splat> splats = buffer.to_vector();
    for (Splat& splat : splats) transform_splat_to_lcc2(splat);
    LodGenerator::write_binary_ply(destination, splats, buffer.num_f_rest());
}

} // namespace

Lcc2Writer::Lcc2Writer(const fs::path& output_dir,
                       const fs::path& splat_transform_path)
    : output_dir_(output_dir), splat_transform_path_(splat_transform_path) {}

void Lcc2Writer::validate_spz_v4(const fs::path& path, size_t point_count, int sh_degree) {
    const SpzHeaderInfo spz = inspect_spz_v4(path);
    if (spz.point_count != point_count) {
        throw std::runtime_error("SPZ/PLY splat count mismatch: " + path.u8string());
    }
    if (spz.sh_degree != static_cast<uint8_t>(sh_degree)) {
        throw std::runtime_error("SPZ/PLY SH degree mismatch: " + path.u8string());
    }
}

std::string Lcc2Writer::generate_guid() {
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_int_distribution<int> hex_digit(0, 15);
    std::ostringstream result;
    result << std::hex;
    for (int i = 0; i < 32; ++i) {
        result << hex_digit(generator);
    }
    return result.str();
}

std::string Lcc2Writer::generate_hierarchy_guid(const std::string& name,
                                                 const Lcc2HierarchyInfo& hierarchy) {
    std::ostringstream canonical;
    canonical << std::setprecision(15) << name << '|' << hierarchy.level_count;
    for (const Lcc2HierarchyNodeInfo& node : hierarchy.nodes) {
        canonical << '|' << node.id << ',' << node.level << ',' << node.payload_index << ','
                  << node.start << ',' << node.count << ',' << node.error;
        for (int axis = 0; axis < 3; ++axis) canonical << ',' << node.bounds.min[axis] << ',' << node.bounds.max[axis];
        for (size_t child : node.children) canonical << ',' << child;
    }
    const std::string bytes = canonical.str();
    auto fnv1a = [&](uint64_t hash) {
        for (unsigned char byte : bytes) {
            hash ^= byte;
            hash *= 1099511628211ULL;
        }
        return hash;
    };
    std::ostringstream result;
    result << std::hex << std::setfill('0') << std::setw(16) << fnv1a(1469598103934665603ULL)
           << std::setw(16) << fnv1a(1099511628211ULL);
    return result.str();
}

std::string Lcc2Writer::json_escape(const std::string& value) {
    std::ostringstream escaped;
    for (unsigned char c : value) {
        switch (c) {
        case '\"': escaped << "\\\""; break;
        case '\\': escaped << "\\\\"; break;
        case '\b': escaped << "\\b"; break;
        case '\f': escaped << "\\f"; break;
        case '\n': escaped << "\\n"; break;
        case '\r': escaped << "\\r"; break;
        case '\t': escaped << "\\t"; break;
        default:
            if (c < 0x20) {
                escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(c) << std::dec;
            } else {
                escaped << static_cast<char>(c);
            }
        }
    }
    return escaped.str();
}

void Lcc2Writer::write_bbox(std::ostream& stream, const BBox& bbox, int indent) {
    const std::string pad(static_cast<size_t>(indent), '\t');
    stream << "{\n"
           << pad << "\t\"min\": [" << bbox.min.x << ", " << bbox.min.y << ", " << bbox.min.z << "],\n"
           << pad << "\t\"max\": [" << bbox.max.x << ", " << bbox.max.y << ", " << bbox.max.z << "]\n"
           << pad << "}";
}

BBox Lcc2Writer::to_lcc2_bbox(const BBox& bbox) {
    BBox result;
    result.min = Vec3f(bbox.min.x, bbox.min.z, -bbox.max.y);
    result.max = Vec3f(bbox.max.x, bbox.max.z, -bbox.min.y);
    return result;
}

void Lcc2Writer::write(const SpatialGrid& grid,
                       const std::vector<fs::path>& lod_files,
                       const std::vector<fs::path>& payload_files,
                       const fs::path& environment_file,
                       const std::string& name,
                       const std::vector<float>& lod_errors,
                       const Lcc2HierarchyInfo* hierarchy) {
    if (lod_files.empty()) {
        throw std::runtime_error("LCC2 output requires at least one 3DGS PLY file");
    }

    std::vector<fs::path> payloads = payload_files.empty() ? lod_files : payload_files;
    std::vector<fs::path> payload_plys = lod_files;
    if (hierarchy) {
        if (hierarchy->level_count == 0 || hierarchy->nodes.empty() ||
            hierarchy->roots.empty() || hierarchy->payloads.empty()) {
            throw std::runtime_error("Generated LCC2 hierarchy is incomplete");
        }
        payloads.clear();
        payload_plys.clear();
        for (const Lcc2PayloadInfo& payload : hierarchy->payloads) {
            payloads.push_back(payload.payload_file);
            payload_plys.push_back(payload.ply_file);
        }
        std::vector<std::vector<std::pair<size_t, size_t>>> ranges(hierarchy->payloads.size());
        auto bounds_overlap_in_volume = [](const BBox& a, const BBox& b) {
            for (int axis = 0; axis < 3; ++axis) {
                if (!(std::min(a.max[axis], b.max[axis]) >
                      std::max(a.min[axis], b.min[axis]))) return false;
            }
            return true;
        };
        for (size_t i = 0; i < hierarchy->roots.size(); ++i) {
            if (hierarchy->roots[i] >= hierarchy->nodes.size()) {
                throw std::runtime_error("Generated hierarchy has an invalid root reference");
            }
            for (size_t j = i + 1; j < hierarchy->roots.size(); ++j) {
                if (hierarchy->roots[j] >= hierarchy->nodes.size()) {
                    throw std::runtime_error("Generated hierarchy has an invalid root reference");
                }
                if (!hierarchy->allow_spatial_bound_overlap &&
                    bounds_overlap_in_volume(hierarchy->nodes[hierarchy->roots[i]].bounds,
                                             hierarchy->nodes[hierarchy->roots[j]].bounds)) {
                    throw std::runtime_error("Generated hierarchy root bounds overlap in volume");
                }
            }
        }
        for (const Lcc2HierarchyNodeInfo& node : hierarchy->nodes) {
            if (node.count == 0 || node.payload_index >= ranges.size()) {
                throw std::runtime_error("Generated hierarchy contains an empty node or invalid payload reference");
            }
            ranges[node.payload_index].push_back({node.start, node.count});
            for (size_t i = 0; i < node.children.size(); ++i) {
                for (size_t j = i + 1; j < node.children.size(); ++j) {
                    if (node.children[i] >= hierarchy->nodes.size() ||
                        node.children[j] >= hierarchy->nodes.size()) {
                        throw std::runtime_error("Generated hierarchy has an invalid child reference");
                    }
                    const BBox& a = hierarchy->nodes[node.children[i]].bounds;
                    const BBox& b = hierarchy->nodes[node.children[j]].bounds;
                    if (!hierarchy->allow_spatial_bound_overlap && bounds_overlap_in_volume(a, b)) {
                        throw std::runtime_error("Generated hierarchy sibling bounds overlap in volume");
                    }
                }
            }
            for (size_t child_id : node.children) {
                if (child_id >= hierarchy->nodes.size() ||
                    hierarchy->nodes[child_id].level <= node.level) {
                    throw std::runtime_error("Generated hierarchy has an invalid parent/child level transition");
                }
                const Lcc2HierarchyNodeInfo& child = hierarchy->nodes[child_id];
                constexpr float tolerance = 1e-4f;
                for (int axis = 0; axis < 3; ++axis) {
                    if (child.bounds.min[axis] < node.bounds.min[axis] - tolerance ||
                        child.bounds.max[axis] > node.bounds.max[axis] + tolerance) {
                        throw std::runtime_error("Generated hierarchy child bounds escape parent bounds");
                    }
                }
                if (!std::isfinite(node.error) || node.error < 0.0f ||
                    node.error + tolerance < child.error) {
                    throw std::runtime_error("Generated hierarchy LOD errors are invalid or non-monotonic");
                }
            }
        }
        for (size_t payload_index = 0; payload_index < ranges.size(); ++payload_index) {
            auto& payload_ranges = ranges[payload_index];
            std::sort(payload_ranges.begin(), payload_ranges.end());
            size_t cursor = 0;
            for (const auto& range : payload_ranges) {
                if (range.first != cursor) {
                    throw std::runtime_error("Generated hierarchy payload ranges are not contiguous");
                }
                cursor += range.second;
            }
            if (cursor != hierarchy->payloads[payload_index].splat_count) {
                throw std::runtime_error("Generated hierarchy payload ranges do not cover the payload exactly");
            }
        }
    } else if (payloads.size() != lod_files.size()) {
        throw std::runtime_error("LCC2 payload count must match the number of LOD files");
    }
    if (!lod_errors.empty() && lod_errors.size() != lod_files.size()) {
        throw std::runtime_error("LOD error count must match the number of LOD files");
    }

    const std::string splat_extension = lowercase_extension(payloads.front());
    if (splat_extension != ".ply" && splat_extension != ".spz") {
        throw std::runtime_error("Unsupported LCC2 splat payload type: " + splat_extension);
    }
    for (const auto& payload : payloads) {
        if (lowercase_extension(payload) != splat_extension) {
            throw std::runtime_error("All LCC2 LOD payloads must use the same file type");
        }
    }
    if (!environment_file.empty() && splat_extension != ".ply") {
        throw std::runtime_error("An environment PLY cannot be mixed with SPZ LCC2 payloads");
    }

    const fs::path splat_dir = output_dir_ / "data" / "3dgs";
    fs::create_directories(splat_dir);

    std::vector<size_t> lod_counts;
    std::vector<std::string> relative_files;
    lod_counts.reserve(hierarchy ? hierarchy->level_count : lod_files.size());
    relative_files.reserve(payloads.size() + 1);

    if (hierarchy) lod_counts = hierarchy->splats_per_level;

    size_t total_splats = 0;
    for (size_t payload_index = 0; payload_index < payloads.size(); ++payload_index) {
        SplatBuffer splats;
        if (!splats.initialize(payload_plys[payload_index])) {
            throw std::runtime_error("Failed to inspect " + payload_plys[payload_index].u8string() + ": " + splats.error());
        }

        if (splat_extension == ".spz") {
            validate_spz_v4(payloads[payload_index], splats.size(), splats.sh_degree());
        }

        const std::string filename = hierarchy
            ? payloads[payload_index].stem().u8string() + splat_extension
            : "lod_" + std::to_string(payload_index) + splat_extension;
        const fs::path destination = splat_dir / filename;
        if (splat_extension == ".spz") {
            const std::vector<fs::path> command{
                splat_transform_path_, "--quiet", "--overwrite", "--spz-version", "4",
                // splat-transform rotations are expressed in engine space. Its
                // PLY/SPZ source transform includes a 180-degree Z rotation, so
                // +90 here produces the required -90 X rotation in raw payload space.
                payloads[payload_index], "--rotate", "90,0,0", destination
            };
            const int exit_code = platform::run_process(command);
            if (exit_code != 0) {
                throw std::runtime_error(
                    "splat-transform failed while rotating LCC2 payload " + payloads[payload_index].u8string() +
                    " (exit code " + std::to_string(exit_code) + ")");
            }
            validate_spz_v4(destination, splats.size(), splats.sh_degree());
        } else {
            write_transformed_ply(payloads[payload_index], destination);
        }
        relative_files.push_back("data/3dgs/" + filename);
        if (!hierarchy) lod_counts.push_back(splats.size());
    }
    for (size_t count : lod_counts) total_splats += count;

    bool has_environment = !environment_file.empty() && fs::exists(environment_file);
    size_t environment_count = 0;
    BBox environment_bbox;
    size_t environment_file_index = 0;
    if (has_environment) {
        SplatBuffer environment;
        if (!environment.initialize(environment_file)) {
            throw std::runtime_error("Failed to inspect " + environment_file.u8string() + ": " + environment.error());
        }
        environment_count = environment.size();
        environment_bbox = to_lcc2_bbox(environment.compute_bbox());
        environment_file_index = relative_files.size();
        write_transformed_ply(environment_file, splat_dir / "environment.ply");
        relative_files.push_back("data/3dgs/environment.ply");
    }

    auto metadata = platform::ofstream_open(output_dir_ / "meta.lcc2", std::ios::out);
    if (!metadata) {
        throw std::runtime_error("Failed to create meta.lcc2");
    }
    metadata << std::setprecision(15);

    metadata << "{\n"
             << "\t\"version\": \"0.0.3\",\n"
             << "\t\"name\": \"" << json_escape(name) << "\",\n"
             << "\t\"description\": \"Converted from PLY by ply2lcc; LCC2 data organization format originated from XGRIDS\",\n"
             << "\t\"epsg\": 0,\n"
             << "\t\"guid\": \"" << (hierarchy ? generate_hierarchy_guid(name, *hierarchy) : generate_guid()) << "\",\n"
             << "\t\"source\": \"ply2lcc\",\n"
             << "\t\"dataType\": \"3DGS\",\n"
             << "\t\"offset\": [0, 0, 0],\n"
             << "\t\"shift\": [0, 0, 0],\n"
             << "\t\"scale\": [1, 1, 1],\n"
             << "\t\"fileType\": \"" << (grid.has_sh() ? "quality" : "portable") << "\",\n"
             << "\t\"totalSplats\": " << total_splats << ",\n"
             << "\t\"lodSplats\": [";
    for (size_t i = 0; i < lod_counts.size(); ++i) {
        if (i != 0) metadata << ", ";
        const size_t lod = (lod_errors.empty() && !hierarchy) ? i : lod_counts.size() - 1 - i;
        metadata << lod_counts[lod];
    }
    metadata << "],\n"
             << "\t\"totalLevels\": " << lod_counts.size() << ",\n"
             << "\t\"virtualLoD\": null,\n";
    if (!lod_errors.empty() || hierarchy) {
        const std::vector<float>& errors = hierarchy ? hierarchy->errors_per_level : lod_errors;
        metadata << "\t\"lodErrors\": [";
        for (size_t i = 0; i < errors.size(); ++i) {
            if (i != 0) metadata << ", ";
            metadata << errors[errors.size() - 1 - i];
        }
        metadata << "],\n";
    }
    metadata
             << "\t\"splatType\": \"" << splat_extension << "\",\n";

    if (has_environment) {
        metadata << "\t\"env\": {\n"
                 << "\t\t\"type\": \"splats\",\n"
                 << "\t\t\"splatsCount\": " << environment_count << ",\n"
                 << "\t\t\"boundingBox\": ";
        write_bbox(metadata, environment_bbox, 2);
        metadata << "\n\t},\n";
    } else {
        metadata << "\t\"env\": null,\n";
    }

    metadata << "\t\"splatExtraAttributes\": null,\n"
             << "\t\"root\": {\n"
             << "\t\t\"id\": \"0\",\n"
             << "\t\t\"boundingBox\": ";
    const BBox lcc2_scene_bbox = to_lcc2_bbox(hierarchy ? hierarchy->bounds : grid.bbox());
    write_bbox(metadata, lcc2_scene_bbox, 2);
    metadata << ",\n"
             << "\t\t\"childNum\": " << (hierarchy ? hierarchy->roots.size() :
                    (lod_errors.empty() ? lod_files.size() : grid.cells().size())) << ",\n"
             << "\t\t\"data\": ";
    if (has_environment) {
        metadata << "{\"env\": {\"name\": " << environment_file_index << "}}";
    } else {
        metadata << "null";
    }
    metadata << ",\n"
             << "\t\t\"splatFiles\": [";
    for (size_t i = 0; i < relative_files.size(); ++i) {
        if (i != 0) metadata << ", ";
        metadata << "\"" << json_escape(relative_files[i]) << "\"";
    }
    metadata << "],\n"
             << "\t\t\"child\": {\n";

    if (hierarchy) {
        std::function<void(size_t, int)> write_node;
        write_node = [&](size_t node_id, int indent) {
            const Lcc2HierarchyNodeInfo& node = hierarchy->nodes[node_id];
            if (node.payload_index >= hierarchy->payloads.size() ||
                node.start + node.count > hierarchy->payloads[node.payload_index].splat_count) {
                throw std::runtime_error("Generated hierarchy node range is outside its payload");
            }
            const std::string pad(static_cast<size_t>(indent), '\t');
            metadata << "{\n" << pad << "\t\"id\": \"node-" << node.id << "\",\n"
                     << pad << "\t\"boundingBox\": ";
            write_bbox(metadata, to_lcc2_bbox(node.bounds), indent + 1);
            metadata << ",\n" << pad << "\t\"lodLevel\": " << (hierarchy->level_count - 1 - node.level)
                     << ",\n" << pad << "\t\"lodError\": " << node.error
                     << ",\n" << pad << "\t\"childNum\": " << node.children.size()
                     << ",\n" << pad << "\t\"data\": {\"3dgs\": {\"name\": " << node.payload_index
                     << ", \"start\": " << node.start << ", \"count\": " << node.count << "}},\n"
                     << pad << "\t\"child\": {";
            if (!node.children.empty()) {
                metadata << "\n";
                for (size_t i = 0; i < node.children.size(); ++i) {
                    metadata << pad << "\t\t\"" << i << "\": ";
                    write_node(node.children[i], indent + 2);
                    metadata << (i + 1 == node.children.size() ? "\n" : ",\n");
                }
                metadata << pad << "\t";
            }
            metadata << "}\n" << pad << "}";
        };
        for (size_t root_index = 0; root_index < hierarchy->roots.size(); ++root_index) {
            metadata << "\t\t\t\"" << root_index << "\": ";
            write_node(hierarchy->roots[root_index], 3);
            metadata << (root_index + 1 == hierarchy->roots.size() ? "\n" : ",\n");
        }
    } else if (lod_errors.empty()) {
        for (size_t lod = 0; lod < lod_files.size(); ++lod) {
            metadata << "\t\t\t\"" << lod << "\": {\n"
                     << "\t\t\t\t\"id\": \"0-" << lod << "\",\n"
                     << "\t\t\t\t\"boundingBox\": ";
            write_bbox(metadata, lcc2_scene_bbox, 4);
            metadata << ",\n"
                     << "\t\t\t\t\"childNum\": 0,\n"
                     << "\t\t\t\t\"data\": {\"3dgs\": {\"name\": " << lod
                     << ", \"start\": 0, \"count\": " << lod_counts[lod] << "}}\n"
                     << "\t\t\t}" << (lod + 1 == lod_files.size() ? "\n" : ",\n");
        }
    } else {
        size_t cell_number = 0;
        for (const auto& cell_pair : grid.cells()) {
            const uint32_t cell_id = cell_pair.first;
            const GridCell& cell = cell_pair.second;
            const uint32_t cell_x = cell_id & 0xffffU;
            const uint32_t cell_y = cell_id >> 16;
            BBox cell_bbox;
            cell_bbox.min = Vec3f(grid.bbox().min.x + cell_x * grid.cell_size_x(),
                                  grid.bbox().min.y + cell_y * grid.cell_size_y(), grid.bbox().min.z);
            cell_bbox.max = Vec3f(std::min(grid.bbox().max.x, cell_bbox.min.x + grid.cell_size_x()),
                                  std::min(grid.bbox().max.y, cell_bbox.min.y + grid.cell_size_y()), grid.bbox().max.z);
            std::vector<size_t> present;
            for (size_t lod = 0; lod < lod_files.size(); ++lod) {
                if (!cell.splat_indices[lod].empty()) present.push_back(lod);
            }
            metadata << "\t\t\t\"" << cell_number << "\": {\n"
                     << "\t\t\t\t\"id\": \"cell-" << cell_id << "\",\n"
                     << "\t\t\t\t\"boundingBox\": ";
            cell_bbox = to_lcc2_bbox(cell_bbox);
            write_bbox(metadata, cell_bbox, 4);
            if (present.empty()) {
                metadata << ",\n\t\t\t\t\"childNum\": 0,\n"
                         << "\t\t\t\t\"data\": null\n";
            } else {
                std::function<void(size_t, int)> write_lod_fields;
                write_lod_fields = [&](size_t depth, int indent) {
                    const size_t lod = present[depth];
                    const auto& indices = cell.splat_indices[lod];
                    for (size_t i = 1; i < indices.size(); ++i) {
                        if (indices[i] != indices[i - 1] + 1) {
                            throw std::runtime_error("Generated LOD payload is not contiguous within spatial cell");
                        }
                    }
                    const std::string pad(static_cast<size_t>(indent), '\t');
                    metadata << ",\n" << pad << "\"lodLevel\": " << (lod_files.size() - 1 - lod)
                             << ",\n" << pad << "\"lodError\": " << lod_errors[lod]
                             << ",\n" << pad << "\"childNum\": " << (depth + 1 < present.size() ? 1 : 0)
                             << ",\n" << pad << "\"data\": {\"3dgs\": {\"name\": " << lod
                             << ", \"start\": " << indices.front() << ", \"count\": " << indices.size() << "}}";
                    if (depth + 1 < present.size()) {
                        const size_t child_lod = present[depth + 1];
                        metadata << ",\n" << pad << "\"child\": {\n"
                                 << pad << "\t\"" << child_lod << "\": {\n"
                                 << pad << "\t\t\"id\": \"cell-" << cell_id << "-lod-" << child_lod << "\",\n"
                                 << pad << "\t\t\"boundingBox\": ";
                        write_bbox(metadata, cell_bbox, indent + 2);
                        write_lod_fields(depth + 1, indent + 2);
                        metadata << "\n" << pad << "\t}\n" << pad << "}";
                    }
                };
                write_lod_fields(0, 4);
                metadata << "\n";
            }
            metadata << "\t\t\t}" << (++cell_number == grid.cells().size() ? "\n" : ",\n");
        }
    }

    metadata << "\t\t}\n"
             << "\t},\n"
             << "\t\"renderingHints\": {\n"
             << "\t\t\"renderMethod\": \"splatting\",\n"
             << "\t\t\"renderMethodVariant\": \"ewa\",\n"
             << "\t\t\"sortingMethod\": \"depth\",\n"
             << "\t\t\"cameraModel\": \"pinhole\"\n"
             << "\t}\n"
             << "}\n";

    if (!metadata) {
        throw std::runtime_error("Failed while writing meta.lcc2");
    }

    auto notice = platform::ofstream_open(output_dir_ / "LCC2-NOTICE.md", std::ios::out);
    if (!notice) {
        throw std::runtime_error("Failed to create LCC2-NOTICE.md");
    }
    notice
        << "# LCC2 format notice\n\n"
        << "The LCC2 data organization format originated from XGRIDS.\n\n"
        << "This package was created by the independent ply2lcc implementation and "
           "is not an official XGRIDS implementation. It modifies the organization "
           "described by the whitepaper by mapping supplied levels into spatial nodes "
           "beneath the root.\n\n"
        << "Specification and license: https://github.com/xgrids/LCC2Whitepaper\n\n"
        << "Use and redistribution of this package are subject to the terms in that whitepaper.\n";
    if (!notice) {
        throw std::runtime_error("Failed while writing LCC2-NOTICE.md");
    }
}

} // namespace ply2lcc
