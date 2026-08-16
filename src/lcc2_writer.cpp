#include "lcc2_writer.hpp"
#include "lod_generator.hpp"
#include "platform.hpp"
#include "splat_buffer.hpp"

#include <fstream>
#include <array>
#include <cctype>
#include <cstdint>
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
                       const std::vector<float>& lod_errors) {
    if (lod_files.empty()) {
        throw std::runtime_error("LCC2 output requires at least one 3DGS PLY file");
    }

    const std::vector<fs::path>& payloads = payload_files.empty() ? lod_files : payload_files;
    if (payloads.size() != lod_files.size()) {
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
    lod_counts.reserve(lod_files.size());
    relative_files.reserve(lod_files.size() + 1);

    size_t total_splats = 0;
    for (size_t lod = 0; lod < lod_files.size(); ++lod) {
        SplatBuffer splats;
        if (!splats.initialize(lod_files[lod])) {
            throw std::runtime_error("Failed to inspect " + lod_files[lod].u8string() + ": " + splats.error());
        }

        if (splat_extension == ".spz") {
            validate_spz_v4(payloads[lod], splats.size(), splats.sh_degree());
        }

        const std::string filename = "lod_" + std::to_string(lod) + splat_extension;
        const fs::path destination = splat_dir / filename;
        if (splat_extension == ".spz") {
            const std::vector<fs::path> command{
                splat_transform_path_, "--quiet", "--overwrite", "--spz-version", "4",
                // splat-transform rotations are expressed in engine space. Its
                // PLY/SPZ source transform includes a 180-degree Z rotation, so
                // +90 here produces the required -90 X rotation in raw payload space.
                payloads[lod], "--rotate", "90,0,0", destination
            };
            const int exit_code = platform::run_process(command);
            if (exit_code != 0) {
                throw std::runtime_error(
                    "splat-transform failed while rotating LCC2 payload " + payloads[lod].u8string() +
                    " (exit code " + std::to_string(exit_code) + ")");
            }
            validate_spz_v4(destination, splats.size(), splats.sh_degree());
        } else {
            write_transformed_ply(payloads[lod], destination);
        }
        relative_files.push_back("data/3dgs/" + filename);
        lod_counts.push_back(splats.size());
        total_splats += splats.size();
    }

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
             << "\t\"guid\": \"" << generate_guid() << "\",\n"
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
        const size_t lod = lod_errors.empty() ? i : lod_counts.size() - 1 - i;
        metadata << lod_counts[lod];
    }
    metadata << "],\n"
             << "\t\"totalLevels\": " << lod_counts.size() << ",\n"
             << "\t\"virtualLoD\": null,\n";
    if (!lod_errors.empty()) {
        metadata << "\t\"lodErrors\": [";
        for (size_t i = 0; i < lod_errors.size(); ++i) {
            if (i != 0) metadata << ", ";
            metadata << lod_errors[lod_errors.size() - 1 - i];
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
    const BBox lcc2_scene_bbox = to_lcc2_bbox(grid.bbox());
    write_bbox(metadata, lcc2_scene_bbox, 2);
    metadata << ",\n"
             << "\t\t\"childNum\": " << (lod_errors.empty() ? lod_files.size() : grid.cells().size()) << ",\n"
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

    if (lod_errors.empty()) {
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
