#ifndef PLY2LCC_LCC2_WRITER_HPP
#define PLY2LCC_LCC2_WRITER_HPP

#include "spatial_grid.hpp"
#include <filesystem>
#include <string>
#include <vector>

namespace ply2lcc {

struct Lcc2PayloadInfo {
    std::filesystem::path ply_file;
    std::filesystem::path payload_file;
    size_t level = 0;
    size_t splat_count = 0;
};

struct Lcc2HierarchyNodeInfo {
    size_t level = 0;
    size_t payload_index = 0;
    size_t start = 0;
    size_t count = 0;
    float error = 0.0f;
};

struct Lcc2HierarchyLeafInfo {
    size_t id = 0;
    BBox bounds;
    std::vector<Lcc2HierarchyNodeInfo> nodes;
};

struct Lcc2HierarchyInfo {
    size_t level_count = 0;
    std::vector<size_t> splats_per_level;
    std::vector<float> errors_per_level;
    std::vector<Lcc2PayloadInfo> payloads;
    std::vector<Lcc2HierarchyLeafInfo> leaves;
    BBox bounds;
};

class Lcc2Writer {
public:
    explicit Lcc2Writer(const std::filesystem::path& output_dir,
                        const std::filesystem::path& splat_transform_path = "splat-transform");

    static void validate_spz_v4(const std::filesystem::path& path,
                                size_t point_count, int sh_degree);

    void write(const SpatialGrid& grid,
               const std::vector<std::filesystem::path>& lod_files,
               const std::vector<std::filesystem::path>& payload_files,
               const std::filesystem::path& environment_file,
               const std::string& name,
               const std::vector<float>& lod_errors = {},
               const Lcc2HierarchyInfo* hierarchy = nullptr);

private:
    static std::string generate_guid();
    static std::string json_escape(const std::string& value);
    static void write_bbox(std::ostream& stream, const BBox& bbox, int indent);
    static BBox to_lcc2_bbox(const BBox& bbox);

    std::filesystem::path output_dir_;
    std::filesystem::path splat_transform_path_;
};

} // namespace ply2lcc

#endif // PLY2LCC_LCC2_WRITER_HPP
