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
    size_t id = 0;
    size_t level = 0;
    size_t payload_index = 0;
    size_t start = 0;
    size_t count = 0;
    float error = 0.0f;
    BBox bounds;
    std::vector<size_t> children;
};

struct Lcc2HierarchyInfo {
    size_t level_count = 0;
    std::vector<size_t> splats_per_level;
    std::vector<float> errors_per_level;
    std::vector<Lcc2PayloadInfo> payloads;
    std::vector<Lcc2HierarchyNodeInfo> nodes;
    std::vector<size_t> roots;
    BBox bounds;
    // Supplied LODs use conservative Gaussian support bounds. Adjacent
    // spatial regions may therefore overlap even though centre ownership does
    // not. Generated hierarchies continue to require disjoint cell bounds.
    bool allow_spatial_bound_overlap = false;
    // Supplied LOD compatibility requires one unary node at every rank for
    // every root so runtimes never substitute a finer mandatory fallback.
    bool require_complete_unary_rank_chains = false;
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
    static std::string generate_hierarchy_guid(const std::string& name,
                                               const Lcc2HierarchyInfo& hierarchy);
    static std::string json_escape(const std::string& value);
    static void write_bbox(std::ostream& stream, const BBox& bbox, int indent);
    static BBox to_lcc2_bbox(const BBox& bbox);

    std::filesystem::path output_dir_;
    std::filesystem::path splat_transform_path_;
};

} // namespace ply2lcc

#endif // PLY2LCC_LCC2_WRITER_HPP
