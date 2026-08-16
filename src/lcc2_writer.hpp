#ifndef PLY2LCC_LCC2_WRITER_HPP
#define PLY2LCC_LCC2_WRITER_HPP

#include "spatial_grid.hpp"
#include <filesystem>
#include <string>
#include <vector>

namespace ply2lcc {

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
               const std::vector<float>& lod_errors = {});

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
