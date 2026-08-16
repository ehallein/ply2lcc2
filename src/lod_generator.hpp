#ifndef PLY2LCC_LOD_GENERATOR_HPP
#define PLY2LCC_LOD_GENERATOR_HPP

#include "types.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace ply2lcc {

struct LodLevelStats {
    size_t input_count = 0;
    size_t output_count = 0;
    size_t cluster_count = 0;
    size_t min_cluster_size = 0;
    size_t max_cluster_size = 0;
    double average_cluster_size = 0.0;
    double mean_covariance_error = 0.0;
    float max_error = 0.0f;
    size_t rejected_merges = 0;
};

struct LodLevel {
    std::vector<Splat> splats;
    float error = 0.0f;
    LodLevelStats stats;
};

class LodGenerator {
public:
    explicit LodGenerator(LodSettings settings);

    // Returns levels in LCC2 order: LOD0 is coarsest, the last level is original.
    std::vector<LodLevel> generate(std::vector<Splat> source) const;

    // Exposed for focused tests and future clustering strategy replacement.
    static Splat merge_cluster(const std::vector<Splat>& splats,
                               const std::vector<size_t>& indices,
                               float* error = nullptr);
    static void validate(const std::vector<Splat>& splats, const std::string& label);
    static void write_binary_ply(const std::filesystem::path& path,
                                 const std::vector<Splat>& splats,
                                 int num_f_rest = 0);

private:
    LodLevel decimate(const std::vector<Splat>& source) const;
    LodLevel cluster(const std::vector<Splat>& source) const;

    LodSettings settings_;
};

} // namespace ply2lcc

#endif
