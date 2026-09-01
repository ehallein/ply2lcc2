#include "lod_generator.hpp"
#include "platform.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace ply2lcc {
namespace {

constexpr double kEpsilon = 1e-8;
constexpr float kShC0 = 0.28209479177387814f;
constexpr float kColourThreshold = 0.30f;
constexpr float kSpatialThresholdFactor = 2.5f;
// Largest bucket extent, as a multiple of the bucket members' own mean scale,
// that cluster_to_count() is still willing to merge into a single Gaussian.
// merge_cluster() folds member separation into the merged covariance, so
// merging Gaussians that do not overlap yields a representative as wide as the
// gap between them. Across a rank ladder each level would then compound the
// previous level's inflation. A ladder that halves the count per rank only
// needs about 1.41x linear growth per level to hold coverage constant, so this
// leaves legitimate LOD growth untouched while refusing the pathological case.
constexpr double kMergeExtentFactor = 4.0;
// Floor for the above, so buckets of very small Gaussians can still merge.
constexpr double kMinMergeExtent = 0.05;
// Gaussian support used for conservative hierarchy bounds. Three standard
// deviations contains 99.7% of a one-dimensional Gaussian. We use the maximum
// principal scale on every world axis, which remains conservative under rotation.
constexpr float kBoundsSigma = 3.0f;

struct Mat3 {
    double v[3][3]{};
};

struct OrderedSplat {
    uint64_t morton;
    size_t index;
};

struct WeightedError {
    float value;
    double weight;
};

double sqr(double value) { return value * value; }

Vec3f rgb(const Splat& splat) {
    return Vec3f(clamp(0.5f + kShC0 * splat.f_dc[0], 0.0f, 1.0f),
                 clamp(0.5f + kShC0 * splat.f_dc[1], 0.0f, 1.0f),
                 clamp(0.5f + kShC0 * splat.f_dc[2], 0.0f, 1.0f));
}

float colour_distance(const Vec3f& a, const Vec3f& b) {
    return std::sqrt(static_cast<float>(sqr(a.x - b.x) + sqr(a.y - b.y) + sqr(a.z - b.z)));
}

float max_scale(const Splat& splat) {
    return std::max({std::exp(splat.scale.x), std::exp(splat.scale.y), std::exp(splat.scale.z)});
}

double importance(const Splat& splat) {
    const double sx = std::exp(splat.scale.x);
    const double sy = std::exp(splat.scale.y);
    const double sz = std::exp(splat.scale.z);
    const double area = std::max({sx * sy, sx * sz, sy * sz});
    return std::max(kEpsilon, static_cast<double>(sigmoid(splat.opacity)) * area);
}

Mat3 covariance(const Splat& splat) {
    double w = splat.rot[0], x = splat.rot[1], y = splat.rot[2], z = splat.rot[3];
    const double norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (norm > kEpsilon) {
        w /= norm; x /= norm; y /= norm; z /= norm;
    } else {
        w = 1.0; x = y = z = 0.0;
    }
    const double r[3][3] = {
        {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)},
        {2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)},
        {2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)}
    };
    const double d[3] = {sqr(std::exp(splat.scale.x)),
                         sqr(std::exp(splat.scale.y)),
                         sqr(std::exp(splat.scale.z))};
    Mat3 result;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            for (int axis = 0; axis < 3; ++axis) {
                result.v[row][col] += r[row][axis] * d[axis] * r[col][axis];
            }
        }
    }
    return result;
}

void eigen_symmetric(Mat3 matrix, double values[3], Mat3& vectors) {
    for (int i = 0; i < 3; ++i) vectors.v[i][i] = 1.0;
    for (int iteration = 0; iteration < 32; ++iteration) {
        int p = 0, q = 1;
        double largest = std::abs(matrix.v[0][1]);
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                if (std::abs(matrix.v[i][j]) > largest) {
                    largest = std::abs(matrix.v[i][j]); p = i; q = j;
                }
            }
        }
        if (largest < 1e-12) break;
        const double angle = 0.5 * std::atan2(2.0 * matrix.v[p][q],
                                              matrix.v[q][q] - matrix.v[p][p]);
        const double c = std::cos(angle), s = std::sin(angle);
        for (int k = 0; k < 3; ++k) {
            const double mkp = matrix.v[k][p], mkq = matrix.v[k][q];
            matrix.v[k][p] = c * mkp - s * mkq;
            matrix.v[k][q] = s * mkp + c * mkq;
        }
        for (int k = 0; k < 3; ++k) {
            const double mpk = matrix.v[p][k], mqk = matrix.v[q][k];
            matrix.v[p][k] = c * mpk - s * mqk;
            matrix.v[q][k] = s * mpk + c * mqk;
        }
        matrix.v[p][q] = matrix.v[q][p] = 0.0;
        for (int k = 0; k < 3; ++k) {
            const double vkp = vectors.v[k][p], vkq = vectors.v[k][q];
            vectors.v[k][p] = c * vkp - s * vkq;
            vectors.v[k][q] = s * vkp + c * vkq;
        }
    }
    for (int i = 0; i < 3; ++i) values[i] = std::max(kEpsilon, matrix.v[i][i]);

    std::array<int, 3> order{0, 1, 2};
    std::sort(order.begin(), order.end(), [&](int a, int b) { return values[a] > values[b]; });
    double sorted_values[3];
    Mat3 sorted_vectors;
    for (int col = 0; col < 3; ++col) {
        sorted_values[col] = values[order[col]];
        for (int row = 0; row < 3; ++row) sorted_vectors.v[row][col] = vectors.v[row][order[col]];
    }
    std::copy(sorted_values, sorted_values + 3, values);
    vectors = sorted_vectors;

    const double det =
        vectors.v[0][0] * (vectors.v[1][1] * vectors.v[2][2] - vectors.v[1][2] * vectors.v[2][1]) -
        vectors.v[0][1] * (vectors.v[1][0] * vectors.v[2][2] - vectors.v[1][2] * vectors.v[2][0]) +
        vectors.v[0][2] * (vectors.v[1][0] * vectors.v[2][1] - vectors.v[1][1] * vectors.v[2][0]);
    if (det < 0.0) for (int row = 0; row < 3; ++row) vectors.v[row][2] = -vectors.v[row][2];
}

Quat matrix_to_quaternion(const Mat3& m) {
    Quat q;
    const double trace = m.v[0][0] + m.v[1][1] + m.v[2][2];
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = static_cast<float>(0.25 * s);
        q.x = static_cast<float>((m.v[2][1] - m.v[1][2]) / s);
        q.y = static_cast<float>((m.v[0][2] - m.v[2][0]) / s);
        q.z = static_cast<float>((m.v[1][0] - m.v[0][1]) / s);
    } else {
        int i = 0;
        if (m.v[1][1] > m.v[0][0]) i = 1;
        if (m.v[2][2] > m.v[i][i]) i = 2;
        const int j = (i + 1) % 3, k = (i + 2) % 3;
        const double s = std::sqrt(std::max(kEpsilon, 1.0 + m.v[i][i] - m.v[j][j] - m.v[k][k])) * 2.0;
        double components[4]{};
        components[i + 1] = 0.25 * s;
        components[0] = (m.v[k][j] - m.v[j][k]) / s;
        components[j + 1] = (m.v[j][i] + m.v[i][j]) / s;
        components[k + 1] = (m.v[k][i] + m.v[i][k]) / s;
        q = Quat(static_cast<float>(components[0]), static_cast<float>(components[1]),
                 static_cast<float>(components[2]), static_cast<float>(components[3]));
    }
    const float norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm > 0.0f) { q.w /= norm; q.x /= norm; q.y /= norm; q.z /= norm; }
    return q;
}

uint64_t expand_bits(uint32_t value) {
    uint64_t x = value & 0x1fffffU;
    x = (x | x << 32) & 0x1f00000000ffffULL;
    x = (x | x << 16) & 0x1f0000ff0000ffULL;
    x = (x | x << 8) & 0x100f00f00f00f00fULL;
    x = (x | x << 4) & 0x10c30c30c30c30c3ULL;
    x = (x | x << 2) & 0x1249249249249249ULL;
    return x;
}

std::vector<OrderedSplat> morton_order(const std::vector<Splat>& splats) {
    BBox bounds;
    for (const Splat& splat : splats) bounds.expand(splat.pos);
    const double extent[3] = {std::max(kEpsilon, static_cast<double>(bounds.max.x - bounds.min.x)),
                              std::max(kEpsilon, static_cast<double>(bounds.max.y - bounds.min.y)),
                              std::max(kEpsilon, static_cast<double>(bounds.max.z - bounds.min.z))};
    std::vector<OrderedSplat> result;
    result.reserve(splats.size());
    for (size_t i = 0; i < splats.size(); ++i) {
        uint32_t q[3];
        for (int axis = 0; axis < 3; ++axis) {
            const double normalized = clamp(static_cast<float>((splats[i].pos[axis] - bounds.min[axis]) / extent[axis]), 0.0f, 1.0f);
            q[axis] = static_cast<uint32_t>(normalized * 2097151.0);
        }
        result.push_back({expand_bits(q[0]) | (expand_bits(q[1]) << 1) | (expand_bits(q[2]) << 2), i});
    }
    std::stable_sort(result.begin(), result.end(), [](const OrderedSplat& a, const OrderedSplat& b) {
        return a.morton < b.morton || (a.morton == b.morton && a.index < b.index);
    });
    return result;
}

float point_distance(const Vec3f& a, const Vec3f& b) {
    return std::sqrt(static_cast<float>(sqr(a.x - b.x) + sqr(a.y - b.y) + sqr(a.z - b.z)));
}

float weightedPositionalError(const std::vector<Splat>& splats,
                              const std::vector<size_t>& indices,
                              const Splat& representative) {
    std::vector<WeightedError> errors;
    errors.reserve(indices.size());
    double total_weight = 0.0;
    for (size_t index : indices) {
        const Splat& source = splats[index];
        const double weight = std::max(kEpsilon, static_cast<double>(sigmoid(source.opacity)));
        errors.push_back({point_distance(source.pos, representative.pos), weight});
        total_weight += weight;
    }
    std::sort(errors.begin(), errors.end(),
              [](const WeightedError& left, const WeightedError& right) {
                  return left.value < right.value;
              });
    const double target = total_weight * 0.95;
    double accumulated = 0.0;
    for (const WeightedError& error : errors) {
        accumulated += error.weight;
        if (accumulated >= target) return error.value;
    }
    return errors.empty() ? 0.0f : errors.back().value;
}

float weightedRepresentationError(const std::vector<Splat>& source,
                                   const std::vector<Splat>& representatives) {
    if (source.empty() || representatives.empty()) return 0.0f;
    std::vector<WeightedError> errors;
    errors.reserve(source.size());
    double total_weight = 0.0;
    for (const Splat& original : source) {
        float nearest = std::numeric_limits<float>::max();
        for (const Splat& representative : representatives)
            nearest = std::min(nearest, point_distance(original.pos, representative.pos));
        const double weight = std::max(kEpsilon, static_cast<double>(sigmoid(original.opacity)));
        errors.push_back({nearest, weight});
        total_weight += weight;
    }
    std::sort(errors.begin(), errors.end(),
              [](const WeightedError& left, const WeightedError& right) {
                  return left.value < right.value;
              });
    const double target = total_weight * 0.95;
    double accumulated = 0.0;
    for (const WeightedError& error : errors) {
        accumulated += error.weight;
        if (accumulated >= target) return error.value;
    }
    return errors.back().value;
}

float logit(float alpha) {
    alpha = clamp(alpha, 1e-6f, 1.0f - 1e-6f);
    return std::log(alpha / (1.0f - alpha));
}

} // namespace

LodGenerator::LodGenerator(LodSettings settings) : settings_(settings) {
    if (settings_.levels < 2) throw std::runtime_error("--lod-levels must be at least 2");
    if (settings_.levels > 32) throw std::runtime_error("--lod-levels must not exceed 32");
    if (settings_.reduction < 2) throw std::runtime_error("--lod-reduction must be at least 2");
    if (settings_.reduction > 1024) throw std::runtime_error("--lod-reduction must not exceed 1024");
}

AdaptiveLodHierarchy LodGenerator::generate_adaptive(const std::vector<Splat>& source,
                                                       size_t max_leaf_splats) const {
    if (source.empty()) throw std::runtime_error("Cannot generate LODs from an empty scene");
    if (max_leaf_splats == 0) throw std::runtime_error("--max-leaf-splats must be greater than zero");
    validate(source, "source LOD");

    std::vector<std::vector<size_t>> memberships;
    std::vector<size_t> root(source.size());
    std::iota(root.begin(), root.end(), size_t{0});
    std::function<void(std::vector<size_t>)> split = [&](std::vector<size_t> indices) {
        if (indices.size() <= max_leaf_splats) {
            memberships.push_back(std::move(indices));
            return;
        }
        BBox centers;
        for (size_t index : indices) centers.expand(source[index].pos);
        const float extent[3] = {centers.max.x - centers.min.x,
                                 centers.max.y - centers.min.y,
                                 centers.max.z - centers.min.z};
        int axis = 0;
        if (extent[1] > extent[axis]) axis = 1;
        if (extent[2] > extent[axis]) axis = 2;
        std::stable_sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
            const float av = source[a].pos[axis], bv = source[b].pos[axis];
            return av < bv || (av == bv && a < b);
        });
        const size_t middle = indices.size() / 2;
        if (middle == 0 || middle == indices.size()) {
            throw std::runtime_error("Adaptive LOD partition failed to make progress");
        }
        std::vector<size_t> low(indices.begin(), indices.begin() + static_cast<ptrdiff_t>(middle));
        std::vector<size_t> high(indices.begin() + static_cast<ptrdiff_t>(middle), indices.end());
        split(std::move(low));
        split(std::move(high));
    };
    split(std::move(root));

    AdaptiveLodHierarchy hierarchy;
    hierarchy.leaves.reserve(memberships.size());
    for (size_t leaf_id = 0; leaf_id < memberships.size(); ++leaf_id) {
        AdaptiveLodLeaf leaf;
        leaf.id = leaf_id;
        leaf.source_indices = std::move(memberships[leaf_id]);
        std::vector<Splat> leaf_source;
        leaf_source.reserve(leaf.source_indices.size());
        for (size_t index : leaf.source_indices) {
            const Splat& splat = source[index];
            leaf_source.push_back(splat);
            const float radius = kBoundsSigma * max_scale(splat);
            leaf.bounds.expand(Vec3f(splat.pos.x - radius, splat.pos.y - radius, splat.pos.z - radius));
            leaf.bounds.expand(Vec3f(splat.pos.x + radius, splat.pos.y + radius, splat.pos.z + radius));
        }
        leaf.levels = generate(std::move(leaf_source));
        hierarchy.level_count = std::max(hierarchy.level_count, leaf.levels.size());
        hierarchy.bounds.expand(leaf.bounds);
        hierarchy.leaves.push_back(std::move(leaf));
    }

    hierarchy.splats_per_level.assign(hierarchy.level_count, 0);
    hierarchy.errors_per_level.assign(hierarchy.level_count, 0.0f);
    for (const AdaptiveLodLeaf& leaf : hierarchy.leaves) {
        const size_t first_level = hierarchy.level_count - leaf.levels.size();
        for (size_t local = 0; local < leaf.levels.size(); ++local) {
            const size_t level = first_level + local;
            hierarchy.splats_per_level[level] += leaf.levels[local].splats.size();
            hierarchy.errors_per_level[level] = std::max(hierarchy.errors_per_level[level], leaf.levels[local].error);
        }
    }
    return hierarchy;
}

SpatialLodHierarchy LodGenerator::generate_spatial(const std::vector<Splat>& source) const {
    if (source.empty()) throw std::runtime_error("Cannot generate LODs from an empty scene");
    if (settings_.max_leaf_splats == 0) throw std::runtime_error("--max-leaf-splats must be greater than zero");
    validate(source, "source LOD");

    struct PartitionNode {
        std::vector<size_t> source_indices;
        BBox bounds;
        size_t depth = 0;
        std::vector<size_t> children;
    };

    // The partition bounds are spatial cells, not Gaussian support bounds. The
    // root remains conservative, while every child cell is clipped at its
    // parent's kd split plane. Consequently siblings can touch, but cannot
    // overlap in volume.
    BBox root_bounds;
    for (const Splat& splat : source) {
        const float radius = kBoundsSigma * max_scale(splat);
        root_bounds.expand(Vec3f(splat.pos.x - radius, splat.pos.y - radius, splat.pos.z - radius));
        root_bounds.expand(Vec3f(splat.pos.x + radius, splat.pos.y + radius, splat.pos.z + radius));
    }

    std::vector<PartitionNode> partition;
    std::vector<size_t> all(source.size());
    std::iota(all.begin(), all.end(), size_t{0});
    size_t max_partition_depth = 0;
    std::function<size_t(std::vector<size_t>, const BBox&, size_t)> split =
        [&](std::vector<size_t> indices, const BBox& cell, size_t depth) -> size_t {
        BBox centers;
        for (size_t index : indices) centers.expand(source[index].pos);
        const double dx = static_cast<double>(centers.max.x) - centers.min.x;
        const double dy = static_cast<double>(centers.max.y) - centers.min.y;
        const double dz = static_cast<double>(centers.max.z) - centers.min.z;
        const double diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
        const bool over_count = indices.size() > settings_.max_leaf_splats;
        const bool over_extent = settings_.max_node_diagonal > 0.0f &&
            diagonal > settings_.max_node_diagonal &&
            indices.size() >= 2 * settings_.min_split_splats;
        const size_t node_id = partition.size();
        partition.push_back({std::move(indices), cell, depth, {}});
        max_partition_depth = std::max(max_partition_depth, depth);
        if (!over_count && !over_extent) {
            return node_id;
        }
        const double extent[3] = {dx, dy, dz};
        int axis = 0;
        if (extent[1] > extent[axis]) axis = 1;
        if (extent[2] > extent[axis]) axis = 2;
        std::vector<size_t>& members = partition[node_id].source_indices;
        std::stable_sort(members.begin(), members.end(), [&](size_t a, size_t b) {
            const float av = source[a].pos[axis], bv = source[b].pos[axis];
            return av < bv || (av == bv && a < b);
        });
        const size_t middle = members.size() / 2;
        if (middle == 0 || middle == members.size()) {
            throw std::runtime_error("Spatial LOD partition failed to make progress");
        }
        std::vector<size_t> low(members.begin(), members.begin() + static_cast<ptrdiff_t>(middle));
        std::vector<size_t> high(members.begin() + static_cast<ptrdiff_t>(middle), members.end());
        const float low_max = source[low.back()].pos[axis];
        const float high_min = source[high.front()].pos[axis];
        const float split_plane = low_max + (high_min - low_max) * 0.5f;
        BBox low_cell = cell;
        BBox high_cell = cell;
        low_cell.max[axis] = split_plane;
        high_cell.min[axis] = split_plane;
        const size_t low_id = split(std::move(low), low_cell, depth + 1);
        const size_t high_id = split(std::move(high), high_cell, depth + 1);
        partition[node_id].children = {low_id, high_id};
        return node_id;
    };
    const size_t partition_root = split(std::move(all), root_bounds, 0);

    SpatialLodHierarchy hierarchy;
    hierarchy.level_count = settings_.levels;
    hierarchy.nodes_per_level.resize(settings_.levels);
    hierarchy.splats_per_level.assign(settings_.levels, 0);
    hierarchy.errors_per_level.assign(settings_.levels, 0.0f);
    const size_t finest_level = settings_.levels - 1;
    const size_t root_depth = max_partition_depth > finest_level
        ? max_partition_depth - finest_level : 0;
    std::vector<size_t> partition_roots;
    std::function<void(size_t)> collect_roots = [&](size_t partition_id) {
        const PartitionNode& node = partition[partition_id];
        if (node.depth >= root_depth || node.children.empty()) {
            partition_roots.push_back(partition_id);
            return;
        }
        for (size_t child : node.children) collect_roots(child);
    };
    collect_roots(partition_root);

    // Emit bottom-up so IDs remain deterministic and payload ordering remains
    // compatible. Every output edge follows an actual kd-tree edge. A terminal
    // singleton may be promoted through remaining ranks, but its membership and
    // cell never change.
    std::function<size_t(size_t, size_t)> emit = [&](size_t partition_id, size_t level) -> size_t {
        const PartitionNode& spatial = partition[partition_id];
        std::vector<size_t> children;
        if (level < finest_level) {
            if (spatial.children.empty()) {
                children.push_back(emit(partition_id, level + 1));
            } else {
                for (size_t child : spatial.children) children.push_back(emit(child, level + 1));
            }
        }

        SpatialLodNode node;
        node.id = hierarchy.nodes.size();
        node.level = level;
        node.source_indices = spatial.source_indices;
        node.children = std::move(children);
        node.bounds = spatial.bounds;
        if (level == finest_level) {
            node.representation.splats.reserve(node.source_indices.size());
            for (size_t index : node.source_indices) node.representation.splats.push_back(source[index]);
            node.representation.stats.input_count = node.source_indices.size();
            node.representation.stats.output_count = node.source_indices.size();
        } else {
            size_t child_splats = 0;
            float child_error = 0.0f;
            for (size_t child_id : node.children) {
                const SpatialLodNode& child = hierarchy.nodes[child_id];
                child_splats += child.representation.splats.size();
                child_error = std::max(child_error, child.representation.error);
            }
            std::vector<Splat> node_source;
            node_source.reserve(node.source_indices.size());
            for (size_t index : node.source_indices) node_source.push_back(source[index]);
            LodSettings node_settings = settings_;
            node_settings.levels = settings_.levels - level;
            const std::vector<LodLevel> candidates = LodGenerator(node_settings).generate(node_source);
            // Use a fixed coarseness factor for all siblings at this level, not per-node.
            // This ensures siblings have comparable error values, preventing LOD flipping
            // due to visibility changes. Target: keep parent roughly 2-4x coarser than children.
            const size_t target_parent_splats = std::max<size_t>(1, child_splats / 3);
            size_t selected = candidates.size() - 1;
            while (selected > 0 &&
                   candidates[selected].splats.size() > target_parent_splats) {
                --selected;
            }
            node.representation = candidates[selected];
            if (node.representation.splats.size() >= child_splats ||
                node.representation.error <= child_error) {
                node.representation = decimate(node_source);
                while (node.representation.splats.size() >= child_splats &&
                       node.representation.splats.size() > 1) {
                    node.representation = decimate(node.representation.splats);
                }
                node.representation.error =
                    weightedRepresentationError(node_source, node.representation.splats);
            }
            node.representation.error = std::max(node.representation.error, child_error);
        }
        hierarchy.nodes_per_level[level].push_back(node.id);
        hierarchy.nodes.push_back(std::move(node));
        return hierarchy.nodes.back().id;
    };
    for (size_t root : partition_roots) hierarchy.roots.push_back(emit(root, 0));

    std::vector<size_t> ownership(source.size(), 0);
    for (size_t node_id : hierarchy.nodes_per_level[finest_level]) {
        for (size_t index : hierarchy.nodes[node_id].source_indices) {
            if (index >= ownership.size()) throw std::runtime_error("Spatial leaf has an invalid source index");
            ++ownership[index];
        }
    }
    if (std::any_of(ownership.begin(), ownership.end(), [](size_t count) { return count != 1; })) {
        throw std::runtime_error("Spatial partition did not assign every finest splat to exactly one leaf");
    }

    auto overlap_volume = [](const BBox& a, const BBox& b) {
        double volume = 1.0;
        for (int axis = 0; axis < 3; ++axis) {
            const double extent = std::min<double>(a.max[axis], b.max[axis]) -
                                  std::max<double>(a.min[axis], b.min[axis]);
            if (!(extent > 0.0)) return 0.0;
            volume *= extent;
        }
        return volume;
    };
    auto record_overlap = [&](SpatialOverlapStats& stats, const BBox& a, const BBox& b) {
        ++stats.pair_count;
        const double volume = overlap_volume(a, b);
        if (volume > 0.0) {
            ++stats.overlap_count;
            stats.total_overlap_volume += volume;
            stats.max_overlap_volume = std::max(stats.max_overlap_volume, volume);
        }
    };
    for (const SpatialLodNode& node : hierarchy.nodes) {
        for (size_t i = 0; i < node.children.size(); ++i) {
            for (size_t j = i + 1; j < node.children.size(); ++j) {
                record_overlap(hierarchy.sibling_overlap,
                               hierarchy.nodes[node.children[i]].bounds,
                               hierarchy.nodes[node.children[j]].bounds);
            }
        }
    }
    for (size_t i = 0; i < hierarchy.roots.size(); ++i) {
        for (size_t j = i + 1; j < hierarchy.roots.size(); ++j) {
            record_overlap(hierarchy.sibling_overlap,
                           hierarchy.nodes[hierarchy.roots[i]].bounds,
                           hierarchy.nodes[hierarchy.roots[j]].bounds);
        }
    }
    const std::vector<size_t>& leaves = hierarchy.nodes_per_level[finest_level];
    for (size_t i = 0; i < leaves.size(); ++i) {
        for (size_t j = i + 1; j < leaves.size(); ++j) {
            record_overlap(hierarchy.leaf_overlap,
                           hierarchy.nodes[leaves[i]].bounds,
                           hierarchy.nodes[leaves[j]].bounds);
        }
    }
    if (hierarchy.sibling_overlap.overlap_count != 0 || hierarchy.leaf_overlap.overlap_count != 0) {
        throw std::runtime_error("Spatial hierarchy contains positive-volume sibling or leaf overlap");
    }

    for (const SpatialLodNode& node : hierarchy.nodes) {
        hierarchy.splats_per_level[node.level] += node.representation.splats.size();
        hierarchy.errors_per_level[node.level] = std::max(hierarchy.errors_per_level[node.level],
                                                           node.representation.error);
    }
    hierarchy.bounds = root_bounds;
    return hierarchy;
}

std::vector<LodLevel> LodGenerator::generate(std::vector<Splat> source) const {
    if (source.empty()) throw std::runtime_error("Cannot generate LODs from an empty scene");
    validate(source, "source LOD");
    const std::vector<Splat> original_source = source;
    std::vector<LodLevel> fine_to_coarse;
    LodLevel original;
    original.splats = std::move(source);
    original.stats.input_count = original.splats.size();
    original.stats.output_count = original.splats.size();
    fine_to_coarse.push_back(std::move(original));
    while (fine_to_coarse.size() < settings_.levels && fine_to_coarse.back().splats.size() > 1) {
        LodLevel next = settings_.method == LodMethod::Decimate
            ? decimate(fine_to_coarse.back().splats)
            : cluster(fine_to_coarse.back().splats);
        if (next.splats.size() >= fine_to_coarse.back().splats.size()) {
            break;
        }
        validate(next.splats, "generated LOD");
        next.error = weightedRepresentationError(original_source, next.splats);
        fine_to_coarse.push_back(std::move(next));
    }
    std::reverse(fine_to_coarse.begin(), fine_to_coarse.end());
    return fine_to_coarse;
}

LodLevel LodGenerator::decimate(const std::vector<Splat>& source) const {
    LodLevel level;
    level.stats.input_count = source.size();
    const size_t target = std::max<size_t>(1, (source.size() + settings_.reduction - 1) / settings_.reduction);
    const auto ordered = morton_order(source);
    level.splats.reserve(target);
    double error_sum = 0.0;
    for (size_t bucket = 0; bucket < target; ++bucket) {
        const size_t begin = bucket * ordered.size() / target;
        const size_t end = (bucket + 1) * ordered.size() / target;
        size_t best = ordered[begin].index;
        double best_importance = importance(source[best]);
        for (size_t i = begin + 1; i < end; ++i) {
            const size_t candidate = ordered[i].index;
            const double score = importance(source[candidate]);
            if (score > best_importance) { best = candidate; best_importance = score; }
        }
        level.splats.push_back(source[best]);
        std::vector<size_t> bucket_indices;
        bucket_indices.reserve(end - begin);
        for (size_t i = begin; i < end; ++i) bucket_indices.push_back(ordered[i].index);
        const float bucket_error = weightedPositionalError(source, bucket_indices, source[best]);
        level.error = std::max(level.error, bucket_error);
        error_sum += bucket_error;
    }
    level.stats.output_count = level.splats.size();
    level.stats.cluster_count = target;
    level.stats.min_cluster_size = source.size() / target;
    level.stats.max_cluster_size = (source.size() + target - 1) / target;
    level.stats.average_cluster_size = static_cast<double>(source.size()) / target;
    level.stats.mean_covariance_error = error_sum / target;
    level.stats.max_error = level.error;
    return level;
}

LodLevel LodGenerator::cluster(const std::vector<Splat>& source) const {
    LodLevel level;
    level.stats.input_count = source.size();
    const auto ordered = morton_order(source);
    const size_t block_size = settings_.reduction;
    BBox bounds;
    double mean_radius = 0.0;
    for (const Splat& splat : source) {
        bounds.expand(splat.pos);
        mean_radius += max_scale(splat);
    }
    mean_radius /= source.size();
    const float diagonal = point_distance(bounds.min, bounds.max);
    const float target_clusters = static_cast<float>(std::max<size_t>(1, source.size() / settings_.reduction));
    const float distribution_threshold = kSpatialThresholdFactor * diagonal / std::cbrt(target_clusters);
    const float spatial_threshold = std::max(0.05f,
        std::min(distribution_threshold, static_cast<float>(8.0 * mean_radius)));
    level.stats.min_cluster_size = std::numeric_limits<size_t>::max();
    double error_sum = 0.0;
    for (size_t begin = 0; begin < ordered.size(); begin += block_size) {
        const size_t end = std::min(ordered.size(), begin + block_size);
        std::vector<std::vector<size_t>> groups;
        std::vector<Vec3f> group_colours;
        std::vector<Vec3f> group_positions;
        for (size_t i = begin; i < end; ++i) {
            const size_t index = ordered[i].index;
            const Vec3f colour = rgb(source[index]);
            size_t best_group = groups.size();
            float best_cost = std::numeric_limits<float>::max();
            for (size_t group = 0; group < groups.size(); ++group) {
                const float colour_delta = colour_distance(colour, group_colours[group]);
                const float position_delta = point_distance(source[index].pos, group_positions[group]);
                if (colour_delta <= kColourThreshold && position_delta <= spatial_threshold) {
                    const float cost = colour_delta / kColourThreshold + position_delta / spatial_threshold;
                    if (cost < best_cost) { best_cost = cost; best_group = group; }
                }
            }
            if (best_group == groups.size()) {
                if (!groups.empty()) ++level.stats.rejected_merges;
                groups.push_back({index});
                group_colours.push_back(colour);
                group_positions.push_back(source[index].pos);
            } else {
                auto& members = groups[best_group];
                const float n = static_cast<float>(members.size());
                group_colours[best_group] = Vec3f((group_colours[best_group].x * n + colour.x) / (n + 1.0f),
                                                  (group_colours[best_group].y * n + colour.y) / (n + 1.0f),
                                                  (group_colours[best_group].z * n + colour.z) / (n + 1.0f));
                group_positions[best_group] = Vec3f((group_positions[best_group].x * n + source[index].pos.x) / (n + 1.0f),
                                                    (group_positions[best_group].y * n + source[index].pos.y) / (n + 1.0f),
                                                    (group_positions[best_group].z * n + source[index].pos.z) / (n + 1.0f));
                members.push_back(index);
            }
        }
        for (const auto& group : groups) {
            Splat representative = merge_cluster(source, group);
            const float error = weightedPositionalError(source, group, representative);
            level.splats.push_back(std::move(representative));
            level.error = std::max(level.error, error);
            error_sum += error;
            level.stats.min_cluster_size = std::min(level.stats.min_cluster_size, group.size());
            level.stats.max_cluster_size = std::max(level.stats.max_cluster_size, group.size());
        }
    }
    level.stats.output_count = level.splats.size();
    level.stats.cluster_count = level.splats.size();
    if (level.splats.empty()) level.stats.min_cluster_size = 0;
    level.stats.average_cluster_size = static_cast<double>(source.size()) / std::max<size_t>(1, level.splats.size());
    level.stats.mean_covariance_error = error_sum / std::max<size_t>(1, level.splats.size());
    level.stats.max_error = level.error;
    return level;
}

Splat LodGenerator::merge_cluster(const std::vector<Splat>& splats,
                                  const std::vector<size_t>& indices,
                                  float* error) {
    if (indices.empty()) throw std::runtime_error("Cannot merge an empty Gaussian cluster");
    Splat result{};
    double total_weight = 0.0, alpha_product = 1.0;
    Vec3f colour_sum;
    for (size_t index : indices) {
        if (index >= splats.size()) throw std::runtime_error("Invalid Gaussian cluster index");
        const Splat& splat = splats[index];
        const double weight = importance(splat);
        total_weight += weight;
        result.pos.x += static_cast<float>(weight * splat.pos.x);
        result.pos.y += static_cast<float>(weight * splat.pos.y);
        result.pos.z += static_cast<float>(weight * splat.pos.z);
        const Vec3f colour = rgb(splat);
        colour_sum.x += static_cast<float>(weight * colour.x);
        colour_sum.y += static_cast<float>(weight * colour.y);
        colour_sum.z += static_cast<float>(weight * colour.z);
        for (size_t coefficient = 0; coefficient < 45; ++coefficient) {
            result.f_rest[coefficient] += static_cast<float>(weight * splat.f_rest[coefficient]);
        }
        alpha_product *= 1.0 - sigmoid(splat.opacity);
    }
    result.pos.x = static_cast<float>(result.pos.x / total_weight);
    result.pos.y = static_cast<float>(result.pos.y / total_weight);
    result.pos.z = static_cast<float>(result.pos.z / total_weight);
    const Vec3f merged_colour(colour_sum.x / static_cast<float>(total_weight),
                              colour_sum.y / static_cast<float>(total_weight),
                              colour_sum.z / static_cast<float>(total_weight));
    result.f_dc[0] = (clamp(merged_colour.x, 0.0f, 1.0f) - 0.5f) / kShC0;
    result.f_dc[1] = (clamp(merged_colour.y, 0.0f, 1.0f) - 0.5f) / kShC0;
    result.f_dc[2] = (clamp(merged_colour.z, 0.0f, 1.0f) - 0.5f) / kShC0;
    for (size_t coefficient = 0; coefficient < 45; ++coefficient) {
        result.f_rest[coefficient] /= static_cast<float>(total_weight);
    }
    result.opacity = logit(static_cast<float>(1.0 - alpha_product));

    Mat3 merged;
    for (size_t index : indices) {
        const Splat& splat = splats[index];
        const double weight = importance(splat);
        const Mat3 own = covariance(splat);
        const double delta[3] = {splat.pos.x - result.pos.x, splat.pos.y - result.pos.y, splat.pos.z - result.pos.z};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                merged.v[row][col] += weight * (own.v[row][col] + delta[row] * delta[col]) / total_weight;
            }
        }
    }
    double eigenvalues[3];
    Mat3 eigenvectors;
    eigen_symmetric(merged, eigenvalues, eigenvectors);
    result.scale = Vec3f(static_cast<float>(std::log(std::sqrt(eigenvalues[0]))),
                         static_cast<float>(std::log(std::sqrt(eigenvalues[1]))),
                         static_cast<float>(std::log(std::sqrt(eigenvalues[2]))));
    const Quat rotation = matrix_to_quaternion(eigenvectors);
    result.rot[0] = rotation.w; result.rot[1] = rotation.x;
    result.rot[2] = rotation.y; result.rot[3] = rotation.z;

    if (error) *error = weightedPositionalError(splats, indices, result);
    return result;
}

LodLevel LodGenerator::cluster_to_count(const std::vector<Splat>& source,
                                        size_t target_count) {
    if (source.empty()) throw std::runtime_error("Cannot synthesize an LOD from an empty region");
    if (target_count == 0 || target_count > source.size()) {
        throw std::runtime_error("Synthesized LOD target must be between one and the local source count");
    }
    LodLevel level;
    level.stats.input_count = source.size();
    const auto ordered = morton_order(source);
    level.splats.reserve(target_count);
    double error_sum = 0.0;
    level.stats.min_cluster_size = std::numeric_limits<size_t>::max();
    for (size_t bucket = 0; bucket < target_count; ++bucket) {
        const size_t begin = bucket * ordered.size() / target_count;
        const size_t end = (bucket + 1) * ordered.size() / target_count;
        std::vector<size_t> indices;
        indices.reserve(end - begin);
        for (size_t i = begin; i < end; ++i) indices.push_back(ordered[i].index);

        // Morton order keeps a bucket's members adjacent along the curve, but in
        // a sparse region "adjacent" can still be tens of metres apart. Merging
        // those would fold the separation into the merged covariance and emit a
        // single Gaussian spanning the gap, which is both wrong and ruinously
        // expensive to rasterize. Where a bucket is too spread out to merge,
        // keep its most important member instead - the same representative
        // choice decimate() makes. Sparse regions therefore thin out rather than
        // smearing, which is the correct coarse-LOD behaviour: there is
        // genuinely less there to show.
        BBox extent;
        double scale_sum = 0.0;
        for (size_t index : indices) {
            extent.expand(source[index].pos);
            scale_sum += max_scale(source[index]);
        }
        const double merge_extent = std::max(kMinMergeExtent,
            kMergeExtentFactor * scale_sum / static_cast<double>(indices.size()));

        float error = 0.0f;
        if (indices.size() > 1 &&
            point_distance(extent.min, extent.max) > merge_extent) {
            size_t best = indices.front();
            double best_importance = importance(source[best]);
            for (size_t index : indices) {
                const double score = importance(source[index]);
                if (score > best_importance) { best = index; best_importance = score; }
            }
            level.splats.push_back(source[best]);
            error = weightedPositionalError(source, indices, source[best]);
            ++level.stats.rejected_merges;
        } else {
            level.splats.push_back(merge_cluster(source, indices, &error));
        }
        level.error = std::max(level.error, error);
        error_sum += error;
        level.stats.min_cluster_size = std::min(level.stats.min_cluster_size, indices.size());
        level.stats.max_cluster_size = std::max(level.stats.max_cluster_size, indices.size());
    }
    level.stats.output_count = target_count;
    level.stats.cluster_count = target_count;
    level.stats.average_cluster_size = static_cast<double>(source.size()) / target_count;
    level.stats.mean_covariance_error = error_sum / target_count;
    level.stats.max_error = level.error;
    return level;
}

void LodGenerator::validate(const std::vector<Splat>& splats, const std::string& label) {
    for (size_t i = 0; i < splats.size(); ++i) {
        const Splat& s = splats[i];
        auto finite = [](float value) { return std::isfinite(value); };
        const float alpha = sigmoid(s.opacity);
        const float sx = std::exp(s.scale.x), sy = std::exp(s.scale.y), sz = std::exp(s.scale.z);
        const float qnorm = std::sqrt(s.rot[0] * s.rot[0] + s.rot[1] * s.rot[1] +
                                      s.rot[2] * s.rot[2] + s.rot[3] * s.rot[3]);
        const Vec3f colour = rgb(s);
        if (!finite(s.pos.x) || !finite(s.pos.y) || !finite(s.pos.z) ||
            !finite(s.scale.x) || !finite(s.scale.y) || !finite(s.scale.z) ||
            !finite(sx) || !finite(sy) || !finite(sz) || sx <= 0.0f || sy <= 0.0f || sz <= 0.0f ||
            !finite(s.opacity) || !(alpha >= 0.0f && alpha <= 1.0f) ||
            !finite(qnorm) || std::abs(qnorm - 1.0f) > 1e-3f ||
            !(colour.x >= 0.0f && colour.x <= 1.0f && colour.y >= 0.0f && colour.y <= 1.0f &&
              colour.z >= 0.0f && colour.z <= 1.0f)) {
            throw std::runtime_error(label + " contains invalid Gaussian at index " + std::to_string(i));
        }
    }
}

void LodGenerator::write_binary_ply(const fs::path& path, const std::vector<Splat>& splats,
                                    int num_f_rest) {
    num_f_rest = std::max(0, std::min(num_f_rest, 45));
    auto file = platform::ofstream_open(path, std::ios::out | std::ios::binary);
    if (!file) throw std::runtime_error("Failed to create generated LOD: " + path.u8string());
    file << "ply\nformat binary_little_endian 1.0\n"
         << "comment generated by ply2lcc offline LOD generator\n"
         << "element vertex " << splats.size() << "\n"
         << "property float x\nproperty float y\nproperty float z\n"
         << "property float f_dc_0\nproperty float f_dc_1\nproperty float f_dc_2\n";
    for (int i = 0; i < num_f_rest; ++i) file << "property float f_rest_" << i << "\n";
    file << "property float opacity\n"
         << "property float scale_0\nproperty float scale_1\nproperty float scale_2\n"
         << "property float rot_0\nproperty float rot_1\nproperty float rot_2\nproperty float rot_3\n"
         << "end_header\n";
    for (const Splat& s : splats) {
        const float prefix[6] = {s.pos.x, s.pos.y, s.pos.z, s.f_dc[0], s.f_dc[1], s.f_dc[2]};
        const float suffix[8] = {s.opacity, s.scale.x, s.scale.y, s.scale.z,
                                 s.rot[0], s.rot[1], s.rot[2], s.rot[3]};
        file.write(reinterpret_cast<const char*>(prefix), sizeof(prefix));
        file.write(reinterpret_cast<const char*>(s.f_rest),
                   static_cast<std::streamsize>(num_f_rest * sizeof(float)));
        file.write(reinterpret_cast<const char*>(suffix), sizeof(suffix));
    }
    if (!file) throw std::runtime_error("Failed while writing generated LOD: " + path.u8string());
}

} // namespace ply2lcc
