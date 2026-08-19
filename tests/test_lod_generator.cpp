#include <gtest/gtest.h>

#include "lod_generator.hpp"

#include <cmath>
#include <cstring>
#include <set>

using namespace ply2lcc;

namespace {

constexpr float kShC0 = 0.28209479177387814f;

Splat make_splat(float x, float y, float z, float r, float g, float b,
                  float alpha = 0.5f, float scale = 0.1f) {
    Splat splat{};
    splat.pos = Vec3f(x, y, z);
    splat.f_dc[0] = (r - 0.5f) / kShC0;
    splat.f_dc[1] = (g - 0.5f) / kShC0;
    splat.f_dc[2] = (b - 0.5f) / kShC0;
    splat.opacity = std::log(alpha / (1.0f - alpha));
    splat.scale = Vec3f(std::log(scale), std::log(scale), std::log(scale));
    splat.rot[0] = 1.0f;
    return splat;
}

float red(const Splat& splat) { return 0.5f + kShC0 * splat.f_dc[0]; }
float green(const Splat& splat) { return 0.5f + kShC0 * splat.f_dc[1]; }
float alpha(const Splat& splat) { return sigmoid(splat.opacity); }

} // namespace

TEST(LodGeneratorTest, WeightedCentroidOfEqualGaussiansIsMidpoint) {
    const std::vector<Splat> source{make_splat(-1, 0, 0, 1, 0, 0),
                                    make_splat(1, 0, 0, 1, 0, 0)};
    const Splat merged = LodGenerator::merge_cluster(source, {0, 1});
    EXPECT_NEAR(merged.pos.x, 0.0f, 1e-5f);
    EXPECT_NEAR(merged.pos.y, 0.0f, 1e-5f);
}

TEST(LodGeneratorTest, ErrorUsesOpacityWeightedPositionPercentile) {
    std::vector<Splat> source;
    for (int i = 0; i < 19; ++i)
        source.push_back(make_splat(0, 0, 0, 1, 1, 1, 0.5f, 0.1f));
    source.push_back(make_splat(100, 0, 0, 1, 1, 1, 0.000001f, 0.1f));

    float error = 0.0f;
    const std::vector<size_t> indices{0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                                      10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    LodGenerator::merge_cluster(source, indices,
                                &error);
    EXPECT_LT(error, 0.01f);

    source.back().pos = Vec3f(0, 0, 0);
    source.back().scale = Vec3f(std::log(100.0f), std::log(100.0f), std::log(100.0f));
    error = 1.0f;
    LodGenerator::merge_cluster(source, indices, &error);
    EXPECT_FLOAT_EQ(error, 0.0f);
}

TEST(LodGeneratorTest, MomentMatchedCovarianceExpandsAlongSeparationAxis) {
    const std::vector<Splat> source{make_splat(-1, 0, 0, 1, 1, 1),
                                    make_splat(1, 0, 0, 1, 1, 1)};
    const Splat merged = LodGenerator::merge_cluster(source, {0, 1});
    const float sx = std::exp(merged.scale.x);
    const float sy = std::exp(merged.scale.y);
    const float sz = std::exp(merged.scale.z);
    EXPECT_GT(sx, sy * 5.0f);
    EXPECT_NEAR(sy, sz, 1e-4f);
}

TEST(LodGeneratorTest, RedMergedWithRedRemainsRed) {
    const std::vector<Splat> source{make_splat(0, 0, 0, 1, 0, 0),
                                    make_splat(0.1f, 0, 0, 1, 0, 0)};
    const Splat merged = LodGenerator::merge_cluster(source, {0, 1});
    EXPECT_NEAR(red(merged), 1.0f, 1e-5f);
    EXPECT_NEAR(green(merged), 0.0f, 1e-5f);
}

TEST(LodGeneratorTest, ColourThresholdKeepsRedAndGreenSeparate) {
    LodSettings settings;
    settings.levels = 2;
    settings.reduction = 2;
    settings.method = LodMethod::Cluster;
    LodGenerator generator(settings);
    const std::vector<Splat> source{make_splat(0, 0, 0, 1, 0, 0),
                                    make_splat(0.01f, 0, 0, 0, 1, 0)};
    const auto levels = generator.generate(source);
    ASSERT_EQ(levels.size(), 1u);
    EXPECT_EQ(levels.front().splats.size(), 2u);
}

TEST(LodGeneratorTest, SpatialThresholdKeepsDistantSplatsSeparate) {
    LodSettings settings;
    settings.levels = 2;
    settings.reduction = 2;
    settings.method = LodMethod::Cluster;
    LodGenerator generator(settings);
    const std::vector<Splat> source{make_splat(0, 0, 0, 1, 1, 1),
                                    make_splat(100, 0, 0, 1, 1, 1)};
    const auto levels = generator.generate(source);
    ASSERT_EQ(levels.size(), 1u);
    EXPECT_EQ(levels.front().splats.size(), 2u);
}

TEST(LodGeneratorTest, AccumulatedOpacityIsBounded) {
    const std::vector<Splat> source{make_splat(0, 0, 0, 1, 1, 1, 0.1f),
                                    make_splat(0, 0, 0, 1, 1, 1, 0.1f)};
    const Splat merged = LodGenerator::merge_cluster(source, {0, 1});
    EXPECT_NEAR(alpha(merged), 0.19f, 1e-5f);
    EXPECT_GT(alpha(merged), 0.0f);
    EXPECT_LT(alpha(merged), 1.0f);
}

TEST(LodGeneratorTest, DecimationReducesAndIsDeterministic) {
    std::vector<Splat> source;
    for (int i = 0; i < 64; ++i) source.push_back(make_splat(static_cast<float>(i), i % 3, 0, 0.5f, 0.5f, 0.5f));
    LodSettings settings;
    settings.levels = 3;
    settings.reduction = 4;
    settings.method = LodMethod::Decimate;
    LodGenerator generator(settings);
    const auto first = generator.generate(source);
    const auto second = generator.generate(source);
    ASSERT_EQ(first.size(), 3u);
    EXPECT_EQ(first[0].splats.size(), 4u);
    EXPECT_EQ(first[1].splats.size(), 16u);
    ASSERT_EQ(first.size(), second.size());
    for (size_t level = 0; level < first.size(); ++level) {
        ASSERT_EQ(first[level].splats.size(), second[level].splats.size());
        EXPECT_EQ(std::memcmp(first[level].splats.data(), second[level].splats.data(),
                              first[level].splats.size() * sizeof(Splat)), 0);
    }
}

TEST(LodGeneratorTest, AdaptivePartitionIsBoundedDeterministicAndComplete) {
    std::vector<Splat> source;
    for (int i = 0; i < 101; ++i) {
        source.push_back(make_splat(static_cast<float>(i % 11), static_cast<float>(i / 11),
                                    static_cast<float>(i % 3), 0.5f, 0.5f, 0.5f));
    }
    LodSettings settings;
    settings.levels = 3;
    settings.reduction = 4;
    settings.method = LodMethod::Decimate;
    LodGenerator generator(settings);
    const auto first = generator.generate_adaptive(source, 16);
    const auto second = generator.generate_adaptive(source, 16);

    ASSERT_GT(first.leaves.size(), 1u);
    ASSERT_EQ(first.leaves.size(), second.leaves.size());
    std::set<size_t> membership;
    for (size_t i = 0; i < first.leaves.size(); ++i) {
        const auto& leaf = first.leaves[i];
        EXPECT_EQ(leaf.id, i);
        EXPECT_FALSE(leaf.source_indices.empty());
        EXPECT_LE(leaf.source_indices.size(), 16u);
        EXPECT_EQ(leaf.source_indices, second.leaves[i].source_indices);
        ASSERT_FALSE(leaf.levels.empty());
        EXPECT_EQ(leaf.levels.back().splats.size(), leaf.source_indices.size());
        for (size_t index : leaf.source_indices) EXPECT_TRUE(membership.insert(index).second);
    }
    EXPECT_EQ(membership.size(), source.size());
}

TEST(LodGeneratorTest, CoincidentPointsSplitWithoutEmptyOrNonProgressingLeaves) {
    std::vector<Splat> source(37, make_splat(1, 2, 3, 0.5f, 0.5f, 0.5f));
    LodSettings settings;
    settings.levels = 2;
    settings.method = LodMethod::Decimate;
    LodGenerator generator(settings);
    const auto hierarchy = generator.generate_adaptive(source, 8);
    EXPECT_EQ(hierarchy.leaves.size(), 8u);
    size_t total = 0;
    for (const auto& leaf : hierarchy.leaves) {
        EXPECT_FALSE(leaf.source_indices.empty());
        EXPECT_LE(leaf.source_indices.size(), 8u);
        total += leaf.source_indices.size();
    }
    EXPECT_EQ(total, source.size());
}

TEST(LodGeneratorTest, LeafBoundsIncludeThreeSigmaSupportAndRemainLocalInHeight) {
    std::vector<Splat> source;
    for (int i = 0; i < 8; ++i) {
        source.push_back(make_splat(static_cast<float>(i), 0, i < 4 ? 0.0f : 100.0f,
                                    0.5f, 0.5f, 0.5f, 0.5f, 0.25f));
    }
    LodSettings settings;
    settings.levels = 2;
    settings.method = LodMethod::Decimate;
    LodGenerator generator(settings);
    const auto hierarchy = generator.generate_adaptive(source, 4);
    ASSERT_EQ(hierarchy.leaves.size(), 2u);
    for (const auto& leaf : hierarchy.leaves) {
        for (int axis = 0; axis < 3; ++axis) {
            EXPECT_TRUE(std::isfinite(leaf.bounds.min[axis]));
            EXPECT_TRUE(std::isfinite(leaf.bounds.max[axis]));
            EXPECT_LE(leaf.bounds.min[axis], leaf.bounds.max[axis]);
        }
        EXPECT_LT(leaf.bounds.max.z - leaf.bounds.min.z, 10.0f);
        for (size_t index : leaf.source_indices) {
            EXPECT_LE(leaf.bounds.min.x, source[index].pos.x - 0.75f + 1e-5f);
            EXPECT_GE(leaf.bounds.max.x, source[index].pos.x + 0.75f - 1e-5f);
        }
    }
}

TEST(LodGeneratorTest, RepresentativesNeverCrossLeafMembershipBoundaries) {
    const std::vector<Splat> source{
        make_splat(0.0f, 0, 0, 0.5f, 0.5f, 0.5f),
        make_splat(0.01f, 0, 0, 0.5f, 0.5f, 0.5f),
        make_splat(100.0f, 0, 0, 0.5f, 0.5f, 0.5f),
        make_splat(100.01f, 0, 0, 0.5f, 0.5f, 0.5f)};
    LodSettings settings;
    settings.levels = 2;
    settings.reduction = 2;
    settings.method = LodMethod::Cluster;
    LodGenerator generator(settings);
    const auto hierarchy = generator.generate_adaptive(source, 2);
    ASSERT_EQ(hierarchy.leaves.size(), 2u);
    for (const Splat& representative : hierarchy.leaves[0].levels.front().splats) {
        EXPECT_LT(representative.pos.x, 50.0f);
    }
    for (const Splat& representative : hierarchy.leaves[1].levels.front().splats) {
        EXPECT_GT(representative.pos.x, 50.0f);
    }
}

TEST(LodGeneratorTest, RejectsImpossibleLeafLimit) {
    LodSettings settings;
    settings.levels = 2;
    LodGenerator generator(settings);
    EXPECT_THROW(generator.generate_adaptive({make_splat(0, 0, 0, 1, 1, 1)}, 0), std::runtime_error);
}

TEST(LodGeneratorTest, SpatialHierarchyBranchesAndPreservesMembership) {
    std::vector<Splat> source;
    for (int cluster = 0; cluster < 2; ++cluster) {
        for (int i = 0; i < 64; ++i) {
            source.push_back(make_splat(cluster * 100.0f + (i % 8) * 0.01f,
                                        (i / 8) * 0.01f, 0, 0.5f, 0.5f, 0.5f));
        }
    }
    LodSettings settings;
    settings.levels = 3;
    settings.reduction = 2;
    settings.method = LodMethod::Decimate;
    settings.max_leaf_splats = 8;
    settings.max_refinement_cost = 10;
    LodGenerator generator(settings);
    const auto hierarchy = generator.generate_spatial(source);

    EXPECT_GT(hierarchy.nodes_per_level.back().size(), hierarchy.roots.size());
    EXPECT_EQ(hierarchy.sibling_overlap.overlap_count, 0u);
    EXPECT_EQ(hierarchy.leaf_overlap.overlap_count, 0u);
    std::vector<size_t> ownership(source.size(), 0);
    for (size_t leaf_id : hierarchy.nodes_per_level.back()) {
        for (size_t index : hierarchy.nodes[leaf_id].source_indices) ++ownership[index];
    }
    EXPECT_TRUE(std::all_of(ownership.begin(), ownership.end(), [](size_t count) { return count == 1; }));
    size_t unary = 0;
    for (const auto& node : hierarchy.nodes) {
        if (node.children.size() == 1) ++unary;
        std::set<size_t> child_membership;
        size_t child_splats = 0;
        for (size_t child_id : node.children) {
            const auto& child = hierarchy.nodes[child_id];
            EXPECT_EQ(child.level, node.level + 1);
            EXPECT_LE(child.representation.error, node.representation.error + 1e-5f);
            for (int axis = 0; axis < 3; ++axis) {
                EXPECT_GE(child.bounds.min[axis], node.bounds.min[axis] - 1e-5f);
                EXPECT_LE(child.bounds.max[axis], node.bounds.max[axis] + 1e-5f);
            }
            child_membership.insert(child.source_indices.begin(), child.source_indices.end());
            child_splats += child.representation.splats.size();
        }
        if (!node.children.empty()) {
            EXPECT_EQ(child_membership, std::set<size_t>(node.source_indices.begin(), node.source_indices.end()));
            const size_t cost = child_splats > node.representation.splats.size()
                ? child_splats - node.representation.splats.size() : 0;
            EXPECT_LE(cost, settings.max_refinement_cost);
        }
    }
    EXPECT_EQ(unary, 0u);
    ASSERT_GE(hierarchy.roots.size(), 2u);
    bool low_root = false, high_root = false;
    for (size_t root_id : hierarchy.roots) {
        const auto& root = hierarchy.nodes[root_id];
        if (root.bounds.max.x < 50.0f) low_root = true;
        if (root.bounds.min.x > 50.0f) high_root = true;
    }
    EXPECT_TRUE(low_root);
    EXPECT_TRUE(high_root);
}

TEST(LodGeneratorTest, SpatialHierarchyHandlesDenseSparseAndPlanarInputDeterministically) {
    std::vector<Splat> source;
    for (int i = 0; i < 90; ++i) {
        source.push_back(make_splat((i % 10) * 0.01f, (i / 10) * 0.01f, 0,
                                    0.5f, 0.5f, 0.5f));
    }
    for (int i = 0; i < 10; ++i) {
        source.push_back(make_splat(100.0f + i, 0, 0, 0.5f, 0.5f, 0.5f));
    }
    LodSettings settings;
    settings.levels = 4;
    settings.reduction = 2;
    settings.method = LodMethod::Decimate;
    settings.max_leaf_splats = 10;
    settings.max_refinement_cost = 12;
    const LodGenerator generator(settings);
    const auto first = generator.generate_spatial(source);
    const auto second = generator.generate_spatial(source);
    ASSERT_EQ(first.nodes.size(), second.nodes.size());
    ASSERT_EQ(first.roots, second.roots);
    for (size_t i = 0; i < first.nodes.size(); ++i) {
        const auto& a = first.nodes[i];
        const auto& b = second.nodes[i];
        EXPECT_EQ(a.level, b.level);
        EXPECT_EQ(a.children, b.children);
        EXPECT_EQ(a.source_indices, b.source_indices);
        EXPECT_EQ(a.representation.splats.size(), b.representation.splats.size());
        EXPECT_FLOAT_EQ(a.representation.error, b.representation.error);
        for (int axis = 0; axis < 3; ++axis) {
            EXPECT_TRUE(std::isfinite(a.bounds.min[axis]));
            EXPECT_TRUE(std::isfinite(a.bounds.max[axis]));
            EXPECT_FLOAT_EQ(a.bounds.min[axis], b.bounds.min[axis]);
            EXPECT_FLOAT_EQ(a.bounds.max[axis], b.bounds.max[axis]);
        }
    }
    for (size_t leaf_id : first.nodes_per_level.back()) {
        EXPECT_LE(first.nodes[leaf_id].source_indices.size(), settings.max_leaf_splats);
    }
}

TEST(LodGeneratorTest, SpatialDiagonalLimitTriggersAdditionalFineSplits) {
    std::vector<Splat> source;
    for (int i = 0; i < 4; ++i) {
        source.push_back(make_splat(static_cast<float>(i), 0, 0, 0.5f, 0.5f, 0.5f));
    }
    LodSettings settings;
    settings.levels = 2;
    settings.method = LodMethod::Decimate;
    settings.max_leaf_splats = 100;
    settings.max_node_diagonal = 0.5f;
    settings.min_split_splats = 1;
    const auto hierarchy = LodGenerator(settings).generate_spatial(source);
    EXPECT_EQ(hierarchy.nodes_per_level.back().size(), 4u);
    for (size_t leaf : hierarchy.nodes_per_level.back()) {
        EXPECT_EQ(hierarchy.nodes[leaf].source_indices.size(), 1u);
    }
}
