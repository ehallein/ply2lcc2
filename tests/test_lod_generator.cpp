#include <gtest/gtest.h>

#include "lod_generator.hpp"

#include <cmath>
#include <cstring>

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
