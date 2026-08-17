#include <gtest/gtest.h>

#include "convert_app.hpp"
#include "splat_buffer.hpp"
#include "lod_generator.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;
using namespace ply2lcc;

namespace {

void write_test_splats(const fs::path& path, int num_f_rest = 0) {
    std::ofstream file(path, std::ios::binary);
    ASSERT_TRUE(file);
    file << "ply\n"
         << "format binary_little_endian 1.0\n"
         << "element vertex 2\n"
         << "property float x\n"
         << "property float y\n"
         << "property float z\n"
         << "property float f_dc_0\n"
         << "property float f_dc_1\n"
         << "property float f_dc_2\n";
    for (int i = 0; i < num_f_rest; ++i) {
        file << "property float f_rest_" << i << "\n";
    }
    file << "property float opacity\n"
         << "property float scale_0\n"
         << "property float scale_1\n"
         << "property float scale_2\n"
         << "property float rot_0\n"
         << "property float rot_1\n"
         << "property float rot_2\n"
         << "property float rot_3\n"
         << "end_header\n";

    const std::array<std::array<float, 14>, 2> rows{{
        {{-1.0f, -2.0f, -3.0f, 0.1f, 0.2f, 0.3f, 0.0f,
          0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}},
        {{4.0f, 5.0f, 6.0f, 0.4f, 0.5f, 0.6f, 1.0f,
          0.1f, 0.2f, 0.3f, 1.0f, 0.0f, 0.0f, 0.0f}}
    }};
    const std::vector<float> rest(static_cast<size_t>(num_f_rest), 0.0f);
    for (const auto& row : rows) {
        file.write(reinterpret_cast<const char*>(row.data()), 6 * sizeof(float));
        file.write(reinterpret_cast<const char*>(rest.data()),
                   static_cast<std::streamsize>(rest.size() * sizeof(float)));
        file.write(reinterpret_cast<const char*>(row.data() + 6), 8 * sizeof(float));
    }
    ASSERT_TRUE(file);
}

void write_test_spz_v4(const fs::path& path, uint32_t point_count, uint8_t sh_degree) {
    std::array<uint8_t, 49> bytes{};
    bytes[0] = 'N'; bytes[1] = 'G'; bytes[2] = 'S'; bytes[3] = 'P';
    bytes[4] = 4;
    bytes[8] = static_cast<uint8_t>(point_count & 0xff);
    bytes[9] = static_cast<uint8_t>((point_count >> 8) & 0xff);
    bytes[10] = static_cast<uint8_t>((point_count >> 16) & 0xff);
    bytes[11] = static_cast<uint8_t>((point_count >> 24) & 0xff);
    bytes[12] = sh_degree;
    bytes[13] = 12;
    bytes[15] = 1;
    bytes[16] = 32;
    bytes[32] = 1;
    bytes[40] = 9;
    std::ofstream file(path, std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
    ASSERT_TRUE(file);
}

std::string read_text(const fs::path& path) {
    std::ifstream file(path);
    std::ostringstream contents;
    contents << file.rdbuf();
    return contents.str();
}

void write_many_splats(const fs::path& path, size_t count) {
    std::vector<Splat> splats;
    splats.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        Splat splat{};
        splat.pos = Vec3f(static_cast<float>(i % 4), static_cast<float>((i / 4) % 4),
                          static_cast<float>(i / 16));
        splat.scale = Vec3f(-2.0f, -2.0f, -2.0f);
        splat.rot[0] = 1.0f;
        splats.push_back(splat);
    }
    LodGenerator::write_binary_ply(path, splats, 0);
}

} // namespace

TEST(Lcc2WriterTest, CreatesVersion003PackageWithReferencedPly) {
    const fs::path base = fs::temp_directory_path() / "ply2lcc_lcc2_writer_test";
    fs::remove_all(base);
    fs::create_directories(base);

    const fs::path input = base / "scene.ply";
    const fs::path output = base / "output";
    write_test_splats(input);

    ConvertConfig config;
    config.input_path = input;
    config.output_dir = output;
    config.output_format = OutputFormat::Lcc2;
    config.include_env = false;

    ConvertApp app(config);
    app.run();

    const fs::path metadata_path = output / "meta.lcc2";
    const fs::path notice_path = output / "LCC2-NOTICE.md";
    const fs::path copied_ply = output / "data" / "3dgs" / "lod_0.ply";
    ASSERT_TRUE(fs::exists(metadata_path));
    ASSERT_TRUE(fs::exists(notice_path));
    ASSERT_TRUE(fs::exists(copied_ply));
    SplatBuffer exported;
    ASSERT_TRUE(exported.initialize(copied_ply)) << exported.error();
    ASSERT_EQ(exported.size(), 2u);
    EXPECT_FLOAT_EQ(exported.pos(0).x, -1.0f);
    EXPECT_FLOAT_EQ(exported.pos(0).y, -3.0f);
    EXPECT_FLOAT_EQ(exported.pos(0).z, 2.0f);
    const Quat rotation = exported[0].rot();
    EXPECT_NEAR(rotation.w, 0.70710678f, 1e-6f);
    EXPECT_NEAR(rotation.x, -0.70710678f, 1e-6f);
    EXPECT_NEAR(rotation.y, 0.0f, 1e-6f);
    EXPECT_NEAR(rotation.z, 0.0f, 1e-6f);

    const std::string metadata = read_text(metadata_path);
    EXPECT_NE(metadata.find("\"version\": \"0.0.3\""), std::string::npos);
    EXPECT_NE(metadata.find("\"splatType\": \".ply\""), std::string::npos);
    EXPECT_NE(metadata.find("\"totalSplats\": 2"), std::string::npos);
    EXPECT_NE(metadata.find("\"lodSplats\": [2]"), std::string::npos);
    EXPECT_NE(metadata.find("\"splatFiles\": [\"data/3dgs/lod_0.ply\"]"), std::string::npos);
    EXPECT_NE(metadata.find("\"min\": [-1, -3, -5]"), std::string::npos);
    EXPECT_NE(metadata.find("\"max\": [4, 6, 2]"), std::string::npos);
    EXPECT_NE(metadata.find("\"3dgs\": {\"name\": 0, \"start\": 0, \"count\": 2}"),
              std::string::npos);
    EXPECT_NE(read_text(notice_path).find("https://github.com/xgrids/LCC2Whitepaper"),
              std::string::npos);

    fs::remove_all(base);
}

TEST(Lcc2WriterTest, StoresMatchingSpzV4Payload) {
    const fs::path base = fs::temp_directory_path() / "ply2lcc_lcc2_spz_writer_test";
    fs::remove_all(base);
    fs::create_directories(base);

    const fs::path input = base / "scene.ply";
    const fs::path payload = base / "scene.spz";
    const fs::path output = base / "output";
    write_test_splats(input);
    write_test_spz_v4(payload, 2, 0);

    ConvertConfig config;
    config.input_path = input;
    config.output_dir = output;
    config.output_format = OutputFormat::Lcc2;
    config.lcc2_payload_paths = {payload};
    config.splat_transform_path = PLY2LCC_FAKE_SPLAT_TRANSFORM;
    config.include_env = false;

    ConvertApp app(config);
    app.run();

    const fs::path copied_spz = output / "data" / "3dgs" / "lod_0.spz";
    ASSERT_TRUE(fs::exists(copied_spz));
    EXPECT_EQ(fs::file_size(copied_spz), fs::file_size(payload));
    const std::string metadata = read_text(output / "meta.lcc2");
    EXPECT_NE(metadata.find("\"splatType\": \".spz\""), std::string::npos);
    EXPECT_NE(metadata.find("data/3dgs/lod_0.spz"), std::string::npos);

    fs::remove_all(base);
}

TEST(Lcc2WriterTest, GeneratedLodUsesSpatialHierarchyAndErrorMetadata) {
    const fs::path base = fs::temp_directory_path() / "ply2lcc_lcc2_generated_lod_test";
    fs::remove_all(base);
    fs::create_directories(base);
    const fs::path input = base / "scene.ply";
    const fs::path output = base / "output";
    write_test_splats(input, 9);

    ConvertConfig config;
    config.input_path = input;
    config.output_dir = output;
    config.output_format = OutputFormat::Lcc2;
    config.include_env = false;
    config.lod.generate = true;
    config.lod.levels = 2;
    config.lod.reduction = 2;
    config.lod.method = LodMethod::Decimate;
    config.splat_transform_path = PLY2LCC_FAKE_SPLAT_TRANSFORM;
    ConvertApp app(config);
    app.run();

    const std::string metadata = read_text(output / "meta.lcc2");
    EXPECT_NE(metadata.find("\"lodErrors\":"), std::string::npos);
    EXPECT_NE(metadata.find("\"lodSplats\": [2, 1]"), std::string::npos);
    EXPECT_NE(metadata.find("\"lodLevel\": 0"), std::string::npos);
    EXPECT_NE(metadata.find("\"lodError\":"), std::string::npos);
    EXPECT_NE(metadata.find("\"start\":"), std::string::npos);
    EXPECT_NE(metadata.find("\"count\":"), std::string::npos);
    EXPECT_NE(metadata.find("\"splatType\": \".spz\""), std::string::npos);
    const size_t leaf = metadata.find("\"id\": \"node-1\"");
    const size_t leaf_data = metadata.find("\"data\": {\"3dgs\"", leaf);
    const size_t leaf_child = metadata.find("\"child\": {", leaf);
    ASSERT_NE(leaf, std::string::npos);
    ASSERT_NE(leaf_data, std::string::npos);
    ASSERT_NE(leaf_child, std::string::npos);
    EXPECT_LT(leaf_data, leaf_child);
    EXPECT_NE(metadata.find("\"lodLevel\": 1", leaf), std::string::npos);
    EXPECT_TRUE(fs::exists(output / "data" / "3dgs" / "lod_0.spz"));
    EXPECT_TRUE(fs::exists(output / "data" / "3dgs" / "lod_1.spz"));
    std::ifstream coarse_spz(output / "data" / "3dgs" / "lod_0.spz", std::ios::binary);
    std::array<uint8_t, 13> coarse_header{};
    coarse_spz.read(reinterpret_cast<char*>(coarse_header.data()), coarse_header.size());
    ASSERT_TRUE(coarse_spz);
    EXPECT_EQ(coarse_header[12], 1);
    EXPECT_FALSE(fs::exists(output / ".generated_lod"));
    fs::remove_all(base);
}


TEST(Lcc2WriterTest, AdaptiveCompatibilityUsesOnePayloadPerLevel) {
    const fs::path base = fs::temp_directory_path() / "ply2lcc_lcc2_adaptive_level_test";
    fs::remove_all(base);
    fs::create_directories(base);
    const fs::path input = base / "dense.ply";
    const fs::path output = base / "output";
    write_many_splats(input, 24);

    ConvertConfig config;
    config.input_path = input;
    config.output_dir = output;
    config.output_format = OutputFormat::Lcc2;
    config.include_env = false;
    config.lod.generate = true;
    config.lod.levels = 2;
    config.lod.reduction = 2;
    config.lod.method = LodMethod::Decimate;
    config.lod.max_leaf_splats = 6;
    config.splat_transform_path = PLY2LCC_FAKE_SPLAT_TRANSFORM;
    ConvertApp(config).run();

    const std::string metadata = read_text(output / "meta.lcc2");
    EXPECT_TRUE(fs::exists(output / "data/3dgs/lod_0.spz"));
    EXPECT_TRUE(fs::exists(output / "data/3dgs/lod_1.spz"));
    EXPECT_FALSE(fs::exists(output / "data/3dgs/lod_0_chunk_0.spz"));
    EXPECT_NE(metadata.find("\"id\": \"node-3\""), std::string::npos);
    EXPECT_NE(metadata.find("\"count\": 6"), std::string::npos);
}

TEST(Lcc2WriterTest, ChunkedPayloadsAreBoundedAndReferenced) {
    const fs::path base = fs::temp_directory_path() / "ply2lcc_lcc2_chunked_test";
    fs::remove_all(base);
    fs::create_directories(base);
    const fs::path input = base / "dense.ply";
    const fs::path output = base / "output";
    write_many_splats(input, 24);

    ConvertConfig config;
    config.input_path = input;
    config.output_dir = output;
    config.output_format = OutputFormat::Lcc2;
    config.include_env = false;
    config.lod.generate = true;
    config.lod.levels = 2;
    config.lod.reduction = 2;
    config.lod.method = LodMethod::Decimate;
    config.lod.max_leaf_splats = 6;
    config.lod.payload_layout = Lcc2PayloadLayout::Chunked;
    config.lod.max_payload_splats = 7;
    config.splat_transform_path = PLY2LCC_FAKE_SPLAT_TRANSFORM;
    ConvertApp(config).run();

    const std::string metadata = read_text(output / "meta.lcc2");
    EXPECT_TRUE(fs::exists(output / "data/3dgs/lod_1_chunk_0.spz"));
    EXPECT_TRUE(fs::exists(output / "data/3dgs/lod_1_chunk_3.spz"));
    EXPECT_NE(metadata.find("data/3dgs/lod_1_chunk_3.spz"), std::string::npos);
    EXPECT_NE(metadata.find("\"name\": 5, \"start\": 0, \"count\": 6"), std::string::npos);
}

TEST(Lcc2WriterTest, ChunkedLayoutRejectsPayloadSmallerThanANode) {
    const fs::path base = fs::temp_directory_path() / "ply2lcc_lcc2_chunk_too_small_test";
    fs::remove_all(base);
    fs::create_directories(base);
    const fs::path input = base / "dense.ply";
    write_many_splats(input, 8);
    ConvertConfig config;
    config.input_path = input;
    config.output_dir = base / "output";
    config.output_format = OutputFormat::Lcc2;
    config.include_env = false;
    config.lod.generate = true;
    config.lod.levels = 2;
    config.lod.method = LodMethod::Decimate;
    config.lod.max_leaf_splats = 8;
    config.lod.payload_layout = Lcc2PayloadLayout::Chunked;
    config.lod.max_payload_splats = 4;
    config.splat_transform_path = PLY2LCC_FAKE_SPLAT_TRANSFORM;
    EXPECT_THROW(ConvertApp(config).run(), std::runtime_error);
}

TEST(Lcc2WriterTest, GeneratedHierarchyMetadataAndPayloadOrderingAreDeterministic) {
    const fs::path base = fs::temp_directory_path() / "ply2lcc_lcc2_deterministic_tree_test";
    fs::remove_all(base);
    fs::create_directories(base);
    const fs::path input = base / "dense.ply";
    write_many_splats(input, 64);
    auto run = [&](const fs::path& output) {
        ConvertConfig config;
        config.input_path = input;
        config.output_dir = output;
        config.output_format = OutputFormat::Lcc2;
        config.include_env = false;
        config.lod.generate = true;
        config.lod.levels = 3;
        config.lod.reduction = 2;
        config.lod.method = LodMethod::Decimate;
        config.lod.max_leaf_splats = 8;
        config.lod.max_refinement_cost = 10;
        config.splat_transform_path = PLY2LCC_FAKE_SPLAT_TRANSFORM;
        ConvertApp(config).run();
    };
    run(base / "first");
    run(base / "second");
    EXPECT_EQ(read_text(base / "first/meta.lcc2"), read_text(base / "second/meta.lcc2"));
    for (int level = 0; level < 3; ++level) {
        const fs::path a = base / "first/data/3dgs" / ("lod_" + std::to_string(level) + ".spz");
        const fs::path b = base / "second/data/3dgs" / ("lod_" + std::to_string(level) + ".spz");
        EXPECT_EQ(fs::file_size(a), fs::file_size(b));
    }
}
