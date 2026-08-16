#include "convert_app.hpp"
#include "config.h"
#include "spatial_grid.hpp"
#include "grid_encoder.hpp"
#include "lcc_writer.hpp"
#include "lcc2_writer.hpp"
#include "collision_encoder.hpp"
#include "splat_buffer.hpp"
#include "platform.hpp"

#include <iostream>
#include <filesystem>
#include <algorithm>
#include <regex>
#include <stdexcept>
#include <chrono>
#include <cctype>

namespace fs = std::filesystem;

namespace ply2lcc {

ConvertApp::ConvertApp(int argc, char** argv)
    : argc_(argc), argv_(argv) {}

ConvertApp::ConvertApp(const ConvertConfig& config)
    : argc_(0), argv_(nullptr)
    , input_path_(config.input_path)
    , output_dir_(config.output_dir)
    , cell_size_x_(config.cell_size_x)
    , cell_size_y_(config.cell_size_y)
    , single_lod_(config.single_lod)
    , output_format_(config.output_format)
    , lcc2_payload_files_(config.lcc2_payload_paths)
    , splat_transform_path_(config.splat_transform_path)
    , lod_settings_(config.lod)
    , include_env_(config.include_env)
    , include_collision_(config.include_collision)
    , include_poses_(config.include_poses)
    , env_file_(config.env_path)
    , collision_file_(config.collision_path)
    , poses_file_(config.poses_path)
{
    // Derive input_dir_ and base_name_ from input_path_
    if (fs::is_directory(input_path_)) {
        input_dir_ = input_path_;
        base_name_ = "point_cloud";
    } else {
        input_dir_ = input_path_.parent_path();
        base_name_ = input_path_.stem().u8string();
    }
}

void ConvertApp::setProgressCallback(ProgressCallback cb) {
    progress_cb_ = std::move(cb);
}

void ConvertApp::setLogCallback(LogCallback cb) {
    log_cb_ = std::move(cb);
}

void ConvertApp::reportProgress(int percent, const std::string& msg) {
    if (progress_cb_) {
        progress_cb_(percent, msg);
    }
}

void ConvertApp::log(const std::string& msg) {
    if (log_cb_) {
        log_cb_(msg);
    } else {
        std::cout << msg;
    }
}

void ConvertApp::run() {
    reportProgress(0, "Starting conversion...");

    parseArgs();
    findPlyFiles();

    reportProgress(2, "Found " + std::to_string(lod_files_.size()) + " LOD files");

    // Create output directory
    fs::create_directories(output_dir_);
    log("Output: " + output_dir_.u8string() + "\n");
    log("Cell size: " + std::to_string(cell_size_x_) + " x " + std::to_string(cell_size_y_) + "\n");

    if (lod_settings_.generate) {
        generateLods();
        reportProgress(4, "Generated " + std::to_string(lod_files_.size()) + " LOD levels");
    }

    // Step 1: Build spatial grid
    reportProgress(5, "Building spatial grid...");
    log("\nPhase 1: Building spatial grid...\n");
    SpatialGrid grid = SpatialGrid::from_files(lod_files_, cell_size_x_, cell_size_y_);

    log("Global bbox: (" + std::to_string(grid.bbox().min.x) + ", " +
        std::to_string(grid.bbox().min.y) + ", " + std::to_string(grid.bbox().min.z) +
        ") - (" + std::to_string(grid.bbox().max.x) + ", " +
        std::to_string(grid.bbox().max.y) + ", " + std::to_string(grid.bbox().max.z) + ")\n");
    log("Created " + std::to_string(grid.cells().size()) + " grid cells\n");
    log("SH: " + (grid.has_sh() ? "degree " + std::to_string(grid.sh_degree()) +
        " (" + std::to_string(grid.num_f_rest()) + " coefficients)" : std::string("none")) + "\n");

    if (output_format_ == OutputFormat::Lcc2) {
        if (!collision_file_.empty()) {
            log("Warning: collision meshes are not yet included in LCC2 output\n");
        }
        if (!poses_file_.empty()) {
            log("Warning: trajectory poses are not part of the LCC2 v0.0.3 specification and were omitted\n");
        }

        reportProgress(90, "Writing LCC2 package...");
        log("\nPhase 2: Writing LCC2 package...\n");
        Lcc2Writer writer(output_dir_, splat_transform_path_);
        writer.write(grid, lod_files_, lcc2_payload_files_, env_file_, base_name_, lod_errors_);
        if (!generated_lod_dir_.empty()) {
            fs::remove_all(generated_lod_dir_);
        }
        reportProgress(100, "Conversion complete!");
        log("\nConversion complete!\n");
        log("Format: LCC2 v0.0.3\n");
        log("Output: " + output_dir_.u8string() + "\n");
        return;
    }

    // Step 2: Encode all data
    reportProgress(15, "Encoding splats...");
    log("\nPhase 2: Encoding splats...\n");
    GridEncoder encoder;
    encoder.set_progress_callback([this](int pct, const std::string& msg) {
        reportProgress(15 + pct * 75 / 100, msg);
    });
    LccData data = encoder.encode(grid, lod_files_);

    // Step 3: Encode environment (if exists)
    if (!env_file_.empty() && fs::exists(env_file_)) {
        log("\nPhase 3: Encoding environment...\n");
        data.environment = encoder.encode_environment(env_file_, grid.has_sh());
        log("  Environment: " + std::to_string(data.environment.count) + " splats\n");
    }

    // Step 4: Encode collision mesh (if exists)
    if (!collision_file_.empty() && fs::exists(collision_file_)) {
        reportProgress(85, "Encoding collision mesh...");
        log("\nPhase 4: Encoding collision mesh...\n");
        CollisionEncoder collision_encoder;
        collision_encoder.set_log_callback([this](const std::string& msg) { log(msg); });
        // Pass scene bbox so collision cells align with splat grid cells
        data.collision = collision_encoder.encode(collision_file_, cell_size_x_, cell_size_y_, grid.bbox());
        if (!data.collision.empty()) {
            log("  Collision: " + std::to_string(data.collision.total_triangles()) + " triangles, " +
                std::to_string(data.collision.cells.size()) + " cells\n");
        }
    }

    // Step 5: adding poses path if exists
    if (!poses_file_.empty() && fs::exists(poses_file_)) {
        data.poses_path = poses_file_;
        log("\nIncluded poses from: " + poses_file_.u8string() + "\n");
    }

    // Step 5: Write all output files
    reportProgress(90, "Writing output files...");
    log("\nPhase 5: Writing LCC data...\n");
    LccWriter writer(output_dir_);
    writer.write(data);

    reportProgress(100, "Conversion complete!");

    log("\nConversion complete!\n");
    log("Total splats: " + std::to_string(data.total_splats) + "\n");
    log("Output: " + output_dir_.u8string() + "\n");
}

void ConvertApp::printUsage() {
    std::cerr << "ply2lcc v" PLY2LCC_VERSION " (built " PLY2LCC_BUILD_TIMESTAMP " UTC)\n"
              << "\n"
              << "Usage: " << argv_[0] << " -i <input.ply> -o <output_dir> [options]\n"
              << "\n"
              << "Options:\n"
              << "  -e <path>          Include environment splats from specified .ply file\n"
              << "  -m <path>          Include collision mesh from specified .ply or .obj file\n"
              << "  -p <path>          Include trajectory poses from specified .json file\n"
              << "  --single-lod       Use only LOD0 even if more LOD files exist\n"
              << "  --format FORMAT    Output format: lcc (default) or lcc2\n"
              << "  --lcc2             Shortcut for --format lcc2\n"
              << "  --lcc2-payload P   Store PLY/SPZ payload P in LCC2 output (repeat per LOD)\n"
              << "  --generate-lod     Generate offline LOD levels (LCC2 only)\n"
              << "  --splat-transform P Path to splat-transform (default: search PATH)\n"
              << "  --lod-levels N     Total levels including original (default: 5)\n"
              << "  --lod-reduction N  Approximate reduction per level (default: 4)\n"
              << "  --lod-method M     cluster (default) or decimate\n"
              << "  --lod-debug        Print detailed LOD generation statistics\n"
              << "  --cell-size X,Y    Grid cell size in meters (default: 30,30)\n";
}

void ConvertApp::parseArgs() {
    for (int i = 1; i < argc_; ++i) {
        std::string arg = argv_[i];
        if (arg == "-i" && i + 1 < argc_) {
            input_path_ = fs::u8path(argv_[++i]);
        } else if (arg == "-o" && i + 1 < argc_) {
            output_dir_ = fs::u8path(argv_[++i]);
        } else if (arg == "-e" && i + 1 < argc_) {
            env_file_ = fs::u8path(argv_[++i]);
            include_env_ = true;
        } else if (arg == "-m" && i + 1 < argc_) {
            collision_file_ = fs::u8path(argv_[++i]);
            include_collision_ = true;
        } else if (arg == "-p" && i + 1 < argc_) {
            poses_file_ = fs::u8path(argv_[++i]);
            include_poses_ = true;
        } else if (arg == "--single-lod") {
            single_lod_ = true;
        } else if (arg == "--lcc2") {
            output_format_ = OutputFormat::Lcc2;
        } else if (arg == "--format") {
            if (i + 1 >= argc_) {
                throw std::runtime_error("Missing value for --format (expected lcc or lcc2)");
            }
            std::string format = argv_[++i];
            if (format == "lcc") {
                output_format_ = OutputFormat::Lcc;
            } else if (format == "lcc2") {
                output_format_ = OutputFormat::Lcc2;
            } else {
                throw std::runtime_error("Invalid output format: " + format + " (expected lcc or lcc2)");
            }
        } else if (arg == "--lcc2-payload") {
            if (i + 1 >= argc_) {
                throw std::runtime_error("Missing path for --lcc2-payload");
            }
            lcc2_payload_files_.push_back(fs::u8path(argv_[++i]));
        } else if (arg == "--generate-lod") {
            lod_settings_.generate = true;
        } else if (arg == "--splat-transform") {
            if (i + 1 >= argc_) throw std::runtime_error("Missing path for --splat-transform");
            splat_transform_path_ = fs::u8path(argv_[++i]);
        } else if (arg == "--lod-levels") {
            if (i + 1 >= argc_) throw std::runtime_error("Missing value for --lod-levels");
            lod_settings_.levels = static_cast<size_t>(std::stoul(argv_[++i]));
        } else if (arg == "--lod-reduction") {
            if (i + 1 >= argc_) throw std::runtime_error("Missing value for --lod-reduction");
            lod_settings_.reduction = static_cast<size_t>(std::stoul(argv_[++i]));
        } else if (arg == "--lod-method") {
            if (i + 1 >= argc_) throw std::runtime_error("Missing value for --lod-method");
            const std::string method = argv_[++i];
            if (method == "cluster") lod_settings_.method = LodMethod::Cluster;
            else if (method == "decimate") lod_settings_.method = LodMethod::Decimate;
            else throw std::runtime_error("Invalid --lod-method: " + method + " (expected cluster or decimate)");
        } else if (arg == "--lod-debug") {
            lod_settings_.debug = true;
        } else if (arg == "--cell-size" && i + 1 < argc_) {
            if (sscanf(argv_[++i], "%f,%f", &cell_size_x_, &cell_size_y_) != 2) {
                throw std::runtime_error("Invalid cell-size format. Use X,Y");
            }
        } else if (arg == "-h" || arg == "--help") {
            printUsage();
            std::exit(EXIT_SUCCESS);
        }
    }

    if (input_path_.empty() || output_dir_.empty()) {
        printUsage();
        throw std::runtime_error("Missing required arguments: -i and -o");
    }

    if (!fs::exists(input_path_)) {
        throw std::runtime_error("Input file not found: " + input_path_.u8string());
    }

    // Extract directory and base name
    input_dir_ = input_path_.parent_path();
    if (input_dir_.empty()) input_dir_ = ".";

    std::string extension = input_path_.extension().u8string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (extension == ".spz") {
        if (output_format_ != OutputFormat::Lcc2) {
            throw std::runtime_error("SPZ input requires --format lcc2");
        }
        input_spz_path_ = input_path_;
        fs::path companion = input_path_;
        companion.replace_extension(".ply");
        if (!fs::exists(companion)) {
            throw std::runtime_error("SPZ input requires a matching PLY for Gaussian decoding: " + companion.u8string());
        }
        input_path_ = companion;
        if (lcc2_payload_files_.empty()) lcc2_payload_files_.push_back(input_spz_path_);
    } else if (extension != ".ply") {
        throw std::runtime_error("Input file must have .ply or .spz extension");
    }
    base_name_ = input_path_.stem().u8string();
    input_dir_ = input_path_.parent_path();
    if (input_dir_.empty()) input_dir_ = ".";

    if (lod_settings_.generate && output_format_ != OutputFormat::Lcc2) {
        throw std::runtime_error("--generate-lod requires --format lcc2");
    }

    log("Input: " + input_path_.u8string() + "\n");
}

void ConvertApp::findPlyFiles() {
    // LOD0 is the base file
    lod_files_.push_back(input_path_);

    // Find numbered LOD files: base_1.ply, base_2.ply, ...
    if (lod_settings_.generate) {
        log("LOD generation requested; existing numbered LOD files will be ignored\n");
    }
    std::regex pattern(base_name_ + "_(\\d+)\\.ply");
    std::vector<std::pair<int, fs::path>> numbered_files;

    for (const auto& entry : fs::directory_iterator(input_dir_)) {
        if (lod_settings_.generate) break;
        std::string filename = entry.path().filename().u8string();
        std::smatch match;
        if (std::regex_match(filename, match, pattern)) {
            int num = std::stoi(match[1].str());
            numbered_files.emplace_back(num, entry.path());
        }
    }

    // Sort by number
    std::sort(numbered_files.begin(), numbered_files.end());

    // Add files until first gap (must be continuous from 1)
    int expected = 1;
    for (const auto& [num, path] : numbered_files) {
        if (num != expected) break;
        lod_files_.push_back(path);
        expected++;
    }

    // Print LOD info
    log("Found " + std::to_string(lod_files_.size()) + " LOD level" +
        (lod_files_.size() > 1 ? "s" : "") + ":\n");

    for (size_t i = 0; i < lod_files_.size(); ++i) {
        std::string filename = lod_files_[i].filename().u8string();
        if (single_lod_ && i > 0) {
            log("  LOD" + std::to_string(i) + ": " + filename + " (skipped: --single-lod)\n");
        } else {
            log("  LOD" + std::to_string(i) + ": " + filename + "\n");
        }
    }

    // Apply single_lod filter
    if (single_lod_ && lod_files_.size() > 1) {
        lod_files_.resize(1);
    }

    if (!lcc2_payload_files_.empty() && !lod_settings_.generate) {
        if (output_format_ != OutputFormat::Lcc2) {
            throw std::runtime_error("--lcc2-payload requires --format lcc2");
        }
        if (lcc2_payload_files_.size() != lod_files_.size()) {
            throw std::runtime_error("Provide exactly one --lcc2-payload for each LOD PLY file");
        }
        for (const auto& payload : lcc2_payload_files_) {
            if (!fs::exists(payload)) {
                throw std::runtime_error("LCC2 payload file not found: " + payload.u8string());
            }
        }
    }

    // Validate environment file (no auto-detect)
    if (include_env_) {
        if (!env_file_.empty() && fs::exists(env_file_)) {
            log("Environment: " + env_file_.u8string() + "\n");
        } else {
            if (!env_file_.empty()) {
                log("Warning: environment file not found: " + env_file_.u8string() + "\n");
            }
            env_file_.clear();
        }
    }

    // Validate collision file (no auto-detect)
    if (include_collision_) {
        if (!collision_file_.empty() && fs::exists(collision_file_)) {
            log("Collision: " + collision_file_.u8string() + "\n");
        } else {
            if (!collision_file_.empty()) {
                log("Warning: collision file not found: " + collision_file_.u8string() + "\n");
            }
            collision_file_.clear();
        }
    }

    // Validate poses file (no auto-detect)
    if (include_poses_) {
        if (!poses_file_.empty() && fs::exists(poses_file_)) {
            log("Poses: " + poses_file_.u8string() + "\n");
        } else {
            if (!poses_file_.empty()) {
                log("Warning: poses file not found: " + poses_file_.u8string() + "\n");
            }
            poses_file_.clear();
        }
    }
}

void ConvertApp::generateLods() {
    const auto started = std::chrono::steady_clock::now();
    SplatBuffer source;
    if (!source.initialize(input_path_)) {
        throw std::runtime_error("Failed to read LOD source " + input_path_.u8string() + ": " + source.error());
    }
    if (!input_spz_path_.empty()) {
        Lcc2Writer::validate_spz_v4(input_spz_path_, source.size(), source.sh_degree());
    } else if (lcc2_payload_files_.size() == 1 && lcc2_payload_files_.front().extension() == ".spz") {
        Lcc2Writer::validate_spz_v4(lcc2_payload_files_.front(), source.size(), source.sh_degree());
    }
    log("\nLOD generation:\n");
    log("  method: " + std::string(lod_settings_.method == LodMethod::Cluster ? "cluster" : "decimate") + "\n");
    log("  reduction target: " + std::to_string(lod_settings_.reduction) + "x\n");
    log("  input splats: " + std::to_string(source.size()) + "\n");

    LodGenerator generator(lod_settings_);
    std::vector<LodLevel> levels = generator.generate(source.to_vector());
    const BBox source_bbox = source.compute_bbox();
    auto cell_index = [&](const Vec3f& pos) {
        const int x = std::max(0, std::min(65535, static_cast<int>(std::floor((pos.x - source_bbox.min.x) / cell_size_x_))));
        const int y = std::max(0, std::min(65535, static_cast<int>(std::floor((pos.y - source_bbox.min.y) / cell_size_y_))));
        return (static_cast<uint32_t>(y) << 16) | static_cast<uint32_t>(x);
    };
    generated_lod_dir_ = output_dir_ / ".generated_lod";
    fs::create_directories(generated_lod_dir_);
    lod_files_.clear();
    lcc2_payload_files_.clear();
    lod_errors_.clear();
    lod_stats_.clear();
    for (size_t lod = 0; lod < levels.size(); ++lod) {
        std::stable_sort(levels[lod].splats.begin(), levels[lod].splats.end(),
                         [&](const Splat& a, const Splat& b) { return cell_index(a.pos) < cell_index(b.pos); });
        const fs::path path = generated_lod_dir_ / ("lod_" + std::to_string(lod) + ".ply");
        LodGenerator::write_binary_ply(path, levels[lod].splats, source.num_f_rest());
        lod_files_.push_back(path);
        lod_errors_.push_back(levels[lod].error);
        lod_stats_.push_back(levels[lod].stats);
        log("  LOD" + std::to_string(lod) + ": " + std::to_string(levels[lod].splats.size()) +
            " splats, error " + std::to_string(levels[lod].error) + "\n");
        if (lod_settings_.debug && lod + 1 < levels.size()) {
            const auto& stats = levels[lod].stats;
            log("    clusters: " + std::to_string(stats.cluster_count) +
                ", avg/min/max size: " + std::to_string(stats.average_cluster_size) + "/" +
                std::to_string(stats.min_cluster_size) + "/" + std::to_string(stats.max_cluster_size) +
                ", mean covariance error: " + std::to_string(stats.mean_covariance_error) +
                ", rejected merges: " + std::to_string(stats.rejected_merges) + "\n");
        }
    }

    log("  encoding SPZ v4 payloads with splat-transform\n");
    for (size_t lod = 0; lod < lod_files_.size(); ++lod) {
        const fs::path spz_path = generated_lod_dir_ / ("lod_" + std::to_string(lod) + ".spz");
        const std::vector<fs::path> command{
            splat_transform_path_, "--quiet", "--overwrite", "--spz-version", "4",
            lod_files_[lod], spz_path
        };
        const int exit_code = platform::run_process(command);
        if (exit_code != 0) {
            throw std::runtime_error(
                "splat-transform failed while encoding LOD" + std::to_string(lod) +
                " as SPZ v4 (exit code " + std::to_string(exit_code) +
                "). Install @playcanvas/splat-transform or pass --splat-transform <path>");
        }
        if (!fs::exists(spz_path)) {
            throw std::runtime_error("splat-transform did not create " + spz_path.u8string());
        }
        Lcc2Writer::validate_spz_v4(spz_path, levels[lod].splats.size(), source.sh_degree());
        lcc2_payload_files_.push_back(spz_path);
    }
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    log("  generation time: " + std::to_string(seconds) + " seconds\n");
}

} // namespace ply2lcc
