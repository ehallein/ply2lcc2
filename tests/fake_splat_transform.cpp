#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    if (argc < 3) return 2;
    bool requested_v4 = false;
    bool requested_lcc2_rotation = false;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--spz-version" && std::string(argv[i + 1]) == "4") {
            requested_v4 = true;
        }
        if (std::string(argv[i]) == "--rotate" && std::string(argv[i + 1]) == "90,0,0") {
            requested_lcc2_rotation = true;
        }
    }
    if (!requested_v4) return 5;
    fs::path input;
    for (int i = 1; i < argc - 1; ++i) {
        const fs::path candidate = fs::u8path(argv[i]);
        if (fs::exists(candidate) &&
            (candidate.extension() == ".ply" || candidate.extension() == ".spz")) {
            input = candidate;
            break;
        }
    }
    if (input.empty()) return 3;
    if (input.extension() == ".spz" && !requested_lcc2_rotation) return 6;
    const fs::path output = fs::u8path(argv[argc - 1]);

    uint32_t count = 0;
    uint8_t rest_count = 0;
    if (input.extension() == ".spz") {
        std::ifstream spz(input, std::ios::binary);
        std::array<uint8_t, 13> header{};
        spz.read(reinterpret_cast<char*>(header.data()), header.size());
        if (!spz) return 3;
        count = static_cast<uint32_t>(header[8]) |
                (static_cast<uint32_t>(header[9]) << 8) |
                (static_cast<uint32_t>(header[10]) << 16) |
                (static_cast<uint32_t>(header[11]) << 24);
        rest_count = header[12] == 0 ? 0 : (header[12] == 1 ? 9 : (header[12] == 2 ? 24 : 45));
    } else {
        std::ifstream ply(input, std::ios::binary);
        if (!ply) return 3;
        std::string line;
        while (std::getline(ply, line)) {
            if (line.rfind("element vertex ", 0) == 0) {
                count = static_cast<uint32_t>(std::stoul(line.substr(15)));
            } else if (line.rfind("property float f_rest_", 0) == 0) {
                ++rest_count;
            } else if (line == "end_header") {
                break;
            }
        }
    }
    const uint8_t degree = rest_count == 0 ? 0 : (rest_count == 9 ? 1 : (rest_count == 24 ? 2 : 3));

    std::array<uint8_t, 49> bytes{};
    bytes[0] = 'N'; bytes[1] = 'G'; bytes[2] = 'S'; bytes[3] = 'P';
    bytes[4] = 4;
    bytes[8] = static_cast<uint8_t>(count & 0xff);
    bytes[9] = static_cast<uint8_t>((count >> 8) & 0xff);
    bytes[10] = static_cast<uint8_t>((count >> 16) & 0xff);
    bytes[11] = static_cast<uint8_t>((count >> 24) & 0xff);
    bytes[12] = degree;
    bytes[13] = 12;
    bytes[15] = 1;
    bytes[16] = 32;
    bytes[32] = 1;
    bytes[40] = 9;
    std::ofstream spz(output, std::ios::binary);
    spz.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    return spz ? 0 : 4;
}
