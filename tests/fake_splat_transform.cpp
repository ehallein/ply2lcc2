#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    if (argc < 3) return 2;
    bool requested_v4 = false;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--spz-version" && std::string(argv[i + 1]) == "4") {
            requested_v4 = true;
        }
    }
    if (!requested_v4) return 5;
    const fs::path input = fs::u8path(argv[argc - 2]);
    const fs::path output = fs::u8path(argv[argc - 1]);

    std::ifstream ply(input, std::ios::binary);
    if (!ply) return 3;
    uint32_t count = 0;
    uint8_t rest_count = 0;
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
