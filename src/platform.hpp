#ifndef PLY2LCC_PLATFORM_HPP
#define PLY2LCC_PLATFORM_HPP

#include <filesystem>
#include <fstream>
#include <cstdio>
#include <cstdint>
#include <cstddef>
#include <cerrno>
#include <cstdlib>

#include <vector>
#include <string>

#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#include <process.h>
#else
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/wait.h>
#endif

namespace platform {
namespace fs = std::filesystem;

/// File handle for memory mapping operations
struct FileHandle {
#ifdef _WIN32
    HANDLE file = INVALID_HANDLE_VALUE;
    HANDLE mapping = nullptr;
#else
    int fd = -1;
#endif
    std::size_t file_size = 0;

    bool valid() const {
#ifdef _WIN32
        return file != INVALID_HANDLE_VALUE;
#else
        return fd >= 0;
#endif
    }
};

/// Memory access pattern hints
enum class AccessHint { Sequential, Random, WillNeed, DontNeed };

/// Open file for memory mapping (read-only)
inline FileHandle file_open(const fs::path& path) {
    FileHandle h;
#ifdef _WIN32
    h.file = CreateFileW(path.wstring().c_str(), GENERIC_READ, FILE_SHARE_READ,
                         nullptr, OPEN_EXISTING,
                         FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN, nullptr);
    if (h.file != INVALID_HANDLE_VALUE) {
        LARGE_INTEGER size;
        if (GetFileSizeEx(h.file, &size)) {
            h.file_size = static_cast<std::size_t>(size.QuadPart);
        }
    }
#else
    h.fd = ::open(path.c_str(), O_RDONLY);
    if (h.fd >= 0) {
        struct stat st;
        if (::fstat(h.fd, &st) == 0) {
            h.file_size = static_cast<std::size_t>(st.st_size);
        }
    }
#endif
    return h;
}

/// Close file handle and release resources
inline void file_close(FileHandle& h) {
#ifdef _WIN32
    if (h.mapping) { CloseHandle(h.mapping); h.mapping = nullptr; }
    if (h.file != INVALID_HANDLE_VALUE) { CloseHandle(h.file); h.file = INVALID_HANDLE_VALUE; }
#else
    if (h.fd >= 0) { ::close(h.fd); h.fd = -1; }
#endif
    h.file_size = 0;
}

/// Map region of file into memory (read-only)
/// Returns nullptr on failure
inline void* mmap_read(FileHandle& h, std::size_t offset, std::size_t length) {
    if (!h.valid()) return nullptr;
#ifdef _WIN32
    if (!h.mapping) {
        h.mapping = CreateFileMappingW(h.file, nullptr, PAGE_READONLY, 0, 0, nullptr);
        if (!h.mapping) return nullptr;
    }
    DWORD offset_high = static_cast<DWORD>(offset >> 32);
    DWORD offset_low = static_cast<DWORD>(offset & 0xFFFFFFFF);
    return MapViewOfFile(h.mapping, FILE_MAP_READ, offset_high, offset_low, length);
#else
    void* addr = ::mmap(nullptr, length, PROT_READ, MAP_PRIVATE, h.fd, static_cast<off_t>(offset));
    return (addr == MAP_FAILED) ? nullptr : addr;
#endif
}

/// Unmap previously mapped region
inline void munmap(void* addr, std::size_t length) {
    if (!addr) return;
#ifdef _WIN32
    (void)length;
    UnmapViewOfFile(addr);
#else
    ::munmap(addr, length);
#endif
}

/// Advise kernel about memory access pattern
inline void madvise(void* addr, std::size_t length, AccessHint hint) {
    if (!addr || length == 0) return;
#ifdef _WIN32
    if (hint == AccessHint::Sequential || hint == AccessHint::WillNeed) {
        WIN32_MEMORY_RANGE_ENTRY range;
        range.VirtualAddress = addr;
        range.NumberOfBytes = length;
        PrefetchVirtualMemory(GetCurrentProcess(), 1, &range, 0);
    }
    // Random and DontNeed have no direct Windows equivalent
#else
    int advice = MADV_NORMAL;
    switch (hint) {
        case AccessHint::Sequential: advice = MADV_SEQUENTIAL; break;
        case AccessHint::Random:     advice = MADV_RANDOM; break;
        case AccessHint::WillNeed:   advice = MADV_WILLNEED; break;
        case AccessHint::DontNeed:   advice = MADV_DONTNEED; break;
    }
    ::madvise(addr, length, advice);
#endif
}

/// Open output file stream (takes fs::path only, preventing accidental std::string overload)
inline std::ofstream ofstream_open(const fs::path& path,
                                   std::ios::openmode mode = std::ios::binary) {
    return std::ofstream(path, mode);
}

/// Open input file stream (takes fs::path only, preventing accidental std::string overload)
inline std::ifstream ifstream_open(const fs::path& path,
                                   std::ios::openmode mode = std::ios::binary) {
    return std::ifstream(path, mode);
}

/// Open FILE* with Unicode path support
/// Caller responsible for fclose()
inline FILE* fopen(const fs::path& path, const char* mode) {
#ifdef _WIN32
    wchar_t wmode[8];
    std::mbstowcs(wmode, mode, 8);
    return _wfopen(path.wstring().c_str(), wmode);
#else
    return std::fopen(path.c_str(), mode);
#endif
}

inline fs::path resolve_executable(const fs::path& executable) {
    if (executable.empty()) return {};

    auto resolve_candidate = [](const fs::path& candidate) -> fs::path {
        if (candidate.empty()) return {};
        if (fs::exists(candidate)) return candidate;
#ifdef _WIN32
        if (!candidate.extension().empty()) return {};
        for (const auto& suffix : { ".exe", ".cmd", ".bat" }) {
            const fs::path with_suffix(candidate.string() + suffix);
            if (fs::exists(with_suffix)) return with_suffix;
        }
#endif
        return {};
    };

    if (const fs::path resolved = resolve_candidate(executable); !resolved.empty()) return resolved;

    const fs::path name = executable.filename();
    if (name.empty()) return executable;

    std::vector<fs::path> search_roots;
    search_roots.push_back(fs::current_path());
    fs::path dir = fs::current_path();
    while (!dir.empty()) {
        search_roots.push_back(dir);
        const fs::path parent = dir.parent_path();
        if (parent == dir) break;
        dir = parent;
    }

    for (const auto& root : search_roots) {
        const fs::path local = root / "node_modules" / ".bin" / name;
        if (const fs::path resolved = resolve_candidate(local); !resolved.empty()) return resolved;
        const fs::path direct = root / name;
        if (const fs::path resolved = resolve_candidate(direct); !resolved.empty()) return resolved;
    }

    const char* path_env = std::getenv("PATH");
    if (path_env) {
        std::string paths = path_env;
        size_t start = 0;
        while (start <= paths.size()) {
            const size_t end = paths.find(
#ifdef _WIN32
                ';',
#else
                ':',
#endif
                start);
            const std::string item = paths.substr(start, end == std::string::npos ? std::string::npos : end - start);
            if (!item.empty()) {
                const fs::path from_path = fs::path(item) / name;
                if (const fs::path resolved = resolve_candidate(from_path); !resolved.empty()) return resolved;
            }
            if (end == std::string::npos) break;
            start = end + 1;
        }
    }

    return executable;
}

/// Run a process without involving a shell. The first item is the executable.
/// Returns its exit code, or 127 when it could not be launched.
inline int run_process(const std::vector<fs::path>& arguments) {
    if (arguments.empty()) return 127;
    std::vector<fs::path> resolved = arguments;
    resolved[0] = resolve_executable(resolved[0]);
#ifdef _WIN32
    std::vector<std::wstring> storage;
    storage.reserve(resolved.size());
    for (const auto& argument : resolved) storage.push_back(argument.wstring());
    std::vector<const wchar_t*> argv;
    argv.reserve(storage.size() + 1);
    for (const auto& argument : storage) argv.push_back(argument.c_str());
    argv.push_back(nullptr);
    const intptr_t result = _wspawnvp(_P_WAIT, argv.front(), argv.data());
    return result == -1 ? 127 : static_cast<int>(result);
#else
    const pid_t pid = ::fork();
    if (pid < 0) return 127;
    if (pid == 0) {
        std::vector<std::string> storage;
        storage.reserve(resolved.size());
        for (const auto& argument : resolved) storage.push_back(argument.string());
        std::vector<char*> argv;
        argv.reserve(storage.size() + 1);
        for (auto& argument : storage) argv.push_back(argument.data());
        argv.push_back(nullptr);
        ::execvp(argv.front(), argv.data());
        ::_exit(127);
    }
    int status = 0;
    while (::waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR) return 127;
    }
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    if (WIFSIGNALED(status)) return 128 + WTERMSIG(status);
    return 127;
#endif
}

/// Set up console and command-line for UTF-8 on Windows.
/// Returns UTF-8 encoded argv (on Linux, returns the original argv as-is).
struct Utf8Args {
    std::vector<std::string> storage;
    std::vector<char*> argv;
    int argc = 0;
};

inline Utf8Args utf8_argv(int argc, char** argv) {
    Utf8Args result;
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);

    int wargc;
    wchar_t** wargv = CommandLineToArgvW(GetCommandLineW(), &wargc);
    if (wargv) {
        result.storage.resize(wargc);
        result.argv.resize(wargc);
        for (int i = 0; i < wargc; ++i) {
            int len = WideCharToMultiByte(CP_UTF8, 0, wargv[i], -1, nullptr, 0, nullptr, nullptr);
            result.storage[i].resize(len - 1);
            WideCharToMultiByte(CP_UTF8, 0, wargv[i], -1, result.storage[i].data(), len, nullptr, nullptr);
            result.argv[i] = result.storage[i].data();
        }
        result.argc = wargc;
        LocalFree(wargv);
        return result;
    }
#endif
    // Fallback / Linux: use original argv
    result.argv.assign(argv, argv + argc);
    result.argc = argc;
    return result;
}

} // namespace platform

#endif // PLY2LCC_PLATFORM_HPP
