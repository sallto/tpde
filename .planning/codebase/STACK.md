# Technology Stack

**Analysis Date:** 2026-01-20

## Languages

**Primary:**
- C++20 - Core compiler framework (`tpde/`, `tpde-llvm/`, `tpde-encodegen/`)
  - Uses concepts, CRTP pattern, constexpr, std::format
  - Header-only templates + minimal .cpp implementation
  - No RTTI, no exceptions

**Secondary:**
- C11 - Instruction encoding libraries (`deps/fadec/`, `deps/disarm/`)
  - Zero-dependency C for freestanding environments
  - Fadec: x86-64 encoding (used via `FADEC_ENCODE2` API)
  - Disarm: AArch64 encoding (used via `de64_*` API)

- Python 3.10+ - Test infrastructure and code generation
  - LLVM lit test runner (`lit.llvm`)
  - Python scripts in `tpde-llvm/test/` for test management
  - Code generation in `deps/fadec/` and `deps/disarm/`

## Runtime

**Environment:**
- Linux (Ubuntu tested via CI)
- ELF-based x86-64 and AArch64 (Armv8.1) platforms

**Package Manager:**
- CMake 3.25+ (minimum)
- Git submodules for bundled dependencies
- No package manager for runtime dependencies

**Build System:**
- CMake with Ninja generator (preferred)
- CMake 3.25 required (per `CMakeLists.txt`)
- LLVM lit integration for test execution

## Frameworks

**Core:**
- CRTP (Curiously Recurring Template Pattern) - Architecture-independent compilation logic
- Header-only template design - `CompilerBase<Adaptor, Derived, Config>`
- Custom assembler - Direct ELF generation (`AssemblerElf.cpp`)

**Testing:**
- LLVM lit (FileCheck-based test framework)
  - Test format: `.ll` (LLVM-IR), `.vir`, `.tir` (custom IR formats)
  - Config: `tpde/test/filetest/lit.cfg.py`, `tpde-llvm/test/lit.cfg.py`
  - Execution: `lit -sv` via CMake targets

**Build/Dev:**
- CMake - Build configuration and dependency management
- Ninja - Build tool (CI uses `-G Ninja`)
- clang-format - Code formatting (`.clang-format` based on LLVM style)
- ccache - Optional compilation caching (Debug builds)

## Key Dependencies

**Critical (bundled via git submodules):**
- `deps/spdlog/` (v?) - Logging framework
  - Header-only with C++20 std::format support
  - Used conditionally via `TPDE_LOGGING` (DebugOnly/ON/OFF)
  - Compiled with `SPDLOG_NO_EXCEPTIONS`

- `deps/fadec/` - x86-64 instruction encoding
  - Fast encoder for x86-64 (no decode functionality used)
  - Used for x64 target support (`TPDE_X64`)
  - API: `fe_enc64` v1, `fe64_*` v2 (v2 used via `FADEC_ENCODE2`)

- `deps/disarm/` - AArch64 instruction encoding
  - Fast encoder for AArch64
  - Used for A64 target support (`TPDE_A64`)
  - API: `de64_*` functions

- `deps/hopscotch-map/` v2.3.1 - Fast hash map
  - Header-only, used by `tpde-llvm`
  - Better performance than std::unordered_map
  - Prime growth policy default

- `deps/args/` 6.4.6 - Argument parsing
  - Header-only, used by `tpde-llc` and `tpde-lli` tools
  - Python argparse-style C++ interface

**Infrastructure (optional):**
- LLVM 20.1 (primary) or 19.1 - For TPDE-LLVM and TPDE-Encodegen
  - Required for LLVM-IR backend and encodegen tool
  - Linked dynamically by default (`TPDE_LINK_LLVM_STATIC=FALSE`)
  - Components: core, irreader, jitlink, orcjit, passes, etc.
  - Tools required for testing: lit, llc, llvm-objdump, FileCheck

- `deps/doxygen-awesome-css/` - Documentation CSS (not loaded at runtime)

## Configuration

**Environment:**
- CMake options control all features:
  - `TPDE_INCLUDE_TESTS=ON` - Enable test suite
  - `TPDE_ENABLE_ENCODEGEN=ON` - Build encodegen tool (needs LLVM)
  - `TPDE_ENABLE_LLVM=ON` - Build LLVM-IR backend (needs LLVM)
  - `TPDE_ENABLE_COVERAGE=OFF` - Enable coverage instrumentation (Clang only)
  - `TPDE_ENABLE_PCH=OFF` - Enable pre-compiled headers
  - `TPDE_BUILD_DOCS=OFF` - Build documentation
  - `TPDE_LOGGING=ON` - Enable logging with spdlog (DebugOnly/ON/OFF)
  - `TPDE_X64=ON` - Enable x86-64 support
  - `TPDE_A64=ON` - Enable AArch64 support
  - `TPDE_LINK_LLVM_STATIC=FALSE` - Link LLVM statically

**Build:**
- CMake build config files: `CMakeLists.txt` at root and per subdirectory
- Lockfile: None (uses git submodules for dependencies)

**Compiler flags:**
- `-Wall -Wextra -Wpedantic` (warnings)
- `-fno-rtti -fno-exceptions` (required)
- `-ffunction-sections -fdata-sections` + `--gc-sections` (linker optimization)
- Debug builds: `-D_GLIBCXX_ASSERTIONS`

## Platform Requirements

**Development:**
- C++20 compliant compiler: Clang 19+ or GCC 14+
- CMake 3.25+
- Ninja build tool (recommended)
- Python 3.10+
- LLVM 20.1 or 19.1 (only for TPDE-LLVM/Encodegen and tests)
  - Prefer LLVM 20.1 (LLVM 19.1 may have test failures due to different code generation)
- Git with submodules support (for cloning dependencies)

**Production:**
- Linux (Ubuntu tested)
- ELF-based OS
- x86-64 or AArch64 (Armv8.1) target architecture
- No runtime dependencies (all dependencies bundled or statically linked)

---

*Stack analysis: 2026-01-20*
