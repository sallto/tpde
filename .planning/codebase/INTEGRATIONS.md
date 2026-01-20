# External Integrations

**Analysis Date:** 2026-01-20

## APIs & External Services

**Instruction Encoding Libraries:**
- **Fadec** - x86-64 instruction encoding
  - SDK/Client: `deps/fadec/include/fadec.h` (C API)
  - Auth: None (bundled submodule)
  - Integration: Linked via `fadec::fadec` CMake target
  - Usage: `tpde/include/tpde/x64/CompilerX64.hpp` uses `FADEC_ENCODE2` API
  - Enabled by: `TPDE_X64=ON` CMake option

- **Disarm** - AArch64 instruction encoding
  - SDK/Client: `deps/disarm/disarm64.h` (C/C++ API)
  - Auth: None (bundled submodule)
  - Integration: Linked via `disarm64::disarm64` CMake target
  - Usage: `tpde/include/tpde/arm64/CompilerA64.hpp` uses `de64_*` API
  - Enabled by: `TPDE_A64=ON` CMake option

**LLVM Toolchain Integration:**
- **LLVM** - Compiler infrastructure
  - SDK/Client: LLVM C++ API (header-only + lib)
  - Auth: None (system package or custom install)
  - Integration: `find_package(LLVM 20.1 CONFIG)` or `find_package(LLVM 19.1 CONFIG)`
  - Components: core, irreader, irprinter, jitlink, orcjit, passes, support, bitreader, bitstreamreader, targetparser, X86, AArch64, CodeGen, mc, asmparser, asmprinter
  - Usage:
    - `tpde-llvm/` - LLVM-IR to TPDE compilation (`LLVMCompiler.hpp`, `OrcCompiler.hpp`)
    - `tpde-encodegen/` - Code generation via LLVM MachineIR
  - Enabled by: `TPDE_ENABLE_LLVM=ON` and `TPDE_ENABLE_ENCODEGEN=ON`
  - Required tools: `clang-${LLVM_VERSION}`, `llvm-objdump`, `lit`, `llc`, `FileCheck`

## Data Storage

**Databases:**
- None (compiler does not use external databases)

**File Storage:**
- Local filesystem only
- ELF object files output (`AssemblerElf.cpp`)
- In-memory code mapping for JIT (`ElfMapper.cpp`)

**Caching:**
- ccache - Optional compilation caching for Debug builds
  - Enabled automatically if `ccache` found in Debug mode
  - Controlled by `CMAKE_CXX_COMPILER_LAUNCHER` in CMake

## Authentication & Identity

**Auth Provider:**
- None (no network authentication)
- All code runs locally

**Identity Management:**
- No external identity systems

## Monitoring & Observability

**Error Tracking:**
- None (no external error tracking service)
- Compile-time error handling via assertions (`TPDE_ASSERTS` macro in `tpde/include/tpde/base.hpp`)

**Logs:**
- spdlog (bundled dependency)
  - Config: `TPDE_LOGGING` (DebugOnly/ON/OFF)
  - Macros: `TPDE_LOG_TRACE`, `TPDE_LOG_DBG`, `TPDE_LOG_INFO`, `TPDE_LOG_WARN`, `TPDE_LOG_ERR`
  - Default: Disabled in Release builds, DebugOnly in Debug builds
  - No remote logging

**Debugging:**
- GDB integration
- `.gdbinit` in repository root
- Assertions enabled in Debug builds (`TPDE_DEBUG`, `TPDE_ASSERTS` macros)

## CI/CD & Deployment

**Hosting:**
- GitHub (source repository)
- GitHub Pages (documentation deployment)
  - Config: `.github/workflows/ci.yml`
  - Deployed from: `build/docs/html/`
  - Condition: `github.ref == 'refs/heads/master'`

**CI Pipeline:**
- GitHub Actions (`.github/workflows/ci.yml`)
  - Jobs:
    - `build-llvm19` - Ubuntu with LLVM 19
    - `build-llvm20` - Container `t0b1fox/tpde` with LLVM 20
    - `deploy-docs` - GitHub Pages deployment
    - `license-check` - REUSE compliance check
  - Steps per build job:
    - Checkout with recursive submodules
    - Install dependencies (ninja-build, cmake, clang/llvm-dev, doxygen)
    - Configure: `cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DTPDE_BUILD_DOCS=ON`
    - Build: `ninja -v -C build`
    - Test: `ninja -v -C build check-tpde`

## Environment Configuration

**Required env vars:**
- `LLVM_DIR` - Optional override for LLVM CMake path
  - Default: `/usr/lib64/llvm20/lib64/cmake/llvm` (per `CMakeLists.txt` line 9)
  - Used by: `find_package(LLVM CONFIG)` in `CMakeLists.txt`

**CMake options (critical):**
- `TPDE_ENABLE_LLVM=ON` - Required for LLVM backend
- `TPDE_ENABLE_ENCODEGEN=ON` - Required for LLVM backend
- `TPDE_INCLUDE_TESTS=ON` - Required for test execution
- `TPDE_X64=ON` - Required for x86-64 support
- `TPDE_A64=ON` - Required for AArch64 support

**Secrets location:**
- No secrets (all code is public)
- GitHub Actions uses no encrypted secrets

## Webhooks & Callbacks

**Incoming:**
- None (no HTTP endpoints)

**Outgoing:**
- None (no HTTP calls during compilation)

## Build System Integration

**CMake Targets:**
- `tpde::tpde` - Core library
- `tpde::tpde_llvm` - LLVM backend library
- `tpde::tpde_encodegen` - Encodegen executable
- `tpde_test` - Test executable (for core framework)
- `tpde-llc` - LLVM-IR to object compiler tool
- `tpde-lli` - JIT execution tool
- `check-tpde` - Run all tests (runs lit)
- `check-tpde-core` - Run core framework tests
- `check-tpde-llvm` - Run LLVM backend tests

**Submodule Dependencies:**
All bundled dependencies are managed via `.gitmodules`:
- `deps/spdlog` - https://github.com/gabime/spdlog.git
- `deps/args` - https://github.com/Taywee/args.git
- `deps/fadec` - https://github.com/aengelke/fadec.git
- `deps/hopscotch-map` - https://github.com/Tessil/hopscotch-map.git
- `deps/disarm` - https://github.com/aengelke/disarm.git
- `deps/doxygen-awesome-css` - https://github.com/jothepro/doxygen-awesome-css.git

## Development Tools Integration

**Code Formatting:**
- clang-format (`.clang-format`)
- Style: LLVM-based with customizations
- Config keys: BasedOnStyle: LLVM, BinPackArguments: false, BinPackParameters: false, InsertTrailingCommas: Wrapped

**Testing Integration:**
- LLVM lit test framework
  - Configuration: `lit.site.cfg.py.in` → `lit.site.cfg.py`
  - FileCheck-based assertions in test files
  - Substitutions: `%tpde_test`, `%objdump`, `%vir_verify`
  - Test formats: `.ll` (LLVM-IR), `.vir`, `.tir`

**Documentation:**
- Doxygen (via `-DTPDE_BUILD_DOCS=ON`)
- doxygen-awesome-css theme
- Output: `build/docs/html/`
- Source: `docs/` directory

---

*Integration audit: 2026-01-20*
