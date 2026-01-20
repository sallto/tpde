# Codebase Structure

**Analysis Date:** 2026-01-20

## Directory Layout

```
[project-root]/
├── tpde/                    # Core compiler framework (header-only)
│   ├── include/tpde/        # Public headers
│   ├── src/                 # Minimal implementation files
│   └── CMakeLists.txt        # tpde library build
├── tpde-llvm/              # LLVM IR integration backend
│   ├── include/tpde-llvm/   # LLVM adaptor headers
│   ├── src/                 # LLVM adaptor implementation
│   └── CMakeLists.txt
├── tpde-encodegen/          # Encoding snippet generator tool
│   └── src/
├── deps/                    # Bundled dependencies
│   ├── fadec/               # x86-64 encoder/decoder
│   ├── disarm/              # AArch64 encoder/decoder
│   ├── spdlog/             # Logging library
│   ├── args/               # Command-line parsing
│   └── hopscotch-map/      # Hash map implementation
├── docs/                    # Doxygen documentation
└── CMakeLists.txt            # Top-level build
```

## Directory Purposes

**tpde/include/tpde/:**
- Purpose: Core public API and framework headers
- Contains: IRAdaptor concept, CompilerBase, Analyzer, Assembler, architecture-specific compilers
- Key files: `CompilerBase.hpp`, `IRAdaptor.hpp`, `Analyzer.hpp`, `ValueRef.hpp`

**tpde/src/:**
- Purpose: Implementation files requiring compilation (non-header-only code)
- Contains: Assembler implementation, ELF emission, string tables, base utilities
- Key files: `Assembler.cpp`, `AssemblerElf.cpp`, `base.cpp`, `ValueAssignment.cpp`

**tpde/src/test/:**
- Purpose: Test infrastructure and TestIR implementation
- Contains: TestIR (simple IR for testing), test compilers for x64/A64
- Key files: `TestIR.hpp`, `TestIRCompiler.cpp`, `TestIRCompilerA64.cpp`

**tpde-llvm/:**
- Purpose: LLVM IR to TPDE adapter
- Contains: LLVMIRAdaptor, JIT compilation support, encode templates
- Key files: `LLVMAdaptor.hpp`, `LLVMCompilerBase.hpp`, `OrcCompiler.cpp`

**deps/:**
- Purpose: External dependencies managed as git submodules
- Contains: fadec (x64 encoding), disarm (A64 encoding), spdlog (logging)
- Generated: Yes (bundled sources)
- Committed: Yes (git submodules)

## Key File Locations

**Entry Points:**
- `tpde/include/tpde/CompilerBase.hpp`: Main compilation driver with compile() method
- `tpde/include/tpde/IRAdaptor.hpp`: User-provided interface concept
- `tpde/src/test/TestIRCompiler.hpp`: Example derived compiler implementation

**Configuration:**
- `CMakeLists.txt`: Top-level build configuration
- `tpde/CMakeLists.txt`: tpde library build
- `tpde/include/tpde/CompilerConfig.hpp`: Compiler configuration options

**Core Logic:**
- `tpde/include/tpde/CompilerBase.hpp`: Architecture-independent compilation (3095 lines)
- `tpde/include/tpde/Analyzer.hpp`: Block layout and liveness (2427 lines)
- `tpde/include/tpde/ValuePartRef.hpp`: Value part access and register allocation (1210 lines)
- `tpde/include/tpde/RegisterFile.hpp`: Register tracking and allocation (241 lines)

**Architecture-Specific:**
- `tpde/include/tpde/x64/CompilerX64.hpp`: x86-64 compilation
- `tpde/include/tpde/arm64/CompilerA64.hpp`: AArch64 compilation
- `tpde/include/tpde/x64/FunctionWriterX64.hpp`: x86-64 instruction encoding helpers
- `tpde/include/tpde/arm64/FunctionWriterA64.hpp`: AArch64 instruction encoding helpers

**Assembler/ELF:**
- `tpde/include/tpde/Assembler.hpp`: Base assembler class
- `tpde/include/tpde/AssemblerElf.hpp`: ELF-specific assembler (477 lines)
- `tpde/src/AssemblerElf.cpp`: ELF emission implementation
- `tpde/src/ElfMapper.cpp`: Runtime code mapping

**Testing:**
- `tpde/src/test/`: Test compiler implementations
- `tpde/test/filetest/`: lit-based test suites (.vir files with CHECK comments)
- Root directory: *.vir files for quick manual testing

## Naming Conventions

**Files:**
- Headers: PascalCase (e.g., `CompilerBase.hpp`, `ValueRef.hpp`)
- Source: PascalCase.cpp matching header (e.g., `Assembler.cpp`)
- Architecture directories: Lowercase with arch name (e.g., `x64/`, `arm64/`)

**Directories:**
- Framework: `tpde/` (core), `tpde-llvm/` (LLVM backend), `tpde-encodegen/` (tool)
- Architecture: `x64/`, `arm64/` within `include/tpde/` and `include/tpde-llvm/src/`
- Utils: `util/` within `include/tpde/` for utility headers

## Where to Add New Code

**New Feature:**
- Primary code: `tpde/include/tpde/` (architecture-independent) or `tpde/include/tpde/x64/`/`arm64/` (architecture-specific)
- Tests: `tpde/src/test/TestIRCompiler.cpp` for new instruction implementations, or new .vir test files

**New Component/Module:**
- Implementation: `tpde/include/tpde/[NewComponent].hpp` for framework components
- Architecture-specific: `tpde/include/tpde/x64/[NewComponent].hpp` or `tpde/include/tpde/arm64/[NewComponent].hpp`

**Utilities:**
- Shared helpers: `tpde/include/tpde/util/[Name].hpp`
- Template utilities already in `util/SmallVector.hpp`, `util/BumpAllocator.hpp`, `util/misc.hpp`

**New IR Integration:**
- IRAdaptor: User-provided class implementing IRAdaptor concept
- Compiler: User-provided class deriving from CompilerX64 or CompilerA64
- Instruction selection: Implement compile_inst() method for each instruction type

**New Target Architecture:**
- Create `tpde/include/tpde/[arch]/` directory
- Implement `Compiler[Arch].hpp` deriving from CompilerBase
- Implement `FunctionWriter[Arch].hpp` for instruction encoding
- Add encoder/decoder library to `deps/`
- Update `tpde/CMakeLists.txt` with new architecture option

## Special Directories

**deps/:**
- Purpose: External dependencies (fadec, disarm64, spdlog)
- Generated: No (bundled as submodules)
- Committed: Yes (git submodules)
- Don't modify directly: These are third-party libraries

**build/, cmake-build-*/, CMakeFiles/:**
- Purpose: Build artifacts and configuration
- Generated: Yes (by cmake)
- Committed: No (in .gitignore)
- Don't edit: Auto-generated

**.planning/:**
- Purpose: Generated documentation from analysis tools
- Generated: Yes
- Committed: Yes (codebase analysis output)

**docs/tpde/:**
- Purpose: Doxygen source documentation
- Generated: No (source markdown)
- Committed: Yes
- Documentation style: Doxygen with Markdown formatting

---

*Structure analysis: 2026-01-20*
