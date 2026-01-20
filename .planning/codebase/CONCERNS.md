# Codebase Concerns

**Analysis Date:** 2026-01-20

## Tech Debt

**Architecture-specific Inconsistencies:**
- Issue: Register allocation and calling convention handling differs between x64 and AArch64 implementations
- Files: `tpde/include/tpde/arm64/CompilerA64.hpp`, `tpde/include/tpde/x64/CompilerX64.hpp`
- Impact: Code quality differences across architectures, harder to maintain, bugs may manifest differently
- Fix approach: Extract common register allocation logic into shared template base, document architecture-specific decisions clearly

**Incomplete Calling Convention Support:**
- Issue: Many calling convention features marked as TODO (sret, byval handling, CSR for SIMD registers)
- Files: `tpde/include/tpde/arm64/CompilerA64.hpp:779`, `tpde/include/tpde/arm64/CompilerA64.hpp:900-901`, `tpde/include/tpde/x64/CompilerX64.hpp:342`, `tpde/include/tpde/x64/CompilerX64.hpp:703`
- Impact: Incorrect code generation for complex function signatures, ABI violations
- Fix approach: Implement full CCAssigner integration, add comprehensive tests for calling conventions

**Jump Table Implementation:**
- Issue: Jump tables for switch statements not moved to read-only data section
- Files: `tpde/include/tpde/arm64/CompilerA64.hpp:2127`
- Impact: Security (writable code memory), potential performance impact
- Fix approach: Add separate read-only data section in FunctionWriter, move jump tables there during compilation

**String Table Deduplication:**
- Issue: No hash table for string deduplication in ELF symbol tables
- Files: `tpde/src/StringTable.cpp:15`, `tpde/src/StringTable.cpp:25`
- Impact: Larger object files, wasted memory
- Fix approach: Implement hash-based deduplication using bundled hopscotch-map dependency

**Value Size Information Missing:**
- Issue: Several TODO comments about missing value size information needed for copying
- Files: `tpde/include/tpde/ValuePartRef.hpp:353`, `tpde/include/tpde/ValuePartRef.hpp:367`, `tpde/include/tpde/ValuePartRef.hpp:499`, `tpde/include/tpde/ValuePartRef.hpp:709`, `tpde/include/tpde/ValuePartRef.hpp:741`
- Impact: Cannot implement certain value operations safely
- Fix approach: Store size in ValueAssignment or pass through IRAdaptor

**Variable Reference Spilling:**
- Issue: Variable references not properly spilled in scratch register allocation
- Files: `tpde/include/tpde/CompilerBase.hpp:2292`
- Impact: Incorrect code when stack variables need temporary registers
- Fix approach: Handle variable ref spilling in scratch register allocation path

**Alignment Support:**
- Issue: Larger stack alignments not supported for function arguments
- Files: `tpde/include/tpde/arm64/CompilerA64.hpp:793`, `tpde/include/tpde/x64/CompilerX64.hpp:614`
- Impact: Alignment requirements for complex types not satisfied
- Fix approach: Implement dynamic stack frame alignment based on max alignment requirement

## Known Bugs

**Entry Block Register Arguments:**
- Issue: Register arguments in entry block not handled properly
- Files: `tpde/include/tpde/Analyzer.hpp:1803`, `tpde/include/tpde/Analyzer.hpp:1813`
- Symptoms: Liveness analysis incorrect for function arguments passed in registers
- Trigger: Functions with arguments in registers (most functions on SysV ABI)
- Workaround: Force stack passing of arguments (inefficient)
- Fix approach: Query CCAssigner for register arguments and add to entry block liveness

**Unreachable Block Handling:**
- Issue: Blocks marked as unreachable but still may have phi inputs processed
- Files: `tpde/include/tpde/Analyzer.hpp:949-961`, `tpde/include/tpde/Analyzer.hpp:2376`
- Symptoms: PHI node processing may reference values from unreachable blocks
- Trigger: CFG with unreachable blocks after optimization
- Workaround: Manual CFG cleanup before compilation
- Fix approach: Filter out unreachable blocks during phi input collection

## Security Considerations

**Code Memory Writability:**
- Risk: Jump tables placed in writable memory sections
- Files: `tpde/include/tpde/arm64/CompilerA64.hpp:2127`
- Impact: Allows self-modifying code, security vulnerability
- Current mitigation: None identified
- Recommendations: Move jump tables to read-only data section, use RELRO hardening flags

**Memory Safety in Critical Paths:**
- Risk: Extensive use of raw pointers and unions without runtime checks in performance-critical code
- Files: `tpde/include/tpde/ValuePartRef.hpp` (unions with ConstantData/ValueData), `tpde/include/tpde/AssignmentPartRef.hpp`
- Impact: Type confusion, memory corruption bugs possible
- Current mitigation: Extensive use of assertions in debug builds, clang-format checks
- Recommendations: Add runtime type tagging for union variants, enable -D_GLIBCXX_ASSERTIONS by default

**Input Validation:**
- Risk: Limited validation of IR input through IRAdaptor interface
- Files: `tpde/include/tpde/IRAdaptor.hpp`
- Impact: Malformed IR can crash compiler or generate incorrect code
- Current mitigation: VerificationIR for post-compilation verification
- Recommendations: Add IR validation pass before compilation, validate IRAdaptor invariants at runtime

## Performance Bottlenecks

**Liveness Analysis O(n²) Loop Common Ancestor:**
- Problem: Finding lowest common ancestor in loop nesting is O(n) per query, making overall liveness O(n²)
- Files: `tpde/include/tpde/Analyzer.hpp:2287-2294`
- Cause: Simple walk-up algorithm without preprocessing
- Improvement path: Implement Gusfield's constant-time LCA algorithm with Euler tour + RMQ preprocessing

**Register Allocation Heuristics:**
- Problem: Spill candidate selection uses complex heuristic that may not be optimal
- Files: `tpde/include/tpde/CompilerBase.hpp:1779-1781`
- Cause: Heuristic based on ref-count, distance to next use, and variable-ref status but not benchmarked
- Improvement path: Benchmark against simple heuristics (LRU, ref-count only), add ML-guided register allocation

**Block Layout Algorithm:**
- Problem: Algorithm selection not benchmarked against simpler approaches
- Files: `tpde/include/tpde/Analyzer.hpp:951`, `tpde/include/tpde/Analyzer.hpp:1015`
- Cause: Current implementation uses complex algorithm but may be overkill for small CFGs
- Improvement path: Add simple RPO fallback for small CFGs (< 100 blocks), benchmark both approaches

**String Table Linear Search:**
- Problem: String lookup in symbol tables is linear
- Files: `tpde/src/StringTable.cpp`
- Cause: No hash-based indexing
- Improvement path: Use hash table from bundled hopscotch-map dependency

**Constant Pooling:**
- Problem: Constants may be duplicated across function calls
- Files: `tpde/include/tpde/x64/CompilerX64.hpp:1329`
- Cause: No deduplication or pooling mechanism
- Improvement path: Implement constant pooling at function or module level

## Fragile Areas

**CRTP-Based Compiler Architecture:**
- Files: `tpde/include/tpde/CompilerBase.hpp` (3095 lines), `tpde/include/tpde/Compiler.hpp`, `tpde/include/tpde/CompilerConfig.hpp`
- Why fragile: Heavy reliance on CRTP makes template instantiation errors cryptic, inheritance chains are deep
- Safe modification: Follow existing CRTP patterns carefully, add type concepts at template boundaries
- Test coverage: Only TestIR tests for simple cases, no stress tests for edge cases

**ValuePartRef/AssignmentPartRef Complexity:**
- Files: `tpde/include/tpde/ValuePartRef.hpp` (1210 lines), `tpde/include/tpde/AssignmentPartRef.hpp`
- Why fragile: Complex state machines with many modes (fixed, stack, variable-ref, constant), multiple interacting flags
- Safe modification: Add comprehensive unit tests for mode transitions, document state diagrams
- Test coverage: Limited - mostly integration tests through TestIR

**Analyzer Liveness Algorithm:**
- Files: `tpde/include/tpde/Analyzer.hpp` (2427 lines)
- Why fragile: Complex data flow analysis with multiple interacting components (liveness, loops, pressure, spills)
- Safe modification: Use VerificationIR to validate after changes, add incremental tests
- Test coverage: Good coverage of basic cases, but edge cases (critical edges, loops, phis) not fully tested

**Architecture-Specific Compilers:**
- Files: `tpde/include/tpde/arm64/CompilerA64.hpp` (2212 lines), `tpde/include/tpde/x64/CompilerX64.hpp` (2145 lines)
- Why fragile: Large files with platform-specific encoding logic, macro-heavy assembly helpers
- Safe modification: Changes should be verified against both architectures, add cross-arch tests
- Test coverage: Many codegen tests exist but may not cover all encoding patterns

## Scaling Limits

**Register File Size:**
- Current capacity: Hard limit of 64 registers due to u64 bitset implementation
- Files: `tpde/include/tpde/RegisterFile.hpp:60`
- Limit: Cannot support AVX-512 with 32 ZMM registers + 32 XMM registers on x64
- Scaling path: Use std::bitset or dynamic bitset for larger register files

**LLVM Dependency:**
- Current capacity: Only supports LLVM 19.1 and 20.1
- Files: `CMakeLists.txt:76`, `CMake/Lists.txt:82-96`
- Limit: Using unsupported LLVM versions may cause compilation or code generation failures
- Scaling path: Increase version support range, add compatibility shim layer for IR differences

**Memory Allocation:**
- Current capacity: Uses bump allocator for compilation, no backpressure
- Files: `tpde/include/tpde/util/BumpAllocator.hpp`
- Limit: Can allocate unlimited memory during compilation
- Scaling path: Add allocation limits and fallback to arena pooling

**CFG Complexity:**
- Current capacity: No explicit limits, but O(n²) algorithms degrading with large functions
- Files: `tpde/include/tpde/Analyzer.hpp:2287-2294`
- Limit: Functions with >10,000 blocks may have unacceptable compilation time
- Scaling path: Implement LCA algorithm, add function splitting for very large CFGs

## Dependencies at Risk

**LLVM Version Compatibility:**
- Risk: Only two LLVM versions supported (19.1, 20.1), hardcoded in build system
- Files: `CMakeLists.txt:9`, `CMakeLists.txt:76`, `CMakeLists.txt:94-96`
- Impact: Cannot build with older or newer LLVM versions, limits platform support
- Migration plan: Support broader version range, add version compatibility testing in CI

**fadec/disarm Assembler Libraries:**
- Risk: Bundled dependencies may fall behind upstream
- Files: `deps/fadec/`, `deps/disarm/`, `tpde/CMakeLists.txt:68-100`
- Impact: Missing instruction encodings for new ISA extensions
- Migration plan: Track upstream releases, add update mechanism, consider vendoring with regular updates

**Platform-Specific Assemblers:**
- Risk: fadec/disarm are the only supported instruction encoders
- Files: `deps/fadec/`, `deps/disarm/`
- Impact: Cannot support new architectures without writing new encoder integration
- Migration plan: Abstract encoder interface, add pluggable encoder architecture

** spdlog Dependency:**
- Risk: Optional but if enabled adds heavy logging framework dependency
- Files: `deps/spdlog/`, `tpde/CMakeLists.txt:25-33`
- Impact: Larger binaries, compilation time impact
- Migration plan: Consider custom minimal logger, make spdlog truly optional without recompilation

## Missing Critical Features

**SIMD/Vector Instruction Support:**
- Problem: AVX on x64 and SVE on AArch64 not implemented
- Files: `tpde/include/tpde/x64/CompilerX64.hpp:70` (TODO comment), `tpde/include/tpde/x64/CompilerX64.hpp:1352` (TODO for AVX/AVX-512)
- Blocks: Vectorized code generation for ML/DL workloads, high-performance computing
- Implementation needed: Add vector register banks to RegisterFile, implement SIMD instruction encoding

**Windows Support:**
- Problem: No Windows platform support, only ELF/Unix systems
- Files: No PE/COFF code generation found, only `AssemblerElf.hpp`
- Blocks: Cross-platform JIT compilation on Windows
- Implementation needed: Add PE/COFF assembler support, Windows calling conventions

**TLS Optimizations:**
- Problem: Only basic TLS model, no support for optimized LocalExec/InitialExec selection
- Files: `tpde/include/tpde/CompilerBase.hpp:2179` (TODO for non-GD model access)
- Blocks: Optimal TLS access for performance-critical code
- Implementation needed: Add TLS model selection based on ELF visibility flags

**DWARF Debug Information:**
- Problem: Minimal DWARF support (only eh-frame for unwinding)
- Files: `tpde/include/tpde/AssemblerElf.hpp:195-446` (eh-frame only)
- Blocks: Source-level debugging, profiler integration
- Implementation needed: Add .debug_info/.debug_line/.debug_abbrev sections generation

**Function Inlining:**
- Problem: No function inlining support (only external calls)
- Files: No inlining pass found in Analyzer or CompilerBase
- Blocks: Performance optimizations that rely on inlining
- Implementation needed: Add inlining pass before block layout, update IRAdaptor to support function cloning

**Tail Call Optimization:**
- Problem: No tail call support
- Files: No tail call transformation in compilers
- Blocks: Functional language compilation, recursive algorithms
- Implementation needed: Detect tail call patterns, emit jump instead of call+ret

## Test Coverage Gaps

**Architecture-Specific Tests:**
- What's not tested: Complex calling conventions (sret, byval), SIMD instructions, TLS models
- Files: `tpde/test/filetest/codegen/` (only 12 basic tests), `tpde-llvm/test/` (mostly LLVM IR, not testing architecture features)
- Risk: Architecture-specific code generation bugs may go undetected
- Priority: High for calling conventions, Medium for SIMD (feature not implemented)

**Error Path Testing:**
- What's not tested: Out-of-register conditions, stack overflow, invalid IR inputs
- Files: No tests found for error recovery paths
- Risk: Compiler crashes or hangs on invalid input instead of graceful failure
- Priority: High (stability concern)

**Stress Testing:**
- What's not tested: Very large functions (>1000 blocks), deeply nested loops, complex CFGs
- Files: Only small test cases (< 100 instructions)
- Risk: O(n²) algorithms may cause timeouts or excessive memory usage
- Priority: Medium (scaling concern for large functions)

**Cross-Architecture Consistency:**
- What's not tested: Same IR compiled for both architectures produces semantically equivalent code
- Files: Separate test sets for x64 and AArch64, no cross-validation
- Risk: Bugs may manifest only on one architecture
- Priority: High (correctness concern)

**Edge Cases:**
- What's not tested: Critical edges, PHI nodes with many inputs, register pressure at block boundaries
- Files: `tpde/test/filetest/analyzer/` has basic phi and edge tests but not comprehensive
- Risk: Incorrect code generation for complex control flow
- Priority: High (correctness concern)

**IRAdaptor Validation:**
- What's not tested: Compliance with IRAdaptor concepts for non-TestIR implementations
- Files: Only TestIR tested, no generic IRAdaptor tests
- Risk: New IR adaptors may violate invariants
- Priority: Medium (developer experience concern)

**Fuzz Testing:**
- What's not tested: Randomized IR input for crash detection
- Files: No fuzz harness found
- Risk: Memory safety bugs may not be discovered
- Priority: Medium (security/stability concern)

---

*Concerns audit: 2026-01-20*
