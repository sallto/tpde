# Architecture

**Analysis Date:** 2026-01-20

## Pattern Overview

**Overall:** CRTP-based layered compiler backend framework

**Key Characteristics:**
- CRTP (Curiously Recurring Template Pattern) for compile-time polymorphism
- Header-only template library with minimal compiled implementation
- IR-agnostic architecture that adapts to different SSA IRs
- Architecture-independent compilation logic separated from ISA-specific code
- Single-pass compilation with integrated register allocation

## Layers

**IRAdaptor Interface:**
- Purpose: Defines contract for IR-specific integration points
- Location: `tpde/include/tpde/IRAdaptor.hpp`
- Contains: Type definitions, iteration interfaces, IR introspection methods
- Depends on: User's IR implementation
- Used by: CompilerBase, Analyzer

**Analyzer:**
- Purpose: Computes block layout and liveness analysis
- Location: `tpde/include/tpde/Analyzer.hpp`
- Contains: DominatorTree, WorkingSetTracker, liveness computation
- Depends on: IRAdaptor
- Used by: CompilerBase

**CompilerBase:**
- Purpose: Architecture-independent compilation driver and register allocation
- Location: `tpde/include/tpde/CompilerBase.hpp`
- Contains: ValueRef, ValuePart, assignment tracking, register allocation logic
- Depends on: IRAdaptor, Analyzer, Assembler
- Used by: Architecture-specific compilers (CompilerX64, CompilerA64)

**Architecture-Specific Compilers:**
- Purpose: Platform-specific compilation extensions and instruction selection
- Location: `tpde/include/tpde/x64/CompilerX64.hpp`, `tpde/include/tpde/arm64/CompilerA64.hpp`
- Contains: Prologue/epilogue generation, calling convention handling, instruction encoding helpers
- Depends on: CompilerBase, FunctionWriter, Assembler
- Used by: User-derived compiler implementations

**Assembler:**
- Purpose: Object file generation and ELF structure management
- Location: `tpde/include/tpde/Assembler.hpp`, `tpde/include/tpde/AssemblerElf.hpp`
- Contains: Section management, symbol tracking, relocations, ELF emission
- Depends on: Platform-specific encoding libraries (fadec, disarm64)
- Used by: CompilerBase, FunctionWriter

**User-Derived Compiler:**
- Purpose: Implements IR-specific instruction selection
- Location: User-defined (e.g., `tpde/src/test/TestIRCompiler.cpp`)
- Contains: `compile_inst()` implementations, encoding snippets
- Depends on: CompilerX64/CompilerA64
- Used by: CompilerBase (via CRTP)

## Data Flow

**Compilation Flow:**

1. IRAdaptor provides module information (functions, globals)
2. CompilerBase::compile() initializes compilation
3. For each function:
   - IRAdaptor::switch_func() selects current function
   - Analyzer computes block layout (reverse post-order) and liveness
   - Compiler generates function prologue and argument setup
   - Compiler iterates blocks in analyzer's layout order
   - For each instruction, calls user's compile_inst()
   - User implementation uses ValueRef/ValuePartRef for register allocation
   - ValueRef provides access to operand/result value assignments
   - FunctionWriter encodes instructions via architecture-specific APIs
4. Function epilogue generated
5. Assembler finalizes ELF object file

**State Management:**
- **Register allocation:** Linear scan with ValuePartRef RAII semantics
- **Assignments:** Tracked via ValLocalIdx indices in arrays
- **PHI nodes:** Special handling for parallel copies at block boundaries
- **Spilling:** Hybrid register/stack allocation based on working set size
- **Constants:** Handled through ValRefSpecial union in ValueRef

## Key Abstractions

**IRAdaptor Concept:**
- Purpose: Polymorphic interface for any SSA IR
- Examples: LLVMIRAdaptor (`tpde-llvm/include/tpde-llvm/LLVMAdaptor.hpp`), TestIR (`tpde/src/test/TestIR.hpp`)
- Pattern: C++20 concept with required methods (func_count(), block_insts(), inst_operands(), etc.)

**CRTP Template Chain:**
- Purpose: Static dispatch without virtual overhead
- Examples: `CompilerBase<Adaptor, Derived, Config>` → `CompilerX64<...>` → `UserCompiler`
- Pattern: Base class calls `derived()->method()` for user-provided overrides

**ValueRef:**
- Purpose: RAII wrapper for value assignments with reference counting
- Examples: Used in `compile_inst()` implementations
- Pattern: Union of AssignmentData (ref-counted assignment) and ValRefSpecial (constants)
- Reference counting triggers automatic register/spill slot release

**ValuePartRef:**
- Purpose: Access to individual parts of multi-part values
- Examples: `value_ref(v).part(0).load_to_reg()`
- Pattern: Provides load_to_reg(), get_stack_off(), set_reg(), salvage()

**RegisterFile:**
- Purpose: Tracks register usage and allocation
- Examples: `compiler->register_file` in CompilerBase
- Pattern: Bitset-based tracking with per-bank registers, lock counts for fixed assignments

**AssignmentPartRef:**
- Purpose: Direct access to assignment without ValueRef RAII
- Examples: Used in register allocation when ValueRef's refcounting is not needed
- Pattern: Lightweight reference to ValueAssignment + part index

## Entry Points

**CompilerBase::compile():**
- Location: `tpde/include/tpde/CompilerBase.hpp`
- Triggers: User calls after setting up IRAdaptor
- Responsibilities: Iterates functions, orchestrates per-function compilation, finalizes assembler

**compile_inst() (user-provided):**
- Location: User's derived compiler class
- Triggers: Called by CompilerBase for each instruction in block order
- Responsibilities: Generate machine code using ValueRef operands and set result assignment

**IRAdaptor::switch_func():**
- Location: User's IRAdaptor implementation
- Triggers: Called by CompilerBase before compiling each function
- Responsibilities: Set internal state for current function, prepare value-to-index mappings

## Error Handling

**Strategy:** Assertions and compile-time checks

**Patterns:**
- Debug assertions (`assert()`) for invariants in debug builds
- Compile-time concepts for IRAdaptor interface validation
- `unreachable()` macro for unreachable code paths
- No exceptions: Code compiles with `-fno-exceptions`
- No RTTI: Code compiles with `-fno-rtti`

## Cross-Cutting Concerns

**Logging:** spdlog integration (optional, enabled with TPDE_LOGGING cmake option)
**Validation:** VerificationIR for instruction correctness (used in tests)
**Memory Management:** BumpAllocator for temporary allocations, SmallVector for inline storage
**Testing:** LLVM lit framework with FileCheck for assembly verification
**Portability:** Architecture independence through CRTP, platform isolation in Assembler

---

*Architecture analysis: 2026-01-20*
