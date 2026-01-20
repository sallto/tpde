# Testing Patterns

**Analysis Date:** 2026-01-20

## Test Framework

**Runner:**
- LLVM lit framework with FileCheck
- Config: `tpde/test/filetest/lit.cfg.py`, `tpde/test/filetest/lit.site.cfg.py`

**Assertion Library:**
- FileCheck (part of LLVM toolchain)

**Run Commands:**
```bash
cmake --build build --target check-tpde-core    # Run all tests via lit
cmake --build build --target tpde_test           # Build test utility
./build/tpde/tpde_test [options] <input.ir>     # Run test utility directly
```

## Test File Organization

**Location:**
- Two test suites:
  1. `tpde/test/filetest/`: Core framework tests using TestIR format (`.tir`, `.vir`)
  2. `tpde-llvm/test/`: LLVM integration tests using LLVM IR format (`.ll`)

**Naming:**
- TestIR tests: `.tir` extension for TestIR format
- Verification tests: `.vir` extension for VerificationIR format
- LLVM tests: `.ll` extension for LLVM IR format
- Test files follow descriptive naming (e.g., `call.ll`, `incompatible.ll`)

**Structure:**
```
tpde/test/filetest/
├── lit.cfg.py              # Lit configuration
├── lit.site.cfg.py.in      # Site-specific config template
└── vir/
    └── vir_verifier.py     # VerificationIR helper script

tpde-llvm/test/
├── diagnostics/            # Error handling tests
│   ├── incompatible.ll
│   ├── large-types.ll
│   ├── unsupported-types.ll
│   └── vector-types.ll
├── elf/                    # ELF generation tests
│   ├── comdat.ll
│   ├── ctors-dtors.ll
│   ├── data-relocs.ll
│   └── ...
└── filetest/               # Full compilation tests
    ├── arm64/
    │   ├── call.ll
    │   ├── inlineasm.ll
    │   └── ...
    └── x64/
        └── (similar structure)
```

## Test Structure

**Suite Organization:**

For FileCheck tests (.ll files):
```llvm
; SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
; SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception

; RUN: tpde-llc --target=aarch64 %s | %objdump | FileCheck %s -check-prefixes=ARM64

define void @test_function() {
; ARM64-LABEL: <test_function>:
; ARM64:         sub sp, sp, #0x10
; ARM64-NEXT:    ret
  ret void
}
```

For diagnostic tests:
```llvm
; RUN: not tpde-llc --target=x86_64 %s | FileCheck %s
; CHECK: unsupported type: x86_fp80
; CHECK-NEXT: Failed to compile function f_x86_fp80_1
```

**Patterns:**
- **No setup/teardown**: Each test is self-contained LLVM IR or TestIR
- **Assertions via CHECK comments**: Expected output embedded as comments
- **Multiple architectures**: Tests often have prefixes for both ARM64 and X64
- **Negative tests**: Use `not tpde-llc` for error cases

**Test execution:**
- Test files are inputs to `tpde-llc` or `tpde_test`
- Output is piped through `llvm-objdump -d -r --no-show-raw-insn --symbolize-operands --no-addresses --x86-asm-syntax=intel -`
- FileCheck validates the disassembly matches expectations

## Mocking

**Framework:** None (compilation framework does not use traditional mocks)

**Patterns:**
- TestIR: A simple custom IR used for testing the core framework
- Minimal mocking needed; tests use actual compilation pipeline

**What to Mock:**
- Not applicable; tests use real IR and real compilation

**What NOT to Mock:**
- Don't mock the assembler; test actual code generation
- Don't mock IR adaptors; test with real TestIR or LLVM IR

## Fixtures and Factories

**Test Data:**
- TestIR format: Simple textual IR format for core framework testing
- LLVM IR format: Full LLVM IR for integration testing

**Location:**
- TestIR definition: `tpde/src/test/TestIR.hpp` and `tpde/src/test/TestIR.cpp`
- Test compilation: `tpde/src/test/TestIRCompiler.hpp` and `tpde/src/test/TestIRCompiler.cpp`

**Test data example (TestIR format):**
```
function @test() {
block %entry:
  %v0 = add(%v1, %v2)
  br %next
block %next:
  ret(%v0)
}
```

**LLVM IR example:**
```llvm
declare {float, float, float} @ret_3_float();

define void @call_3_float() {
  %c = call {float, float, float} @ret_3_float()
  ret void
}
```

## Coverage

**Requirements:** None explicitly enforced

**View Coverage:**
- No coverage measurement tool integrated
- Coverage determined by test suite completeness

**Test Coverage Areas:**
- **Core compilation pipeline**: Covered by TestIR tests
- **Architecture-specific codegen**: Covered by both TestIR and LLVM tests
- **Error handling**: Covered by diagnostic tests
- **ELF generation**: Covered by ELF tests
- **Calling conventions**: Covered by call/return tests

## Test Types

**Unit Tests:**
- Tests for specific compilation features (e.g., register allocation, liveness analysis)
- Implemented via simple TestIR functions testing one feature at a time
- Located in `tpde/test/filetest/`

**Integration Tests:**
- Full compilation from LLVM IR to ELF object file
- Tests complete pipeline including instruction selection and codegen
- Located in `tpde-llvm/test/filetest/`

**E2E Tests:**
- Similar to integration tests; full compilation from IR to disassembly
- Tests real-world patterns (calls, multiple functions, complex control flow)

## Common Patterns

**Multiple Architecture Testing:**
```llvm
; RUN: tpde-llc --target=x86_64 %s | %objdump | FileCheck %s -check-prefix=X64
; RUN: tpde-llc --target=aarch64 %s | %objdump | FileCheck %s -check-prefix=ARM64
```

**Error Testing:**
```llvm
; RUN: not tpde-llc --target=x86_64 %s | FileCheck %s
; CHECK: unsupported type: x86_fp80
```

**Test Utility Usage:**
```bash
# Print IR for debugging
./build/tpde/tpde_test --print-ir test.ir

# Run analysis only
./build/tpde/tpde_test --run-until=analyzer test.ir

# Print specific analysis results
./build/tpde/tpde_test --print-rpo test.ir
./build/tpde/tpde_test --print-liveness test.ir
./build/tpde/tpde_test --print-pressure test.ir

# Output object file
./build/tpde/tpde_test --arch=x64 -o test.o test.ir
```

**TestIR Compiler Pattern:**
```cpp
struct TestIRCompilerX64
    : x64::CompilerX64<TestIRAdaptor, TestIRCompilerX64, TestCompilerBase> {
  bool compile_inst(IRInstRef inst_idx, InstRange) noexcept override {
    switch (value.op) {
      case add: return compile_add(inst_idx);
      case sub: return compile_sub(inst_idx);
      // ...
    }
  }
};
```

## Debugging Tests

**Verbose Logging:**
- Use `TPDE_LOG_LEVEL` environment variable or `-l` flag
- Levels: 0=NONE, 1=ERR, 2=WARN (default), 3=INFO, 4=DEBUG, 5+=TRACE

**Print Intermediate Results:**
```bash
# Print reverse post-order
./build/tpde/tpde_test --print-rpo test.ir

# Print block layout
./build/tpde/tpde_test --print-layout test.ir

# Print liveness
./build/tpde/tpde_test --print-liveness test.ir

# Print register pressure
./build/tpde/tpde_test --print-pressure test.ir

# Print dominator tree
./build/tpde/tpde_test --print-domtree test.ir
```

**Precise Liveness:**
```bash
# Use per-block next-use distances
./build/tpde/tpde_test --precise-liveness --print-liveness test.ir
```

---

*Testing analysis: 2026-01-20*
