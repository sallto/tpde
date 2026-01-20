# Coding Conventions

**Analysis Date:** 2026-01-20

## Naming Patterns

**Files:**
- Headers: PascalCase with `.hpp` extension (e.g., `CompilerBase.hpp`, `ValueRef.hpp`)
- Source: PascalCase with `.cpp` extension (e.g., `Assembler.cpp`, `base.cpp`)
- Tests: PascalCase with `.hpp`/`.cpp` extension (e.g., `TestIR.hpp`, `TestIRCompiler.cpp`)
- Architecture-specific: Subdirectories `x64/` and `arm64/` for architecture code

**Functions:**
- snake_case for methods and free functions (e.g., `compile_inst()`, `print_rpo()`)
- Prefix underscores rarely used; prefer clear naming

**Variables:**
- snake_case for local variables and parameters (e.g., `cur_block_idx`, `available`)
- Trailing underscores for member variables in some cases (e.g., `compiler_`, `available_`)
- Short aliases: `u8`, `u16`, `u32`, `u64`, `i8`, `i16`, `i32`, `i64` (from `tpde/base.hpp`)

**Types:**
- PascalCase for classes and structs (e.g., `ValueRef`, `ScratchReg`, `CCAssignment`)
- PascalCase for type aliases (e.g., `IRValueRef`, `IRFuncRef`, `InstRange`)
- Enum class style for enums (e.g., `enum class TLSMode`, `enum class BlockIndex`)

## Code Style

**Formatting:**
- clang-format based on LLVM style (`.clang-format`)
- `BasedOnStyle: LLVM`
- `BinPackArguments: false`, `BinPackParameters: false` - one arg per line preferred
- `InsertBraces: true` - braces always required
- `IndentWrappedFunctionNames: true`
- `SortUsingDeclarations: Lexicographic` - using declarations sorted
- `AlignArrayOfStructures: Right`
- `InsertNewlineAtEOF: true`

**Linting:**
- No explicit linter detected beyond clang-format

## Import Organization

**Order:**
1. Related headers (e.g., `"CompilerConfig.hpp"`, `"base.hpp"`)
2. Standard library headers (e.g., `<algorithm>`, `<format>`)
3. External library headers (e.g., `<spdlog/spdlog.h>`)

**Path Aliases:**
- No path aliases configured; relative includes used throughout

**Headers:**
- `#pragma once` used universally (no `#ifndef` guards)
- All headers include SPDX license comment at top

**Include pattern example:**
```cpp
// SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
//
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#pragma once

#include "CompilerConfig.hpp"
#include "base.hpp"

#include <algorithm>
#include <format>
```

## Error Handling

**Constraints:**
- **No exceptions**: `-fno-exceptions` compilation flag
- **No RTTI**: `-fno-rtti` compilation flag
- Error handling via return values and assertions

**Patterns:**
- `bool` return values for operations that can fail (e.g., `compile_inst()`)
- `std::optional` for potentially missing values (e.g., `val_ref_special()`)
- `TPDE_UNREACHABLE(msg)` macro for unreachable code paths:
  ```cpp
  #ifndef NDEBUG
    #define TPDE_UNREACHABLE(msg) assert(0 && (msg))
  #else
    #define TPDE_UNREACHABLE(msg) __builtin_unreachable()
  #endif
  ```
- `TPDE_FATAL(msg)` calls `fatal_error()` for unrecoverable errors

**Assertions:**
- `assert()` used for invariants (enabled with `TPDE_ASSERTS` or `TPDE_DEBUG` in Debug builds)
- Debug-only checks wrapped in `#ifndef NDEBUG`

## Logging

**Framework:** spdlog (optional, controlled by `TPDE_LOGGING`)

**Levels:**
- `TPDE_LOG_TRACE(...)`: Trace level (debug builds only)
- `TPDE_LOG_DBG(...)`: Debug level (debug builds only)
- `TPDE_LOG_INFO(...)`: Info level
- `TPDE_LOG_WARN(...)`: Warning level
- `TPDE_LOG_ERR(...)`: Error level

**Macro definition (conditional):**
```cpp
#ifdef TPDE_LOGGING
  #include <spdlog/spdlog.h>
  #define TPDE_LOG(level, ...) \
    (spdlog::should_log(level) ? spdlog::log(level, __VA_ARGS__) : (void)0)
#else
  #define TPDE_LOG_TRACE(...)
  #define TPDE_LOG_DBG(...)
  #define TPDE_LOG_INFO(...)
  #define TPDE_LOG_WARN(...)
  #define TPDE_LOG_ERR(...)
#endif
```

## Comments

**When to Comment:**
- TODO/FIXME comments for known issues and future improvements
- Inline comments for complex logic or non-obvious implementation details
- Section comments for major logical divisions

**SPDX License Headers:**
Required at top of every source file:
```cpp
// SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
//
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
```

**TODO/FIXME Pattern:**
```cpp
// TODO(ts): maybe do this smarter, but when is this really relevant?
// TODO(ts): support CSR of Qx/Vx registers, not just Dx
```

**JSDoc/TSDoc:**
- C++ documentation comments on public APIs
- Doxygen-style comments used:
  ```cpp
  /// Allocate register in the specified bank, optionally excluding certain
  /// non-fixed registers. Spilling can be disabled for spill code to avoid
  /// recursion; if spilling is disabled, the allocation can fail.
  AsmReg alloc(RegBank bank) noexcept;
  ```

## Function Design

**Size:**
- Functions range from small (10-20 lines) to large (200+ lines for complex logic)
- Complex compilation methods (e.g., `compile_inst()`) use switch statements
- Template-heavy code in header files for performance

**Parameters:**
- Pass by value for small types (e.g., `ValLocalIdx`, enums)
- Pass by const reference for larger types (e.g., `std::span`, `SmallVector`)
- Pointer parameters when ownership is unclear (e.g., `CompilerBase *compiler`)

**Return Values:**
- `bool` for success/failure operations
- `void` for operations that cannot fail (or use assertions)
- Enum types for state representation
- Move semantics for resource-managing types (e.g., `ScratchReg`, `ValueRef`)

**noexcept:**
- `noexcept` heavily used on methods that don't throw (essentially all methods)
- Move constructors and assignment operators marked `noexcept`

## Module Design

**Exports:**
- Header-only library design: most code in `tpde/include/tpde/`
- Minimal `.cpp` files for implementation requiring non-header-friendly code
- Public API is header-only templates + minimal runtime support

**Barrel Files:**
- No barrel files detected; each header included explicitly

**Namespaces:**
- Root namespace: `tpde`
- Sub-namespace: `tpde::test` for test utilities
- Sub-namespace: `tpde::x64` for x86-64 specific code
- Sub-namespace: `tpde::a64` for AArch64 specific code
- Sub-namespace: `tpde::util` for utilities

**Template Patterns:**
- CRTP (Curiously Recurring Template Pattern) extensively used:
  ```cpp
  template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
  class CompilerBase { /* ... */ };

  // Usage:
  class MyCompiler : public CompilerBase<MyAdaptor, MyCompiler, MyConfig> { /* ... */ };
  ```
- Template parameters ordered: `IRAdaptor Adaptor, typename Derived, CompilerConfig Config`

## Design Patterns

**CRTP:**
- `CompilerBase<Adaptor, Derived, Config>`: Base class for compiler implementations
- Architecture-specific: `CompilerX64<Adaptor, Derived, MiddleBase, Config>` and `CompilerA64<Adaptor, Derived, MiddleBase, Config>`

**RAII:**
- Resource management through move-only types (e.g., `ScratchReg`, `ValueRef`)
- Destructor automatically releases resources

**Type Erasure:**
- Concepts used to constrain template parameters
- `IRAdaptor` concept defines interface for IR integration

**Builder Pattern:**
- `RetBuilder` and `CallBuilder` for constructing complex operations

## Constraints and Invariants

**Compilation Constraints:**
- `-fno-rtti`: No runtime type information
- `-fno-exceptions`: No exception support
- C++20 standard required

**Code Constraints:**
- Header-only templates preferred for performance
- Minimal runtime allocation (stack-based where possible)
- Move semantics for efficiency
- `noexcept` on most methods

**Debug Build Invariants:**
- `TPDE_DEBUG` defined in Debug builds enables extra checks
- `TPDE_ASSERTS` enables assertion checks (default in Debug)
- Liveness checks in debug builds for `ValueRef` usage

**Type Constraints:**
- Concepts heavily used to enforce interfaces
- `static_assert` for compile-time checks
- Bit-level operations for low-level encoding

---

*Convention analysis: 2026-01-20*
