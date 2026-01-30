// SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
//
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#pragma once

#include "TestIR.hpp"
#include "TestIRAdaptor.hpp"
#include "tpde/base.hpp"
#include <vector>
#include "tpde/x64/CompilerX64.hpp"

namespace tpde::test {


// Define the struct in the tpde::test namespace so it's a complete type
struct TestIRCompilerX64
    : x64::CompilerX64<TestIRAdaptor, TestIRCompilerX64> {
  using Base = x64::CompilerX64<TestIRAdaptor, TestIRCompilerX64>;

  bool no_fixed_assignments;

  explicit TestIRCompilerX64(TestIRAdaptor *adaptor, bool no_fixed_assignments)
      : Base{adaptor}, no_fixed_assignments(no_fixed_assignments) {}

  bool cur_func_may_emit_calls() const {
    return this->ir()->functions[this->adaptor->cur_func].has_call;
  }

  SymRef cur_personality_func() const { return {}; }


  AsmReg select_fixed_assignment_reg(AssignmentPartRef ap,
                                     const IRValueRef value) {
    if (no_fixed_assignments && !try_force_fixed_assignment(value)) {
      return AsmReg::make_invalid();
    }

    return Base::select_fixed_assignment_reg(ap, value);
  }

  bool try_force_fixed_assignment(const IRValueRef value) const {
    return ir()->values[static_cast<u32>(value)].force_fixed_assignment;
  }

  std::optional<ValRefSpecial> val_ref_special(IRValueRef) { return {}; }

  ValuePart val_part_ref_special(ValRefSpecial &, u32) {
    TPDE_UNREACHABLE("val_part_ref_special on IR without special values");
  }

  void define_func_idx(IRFuncRef func, const u32 idx) {
    assert(static_cast<u32>(func) == idx);
    (void)func;
    (void)idx;
  }

  [[nodiscard]] bool compile_inst(IRInstRef, InstRange);

  TestIR *ir() { return this->adaptor->ir; }

  const TestIR *ir() const { return this->adaptor->ir; }

  bool compile_add(IRInstRef);
  bool compile_sub(IRInstRef);
  bool compile_condselect(IRInstRef);
};

// Factory function to create a TestIRCompilerX64 instance
// The returned pointer must be deleted by the caller using destroy_test_ir_compiler_x64
TestIRCompilerX64 *create_test_ir_compiler_x64(TestIRAdaptor *adaptor,
                                                bool no_fixed_assignments);

// Destructor function for the compiler instance
void destroy_test_ir_compiler_x64(TestIRCompilerX64 *compiler);

// Original compile function
std::vector<u8> compile_ir_x64(TestIR *ir, bool no_fixed_assignments);

} // namespace tpde::test
