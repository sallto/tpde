// SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
//
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#pragma once

#include "TestIR.hpp"
#include "tpde/CompilerBase.hpp"

#include <iostream>
#include <utility>

namespace tpde::test {
template <typename Adaptor, typename Derived, typename Config>
struct TestCompilerBase : CompilerBase<TestIRAdaptor, Derived, Config> {
  using Base = CompilerBase<TestIRAdaptor, Derived, Config>;
  u64 register_idx = 0;
  explicit TestCompilerBase(TestIRAdaptor *adaptor) : Base{adaptor} {}
  void analysis_end() {
    if (this->derived()->recommended_registers.empty()) {
      return;
    }
    this->analyzer.recommended_registers.resize(
        this->adaptor->cur_highest_val_idx() + 1);
    for (u32 i = 0; i < this->adaptor->cur_highest_val_idx(); i++) {
      this->analyzer.recommended_registers[i] =
          this->derived()->recommended_registers[register_idx++];
    }
  }
};
} // namespace tpde::test
