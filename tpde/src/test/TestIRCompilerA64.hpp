// SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
//
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#pragma once

#include "TestIR.hpp"
#include "tpde/base.hpp"

namespace tpde::test {
bool compile_ir_arm64(TestIR *ir,
                      bool no_fixed_assignments,
                      const std::string &obj_out_path,
                      std::vector<Reg> registers);
}
