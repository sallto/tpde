// SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
//
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception

#include "TestIRCompiler.hpp"

namespace tpde::test {
using namespace tpde::x64;

bool TestIRCompilerX64::compile_inst(IRInstRef inst_idx, InstRange) noexcept {
  const TestIR::Value &value =
      this->analyzer.adaptor->ir->values[static_cast<u32>(inst_idx)];
  assert(value.type == TestIR::Value::Type::normal ||
         value.type == TestIR::Value::Type::terminator);

  switch (value.op) {
    using enum TestIR::Value::Op;
  case add: return compile_add(inst_idx);
  case sub: return compile_sub(inst_idx);
  case div: return compile_div(inst_idx);
  case condselect: return compile_condselect(inst_idx);
  case terminate:
  case ret: {
    RetBuilder rb{*derived(), *cur_cc_assigner()};
    if (value.op_count == 1) {
      const auto op = static_cast<IRValueRef>(
          this->adaptor->ir->value_operands[value.op_begin_idx]);

      rb.add(op);
    }
    rb.ret();
    return true;
  }
  case alloca: return true;
  case br: {
    auto block_idx = ir()->value_operands[value.op_begin_idx];
    auto spilled = this->spill_before_branch();

    this->generate_branch_to_block(
        Jump::jmp, static_cast<IRBlockRef>(block_idx), false, true);

    this->release_spilled_regs(spilled);
    return true;
  }
  case condbr: {
    auto val_idx =
        static_cast<IRValueRef>(ir()->value_operands[value.op_begin_idx]);
    auto true_block =
        static_cast<IRBlockRef>(ir()->value_operands[value.op_begin_idx + 1]);
    auto false_block =
        static_cast<IRBlockRef>(ir()->value_operands[value.op_begin_idx + 2]);

    auto [_, val] = this->val_ref_single(val_idx);

    auto true_needs_split = this->branch_needs_split(true_block);
    auto false_needs_split = this->branch_needs_split(false_block);

    auto val_reg = val.load_to_reg();

    auto spilled = this->spill_before_branch();

    ASM(CMP64ri, val_reg, 0);
    if (this->analyzer.block_ref(this->next_block()) == true_block) {
      this->generate_branch_to_block(
          Jump::je, false_block, false_needs_split, false);
      this->generate_branch_to_block(Jump::jmp, true_block, false, true);
    } else if (this->analyzer.block_ref(this->next_block()) == false_block) {
      this->generate_branch_to_block(
          Jump::jne, true_block, true_needs_split, false);
      this->generate_branch_to_block(Jump::jmp, false_block, false, true);
    } else if (!true_needs_split) {
      this->generate_branch_to_block(Jump::jne, true_block, false, false);
      this->generate_branch_to_block(Jump::jmp, false_block, false, true);
    } else {
      this->generate_branch_to_block(
          Jump::je, false_block, false_needs_split, false);
      this->generate_branch_to_block(Jump::jmp, true_block, false, true);
    }

    this->release_spilled_regs(spilled);
    return true;
  }
  case call: {
    const auto func_idx = value.call_func_idx;
    auto operands = std::span<IRValueRef>{
        reinterpret_cast<IRValueRef *>(ir()->value_operands.data() +
                                       value.op_begin_idx),
        value.op_count};

    auto res_ref = this->result_ref(static_cast<IRValueRef>(inst_idx));

    util::SmallVector<CallArg, 8> arguments{};
    for (auto op : operands) {
      arguments.push_back(CallArg{op});
    }

    this->generate_call(this->func_syms[func_idx], arguments, &res_ref);
    return true;
  }
  default: return false;
  }

  return false;
}

bool TestIRCompilerX64::compile_add(IRInstRef inst_idx) noexcept {
  const TestIR::Value &value = ir()->values[static_cast<u32>(inst_idx)];

  const auto lhs_idx =
      static_cast<IRValueRef>(ir()->value_operands[value.op_begin_idx]);
  const auto rhs_idx =
      static_cast<IRValueRef>(ir()->value_operands[value.op_begin_idx + 1]);

  auto [lhs_vr, lhs] = this->val_ref_single(lhs_idx);
  auto [rhs_vr, rhs] = this->val_ref_single(rhs_idx);
  auto [res_vr, res] =
      this->result_ref_single(static_cast<IRValueRef>(inst_idx));
  AsmReg lhs_reg = lhs.load_to_preferred_reg();
  AsmReg rhs_reg = rhs.load_to_preferred_reg();
  AsmReg res_reg = res.alloc_try_reuse(lhs);

  if (res_reg == lhs_reg) {
    ASM(ADD64rr, res_reg, rhs_reg);
  } else {
    ASM(LEA64rm, res_reg, FE_MEM(lhs_reg, 1, rhs_reg, 0));
  }
  res.set_modified();
  return true;
}

bool TestIRCompilerX64::compile_sub(IRInstRef inst_idx) noexcept {
  const TestIR::Value &value = ir()->values[static_cast<u32>(inst_idx)];

  const auto lhs_idx =
      static_cast<IRValueRef>(ir()->value_operands[value.op_begin_idx]);
  const auto rhs_idx =
      static_cast<IRValueRef>(ir()->value_operands[value.op_begin_idx + 1]);

  auto lhs = this->val_ref(lhs_idx);
  auto rhs = this->val_ref(rhs_idx);

  ValuePartRef result = lhs.part(0).into_temporary();
  ASM(SUB64rr, result.cur_reg(), rhs.part(0).load_to_reg());
  this->result_ref(static_cast<IRValueRef>(inst_idx))
      .part(0)
      .set_value(std::move(result));
  return true;
}

bool TestIRCompilerX64::compile_div(IRInstRef inst_idx) noexcept {
  const TestIR::Value &value = ir()->values[static_cast<u32>(inst_idx)];

  const auto lhs_idx =
      static_cast<IRValueRef>(ir()->value_operands[value.op_begin_idx]);
  const auto rhs_idx =
      static_cast<IRValueRef>(ir()->value_operands[value.op_begin_idx + 1]);

  auto [lhs_vr, lhs] = this->val_ref_single(lhs_idx);
  auto [rhs_vr, rhs] = this->val_ref_single(rhs_idx);
  auto [res_vr, res] = this->result_ref_single(static_cast<IRValueRef>(inst_idx));

  // Reserve RAX and RDX registers (DIV instruction overwrites both)
  ScratchReg rax_scratch{this};
  ScratchReg rdx_scratch{this};
  AsmReg rdx_reg = rdx_scratch.alloc_specific(AsmReg::DX);

  // Load dividend into RAX
  lhs.load_to_specific(AsmReg::AX);

  // Zero RDX for unsigned division
  ASM(XOR64rr, rdx_reg, rdx_reg);

  // Load divisor and perform division
  AsmReg divisor_reg = rhs.load_to_reg();
  ASM(DIV64r, divisor_reg);

  // Move quotient from RAX to result
  res.set_value(std::move(lhs));

  return true;
}

bool TestIRCompilerX64::compile_condselect(IRInstRef inst_idx) noexcept {
  const TestIR::Value &value = ir()->values[static_cast<u32>(inst_idx)];

  const auto lhs_comp_idx =
      static_cast<IRValueRef>(ir()->value_operands[value.op_begin_idx]);
  const auto rhs_comp_idx =
      static_cast<IRValueRef>(ir()->value_operands[value.op_begin_idx + 1]);
  const auto lhs_idx =
      static_cast<IRValueRef>(ir()->value_operands[value.op_begin_idx + 2]);
  const auto rhs_idx =
      static_cast<IRValueRef>(ir()->value_operands[value.op_begin_idx + 3]);

  {
    auto [lhs_vr, lhs] = this->val_ref_single(lhs_comp_idx);
    auto [rhs_vr, rhs] = this->val_ref_single(rhs_comp_idx);

    ASM(CMP64rr, lhs.load_to_reg(), rhs.load_to_reg());
  }

  auto [lhs_vr, lhs] = this->val_ref_single(lhs_idx);
  auto [rhs_vr, rhs] = this->val_ref_single(rhs_idx);
  auto [res_vr, res] =
      this->result_ref_single(static_cast<IRValueRef>(inst_idx));

  auto res_tmp = std::move(lhs).into_temporary();
  auto rhs_reg = rhs.load_to_reg();

  Jump cc;
  switch (static_cast<TestIR::Value::Cond>(
      ir()->value_operands[value.op_begin_idx + 4])) {
    using enum TestIR::Value::Cond;
  case eq: cc = Jump::je; break;
  case neq: cc = Jump::jne; break;
  case uge: cc = Jump::jae; break;
  case ugt: cc = Jump::ja; break;
  case ule: cc = Jump::jbe; break;
  case ult: cc = Jump::jb; break;
  case sge: cc = Jump::jge; break;
  case sgt: cc = Jump::jg; break;
  case sle: cc = Jump::jle; break;
  case slt: cc = Jump::jl; break;
  default: return false;
  }
  cc = invert_jump(cc);

  generate_raw_cmov(cc, res_tmp.cur_reg(), rhs_reg, true);

  res.set_value(std::move(res_tmp));
  return true;
}
} // namespace tpde::test
