// SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
//
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#pragma once
#include "ValuePartRef.hpp"
#include "ValueRef.hpp"

namespace tpde {

/// Owned unspillable and unevictable temporary register with RAII semantics.
template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
struct CompilerBase<Adaptor, Derived, Config>::ScratchReg {
private:
  CompilerBase *compiler;
  // TODO(ts): get this using the CompilerConfig?
  AsmReg reg = AsmReg::make_invalid();

public:
  /// Constructor, no register.
  explicit ScratchReg(CompilerBase *compiler) : compiler(compiler) {}

  explicit ScratchReg(const ScratchReg &) = delete;
  ScratchReg(ScratchReg &&);

  ~ScratchReg() { reset(); }

  ScratchReg &operator=(const ScratchReg &) = delete;
  ScratchReg &operator=(ScratchReg &&);

  /// Whether a register is currently allocated.
  bool has_reg() const { return reg.valid(); }

  /// The allocated register.
  AsmReg cur_reg() const {
    assert(has_reg());
    return reg;
  }

  /// Allocate a specific register.
  AsmReg alloc_specific(AsmReg reg);

  /// Allocate a general-purpose register.
  AsmReg alloc_gp() { return alloc(Config::GP_BANK); }

  /// Allocate register in the specified bank. Does nothing if it holds a
  /// register in the specified bank. Evicts a register if required.
  AsmReg alloc(RegBank bank);

  /// Release register, marking it as unused; returns the register.
  AsmReg release() {
    AsmReg res = reg;
    reset();
    return res;
  }

  /// Deallocate register.
  void reset();

  /// @internal Forcefully change register without updating register file.
  /// Avoid.
  void force_set_reg(AsmReg reg) { this->reg = reg; }
};

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
CompilerBase<Adaptor, Derived, Config>::ScratchReg::ScratchReg(
    ScratchReg &&other) {
  this->compiler = other.compiler;
  this->reg = other.reg;
  other.reg = AsmReg::make_invalid();
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::ScratchReg &
    CompilerBase<Adaptor, Derived, Config>::ScratchReg::operator=(
        ScratchReg &&other) {
  if (this == &other) {
    return *this;
  }

  reset();
  this->compiler = other.compiler;
  this->reg = other.reg;
  other.reg = AsmReg::make_invalid();
  return *this;
}


template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::AsmReg
    CompilerBase<Adaptor, Derived, Config>::ScratchReg::alloc_specific(
        AsmReg reg) {
  assert(compiler->may_change_value_state());
  assert(!compiler->register_file.is_fixed(reg));
  reset();

  if (compiler->register_file.is_used(reg)) {
    // auto local_idx = compiler->register_file.reg_local_idx(reg);
    // const auto &pli =
    // compiler->analyzer.precise_liveness[static_cast<u32>(compiler->cur_block_idx)];
    // auto [c,n] = compiler->analyzer.get_current_and_next_use(pli, local_idx,
    // compiler->cur_instr_idx); todo(salto):fix in valuepart as well
    if (compiler->register_file.reg_local_idx(reg) != INVALID_VAL_LOCAL_IDX) {
      ValueAssignment *assignment =
          compiler->val_assignment(compiler->register_file.reg_local_idx(reg));
      if (assignment && assignment->variable_ref) {
        compiler->evict_reg(reg);
        compiler->register_file.mark_used(reg, INVALID_VAL_LOCAL_IDX, 0);
        compiler->register_file.mark_clobbered(reg);
        compiler->register_file.mark_fixed(reg);
        this->reg = reg;
        return reg;
      }
    }
    // we are an empty scratch reg so we just shuffle the target register away.
    auto &reg_file = compiler->register_file;
    bool success = compiler->repair_argument(
        INVALID_VAL_LOCAL_IDX,
        0,
        8,
        reg_file.reg_bank(reg),
        (1ull << reg.id()),
        (reg_file.allocatable & ~reg_file.used) &
            reg_file.bank_regs(reg_file.reg_bank(reg)),
        0,
        this->has_reg() ? this->cur_reg() : AsmReg::make_invalid());
    if (success) [[likely]] {
      auto moves = compiler->sequentialize(compiler->parallel_copies);
      for (auto move : moves) {
        if (move.value_idx != INVALID_VAL_LOCAL_IDX) {
          ValueAssignment *assignment =
              compiler->val_assignment(move.value_idx);
          if (!assignment) {
            continue;
          }
          AssignmentPartRef ap{assignment, move.part_idx};
          
          compiler->global_assign(move.value_idx, move.dst);

          ap.mov(compiler, move.value_idx, move.dst);
#ifndef NDEBUG

          compiler->verification_ir.emit_active_reg_move(
              move.src, move.dst, move.size);

#endif
        } else {
          compiler->derived()->mov(move.dst, move.src, 8);
          reg_file.mark_used(Reg{move.dst}, move.value_idx, move.part_idx);
          reg_file.mark_clobbered(Reg{move.dst});
          reg_file.mark_fixed(Reg{move.dst});
        }
      }
      // target register must now be free
      compiler->parallel_copies.clear();
    } else {
      compiler->evict_reg(reg);
    }
  }

  compiler->register_file.mark_used(reg, INVALID_VAL_LOCAL_IDX, 0);
  compiler->register_file.mark_clobbered(reg);
  compiler->register_file.mark_fixed(reg);
  this->reg = reg;
  return reg;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
CompilerBase<Adaptor, Derived, Config>::AsmReg
    CompilerBase<Adaptor, Derived, Config>::ScratchReg::alloc(
        RegBank bank) {
  assert(compiler->may_change_value_state());

  auto &reg_file = compiler->register_file;
  if (!reg.invalid()) {
    assert(bank == reg_file.reg_bank(reg));
    return reg;
  }
  TPDE_LOG_TRACE("Allocating new register");
  // todo(salto)
  auto local = compiler->select_reg(bank, /*exclusion_mask=*/0);
  // compiler->tree_ra_ctx->global_regs.push_back(global);
  // compiler->tree_ra_ctx->used_global_regs |= (1ull << reg.id()),
  reg = local;
  reg_file.mark_used(reg, INVALID_VAL_LOCAL_IDX, 0);
  reg_file.mark_clobbered(reg);
  reg_file.mark_fixed(reg);
  return reg;
}

  template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
  void CompilerBase<Adaptor, Derived, Config>::ScratchReg::reset()  {
    if (reg.invalid()) {
      return;
    }

    compiler->register_file.unmark_fixed(reg);
    compiler->register_file.unmark_used(reg);
    reg = AsmReg::make_invalid();
  }
} // namespace tpde
