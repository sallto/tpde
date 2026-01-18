// SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
//
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#pragma once
#include "ValuePartRef.hpp"
#include "ValueRef.hpp"

namespace tpde {
    template<IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
    struct CompilerBase<Adaptor, Derived, Config>::ScratchReg {
    private:
        CompilerBase *compiler;
        // TODO(ts): get this using the CompilerConfig?
        AsmReg reg = AsmReg::make_invalid();

        bool repair_argument(CompilerBase *compiler,
                             ValLocalIdx var,
                             typename RegisterFile::RegBitSet constraints,
                             typename RegisterFile::RegBitSet available,
                             typename RegisterFile::RegBitSet forbidden = 0);

    public:
        explicit ScratchReg(CompilerBase *compiler) : compiler(compiler) {
        }

        explicit ScratchReg(const ScratchReg &) = delete;

        ScratchReg(ScratchReg &&) noexcept;

        ~ScratchReg() noexcept { reset(); }

        ScratchReg &operator=(const ScratchReg &) = delete;

        ScratchReg &operator=(ScratchReg &&) noexcept;

        bool has_reg() const noexcept { return reg.valid(); }

        AsmReg cur_reg() const noexcept {
            assert(has_reg());
            return reg;
        }

        AsmReg alloc_specific(AsmReg reg) noexcept;

        AsmReg alloc_gp() noexcept { return alloc(Config::GP_BANK); }

        /// Allocate register in the specified bank, optionally excluding certain
  /// non-fixed registers. Spilling can be disabled for spill code to avoid
  /// recursion; if spilling is disabled, the allocation can fail.
        AsmReg alloc(RegBank bank) noexcept;

        /// Allocate register, try to match the recommended registor of target
        AsmReg alloc_rec(RegBank bank, ValuePart &target) noexcept;

        AsmReg release() noexcept {
            AsmReg res = reg;
            reset();
            return res;
        }

        void reset() noexcept;

        /// Forcefully change register without updating register file. Avoid.
        void force_set_reg(AsmReg reg) noexcept { this->reg = reg; }
    };

    template<IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
    CompilerBase<Adaptor, Derived, Config>::ScratchReg::ScratchReg(
        ScratchReg &&other) noexcept {
        this->compiler = other.compiler;
        this->reg = other.reg;
        other.reg = AsmReg::make_invalid();
    }

    template<IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
    typename CompilerBase<Adaptor, Derived, Config>::ScratchReg &
    CompilerBase<Adaptor, Derived, Config>::ScratchReg::operator=(
        ScratchReg &&other) noexcept {
        if (this == &other) {
            return *this;
        }

        reset();
        this->compiler = other.compiler;
        this->reg = other.reg;
        other.reg = AsmReg::make_invalid();
        return *this;
    }

    template<IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
    bool CompilerBase<Adaptor, Derived, Config>::ScratchReg::repair_argument(
        CompilerBase *compiler,
        ValLocalIdx var,
        typename RegisterFile::RegBitSet constraints,
        typename RegisterFile::RegBitSet available,
        typename RegisterFile::RegBitSet forbidden) {
        auto &reg_file = compiler->register_file;
        Reg reg = Reg::make_invalid();
        std::unordered_set<ValLocalIdx> operands;
        // commented out current_instr usage
        // for (auto operand:
        //      compiler->adaptor->inst_operands(*compiler->tree_ra_ctx->current_instr)) {
        //   if (compiler->adaptor->val_ignore_in_liveness_analysis(operand)) {
        //     operands.insert(INVALID_VAL_LOCAL_IDX);
        //     continue;
        //   }
        //   operands.insert(compiler->adaptor->val_local_idx(operand));
        // }
        bool success = false;
        typename RegisterFile::RegBitSet allowed =
                constraints & (~forbidden); // todo(salto): constraints
        while (reg == Reg::make_invalid() && allowed != 0) {
            for (u64 candidate: util::BitSetIterator<>(allowed)) {
                if (reg_file.is_used(Reg{candidate}) &&
                    (!operands.contains(reg_file.reg_local_idx(Reg{candidate})) &&
                     !reg_file.is_fixed(Reg{candidate}))) {
                    reg = Reg{candidate};
                    break;
                }
            }
            if (reg == Reg::make_invalid()) {
                // todo(salto): choose color from allowed
                //reg =compiler->register_file.find_first_free_excluding(reg_file.reg_bank(this->cur_reg()),forbidden);
                reg = Reg{*util::BitSetIterator<>(allowed).begin()};
            }
            ValLocalIdx pawn = reg_file.reg_local_idx(reg);
            // todo(salto): constraints of pawn?
            // |
            //(this->has_reg() ? (1ull << this->cur_reg().id()) : 0ull)
            typename RegisterFile::RegBitSet pawnAllowed =
                    (available) &
                    (~forbidden);
            if (pawnAllowed != 0) {
                Reg pawnReg = compiler->register_file.find_first_free_excluding(reg_file.reg_bank(reg), ~pawnAllowed);
                compiler->parallel_copies.emplace_back(
                    pawnReg,
                    reg,
                    8,
                    pawn,
                    reg_file.reg_part(reg));
                success = true;
            } else {
                success = repair_argument(compiler,
                                          pawn,
                                          available | (1ull << this->cur_reg().id()),
                                          forbidden | (1ull << reg.id()));
            }
            if (!success) {
                allowed &= ~(1ull << reg.id());
                reg = Reg::make_invalid();
            }
        }
        if (reg != Reg::make_invalid()) {
            if (this->has_reg())
                compiler->parallel_copies.emplace_back(
                    Reg{reg}, this->cur_reg(), 8, var, reg_file.reg_part(reg));
            return true;
        }
        return false;
    }

    template<IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
    typename CompilerBase<Adaptor, Derived, Config>::AsmReg
    CompilerBase<Adaptor, Derived, Config>::ScratchReg::alloc_specific(
        AsmReg reg) noexcept {
        assert(compiler->may_change_value_state());
        assert(!compiler->register_file.is_fixed(reg));
        reset();

        if (compiler->register_file.is_used(reg)) {
            //auto local_idx = compiler->register_file.reg_local_idx(reg);
            //const auto &pli = compiler->analyzer.precise_liveness[static_cast<u32>(compiler->cur_block_idx)];
            //auto [c,n] = compiler->analyzer.get_current_and_next_use(pli, local_idx, compiler->cur_instr_idx);

            // we are an empty scratch reg so we just shuffle the target register away.
            auto &reg_file = compiler->register_file;
            bool success =
                    repair_argument(compiler,
                                    INVALID_VAL_LOCAL_IDX,
                                    (1ull << reg.id()),
                                    (reg_file.allocatable & ~reg_file.used) &
                                    reg_file.bank_regs(reg_file.reg_bank(reg)));
            if (success) [[likely]] {
                auto moves =
                        compiler->sequentialize(compiler->parallel_copies);
                for (auto move: moves) {
                    if (move.value_idx != INVALID_VAL_LOCAL_IDX) {
                        ValueAssignment *assignment = compiler->val_assignment(move.value_idx);
                        if (!assignment)
                            continue;
                        AssignmentPartRef ap{assignment, move.part_idx};
                        if (ap.register_valid() && assignment->pending_free) {
                            ap.set_register_valid(false);
                            reg_file.unmark_used(move.src);
                            continue;
                        }
                        compiler->global_assign(move.value_idx, move.dst);
                        ValueRef vr{compiler, move.value_idx};
                        vr.disown();
                        vr.part_unowned(move.part_idx).mov(move.dst);
                    } else {
                        compiler->derived()->mov(move.dst, move.src, 8);
                        reg_file.mark_used(
                            Reg{move.dst}, move.value_idx, move.part_idx);
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

    template<IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
    CompilerBase<Adaptor, Derived, Config>::AsmReg
    CompilerBase<Adaptor, Derived, Config>::ScratchReg::alloc(
        RegBank bank) noexcept {
        assert(compiler->may_change_value_state());

        auto &reg_file = compiler->register_file;
        if (!reg.invalid()) {
            assert(bank == reg_file.reg_bank(reg));
            return reg;
        }
        TPDE_LOG_INFO("Allocating new register");
        // todo(salto)
        auto [local, global] = compiler->select_reg(bank, /*exclusion_mask=*/0);
        //compiler->tree_ra_ctx->global_regs.push_back(global);
        //compiler->tree_ra_ctx->used_global_regs |= (1ull << reg.id()),
        reg = local;
        reg_file.mark_used(reg, INVALID_VAL_LOCAL_IDX, 0);
        reg_file.mark_clobbered(reg);
        reg_file.mark_fixed(reg);
        return reg;
    }

    template<IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
    CompilerBase<Adaptor, Derived, Config>::AsmReg
    CompilerBase<Adaptor, Derived, Config>::ScratchReg::alloc_rec(
        RegBank bank, ValuePart &target) noexcept {
        assert(compiler->may_change_value_state());

        auto &reg_file = compiler->register_file;
        // todo(salto): should the scratch be moved?
        if (!reg.invalid()) {
            assert(bank == reg_file.reg_bank(reg));
            return reg;
        }
        if (target.preferred_register() == 255) {
            return alloc(bank);
        }
        auto ideal_reg = Reg{target.preferred_register()};
        if (compiler->register_file.is_fixed(ideal_reg)) {
            return alloc(bank);
        }
        if (compiler->register_file.is_used(ideal_reg)) {
            compiler->evict_reg(ideal_reg);
            reset();
        }
        reg = ideal_reg;
        reg_file.mark_used(reg, INVALID_VAL_LOCAL_IDX, 0);
        reg_file.mark_clobbered(reg);
        reg_file.mark_fixed(reg);
        return reg;
    }

    template<IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
    void CompilerBase<Adaptor, Derived, Config>::ScratchReg::reset() noexcept {
        if (reg.invalid()) {
            return;
        }

        compiler->register_file.unmark_fixed(reg);
        compiler->register_file.unmark_used(reg);
        reg = AsmReg::make_invalid();
    }
} // namespace tpde