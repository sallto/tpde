// SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
//
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#pragma once

#include <algorithm>
#include <array>
#include <format>
#include <iostream>
#include <limits>
#include <llvm/ADT/SetVector.h>
#include <ostream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "DominatorTree.hpp"
#include "IRAdaptor.hpp"
#include "RegisterFile.hpp"
#include "tpde/ValLocalIdx.hpp"
#include "tpde/base.hpp"
#include "util/SmallBitSet.hpp"
#include "util/SmallVector.hpp"

namespace tpde {
    /// An index into block_layout
    enum class BlockIndex : u32 {
    };

    static constexpr BlockIndex INVALID_BLOCK_IDX = static_cast<BlockIndex>(~0u);


    template<IRAdaptor Adaptor, typename CompilerType>
    struct Analyzer {
        // some forwards for the IR type defs
        using IRValueRef = typename Adaptor::IRValueRef;
        using IRInstRef = typename Adaptor::IRInstRef;
        using IRBlockRef = typename Adaptor::IRBlockRef;
        using IRFuncRef = typename Adaptor::IRFuncRef;

        static constexpr IRBlockRef INVALID_BLOCK_REF = Adaptor::INVALID_BLOCK_REF;


        static constexpr size_t SMALL_BLOCK_NUM = 64;
        static constexpr size_t SMALL_VALUE_NUM = 128;

        /// Reference to the adaptor
        Adaptor *adaptor;

        /// Reference to the compiler base
        CompilerType *compiler;

        /// The block layout, a BlockIndex is an index into this array
        util::SmallVector<IRBlockRef, SMALL_BLOCK_NUM> block_layout = {};

        /// For each BlockIndex, the corresponding loop
        // TODO(ts): add the delayed free list in here to save on allocations?
        util::SmallVector<u32, SMALL_BLOCK_NUM> block_loop_map = {};

        struct Loop {
            u32 level;
            u32 parent;
            // [begin, end[
            BlockIndex begin = INVALID_BLOCK_IDX, end = INVALID_BLOCK_IDX;

            // for building the loop tree, we accumulate the number of blocks here
            u32 num_blocks = 0;
            // TODO(ts): add skip_target?

            u32 definitions = 0, definitions_in_childs = 0;
            u32 max_gp_pressure = 0;
            u32 max_fp_pressure = 0;
            bool is_irreducible = false;
        };

        util::SmallVector<Loop, 16> loops = {};

        // TODO(ts): move all struct definitions to the top?
        struct LivenessInfo {
            /// Block that owns the value definition.
            BlockIndex definition_block;
            // [first, last]
            BlockIndex first, last;
            u32 ref_count;
            u32 lowest_common_loop;

            // TODO(ts): maybe outsource both these booleans to a bitset?
            // we're wasting a lot of space here

            /// The value may not be deallocated until the last block is finished
    /// even if the reference count hits 0
            bool last_full;

            u16 epoch = 0;
        };

        util::SmallVector<LivenessInfo, SMALL_VALUE_NUM> liveness = {};
        /// Epoch of liveness information, entries with a value not equal to this
  /// epoch are invalid. This is an optimization to avoid clearing the entire
  /// liveness vector for every function, which is important for functions with
  /// many values that are ignored for the liveness analysis (e.g., var refs).
  /// also used for value parts cache.
        u16 liveness_epoch = 0;
        u32 liveness_max_value;

        static constexpr u32 DEF_BIT = 1u << 31;
        static constexpr u32 INF = std::numeric_limits<u32>::max();
        static constexpr u32 LOOP_EXIT_PENALTY = 10'000'000u;

        struct PreciseLivenessInfo {
            // val_local_idx -> list of next-use distances from the start of each block.
            // first entry is the epoch

            std::unordered_map<ValLocalIdx, util::SmallVector<u32, 16> > next_uses;
            util::SmallVector<std::pair<ValLocalIdx, u32>, 64> live_through;
        };

        // pro block liveness info
        util::SmallVector<PreciseLivenessInfo, 32> precise_liveness;
        /// Initial working set selected at block entry for loop headers.
        std::unordered_map<BlockIndex, std::unordered_set<ValLocalIdx> >
        initial_working_set_by_block = {};
        // If a value is spilled at any point, it is marked here. During codegen we
        // spill immediately after definition to avoid storing the spill location.
        util::SmallBitSet<SMALL_VALUE_NUM> spilled_values;
        // values that are alive after a call
        util::SmallVector<ValLocalIdx, SMALL_VALUE_NUM> callee_saved_values;
        /// Mapping from ValLocalIdx to simple PHI web index.
        /// Values with INVALID_WEB_IDX were not assigned to any web.
        static constexpr u32 INVALID_WEB_IDX = ~0u;
        util::SmallVector<Reg, SMALL_VALUE_NUM> recommended_registers = {};

        struct RegParts {
            u16 epoch = 0;
            u8 gp_regs = 255;
            u8 fp_regs = 255;

            explicit RegParts(u16 epoch, u8 gp_regs, u8 fp_regs)
                : epoch(epoch), gp_regs(gp_regs), fp_regs(fp_regs) {
            }

            explicit RegParts() : epoch(0), gp_regs(255), fp_regs(255) {
            };
        };

        static_assert(sizeof(RegParts) == 4);
        util::SmallVector<RegParts, SMALL_VALUE_NUM> value_parts_cache = {};

        /// Helper class to track the working set of values in registers during spill
          /// analysis. Maintains consistency between the set of values and the register
          /// count.
        class WorkingSetTracker {
        private:
            using IRValueRef = typename Adaptor::IRValueRef;
            std::unordered_set<ValLocalIdx> values_;
            u32 used_gp_regs_ = 0;
            u32 used_fp_regs_ = 0;
            u32 result_gp_regs_ = 0;
            u32 result_fp_regs_ = 0;
            Analyzer *analyzer_;

        public:
            explicit WorkingSetTracker(

                Analyzer *analyzer) : analyzer_(analyzer) {
            }

            /// Insert a value into the working set.
            /// Returns true if the value was newly inserted.
            bool insert(ValLocalIdx val_idx) {
                const auto [it, inserted] = values_.insert(val_idx);
                if (inserted) {
                    const RegParts parts = analyzer_->ensure_parts_cached(val_idx);

                    used_gp_regs_ += parts.gp_regs;
                    used_fp_regs_ += parts.fp_regs;
                }
                return inserted;
            }

            /// Erase a value from the working set.
            /// Returns true if the value was found and erased.
            bool erase(ValLocalIdx val_idx) {
                const auto it = values_.find(val_idx);
                if (it != values_.end()) {
                    const RegParts parts = analyzer_->ensure_parts_cached(val_idx);

                    used_gp_regs_ -= parts.gp_regs;
                    used_fp_regs_ -= parts.fp_regs;
                    values_.erase(it);
                    return true;
                }
                return false;
            }

            /// Check if a value is in the working set.
            bool contains(ValLocalIdx val_idx) const {
                return values_.contains(val_idx);
            }

            /// Clear all values from the working set.
            void clear() {
                values_.clear();
                used_gp_regs_ = 0;
                used_fp_regs_ = 0;
                result_gp_regs_ = 0;
                result_fp_regs_ = 0;
            }

            /// Replace the working set with a new set of values.
            void replace_with(const std::unordered_set<ValLocalIdx> &new_values) {
                values_ = new_values;
                recalculate_used_regs();
            }

            /// Replace the working set with values from a vector.
            void replace_with(const util::SmallVector<ValLocalIdx, 16> &new_values) {
                values_.clear();
                values_.insert(new_values.begin(), new_values.end());
                recalculate_used_regs();
            }

            /// Get the current register usage.
            u32 used_gp_regs() const { return used_gp_regs_; }
            u32 used_fp_regs() const { return used_fp_regs_; }

            u32 used_gp_regs(bool include_results) const {
                return used_gp_regs_ + (include_results ? result_gp_regs_ : 0);
            }

            u32 used_fp_regs(bool include_results) const {
                return used_fp_regs_ + (include_results ? result_fp_regs_ : 0);
            }

            /// Check if there's capacity for additional registers.
            bool fits_into(u32 total_gp_capacity,
                           u32 total_fp_capacity,
                           bool include_results) const {
                const u32 gp_regs = used_gp_regs_ + (include_results ? result_gp_regs_ : 0);
                const u32 fp_regs = used_fp_regs_ + (include_results ? result_fp_regs_ : 0);
                return gp_regs <= total_gp_capacity && fp_regs <= total_fp_capacity;
            }

            /// Check if a specific value can fit.
            bool can_fit(ValLocalIdx val_idx,
                         u32 total_gp_capacity,
                         u32 total_fp_capacity) const {
                const RegParts parts = analyzer_->ensure_parts_cached(val_idx);

                return used_gp_regs_ + parts.gp_regs <= total_gp_capacity &&
                       used_fp_regs_ + parts.fp_regs <= total_fp_capacity;
            }


            /// Insert a result value into the working set without increasing used regs.
            /// Returns true if the value was newly inserted.
            bool insert_as_result(ValLocalIdx val_idx) {
                const auto [it, inserted] = values_.insert(val_idx);
                if (inserted) {
                    const auto parts = analyzer_->ensure_parts_cached(val_idx);
                    result_gp_regs_ += parts.gp_regs;
                    result_fp_regs_ += parts.fp_regs;
                }
                return inserted;
            }

            /// Commit result register usage into the main counters.
            void commit_result_regs() {
                used_gp_regs_ += result_gp_regs_;
                used_fp_regs_ += result_fp_regs_;
                result_gp_regs_ = 0;
                result_fp_regs_ = 0;
            }

            /// Iteration support (const).
            auto begin() const { return values_.begin(); }
            auto end() const { return values_.end(); }

            /// Iteration support (non-const).
            auto begin() { return values_.begin(); }
            auto end() { return values_.end(); }

        private:
            /// Recalculate used registers from scratch based on current values.
            void recalculate_used_regs() {
                used_gp_regs_ = 0;
                used_fp_regs_ = 0;
                result_gp_regs_ = 0;
                result_fp_regs_ = 0;
                for (const auto val_idx: values_) {
                    const RegParts parts = analyzer_->ensure_parts_cached(val_idx);
                    used_gp_regs_ += parts.gp_regs;
                    used_fp_regs_ += parts.fp_regs;
                }
            }
        };

        u32 num_insts;


        struct BlockPressure {
            u32 gp_pressure = 0;
            u32 fp_pressure = 0;
        };

        struct ValueInterval {
            u32 first;
            u32 last;
        };

        struct SpillCandidate {
            ValLocalIdx val_idx;
            u32 current_use;
            u32 next_use;
            RegParts parts;
        };

        util::SmallVector<BlockPressure, SMALL_BLOCK_NUM> block_pressure = {};
        util::SmallVector<util::SmallVector<ValLocalIdx, 4>, SMALL_VALUE_NUM> web_members = {};
        util::SmallVector<u32, SMALL_VALUE_NUM> web_parent = {};

        /// Dominator tree for control flow analysis
        DominatorTree<Adaptor, BlockIndex> dominator_tree = {};

        explicit Analyzer(Adaptor *adaptor, CompilerType *compiler = nullptr)
            : adaptor(adaptor), compiler(compiler) {
        }

        /// Start the compilation of a new function and build the loop tree and
  /// liveness information. Previous information is discarded.
        void switch_func(IRFuncRef func);

        IRBlockRef block_ref(const BlockIndex idx) const noexcept {
            assert(static_cast<u32>(idx) <= block_layout.size());
            if (static_cast<u32>(idx) == block_layout.size()) {
                // this might be called with next_block() which is invalid for the
                // last block
                return INVALID_BLOCK_REF;
            }
            return block_layout[static_cast<u32>(idx)];
        }

        BlockIndex block_idx(IRBlockRef block_ref) const noexcept {
            return static_cast<BlockIndex>(adaptor->block_info(block_ref));
        }

        const LivenessInfo &liveness_info(const ValLocalIdx val_idx) const noexcept {
            assert(static_cast<u32>(val_idx) < liveness.size());
            assert(liveness[static_cast<u32>(val_idx)].epoch == liveness_epoch &&
                "access to liveness of ignored value");
            return liveness[static_cast<u32>(val_idx)];
        }

        u32 block_loop_idx(const BlockIndex idx) const noexcept {
            return block_loop_map[static_cast<u32>(idx)];
        }

        const Loop &loop_from_idx(const u32 idx) const noexcept { return loops[idx]; }

        bool block_has_multiple_incoming(const BlockIndex idx) const noexcept {
            return block_has_multiple_incoming(block_ref(idx));
        }

        bool block_has_multiple_incoming(IRBlockRef block_ref) const noexcept {
            return (adaptor->block_info2(block_ref) & 0b11) == 2;
        }

        bool block_has_phis(BlockIndex idx) const noexcept {
            return block_has_phis(block_ref(idx));
        }

        bool block_has_phis(IRBlockRef block_ref) const noexcept {
            return (adaptor->block_info2(block_ref) & 0b1'0000) != 0;
        }

        BlockIndex immediate_dominator(BlockIndex idx) const noexcept {
            return dominator_tree.get_idom(idx);
        }

        bool dominates(BlockIndex lhs, BlockIndex rhs) const noexcept {
            return dominator_tree.dominates(lhs, rhs);
        }

        void print_rpo(std::ostream &os) const;

        void print_block_layout(std::ostream &os) const;

        void print_loops(std::ostream &os) const;

        void print_liveness(std::ostream &os) const;

        void print_precise_liveness(std::ostream &os);

        void print_phi_webs(std::ostream &os) const;

        u32 find_web_idx(u32 idx);

    protected:
        using PhiWebMembers = std::unordered_map<u32, util::SmallVector<ValLocalIdx, 8> >;

        void collect_phi_web_members(PhiWebMembers &web_members) const;

        void collect_value_refs(
            std::unordered_map<ValLocalIdx, IRValueRef> &value_refs) const;

        // for use during liveness analysis
        LivenessInfo &liveness_maybe(const IRValueRef val) noexcept;

        void build_block_layout();

        void build_loop_tree_and_block_layout(
            const util::SmallVector<IRBlockRef, SMALL_BLOCK_NUM> &block_rpo,
            const util::SmallVector<u32, SMALL_BLOCK_NUM> &loop_parent,
            const util::SmallBitSet<256> &loop_heads);

        /// Builds a vector of block references in reverse post-order.
        void build_rpo_block_order(
            util::SmallVector<IRBlockRef, SMALL_BLOCK_NUM> &out) const noexcept;

        /// Creates a bitset of loop_heads and sets the parent loop of each block.
  /// The u32 in loop_parent is an index into block_rpo
        void identify_loops(
            const util::SmallVector<IRBlockRef, SMALL_BLOCK_NUM> &block_rpo,
            util::SmallVector<u32, SMALL_BLOCK_NUM> &loop_parent,
            util::SmallBitSet<256> &loop_heads) const noexcept;

        void compute_liveness() noexcept;

    public:
        std::pair<u32, u32> get_current_and_next_use(const PreciseLivenessInfo &pli,
                                                     ValLocalIdx val_idx,
                                                     const u32 idx);

    protected:
        void compute_precise_liveness() noexcept;

        void compute_spills() noexcept;

        void limit(util::SmallVector<SpillCandidate, 16> &W_next_uses,
                   const u32 NUM_GP_REGS,
                   const u32 NUM_FP_REGS,
                   u32 &idx,
                   WorkingSetTracker &working_set,
                   bool after_instr = false);

        // todo maybe always inline
        bool is_parts_cached(ValLocalIdx val_idx) const {
            assert(val_idx < value_parts_cache.size());
            return value_parts_cache[val_idx].epoch == liveness_epoch;
        }

        RegParts ensure_parts_cached(ValLocalIdx val_idx) {
            const u32 val_idx_u32 = static_cast<u32>(val_idx);
            assert(val_idx_u32 < value_parts_cache.size());
            if (value_parts_cache[val_idx_u32].epoch != liveness_epoch) {
                value_parts_cache[val_idx_u32].epoch = liveness_epoch;
                const auto value_parts = adaptor->val_parts(val_idx);
                std::array<u8, 2> parts = {0u, 0u};
                for (u32 i = 0; i < value_parts.count(); ++i) {
                    const u8 bank_id = value_parts.reg_bank(i).id();
                    assert(bank_id < parts.size());
                    ++parts[bank_id];
                }
                value_parts_cache[val_idx_u32].epoch = liveness_epoch;
                value_parts_cache[val_idx_u32].gp_regs = parts[0];
                value_parts_cache[val_idx_u32].fp_regs = parts[1];
            }
            return value_parts_cache[val_idx_u32];
        }
    };

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::switch_func([[maybe_unused]] IRFuncRef func) {
        build_block_layout();
        compute_liveness();
        //todo(salto): add options to disable precise liveness / spill computation
        compiler->derived()->analysis_precise_liveness_start();

        compute_precise_liveness();

        compiler->derived()->analysis_precise_liveness_end();

        compiler->derived()->analysis_spills_start();

        compute_spills();

        compiler->derived()->analysis_spills_end();
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::print_rpo(std::ostream &os) const {
        // build_rpo_block_order clobbers block data, so save and restore.
        util::SmallVector<std::tuple<IRBlockRef, u32, u32>, SMALL_BLOCK_NUM> data;
        for (IRBlockRef cur: adaptor->cur_blocks()) {
            data.emplace_back(cur, adaptor->block_info(cur), adaptor->block_info2(cur));
        }

        util::SmallVector<IRBlockRef, SMALL_BLOCK_NUM> rpo;
        build_rpo_block_order(rpo);
        for (u32 i = 0; i < rpo.size(); ++i) {
            os << std::format("  {}: {}\n", i, adaptor->block_fmt_ref(rpo[i]));
        }

        for (const auto &[cur, val1, val2]: data) {
            adaptor->block_set_info(cur, val1);
            adaptor->block_set_info2(cur, val2);
        }
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::print_block_layout(std::ostream &os) const {
        for (u32 i = 0; i < block_layout.size(); ++i) {
            os << std::format("  {}: {}\n", i, adaptor->block_fmt_ref(block_layout[i]));
        }
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::print_loops(std::ostream &os) const {
        for (u32 i = 0; i < loops.size(); ++i) {
            const auto &loop = loops[i];
            os << std::format("  {}: level {}, parent {}, {}->{}, irreducible: {}\n",
                              i,
                              loop.level,
                              loop.parent,
                              static_cast<u32>(loop.begin),
                              static_cast<u32>(loop.end),
                              loop.is_irreducible);
        }
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::print_liveness(std::ostream &os) const {
        for (u32 i = 0; i <= liveness_max_value; ++i) {
            if (liveness[i].epoch != liveness_epoch) {
                os << std::format("  {}: ignored\n", i);
                continue;
            }

            const auto &info = liveness[i];
            os << std::format("  {}: {} refs, {}->{} ({}->{}), lf: {}\n",
                              i,
                              info.ref_count,
                              static_cast<u32>(info.first),
                              static_cast<u32>(info.last),
                              adaptor->block_fmt_ref(block_ref(info.first)),
                              adaptor->block_fmt_ref(block_ref(info.last)),
                              info.last_full);
        }
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::print_precise_liveness(
        std::ostream &os) {
        const u32 num_blocks = static_cast<u32>(block_layout.size());
        util::SmallVector<u32, 64> block_lengths;
        block_lengths.resize(num_blocks);
        for (u32 block_idx = 0; block_idx < num_blocks; ++block_idx) {
            os << std::format("  Block {} ({}):\n",
                              block_idx,
                              adaptor->block_fmt_ref(block_layout[block_idx]));

            if (block_idx >= precise_liveness.size()) {
                os << "    <no precise liveness>\n";
                continue;
            }
            auto &pli = precise_liveness[block_idx];


            util::SmallVector<ValLocalIdx, SMALL_VALUE_NUM> values;
            for (auto &entry: pli.next_uses) {
                values.push_back(entry.first);
            }
            for (auto &[val_idx, _]: pli.live_through) {
                values.push_back(val_idx);
            }
            if (values.empty()) {
                os << "    no tracked values\n";
                continue;
            }
            std::sort(values.begin(),
                      values.end(),
                      [](const ValLocalIdx lhs, const ValLocalIdx rhs) {
                          return static_cast<u32>(lhs) < static_cast<u32>(rhs);
                      });

            for (auto val_idx: values) {
                auto it = pli.next_uses.find(val_idx);
                if (it == pli.next_uses.end()) {
                    const auto val = std::lower_bound(pli.live_through.begin(), pli.live_through.end(), val_idx,
                                                      [](const auto &lhs, const auto &rhs) {
                                                          return lhs.first < rhs;
                                                      });
                    if (val == pli.live_through.end() || val->first != val_idx) {
                        continue;
                    }
                    precise_liveness[block_idx].next_uses[val_idx].push_back(val->second);
                    it = pli.next_uses.find(val_idx);
                }
                auto &uses = it->second;

                os << std::format("    val {}: [", static_cast<u32>(val_idx));
                for (u32 i = 0; i < uses.size(); ++i) {
                    if (i != 0) {
                        os << ", ";
                    }
                    const auto dist = uses[i];
                    if (dist == std::numeric_limits<u32>::max()) {
                        os << "inf";
                    } else if ((dist & DEF_BIT) != 0) {
                        os << std::format("def@{}", dist & ~DEF_BIT);
                    } else {
                        os << dist;
                    }
                }
                os << "]\n";
            }
        }
    }


    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::collect_value_refs(
        std::unordered_map<ValLocalIdx, IRValueRef> &value_refs) const {
        value_refs.clear();
        value_refs.reserve(liveness_max_value + 1);

        const auto remember_value = [&](const IRValueRef value) {
            if (adaptor->val_ignore_in_liveness_analysis(value)) {
                return;
            }
            const ValLocalIdx val_idx = adaptor->val_local_idx(value);
            if (static_cast<u32>(val_idx) <= liveness_max_value) {
                value_refs.try_emplace(val_idx, value);
            }
        };

        for (const IRValueRef arg: adaptor->cur_args()) {
            remember_value(arg);
        }
        for (const IRBlockRef block: block_layout) {
            for (const IRValueRef phi: adaptor->block_phis(block)) {
                remember_value(phi);
            }
            for (const IRInstRef inst: adaptor->block_insts(block)) {
                for (const IRValueRef result: adaptor->inst_results(inst)) {
                    remember_value(result);
                }
                for (const IRValueRef operand: adaptor->inst_operands(inst)) {
                    remember_value(operand);
                }
            }
        }
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::print_phi_webs(std::ostream &os) const {
        std::unordered_map<ValLocalIdx, IRValueRef> value_refs;
        collect_value_refs(value_refs);


        for (u32 web_idx = 0; web_idx < web_members.size(); ++web_idx) {
            auto &members = web_members[web_idx];

            if (members.size() < 2) {
                continue;
            }


            os << std::format("  web {}: {{ ", web_idx);
            for (u32 i = 0; i < members.size(); ++i) {
                if (i != 0) {
                    os << ", ";
                }

                const ValLocalIdx val_idx = members[i];
                std::string_view value_name = "<unknown>";
                if (const auto it = value_refs.find(val_idx); it != value_refs.end()) {
                    value_name = adaptor->value_fmt_ref(it->second);
                }

                os << std::format("({}, {})", static_cast<u32>(val_idx), value_name);
            }
            os << " }\n";
        }
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    u32 Analyzer<Adaptor, CompilerType>::find_web_idx(const u32 idx) {
        const u32 parent = web_parent[idx];
        if (parent == idx) {
            return idx;
        }
        const u32 root = find_web_idx(parent);
        web_parent[idx] = root;

        return root;
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    typename Analyzer<Adaptor, CompilerType>::LivenessInfo &
    Analyzer<Adaptor, CompilerType>::liveness_maybe(const IRValueRef val) noexcept {
        const ValLocalIdx val_idx = adaptor->val_local_idx(val);
        if constexpr (Adaptor::TPDE_PROVIDES_HIGHEST_VAL_IDX) {
            assert(liveness.size() > static_cast<u32>(val_idx));
            return liveness[static_cast<u32>(val_idx)];
        } else {
            if (liveness_max_value <= static_cast<u32>(val_idx)) {
                liveness_max_value = static_cast<u32>(val_idx);
                if (liveness.size() <= liveness_max_value) {
                    // TODO: better growth strategy?
                    liveness.resize(liveness_max_value + 0x100);
                }
            }
            return liveness[static_cast<u32>(val_idx)];
        }
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::build_block_layout() {
        util::SmallVector<IRBlockRef, SMALL_BLOCK_NUM> block_rpo{};
        build_rpo_block_order(block_rpo);

        util::SmallVector<u32, SMALL_BLOCK_NUM> loop_parent{};
        util::SmallBitSet<256> loop_heads{};

        // TODO(ts): print out this step?
        identify_loops(block_rpo, loop_parent, loop_heads);
        assert(loop_parent.size() == block_rpo.size());
        // the entry block is always the loop head for the root loop
        loop_heads.mark_set(0);

        build_loop_tree_and_block_layout(block_rpo, loop_parent, loop_heads);
        dominator_tree.compute(adaptor, block_layout);
        //dominator_tree.print(std::cerr, adaptor, block_layout);
        assert(loop_parent.size() == block_rpo.size());
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::build_loop_tree_and_block_layout(
        const util::SmallVector<IRBlockRef, SMALL_BLOCK_NUM> &block_rpo,
        const util::SmallVector<u32, SMALL_BLOCK_NUM> &loop_parent,
        const util::SmallBitSet<256> &loop_heads) {
        // TODO(ts): maybe merge this into the block_rpo?
        struct BlockLoopInfo {
            u32 loop_idx;
            u32 rpo_idx;
        };

        util::SmallVector<BlockLoopInfo, SMALL_BLOCK_NUM> loop_blocks;
        loop_blocks.resize_uninitialized(block_rpo.size());
        for (u32 i = 0; i < block_rpo.size(); ++i) {
            loop_blocks[i] = BlockLoopInfo{~0u, i};
        }

        // if we have not seen the parent loop before, we need to recursively insert
        // them.
        // the recursive call only happens with irreducible control-flow
        const auto build_or_get_parent_loop = [&](const u32 i,
                                                  const auto &self) -> u32 {
            const auto parent = loop_parent[i];
            if (loop_blocks[parent].loop_idx != ~0u) {
                // we have already seen that block and given it a loop
                return loop_blocks[parent].loop_idx;
            } else {
                // get the parent loop and build the loop for this block
                const auto parent_loop_idx = self(parent, self);
                const auto loop_idx = loops.size();
                loops.push_back(Loop{
                    .level = loops[parent_loop_idx].level + 1,
                    .parent = parent_loop_idx
                });
                loop_blocks[parent].loop_idx = loop_idx;
                return loop_idx;
            }
        };

        // entry is always the head of the top-level loop
        loops.clear();
        loops.push_back(Loop{.level = 0, .parent = 0, .num_blocks = 1});
        loop_blocks[0].loop_idx = 0;

        for (u32 i = 1; i < loop_parent.size(); ++i) {
            const u32 parent_loop =
                    build_or_get_parent_loop(i, build_or_get_parent_loop);

            if (loop_heads.is_set(i)) {
                // if the loop is irreducible, we might have already inserted it so
                // check for that.
                //
                // NOTE: we could also get away with unsetting loop_heads for that
                // loop if it is irreducible if we would not count the loop_head in
                // its own loop's num_blocks but in its parent
                auto loop_idx = loop_blocks[i].loop_idx;
                if (loop_idx == ~0u) [[likely]] {
                    loop_idx = loops.size();
                    loops.push_back(
                        Loop{.level = loops[parent_loop].level + 1, .parent = parent_loop});
                    loop_blocks[i].loop_idx = loop_idx;
                } else {
                    loops[loop_idx].is_irreducible = true;
                }
                ++loops[loop_idx].num_blocks;
            } else {
                loop_blocks[i].loop_idx = parent_loop;
                ++loops[parent_loop].num_blocks;
            }
        }

        // acummulate the total number of blocks in a loop by iterating over them in
        // reverse-order. this works since we always push the parents first
        // leave out the first loop since it's its own parent
        for (u32 i = loops.size() - 1; i > 0; --i) {
            const auto &loop = loops[i];
            loops[loop.parent].num_blocks += loop.num_blocks;
        }

        assert(loops[0].num_blocks == block_rpo.size());

        // now layout the blocks by iterating in RPO and either place them at the
        // current offset of the parent loop or, if they are a new loop, place the
        // whole loop at the offset of the parent. this will ensure that blocks
        // surrounded by loops will also be layouted that way in the final order.
        //
        // however, the way this is implemented causes the loop index to not
        // correspond 1:1 with the final layout, though they will still be tightly
        // packed and only the order inside a loop may change. note(ts): this could
        // be mitigated with another pass i think.
        //
        // NB: we don't clear block_layout/block_loop_map, all entries are overwritten
        block_layout.resize_uninitialized(block_rpo.size());
        // TODO(ts): merge this with block_layout for less malloc calls?
        // however, we will mostly need this in the liveness computation so it may
        // be better for cache utilization to keep them separate
        block_loop_map.resize_uninitialized(block_rpo.size());

        loops[0].begin = loops[0].end = static_cast<BlockIndex>(0);

        const auto layout_loop = [&](const u32 loop_idx, const auto &self) -> void {
            assert(loops[loop_idx].begin == INVALID_BLOCK_IDX);
            const auto parent = loops[loop_idx].parent;
            if (loops[parent].begin == INVALID_BLOCK_IDX) {
                // should only happen with irreducible control-flow
                self(parent, self);
            }

            const auto loop_begin = loops[parent].end;
            loops[parent].end = static_cast<BlockIndex>(static_cast<u32>(loop_begin) +
                                                        loops[loop_idx].num_blocks);
            assert(static_cast<u32>(loops[parent].end) -
                static_cast<u32>(loops[parent].begin) <=
                loops[parent].num_blocks);

            loops[loop_idx].begin = loops[loop_idx].end = loop_begin;
        };

        for (u32 i = 0u; i < block_rpo.size(); ++i) {
            const auto loop_idx = loop_blocks[i].loop_idx;
            if (loops[loop_idx].begin == INVALID_BLOCK_IDX) {
                layout_loop(loop_idx, layout_loop);
            }

            const auto block_ref = block_rpo[loop_blocks[i].rpo_idx];
            const auto block_idx = static_cast<u32>(loops[loop_idx].end);
            loops[loop_idx].end = static_cast<BlockIndex>(block_idx + 1);

            block_layout[block_idx] = block_ref;
            block_loop_map[block_idx] = loop_idx;
            adaptor->block_set_info(block_ref, block_idx);
        }

        assert(static_cast<u32>(loops[0].end) == block_rpo.size());
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::build_rpo_block_order(
        util::SmallVector<IRBlockRef, SMALL_BLOCK_NUM> &out) const noexcept {
        out.clear();

        u32 num_blocks = 0;
        {
            // Initialize the block info
            u32 idx = 0;
            for (IRBlockRef cur: adaptor->cur_blocks()) {
                adaptor->block_set_info(cur, idx);
                adaptor->block_set_info2(cur, 0);
                ++idx;
            }
            num_blocks = idx;
        }
        out.resize_uninitialized(num_blocks);

        // implement the RPO generation using a simple stack that also walks in
        // post-order and then reverse at the end. However, consider the following
        // CFG
        //
        // A:
        //  - B:
        //    - D
        //    - E
        //  - C:
        //    - F
        //    - G
        //
        // which has valid RPOs A B D E C F G, A B D E C G F, A B E D C F G, ...
        // which is not very nice for loops since in many IRs the order of the block
        // has some meaning for the layout so we sort the pushed children by the
        // order in the block list
        util::SmallVector<IRBlockRef, SMALL_BLOCK_NUM / 2> stack;
        stack.push_back(adaptor->cur_entry_block());

        // NOTE(ts): because we process children in reverse order
        // this gives a bit funky results with irreducible loops
        // but it should be fine I think
        auto rpo_idx = num_blocks - 1;
        while (!stack.empty()) {
            const auto cur_node = stack.back();

            // have we already added the block to the RPO list?
            // if so we just skip it
            if (adaptor->block_info2(cur_node) & 0b1000) {
                stack.pop_back();
                continue;
            }

            // have we already pushed the children of the nodes to the stack and
            // processed them? if so we can add the block to the post-order,
            // otherwise add the children and wait for them to be processed
            if (adaptor->block_info2(cur_node) & 0b100) {
                stack.pop_back();
                // set the blocks RPO index and push it to the RPO list
                adaptor->block_set_info(cur_node, rpo_idx);
                // mark the block as already being on the RPO list
                adaptor->block_set_info2(cur_node,
                                         adaptor->block_info2(cur_node) | 0b1000);
                out[rpo_idx] = cur_node;
                --rpo_idx;
                continue;
            }

            // mark as visited and add the successors
            adaptor->block_set_info2(cur_node, adaptor->block_info2(cur_node) | 0b100);

            const auto start_idx = stack.size();
            // push the successors onto the stack to visit them
            for (const auto succ: adaptor->block_succs(cur_node)) {
                assert(succ != adaptor->cur_entry_block());
                const u32 info = adaptor->block_info2(succ);
                if ((info & 0b11) != 0) {
                    if ((info & 0b11) == 1) {
                        // note that succ has more than one incoming edge
                        adaptor->block_set_info2(succ, (info & ~0b11) | 2);
                    }
                } else {
                    // TODO(ts): if we just do a post-order traversal and reverse
                    // everything at the end we could use only block_info to check
                    // whether we already saw a block. And I forgot why?
                    adaptor->block_set_info2(succ, info | 1);
                }

                // if the successor is already on the rpo list or it has been
                // already visited and children added
                if (adaptor->block_info2(succ) & 0b1100) {
                    continue;
                }

                stack.push_back(succ);
            }

            // Order the pushed children by their original block
            // index since the children get visited in reverse and then inserted
            // in reverse order in the rpo list @_@
            const auto len = stack.size() - start_idx;
            if (len <= 1) {
                continue;
            }

            if (len == 2) {
                if (adaptor->block_info(stack[start_idx]) >
                    adaptor->block_info(stack[start_idx + 1])) {
                    std::swap(stack[start_idx], stack[start_idx + 1]);
                }
                continue;
            }

            std::sort(stack.begin() + start_idx,
                      stack.end(),
                      [this](const IRBlockRef lhs, const IRBlockRef rhs) {
                          // note(ts): this may have not so nice performance
                          // characteristics if the block lookup is a hashmap so
                          // maybe cache this for larger lists?
                          return adaptor->block_info(lhs) < adaptor->block_info(rhs);
                      });
        }

        if (rpo_idx != 0xFFFF'FFFF) {
            // there are unreachable blocks
            // so we did not fill up the whole array and need to shift it
            // TODO(ts): benchmark this against filling up a vector and always
            // reversing it tho it should be better i think
            out.erase(out.begin(), out.begin() + 1 + rpo_idx);

            // need to fixup the RPO index for blocks as well :/
            for (auto i = 0u; i < out.size(); ++i) {
                adaptor->block_set_info(out[i], i);
            }

#ifndef NDEBUG
            // In debug builds, reset block index of unreachable blocks.
            for (IRBlockRef cur: adaptor->cur_blocks()) {
                if (adaptor->block_info2(cur) == 0) {
                    adaptor->block_set_info(cur, 0xFFFF'FFFF);
                }
            }
#endif
        }

#ifdef TPDE_LOGGING
        TPDE_LOG_TRACE("Finished building RPO for blocks:");
        for (u32 i = 0; i < out.size(); ++i) {
            TPDE_LOG_TRACE("Index {}: {}", i, adaptor->block_fmt_ref(out[i]));
        }
#endif
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::identify_loops(
        const util::SmallVector<IRBlockRef, SMALL_BLOCK_NUM> &block_rpo,
        util::SmallVector<u32, SMALL_BLOCK_NUM> &loop_parent,
        util::SmallBitSet<256> &loop_heads) const noexcept {
        loop_parent.clear();
        loop_parent.resize(block_rpo.size());
        loop_heads.clear();
        loop_heads.resize(block_rpo.size());

        // Implement the modified algorithm from Wei et al.: A New Algorithm for
        // Identifying Loops in Decompilation
        // in a non-recursive form

        // TODO(ts): ask the adaptor if it can store this information for us?
        // then we could save the allocation here
        struct BlockInfo {
            bool traversed;
            bool self_loop;
            u32 dfsp_pos;
            u32 iloop_header;
        };

        util::SmallVector<BlockInfo, SMALL_BLOCK_NUM> block_infos;
        block_infos.resize(block_rpo.size());

        struct TravState {
            u32 block_idx; // b0 from the paper
            u32 dfsp_pos;
            u32 nh;
            u8 state = 0;
            decltype(adaptor->block_succs(adaptor->cur_entry_block()).begin()) succ_it;
            decltype(adaptor->block_succs(adaptor->cur_entry_block()).end()) end_it;
        };

        // The algorithm will reach the depth of the CFG so the small vector needs
        // to be relatively big
        // TODO(ts); maybe use the recursive form for small CFGs since that may be
        // faster or use stack switching since the non-recursive version is really
        // ugly
        util::SmallVector<TravState, SMALL_BLOCK_NUM> trav_state;

        trav_state.push_back(
            TravState{
                .block_idx = 0,
                .dfsp_pos = 1,
                .succ_it = adaptor->block_succs(block_rpo[0]).begin(),
                .end_it = adaptor->block_succs(block_rpo[0]).end()
            });

        const auto tag_lhead = [&block_infos](u32 b, u32 h) {
            if (b == h || h == 0) {
                return;
            }

            auto cur1 = b, cur2 = h;
            while (block_infos[cur1].iloop_header != 0) {
                const auto ih = block_infos[cur1].iloop_header;
                if (ih == cur2) {
                    return;
                }
                if (block_infos[ih].dfsp_pos < block_infos[cur2].dfsp_pos) {
                    block_infos[cur1].iloop_header = cur2;
                    cur1 = cur2;
                    cur2 = ih;
                } else {
                    cur1 = ih;
                }
            }
            block_infos[cur1].iloop_header = cur2;
        };

        // TODO(ts): this can be optimized and more condensed so that the state
        // variable becomes unnecessary
        while (!trav_state.empty()) {
            auto &state = trav_state.back();
            const auto block_idx = state.block_idx;
            switch (state.state) {
                case 0: {
                    // entry
                    block_infos[block_idx].traversed = true;
                    block_infos[block_idx].dfsp_pos = state.dfsp_pos;

                    // TODO(ts): somehow make sure that the iterators can live when
                    // succs is destroyed?
                    // auto succs    = adaptor->block_succs(block_rpo[block_idx]);
                    // state.succ_it = succs.begin();
                    // state.end_it  = succs.end();
                    state.state = 1;
                }
                    [[fallthrough]];
                case 1: {
                loop_inner:
                    auto cont = false;
                    while (state.succ_it != state.end_it) {
                        const auto succ_idx = adaptor->block_info(*state.succ_it);
                        if (succ_idx == block_idx) {
                            block_infos[block_idx].self_loop = true;
                        }

                        if (block_infos[succ_idx].traversed) {
                            if (block_infos[succ_idx].dfsp_pos > 0) {
                                tag_lhead(block_idx, succ_idx);
                            } else if (block_infos[succ_idx].iloop_header != 0) {
                                auto h_idx = block_infos[succ_idx].iloop_header;
                                if (block_infos[h_idx].dfsp_pos > 0) {
                                    tag_lhead(block_idx, h_idx);
                                } else {
                                    while (block_infos[h_idx].iloop_header != 0) {
                                        h_idx = block_infos[h_idx].iloop_header;
                                        if (block_infos[h_idx].dfsp_pos > 0) {
                                            tag_lhead(block_idx, h_idx);
                                            break;
                                        }
                                    }
                                }
                            }
                            ++state.succ_it;
                            continue;
                        }

                        // recurse
                        cont = true;
                        state.state = 2;
                        // TODO(ts): somehow make sure that the iterators can live when
                        // succs is destroyed?
                        auto succs = adaptor->block_succs(block_rpo[succ_idx]);
                        trav_state.push_back(TravState{
                            .block_idx = succ_idx,
                            .dfsp_pos = state.dfsp_pos + 1,
                            .succ_it = succs.begin(),
                            .end_it = succs.end()
                        });
                        break;
                    }

                    if (cont) {
                        continue;
                    }

                    block_infos[block_idx].dfsp_pos = 0;
                    trav_state.pop_back();
                    if (trav_state.empty()) {
                        break;
                    }
                    trav_state.back().nh = block_infos[block_idx].iloop_header;
                    continue;
                }
                case 2: {
                    const auto nh = state.nh;
                    tag_lhead(block_idx, nh);

                    ++state.succ_it;
                    state.state = 1;
                    goto loop_inner;
                }
            }
        }

        for (u32 i = 0; i < block_rpo.size(); ++i) {
            auto &info = block_infos[i];
            if (info.iloop_header != 0) {
                loop_parent[i] = info.iloop_header;
                loop_heads.mark_set(info.iloop_header);
            }
            if (info.self_loop) {
                loop_heads.mark_set(i);
            }
        }
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::compute_precise_liveness() noexcept {
        // todo(salto): irreducible loops?
        TPDE_LOG_TRACE("Starting Precise Liveness Analysis");
        const u32 num_blocks = static_cast<u32>(block_layout.size());
        // if the next use is "across" a loop, assign a penaltiy to encorouge
        // spilling this var before the loop.

        // todo(salto): think about epoch system like in og liveness analysis?
        precise_liveness.resize(num_blocks);
        for (auto &pli: precise_liveness) {
            pli.next_uses.clear();
            pli.live_through.clear();
        }
        // Ensure spilled_values is sized appropriately
        // todo(salto): fix
        spilled_values.resize(liveness_max_value + 1);
        spilled_values.zero();
        value_parts_cache.resize(liveness_max_value + 1);


        // Build DFS order that skips loop/back edges (successors already on
        // stack).
        util::SmallBitSet<SMALL_BLOCK_NUM> dfs_on_stack;
        dfs_on_stack.resize(num_blocks);
        util::SmallBitSet<SMALL_BLOCK_NUM> dfs_visited;
        dfs_visited.resize(num_blocks);
        util::SmallVector<u32, SMALL_BLOCK_NUM> postorder;
        postorder.reserve(num_blocks);
        util::SmallVector<util::SmallVector<u32, 4>, SMALL_BLOCK_NUM> non_loop_succs;
        non_loop_succs.resize(num_blocks);

        const auto dfs = [&](auto &&self, const u32 block_idx) -> void {
            dfs_on_stack.mark_set(block_idx); // on stack
            const IRBlockRef block = block_layout[block_idx];
            for (const IRBlockRef succ_ref: adaptor->block_succs(block)) {
                const u32 succ_idx = adaptor->block_info(succ_ref);
                if (dfs_on_stack.is_set(succ_idx)) {
                    // loop/back edge, ignore for this analysis
                    continue;
                }
                non_loop_succs[block_idx].push_back(succ_idx);
                if (!dfs_visited.is_set(succ_idx)) {
                    self(self, succ_idx);
                }
            }
            dfs_on_stack.mark_unset(block_idx);
            dfs_visited.mark_set(block_idx);
            postorder.push_back(block_idx);
        };

        dfs(dfs, 0);


        const auto is_phi_def = [this](u32 block_index, u32 firstUse) {
            // all phis are considered the only 0th instruction so firstUse ==
            // DEF_BIT + 0 means it's a phi def.
            return this->block_has_phis(static_cast<BlockIndex>(block_index)) &&
                   firstUse == DEF_BIT;
        };

        const auto is_live_in = [&](const u32 block_idx,
                                    const ValLocalIdx val_idx) noexcept {
            const auto &pli = precise_liveness[block_idx];
            const auto it = pli.next_uses.find(val_idx);
            if (it == pli.next_uses.end()) {
                // should be small
                for (const auto &entry: pli.live_through) {
                    if (entry.first == val_idx) {
                        return true;
                    }
                }
                return false;
            }
            const auto &vec = it->second;
            if (vec.empty()) {
                return false;
            }
            const auto first = vec.front();
            if (first == INF) {
                return false;
            }
            // values defined in the block are never live-in.
            return (first & DEF_BIT) == 0;
        };

        for (u32 order_idx = 0; order_idx < postorder.size(); ++order_idx) {
            const u32 block_idx = postorder[order_idx];
            const u32 cur_loop_idx = block_loop_map[block_idx];
            const IRBlockRef block = block_layout[block_idx];
            const bool has_phis = true;

            auto &pli = precise_liveness[block_idx];


            for (const IRValueRef phi: adaptor->block_phis(block)) {
                if (adaptor->val_ignore_in_liveness_analysis(phi)) {
                    continue;
                }
                pli.next_uses[adaptor->val_local_idx(phi)].push_back(DEF_BIT);
            }
            u32 inst_i = 0;
            for (const auto &inst: adaptor->block_insts(block)) {
                for (const IRValueRef operand: adaptor->inst_operands(inst)) {
                    if (adaptor->val_ignore_in_liveness_analysis(operand)) {
                        continue;
                    }
                    const auto val_idx = adaptor->val_local_idx(operand);
                    if (!pli.next_uses[val_idx].empty() &&
                        pli.next_uses[val_idx].back() ==
                        static_cast<u32>(inst_i) + has_phis)[[unlikely]] {
                        //todo check if this ever happens
                        continue;
                    }

                    pli.next_uses[val_idx].push_back(static_cast<u32>(inst_i) + has_phis);
                }

                // Register definitions with DEF_BIT.
                for (const IRValueRef res: adaptor->inst_results(inst)) {
                    if (adaptor->val_ignore_in_liveness_analysis(res)) {
                        continue;
                    }
                    pli.next_uses[adaptor->val_local_idx(res)].push_back(
                        DEF_BIT | (static_cast<u32>(inst_i) + has_phis));
                }
                ++inst_i;
            }
            assert(inst_i + has_phis < LOOP_EXIT_PENALTY &&
                "block is larger than the loop exit penalty, spilling will be "
                "suboptimal");
            const u32 block_span_with_phis = inst_i + has_phis;
            const auto is_loop_exit_edge = [&](const u32 succ_idx) -> bool {
                const u32 succ_loop_idx = block_loop_map[succ_idx];
                if (succ_loop_idx == cur_loop_idx) {
                    return false;
                }

                // different loop on the same level
                if (loops[succ_loop_idx].level == loops[cur_loop_idx].level) {
                    return true;
                }

                auto loop_it = cur_loop_idx;
                while (loop_it != 0u) {
                    loop_it = loops[loop_it].parent;
                    if (loop_it == succ_loop_idx) {
                        return true;
                    }
                }

                return false;
            };

            const auto add_phi_incoming = [&](const u32 succ_idx,
                                              const IRBlockRef succ_ref) {
                const u32 exit_penalty =
                        is_loop_exit_edge(succ_idx) ? LOOP_EXIT_PENALTY : 0u;

                for (const IRValueRef phi: adaptor->block_phis(succ_ref)) {
                    const auto phi_ref = adaptor->val_as_phi(phi);
                    const u32 slot_count = phi_ref.incoming_count();
                    for (u32 i = 0; i < slot_count; ++i) {
                        if (phi_ref.incoming_block_for_slot(i) != block) {
                            continue;
                        }
                        const IRValueRef incoming_val = phi_ref.incoming_val_for_slot(i);
                        if (adaptor->val_ignore_in_liveness_analysis(incoming_val)) {
                            continue;
                        }
                        const auto incoming_idx = adaptor->val_local_idx(incoming_val);
                        const auto dist = block_span_with_phis + exit_penalty;

                        if (!pli.next_uses.contains(incoming_idx)) {
                            pli.next_uses[incoming_idx].push_back(dist);
                            continue;
                        }


                        // choose the smallest possible distance from all successors
                        pli.next_uses[incoming_idx].back() =
                                std::min(pli.next_uses[incoming_idx].back(), dist);
                    }
                }
            };

            for (auto &[k, entries]: pli.next_uses) {
                entries.push_back(INF);
            }
            // Handle PHI definitions at position 0.
            // Account for PHI uses on all successor edges, including loop/back
            // edges.
            for (const IRBlockRef succ_ref: adaptor->block_succs(block)) {
                const u32 succ_idx = adaptor->block_info(succ_ref);
                add_phi_incoming(succ_idx, succ_ref);
            }

            for (const u32 succ_idx: non_loop_succs[block_idx]) {
                const u32 exit_penalty =
                        is_loop_exit_edge(succ_idx) ? LOOP_EXIT_PENALTY : 0u;

                assert(succ_idx < precise_liveness.size());

                const auto &succ_info = precise_liveness[succ_idx];
                for (const auto &entry: succ_info.next_uses) {
                    const ValLocalIdx val_idx = entry.first;

                    const auto &succ_vec = entry.second;
                    if (succ_vec.empty()) {
                        continue;
                    }

                    u32 succ_first = succ_vec[0];
                    if (succ_first == INF) {
                        continue;
                    }
                    if ((succ_first & DEF_BIT) != 0) {
                        // Definition in successor does not require the incoming value.
                        continue;
                    }
                    const u32 dist = block_span_with_phis + exit_penalty + succ_first;
                    if (!pli.next_uses.contains(val_idx)) {
                        pli.next_uses[val_idx].push_back(dist);
                        continue;
                    }


                    // choose the smallest possible distance from all successors
                    pli.next_uses[val_idx].back() =
                            std::min(pli.next_uses[val_idx].back(), dist);
                }
            }
        }

        // Propagate loop-carried liveness without fixed-point iteration.
        {
            util::SmallVector<util::SmallVector<u32, 8>, 16> loop_children;
            loop_children.resize(loops.size());
            for (u32 loop_idx = 1; loop_idx < loops.size(); ++loop_idx) {
                const auto parent = loops[loop_idx].parent;
                if (parent < loop_children.size()) {
                    loop_children[parent].push_back(loop_idx);
                }
            }

            util::SmallVector<util::SmallVector<u32, SMALL_BLOCK_NUM>, 16> loop_blocks;
            loop_blocks.resize(loops.size());
            for (u32 block_idx = 0; block_idx < num_blocks; ++block_idx) {
                const u32 loop_idx = block_loop_map[block_idx];
                loop_blocks[loop_idx].push_back(block_idx);
            }

            const auto ensure_live_in_out =
                    [&](const u32 block_idx,
                        const util::SmallVector<ValLocalIdx, 16> &live_vals) {
                if (live_vals.empty()) {
                    return;
                }

                auto &pli = precise_liveness[block_idx];
                // todo(salto): optimize
                const u32 block_span = std::ranges::distance(adaptor->block_insts(
                                           block_ref(BlockIndex{block_idx}))) +
                                       block_has_phis(BlockIndex{block_idx});

                for (const auto val_idx: live_vals) {
                    auto it = pli.next_uses.find(val_idx);
                    if (it == pli.next_uses.end()) {
                        pli.live_through.emplace_back(val_idx, block_span);
                        continue;
                    }
                    auto &vec = it->second;
                    // live-in and live-out, but not used in the block

                    auto &first = vec[0];
                    if (first == INF) {
                        first = 0;
                    }

                    auto &tail = vec.back();
                    if (tail == INF) {
                        tail = block_span;
                    } else if ((tail & DEF_BIT) != 0) {
                        vec.push_back(block_span);
                    } else if (tail < block_span) {
                        tail = block_span;
                    }
                }
            };

            const auto loop_tree_dfs = [&](const auto &self,
                                           const u32 loop_idx) -> void {
                const auto header_block_idx = static_cast<u32>(loops[loop_idx].begin);

                util::SmallVector<ValLocalIdx, 16> live_loop;

                const auto &header_pl = precise_liveness[header_block_idx];
                for (const auto phi_instr:
                     adaptor->block_phis(block_ref(loops[loop_idx].begin))) {
                    // todo(salto): check ignore liveness?
                    const auto phi_def = adaptor->val_local_idx(phi_instr);
                    const auto it = header_pl.next_uses.find(phi_def);
                    if (it == header_pl.next_uses.end()) {
                        continue;
                    }
                    const auto &vec = it->second;
                    if (!vec.empty() && vec.back() != INF) {
                        // live_loop.push_back(phi_def);
                    }
                }

                for (const auto &entry: header_pl.next_uses) {
                    const auto val_idx = entry.first;
                    if (!is_live_in(header_block_idx, val_idx)) {
                        continue;
                    }
                    /*if (is_phi_def(header_block_idx, entry.second[0])) {
            continue;
            }*/
                    live_loop.push_back(val_idx);
                }

                const IRBlockRef header_block = block_layout[header_block_idx];
                for (const IRValueRef phi: adaptor->block_phis(header_block)) {
                    const auto phi_ref = adaptor->val_as_phi(phi);
                    const u32 slot_count = phi_ref.incoming_count();
                    for (u32 i = 0; i < slot_count; ++i) {
                        const IRBlockRef incoming_block = phi_ref.incoming_block_for_slot(i);
                        const u32 incoming_idx = adaptor->block_info(incoming_block);
                        if (incoming_idx >= block_loop_map.size() || block_loop_map[incoming_idx] != loop_idx) {
                            continue;
                        }
                        const IRValueRef incoming_val = phi_ref.incoming_val_for_slot(i);
                        if (adaptor->val_ignore_in_liveness_analysis(incoming_val)) {
                            continue;
                        }
                        const auto val_idx = adaptor->val_local_idx(incoming_val);
                        live_loop.push_back(val_idx);
                    }
                }


                // Ensure loop-carried values are live through the header as well.
                std::sort(live_loop.begin(), live_loop.end());
                ensure_live_in_out(header_block_idx, live_loop);

                util::SmallVector<ValLocalIdx, 16> live_loop_no_header_phis;
                for (const auto val_idx: live_loop) {
                    auto it = precise_liveness[header_block_idx].next_uses.find(val_idx);
                    if (it == precise_liveness[header_block_idx].next_uses.end() || it->second[0] != (DEF_BIT | 0)) {
                        live_loop_no_header_phis.push_back(val_idx);
                    }
                }

                for (const auto child_block: loop_blocks[loop_idx]) {
                    if (child_block == header_block_idx) {
                        continue;
                    }
                    ensure_live_in_out(child_block, live_loop_no_header_phis);
                }

                for (const auto child_loop: loop_children[loop_idx]) {
                    const auto child_header = static_cast<u32>(loops[child_loop].begin);
                    ensure_live_in_out(child_header, live_loop_no_header_phis);
                    self(self, child_loop);
                }
            };

            for (const auto root_loop: loop_children[0]) {
                loop_tree_dfs(loop_tree_dfs, root_loop);
            }
            for (auto &entry: precise_liveness) {
                std::stable_sort(entry.live_through.begin(), entry.live_through.end(),
                                 [](const auto &lhs, const auto &rhs) {
                                     if (lhs.first == rhs.first) {
                                         return lhs.second < rhs.second;
                                     }
                                     return lhs.first < rhs.first;
                                 });
            }
        }

        // Calculate per-block register pressure using interval overlap
        block_pressure.resize(num_blocks);
        for (auto &bp: block_pressure) {
            bp = BlockPressure{};
        }

        for (u32 block_idx = 0; block_idx < num_blocks; ++block_idx) {
            const auto &pli = precise_liveness[block_idx];
            const IRBlockRef block = block_layout[block_idx];
            const bool has_phis = block_has_phis(block);

            util::SmallVector<std::pair<ValueInterval, ValLocalIdx>, 64> intervals;

            for (const auto &[val_idx, uses]: pli.next_uses) {
                if (static_cast<u32>(val_idx) >= value_parts_cache.size()) {
                    continue;
                }

                u32 first = INF;
                for (const auto use: uses) {
                    if (use != INF) {
                        first = use;
                        break;
                    }
                }

                if (first == INF) {
                    continue;
                }

                u32 interval_first;
                if (first & DEF_BIT) {
                    interval_first = INF;
                    for (const auto use: uses) {
                        if (use != INF && (use & DEF_BIT) == 0) {
                            interval_first = use;
                            break;
                        }
                    }
                } else {
                    interval_first = 0;
                }

                if (interval_first == INF) {
                    continue;
                }

                u32 interval_last = 0;
                for (u32 i = uses.size(); i-- > 0;) {
                    if (uses[i] != INF) {
                        interval_last = uses[i];
                        break;
                    }
                }

                intervals.emplace_back(
                    ValueInterval{.first = interval_first, .last = interval_last},
                    val_idx);
            }
            for (const auto &[val_idx, live_out]: pli.live_through) {
                intervals.emplace_back(
                    ValueInterval{.first = 0, .last = live_out},
                    val_idx);
            }

            if (intervals.empty()) {
                continue;
            }

            const u32 block_span =
                    std::ranges::distance(adaptor->block_insts(block)) + (has_phis ? 1 : 0);

            struct Event {
                u32 pos;
                i32 delta;
                u32 gp_parts;
                u32 fp_parts;
            };

            util::SmallVector<Event, 128> events;
            // todo salto optimzie this bullshit
            for (const auto &[interval, val_idx]: intervals) {
                const auto parts = ensure_parts_cached(val_idx);
                const u32 gp_parts = parts.gp_regs;
                const u32 fp_parts = parts.fp_regs;

                events.push_back(Event{
                    .pos = interval.first,
                    .delta = 1,
                    .gp_parts = gp_parts,
                    .fp_parts = fp_parts
                });
                events.push_back(Event{
                    .pos = interval.last + 1,
                    .delta = -1,
                    .gp_parts = gp_parts,
                    .fp_parts = fp_parts
                });
            }

            std::sort(events.begin(), events.end(), [](const Event &a, const Event &b) {
                return a.pos < b.pos;
            });

            u32 gp_pressure = 0;
            u32 fp_pressure = 0;
            u32 max_gp = 0;
            u32 max_fp = 0;

            u32 event_idx = 0;
            for (u32 pos = 0; pos <= block_span; ++pos) {
                while (event_idx < events.size() && events[event_idx].pos == pos) {
                    gp_pressure += events[event_idx].delta * events[event_idx].gp_parts;
                    fp_pressure += events[event_idx].delta * events[event_idx].fp_parts;
                    ++event_idx;
                }

                if (pos < block_span) {
                    max_gp = std::max(max_gp, gp_pressure);
                    max_fp = std::max(max_fp, fp_pressure);
                }
            }

            block_pressure[block_idx].gp_pressure = max_gp;
            block_pressure[block_idx].fp_pressure = max_fp;
        }
        /*
            uint64_t total_values = 0;
            uint64_t total_values_liveness = 0;
            for (auto i = 0; i < num_blocks; ++i) {
                std::printf("[");
                for (const auto &entry: precise_liveness[i].next_uses) {
                    std::printf("%d,", entry.second.size() - 1);
                }
                std::printf("],\n");
            }*/

        TPDE_LOG_TRACE("Precise Liveness Analysis completed");
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    std::pair<u32, u32> Analyzer<Adaptor, CompilerType>::get_current_and_next_use(
        const PreciseLivenessInfo &pli, const ValLocalIdx val_idx, const u32 instr_idx) {
        // calculate current (before idx) and next use (after execution of the
        // current instruction). operands that die with the instruction would be
        // [idx, INF].
        const auto idx = instr_idx + 1; // Phis are the zeroth instruction

        // Find the next_uses vector for this val_idx
        const auto it = pli.next_uses.find(val_idx);
        if (it == pli.next_uses.end()) {
            auto entr = std::lower_bound(pli.live_through.begin(), pli.live_through.end(), val_idx,
                                         [](const auto &lhs, const auto &rhs) {
                                             return lhs.first < rhs;
                                         });
            if (entr == pli.live_through.end() || entr->first != val_idx) {
                // Value not found in liveness info, return INF (no uses)
                return {INF, INF};
            }

            return {entr->second, entr->second};
        }

        const auto &vec = it->second;

#ifndef NDEBUG
        // Verify the vector is sorted (ignoring DEF_BIT) in debug builds
        for (u32 i = 1; i < vec.size(); ++i) {
            const u32 prev_val = vec[i - 1] & ~DEF_BIT;
            const u32 curr_val = vec[i] & ~DEF_BIT;
            assert((prev_val <= curr_val || curr_val == (INF & ~DEF_BIT)) &&
                "get_current_and_next_use: vector is not sorted");
        }
#endif

        // results can't be spilled before they are defined, so we must avoid
        // spilling them, therefore return 0.
        if (vec[0] == (idx | DEF_BIT)) {
            assert(vec.size() >= 2);
            return {0u, vec[1]};
        }
        if (vec[0] == INF) {
            return {INF, INF};
        }
        if ((!(vec[0] & DEF_BIT) && vec[0] > idx)) {
            return {vec[0], vec[0]};
        } else if (!(vec[0] & DEF_BIT) && vec[0] == idx) {
            assert(vec.size() >= 2);
            return {vec[0], vec[1]};
        }

        // Binary search for the first element >= idx (ignoring DEF_BIT)
        // We start from index 1 since we already checked vec[0] above
        const auto vec_it = std::lower_bound(
            vec.begin() + 1, vec.end(), idx, [](const u32 dist, const u32 target) {
                // Compare distances, masking off the DEF_BIT
                return (dist & ~DEF_BIT) < target;
            });

        if (vec_it == vec.end()) {
            return {INF, INF};
        }

        const u32 dist = *vec_it;
        if (dist == INF) {
            return {INF, INF};
        }
        // vec[i+1] must exist for the live-out entry
        const u32 next_idx = std::distance(vec.begin(), vec_it) + 1;
        if (next_idx >= vec.size()) {
            return {dist, INF};
        }
        return {dist, dist == idx ? vec[next_idx] : dist};
    };

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::compute_spills() noexcept {
        // Based on "Register Spilling and Live-Range Splitting for
        // SSA-Form Programs" by Hack et al. 2008
        // Simplified since we don't store reload or spill positions.
        TPDE_LOG_TRACE("Starting Spill Analysis");

        // todo(salto): realistically basically no values will have parts from  2
        // different banks.
        //  so optimize for this case.


        // Build simple PHI coalescing webs using union-find.
        // We only merge single-part PHIs with single-part incoming values that are
        // compatible and non-overlapping.

        util::SmallVector<u8, SMALL_VALUE_NUM> web_rank;
        util::SmallVector<util::SmallVector<u32, 2>, SMALL_VALUE_NUM> web_phi_blocks;

        util::SmallVector<BlockIndex, SMALL_VALUE_NUM> val_def_blocks;
        web_parent.resize(liveness_max_value + 1);
        web_rank.resize(liveness_max_value + 1, 0u);
        web_phi_blocks.resize(liveness_max_value + 1);
        web_members.resize(liveness_max_value + 1);
        val_def_blocks.resize(liveness_max_value + 1, INVALID_BLOCK_IDX);
        callee_saved_values.clear();

        for (u32 i = 0; i <= liveness_max_value; ++i) {
            web_members[i].clear();
            if (i < liveness.size() && liveness[i].epoch == liveness_epoch) {
                web_parent[i] = i;
                web_members[i].push_back(static_cast<ValLocalIdx>(i));
            } else {
                web_parent[i] = INVALID_WEB_IDX;
            }
        }

        if (!block_layout.empty()) {
            const BlockIndex entry_block_idx = BlockIndex{0};
            for (const IRValueRef arg: adaptor->cur_args()) {
                if (adaptor->val_ignore_in_liveness_analysis(arg)) {
                    continue;
                }

                const ValLocalIdx arg_idx = adaptor->val_local_idx(arg);
                const u32 arg_idx_u32 = static_cast<u32>(arg_idx);
                if (arg_idx_u32 >= val_def_blocks.size() ||
                    web_parent[arg_idx_u32] == INVALID_WEB_IDX) {
                    continue;
                }
                val_def_blocks[arg_idx_u32] = entry_block_idx;
            }
        }


        for (u32 block_idx_u32 = 0; block_idx_u32 < block_layout.size(); ++block_idx_u32) {
            const IRBlockRef block = block_layout[block_idx_u32];
            for (const IRValueRef phi: adaptor->block_phis(block)) {
                if (adaptor->val_ignore_in_liveness_analysis(phi)) {
                    continue;
                }

                const u32 phi_idx_u32 = static_cast<u32>(adaptor->val_local_idx(phi));
                if (phi_idx_u32 >= web_phi_blocks.size() ||
                    web_parent[phi_idx_u32] == INVALID_WEB_IDX) {
                    continue;
                }

                val_def_blocks[phi_idx_u32] = BlockIndex{block_idx_u32};
                web_phi_blocks[phi_idx_u32].push_back(block_idx_u32);
            }

            for (const IRInstRef inst: adaptor->block_insts(block)) {
                for (const IRValueRef result: adaptor->inst_results(inst)) {
                    if (adaptor->val_ignore_in_liveness_analysis(result)) {
                        continue;
                    }

                    const ValLocalIdx result_idx = adaptor->val_local_idx(result);
                    const u32 result_idx_u32 = static_cast<u32>(result_idx);
                    if (result_idx_u32 >= val_def_blocks.size() ||
                        web_parent[result_idx_u32] == INVALID_WEB_IDX) {
                        continue;
                    }

                    val_def_blocks[result_idx_u32] = BlockIndex{block_idx_u32};
                }
            }
        }

        const auto roots_share_phi_def_block =
                [&](const u32 lhs_root, const u32 rhs_root) -> bool {
            const auto &lhs_blocks = web_phi_blocks[lhs_root];
            const auto &rhs_blocks = web_phi_blocks[rhs_root];
            for (const u32 rhs_block_idx: rhs_blocks) {
                if (std::ranges::find(lhs_blocks, rhs_block_idx) != lhs_blocks.end()) {
                    return true;
                }
            }
            return false;
        };

        const auto merge_phi_blocks = [&](const u32 dst_root, const u32 src_root) {
            auto &dst_blocks = web_phi_blocks[dst_root];
            auto &src_blocks = web_phi_blocks[src_root];
            for (const u32 src_block_idx: src_blocks) {
                if (std::ranges::find(dst_blocks, src_block_idx) == dst_blocks.end()) {
                    dst_blocks.push_back(src_block_idx);
                }
            }
            src_blocks.clear();
        };

        const auto merge_web_members = [&](const u32 dst_root, const u32 src_root) {
            auto &dst_members = web_members[dst_root];
            auto &src_members = web_members[src_root];
            for (const ValLocalIdx src_member: src_members) {
                if (std::ranges::find(dst_members, src_member) == dst_members.end()) {
                    dst_members.push_back(src_member);
                }
            }
            src_members.clear();
        };

        const auto is_live_in = [&](const u32 block_idx,
                                    const ValLocalIdx val_idx) noexcept {
            if (block_idx >= precise_liveness.size()) {
                return false;
            }

            const auto &pli = precise_liveness[block_idx];
            const auto it = pli.next_uses.find(val_idx);
            if (it == pli.next_uses.end()) {
                auto entr = std::lower_bound(pli.live_through.begin(), pli.live_through.end(), val_idx,
                                             [](const auto &lhs, const auto &rhs) {
                                                 return lhs.first < rhs;
                                             });
                if (entr == pli.live_through.end() || entr->first != val_idx) {
                    return false;
                }
                return true;
            }

            const auto &vec = it->second;
            if (vec.empty()) {
                return false;
            }

            for (const u32 entry: vec) {
                if (entry == INF) {
                    continue;
                }
                return (entry & DEF_BIT) == 0;
            }

            return false;
        };


        const auto precise_block_interval =
                [&](const u32 block_idx,
                    const ValLocalIdx val_idx,
                    u32 &interval_first,
                    u32 &interval_last) -> bool {
            if (block_idx >= precise_liveness.size()) {
                return false;
            }

            const auto &pli = precise_liveness[block_idx];
            const auto it = pli.next_uses.find(val_idx);
            if (it == pli.next_uses.end()) {
                return false;
            }

            const auto &uses = it->second;
            if (uses.empty()) {
                return false;
            }

            u32 first = INF;
            for (const auto use: uses) {
                if (use != INF) {
                    first = use;
                    break;
                }
            }
            if (first == INF) {
                return false;
            }

            if ((first & DEF_BIT) != 0) {
                interval_first = INF;
                for (const auto use: uses) {
                    if (use != INF && (use & DEF_BIT) == 0) {
                        interval_first = use;
                        break;
                    }
                }
                if (interval_first == INF) {
                    return true;
                }
            } else {
                // live-in values occupy the register from block entry
                interval_first = 0;
            }

            interval_last = INF;
            for (u32 i = uses.size(); i-- > 0;) {
                if (uses[i] != INF) {
                    interval_last = uses[i] & ~DEF_BIT;
                    break;
                }
            }

            return interval_last != INF;
        };

        const auto live_ranges_overlap =
                [&](const ValLocalIdx lhs_idx, const ValLocalIdx rhs_idx) -> bool {
            const u32 lhs = static_cast<u32>(lhs_idx);
            const u32 rhs = static_cast<u32>(rhs_idx);
            if (lhs >= liveness.size() || rhs >= liveness.size()) {
                return true;
            }

            const auto &lhs_liveness = liveness[lhs];
            const auto &rhs_liveness = liveness[rhs];
            if (lhs_liveness.epoch != liveness_epoch ||
                rhs_liveness.epoch != liveness_epoch) {
                return true;
            }

            // Fast coarse disjointness check on block intervals.
            if (lhs_liveness.last < rhs_liveness.first ||
                rhs_liveness.last < lhs_liveness.first) {
                return false;
            }

            const u32 block_begin = std::min(static_cast<u32>(lhs_liveness.first),
                                             static_cast<u32>(rhs_liveness.first));
            const u32 block_end = std::max(static_cast<u32>(lhs_liveness.last),
                                           static_cast<u32>(rhs_liveness.last));

            for (u32 block_idx = block_begin; block_idx <= block_end; ++block_idx) {
                u32 lhs_first = 0;
                u32 lhs_last = 0;
                u32 rhs_first = 0;
                u32 rhs_last = 0;
                if (!precise_block_interval(block_idx, lhs_idx, lhs_first, lhs_last)) {
                    continue;
                }
                if (!precise_block_interval(block_idx, rhs_idx, rhs_first, rhs_last)) {
                    continue;
                }
                return true;
            }

            return false;
        };
        const auto web_union = [&](const u32 lhs, const u32 rhs) {
            return;
            if (lhs == rhs || lhs == INVALID_WEB_IDX || rhs == INVALID_WEB_IDX) {
                return;
            }

            u32 lhs_root = find_web_idx(lhs);
            u32 rhs_root = find_web_idx(rhs);
            if (lhs_root == rhs_root) {
                return;
            }

            if (roots_share_phi_def_block(lhs_root, rhs_root)) {
                return;
            }

            const auto &lhs_members = web_members[lhs_root];
            const auto &rhs_members = web_members[rhs_root];
            // todo(salto): can be done in linear time see boissout
            for (const ValLocalIdx lhs_member: lhs_members) {
                const u32 lhs_member_u32 = static_cast<u32>(lhs_member);
                const BlockIndex lhs_def_block =
                        lhs_member_u32 < val_def_blocks.size()
                            ? val_def_blocks[lhs_member_u32]
                            : INVALID_BLOCK_IDX;
                for (const ValLocalIdx rhs_member: rhs_members) {
                    const u32 rhs_member_u32 = static_cast<u32>(rhs_member);
                    const BlockIndex rhs_def_block =
                            rhs_member_u32 < val_def_blocks.size()
                                ? val_def_blocks[rhs_member_u32]
                                : INVALID_BLOCK_IDX;
                    if (lhs_def_block != INVALID_BLOCK_IDX &&
                        is_live_in(static_cast<u32>(lhs_def_block), rhs_member)) {
                        return;
                    }
                    if (rhs_def_block != INVALID_BLOCK_IDX &&
                        is_live_in(static_cast<u32>(rhs_def_block), lhs_member)) {
                        return;
                    }
                }
            }

            if (web_rank[lhs_root] < web_rank[rhs_root]) {
                std::swap(lhs_root, rhs_root);
            }
            web_parent[rhs_root] = lhs_root;
            merge_phi_blocks(lhs_root, rhs_root);
            merge_web_members(lhs_root, rhs_root);
            if (web_rank[lhs_root] == web_rank[rhs_root]) {
                ++web_rank[lhs_root];
            }
        };

        for (const IRBlockRef block: block_layout) {
            for (const IRValueRef phi: adaptor->block_phis(block)) {
                if (adaptor->val_ignore_in_liveness_analysis(phi)) {
                    continue;
                }

                const auto phi_parts = adaptor->val_parts(phi);
                if (phi_parts.count() != 1) {
                    // ignore multipart phis
                    continue;
                }

                const ValLocalIdx phi_idx = adaptor->val_local_idx(phi);
                const u32 phi_idx_u32 = static_cast<u32>(phi_idx);
                if (phi_idx_u32 >= web_parent.size() ||
                    web_parent[phi_idx_u32] == INVALID_WEB_IDX) {
                    continue;
                }

                const u8 phi_bank_id = phi_parts.reg_bank(0).id();
                const u32 phi_part_size = phi_parts.size_bytes(0);

                const auto phi_ref = adaptor->val_as_phi(phi);
                const u32 slot_count = phi_ref.incoming_count();
                for (u32 slot = 0; slot < slot_count; ++slot) {
                    const IRBlockRef incoming_block = phi_ref.incoming_block_for_slot(slot);

                    const IRValueRef incoming_val = phi_ref.incoming_val_for_slot(slot);
                    if (adaptor->val_ignore_in_liveness_analysis(incoming_val)) {
                        continue;
                    }

                    const auto incoming_parts = adaptor->val_parts(incoming_val);
                    if (incoming_parts.count() != 1) {
                        continue;
                    }
                    if (incoming_parts.reg_bank(0).id() != phi_bank_id) {
                        continue;
                    }
                    if (incoming_parts.size_bytes(0) != phi_part_size) {
                        continue;
                    }

                    const ValLocalIdx incoming_idx = adaptor->val_local_idx(incoming_val);
                    const u32 incoming_idx_u32 = static_cast<u32>(incoming_idx);
                    if (incoming_idx_u32 >= web_parent.size() ||
                        web_parent[incoming_idx_u32] == INVALID_WEB_IDX) {
                        continue;
                    }


                    web_union(phi_idx_u32, incoming_idx_u32);
                }
            }
        }


        // #ifdef TPDE_LOGGING
        //     TPDE_LOG_ERR("PHI webs after build:");
        //     std::ostringstream phi_web_dump;
        //     print_phi_webs(phi_web_dump);
        //     std::istringstream phi_web_lines(phi_web_dump.str());
        //     for (std::string line; std::getline(phi_web_lines, line);) {
        //         TPDE_LOG_ERR("{}", line);
        //     }
        // #endif


        // todo(salto): multi-part values?
        // todo(salto): ordered set for W
        constexpr u32 NUM_GP_REGS = CompilerType::ConfigType::SPILL_NUM_GP_REGS;
        constexpr u32 NUM_FP_REGS = CompilerType::ConfigType::SPILL_NUM_FP_REGS;
        constexpr u32 NUM_CALLER_SAVED_GP =
                CompilerType::ConfigType::CALLER_SAVED_GP_REGS;
        constexpr u32 NUM_CALLER_SAVED_FP =
                CompilerType::ConfigType::CALLER_SAVED_FP_REGS;

        // The set of values in registers at the end of a block
        // compared to the original algorithm, we can avoid the set S (spilled
        // values), since all spills are after definition
        std::unordered_map<BlockIndex, std::unordered_set<ValLocalIdx> > W_exits;

        // todo(salto): arguments on the stack don't need to be added to W in
        // entry, but register arguments need to be in W at the start.


        // todo(salto): values with multiple regbanks
        // todo(salto): ignore liveness?

        // todo(salto): constants?
        // todo(salto): instruction fused? How to handle?
        // todo(salto): irregular control flow
        // todo(salto): handle values with >5 parts seperately?


        // todo(salto): register arguments
        WorkingSetTracker working_set(this);

        // block -> val_idx -> # of predecessor that have val_idx in W at their
        // end. todo(salto): better datastructure!
        util::SmallVector<std::unordered_map<ValLocalIdx, u32>, SMALL_BLOCK_NUM>
                W_entry_freq;
        W_entry_freq.resize(block_layout.size());
        util::SmallVector<u32, 16> W_max_entry_freq;
        W_max_entry_freq.resize(block_layout.size(), 0u);
        initial_working_set_by_block.clear();
        util::SmallBitSet<SMALL_BLOCK_NUM> loop_header_blocks;
        loop_header_blocks.resize(static_cast<u32>(block_layout.size()));
        loop_header_blocks.zero();
        for (u32 loop_idx = 1; loop_idx < loops.size(); ++loop_idx) {
            const auto header_idx = static_cast<u32>(loops[loop_idx].begin);
            if (header_idx < block_layout.size()) {
                loop_header_blocks.mark_set(header_idx);
            }
        }

        for (u32 i = 0; i < this->block_layout.size(); ++i) {
            // TODO(salto): Handle register arguments in entry block
            // For the entry block (i == 0), we should initialize W with arguments
            // that are passed in registers. This requires determining which
            // arguments get register assignments based on the calling convention.
            // Currently, argument assignments are determined later during prologue
            // generation, so we can't easily determine this here without
            // duplicating the CCAssigner logic.
            const auto block = this->block_layout[i];
            const BlockIndex block_idx_cur = block_idx(block);
            const u32 block_idx_u32 = static_cast<u32>(block_idx_cur);
            const bool is_loop_header_block = loop_header_blocks.is_set(block_idx_u32);
            if (Adaptor::TPDE_LIVENESS_VISIT_ARGS && i == 0) {
                assert(block_layout[0] == compiler->adaptor->cur_entry_block());
                // TODO: Query cc_assigner to determine which args are in registers
                // and add them to W with appropriate used_regs tracking
                auto *cc_assiger = compiler->cur_cc_assigner();
                const auto &cc_info = cc_assiger->get_ccinfo();
                const u64 arg_regs = cc_info.arg_regs;
                u64 free_regs = std::popcount(arg_regs);
                for (const IRValueRef arg: adaptor->cur_args()) {
                    // todo(salto): is this correct
                    auto local_idx = adaptor->val_local_idx(arg);
                    const RegParts parts = ensure_parts_cached(local_idx);
                    if (free_regs < parts.gp_regs + parts.fp_regs) {
                        // rest of args must be on stack
                        break;
                    }
                    // fixme(salto): multi-part, ignore_liveness?
                    working_set.insert(local_idx);
                    free_regs -= parts.gp_regs + parts.fp_regs;
                }
            } else {
                // We need to choose which values to keep in W across the multiple
                // incoming edges. prefer values that are used in many predecessors.
                util::SmallVector<ValLocalIdx, 16> incoming_from_all;
                const u32 max_seen_freq = W_max_entry_freq[block_idx_u32];
                for (const auto [val_idx, freq]: W_entry_freq[block_idx_u32]) {
                    if (freq != max_seen_freq) {
                        continue;
                    }
                    incoming_from_all.push_back(val_idx);
                }

                if (is_loop_header_block) {
                    util::SmallVector<ValLocalIdx, 16> loop_local_values;
                    std::unordered_set<ValLocalIdx> selected_values;
                    u32 loop_local_gp_regs = 0;
                    u32 loop_local_fp_regs = 0;


                    const u32 half_gp_regs = NUM_GP_REGS / 2;
                    const u32 half_fp_regs = NUM_FP_REGS / 2;
                    std::array<u8, 2> phi_parts = {0u, 0u};
                    std::unordered_set<ValLocalIdx> header_phi_defs;
                    for (const auto phi: adaptor->block_phis(block)) {
                        if (adaptor->val_ignore_in_liveness_analysis(phi)) {
                            continue;
                        }
                        header_phi_defs.insert(adaptor->val_local_idx(phi));
                        const RegParts parts = ensure_parts_cached(adaptor->val_local_idx(phi));
                        phi_parts[0] += parts.gp_regs;
                        phi_parts[1] += parts.fp_regs;
                    }
                    //if (phi_parts[0] > NUM_GP_REGS - 4) {
                    //    phi_parts[0] = half_gp_regs;
                    //}

                    std::unordered_set<ValLocalIdx> header_local_defs;
                    for (const auto inst: adaptor->block_insts(block)) {
                        for (const auto result: adaptor->inst_results(inst)) {
                            if (adaptor->val_ignore_in_liveness_analysis(result)) {
                                continue;
                            }
                            header_local_defs.insert(adaptor->val_local_idx(result));
                        }
                    }

                    struct HeaderCandidate {
                        ValLocalIdx val_idx;
                        u32 entry_next_use;
                        u32 num_parts;
                    };

                    util::SmallVector<HeaderCandidate, 16> header_candidates;
                    const auto &header_pli = precise_liveness[block_idx_u32];
                    for (const auto &entry: header_pli.next_uses) {
                        const auto val_idx = entry.first;
                        if (selected_values.contains(val_idx) ||
                            header_phi_defs.contains(val_idx) ||
                            header_local_defs.contains(val_idx)) {
                            continue;
                        }

                        const u32 entry_next_use =
                                get_current_and_next_use(header_pli, val_idx, 0).first;
                        if (entry_next_use >= LOOP_EXIT_PENALTY || entry_next_use == INF) {
                            continue;
                        }
                        if (entry_next_use & DEF_BIT) {
                            continue;
                        }
                        if (entry_next_use > std::ranges::distance(adaptor->block_insts(
                                block_ref(BlockIndex{block_idx_u32})))) {
                            continue;
                        }
                        // todo should be infavid validx
                        if (static_cast<u32>(val_idx) >= value_parts_cache.size()) {
                            continue;
                        }
                        const RegParts parts = ensure_parts_cached(val_idx);

                        // todo(salt): num_parts why?
                        header_candidates.push_back(
                            HeaderCandidate{
                                .val_idx = val_idx, .entry_next_use = entry_next_use,
                                .num_parts = u32(parts.gp_regs + parts.fp_regs)
                            });
                    }
                    for (const auto &entry: header_pli.live_through) {
                        const auto val_idx = entry.first;
                        if (selected_values.contains(val_idx) ||
                            header_phi_defs.contains(val_idx) ||
                            header_local_defs.contains(val_idx)) {
                            continue;
                        }
                        header_candidates.push_back(
                            HeaderCandidate{
                                .val_idx = val_idx, .entry_next_use = entry.second,
                                .num_parts = u32(
                                    ensure_parts_cached(val_idx).gp_regs + ensure_parts_cached(val_idx).fp_regs)
                            });
                    }

                    std::sort(header_candidates.begin(),
                              header_candidates.end(),
                              [](const auto &a, const auto &b) {
                                  if (a.entry_next_use == b.entry_next_use) {
                                      if (a.num_parts == b.num_parts) {
                                          return a.val_idx < b.val_idx;
                                      }
                                      return a.num_parts < b.num_parts;
                                  }
                                  return a.entry_next_use < b.entry_next_use;
                              });
                    const auto gp_threshold = NUM_GP_REGS - block_pressure[block_idx_u32].gp_pressure - 1;
                    const auto fp_threshold = NUM_FP_REGS - block_pressure[block_idx_u32].fp_pressure - 1;
                    for (const auto &candidate: header_candidates) {
                        if (loop_local_gp_regs > gp_threshold ||
                            loop_local_fp_regs > fp_threshold) {
                            break;
                        }
                        const auto parts = ensure_parts_cached(candidate.val_idx);
                        const bool gp_has_budget = loop_local_gp_regs < gp_threshold;
                        const bool fp_has_budget = loop_local_fp_regs < fp_threshold;
                        if ((!gp_has_budget && parts.gp_regs > 0) ||
                            (!fp_has_budget && parts.fp_regs > 0)) {
                            continue;
                        }

                        loop_local_values.push_back(candidate.val_idx);
                        selected_values.insert(candidate.val_idx);
                        loop_local_gp_regs += parts.gp_regs;
                        loop_local_fp_regs += parts.fp_regs;
                    }
                    if (false && loop_local_gp_regs < gp_threshold / 2 && loop_local_fp_regs < fp_threshold / 2) {
                        std::sort(incoming_from_all.begin(), incoming_from_all.end(),
                                  [&](const auto &a, const auto &b) {
                                      return get_current_and_next_use(
                                                 precise_liveness[block_idx_u32],
                                                 a,
                                                 0)
                                             .first <
                                             get_current_and_next_use(
                                                 precise_liveness[block_idx_u32],
                                                 b,
                                                 0)
                                             .first;
                                  });
                        for (const auto val_idx: incoming_from_all) {
                            if (selected_values.contains(val_idx))
                                continue;
                            const bool gp_has_budget = loop_local_gp_regs < gp_threshold / 2;
                            const bool fp_has_budget = loop_local_fp_regs < fp_threshold / 2;
                            if ((!gp_has_budget) ||
                                (!fp_has_budget)) {
                                break;
                            }

                            const auto entry_next_use =
                                    get_current_and_next_use(precise_liveness[block_idx_u32], val_idx, 0).first;
                            loop_local_values.push_back(val_idx);
                            selected_values.insert(val_idx);
                            const auto parts = ensure_parts_cached(val_idx);
                            loop_local_gp_regs += parts.gp_regs;
                            loop_local_fp_regs += parts.fp_regs;
                        }
                    }


                    incoming_from_all.clear();
                    incoming_from_all.append(loop_local_values.begin(), loop_local_values.end());
                }
                std::array<u8, 2> from_all_registers = {0u, 0u};
                for (const auto val_idx: incoming_from_all) {
                    const auto parts = ensure_parts_cached(val_idx);
                    from_all_registers[0] += parts.gp_regs;
                    from_all_registers[1] += parts.fp_regs;
                }

                if (from_all_registers[0] > NUM_GP_REGS ||
                    from_all_registers[1] > NUM_FP_REGS) [[unlikely]] {
                    // prefer values that are used soon.
                    std::sort(
                        incoming_from_all.begin(),
                        incoming_from_all.end(),
                        [&](const auto &a, const auto &b) {
                            return get_current_and_next_use(
                                       precise_liveness[block_idx_u32],
                                       a,
                                       0)
                                   .first <
                                   get_current_and_next_use(
                                       precise_liveness[block_idx_u32],
                                       b,
                                       0)
                                   .first;
                        });
                    working_set.clear();
                    for (const auto val_idx: incoming_from_all) {
                        if (!working_set.can_fit(val_idx, NUM_GP_REGS, NUM_FP_REGS)) {
                            break;
                        }
                        working_set.insert(val_idx);
                    }
                } else {
                    working_set.replace_with(incoming_from_all);
                }
            }
            if (is_loop_header_block) {
                auto loop_idx = block_loop_idx(block_idx_cur);
                auto loop = loop_from_idx(loop_idx);
                //todo(salto): tune
                if (loop.level < 300) {
                    auto &initial_working_set = initial_working_set_by_block[block_idx_cur];
                    initial_working_set.clear();
                    initial_working_set.insert(working_set.begin(), working_set.end());
                }
            }

            // W is the working set. The values in registers
            // keep used registers seperately, since one value can use multiple
            // registers.
            for (const auto phi: adaptor->block_phis(block)) {
                // todo(salto): should we be able to spill phis?
                const auto val_idx = adaptor->val_local_idx(phi);
                working_set.insert(val_idx);
            }

            u32 idx = 0;
            bool last_was_call = false;
            for (const auto inst: adaptor->block_insts(block)) {
                if (last_was_call) {
                    for (const auto val_idx: working_set) {
                        const auto parts = ensure_parts_cached(val_idx);
                        const auto &uses = get_current_and_next_use(precise_liveness[block_idx_u32], val_idx, idx);
                        if (parts.gp_regs + parts.fp_regs == 1 && uses.first != INF && uses.second < idx + 10) {
                            callee_saved_values.push_back(val_idx);
                        }
                    }
                }
                for (const auto operand: adaptor->inst_operands(inst)) {
                    if (adaptor->val_ignore_in_liveness_analysis(operand)) {
                        // what can we do here?
                        continue;
                    }
                    const auto val_idx = adaptor->val_local_idx(operand);
                    working_set.insert(val_idx);
                }

                // we process results and operands at once whenever possible. The
                // original algorithm seperates them, as it effects the optimal spill
                // position. since we don't need to know the spill position, we can
                // process operands
                // + results together. we still need to ensure that there are free
                // registers for the results. instead of limit(W, NUM_*_REGS) and then
                // limit(W+results, NUM_*_REGS-NUM_RESULT_REGs), we can avoid one of
                // the limits in most cases.
                std::array<u32, 2> num_result_regs = {0u, 0u};
                for (const auto result: adaptor->inst_results(inst)) {
                    if (adaptor->val_ignore_in_liveness_analysis(result)) {
                        // what can we do here?
                        continue;
                    }
                    // the original algorithm doesn't add results to W until after both
                    // limits. Iterating through results could be expensive, so we wan't
                    // to avoid iterating it twice. Results have a current_use of 0 so
                    // they will not be spilled.
                    auto val_idx = adaptor->val_local_idx(result);
                    working_set.insert_as_result(val_idx);

                    const auto parts = ensure_parts_cached(val_idx);
                    num_result_regs[0] += parts.gp_regs;
                    num_result_regs[1] += parts.fp_regs;
                }
                // todo(salto): check wether if with [[unlikely]] has better
                // performance
                const bool has_call = adaptor->inst_has_call(inst);
                last_was_call = has_call;
                // capacity after is lower for calls due to caller-saved registers.
                // But the call results are already in the registers automatically.
                //todo(salto): fix potential underflow
                const u32 capacity_before_instr_gp = NUM_GP_REGS;
                const u32 capacity_after_instr_gp =
                        NUM_GP_REGS - static_cast<u32>(has_call) *
                        (NUM_CALLER_SAVED_GP - num_result_regs[0]);
                const u32 capacity_before_instr_fp = NUM_FP_REGS;
                const u32 capacity_after_instr_fp =
                        NUM_FP_REGS - static_cast<u32>(has_call) *
                        (NUM_CALLER_SAVED_FP - num_result_regs[1]);
                // we still have enough registers for both results and operands at the
                // same time, no spills needed
                if (working_set.fits_into(
                    capacity_after_instr_gp, capacity_after_instr_fp, true)) {
                    working_set.commit_result_regs();
                    ++idx;
                    continue;
                }
                // todo(salto): we could store dead after instr during the initial
                // loop, then maybe we can avoid some sorts We spill the furthest
                // next-use value. (val_idx, ,current_use,next_use, parts)
                util::SmallVector<SpillCandidate, 16> W_next_uses;
                util::SmallVector<ValLocalIdx, 16> dead_values;
                for (const auto val_idx: working_set) {
                    const auto [current_use, next_use] = get_current_and_next_use(
                        precise_liveness[static_cast<u32>(block_idx(block))], val_idx, idx);
                    assert(current_use != INF ||
                        "Value is used, but its liveness was not computed.");
                    // we can evict all dead values by default.
                    if (current_use == INF) {
                        dead_values.push_back(val_idx);
                        // todo(salto): maybe break if we already have enough registers?
                        continue;
                    }
                    W_next_uses.emplace_back(
                        val_idx, current_use, next_use, ensure_parts_cached(val_idx));
                }
                // Remove dead values
                for (const auto val_idx: dead_values) {
                    working_set.erase(val_idx);
                }
                // Evicting dead values was enough to free up registers, avoid more
                // expensive spill calculation.
                if (working_set.fits_into(
                    capacity_after_instr_gp, capacity_after_instr_fp, true)) {
                    working_set.commit_result_regs();
                    ++idx;
                    continue;
                }
                if (!working_set.fits_into(
                    capacity_before_instr_gp, capacity_before_instr_fp, false)) {
                    // sort by current use since we need space for operands
                    std::sort(W_next_uses.begin(),
                              W_next_uses.end(),
                              [](const auto &a, const auto &b) {
                                  // todo(salto): maybe prefer spilling values with
                                  // more parts? todo(salto): evaluate if more
                                  // expensive sort is worth it

                                  return (a.current_use == b.current_use)
                                             ? (a.next_use > b.next_use)
                                             : a.current_use > b.current_use;
                              });
                    limit(W_next_uses,
                          capacity_before_instr_gp + num_result_regs[0],
                          capacity_before_instr_fp + num_result_regs[1],
                          idx,
                          working_set,
                          false);
                }

                // we are limited by the results.
                if (!working_set.fits_into(
                    capacity_after_instr_gp, capacity_after_instr_fp, true)) {
                    // sort by next use instead of current use since we have
                    // enough space for all the operands and we might be able to
                    // evict a operand for a result. we try to avoid 2 sorts as
                    // much as possible.
                    std::sort(W_next_uses.begin(),
                              W_next_uses.end(),
                              [](const auto &a, const auto &b) {
                                  // todo(salto): maybe prefer spilling values with
                                  // more parts?
                                  return a.next_use > b.next_use;
                              });
                    limit(W_next_uses,
                          capacity_after_instr_gp,
                          capacity_after_instr_fp,
                          idx,
                          working_set,
                          true);
                }
                working_set.commit_result_regs();
                ++idx;
            }
            for (const auto val_idx: working_set) {
                for (const auto succ: adaptor->block_succs(block)) {
                    const auto succ_idx = block_idx(succ);
                    const auto &pli = precise_liveness[static_cast<u32>(succ_idx)];
                    const auto it = pli.next_uses.find(val_idx);
                    if (it == pli.next_uses.end()) {
                        continue;
                    }
                    const auto &vec = it->second;
                    if (vec.empty()) {
                        continue;
                    }
                    const u32 updated_freq = ++W_entry_freq[static_cast<u32>(succ_idx)][val_idx];
                    if (updated_freq > W_max_entry_freq[static_cast<u32>(succ_idx)]) {
                        W_max_entry_freq[static_cast<u32>(succ_idx)] = updated_freq;
                    }
                }
            }
        }
        TPDE_LOG_TRACE("Spill Analysis completed");
        std::sort(callee_saved_values.begin(), callee_saved_values.end());
    }

    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::limit(
        util::SmallVector<SpillCandidate, 16> &W_next_uses,
        const u32 NUM_GP_REGS,
        const u32 NUM_FP_REGS,
        u32 &idx,
        WorkingSetTracker &working_set,
        bool after_instr) {
        // Calculate how many registers need to be freed
        const u32 used_gp_regs = working_set.used_gp_regs(true);
        const u32 used_fp_regs = working_set.used_fp_regs(true);
        u32 gp_to_free =
                used_gp_regs > NUM_GP_REGS ? (used_gp_regs - NUM_GP_REGS) : 0;
        u32 fp_to_free =
                used_fp_regs > NUM_FP_REGS ? (used_fp_regs - NUM_FP_REGS) : 0;

        if (gp_to_free == 0 && fp_to_free == 0) {
            return; // Already within capacity
        }

        const bool gp_limited = gp_to_free > 0 && fp_to_free == 0;
        const bool fp_limited = fp_to_free > 0 && gp_to_free == 0;

        // Phase 1: Collect candidate values that could be spilled
        // These are selected in priority order (respecting the caller's sort)
        util::SmallVector<SpillCandidate, 16> candidates;
        u32 total_gp_in_candidates = 0;
        u32 total_fp_in_candidates = 0;

        for (const auto &entry: W_next_uses) {
            // we should have already evicted all dead values.
            assert(entry.current_use != INF);

            candidates.push_back(entry);
            total_gp_in_candidates += entry.parts.gp_regs;
            total_fp_in_candidates += entry.parts.fp_regs;

            const bool enough_gp = total_gp_in_candidates >= gp_to_free;
            const bool enough_fp = total_fp_in_candidates >= fp_to_free;
            if ((gp_limited && enough_gp) || (fp_limited && enough_fp) ||
                (enough_gp && enough_fp)) {
                break;
            }
        }

        // Phase 2: Optimize selection if we have excess capacity
        // mainly happens with large vectors, since we can only spill whole vector
        // atm.
        const u32 excess_gp = total_gp_in_candidates > gp_to_free
                                  ? total_gp_in_candidates - gp_to_free
                                  : 0;
        const u32 excess_fp = total_fp_in_candidates > fp_to_free
                                  ? total_fp_in_candidates - fp_to_free
                                  : 0;

        // If we have excess and multiple candidates, reorder by size
        // This can reduce the number of values spilled
        if ((excess_gp > 0 || excess_fp > 0) && candidates.size() > 1) [[unlikely]] {
            std::sort(candidates.begin(),
                      candidates.end(),
                      [&](const auto &a, const auto &b) {
                          const u32 total_a = a.parts.gp_regs + a.parts.fp_regs;
                          const u32 total_b = b.parts.gp_regs + b.parts.fp_regs;
                          // Sort by num_parts (descending) - prefer spilling larger
                          // values
                          return total_a > total_b;
                      });
        }

        auto spill_pass = [&](u32 &gp_needed, u32 &fp_needed, bool single_bank) {
            u32 gp_freed = 0;
            u32 fp_freed = 0;
            for (const auto &entry: candidates) {
                if (gp_freed >= gp_needed && fp_freed >= fp_needed) {
                    break; // Freed enough registers
                }
                if (single_bank) {
                    if (gp_needed > 0 && gp_freed >= gp_needed) {
                        break;
                    }
                    if (fp_needed > 0 && fp_freed >= fp_needed) {
                        break;
                    }
                }
                // Skip value if it is not in the needed bank
                if ((gp_needed <= gp_freed && entry.parts.fp_regs == 0) || (
                        fp_needed <= fp_freed && entry.parts.gp_regs == 0)) {
                    continue;
                }


                // don't spill if the value is dead after the instruction
                if (!(after_instr && (entry.next_use == INF))) {
                    TPDE_LOG_TRACE("Spilling value {} with current use {} and {} parts at "
                                   "instruction idx {}",
                                   static_cast<u32>(entry.val_idx),
                                   entry.current_use,
                                   static_cast<u32>(entry.parts.gp_regs + entry.parts.fp_regs),
                                   idx);
                    // todo(salto): check if already set?
                    spilled_values.mark_set(static_cast<u32>(entry.val_idx));
                }

                working_set.erase(entry.val_idx);
                gp_freed += entry.parts.gp_regs;
                fp_freed += entry.parts.fp_regs;
            }

            gp_needed = gp_freed >= gp_needed ? 0 : gp_needed - gp_freed;
            fp_needed = fp_freed >= fp_needed ? 0 : fp_needed - fp_freed;
        };

        if (gp_limited || fp_limited) {
            spill_pass(gp_to_free, fp_to_free, true);
            if ((gp_to_free > 0 || fp_to_free > 0) &&
                (working_set.used_gp_regs(true) > NUM_GP_REGS ||
                 working_set.used_fp_regs(true) > NUM_FP_REGS)) [[unlikely]] {
                spill_pass(gp_to_free, fp_to_free, false);
            }
            return;
        }

        spill_pass(gp_to_free, fp_to_free, false);
    }


    template<IRAdaptor Adaptor, typename CompilerType>
    void Analyzer<Adaptor, CompilerType>::compute_liveness() noexcept {
        // implement the liveness algorithm described in
        // http://databasearchitects.blogspot.com/2020/04/linear-time-liveness-analysis.html
        // and Kohn et al.: Adaptive Execution of Compiled Queries
        // TODO(ts): also expose the two-pass liveness algo as an option

        TPDE_LOG_TRACE("Starting Liveness Analysis");

        // Bump epoch. On overflow, we must clear all liveness info entries.
        if (++liveness_epoch == 0) {
            liveness.clear();
            liveness_epoch = 1;
        }

        if constexpr (Adaptor::TPDE_PROVIDES_HIGHEST_VAL_IDX) {
            liveness_max_value = adaptor->cur_highest_val_idx();
            if (liveness_max_value >= liveness.size()) {
                liveness.resize(liveness_max_value + 1);
            }
        } else {
            liveness_max_value = 0;
        }

        num_insts = 0;

        const auto visit = [this](const IRValueRef value, const u32 block_idx) {
            TPDE_LOG_TRACE("  Visiting value {} in block {}",
                           adaptor->value_fmt_ref(value),
                           block_idx);
            if (adaptor->val_ignore_in_liveness_analysis(value)) {
                TPDE_LOG_TRACE("    value is ignored");
                return;
            }

            auto &liveness = liveness_maybe(value);
            if (liveness.epoch != liveness_epoch) {
                TPDE_LOG_TRACE("    initializing liveness info, lcl is {}",
                               block_loop_map[block_idx]);
                liveness = LivenessInfo{
                    .definition_block = static_cast<BlockIndex>(block_idx),
                    .first = static_cast<BlockIndex>(block_idx),
                    .last = static_cast<BlockIndex>(block_idx),
                    .ref_count = 1,
                    .lowest_common_loop = block_loop_map[block_idx],
                    .last_full = false,
                    .epoch = liveness_epoch,
                };
                return;
            }

            assert(liveness.ref_count != ~0u && "used value without definition");

            ++liveness.ref_count;
            TPDE_LOG_TRACE("    increasing ref_count to {}", liveness.ref_count);

            // helpers
            const auto update_for_block_only =
                    [&liveness, block_idx](bool check_for_backwards_extension) {
                const auto old_first = static_cast<u32>(liveness.first);
                const auto old_last = static_cast<u32>(liveness.last);

                const auto new_first = std::min(old_first, block_idx);
                const auto new_last = std::max(old_last, block_idx);
                liveness.first = static_cast<BlockIndex>(new_first);
                liveness.last = static_cast<BlockIndex>(new_last);

                // if last changed, we don't need to extend the lifetime to the end
                // of the last block except if we extended the liveness interval
                // to an outside loop which means we saw a use in a previous block
                if (old_last == new_last && check_for_backwards_extension) {
                    liveness.last_full = true;
                } else if (old_last != new_last) {
                    liveness.last_full = false;
                }
            };

            const auto update_for_loop = [&liveness](
                const Loop &loop,
                bool check_for_backwards_extension) {
                const auto old_first = static_cast<u32>(liveness.first);
                const auto old_last = static_cast<u32>(liveness.last);

                const auto new_first = std::min(old_first, static_cast<u32>(loop.begin));
                const auto new_last = std::max(old_last, static_cast<u32>(loop.end) - 1);
                liveness.first = static_cast<BlockIndex>(new_first);
                liveness.last = static_cast<BlockIndex>(new_last);

                // if last changed, set last_full to true
                // since the values need to be allocated when the loop is active
                // Otherwise, if the liveness interval was extended backwards we need to
                // mark the liveness intervall as full as well
                if (old_last != new_last || check_for_backwards_extension) {
                    liveness.last_full = true;
                }
            };

            const auto block_loop_idx = block_loop_map[block_idx];
            if (liveness.lowest_common_loop == block_loop_idx) {
                // just extend the liveness interval
                TPDE_LOG_TRACE("    lcl is same as block loop");
                update_for_block_only(false);

                TPDE_LOG_TRACE("    new interval {}->{}, lf: {}",
                               static_cast<u32>(liveness.first),
                               static_cast<u32>(liveness.last),
                               liveness.last_full);
                return;
            }

            const Loop &liveness_loop = loops[liveness.lowest_common_loop];
            const Loop &block_loop = loops[block_loop_idx];

            if (liveness_loop.level < block_loop.level &&
                static_cast<u32>(block_loop.begin) <
                static_cast<u32>(liveness_loop.end)) {
                assert(static_cast<u32>(block_loop.end) <=
                    static_cast<u32>(liveness_loop.end));

                TPDE_LOG_TRACE("    block_loop {} is nested inside lcl", block_loop_idx);
                // The current use is nested inside the loop of the liveness
                // interval so we only need to get the loop at level
                // (liveness_loop.level + 1) that contains block_loop and extend the
                // liveness interval
                const auto target_level = liveness_loop.level + 1;
                auto cur_loop_idx = block_loop_idx;
                auto cur_level = block_loop.level;
                // TODO(ts): we could also skip some loops here since we know that
                // there have to be at least n=(cur_level-target_level) loops
                // between cur_loop and target_loop
                // so we could choose cur_loop_idx = min(cur_parent, cur_idx-n)?
                // however this might jump into more nested loops
                // so maybe check (cur_idx-n).level first?
                while (cur_level != target_level) {
                    cur_loop_idx = loops[cur_loop_idx].parent;
                    --cur_level;
                }
                assert(loops[cur_loop_idx].level == target_level);
                TPDE_LOG_TRACE("    target_loop is {}", cur_loop_idx);
                update_for_loop(loops[cur_loop_idx], false);

                TPDE_LOG_TRACE("    new interval {}->{}, lf: {}",
                               static_cast<u32>(liveness.first),
                               static_cast<u32>(liveness.last),
                               liveness.last_full);
                return;
            }

            TPDE_LOG_TRACE("    block_loop {} is nested higher or in a different loop",
                           block_loop_idx);
            // need to update the lowest common loop to contain both liveness_loop
            // and block_loop and then extend the interval accordingly

            // TODO(ts): this algorithm is currently worst-case O(n) which makes the
            // whole liveness analysis worst-case O(n^2) which is not good. However,
            // we expect the number of loops to be much smaller than the number of
            // values so it should be fine for most programs. To be safe, we should
            // implement the algorithm from "Gusfield: Constant-Time Lowest Common
            // Ancestor Retrieval" which seems to be the one with the most
            // reasonable overhead. It is however not zero and the queries are also
            // not exactly free so we should definitely benchmark both version first
            // with both small and large programs

            auto lhs_idx = liveness.lowest_common_loop;
            auto rhs_idx = block_loop_idx;
            auto prev_rhs = rhs_idx;
            auto prev_lhs = lhs_idx;
            while (lhs_idx != rhs_idx) {
                const auto lhs_level = loops[lhs_idx].level;
                const auto rhs_level = loops[rhs_idx].level;
                if (lhs_level > rhs_level) {
                    prev_lhs = lhs_idx;
                    lhs_idx = loops[lhs_idx].parent;
                } else if (lhs_level < rhs_level) {
                    prev_rhs = rhs_idx;
                    rhs_idx = loops[rhs_idx].parent;
                } else {
                    prev_lhs = lhs_idx;
                    prev_rhs = rhs_idx;
                    lhs_idx = loops[lhs_idx].parent;
                    rhs_idx = loops[rhs_idx].parent;
                }
            }

            assert(static_cast<u32>(loops[lhs_idx].begin) <=
                static_cast<u32>(liveness_loop.begin));
            assert(static_cast<u32>(loops[lhs_idx].end) >=
                static_cast<u32>(liveness_loop.end));
            TPDE_LOG_TRACE("    new lcl is {}", lhs_idx);

            liveness.lowest_common_loop = lhs_idx;

            // extend for the full loop that contains liveness_loop and is nested
            // directly in lcl
            assert(loops[prev_lhs].parent == lhs_idx);
            assert(static_cast<u32>(loops[prev_lhs].begin) <=
                static_cast<u32>(liveness_loop.begin));
            assert(static_cast<u32>(loops[prev_lhs].end) >=
                static_cast<u32>(liveness_loop.end));
            update_for_loop(loops[prev_lhs], false);

            // extend by block if the block_loop is the lcl
            // or by prev_rhs (the loop containing block_loop nested directly in
            // lcl) otherwise
            if (lhs_idx == block_loop_idx) {
                update_for_block_only(true);
            } else {
                assert(loops[prev_rhs].parent == lhs_idx);
                assert(loops[prev_rhs].level == loops[lhs_idx].level + 1);
                update_for_loop(loops[prev_rhs], true);
            }

            TPDE_LOG_TRACE("    new interval {}->{}, lf: {}",
                           static_cast<u32>(liveness.first),
                           static_cast<u32>(liveness.last),
                           liveness.last_full);
        };

        assert(block_layout[0] == adaptor->cur_entry_block());
        if constexpr (Adaptor::TPDE_LIVENESS_VISIT_ARGS) {
            for (const IRValueRef arg: adaptor->cur_args()) {
                visit(arg, 0);
            }
        }

        for (u32 block_idx = 0; block_idx < block_layout.size(); ++block_idx) {
            IRBlockRef block = block_layout[block_idx];
            TPDE_LOG_TRACE(
                "Analyzing block {} ('{}')", block_idx, adaptor->block_fmt_ref(block));
            const auto block_loop_idx = block_loop_map[block_idx];

            bool has_phis = false;
            for (const IRValueRef phi: adaptor->block_phis(block)) {
                TPDE_LOG_TRACE("Analyzing phi {}", adaptor->value_fmt_ref(phi));
                has_phis = true;

                const auto phi_ref = adaptor->val_as_phi(phi);
                const u32 slot_count = phi_ref.incoming_count();
                for (u32 i = 0; i < slot_count; ++i) {
                    const IRBlockRef incoming_block = phi_ref.incoming_block_for_slot(i);
                    const IRValueRef incoming_value = phi_ref.incoming_val_for_slot(i);
                    if (adaptor->block_info2(incoming_block) == 0) {
                        TPDE_LOG_TRACE("ignoring phi input from unreachable pred ({})",
                                       adaptor->block_fmt_ref(incoming_block));
                        continue;
                    }
                    const auto incoming_block_idx = adaptor->block_info(incoming_block);

                    // mark the incoming value as used in the incoming block
                    visit(incoming_value, incoming_block_idx);
                    // mark the PHI-value as used in the incoming block
                    visit(phi, incoming_block_idx);
                }
            }

            if (has_phis) {
                adaptor->block_set_info2(block, adaptor->block_info2(block) | 0b1'0000);
            }

            for (const IRInstRef inst: adaptor->block_insts(block)) {
                TPDE_LOG_TRACE("Analyzing instruction {}", adaptor->inst_fmt_ref(inst));
                for (const IRValueRef res: adaptor->inst_results(inst)) {
                    // mark the value as used in the current block
                    visit(res, block_idx);
                    ++loops[block_loop_idx].definitions;
                }

                for (const IRValueRef operand: adaptor->inst_operands(inst)) {
                    visit(operand, block_idx);
                }

                num_insts += 1;
            }
        }

        // fill out the definitions_in_childs counters
        // (skip 0 since it has itself as a parent)
        for (u32 idx = loops.size() - 1; idx != 0; --idx) {
            auto &loop = loops[idx];
            loops[loop.parent].definitions_in_childs +=
                    loop.definitions_in_childs + loop.definitions;
        }

#ifdef TPDE_ASSERTS
        // reset the incorrect ref_counts in the liveness infos
        for (auto &entry: liveness) {
            if (entry.ref_count == ~0u) {
                entry.ref_count = 0;
            }
        }
#endif

        TPDE_LOG_TRACE("Finished Liveness Analysis");
    }
} // namespace tpde
