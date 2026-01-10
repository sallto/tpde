// SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
//
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#pragma once

#include <algorithm>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <fstream>
#include <vector>

#include "Analyzer.hpp"
#include "Compiler.hpp"
#include "CompilerConfig.hpp"
#include "IRAdaptor.hpp"
#include "tpde/AssignmentPartRef.hpp"
#include "tpde/FunctionWriter.hpp"
#include "tpde/RegisterFile.hpp"
#include "tpde/ValLocalIdx.hpp"
#include "tpde/ValueAssignment.hpp"
#include "tpde/VerificationIR.hpp"
#include "tpde/base.hpp"
#include "tpde/util/function_ref.hpp"
#include "tpde/util/misc.hpp"

#include <map>

namespace tpde {

// Threshold for triggering hybrid register/stack allocation for phi nodes
// todo(salto): maybe do this smarter, but when is this really relevant?
static constexpr u32 PHI_REGISTER_THRESHOLD = 12;

// TODO(ts): formulate concept for full compiler so that there is *some* check
// whether all the required derived methods are implemented?

template <IRAdaptor Adaptor>
using VIR = VerificationIR<Adaptor, typename Analyzer<Adaptor>::BlockIndex, typename Adaptor::IRInstRef>;

/// Thread-local storage access mode
enum class TLSModel {
  GlobalDynamic,
  LocalDynamic,
  InitialExec,
  LocalExec,
};

struct CCAssignment {
  Reg reg = Reg::make_invalid(); ///< Assigned register, invalid implies stack.

  /// If non-zero, indicates that this and the next N values must be
  /// assigned to consecutive registers, or to the stack. The following
  /// values must be in the same register bank as this value. u8 is sufficient,
  /// no architecture has more than 255 parameter registers in a single bank.
  u8 consecutive = 0;
  bool sret : 1 = false; ///< Argument is return value pointer.

  /// The argument is passed by value on the stack. The provided argument is a
  /// pointer; for the call, size bytes will be copied into the corresponding
  /// stack slot. Behaves like LLVM's byval.
  ///
  /// Note: On x86-64 SysV, this is used to pass larger structs in memory. Note
  /// that AArch64 AAPCS doesn't use byval for structs, instead, the pointer is
  /// passed without byval and it is the responsibility of the caller to
  /// explicitly copy the value.
  bool byval : 1 = false;

  /// Extend integer argument. Highest bit indicates signed-ness, lower bits
  /// indicate the source width from which the argument should be extended.
  u8 int_ext = 0;

  u8 align = 0;             ///< Argument alignment
  RegBank bank = RegBank{}; ///< Register bank to assign the value to.
  u32 size = 0;             ///< Argument size, for byval the stack slot size.
  u32 stack_off = 0; ///< Assigned stack slot, only valid if reg is invalid.
};

struct CCInfo {
  // TODO: use RegBitSet
  const u64 allocatable_regs;
  const u64 callee_saved_regs;
  /// Possible argument registers; these registers will not be allocated until
  /// all arguments have been assigned.
  const u64 arg_regs;
  /// Size of red zone below stack pointer.
  const u8 red_zone_size = 0;
};

class CCAssigner {
public:
  const CCInfo *ccinfo;

  CCAssigner(const CCInfo &ccinfo) noexcept : ccinfo(&ccinfo) {}
  virtual ~CCAssigner() noexcept {}

  virtual void reset() noexcept = 0;

  const CCInfo &get_ccinfo() const noexcept { return *ccinfo; }

  virtual void assign_arg(CCAssignment &cca) noexcept = 0;
  virtual u32 get_stack_size() noexcept = 0;
  /// Some calling conventions need different call behavior when calling a
  /// vararg function.
  virtual bool is_vararg() const noexcept { return false; }
  virtual void assign_ret(CCAssignment &cca) noexcept = 0;
};

/// The base class for the compiler.
/// It implements the main platform independent compilation logic and houses the
/// analyzer
template <IRAdaptor Adaptor,
          typename Derived,
          CompilerConfig Config = CompilerConfigDefault>
struct CompilerBase {
  // some forwards for the IR type defs
  using IRValueRef = typename Adaptor::IRValueRef;
  using IRInstRef = typename Adaptor::IRInstRef;
  using IRBlockRef = typename Adaptor::IRBlockRef;
  using IRFuncRef = typename Adaptor::IRFuncRef;

  using BlockIndex = typename Analyzer<Adaptor>::BlockIndex;

  using Assembler = typename Config::Assembler;
  using AsmReg = typename Config::AsmReg;

  using RegisterFile = tpde::RegisterFile<Config::NUM_BANKS, 32>;

  /// A default implementation for ValRefSpecial.
  // Note: Subclasses can override this, always used Derived::ValRefSpecial.
  struct ValRefSpecial {
    uint8_t mode = 4;
    u64 const_data;
  };

#pragma region CompilerData
  Adaptor *adaptor;
  Analyzer<Adaptor> analyzer;

  // data for frame management

  struct {
    /// The current size of the stack frame
    u32 frame_size = 0;
    /// Whether the stack frame might have dynamic alloca. Dynamic allocas may
    /// require a different and less efficient frame setup.
    bool has_dynamic_alloca;
    /// Whether the function is guaranteed to be a leaf function. Throughout the
    /// entire function, the compiler may assume the absence of function calls.
    bool is_leaf_function;
    /// Whether the function actually includes a call. There are cases, where it
    /// is not clear from the beginning whether a function has function calls.
    /// If a function has no calls, this will allow using the red zone
    /// guaranteed by some ABIs.
    bool generated_call;
    /// Free-Lists for 1/2/4/8/16 sized allocations
    // TODO(ts): make the allocations for 4/8 different from the others
    // since they are probably the one's most used?
    util::SmallVector<i32, 16> fixed_free_lists[5] = {};
    /// Free-Lists for all other sizes
    // TODO(ts): think about which data structure we want here
    std::unordered_map<u32, std::vector<i32>> dynamic_free_lists{};
  } stack = {};

  typename Analyzer<Adaptor>::BlockIndex cur_block_idx;

  // Assignments

  static constexpr ValLocalIdx INVALID_VAL_LOCAL_IDX =
      static_cast<ValLocalIdx>(~0u);

  // TODO(ts): think about different ways to store this that are maybe more
  // compact?
  struct {
    AssignmentAllocator allocator;

    std::array<u32, Config::NUM_BANKS> cur_fixed_assignment_count = {};
    util::SmallVector<ValueAssignment *, Analyzer<Adaptor>::SMALL_VALUE_NUM>
        value_ptrs;

    ValLocalIdx variable_ref_list;
    util::SmallVector<ValLocalIdx, Analyzer<Adaptor>::SMALL_BLOCK_NUM>
        delayed_free_lists;
  } assignments = {};

  RegisterFile register_file;
  enum class MoveStatus {
    TO_MOVE,
    MOVING,
    DONE
  };
  struct RegisterMove {
    ValLocalIdx value_idx = INVALID_VAL_LOCAL_IDX;
    u32 part_idx = 0;
    Reg dst;
    Reg src;
    u8 size{};
    MoveStatus status = MoveStatus::TO_MOVE;

    RegisterMove() = default;
    RegisterMove(Reg d, Reg s, u8 sz,
                 ValLocalIdx val = INVALID_VAL_LOCAL_IDX,
                 u32 part = 0) noexcept
        : value_idx(val), part_idx(part), dst(d), src(s), size(sz) {}
  };
  using MoveList = util::SmallVector<RegisterMove, 16>;
#ifndef NDEBUG
  /// Whether we are currently in the middle of generating branch-related code
  /// and therefore must not change any value-related state.
  bool generating_branch = false;
#endif
  struct RegisterState {
    std::array<ValLocalIdx,64> registers{}; bool valid=false;
    RegisterState() {
      registers.fill(INVALID_VAL_LOCAL_IDX);
    }
  };
  util::SmallVector<RegisterState, Analyzer<Adaptor>::SMALL_BLOCK_NUM> block_states;
  struct ValueState {
    ValLocalIdx val_local_idx;

    util::SmallVector<Reg, 4> registers;
    [[nodiscard]] explicit ValueState(ValLocalIdx val_local_idx, u32 parts)
        : val_local_idx(val_local_idx) {
      registers.resize(parts + 1);
    }
    ValueState() : val_local_idx(INVALID_VAL_LOCAL_IDX) {}
    ~ValueState() = default;

    ValueState(const ValueState &) = delete;
    ValueState &operator=(const ValueState &) = delete;

    ValueState(ValueState &&) = default;
    ValueState &operator=(ValueState &&) = default;

    void push_back(Reg reg, u32 part) { registers[part] = reg; }
  };
  std::unordered_map<BlockIndex, util::SmallVector<ValueState>> block_regs;
  std::unordered_map<ValLocalIdx, util::SmallVector<Reg>> phi_regs;

#ifndef NDEBUG
  VIR<Adaptor> verification_ir;
#endif

private:
  /// Default CCAssigner if the implementation doesn't override cur_cc_assigner.
  typename Config::DefaultCCAssigner default_cc_assigner;

public:
  Assembler assembler;
  Config::FunctionWriter text_writer;
  // TODO(ts): smallvector?
  std::vector<SymRef> func_syms;
  // TODO(ts): combine this with the block vectors in the analyzer to save on
  // allocations
  util::SmallVector<Label> block_labels;

  util::SmallVector<std::pair<SymRef, SymRef>, 4> personality_syms = {};

  struct ScratchReg;
  class ValuePart;
  struct ValuePartRef;
  struct ValueRef;
  struct GenericValuePart;
#pragma endregion

  struct InstRange {
    using Range = decltype(std::declval<Adaptor>().block_insts(
        std::declval<IRBlockRef>()));
    using Iter = decltype(std::declval<Range>().begin());
    using EndIter = decltype(std::declval<Range>().end());
    Iter from;
    EndIter to;
  };

  struct CallArg {
    enum class Flag : u8 {
      none,
      zext,
      sext,
      sret,
      byval
    };

    explicit CallArg(IRValueRef value,
                     Flag flags = Flag::none,
                     u8 byval_align = 0,
                     u32 byval_size = 0)
        : value(value),
          flag(flags),
          byval_align(byval_align),
          byval_size(byval_size) {}

    IRValueRef value;
    Flag flag;
    u8 byval_align;
    u8 ext_bits = 0;
    u32 byval_size;
  };

  template <typename CBDerived>
  class CallBuilderBase {
  protected:
    Derived &compiler;
    CCAssigner &assigner;

    RegisterFile::RegBitSet arg_regs{};

  public:
    CallBuilderBase(Derived &compiler, CCAssigner &assigner) noexcept
        : compiler(compiler), assigner(assigner) {}

    // CBDerived needs:
    // void add_arg_byval(ValuePart &vp, CCAssignment &cca) noexcept;
    // void add_arg_stack(ValuePart &vp, CCAssignment &cca) noexcept;
    // void call_impl(std::variant<SymRef, ValuePart> &&) noexcept;
    CBDerived *derived() noexcept { return static_cast<CBDerived *>(this); }

    void add_arg(ValuePart &&vp, CCAssignment cca) noexcept;
    void add_arg(const CallArg &arg, u32 part_count) noexcept;
    void add_arg(const CallArg &arg) noexcept {
      add_arg(std::move(arg), compiler.adaptor->val_parts(arg.value).count());
    }

    // evict registers, do call, reset stack frame
    void call(std::variant<SymRef, ValuePart>) noexcept;

    void add_ret(ValuePart &vp, CCAssignment cca) noexcept;
    void add_ret(ValuePart &&vp, CCAssignment cca) noexcept {
      add_ret(vp, cca);
    }
    void add_ret(ValueRef &vr) noexcept;
  };

  class RetBuilder {
    Derived &compiler;
    CCAssigner &assigner;

    RegisterFile::RegBitSet ret_regs{};

  public:
    RetBuilder(Derived &compiler, CCAssigner &assigner) noexcept
        : compiler(compiler), assigner(assigner) {
      assigner.reset();
    }

    void add(ValuePart &&vp, CCAssignment cca) noexcept;
    void add(IRValueRef val) noexcept;

    void ret() noexcept;
  };

  /// Initialize a CompilerBase, should be called by the derived classes
  explicit CompilerBase(Adaptor *adaptor)
      : adaptor(adaptor), analyzer(adaptor), assembler() {
    static_assert(std::is_base_of_v<CompilerBase, Derived>);
    static_assert(Compiler<Derived, Config>);
  }

  /// shortcut for casting to the Derived class so that overloading
  /// works
  Derived *derived() { return static_cast<Derived *>(this); }

  const Derived *derived() const { return static_cast<const Derived *>(this); }

  [[nodiscard]] ValLocalIdx val_idx(const IRValueRef value) const noexcept {
    return analyzer.adaptor->val_local_idx(value);
  }

  [[nodiscard]] ValueAssignment *
      val_assignment(const ValLocalIdx idx) noexcept {
    return assignments.value_ptrs[static_cast<u32>(idx)];
  }

  /// Compile the functions returned by Adaptor::funcs
  ///
  /// \warning If you intend to call this multiple times, you must call reset
  ///   in-between the calls.
  ///
  /// \returns Whether the compilation was successful
  bool compile();

  /// Reset any leftover data from the previous compilation such that it will
  /// not affect the next compilation
  void reset();

#ifndef NDEBUG
private:
  VIR<Adaptor>::Allocation
      get_allocation(AssignmentPartRef ap,
                     const bool reload = false) const noexcept {
    // in reload_to_reg, register_valid can be true but the ap still needs to be
    // loaded
    if (ap.register_valid() && !reload) {
      return typename VIR<Adaptor>::Allocation(ap.get_reg());
    } else if (ap.stack_valid() && !ap.variable_ref()) {
      return typename VIR<Adaptor>::Allocation(ap.frame_off());
    }
    return typename VIR<Adaptor>::Allocation(Reg::make_invalid());
  }

  std::unordered_map<ValLocalIdx, std::vector<Reg>> final_assignments;
  // Value changed its register during codegen
  void vir_final_assignment(ValLocalIdx local_idx, Reg reg) noexcept {
    final_assignments[local_idx].push_back(reg);
  }

public:
  void vir_emit_new_use(Reg reg) {
    final_assignments[INVALID_VAL_LOCAL_IDX].push_back(reg);
  }
  void vir_emit_def(Reg reg) {
    verification_ir.materialize_constant(reg);
  }

private:
  /// Emit PHI node assignment (debug-only)
  void vir_emit_phi(IRValueRef phi_value, ValLocalIdx phi_idx, u32 part_idx, Reg reg) noexcept {
    using VIRType = VIR<Adaptor>;

    typename VIRType::Allocation phi_alloc(reg);
    ValueAssignment *assignment = val_assignment(phi_idx);
    if (assignment) {
      AssignmentPartRef ap{assignment, part_idx};
      phi_alloc = get_allocation(ap);
    }

    // Capture incoming values and their source blocks
    util::SmallVector<std::tuple<BlockIndex, ValLocalIdx, typename VIRType::Allocation>, 4> incoming_data;
    auto phi_ref = adaptor->val_as_phi(phi_value);
    u32 incoming_count = phi_ref.incoming_count();

    for (u32 i = 0; i < incoming_count; ++i) {
      IRBlockRef incoming_block = phi_ref.incoming_block_for_slot(i);
      IRValueRef incoming_val = phi_ref.incoming_val_for_slot(i);
      ValLocalIdx incoming_val_idx = INVALID_VAL_LOCAL_IDX;


      // Get allocation of incoming value at the source block
      auto incoming_block_idx =
          static_cast<BlockIndex>(analyzer.block_idx(incoming_block));
      typename VIRType::Allocation incoming_alloc(Reg::make_invalid());
      // consts or variable_refs
      if (adaptor->val_ignore_in_liveness_analysis(incoming_val)) {
        incoming_data.push_back(std::make_tuple(
            incoming_block_idx, INVALID_VAL_LOCAL_IDX, incoming_alloc));
        continue;
      }
      incoming_val_idx = adaptor->val_local_idx(incoming_val);
      // Try to get allocation from the assignment
      ValueAssignment *incoming_assignment = val_assignment(incoming_val_idx);
      if (incoming_assignment) {
        AssignmentPartRef ap{incoming_assignment, part_idx};
        incoming_alloc = get_allocation(ap);
      }

      incoming_data.push_back(std::make_tuple(incoming_block_idx, incoming_val_idx, incoming_alloc));
    }

    verification_ir.vir_emit_phi(phi_idx, part_idx, phi_alloc, incoming_data);
  }

public:
#endif

  /// Get CCAssigner for current function.
  CCAssigner *cur_cc_assigner() noexcept { return &default_cc_assigner; }

  void init_assignment(IRValueRef value, ValLocalIdx local_idx) noexcept;

private:
  /// Frees an assignment, its stack slot and registers
  void free_assignment(ValLocalIdx local_idx, ValueAssignment *) noexcept;

public:
  /// Release an assignment when reference count drops to zero, either frees
  /// the assignment immediately or delays free to the end of the live range.
  void release_assignment(ValLocalIdx local_idx, ValueAssignment *) noexcept;

  /// Init a variable-ref assignment
  void init_variable_ref(ValLocalIdx local_idx, u32 var_ref_data) noexcept;
  /// Init a variable-ref assignment
  void init_variable_ref(IRValueRef value, u32 var_ref_data) noexcept {
    init_variable_ref(adaptor->val_local_idx(value), var_ref_data);
  }

  i32 allocate_stack_slot(u32 size) noexcept;
  void free_stack_slot(u32 slot, u32 size) noexcept;

  template <typename Fn>
  void handle_func_arg(u32 arg_idx, IRValueRef arg, Fn add_arg) noexcept;

  ValueRef val_ref(IRValueRef value) noexcept;

  std::pair<ValueRef, ValuePartRef> val_ref_single(IRValueRef value) noexcept;

  /// Get a defining reference to a value
  ValueRef result_ref(IRValueRef value) noexcept;

  std::pair<ValueRef, ValuePartRef>
      result_ref_single(IRValueRef value) noexcept;

  /// Make dst an alias for src, which must be a non-constant value with an
  /// identical part configuration. src must be in its last use (is_owned()),
  /// and the assignment will be repurposed for dst, keeping all assigned
  /// registers and stack slots.
  ValueRef result_ref_alias(IRValueRef dst, ValueRef &&src) noexcept;

  /// Initialize value as a pointer into a stack variable (i.e., a value
  /// allocated from cur_static_allocas() or similar) with an offset. The
  /// result value will be a stack variable itself.
  ValueRef result_ref_stack_slot(IRValueRef value,
                                 AssignmentPartRef base,
                                 i32 off) noexcept;

  [[deprecated("Use ValuePartRef::set_value")]]
  void set_value(ValuePartRef &val_ref, ScratchReg &scratch) noexcept;
  [[deprecated("Use ValuePartRef::set_value")]]
  void set_value(ValuePartRef &&val_ref, ScratchReg &scratch) noexcept {
    set_value(val_ref, scratch);
  }

  /// Get generic value part into a single register, evaluating expressions
  /// and materializing immediates as required.
  AsmReg gval_as_reg(GenericValuePart &gv) noexcept;

  /// Like gval_as_reg; if the GenericValuePart owns a reusable register
  /// (either a ScratchReg, possibly due to materialization, or a reusable
  /// ValuePartRef), store it in dst.
  AsmReg gval_as_reg_reuse(GenericValuePart &gv, ScratchReg &dst) noexcept;

private:
  Reg select_reg_evict(RegBank bank, u64 exclusion_mask) noexcept;

public:
  /// Select an available register, evicting loaded values if needed.
  Reg select_reg(RegBank bank, u64 exclusion_mask) noexcept {
    Reg res = register_file.find_first_free_excluding(bank, exclusion_mask);
    if (res.valid()) [[likely]] {
      return res;
    }
    return select_reg_evict(bank, exclusion_mask);
  }

  /// Reload a value part from memory or recompute variable address.
  void reload_to_reg(AsmReg dst, AssignmentPartRef ap) noexcept;

  void allocate_spill_slot(AssignmentPartRef ap) noexcept;

  /// Ensure the value is spilled in its stack slot (except variable refs).
  void spill(AssignmentPartRef ap) noexcept;

  /// Evict the value from its register, spilling if needed, and free register.
  void evict(AssignmentPartRef ap) noexcept;

  /// Evict the value from the register, spilling if needed, and free register.
  void evict_reg(Reg reg) noexcept;

  /// Free the register. Requires that the contained value is already spilled.
  void free_reg(Reg reg) noexcept;

  // TODO(ts): switch to a branch_spill_before naming style?
  typename RegisterFile::RegBitSet
      spill_before_branch(bool force_spill = false) noexcept;
  void release_spilled_regs(typename RegisterFile::RegBitSet) noexcept;

  /// When reaching a point in the function where no other blocks will be
  /// reached anymore, use this function to release register assignments after
  /// the end of that block so the compiler does not accidentally use
  /// registers which don't contain any values
  void release_regs_after_return() noexcept;

  /// Generate a switch at the end of a basic block. Only the lowest bits of the
  /// condition are considered. The condition must be a general-purpose
  /// register. The cases must be sorted and every case value must appear at
  /// most once.
  void generate_switch(
      ScratchReg &&cond,
      u32 width,
      IRBlockRef default_block,
      std::span<const std::pair<u64, IRBlockRef>> cases) noexcept;

  /// Indicate beginning of region.
  void begin_branch_region() noexcept {
#ifndef NDEBUG
    assert(!generating_branch);
    generating_branch = true;
#endif
  }

  /// Indicate end of region.
  void end_branch_region() noexcept {
#ifndef NDEBUG
    assert(generating_branch);
    generating_branch = false;
    verification_ir.end_branch();
#endif
  }

#ifndef NDEBUG
  // todo(salto): double check that this is really always ok
  bool may_change_value_state() const noexcept { return true; }

#endif


  void move_one(u32 i, MoveList &moves, MoveList &result) noexcept {
    if (moves[i].src == moves[i].dst) {
      return;
    }
    moves[i].status = MoveStatus::MOVING;
    for (u32 j = 0; j < moves.size(); j++) {
      if (moves[j].src == moves[i].dst) {
        switch (moves[j].status) {
        case MoveStatus::TO_MOVE: {
          move_one(j, moves, result);
          break;
        }
        case MoveStatus::MOVING: {
          auto tmp = this->select_reg(register_file.reg_bank(moves[j].src), 0);
          // todo(salto): what if no reg is available, shouldn't happen, since
          // phis leave 2 free registers todo(salto): call derived->mov
          result.emplace_back(tmp,
                              moves[j].src,
                              moves[j].size,
                              moves[j].value_idx,
                              moves[j].part_idx);
          moves[j].src = tmp;
          break;
        }
        case MoveStatus::DONE: {
          // already done
          break;
        }
        }
      }
    }
    result.emplace_back(moves[i].dst, moves[i].src, moves[i].size,
                        moves[i].value_idx, moves[i].part_idx);
    moves[i].status = MoveStatus::DONE;
  }

  /*
  Order a list of moves in a way that they behave as if they are executed in
  parralel. Avoids swap problem and resolves dependency chains between moves.
  See: Silvain Rideau and Xavier Leroy. 2010. Validating register
  allocation and spilling.
  */
  MoveList sequentialize(MoveList &moves) noexcept {
      MoveList result;
    for (u32 i = 0; i < moves.size(); ++i) {
      if( moves[i].status == MoveStatus::TO_MOVE ) {
        move_one(i,moves,result);
      }
    }
    return result;
  }

  void move_values_to_match(BlockIndex target) noexcept {
    auto cur_block_ref = analyzer.block_ref(cur_block_idx);
    // next block immediately follows the current block and there is no control
    // flow inbetween. We can use the Register state of the current block for
    // the next one.
    // no moves necessary
    if (!analyzer.block_has_multiple_incoming(target) &&
        std::distance(adaptor->block_succs(cur_block_ref).begin(),
                      adaptor->block_succs(cur_block_ref).end()) == 1 &&
        analyzer.block_idx(*adaptor->block_succs(cur_block_ref).begin()) ==
            next_block()) {
      return;
    }
    MoveList moves;
    if (analyzer.block_has_phis(target)) {
      move_to_phi_nodes_impl(target,moves);
    }

    auto block_state_it = block_regs.find(target);
    // another branch to target was already generated, we *must* use the same register layout
    if (block_state_it != block_regs.end()) {
      for (ValueState &state : block_state_it->second) {
        ValueAssignment *va = val_assignment(state.val_local_idx);
        // value was already freed, won't be used again
        if (!va)
          continue;
        for (u32 i = 0; i < va->part_count; ++i) {
            AssignmentPartRef ap{va, i};
          auto cur_reg = ap.get_reg();
          if (!state.registers[i].valid()) {
            continue;
          }
          if (!ap.register_valid()) {
            // doesn't evict registers, so it is safe to call
            reload_to_reg(state.registers[i], ap);
          }
          moves.emplace_back(state.registers[i],
                               cur_reg,
                               ap.part_size(),
                               state.val_local_idx,
                               i);
          }
      }
    } else {
      // todo check for conflicting moves?

      std::unordered_set<ValLocalIdx> seen;
      for (auto reg : register_file.used_regs()) {
        auto local_idx = register_file.reg_local_idx(Reg{reg});
        if (local_idx == INVALID_VAL_LOCAL_IDX) {
          // scratch regs and constants can never be held across blocks
          // (outside of constants in phis, which are handled elsewhere)
          continue;
        }
        if (seen.contains(local_idx)) {
          continue;
        }
        seen.insert(local_idx);
        auto *assignment = val_assignment(local_idx);

        block_regs[target].emplace_back(
            local_idx, assignment->part_count);
        // save all parts of a value in sequence since they are stored as a
        // Vector
        for (u32 i = 0; i < assignment->part_count; i++) {
          AssignmentPartRef ap{val_assignment(local_idx), i};
          if (ap.fixed_assignment()) {
            // fixed registers do not need to be moved
            continue;
          }

          if (!ap.modified() || ap.variable_ref()) {
            // No need to spill values that were already spilled or are variable
            // refs.
            continue;
          }

          const auto &liveness = analyzer.liveness_info(local_idx);
          if (liveness.last < target) {
            // No need to save value if it dies before the target
            continue;
          }
          block_regs[target][block_regs[target].size()-1].push_back(Reg{ap.get_reg()},i);
        }

      }
    }

    MoveList result = sequentialize(moves);
    // todo(salto): maybe execute the mov in sequentialize directly
    for (auto move : result) {
      derived()->mov(move.dst,move.src,move.size);
    }
  }

  void move_to_phi_nodes_impl(BlockIndex target, MoveList& moves) noexcept;

  /// Count available registers in a specific bank
  u32 count_available_registers(RegBank bank) const noexcept {
    auto free_regs = register_file.allocatable & ~register_file.used &
                     register_file.bank_regs(bank);
    return std::popcount(free_regs);
  }

  /// Count total available registers across all banks
  u32 count_total_available_registers() const noexcept {
    auto free_regs = register_file.allocatable & ~register_file.used;
    return std::popcount(free_regs);
  }

  bool branch_needs_split(IRBlockRef target) noexcept {
    // for now, if the target has PHI-nodes, we split
    return analyzer.block_has_phis(target);
  }

  BlockIndex next_block() const noexcept;

  bool try_force_fixed_assignment(IRValueRef) const noexcept { return false; }

  bool hook_post_func_sym_init() noexcept { return true; }

  void analysis_start() noexcept {}

  void analysis_end() noexcept {}

  void reloc_text(SymRef sym, u32 type, u64 offset, i64 addend = 0) noexcept {
    this->assembler.reloc_sec(
        text_writer.get_sec_ref(), sym, type, offset, addend);
  }

  void label_place(Label label) noexcept {
    this->text_writer.label_place(label, text_writer.offset());
  }

protected:
  SymRef get_personality_sym() noexcept;

  bool compile_func(IRFuncRef func, u32 func_idx) noexcept;

  bool compile_block(IRBlockRef block, u32 block_idx) noexcept;
};
} // namespace tpde

#include "GenericValuePart.hpp"
#include "ScratchReg.hpp"
#include "ValuePartRef.hpp"
#include "ValueRef.hpp"

namespace tpde {

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
template <typename CBDerived>
void CompilerBase<Adaptor, Derived, Config>::CallBuilderBase<
    CBDerived>::add_arg(ValuePart &&vp, CCAssignment cca) noexcept {
  if (!cca.byval) {
    cca.bank = vp.bank();
    cca.size = vp.part_size();
  }

  assigner.assign_arg(cca);
  bool needs_ext = cca.int_ext != 0;
  bool ext_sign = cca.int_ext >> 7;
  unsigned ext_bits = cca.int_ext & 0x3f;

  if (cca.byval) {
    derived()->add_arg_byval(vp, cca);
    vp.reset(&compiler);
  } else if (!cca.reg.valid()) {
    if (needs_ext) {
      auto ext = std::move(vp).into_extended(&compiler, ext_sign, ext_bits, 64);
      derived()->add_arg_stack(ext, cca);
      ext.reset(&compiler);
    } else {
      derived()->add_arg_stack(vp, cca);
    }
    vp.reset(&compiler);
  } else {
    u32 size = vp.part_size();
    if (vp.is_in_reg(cca.reg)) {
      if (!vp.can_salvage()) {
        compiler.evict_reg(cca.reg);
      } else {
        vp.salvage(&compiler);
      }
      if (needs_ext) {
        compiler.generate_raw_intext(cca.reg, cca.reg, ext_sign, ext_bits, 64);
      }
    } else {
      if (compiler.register_file.is_used(cca.reg)) {
        compiler.evict_reg(cca.reg);
      }
      if (vp.can_salvage()) {
        AsmReg vp_reg = vp.salvage(&compiler);
#ifndef NDEBUG
        vp.assignment().set_reg(cca.reg); // ensure the arguments are registered
                                          // in the correct registers for VerificationIR
#endif
        if (needs_ext) {
          compiler.generate_raw_intext(cca.reg, vp_reg, ext_sign, ext_bits, 64);
        } else {
          compiler.mov(cca.reg, vp_reg, size);
        }
      } else {
        vp.reload_into_specific_fixed(&compiler, cca.reg);
        if (needs_ext) {
          compiler.generate_raw_intext(
              cca.reg, cca.reg, ext_sign, ext_bits, 64);
        }
      }
    }
    vp.reset(&compiler);
    assert(!compiler.register_file.is_used(cca.reg));
    compiler.register_file.mark_clobbered(cca.reg);
    compiler.register_file.allocatable &= ~(u64{1} << cca.reg.id());
    arg_regs |= (1ull << cca.reg.id());
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
template <typename CBDerived>
void CompilerBase<Adaptor, Derived, Config>::CallBuilderBase<
    CBDerived>::add_arg(const CallArg &arg, u32 part_count) noexcept {
  ValueRef vr = compiler.val_ref(arg.value);

  if (arg.flag == CallArg::Flag::byval) {
    assert(part_count == 1);
    add_arg(vr.part(0),
            CCAssignment{
                .byval = true,
                .align = arg.byval_align,
                .size = arg.byval_size,
            });
    return;
  }

  u32 align = 1;
  bool consecutive = false;
  u32 consec_def = 0;
  if (compiler.arg_is_int128(arg.value)) {
    // TODO: this also applies to composites with 16-byte alignment
    align = 16;
    consecutive = true;
  } else if (part_count > 1 &&
             !compiler.arg_allow_split_reg_stack_passing(arg.value)) {
    consecutive = true;
    if (part_count > UINT8_MAX) {
      // Must be completely passed on the stack.
      consecutive = false;
      consec_def = -1;
    }
  }

  for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
    u8 int_ext = 0;
    if (arg.flag == CallArg::Flag::sext || arg.flag == CallArg::Flag::zext) {
      assert(arg.ext_bits != 0 && "cannot extend zero-bit integer");
      int_ext = arg.ext_bits | (arg.flag == CallArg::Flag::sext ? 0x80 : 0);
    }
    derived()->add_arg(
        vr.part(part_idx),
        CCAssignment{
            .consecutive =
                u8(consecutive ? part_count - part_idx - 1 : consec_def),
            .sret = arg.flag == CallArg::Flag::sret,
            .int_ext = int_ext,
            .align = u8(part_idx == 0 ? align : 1),
        });
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
template <typename CBDerived>
void CompilerBase<Adaptor, Derived, Config>::CallBuilderBase<CBDerived>::call(
    std::variant<SymRef, ValuePart> target) noexcept {
  assert(!compiler.stack.is_leaf_function && "leaf func must not have calls");
  compiler.stack.generated_call = true;
  typename RegisterFile::RegBitSet skip_evict = arg_regs;
  if (auto *vp = std::get_if<ValuePart>(&target); vp && vp->can_salvage()) {
    // call_impl will reset vp, thereby unlock+free the register.
    assert(vp->cur_reg_unlocked().valid() && "can_salvage implies register");
    skip_evict |= (1ull << vp->cur_reg_unlocked().id());
  }

  auto clobbered = ~assigner.get_ccinfo().callee_saved_regs;
  for (auto reg_id : util::BitSetIterator<>{compiler.register_file.used &
                                            clobbered & ~skip_evict}) {
    compiler.evict_reg(AsmReg{reg_id});
    compiler.register_file.mark_clobbered(Reg{reg_id});
  }

  derived()->call_impl(std::move(target));

  assert((compiler.register_file.allocatable & arg_regs) == 0);
  compiler.register_file.allocatable |= arg_regs;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
template <typename CBDerived>
void CompilerBase<Adaptor, Derived, Config>::CallBuilderBase<
    CBDerived>::add_ret(ValuePart &vp, CCAssignment cca) noexcept {
  cca.bank = vp.bank();
  cca.size = vp.part_size();
  assigner.assign_ret(cca);
  assert(cca.reg.valid() && "return value must be in register");
  vp.set_value_reg(&compiler, cca.reg);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
template <typename CBDerived>
void CompilerBase<Adaptor, Derived, Config>::CallBuilderBase<
    CBDerived>::add_ret(ValueRef &vr) noexcept {
  assert(vr.has_assignment());
  u32 part_count = vr.assignment()->part_count;
  for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
    CCAssignment cca;
    add_ret(vr.part(part_idx), cca);
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::RetBuilder::add(
    ValuePart &&vp, CCAssignment cca) noexcept {
  cca.bank = vp.bank();
  u32 size = cca.size = vp.part_size();
  assigner.assign_ret(cca);
  assert(cca.reg.valid() && "indirect return value must use sret argument");

  bool needs_ext = cca.int_ext != 0;
  bool ext_sign = cca.int_ext >> 7;
  unsigned ext_bits = cca.int_ext & 0x3f;

  if (vp.is_in_reg(cca.reg)) {
    if (!vp.can_salvage()) {
      compiler.evict_reg(cca.reg);
    } else {
      vp.salvage(&compiler);
    }
    if (needs_ext) {
      compiler.generate_raw_intext(cca.reg, cca.reg, ext_sign, ext_bits, 64);
    }
  } else {
    if (compiler.register_file.is_used(cca.reg)) {
      compiler.evict_reg(cca.reg);
    }
    if (vp.can_salvage()) {
      AsmReg vp_reg = vp.salvage(&compiler);
      if (needs_ext) {
        compiler.generate_raw_intext(cca.reg, vp_reg, ext_sign, ext_bits, 64);
      } else {
        compiler.mov(cca.reg, vp_reg, size);
      }
    } else {
      vp.reload_into_specific_fixed(&compiler, cca.reg);
      if (needs_ext) {
        compiler.generate_raw_intext(cca.reg, cca.reg, ext_sign, ext_bits, 64);
      }
    }
  }
  vp.reset(&compiler);
  assert(!compiler.register_file.is_used(cca.reg));
  compiler.register_file.mark_clobbered(cca.reg);
  compiler.register_file.allocatable &= ~(u64{1} << cca.reg.id());
  ret_regs |= (1ull << cca.reg.id());
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::RetBuilder::add(
    IRValueRef val) noexcept {
  u32 part_count = compiler.adaptor->val_parts(val).count();
  ValueRef vr = compiler.val_ref(val);
  for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
    add(vr.part(part_idx), CCAssignment{});
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::RetBuilder::ret() noexcept {
  assert((compiler.register_file.allocatable & ret_regs) == 0);
  compiler.register_file.allocatable |= ret_regs;

  compiler.gen_func_epilog();
  compiler.release_regs_after_return();
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
bool CompilerBase<Adaptor, Derived, Config>::compile() {
  // create function symbols
  text_writer.switch_section(
      assembler.get_section(assembler.get_text_section()));

  assert(func_syms.empty());
  for (const IRFuncRef func : adaptor->funcs()) {
    auto binding = Assembler::SymBinding::GLOBAL;
    if (adaptor->func_has_weak_linkage(func)) {
      binding = Assembler::SymBinding::WEAK;
    } else if (adaptor->func_only_local(func)) {
      binding = Assembler::SymBinding::LOCAL;
    }
    if (adaptor->func_extern(func)) {
      func_syms.push_back(derived()->assembler.sym_add_undef(
          adaptor->func_link_name(func), binding));
    } else {
      func_syms.push_back(derived()->assembler.sym_predef_func(
          adaptor->func_link_name(func), binding));
    }
    derived()->define_func_idx(func, func_syms.size() - 1);
  }

  if (!derived()->hook_post_func_sym_init()) {
    TPDE_LOG_ERR("hook_pust_func_sym_init failed");
    return false;
  }

  // TODO(ts): create function labels?

  bool success = true;

  u32 func_idx = 0;
  for (const IRFuncRef func : adaptor->funcs()) {
    if (adaptor->func_extern(func)) {
      TPDE_LOG_TRACE("Skipping compilation of func {}",
                     adaptor->func_link_name(func));
      ++func_idx;
      continue;
    }

    TPDE_LOG_TRACE("Compiling func {}", adaptor->func_link_name(func));
    if (!derived()->compile_func(func, func_idx)) {
      TPDE_LOG_ERR("Failed to compile function {}",
                   adaptor->func_link_name(func));
      success = false;
    }
    ++func_idx;
  }

  text_writer.flush();
  assembler.finalize();

  // TODO(ts): generate object/map?

  return success;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::reset() {
  adaptor->reset();

  for (auto &e : stack.fixed_free_lists) {
    e.clear();
  }
  stack.dynamic_free_lists.clear();

  assembler.reset();
  func_syms.clear();
  block_labels.clear();
  personality_syms.clear();
#ifndef NDEBUG
  verification_ir.reset();
#endif
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::init_assignment(
    IRValueRef value, ValLocalIdx local_idx) noexcept {
  assert(val_assignment(local_idx) == nullptr);
  TPDE_LOG_TRACE("Initializing assignment for value {}",
                 static_cast<u32>(local_idx));

  const auto parts = adaptor->val_parts(value);
  const u32 part_count = parts.count();
  assert(part_count > 0);
  auto *assignment = assignments.allocator.allocate(part_count);
  assignments.value_ptrs[static_cast<u32>(local_idx)] = assignment;

  u32 max_part_size = 0;
  for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
    auto ap = AssignmentPartRef{assignment, part_idx};
    ap.reset();
    ap.set_bank(parts.reg_bank(part_idx));
    const u32 size = parts.size_bytes(part_idx);
    assert(size > 0);
    max_part_size = std::max(max_part_size, size);
    ap.set_part_size(size);
    // todo(salto): multi-part values
    if (part_idx == 0) {
      auto reg = analyzer.get_recommended_reg(local_idx);
      if (reg.valid()) {
        ap.set_reg(reg, true);
      }
    }
  }

  const auto &liveness = analyzer.liveness_info(local_idx);

  // if there is only one part, try to hand out a fixed assignment
  // if the value is used for longer than one block and there aren't too many
  // definitions in child loops this could interfere with
  // TODO(ts): try out only fixed assignments if the value is live for more
  // than two blocks?
  // TODO(ts): move this to ValuePartRef::alloc_reg to be able to defer this
  // for results?
  if (part_count == 1) {
    const auto &cur_loop =
        analyzer.loop_from_idx(analyzer.block_loop_idx(cur_block_idx));
    auto ap = AssignmentPartRef{assignment, 0};

    auto try_fixed =
        liveness.last > cur_block_idx &&
        cur_loop.definitions_in_childs +
                assignments.cur_fixed_assignment_count[ap.bank().id()] <
            Derived::NUM_FIXED_ASSIGNMENTS[ap.bank().id()];
    if (derived()->try_force_fixed_assignment(value)) {
      try_fixed = assignments.cur_fixed_assignment_count[ap.bank().id()] <
                  Derived::NUM_FIXED_ASSIGNMENTS[ap.bank().id()];
    }

    if (try_fixed) {
      // check if there is a fixed register available
      AsmReg reg = derived()->select_fixed_assignment_reg(ap, value);
      TPDE_LOG_TRACE("Trying to assign fixed reg to value {}",
                     static_cast<u32>(local_idx));

      // TODO: if the register is used, we can free it most of the time, but not
      // always, e.g. for PHI nodes. Detect this case and free_reg otherwise.
      if (!reg.invalid() && !register_file.is_used(reg)) {
        TPDE_LOG_TRACE("Assigning fixed assignment to reg {} for value {}",
                       reg.id(),
                       static_cast<u32>(local_idx));
        ap.set_reg(reg);
        ap.set_register_valid(true);
        ap.set_fixed_assignment(true);
        register_file.mark_used(reg, local_idx, 0);
        register_file.inc_lock_count(reg); // fixed assignments always locked
        register_file.mark_clobbered(reg);
        ++assignments.cur_fixed_assignment_count[ap.bank().id()];
      }
    }
  }

  const auto last_full = liveness.last_full;
  const auto ref_count = liveness.ref_count;

  assert(max_part_size <= 256);
  assignment->max_part_size = max_part_size;
  assignment->pending_free = false;
  assignment->variable_ref = false;
  assignment->stack_variable = false;
  assignment->delay_free = last_full;
  assignment->part_count = part_count;
  assignment->frame_off = 0;
  assignment->references_left = ref_count;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::free_assignment(
    ValLocalIdx local_idx, ValueAssignment *assignment) noexcept {
  TPDE_LOG_TRACE("Freeing assignment for value {}",
                 static_cast<u32>(local_idx));

  assert(assignments.value_ptrs[static_cast<u32>(local_idx)] == assignment);
  assignments.value_ptrs[static_cast<u32>(local_idx)] = nullptr;
  const auto is_var_ref = assignment->variable_ref;
  const u32 part_count = assignment->part_count;

  // free registers
  for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
    auto ap = AssignmentPartRef{assignment, part_idx};
#ifndef NDEBUG
    // we need to capture the register here, since it will be freed and is no
    // longer
    // available after compile_inst
    vir_final_assignment(local_idx, ap.get_reg());
    #endif
    if (ap.fixed_assignment()) [[unlikely]] {
      const auto reg = ap.get_reg();
      assert(register_file.is_fixed(reg));
      assert(register_file.reg_local_idx(reg) == local_idx);
      assert(register_file.reg_part(reg) == part_idx);
      --assignments.cur_fixed_assignment_count[ap.bank().id()];
      register_file.dec_lock_count_must_zero(reg); // release lock for fixed reg
      register_file.unmark_used(reg);
    } else if (ap.register_valid()) {
      const auto reg = ap.get_reg();
      assert(!register_file.is_fixed(reg));
      register_file.unmark_used(reg);
    }
  }

  // Also free any registers in register_file that are marked for this value
  // but don't have register_valid set (e.g., temporary phi registers)
  for (auto reg_id : register_file.used_regs()) {
    if (register_file.reg_local_idx(Reg{reg_id}) == local_idx) {
      Reg reg{reg_id};
      if (!register_file.is_fixed(reg)) {
        TPDE_LOG_TRACE("Freeing temporary register {} for value {}",
                       reg_id,
                       static_cast<u32>(local_idx));
        register_file.unmark_used(reg);
      }
    }
  }

#ifdef TPDE_ASSERTS
  for (auto reg_id : register_file.used_regs()) {
    assert(register_file.reg_local_idx(AsmReg{reg_id}) != local_idx &&
           "freeing assignment that is still referenced by a register");
  }
#endif

  // variable references do not have a stack slot
  if (!is_var_ref && assignment->frame_off != 0) {
    free_stack_slot(assignment->frame_off, assignment->size());
  }

  assignments.allocator.deallocate(assignment);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
[[gnu::noinline]] void
    CompilerBase<Adaptor, Derived, Config>::release_assignment(
        ValLocalIdx local_idx, ValueAssignment *assignment) noexcept {
  TPDE_LOG_TRACE("Releasing assignment for value {}", static_cast<u32>(local_idx));
  if (!assignment->delay_free) {
    free_assignment(local_idx, assignment);
    return;
  }

  // need to wait until release
  TPDE_LOG_TRACE("Delay freeing assignment for value {}",
                 static_cast<u32>(local_idx));
  const auto &liveness = analyzer.liveness_info(local_idx);
  auto &free_list_head = assignments.delayed_free_lists[u32(liveness.last)];
  assignment->next_delayed_free_entry = free_list_head;
  assignment->pending_free = true;
  free_list_head = local_idx;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::init_variable_ref(
    ValLocalIdx local_idx, u32 var_ref_data) noexcept {
  TPDE_LOG_TRACE("Initializing variable-ref assignment for value {}",
                 static_cast<u32>(local_idx));

  assert(val_assignment(local_idx) == nullptr);
  auto *assignment = assignments.allocator.allocate_slow(1, true);
  assignments.value_ptrs[static_cast<u32>(local_idx)] = assignment;

  assignment->max_part_size = Config::PLATFORM_POINTER_SIZE;
  assignment->variable_ref = true;
  assignment->stack_variable = false;
  assignment->part_count = 1;
  assignment->var_ref_custom_idx = var_ref_data;
  assignment->next_delayed_free_entry = assignments.variable_ref_list;

  assignments.variable_ref_list = local_idx;

  AssignmentPartRef ap{assignment, 0};
  ap.reset();
  ap.set_bank(Config::GP_BANK);
  ap.set_part_size(Config::PLATFORM_POINTER_SIZE);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
i32 CompilerBase<Adaptor, Derived, Config>::allocate_stack_slot(
    u32 size) noexcept {
  unsigned align_bits = 4;
  if (size == 0) {
    return 0; // 0 is the "invalid" stack slot
  } else if (size <= 16) {
    // Align up to next power of two.
    u32 free_list_idx = size == 1 ? 0 : 32 - util::cnt_lz<u32>(size - 1);
    assert(size <= 1u << free_list_idx);
    size = 1 << free_list_idx;
    align_bits = free_list_idx;

    if (!stack.fixed_free_lists[free_list_idx].empty()) {
      auto slot = stack.fixed_free_lists[free_list_idx].back();
      stack.fixed_free_lists[free_list_idx].pop_back();
      return slot;
    }
  } else {
    size = util::align_up(size, 16);
    auto it = stack.dynamic_free_lists.find(size);
    if (it != stack.dynamic_free_lists.end() && !it->second.empty()) {
      const auto slot = it->second.back();
      it->second.pop_back();
      return slot;
    }
  }

  assert(stack.frame_size != ~0u &&
         "cannot allocate stack slot before stack frame is initialized");

  // Align frame_size to align_bits
  for (u32 list_idx = util::cnt_tz(stack.frame_size); list_idx < align_bits;
       list_idx = util::cnt_tz(stack.frame_size)) {
    i32 slot = stack.frame_size;
    if constexpr (Config::FRAME_INDEXING_NEGATIVE) {
      slot = -(slot + (1ull << list_idx));
    }
    stack.fixed_free_lists[list_idx].push_back(slot);
    stack.frame_size += 1ull << list_idx;
  }

  auto slot = stack.frame_size;
  assert(slot != 0 && "stack slot 0 is reserved");
  stack.frame_size += size;

  if constexpr (Config::FRAME_INDEXING_NEGATIVE) {
    slot = -(slot + size);
  }
  return slot;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::free_stack_slot(
    u32 slot, u32 size) noexcept {
  if (size == 0) [[unlikely]] {
    assert(slot == 0 && "unexpected slot for zero-sized stack-slot?");
    // Do nothing.
  } else if (size <= 16) [[likely]] {
    u32 free_list_idx = size == 1 ? 0 : 32 - util::cnt_lz<u32>(size - 1);
    stack.fixed_free_lists[free_list_idx].push_back(slot);
  } else {
    size = util::align_up(size, 16);
    stack.dynamic_free_lists[size].push_back(slot);
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
template <typename Fn>
void CompilerBase<Adaptor, Derived, Config>::handle_func_arg(
    u32 arg_idx, IRValueRef arg, Fn add_arg) noexcept {
  ValueRef vr = derived()->result_ref(arg);
  if (adaptor->cur_arg_is_byval(arg_idx)) {
    std::optional<i32> byval_frame_off =
        add_arg(vr.part(0),
                CCAssignment{
                    .byval = true,
                    .align = u8(adaptor->cur_arg_byval_align(arg_idx)),
                    .size = adaptor->cur_arg_byval_size(arg_idx),
                });

    if (byval_frame_off) {
      // We need to convert the assignment into a stack variable ref.
      ValLocalIdx local_idx = val_idx(arg);
      // TODO: we shouldn't create the result_ref for such cases in the first
      // place. However, this is not easy to detect up front, it depends on the
      // target and the calling convention whether this is possible.
      vr.reset();
      // Value assignment might have been free'd by ValueRef reset.
      if (ValueAssignment *assignment = val_assignment(local_idx)) {
        free_assignment(local_idx, assignment);
      }
      init_variable_ref(local_idx, 0);
      ValueAssignment *assignment = this->val_assignment(local_idx);
      assignment->stack_variable = true;
      assignment->frame_off = *byval_frame_off;
    }
    return;
  }

  if (adaptor->cur_arg_is_sret(arg_idx)) {
    add_arg(vr.part(0), CCAssignment{.sret = true});
    return;
  }

  const u32 part_count = vr.assignment()->part_count;

  u32 align = 1;
  u32 consecutive = 0;
  u32 consec_def = 0;
  if (derived()->arg_is_int128(arg)) {
    // TODO: this also applies to composites with 16-byte alignment
    align = 16;
    consecutive = 1;
  } else if (part_count > 1 &&
             !derived()->arg_allow_split_reg_stack_passing(arg)) {
    consecutive = 1;
    if (part_count > UINT8_MAX) {
      // Must be completely passed on the stack.
      consecutive = 0;
      consec_def = -1;
    }
  }

  for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
    add_arg(vr.part(part_idx),
            CCAssignment{
                .consecutive =
                    u8(consecutive ? part_count - part_idx - 1 : consec_def),
                .align = u8(part_idx == 0 ? align : 1),
            });
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::ValueRef
    CompilerBase<Adaptor, Derived, Config>::val_ref(IRValueRef value) noexcept {
  if (auto special = derived()->val_ref_special(value); special) {
    return ValueRef{this, std::move(*special)};
  }

  const ValLocalIdx local_idx = analyzer.adaptor->val_local_idx(value);
  assert(val_assignment(local_idx) != nullptr && "value use before def");
  return ValueRef{this, local_idx};
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
std::pair<typename CompilerBase<Adaptor, Derived, Config>::ValueRef,
          typename CompilerBase<Adaptor, Derived, Config>::ValuePartRef>
    CompilerBase<Adaptor, Derived, Config>::val_ref_single(
        IRValueRef value) noexcept {
  std::pair<ValueRef, ValuePartRef> res{val_ref(value), this};
  res.second = res.first.part(0);
  return res;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::ValueRef
    CompilerBase<Adaptor, Derived, Config>::result_ref(
        IRValueRef value) noexcept {
  const ValLocalIdx local_idx = analyzer.adaptor->val_local_idx(value);
  if (val_assignment(local_idx) == nullptr) {
    init_assignment(value, local_idx);
  }
  return ValueRef{this, local_idx};
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
std::pair<typename CompilerBase<Adaptor, Derived, Config>::ValueRef,
          typename CompilerBase<Adaptor, Derived, Config>::ValuePartRef>
    CompilerBase<Adaptor, Derived, Config>::result_ref_single(
        IRValueRef value) noexcept {
  std::pair<ValueRef, ValuePartRef> res{result_ref(value), this};
  res.second = res.first.part(0);
  return res;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::ValueRef
    CompilerBase<Adaptor, Derived, Config>::result_ref_alias(
        IRValueRef dst, ValueRef &&src) noexcept {
  const ValLocalIdx local_idx = analyzer.adaptor->val_local_idx(dst);
  assert(!val_assignment(local_idx) && "alias target already defined");
  assert(src.has_assignment() && "alias src must have an assignment");
  // Consider implementing aliases where multiple values share the same
  // assignment, e.g. for implementing trunc as a no-op sharing registers. Live
  // ranges can be merged and reference counts added. However, this needs
  // additional infrastructure to clear one of the value_ptrs.
  assert(src.is_owned() && "alias src must be owned");

  ValueAssignment *assignment = src.assignment();
  u32 part_count = assignment->part_count;
  assert(!assignment->pending_free);
  assert(!assignment->variable_ref);
  assert(!assignment->pending_free);
#ifndef NDEBUG
  {
    const auto &src_liveness = analyzer.liveness_info(src.local_idx());
    assert(!src_liveness.last_full);          // implied by is_owned()
    assert(assignment->references_left == 1); // implied by is_owned()

    // Validate that part configuration is identical.
    const auto parts = adaptor->val_parts(dst);
    assert(parts.count() == part_count);
    for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
      AssignmentPartRef ap{assignment, part_idx};
      assert(parts.reg_bank(part_idx) == ap.bank());
      assert(parts.size_bytes(part_idx) == ap.part_size());
    }
  }
#endif

  // Update local_idx of registers.
  for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
    AssignmentPartRef ap{assignment, part_idx};
    if (ap.register_valid()) {
      register_file.update_reg_assignment(ap.get_reg(), local_idx, part_idx);
    }
  }

  const auto &liveness = analyzer.liveness_info(local_idx);
  assignment->delay_free = liveness.last_full;
  assignment->references_left = liveness.ref_count;
  assignments.value_ptrs[static_cast<u32>(src.local_idx())] = nullptr;
  assignments.value_ptrs[static_cast<u32>(local_idx)] = assignment;
  src.disown();
  src.reset();

  return ValueRef{this, local_idx};
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::ValueRef
    CompilerBase<Adaptor, Derived, Config>::result_ref_stack_slot(
        IRValueRef dst, AssignmentPartRef base, i32 off) noexcept {
  const ValLocalIdx local_idx = analyzer.adaptor->val_local_idx(dst);
  assert(!val_assignment(local_idx) && "new value already defined");
  init_variable_ref(local_idx, 0);
  ValueAssignment *assignment = this->val_assignment(local_idx);
  assignment->stack_variable = true;
  assignment->frame_off = base.variable_stack_off() + off;
  return ValueRef{this, local_idx};
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::set_value(
    ValuePartRef &val_ref, ScratchReg &scratch) noexcept {
  val_ref.set_value(std::move(scratch));
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::AsmReg
    CompilerBase<Adaptor, Derived, Config>::gval_as_reg(
        GenericValuePart &gv) noexcept {
  if (std::holds_alternative<ScratchReg>(gv.state)) {
    return std::get<ScratchReg>(gv.state).cur_reg();
  }
  if (std::holds_alternative<ValuePartRef>(gv.state)) {
    auto &vpr = std::get<ValuePartRef>(gv.state);
    if (vpr.has_reg()) {
      return vpr.cur_reg();
    }
    return vpr.load_to_reg();
  }
  if (auto *expr = std::get_if<typename GenericValuePart::Expr>(&gv.state)) {
    if (expr->has_base() && !expr->has_index() && expr->disp == 0) {
      return expr->base_reg();
    }
    return derived()->gval_expr_as_reg(gv);
  }
  TPDE_UNREACHABLE("gval_as_reg on empty GenericValuePart");
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::AsmReg
    CompilerBase<Adaptor, Derived, Config>::gval_as_reg_reuse(
        GenericValuePart &gv, ScratchReg &dst) noexcept {
  AsmReg reg = gval_as_reg(gv);
  if (!dst.has_reg()) {
    if (auto *scratch = std::get_if<ScratchReg>(&gv.state)) {
      dst = std::move(*scratch);
    } else if (auto *val_ref = std::get_if<ValuePartRef>(&gv.state)) {
      if (val_ref->can_salvage()) {
        dst.alloc_specific(val_ref->salvage());
        assert(dst.cur_reg() == reg && "salvaging unsuccessful");
      }
    }
  }
  return reg;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
Reg CompilerBase<Adaptor, Derived, Config>::select_reg_evict(
    RegBank bank, u64 exclusion_mask) noexcept {
  TPDE_LOG_DBG("select_reg_evict for bank {}", bank.id());
  auto candidates =
      register_file.used & register_file.bank_regs(bank) & ~exclusion_mask;

  Reg candidate = Reg::make_invalid();
  u32 max_score = 0;
  for (auto reg_id : util::BitSetIterator<>(candidates)) {
    Reg reg{reg_id};
    if (register_file.is_fixed(reg)) {
      continue;
    }

    // Must be an evictable value, not a temporary.
    auto local_idx = register_file.reg_local_idx(reg);
    u32 part = register_file.reg_part(Reg{reg});
    assert(local_idx != INVALID_VAL_LOCAL_IDX);
    ValueAssignment *va = val_assignment(local_idx);
    AssignmentPartRef ap{va, part};

    // We want to sort registers by the following (ordered by priority):
    // - stack variable ref (~1 add/sub to reconstruct)
    // - other variable ref (1-2 instrs to reconstruct)
    // - already spilled (no store needed)
    // - last use farthest away (most likely to get spilled anyhow, so there's
    //   not much harm in spilling earlier)
    // - lowest ref-count (least used)
    //
    // TODO: evaluate and refine this heuristic

    // TODO: evict stack variable refs before others
    if (ap.variable_ref()) {
      TPDE_LOG_DBG("  r{} ({}) is variable-ref", reg_id, u32(local_idx));
      candidate = reg;
      break;
    }

    u32 score = 0;
    if (ap.stack_valid()) {
      score |= u32{1} << 31;
    }

    const auto &liveness = analyzer.liveness_info(local_idx);
    u32 last_use_dist = u32(liveness.last) - u32(cur_block_idx);
    score |= (last_use_dist < 0x8000 ? 0x8000 - last_use_dist : 0) << 16;

    u32 refs_left = va->pending_free ? 0 : va->references_left;
    score |= (refs_left < 0xffff ? 0x10000 - refs_left : 1);

    TPDE_LOG_DBG("  r{} ({}:{}) rc={}/{} live={}-{}{} spilled={} score={:#x}",
                 reg_id,
                 u32(local_idx),
                 part,
                 refs_left,
                 liveness.ref_count,
                 u32(liveness.first),
                 u32(liveness.last),
                 &"*"[!liveness.last_full],
                 ap.stack_valid(),
                 score);

    assert(score != 0);
    if (score > max_score) {
      candidate = reg;
      max_score = score;
    }
  }
  if (candidate.invalid()) [[unlikely]] {
    TPDE_FATAL("ran out of registers for scratch registers");
  }
  TPDE_LOG_DBG("  selected r{}", candidate.id());
  evict_reg(candidate);
  return candidate;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::reload_to_reg(
    AsmReg dst, AssignmentPartRef ap) noexcept {
#ifndef NDEBUG
  // forcefully get stack allocation
  typename VIR<Adaptor>::Allocation from = get_allocation(ap,true);
#endif
  if (!ap.variable_ref()) {
    assert(ap.stack_valid());
    derived()->load_from_stack(dst, ap.frame_off(), ap.part_size());
  } else if (ap.is_stack_variable()) {
    derived()->load_address_of_stack_var(dst, ap);
  } else if constexpr (!Config::DEFAULT_VAR_REF_HANDLING) {
    derived()->load_address_of_var_reference(dst, ap);
  } else {
    TPDE_UNREACHABLE("non-stack-variable needs custom var-ref handling");
  }
#ifndef NDEBUG
  typename VIR<Adaptor>::Allocation to(dst);
  // After reload, the register should be marked in register_file
  ValLocalIdx val_idx = INVALID_VAL_LOCAL_IDX;
  u32 part_idx = 0;

  // Check if dst register is marked as used before querying
  if (register_file.is_used(dst)) {
    val_idx = register_file.reg_local_idx(dst);
    part_idx = register_file.reg_part(dst);
  }

  // If not in register_file yet, try to get from ap if it's valid
  if (val_idx == INVALID_VAL_LOCAL_IDX && ap.register_valid() &&
      register_file.is_used(ap.get_reg())) {
    val_idx = register_file.reg_local_idx(ap.get_reg());
    part_idx = register_file.reg_part(ap.get_reg());
  }
  // If still invalid, search assignments (fallback)
  if (val_idx == INVALID_VAL_LOCAL_IDX) {
    for (u32 i = 0; i < assignments.value_ptrs.size(); ++i) {
      if (assignments.value_ptrs[i] == ap.assignment()) {
        val_idx = ValLocalIdx(i);
        // Find part index by checking all parts
        ValueAssignment *va = ap.assignment();
        for (u32 p = 0; p < va->part_count; ++p) {
          AssignmentPartRef test_ap{va, p};
          if (test_ap.get_reg() == ap.get_reg() ||
              (test_ap.stack_valid() && ap.stack_valid() &&
               test_ap.frame_off() == ap.frame_off())) {
            part_idx = p;
            break;
          }
        }
        break;
      }
    }
    if (val_idx != INVALID_VAL_LOCAL_IDX) {
      verification_ir.emit_edit(VIR<Adaptor>::EditKind::Reload, from, to, val_idx, part_idx, ap.part_size());
    }
  } else {
    verification_ir.emit_edit(VIR<Adaptor>::EditKind::Reload,
                              from,
                              to,
                              val_idx,
                              part_idx,
                              ap.part_size());
  }
#endif
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::allocate_spill_slot(
    AssignmentPartRef ap) noexcept {
  assert(!ap.variable_ref() && "cannot allocate spill slot for variable ref");
  if (ap.assignment()->frame_off == 0) {
    assert(!ap.stack_valid() && "stack-valid set without spill slot");
    ap.assignment()->frame_off = allocate_stack_slot(ap.assignment()->size());
    assert(ap.assignment()->frame_off != 0);
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::spill(
    AssignmentPartRef ap) noexcept {
  assert(may_change_value_state());
  if (!ap.stack_valid() && !ap.variable_ref()) {
    assert(ap.register_valid() && "cannot spill uninitialized assignment part");
    allocate_spill_slot(ap);
    derived()->spill_reg(ap.get_reg(), ap.frame_off(), ap.part_size());
    ap.set_stack_valid();
#ifndef NDEBUG
{
    typename VIR<Adaptor>::Allocation from =
        get_allocation(ap);
      ValLocalIdx val_idx = register_file.reg_local_idx(ap.get_reg());
      if (val_idx != INVALID_VAL_LOCAL_IDX) {
        typename VIR<Adaptor>::Allocation to(ap.frame_off());
        u32 part_idx = register_file.reg_part(ap.get_reg());
        verification_ir.emit_edit(
            VIR<Adaptor>::EditKind::Spill, from, to, val_idx, part_idx, ap.part_size());
      }
    }
#endif
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::evict(
    AssignmentPartRef ap) noexcept {
  assert(may_change_value_state());
  assert(ap.register_valid());
  derived()->spill(ap);
  ap.set_register_valid(false);
  register_file.unmark_used(ap.get_reg());
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::evict_reg(Reg reg) noexcept {
  assert(may_change_value_state());
  assert(!register_file.is_fixed(reg));
  assert(register_file.reg_local_idx(reg) != INVALID_VAL_LOCAL_IDX);

  ValLocalIdx local_idx = register_file.reg_local_idx(reg);
  auto part = register_file.reg_part(reg);
  AssignmentPartRef evict_part{val_assignment(local_idx), part};

  // Handle case where register is marked as used but assignment doesn't have
  // register_valid set (can happen for temporary phi registers during move
  // resolution)
  if (!evict_part.register_valid() ) {
    // Just unmark the register without spilling
    register_file.unmark_used(reg);
    return;
  }

  assert(evict_part.get_reg() == reg);
  derived()->spill(evict_part);
  evict_part.set_register_valid(false);
  register_file.unmark_used(reg);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::free_reg(Reg reg) noexcept {
  assert(may_change_value_state());
  assert(!register_file.is_fixed(reg));
  assert(register_file.reg_local_idx(reg) != INVALID_VAL_LOCAL_IDX);

  ValLocalIdx local_idx = register_file.reg_local_idx(reg);
  auto part = register_file.reg_part(reg);
  AssignmentPartRef ap{val_assignment(local_idx), part};
  assert(ap.register_valid());
  assert(ap.get_reg() == reg);
  assert(!ap.modified() || ap.variable_ref());
  ap.set_register_valid(false);
  register_file.unmark_used(reg);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::RegisterFile::RegBitSet
    CompilerBase<Adaptor, Derived, Config>::  spill_before_branch(
        bool force_spill) noexcept {
  // since we do not explicitly keep track of register assignments per block,
  // whenever we might branch off to a block that we do not directly compile
  // afterwards (i.e. the register assignments might change in between), we
  // need to spill all registers which are not fixed and remove them from the
  // register state.
  //
  // This leads to worse codegen but saves a significant overhead to
  // store/manage the register assignment for each block (256 bytes/block for
  // x64) and possible compile-time as there might be additional logic to move
  // values around

  // First, we consider the case that the current block only has one successor
  // which is compiled directly after the current one, in which case we do not
  // have to spill anything.
  //
  // Secondly, if the next block has multiple incoming edges, we always have
  // to spill and remove from the register assignment. Otherwise, we
  // only need to spill values if they are alive in any successor which is not
  // the next block.
  //
  // Values which are only read from PHI-Nodes and have no extended lifetimes,
  // do not need to be spilled as they die at the edge.

  using RegBitSet = typename RegisterFile::RegBitSet;

  assert(may_change_value_state());

  const IRBlockRef cur_block_ref = analyzer.block_ref(cur_block_idx);
  // Earliest succeeding block after the current block that is not the
  // immediately succeeding block. Used to determine whether a value needs to
  // be spilled.
  BlockIndex earliest_next_succ = Analyzer<Adaptor>::INVALID_BLOCK_IDX;

  bool must_spill = force_spill;
  if (!must_spill) {
    // We must always spill if no block is immediately succeeding or that block
    // has multiple incoming edges.
    auto next_block_is_succ = false;
    auto next_block_has_multiple_incoming = false;
    u32 succ_count = 0;
    for (const IRBlockRef succ : adaptor->block_succs(cur_block_ref)) {
      ++succ_count;
      BlockIndex succ_idx = analyzer.block_idx(succ);
      if (u32(succ_idx) == u32(cur_block_idx) + 1) {
        next_block_is_succ = true;
        if (analyzer.block_has_multiple_incoming(succ)) {
          next_block_has_multiple_incoming = true;
        }
      } else if (succ_idx > cur_block_idx && succ_idx < earliest_next_succ) {
        earliest_next_succ = succ_idx;
      }
    }

    must_spill = !next_block_is_succ || next_block_has_multiple_incoming;

    if (succ_count == 1 && !must_spill) {
      return RegBitSet{};
    }
  }
  return RegBitSet{};
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::release_spilled_regs(
    typename RegisterFile::RegBitSet regs) noexcept {
  assert(may_change_value_state());

  // TODO(ts): needs changes for other RegisterFile impls
  for (auto reg_id : util::BitSetIterator<>{regs & register_file.used}) {
    if (!register_file.is_fixed(Reg{reg_id})) {
      free_reg(Reg{reg_id});
    }
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::
    release_regs_after_return() noexcept {
  // we essentially have to free all non-fixed registers
  for (auto reg_id : register_file.used_regs()) {
    if (!register_file.is_fixed(Reg{reg_id})&&(register_file.reg_local_idx(Reg{reg_id}) == INVALID_VAL_LOCAL_IDX || val_assignment(register_file.reg_local_idx(Reg{reg_id}))->references_left == 0)) {
      free_reg(Reg{reg_id});
    }
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::generate_switch(
    ScratchReg &&cond,
    u32 width,
    IRBlockRef default_block,
    std::span<const std::pair<u64, IRBlockRef>> cases) noexcept {
  // This function takes cond as a ScratchReg as opposed to a ValuePart, because
  // the ValueRef for the condition must be ref-counted before we enter the
  // branch region.

  assert(width <= 64);
  // We don't support sections with more than 4 GiB, so switches with more than
  // 4G cases are impossible to support.
  assert(cases.size() < UINT32_MAX && "large switches are unsupported");

  AsmReg cmp_reg = cond.cur_reg();
  bool width_is_32 = width <= 32;
  if (u32 dst_width = util::align_up(width, 32); width != dst_width) {
    derived()->generate_raw_intext(cmp_reg, cmp_reg, false, width, dst_width);
  }

  // We must not evict any registers in the branching code, as we don't track
  // the individual value states per block. Hence, we must not allocate any
  // registers (e.g., for constants, jump table address) below.
  ScratchReg tmp_scratch{this};
  AsmReg tmp_reg = tmp_scratch.alloc_gp();

  const auto spilled = this->spill_before_branch();
  this->begin_branch_region();

  // because some blocks might have PHI-values we need to first jump to a
  // label which then fixes the registers and then jumps to the block
  // TODO(ts): check which blocks need PHIs and otherwise jump directly to
  // them? probably better for branch predictor

  tpde::util::SmallVector<tpde::Label, 64> case_labels;
  for (auto i = 0u; i < cases.size(); ++i) {
    case_labels.push_back(this->text_writer.label_create());
  }

  const auto default_label = this->text_writer.label_create();

  const auto build_range = [&,
                            this](size_t begin, size_t end, const auto &self) {
    assert(begin <= end);
    const auto num_cases = end - begin;
    if (num_cases <= 4) {
      // if there are four or less cases we just compare the values
      // against each of them
      for (auto i = 0u; i < num_cases; ++i) {
        derived()->switch_emit_cmpeq(case_labels[begin + i],
                                     cmp_reg,
                                     tmp_reg,
                                     cases[begin + i].first,
                                     width_is_32);
      }

      derived()->generate_raw_jump(Derived::Jump::jmp, default_label);
      return;
    }

    // check if the density of the values is high enough to warrant building
    // a jump table
    auto range = cases[end - 1].first - cases[begin].first;
    // we will get wrong results if range is -1 so skip the jump table if
    // that is the case
    if (range != 0xFFFF'FFFF'FFFF'FFFF && (range / num_cases) < 8) {
      // for gcc, it seems that if there are less than 8 values per
      // case it will build a jump table so we do that, too

      // the actual range is one greater than the result we get from
      // subtracting so adjust for that
      range += 1;

      tpde::util::SmallVector<tpde::Label, 32> label_vec;
      std::span<tpde::Label> labels;
      if (range == num_cases) {
        labels = std::span{case_labels.begin() + begin, num_cases};
      } else {
        label_vec.resize(range, default_label);
        for (auto i = 0u; i < num_cases; ++i) {
          label_vec[cases[begin + i].first - cases[begin].first] =
              case_labels[begin + i];
        }
        labels = std::span{label_vec.begin(), range};
      }

      // Give target the option to emit a jump table.
      if (derived()->switch_emit_jump_table(default_label,
                                            labels,
                                            cmp_reg,
                                            tmp_reg,
                                            cases[begin].first,
                                            cases[end - 1].first,
                                            width_is_32)) {
        return;
      }
    }

    // do a binary search step
    const auto half_len = num_cases / 2;
    const auto half_value = cases[begin + half_len].first;
    const auto gt_label = this->text_writer.label_create();

    // this will cmp against the input value, jump to the case if it is
    // equal or to gt_label if the value is greater. Otherwise it will
    // fall-through
    derived()->switch_emit_binary_step(case_labels[begin + half_len],
                                       gt_label,
                                       cmp_reg,
                                       tmp_reg,
                                       half_value,
                                       width_is_32);
    // search the lower half
    self(begin, begin + half_len, self);

    // and the upper half
    this->label_place(gt_label);
    self(begin + half_len + 1, end, self);
  };

  build_range(0, case_labels.size(), build_range);

  // write out the labels
  this->label_place(default_label);
  derived()->generate_branch_to_block(
      Derived::Jump::jmp, default_block, false, false);

  for (auto i = 0u; i < cases.size(); ++i) {
    this->label_place(case_labels[i]);
    derived()->generate_branch_to_block(
        Derived::Jump::jmp, cases[i].second, false, false);
  }

  this->end_branch_region();
  this->release_spilled_regs(spilled);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::move_to_phi_nodes_impl(
    BlockIndex target, MoveList &moves) noexcept {
  // PHI-nodes are always moved to their stack-slot (unless they are fixed)
  //
  // However, we need to take care of PHI-dependencies (cycles and chains)
  // as to not overwrite values which might be needed.
  //
  // In most cases, we expect the number of PHIs to be small but we want to
  // stay reasonably efficient even with larger numbers of PHIs
  // todo(salto): more phis than registers

  struct ScratchWrapper {
    Derived *self;
    AsmReg cur_reg = AsmReg::make_invalid();
    bool backed_up = false;
    bool was_modified = false;
    u8 part = 0;
    ValLocalIdx local_idx = INVALID_VAL_LOCAL_IDX;

    ScratchWrapper(Derived *self) : self{self} {}

    ~ScratchWrapper() { reset(); }

    void reset() {
      if (cur_reg.invalid()) {
        return;
      }

      self->register_file.unmark_fixed(cur_reg);
      self->register_file.unmark_used(cur_reg);

      if (backed_up) {
        // restore the register state
        // TODO(ts): do we actually need the reload?
        auto *assignment = self->val_assignment(local_idx);
        // check if the value was free'd, then we dont need to restore
        // it
        if (assignment) {
          auto ap = AssignmentPartRef{assignment, part};
          if (!ap.variable_ref()) {
            // TODO(ts): assert that this always happens?
            assert(ap.stack_valid());
            self->load_from_stack(cur_reg, ap.frame_off(), ap.part_size());
          }
          ap.set_reg(cur_reg);
          ap.set_register_valid(true);
          ap.set_modified(was_modified);
          self->register_file.mark_used(cur_reg, local_idx, part);
        }
        backed_up = false;
      }
      cur_reg = AsmReg::make_invalid();
    }

    AsmReg alloc_from_bank(RegBank bank, u64 exclusion_mask=0) {
      if (cur_reg.valid() && self->register_file.reg_bank(cur_reg) == bank) {
        return cur_reg;
      }
      if (cur_reg.valid()) {
        reset();
      }

      // TODO(ts): try to first find a non callee-saved/clobbered
      // register...
      auto &reg_file = self->register_file;
      auto reg = reg_file.find_first_free_excluding(bank, exclusion_mask);
      if (reg.invalid()) {
        // TODO(ts): use clock here?
        reg = reg_file.find_first_nonfixed_excluding(bank, exclusion_mask);
        if (reg.invalid()) {
          TPDE_FATAL("ran out of registers for scratch registers");
        }

        backed_up = true;
        local_idx = reg_file.reg_local_idx(reg);
        part = reg_file.reg_part(reg);
        AssignmentPartRef ap{self->val_assignment(local_idx), part};
        was_modified = ap.modified();
        // TODO(ts): this does not spill for variable refs
        // We don't use evict_reg here, as we know that we can't change the
        // value state.
        assert(ap.register_valid() && ap.get_reg() == reg);
        if (!ap.stack_valid() && !ap.variable_ref()) {
          self->spill(ap);
        }
        ap.set_register_valid(false);
        reg_file.unmark_used(reg);
      }

      reg_file.mark_used(reg, INVALID_VAL_LOCAL_IDX, 0);
      reg_file.mark_clobbered(reg);
      reg_file.mark_fixed(reg);
      cur_reg = reg;
      return reg;
    }

    ScratchWrapper &operator=(const ScratchWrapper &) = delete;
    ScratchWrapper &operator=(ScratchWrapper &&) = delete;
  };

  IRBlockRef target_ref = analyzer.block_ref(target);
  IRBlockRef cur_ref = analyzer.block_ref(cur_block_idx);

  // collect all the nodes
  struct NodeEntry {
    IRValueRef phi;
    IRValueRef incoming_val;
    ValLocalIdx phi_local_idx;
    // local idx of same-block phi node that needs special handling
    ValLocalIdx incoming_phi_local_idx = INVALID_VAL_LOCAL_IDX;
    // bool incoming_is_phi;
    u32 ref_count;
    u32 references_left = 0;        // For prioritization
    bool allocate_to_stack = false; // Flag for stack allocation

    bool operator<(const NodeEntry &other) const noexcept {
      return phi_local_idx < other.phi_local_idx;
    }

    bool operator<(ValLocalIdx other) const noexcept {
      return phi_local_idx < other;
    }
  };

  util::SmallVector<NodeEntry, 16> nodes;
  for (IRValueRef phi : adaptor->block_phis(target_ref)) {
    ValLocalIdx phi_local_idx = adaptor->val_local_idx(phi);
    auto incoming = adaptor->val_as_phi(phi).incoming_val_for_block(cur_ref);

    // Get references_left for prioritization
    ValueAssignment *assignment = val_assignment(phi_local_idx);
    u32 refs_left = assignment ? assignment->references_left : 0;


    nodes.emplace_back(NodeEntry{.phi = phi,
                                 .incoming_val = incoming,
                                 .phi_local_idx = phi_local_idx,
                                 .references_left = refs_left});
  }

  // We check that the block has phi nodes before getting here.
  assert(!nodes.empty() && "block marked has having phi nodes has none");

  // Determine allocation strategy: hybrid register/stack if too many phi nodes
  const u32 phi_count = static_cast<u32>(nodes.size());
  // todo(salto): do this for each bank seperately, so we guarantee at least 1 free register
  const u32 available_regs = count_total_available_registers();

  // Reserve some registers for scratch operations during phi resolution
  constexpr u32 reserved_for_scratch = 2;
  const u32 regs_for_phis = std::max(0u, std::min(available_regs-phi_count-reserved_for_scratch, PHI_REGISTER_THRESHOLD));


  bool use_hybrid_allocation =phi_count > regs_for_phis;

  if (use_hybrid_allocation) {
    TPDE_LOG_DBG("Using hybrid phi allocation: {} phi nodes, {} available regs",
                 phi_count,
                 available_regs);

    // Sort by references_left (descending) to prioritize high-use phi nodes
    // TODO(salto): Test different heuristics for phi node prioritization:
    // - Could prioritize by liveness length
    // - Could prioritize by use in current block vs successors
    // - Could use a combination of factors
    std::sort(
        nodes.begin(), nodes.end(), [](const NodeEntry &a, const NodeEntry &b) {
          return a.references_left > b.references_left;
        });

    // Mark phi nodes beyond register capacity for stack allocation
    for (u32 i = regs_for_phis; i < phi_count; ++i) {
      nodes[i].allocate_to_stack = true;
      TPDE_LOG_TRACE("Phi node {} will be allocated to stack",
                     static_cast<u32>(nodes[i].phi_local_idx));
    }

  }
  typename RegisterFile::RegBitSet used_phi_regs = 0;

  const auto move_to_phi_reg = [this, &moves, &used_phi_regs](
                                   IRValueRef phi,
                                   IRValueRef incoming_val,
                                   bool force_stack) {
    auto phi_vr = derived()->result_ref(phi);
    // We access the phi here
    // phi_vr.disown();
    auto val_vr = derived()->val_ref(incoming_val);
    ValLocalIdx incoming_val_idx = INVALID_VAL_LOCAL_IDX;
    if (!adaptor->val_ignore_in_liveness_analysis(incoming_val)) {
      incoming_val_idx = adaptor->val_local_idx(incoming_val);
    }
    // useless or unused phi
    if (phi == incoming_val ) { // todo(salto): || phi_vr.assignment()->references_left == 1?
      return;
    }

    u32 part_count = phi_vr.assignment()->part_count;
    for (u32 i = 0; i < part_count; ++i) {
      ScratchWrapper scratch{derived()};
      AssignmentPartRef phi_ap{phi_vr.assignment(), i};
      ValuePartRef val_vpr = val_vr.part(i);

      // Handle forced stack allocation for phi nodes that can't fit in
      // registers
      // we need to make sure that phi nodes that already have a register don't get a stack slot here
      if ((phi_ap.stack_valid() && (!(phi_regs[adaptor->val_local_idx(phi)].size() > 0))) || (force_stack && !phi_ap.fixed_assignment())) {
        // Allocate stack slot for this phi node

        allocate_spill_slot(phi_ap);

        // Load incoming value to temporary register if needed
        AsmReg reg = val_vpr.cur_reg_unlocked();
        if (!reg.valid()) {
          reg = scratch.alloc_from_bank(val_vpr.bank());
          val_vpr.reload_into_specific_fixed(reg);
        }

        // Spill directly to phi's stack slot
        derived()->spill_reg(reg, phi_ap.frame_off(), phi_ap.part_size());
        phi_ap.set_stack_valid();

        #ifndef NDEBUG
        // Capture spill edit for phi resolution
        typename VIR<Adaptor>::Allocation spill_from(reg);
        typename VIR<Adaptor>::Allocation spill_to(phi_ap.frame_off());
        verification_ir.emit_edit(VIR<Adaptor>::EditKind::Spill,
                                  spill_from,
                                  spill_to,
                                  adaptor->val_local_idx(phi),
                                  i,
                                  phi_ap.part_size());
        #endif
        TPDE_LOG_TRACE("Phi {} part {} allocated to stack at offset {}",
                       static_cast<u32>(adaptor->val_local_idx(phi)),
                       i,
                       phi_ap.frame_off());
        continue;
      }

      AsmReg reg = val_vpr.cur_reg_unlocked();

      if (!reg.valid()) {
        if (phi_ap.fixed_assignment()) {
          val_vpr.reload_into_specific_fixed(phi_ap.get_reg());
          reg = phi_ap.get_reg();
        } else {
          // todo(salto): move to reg from phi if already assigned
          if (phi_regs[adaptor->val_local_idx(phi)].size() > i) {
            auto target_phi_reg = phi_regs[adaptor->val_local_idx(phi)][i];
            // can assign directly to phi_reg
            if (!register_file.is_used(target_phi_reg) ||
                register_file.reg_local_idx(target_phi_reg) ==
                    adaptor->val_local_idx(phi)) {
              val_vpr.reload_into_specific_fixed(target_phi_reg);
              reg = target_phi_reg;
            } else {
              // need a intermediate register, sequentialize will resolve move issues later
              reg = scratch.alloc_from_bank(val_vpr.bank(), used_phi_regs);
              val_vpr.reload_into_specific_fixed(reg);
            }


          } else {
            reg = scratch.alloc_from_bank(val_vpr.bank(), used_phi_regs);
            val_vpr.reload_into_specific_fixed(reg);
          }
        }
      }

      // was already assigned by a different branch

      if (phi_regs[adaptor->val_local_idx(phi)].size() > i) {
        auto target_phi_reg = phi_regs[adaptor->val_local_idx(phi)][i];
        used_phi_regs |= (1 << target_phi_reg.id());
        moves.emplace_back(
            target_phi_reg,
                           reg,
                           val_vpr.part_size(),
                           incoming_val_idx,
                           i);
      } else {
        if (phi_ap.fixed_assignment()) {
          used_phi_regs |= (1<<phi_ap.get_reg().id());
          phi_regs[adaptor->val_local_idx(phi)].push_back(phi_ap.get_reg());
          moves.emplace_back(
              phi_ap.get_reg(), reg, val_vpr.part_size(), incoming_val_idx, i);
          continue;
        }
        // no assigned registers. Avoid moves on this edge if possible.
        if (val_vr.last_ref()) {
          used_phi_regs |= (1 << reg.id());
          phi_regs[adaptor->val_local_idx(phi)].push_back(reg);
        } else {
          // todo(salto): preferred register

          // incoming_val may be used outside the phi, so we need a separate reg
          // for it.


          auto phi_reg = scratch.alloc_from_bank(phi_ap.bank(),used_phi_regs);
          if (!phi_reg.valid()) {
            // Spill phi to stack if no register available
            allocate_spill_slot(phi_ap);
            phi_ap.set_stack_valid();
            TPDE_LOG_TRACE(
                "Phi {} part {} spilled to stack due to register exhaustion",
                static_cast<u32>(adaptor->val_local_idx(phi)),
                i);
            #ifndef NDEBUG
            // Capture spill edit for phi resolution
            typename VIR<Adaptor>::Allocation from(val_vpr.cur_reg_unlocked());
            typename VIR<Adaptor>::Allocation to(phi_ap.frame_off());
            verification_ir.emit_edit(VIR<Adaptor>::EditKind::Spill,
                                      from,
                                      to,
                                      adaptor->val_local_idx(phi),
                                      i,
                                      phi_ap.part_size());
            #endif
            continue;
          }

          used_phi_regs |= (1 << phi_reg.id());
          phi_regs[adaptor->val_local_idx(phi)].push_back(phi_reg);
          moves.emplace_back(
              phi_reg, reg, val_vpr.part_size(), incoming_val_idx, i);
        }

    }
  }
  };

  for (u32 i = 0; i < nodes.size(); ++i) {
    NodeEntry &node = nodes[i];
    move_to_phi_reg(node.phi, node.incoming_val, node.allocate_to_stack);
  }

#ifndef NDEBUG
  //todo(salto): decide if we need the parrarel moves
  // Capture parallel moves for verification IR
  util::SmallVector<std::pair<typename VIR<Adaptor>::Operand, typename VIR<Adaptor>::Operand>, 4> parallel_moves;
  u32 temp_move_counter = 0;
  auto find_value_for_reg_parallel = [&](Reg reg) -> std::pair<ValLocalIdx, u32> {
    if (!reg.valid()) {
      return {INVALID_VAL_LOCAL_IDX, 0};
    }
    if (register_file.is_used(reg)) {
      ValLocalIdx idx = register_file.reg_local_idx(reg);
      if (idx != INVALID_VAL_LOCAL_IDX) {
        u32 part = register_file.reg_part(reg);
        return {idx, part};
      }
    }
    for (u32 i = 0; i < assignments.value_ptrs.size(); ++i) {
      ValueAssignment *va = assignments.value_ptrs[i];
      if (!va) continue;
      for (u32 part = 0; part < va->part_count; ++part) {
        AssignmentPartRef ap{va, part};
        if (ap.get_reg() == reg) {
          return {static_cast<ValLocalIdx>(i), part};
        }
      }
    }
    return {INVALID_VAL_LOCAL_IDX, 0};
  };
  for (auto move : moves) {
    if (move.src == move.dst) {
      continue;
    }
    ValLocalIdx src_val_idx = move.value_idx;
    u32 src_part = move.part_idx;
    if (src_val_idx == INVALID_VAL_LOCAL_IDX) {
      auto [found_src_idx, found_src_part] = find_value_for_reg_parallel(move.src);
      src_val_idx = found_src_idx;
      src_part = found_src_part;
    }
    auto [dst_val_idx, dst_part] = find_value_for_reg_parallel(move.dst);

    if (src_val_idx != INVALID_VAL_LOCAL_IDX && dst_val_idx != INVALID_VAL_LOCAL_IDX) {
      typename VIR<Adaptor>::Operand dst_op;
      dst_op.val_idx = dst_val_idx;
      dst_op.alloc = typename VIR<Adaptor>::Allocation(move.dst, dst_part);
      typename VIR<Adaptor>::Operand src_op;
      src_op.val_idx = src_val_idx;
      src_op.alloc = typename VIR<Adaptor>::Allocation(move.src, src_part);
      parallel_moves.emplace_back(src_op, dst_op);
    } else if (src_val_idx != INVALID_VAL_LOCAL_IDX) {
      typename VIR<Adaptor>::Operand dst_op;
      dst_op.val_idx = dst_val_idx;
      dst_op.alloc = typename VIR<Adaptor>::Allocation(move.dst, dst_part);
      typename VIR<Adaptor>::Operand src_op;
      src_op.val_idx = src_val_idx;
      src_op.alloc = typename VIR<Adaptor>::Allocation(move.src, src_part);
      parallel_moves.emplace_back(src_op, dst_op);
    } else {
      // Temporary move - both registers unknown, use temporary virtual register
      ValLocalIdx temp_val_idx = static_cast<ValLocalIdx>(static_cast<u32>(cur_block_idx) | 0x70000000u | (temp_move_counter << 16));
      temp_move_counter++;
      typename VIR<Adaptor>::Operand dst_op;
      dst_op.val_idx = temp_val_idx;
      dst_op.alloc = typename VIR<Adaptor>::Allocation(move.dst, 0);
      typename VIR<Adaptor>::Operand src_op;
      src_op.val_idx = temp_val_idx;
      src_op.alloc = typename VIR<Adaptor>::Allocation(move.src, 0);
      parallel_moves.emplace_back(src_op, dst_op);
    }
  }
  if (!parallel_moves.empty()) {
    verification_ir.emit_edge_parallel_move(cur_block_idx, target, std::move(parallel_moves));
  }
#endif
  for (auto move:moves) {
    if (register_file.is_used(move.dst)&&register_file.reg_local_idx(move.dst) == INVALID_VAL_LOCAL_IDX) {
      register_file.unmark_used(move.dst);
    }
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::BlockIndex
    CompilerBase<Adaptor, Derived, Config>::next_block() const noexcept {
  return static_cast<BlockIndex>(static_cast<u32>(cur_block_idx) + 1);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
SymRef CompilerBase<Adaptor, Derived, Config>::get_personality_sym() noexcept {
  SymRef personality_sym;
  if (this->adaptor->cur_needs_unwind_info()) {
    SymRef personality_func = derived()->cur_personality_func();
    if (personality_func.valid()) {
      for (const auto &[fn_sym, ptr_sym] : personality_syms) {
        if (fn_sym == personality_func) {
          personality_sym = ptr_sym;
          break;
        }
      }

      if (!personality_sym.valid()) {
        // create symbol that contains the address of the personality
        // function
        u32 off;
        static constexpr std::array<u8, 8> zero{};

        auto rodata = this->assembler.get_data_section(true, true);
        personality_sym = this->assembler.sym_def_data(
            rodata, "", zero, 8, Assembler::SymBinding::LOCAL, &off);
        this->assembler.reloc_abs(rodata, personality_func, off, 0);

        personality_syms.emplace_back(personality_func, personality_sym);
      }
    }
  }
  return personality_sym;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
bool CompilerBase<Adaptor, Derived, Config>::compile_func(
    const IRFuncRef func, const u32 func_idx) noexcept {
  if (!adaptor->switch_func(func)) {
    return false;
  }
  derived()->analysis_start();
  analyzer.switch_func(func);
  derived()->analysis_end();

#ifndef NDEBUG
  stack.frame_size = ~0u;
#endif
  for (auto &e : stack.fixed_free_lists) {
    e.clear();
  }
  stack.dynamic_free_lists.clear();
  // TODO: sort out the inconsistency about adaptor vs. compiler methods.
  stack.has_dynamic_alloca = this->adaptor->cur_has_dynamic_alloca();
  stack.is_leaf_function = !derived()->cur_func_may_emit_calls();
  stack.generated_call = false;

  assignments.cur_fixed_assignment_count = {};
  assert(std::ranges::none_of(assignments.value_ptrs, std::identity{}));
  if (assignments.value_ptrs.size() < analyzer.liveness.size()) {
    assignments.value_ptrs.resize(analyzer.liveness.size());
  }

  assignments.allocator.reset();
  assignments.variable_ref_list = INVALID_VAL_LOCAL_IDX;
  assignments.delayed_free_lists.clear();
  assignments.delayed_free_lists.resize(analyzer.block_layout.size(),
                                        INVALID_VAL_LOCAL_IDX);

  cur_block_idx =
      static_cast<BlockIndex>(analyzer.block_idx(adaptor->cur_entry_block()));
  block_states.resize(analyzer.block_layout.size());
  block_regs.clear();
  phi_regs.clear();
  register_file.reset();
#ifndef NDEBUG
  generating_branch = false;
  verification_ir.reset();
  verification_ir.set_func_name(adaptor->func_link_name(func));
#endif

  // Simple heuristic for initial allocation size
  u32 expected_code_size = 0x8 * analyzer.num_insts + 0x40;
  this->text_writer.begin_func(expected_code_size);

  derived()->start_func(func_idx);

  block_labels.clear();
  block_labels.resize_uninitialized(analyzer.block_layout.size());
  for (u32 i = 0; i < analyzer.block_layout.size(); ++i) {
    block_labels[i] = text_writer.label_create();
  }

  // TODO(ts): place function label
  // TODO(ts): make function labels optional?

  CCAssigner *cc_assigner = derived()->cur_cc_assigner();
  assert(cc_assigner != nullptr);

  register_file.allocatable = cc_assigner->get_ccinfo().allocatable_regs;

  // This initializes the stack frame, which must reserve space for
  // callee-saved registers, vararg save area, etc.
  cc_assigner->reset();
  derived()->gen_func_prolog_and_args(cc_assigner);

#ifndef NDEBUG
  // After gen_func_prolog_and_args, explicitly capture all function arguments
  // to ensure they appear first in the verification IR according to calling convention
  // Iterate through arguments and capture their register assignments
  for (const IRValueRef arg : adaptor->cur_args()) {
    ValLocalIdx arg_idx = adaptor->val_local_idx(arg);
    if (arg_idx == INVALID_VAL_LOCAL_IDX) continue;

    ValueAssignment *assignment = val_assignment(arg_idx);
    if (!assignment) continue;

    const auto parts = adaptor->val_parts(arg);
    const u32 part_count = parts.count();
    for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
      AssignmentPartRef ap{assignment, part_idx};
      if (ap.register_valid()) {
        Reg reg = ap.get_reg();
        BlockIndex entry_block_idx = static_cast<BlockIndex>(analyzer.block_idx(adaptor->cur_entry_block()));
        typename VIR<Adaptor>::Allocation alloc(reg);
        verification_ir.emit_arg(entry_block_idx, arg_idx, part_idx, alloc);
      } else if (ap.stack_valid()) {
        // Argument on stack - capture with stack allocation
        i32 stack_off = ap.frame_off();
        BlockIndex entry_block_idx = static_cast<BlockIndex>(analyzer.block_idx(adaptor->cur_entry_block()));
        typename VIR<Adaptor>::Allocation alloc(stack_off);
        verification_ir.emit_arg(entry_block_idx, arg_idx, part_idx, alloc);
      }
    }
  }
#endif

  for (const IRValueRef alloca : adaptor->cur_static_allocas()) {
    auto size = adaptor->val_alloca_size(alloca);
    size = util::align_up(size, adaptor->val_alloca_align(alloca));

    ValLocalIdx local_idx = adaptor->val_local_idx(alloca);
    init_variable_ref(local_idx, 0);
    ValueAssignment *assignment = val_assignment(local_idx);
    assignment->stack_variable = true;
    assignment->frame_off = allocate_stack_slot(size);
  }

  if constexpr (!Config::DEFAULT_VAR_REF_HANDLING) {
    derived()->setup_var_ref_assignments();
  }

  for (u32 i = 0; i < analyzer.block_layout.size(); ++i) {
    const auto block_ref = analyzer.block_layout[i];
    TPDE_LOG_TRACE(
        "Compiling block {} ({})", i, adaptor->block_fmt_ref(block_ref));
    if (!derived()->compile_block(block_ref, i)) [[unlikely]] {
      TPDE_LOG_ERR("Failed to compile block {} ({})",
                   i,
                   adaptor->block_fmt_ref(block_ref));
      // Ensure invariant that value_ptrs only contains nullptr at the end.
      assignments.value_ptrs.clear();
      return false;
    }
  }

  // Reset all variable-ref assignment pointers to nullptr.
  ValLocalIdx variable_ref_list = assignments.variable_ref_list;
  while (variable_ref_list != INVALID_VAL_LOCAL_IDX) {
    u32 idx = u32(variable_ref_list);
    ValLocalIdx next = assignments.value_ptrs[idx]->next_delayed_free_entry;
    assignments.value_ptrs[idx] = nullptr;
    variable_ref_list = next;
  }

  assert(std::ranges::none_of(assignments.value_ptrs, std::identity{}) &&
         "found non-freed ValueAssignment, maybe missing ref-count?");

  derived()->finish_func(func_idx);
  this->text_writer.finish_func();

#ifndef NDEBUG
  //todo(salto): decide on smarter location for file
  // Write verification IR to file
  std::string vir_filename = verification_ir.get_func_name() + ".vir";
  verification_ir.write_to_file(vir_filename);
#endif

  return true;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
bool CompilerBase<Adaptor, Derived, Config>::compile_block(
    const IRBlockRef block, const u32 block_idx) noexcept {
  cur_block_idx =
      static_cast<typename Analyzer<Adaptor>::BlockIndex>(block_idx);

  label_place(block_labels[block_idx]);
  #ifndef NDEBUG
  verification_ir.set_current_block(cur_block_idx);
  #endif

  auto state_it = block_regs.find(cur_block_idx);
  if (state_it != block_regs.end() || analyzer.block_has_phis(cur_block_idx)) {
    for (IRValueRef phi:adaptor->block_phis(block)) {
      auto phi_idx = adaptor->val_local_idx(phi);
      ValueAssignment *assignment = this->val_assignment(phi_idx); //todo val_assignment null
      // phi is unused and already freed
      if (!assignment) {
        continue;
      }
      for (u32 i = 0; i < assignment->part_count; i++) {
        auto ap = AssignmentPartRef{assignment, i};
        Reg reg = Reg::make_invalid();
        if (ap.fixed_assignment()) {
          reg = ap.get_reg();
        } else if (phi_regs.find(phi_idx) != phi_regs.end() && phi_regs[phi_idx].size() > i) {
          reg = phi_regs[phi_idx][i];
          ap.set_reg(reg);
          ap.set_register_valid(true);
          if (!register_file.is_used(reg)) {
            register_file.mark_used(reg,phi_idx,i);
          }

        } else if (ap.stack_valid()) {
          // PHI is on stack - keep it on stack, will load when used
          reg = Reg::make_invalid();
        } else {
          TPDE_UNREACHABLE("PHI node is neither on stack nor in a register");
        }

#ifndef NDEBUG
        // Capture PHI node with virtual register and assembly register/stack
        // location
        assert(reg.valid() || ap.stack_valid());
        vir_emit_phi(phi, phi_idx, i, reg.valid() ? reg : Reg::make_invalid());

#endif
      }
    }
    if (state_it != block_regs.end()) {
      for (ValueState &state: state_it->second) {
        ValueAssignment *assignment = this->val_assignment(state.val_local_idx);
        if (!assignment)
          continue;
        for (u32 i =0; i<assignment->part_count;i++) {

          auto ap = AssignmentPartRef{assignment, i};
          auto reg = state.registers[i];
          if (!reg.valid())
            continue;
          ap.set_reg(reg);
          ap.set_register_valid(true);
          if (!register_file.is_used(reg))
            register_file.mark_used(reg,state.val_local_idx,i);
        }
      }
    }
  }
  auto &&val_range = adaptor->block_insts(block);
  auto end = val_range.end();
  for (auto it = val_range.begin(); it != end; ++it) {
    const IRInstRef inst = *it;
    if (this->adaptor->inst_fused(inst)) {
      continue;
    }

#ifndef NDEBUG
    // Capture instruction uses (operands) before compilation, as they may be free later
    // For branches, these are the condition values
    util::SmallVector<typename VIR<Adaptor>::Operand, 4> uses;

    for (IRValueRef operand : adaptor->inst_operands(inst)) {
      if (auto vrsp = derived()->val_ref_special(operand); vrsp) {
        continue;
      }
      if (adaptor->val_ignore_in_liveness_analysis(operand)) {
        continue;
      }
      ValLocalIdx op_idx = adaptor->val_local_idx(operand);
      if (op_idx == INVALID_VAL_LOCAL_IDX) {
        continue; // Skip constants/undef
      }

      ValueAssignment *op_assignment = val_assignment(op_idx);
      if (!op_assignment)
        continue; // Not yet assigned


      const auto parts = adaptor->val_parts(operand);
      const u32 part_count = parts.count();
      for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
        AssignmentPartRef ap{op_assignment, part_idx};
        typename VIR<Adaptor>::Operand op;
        op.val_idx = op_idx;
        op.alloc = get_allocation(ap);
        op.alloc.part_idx = part_idx;
        uses.push_back(op);
      }
    }

    // Store condition uses BEFORE compile_inst (which calls generate_branch_to_block)
    // Make a copy since we'll need uses for non-branch instructions too
    util::SmallVector<typename VIR<Adaptor>::Operand, 4> branch_condition;
    for (const auto &op : uses) {
      branch_condition.push_back(op);
    }
    verification_ir.set_branch_condition(std::move(branch_condition));
    // don't capture moves during codegen. They are not necessary for VIR.
    verification_ir.active_compilation=true;
#endif

    auto it_cpy = it;
    ++it_cpy;
     if (!derived()->compile_inst(inst, InstRange{.from = it_cpy, .to = end}))
         [[unlikely]] {
       TPDE_LOG_ERR("Failed to compile instruction {}",
                    this->adaptor->inst_fmt_ref(inst));
       return false;
     }

     // Post-process instruction results for spilling
     for (IRValueRef result : adaptor->inst_results(inst)) {
       ValLocalIdx res_idx = adaptor->val_local_idx(result);
       if (res_idx == INVALID_VAL_LOCAL_IDX) continue;
       u32 idx = static_cast<u32>(res_idx);
       if (idx < analyzer.spilled_values.bit_size && analyzer.spilled_values.is_set(idx)) {
         ValueAssignment *assignment = val_assignment(res_idx);
         if (!assignment) continue;
         const auto parts = adaptor->val_parts(result);
         const u32 part_count = parts.count();
         for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
           AssignmentPartRef ap{assignment, part_idx};
           if (ap.register_valid()) {
            TPDE_LOG_INFO("Spilling result {}", static_cast<u32>(res_idx));
             spill(ap);
           }
         }
       }
     }

 #ifndef NDEBUG
    {
      verification_ir.active_compilation=false;
      // Update uses with final allocations after compilation
      // (allocations may have changed during compilation, e.g., values moved to
      // registers)
      for (auto &use : uses) {
        ValueAssignment *use_assignment = val_assignment(use.val_idx);
        if (use_assignment) {
          AssignmentPartRef ap{use_assignment, use.alloc.part_idx};
          use.alloc = get_allocation(ap);
        } else {
          use.alloc = typename VIR<Adaptor>::Allocation(
              final_assignments[use.val_idx][use.alloc.part_idx]);
        }
      }
      // constants
      for (Reg reg:final_assignments[INVALID_VAL_LOCAL_IDX]) {
        typename VIR<Adaptor>::Operand op;
        op.val_idx = INVALID_VAL_LOCAL_IDX;
        op.alloc =typename VIR<Adaptor>::Allocation(reg, 0);
      }

      util::SmallVector<typename VIR<Adaptor>::Operand, 2> defs;
      ValLocalIdx inst_id = INVALID_VAL_LOCAL_IDX;
      for (IRValueRef result : adaptor->inst_results(inst)) {
        ValLocalIdx res_idx = adaptor->val_local_idx(result);
        if (res_idx == INVALID_VAL_LOCAL_IDX) {
          continue;
        }

        // Use the first result's val_idx as the instruction identifier
        if (inst_id == INVALID_VAL_LOCAL_IDX) {
          inst_id = res_idx;
        }

        ValueAssignment *res_assignment = val_assignment(res_idx);
        if (!res_assignment) {
          continue; // Should exist after compile_inst, unless the result was already freed
        }

        const auto parts = adaptor->val_parts(result);
        const u32 part_count = parts.count();
        for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
          AssignmentPartRef ap{res_assignment, part_idx};
          typename VIR<Adaptor>::Operand op;
          op.val_idx = res_idx;
          op.alloc = get_allocation(ap);
          op.alloc.part_idx = part_idx;
          defs.push_back(op);
        }
      }

      // If no result, use a marker based on the block index
      if (inst_id == INVALID_VAL_LOCAL_IDX) {
        inst_id = static_cast<ValLocalIdx>(static_cast<u32>(cur_block_idx) |
                                           0x60000000u);
      }

      // Emit the instruction operation (non-branch instructions)
      verification_ir.emit_inst_op(inst_id, std::move(uses), std::move(defs));
    }
#endif
  }

#ifndef NDEBUG
  // Some consistency checks. Register assignment information must match, all
  // used registers must have an assignment (no temporaries across blocks), and
  // fixed registers must be fixed assignments.
  // Note: Temporary phi registers during move resolution may not have register_valid set
  for (auto reg_id : register_file.used_regs()) {
    Reg reg{reg_id};
    ValLocalIdx local_idx = register_file.reg_local_idx(reg);
    assert(local_idx != INVALID_VAL_LOCAL_IDX);
    AssignmentPartRef ap{val_assignment(local_idx), register_file.reg_part(reg)};

    // Skip consistency check for temporary phi registers (used during move resolution)
    if (!ap.register_valid()) {
      continue;
    }

    assert(ap.get_reg() == reg);
    assert(!register_file.is_fixed(reg) || ap.fixed_assignment());
  }
#endif

  if (static_cast<u32>(assignments.delayed_free_lists[block_idx]) != ~0u) {
    auto list_entry = assignments.delayed_free_lists[block_idx];
    while (static_cast<u32>(list_entry) != ~0u) {
      auto *assignment = assignments.value_ptrs[static_cast<u32>(list_entry)];
      auto next_entry = assignment->next_delayed_free_entry;
      derived()->free_assignment(list_entry, assignment);
      list_entry = next_entry;
    }
  }
  return true;
}

} // namespace tpde
