// SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
//
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#pragma once

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <sys/wait.h>
#include <unordered_map>
#include <unordered_set>
#include <variant>
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
using VIR = VerificationIR<Adaptor, BlockIndex, typename Adaptor::IRInstRef>;

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

  CCAssigner(const CCInfo &ccinfo) : ccinfo(&ccinfo) {}
  virtual ~CCAssigner() {}

  virtual void reset() = 0;

  const CCInfo &get_ccinfo() const { return *ccinfo; }

  virtual void assign_arg(CCAssignment &cca) = 0;
  virtual u32 get_stack_size() = 0;
  /// Some calling conventions need different call behavior when calling a
  /// vararg function.
  virtual bool is_vararg() const { return false; }
  virtual void assign_ret(CCAssignment &cca) = 0;
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

  // BlockIndex is now defined at namespace scope in Analyzer.hpp
  using BlockIndex = tpde::BlockIndex;

  using ConfigType = Config;
  using Assembler = typename Config::Assembler;
  using AsmReg = typename Config::AsmReg;

  using RegisterFile = tpde::RegisterFile<Config::NUM_BANKS, 32>;
  using Analyzer = tpde::Analyzer<Adaptor, CompilerBase>;

  /// A default implementation for ValRefSpecial.
  // Note: Subclasses can override this, always used Derived::ValRefSpecial.
  struct ValRefSpecial {
    uint8_t mode = 4;
    u64 const_data;
  };

#pragma region CompilerData
  Adaptor *adaptor;
  Analyzer analyzer;

  // data for frame management

  struct {
    /// The current size of the stack frame
    u32 frame_size = 0;
    /// Whether the stack frame might have dynamic alloca. Dynamic allocas may
    /// require a different and less efficient frame setup. As static allocas
    /// can be converted into dynamic allocas, this is only valid after static
    /// allocas were processed.
    bool has_dynamic_alloca;
    /// Whether the function is guaranteed to be a leaf function. Throughout the
    /// entire function, the compiler may assume the absence of function calls.
    bool is_leaf_function;
    /// Whether the function actually includes a call. There are cases, where it
    /// is not clear from the beginning whether a function has function calls.
    /// If a function has no calls, this will allow using the red zone
    /// guaranteed by some ABIs.
    bool generated_call;
    /// Whether the stack frame is used. If the stack frame is never used, the
    /// frame setup can in some cases be omitted entirely.
    bool frame_used;
    /// Free-Lists for 1/2/4/8/16 sized allocations
    // TODO(ts): make the allocations for 4/8 different from the others
    // since they are probably the one's most used?
    util::SmallVector<i32, 16> fixed_free_lists[5] = {};
    /// Free-Lists for all other sizes
    // TODO(ts): think about which data structure we want here
    std::unordered_map<u32, std::vector<i32>> dynamic_free_lists{};
  } stack = {};

  BlockIndex cur_block_idx;
  u32 cur_instr_idx;
  RegisterFile::RegBitSet used_phi_regs_global = 0;

  // Assignments

  static constexpr ValLocalIdx INVALID_VAL_LOCAL_IDX =
      static_cast<ValLocalIdx>(~0u);

  // TODO(ts): think about different ways to store this that are maybe more
  // compact?
  struct {
    AssignmentAllocator allocator;

    std::array<u32, Config::NUM_BANKS> cur_fixed_assignment_count = {};
    util::SmallVector<ValueAssignment *, Analyzer::SMALL_VALUE_NUM> value_ptrs;

    ValLocalIdx variable_ref_list;
    util::SmallVector<ValLocalIdx, Analyzer::SMALL_BLOCK_NUM>
        delayed_free_lists;
  } assignments = {};

  RegisterFile register_file;
  RegisterFile global_register_file;

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
    RegisterMove(Reg d,
                 Reg s,
                 u8 sz,
                 ValLocalIdx val = INVALID_VAL_LOCAL_IDX,
                 u32 part = 0)
        : value_idx(val), part_idx(part), dst(d), src(s), size(sz) {}
  };
  using MoveList = util::SmallVector<RegisterMove, 16>;

  /// Pending argument for lazy CallBuilder execution
  struct PendingArg {
    enum class Kind : u8 {
      REG_TO_REG,   // Register-to-register move
      STACK_TO_REG, // Load from stack to register
      CONST_TO_REG, // Materialize constant to register
      TO_STACK,     // Store to stack slot
      BYVAL,        // byval memory copy
    };

    Kind kind = Kind::REG_TO_REG;
    u8 int_ext = 0; // Extension: bit 7 = sign, bits 0-5 = width
    u8 size = 0;
    RegBank bank{};
    Reg target_reg = Reg::make_invalid(); // For *_TO_REG kinds
    u32 stack_off = 0;                    // For TO_STACK/BYVAL
    u32 byval_size = 0;                   // For BYVAL

    // Source info
    Reg source_reg = Reg::make_invalid(); // For REG_TO_REG
    i32 frame_off = 0;                    // For STACK_TO_REG
    ValLocalIdx local_idx = INVALID_VAL_LOCAL_IDX;
    u32 part_idx = 0;

    // For constants
    u64 const_data = 0;
    const u64 *const_ptr = nullptr;
    bool const_inline = false;
  };

  using PendingArgList = util::SmallVector<PendingArg, 8>;

  /// Tree Register Allocator context for tracking parallel copies during
  // TreeRAContext retired, replaced with global_register_file
  // /// instruction compilation. Based on the tree scan register allocation
  // /// algorithm which processes operations in a single pass.
  // struct TreeRAContext {
  //   /// List of parallel copies
  //   MoveList parallel_copies;
  //   RegisterFile::RegBitSet used_global_regs = 0;
  //   std::unordered_map<ValLocalIdx, AsmReg> global_regs;
  //   // const IRInstRef *current_instr; // necessary to choose good repair
  //   registers - commented out TreeRAContext(RegisterFile::RegBitSet
  //   used_global_regs,
  //                 const std::unordered_map<ValLocalIdx, AsmReg> &
  //                 global_regs,
  //                 const IRInstRef * /*current_instr*/)
  //     : used_global_regs(used_global_regs),
  //       global_regs(global_regs)/*,
  //       current_instr(current_instr)*/ {
  //   }

  //   explicit TreeRAContext(const IRInstRef * /*current_instr*/) :
  //   used_global_regs(), global_regs() /*,
  //                                                            current_instr{current_instr}*/
  //                                                            {
  //   }

  //   void assign(ValLocalIdx idx, Reg reg) {
  //     assert(reg!=Reg::make_invalid() && "tried to assign invalid register as
  //     global color"); if (this->used_global_regs & (1ull << reg.id())) {
  //       assert(global_regs[idx]==reg);
  //       return;
  //     }
  //     if (global_regs.contains(idx)) {
  //       this->used_global_regs &= ~(1ull << global_regs[idx].id());
  //       this->used_global_regs |= (1ull << reg.id());
  //       global_regs.insert_or_assign(idx, reg);
  //     } else {
  //       this->used_global_regs |= (1ull << reg.id());
  //       global_regs.emplace(idx, reg);
  //     }
  //   }

  //   void unassign(ValLocalIdx idx) {
  //     auto reg = global_regs[idx];
  //     this->used_global_regs &= ~(1ull << global_regs[idx].id());
  //     global_regs.erase(idx);
  //   }
  // };

  // TreeRAContext *tree_ra_ctx = nullptr;
  MoveList parallel_copies;

  void global_assign(ValLocalIdx idx, Reg reg) {
    /*if (global_register_file.is_used(reg) &&
       global_register_file.reg_local_idx(reg) == idx) {
     return;
   }
   if (global_register_file.is_used(reg)) {
     // unassign the old one if different
     // ValLocalIdx old_idx = global_register_file.reg_local_idx(reg);
     global_register_file.unmark_used(reg);
     global_register_file.mark_used(reg, idx, 0); // assume part 0 for now
   } else {
     global_register_file.mark_used(reg, idx, 0);
   }*/
  }

  void global_unassign(ValLocalIdx idx) {
    /*for (auto reg_id : global_register_file.used_regs()) {
      if (global_register_file.reg_local_idx(Reg{reg_id}) == idx) {
        global_register_file.unmark_used(Reg{reg_id});
        break;
      }
    }*/
  }

  Reg global_reg_for(ValLocalIdx idx) const {
    /*for (auto reg_id : global_register_file.used_regs()) {
      if (global_register_file.reg_local_idx(Reg{reg_id}) == idx) {
        return Reg{reg_id};
      }
    }
    */
    return Reg::make_invalid();
  }
#ifndef NDEBUG
  /// Whether we are currently in the middle of generating branch-related code
  /// and therefore must not change any value-related state.
  bool generating_branch = false;
#endif
  struct RegisterState {
    std::array<ValLocalIdx, 64> registers{};
    bool valid = false;
    RegisterState() { registers.fill(INVALID_VAL_LOCAL_IDX); }
  };

  util::SmallVector<RegisterState, Analyzer::SMALL_BLOCK_NUM> block_states;

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
  using PhiRegList = util::SmallVector<Reg>;
  using PhiRegMap = std::unordered_map<ValLocalIdx, PhiRegList>;
  std::unordered_map<BlockIndex, PhiRegMap> phi_regs;

#ifndef NDEBUG
  VIR<Adaptor> verification_ir;
#endif

private:
  /// Default CCAssigner if the implementation doesn't override cur_cc_assigner.
  typename Config::DefaultCCAssigner default_cc_assigner;

public:
  typename Config::Assembler assembler;
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

  /// Call argument, enhancing an IRValueRef with information on how to pass it.
  struct CallArg {
    enum class Flag : u8 {
      none,        ///< No extra handling.
      zext,        ///< Scalar integer, zero-extend to target-specific size.
      sext,        ///< Scalar integer, sign-extend to target-specific size.
      sret,        ///< Struct return pointer.
      byval,       ///< Value is copied into corresponding stack slot.
      allow_split, ///< Value parts can be split across stack/registers.
    };

    explicit CallArg(IRValueRef value,
                     Flag flags = Flag::none,
                     u8 byval_align = 1,
                     u32 byval_size = 0)
        : value(value),
          flag(flags),
          byval_align(byval_align),
          byval_size(byval_size) {}

    IRValueRef value; ///< Argument IR value.
    Flag flag;        ///< Value handling flag.
    u8 byval_align;   ///< For Flag::byval, the stack alignment.
    u8 ext_bits = 0;  ///< For Flag::zext and Flag::sext, the source bit width.
    u32 byval_size;   ///< For Flag::byval, the argument size.
  };

  /// Base class for target-specific CallBuilder implementations.
  template <typename CBDerived>
  class CallBuilderBase {
  protected:
    Derived &compiler;
    CCAssigner &assigner;

    RegisterFile::RegBitSet arg_regs{};

    // Pending arguments for lazy execution
    PendingArgList pending_args{};
    // Track source registers to detect eviction
    RegisterFile::RegBitSet source_regs{};

  public:
    CallBuilderBase(Derived &compiler, CCAssigner &assigner)
        : compiler(compiler), assigner(assigner) {}

    // CBDerived needs:
    // void add_arg_byval(ValuePart &vp, CCAssignment &cca);
    // void add_arg_stack(ValuePart &vp, CCAssignment &cca);
    // void call_impl(std::variant<SymRef, ValuePart> &&);
    CBDerived *derived() { return static_cast<CBDerived *>(this); }

  public:
    /// Add a value part as argument. cca must be populated with information
    /// about the argument, except for the reg/stack_off, which are set by the
    /// CCAssigner. If no register bank is assigned, the register bank and size
    /// are retrieved from the value part, otherwise, the size must be set, too.
    void add_arg(ValuePart &&vp, CCAssignment cca);
    /// Add a full IR value as argument, with an explicit number of parts.
    /// Values are decomposed into their parts and are typically either fully
    /// in registers or fully on the stack (except CallArg::Flag::allow_split).
    void add_arg(const CallArg &arg, u32 part_count);
    /// Add a full IR value as argument. The number of value parts must be
    /// exposed via val_parts. Values are decomposed into their parts and are
    /// typically either fully in registers or fully on the stack (except
    /// CallArg::Flag::allow_split).
    void add_arg(const CallArg &arg) {
      add_arg(std::move(arg), compiler.adaptor->val_parts(arg.value).count());
    }

    /// Generate the function call (evict registers, call, reset stack frame).
    void call(std::variant<SymRef, ValuePart>);

    /// Assign next return value part to vp.
    void add_ret(ValuePart &vp, CCAssignment cca);
    /// Assign next return value part to vp.
    void add_ret(ValuePart &&vp, CCAssignment cca) { add_ret(vp, cca); }
    /// Assign return values to the IR value.
    void add_ret(ValueRef &vr);
  };

  class RetBuilder {
    Derived &compiler;
    CCAssigner &assigner;

    RegisterFile::RegBitSet ret_regs{};

  public:
    RetBuilder(Derived &compiler, CCAssigner &assigner)
        : compiler(compiler), assigner(assigner) {
      assigner.reset();
    }

    void add(ValuePart &&vp, CCAssignment cca);
    void add(IRValueRef val);

    void ret();
  };

  /// Initialize a CompilerBase, should be called by the derived classes
  explicit CompilerBase(Adaptor *adaptor)
      : adaptor(adaptor), analyzer(adaptor, this), assembler() {
    static_assert(std::is_base_of_v<CompilerBase, Derived>);
    static_assert(Compiler<Derived, Config>);
  }

  /// shortcut for casting to the Derived class so that overloading
  /// works
  Derived *derived() { return static_cast<Derived *>(this); }

  const Derived *derived() const { return static_cast<const Derived *>(this); }

  [[nodiscard]] ValLocalIdx val_idx(const IRValueRef value) const {
    return analyzer.adaptor->val_local_idx(value);
  }

  [[nodiscard]] ValueAssignment *val_assignment(const ValLocalIdx idx) {
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
  VIR<Adaptor>::Allocation get_allocation(AssignmentPartRef ap,
                                          const bool reload = false) const {
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
  void vir_final_assignment(ValLocalIdx local_idx, Reg reg) {
    final_assignments[local_idx].push_back(reg);
  }

  // Record argument move destination for VIR tracking
  void vir_record_arg_move(ValLocalIdx local_idx, u32 part_idx, Reg reg) {
    auto &vec = final_assignments[local_idx];
    if (vec.size() <= part_idx) {
      vec.resize(part_idx + 1, Reg::make_invalid());
    }
    vec[part_idx] = reg;
  }

public:
  void vir_emit_new_use(Reg reg) {
    final_assignments[INVALID_VAL_LOCAL_IDX].push_back(reg);
  }
  void vir_emit_def(Reg reg) { verification_ir.materialize_constant(reg); }

private:
  /// Emit PHI node assignment (debug-only)
  void vir_emit_phi(IRValueRef phi_value,
                    ValLocalIdx phi_idx,
                    u32 part_idx,
                    Reg reg) {
    using VIRType = VIR<Adaptor>;

    typename VIRType::Allocation phi_alloc(reg);
    ValueAssignment *assignment = val_assignment(phi_idx);
    if (assignment) {
      AssignmentPartRef ap{assignment, part_idx};
      phi_alloc = get_allocation(ap);
    }

    // Capture incoming values and their source blocks
    util::SmallVector<
        std::tuple<BlockIndex, ValLocalIdx, typename VIRType::Allocation>,
        4>
        incoming_data;
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

      incoming_data.push_back(std::make_tuple(
          incoming_block_idx, incoming_val_idx, incoming_alloc));
    }

    verification_ir.vir_emit_phi(phi_idx, part_idx, phi_alloc, incoming_data);
  }

public:
#endif

  /// Get CCAssigner for current function.
  CCAssigner *cur_cc_assigner() { return &default_cc_assigner; }

  void init_assignment(IRValueRef value, ValLocalIdx local_idx);

private:
  /// Frees an assignment, its stack slot and registers
  void free_assignment(ValLocalIdx local_idx, ValueAssignment *);

public:
  /// Release an assignment when reference count drops to zero, either frees
  /// the assignment immediately or delays free to the end of the live range.
  void release_assignment(ValLocalIdx local_idx, ValueAssignment *);

  /// Init a variable-ref assignment
  void init_variable_ref(ValLocalIdx local_idx, u32 var_ref_data);
  /// Init a variable-ref assignment
  void init_variable_ref(IRValueRef value, u32 var_ref_data) {
    init_variable_ref(adaptor->val_local_idx(value), var_ref_data);
  }

  /// \name Stack Slots
  /// @{

  /// Allocate a static stack slot.
  i32 allocate_stack_slot(u32 size);
  /// Free a static stack slot.
  void free_stack_slot(u32 slot, u32 size);

  /// @}

  /// Assign function argument in prologue. \ref align can be used to increase
  /// the minimal stack alignment of the first part of the argument. If \ref
  /// allow_split is set, the argument can be passed partially in registers,
  /// otherwise (default) it must be either passed completely in registers or
  /// completely on the stack.
  void prologue_assign_arg(CCAssigner *cc_assigner,
                           u32 arg_idx,
                           IRValueRef arg,
                           u32 align = 1,
                           bool allow_split = false);

  /// \name Value References
  /// @{

  /// Get a using reference to a value.
  ValueRef val_ref(IRValueRef value);

  /// Get a using reference to a single-part value and provide direct access to
  /// the only part. This is a convenience function; note that the ValueRef must
  /// outlive the ValuePartRef (i.e. auto p = val_ref().part(0); won't work, as
  /// the value will possibly be deallocated when the ValueRef is destroyed).
  std::pair<ValueRef, ValuePartRef> val_ref_single(IRValueRef value);

  /// Get a defining reference to a value.
  ValueRef result_ref(IRValueRef value);

  /// Get a defining reference to a single-part value and provide direct access
  /// to the only part. Similar to val_ref_single().
  std::pair<ValueRef, ValuePartRef> result_ref_single(IRValueRef value);

  /// Make dst an alias for src, which must be a non-constant value with an
  /// identical part configuration. src must be in its last use (is_owned()),
  /// and the assignment will be repurposed for dst, keeping all assigned
  /// registers and stack slots.
  ValueRef result_ref_alias(IRValueRef dst, ValueRef &&src);

  /// Initialize value as a pointer into a stack variable (i.e., a value
  /// allocated from cur_static_allocas() or similar) with an offset. The
  /// result value will be a stack variable itself.
  ValueRef
      result_ref_stack_slot(IRValueRef value, AssignmentPartRef base, i32 off);

  /// @}

  [[deprecated("Use ValuePartRef::set_value")]]
  void set_value(ValuePartRef &val_ref, ScratchReg &scratch);
  [[deprecated("Use ValuePartRef::set_value")]]
  void set_value(ValuePartRef &&val_ref, ScratchReg &scratch) {
    set_value(val_ref, scratch);
  }

  /// Get generic value part into a single register, evaluating expressions
  /// and materializing immediates as required.
  AsmReg gval_as_reg(GenericValuePart &gv);

  /// Like gval_as_reg; if the GenericValuePart owns a reusable register
  /// (either a ScratchReg, possibly due to materialization, or a reusable
  /// ValuePartRef), store it in dst.
  AsmReg gval_as_reg_reuse(GenericValuePart &gv, ScratchReg &dst);

  /// Like gval_as_reg; if the GenericValuePart owns a reusable register
  /// (either a ScratchReg, possibly due to materialization, or a reusable
  /// ValuePartRef), store it in dst.
  AsmReg gval_as_reg_reuse(GenericValuePart &gv, ValuePart &dst);

  bool repair_argument(ValLocalIdx var,
                       u32 part,
                       u8 size,
                       RegBank bank,
                       typename RegisterFile::RegBitSet constraints,
                       typename RegisterFile::RegBitSet available,
                       typename RegisterFile::RegBitSet forbidden = 0,
                       AsmReg current_reg = AsmReg::make_invalid());

private:
  /// @internal Select register when a value needs to be evicted.
  Reg select_reg_evict(RegBank bank);

public:
  /// \name Low-Level Assignment Register Handling
  /// @{

  /// Select an available register, evicting loaded values if needed.
  /// Return local, global register
  Reg select_reg(RegBank bank, u64 exclusion_mask) {
    // todo(salto): fix lookups to global reg file ex. add_i128_no_salvage_reg
    Reg res = register_file.find_first_free_excluding(bank, exclusion_mask);
    if (res.valid()) [[likely]] {
      return res;
    }

    return select_reg_evict(bank);
  }

  /// Reload a value part from memory or recompute variable address.
  void reload_to_reg(AsmReg dst, AssignmentPartRef ap);

  /// Allocate a stack slot for an assignment.
  void allocate_spill_slot(AssignmentPartRef ap);

  /// Ensure the value is spilled in its stack slot (except variable refs).
  void spill(AssignmentPartRef ap);

  /// Evict the value from its register, spilling if needed, and free register.
  void evict(AssignmentPartRef ap);

  /// Evict the value from the register, spilling if needed, and free register.
  void evict_reg(Reg reg);

  /// Lazily free a register using parallel moves when possible.
  void lazy_free_reg(Reg reg);

  /// Free the register. Requires that the contained value is already spilled.
  void free_reg(Reg reg);

  /// Spill all caller-saved registers before a call that may branch. (ex.
  /// LLVMIR invoke)
  typename RegisterFile::RegBitSet spill_caller_saved_before_call(
      typename RegisterFile::RegBitSet call_arguments);

  /// @}

  /// \name High-Level Branch Generation
  /// @{

  /// Generate an unconditional branch at the end of a basic block. No further
  /// instructions must follow. If target is the next block in the block order,
  /// the branch is omitted.
  void generate_uncond_branch(IRBlockRef target);

  /// Generate an conditional branch at the end of a basic block.
  template <typename Jump>
  void generate_cond_branch(Jump jmp,
                            IRBlockRef true_target,
                            IRBlockRef false_target);

  /// Generate a switch at the end of a basic block. Only the lowest bits of the
  /// condition are considered. The condition must be a general-purpose
  /// register. The cases must be sorted and every case value must appear at
  /// most once.
  void generate_switch(ScratchReg &&cond,
                       u32 width,
                       IRBlockRef default_block,
                       std::span<const std::pair<u64, IRBlockRef>> cases);

  /// @}

  /// \name Low-Level Branch Primitives
  /// The general flow of using these low-level primitive is:
  /// 1. spill_before_branch()
  /// 2. begin_branch_region()
  /// 3. One or more calls to generate_branch_to_block()
  /// 4. end_branch_region()
  /// 5. release_spilled_regs()
  /// @{

  // TODO(ts): switch to a branch_spill_before naming style?
  /// Spill values that need to be spilled for later blocks. Returns the set
  /// of registers that will be free'd at the end of the block; pass this to
  /// release_spilled_regs().
  typename RegisterFile::RegBitSet
      spill_before_branch(bool force_spill = false);
  /// Free registers marked by spill_before_branch().
  void release_spilled_regs(typename RegisterFile::RegBitSet);

  /// When reaching a point in the function where no other blocks will be
  /// reached anymore, use this function to release register assignments after
  /// the end of that block so the compiler does not accidentally use
  /// registers which don't contain any values
  void release_regs_after_return();

  /// Indicate beginning of region where value-state must not change.
  void begin_branch_region() {
#ifndef NDEBUG
    assert(!generating_branch);
    generating_branch = true;
#endif
  }

  /// Indicate end of region where value-state must not change.
  void end_branch_region() {
#ifndef NDEBUG
    assert(generating_branch);
    generating_branch = false;
    verification_ir.end_branch();
#endif
  }

  /// Generate a branch to a basic block; execution continues afterwards.
  /// Multiple calls to this function can be used to build conditional branches.
  /// @tparam Jump Target-defined Jump type (e.g., CompilerX64::Jump).
  /// @param needs_split Result of branch_needs_split(); pass false for an
  ///  unconditional branch.
  /// @param last_inst Whether fall-through to target is possible.
  template <typename Jump>
  void generate_branch_to_block(Jump jmp,
                                IRBlockRef target,
                                bool needs_split,
                                bool last_inst);

#ifndef NDEBUG
  // todo(salto): double check that this is really always ok
  bool may_change_value_state() const { return true; }

#endif


  void move_one(u32 i, MoveList &moves, MoveList &result) {
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
          auto tmp =
              derived()->select_reg(register_file.reg_bank(moves[j].src), 0);
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
    result.emplace_back(moves[i].dst,
                        moves[i].src,
                        moves[i].size,
                        moves[i].value_idx,
                        moves[i].part_idx);
    moves[i].status = MoveStatus::DONE;
  }

  /*
  Order a list of moves in a way that they behave as if they are executed in
  parralel. Avoids swap problem and resolves dependency chains between moves.
  See: Silvain Rideau and Xavier Leroy. 2010. Validating register
  allocation and spilling.
  */
  MoveList sequentialize(MoveList &moves) {
    MoveList result;
    for (u32 i = 0; i < moves.size(); ++i) {
      if (moves[i].status == MoveStatus::TO_MOVE) {
        move_one(i, moves, result);
      }
    }
    return result;
  }

  void move_values_to_match(BlockIndex target) {
    // next block immediately follows the current block and there is no control
    // flow inbetween. We can use the Register state of the current block for
    // the next one.
    // no moves necessary

    MoveList moves;
    typename RegisterFile::RegBitSet phi_regs = 0;
    if (analyzer.block_has_phis(target)) {
      phi_regs = move_to_phi_nodes_impl(target, moves);
    }

    auto block_state_it = block_regs.find(target);
    // another branch to target was already generated, we *must* use the same
    // register layout
    if (block_state_it != block_regs.end()) {
      for (ValueState &state : block_state_it->second) {
        // registers must be cleared. Ex. landing pads
        if (state.val_local_idx == INVALID_VAL_LOCAL_IDX) {
          for (u32 i = 0; i < state.registers.size(); ++i) {
            if (state.registers[i].valid()) {
              this->evict_reg(state.registers[i]);
            }
          }
          continue;
        }
        ValueAssignment *va = val_assignment(state.val_local_idx);
        // value was already freed, won't be used again
        if (!va) {
          continue;
        }
        for (u32 i = 0; i < va->part_count; ++i) {
          AssignmentPartRef ap{va, i};
          auto cur_reg = ap.get_reg();
          if (!state.registers[i].valid()) {
            continue;
          }
          if (!ap.register_valid()) {
            if (register_file.is_used(state.registers[i]) ||
                (phi_regs & (1ull << state.registers[i].id()))) {
              auto reg = this->select_reg(
                  register_file.reg_bank(state.registers[i]), phi_regs);

              reload_to_reg(reg, ap);
              register_file.mark_used(reg, state.val_local_idx, i);
              ap.set_register_valid(true);
              ap.set_reg(reg);
              cur_reg = reg;
            } else {
              if (ap.stack_valid()) {
                reload_to_reg(state.registers[i], ap);
              }
              continue;
            }
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
        // our register is used as a phi. if we still need the value otherwise,
        // we either need to move it to a different reg or spill it.
        // todo(salto): implement moving to different reg.
        if (phi_regs & (1ull << reg)) {
          if (this->phi_regs[target].contains(local_idx)) {
            // value is the phi, no need to do anything
            continue;
          }
          ValueAssignment *assignment = val_assignment(local_idx);
          if (!assignment) {
            continue;
          }
          for (u32 i = 0; i < assignment->part_count; i++) {
            AssignmentPartRef ap{assignment, i};
            if (ap.fixed_assignment()) {
              // fixed registers do not need to be moved
              continue;
            }
            if (!ap.modified() || ap.variable_ref()) {
              // No need to spill values that were already spilled or are
              // variable refs.
              continue;
            }
            if (assignment->pending_free) {
              continue;
            }
            // this assignment is a phi
            if (!ap.register_valid()) {
              continue;
            }
            spill(ap);
          }
          continue;
        }
        if (seen.contains(local_idx)) {
          continue;
        }
        seen.insert(local_idx);
        auto *assignment = val_assignment(local_idx);

        block_regs[target].emplace_back(local_idx, assignment->part_count);
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
          // fix global colors
          Reg global_reg = global_reg_for(local_idx);
          if (global_reg.valid() && global_reg != ap.get_reg()) {
            moves.emplace_back(
                global_reg, ap.get_reg(), ap.part_size(), local_idx, i);
            this->register_file.unmark_used(ap.get_reg());
            this->register_file.mark_used(global_reg, local_idx, i);
            ap.set_reg(global_reg);
            block_regs[target][block_regs[target].size() - 1].push_back(
                global_reg, i);
          } else {
            block_regs[target][block_regs[target].size() - 1].push_back(
                Reg{ap.get_reg()}, i);
          }
        }
      }
    }

    // prevent any phi registers from being used as temporaries for swap resolution″
    auto prev_alloc = register_file.allocatable & phi_regs;
    register_file.allocatable &= ~phi_regs;
    MoveList result = sequentialize(moves);
    register_file.allocatable |= prev_alloc;
    // todo(salto): maybe execute the mov in sequentialize directly
    for (auto move : result) {
      if (move.value_idx != INVALID_VAL_LOCAL_IDX) {
        ValueAssignment *assignment = this->val_assignment(move.value_idx);
        if (!assignment) {
          continue;
        }
        AssignmentPartRef ap{assignment, move.part_idx};

        if (ap.fixed_assignment()) {
          this->derived()->mov(move.dst, move.src, ap.part_size());
          continue;
        }
        this->global_assign(move.value_idx, move.dst);
        if (register_file.is_used(Reg{move.dst}) &&
            move.value_idx != register_file.reg_local_idx(Reg{move.dst})) {
          this->evict_reg(Reg{move.dst});
        }
        this->derived()->mov(move.dst, move.src, ap.part_size());
        if (ap.register_valid() && register_file.is_used(ap.get_reg())) {
          register_file.unmark_used(ap.get_reg());
        }
        if (!register_file.is_used(Reg{move.src}) &&
            register_file.allocatable & ~(1ull << move.src.id())) {
          register_file.allocatable |= (1ull << move.src.id());
        }
        ap.set_register_valid(true);
        ap.set_reg(Reg{move.dst});
        this->register_file.mark_used(
            Reg{move.dst}, move.value_idx, move.part_idx);
        this->register_file.mark_clobbered(Reg{move.dst});
      } else {
        this->derived()->mov(move.dst, move.src, 8);
        this->register_file.mark_used(
            Reg{move.dst}, move.value_idx, move.part_idx);
        this->register_file.mark_clobbered(Reg{move.dst});
      }
    }
  }

  typename RegisterFile::RegBitSet move_to_phi_nodes_impl(BlockIndex target,
                                                          MoveList &moves);

  /// Count available registers in a specific bank
  u32 count_available_registers(RegBank bank) const {
    auto free_regs = register_file.allocatable & ~register_file.used &
                     register_file.bank_regs(bank);
    return std::popcount(free_regs);
  }

  /// Count total available registers across all banks
  u32 count_total_available_registers() const {
    auto free_regs = register_file.allocatable & ~register_file.used;
    return std::popcount(free_regs);
  }

  /// Whether branch to a block requires additional instructions and therefore
  /// a direct jump to the block is not possible.
  bool branch_needs_split(IRBlockRef target) {
    // for now, if the target has PHI-nodes, we split
    return analyzer.block_has_phis(target);
  }

  /// @}

  BlockIndex next_block() const;

  bool try_force_fixed_assignment(IRValueRef) const { return false; }

  bool hook_post_func_sym_init() { return true; }

  void analysis_start() {}

  void analysis_end() {}

  void reloc_text(SymRef sym, u32 type, u64 offset, i64 addend = 0) {
    this->assembler.reloc_sec(
        text_writer.get_sec_ref(), sym, type, offset, addend);
  }

  /// Convenience function to place a label at the current position.
  void label_place(Label label) {
    this->text_writer.label_place(label, text_writer.offset());
  }

protected:
  SymRef get_personality_sym();

  bool compile_func(IRFuncRef func, u32 func_idx);

  bool compile_block(IRBlockRef block, u32 block_idx);
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
    CBDerived>::add_arg(ValuePart &&vp, CCAssignment cca) {
  if (!cca.byval && cca.bank == RegBank{}) {
    cca.bank = vp.bank();
    cca.size = vp.part_size();
  }

  assigner.assign_arg(cca);

  PendingArg arg{};
  arg.int_ext = cca.int_ext;
  arg.size = cca.size;
  arg.bank = cca.bank;

  if (cca.byval) {
    // can't really do this better
    derived()->add_arg_byval(vp, cca);
    vp.reset(&compiler);
    return;
  }

  if (!cca.reg.valid()) {
    // Stack destination - execute immediately
    bool needs_ext = cca.int_ext != 0;
    bool ext_sign = cca.int_ext >> 7;
    unsigned ext_bits = cca.int_ext & 0x3f;

    if (needs_ext) {
      auto ext = std::move(vp).into_extended(&compiler, ext_sign, ext_bits, 64);
      derived()->add_arg_stack(ext, cca);
      ext.reset(&compiler);
    } else {
      derived()->add_arg_stack(vp, cca);
    }
    vp.reset(&compiler);
    return;
  }

  // Register destination
  arg.target_reg = cca.reg;
  arg_regs |= (1ull << cca.reg.id());

  if (vp.is_const()) {
    arg.kind = PendingArg::Kind::CONST_TO_REG;
    auto cdata = vp.const_data();
    // For constants that fit in 8 bytes, copy inline
    if (arg.size <= 8) {
      arg.const_data = cdata[0];
      arg.const_inline = true;
    } else {
      arg.const_ptr = cdata.data();
      arg.const_inline = false;
    }
  } else if (vp.has_assignment()) {
    auto ap = vp.assignment();
    arg.local_idx = vp.local_idx();
    arg.part_idx = vp.part();
    if (ap.register_valid()) {
      arg.kind = PendingArg::Kind::REG_TO_REG;
      arg.source_reg = ap.get_reg();
      source_regs |= (1ull << ap.get_reg().id());
      compiler.register_file.allocatable &= ~source_regs;
      /*if(ap.assignment()->pending_free || ap.assignment()->references_left
      <=1){ compiler.register_file.update_reg_assignment(arg.source_reg,
      INVALID_VAL_LOCAL_IDX, 0); ap.set_register_valid(false);
      }*/

    } else if (ap.stack_valid()) {
      arg.kind = PendingArg::Kind::STACK_TO_REG;
      if (!ap.variable_ref()) {
        arg.frame_off = ap.frame_off();
      }
    } else {
      // var-refs and sret
      // todo(salto): test unlikely
      //  not worth optimizing
      if (compiler.register_file.is_used(cca.reg)) {
        compiler.evict_reg(cca.reg);
      }

      vp.load_to_specific(&compiler, cca.reg);
      source_regs |= (1ull << cca.reg.id());
      compiler.register_file.allocatable &= ~source_regs;
      vp.reset(&compiler);
      return;
    }
  } else {
    // Temporary register without assignment
    AsmReg src = vp.cur_reg_unlocked();
    if (src.valid()) {
      arg.kind = PendingArg::Kind::REG_TO_REG;
      arg.source_reg = src;
      source_regs |= (1ull << src.id());
      compiler.register_file.allocatable &= ~source_regs;
    }
  }
  // todo(salto): alloca_call

  pending_args.push_back(arg);
  vp.reset(&compiler);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
template <typename CBDerived>
void CompilerBase<Adaptor, Derived, Config>::CallBuilderBase<
    CBDerived>::add_arg(const CallArg &arg, u32 part_count) {
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

  u32 align = arg.byval_align;
  bool allow_split = arg.flag == CallArg::Flag::allow_split;

  for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
    u8 int_ext = 0;
    if (arg.flag == CallArg::Flag::sext || arg.flag == CallArg::Flag::zext) {
      assert(arg.ext_bits != 0 && "cannot extend zero-bit integer");
      int_ext = arg.ext_bits | (arg.flag == CallArg::Flag::sext ? 0x80 : 0);
    }
    u32 remaining = part_count < 256 ? part_count - part_idx - 1 : 255;
    derived()->add_arg(vr.part(part_idx),
                       CCAssignment{
                           .consecutive = u8(allow_split ? 0 : remaining),
                           .sret = arg.flag == CallArg::Flag::sret,
                           .int_ext = int_ext,
                           .align = u8(part_idx == 0 ? align : 1),
                       });
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
template <typename CBDerived>
void CompilerBase<Adaptor, Derived, Config>::CallBuilderBase<CBDerived>::call(
    std::variant<SymRef, ValuePart> target) {
  assert(!compiler.stack.is_leaf_function && "leaf func must not have calls");
  compiler.stack.generated_call = true;
  compiler.spill_caller_saved_before_call(arg_regs);

  // Phase 1: Update evicted sources - check if any REG_TO_* sources were
  // spilled
  for (auto &arg : pending_args) {
    if (arg.kind != PendingArg::Kind::REG_TO_REG &&
        arg.kind != PendingArg::Kind::TO_STACK) {
      continue;
    }
    if (arg.local_idx == INVALID_VAL_LOCAL_IDX) {
      continue;
    }

    ValueAssignment *va = compiler.val_assignment(arg.local_idx);
    if (!va) {
      continue;
    }

    AssignmentPartRef ap{va, arg.part_idx};
    if (!ap.register_valid()) {
      // Was evicted - switch to stack load
      if (arg.kind == PendingArg::Kind::REG_TO_REG) {
        arg.kind = PendingArg::Kind::STACK_TO_REG;
      }
      // For TO_STACK, we'll handle it by loading to temp first
      if (ap.stack_valid()) {
        arg.frame_off = ap.frame_off();
      }
    } else if (ap.get_reg() != arg.source_reg) {
      // Moved to different register
      arg.source_reg = ap.get_reg();
    }
  }

  // Phase 2: Execute byval copies
  for (auto &arg : pending_args) {
    if (arg.kind != PendingArg::Kind::BYVAL) {
      continue;
    }

    // Create ValuePart for source
    ValuePart vp{arg.bank};
    if (arg.local_idx != INVALID_VAL_LOCAL_IDX) {
      ValueAssignment *va = compiler.val_assignment(arg.local_idx);
      if (va) {
        vp = ValuePart{arg.local_idx, va, arg.part_idx, false};
      }
    }

    CCAssignment cca{
        .byval = true,
        .size = arg.byval_size,
        .stack_off = arg.stack_off,
    };
    derived()->add_arg_byval(vp, cca);
    vp.reset(&compiler);
  }

  // Phase 3: Build parallel move list for register-to-register moves
  MoveList moves;
  for (auto &arg : pending_args) {
    if (arg.kind != PendingArg::Kind::REG_TO_REG) {
      continue;
    }
    if (arg.source_reg == arg.target_reg) {
      if (arg.int_ext != 0) {
        // todo(salto): can there be a case where we need the upper bits of reg
        // for a different arg?
        bool ext_sign = arg.int_ext >> 7;
        unsigned ext_bits = arg.int_ext & 0x3f;
        compiler.generate_raw_intext(
            arg.target_reg, arg.target_reg, ext_sign, ext_bits, 64);
      }

      // Already in place - just mark clobbered
      if (compiler.register_file.is_used(arg.target_reg)) {
        compiler.evict_reg(arg.target_reg);
      }
      compiler.register_file.mark_clobbered(arg.target_reg);
      compiler.register_file.allocatable &= ~(u64{1} << arg.target_reg.id());
      continue;
    }

    RegisterMove move{
        arg.target_reg, arg.source_reg, arg.size, arg.local_idx, arg.part_idx};
    moves.push_back(move);
  }

  // Phase 4: Sequentialize and execute register-to-register moves
  if (!moves.empty()) {
    // seqeuntalize may need an additional register and one of the arguments can
    // already be freed. make sure it never uses those registers.
    compiler.register_file.allocatable &= ~source_regs;
    MoveList ordered = compiler.sequentialize(moves);
    for (auto &move : ordered) {
      // Find the corresponding pending arg to check for extensions
      u8 int_ext = 0;
      for (const auto &arg : pending_args) {
        if (arg.kind == PendingArg::Kind::REG_TO_REG &&
            arg.target_reg == move.dst) {
          int_ext = arg.int_ext;
          break;
        }
      }

      if (compiler.register_file.is_used(move.dst)) {
        compiler.evict_reg(move.dst);
      }


      if (int_ext != 0) {
        bool ext_sign = int_ext >> 7;
        unsigned ext_bits = int_ext & 0x3f;
        compiler.generate_raw_intext(
            move.dst, move.src, ext_sign, ext_bits, 64);
      } else {
        compiler.mov(move.dst, move.src, move.size);
      }
#ifndef NDEBUG
      // Emit the move for VIR tracking and record final location
      if (move.value_idx != INVALID_VAL_LOCAL_IDX) {
        compiler.verification_ir.emit_call_arg_move(
            move.src, move.dst, move.size);
        compiler.vir_record_arg_move(move.value_idx, move.part_idx, move.dst);
      }
#endif
      compiler.register_file.mark_clobbered(move.dst);
      compiler.register_file.allocatable &= ~(u64{1} << move.dst.id());
    }
  }

  // Phase 5: Execute stack-to-register loads
  for (auto &arg : pending_args) {
    if (arg.kind != PendingArg::Kind::STACK_TO_REG) {
      continue;
    }

    // Evict target register if used
    if (compiler.register_file.is_used(arg.target_reg)) {
      compiler.evict_reg(arg.target_reg);
    }

    if (arg.local_idx != INVALID_VAL_LOCAL_IDX) {
      if (ValueAssignment *va = compiler.val_assignment(arg.local_idx);
          va && va->variable_ref && va->stack_variable) {
        AssignmentPartRef ap{va, arg.part_idx};
        compiler.reload_to_reg(arg.target_reg, ap);
      } else {
        compiler.load_from_stack(arg.target_reg, arg.frame_off, arg.size);
      }
    } else {
      compiler.load_from_stack(arg.target_reg, arg.frame_off, arg.size);
    }

    // Handle extension if needed
    if (arg.int_ext != 0) {
      bool ext_sign = arg.int_ext >> 7;
      unsigned ext_bits = arg.int_ext & 0x3f;
      compiler.generate_raw_intext(
          arg.target_reg, arg.target_reg, ext_sign, ext_bits, 64);
    }

    compiler.register_file.mark_clobbered(arg.target_reg);
    compiler.register_file.allocatable &= ~(u64{1} << arg.target_reg.id());
  }

  // Phase 6: Materialize constants to registers
  for (auto &arg : pending_args) {
    if (arg.kind != PendingArg::Kind::CONST_TO_REG) {
      continue;
    }

    // Evict target register if used
    if (compiler.register_file.is_used(arg.target_reg)) {
      compiler.evict_reg(arg.target_reg);
    }

    // Create constant ValuePart and load to register
    ValuePart vp{arg.bank};
    if (arg.const_inline) {
      vp = ValuePart{arg.const_data, arg.size, arg.bank};
    } else {
      vp = ValuePart{arg.const_ptr, arg.size, arg.bank};
    }

    vp.reload_into_specific_fixed(&compiler, arg.target_reg);

    // Handle extension if needed
    if (arg.int_ext != 0) {
      bool ext_sign = arg.int_ext >> 7;
      unsigned ext_bits = arg.int_ext & 0x3f;
      compiler.generate_raw_intext(
          arg.target_reg, arg.target_reg, ext_sign, ext_bits, 64);
    }

    vp.reset(&compiler);
    compiler.register_file.mark_clobbered(arg.target_reg);
    compiler.register_file.allocatable &= ~(u64{1} << arg.target_reg.id());
  }

  // Phase 7: Evict remaining clobbered registers
  typename RegisterFile::RegBitSet skip_evict = 0;
  if (auto *vp = std::get_if<ValuePart>(&target); vp && vp->can_salvage()) {
    assert(vp->cur_reg_unlocked().valid() && "can_salvage implies register");
    skip_evict |= (1ull << vp->cur_reg_unlocked().id());
  }

  auto clobbered = ~assigner.get_ccinfo().callee_saved_regs;
  for (auto reg_id : util::BitSetIterator<>{compiler.register_file.used &
                                            clobbered & ~skip_evict}) {
    compiler.evict_reg(AsmReg{reg_id});
    compiler.register_file.mark_clobbered(Reg{reg_id});
  }

  // Phase 8: Execute call
  derived()->call_impl(std::move(target));

  // Phase 9: Reset state
  // assert((compiler.register_file.allocatable & arg_regs) == 0);
  compiler.register_file.allocatable |= arg_regs;
  compiler.register_file.allocatable |= source_regs;
  pending_args.clear();
  source_regs = 0;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
template <typename CBDerived>
void CompilerBase<Adaptor, Derived, Config>::CallBuilderBase<
    CBDerived>::add_ret(ValuePart &vp, CCAssignment cca) {
  cca.bank = vp.bank();
  cca.size = vp.part_size();
  assigner.assign_ret(cca);
  assert(cca.reg.valid() && "return value must be in register");
  vp.set_value_reg(&compiler, cca.reg);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
template <typename CBDerived>
void CompilerBase<Adaptor, Derived, Config>::CallBuilderBase<
    CBDerived>::add_ret(ValueRef &vr) {
  assert(vr.has_assignment());
  u32 part_count = vr.assignment()->part_count;
  for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
    CCAssignment cca;
    add_ret(vr.part(part_idx), cca);
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::RetBuilder::add(ValuePart &&vp,
                                                             CCAssignment cca) {
  cca.bank = vp.bank();
  u32 size = cca.size = vp.part_size();
  assigner.assign_ret(cca);
  assert(cca.reg.valid() && "indirect return value must use sret argument");

  bool needs_ext = cca.int_ext != 0;
  bool ext_sign = cca.int_ext >> 7;
  unsigned ext_bits = cca.int_ext & 0x3f;

  // todo(salto): avoid spills here
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
void CompilerBase<Adaptor, Derived, Config>::RetBuilder::add(IRValueRef val) {
  u32 part_count = this->compiler.adaptor->val_parts(val).count();
  ValueRef vr = compiler.val_ref(val);
  for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
    add(vr.part(part_idx), CCAssignment{});
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::RetBuilder::ret() {
  assert((compiler.register_file.allocatable & ret_regs) == 0);
  compiler.register_file.allocatable |= ret_regs;

  compiler.gen_func_epilog();
  compiler.release_regs_after_return();
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
bool CompilerBase<Adaptor, Derived, Config>::compile() {
  // create function symbols
  text_writer.begin_module(assembler);
  text_writer.switch_section(
      assembler.get_section(assembler.get_default_section(SectionKind::Text)));

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
  text_writer.end_module();
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
    IRValueRef value, ValLocalIdx local_idx) {
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
      if (!reg.invalid() && !(used_phi_regs_global & (1ull << reg.id())) &&
          !register_file.is_used(reg)) {
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
    ValLocalIdx local_idx, ValueAssignment *assignment) {
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
      if (global_reg_for(local_idx).valid()) {
        global_unassign(local_idx);
      }
      const auto reg = ap.get_reg();
      assert(!register_file.is_fixed(reg));
      register_file.unmark_used(reg);
    }
  }

  if constexpr (WithAsserts) {
    for (auto reg_id : register_file.used_regs()) {
      assert(register_file.reg_local_idx(AsmReg{reg_id}) != local_idx &&
             "freeing assignment that is still referenced by a register");
    }
  }

  // variable references do not have a stack slot
  bool has_stack = Config::FRAME_INDEXING_NEGATIVE ? assignment->frame_off < 0
                                                   : assignment->frame_off != 0;
  if (!is_var_ref && has_stack) {
    free_stack_slot(assignment->frame_off, assignment->size());
  }

  assignments.allocator.deallocate(assignment);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
[[gnu::noinline]] void
    CompilerBase<Adaptor, Derived, Config>::release_assignment(
        ValLocalIdx local_idx, ValueAssignment *assignment) {
  if (!assignment->delay_free) {
    if (global_reg_for(local_idx).valid()) {
      global_unassign(local_idx);
    }
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
    ValLocalIdx local_idx, u32 var_ref_data) {
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
i32 CompilerBase<Adaptor, Derived, Config>::allocate_stack_slot(u32 size) {
  this->stack.frame_used = true;
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
void CompilerBase<Adaptor, Derived, Config>::free_stack_slot(u32 slot,
                                                             u32 size) {
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
void CompilerBase<Adaptor, Derived, Config>::prologue_assign_arg(
    CCAssigner *cc_assigner,
    u32 arg_idx,
    IRValueRef arg,
    u32 align,
    bool allow_split) {
  ValueRef vr = derived()->result_ref(arg);
  if (adaptor->cur_arg_is_byval(arg_idx)) {
    CCAssignment cca{
        .byval = true,
        .align = u8(adaptor->cur_arg_byval_align(arg_idx)),
        .size = adaptor->cur_arg_byval_size(arg_idx),
    };
    cc_assigner->assign_arg(cca);
    std::optional<i32> byval_frame_off =
        derived()->prologue_assign_arg_part(vr.part(0), cca);

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
    assert(vr.assignment()->part_count == 1 && "sret must be single-part");
    ValuePartRef vp = vr.part(0);
    CCAssignment cca{
        .sret = true, .bank = vp.bank(), .size = Config::PLATFORM_POINTER_SIZE};
    cc_assigner->assign_arg(cca);
    derived()->prologue_assign_arg_part(std::move(vp), cca);
    return;
  }

  const u32 part_count = vr.assignment()->part_count;
  for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
    ValuePartRef vp = vr.part(part_idx);
    u32 remaining = part_count < 256 ? part_count - part_idx - 1 : 255;
    CCAssignment cca{
        .consecutive = u8(allow_split ? 0 : remaining),
        .align = u8(part_idx == 0 ? align : 1),
        .bank = vp.bank(),
        .size = vp.part_size(),
    };
    cc_assigner->assign_arg(cca);
    derived()->prologue_assign_arg_part(std::move(vp), cca);
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::ValueRef
    CompilerBase<Adaptor, Derived, Config>::val_ref(IRValueRef value) {
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
    CompilerBase<Adaptor, Derived, Config>::val_ref_single(IRValueRef value) {
  std::pair<ValueRef, ValuePartRef> res{val_ref(value), this};
  res.second = res.first.part(0);
  return res;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::ValueRef
    CompilerBase<Adaptor, Derived, Config>::result_ref(IRValueRef value) {
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
        IRValueRef value) {
  std::pair<ValueRef, ValuePartRef> res{result_ref(value), this};
  res.second = res.first.part(0);
  return res;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::ValueRef
    CompilerBase<Adaptor, Derived, Config>::result_ref_alias(IRValueRef dst,
                                                             ValueRef &&src) {
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
  if constexpr (WithAsserts) {
    const auto &src_liveness = analyzer.liveness_info(src.local_idx());
    assert(!src_liveness.last_full);          // implied by is_owned()
    assert(assignment->references_left == 1); // implied by is_owned()

    // Validate that part configuration is identical.
    const auto parts = derived()->val_parts(dst);
    assert(parts.count() == part_count);
    for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
      AssignmentPartRef ap{assignment, part_idx};
      assert(parts.reg_bank(part_idx) == ap.bank());
      assert(parts.size_bytes(part_idx) == ap.part_size());
    }
  }

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
        IRValueRef dst, AssignmentPartRef base, i32 off) {
  const ValLocalIdx local_idx = analyzer.adaptor->val_local_idx(dst);
  assert(!val_assignment(local_idx) && "new value already defined");
  init_variable_ref(local_idx, 0);
  ValueAssignment *assignment = this->val_assignment(local_idx);
  assignment->stack_variable = true;
  assignment->frame_off = base.variable_stack_off() + off;
  return ValueRef{this, local_idx};
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::set_value(ValuePartRef &val_ref,
                                                       ScratchReg &scratch) {
  val_ref.set_value(std::move(scratch));
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::AsmReg
    CompilerBase<Adaptor, Derived, Config>::gval_as_reg(GenericValuePart &gv) {
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
        GenericValuePart &gv, ScratchReg &dst) {
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
typename CompilerBase<Adaptor, Derived, Config>::AsmReg
    CompilerBase<Adaptor, Derived, Config>::gval_as_reg_reuse(
        GenericValuePart &gv, ValuePart &dst) {
  AsmReg reg = gval_as_reg(gv);
  if (!dst.has_reg() &&
      (!dst.has_assignment() || !dst.assignment().fixed_assignment())) {
    // TODO: make this less expensive
    if (auto *scratch = std::get_if<ScratchReg>(&gv.state)) {
      dst.set_value(this, std::move(*scratch));
      if (dst.has_assignment()) {
        dst.lock(this);
      }
    } else if (auto *val_ref = std::get_if<ValuePartRef>(&gv.state)) {
      if (val_ref->can_salvage()) {
        dst.set_value(this, std::move(*val_ref));
        if (dst.has_assignment()) {
          dst.lock(this);
        }
      }
    }
  }
  return reg;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
Reg CompilerBase<Adaptor, Derived, Config>::select_reg_evict(RegBank bank) {
  TPDE_LOG_DBG("select_reg_evict for bank {}", bank.id());
  auto candidates = register_file.used & register_file.bank_regs(bank);

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
      score = -1;
    }

    const auto &liveness = analyzer.liveness_info(local_idx);
    u32 last_use_dist = u32(liveness.last) - u32(cur_block_idx);
    score |= (last_use_dist < 0x8000 ? 0x8000 - last_use_dist : 0) << 16;

    u32 refs_left = va->pending_free ? 0 : va->references_left;
    score |= (refs_left < 0xffff ? 0x10000 - refs_left : 1);

    TPDE_LOG_DBG("  r{} ({}:{}) rc={}/{} live={}-{}{} spilled={} score={:#x}",
                 reg_id,
                 static_cast<u32>(local_idx),
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
  if (global_reg_for(register_file.reg_local_idx(candidate)).valid()) {
    global_unassign(register_file.reg_local_idx(candidate));
  }
  evict_reg(candidate);
  return candidate;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::reload_to_reg(
    AsmReg dst, AssignmentPartRef ap) {
#ifndef NDEBUG
  // forcefully get stack allocation
  typename VIR<Adaptor>::Allocation from = get_allocation(ap, true);
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
  }

  if (ap.is_stack_variable() || ap.variable_ref()) {
    if (val_idx != INVALID_VAL_LOCAL_IDX) {
      util::SmallVector<typename VIR<Adaptor>::Operand, 1> defs;
      typename VIR<Adaptor>::Operand def;
      def.val_idx = val_idx;
      def.alloc = to;
      def.alloc.part_idx = part_idx;
      defs.push_back(def);
      verification_ir.emit_inst_op(val_idx, {}, std::move(defs));
    }
  } else if (val_idx != INVALID_VAL_LOCAL_IDX) {
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
    AssignmentPartRef ap) {
  assert(!ap.variable_ref() && "cannot allocate spill slot for variable ref");
  if (ap.assignment()->frame_off == 0) {
    assert(!ap.stack_valid() && "stack-valid set without spill slot");
    ap.assignment()->frame_off = allocate_stack_slot(ap.assignment()->size());
    assert(ap.assignment()->frame_off != 0);
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::spill(AssignmentPartRef ap) {
  assert(may_change_value_state());
  if (!ap.stack_valid() && !ap.variable_ref()) {
    assert(ap.register_valid() && "cannot spill uninitialized assignment part");
    allocate_spill_slot(ap);
    derived()->spill_reg(ap.get_reg(), ap.frame_off(), ap.part_size());
    ap.set_stack_valid();
#ifndef NDEBUG
    {
      typename VIR<Adaptor>::Allocation from = get_allocation(ap);
      ValLocalIdx val_idx = register_file.reg_local_idx(ap.get_reg());
      if (val_idx != INVALID_VAL_LOCAL_IDX) {
        typename VIR<Adaptor>::Allocation to(ap.frame_off());
        u32 part_idx = register_file.reg_part(ap.get_reg());
        verification_ir.emit_edit(VIR<Adaptor>::EditKind::Spill,
                                  from,
                                  to,
                                  val_idx,
                                  part_idx,
                                  ap.part_size());
      }
    }
#endif
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::evict(AssignmentPartRef ap) {
  assert(may_change_value_state());
  assert(ap.register_valid());
  derived()->spill(ap);
  ap.set_register_valid(false);
  register_file.unmark_used(ap.get_reg());
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::evict_reg(Reg reg) {
  assert(may_change_value_state());
  assert(!register_file.is_fixed(reg));
  assert(register_file.reg_local_idx(reg) != INVALID_VAL_LOCAL_IDX);

  ValLocalIdx local_idx = register_file.reg_local_idx(reg);
  auto part = register_file.reg_part(reg);
  AssignmentPartRef evict_part{val_assignment(local_idx), part};

  // Handle case where register is marked as used but assignment doesn't have
  // register_valid set (can happen for temporary phi registers during move
  // resolution)
  if (!evict_part.register_valid()) {
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
void CompilerBase<Adaptor, Derived, Config>::lazy_free_reg(Reg reg) {
  assert(may_change_value_state());
  assert(!register_file.is_fixed(reg));

  if (!register_file.is_used(reg)) {
    return;
  }

  ValLocalIdx local_idx = register_file.reg_local_idx(reg);
  if (local_idx == INVALID_VAL_LOCAL_IDX) {
    register_file.unmark_used(reg);
    return;
  }

  auto part = register_file.reg_part(reg);
  AssignmentPartRef ap{val_assignment(local_idx), part};
  if (!ap.register_valid()) {
    register_file.unmark_used(reg);
    return;
  }

  RegBank bank = register_file.reg_bank(reg);
  auto available = (register_file.allocatable & ~register_file.used) &
                   register_file.bank_regs(bank);
  bool success = repair_argument(local_idx,
                                 part,
                                 ap.part_size(),
                                 bank,
                                 (1ull << reg.id()),
                                 available,
                                 0,
                                 AsmReg::make_invalid());
  if (!success) {
    evict_reg(reg);
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::free_reg(Reg reg) {
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
    CompilerBase<Adaptor, Derived, Config>::spill_caller_saved_before_call(
        typename RegisterFile::RegBitSet call_arguments) {
  using RegBitSet = typename RegisterFile::RegBitSet;

  assert(may_change_value_state());

  const RegBitSet caller_saved = ~register_file.callee_saved;
  const RegBitSet spillable = register_file.used & caller_saved;
  RegBitSet spilled = {};

  for (auto reg_id : util::BitSetIterator<>{spillable}) {
    const Reg reg{reg_id};
    if (register_file.is_fixed(reg)) {
      continue;
    }
    if (register_file.reg_local_idx(reg) == INVALID_VAL_LOCAL_IDX) {
      register_file.unmark_used(reg);
      spilled |= (1ull << reg_id);
      continue;
    }

    AssignmentPartRef ap{val_assignment(register_file.reg_local_idx(reg)),
                         register_file.reg_part(reg)};
    if (!ap.register_valid()) {
      register_file.unmark_used(reg);
      spilled |= (1ull << reg_id);
      continue;
    }

    if ((call_arguments & (1ull << reg_id)) &&
        ap.assignment()->references_left == 1) {
      continue;
    }

    derived()->spill(ap);
    ap.set_register_valid(false);
    register_file.unmark_used(reg);
    spilled |= (1ull << reg_id);
  }

  return spilled;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::RegisterFile::RegBitSet
    CompilerBase<Adaptor, Derived, Config>::spill_before_branch(
        bool force_spill) {
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
  //
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
  //
  // ...
  //
  //   if (succ_count == 1 && !must_spill) {
  //     return RegBitSet{};
  //   }
  // }
  using RegBitSet = typename RegisterFile::RegBitSet;
  return RegBitSet{};
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
bool CompilerBase<Adaptor, Derived, Config>::repair_argument(
    ValLocalIdx var,
    u32 part,
    u8 size,
    RegBank bank,
    typename RegisterFile::RegBitSet constraints,
    typename RegisterFile::RegBitSet available,
    typename RegisterFile::RegBitSet forbidden,
    AsmReg current_reg) {
  auto &reg_file = register_file;
  Reg reg = Reg::make_invalid();
  std::unordered_set<ValLocalIdx> operands;
  // commented out current_instr usage
  // for (auto operand:
  // compiler->adaptor->inst_operands(*compiler->tree_ra_ctx->current_instr)) {
  //   operands.insert(compiler->adaptor->val_local_idx(operand));
  // }
  bool success = false;
  typename RegisterFile::RegBitSet allowed =
      constraints & (~forbidden); // todo(salto): constraints
  while (reg == Reg::make_invalid() && allowed != 0) {
    for (u64 candidate : util::BitSetIterator<>(allowed)) {
      if (reg_file.is_used(Reg{candidate}) &&
          (!operands.contains(reg_file.reg_local_idx(Reg{candidate})) &&
           !reg_file.is_fixed(Reg{candidate}))) {
        reg = Reg{candidate};
        break;
      }
    }
    if (reg == Reg::make_invalid()) {
      // todo(salto): choose color from allowed
      reg = Reg{*util::BitSetIterator<>(allowed).begin()};
    }
    ValLocalIdx pawn = reg_file.reg_local_idx(reg);
    // todo(salto): constraints of pawn?
    u64 current_reg_bit =
        current_reg.valid() ? (1ull << current_reg.id()) : 0ull;
    typename RegisterFile::RegBitSet pawn_allowed =
        (available | current_reg_bit) & (~forbidden);
    auto pawn_part = reg_file.reg_part(reg);
    if (pawn_allowed != 0) {
      Reg pawn_reg = reg_file.find_first_free_excluding(bank, ~pawn_allowed);
      parallel_copies.emplace_back(pawn_reg, reg, 8, pawn, pawn_part);
      success = true;
    } else {
      success = repair_argument(pawn,
                                pawn_part,
                                size,
                                bank,
                                available | current_reg_bit,
                                forbidden | (1ull << reg.id()),
                                0,
                                current_reg);
    }
    if (!success) {
      allowed &= ~(1ull << reg.id());
      reg = Reg::make_invalid();
    }
  }
  if (reg != Reg::make_invalid()) {
    if (current_reg.valid()) {
      parallel_copies.emplace_back(Reg{reg}, current_reg, size, var, part);
    }
    return true;
  }
  return false;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::release_spilled_regs(
    typename RegisterFile::RegBitSet regs) {
  assert(may_change_value_state());

  // TODO(ts): needs changes for other RegisterFile impls
  for (auto reg_id : util::BitSetIterator<>{regs & register_file.used}) {
    if (!register_file.is_fixed(Reg{reg_id})) {
      free_reg(Reg{reg_id});
    }
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::release_regs_after_return() {
  // we essentially have to free all non-fixed registers
  for (auto reg_id : register_file.used_regs()) {
    if (!register_file.is_fixed(Reg{reg_id}) &&
        (register_file.reg_local_idx(Reg{reg_id}) == INVALID_VAL_LOCAL_IDX ||
         val_assignment(register_file.reg_local_idx(Reg{reg_id}))
                 ->references_left == 0)) {
      free_reg(Reg{reg_id});
    }
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
template <typename Jump>
void CompilerBase<Adaptor, Derived, Config>::generate_branch_to_block(
    Jump jmp, IRBlockRef target, bool needs_split, bool last_inst) {
  BlockIndex target_idx = this->analyzer.block_idx(target);
  Label target_label = this->block_labels[u32(target_idx)];
  if (!needs_split) {
    move_values_to_match(target_idx);
    if (!last_inst || target_idx != this->next_block()) {
      derived()->generate_raw_jump(jmp, target_label);
    }
  } else {
    Label tmp_label = this->text_writer.label_create();
    derived()->generate_raw_jump(derived()->invert_jump(jmp), tmp_label);
    move_values_to_match(target_idx);
    derived()->generate_raw_jump(Jump::jmp, target_label);
    this->label_place(tmp_label);
  }
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::generate_uncond_branch(
    IRBlockRef target) {
  auto spilled = spill_before_branch();
  begin_branch_region();

  generate_branch_to_block(Derived::Jump::jmp, target, false, true);

  end_branch_region();
  release_spilled_regs(spilled);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
template <typename Jump>
void CompilerBase<Adaptor, Derived, Config>::generate_cond_branch(
    Jump jmp, IRBlockRef true_target, IRBlockRef false_target) {
  IRBlockRef next = analyzer.block_ref(next_block());

  bool true_needs_split = branch_needs_split(true_target);
  bool false_needs_split = branch_needs_split(false_target);

  auto spilled = spill_before_branch();
  begin_branch_region();

  if (next == true_target || (next != false_target && true_needs_split)) {
    generate_branch_to_block(
        derived()->invert_jump(jmp), false_target, false_needs_split, false);
    generate_branch_to_block(Derived::Jump::jmp, true_target, false, true);
  } else if (next == false_target) {
    generate_branch_to_block(jmp, true_target, true_needs_split, false);
    generate_branch_to_block(Derived::Jump::jmp, false_target, false, true);
  } else {
    assert(!true_needs_split);
    generate_branch_to_block(jmp, true_target, false, false);
    generate_branch_to_block(Derived::Jump::jmp, false_target, false, true);
  }

  end_branch_region();
  release_spilled_regs(spilled);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
void CompilerBase<Adaptor, Derived, Config>::generate_switch(
    ScratchReg &&cond,
    u32 width,
    IRBlockRef default_block,
    std::span<const std::pair<u64, IRBlockRef>> cases) {
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
  // make sure we don't get a phi register for the consts
  AsmReg tmp_reg = this->select_reg(Config::GP_BANK, used_phi_regs_global);
  ScratchReg tmp_scratch{this};
  tmp_scratch.alloc_specific(tmp_reg);

  const auto spilled = this->spill_before_branch();
  this->begin_branch_region();

  tpde::util::SmallVector<tpde::Label, 64> case_labels;
  // Labels that need an intermediate block to setup registers. This is
  // separate, because most switch targets don't need this.
  tpde::util::SmallVector<std::pair<tpde::Label, IRBlockRef>, 64> case_blocks;
  for (auto i = 0u; i < cases.size(); ++i) {
    // If the target might need additional register moves, we can't branch there
    // immediately.
    // TODO: more precise condition?
    BlockIndex target = this->analyzer.block_idx(cases[i].second);
    if (analyzer.block_has_phis(target)) {
      case_labels.push_back(this->text_writer.label_create());
      case_blocks.emplace_back(case_labels.back(), cases[i].second);
    } else {
      case_labels.push_back(this->block_labels[u32(target)]);
    }
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
    u64 low_bound = cases[begin].first;
    u64 high_bound = cases[end - 1].first;
    auto range = high_bound - low_bound + 1;
    // we will get wrong results if range is 0 so skip the jump table if
    // that is the case
    if (range != 0 && (range / num_cases) < 8) {
      // for gcc, it seems that if there are less than 8 values per
      // case it will build a jump table so we do that, too

      // Give target the option to emit a jump table.
      auto *jt = derived()->switch_create_jump_table(
          default_label, cmp_reg, tmp_reg, low_bound, high_bound, width_is_32);
      if (jt) {
        if (range == num_cases) {
          std::copy(case_labels.begin() + begin,
                    case_labels.begin() + end,
                    jt->labels().begin());
        } else {
          std::ranges::fill(jt->labels(), default_label);
          for (auto i = begin; i != end; ++i) {
            jt->labels()[cases[i].first - low_bound] = case_labels[i];
          }
        }
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

  for (const auto &[label, target] : case_blocks) {
    // Branch predictors typically have problems if too many branches follow too
    // closely. Ensure a minimum alignment.
    this->text_writer.align(8);
    this->label_place(label);
    derived()->generate_branch_to_block(
        Derived::Jump::jmp, target, false, false);
  }

  this->end_branch_region();
  this->release_spilled_regs(spilled);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::RegisterFile::RegBitSet
    CompilerBase<Adaptor, Derived, Config>::move_to_phi_nodes_impl(
        BlockIndex target, MoveList &moves) {
  // PHI-nodes are always moved to their stack-slot (unless they are fixed)
  //
  // However, we need to take care of PHI-dependencies (cycles and chains)
  // as to not overwrite values which might be needed.
  //
  // In most cases, we expect the number of PHIs to be small but we want to
  // stay reasonably efficient even with larger numbers of PHIs
  // todo(salto): more phis than registers
  IRBlockRef target_ref = analyzer.block_ref(target);
  IRBlockRef cur_ref = analyzer.block_ref(cur_block_idx);

  auto &target_phi_regs = phi_regs[target];

  if constexpr (WithAsserts) {
    if (!target_phi_regs.empty()) {
    }
    for (auto reg : register_file.used_regs()) {
      if (register_file.is_fixed(Reg{reg}) &&
          register_file.reg_local_idx(Reg{reg}) == INVALID_VAL_LOCAL_IDX &&
          used_phi_regs_global & (1ull << reg)) {
        for (auto &entry : target_phi_regs) {
          auto val_idx = entry.first;
          if (!val_assignment(val_idx)) {
            continue;
          }
          ValueAssignment *va = val_assignment(val_idx);
          for (u32 part_idx = 0; part_idx < va->part_count; ++part_idx) {
            assert(entry.second[part_idx] != Reg{reg} &&
                   "one of the phi registers is held as a Scratch across a "
                   "branch. PHI resolution impossible.");
          }
        }
      }
    }
  }

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

    bool operator<(const NodeEntry &other) const {
      return phi_local_idx < other.phi_local_idx;
    }

    bool operator<(ValLocalIdx other) const { return phi_local_idx < other; }
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

  typename RegisterFile::RegBitSet used_phi_regs = 0;

  const auto execute_parallel_copies = [&]() {
    if (parallel_copies.empty()) {
      return;
    }
    auto ordered = sequentialize(parallel_copies);
    auto &reg_file = register_file;
    for (auto move : ordered) {
      if (move.value_idx != INVALID_VAL_LOCAL_IDX) {
        ValueAssignment *assignment = this->val_assignment(move.value_idx);
        if (!assignment) {
          continue;
        }
        AssignmentPartRef ap{assignment, move.part_idx};

        if (ap.fixed_assignment()) {
          this->derived()->mov(move.dst, move.src, ap.part_size());
          continue;
        }
        this->global_assign(move.value_idx, move.dst);
        if (register_file.is_used(Reg{move.dst}) &&
            move.value_idx != register_file.reg_local_idx(Reg{move.dst})) {
          this->evict_reg(Reg{move.dst});
        }
        this->derived()->mov(move.dst, move.src, ap.part_size());
        if (ap.register_valid()) {
          register_file.unmark_used(ap.get_reg());
        }
        ap.set_register_valid(true);
        ap.set_reg(Reg{move.dst});
        this->register_file.mark_used(
            Reg{move.dst}, move.value_idx, move.part_idx);
        this->register_file.mark_clobbered(Reg{move.dst});
        #ifndef NDEBUG
        verification_ir.emit_active_reg_move(move.src, move.dst, move.size);
        #endif
      } else {
        this->derived()->mov(move.dst, move.src, 8);
        this->register_file.mark_used(
            Reg{move.dst}, move.value_idx, move.part_idx);
        this->register_file.mark_clobbered(Reg{move.dst});
      }
    }
    parallel_copies.clear();
  };

  const auto free_target_reg = [&](Reg target_reg, u8 size) {
    if (!register_file.is_used(target_reg)) {
      return;
    }
    if (register_file.is_fixed(target_reg)) {
      TPDE_FATAL("attempted to free fixed phi target register");
    }
    if (register_file.reg_local_idx(target_reg) == INVALID_VAL_LOCAL_IDX) {
      register_file.unmark_used(target_reg);
      return;
    }

    RegBank bank = register_file.reg_bank(target_reg);
    auto available = (register_file.allocatable & ~register_file.used) &
                     register_file.bank_regs(bank);
    bool success = repair_argument(INVALID_VAL_LOCAL_IDX,
                                   0,
                                   size,
                                   bank,
                                   (1ull << target_reg.id()),
                                   available,
                                   0,
                                   AsmReg::make_invalid());
    if (success) {
      execute_parallel_copies();
    } else {
      evict_reg(target_reg);
    }
  };

  const auto move_phi_to_target = [&](IRValueRef phi,
                                      IRValueRef incoming_val,
                                      const PhiRegList &target_regs) {
    ValLocalIdx phi_local_idx = adaptor->val_local_idx(phi);
    // only needed for ref counts.
    ValueRef phi_ref = result_ref(phi);
    ValueAssignment *phi_assignment = val_assignment(phi_local_idx);
    assert(phi_assignment && "phi node has no assignment");

    ValueRef incoming_ref = val_ref(incoming_val);
    if (!adaptor->val_ignore_in_liveness_analysis(incoming_val)) {
      const auto incoming_local_idx = adaptor->val_local_idx(incoming_val);
      if (incoming_local_idx == phi_local_idx) {
        return;
      }
    }
    const bool incoming_last_ref = incoming_ref.last_ref();

    for (u32 part = 0; part < phi_assignment->part_count; ++part) {
      AssignmentPartRef phi_ap{phi_assignment, part};
      auto incoming_part = incoming_ref.part(part);
      Reg incoming_reg = incoming_part.cur_reg_unlocked();
      Reg target_reg = Reg::make_invalid();
      if (target_regs.size() > part) {
        target_reg = target_regs[part];
      }


      if (phi_ap.fixed_assignment()) {
        if (incoming_reg.valid()) {
          // we can't move immediately here since incoming could also be a phi that might need our value.
          moves.emplace_back(
              target_reg, incoming_reg, phi_ap.part_size(), phi_local_idx, part);
        } else {
          incoming_part.reload_into_specific_fixed(phi_ap.get_reg());
        }
        continue;
      }
      if (!target_reg.valid()) {
        if (!phi_ap.stack_valid()) {
          allocate_spill_slot(phi_ap); // todo(salto): should be unreachable
        }
        if (incoming_reg.valid()) {
          derived()->spill_reg(
              incoming_reg, phi_ap.frame_off(), phi_ap.part_size());
        } else {
          ScratchReg scratch = std::move(incoming_part).into_scratch();
          derived()->spill_reg(
              scratch.cur_reg(), phi_ap.frame_off(), phi_ap.part_size());
        }
        phi_ap.set_stack_valid();
        continue;
      }


      if (incoming_reg.valid() && !incoming_last_ref) {
        moves.emplace_back(
            target_reg, incoming_reg, phi_ap.part_size(), phi_local_idx, part);
      } else if (incoming_reg.valid() && incoming_last_ref) {
        register_file.allocatable &=
            ~(1ull << incoming_reg.id()); // todo(salto): undo this
        moves.emplace_back(
            target_reg, incoming_reg, phi_ap.part_size(), phi_local_idx, part);
      } else {
        if (register_file.is_used(target_reg)) {
          evict_reg(target_reg);
        }
        incoming_part.reload_into_specific_fixed(target_reg);
      }
      used_phi_regs |= (1ull << target_reg.id());
      used_phi_regs_global |= (1ull << target_reg.id());
    }
  };

  const auto allocate_phi = [&](IRValueRef phi,
                                IRValueRef incoming_val,
                                bool allocate_to_stack) {
    ValueRef phi_ref = result_ref(phi);
    ValueRef incoming_ref = val_ref(incoming_val);
    ValLocalIdx phi_local_idx = adaptor->val_local_idx(phi);
    ValueAssignment *phi_assignment = val_assignment(phi_local_idx);
    assert(phi_assignment && "phi node has no assignment");


    PhiRegList target_regs;
    target_regs.resize(phi_assignment->part_count, Reg::make_invalid());
    typename RegisterFile::RegBitSet new_phi_regs = 0;

    bool allow_regs = !allocate_to_stack;
    for (u32 part = 0; part < phi_assignment->part_count; ++part) {
      AssignmentPartRef phi_ap{phi_assignment, part};
      ValuePartRef incoming_part = incoming_ref.part(part);
      Reg incoming_reg = incoming_part.cur_reg_unlocked();
      RegBank bank = phi_ap.bank();
      auto exclusion = used_phi_regs | new_phi_regs;
      Reg selected;
      if (phi_ap.fixed_assignment()) {
        TPDE_LOG_TRACE("Phi part {} has fixed assignment to reg {}",
                       part,
                       static_cast<u32>(phi_ap.get_reg().id()));
        selected = phi_ap.get_reg();
        target_regs[part] = phi_ap.get_reg();
        new_phi_regs |= (1ull << target_regs[part].id());
        if (incoming_reg.valid()) {
          // we can't move immediately here since incoming could also be a phi that might need our value.
          moves.emplace_back(
              selected, incoming_reg, phi_ap.part_size(), phi_local_idx, part);
        } else {
          incoming_part.reload_into_specific_fixed(selected);
        }
        continue;
      }
      if (!allow_regs) {
        // not fixed and should be on the stack.
        continue;
      }

      // if our incoming is dead after the phi we can reuse the register
      if (incoming_ref.last_ref() && incoming_reg.valid()  && !register_file.is_fixed(incoming_reg)) {
        selected = incoming_reg;
      } else {
        selected = register_file.find_first_free_excluding(bank, exclusion);
      }

      // if (selected.invalid() && incoming_ref.last_ref()) {
      //   if (incoming_reg.valid() &&
      //       register_file.reg_bank(incoming_reg) == bank &&
      //       ((exclusion & (1ull << incoming_reg.id())) == 0)) {
      //     selected = incoming_reg;
      //   }
      // }

      if (selected.invalid()) {
        allow_regs = false;
        continue;
      }

      if (incoming_reg.valid() && !incoming_ref.last_ref()) {
        moves.emplace_back(
            selected, incoming_reg, phi_ap.part_size(), phi_local_idx, part);
      } else if (incoming_reg.valid() && incoming_ref.last_ref()) {
        register_file.allocatable &=
            ~(1ull << incoming_reg.id()); // todo(salto): undo this
        moves.emplace_back(
            selected, incoming_reg, phi_ap.part_size(), phi_local_idx, part);
      } else {
        incoming_part.reload_into_specific_fixed(selected);
      }

      target_regs[part] = selected;
      new_phi_regs |= (1ull << selected.id());
    }

    if (!allow_regs) {
      new_phi_regs = 0;
      for (u32 part = 0; part < phi_assignment->part_count; ++part) {
        AssignmentPartRef phi_ap{phi_assignment, part};
        if (phi_ap.fixed_assignment()) {
          target_regs[part] = phi_ap.get_reg();
          new_phi_regs |= (1ull << target_regs[part].id());
        } else {
          target_regs[part] = Reg::make_invalid();
          // Allocate spill slot and move the incoming value into it.
          if (!phi_ap.stack_valid()) {
            allocate_spill_slot(phi_ap);
          }
          auto incoming_part = incoming_ref.part(part);
          Reg incoming_reg = incoming_part.cur_reg_unlocked();
          if (incoming_reg.valid()) {
            derived()->spill_reg(
                incoming_reg, phi_ap.frame_off(), phi_ap.part_size());
          } else {
            ScratchReg scratch = std::move(incoming_part).into_scratch();
            derived()->spill_reg(
                scratch.cur_reg(), phi_ap.frame_off(), phi_ap.part_size());
          }
          phi_ap.set_register_valid(false);
          phi_ap.set_stack_valid();
        }
      }
    }

    used_phi_regs |= new_phi_regs;
    used_phi_regs_global |= new_phi_regs;
    target_phi_regs.insert_or_assign(phi_local_idx, std::move(target_regs));
  };

  // We check that the block has phi nodes before getting here.
  assert(!nodes.empty() && "block marked has having phi nodes has none");

  // Determine allocation strategy: hybrid register/stack if too many phi nodes
  const u32 phi_count = static_cast<u32>(nodes.size());
  // todo(salto): do this for each bank seperately, so we guarantee at least 1
  // free register
  const u32 available_regs = count_total_available_registers();

  // Reserve some registers for scratch operations during phi resolution
  constexpr u32 reserved_for_scratch = 2;
  const u32 regs_for_phis =
      std::max(0u,
               std::min(available_regs - phi_count - reserved_for_scratch,
                        PHI_REGISTER_THRESHOLD));


  bool use_hybrid_allocation = phi_count > regs_for_phis;

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
  if (target_phi_regs.empty()) {
    // First, allocate registers for phi nodes that can be allocated to
    // registers
    for (u32 i = 0; i < nodes.size(); ++i) {
      NodeEntry &node = nodes[i];

      allocate_phi(node.phi, node.incoming_val, node.allocate_to_stack);
    }
  } else {
    // Already have an allocation from a previous block. Move phis to correct
    // registers and stack slots.
    for (u32 i = 0; i < nodes.size(); ++i) {
      NodeEntry &node = nodes[i];
      auto &target_regs = target_phi_regs.at(node.phi_local_idx);
      move_phi_to_target(node.phi, node.incoming_val, target_regs);
    }
  }

#ifndef NDEBUG
  // todo(salto): decide if we need the parrarel moves
  //  Capture parallel moves for verification IR
  util::SmallVector<
      std::pair<typename VIR<Adaptor>::Operand, typename VIR<Adaptor>::Operand>,
      4>
      parallel_moves;
  u32 temp_move_counter = 0;
  auto find_value_for_reg_parallel =
      [&](Reg reg) -> std::pair<ValLocalIdx, u32> {
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
      if (!va) {
        continue;
      }
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
      auto [found_src_idx, found_src_part] =
          find_value_for_reg_parallel(move.src);
      src_val_idx = found_src_idx;
      src_part = found_src_part;
    }
    auto [dst_val_idx, dst_part] = find_value_for_reg_parallel(move.dst);

    if (src_val_idx != INVALID_VAL_LOCAL_IDX &&
        dst_val_idx != INVALID_VAL_LOCAL_IDX) {
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
      ValLocalIdx temp_val_idx =
          static_cast<ValLocalIdx>(static_cast<u32>(cur_block_idx) |
                                   0x70000000u | (temp_move_counter << 16));
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
    verification_ir.emit_edge_parallel_move(
        cur_block_idx, target, std::move(parallel_moves));
  }
#endif
  for (auto move : moves) {
    if (register_file.is_used(move.dst) &&
        register_file.reg_local_idx(move.dst) == INVALID_VAL_LOCAL_IDX) {
      register_file.unmark_used(move.dst);
    }
  }
  return used_phi_regs;
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
typename CompilerBase<Adaptor, Derived, Config>::BlockIndex
    CompilerBase<Adaptor, Derived, Config>::next_block() const {
  return static_cast<BlockIndex>(static_cast<u32>(cur_block_idx) + 1);
}

template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
SymRef CompilerBase<Adaptor, Derived, Config>::get_personality_sym() {
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

        auto rodata =
            this->assembler.get_default_section(SectionKind::DataRelRO);
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
bool CompilerBase<Adaptor, Derived, Config>::compile_func(const IRFuncRef func,
                                                          const u32 func_idx) {
  if (!adaptor->switch_func(func)) {
    return false;
  }
  derived()->analysis_start();
  analyzer.switch_func(func);
  derived()->analysis_end();

  if constexpr (WithAsserts) {
    stack.frame_size = ~0u;
  }
  for (auto &e : stack.fixed_free_lists) {
    e.clear();
  }
  stack.dynamic_free_lists.clear();
  // TODO: sort out the inconsistency about adaptor vs. compiler methods.
  stack.has_dynamic_alloca = this->adaptor->cur_has_dynamic_alloca();
  stack.is_leaf_function = !derived()->cur_func_may_emit_calls();
  stack.generated_call = false;
  stack.frame_used = false;

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
  used_phi_regs_global = 0;
  register_file.reset();
  global_register_file.reset();
  // if (tree_ra_ctx) {
  //   tree_ra_ctx->global_regs.clear();
  //   tree_ra_ctx->used_global_regs = 0;
  // }
  parallel_copies.clear();
#ifndef NDEBUG
  generating_branch = false;
  verification_ir = VIR<Adaptor>();
  verification_ir.set_func_name(adaptor->func_link_name(func));
  final_assignments.clear();
#endif

  // Simple heuristic for initial allocation size
  u32 expected_code_size = 0x8 * analyzer.num_insts + 0x40;
  this->text_writer.begin_func(/*alignment=*/16, expected_code_size);

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
  global_register_file.allocatable = cc_assigner->get_ccinfo().allocatable_regs;
  register_file.callee_saved = cc_assigner->get_ccinfo().callee_saved_regs;
  global_register_file.callee_saved =
      cc_assigner->get_ccinfo().callee_saved_regs;

  cc_assigner->reset();
  // Temporarily prevent argument registers from being assigned.
  const CCInfo &cc_info = cc_assigner->get_ccinfo();
  assert((cc_info.allocatable_regs & cc_info.arg_regs) == cc_info.arg_regs &&
         "argument registers must also be allocatable");
  this->register_file.allocatable &= ~cc_info.arg_regs;


  // Begin prologue, prepare for handling arguments.
  derived()->prologue_begin(cc_assigner);
  u32 arg_idx = 0;
  for (const IRValueRef arg : this->adaptor->cur_args()) {
    // Init assignment for all arguments. This can be substituted for more
    // complex mappings of arguments to value parts.
    derived()->prologue_assign_arg(cc_assigner, arg_idx++, arg);
  }
#ifndef NDEBUG
  // After gen_func_prolog_and_args, explicitly capture all function arguments
  // to ensure they appear first in the verification IR according to calling
  // convention Iterate through arguments and capture their register assignments
  for (const IRValueRef arg : adaptor->cur_args()) {
    ValLocalIdx arg_idx = adaptor->val_local_idx(arg);
    if (arg_idx == INVALID_VAL_LOCAL_IDX) {
      continue;
    }

    ValueAssignment *assignment = val_assignment(arg_idx);
    if (!assignment) {
      continue;
    }

    const auto parts = adaptor->val_parts(arg);
    const u32 part_count = parts.count();
    for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
      AssignmentPartRef ap{assignment, part_idx};
      if (ap.register_valid()) {
        Reg reg = ap.get_reg();
        BlockIndex entry_block_idx = static_cast<BlockIndex>(
            analyzer.block_idx(adaptor->cur_entry_block()));
        typename VIR<Adaptor>::Allocation alloc(reg);
        verification_ir.emit_arg(entry_block_idx, arg_idx, part_idx, alloc);
      } else if (ap.stack_valid()) {
        // Argument on stack - capture with stack allocation
        i32 stack_off = ap.frame_off();
        BlockIndex entry_block_idx = static_cast<BlockIndex>(
            analyzer.block_idx(adaptor->cur_entry_block()));
        typename VIR<Adaptor>::Allocation alloc(stack_off);
        verification_ir.emit_arg(entry_block_idx, arg_idx, part_idx, alloc);
      }
    }
  }
#endif
  // Finish prologue, storing relevant data from the argument cc_assigner.
  derived()->prologue_end(cc_assigner);

  this->register_file.allocatable |= cc_info.arg_regs;

  // Small allocas get stack slot, larger allocas need dynamic allocations.
  util::SmallVector<std::tuple<IRValueRef, u32, u32>> dyn_allocas;
  for (const IRValueRef alloca : adaptor->cur_static_allocas()) {
    auto size = adaptor->val_alloca_size(alloca);
    auto align = adaptor->val_alloca_align(alloca);
    if (align > 16 || size > Derived::MaxStaticAllocaSize) {
      stack.has_dynamic_alloca = true;
      dyn_allocas.emplace_back(alloca, size, align);
      continue;
    }

    ValLocalIdx local_idx = adaptor->val_local_idx(alloca);
    init_variable_ref(local_idx, 0);
    ValueAssignment *assignment = val_assignment(local_idx);
    assignment->stack_variable = true;
    assignment->frame_off = allocate_stack_slot(util::align_up(size, align));
  }

  if constexpr (!Config::DEFAULT_VAR_REF_HANDLING) {
    derived()->setup_var_ref_assignments();
  }

  for (auto &[alloca, size, align] : dyn_allocas) {
    auto [_, vr] = this->result_ref_single(alloca);
    derived()->alloca_fixed(size, align, vr);
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
  // Write verification IR to file
  std::string vir_filename = "/tmp/" + verification_ir.get_func_name() + ".vir";
  verification_ir.write_to_file(vir_filename);
#endif

  return true;
}


template <IRAdaptor Adaptor, typename Derived, CompilerConfig Config>
bool CompilerBase<Adaptor, Derived, Config>::compile_block(
    const IRBlockRef block, const u32 block_idx) {
  cur_block_idx = static_cast<BlockIndex>(block_idx);

  label_place(block_labels[block_idx]);
#ifndef NDEBUG
  verification_ir.set_current_block(cur_block_idx);
#endif

  auto state_it = block_regs.find(cur_block_idx);
  if (state_it != block_regs.end() || analyzer.block_has_phis(cur_block_idx)) {
    auto phi_block_it = phi_regs.find(cur_block_idx);
    const PhiRegMap *phi_reg_map =
        phi_block_it != phi_regs.end() ? &phi_block_it->second : nullptr;
    for (IRValueRef phi : adaptor->block_phis(block)) {
      auto phi_idx = adaptor->val_local_idx(phi);
      ValueAssignment *assignment =
          this->val_assignment(phi_idx); // todo val_assignment null
      // phi is unused and already freed
      if (!assignment) {
        continue;
      }
      for (u32 i = 0; i < assignment->part_count; i++) {
        auto ap = AssignmentPartRef{assignment, i};
        Reg reg = Reg::make_invalid();
        if (ap.fixed_assignment()) {
          reg = ap.get_reg();
        } else if (phi_reg_map) {
          auto phi_reg_it = phi_reg_map->find(phi_idx);
          if (phi_reg_it != phi_reg_map->end() &&
              phi_reg_it->second.size() > i) {
            reg = phi_reg_it->second[i];
          }
        }
        if (reg.valid()) {
          ap.set_reg(reg);
          // we cas savely unmark it used
          // used_phi_regs_global &= ~(1ull << reg.id());
          ap.set_register_valid(true);
          if (register_file.is_used(reg)) {
            register_file.update_reg_assignment(reg, phi_idx, i);
          } else {
            register_file.mark_used(reg, phi_idx, i);
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
      for (ValueState &state : state_it->second) {
        if (state.val_local_idx == INVALID_VAL_LOCAL_IDX) {
          for (u32 i = 0; i < state.registers.size(); i++) {
            auto reg = state.registers[i];
            if (reg.valid() && register_file.is_used(reg)) {
              this->evict_reg(reg);
            }
          }
          continue;
        }
        ValueAssignment *assignment = this->val_assignment(state.val_local_idx);
        if (!assignment) {
          continue;
        }
        for (u32 i = 0; i < assignment->part_count; i++) {
          auto ap = AssignmentPartRef{assignment, i};
          auto reg = state.registers[i];
          if (!reg.valid()) {
            continue;
          }
          if (ap.register_valid() && ap.get_reg() != reg) {
            register_file.unmark_used(ap.get_reg());
          }
          ap.set_reg(reg);
          ap.set_register_valid(true);
          if (register_file.is_used(reg) &&
              (register_file.reg_local_idx(reg) != state.val_local_idx ||
               register_file.reg_part(reg) != i)) {
            if (register_file.reg_local_idx(reg) != INVALID_VAL_LOCAL_IDX) {
              ValueAssignment *other =
                  this->val_assignment(register_file.reg_local_idx(reg));
              if (other) {
                auto other_ap =
                    AssignmentPartRef{other, register_file.reg_part(reg)};
                if (other_ap.register_valid()) {
                  other_ap.set_register_valid(false);
                }
              }
            }
            register_file.update_reg_assignment(reg, state.val_local_idx, i);
          } else if (!register_file.is_used(reg)) {
            register_file.mark_used(reg, state.val_local_idx, i);
          }
        }
      }
    }
  }
  cur_instr_idx = 0;
  auto &&val_range = adaptor->block_insts(block);
  auto end = val_range.end();
  for (auto it = val_range.begin(); it != end; ++it, cur_instr_idx++) {
    const IRInstRef inst = *it;
    if (this->adaptor->inst_fused(inst)) {
      continue;
    }

#ifndef NDEBUG
    // Capture instruction uses (operands) before compilation, as they may be
    // free later For branches, these are the condition values
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
      if (!op_assignment) {
        continue; // Not yet assigned
      }


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

    // Store condition uses BEFORE compile_inst (which calls
    // generate_branch_to_block) Make a copy since we'll need uses for
    // non-branch instructions too
    util::SmallVector<typename VIR<Adaptor>::Operand, 4> branch_condition;
    for (const auto &op : uses) {
      branch_condition.push_back(op);
    }
    verification_ir.set_branch_condition(std::move(branch_condition));
    // don't capture moves during codegen. They are not necessary for VIR.
    verification_ir.active_compilation = true;
#endif
    // if (!tree_ra_ctx) {
    //   tree_ra_ctx = new TreeRAContext(&inst);
    // } else {
    //   tree_ra_ctx->current_instr = &inst;
    // }
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
      if (res_idx == INVALID_VAL_LOCAL_IDX) {
        continue;
      }
      u32 idx = static_cast<u32>(res_idx);
      if (idx < analyzer.spilled_values.bit_size &&
          analyzer.spilled_values.is_set(idx)) {
        ValueAssignment *assignment = val_assignment(res_idx);
        if (!assignment) {
          continue;
        }
        const auto parts = adaptor->val_parts(result);
        const u32 part_count = parts.count();
        for (u32 part_idx = 0; part_idx < part_count; ++part_idx) {
          AssignmentPartRef ap{assignment, part_idx};
          if (ap.register_valid()) {
            TPDE_LOG_INFO("Spilling result {} from register {}",
                          static_cast<u32>(res_idx),
                          ap.get_reg().id());
            spill(ap);
          }
        }
      }
    }

#ifndef NDEBUG
    {
      verification_ir.active_compilation = false;
      // Update uses with final allocations after compilation
      // (allocations may have changed during compilation, e.g., values moved to
      // registers)
      for (auto &use : uses) {
        // Check if we recorded a final assignment (from arg moves) - takes
        // precedence
        auto fa_it = final_assignments.find(use.val_idx);
        if (fa_it != final_assignments.end() &&
            fa_it->second.size() > use.alloc.part_idx &&
            fa_it->second[use.alloc.part_idx].valid()) {
          use.alloc = typename VIR<Adaptor>::Allocation(
              fa_it->second[use.alloc.part_idx], use.alloc.part_idx);
        } else if (ValueAssignment *use_assignment =
                       val_assignment(use.val_idx)) {
          AssignmentPartRef ap{use_assignment, use.alloc.part_idx};
          use.alloc = get_allocation(ap);
        }
        // If neither, keep the original captured allocation
      }
      // constants
      for (Reg reg : final_assignments[INVALID_VAL_LOCAL_IDX]) {
        typename VIR<Adaptor>::Operand op;
        op.val_idx = INVALID_VAL_LOCAL_IDX;
        op.alloc = typename VIR<Adaptor>::Allocation(reg, 0);
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
          continue; // Should exist after compile_inst, unless the result was
                    // already freed
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

  if constexpr (WithAsserts) {
    // Some consistency checks. Register assignment information must match, all
    // used registers must have an assignment (no temporaries across blocks),
    // and fixed registers must be fixed assignments.
    for (auto reg_id : register_file.used_regs()) {
      Reg reg{reg_id};
      assert(register_file.reg_local_idx(reg) != INVALID_VAL_LOCAL_IDX);
      AssignmentPartRef ap{val_assignment(register_file.reg_local_idx(reg)),
                           register_file.reg_part(reg)};
      assert(ap.register_valid());
      assert(ap.get_reg() == reg);
      assert(!register_file.is_fixed(reg) || ap.fixed_assignment());
    }
  }

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
