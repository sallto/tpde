// SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
//
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#pragma once

#include "tpde/base.hpp"
#include "tpde/ValLocalIdx.hpp"
#include "tpde/util/SmallVector.hpp"
#include <string>
#include <functional>
#include <fstream>
#include <algorithm>
#include <vector>
#include <map>
#include <tuple>

namespace tpde {

/// Verification IR for checking SSA destruction and register allocation
template <typename Adaptor, typename BlockIndexType, typename IRInstRefType>
struct VerificationIR {

  class Formattable {
  public:
    virtual ~Formattable() = default;
    virtual std::string format() const = 0;
  };
  using BlockIndex = BlockIndexType;
  using IRInstRef = IRInstRefType;
  enum class EditKind : u8 {
    Move,         // Move between registers
    Spill,        // Spill register to stack
    Reload,       // Reload from stack to register
    ParallelMove, // Parallel move on edge (phi resolution)
    RegMove       // Register-to-register move (for phi resolution and general moves)
  };

  static std::string format_edit_kind(EditKind kind) {
    switch (kind) {
    case EditKind::Move: return "move";
    case EditKind::Spill: return "spill";
    case EditKind::Reload: return "reload";
    case EditKind::RegMove: return "regmove";
    case EditKind::ParallelMove: return "parallel";
    }
    return "unknown";
  }

  struct Allocation {
    bool is_stack;
    u32 part_idx;
    union {
      Reg reg;
      i32 stack_off;
    };

    Allocation() : is_stack(false), part_idx(0), reg(Reg::make_invalid()) {}
    Allocation(Reg r, u32 part = 0) : is_stack(false), part_idx(part), reg(r) {}
    Allocation(i32 off, u32 part = 0) : is_stack(true), part_idx(part), stack_off(off) {}

    bool operator==(const Allocation &other) const {
      if (is_stack != other.is_stack || part_idx != other.part_idx) return false;
      return is_stack ? (stack_off == other.stack_off) : (reg == other.reg);
    }

    std::string format() const {
      if (is_stack) {
        return part_idx > 0 ? std::format("[sp+{}]:{}", stack_off, part_idx) : std::format("[sp+{}]", stack_off);
      } else {
        return part_idx > 0 ? std::format("r{}:{}", static_cast<u32>(reg.id()), part_idx) : std::format("r{}", static_cast<u32>(reg.id()));
      }
    }
  };
  
  // Hash function for Allocation
  struct AllocationHash {
    size_t operator()(const Allocation &a) const noexcept {
      if (a.is_stack) {
        return std::hash<i32>{}(a.stack_off);
      } else {
        // Reg has an id() method returning u8
        return std::hash<u8>{}(a.reg.id());
      }
    }
  };

  struct Operand {
    ValLocalIdx val_idx;
    Allocation alloc;
    bool is_constant = false;
    u64 constant_value = 0;

    std::string format() const {
      return std::format("%v{}@{}", static_cast<u32>(val_idx), alloc.format());
    }

    std::string format_with_current_alloc(const std::map<std::pair<ValLocalIdx, u32>, Allocation> &current_allocs) const {
      auto key = std::make_pair(val_idx, alloc.part_idx);
      auto it = current_allocs.find(key);
      const Allocation &alloc = it != current_allocs.end() ? it->second : this->alloc;
      return std::format("%v{}@{}", static_cast<u32>(val_idx), alloc.format());
    }
  };

  struct PhiIncoming {
    BlockIndex from_block;
    ValLocalIdx val_idx;
    Allocation alloc;

    std::string format() const {
      return std::format("b{}, %v{}@{}", static_cast<u32>(from_block), static_cast<u32>(val_idx), alloc.format());
    }
  };

  struct InstOp : Formattable {
    ValLocalIdx inst_id; // Identifier for the instruction (uses result val_idx if available, otherwise marker)
    util::SmallVector<Operand, 4> uses;  // Input operands
    util::SmallVector<Operand, 2> defs;  // Output operands
    BlockIndex branch_target = static_cast<BlockIndex>(~0u); // Target block for branches (invalid if not a branch)
    util::SmallVector<PhiIncoming, 4> phi_incomings; // Incoming values for PHI nodes (empty if not a PHI)
    std::string jump_type; // Jump type for branches (e.g., "jmp", "je", "jne")
    bool is_argument = false; // True if this is a function argument

    InstOp(ValLocalIdx id, util::SmallVector<Operand, 4> u, util::SmallVector<Operand, 2> d)
      : inst_id(id), uses(std::move(u)), defs(std::move(d)), branch_target(static_cast<BlockIndex>(~0u)), is_argument(false) {}
    InstOp(InstOp &&) noexcept = default;
    InstOp &operator=(InstOp &&) noexcept = default;

    std::string format(const std::map<std::pair<ValLocalIdx, u32>, Allocation> &current_allocs) const {
      if (is_argument) {
        if (defs.empty()) return "";
        std::string defs_str;
        for (size_t i = 0; i < defs.size(); ++i) {
          if (i > 0) defs_str += ",";
          defs_str += defs[i].format();
        }
        return std::format("  op defs={}\n", defs_str);
      } else if (!phi_incomings.empty()) {
        if (defs.empty()) return "";
        std::string incomings_str;
        for (size_t i = 0; i < phi_incomings.size(); ++i) {
          if (i > 0) incomings_str += ", ";
          incomings_str += phi_incomings[i].format();
        }
        return std::format("  phi {} [{}]\n", defs[0].format(), incomings_str);
      } else if (static_cast<u32>(branch_target) != ~0u) {
        std::string target_str = (static_cast<u32>(branch_target) & 0x80000000u)
          ? std::format("split_b{}", static_cast<u32>(branch_target) & 0x7FFFFFFFu)
          : std::format("b{}", static_cast<u32>(branch_target));
        if (uses.empty()) return std::format("  {} {}\n", jump_type, target_str);
        std::string uses_str;
        for (size_t i = 0; i < uses.size(); ++i) {
          if (i > 0) uses_str += ",";
          uses_str += uses[i].format_with_current_alloc(current_allocs);
        }
        return std::format("  {} {} uses={}\n", jump_type, target_str, uses_str);
      } else {
        if (uses.empty() && defs.empty()) return "";
        std::string uses_str, defs_str;
        if (!uses.empty()) {
          for (size_t i = 0; i < uses.size(); ++i) {
            if (i > 0) uses_str += ",";
            uses_str += uses[i].format_with_current_alloc(current_allocs);
          }
        }
        if (!defs.empty()) {
          for (size_t i = 0; i < defs.size(); ++i) {
            if (i > 0) defs_str += ",";
            defs_str += defs[i].format();
          }
        }
        if (uses_str.empty() && defs_str.empty()) return "";
        if (uses_str.empty()) return std::format("  op defs={}\n", defs_str);
        if (defs_str.empty()) return std::format("  op uses={}\n", uses_str);
        return std::format("  op uses={} defs={}\n", uses_str, defs_str);
      }
    }

    std::string format() const override {
      return format(std::map<std::pair<ValLocalIdx, u32>, Allocation>{});
    }
  };

  struct Edit : Formattable {
    EditKind kind;

    // Common fields for most edits
    Allocation from;
    Allocation to;
    ValLocalIdx val_idx;
    u32 size;

    // For ParallelMove
    util::SmallVector<std::pair<Operand, Operand>, 4> parallel_moves;

    // Constructors for different edit kinds
    Edit(EditKind k, Allocation f, Allocation t, ValLocalIdx v, u32 s)
      : kind(k), from(f), to(t), val_idx(v), size(s) {}

    Edit(EditKind k, Reg src, Reg dst, u32 s)
      : kind(k), from(Allocation(src)), to(Allocation(dst)), size(s) {}

    Edit(EditKind k, util::SmallVector<std::pair<Operand, Operand>, 4> moves)
      : kind(k), parallel_moves(std::move(moves)) {}

    std::string format() const override {
      std::string kind_str = format_edit_kind(kind);

      if (kind == EditKind::RegMove) {
        return std::format("  edit {} {} -> {} {}b\n", kind_str, from.format(), to.format(), size);
      } else if (kind == EditKind::ParallelMove) {
        std::string result = "  parallel ";
        for (size_t i = 0; i < parallel_moves.size(); ++i) {
          if (i > 0) result += ", ";
          const auto &[src, dst] = parallel_moves[i];
          result += src.format() + " -> " + dst.format();
        }
        result += "\n";
        return result;
      } else {
        return std::format("  edit {} {} -> {} %v{} {}b\n", kind_str, from.format(), to.format(), static_cast<u32>(val_idx), size);
      }
    }
  };
  enum class BlockEntryKind {
    Inst,
    Edit
  };

  struct BlockEntry {
    BlockEntryKind kind;
    union {
      InstOp inst;
      Edit edit;
    };

    BlockEntry(InstOp i) : kind(BlockEntryKind::Inst), inst(std::move(i)) {}
    BlockEntry(Edit e) : kind(BlockEntryKind::Edit), edit(std::move(e)) {}

    BlockEntry(BlockEntry &&other) noexcept : kind(other.kind) {
      if (kind == BlockEntryKind::Inst) {
        new (&inst) InstOp(std::move(other.inst));
      } else {
        new (&edit) Edit(std::move(other.edit));
      }
    }

    BlockEntry &operator=(BlockEntry &&other) noexcept {
      if (this != &other) {
        this->~BlockEntry();
        kind = other.kind;
        if (kind == BlockEntryKind::Inst) {
          new (&inst) InstOp(std::move(other.inst));
        } else {
          new (&edit) Edit(std::move(other.edit));
        }
      }
      return *this;
    }

    ~BlockEntry() {
      if (kind == BlockEntryKind::Inst) {
        inst.~InstOp();
      } else {
        edit.~Edit();
      }
    }

    std::string format(const std::map<std::pair<ValLocalIdx, u32>, Allocation> &current_allocs) const {
      if (kind == BlockEntryKind::Inst) {
        return inst.format(current_allocs);
      } else {
        return edit.format();
      }
    }
  };

  struct BlockInfo {
      BlockIndex block_idx;
      util::SmallVector<BlockEntry, 24> entries;

      BlockInfo() = default;
      BlockInfo(BlockInfo &&) noexcept = default;
      BlockInfo &operator=(BlockInfo &&) noexcept = default;

    std::string format() const {
      std::string result =
          (static_cast<u32>(block_idx) & 0x80000000u)
              ? std::format("block split_b{}:\n",
                            static_cast<u32>(block_idx) & 0x7FFFFFFFu)
              : std::format("block b{}:\n", static_cast<u32>(block_idx));
      std::map<std::pair<ValLocalIdx, u32>, Allocation> current_allocs;
      for (const auto &entry : entries) {
        if (entry.kind == BlockEntryKind::Edit) {
          if (entry.edit.kind == EditKind::Reload) {
            current_allocs[{entry.edit.val_idx, entry.edit.from.part_idx}] =
                entry.edit.to;
          } else if (entry.edit.kind == EditKind::Spill) {
            current_allocs[{entry.edit.val_idx, entry.edit.from.part_idx}] =
                entry.edit.to;
          }
        }
        result += entry.format(current_allocs);
      }
      result += "\n";
      return result;
    }
  };

  struct EdgeInfo {
    BlockIndex from;
    BlockIndex to;
    util::SmallVector<Edit, 4> parallel_moves;

    EdgeInfo() = default;
    EdgeInfo(EdgeInfo &&) noexcept = default;
    EdgeInfo &operator=(EdgeInfo &&) noexcept = default;

    std::string format() const {
      std::string result = std::format("edge b{} -> b{}:\n", static_cast<u32>(from), static_cast<u32>(to));
      for (const auto &edit : parallel_moves) {
        if (edit.kind == EditKind::ParallelMove) {
          result += "  parallel ";
          for (size_t i = 0; i < edit.parallel_moves.size(); ++i) {
            if (i > 0) result += ", ";
            const auto &[src, dst] = edit.parallel_moves[i];
            result += src.format() + " -> " + dst.format();
          }
          result += "\n";
        }
      }
      result += "\n";
      return result;
    }
  };

  util::SmallVector<BlockInfo, 8> blocks;
  util::SmallVector<EdgeInfo, 8> edges;
  std::string func_name;
  
  /// Track condition value for current branch (set before generate_branch_to_block)
  util::SmallVector<Operand, 4> current_branch_condition;
  
  // Split block management
  BlockIndex current_split_block = static_cast<BlockIndex>(~0u);
  std::map<std::pair<BlockIndex, BlockIndex>, BlockIndex> split_blocks;
  
  // Current block index (set by CompilerBase)
  BlockIndex current_block_idx = static_cast<BlockIndex>(~0u);
  bool active_compilation = false;

  void reset() {
    blocks.clear();
    edges.clear();
    func_name.clear();
    current_branch_condition.clear();
    current_split_block = static_cast<BlockIndex>(~0u);
    split_blocks.clear();
    current_block_idx = static_cast<BlockIndex>(~0u);
    active_compilation = false;
    next_constant_vreg = 0x80000000u;
  }

  void set_current_block(BlockIndex block_idx) noexcept {
    current_block_idx = block_idx;
  }

  /// Get current block index for edits (returns split block if in split,
  /// otherwise current block)
  BlockIndex get_edit_block() const noexcept {
    if (static_cast<u32>(current_split_block) != ~0u) {
      return current_split_block;
    }
    return current_block_idx;
  }

  void set_branch_condition(util::SmallVector<Operand, 4> ops) noexcept {
    current_branch_condition = std::move(ops);
  }

  void clear_branch_condition() noexcept {
    current_branch_condition.clear();
  }
  u32 next_constant_vreg =0x80000000u;
  ValLocalIdx materialize_constant(Reg reg) noexcept {
    auto constant_vreg = static_cast<ValLocalIdx>(++next_constant_vreg);
    util::SmallVector<Operand, 1> defs;
    defs.push_back({constant_vreg, Allocation(reg)});
    defs[0].is_constant = true;
    emit_inst_op(constant_vreg, {}, std::move(defs));
    return constant_vreg;
  }


  /// Get branch condition (for CMP emission)
  const util::SmallVector<Operand, 4>& get_branch_condition() const noexcept {
    return current_branch_condition;
  }
  
  /// Check if branch condition is set
  bool has_branch_condition() const noexcept {
    return !current_branch_condition.empty();
  }
  
  /// Set function name
  template<typename StringLike>
  void set_func_name(const StringLike &name) noexcept {
    func_name = std::string(name);
  }
  
  /// Get function name
  const std::string &get_func_name() const noexcept {
    return func_name;
  }
  
  BlockIndex begin_branch(BlockIndex from_block, BlockIndex to_block, bool is_split) noexcept {
    if (is_split) {
      auto key = std::make_pair(from_block, to_block);
      auto it = split_blocks.find(key);
      if (it == split_blocks.end()) {
        BlockIndex split_idx = static_cast<BlockIndex>(static_cast<u32>(to_block) | 0x80000000u);
        split_blocks[key] = split_idx;
        current_split_block = split_idx;
        BlockInfo bi;
        bi.block_idx = split_idx;
        blocks.push_back(std::move(bi));
      } else {
        current_split_block = it->second;
      }
      return current_split_block;
    } else {
      current_split_block = static_cast<BlockIndex>(~0u);
      return from_block;
    }
  }


  void end_branch() noexcept {
    current_split_block = static_cast<BlockIndex>(~0u);
  }

  void emit_inst_op(ValLocalIdx inst_id,
                    util::SmallVector<Operand, 4> uses,
                    util::SmallVector<Operand, 2> defs) noexcept {
    BlockIndex block_idx = current_block_idx;
    if (current_split_block != static_cast<BlockIndex>(~0u)) {
      block_idx = current_split_block;
    }
    InstOp inst_op{inst_id, std::move(uses), std::move(defs)};

    // Find or create block info
    auto it = std::find_if(
        blocks.begin(), blocks.end(), [block_idx](const BlockInfo &bi) {
          return bi.block_idx == block_idx;
        });
    if (it == blocks.end()) {
      BlockInfo block_info;
      block_info.block_idx = block_idx;
      blocks.push_back(std::move(block_info));
      it = blocks.end() - 1;
    }
    it->entries.push_back(std::move(inst_op));
  }
  
  /// Emit edit operation (uses get_edit_block())
  void emit_edit(EditKind kind,
                  Allocation from, Allocation to,
                  ValLocalIdx val_idx,
                  u32 part_idx,
                  u32 size) noexcept {
    from.part_idx = part_idx;
    BlockIndex edit_block = get_edit_block();
    Edit edit{kind, from, to, val_idx, size};

    // Find or create block info
    auto it = std::find_if(blocks.begin(), blocks.end(),
                          [edit_block](const BlockInfo &bi) {
                            return bi.block_idx == edit_block;
                          });
    if (it == blocks.end()) {
      BlockInfo block_info;
      block_info.block_idx = edit_block;
      blocks.push_back(std::move(block_info));
      it = blocks.end() - 1;
    }
    it->entries.push_back(std::move(edit));
  }
  
  /// Emit register-to-register move (uses get_edit_block())
  void emit_reg_move(Reg src, Reg dst, u32 size) noexcept {
    if (active_compilation) {
      return;
    }
    BlockIndex edit_block = get_edit_block();
    Edit edit{EditKind::RegMove, src, dst, size};

    // Find or create block info
    auto it = std::find_if(blocks.begin(), blocks.end(),
                          [edit_block](const BlockInfo &bi) {
                            return bi.block_idx == edit_block;
                          });
    if (it == blocks.end()) {
      BlockInfo block_info;
      block_info.block_idx = edit_block;
      blocks.push_back(std::move(block_info));
      it = blocks.end() - 1;
    }
    it->entries.push_back(std::move(edit));
  }
  
  void emit_arg(BlockIndex entry_block_idx, ValLocalIdx val_idx, u32 part_idx, Allocation alloc) noexcept {
    alloc.part_idx = part_idx;
    util::SmallVector<Operand, 1> defs;
    defs.push_back({val_idx, alloc});
    InstOp inst_op{val_idx, {}, std::move(defs)};
    inst_op.is_argument = true;
    auto it = std::find_if(blocks.begin(), blocks.end(),
                          [entry_block_idx](const BlockInfo &bi) { return bi.block_idx == entry_block_idx; });
    if (it == blocks.end()) {
      BlockInfo bi;
      bi.block_idx = entry_block_idx;
      blocks.push_back(std::move(bi));
      it = blocks.end() - 1;
    }
    it->entries.push_back(std::move(inst_op));
  }
  
  void emit_phi(ValLocalIdx phi_idx, u32 part_idx, Allocation phi_alloc, util::SmallVector<PhiIncoming, 4> incomings) noexcept {
    phi_alloc.part_idx = part_idx;
    util::SmallVector<Operand, 1> defs;
    defs.push_back({phi_idx, phi_alloc});
    InstOp inst_op{phi_idx, {}, std::move(defs)};
    inst_op.phi_incomings = std::move(incomings);
    auto it = std::find_if(blocks.begin(), blocks.end(),
                          [this](const BlockInfo &bi) { return bi.block_idx == current_block_idx; });
    if (it == blocks.end()) {
      BlockInfo bi;
      bi.block_idx = current_block_idx;
      blocks.push_back(std::move(bi));
      it = blocks.end() - 1;
    }
    it->entries.push_back(std::move(inst_op));
  }

  template<typename IncomingData>
  void vir_emit_phi(ValLocalIdx phi_idx, u32 part_idx, Allocation phi_alloc,
                    const IncomingData &incoming_data) noexcept {
    util::SmallVector<PhiIncoming, 4> incomings;
    for (const auto &[block_idx, val_idx, alloc] : incoming_data) {
      PhiIncoming incoming{block_idx, val_idx, alloc};
      incoming.alloc.part_idx = part_idx;
      incomings.push_back(incoming);
    }
    emit_phi(phi_idx, part_idx, phi_alloc, std::move(incomings));
  }
  
  void emit_branch(ValLocalIdx inst_id, util::SmallVector<Operand, 4> uses, BlockIndex target) noexcept {
    InstOp inst_op{inst_id, std::move(uses), {}};
    inst_op.branch_target = target;
    auto it = std::find_if(blocks.begin(), blocks.end(),
                          [this](const BlockInfo &bi) { return bi.block_idx == current_block_idx; });
    if (it == blocks.end()) {
      BlockInfo bi;
      bi.block_idx = current_block_idx;
      blocks.push_back(std::move(bi));
      it = blocks.end() - 1;
    }
    it->entries.push_back(std::move(inst_op));
  }

  void capture_branch(const char* jump_type,
                      BlockIndex target_block,
                      bool is_split) noexcept {
    // Set up split block for edits
    BlockIndex current_block= current_split_block !=  static_cast<BlockIndex>(~0u)?current_split_block:current_block_idx;
    BlockIndex edit_block =
        begin_branch(current_block_idx, target_block, is_split);

    // Get condition uses if available
    util::SmallVector<Operand, 4> uses;
    if (!current_branch_condition.empty()) {
      uses = std::move(current_branch_condition);
      current_branch_condition.clear();
    }
    
    // Determine the actual branch target:
    // - If splitting, the conditional jump goes to the split block
    // - Otherwise, it goes directly to the target
    BlockIndex branch_target = is_split ? edit_block : target_block;


    // Create branch instruction - always in the source block
    // Use a marker val_idx based on the block index (will never conflict with real values)
    ValLocalIdx branch_id = static_cast<ValLocalIdx>(static_cast<u32>(current_block) | 0x40000000u);
    InstOp inst_op{
      branch_id,
        std::move(uses),
        util::SmallVector<Operand, 0>{} // Branches don't define
    };
    inst_op.branch_target = branch_target;
    inst_op.jump_type = jump_type ? jump_type : "jmp";
    
    // Find or create block info for the SOURCE block
    auto it = std::find_if(blocks.begin(), blocks.end(),
                          [this, current_block](const BlockInfo &bi) {
                            return bi.block_idx == current_block;
                           });
    if (it == blocks.end()) {
      BlockInfo block_info;
      block_info.block_idx = current_block;
      blocks.push_back(std::move(block_info));
      it = blocks.end() - 1;
    }
    
    // Check if a branch already exists and replace it
    auto branch_it = std::find_if(it->entries.begin(), it->entries.end(),
                                   [branch_target](const BlockEntry &entry) {
                       return entry.kind == BlockEntryKind::Inst &&
                              entry.inst.branch_target == branch_target;
                     });
    if (branch_it != it->entries.end()) {
      // Replace existing branch with the one from x64 (which has jump type)
      branch_it->inst = std::move(inst_op);
    } else {
      it->entries.push_back(std::move(inst_op));
    }
    if (is_split) {
      current_split_block = edit_block;
    } else {
      current_split_block = static_cast<BlockIndex>(~0u);
    }
  }

  /// Emit parallel move on edge (currently unused)
  void emit_edge_parallel_move(
      BlockIndex from,
      BlockIndex to,
      util::SmallVector<std::pair<Operand, Operand>, 4> moves) noexcept {
    EdgeInfo edge;
    edge.from = from;
    edge.to = to;
    Edit edit{EditKind::ParallelMove, std::move(moves)};
    edge.parallel_moves.push_back(std::move(edit));
    edges.push_back(std::move(edge));
  }



  void write_to_file(const std::string &filename) const noexcept {
    std::ofstream out(filename);
    if (!out.is_open()) {
      return;
    }

    // Write function header
    out << std::format("function {}\n\n", func_name);

    for (const auto &block : blocks) {
      out << block.format();
    }

    for (const auto &edge : edges) {
      out << edge.format();
    }
  }
};

} // namespace tpde
