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
  using BlockIndex = BlockIndexType;
  using IRInstRef = IRInstRefType;
  enum class EditKind : u8 {
    Move,         // Move between registers
    Spill,        // Spill register to stack
    Reload,       // Reload from stack to register
    ParallelMove, // Parallel move on edge (phi resolution)
    RegMove       // Register-to-register move (for phi resolution and general moves)
  };

  struct Allocation {
    bool is_stack;
    union {
      Reg reg;
      i32 stack_off;
    };
    
    Allocation() : is_stack(false), reg(Reg::make_invalid()) {}
    Allocation(Reg r) : is_stack(false), reg(r) {}
    Allocation(i32 off) : is_stack(true), stack_off(off) {}
    
    bool operator==(const Allocation &other) const {
      if (is_stack != other.is_stack) return false;
      return is_stack ? (stack_off == other.stack_off) : (reg == other.reg);
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
    u32 part_idx;
    Allocation alloc;
  };

  struct PhiIncoming {
    BlockIndex from_block;
    ValLocalIdx val_idx;
    u32 part_idx;
    Allocation alloc; // Allocation of incoming value at the source block
  };

  struct InstOp {
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
  };

  struct Edit {
    EditKind kind;
    Allocation from;
    Allocation to;
    ValLocalIdx val_idx;
    u32 part_idx;
    u32 size;
    
    // For ParallelMove
    util::SmallVector<std::pair<Operand, Operand>, 4> parallel_moves;
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
  };

  struct BlockInfo {
      BlockIndex block_idx;
      util::SmallVector<BlockEntry, 24> entries;
      
      BlockInfo() = default;
      BlockInfo(BlockInfo &&) noexcept = default;
      BlockInfo &operator=(BlockInfo &&) noexcept = default;
  };

  struct EdgeInfo {
    BlockIndex from;
    BlockIndex to;
    util::SmallVector<Edit, 4> parallel_moves;

    EdgeInfo() = default;
    EdgeInfo(EdgeInfo &&) noexcept = default;
    EdgeInfo &operator=(EdgeInfo &&) noexcept = default;
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
  
  void reset() {
    blocks.clear();
    edges.clear();
    func_name.clear();
    current_branch_condition.clear();
    current_split_block = static_cast<BlockIndex>(~0u);
    split_blocks.clear();
    current_block_idx = static_cast<BlockIndex>(~0u);
  }
  
  /// Set current block index
  void set_current_block(BlockIndex block_idx) noexcept {
    current_block_idx = block_idx;
  }
  
  /// Get current block index for edits (returns split block if in split, otherwise current block)
  BlockIndex get_edit_block() const noexcept {
    if (static_cast<u32>(current_split_block) != ~0u) {
      return current_split_block;
    }
    return current_block_idx;
  }
  
  /// Set branch condition operands
  void set_branch_condition(util::SmallVector<Operand, 4> ops) noexcept {
    current_branch_condition = std::move(ops);
  }
  
  /// Clear branch condition
  void clear_branch_condition() noexcept {
    current_branch_condition.clear();
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
  
  /// Begin branch region (returns edit block index)
  BlockIndex begin_branch(BlockIndex from_block, BlockIndex to_block, bool is_split) noexcept {
    if (is_split) {
      // Create or get split block
      auto key = std::make_pair(from_block, to_block);
      auto it = split_blocks.find(key);
      if (it == split_blocks.end()) {
        // Create new split block index (use high bits to distinguish from real blocks)
        BlockIndex split_idx = static_cast<BlockIndex>(static_cast<u32>(to_block) | 0x80000000u);
        split_blocks[key] = split_idx;
        current_split_block = split_idx;
        
        // Create block info for split block
        BlockInfo block_info;
        block_info.block_idx = split_idx;
        blocks.push_back(std::move(block_info));
      } else {
        current_split_block = it->second;
      }
      return current_split_block;
    } else {
      current_split_block = static_cast<BlockIndex>(~0u);
      return from_block;
    }
  }
  
  /// End branch region
  void end_branch() noexcept {
    current_split_block = static_cast<BlockIndex>(~0u);
  }
  
  /// Emit instruction operation (uses current_block_idx)
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
    BlockIndex edit_block = get_edit_block();
    Edit edit;
    edit.kind = kind;
    edit.from = from;
    edit.to = to;
    edit.val_idx = val_idx;
    edit.part_idx = part_idx;
    edit.size = size;
    
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
  void emit_reg_move(Reg src, Reg dst, ValLocalIdx val_idx, u32 part_idx, u32 size) noexcept {
    BlockIndex edit_block = get_edit_block();
    Edit edit;
    edit.kind = EditKind::RegMove;
    edit.from = Allocation(src);
    edit.to = Allocation(dst);
    edit.val_idx = val_idx;
    edit.part_idx = part_idx;
    edit.size = size;

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
  
  /// Emit function argument assignment
  void emit_arg(BlockIndex entry_block_idx, ValLocalIdx val_idx, u32 part_idx, Allocation alloc) noexcept {
    util::SmallVector<Operand, 0> uses; // Arguments have no uses
    util::SmallVector<Operand, 1> defs;
    Operand op;
    op.val_idx = val_idx;
    op.part_idx = part_idx;
    op.alloc = alloc;
    defs.push_back(op);
    
    // Use the argument's val_idx as the instruction identifier
    InstOp inst_op{val_idx, std::move(uses), std::move(defs)};
    inst_op.is_argument = true; // Mark as argument
    
    // Find or create block info for entry block
    auto it = std::find_if(blocks.begin(), blocks.end(),
                          [entry_block_idx](const BlockInfo &bi) {
                            return bi.block_idx == entry_block_idx;
                          });
    if (it == blocks.end()) {
      BlockInfo block_info;
      block_info.block_idx = entry_block_idx;
      blocks.push_back(std::move(block_info));
      it = blocks.end() - 1;
    }
    // Arguments are added to the entry block - output code will ensure they come first
    it->entries.push_back(std::move(inst_op));
  }
  
  /// Emit PHI node assignment (uses current_block_idx)
  void emit_phi(ValLocalIdx phi_idx, u32 part_idx,
                Allocation phi_alloc,
                util::SmallVector<PhiIncoming, 4> incomings) noexcept {
    BlockIndex block_idx = current_block_idx;
    util::SmallVector<Operand, 0> uses; // PHI uses come from parallel moves
    util::SmallVector<Operand, 1> defs;
    Operand op;
    op.val_idx = phi_idx;
    op.part_idx = part_idx;
    op.alloc = phi_alloc;
    defs.push_back(op);
    
    // Use the PHI's val_idx as the instruction identifier
    InstOp inst_op{phi_idx, std::move(uses), std::move(defs)};
    inst_op.phi_incomings = std::move(incomings);
    
    // Find or create block info
    auto it = std::find_if(blocks.begin(), blocks.end(),
                          [block_idx](const BlockInfo &bi) {
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
  
  /// Emit PHI node with incoming values (helper method that collects incoming data)
  /// Takes incoming data as a vector of tuples: (block_idx, val_idx, allocation)
  template<typename IncomingData>
  void vir_emit_phi(ValLocalIdx phi_idx, u32 part_idx, Allocation phi_alloc,
                    const IncomingData &incoming_data) noexcept {
    util::SmallVector<PhiIncoming, 4> incomings;
    for (const auto &[block_idx, val_idx, alloc] : incoming_data) {
      if (val_idx == static_cast<ValLocalIdx>(~0u)) continue; // Skip constants/undef
      PhiIncoming incoming;
      incoming.from_block = block_idx;
      incoming.val_idx = val_idx;
      incoming.part_idx = part_idx;
      incoming.alloc = alloc;
      incomings.push_back(incoming);
    }
    emit_phi(phi_idx, part_idx, phi_alloc, std::move(incomings));
  }
  
  /// Emit branch instruction (uses current_block_idx)
  void emit_branch(ValLocalIdx inst_id,
                   util::SmallVector<Operand, 4> uses, BlockIndex target) noexcept {
    BlockIndex block_idx = current_block_idx;
    util::SmallVector<Operand, 0> defs; // Branches don't define values
    
    // Create InstOp with branch target
    InstOp inst_op{inst_id, std::move(uses), std::move(defs)};
    inst_op.branch_target = target;
    
    // Find or create block info
    auto it = std::find_if(blocks.begin(), blocks.end(),
                          [block_idx](const BlockInfo &bi) {
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
  
  /// Capture branch instruction (uses current_block_idx)
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

    // Note: For split conditional branches, the jmp to final target will be
    // added by the normal code generation flow after move_values_to_match is
    // called.
    // This ensures the correct ordering: spill operations first, then jmp.
  }

  /// Emit parallel move on edge
  void emit_edge_parallel_move(
      BlockIndex from,
      BlockIndex to,
      util::SmallVector<std::pair<Operand, Operand>, 4> moves) noexcept {
    EdgeInfo edge;
    edge.from = from;
    edge.to = to;
    Edit edit;
    edit.kind = EditKind::ParallelMove;
    edit.parallel_moves = std::move(moves);
    edge.parallel_moves.push_back(std::move(edit));
    edges.push_back(std::move(edge));
  }
  
  /// Write verification IR to file
  void write_to_file(const std::string &filename) const noexcept {
    std::ofstream out(filename);
    if (!out.is_open()) {
      return; // Can't log here, just fail silently
    }

    out << "function " << func_name << "\n\n";

    // Write blocks
    for (const auto &block : blocks) {
      // Check if this is a split block
      if (static_cast<u32>(block.block_idx) & 0x80000000u) {
        // This is a split block - output it with a special name
        u32 base_block = static_cast<u32>(block.block_idx) & 0x7FFFFFFFu;
        out << "block split_b" << base_block << ":\n";
      } else {
        out << "block b" << static_cast<u32>(block.block_idx) << ":\n";
      }

      // Track current allocation for each value as we process entries
      std::map<std::pair<ValLocalIdx, u32>, Allocation> current_allocs;

      // Output entries in the exact order they appear in block.entries
      for (const auto &entry : block.entries) {
        if (entry.kind == BlockEntryKind::Edit) {
          const auto &edit = entry.edit;
          // Update current allocation for reload operations
          if (edit.kind == EditKind::Reload) {
            current_allocs[{edit.val_idx, edit.part_idx}] = edit.to;
          }
          out << "  edit ";
          switch (edit.kind) {
          case EditKind::Move: out << "move "; break;
          case EditKind::Spill:
              out << "spill ";
              break;
            case EditKind::Reload:
              out << "reload ";
              break;
            case EditKind::RegMove:
              out << "regmove ";
              break;
            case EditKind::ParallelMove:
              out << "parallel ";
              break;
          }
          if (edit.from.is_stack) {
            out << "[sp+" << edit.from.stack_off << "]";
          } else {
            out << "r" << static_cast<u32>(edit.from.reg.id());
          }
          out << " -> ";
          if (edit.to.is_stack) {
            out << "[sp+" << edit.to.stack_off << "]";
          } else {
            out << "r" << static_cast<u32>(edit.to.reg.id());
          }
          out << " %v" << static_cast<u32>(edit.val_idx);
          if (edit.part_idx > 0) out << ":" << edit.part_idx;
          out << "\n";
        } else {
          const auto &inst_op = entry.inst;
          // Check if this is an argument
          if (inst_op.is_argument) {
            out << "  op";
            // Arguments only have defs, no uses
            if (!inst_op.defs.empty()) {
              out << " defs=";
              for (size_t i = 0; i < inst_op.defs.size(); ++i) {
                if (i > 0) {
                  out << ",";
                }
                const auto &op = inst_op.defs[i];
                out << "%v" << static_cast<u32>(op.val_idx);
                if (op.part_idx > 0) {
                  out << ":" << op.part_idx;
                }
                out << "@";
                if (op.alloc.is_stack) {
                  out << "[sp+" << op.alloc.stack_off << "]";
                } else {
                  out << "r" << static_cast<u32>(op.alloc.reg.id());
                }
              }
            }
            out << "\n";
          } else if (!inst_op.phi_incomings.empty()) {
            // This is a PHI node - format as phi with incomings
            if (!inst_op.defs.empty()) {
              const auto &def = inst_op.defs[0];
              out << "  phi %v" << static_cast<u32>(def.val_idx);
              if (def.part_idx > 0) out << ":" << def.part_idx;
              out << "@";
              if (def.alloc.is_stack) {
                out << "[sp+" << def.alloc.stack_off << "]";
              } else {
                out << "r" << static_cast<u32>(def.alloc.reg.id());
              }
              out << " [";
              for (size_t i = 0; i < inst_op.phi_incomings.size(); ++i) {
                if (i > 0) out << ", ";
                const auto &inc = inst_op.phi_incomings[i];
                out << "b" << static_cast<u32>(inc.from_block) << ", %v" << static_cast<u32>(inc.val_idx);
                if (inc.part_idx > 0) out << ":" << inc.part_idx;
                out << "@";
                if (inc.alloc.is_stack) {
                  out << "[sp+" << inc.alloc.stack_off << "]";
                } else {
                  out << "r" << static_cast<u32>(inc.alloc.reg.id());
                }
              }
              out << "]\n";
            }
          } else if (static_cast<u32>(inst_op.branch_target) != ~0u) {
            // This is a branch instruction
            // Check if this is a split block target
            if (static_cast<u32>(inst_op.branch_target) & 0x80000000u) {
              // This is a split block - format as split_bX
              u32 base_block =
                  static_cast<u32>(inst_op.branch_target) & 0x7FFFFFFFu;
              out << "  " << inst_op.jump_type << " split_b" << base_block;
            } else {
              out << "  " << inst_op.jump_type << " b"
                  << static_cast<u32>(inst_op.branch_target);
            }
            // Write uses if any (for conditional branches)
            if (!inst_op.uses.empty()) {
              out << " uses=";
              for (size_t i = 0; i < inst_op.uses.size(); ++i) {
                if (i > 0) {
                  out << ",";
                }
                const auto &op = inst_op.uses[i];
                out << "%v" << static_cast<u32>(op.val_idx);
                if (op.part_idx > 0) {
                  out << ":" << op.part_idx;
                }
                out << "@";
                // Use current allocation if available, otherwise use the
                // operand's allocation
                auto key = std::make_pair(op.val_idx, op.part_idx);
                auto it = current_allocs.find(key);
                if (it != current_allocs.end()) {
                  const auto &alloc = it->second;
                  if (alloc.is_stack) {
                    out << "[sp+" << alloc.stack_off << "]";
                  } else {
                    out << "r" << static_cast<u32>(alloc.reg.id());
                  }
                } else {
                  if (op.alloc.is_stack) {
                    out << "[sp+" << op.alloc.stack_off << "]";
                  } else {
                    out << "r" << static_cast<u32>(op.alloc.reg.id());
                  }
                }
              }
            }
            out << "\n";
          } else {
            // Regular instruction - skip if it has no uses and no defs (empty
            // op)
            if (inst_op.uses.empty() && inst_op.defs.empty()) {
              continue; // Skip empty instructions
            }
            out << "  op";
            // Write uses
            if (!inst_op.uses.empty()) {
              out << " uses=";
              for (size_t i = 0; i < inst_op.uses.size(); ++i) {
                if (i > 0) out << ",";
                const auto &op = inst_op.uses[i];
                out << "%v" << static_cast<u32>(op.val_idx);
                if (op.part_idx > 0) out << ":" << op.part_idx;
                out << "@";
                // Use current allocation if available, otherwise use the
                // operand's allocation
                auto key = std::make_pair(op.val_idx, op.part_idx);
                auto it = current_allocs.find(key);
                if (it != current_allocs.end()) {
                  const auto &alloc = it->second;
                  if (alloc.is_stack) {
                    out << "[sp+" << alloc.stack_off << "]";
                  } else {
                    out << "r" << static_cast<u32>(alloc.reg.id());
                  }
                } else {
                  if (op.alloc.is_stack) {
                    out << "[sp+" << op.alloc.stack_off << "]";
                  } else {
                    out << "r" << static_cast<u32>(op.alloc.reg.id());
                  }
                }
              }
            }
            // Write defs
            if (!inst_op.defs.empty()) {
              out << " defs=";
              for (size_t i = 0; i < inst_op.defs.size(); ++i) {
                if (i > 0) out << ",";
                const auto &op = inst_op.defs[i];
                out << "%v" << static_cast<u32>(op.val_idx);
                if (op.part_idx > 0) out << ":" << op.part_idx;
                out << "@";
                if (op.alloc.is_stack) {
                  out << "[sp+" << op.alloc.stack_off << "]";
                } else {
                  out << "r" << static_cast<u32>(op.alloc.reg.id());
                }
              }
            }
            out << "\n";
          }
        }
      }
      out << "\n";
    }

    // Write edges
    for (const auto &edge : edges) {
      out << "edge b" << static_cast<u32>(edge.from) << " -> b" << static_cast<u32>(edge.to) << ":\n";
      for (const auto &edit : edge.parallel_moves) {
        if (edit.kind == EditKind::ParallelMove) {
          out << "  parallel ";
          for (size_t i = 0; i < edit.parallel_moves.size(); ++i) {
            if (i > 0) out << ", ";
            const auto &[src, dst] = edit.parallel_moves[i];
            out << "%v" << static_cast<u32>(src.val_idx);
            if (src.part_idx > 0) out << ":" << src.part_idx;
            out << "@";
            if (src.alloc.is_stack) {
              out << "[sp+" << src.alloc.stack_off << "]";
            } else {
              out << "r" << static_cast<u32>(src.alloc.reg.id());
            }
            out << " -> %v" << static_cast<u32>(dst.val_idx);
            if (dst.part_idx > 0) out << ":" << dst.part_idx;
            out << "@";
            if (dst.alloc.is_stack) {
              out << "[sp+" << dst.alloc.stack_off << "]";
            } else {
              out << "r" << static_cast<u32>(dst.alloc.reg.id());
            }
          }
          out << "\n";
        }
      }
      out << "\n";
    }
  }
};

} // namespace tpde

