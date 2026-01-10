// SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#pragma once

#include <ostream>
#include <format>
#include <algorithm>

#include "base.hpp"
#include "IRAdaptor.hpp"
#include "util/SmallVector.hpp"

namespace tpde {

template <IRAdaptor Adaptor>
struct DominatorTree {
  using IRBlockRef = typename Adaptor::IRBlockRef;

  // Block index type - same as in Analyzer
  enum class BlockIndex : u32 {};
  static constexpr BlockIndex INVALID_BLOCK_IDX = static_cast<BlockIndex>(~0u);

  // Storage indexed by BlockIndex
  // idom[i] = immediate dominator block index of block i
  util::SmallVector<BlockIndex, 64> idom;

  // children[i] = list of blocks that i immediately dominates
  util::SmallVector<util::SmallVector<BlockIndex, 4>, 64> children;

  // DFS numbering for O(1) dominates() queries
  // A dominates B iff dfs_num_in[A] <= dfs_num_in[B] and dfs_num_out[B] <= dfs_num_out[A]
  util::SmallVector<u32, 64> dfs_num_in;
  util::SmallVector<u32, 64> dfs_num_out;
  u32 dfs_counter = 0;

  // Compute dominator tree given block layout (in RPO order)
  void compute(Adaptor* adaptor,
               const util::SmallVector<IRBlockRef, 64>& block_layout) noexcept;

  // Query: does block a dominate block b?
  bool dominates(BlockIndex a, BlockIndex b) const noexcept {
    const auto a_idx = static_cast<u32>(a);
    const auto b_idx = static_cast<u32>(b);
    if (a_idx >= dfs_num_in.size() || b_idx >= dfs_num_in.size()) {
      return false;
    }
    return dfs_num_in[a_idx] <= dfs_num_in[b_idx] &&
           dfs_num_out[b_idx] <= dfs_num_out[a_idx];
  }

  // Get immediate dominator of a block
  BlockIndex get_idom(BlockIndex block_idx) const noexcept {
    const auto idx = static_cast<u32>(block_idx);
    if (idx >= idom.size()) {
      return INVALID_BLOCK_IDX;
    }
    return idom[idx];
  }

  // Print dominator tree structure
  void print(std::ostream& os, Adaptor* adaptor,
             const util::SmallVector<IRBlockRef, 64>& block_layout) const;

 private:
  // Helper: DFS traversal to assign DFS numbers (pre/post-order)
  void assignDFSNumbers(BlockIndex node, Adaptor* adaptor,
                        const util::SmallVector<IRBlockRef, 64>& block_layout);

  // Helper: compute immediate dominators using simplified Lengauer-Tarjan
  void computeIDom(Adaptor* adaptor,
                   const util::SmallVector<IRBlockRef, 64>& block_layout) noexcept;
};

// Implementation
template <IRAdaptor Adaptor>
void DominatorTree<Adaptor>::compute(
    Adaptor* adaptor, const util::SmallVector<IRBlockRef, 64>& block_layout) noexcept {
  if (block_layout.empty()) {
    return;
  }

  // Initialize storage
  const u32 num_blocks = static_cast<u32>(block_layout.size());
  idom.resize_uninitialized(num_blocks);
  children.resize_uninitialized(num_blocks);
  dfs_num_in.resize_uninitialized(num_blocks);
  dfs_num_out.resize_uninitialized(num_blocks);

  for (u32 i = 0; i < num_blocks; ++i) {
    idom[i] = INVALID_BLOCK_IDX;
    children[i].clear();
    dfs_num_in[i] = 0;
    dfs_num_out[i] = 0;
  }

  // Compute immediate dominators
  computeIDom(adaptor, block_layout);

  // Build children relationships
  for (u32 i = 1; i < num_blocks; ++i) {
    const auto block_idx = static_cast<BlockIndex>(i);
    const auto parent_idx = idom[i];
    if (parent_idx != INVALID_BLOCK_IDX) {
      const auto parent = static_cast<u32>(parent_idx);
      children[parent].push_back(block_idx);
    }
  }

  // Assign DFS numbers via tree walk for O(1) dominates() queries
  dfs_counter = 0;
  assignDFSNumbers(static_cast<BlockIndex>(0), adaptor, block_layout);
}

template <IRAdaptor Adaptor>
void DominatorTree<Adaptor>::computeIDom(
    Adaptor* adaptor, const util::SmallVector<IRBlockRef, 64>& block_layout) noexcept {
  const u32 num_blocks = static_cast<u32>(block_layout.size());

  // Simple algorithm: iterate until convergence
  // For each block (except entry), set IDom to common dominator of all predecessors
  // Entry block (index 0) has no dominator
  idom[0] = static_cast<BlockIndex>(0);  // Entry dominates itself

  // Initialize IDom to the first predecessor
  for (u32 i = 1; i < num_blocks; ++i) {
    idom[i] = INVALID_BLOCK_IDX;
  }

  // Iterate until fixed point
  bool changed = true;
  while (changed) {
    changed = false;

    for (u32 i = 1; i < num_blocks; ++i) {
      const IRBlockRef block = block_layout[i];
      BlockIndex new_idom = INVALID_BLOCK_IDX;

      // Process all predecessors of this block
      bool first_pred = true;
      for (u32 j = 0; j < num_blocks; ++j) {
        const IRBlockRef pred_candidate = block_layout[j];
        
        // Check if pred_candidate is a predecessor of block
        bool is_pred = false;
        for (const auto succ : adaptor->block_succs(pred_candidate)) {
          if (succ == block) {
            is_pred = true;
            break;
          }
        }

        if (!is_pred) {
          continue;
        }

        const auto pred_idx = static_cast<BlockIndex>(j);

        if (first_pred) {
          new_idom = pred_idx;
          first_pred = false;
        } else {
          // Find common dominator: walk up from both until they meet
          BlockIndex a = new_idom;
          BlockIndex b = pred_idx;

          while (a != b) {
            const auto a_val = static_cast<u32>(a);
            const auto b_val = static_cast<u32>(b);
            
            // Walk up the one that's deeper (higher index in RPO = deeper in tree)
            // This is a simple heuristic; proper implementation would use tree levels
            if (a_val > b_val) {
              a = idom[a_val];
              if (a == INVALID_BLOCK_IDX) {
                a = static_cast<BlockIndex>(0);  // Reached top
              }
            } else {
              b = idom[b_val];
              if (b == INVALID_BLOCK_IDX) {
                b = static_cast<BlockIndex>(0);  // Reached top
              }
            }
          }
          new_idom = a;
        }
      }

      // Update if changed
      if (new_idom != idom[i]) {
        idom[i] = new_idom;
        changed = true;
      }
    }
  }
}

template <IRAdaptor Adaptor>
void DominatorTree<Adaptor>::assignDFSNumbers(
    BlockIndex node, Adaptor* adaptor,
    const util::SmallVector<IRBlockRef, 64>& block_layout) {
  const auto node_idx = static_cast<u32>(node);
  dfs_num_in[node_idx] = dfs_counter++;

  // Visit children in order
  for (const auto child : children[node_idx]) {
    assignDFSNumbers(child, adaptor, block_layout);
  }

  dfs_num_out[node_idx] = dfs_counter++;
}

template <IRAdaptor Adaptor>
void DominatorTree<Adaptor>::print(
    std::ostream& os, Adaptor* adaptor,
    const util::SmallVector<IRBlockRef, 64>& block_layout) const {
  if (block_layout.empty()) {
    return;
  }

  // Helper function to recursively print tree nodes
  const auto print_node = [&](auto& self, BlockIndex node, int indent) -> void {
    const auto node_idx = static_cast<u32>(node);
    const auto indent_str = std::string(indent * 2, ' ');

    if (node_idx < block_layout.size()) {
      if (node_idx == 0) {
        // Entry block (root)
        os << indent_str << adaptor->block_fmt_ref(block_layout[node_idx]) << " (root)\n";
      } else {
        // Non-root block - show immediate dominator
        const auto idom_idx = static_cast<u32>(idom[node_idx]);
        if (idom_idx < block_layout.size()) {
          os << indent_str << adaptor->block_fmt_ref(block_layout[node_idx]) << " (idom: "
             << adaptor->block_fmt_ref(block_layout[idom_idx]) << ")\n";
        }
      }
    }

    // Recursively print children
    for (const auto child : children[node_idx]) {
      self(self, child, indent + 1);
    }
  };

  // Start printing from root
  print_node(print_node, static_cast<BlockIndex>(0), 0);
}

}  // namespace tpde
