#!/usr/bin/env python3
import sys
from math import ceil


def build_tree(values, insts, counter):
    """
    Recursively reduce values pairwise into a single SSA value.
    Returns: (name, counter)
    """
    if len(values) == 1:
        return values[0], counter

    next_vals = []
    i = 0
    while i < len(values):
        if i + 1 < len(values):
            lhs = values[i]
            rhs = values[i + 1]
            name = f"%t{counter}"
            insts.append(f"  {name} = add i32 {lhs}, {rhs}")
            next_vals.append(name)
            counter += 1
            i += 2
        else:
            # Odd count → carry last value forward
            next_vals.append(values[i])
            i += 1

    return build_tree(next_vals, insts, counter)


def generate_llvm(n):
    # Define the list of input values as constants %v0, %v1 … (we use inline constants)
    values = [str(i) for i in range(n)]

    insts = []
    root, _ = build_tree(values, insts, 0)

    # Emit full module
    out = []
    out.append("; Auto-generated LLVM IR for summing i32s in a tree")
    out.append("define i32 @main() {")
    out.extend(insts)
    out.append(f"  ret i32 {root}")
    out.append("}")
    return "\n".join(out)


if __name__ == "__main__":
    n = int(10000)

    print(generate_llvm(n))
