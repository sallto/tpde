#!/usr/bin/env python3
"""Symbolic executor for verifying .vir files."""

import random
import re
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple


@dataclass
class Operand:
    """Represents a virtual register and its assembly register or stack slot."""

    vreg: str  # e.g., "v0"
    areg: Optional[str]  # e.g., "r7" or None for stack operands
    part: Optional[int] = None  # e.g., 1 for %v1:1@r2, None for single-part
    stack_offset: Optional[int] = None  # e.g., -48 for %v0@[sp+-48]


@dataclass
class Operation:
    uses: List[Operand]
    defs: List[Operand]


@dataclass
class RegMove:
    """A register move operation."""

    src_reg: str  # e.g., "r7"
    dst_reg: str  # e.g., "r6"
    vreg: Optional[str] = None  # e.g., "v0" or "v1:1", optional
    size: Optional[int] = None  # e.g., 4 for 4b


@dataclass
class PhiIncoming:
    from_block: str
    vreg: str
    areg: str  # May be r255 (invalid/placeholder)


@dataclass
class PhiNode:
    target: Operand
    incomings: List[PhiIncoming]


@dataclass
class SpillOp:
    src_reg: str  # e.g., "r7"
    stack_offset: int  # e.g., -44 for [sp+-44]
    vreg: str  # e.g., "v0"
    size: int  # e.g., 4 for 4bytes
    part: Optional[int] = None  # For multi-part registers


@dataclass
class ReloadOp:
    stack_offset: int  # e.g., -44 for [sp+-44]
    dst_reg: str  # e.g., "r6"
    vreg: str  # e.g., "v0"
    size: int  # e.g., 4 for 4b
    part: Optional[int] = None  # For multi-part registers


@dataclass
class Block:
    name: str
    operations: List[Operation]
    regmoves: List[RegMove]
    spill_ops: List[SpillOp]
    reload_ops: List[ReloadOp]
    phi_nodes: List[PhiNode]
    instructions: List[
        Tuple[str, object]
    ]  # List of ('op', Operation), ('regmove', RegMove), ('spill', SpillOp), ('reload', ReloadOp) in order
    jmp_target: Optional[str]  # For jmp
    jcond_target: Optional[str]  # For jcond
    jcond_uses: Optional[Operand]  # For jcond


@dataclass
class Edge:
    """An edge between blocks."""

    from_block: str
    to_block: str


@dataclass
class Function:
    """A function in the .vir file."""

    name: str
    blocks: Dict[str, Block]
    edges: List[Edge]
    vreg_to_number: Dict[str, int]
    used_numbers: Set[int]
    stack_memory: Dict[int, int]
    stack_memory_parts: Dict[Tuple[int, int], int]
    occupied_offsets: Dict[int, str]


class VirVerifier:
    """Symbolic executor for .vir files."""

    def __init__(self, input: str):
        self.input = input
        self.functions: List[Function] = []

    def parse(self):
        """Parse the .vir file."""
        lines = self.input.splitlines()
        lines = [line.strip() for line in lines if line.strip()]

        i = 0
        while i < len(lines):
            if lines[i].startswith("function"):
                function_name = lines[i].split("function", 1)[1].strip()
                i += 1

                blocks = {}
                edges = []
                vreg_to_number = {}
                used_numbers = set()
                stack_memory = {}
                stack_memory_parts = {}
                occupied_offsets = {}

                # Parse blocks for this function
                while i < len(lines) and not lines[i].startswith("function"):
                    if lines[i].startswith("block "):
                        block_name = lines[i].split("block", 1)[1].strip().rstrip(":")
                        i += 1

                        operations = []
                        regmoves = []
                        spill_ops = []
                        reload_ops = []
                        phi_nodes = []
                        instructions = []  # Store operations, regmoves, spills, and reloads in order
                        jmp_target = None
                        jcond_target = None
                        jcond_uses = None

                        while (
                            i < len(lines)
                            and not lines[i].startswith("block ")
                            and not lines[i].startswith("edge ")
                            and not lines[i].startswith("function")
                        ):
                            line = lines[i]

                            if line.startswith("op "):
                                op = self._parse_operation(line)
                                operations.append(op)
                                instructions.append(("op", op))
                            elif line.startswith("edit regmove "):
                                regmove = self._parse_regmove(line)
                                regmoves.append(regmove)
                                instructions.append(("regmove", regmove))
                            elif line.startswith("edit spill "):
                                spill = self._parse_spill(line)
                                spill_ops.append(spill)
                                instructions.append(("spill", spill))
                            elif line.startswith("edit reload "):
                                reload = self._parse_reload(line)
                                reload_ops.append(reload)
                                instructions.append(("reload", reload))
                            elif line.startswith("phi "):
                                phi = self._parse_phi(line)
                                phi_nodes.append(phi)
                            elif line.startswith("jmp "):
                                jmp_target = line.split("jmp", 1)[1].strip()
                            elif line.startswith("jcond "):
                                jcond_target, jcond_uses = self._parse_jcond(line)

                            i += 1

                        blocks[block_name] = Block(
                            name=block_name,
                            operations=operations,
                            regmoves=regmoves,
                            spill_ops=spill_ops,
                            reload_ops=reload_ops,
                            phi_nodes=phi_nodes,
                            instructions=instructions,
                            jmp_target=jmp_target,
                            jcond_target=jcond_target,
                            jcond_uses=jcond_uses,
                        )
                    elif lines[i].startswith("edge "):
                        edge = self._parse_edge(lines[i])
                        if edge:
                            edges.append(edge)
                        i += 1
                        # Skip parallel move line if present
                        if i < len(lines) and lines[i].startswith("  parallel "):
                            i += 1
                    else:
                        i += 1

                func = Function(
                    name=function_name,
                    blocks=blocks,
                    edges=edges,
                    vreg_to_number=vreg_to_number,
                    used_numbers=used_numbers,
                    stack_memory=stack_memory,
                    stack_memory_parts=stack_memory_parts,
                    occupied_offsets=occupied_offsets,
                )
                self.functions.append(func)
            else:
                i += 1

    def _parse_operand(self, s: str) -> Operand:
        """Parse an operand like %v0@r7, %v1:1@r2, or %v0@[sp+-48]."""
        # Try stack operand syntax first: %v0@[sp+-48]
        match = re.match(r"%v(\d+)(?::(\d+))?@\[sp\+([+-]?\d+)\]", s)
        if match:
            part = int(match.group(2)) if match.group(2) else None
            return Operand(
                vreg=f"v{match.group(1)}",
                areg=None,
                part=part,
                stack_offset=int(match.group(3)),
            )

        # Try multi-part syntax first: %v1:1@r2
        match = re.match(r"%v(\d+):(\d+)@r(\d+)", s)
        if match:
            return Operand(
                vreg=f"v{match.group(1)}",
                areg=f"r{match.group(3)}",
                part=int(match.group(2)),
            )

        # Fall back to single-part syntax: %v0@r7
        match = re.match(r"%v(\d+)@r(\d+)", s)
        if match:
            return Operand(vreg=f"v{match.group(1)}", areg=f"r{match.group(2)}")
        raise ValueError(f"Invalid operand: {s}")

    def _parse_operation(self, line: str) -> Operation:
        """Parse an operation line."""
        uses = []
        defs = []

        # Parse uses
        uses_match = re.search(r"uses=([^ ]+)", line)
        if uses_match:
            uses_str = uses_match.group(1)
            for op_str in uses_str.split(","):
                uses.append(self._parse_operand(op_str.strip()))

        # Parse defs
        defs_match = re.search(r"defs=([^ ]+)", line)
        if defs_match:
            defs_str = defs_match.group(1)
            for op_str in defs_str.split(","):
                defs.append(self._parse_operand(op_str.strip()))

        return Operation(uses=uses, defs=defs)

    def _parse_regmove(self, line: str) -> RegMove:
        """Parse a regmove line: edit regmove r7 -> r6 %v0 4b or edit regmove r7 -> r6 %v1:1 4b"""
        # Try syntax with size: edit regmove r7 -> r6 %v0 4b
        match = re.match(
            r"edit regmove r(\d+) -> r(\d+) %v(\d+)(?::(\d+))? (\d+)b", line
        )
        if match:
            vreg = f"v{match.group(3)}"
            if match.group(4):
                vreg += f":{match.group(4)}"
            return RegMove(
                src_reg=f"r{match.group(1)}",
                dst_reg=f"r{match.group(2)}",
                vreg=vreg,
                size=int(match.group(5)),
            )

        # Fall back to old syntax without size: edit regmove r7 -> r6 %v0 or edit regmove r7 -> r6 %v1:1
        match = re.match(r"edit regmove r(\d+) -> r(\d+) %v(\d+)(?::(\d+))?", line)
        if match:
            vreg = f"v{match.group(3)}"
            if match.group(4):
                vreg += f":{match.group(4)}"
            return RegMove(
                src_reg=f"r{match.group(1)}", dst_reg=f"r{match.group(2)}", vreg=vreg
            )

        # Fall back to single-part syntax without vreg: edit regmove r7 -> r6
        match = re.match(r"edit regmove r(\d+) -> r(\d+)", line)
        if match:
            return RegMove(
                src_reg=f"r{match.group(1)}",
                dst_reg=f"r{match.group(2)}",
            )
        raise ValueError(f"Invalid regmove: {line}")

    def _parse_spill(self, line: str) -> SpillOp:
        """Parse a spill line: edit spill r7 -> [sp+-44] %v0 4b or edit spill r7 -> [sp+-44] %v1:1 4b"""
        # Handle syntax with size: edit spill r7 -> [sp+-44] %v0 4b
        match = re.match(
            r"edit spill r(\d+) -> \[sp\+([+-]?\d+)\] %v(\d+)(?::(\d+))? (\d+)b", line
        )
        if match:
            vreg = f"v{match.group(3)}"
            part = None
            if match.group(4):
                vreg += f":{match.group(4)}"
                part = int(match.group(4))
            return SpillOp(
                src_reg=f"r{match.group(1)}",
                stack_offset=int(match.group(2)),
                vreg=vreg,
                size=int(match.group(5)),
                part=part,
            )

        # Fall back to old syntax without size: edit spill r7 -> [sp+-44] %v0 or edit spill r7 -> [sp+-44] %v1:1
        match = re.match(
            r"edit spill r(\d+) -> \[sp\+([+-]?\d+)\] %v(\d+)(?::(\d+))?", line
        )
        if match:
            vreg = f"v{match.group(3)}"
            part = None
            if match.group(4):
                vreg += f":{match.group(4)}"
                part = int(match.group(4))
            return SpillOp(
                src_reg=f"r{match.group(1)}",
                stack_offset=int(match.group(2)),
                vreg=vreg,
                size=4,  # Default size
                part=part,
            )
        raise ValueError(f"Invalid spill: {line}")

    def _parse_reload(self, line: str) -> ReloadOp:
        """Parse a reload line: edit reload [sp+-44] -> r6 %v0 4b or edit reload [sp+-44] -> r6 %v1:1 4b"""
        # Handle syntax with size: edit reload [sp+-44] -> r6 %v0 4b
        match = re.match(
            r"edit reload \[sp\+([+-]?\d+)\] -> r(\d+) %v(\d+)(?::(\d+))? (\d+)b", line
        )
        if match:
            vreg = f"v{match.group(3)}"
            part = None
            if match.group(4):
                vreg += f":{match.group(4)}"
                part = int(match.group(4))
            return ReloadOp(
                stack_offset=int(match.group(1)),
                dst_reg=f"r{match.group(2)}",
                vreg=vreg,
                size=int(match.group(5)),
                part=part,
            )

        # Fall back to old syntax without size: edit reload [sp+-44] -> r6 %v0 or edit reload [sp+-44] -> r6 %v1:1
        match = re.match(
            r"edit reload \[sp\+([+-]?\d+)\] -> r(\d+) %v(\d+)(?::(\d+))?", line
        )
        if match:
            vreg = f"v{match.group(3)}"
            part = None
            if match.group(4):
                vreg += f":{match.group(4)}"
                part = int(match.group(4))
            return ReloadOp(
                stack_offset=int(match.group(1)),
                dst_reg=f"r{match.group(2)}",
                vreg=vreg,
                size=4,  # Default size
                part=part,
            )
        raise ValueError(f"Invalid reload: {line}")

    def _parse_phi(self, line: str) -> PhiNode:
        """Parse a phi node: phi %v2@r6 [b0, %v0@r255, b2, %v5@r255] or phi %v4:1@r3 [b0, %v1:1@r255, b1, %v5:1@r255]"""
        # Extract target - handle both single-part and multi-part syntax
        target_match = re.match(r"phi (%v\d+(?::\d+)?@r\d+) \[", line)
        if not target_match:
            raise ValueError(f"Invalid phi: {line}")
        target = self._parse_operand(target_match.group(1))

        # Extract incomings
        incomings = []
        incoming_parts = re.findall(r"\[(.*)\]", line)
        if incoming_parts:
            parts = incoming_parts[0].split(",")
            i = 0
            while i < len(parts):
                from_block = parts[i].strip()
                i += 1
                if i < len(parts):
                    operand_str = parts[i].strip()
                    operand = self._parse_operand(operand_str)
                    incomings.append(
                        PhiIncoming(
                            from_block=from_block,
                            vreg=operand.vreg
                            + ("" if not operand.part else ":" + str(operand.part)),
                            areg=operand.areg,
                        )
                    )
                    i += 1

        return PhiNode(target=target, incomings=incomings)

    def _parse_jcond(self, line: str) -> Tuple[Optional[str], Optional[Operand]]:
        """Parse a jcond line: jcond b3 uses=%v3@r8 or jcond b3 uses=%v1:1@r8"""
        target_match = re.search(r"jcond (\w+)", line)
        uses_match = re.search(r"uses=(%v\d+(?::\d+)?@r\d+)", line)

        target = target_match.group(1) if target_match else None
        uses = self._parse_operand(uses_match.group(1)) if uses_match else None

        return target, uses

    def _parse_edge(self, line: str) -> Optional[Edge]:
        """Parse an edge line: edge b0 -> b1:"""
        match = re.match(r"edge (\w+) -> (\w+):", line)
        if match:
            return Edge(from_block=match.group(1), to_block=match.group(2))
        return None

    def _process_spill(
        self, spill: SpillOp, reg_state: Dict[str, int], block_name: str, func: Function
    ):
        """Process a spill operation."""

        # Check for overlaps with existing spills (but allow spills of the same vreg)
        spill_offsets = set(range(spill.stack_offset, spill.stack_offset + spill.size))
        overlapping = spill_offsets & set(func.occupied_offsets.keys())
        if overlapping and overlapping != spill_offsets:
            overlapping_vregs = {
                func.occupied_offsets[offset] for offset in overlapping
            }
            raise ValueError(
                f"Spill in {block_name} of function {func.name}: spill at offset {spill.stack_offset} "
                f"with size {spill.size} overlaps with existing spills at offsets {overlapping} "
                f"(occupied by {overlapping_vregs})"
            )

        # Get value from source register
        if spill.src_reg not in reg_state:
            raise ValueError(
                f"Spill in {block_name} of function {func.name}: source register {spill.src_reg} not in state"
            )

        src_value = reg_state[spill.src_reg]
        expected_vreg_num = func.vreg_to_number[spill.vreg]

        # Verify the source register contains the expected value
        if src_value != expected_vreg_num:
            raise ValueError(
                f"Spill in {block_name} of function {func.name}: register {spill.src_reg} contains "
                f"vreg number {src_value}, expected {expected_vreg_num} "
                f"(for {spill.vreg})"
            )

        # Store in stack memory
        if spill.part is not None:
            # Multi-part register
            func.stack_memory_parts[(spill.stack_offset, spill.part)] = src_value
        else:
            # Single-part register
            func.stack_memory[spill.stack_offset] = src_value

        # Mark offsets as occupied
        for offset in spill_offsets:
            func.occupied_offsets[offset] = spill.vreg

    def _process_reload(
        self,
        reload: ReloadOp,
        reg_state: Dict[str, int],
        block_name: str,
        func: Function,
    ):
        """Process a reload operation."""

        expected_vreg_num = func.vreg_to_number[reload.vreg]

        # Get value from stack memory
        if reload.part is not None:
            # Multi-part register
            stack_key = (reload.stack_offset, reload.part)
            if stack_key not in func.stack_memory_parts:
                raise ValueError(
                    f"Reload in {block_name} of function {func.name}: stack location [sp+{reload.stack_offset}] "
                    f"part {reload.part} not spilled"
                )
            stack_value = func.stack_memory_parts[stack_key]
        else:
            # Single-part register
            if reload.stack_offset not in func.stack_memory:
                raise ValueError(
                    f"Reload in {block_name} of function {func.name}: stack location [sp+{reload.stack_offset}] not spilled"
                )
            stack_value = func.stack_memory[reload.stack_offset]

        # Verify the stack contains the expected value
        if stack_value != expected_vreg_num:
            raise ValueError(
                f"Reload in {block_name} of function {func.name}: stack location [sp+{reload.stack_offset}] "
                f"contains vreg number {stack_value}, expected {expected_vreg_num} "
                f"(for {reload.vreg})"
            )

        # Load into destination register
        reg_state[reload.dst_reg] = expected_vreg_num

    def _assign_vreg_number(self, vreg: str, func: Function) -> int:
        """Assign a unique number to a virtual register."""
        if vreg in func.vreg_to_number:
            return func.vreg_to_number[vreg]

        # Try to use the number from vreg name (v0 -> 0, v1:1 -> 1)
        match = re.match(r"v(\d+)(?::\d+)?", vreg)
        if match:
            base_num = int(match.group(1))
            # For multi-part registers, use a unique number to avoid conflicts
            if ":" in vreg:
                # This is a multi-part register, use a unique number
                while True:
                    num = random.randint(
                        10000, 99999
                    )  # Use higher range for multi-part
                    if num not in func.used_numbers:
                        func.vreg_to_number[vreg] = num
                        func.used_numbers.add(num)
                        return num
            else:
                # This is a single-part register, use the base number if available
                if base_num not in func.used_numbers:
                    func.vreg_to_number[vreg] = base_num
                    func.used_numbers.add(base_num)
                    return base_num

        # Assign a random unique number
        while True:
            num = random.randint(
                1000, 9999
            )  # Use range that won't conflict with v0, v1, etc.
            if num not in func.used_numbers:
                func.vreg_to_number[vreg] = num
                func.used_numbers.add(num)
                return num

    def _collect_all_vregs(self, func: Function):
        """Collect all virtual registers from the IR for a function."""
        vregs = set()

        for block in func.blocks.values():
            for op in block.operations:
                for use in op.uses:
                    vregs.add(use.vreg)
                for def_ in op.defs:
                    vregs.add(def_.vreg)
            for spill in block.spill_ops:
                vregs.add(spill.vreg)
            for reload in block.reload_ops:
                vregs.add(reload.vreg)
            for phi in block.phi_nodes:
                vregs.add(phi.target.vreg)
                for inc in phi.incomings:
                    vregs.add(inc.vreg)

        for vreg in vregs:
            self._assign_vreg_number(vreg, func)

    def _verify_function(self, func: Function):
        """Verify a single function."""
        # Initialize worklist with entry block
        worklist = deque(
            [("b0", {}, {}, {}, {}, None)]
        )  # (block_name, register_state, stack_memory, stack_memory_parts, occupied_offsets, incoming_edge)
        visited_edges: Set[Tuple[str, str]] = set()

        predecessors: Dict[str, Set[str]] = {name: set() for name in func.blocks}
        for edge in func.edges:
            if edge.to_block in predecessors:
                predecessors[edge.to_block].add(edge.from_block)
        for block in func.blocks.values():
            if block.jcond_target:
                if block.jcond_target in predecessors:
                    predecessors[block.jcond_target].add(block.name)
                if block.jmp_target and block.jmp_target in predecessors:
                    predecessors[block.jmp_target].add(block.name)
            elif block.jmp_target and block.jmp_target in predecessors:
                predecessors[block.jmp_target].add(block.name)

        def _adjust_edge_for_split(
            block: Block,
            successor: str,
            incoming_edge: Optional[Tuple[str, str]],
        ) -> Tuple[str, str]:
            if not block.name.startswith("split_"):
                return (block.name, successor)
            if not incoming_edge:
                raise ValueError(
                    f"Split block {block.name} in function {func.name}: missing incoming edge"
                )

            split_preds = predecessors.get(block.name, set())
            if len(split_preds) != 1:
                raise ValueError(
                    f"Split block {block.name} in function {func.name}: expected 1 predecessor, "
                    f"found {len(split_preds)}"
                )
            original_pred = next(iter(split_preds))
            if original_pred != incoming_edge[0]:
                raise ValueError(
                    f"Split block {block.name} in function {func.name}: predecessor {original_pred} "
                    f"does not match incoming edge from {incoming_edge[0]}"
                )

            if successor not in func.blocks:
                raise ValueError(
                    f"Split block {block.name} in function {func.name}: successor {successor} not found"
                )
            successor_block = func.blocks[successor]
            for phi in successor_block.phi_nodes:
                incoming_blocks = {inc.from_block for inc in phi.incomings}
                if original_pred not in incoming_blocks:
                    raise ValueError(
                        f"Split block {block.name} in function {func.name}: predecessor {original_pred} "
                        f"not listed in phi for block {successor}"
                    )

            return (original_pred, successor)

        while worklist:
            (
                block_name,
                reg_state,
                stack_memory,
                stack_memory_parts,
                occupied_offsets,
                incoming_edge,
            ) = worklist.popleft()

            if block_name not in func.blocks:
                raise ValueError(
                    f"Block {block_name} in function {func.name} not found"
                )

            block = func.blocks[block_name]

            # Mark edge as visited
            if incoming_edge:
                visited_edges.add(incoming_edge)

            # Restore stack state for this path
            func.stack_memory = stack_memory.copy()
            func.stack_memory_parts = stack_memory_parts.copy()
            func.occupied_offsets = occupied_offsets.copy()

            # Process phi nodes first (they happen at block entry)
            for phi in block.phi_nodes:
                if incoming_edge:
                    from_block = incoming_edge[0]
                    # Find the incoming value for this predecessor
                    incoming = None
                    for inc in phi.incomings:
                        if inc.from_block == from_block:
                            incoming = inc
                            break

                    if not incoming:
                        # Skip phi check for predecessors not listed in phi incomings
                        continue

                    # Check that the target register contains the incoming virtual register's number
                    # (The parallel moves should have moved the incoming value to the target register)
                    incoming_vreg_num = func.vreg_to_number[incoming.vreg]
                    if phi.target.areg not in reg_state:
                        raise ValueError(
                            f"Phi node {phi.target.vreg}@{phi.target.areg} in {block_name} of function {func.name}: "
                            f"target register {phi.target.areg} not in state"
                        )
                    actual_value = reg_state[phi.target.areg]
                    # special case for values without vallocalidx, we can only check that there exists a value not that its the correct one
                    if actual_value != incoming_vreg_num and not (
                        incoming_vreg_num > 2147483660 and actual_value > 214748366
                    ):
                        raise ValueError(
                            f"Phi node {phi.target.vreg}@{phi.target.areg} in {block_name} of function {func.name} from {from_block}: "
                            f"register {phi.target.areg} contains {actual_value}, "
                            f"expected {incoming_vreg_num} (from {incoming.vreg})"
                        )

                    # After phi resolution, the register should contain the target vreg's number
                    target_vreg_num = func.vreg_to_number[phi.target.vreg]
                    reg_state[phi.target.areg] = target_vreg_num

            # Process instructions in order (operations and regmoves interleaved)
            for inst_type, inst in block.instructions:
                if inst_type == "op":
                    op = inst
                    # Check uses
                    for use in op.uses:
                        expected_vreg_num = func.vreg_to_number[use.vreg]
                        if use.stack_offset is not None:
                            if use.part is not None:
                                stack_key = (use.stack_offset, use.part)
                                if stack_key not in func.stack_memory_parts:
                                    raise ValueError(
                                        f"Operation in {block_name} of function {func.name}: stack location [sp+{use.stack_offset}] "
                                        f"part {use.part} not spilled"
                                    )
                                actual_vreg_num = func.stack_memory_parts[stack_key]
                            else:
                                if use.stack_offset not in func.stack_memory:
                                    raise ValueError(
                                        f"Operation in {block_name} of function {func.name}: stack location [sp+{use.stack_offset}] not spilled"
                                    )
                                actual_vreg_num = func.stack_memory[use.stack_offset]
                            if actual_vreg_num != expected_vreg_num:
                                raise ValueError(
                                    f"Operation in {block_name} of function {func.name}: stack location [sp+{use.stack_offset}] contains "
                                    f"vreg number {actual_vreg_num}, expected {expected_vreg_num} "
                                    f"(for {use.vreg})"
                                )
                            continue
                        if use.areg not in reg_state:
                            raise ValueError(
                                f"Operation in {block_name} of function {func.name}: register {use.areg} not in state "
                                f"(expected {use.vreg} with number {expected_vreg_num})"
                            )
                        actual_vreg_num = reg_state[use.areg]
                        if actual_vreg_num != expected_vreg_num:
                            raise ValueError(
                                f"Operation in {block_name} of function {func.name}: register {use.areg} contains "
                                f"vreg number {actual_vreg_num}, expected {expected_vreg_num} "
                                f"(for {use.vreg})"
                            )

                    # Apply defs
                    for def_ in op.defs:
                        vreg_num = func.vreg_to_number[def_.vreg]
                        reg_state[def_.areg] = vreg_num

                elif inst_type == "regmove":
                    regmove = inst
                    # Get value from source register
                    if regmove.src_reg not in reg_state:
                        raise ValueError(
                            f"Regmove in {block_name} of function {func.name}: source register {regmove.src_reg} not in state"
                        )
                    vreg_num = reg_state[regmove.src_reg]

                    # Write to target register (overwrite) - this establishes that dst now contains the annotated vreg
                    reg_state[regmove.dst_reg] = vreg_num

                elif inst_type == "spill":
                    spill = inst
                    self._process_spill(spill, reg_state, block_name, func)

                elif inst_type == "reload":
                    reload = inst
                    self._process_reload(reload, reg_state, block_name, func)

            # Process jumps
            new_reg_state = reg_state.copy()
            new_stack_memory = func.stack_memory.copy()
            new_stack_memory_parts = func.stack_memory_parts.copy()
            new_occupied_offsets = func.occupied_offsets.copy()

            if block.jmp_target:
                # Unconditional jump
                edge = _adjust_edge_for_split(block, block.jmp_target, incoming_edge)
                if edge not in visited_edges:
                    worklist.append(
                        (
                            block.jmp_target,
                            new_reg_state,
                            new_stack_memory,
                            new_stack_memory_parts,
                            new_occupied_offsets,
                            edge,
                        )
                    )

            if block.jcond_target:
                # Conditional jump - enqueue both targets
                # True branch (jcond target)
                edge_true = _adjust_edge_for_split(
                    block, block.jcond_target, incoming_edge
                )
                if edge_true not in visited_edges:
                    worklist.append(
                        (
                            block.jcond_target,
                            new_reg_state.copy(),
                            new_stack_memory.copy(),
                            new_stack_memory_parts.copy(),
                            new_occupied_offsets.copy(),
                            edge_true,
                        )
                    )

                # False branch (fall through to jmp_target or implicit exit)
                if block.jmp_target:
                    edge_false = _adjust_edge_for_split(
                        block, block.jmp_target, incoming_edge
                    )
                    if edge_false not in visited_edges:
                        worklist.append(
                            (
                                block.jmp_target,
                                new_reg_state.copy(),
                                new_stack_memory.copy(),
                                new_stack_memory_parts.copy(),
                                new_occupied_offsets.copy(),
                                edge_false,
                            )
                        )

        # Verify all edges were visited
        expected_edges = set((e.from_block, e.to_block) for e in func.edges)
        # Also add implicit edges from jumps
        for block in func.blocks.values():
            if block.jcond_target:
                # Conditional jump creates edge to jcond target
                expected_edges.add((block.name, block.jcond_target))
                # Fall-through edge (jmp after jcond, or implicit exit)
                if block.jmp_target:
                    expected_edges.add((block.name, block.jmp_target))
            elif block.jmp_target:
                # Unconditional jump
                expected_edges.add((block.name, block.jmp_target))

        # Treat split blocks as transparent for edge expectations
        adjusted_expected_edges: Set[Tuple[str, str]] = set()
        for from_block, to_block in expected_edges:
            if from_block.startswith("split_"):
                split_preds = predecessors.get(from_block, set())
                if len(split_preds) != 1:
                    raise ValueError(
                        f"Split block {from_block} in function {func.name}: expected 1 predecessor, "
                        f"found {len(split_preds)}"
                    )
                original_pred = next(iter(split_preds))
                adjusted_expected_edges.add((original_pred, to_block))
            else:
                adjusted_expected_edges.add((from_block, to_block))

        unvisited = adjusted_expected_edges - visited_edges
        if unvisited:
            raise ValueError(
                f"Not all edges were visited in function {func.name}: {unvisited}"
            )

    def verify(self, quiet: bool = False):
        """Verify the .vir file."""
        self.parse()

        all_passed = True
        for func in self.functions:
            self._collect_all_vregs(func)
            try:
                self._verify_function(func)
                if not quiet:
                    print(f"Function {func.name} is valid")
            except Exception as e:
                if not quiet:
                    print(f"Function {func.name} verification failed: {e}")
                all_passed = False
                continue

        return all_passed


def main():
    import sys

    quiet = False
    args = []
    for arg in sys.argv[1:]:
        if arg == "-q":
            quiet = True
        else:
            args.append(arg)
    if len(args) > 1:
        print(f"Usage: {sys.argv[0]} [-q] <file.vir>/<stdin>")
        sys.exit(1)
    if len(args) == 0:
        input = sys.stdin.read()
    else:
        with open(args[0], "r") as f:
            input = f.read()
    verifier = VirVerifier(input)
    try:
        all_passed = verifier.verify(quiet=quiet)
        if all_passed:
            return 0
        # don't exit one normally as FileCheck expect return code 0
        if quiet:
            exit(1)
    except Exception as e:
        raise


if __name__ == "__main__":
    main()
