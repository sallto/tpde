# SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
#
# SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception

# see https://sourceware.org/gdb/current/onlinedocs/gdb.html/Writing-a-Pretty_002dPrinter.html

import gdb

_X64_REG_NAMES = {
    0: "rax",
    1: "rcx",
    2: "rdx",
    3: "rbx",
    4: "rsp",
    5: "rbp",
    6: "rsi",
    7: "rdi",
    8: "r8",
    9: "r9",
    10: "r10",
    11: "r11",
    12: "r12",
    13: "r13",
    14: "r14",
    15: "r15",
    32: "xmm0",
    33: "xmm1",
    34: "xmm2",
    35: "xmm3",
    36: "xmm4",
    37: "xmm5",
    38: "xmm6",
    39: "xmm7",
    40: "xmm8",
    41: "xmm9",
    42: "xmm10",
    43: "xmm11",
    44: "xmm12",
    45: "xmm13",
    46: "xmm14",
    47: "xmm15",
}


class EnumClassPrettyPrinter(gdb.ValuePrinter):
    def __init__(self, val):
        self.__val = val

    def to_string(self):
        return str(self.__val.cast(gdb.lookup_type("uint32_t")))


def _reg_id(val):
    try:
        return int(val["reg_id"])
    except gdb.error:
        reg_type = gdb.lookup_type("tpde::Reg")
        return int(val.cast(reg_type)["reg_id"])


def _format_generic_reg(reg_id):
    if reg_id == 0xFF:
        return "<invalid>"
    return f"r{reg_id}"


def _format_x64_reg(reg_id):
    if reg_id == 0xFF:
        return "<invalid>"
    return _X64_REG_NAMES.get(reg_id, f"r{reg_id}")


def _format_a64_reg(reg_id):
    if reg_id == 0xFF:
        return "<invalid>"
    if 0 <= reg_id <= 28:
        return f"x{reg_id}"
    if reg_id == 29:
        return "fp"
    if reg_id == 30:
        return "lr"
    if reg_id == 31:
        return "sp"
    if 32 <= reg_id <= 63:
        return f"v{reg_id - 32}"
    return f"r{reg_id}"


class RegPrettyPrinter(gdb.ValuePrinter):
    def __init__(self, val):
        self.__val = val

    def to_string(self):
        return _format_generic_reg(_reg_id(self.__val))

    def children(self):
        yield "reg_id", self.__val["reg_id"]


class AsmRegPrettyPrinter(gdb.ValuePrinter):
    def __init__(self, val):
        self.__val = val

    def to_string(self):
        reg_id = _reg_id(self.__val)
        type_name = self.__val.type.strip_typedefs().tag
        if not type_name:
            type_name = str(self.__val.type.strip_typedefs())
        if "x64" in type_name:
            return _format_x64_reg(reg_id)
        if "a64" in type_name:
            return _format_a64_reg(reg_id)
        return _format_generic_reg(reg_id)


def register_tpde_printers():
    from gdb import printing

    pp = gdb.printing.RegexpCollectionPrettyPrinter("tpde")
    pp.add_printer("reg", "^tpde::Reg$", RegPrettyPrinter)
    pp.add_printer("asmReg", "^tpde::.*::AsmReg$", AsmRegPrettyPrinter)
    pp.add_printer(
        "blockIndex", "^tpde::Analyzer<.*>::BlockIndex", EnumClassPrettyPrinter
    )
    pp.add_printer(
        "valLocalIndex", "^tpde::CompilerBase<.*>::ValLocalIdx", EnumClassPrettyPrinter
    )
    pp.add_printer(
        "testValueRef", "^tpde::test::TestIRAdaptor::IRValueRef", EnumClassPrettyPrinter
    )
    pp.add_printer(
        "testBlockRef", "^tpde::test::TestIRAdaptor::IRBlockRef", EnumClassPrettyPrinter
    )
    pp.add_printer(
        "testFuncRef", "^tpde::test::TestIRAdaptor::IRFuncRef", EnumClassPrettyPrinter
    )
    gdb.printing.register_pretty_printer(gdb.current_objfile(), pp)
