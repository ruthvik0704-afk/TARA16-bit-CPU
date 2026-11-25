#!/usr/bin/env python3
"""
microcode_gen.py - control ROM generator for the TARA CPU (TH6, group G32).

Builds the 2048-word control ROM from a symbolic description of each
instruction, runs the consistency checks required by the assignment, and
writes:

    control_rom.hex      Logisim image ("v2.0 raw"), one 32-bit word per address
    microcode_gen.log    field layout, microprogram listing and check results

Usage:
    python microcode_gen.py
    python microcode_gen.py -o control_rom.hex --log microcode_gen.log
    python microcode_gen.py --verify G32_th6_tara.circ

--verify also compares the finished image with the ROM stored in a circuit file.

ROM address (11 bits):   { Z, N, OPCODE[4:0], TSTATE[3:0] }
    bit 10 = Z, bit 9 = N   (latched outputs of status_reg)
    bits 8-4 = OPCODE       (from tara_decoder)
    bits 3-0 = TSTATE       (from the T-state counter)
"""

import argparse
import sys
import xml.etree.ElementTree as ET

# --------------------------------------------------------------------------
# 1. Control word layout: (name, least significant bit, width)
# --------------------------------------------------------------------------
ROM_WIDTH = 32
FIELDS = [
    ("BUS_SOURCE",  0, 3),   # which source drives the bus (encoded, so only one)
    ("PC_CTRL",     4, 2),   # 00 hold, 01 PC+2, 10 branch target, 11 bus
    ("MAR_LOAD",    7, 1),
    ("MDR_LOAD",    8, 1),
    ("IR_LOAD",     9, 1),
    ("ACC_LOAD",   10, 1),
    ("RF_WRITE",   11, 1),
    ("MEM_READ",   12, 1),
    ("MEM_WRITE",  13, 1),
    ("ALU_SELECT", 14, 5),   # the ALU uses TARA opcode numbers
    ("STATUS_WE",  19, 1),
    ("STEP_RESET", 20, 1),
    ("HALT",       21, 1),
    ("ALU_SRC_IMM", 22, 1),  # ALU B input: 0 = register RB, 1 = immediate
    ("IMM_SEL",    23, 1),   # immediate to the ALU: 0 = imm8, 1 = off5
    ("RA_SEL",     24, 2),   # port A index: default, RD field, R6, R7
    ("WR_SEL",     26, 2),   # write index: RD field, R6, R7
    ("B_CONST",    28, 1),   # ALU B input takes the constant below
    ("CONST_SEL",  29, 1),   # constant: 0 = 0x0000, 1 = 0x0002
    ("BYTE_SEL",   30, 1),   # byte lane select and byte write-enable in ram_word
]
# Bits 3, 6 and 31 are unused. rel8 or rel11 is chosen in hardware by OPCODE[1].
FIELD = {name: (lsb, width) for name, lsb, width in FIELDS}
USED_MASK = 0
for _name, _lsb, _width in FIELDS:
    USED_MASK |= ((1 << _width) - 1) << _lsb

# --------------------------------------------------------------------------
# 2. Symbolic values for the multi-bit fields
# --------------------------------------------------------------------------
BUS_NONE, BUS_PC, BUS_MDR, BUS_MEM, BUS_QA, BUS_QB, BUS_ACC, BUS_IMM = range(8)
BUS_NAMES = ["none", "PC", "MDR", "MEM", "QA", "QB", "ACC", "IMM"]

PC_HOLD, PC_INC2, PC_TARGET, PC_BUS = range(4)
PC_NAMES = ["hold", "+2", "<-target", "<-bus"]

RA_DEFAULT, RA_RD, RA_R6, RA_R7 = range(4)   # RA_DEFAULT: RD for format F3, else RA
RA_NAMES = ["default", "RD", "R6", "R7"]

WR_RD, WR_R6, WR_R7 = range(3)
WR_NAMES = ["RD", "R6", "R7", "invalid"]

# Opcodes (from the TARA ISA table)
NOP, HLT, MOV, LIL, LIH = 0, 1, 2, 3, 4
LDW, STW, LDB, STB = 5, 6, 7, 8
ADD, SUB, ADDI = 9, 10, 11
AND, OR, XOR, NOT, SHL, SHR, SLT = 13, 14, 15, 16, 17, 18, 19
BZ, BN, JMP, CALL, RET, PUSH, POP = 20, 21, 22, 23, 24, 25, 26
MNEMONIC = {
    NOP: "NOP", HLT: "HLT", MOV: "MOV", LIL: "LIL", LIH: "LIH",
    LDW: "LDW", STW: "STW", LDB: "LDB", STB: "STB",
    ADD: "ADD", SUB: "SUB", ADDI: "ADDI", AND: "AND", OR: "OR",
    XOR: "XOR", NOT: "NOT", SHL: "SHL", SHR: "SHR", SLT: "SLT",
    BZ: "BZ", BN: "BN", JMP: "JMP", CALL: "CALL", RET: "RET",
    PUSH: "PUSH", POP: "POP",
}
# ALU_SELECT values that the ALU actually implements (LIH = 4 merges bytes)
ALU_VALID = {LIH, ADD, SUB, AND, OR, XOR, NOT, SHL, SHR, SLT}

# Which condition bit an opcode's words are allowed to depend on.
CONDITIONAL = {BZ: "Z", BN: "N"}

N_TSTATES = 16
N_OPCODES = 32
FETCH_STATES = 3          # T0, T1, T2; execution starts at T3


# --------------------------------------------------------------------------
# 3. Packing and unpacking
# --------------------------------------------------------------------------
def word(**fields):
    """Pack named fields into one control word. Unnamed fields are 0."""
    value = 0
    for name, v in fields.items():
        if name not in FIELD:
            raise ValueError("unknown control field: %s" % name)
        lsb, width = FIELD[name]
        if not 0 <= v < (1 << width):
            raise ValueError("%s = %d does not fit in %d bit(s)" % (name, v, width))
        value |= v << lsb
    return value


def get(w, name):
    lsb, width = FIELD[name]
    return (w >> lsb) & ((1 << width) - 1)


def address(z, n, opcode, tstate):
    return (z << 10) | (n << 9) | (opcode << 4) | tstate


def describe(w):
    """Human-readable form of a control word, for the listing."""
    parts = []
    if get(w, "BUS_SOURCE"):
        parts.append("bus=" + BUS_NAMES[get(w, "BUS_SOURCE")])
    if get(w, "PC_CTRL"):
        parts.append("PC" + PC_NAMES[get(w, "PC_CTRL")])
    if get(w, "ALU_SELECT"):
        parts.append("ALU=" + MNEMONIC.get(get(w, "ALU_SELECT"), str(get(w, "ALU_SELECT"))))
    if get(w, "RA_SEL"):
        parts.append("portA=" + RA_NAMES[get(w, "RA_SEL")])
    if get(w, "WR_SEL"):
        parts.append("write=" + WR_NAMES[get(w, "WR_SEL")])
    for name, _lsb, width in FIELDS:
        if width == 1 and get(w, name):
            parts.append(name)
    return ", ".join(parts) if parts else "(nothing)"


# --------------------------------------------------------------------------
# 4. The microprogram
# --------------------------------------------------------------------------
# Safe word for every unspecified address: return to T0, change nothing.
SAFE = word(STEP_RESET=1)

# Common fetch, placed at T0-T2 of every opcode and every Z/N combination,
# because during fetch the IR still holds the previous instruction.
FETCH = [
    word(BUS_SOURCE=BUS_PC, MAR_LOAD=1),                               # T0: MAR <- PC
    word(BUS_SOURCE=BUS_MEM, MEM_READ=1, MDR_LOAD=1, PC_CTRL=PC_INC2),  # T1: MDR <- Mem[MAR]; PC <- PC+2
    word(BUS_SOURCE=BUS_MDR, IR_LOAD=1),                               # T2: IR <- MDR
]

# Write the accumulator into the destination register and finish.
WRITE_BACK = word(BUS_SOURCE=BUS_ACC, RF_WRITE=1, STEP_RESET=1)


def alu_reg(op):
    """rd <- ra (op) rb, through ACC. Flags are latched as a debugging aid."""
    return [word(ALU_SELECT=op, ACC_LOAD=1, STATUS_WE=1), WRITE_BACK]


def alu_imm(op, status=1):
    """rd <- rd (op) imm8, through ACC. The datapath reads RD on port A for F3."""
    return [word(ALU_SELECT=op, ALU_SRC_IMM=1, ACC_LOAD=1, STATUS_WE=status), WRITE_BACK]


# Effective address for F4: ACC <- Rb + sign-extended off5, then MAR <- ACC.
EA = [
    word(ALU_SELECT=ADD, ALU_SRC_IMM=1, IMM_SEL=1, ACC_LOAD=1),
    word(BUS_SOURCE=BUS_ACC, MAR_LOAD=1),
]


def load(byte):
    """Rd <- Mem[EA]. For a byte, ram_word selects the lane and zero-extends."""
    return EA + [word(BUS_SOURCE=BUS_MEM, MEM_READ=1, MDR_LOAD=1, BYTE_SEL=byte),
                 word(BUS_SOURCE=BUS_MDR, RF_WRITE=1, STEP_RESET=1)]


def store(byte):
    """Mem[EA] <- Rs. Rs is in the RD field and drives the bus through port A."""
    return EA + [word(BUS_SOURCE=BUS_QA, RA_SEL=RA_RD, MEM_WRITE=1, BYTE_SEL=byte, STEP_RESET=1)]


# BZ and BN: flags from Rt at T3 (Rt + 0, result discarded), then the latched
# condition selects one of these two words at T4 through the ROM address.
FLAGS_FROM_RT = word(RA_SEL=RA_RD, ALU_SELECT=ADD, B_CONST=1, STATUS_WE=1)
TAKEN = word(PC_CTRL=PC_TARGET, STEP_RESET=1)
NOT_TAKEN = SAFE

# Execute states, starting at T3, in the order the assignment lists them.
EXECUTE = {
    NOP:  [word(STEP_RESET=1)],
    HLT:  [word(HALT=1)],
    MOV:  [word(BUS_SOURCE=BUS_QA, RF_WRITE=1, STEP_RESET=1)],           # rd <- rs
    LIL:  [word(BUS_SOURCE=BUS_IMM, RF_WRITE=1, STEP_RESET=1)],          # rd <- zero-extended imm8
    LIH:  alu_imm(LIH, status=0),                                        # rd[15:8] <- imm8, low byte kept
    ADD:  alu_reg(ADD),
    SUB:  alu_reg(SUB),
    ADDI: alu_imm(ADD),                                                  # rd <- rd + sign-extended imm8
    AND:  alu_reg(AND),
    OR:   alu_reg(OR),
    XOR:  alu_reg(XOR),
    NOT:  alu_reg(NOT),                                                  # rd <- ~rs (rs on port A)
    SHL:  alu_imm(SHL),                                                  # rd <- rd << imm[4:0]
    SHR:  alu_imm(SHR),                                                  # rd <- rd >> imm[4:0]
    SLT:  alu_reg(SLT),
    LDW:  load(0),
    STW:  store(0),
    LDB:  load(1),
    STB:  store(1),
    BZ:   [FLAGS_FROM_RT, NOT_TAKEN],                                    # T4 is overridden in build_rom
    BN:   [FLAGS_FROM_RT, NOT_TAKEN],                                    # T4 is overridden in build_rom
    JMP:  [word(PC_CTRL=PC_TARGET, STEP_RESET=1)],
    # R6 and PC load on the same edge, and the bus still carries the old PC (PC_next).
    CALL: [word(BUS_SOURCE=BUS_PC, RF_WRITE=1, WR_SEL=WR_R6, PC_CTRL=PC_TARGET, STEP_RESET=1)],
    RET:  [word(BUS_SOURCE=BUS_QA, RA_SEL=RA_R6, PC_CTRL=PC_BUS, STEP_RESET=1)],
    PUSH: [word(RA_SEL=RA_R7, ALU_SELECT=SUB, B_CONST=1, CONST_SEL=1, ACC_LOAD=1),   # ACC <- R7 - 2
           word(BUS_SOURCE=BUS_ACC, MAR_LOAD=1, RF_WRITE=1, WR_SEL=WR_R7),           # R7, MAR <- ACC
           word(BUS_SOURCE=BUS_QA, RA_SEL=RA_RD, MEM_WRITE=1, STEP_RESET=1)],        # Mem[R7] <- Rs
    POP:  [word(BUS_SOURCE=BUS_QA, RA_SEL=RA_R7, MAR_LOAD=1),                        # MAR <- R7
           word(BUS_SOURCE=BUS_MEM, MEM_READ=1, RF_WRITE=1),                         # Rd <- Mem[MAR]
           word(RA_SEL=RA_R7, ALU_SELECT=ADD, B_CONST=1, CONST_SEL=1, ACC_LOAD=1),   # ACC <- R7 + 2
           word(BUS_SOURCE=BUS_ACC, RF_WRITE=1, WR_SEL=WR_R7, STEP_RESET=1)],        # R7 <- ACC
}
IMPLEMENTED = sorted(EXECUTE)


def build_rom():
    rom = [SAFE] * (4 * N_OPCODES * N_TSTATES)
    for z in (0, 1):
        for n in (0, 1):
            for opcode in range(N_OPCODES):
                for t, w in enumerate(FETCH):
                    rom[address(z, n, opcode, t)] = w
                for i, w in enumerate(EXECUTE.get(opcode, [])):
                    rom[address(z, n, opcode, FETCH_STATES + i)] = w
            # The second execute state of a branch depends on the latched condition.
            rom[address(z, n, BZ, FETCH_STATES + 1)] = TAKEN if z else NOT_TAKEN
            rom[address(z, n, BN, FETCH_STATES + 1)] = TAKEN if n else NOT_TAKEN
    return rom


# --------------------------------------------------------------------------
# 5. Consistency checks (run on the finished ROM image, not on the tables)
# --------------------------------------------------------------------------
def where(addr):
    z, n = (addr >> 10) & 1, (addr >> 9) & 1
    opcode, t = (addr >> 4) & 31, addr & 15
    return "addr 0x%03X (Z=%d N=%d op=%d %s T%d)" % (
        addr, z, n, opcode, MNEMONIC.get(opcode, "undefined"), t)


def check_one_bus_source(rom):
    """No control word selects more than one bus source."""
    errors = []
    for a, w in enumerate(rom):
        if w & ~USED_MASK:
            errors.append("%s sets unused bits: 0x%08X" % (where(a), w))
    note = ("BUS_SOURCE is a 3-bit encoded field decoded to one-hot enables, so two "
            "sources cannot be selected; all codes are valid and unused bits are 0")
    return errors, note


def check_terminates(rom):
    """Every implemented instruction eventually asserts STEP_RESET or HALT."""
    errors, longest = [], 0
    for opcode in IMPLEMENTED:
        for zn in range(4):
            z, n = zn >> 1, zn & 1
            for t in range(N_TSTATES):
                w = rom[address(z, n, opcode, t)]
                if get(w, "STEP_RESET") or get(w, "HALT"):
                    longest = max(longest, t + 1)
                    break
            else:
                errors.append("%s never asserts STEP_RESET or HALT (Z=%d N=%d)"
                              % (MNEMONIC[opcode], z, n))
    return errors, "%d instructions checked; longest takes %d states" % (len(IMPLEMENTED), longest)


def check_mem_exclusive(rom):
    """MEM_READ and MEM_WRITE are never asserted simultaneously."""
    errors = [where(a) + " asserts MEM_READ and MEM_WRITE"
              for a, w in enumerate(rom) if get(w, "MEM_READ") and get(w, "MEM_WRITE")]
    return errors, "%d words checked" % len(rom)


def check_valid_source(rom):
    """Write enables are never asserted without valid source data."""
    errors = []
    for a, w in enumerate(rom):
        bus = get(w, "BUS_SOURCE")
        # ram_word.DIN is on the bus, so MEM_WRITE needs a bus source too.
        for name in ("MAR_LOAD", "MDR_LOAD", "IR_LOAD", "RF_WRITE", "MEM_WRITE"):
            if get(w, name) and bus == BUS_NONE:
                errors.append("%s asserts %s with nothing driving the bus" % (where(a), name))
        if get(w, "PC_CTRL") == PC_BUS and bus == BUS_NONE:
            errors.append("%s loads PC from the bus with nothing driving it" % where(a))
        if bus == BUS_MEM and not get(w, "MEM_READ"):
            errors.append("%s puts memory on the bus without MEM_READ" % where(a))
        if get(w, "MEM_READ") and bus != BUS_MEM:
            errors.append("%s asserts MEM_READ but memory is not the bus source" % where(a))
        if get(w, "MEM_WRITE") and bus == BUS_MEM:
            errors.append("%s writes memory from memory" % where(a))
        if get(w, "ACC_LOAD") and get(w, "ALU_SELECT") not in ALU_VALID:
            errors.append("%s loads ACC from an unimplemented ALU operation" % where(a))
        if get(w, "STATUS_WE") and get(w, "ALU_SELECT") not in ALU_VALID:
            errors.append("%s latches flags from an unimplemented ALU operation" % where(a))
        if get(w, "WR_SEL") == 3:
            errors.append("%s uses the undefined WR_SEL code" % where(a))
        if get(w, "BYTE_SEL") and not (get(w, "MEM_READ") or get(w, "MEM_WRITE")):
            errors.append("%s asserts BYTE_SEL without a memory access" % where(a))
        if get(w, "HALT") and w != word(HALT=1):
            errors.append("%s asserts HALT together with other signals" % where(a))
    return errors, "bus-driven loads, memory, PC, ACC, flags, WR_SEL, BYTE_SEL and HALT words checked"


def check_conditions(rom):
    """Conditional-branch words differ only in the relevant condition signal."""
    errors = []
    for opcode in range(N_OPCODES):
        depends = CONDITIONAL.get(opcode)          # None, "Z" or "N"
        for t in range(N_TSTATES):
            w = {(z, n): rom[address(z, n, opcode, t)] for z in (0, 1) for n in (0, 1)}
            if depends != "Z" and (w[0, 0] != w[1, 0] or w[0, 1] != w[1, 1]):
                errors.append("op=%d T%d depends on Z but must not" % (opcode, t))
            if depends != "N" and (w[0, 0] != w[0, 1] or w[1, 0] != w[1, 1]):
                errors.append("op=%d T%d depends on N but must not" % (opcode, t))
    # A branch must latch the condition first, and only PC_CTRL may then differ.
    pc_mask = ((1 << FIELD["PC_CTRL"][1]) - 1) << FIELD["PC_CTRL"][0]
    for opcode, flag in CONDITIONAL.items():
        first = rom[address(0, 0, opcode, FETCH_STATES)]
        if not get(first, "STATUS_WE"):
            errors.append("%s does not assert STATUS_WE at its first execute state" % MNEMONIC[opcode])
        for t in range(N_TSTATES):
            off = rom[address(0, 0, opcode, t)]
            on = rom[address(*((1, 0) if flag == "Z" else (0, 1)), opcode, t)]
            if (off ^ on) & ~pc_mask:
                errors.append("%s T%d taken and not-taken words differ outside PC_CTRL"
                              % (MNEMONIC[opcode], t))
            if t == FETCH_STATES + 1 and (get(on, "PC_CTRL") != PC_TARGET or get(off, "PC_CTRL") != PC_HOLD):
                errors.append("%s T%d does not branch on its condition" % (MNEMONIC[opcode], t))
    note = ("conditional opcodes: %s; first execute state latches flags and the taken "
            "word differs only in PC_CTRL" % ", ".join(MNEMONIC[o] for o in sorted(CONDITIONAL)))
    return errors, note


def check_undefined(rom):
    """No undefined opcode is capable of modifying architectural state."""
    errors = []
    changing = ("MAR_LOAD", "MDR_LOAD", "IR_LOAD", "ACC_LOAD", "RF_WRITE",
                "MEM_WRITE", "STATUS_WE", "HALT")
    undefined = [op for op in range(N_OPCODES) if op not in EXECUTE]
    for opcode in undefined:
        for zn in range(4):
            z, n = zn >> 1, zn & 1
            for t in range(N_TSTATES):
                a = address(z, n, opcode, t)
                w = rom[a]
                if t < FETCH_STATES:
                    if w != FETCH[t]:
                        errors.append("%s is not the common fetch word" % where(a))
                    continue
                if any(get(w, name) for name in changing) or get(w, "PC_CTRL") != PC_HOLD:
                    errors.append("%s can modify state" % where(a))
                if not get(w, "STEP_RESET"):
                    errors.append("%s does not return to T0" % where(a))
    note = ("%d undefined opcodes (%s, including MUL = 12): after the common fetch every "
            "state is the safe step-reset word" % (len(undefined), ", ".join(str(o) for o in undefined)))
    return errors, note


CHECKS = [
    ("no control word selects more than one bus source", check_one_bus_source),
    ("every implemented instruction eventually asserts STEP_RESET or HALT", check_terminates),
    ("MEM_READ and MEM_WRITE are never asserted simultaneously", check_mem_exclusive),
    ("write enables are never asserted without valid source data", check_valid_source),
    ("conditional-branch words differ only in the relevant condition signal", check_conditions),
    ("no undefined opcode can modify architectural state", check_undefined),
]


# --------------------------------------------------------------------------
# 6. Output
# --------------------------------------------------------------------------
def write_hex(rom, path):
    digits = ROM_WIDTH // 4
    with open(path, "w", newline="\n") as f:
        f.write("v2.0 raw\n")
        for i in range(0, len(rom), 8):
            f.write(" ".join("%0*x" % (digits, w) for w in rom[i:i + 8]) + "\n")


def read_circuit_rom(path):
    """Return the control ROM stored in a Logisim circuit file."""
    for comp in ET.parse(path).getroot().iter("comp"):
        if comp.get("name") != "ROM":
            continue
        for attr in comp.findall("a"):
            if attr.get("name") == "contents":
                words = []
                for token in attr.text.split("\n", 1)[1].split():
                    count, _star, value = token.rpartition("*")
                    words += [int(value, 16)] * (int(count) if count else 1)
                return words + [0] * (4 * N_OPCODES * N_TSTATES - len(words))
    raise ValueError("no ROM found in %s" % path)


def main():
    parser = argparse.ArgumentParser(description="TARA control ROM generator (TH6)")
    parser.add_argument("-o", "--output", default="control_rom.hex")
    parser.add_argument("--log", default="microcode_gen.log")
    parser.add_argument("--verify", help="circuit file whose ROM must equal the generated image")
    args = parser.parse_args()

    lines = []

    def log(text=""):
        print(text)
        lines.append(text)

    rom = build_rom()

    log("TARA control ROM generator (TH6)")
    log("ROM: %d words x %d bits, address = {Z, N, OPCODE[4:0], TSTATE[3:0]}"
        % (len(rom), ROM_WIDTH))
    log()
    log("Control word layout")
    for name, lsb, width in FIELDS:
        bits = "%d" % lsb if width == 1 else "%d-%d" % (lsb + width - 1, lsb)
        log("  bits %-6s %s" % (bits, name))
    log("  bits 3, 6, 31 unused")
    log()
    log("Microprogram (identical for all four Z/N combinations except where shown)")
    for t, w in enumerate(FETCH):
        log("  fetch       T%d  %08x  %s" % (t, w, describe(w)))
    for opcode in IMPLEMENTED:
        for i, w in enumerate(EXECUTE[opcode]):
            t = FETCH_STATES + i
            if opcode in CONDITIONAL and i == 1:
                flag = CONDITIONAL[opcode]
                log("  %2d %-5s    T%d  %08x  %s=1: %s"
                    % (opcode, MNEMONIC[opcode], t, TAKEN, flag, describe(TAKEN)))
                log("  %2d %-5s    T%d  %08x  %s=0: %s"
                    % (opcode, MNEMONIC[opcode], t, NOT_TAKEN, flag, describe(NOT_TAKEN)))
            else:
                log("  %2d %-5s    T%d  %08x  %s" % (opcode, MNEMONIC[opcode], t, w, describe(w)))
    log("  all other addresses: %08x  %s" % (SAFE, describe(SAFE)))
    log()
    log("Consistency checks")
    failed = 0
    for number, (title, check) in enumerate(CHECKS, 1):
        errors, note = check(rom)
        log("  [%s] %d. %s" % ("PASS" if not errors else "FAIL", number, title))
        log("         %s" % note)
        for e in errors[:10]:
            log("         ERROR: %s" % e)
        if len(errors) > 10:
            log("         ... and %d more" % (len(errors) - 10))
        failed += bool(errors)
    log()

    if args.verify:
        stored = read_circuit_rom(args.verify)
        different = [a for a in range(len(rom)) if rom[a] != stored[a]]
        log("Comparison with the ROM in the circuit file")
        log("  [%s] %d of %d words differ"
            % ("PASS" if not different else "FAIL", len(different), len(rom)))
        for a in different[:10]:
            log("         %s: generated %08x, circuit %08x" % (where(a), rom[a], stored[a]))
        log()
        failed += bool(different)

    if failed:
        log("RESULT: %d check(s) FAILED - no ROM image written" % failed)
    else:
        write_hex(rom, args.output)
        log("RESULT: all checks passed")
        log("Wrote %s (%d words)" % (args.output, len(rom)))

    with open(args.log, "w", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
