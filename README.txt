TH6: MEMORY, BRANCHES, ADVANCED INSTRUCTIONS AND INTEGRATION
TARA processor, CS2300

Group 32
CS25B033, CS25B090


FILES
-----
G32_th6_tara.circ    Complete processor. Main circuit: control_rom
microcode_gen.py     Generates the control ROM and runs the consistency checks
control_rom.hex      Generated ROM image, 2048 x 32 bits
microcode_gen.log    Field layout, microprogram listing and check results
G32_th6_report.pdf   Report explaining the circuit
split_mem.sh         Optional: splits a TARA memory CSV into two RAM images
programs/            The six integration programs (.asm, _low.hex, _high.hex)
<n>_<program>_registervalues.png   Screenshot of the final registers
<n>_<program>_ram_word.png         Screenshot of ram_word

The ROM image is already stored in G32_th6_tara.circ. It does not need to be
loaded again unless the microcode changes.


REQUIREMENTS
------------
Logisim-evolution 4.1.0
Python 3 (only to run the generator)
Bash and awk (only to run split_mem.sh)


RUNNING A PROGRAM
-----------------
1. Open G32_th6_tara.circ in Logisim-evolution. Stay in control_rom.
2. Enter ram_word through the running circuit: double-click the main_datapath
   block, then the ram_word block. Do not open it from the explorer panel,
   which shows a separate copy.
3. Clear both RAMs if another program was loaded before:
   right-click each RAM > Clear Contents.
4. Load the images (right-click a RAM > Load Image):
       Upper RAM (low byte, bits 7:0)    programs/<n>_<program>_low.hex
       Lower RAM (high byte, bits 15:8)  programs/<n>_<program>_high.hex
5. Return to control_rom. Toggle RST to 1 and back to 0.
6. Turn on Simulate > Auto-Tick and wait for the HLT lamp to light.
7. Read R0 to R7 and PC from the output pins.


EXPECTED RESULTS AT HLT (hexadecimal; R6 and R7 stay 0000)
----------------------------------------------------------
#  Program             R0    R1    R2    R3    R4    R5    PC    Answer
1  Register addition   0000  0019  0011  002A  0000  0000  0008  R3 = 25 + 17
2  Memory subtraction  0400  003A  0064  003A  002A  0000  0016  R4 = 100 - 58
3  Parity of B5A3      0000  B5A3  0001  0001  0000  0001  0016  R2 = 1 (odd)
4  Multiplication      0000  0006  0000  002A  0000  0000  0010  R3 = 6 x 7
5  Array sum           0400  0000  05DC  040A  01F4  0000  0032  R2 = 1500
6  Array maximum       0400  0000  012C  040A  FF9C  0000  003A  R2 = 300

PC reads one instruction past HLT, because HLT has already been fetched.


REGENERATING THE ROM
--------------------
    python microcode_gen.py
    python microcode_gen.py --verify G32_th6_tara.circ

The first command writes control_rom.hex and microcode_gen.log. The second also
compares the generated image with the ROM stored in the circuit. No image is
written if a check or the comparison fails.

After changing the microcode, right-click the ROM in control_rom, choose
Load Image, select control_rom.hex, and save the circuit.


DESIGN DECISIONS
----------------
ROM address     {Z, N, OPCODE[4:0], TSTATE[3:0]}, using latched Z and N
PC-relative     target = PC_next + ext(rel) x 2, where PC_next = address + 2
                Assembler: rel = (label - (address + 2)) / 2
off5            Sign-extended, -16 to +15 bytes
Byte order      The even address holds the word's high byte, as in TARA Studio
STB             Byte write-enable, no extra states
LDW, STW        ADDR[0] is ignored; use even addresses
BZ, BN          T3 latches flags from Rt + 0; T4 uses the latched Z or N
CALL            R6 and PC load on the same clock edge, so R6 = address + 2
PUSH            R7 <- R7 - 2, then Mem[R7] <- Rs
                PUSH R7 stores the decremented value
POP             Rd <- Mem[R7], then R7 <- R7 + 2
                POP R7 leaves R7 = Mem[old R7] + 2
Stack pointer   Initialise R7 to a valid, even, writable address before use
MUL (12), 27-31 Common fetch, then a safe STEP_RESET word; no state changes
