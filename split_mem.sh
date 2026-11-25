#!/usr/bin/env bash
# split_mem.sh - split a TARA memory CSV into the two Logisim RAM images of ram_word.
#
# TARA is big-endian: the byte at the EVEN address is the high byte (bits 15-8).
#   <prefix>_HI.hex -> load into the LOWER RAM in ram_word (bits 15-8)
#   <prefix>_LO.hex -> load into the UPPER RAM in ram_word (bits 7-0)
# RAM row n = memory word at byte address 2n.
#
# Usage:
#   ./split_mem.sh [-b] memory.csv [out_prefix]
#
# Accepted CSV rows (all numbers hexadecimal, "0x" optional; separator , ; or tab):
#   word                 one 16-bit word per row, stored from address 0 upwards
#   address,word         byte address (must be even) and a 16-bit word
#   address,byte         with -b: byte address (even = high byte, odd = low byte) and an 8-bit value
# Blank lines, lines starting with #, and a header row (e.g. "address,value") are ignored.
# Addresses not mentioned are 0.

set -euo pipefail

byte_mode=0
if [[ "${1:-}" == "-b" ]]; then byte_mode=1; shift; fi
if [[ $# -lt 1 || $# -gt 2 ]]; then
    sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//' >&2
    exit 1
fi

csv=$1
[[ -r "$csv" ]] || { echo "error: cannot read '$csv'" >&2; exit 1; }
prefix=${2:-${csv%.*}}

awk -v BYTE="$byte_mode" -v HI="${prefix}_HI.hex" -v LO="${prefix}_LO.hex" '
function hexval(s,    t, v, i, c, d) {
    t = tolower(s); sub(/^0x/, "", t)
    if (t == "" || t !~ /^[0-9a-f]+$/) return -1
    v = 0
    for (i = 1; i <= length(t); i++) {
        c = substr(t, i, 1); d = index("0123456789abcdef", c) - 1
        v = v * 16 + d
    }
    return v
}
function fail(msg) { printf("error: line %d: %s\n", NR, msg) > "/dev/stderr"; bad = 1; exit 1 }
BEGIN { FS = "[,;\t]"; maxrow = -1; next_word = 0; rows = 0 }
{
    sub(/\r$/, "")                                   # Windows line endings
    gsub(/[ ]+/, "")                                 # stray spaces
    if ($0 == "" || $0 ~ /^#/) next
    if (NF == 1) {                                   # plain list of words
        if (BYTE) fail("byte mode (-b) needs address,byte rows")
        v = hexval($1)
        if (v < 0) { if (rows == 0) next; fail("not a hex value: " $1) }
        if (v > 65535) fail("value " $1 " does not fit in 16 bits")
        row = next_word++; hi[row] = int(v / 256); lo[row] = v % 256; seen[row] = 1
    } else {
        a = hexval($1); v = hexval($2)
        if (a < 0 || v < 0) { if (rows == 0) next; fail("not hex: " $0) }   # header row
        row = int(a / 2)
        if (BYTE) {
            if (v > 255) fail("byte value " $2 " does not fit in 8 bits")
            if (a % 2 == 0) hi[row] = v; else lo[row] = v
        } else {
            if (a % 2 == 1) fail("word address " $1 " is odd (use -b for byte rows)")
            if (v > 65535) fail("value " $2 " does not fit in 16 bits")
            if (row in seen) printf("warning: line %d: address %s given twice, last value kept\n", NR, $1) > "/dev/stderr"
            hi[row] = int(v / 256); lo[row] = v % 256
        }
        seen[row] = 1
    }
    rows++
    if (row > maxrow) maxrow = row
    if (row > 32767) fail("address beyond the 64 KB address space")
}
END {
    if (bad) exit 1
    if (maxrow < 0) { print "error: no memory values found" > "/dev/stderr"; exit 1 }
    print "v2.0 raw" > HI; print "v2.0 raw" > LO
    for (r = 0; r <= maxrow; r++) {
        sep = (r % 8 == 7 || r == maxrow) ? "\n" : " "
        printf("%x%s", (r in hi) ? hi[r] : 0, sep) > HI
        printf("%x%s", (r in lo) ? lo[r] : 0, sep) > LO
    }
    printf("%d value(s) read, %d RAM rows (byte addresses 0x0000-0x%04x)\n", rows, maxrow + 1, 2 * maxrow + 1)
    printf("  %s -> LOWER RAM in ram_word (high byte, bits 15-8)\n", HI)
    printf("  %s -> UPPER RAM in ram_word (low byte, bits 7-0)\n", LO)
}
' "$csv"
