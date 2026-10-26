#!/bin/sh
# Encoder + decoder simulation with Icarus Verilog, run from this folder.
#   ./run.sh NAME [PARAM=value ...]    e.g. ./run.sh comb COMB=2 FRAMES=3
# Parameters (tb_equiv.sv): COMB, SHARP, FRAMES, PATTERN, INTERLACE, UPSET.
name=$1; shift
IV=${IVERILOG:-iverilog}; VVP=${VVP:-vvp}
R=../../../../rtl/composite
mkdir -p "$name" && cd "$name" || exit 1
P=""
for a in "$@"; do P="$P -Ptb.$a"; done
$IV -g2012 $P -o tb.vvp ../tb_equiv.sv ../composite_decoder_ref.sv \
	$R/composite_decoder.sv $R/cxa1645.sv $R/mul_logic.sv &&
	$VVP -n tb.vvp > result.txt 2>&1
tail -4 result.txt
