#!/bin/sh
# Full Quartus flow for the PSX project, run from the repository root.
#   tools/composite/build.sh REPORT_DIR [SEED]
# QUARTUS_BIN defaults to the Quartus 17.0 Lite install used for releases.
R=$1; SEED=${2:-1}; Q=${QUARTUS_BIN:-C:/intelFPGA_lite/17.0/quartus/bin64}
mkdir -p "$R"
run() { echo "$1" > "$R/stage.txt"; step=$1; shift; "$@" > "$R/$step.log" 2>&1 || { echo "Failed: $step" > "$R/result.txt"; exit 1; }; }
run prepare "$Q/quartus_sh" -t sys/build_id.tcl pre_flow PSX PSX
run map "$Q/quartus_map" PSX -c PSX --parallel=4 --write_settings_files=off
run fit "$Q/quartus_fit" PSX -c PSX --parallel=4 --seed=$SEED --write_settings_files=off
run asm "$Q/quartus_asm" PSX -c PSX --write_settings_files=off
run sta "$Q/quartus_sta" PSX -c PSX
cp output_files/PSX.*.summary output_files/PSX.rbf "$R/"
echo "Build completed" > "$R/result.txt"
