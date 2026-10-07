#!/usr/bin/env bash
# Compile the RTL and run the testbenches with GHDL.
#   ./run_ghdl.sh                     all testbenches
#   ./run_ghdl.sh tb_cordic_nfu       one testbench
#   WAVE=1 ./run_ghdl.sh tb_int8_mac  also dump <tb>.ghw for GTKWave
set -e
cd "$(dirname "$0")"

GHDL=${GHDL:-ghdl}
STD=--std=08
WORK=work
mkdir -p "$WORK"

RTL="neuroflex_pkg int8_mac_engine seq_divider cordic_nfu weight_mem input_window_buf
     hidden_state_buf perf_counter hps_bridge_if gru_controller dense_argmax
     neuroflex_top de10nano_top"
ALL_TB="tb_int8_mac tb_cordic_nfu tb_neuroflex_top"

for f in $RTL; do
  "$GHDL" -a $STD --workdir="$WORK" "../rtl/$f.vhd"
done
for tb in $ALL_TB; do
  "$GHDL" -a $STD --workdir="$WORK" "../tb/$tb.vhd"
done

for tb in ${*:-$ALL_TB}; do
  echo "=== $tb ==="
  WAVE_OPT=""
  if [ -n "$WAVE" ]; then WAVE_OPT="--wave=$tb.ghw"; fi
  "$GHDL" -r $STD --workdir="$WORK" "$tb" --ieee-asserts=disable-at-0 $WAVE_OPT
done
