# ModelSim / Questa (Intel FPGA edition). Run from the sim/ directory:
#   vsim -c -do run_modelsim.do
vlib work
vmap work work

set rtl {neuroflex_pkg int8_mac_engine seq_divider cordic_nfu weight_mem input_window_buf
         hidden_state_buf perf_counter hps_bridge_if gru_controller dense_argmax
         neuroflex_top de10nano_top}
foreach f $rtl {
  vcom -2008 -work work ../rtl/$f.vhd
}
foreach tb {tb_int8_mac tb_cordic_nfu tb_neuroflex_top} {
  vcom -2008 -work work ../tb/$tb.vhd
}

foreach tb {tb_int8_mac tb_cordic_nfu tb_neuroflex_top} {
  vsim -quiet work.$tb
  run -all
  quit -sim
}
quit -f
