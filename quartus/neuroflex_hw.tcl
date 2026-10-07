# Platform Designer component: NeuroFlex-CORDIC TinyGRU accelerator.
# Add this folder to the IP search path, then connect s0 to h2f_lw_axi_master.

package require -exact qsys 16.1

set_module_property NAME neuroflex
set_module_property DISPLAY_NAME "NeuroFlex-CORDIC TinyGRU Accelerator"
set_module_property DESCRIPTION "INT8 MAC + shared CORDIC TinyGRU inference engine"
set_module_property VERSION 1.0
set_module_property GROUP "NeuroFlex"
set_module_property EDITABLE true
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true

set rtl_files {
  neuroflex_pkg.vhd int8_mac_engine.vhd seq_divider.vhd cordic_nfu.vhd
  weight_mem.vhd input_window_buf.vhd hidden_state_buf.vhd perf_counter.vhd
  hps_bridge_if.vhd gru_controller.vhd dense_argmax.vhd neuroflex_top.vhd
}
set mif_files {tinygru_wrow.mif tinygru_bias.mif imu_window.mif}

foreach fs {QUARTUS_SYNTH SIM_VHDL} {
  add_fileset $fs $fs "" ""
  set_fileset_property $fs TOP_LEVEL neuroflex_top
  set_fileset_property $fs ENABLE_RELATIVE_INCLUDE_PATHS false
  foreach f $rtl_files {
    add_fileset_file $f VHDL PATH ../rtl/$f
  }
  foreach f $mif_files {
    add_fileset_file $f MIF PATH $f
  }
}

# Clock / reset
add_interface clock clock end
set_interface_property clock clockRate 0
add_interface_port clock clk clk Input 1

add_interface reset reset end
set_interface_property reset associatedClock clock
set_interface_property reset synchronousEdges DEASSERT
add_interface_port reset reset reset Input 1

# Avalon-MM slave, 8 KB span (2048 x 32-bit words), fixed read latency 1
add_interface s0 avalon end
set_interface_property s0 addressUnits WORDS
set_interface_property s0 associatedClock clock
set_interface_property s0 associatedReset reset
set_interface_property s0 bitsPerSymbol 8
set_interface_property s0 burstOnBurstBoundariesOnly false
set_interface_property s0 explicitAddressSpan 0
set_interface_property s0 holdTime 0
set_interface_property s0 linewrapBursts false
set_interface_property s0 maximumPendingReadTransactions 0
set_interface_property s0 readLatency 1
set_interface_property s0 readWaitTime 0
set_interface_property s0 setupTime 0
set_interface_property s0 timingUnits Cycles
set_interface_property s0 writeWaitTime 0
add_interface_port s0 avs_address   address   Input  11
add_interface_port s0 avs_write     write     Input  1
add_interface_port s0 avs_writedata writedata Input  32
add_interface_port s0 avs_read      read      Input  1
add_interface_port s0 avs_readdata  readdata  Output 32

# Conduit: external start / status (tie ext_* to 0 when only the HPS is used)
add_interface status conduit end
set_interface_property status associatedClock clock
set_interface_property status associatedReset reset
add_interface_port status ext_start   ext_start   Input  1
add_interface_port status ext_mode    ext_mode    Input  2
add_interface_port status ext_win_sel ext_win_sel Input  1
add_interface_port status st_busy     st_busy     Output 1
add_interface_port status st_done     st_done     Output 1
add_interface_port status st_class    st_class    Output 8
add_interface_port status st_score    st_score    Output 16
