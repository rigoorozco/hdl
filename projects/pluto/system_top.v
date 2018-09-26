// ***************************************************************************
// ***************************************************************************
// Copyright 2014 - 2017 (c) Analog Devices, Inc. All rights reserved.
//
// In this HDL repository, there are many different and unique modules, consisting
// of various HDL (Verilog or VHDL) components. The individual modules are
// developed independently, and may be accompanied by separate and unique license
// terms.
//
// The user should read each of these license terms, and understand the
// freedoms and responsibilities that he or she has by using this source/core.
//
// This core is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE.
//
// Redistribution and use of source or resulting binaries, with or without modification
// of this file, are permitted under one of the following two license terms:
//
//   1. The GNU General Public License version 2 as published by the
//      Free Software Foundation, which can be found in the top level directory
//      of this repository (LICENSE_GPL2), and also online at:
//      <https://www.gnu.org/licenses/old-licenses/gpl-2.0.html>
//
// OR
//
//   2. An ADI specific BSD license, which can be found in the top level directory
//      of this repository (LICENSE_ADIBSD), and also on-line at:
//      https://github.com/analogdevicesinc/hdl/blob/master/LICENSE_ADIBSD
//      This will allow to generate bit files and not release the source code,
//      as long as it attaches to an ADI device.
//
// ***************************************************************************
// ***************************************************************************

`timescale 1ns/100ps

module system_top (

  inout   [14:0]  ddr_addr,
  inout   [ 2:0]  ddr_ba,
  inout           ddr_cas_n,
  inout           ddr_ck_n,
  inout           ddr_ck_p,
  inout           ddr_cke,
  inout           ddr_cs_n,
  inout   [ 1:0]  ddr_dm,
  inout   [15:0]  ddr_dq,
  inout   [ 1:0]  ddr_dqs_n,
  inout   [ 1:0]  ddr_dqs_p,
  inout           ddr_odt,
  inout           ddr_ras_n,
  inout           ddr_reset_n,
  inout           ddr_we_n,

  inout           fixed_io_ddr_vrn,
  inout           fixed_io_ddr_vrp,
  inout   [31:0]  fixed_io_mio,
  inout           fixed_io_ps_clk,
  inout           fixed_io_ps_porb,
  inout           fixed_io_ps_srstb,

  inout           iic_scl,
  inout           iic_sda,

  inout           gpio_bd,

  input           rx_clk_in,
  input           rx_frame_in,
  input   [11:0]  rx_data_in,
  output          tx_clk_out,
  output          tx_frame_out,
  output  [11:0]  tx_data_out,

  output          enable,
  output          txnrx,
  input           clk_out,

  inout           gpio_resetb,
  inout           gpio_en_agc,
  inout   [ 3:0]  gpio_ctl,
  inout   [ 7:0]  gpio_status,

  output          spi_csn,
  output          spi_clk,
  output          spi_mosi,
  input           spi_miso);

  // internal signals

  wire    [16:0]  gpio_i;
  wire    [16:0]  gpio_o;
  wire    [16:0]  gpio_t;
  
  wire            bus_clk;
  wire            bus_rst;
  wire            radio_clk;
  wire            radio_rst;
  
  wire    [63:0]  h2s_tdata;
  wire            h2s_tlast;
  wire            h2s_tvalid;
  wire            h2s_tready;
  
  wire    [63:0]  s2h_tdata;
  wire            s2h_tlast;
  wire            s2h_tvalid;
  wire            s2h_tready;
  
  wire    [31:0]  core_set_data;
  wire    [7:0]   core_set_addr;
  wire            core_set_stb;
  wire    [31:0]  core_rb_data;
  
  wire    [31:0]  xbar_set_data;
  wire    [31:0]  xbar_set_addr;
  wire            xbar_set_stb;
  
  wire    [31:0]  xbar_rb_data;
  wire    [31:0]  xbar_rb_addr;
  wire            xbar_rb_stb;
  
  wire            mimo;
  wire            codec_arst;
  
  wire    [11:0]  rx_i0, rx_q0, rx_i1, rx_q1, tx_i0, tx_q0, tx_i1, tx_q1;
  wire    [31:0]  rx_data0, rx_data1, tx_data0, tx_data1;
  
  assign          rx_data0      = {rx_i0,4'd0,rx_q0,4'd0};
  assign          rx_data1      = {rx_i1,4'd0,rx_q1,4'd0};
  assign          {tx_i0,tx_q0} = {tx_data0[31:20],tx_data0[15:4]};
  assign          {tx_i1,tx_q1} = {tx_data1[31:20],tx_data1[15:4]};
  
  wire            rx_stb, tx_stb;
  
  e310_io e310_io (
    .areset(codec_arst),
    .mimo(mimo),
    // Baseband sample interface
    .radio_clk(radio_clk),
    .radio_rst(radio_rst),
    .rx_i0(rx_i0),
    .rx_q0(rx_q0),
    .rx_i1(rx_i1),
    .rx_q1(rx_q1),
    .rx_stb(rx_stb),
    .tx_i0(tx_i0),
    .tx_q0(tx_q0),
    .tx_i1(tx_i1),
    .tx_q1(tx_q1),
    .tx_stb(tx_stb),
    // AD9361 interface
    .rx_clk(rx_clk_in),      /*CAT_DATA_CLK*/
    .rx_frame(rx_frame_in),  /*CAT_RX_FRAME*/
    .rx_data(rx_data_in),    /*CAT_P0_D*/
    .tx_clk(tx_clk_out),     /*CAT_FB_CLK*/
    .tx_frame(tx_frame_out), /*CAT_TX_FRAME*/
    .tx_data(tx_data_out));  /*CAT_P1_D*/
    
  assign enable = 1'b1;
  assign txnrx  = 1'b1;
  
  wire       pps, clk_pps, clk_tcxo;
  
  assign gpio_i[16:15] = gpio_o[16:15];
  // instantiations

  ad_iobuf #(.DATA_WIDTH(15)) i_iobuf (
    .dio_t (gpio_t[14:0]),
    .dio_i (gpio_o[14:0]),
    .dio_o (gpio_i[14:0]),
    .dio_p ({ gpio_bd,            // 14:14
              gpio_resetb,        // 13:13
              gpio_en_agc,        // 12:12
              gpio_ctl,           // 11: 8
              gpio_status}));     //  7: 0

  system_wrapper i_system_wrapper (
    .ddr_addr (ddr_addr),
    .ddr_ba (ddr_ba),
    .ddr_cas_n (ddr_cas_n),
    .ddr_ck_n (ddr_ck_n),
    .ddr_ck_p (ddr_ck_p),
    .ddr_cke (ddr_cke),
    .ddr_cs_n (ddr_cs_n),
    .ddr_dm (ddr_dm),
    .ddr_dq (ddr_dq),
    .ddr_dqs_n (ddr_dqs_n),
    .ddr_dqs_p (ddr_dqs_p),
    .ddr_odt (ddr_odt),
    .ddr_ras_n (ddr_ras_n),
    .ddr_reset_n (ddr_reset_n),
    .ddr_we_n (ddr_we_n),
    .fixed_io_ddr_vrn (fixed_io_ddr_vrn),
    .fixed_io_ddr_vrp (fixed_io_ddr_vrp),
    .fixed_io_mio (fixed_io_mio),
    .fixed_io_ps_clk (fixed_io_ps_clk),
    .fixed_io_ps_porb (fixed_io_ps_porb),
    .fixed_io_ps_srstb (fixed_io_ps_srstb),
    .gpio_i (gpio_i),
    .gpio_o (gpio_o),
    .gpio_t (gpio_t),
    .iic_main_scl_io (iic_scl),
    .iic_main_sda_io (iic_sda),
    .ps_intr_00 (1'b0),
    .ps_intr_01 (1'b0),
    .ps_intr_02 (1'b0),
    .ps_intr_03 (1'b0),
    .ps_intr_04 (1'b0),
    .ps_intr_05 (1'b0),
    .ps_intr_06 (1'b0),
    .ps_intr_07 (1'b0),
    .ps_intr_08 (1'b0),
    .ps_intr_09 (1'b0),
    .ps_intr_10 (1'b0),
    .ps_intr_11 (1'b0),
    .ps_intr_12 (1'b0),
    .ps_intr_13 (1'b0),
    .bus_clk(bus_clk),
    .bus_rst(bus_rst),
    .clk_40mhz(clk_tcxo),
    .clk_10mhz(clk_pps),
    .h2s_tdata(h2s_tdata),
    .h2s_tlast(h2s_tlast),
    .h2s_tvalid(h2s_tvalid),
    .h2s_tready(h2s_tready),
    .s2h_tdata(s2h_tdata),
    .s2h_tlast(s2h_tlast),
    .s2h_tvalid(s2h_tvalid),
    .s2h_tready(s2h_tready),
    .core_set_data(core_set_data),
    .core_set_addr(core_set_addr),
    .core_set_stb(core_set_stb),
    .core_rb_data(core_rb_data),
    .xbar_set_data(xbar_set_data),
    .xbar_set_addr(xbar_set_addr),
    .xbar_set_stb(xbar_set_stb),
    .xbar_rb_data(xbar_rb_data),
    .xbar_rb_addr(xbar_rb_addr),
    .xbar_rb_stb(xbar_rb_stb),
    .spi0_clk_i (1'b0),
    .spi0_clk_o (spi_clk),
    .spi0_csn_0_o (spi_csn),
    .spi0_csn_1_o (),
    .spi0_csn_2_o (),
    .spi0_csn_i (1'b1),
    .spi0_sdi_i (spi_miso),
    .spi0_sdo_i (1'b0),
    .spi0_sdo_o (spi_mosi));
    
  wire [1:0] pps_select;
  wire is_10meg, is_pps, reflck, plllck; // reference status bits
  
  ppsloop ppslp (
    .reset(1'b0),
    .xoclk(clk_tcxo), .ppsgps(clk_pps), .ppsext(clk_pps),
    .refsel(pps_select),
    .lpps(pps),
    .is10meg(is_10meg), .ispps(is_pps), .reflck(reflck), .plllck(plllck),
    .sclk(), .mosi(), .sync_n(),
    .dac_dflt(16'h7fff));
    
  reg [3:0] tcxo_status, st_rsync;

  always @(posedge bus_clk) begin
    /* status signals originate from other than the bus_clk domain so re-sync
       before passing to e300_core
     */
    st_rsync <= {plllck, is_10meg, is_pps, reflck};
    tcxo_status <= st_rsync;
  end

  e310_core e310_core0 (
    .bus_clk(bus_clk),
    .bus_rst(bus_rst),

    .h2s_tdata(h2s_tdata),
    .h2s_tlast(h2s_tlast),
    .h2s_tvalid(h2s_tvalid),
    .h2s_tready(h2s_tready),

    .s2h_tdata(s2h_tdata),
    .s2h_tlast(s2h_tlast),
    .s2h_tvalid(s2h_tvalid),
    .s2h_tready(s2h_tready),

    .radio_clk(radio_clk),
    .radio_rst(radio_rst),
   
    .rx_stb0(rx_stb),
    .rx_data0(rx_data0),
    .tx_stb0(tx_stb),
    .tx_data0(tx_data0),
    .rx_stb1(rx_stb),
    .rx_data1(rx_data1),
    .tx_stb1(tx_stb),
    .tx_data1(tx_data1),

    // Unused
    .db_gpio0(),
    .db_gpio1(),
    .leds0(),
    .leds1(),

    // Unused
    .fp_gpio_in0(6'd0),
    .fp_gpio_out0(),
    .fp_gpio_ddr0(),
    
    // Unused
    .spi_sen(),
    .spi_sclk(),
    .spi_mosi(),
    .spi_miso(1'd0),

    .set_data(core_set_data),
    .set_addr(core_set_addr),
    .set_stb(core_set_stb),
    .rb_data(core_rb_data),

    .xbar_set_data(xbar_set_data),
    .xbar_set_addr(xbar_set_addr),
    .xbar_set_stb(xbar_set_stb),
    .xbar_rb_data(xbar_rb_data),
    .xbar_rb_addr(xbar_rb_addr),
    .xbar_rb_stb(xbar_rb_stb),

    .pps_select(pps_select),
    .pps(pps),
    .tcxo_status(tcxo_status),

   .lock_signals(), /*CAT_CTRL_OUT[7:6]*/
   .mimo(mimo),
   .codec_arst(codec_arst),
   .debug());

endmodule

// ***************************************************************************
// ***************************************************************************
