// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Self-checking testbench for bp_gp_core (global perceptron), driven by a
//   g_perceptron golden dump from the patched CBP5 harness. The DUT instance
//   is named dut and is the synthesized unit; the weight table is the
//   behavioral bp_sram instantiated here, next to dut (NUM_PERCEPTRONS rows of
//   (GHR_LEN+1)*WEIGHT_BITS bits, all zero), so the same bench runs the RTL
//   and, with POST_SYN_SIM, the synthesized/routed netlist (plain vector
//   ports: the branch only drops the parameters). Stimulus, checking, drain,
//   report and VCD come from tb_bp_common.svh.
//
//   The stimulus file is given with +STIM=<path> or the STIM_FILE parameter.
//
// Parameters:
//   GHR_LEN         - global history length (must match the DUT and the dump)
//   NUM_PERCEPTRONS - table rows (must match the DUT and the dump)
//   WEIGHT_BITS     - bits per weight (must match the DUT and the dump)
//   THETA_ALPHA_PCT - threshold scale in percent (must match the DUT and dump)
//   PC_SHIFT        - PC shift before indexing (must match the DUT and dump)
//   Y_REG           - DUT prediction timing (RTL only; baked into a netlist)
//   MOD_RECIP       - DUT index modulo style (RTL only; baked into a netlist)
//   STIM_FILE       - default stimulus path ("" = +STIM=<path> required)
//   MAX_LINES       - stop after this many branch lines (0 = whole file)
//   TIMEOUT_CYCLES  - max cycles to wait for a ready or valid
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

`ifndef CLK_PERIOD_NS
`define CLK_PERIOD_NS 10
`endif

`ifdef POST_SYN_SIM
// The gate-level flows compile only the netlist, the cell models and the
// bench, not the project RTL, so the bench pulls in its SRAM model itself.
// The include path is resolved relative to the flow's run directory
// (scripts/<flow>/), hence the two levels up to the repository root.
`include "../../projects/branch-prediction/rtl/bp_sram.sv"
`endif

/* verilator lint_off UNUSEDSIGNAL */

module tb_bp_gp_core #(
    parameter int unsigned GHR_LEN         = 18,
    parameter int unsigned NUM_PERCEPTRONS = 287,
    parameter int unsigned WEIGHT_BITS     = 6,
    parameter int unsigned THETA_ALPHA_PCT = 25,
    parameter int unsigned PC_SHIFT        = 2,
    parameter bit          Y_REG           = 1'b0,
    parameter bit          MOD_RECIP       = 1'b1,
    parameter string       STIM_FILE       = "",
    parameter int unsigned MAX_LINES       = 0,
    parameter int unsigned TIMEOUT_CYCLES  = 100
);

    localparam string       DUT_NAME = "bp_gp_core";
    localparam int unsigned PC_W     = 64;
    localparam int unsigned ROW_W    = (GHR_LEN + 1) * WEIGHT_BITS;
    localparam int unsigned IDX_W    = $clog2(NUM_PERCEPTRONS);

    logic             clk_i;
    logic             rst_ni;

    logic             pred_req_valid;
    logic             pred_req_ready;
    logic [ PC_W-1:0] pred_req_pc;
    logic             pred_resp_valid;
    logic             pred_resp_taken;

    logic             upd_valid;
    logic             upd_ready;
    logic             upd_is_cond;
    logic [ PC_W-1:0] upd_pc;
    logic             upd_taken;
    logic [ PC_W-1:0] upd_target;

    logic             wt_re;
    logic [IDX_W-1:0] wt_raddr;
    logic [ROW_W-1:0] wt_rdata;
    logic             wt_we;
    logic [IDX_W-1:0] wt_waddr;
    logic [ROW_W-1:0] wt_wdata;

    logic [      7:0] sram_rd_now;
    logic [      7:0] sram_wr_now;

    // -------------------------------------------------------------------------
    // DUT and weight table
    // -------------------------------------------------------------------------
`ifdef POST_SYN_SIM
    bp_gp_core dut (
`else
    bp_gp_core #(
        .GHR_LEN          (GHR_LEN),
        .NUM_PERCEPTRONS  (NUM_PERCEPTRONS),
        .WEIGHT_BITS      (WEIGHT_BITS),
        .THETA_ALPHA_PCT  (THETA_ALPHA_PCT),
        .PC_SHIFT         (PC_SHIFT),
        .Y_REG            (Y_REG),
        .MOD_RECIP        (MOD_RECIP),
        .PC_W             (PC_W)
    ) dut (
`endif
        .clk_i            (clk_i),
        .rst_ni           (rst_ni),
        .pred_req_valid_i (pred_req_valid),
        .pred_req_ready_o (pred_req_ready),
        .pred_req_pc_i    (pred_req_pc),
        .pred_resp_valid_o(pred_resp_valid),
        .pred_resp_taken_o(pred_resp_taken),
        .upd_valid_i      (upd_valid),
        .upd_ready_o      (upd_ready),
        .upd_is_cond_i    (upd_is_cond),
        .upd_pc_i         (upd_pc),
        .upd_taken_i      (upd_taken),
        .upd_target_i     (upd_target),
        .wt_re_o          (wt_re),
        .wt_raddr_o       (wt_raddr),
        .wt_rdata_i       (wt_rdata),
        .wt_we_o          (wt_we),
        .wt_waddr_o       (wt_waddr),
        .wt_wdata_o       (wt_wdata)
    );

    bp_sram #(
        .DEPTH  (NUM_PERCEPTRONS),
        .WIDTH  (ROW_W),
        .INIT   ('0)
    ) i_weights (
        .clk_i  (clk_i),
        .re_i   (wt_re),
        .raddr_i(wt_raddr),
        .rdata_o(wt_rdata),
        .we_i   (wt_we),
        .waddr_i(wt_waddr),
        .wdata_i(wt_wdata)
    );

    assign sram_rd_now = 8'(wt_re);
    assign sram_wr_now = 8'(wt_we);

    // -------------------------------------------------------------------------
    // Bench-specific strings for the shared body
    // -------------------------------------------------------------------------
    function automatic string tb_params();
        return $sformatf("GHR_LEN=%0d NUM_PERCEPTRONS=%0d WEIGHT_BITS=%0d THETA_ALPHA_PCT=%0d PC_SHIFT=%0d Y_REG=%0d MOD_RECIP=%0d",
                         GHR_LEN, NUM_PERCEPTRONS, WEIGHT_BITS, THETA_ALPHA_PCT, PC_SHIFT, Y_REG, MOD_RECIP);
    endfunction

    function automatic string tb_debug();
`ifdef POST_SYN_SIM
        return "";
`else
        // y_comb, not y_q: with Y_REG = 0, y_q is latched only on the edge
        // after the response; the row read data (and so y_comb) stays valid.
        return $sformatf("ghr %h row %0d y %0d", dut.ghr_q, dut.idx_q, $signed(dut.y_comb));
`endif
    endfunction

    `include "../../projects/branch-prediction/tb/tb_bp_common.svh"

endmodule