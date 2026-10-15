// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Self-checking testbench for bp_gshare_core, driven by a gshare golden
//   dump from the patched CBP5 harness. The DUT instance is named dut and is
//   the synthesized unit; the PHT is the behavioral bp_sram instantiated here,
//   next to dut, so the same bench runs the RTL and, with POST_SYN_SIM, the
//   synthesized/routed netlist (plain vector ports: the branch only drops the
//   parameters). Stimulus, prediction check, drain, report, VCD and the
//   activity window come from tb_bp_common.svh, as in the other benches.
//
//   The stimulus file is given with +STIM=<path> or the STIM_FILE parameter.
//
// Parameters:
//   GHR_BITS         - global history length (must match the DUT and the dump)
//   INDEX_BITS       - log2(PHT entries) (must match the DUT and the dump)
//   PC_SHIFT         - PC shift before indexing (must match the DUT and the dump)
//   CTR_BITS         - PHT counter width (must match the DUT)
//   STIM_FILE        - default stimulus path ("" = +STIM=<path> required)
//   MAX_LINES        - stop after this many branch lines (0 = whole file)
//   VCD_START        - first branch line of the activity window (0 = from
//                      reset; see tb_bp_common.svh)
//   TIMEOUT_CYCLES   - max cycles to wait for a ready or valid
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

`ifndef CLK_PERIOD_NS
`define CLK_PERIOD_NS 10
`endif

`ifdef POST_SYN_SIM
// The gate-level flows compile only the netlist, the cell models and the
// bench; bp_sram is a bench-side model, so the bench pulls it in (path
// relative to scripts/post-syn-sim/ or scripts/post-pnr-sim/).
`include "../../projects/branch-prediction/rtl/bp_sram.sv"
`endif

/* verilator lint_off UNUSEDSIGNAL */

module tb_bp_gshare_core #(
    parameter int unsigned GHR_BITS       = 14,
    parameter int unsigned INDEX_BITS     = 14,
    parameter int unsigned PC_SHIFT       = 2,
    parameter int unsigned CTR_BITS       = 2,
    parameter string       STIM_FILE      = "",
    parameter int unsigned MAX_LINES      = 0,
    parameter int unsigned VCD_START      = 0,
    parameter int unsigned TIMEOUT_CYCLES = 100
);

    localparam string               DUT_NAME = "bp_gshare_core";
    localparam int unsigned         PC_W     = 64;
    localparam logic [CTR_BITS-1:0] CTR_INIT = CTR_BITS'(1 << (CTR_BITS - 1));

    logic                  clk_i;
    logic                  rst_ni;

    logic                  pred_req_valid;
    logic                  pred_req_ready;
    logic [      PC_W-1:0] pred_req_pc;
    logic                  pred_resp_valid;
    logic                  pred_resp_taken;

    logic                  upd_valid;
    logic                  upd_ready;
    logic                  upd_is_cond;
    logic [      PC_W-1:0] upd_pc;
    logic                  upd_taken;
    logic [      PC_W-1:0] upd_target;

    logic                  pht_re;
    logic [INDEX_BITS-1:0] pht_raddr;
    logic [  CTR_BITS-1:0] pht_rdata;
    logic                  pht_we;
    logic [INDEX_BITS-1:0] pht_waddr;
    logic [  CTR_BITS-1:0] pht_wdata;

    logic [           7:0] sram_rd_now;
    logic [           7:0] sram_wr_now;

    // -------------------------------------------------------------------------
    // DUT and PHT
    // -------------------------------------------------------------------------
`ifdef POST_SYN_SIM
    bp_gshare_core dut (
`else
    bp_gshare_core #(
        .GHR_BITS         (GHR_BITS),
        .INDEX_BITS       (INDEX_BITS),
        .PC_SHIFT         (PC_SHIFT),
        .CTR_BITS         (CTR_BITS),
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
        .pht_re_o         (pht_re),
        .pht_raddr_o      (pht_raddr),
        .pht_rdata_i      (pht_rdata),
        .pht_we_o         (pht_we),
        .pht_waddr_o      (pht_waddr),
        .pht_wdata_o      (pht_wdata)
    );

    bp_sram #(
        .DEPTH  (1 << INDEX_BITS),
        .WIDTH  (CTR_BITS),
        .INIT   (CTR_INIT)
    ) i_pht (
        .clk_i  (clk_i),
        .re_i   (pht_re),
        .raddr_i(pht_raddr),
        .rdata_o(pht_rdata),
        .we_i   (pht_we),
        .waddr_i(pht_waddr),
        .wdata_i(pht_wdata)
    );

    assign sram_rd_now = 8'(pht_re);
    assign sram_wr_now = 8'(pht_we);

    // -------------------------------------------------------------------------
    // Bench-specific strings for the shared body
    // -------------------------------------------------------------------------
    function automatic string tb_params();
        return $sformatf("GHR_BITS=%0d INDEX_BITS=%0d PC_SHIFT=%0d CTR_BITS=%0d",
                         GHR_BITS, INDEX_BITS, PC_SHIFT, CTR_BITS);
    endfunction

    function automatic string tb_debug();
`ifdef POST_SYN_SIM
        return "";
`else
        return $sformatf("ghr %h idx %h", dut.ghr_q, dut.idx_q);
`endif
    endfunction

    `include "../../projects/branch-prediction/tb/tb_bp_common.svh"

endmodule