// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Self-contained gshare predictor: bp_gshare_core plus its pattern history
//   table (bp_sram, 2^INDEX_BITS x CTR_BITS, every counter initialized to
//   weakly taken as gshare.h's constructor does). Exposes only the shared
//   predictor interface, so it is the block to integrate into a core later.
//
//   Do NOT synthesize this module as is: bp_sram is a behavioral model and
//   would be built from flip-flops. For PPA, synthesize bp_gshare_core
//   (TOP_LEVEL=bp_gshare_core); when integrating, replace bp_sram with an
//   SRAM macro wrapper with the same ports, or blackbox it.
//
// Parameters:
//   GHR_BITS   - global history length (1..32)
//   INDEX_BITS - log2(PHT entries) (1..31)
//   PC_SHIFT   - right shift of the PC before indexing (0..3)
//   CTR_BITS   - PHT counter width; 2 matches gshare.h
//   PC_W       - PC / target width of the shared predictor interface
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_gshare_top #(
    parameter int unsigned GHR_BITS   = 14,
    parameter int unsigned INDEX_BITS = 14,
    parameter int unsigned PC_SHIFT   = 2,
    parameter int unsigned CTR_BITS   = 2,
    parameter int unsigned PC_W       = 64
) (
    input  logic            clk_i,
    input  logic            rst_ni,

    // predict channel (CBP5 GetPrediction)
    input  logic            pred_req_valid_i,
    output logic            pred_req_ready_o,
    input  logic [PC_W-1:0] pred_req_pc_i,
    output logic            pred_resp_valid_o,
    output logic            pred_resp_taken_o,

    // update channel (CBP5 UpdatePredictor / TrackOtherInst)
    input  logic            upd_valid_i,
    output logic            upd_ready_o,
    input  logic            upd_is_cond_i,
    input  logic [PC_W-1:0] upd_pc_i,
    input  logic            upd_taken_i,
    input  logic [PC_W-1:0] upd_target_i
);

    // Weakly taken: 2'b10 for 2-bit counters (gshare.h PHT_CTR_INIT).
    localparam logic [CTR_BITS-1:0] CTR_INIT = CTR_BITS'(1 << (CTR_BITS - 1));

    logic                  pht_re;
    logic [INDEX_BITS-1:0] pht_raddr;
    logic [  CTR_BITS-1:0] pht_rdata;
    logic                  pht_we;
    logic [INDEX_BITS-1:0] pht_waddr;
    logic [  CTR_BITS-1:0] pht_wdata;

    bp_gshare_core #(
        .GHR_BITS         (GHR_BITS),
        .INDEX_BITS       (INDEX_BITS),
        .PC_SHIFT         (PC_SHIFT),
        .CTR_BITS         (CTR_BITS),
        .PC_W             (PC_W)
    ) i_core (
        .clk_i            (clk_i),
        .rst_ni           (rst_ni),
        .pred_req_valid_i (pred_req_valid_i),
        .pred_req_ready_o (pred_req_ready_o),
        .pred_req_pc_i    (pred_req_pc_i),
        .pred_resp_valid_o(pred_resp_valid_o),
        .pred_resp_taken_o(pred_resp_taken_o),
        .upd_valid_i      (upd_valid_i),
        .upd_ready_o      (upd_ready_o),
        .upd_is_cond_i    (upd_is_cond_i),
        .upd_pc_i         (upd_pc_i),
        .upd_taken_i      (upd_taken_i),
        .upd_target_i     (upd_target_i),
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

endmodule