// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Self-contained global perceptron predictor: bp_gp_core plus its weight
//   table (bp_sram, NUM_PERCEPTRONS rows of (GHR_LEN+1)*WEIGHT_BITS bits, all
//   weights zero at start as g_perceptron.h's constructor). Exposes only the
//   shared predictor interface, so it is the block to integrate into a core.
//
//   Do NOT synthesize this module as is: bp_sram is a behavioral model and
//   would be built from flip-flops. For PPA, synthesize bp_gp_core
//   (TOP_LEVEL=bp_gp_core). The SRAM is costed separately; a non-power-of-two
//   NUM_PERCEPTRONS is costed as the next power of two (unused rows).
//
// Parameters:
//   GHR_LEN         - global history length (1..64)
//   NUM_PERCEPTRONS - table rows (>= 2)
//   WEIGHT_BITS     - bits per weight (2..8)
//   THETA_ALPHA_PCT - threshold scale in percent (1..1000)
//   PC_SHIFT        - right shift of the PC before indexing (0..3)
//   Y_REG           - 0: predict in the cycle the row arrives; 1: one later
//   MOD_RECIP       - index modulo: 1 reciprocal multiply, 0 generic divider
//   PC_W            - PC / target width of the shared predictor interface
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_gp_top #(
    parameter int unsigned GHR_LEN         = 18,
    parameter int unsigned NUM_PERCEPTRONS = 287,
    parameter int unsigned WEIGHT_BITS     = 6,
    parameter int unsigned THETA_ALPHA_PCT = 25,
    parameter int unsigned PC_SHIFT        = 2,
    parameter bit          Y_REG           = 1'b0,
    parameter bit          MOD_RECIP       = 1'b1,
    parameter int unsigned PC_W            = 64
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

    localparam int unsigned ROW_W = (GHR_LEN + 1) * WEIGHT_BITS;
    localparam int unsigned IDX_W = $clog2(NUM_PERCEPTRONS);

    logic             wt_re;
    logic [IDX_W-1:0] wt_raddr;
    logic [ROW_W-1:0] wt_rdata;
    logic             wt_we;
    logic [IDX_W-1:0] wt_waddr;
    logic [ROW_W-1:0] wt_wdata;

    bp_gp_core #(
        .GHR_LEN          (GHR_LEN),
        .NUM_PERCEPTRONS  (NUM_PERCEPTRONS),
        .WEIGHT_BITS      (WEIGHT_BITS),
        .THETA_ALPHA_PCT  (THETA_ALPHA_PCT),
        .PC_SHIFT         (PC_SHIFT),
        .Y_REG            (Y_REG),
        .MOD_RECIP        (MOD_RECIP),
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

endmodule