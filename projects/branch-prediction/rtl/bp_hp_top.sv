// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Self-contained hashed perceptron predictor: bp_hp_core plus its tables as
//   behavioral bp_sram models (bias: BIAS_ENTRIES x BIAS_WEIGHT_BITS; table t:
//   2^LOG_t x WBITS_t; all weights zero at start, as hp_core.h). Exposes only
//   the shared predictor interface.
//
//   Do NOT synthesize this module as is: bp_sram is a behavioral model and
//   would be built from flip-flops. For PPA, synthesize bp_hp_core
//   (TOP_LEVEL=bp_hp_core) and cost the SRAMs separately.
//
// Parameters:
//   see bp_hp_core
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_hp_top #(
    parameter int unsigned NUM_TABLES       = 6,
    parameter int unsigned T0_LOG           = 8,
    parameter int unsigned T1_LOG           = 8,
    parameter int unsigned T2_LOG           = 8,
    parameter int unsigned T3_LOG           = 8,
    parameter int unsigned T4_LOG           = 8,
    parameter int unsigned T5_LOG           = 8,
    parameter int unsigned T6_LOG           = 0,
    parameter int unsigned T7_LOG           = 0,
    parameter int unsigned T0_WBITS         = 3,
    parameter int unsigned T1_WBITS         = 3,
    parameter int unsigned T2_WBITS         = 3,
    parameter int unsigned T3_WBITS         = 3,
    parameter int unsigned T4_WBITS         = 3,
    parameter int unsigned T5_WBITS         = 3,
    parameter int unsigned T6_WBITS         = 0,
    parameter int unsigned T7_WBITS         = 0,
    parameter int unsigned T0_HIST          = 2,
    parameter int unsigned T1_HIST          = 3,
    parameter int unsigned T2_HIST          = 5,
    parameter int unsigned T3_HIST          = 7,
    parameter int unsigned T4_HIST          = 11,
    parameter int unsigned T5_HIST          = 16,
    parameter int unsigned T6_HIST          = 0,
    parameter int unsigned T7_HIST          = 0,
    parameter int unsigned BIAS_ENTRIES     = 256,
    parameter int unsigned BIAS_WEIGHT_BITS = 5,
    parameter int unsigned THETA_BITS       = 12,
    parameter int unsigned TC_BITS          = 7,
    parameter int unsigned HP_FOLDS         = 3,
    parameter int unsigned HP_PATH_BITS     = 16,
    parameter int unsigned HP_PCMIX         = 1,
    parameter int unsigned HP_PC_SHIFT      = 2,
    parameter bit          Y_REG            = 1'b0,
    parameter int unsigned PC_W             = 64
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

    function automatic int unsigned t_log(input int unsigned t);
        case (t)
            0: t_log = T0_LOG;  1: t_log = T1_LOG;  2: t_log = T2_LOG;  3: t_log = T3_LOG;
            4: t_log = T4_LOG;  5: t_log = T5_LOG;  6: t_log = T6_LOG;  7: t_log = T7_LOG;
            default: t_log = 0;
        endcase
    endfunction

    function automatic int unsigned t_wbits(input int unsigned t);
        case (t)
            0: t_wbits = T0_WBITS;  1: t_wbits = T1_WBITS;  2: t_wbits = T2_WBITS;  3: t_wbits = T3_WBITS;
            4: t_wbits = T4_WBITS;  5: t_wbits = T5_WBITS;  6: t_wbits = T6_WBITS;  7: t_wbits = T7_WBITS;
            default: t_wbits = 0;
        endcase
    endfunction

    function automatic int unsigned a_off(input int unsigned t);
        a_off = 0;
        for (int unsigned j = 0; j < t; j++) a_off = a_off + t_log(j);
    endfunction

    function automatic int unsigned d_off(input int unsigned t);
        d_off = 0;
        for (int unsigned j = 0; j < t; j++) d_off = d_off + t_wbits(j);
    endfunction

    localparam int unsigned BIAS_LOG = $clog2(BIAS_ENTRIES);
    localparam int unsigned ADDR_TOT = a_off(8);
    localparam int unsigned DATA_TOT = d_off(8);

    logic                        bias_re, bias_we;
    logic [        BIAS_LOG-1:0] bias_raddr, bias_waddr;
    logic [BIAS_WEIGHT_BITS-1:0] bias_rdata, bias_wdata;
    logic [      NUM_TABLES-1:0] wt_re, wt_we;
    logic [        ADDR_TOT-1:0] wt_raddr, wt_waddr;
    logic [        DATA_TOT-1:0] wt_rdata, wt_wdata;

    bp_hp_core #(
        .NUM_TABLES       (NUM_TABLES),
        .T0_LOG           (T0_LOG),
        .T1_LOG           (T1_LOG),
        .T2_LOG           (T2_LOG),
        .T3_LOG           (T3_LOG),
        .T4_LOG           (T4_LOG),
        .T5_LOG           (T5_LOG),
        .T6_LOG           (T6_LOG),
        .T7_LOG           (T7_LOG),
        .T0_WBITS         (T0_WBITS),
        .T1_WBITS         (T1_WBITS),
        .T2_WBITS         (T2_WBITS),
        .T3_WBITS         (T3_WBITS),
        .T4_WBITS         (T4_WBITS),
        .T5_WBITS         (T5_WBITS),
        .T6_WBITS         (T6_WBITS),
        .T7_WBITS         (T7_WBITS),
        .T0_HIST          (T0_HIST),
        .T1_HIST          (T1_HIST),
        .T2_HIST          (T2_HIST),
        .T3_HIST          (T3_HIST),
        .T4_HIST          (T4_HIST),
        .T5_HIST          (T5_HIST),
        .T6_HIST          (T6_HIST),
        .T7_HIST          (T7_HIST),
        .BIAS_ENTRIES     (BIAS_ENTRIES),
        .BIAS_WEIGHT_BITS (BIAS_WEIGHT_BITS),
        .THETA_BITS       (THETA_BITS),
        .TC_BITS          (TC_BITS),
        .HP_FOLDS         (HP_FOLDS),
        .HP_PATH_BITS     (HP_PATH_BITS),
        .HP_PCMIX         (HP_PCMIX),
        .HP_PC_SHIFT      (HP_PC_SHIFT),
        .Y_REG           (Y_REG),
        .PC_W            (PC_W)
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
        .bias_re_o        (bias_re),
        .bias_raddr_o     (bias_raddr),
        .bias_rdata_i     (bias_rdata),
        .bias_we_o        (bias_we),
        .bias_waddr_o     (bias_waddr),
        .bias_wdata_o     (bias_wdata),
        .wt_re_o          (wt_re),
        .wt_raddr_o       (wt_raddr),
        .wt_rdata_i       (wt_rdata),
        .wt_we_o          (wt_we),
        .wt_waddr_o       (wt_waddr),
        .wt_wdata_o       (wt_wdata)
    );

    bp_sram #(
        .DEPTH  (BIAS_ENTRIES),
        .WIDTH  (BIAS_WEIGHT_BITS)
    ) i_bias (
        .clk_i  (clk_i),
        .re_i   (bias_re),
        .raddr_i(bias_raddr),
        .rdata_o(bias_rdata),
        .we_i   (bias_we),
        .waddr_i(bias_waddr),
        .wdata_i(bias_wdata)
    );

    for (genvar t = 0; t < NUM_TABLES; t++) begin : g_wt
        localparam int unsigned LG = t_log(t);
        localparam int unsigned WB = t_wbits(t);
        bp_sram #(
            .DEPTH  (1 << LG),
            .WIDTH  (WB)
        ) i_wt (
            .clk_i  (clk_i),
            .re_i   (wt_re[t]),
            .raddr_i(wt_raddr[a_off(t) +: LG]),
            .rdata_o(wt_rdata[d_off(t) +: WB]),
            .we_i   (wt_we[t]),
            .waddr_i(wt_waddr[a_off(t) +: LG]),
            .wdata_i(wt_wdata[d_off(t) +: WB])
        );
    end

endmodule
