// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Integration of bp_tage_core with its tables as behavioral bp_sram models:
//   NB tagged banks (2^LOGG rows x 2 entries), the bimodal prediction bits
//   (2^LOGB x 1, reset 0) and the bimodal hysteresis (2^(LOGB-1) x 2,
//   reset 1), the C++ constructor's initial values. INTEGRATION ONLY: never
//   synthesize this module (bp_sram would become flip-flops). Synthesize
//   bp_tage_core and cost the SRAMs separately from the testbench access
//   counts. Same predict / update interface as the core.
//
// Parameters:
//   see bp_tage_core (NHIST, LOGT, LOGASSOC, LOGB, TBITS, CB_LMP, ILEN_CAP,
//   T1_HIST..T14_HIST, PC_W)
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_tage_top #(
    parameter int unsigned NHIST    = 12,
    parameter int unsigned LOGT     = 6,
    parameter int unsigned LOGASSOC = 0,
    parameter int unsigned LOGB     = 11,
    parameter int unsigned TBITS    = 10,
    parameter int unsigned CB_LMP   = 0,
    parameter int unsigned ILEN_CAP = 1,
    parameter int unsigned T1_HIST  = 4,
    parameter int unsigned T2_HIST  = 8,
    parameter int unsigned T3_HIST  = 20,
    parameter int unsigned T4_HIST  = 24,
    parameter int unsigned T5_HIST  = 28,
    parameter int unsigned T6_HIST  = 32,
    parameter int unsigned T7_HIST  = 36,
    parameter int unsigned T8_HIST  = 40,
    parameter int unsigned T9_HIST  = 44,
    parameter int unsigned T10_HIST = 48,
    parameter int unsigned T11_HIST = 80,
    parameter int unsigned T12_HIST = 104,
    parameter int unsigned T13_HIST = 0,
    parameter int unsigned T14_HIST = 0,
    parameter int unsigned PC_W     = 64
) (
    input  logic            clk_i,
    input  logic            rst_ni,

    input  logic            pred_req_valid_i,
    output logic            pred_req_ready_o,
    input  logic [PC_W-1:0] pred_req_pc_i,
    output logic            pred_resp_valid_o,
    output logic            pred_resp_taken_o,

    input  logic            upd_valid_i,
    output logic            upd_ready_o,
    input  logic            upd_is_cond_i,
    input  logic [PC_W-1:0] upd_pc_i,
    input  logic            upd_taken_i,
    input  logic [PC_W-1:0] upd_target_i
);

    localparam int unsigned LOGG   = LOGT - LOGASSOC;
    localparam int unsigned SH_OFF = 2 * ((NHIST / 2 + 1) / 2);
    localparam int unsigned NB     = (NHIST - SH_OFF) / 2 + SH_OFF / 2;
    localparam int unsigned ROW_W  = 2 * (TBITS + 5);

    logic [NB-1:0]            tb_re, tb_we;
    logic [NB-1:0][ LOGG-1:0] tb_raddr, tb_waddr;
    logic [NB-1:0][ROW_W-1:0] tb_rdata, tb_wdata;
    logic                     bp_re, bp_we, bp_rdata, bp_wdata;
    logic [      LOGB-1:0]    bp_raddr, bp_waddr;
    logic                     bh_re, bh_we;
    logic [      LOGB-2:0]    bh_raddr, bh_waddr;
    logic [           1:0]    bh_rdata, bh_wdata;

    bp_tage_core #(
        .NHIST   (NHIST),
        .LOGT    (LOGT),
        .LOGASSOC(LOGASSOC),
        .LOGB    (LOGB),
        .TBITS   (TBITS),
        .CB_LMP  (CB_LMP),
        .ILEN_CAP(ILEN_CAP),
        .T1_HIST (T1_HIST),
        .T2_HIST (T2_HIST),
        .T3_HIST (T3_HIST),
        .T4_HIST (T4_HIST),
        .T5_HIST (T5_HIST),
        .T6_HIST (T6_HIST),
        .T7_HIST (T7_HIST),
        .T8_HIST (T8_HIST),
        .T9_HIST (T9_HIST),
        .T10_HIST(T10_HIST),
        .T11_HIST(T11_HIST),
        .T12_HIST(T12_HIST),
        .T13_HIST(T13_HIST),
        .T14_HIST(T14_HIST),
        .PC_W    (PC_W)
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
        .tb_re_o          (tb_re),
        .tb_raddr_o       (tb_raddr),
        .tb_rdata_i       (tb_rdata),
        .tb_we_o          (tb_we),
        .tb_waddr_o       (tb_waddr),
        .tb_wdata_o       (tb_wdata),
        .bp_re_o          (bp_re),
        .bp_raddr_o       (bp_raddr),
        .bp_rdata_i       (bp_rdata),
        .bp_we_o          (bp_we),
        .bp_waddr_o       (bp_waddr),
        .bp_wdata_o       (bp_wdata),
        .bh_re_o          (bh_re),
        .bh_raddr_o       (bh_raddr),
        .bh_rdata_i       (bh_rdata),
        .bh_we_o          (bh_we),
        .bh_waddr_o       (bh_waddr),
        .bh_wdata_o       (bh_wdata)
    );

    for (genvar b = 0; b < NB; b++) begin : g_tb
        bp_sram #(
            .DEPTH(1 << LOGG),
            .WIDTH(ROW_W)
        ) i_tb (
            .clk_i  (clk_i),
            .re_i   (tb_re[b]),
            .raddr_i(tb_raddr[b]),
            .rdata_o(tb_rdata[b]),
            .we_i   (tb_we[b]),
            .waddr_i(tb_waddr[b]),
            .wdata_i(tb_wdata[b])
        );
    end

    bp_sram #(
        .DEPTH(1 << LOGB),
        .WIDTH(1),
        .INIT (1'b0)
    ) i_bp (
        .clk_i  (clk_i),
        .re_i   (bp_re),
        .raddr_i(bp_raddr),
        .rdata_o(bp_rdata),
        .we_i   (bp_we),
        .waddr_i(bp_waddr),
        .wdata_i(bp_wdata)
    );

    bp_sram #(
        .DEPTH(1 << (LOGB - 1)),
        .WIDTH(2),
        .INIT (2'b01)
    ) i_bh (
        .clk_i  (clk_i),
        .re_i   (bh_re),
        .raddr_i(bh_raddr),
        .rdata_o(bh_rdata),
        .we_i   (bh_we),
        .waddr_i(bh_waddr),
        .wdata_i(bh_wdata)
    );

endmodule