// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Self-checking testbench for bp_hp_core (hashed perceptron), driven by a
//   hashed_perceptron golden dump from the patched CBP5 harness. The DUT
//   instance is named dut and is the synthesized unit; the bias table and
//   the weight tables are behavioral bp_sram models instantiated here, next
//   to dut, so the same bench runs the RTL and, with POST_SYN_SIM, the
//   netlist. Stimulus, prediction check, drain, report and VCD come from
//   tb_bp_common.svh.
//
//   State trace (+STATE=<path> or STATE_FILE): the file hashed_perceptron.h
//   writes when run with HP_STATE=<path>, one line per UpdatePredictor
//   (kind 1) / TrackOtherInst (kind 0), in golden-dump order:
//     kind pred sum train theta tc bidx i0 .. i7 phist ghist ckv cks
//   Sampled at the falling edge (bench inputs and DUT state both stable): a
//   conditional line is checked when the core is idle again after its
//   update, an unconditional line at its handshake (the core ignores it).
//   The first difference is fatal (branch number and every differing
//   field). Compared:
//     always    kind, train (a write-back happened), and the weight checksum
//               when ckv = 1 (on a shadow copy of every table kept from the
//               write ports, so it also runs on the netlist)
//     RTL only  pred, sum, bias / table indices, theta, tc, path history,
//               global history (newest min(64, longest history) bits)
//   With no state file only the predictions are checked.
//
//   Parameters must match the dump's hashed_perceptron build; T<i>_HIST are
//   the PREDICTOR_HIST_LENS values of its CBP5 .log.
//
// Parameters:
//   NUM_TABLES .. HP_PC_SHIFT - as bp_hp_core (must match the DUT and dump)
//   Y_REG                     - DUT prediction timing (RTL only)
//   STIM_FILE                 - default golden dump ("" = +STIM=<path>)
//   STATE_FILE                - default state trace ("" = +STATE=<path> or none)
//   MAX_LINES                 - stop after this many branch lines (0 = all)
//   TIMEOUT_CYCLES            - max cycles to wait for a ready or valid
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

`ifndef CLK_PERIOD_NS
`define CLK_PERIOD_NS 10
`endif

`ifdef POST_SYN_SIM
// The gate-level flows compile only the netlist, the cell models and the
// bench; bp_sram is a bench-side model, so the bench pulls it in (path
// relative to scripts/<flow>/).
`include "../../projects/branch-prediction/rtl/bp_sram.sv"
`endif

/* verilator lint_off UNUSEDSIGNAL */

module tb_bp_hp_core #(
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
    parameter string       STIM_FILE        = "",
    parameter string       STATE_FILE       = "",
    parameter int unsigned MAX_LINES        = 0,
    parameter int unsigned TIMEOUT_CYCLES   = 100
);

    localparam string       DUT_NAME = "bp_hp_core";
    localparam int unsigned PC_W     = 64;

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

    function automatic int unsigned t_hist(input int unsigned t);
        case (t)
            0: t_hist = T0_HIST;  1: t_hist = T1_HIST;  2: t_hist = T2_HIST;  3: t_hist = T3_HIST;
            4: t_hist = T4_HIST;  5: t_hist = T5_HIST;  6: t_hist = T6_HIST;  7: t_hist = T7_HIST;
            default: t_hist = 0;
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

    // first shadow entry of table t (the bias occupies 0 .. BIAS_ENTRIES-1)
    function automatic int unsigned e_base(input int unsigned t);
        e_base = BIAS_ENTRIES;
        for (int unsigned j = 0; j < t; j++) e_base = e_base + (1 << t_log(j));
    endfunction

    localparam int unsigned BIAS_LOG = $clog2(BIAS_ENTRIES);
    localparam int unsigned ADDR_TOT = a_off(8);
    localparam int unsigned DATA_TOT = d_off(8);
    localparam int unsigned GH_LEN   = t_hist(NUM_TABLES - 1);
    localparam int unsigned SH_N     = e_base(NUM_TABLES);

    logic                        clk_i;
    logic                        rst_ni;

    logic                        pred_req_valid;
    logic                        pred_req_ready;
    logic [            PC_W-1:0] pred_req_pc;
    logic                        pred_resp_valid;
    logic                        pred_resp_taken;

    logic                        upd_valid;
    logic                        upd_ready;
    logic                        upd_is_cond;
    logic [            PC_W-1:0] upd_pc;
    logic                        upd_taken;
    logic [            PC_W-1:0] upd_target;

    logic                        bias_re, bias_we;
    logic [        BIAS_LOG-1:0] bias_raddr, bias_waddr;
    logic [BIAS_WEIGHT_BITS-1:0] bias_rdata, bias_wdata;
    logic [      NUM_TABLES-1:0] wt_re, wt_we;
    logic [        ADDR_TOT-1:0] wt_raddr, wt_waddr;
    logic [        DATA_TOT-1:0] wt_rdata, wt_wdata;

    logic [                 7:0] sram_rd_now;
    logic [                 7:0] sram_wr_now;

    // -------------------------------------------------------------------------
    // DUT and tables
    // -------------------------------------------------------------------------
`ifdef POST_SYN_SIM
    bp_hp_core dut (
`else
    bp_hp_core #(
        .NUM_TABLES        (NUM_TABLES),
        .T0_LOG            (T0_LOG),
        .T1_LOG            (T1_LOG),
        .T2_LOG            (T2_LOG),
        .T3_LOG            (T3_LOG),
        .T4_LOG            (T4_LOG),
        .T5_LOG            (T5_LOG),
        .T6_LOG            (T6_LOG),
        .T7_LOG            (T7_LOG),
        .T0_WBITS          (T0_WBITS),
        .T1_WBITS          (T1_WBITS),
        .T2_WBITS          (T2_WBITS),
        .T3_WBITS          (T3_WBITS),
        .T4_WBITS          (T4_WBITS),
        .T5_WBITS          (T5_WBITS),
        .T6_WBITS          (T6_WBITS),
        .T7_WBITS          (T7_WBITS),
        .T0_HIST           (T0_HIST),
        .T1_HIST           (T1_HIST),
        .T2_HIST           (T2_HIST),
        .T3_HIST           (T3_HIST),
        .T4_HIST           (T4_HIST),
        .T5_HIST           (T5_HIST),
        .T6_HIST           (T6_HIST),
        .T7_HIST           (T7_HIST),
        .BIAS_ENTRIES      (BIAS_ENTRIES),
        .BIAS_WEIGHT_BITS  (BIAS_WEIGHT_BITS),
        .THETA_BITS        (THETA_BITS),
        .TC_BITS           (TC_BITS),
        .HP_FOLDS          (HP_FOLDS),
        .HP_PATH_BITS      (HP_PATH_BITS),
        .HP_PCMIX          (HP_PCMIX),
        .HP_PC_SHIFT       (HP_PC_SHIFT),
        .Y_REG            (Y_REG),
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

    assign sram_rd_now = 8'(bias_re) + 8'($countones(wt_re));
    assign sram_wr_now = 8'(bias_we) + 8'($countones(wt_we));

    // -------------------------------------------------------------------------
    // Bench-specific strings for the shared body
    // -------------------------------------------------------------------------
    function automatic string tb_params();
        return $sformatf("NUM_TABLES=%0d logs=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d wbits=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d bias=%0dx%0d folds=%0d path=%0d pcmix=%0d Y_REG=%0d",
                         NUM_TABLES, T0_LOG, T1_LOG, T2_LOG, T3_LOG, T4_LOG, T5_LOG, T6_LOG, T7_LOG,
                         T0_WBITS, T1_WBITS, T2_WBITS, T3_WBITS, T4_WBITS, T5_WBITS, T6_WBITS, T7_WBITS,
                         BIAS_ENTRIES, BIAS_WEIGHT_BITS, HP_FOLDS, HP_PATH_BITS, HP_PCMIX, Y_REG);
    endfunction

    function automatic string tb_debug();
`ifdef POST_SYN_SIM
        return "";
`else
        // y_comb, not y_q: with Y_REG = 0 the response comes from y_comb
        return $sformatf("y %0d theta %0d tc %0d bias idx %0d ghist %h",
                         $signed(dut.y_comb), dut.theta_q, $signed(dut.tc_q), dut.bidx_q, dut.gh_q);
`endif
    endfunction

    `include "../../projects/branch-prediction/tb/tb_bp_common.svh"

    // -------------------------------------------------------------------------
    // Shadow copy of every weight (bias at 0.., table t at e_base(t)..)
    // -------------------------------------------------------------------------
    logic [7:0] sh [SH_N];

    initial begin
        for (int unsigned i = 0; i < SH_N; i++) sh[i] = 8'd0;
    end

    always @(posedge clk_i) begin
        if (bias_we) sh[32'(bias_waddr)] <= 8'(bias_wdata);
        for (int unsigned t = 0; t < NUM_TABLES; t++) begin
            if (wt_we[t])
                sh[e_base(t) + 32'((wt_waddr >> a_off(t)) & ((1 << t_log(t)) - 1))]
                    <= 8'((wt_wdata >> d_off(t)) & ((1 << t_wbits(t)) - 1));
        end
    end

    // hp_core.h weight_checksum(): sum over arrays a (0 = bias, 1 + t) and
    // entries e of (w & (2^width - 1)) * (2 * ((a << 24) | e) + 1), mod 2^32
    function automatic int unsigned weight_checksum();
        int unsigned c;
        c = 0;
        for (int unsigned e = 0; e < BIAS_ENTRIES; e++)
            c += 32'(sh[e]) * (2 * e + 1);
        for (int unsigned t = 0; t < NUM_TABLES; t++)
            for (int unsigned e = 0; e < (1 << t_log(t)); e++)
                c += 32'(sh[e_base(t) + e]) * (2 * (((t + 1) << 24) | e) + 1);
        return c;
    endfunction

    // -------------------------------------------------------------------------
    // Per-branch state check against the hashed_perceptron.h state trace
    // -------------------------------------------------------------------------
    int              st_fd     = 0;
    longint unsigned st_lines  = 0;
    longint unsigned st_cksums = 0;
    logic            pend_cond = 1'b0;   // conditional update in progress
    logic            saw_wr    = 1'b0;   // ... and it wrote the tables

    function automatic void chk(inout string bad, input string name, input longint rtl,
                                input longint ref_v);
        if (rtl != ref_v) bad = {bad, $sformatf(" %s rtl=%0d ref=%0d", name, rtl, ref_v)};
    endfunction

    task automatic check_state(input logic kind_rtl, input logic train_rtl);
        string           line;
        string           bad;
        int              nf;
        int              f_kind, f_pred, f_sum, f_train, f_theta, f_tc, f_ckv;
        int unsigned     f_bidx, f_phist, f_cks;
        int unsigned     f_i [8];
        longint unsigned f_gh, gh_mask;
        int unsigned     ghl;

        line = "";
        do begin
            if ($fgets(line, st_fd) == 0) fail("state trace ended before the golden dump");
        end while (line.len() < 2 || line.getc(0) == "#");

        nf = $sscanf(line, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %h %h %d %h",
                     f_kind, f_pred, f_sum, f_train, f_theta, f_tc, f_bidx,
                     f_i[0], f_i[1], f_i[2], f_i[3], f_i[4], f_i[5], f_i[6], f_i[7],
                     f_phist, f_gh, f_ckv, f_cks);
        st_lines++;
        if (nf != 19) fail($sformatf("state trace line %0d: expected 19 fields, got %0d", st_lines, nf));

        bad = "";
        chk(bad, "kind", longint'(kind_rtl), longint'(f_kind));
        if (kind_rtl) chk(bad, "train", longint'(train_rtl), longint'(f_train));
`ifndef POST_SYN_SIM
        if (kind_rtl) begin
            chk(bad, "pred", longint'(~dut.y_q[$bits(dut.y_q)-1]), longint'(f_pred));
            chk(bad, "sum",  longint'($signed(dut.y_q)),           longint'(f_sum));
            chk(bad, "bidx", longint'(dut.bidx_q),                 longint'(f_bidx));
            for (int unsigned t = 0; t < NUM_TABLES; t++)
                chk(bad, $sformatf("idx%0d", t),
                    longint'((dut.idx_q >> a_off(t)) & ((1 << t_log(t)) - 1)), longint'(f_i[t]));
        end
        chk(bad, "theta", longint'(dut.theta_q),        longint'(f_theta));
        chk(bad, "tc",    longint'($signed(dut.tc_q)),  longint'(f_tc));
        chk(bad, "phist", (HP_PATH_BITS > 0) ? longint'(dut.phist_q) : 64'sd0, longint'(f_phist));
        ghl     = (GH_LEN < 64) ? GH_LEN : 64;
        gh_mask = (ghl == 64) ? '1 : ((64'd1 << ghl) - 64'd1);
        chk(bad, "ghist", longint'(64'(dut.gh_q) & gh_mask), longint'(f_gh & gh_mask));
`endif
        if (f_ckv != 0) begin
            st_cksums++;
            chk(bad, "checksum", longint'(weight_checksum()), longint'(f_cks));
        end

        if (bad != "")
            fail($sformatf("STATE MISMATCH branch %0d (%s):%s", st_lines,
                           kind_rtl ? "conditional" : "unconditional", bad));
    endtask

    initial begin
        string st_path;
        st_path = STATE_FILE;
        void'($value$plusargs("STATE=%s", st_path));
        if (st_path != "") begin
            st_fd = $fopen(st_path, "r");
            if (st_fd == 0) fail($sformatf("cannot open state trace '%s'", st_path));
            $display("State trace: %s\n", st_path);
        end
    end

    // Falling edge: the bench drives its inputs T_SETTLE after the rising
    // edge and the DUT changes only on rising edges, so both are stable here.
    always @(negedge clk_i) begin
        if (counting && st_fd != 0) begin
            if (pend_cond) begin
                if (bias_we) saw_wr = 1'b1;
                if (pred_req_ready) begin            // update finished
                    check_state(1'b1, saw_wr);
                    pend_cond = 1'b0;
                    saw_wr    = 1'b0;
                end
            end
            if (upd_valid && upd_ready) begin        // handshake on the next edge
                if (upd_is_cond) pend_cond = 1'b1;
                else             check_state(1'b0, 1'b0);
            end
        end
    end

    final begin
        if (st_fd != 0) begin
            $display("TB_STATE_LINES        : %0d", st_lines);
            $display("TB_STATE_CKSUMS       : %0d", st_cksums);
        end
    end

endmodule
