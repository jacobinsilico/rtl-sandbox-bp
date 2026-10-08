// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Self-checking testbench for bp_tage_core. The stimulus, prediction check,
//   drain and report come from tb_bp_common.svh (golden dump of the patched
//   CBP5 harness, +STIM=<path>). The banks are behavioral bp_sram models
//   instantiated here, next to dut (instance name dut, the synthesized unit).
//
//   State trace (+STATE=<path> or STATE_FILE): the file tage_cb.h writes when
//   run with TAGE_CB_STATE=<path>, one line per UpdatePredictor (kind 1) /
//   TrackOtherInst (kind 0), in golden-dump order:
//     kind hit alt hc lmp hcpred altt tpred pred pweak alloc na pen rng
//     seed tick cm11 clc uaon c50 c1631 phist ptghist numero pcblock ckv cks
//   Each time the core returns to idle after an update (pred_req_ready
//   rises) the next line is compared with the core's state, and the first
//   difference is fatal (branch number and every differing field). Compared:
//     always      kind; the table checksum when ckv = 1 (computed on a shadow
//                 copy of every SRAM kept from the write ports, so it also
//                 runs on the netlist)
//     RTL only    prediction decisions (kind 1), ALLOC / NA / Penalty, the
//                 number of MYRANDOM calls (cycles with dut.rng_en), Seed,
//                 TICK, CountMiss11, CountLowConf, use_alt_on_na, COUNT50,
//                 COUNT16_31, phist, ptghist, Numero, PCBLOCK
//   TB_STATE_LINES / TB_STATE_CKSUMS are printed at the end. With no state
//   file only the predictions are checked.
//
//   Parameters must match the dump's tage_cb build; T<i>_HIST are the
//   PREDICTOR_HIST_LENS values of its CBP5 .log.
//
// Parameters:
//   NHIST, LOGT, LOGASSOC, LOGB, TBITS, CB_LMP, ILEN_CAP, T1_HIST..T14_HIST
//                    - as bp_tage_core
//   STIM_FILE        - default golden dump ("" = +STIM=<path> required)
//   STATE_FILE       - default state trace ("" = +STATE=<path> or none)
//   MAX_LINES        - stop after this many branch lines (0 = whole file)
//   TIMEOUT_CYCLES   - max cycles to wait for a ready or valid (a u-reset
//                      sweep alone takes 2^LOGG + 1 cycles)
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

module tb_bp_tage_core #(
    parameter int unsigned NHIST          = 12,
    parameter int unsigned LOGT           = 6,
    parameter int unsigned LOGASSOC       = 0,
    parameter int unsigned LOGB           = 11,
    parameter int unsigned TBITS          = 10,
    parameter int unsigned CB_LMP         = 0,
    parameter int unsigned ILEN_CAP       = 1,
    parameter int unsigned T1_HIST        = 4,
    parameter int unsigned T2_HIST        = 8,
    parameter int unsigned T3_HIST        = 20,
    parameter int unsigned T4_HIST        = 24,
    parameter int unsigned T5_HIST        = 28,
    parameter int unsigned T6_HIST        = 32,
    parameter int unsigned T7_HIST        = 36,
    parameter int unsigned T8_HIST        = 40,
    parameter int unsigned T9_HIST        = 44,
    parameter int unsigned T10_HIST       = 48,
    parameter int unsigned T11_HIST       = 80,
    parameter int unsigned T12_HIST       = 104,
    parameter int unsigned T13_HIST       = 0,
    parameter int unsigned T14_HIST       = 0,
    parameter string       STIM_FILE      = "",
    parameter string       STATE_FILE     = "",
    parameter int unsigned MAX_LINES      = 0,
    parameter int unsigned TIMEOUT_CYCLES = 4096
);

    localparam int unsigned PC_W     = 64;
    localparam string       DUT_NAME = "bp_tage_core";

    localparam int unsigned LOGG     = LOGT - LOGASSOC;
    localparam int unsigned SH_OFF   = 2 * ((NHIST / 2 + 1) / 2);
    localparam int unsigned NDPAIR   = (NHIST - SH_OFF) / 2;
    localparam int unsigned NB       = NDPAIR + SH_OFF / 2;
    localparam int unsigned E_W      = TBITS + 5;
    localparam int unsigned ROW_W    = 2 * E_W;
    localparam int unsigned ROWS     = 1 << LOGG;
    localparam int unsigned BIM_N    = 1 << LOGB;
    localparam int unsigned HYS_N    = 1 << (LOGB - 1);

    logic                     clk_i;
    logic                     rst_ni;

    logic                     pred_req_valid;
    logic                     pred_req_ready;
    logic [       PC_W-1:0]   pred_req_pc;
    logic                     pred_resp_valid;
    logic                     pred_resp_taken;

    logic                     upd_valid;
    logic                     upd_ready;
    logic                     upd_is_cond;
    logic [       PC_W-1:0]   upd_pc;
    logic                     upd_taken;
    logic [       PC_W-1:0]   upd_target;

    logic [NB-1:0]            tb_re, tb_we;
    logic [NB-1:0][ LOGG-1:0] tb_raddr, tb_waddr;
    logic [NB-1:0][ROW_W-1:0] tb_rdata, tb_wdata;
    logic                     bp_re, bp_we, bp_rdata, bp_wdata;
    logic [       LOGB-1:0]   bp_raddr, bp_waddr;
    logic                     bh_re, bh_we;
    logic [       LOGB-2:0]   bh_raddr, bh_waddr;
    logic [            1:0]   bh_rdata, bh_wdata;

    logic [            7:0]   sram_rd_now, sram_wr_now;

    // -------------------------------------------------------------------------
    // DUT and tables
    // -------------------------------------------------------------------------
`ifdef POST_SYN_SIM
    bp_tage_core dut (
`else
    bp_tage_core #(
        .NHIST            (NHIST),
        .LOGT             (LOGT),
        .LOGASSOC         (LOGASSOC),
        .LOGB             (LOGB),
        .TBITS            (TBITS),
        .CB_LMP           (CB_LMP),
        .ILEN_CAP         (ILEN_CAP),
        .T1_HIST          (T1_HIST),
        .T2_HIST          (T2_HIST),
        .T3_HIST          (T3_HIST),
        .T4_HIST          (T4_HIST),
        .T5_HIST          (T5_HIST),
        .T6_HIST          (T6_HIST),
        .T7_HIST          (T7_HIST),
        .T8_HIST          (T8_HIST),
        .T9_HIST          (T9_HIST),
        .T10_HIST         (T10_HIST),
        .T11_HIST         (T11_HIST),
        .T12_HIST         (T12_HIST),
        .T13_HIST         (T13_HIST),
        .T14_HIST         (T14_HIST),
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
            .DEPTH(ROWS),
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
        .DEPTH(BIM_N),
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
        .DEPTH(HYS_N),
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

    assign sram_rd_now = 8'($countones(tb_re)) + 8'(bp_re) + 8'(bh_re);
    assign sram_wr_now = 8'($countones(tb_we)) + 8'(bp_we) + 8'(bh_we);

    // -------------------------------------------------------------------------
    // Hooks for tb_bp_common.svh
    // -------------------------------------------------------------------------
    function automatic string tb_params();
        return $sformatf("NHIST=%0d LOGT=%0d LOGASSOC=%0d LOGB=%0d TBITS=%0d CB_LMP=%0d banks=%0d x %0d x %0d",
                         NHIST, LOGT, LOGASSOC, LOGB, TBITS, CB_LMP, NB, ROWS, ROW_W);
    endfunction

    function automatic string tb_debug();
`ifndef POST_SYN_SIM
        return $sformatf("hit %0d alt %0d hc %0d weak %0d numero %0d pcblock %h seed %h",
                         dut.p_hit, dut.p_alt, dut.p_hc, dut.p_pweak, dut.num_q, dut.pcb_q,
                         dut.seed_q);
`else
        return "";
`endif
    endfunction

    `include "../../projects/branch-prediction/tb/tb_bp_common.svh"

    // -------------------------------------------------------------------------
    // Shadow copy of every table, from the SRAM write ports (for the checksum)
    // -------------------------------------------------------------------------
    logic [ROW_W-1:0] sh_tb [NB][ROWS];
    logic             sh_bp [BIM_N];
    logic [      1:0] sh_bh [HYS_N];

    initial begin
        for (int unsigned b = 0; b < NB; b++)
            for (int unsigned r = 0; r < ROWS; r++) sh_tb[b][r] = '0;
        for (int unsigned i = 0; i < BIM_N; i++) sh_bp[i] = 1'b0;
        for (int unsigned i = 0; i < HYS_N; i++) sh_bh[i] = 2'b01;
    end

    always @(posedge clk_i) begin
        for (int unsigned b = 0; b < NB; b++)
            if (tb_we[b]) sh_tb[b][tb_waddr[b]] <= tb_wdata[b];
        if (bp_we) sh_bp[bp_waddr] <= bp_wdata;
        if (bh_we) sh_bh[bh_waddr] <= bh_wdata;
    end

    // tage_cb.h table_checksum(): order-independent, mod 2^32
    //   sum ((tag << 5) | (u << 3) | ctr) * (2 * ((array << 20) | entry) + 1)
    //   + sum (pred | hyst << 1) * (2 * (((NARRAYS + 1) << 20) | i) + 1)
    function automatic int unsigned table_checksum();
        int unsigned c;
        c = 0;
        for (int unsigned b = 0; b < NB; b++) begin
            for (int unsigned r = 0; r < ROWS; r++) begin
                for (int unsigned s = 0; s < 2; s++) begin
                    logic [E_W-1:0] e;
                    int unsigned    a, idx, v;
                    e = sh_tb[b][r][s*E_W +: E_W];
                    if (b < 2 * NDPAIR) begin
                        a   = 2 * (b / 2 + 1) - 1 + s;    // doubled pair, half b % 2
                        idx = (r << 1) | (b % 2);
                    end else begin
                        a   = 2 * (b - NDPAIR + 1) - 1 + s;
                        idx = r;
                    end
                    v = (32'(e[E_W-1:5]) << 5) | (32'(e[4:3]) << 3) | 32'(e[2:0]);
                    c += v * (2 * ((a << 20) | idx) + 1);
                end
            end
        end
        for (int unsigned i = 0; i < BIM_N; i++) begin
            int unsigned v;
            v = 32'(sh_bp[i]);
            if (i < HYS_N) v |= 32'(sh_bh[i]) << 1;
            c += v * (2 * (((SH_OFF + 1) << 20) | i) + 1);
        end
        return c;
    endfunction

    // -------------------------------------------------------------------------
    // Per-branch state check against the tage_cb.h state trace
    // -------------------------------------------------------------------------
    int              st_fd     = 0;
    longint unsigned st_lines  = 0;
    longint unsigned st_cksums = 0;
    int unsigned     rng_cnt   = 0;
    logic            was_busy  = 1'b0;
    logic            saw_pred  = 1'b0;

    function automatic void chk(inout string bad, input string name, input longint rtl,
                                input longint ref_v, input bit hex);
        if (rtl != ref_v) begin
            if (hex) bad = {bad, $sformatf(" %s rtl=%0h ref=%0h", name, rtl, ref_v)};
            else     bad = {bad, $sformatf(" %s rtl=%0d ref=%0d", name, rtl, ref_v)};
        end
    endfunction

    task automatic check_state(input logic kind_rtl, input int unsigned rngc);
        string       line;
        string       bad;
        int          nf;
        int          f_kind, f_hit, f_alt, f_hc, f_lmp, f_hcp, f_altt, f_tpred, f_pred;
        int          f_pweak, f_alloc, f_na, f_pen, f_rng;
        int unsigned f_seed, f_c50, f_c1631, f_phist, f_ptg, f_pcb, f_cks;
        int          f_tick, f_cm11, f_clc, f_uaon, f_num, f_ckv;

        line = "";
        do begin
            if ($fgets(line, st_fd) == 0) fail("state trace ended before the golden dump");
        end while (line.len() < 2 || line.getc(0) == "#");

        nf = $sscanf(line, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %h %d %d %d %d %h %h %h %h %d %h %d %h",
                     f_kind, f_hit, f_alt, f_hc, f_lmp, f_hcp, f_altt, f_tpred, f_pred, f_pweak,
                     f_alloc, f_na, f_pen, f_rng, f_seed, f_tick, f_cm11, f_clc, f_uaon,
                     f_c50, f_c1631, f_phist, f_ptg, f_num, f_pcb, f_ckv, f_cks);
        st_lines++;
        if (nf != 27) fail($sformatf("state trace line %0d: expected 27 fields, got %0d", st_lines, nf));

        bad = "";
        chk(bad, "kind", longint'(kind_rtl), longint'(f_kind), 1'b0);
`ifndef POST_SYN_SIM
        if (kind_rtl) begin
            chk(bad, "hit",    longint'(dut.hit_q),   longint'(f_hit),   1'b0);
            chk(bad, "alt",    longint'(dut.alt_q),   longint'(f_alt),   1'b0);
            chk(bad, "hc",     longint'(dut.hc_q),    longint'(f_hc),    1'b0);
            chk(bad, "lmp",    longint'(dut.lmp_q),   longint'(f_lmp),   1'b0);
            chk(bad, "hcpred", longint'(dut.hcp_q),   longint'(f_hcp),   1'b0);
            chk(bad, "altt",   longint'(dut.altt_q),  longint'(f_altt),  1'b0);
            chk(bad, "tpred",  longint'(dut.tpred_q), longint'(f_tpred), 1'b0);
            chk(bad, "pred",   longint'((CB_LMP != 0) ? dut.lmp_q : dut.tpred_q), longint'(f_pred), 1'b0);
            chk(bad, "pweak",  longint'(dut.pweak_q), longint'(f_pweak), 1'b0);
            chk(bad, "alloc",  longint'(dut.alloc_q), longint'(f_alloc), 1'b0);
            chk(bad, "na",     dut.alloc_q ? longint'(dut.na_q)  : 64'sd0, longint'(f_na),  1'b0);
            chk(bad, "pen",    dut.alloc_q ? longint'(dut.pen_q) : 64'sd0, longint'(f_pen), 1'b0);
            chk(bad, "rng",    longint'(rngc),        longint'(f_rng),   1'b0);
        end
        chk(bad, "seed",    longint'(dut.seed_q),             longint'(f_seed),  1'b1);
        chk(bad, "tick",    longint'(dut.tick_q),             longint'(f_tick),  1'b0);
        chk(bad, "cm11",    longint'($signed(dut.cm11_q)),    longint'(f_cm11),  1'b0);
        chk(bad, "clc",     longint'($signed(dut.clc_q)),     longint'(f_clc),   1'b0);
        chk(bad, "uaon",    longint'($signed(dut.uaon_q)),    longint'(f_uaon),  1'b0);
        chk(bad, "c50",     longint'(dut.c50_q),              longint'(f_c50),   1'b1);
        chk(bad, "c1631",   longint'(dut.c1631_q),            longint'(f_c1631), 1'b1);
        chk(bad, "phist",   longint'(dut.phist_q),            longint'(f_phist), 1'b1);
        chk(bad, "ptghist", longint'(dut.ptg_q),              longint'(f_ptg),   1'b1);
        chk(bad, "numero",  longint'(dut.num_q),              longint'(f_num),   1'b0);
        chk(bad, "pcblock", longint'(dut.pcb_q),              longint'(f_pcb),   1'b1);
`endif
        if (f_ckv != 0) begin
            st_cksums++;
            chk(bad, "checksum", longint'(table_checksum()), longint'(f_cks), 1'b1);
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

    // Sampled T_SETTLE after each rising edge, like the rest of the bench.
    // pred_req_ready rising = an update (conditional or not) has completed;
    // pred_resp_valid seen since the last completion = it was conditional.
    always @(posedge clk_i) begin
        #(T_SETTLE);
        if (counting && st_fd != 0) begin
`ifndef POST_SYN_SIM
            rng_cnt += 32'(dut.rng_en);
`endif
            if (pred_resp_valid) saw_pred = 1'b1;
            if (pred_req_ready && was_busy) begin
                check_state(saw_pred, rng_cnt);
                rng_cnt  = 0;
                saw_pred = 1'b0;
            end
            was_busy = !pred_req_ready;
        end
    end

    final begin
        if (st_fd != 0) begin
            $display("TB_STATE_LINES        : %0d", st_lines);
            $display("TB_STATE_CKSUMS       : %0d", st_cksums);
        end
    end

endmodule