// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Self-checking testbench for bp_gshare_core, driven by a golden dump from
//   the patched CBP5 harness (or a handwritten file in the same format), one
//   branch per line:
//     is_cond pc taken target ref_pred        (pc and target in hex)
//   Lines starting with '#' are skipped. For every conditional branch the
//   bench requests a prediction, checks it against ref_pred (the C++
//   prediction) and sends the update; unconditional branches are sent as
//   TrackOtherInst updates. The first mismatch is reported with its line
//   number and PC (in RTL simulation also the GHR and index) and is fatal.
//
//   The DUT instance is named dut and is the synthesized unit. The PHT is the
//   behavioral bp_sram instantiated here, next to dut, so the same bench runs
//   the RTL and, with POST_SYN_SIM, the synthesized/routed netlist (whose
//   ports are plain vectors, so the branch only drops the parameters). Dumps
//   activity.vcd of dut when compiled with VCD; bound the run with MAX_LINES
//   for gate-level power runs.
//
//   Inputs are driven T_SETTLE after a rising edge and outputs are sampled
//   T_SETTLE after a rising edge, so the DUT always samples stable values.
//
//   Report: KEY : value lines, then PASSED or a fatal error. TB_RTL_MISPRED
//   must equal TB_REF_MISPRED. TB_SRAM_READS / TB_SRAM_WRITES are the PHT
//   access counts for the SRAM energy estimate (the SRAM is outside dut, so
//   the power analysis does not see it).
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
//   TIMEOUT_CYCLES   - max cycles to wait for a ready or valid
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

`ifndef CLK_PERIOD_NS
`define CLK_PERIOD_NS 10
`endif

`ifdef POST_SYN_SIM
// The gate-level flows compile only the netlist, the cell models and the
// bench, not the project RTL. bp_sram is a bench-side model (never part of
// the netlist), so the bench pulls it in itself. Verilator resolves the path
// relative to the flow's run directory (scripts/post-syn-sim/ or
// scripts/post-pnr-sim/), hence the two levels up to the repository root.
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
    parameter int unsigned TIMEOUT_CYCLES = 100
);

    localparam real CLK_PERIOD = `CLK_PERIOD_NS;
    localparam real CLK_HALF   = CLK_PERIOD / 2.0;
    localparam real T_SETTLE   = CLK_PERIOD / 10.0;

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

    // Activity counters. counting is set when reset is released; using it
    // instead of rst_ni keeps the bench's own logic off the DUT's async reset.
    logic            counting  = 1'b0;
    longint unsigned n_cycles  = 0;
    longint unsigned n_sram_rd = 0;
    longint unsigned n_sram_wr = 0;

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

    initial clk_i = 1'b0;
    always #(CLK_HALF) clk_i = ~clk_i;

    // -------------------------------------------------------------------------
    // Activity counters (values the rising edge acts on)
    // -------------------------------------------------------------------------
    always @(posedge clk_i) begin
        if (counting) begin
            n_cycles  <= n_cycles  + 64'd1;
            n_sram_rd <= n_sram_rd + 64'(pht_re);
            n_sram_wr <= n_sram_wr + 64'(pht_we);
        end
    end

    // -------------------------------------------------------------------------
    // Transactions. Every task starts and ends T_SETTLE after a rising edge.
    // -------------------------------------------------------------------------
    task automatic step;
        @(posedge clk_i);
        #(T_SETTLE);
    endtask

    task automatic fail(input string msg);
`ifdef VCD
        $dumpoff;
`endif
        $error("%s", msg);
        $fatal;
    endtask

    task automatic predict(input logic [PC_W-1:0] pc, output logic taken);
        int unsigned waited;
        waited = 0;
        while (!pred_req_ready) begin
            step;
            if (++waited > TIMEOUT_CYCLES) fail("timeout waiting for pred_req_ready");
        end
        pred_req_valid = 1'b1;
        pred_req_pc    = pc;
        step;                                 // handshake on this edge
        pred_req_valid = 1'b0;
        waited = 0;
        while (!pred_resp_valid) begin
            step;
            if (++waited > TIMEOUT_CYCLES) fail("timeout waiting for pred_resp_valid");
        end
        taken = pred_resp_taken;
    endtask

    task automatic update(input logic            is_cond,
                          input logic [PC_W-1:0] pc,
                          input logic            taken,
                          input logic [PC_W-1:0] target);
        int unsigned waited;
        waited = 0;
        while (!upd_ready) begin
            step;
            if (++waited > TIMEOUT_CYCLES) fail("timeout waiting for upd_ready");
        end
        upd_valid   = 1'b1;
        upd_is_cond = is_cond;
        upd_pc      = pc;
        upd_taken   = taken;
        upd_target  = target;
        step;                                 // handshake on this edge
        upd_valid   = 1'b0;
    endtask

    // -------------------------------------------------------------------------
    // Stimulus, checking and report
    // -------------------------------------------------------------------------
    initial begin
        string           stim;
        string           line;
        int              fd;
        int              nfields;
        int unsigned     lineno;
        int              f_is_cond;
        int              f_taken;
        int              f_ref;
        logic [PC_W-1:0] f_pc;
        logic [PC_W-1:0] f_target;
        logic            rtl_taken;
        longint unsigned n_lines;
        longint unsigned n_cond;
        longint unsigned n_uncond;
        longint unsigned n_mis_rtl;
        longint unsigned n_mis_ref;

        stim = STIM_FILE;
        void'($value$plusargs("STIM=%s", stim));
        if (stim == "") fail("no stimulus file: pass +STIM=<path> or set STIM_FILE");
        fd = $fopen(stim, "r");
        if (fd == 0) fail($sformatf("cannot open stimulus file '%s'", stim));

        $display("\nStarting bp_gshare_core verification (GHR_BITS=%0d INDEX_BITS=%0d PC_SHIFT=%0d CTR_BITS=%0d)",
                 GHR_BITS, INDEX_BITS, PC_SHIFT, CTR_BITS);
        $display("Stimulus: %s\n", stim);
`ifdef VCD
        $dumpfile("activity.vcd");
        $dumpvars(0, dut);
`endif

        rst_ni         = 1'b0;
        pred_req_valid = 1'b0;
        pred_req_pc    = '0;
        upd_valid      = 1'b0;
        upd_is_cond    = 1'b0;
        upd_pc         = '0;
        upd_taken      = 1'b0;
        upd_target     = '0;
        n_lines        = 0;
        n_cond         = 0;
        n_uncond       = 0;
        n_mis_rtl      = 0;
        n_mis_ref      = 0;
        lineno         = 0;
        repeat (3) @(posedge clk_i);
        #(T_SETTLE);
        rst_ni   = 1'b1;
        counting = 1'b1;

        while ($fgets(line, fd) != 0) begin
            lineno++;
            if (line.len() == 0 || line.getc(0) == "#" || line.getc(0) == "\n") continue;

            nfields = $sscanf(line, "%d %h %d %h %d", f_is_cond, f_pc, f_taken, f_target, f_ref);
            if (nfields != 5)
                fail($sformatf("%s:%0d: expected 5 fields, got %0d", stim, lineno, nfields));
            n_lines++;

            if (f_is_cond != 0) begin
                n_cond++;
                predict(f_pc, rtl_taken);
                if (rtl_taken != (f_taken != 0)) n_mis_rtl++;
                if (f_ref != f_taken)            n_mis_ref++;
                if (rtl_taken != (f_ref != 0)) begin
`ifndef POST_SYN_SIM
                    $display("  ghr %h idx %h", dut.ghr_q, dut.idx_q);
`endif
                    fail($sformatf("MISMATCH line %0d (cond branch %0d) pc %h: rtl %0d ref %0d taken %0d",
                                   lineno, n_cond, f_pc, rtl_taken, f_ref, f_taken));
                end
                update(1'b1, f_pc, f_taken != 0, f_target);
            end else begin
                n_uncond++;
                update(1'b0, f_pc, f_taken != 0, f_target);
            end

            if (MAX_LINES != 0 && n_lines >= 64'(MAX_LINES)) break;
        end
        $fclose(fd);

        // Drain: the last update may still be completing (its write-back
        // happens after the update handshake). Wait until the core is idle,
        // so the counters include every access of every branch.
        begin
            int unsigned waited;
            waited = 0;
            while (!pred_req_ready) begin
                step;   // the counters sample this edge before step returns
                if (++waited > TIMEOUT_CYCLES) fail("timeout waiting for the core to drain");
            end
        end

`ifdef VCD
        $dumpoff;
`endif
        $display("TB_LINES              : %0d", n_lines);
        $display("TB_COND_BR            : %0d", n_cond);
        $display("TB_UNCOND_BR          : %0d", n_uncond);
        $display("TB_RTL_MISPRED        : %0d", n_mis_rtl);
        $display("TB_REF_MISPRED        : %0d", n_mis_ref);
        $display("TB_RTL_MPKBr          : %0.4f",
                 n_cond != 0 ? 1000.0 * real'(n_mis_rtl) / real'(n_cond) : 0.0);
        $display("TB_CYCLES             : %0d", n_cycles);
        $display("TB_SRAM_READS         : %0d", n_sram_rd);
        $display("TB_SRAM_WRITES        : %0d", n_sram_wr);

        if (n_lines == 0) fail("stimulus file contains no branch lines");
        if (n_mis_rtl != n_mis_ref)
            fail($sformatf("misprediction count differs: rtl %0d ref %0d", n_mis_rtl, n_mis_ref));

        $display("\nbp_gshare_core: all %0d branches (%0d conditional) match the reference. PASSED!\n",
                 n_lines, n_cond);
        $finish;
    end

endmodule