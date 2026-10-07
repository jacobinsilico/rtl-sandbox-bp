// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Shared body of every predictor testbench (tb_bp_<predictor>_core). It is
//   `include'd INSIDE the bench module, after the DUT and SRAM instances, and
//   provides the clock, reset, golden-dump stimulus, per-branch checking, the
//   drain at the end, the report and the VCD dump. A bench therefore contains
//   only what differs between predictors: its parameters, the DUT, the SRAM
//   models, the SRAM access count per cycle and a debug string.
//
//   Golden dump format (patched CBP5 harness), one branch per line:
//     is_cond pc taken target ref_pred        (pc and target in hex)
//   Lines starting with '#' are skipped. Conditional branch: predict, check
//   the prediction against ref_pred (fatal on the first mismatch), update.
//   Unconditional branch: TrackOtherInst update only.
//
//   Inputs are driven T_SETTLE after a rising edge and outputs are sampled
//   T_SETTLE after a rising edge, so the DUT always samples stable values.
//   After the last line the bench drains (waits until the core is idle), so
//   the write-back of the last branch is simulated and counted.
//
//   Report (KEY : value lines, then PASSED or a fatal error):
//     TB_LINES, TB_COND_BR, TB_UNCOND_BR, TB_RTL_MISPRED (must equal
//     TB_REF_MISPRED), TB_RTL_MPKBr, TB_CYCLES, TB_SRAM_READS, TB_SRAM_WRITES
//     (summed over all SRAMs; the SRAMs are outside dut, so the power
//     analysis does not see them -- these counts feed the SRAM energy).
//
//   The including module must declare, BEFORE this include:
//     parameters  STIM_FILE (string), MAX_LINES, TIMEOUT_CYCLES
//     localparam  PC_W, DUT_NAME (string)
//     logic       clk_i, rst_ni
//     logic       pred_req_valid, pred_req_ready, pred_resp_valid,
//                 pred_resp_taken, upd_valid, upd_ready, upd_is_cond,
//                 upd_taken
//     logic [PC_W-1:0] pred_req_pc, upd_pc, upd_target
//     logic [7:0] sram_rd_now, sram_wr_now   SRAM reads / writes the DUT
//                                            issues in the current cycle
//     function automatic string tb_params()  parameter summary (banner)
//     function automatic string tb_debug()   DUT state for a mismatch
//                                            report ("" when unavailable)
// -----------------------------------------------------------------------------

    localparam real CLK_PERIOD = `CLK_PERIOD_NS;
    localparam real CLK_HALF   = CLK_PERIOD / 2.0;
    localparam real T_SETTLE   = CLK_PERIOD / 10.0;

    // -------------------------------------------------------------------------
    // Clock and activity counters. counting is set when reset is released.
    // -------------------------------------------------------------------------
    logic            counting  = 1'b0;
    longint unsigned n_cycles  = 0;
    longint unsigned n_sram_rd = 0;
    longint unsigned n_sram_wr = 0;

    initial clk_i = 1'b0;
    always #(CLK_HALF) clk_i = ~clk_i;

    always @(posedge clk_i) begin
        if (counting) begin
            n_cycles  <= n_cycles  + 64'd1;
            n_sram_rd <= n_sram_rd + 64'(sram_rd_now);
            n_sram_wr <= n_sram_wr + 64'(sram_wr_now);
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

    task automatic drain;
        int unsigned waited;
        waited = 0;
        while (!pred_req_ready) begin
            step;                             // the counters sample this edge
            if (++waited > TIMEOUT_CYCLES) fail("timeout waiting for the core to drain");
        end
    endtask

    // -------------------------------------------------------------------------
    // Stimulus, checking and report
    // -------------------------------------------------------------------------
    initial begin
        string           stim;
        string           line;
        string           dbg;
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

        $display("\nStarting %s verification (%s)", DUT_NAME, tb_params());
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
                    dbg = tb_debug();
                    if (dbg != "") $display("  %s", dbg);
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

        drain;

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

        $display("\n%s: all %0d branches (%0d conditional) match the reference. PASSED!\n",
                 DUT_NAME, n_lines, n_cond);
        $finish;
    end
