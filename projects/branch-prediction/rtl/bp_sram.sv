// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Behavioral 1R1W SRAM model with a synchronous read (1-cycle latency), the
//   timing of a real SRAM macro: the read address is sampled on a rising edge
//   and rdata_o is valid after that edge until the next read. Shared by every
//   predictor (gshare PHT, perceptron weight tables, TAGE tables).
//
//   SIMULATION ONLY. Never synthesize this module: Yosys would build the array
//   out of DEPTH * WIDTH flip-flops plus a read mux. Synthesize the predictor
//   core instead; the SRAM ports are its I/O, and the memory itself is costed
//   separately (CACTI / SRAM macro data, using the testbench access counts).
//
//   Read-during-write to the same address returns the OLD data (read-first):
//   both assignments are nonblocking. With one branch in flight this never
//   happens; it matters once the pipeline overlaps predict and update.
//
//   The initial block stands in for the C++ constructor that initializes the
//   table; a real SRAM has no reset.
//
//   Access counters (Verilator only, like the address checks): every
//   instance counts its reads (rd), writes (wr), cycles with a read AND a
//   write (rw: a single-port 1RW macro could not serve them) and the subset
//   of those with raddr_i == waddr_i (rwa: the read-during-write case above),
//   from reset release on and inside the bench's activity window. The bench
//   switches them with bp_sram_pkg::count_en and bp_sram_pkg::win_en
//   (tb_bp_common.svh) on the same edges as its own TB_SRAM_* and
//   TB_WIN_SRAM_* counters, so the per-instance counts add up to those
//   totals (evaluate.py checks this). A final block prints one line per
//   instance (%m = its hierarchical name):
//     TB_SRAM_INST <%m> DEPTH d WIDTH w RD n WR n RW n RWA n
//                  WIN_RD n WIN_WR n WIN_RW n WIN_RWA n
//   bp_sram_pkg sits in this file because only this model and the bench use
//   it; the gate-level benches `include this file, and in the RTL flow it
//   must be compiled before the bench (rtl/ before tb/ in the file list).
//
// Parameters:
//   DEPTH - number of entries (>= 2)
//   WIDTH - bits per entry
//   INIT  - value every entry holds at time 0
// -----------------------------------------------------------------------------

`ifndef BP_SRAM_SV
`define BP_SRAM_SV

`timescale 1 ns/1 ps

`ifdef VERILATOR
// Switches of the access counters, set by the bench (tb_bp_common.svh)
/* verilator lint_off DECLFILENAME */
package bp_sram_pkg;
    bit count_en;   // reset released: the counters count from here
    bit win_en;     // the activity window is open
endpackage
/* verilator lint_on DECLFILENAME */
`endif

module bp_sram #(
    parameter int unsigned      DEPTH  = 1024,
    parameter int unsigned      WIDTH  = 8,
    parameter logic [WIDTH-1:0] INIT   = '0,
    localparam int unsigned     ADDR_W = $clog2(DEPTH)
) (
    input  logic              clk_i,

    // read port
    input  logic              re_i,
    input  logic [ADDR_W-1:0] raddr_i,
    output logic [ WIDTH-1:0] rdata_o,

    // write port
    input  logic              we_i,
    input  logic [ADDR_W-1:0] waddr_i,
    input  logic [ WIDTH-1:0] wdata_i
);
    /* verilator tracing_off */   // bench-side model: never dumped

    if (DEPTH < 2) begin : g_check_depth
        $error("bp_sram: DEPTH must be >= 2");
    end

    logic [WIDTH-1:0] mem_q [DEPTH];

    initial begin
        for (int unsigned i = 0; i < DEPTH; i++) begin
            mem_q[i] = INIT;
        end
    end

    always_ff @(posedge clk_i) begin
        if (we_i) mem_q[waddr_i] <= wdata_i;
        if (re_i) rdata_o        <= mem_q[raddr_i];
    end

`ifdef VERILATOR
    // A non-power-of-two DEPTH leaves addresses that do not exist.
    always_ff @(posedge clk_i) begin
        if (re_i && 32'(raddr_i) >= DEPTH)
            $error("bp_sram: read address %0d out of range (DEPTH %0d)", raddr_i, DEPTH);
        if (we_i && 32'(waddr_i) >= DEPTH)
            $error("bp_sram: write address %0d out of range (DEPTH %0d)", waddr_i, DEPTH);
    end

    // Access counters (see the header): n_* from reset release, w_* in the
    // activity window
    longint unsigned n_rd  = 0;
    longint unsigned n_wr  = 0;
    longint unsigned n_rw  = 0;
    longint unsigned n_rwa = 0;
    longint unsigned w_rd  = 0;
    longint unsigned w_wr  = 0;
    longint unsigned w_rw  = 0;
    longint unsigned w_rwa = 0;

    logic rw_now;
    logic rwa_now;

    assign rw_now  = re_i & we_i;
    assign rwa_now = rw_now & (raddr_i == waddr_i);

    always @(posedge clk_i) begin
        if (bp_sram_pkg::count_en) begin
            n_rd  <= n_rd  + 64'(re_i);
            n_wr  <= n_wr  + 64'(we_i);
            n_rw  <= n_rw  + 64'(rw_now);
            n_rwa <= n_rwa + 64'(rwa_now);
            if (bp_sram_pkg::win_en) begin
                w_rd  <= w_rd  + 64'(re_i);
                w_wr  <= w_wr  + 64'(we_i);
                w_rw  <= w_rw  + 64'(rw_now);
                w_rwa <= w_rwa + 64'(rwa_now);
            end
        end
    end

    final begin
        if (bp_sram_pkg::count_en)
            $display("TB_SRAM_INST %m DEPTH %0d WIDTH %0d RD %0d WR %0d RW %0d RWA %0d WIN_RD %0d WIN_WR %0d WIN_RW %0d WIN_RWA %0d",
                     DEPTH, WIDTH, n_rd, n_wr, n_rw, n_rwa, w_rd, w_wr, w_rw, w_rwa);
    end
`endif

endmodule

`endif