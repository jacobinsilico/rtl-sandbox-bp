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
// Parameters:
//   DEPTH - number of entries (>= 2)
//   WIDTH - bits per entry
//   INIT  - value every entry holds at time 0
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

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
`endif

endmodule