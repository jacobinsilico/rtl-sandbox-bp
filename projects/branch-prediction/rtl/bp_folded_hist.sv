// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Folded global history register (P. Michaud, CBP-1): a CLEN-bit cyclic
//   compression of the last OLEN history bits, as folded_history in the CBP5
//   predictors. One push inserts NPUSH bits; for each bit k (in_i[0] first)
//     comp = (comp << 1) ^ in_i[k]
//     comp ^= out_i[k] << (OLEN % CLEN)
//     comp ^= comp >> CLEN;  comp &= 2^CLEN - 1
//   out_i[k] is the bit that leaves the OLEN-bit window at step k, i.e. the
//   bit inserted OLEN steps before it. The caller taps it from its history
//   shift register (newest bit at index 0): out_i[k] = ghist[OLEN-1-k] read
//   before the push. Resets to 0, like folded_history::init.
//
// Parameters:
//   OLEN  - original (unfolded) history length in bits (>= NPUSH)
//   CLEN  - compressed length in bits (>= 2)
//   NPUSH - bits inserted per push
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_folded_hist #(
    parameter int unsigned OLEN  = 64,
    parameter int unsigned CLEN  = 11,
    parameter int unsigned NPUSH = 4
) (
    input  logic             clk_i,
    input  logic             rst_ni,

    input  logic             push_i,   // insert NPUSH bits this cycle
    input  logic [NPUSH-1:0] in_i,     // new bits, in_i[0] inserted first
    input  logic [NPUSH-1:0] out_i,    // bits leaving the window, per step
    output logic [ CLEN-1:0] comp_o
);

    if (CLEN < 2) begin : g_check_clen
        $error("bp_folded_hist: CLEN must be >= 2");
    end
    if (OLEN < NPUSH) begin : g_check_olen
        $error("bp_folded_hist: OLEN must be >= NPUSH");
    end

    localparam int unsigned OUTPOINT = OLEN % CLEN;

    logic [CLEN-1:0] comp_q, comp_d;
    logic [  CLEN:0] step_t;   // one step before the wrap-around fold

    always_comb begin
        comp_d = comp_q;
        step_t = '0;
        for (int unsigned k = 0; k < NPUSH; k++) begin
            step_t           = {comp_d, in_i[k]};
            step_t[OUTPOINT] = step_t[OUTPOINT] ^ out_i[k];
            comp_d           = step_t[CLEN-1:0];
            comp_d[0]        = comp_d[0] ^ step_t[CLEN];
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni)     comp_q <= '0;
        else if (push_i) comp_q <= comp_d;
    end

    assign comp_o = comp_q;

endmodule
