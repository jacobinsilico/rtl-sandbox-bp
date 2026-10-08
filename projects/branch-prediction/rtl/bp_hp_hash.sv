// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Index of one hashed-perceptron weight table, combinational, bit-exact
//   with hp_core.h Core::table_index() for HP_HASH=1:
//     s   = TBL % (LG - 1) + 1
//     idx = rotl(fold(a ^ (a >> s)), (3*TBL) % LG)
//         ^ rotl(fold(f_k), ROT_k)                    k < FOLDS
//         ^ rotl(fold(phist & (2^PB - 1)), (5*TBL + 1) % LG)   PB = min(HLEN,
//                                                    PATH_BITS), if PB > 0
//   a = pcmix(pc) (computed once by the core), f_k = folded histories,
//   fold() = XOR of every LG-bit chunk (hp::fold_to), rotl() = rotate left
//   inside LG bits (hp::rotl_n). Every shift and rotation is an elaboration
//   constant, so this is XOR trees and wiring only.
//
// Parameters:
//   TBL       - table number (0 = shortest history)
//   LG        - log2 of the table's entries (2..24)
//   FOLDS     - folded histories used (1..3, HP_FOLDS)
//   ROT0..2   - rotation of each folded history into the index
//   HLEN      - history length of the table (bits)
//   PATH_BITS - path history width (0 = no path term, HP_PATH_BITS)
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_hp_hash #(
    parameter int unsigned TBL       = 0,
    parameter int unsigned LG        = 8,
    parameter int unsigned FOLDS     = 3,
    parameter int unsigned ROT0      = 0,
    parameter int unsigned ROT1      = 0,
    parameter int unsigned ROT2      = 0,
    parameter int unsigned HLEN      = 4,
    parameter int unsigned PATH_BITS = 16
) (
    input  logic [  63:0] a_i,       // pcmix(pc)
    input  logic [  30:0] f0_i,      // folded histories, zero-extended
    input  logic [  30:0] f1_i,
    input  logic [  30:0] f2_i,
    input  logic [  30:0] phist_i,   // path history, zero-extended
    output logic [LG-1:0] idx_o
);

    if (LG < 2 || LG > 24) begin : g_check_lg
        $error("bp_hp_hash: LG must be in 2..24");
    end
    if (FOLDS < 1 || FOLDS > 3) begin : g_check_folds
        $error("bp_hp_hash: FOLDS must be in 1..3");
    end
    if (PATH_BITS > 31) begin : g_check_path
        $error("bp_hp_hash: PATH_BITS must be in 0..31");
    end

    localparam int unsigned S     = (TBL % (LG - 1)) + 1;
    localparam int unsigned R_PC  = (TBL * 3) % LG;
    localparam int unsigned R_P   = (TBL * 5 + 1) % LG;
    localparam int unsigned PB    = (HLEN < PATH_BITS) ? HLEN : PATH_BITS;
    localparam logic [30:0] PMASK = (PB == 0) ? 31'd0 : 31'((64'd1 << PB) - 64'd1);

    // XOR of every LG-bit chunk of x (hp::fold_to)
    function automatic logic [LG-1:0] fold64(input logic [63:0] x);
        logic [LG-1:0] r;
        r = '0;
        for (int unsigned c = 0; c * LG < 64; c++) r ^= LG'(x >> (c * LG));
        return r;
    endfunction

    // rotate left inside LG bits by a constant r < LG (hp::rotl_n)
    function automatic logic [LG-1:0] rotl(input logic [LG-1:0] x, input int unsigned r);
        if (r == 0) return x;
        return (x << r) | (x >> (LG - r));
    endfunction

    logic [63:0] pcs;
    logic [LG-1:0] x;

    always_comb begin
        pcs = a_i ^ (a_i >> S);
        x   = rotl(fold64(pcs), R_PC);
        x   = x ^ rotl(fold64(64'(f0_i)), ROT0);
        if (FOLDS > 1) x = x ^ rotl(fold64(64'(f1_i)), ROT1);
        if (FOLDS > 2) x = x ^ rotl(fold64(64'(f2_i)), ROT2);
        if (PB > 0)    x = x ^ rotl(fold64(64'(phist_i & PMASK)), R_P);
        idx_o = x;
    end

    /* verilator lint_off UNUSEDSIGNAL */
    logic unused_ok;
    assign unused_ok = ^{f1_i, f2_i, phist_i};
    /* verilator lint_on UNUSEDSIGNAL */

endmodule
