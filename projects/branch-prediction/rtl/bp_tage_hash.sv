// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Index and tag hash of one logical TAGE table, combinational, bit-exact
//   with tage_cb.h gindex() / gtag() / F() for AHEAD = 0. Evaluated once per
//   fetch block (the core registers the results). All shift amounts are
//   elaboration constants, so this is XOR trees and wiring only.
//
//   F(phist, M, TBL): A = phist & (2^M - 1), M = min(OLEN, 27);
//     A1 = A & (2^LOGG - 1); A2 = A >> LOGG;
//     if TBL < LOGG: A2 = ((A2 << TBL) & (2^LOGG - 1)) ^ (A2 >> (LOGG - TBL))
//     A = A1 ^ A2;
//     if TBL < LOGG: A  = ((A  << TBL) & (2^LOGG - 1)) ^ (A  >> (LOGG - TBL))
//     Note: F uses LOGG, not the table's index width, and A2 is NOT reduced
//     to LOGG bits, so F can be wider than LOGG bits (as in the C++).
//   gindex (unsigned, 32-bit):
//     I = pc ^ (pc >> (|LOGG_I - TBL| + 1)) ^ ci ^ F
//     idx = (I ^ (I >> LOGG_I) ^ (I >> 2*LOGG_I)) & (2^LOGG_I - 1)
//   gtag (C int: ">>" is arithmetic, written as sign replication):
//     t = pc ^ (pc >> 2)
//     t = (t >> 1) ^ ((t & 1) << 10) ^ F;  t ^= ct0 ^ (ct1 << 1)
//     t ^= t >> TBITS;  t ^= t >> (TBITS - 2);  tag = t & (2^TBITS - 1)
//   The arithmetic shifts matter only when pc[31] = 1 and TBITS >= 12.
//
// Parameters:
//   TBL    - logical table number, 1..NHIST
//   LOGG   - log2 entries of one way of a logical table (F() uses it)
//   LOGG_I - index width (LOGG + 1 for T1 with CB_SHARED, else LOGG)
//   OLEN   - history length of the table in bits (C++ m[TBL])
//   CI_W   - width of the index fold ci_i (1 and unused when DO_IDX = 0)
//   TBITS  - tag width (>= 4)
//   DO_IDX - 1: compute idx_o; 0: idx_o = 0 (even tables: CB_ADJACENT
//            replaces their index by the odd neighbour's)
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_tage_hash #(
    parameter int unsigned TBL    = 1,
    parameter int unsigned LOGG   = 6,
    parameter int unsigned LOGG_I = 7,
    parameter int unsigned OLEN   = 4,
    parameter int unsigned CI_W   = 21,
    parameter int unsigned TBITS  = 10,
    parameter bit          DO_IDX = 1'b1
) (
    input  logic [      31:0] pc_i,      // PCBLOCK[31:0]
    input  logic [      26:0] phist_i,
    input  logic [  CI_W-1:0] ci_i,
    input  logic [      12:0] ct0_i,
    input  logic [      10:0] ct1_i,
    output logic [LOGG_I-1:0] idx_o,
    output logic [ TBITS-1:0] tag_o
);

    if (TBITS < 4 || TBITS > 16) begin : g_check_tbits
        $error("bp_tage_hash: TBITS must be in 4..16");
    end
    if (LOGG < 2 || LOGG_I < LOGG || LOGG_I > 15) begin : g_check_logg
        $error("bp_tage_hash: need 2 <= LOGG <= LOGG_I <= 15");
    end
    if (CI_W < 1 || CI_W > 27) begin : g_check_ciw
        $error("bp_tage_hash: CI_W must be in 1..27");
    end

    localparam int unsigned M     = (OLEN > 27) ? 27 : OLEN;
    localparam bit          ROT   = (TBL < LOGG);
    localparam int unsigned RSH   = ROT ? (LOGG - TBL) : 1;
    localparam int unsigned PSH   = ((LOGG_I > TBL) ? (LOGG_I - TBL) : (TBL - LOGG_I)) + 1;
    localparam logic [31:0] LMASK = (32'd1 << LOGG) - 32'd1;
    localparam logic [31:0] MMASK = (32'd1 << M) - 32'd1;

    logic [31:0] fa, fa1, fa2, fmix;
    logic [31:0] ix;
    logic [31:0] t0, t1, t2, t3;

    // F(phist, M, TBL)
    always_comb begin
        fa  = 32'(phist_i) & MMASK;
        fa1 = fa & LMASK;
        fa2 = fa >> LOGG;
        if (ROT) fa2 = ((fa2 << TBL) & LMASK) ^ (fa2 >> RSH);
        fmix = fa1 ^ fa2;
        if (ROT) fmix = ((fmix << TBL) & LMASK) ^ (fmix >> RSH);
    end

    // gindex
    always_comb begin
        ix = '0;
        if (DO_IDX) begin
            ix = pc_i ^ (pc_i >> PSH) ^ 32'(ci_i) ^ fmix;
            ix = ix ^ (ix >> LOGG_I) ^ (ix >> (2 * LOGG_I));
        end
        idx_o = ix[LOGG_I-1:0];
    end

    // gtag
    always_comb begin
        t0    = pc_i ^ (pc_i >> 2);
        t1    = {t0[31], t0[31:1]} ^ {21'd0, t0[0], 10'd0} ^ fmix;
        t1    = t1 ^ 32'(ct0_i) ^ {20'd0, ct1_i, 1'b0};
        t2    = t1 ^ {{TBITS{t1[31]}}, t1[31:TBITS]};
        t3    = t2 ^ {{(TBITS - 2){t2[31]}}, t2[31:TBITS-2]};
        tag_o = t3[TBITS-1:0];
    end

    /* verilator lint_off UNUSEDSIGNAL */
    logic unused_ok;
    assign unused_ok = ^{ix[31:LOGG_I], t3[31:TBITS], ci_i, phist_i};
    /* verilator lint_on UNUSEDSIGNAL */

endmodule