// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   One step of the cookbook TAGE pseudo-random generator (tage_cb.h
//   MYRANDOM), combinational. Seed is a C int: every add wraps mod 2^32 and
//   ">>" is an arithmetic shift:
//     s = seed + 1 + phist
//     s = (s >> 21) + (s << 11)
//     s = s + ptghist
//     s = (s >> 10) + (s << 22)
//     s = s + GTAG[4]            (zero-extended)
//   The new seed is also the returned random value. ptghist is the C++
//   global history pointer: a 32-bit counter that goes down by 4 per block.
//
//   Three adds in series instead of five. In (s >> 21) + (s << 11) the
//   arithmetic shift contributes s[31:21] in the low 11 bits and copies of
//   s[31] above them, i.e. -s[31] in the upper 21 bits, where (s << 11)
//   holds s[20:0]:
//     (s >> 21) + (s << 11) = {s[20:0] - s[31], s[31:21]}
//   and likewise (s >> 10) + (s << 22) = {s[9:0] - s[31], s[31:10]}. The
//   "- s[31]" is taken from a second, narrow adder that computes the same
//   sum minus one in parallel (s1m, s3m), selected by the sign bit: a
//   multiplexer instead of a carry chain. Every add has two operands, so
//   each maps to one parallel-prefix adder: seed + phist + 1 is written as
//   seed - ~phist (one adder with carry-in), and s3m adds ptghist[9:0] - 1,
//   computed from the ptghist input off the seed path. Bit-exact with the
//   five-step form (checked in C++ on edge values, random inputs and the
//   recurrence).
//
// Parameters:
//   TBITS - tag width (GTAG[4] width)
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_tage_rng #(
    parameter int unsigned TBITS = 12
) (
    input  logic [     31:0] seed_i,
    input  logic [     26:0] phist_i,
    input  logic [     31:0] ptghist_i,
    input  logic [TBITS-1:0] gtag4_i,
    output logic [     31:0] seed_o
);

    if (TBITS < 1 || TBITS > 31) begin : g_check_tbits
        $error("bp_tage_rng: TBITS must be in 1..31");
    end

    logic [31:0] s1, s2, s3, s4;
    logic [20:0] s1m;   // (s1 - 1)[20:0]
    logic [ 9:0] km;    // ptghist[9:0] - 1
    logic [ 9:0] s3m;   // (s3 - 1)[9:0]

    always_comb begin
        s1     = seed_i - ~32'(phist_i);          // seed + phist + 1
        s1m    = seed_i[20:0] + phist_i[20:0];
        s2     = {s1[31] ? s1m : s1[20:0], s1[31:21]};
        s3     = s2 + ptghist_i;
        km     = ptghist_i[9:0] - 10'd1;
        s3m    = s2[9:0] + km;
        s4     = {s3[31] ? s3m : s3[9:0], s3[31:10]};
        seed_o = s4 + 32'(gtag4_i);
    end

endmodule