// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   One step of the cookbook TAGE pseudo-random generator (tage_cb.h
//   MYRANDOM), combinational. Seed is a C int: every add wraps mod 2^32 and
//   ">>" is an arithmetic shift, written out as explicit sign replication so
//   no signed SystemVerilog operator is involved:
//     s = seed + 1 + phist
//     s = (s >> 21) + (s << 11)
//     s = s + ptghist
//     s = (s >> 10) + (s << 22)
//     s = s + GTAG[4]            (zero-extended)
//   The new seed is also the returned random value. ptghist is the C++
//   global history pointer: a 32-bit counter that goes down by 4 per block.
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

    always_comb begin
        s1     = seed_i + 32'd1 + 32'(phist_i);
        s2     = {{21{s1[31]}}, s1[31:21]} + {s1[20:0], 11'd0};
        s3     = s2 + ptghist_i;
        s4     = {{10{s3[31]}}, s3[31:10]} + {s3[9:0], 22'd0};
        seed_o = s4 + 32'(gtag4_i);
    end

endmodule