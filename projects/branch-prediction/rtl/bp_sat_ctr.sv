// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Combinational saturating counter step: ctr_o = ctr_i + 1 when inc_i is 1,
//   ctr_i - 1 when inc_i is 0, holding at the range limits. Equivalent to the
//   CBP5 SatIncrement / SatDecrement helpers in the unsigned case.
//
//   Unsigned range: 0 .. 2^WIDTH - 1         (gshare PHT, TAGE ctr / u bits)
//   Signed range:   -2^(WIDTH-1) .. 2^(WIDTH-1) - 1, two's complement
//                   (perceptron weights). Check this against the C++ weight
//                   clamp when the perceptrons are ported: some
//                   implementations clamp to a symmetric range instead.
//
// Parameters:
//   WIDTH  - counter width in bits (>= 1; >= 2 when SIGNED)
//   SIGNED - 0 = unsigned range, 1 = two's complement range
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_sat_ctr #(
    parameter int unsigned WIDTH  = 2,
    parameter bit          SIGNED = 1'b0
) (
    input  logic [WIDTH-1:0] ctr_i,
    input  logic             inc_i,
    output logic [WIDTH-1:0] ctr_o
);

    if (WIDTH < 1 || (SIGNED && WIDTH < 2)) begin : g_check_width
        $error("bp_sat_ctr: WIDTH must be >= 1 (>= 2 when SIGNED)");
    end

    localparam logic [WIDTH-1:0] ALL_ONES = '1;
    localparam logic [WIDTH-1:0] CTR_MAX  = SIGNED ? (ALL_ONES >> 1)  : ALL_ONES;
    localparam logic [WIDTH-1:0] CTR_MIN  = SIGNED ? ~(ALL_ONES >> 1) : '0;

    always_comb begin
        if (inc_i) ctr_o = (ctr_i == CTR_MAX) ? ctr_i : ctr_i + WIDTH'(1);
        else       ctr_o = (ctr_i == CTR_MIN) ? ctr_i : ctr_i - WIDTH'(1);
    end

endmodule
