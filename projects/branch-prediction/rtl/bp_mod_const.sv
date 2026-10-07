// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Combinational remainder by a constant: rem_o = x_i % DIVISOR (unsigned),
//   bit-exact for every x_i. Three implementations:
//
//   - power-of-two DIVISOR: a bit slice (no logic).
//   - RECIP = 1 (default): multiplication by the reciprocal (Granlund &
//     Montgomery, PLDI 1994). With L = ceil(log2 DIVISOR), S = IN_W + L and
//     M = ceil(2^S / DIVISOR), q = floor(x * M / 2^S) equals floor(x /
//     DIVISOR) for every IN_W-bit x, because M*DIVISOR - 2^S < DIVISOR <=
//     2^L. Since the remainder is below 2^L, only the low L bits of q and of
//     x are needed: rem = (x - q*DIVISOR) mod 2^L. The cost is one multiply
//     by a constant (a shallow adder tree) instead of a divider.
//   - RECIP = 0: the generic x % DIVISOR, which synthesis builds as an array
//     divider (about IN_W subtract stages in series). Kept for comparison.
//
//   Kept as its own module so synthesis with KEEP_HIERARCHY / KEEP_MODULES
//   reports its area separately: it exists only because the sweep-optimal
//   table sizes are not powers of two, and a silicon design would drop it.
//
// Parameters:
//   IN_W    - width of x_i (1..32)
//   DIVISOR - constant divisor (>= 2, < 2^IN_W)
//   RECIP   - 1: reciprocal multiply (default); 0: generic modulo
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_mod_const #(
    parameter int unsigned  IN_W    = 32,
    parameter int unsigned  DIVISOR = 3,
    parameter bit           RECIP   = 1'b1,
    localparam int unsigned OUT_W   = $clog2(DIVISOR)
) (
    input  logic [ IN_W-1:0] x_i,
    output logic [OUT_W-1:0] rem_o
);

    if (DIVISOR < 2) begin : g_check_div
        $error("bp_mod_const: DIVISOR must be >= 2");
    end
    if (IN_W < 1 || IN_W > 32) begin : g_check_inw
        $error("bp_mod_const: IN_W must be in 1..32");
    end
    if (OUT_W > IN_W) begin : g_check_w
        $error("bp_mod_const: DIVISOR does not fit in IN_W bits");
    end

    localparam bit IS_POW2 = ((DIVISOR & (DIVISOR - 1)) == 0);

    if (IS_POW2) begin : g_pow2
        assign rem_o = x_i[OUT_W-1:0];

    end else if (RECIP) begin : g_recip
        localparam int unsigned     L      = OUT_W;
        localparam int unsigned     S      = IN_W + L;           // <= 63
        localparam int unsigned     PROD_W = 2 * IN_W + 2;
        localparam longint unsigned M      = ((64'd1 << S) + 64'(DIVISOR) - 64'd1) / 64'(DIVISOR);

        /* verilator lint_off UNUSEDSIGNAL */
        logic [PROD_W-1:0] prod;     // x * M; only bits [S +: L] are used
        /* verilator lint_on UNUSEDSIGNAL */
        logic [     L-1:0] q_lo;     // low L bits of floor(x / DIVISOR)

        assign prod  = PROD_W'(x_i) * PROD_W'(M);
        assign q_lo  = prod[S +: L];
        assign rem_o = L'(x_i[L-1:0] - L'(q_lo * L'(DIVISOR)));

    end else begin : g_div
        localparam logic [IN_W-1:0] D = IN_W'(DIVISOR);
        /* verilator lint_off UNUSEDSIGNAL */
        logic [IN_W-1:0] rem_full;   // < DIVISOR, so the upper bits are 0
        /* verilator lint_on UNUSEDSIGNAL */
        assign rem_full = x_i % D;
        assign rem_o    = rem_full[OUT_W-1:0];
    end

endmodule