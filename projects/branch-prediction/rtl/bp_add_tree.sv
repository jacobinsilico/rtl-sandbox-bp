// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Combinational balanced adder tree: sum_o = sum of N signed IN_W-bit
//   inputs, exact (no overflow). The inputs are packed into one vector,
//   input k at in_i[k*IN_W +: IN_W]. The tree has ceil(log2 N) levels; each
//   level adds pairs and grows the width by one bit, so level l is
//   IN_W + l bits wide and the result is IN_W + ceil(log2 N) bits. N is
//   padded to a power of two with zero inputs, which synthesis removes.
//
//   Written as an explicit tree (not a summing loop) so the depth is
//   logarithmic in N regardless of how the synthesis tool restructures
//   arithmetic chains. Used by both perceptron cores.
//
// Parameters:
//   N    - number of inputs (>= 1)
//   IN_W - width of each signed input (>= 1)
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_add_tree #(
    parameter int unsigned  N      = 4,
    parameter int unsigned  IN_W   = 8,
    localparam int unsigned LEVELS = (N > 1) ? $clog2(N) : 0,
    localparam int unsigned OUT_W  = IN_W + LEVELS
) (
    input  logic [N*IN_W-1:0] in_i,
    output logic [ OUT_W-1:0] sum_o
);

    if (N < 1 || IN_W < 1) begin : g_check
        $error("bp_add_tree: N and IN_W must be >= 1");
    end

    localparam int unsigned P = 1 << LEVELS;   // N padded to a power of two

    for (genvar l = 0; l <= LEVELS; l++) begin : g_lvl
        localparam int unsigned W  = IN_W + l;   // width of a value at level l
        localparam int unsigned NN = P >> l;     // values at level l

        logic [NN*W-1:0] v;

        if (l == 0) begin : g_in
            for (genvar k = 0; k < NN; k++) begin : g_k
                if (k < N) begin : g_used
                    assign v[k*W +: W] = in_i[k*IN_W +: IN_W];
                end else begin : g_pad
                    assign v[k*W +: W] = '0;
                end
            end
        end else begin : g_add
            for (genvar k = 0; k < NN; k++) begin : g_k
                // Both operands signed -> sign-extended to W before adding.
                assign v[k*W +: W] =
                    W'($signed(g_lvl[l-1].v[(2*k)  *(W-1) +: (W-1)])) +
                    W'($signed(g_lvl[l-1].v[(2*k+1)*(W-1) +: (W-1)]));
            end
        end
    end

    assign sum_o = g_lvl[LEVELS].v;

endmodule