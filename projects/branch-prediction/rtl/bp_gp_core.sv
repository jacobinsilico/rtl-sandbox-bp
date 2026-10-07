// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Global-history perceptron core with a FIXED training threshold (Jimenez &
//   Lin), the RTL counterpart of g_perceptron.h in the CBP5 simulator. This is
//   the unit that is synthesized: global history, row index, dot product,
//   training decision, weight update and control. The weight table is an SRAM
//   outside this module (one row per perceptron, read and written whole); its
//   ports are this module's I/O.
//
//   Row index (as g_perceptron.h):
//     idx = ((p[31:0] ^ p[63:32]) % NUM_PERCEPTRONS),  p = pc >> PC_SHIFT
//   The modulo is bp_mod_const (a slice for power-of-two tables, otherwise
//   a reciprocal multiply). The index is registered before the table read
//   (its own pipeline stage, as a front end computing the index from the
//   next PC one cycle ahead), so the modulo never shares a cycle with the
//   SRAM access or the dot product.
//
//   Row layout in the SRAM: weight j at bits [j*WEIGHT_BITS +: WEIGHT_BITS],
//   j = 0 the bias, j = i+1 paired with GHR bit i (bit 0 = newest outcome).
//
//   Prediction: y = w0 + sum_i (ghr[i] ? +w(i+1) : -w(i+1)) (bp_add_tree);
//   predict taken iff y >= 0. All-zero initial weights give y = 0, taken.
//
//   Training (UpdatePredictor): when mispredicted or |y| <= THETA, every
//   weight steps once toward agreement with the outcome (bias toward taken,
//   w(i+1) toward ghr[i] == taken), saturating at the WEIGHT_BITS two's-
//   complement range (bp_sat_ctr, SIGNED). Otherwise the table is untouched.
//   THETA = (THETA_ALPHA_PCT * (193*GHR_LEN + 1400)) / 10000, the C++ integer
//   formula. The history shifts in the outcome after training, on every
//   conditional branch.
//
//   State kept between predict and update (project-wide rule): the row index
//   and y in flops; the weight row is re-read from the SRAM when training.
//
//   Protocol (stage 1: non-speculative, ONE branch in flight):
//     S_IDLE     - pred_req_ready_o = 1. A predict handshake registers the
//                  row index computed from the PC. An UNCONDITIONAL update
//                  (CBP5 TrackOtherInst) is also accepted here and ignored,
//                  as g_perceptron.h does.
//     S_IDX      - the row is read at the registered index.
//     S_RD       - the row is valid; y is computed and latched. With
//                  Y_REG = 0 the prediction is returned in this cycle.
//     S_PRED     - only with Y_REG = 1: the prediction is returned from the
//                  registered y (one cycle later, shorter critical path).
//     S_WAIT_UPD - waits for the CONDITIONAL update. Its handshake decides
//                  training from y_q and the outcome. No training: the
//                  history shifts and the core returns to S_IDLE (no SRAM
//                  access). Training: the row is re-read at idx_q.
//     S_UPD      - the re-read row is valid: the stepped row is written back
//                  and the history shifts.
//   Cycles per conditional branch: 4 (no training) or 5 (training), plus one
//   with Y_REG = 1. Unconditional branch: 1.
//
//   The re-read returns what g_perceptron.h reads in UpdatePredictor: with
//   one branch in flight nothing writes the row in between, and the GHR used
//   for training (the C++ last_ghr) equals ghr_q until the update ends.
//
//   Interface signals the global perceptron does not use (upd_pc_i,
//   upd_target_i) are kept so all predictor cores share one interface.
//
// Parameters:
//   GHR_LEN         - global history length (1..64 here)
//   NUM_PERCEPTRONS - table rows (>= 2; need not be a power of two)
//   WEIGHT_BITS     - bits per weight (2..8)
//   THETA_ALPHA_PCT - threshold scale in percent (1..1000)
//   PC_SHIFT        - right shift of the PC before indexing (0..3)
//   Y_REG           - 0: predict in the cycle the row arrives; 1: register y
//                     first and predict one cycle later
//   MOD_RECIP       - index modulo: 1 reciprocal multiply (default), 0 the
//                     generic divider (comparison only; very long path)
//   PC_W            - PC / target width of the shared predictor interface
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_gp_core #(
    parameter int unsigned  GHR_LEN         = 18,
    parameter int unsigned  NUM_PERCEPTRONS = 287,
    parameter int unsigned  WEIGHT_BITS     = 6,
    parameter int unsigned  THETA_ALPHA_PCT = 25,
    parameter int unsigned  PC_SHIFT        = 2,
    parameter bit           Y_REG           = 1'b0,
    parameter bit           MOD_RECIP       = 1'b1,
    parameter int unsigned  PC_W            = 64,
    localparam int unsigned NUM_W           = GHR_LEN + 1,
    localparam int unsigned ROW_W           = NUM_W * WEIGHT_BITS,
    localparam int unsigned IDX_W           = $clog2(NUM_PERCEPTRONS)
) (
    input  logic             clk_i,
    input  logic             rst_ni,

    // predict channel (CBP5 GetPrediction)
    input  logic             pred_req_valid_i,
    output logic             pred_req_ready_o,
    input  logic [ PC_W-1:0] pred_req_pc_i,
    output logic             pred_resp_valid_o,
    output logic             pred_resp_taken_o,

    // update channel (CBP5 UpdatePredictor / TrackOtherInst)
    input  logic             upd_valid_i,
    output logic             upd_ready_o,
    input  logic             upd_is_cond_i,
    input  logic [ PC_W-1:0] upd_pc_i,
    input  logic             upd_taken_i,
    input  logic [ PC_W-1:0] upd_target_i,

    // weight table SRAM ports (1R1W, synchronous read, one row per access)
    output logic             wt_re_o,
    output logic [IDX_W-1:0] wt_raddr_o,
    input  logic [ROW_W-1:0] wt_rdata_i,
    output logic             wt_we_o,
    output logic [IDX_W-1:0] wt_waddr_o,
    output logic [ROW_W-1:0] wt_wdata_o
);

    // -------------------------------------------------------------------------
    // Elaboration checks (as g_perceptron.h, plus the RTL history limit)
    // -------------------------------------------------------------------------
    if (GHR_LEN < 1 || GHR_LEN > 64) begin : g_check_ghr
        $error("bp_gp_core: GHR_LEN must be in 1..64");
    end
    if (NUM_PERCEPTRONS < 2) begin : g_check_rows
        $error("bp_gp_core: NUM_PERCEPTRONS must be >= 2");
    end
    if (WEIGHT_BITS < 2 || WEIGHT_BITS > 8) begin : g_check_wbits
        $error("bp_gp_core: WEIGHT_BITS must be in 2..8");
    end
    if (THETA_ALPHA_PCT < 1 || THETA_ALPHA_PCT > 1000) begin : g_check_alpha
        $error("bp_gp_core: THETA_ALPHA_PCT must be in 1..1000");
    end
    if (PC_SHIFT > 3) begin : g_check_shift
        $error("bp_gp_core: PC_SHIFT must be in 0..3");
    end
    if (PC_W != 64) begin : g_check_pcw
        $error("bp_gp_core: the index hash folds a 64-bit PC (PC_W = 64)");
    end

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------
    localparam int unsigned THETA  = (THETA_ALPHA_PCT * (193 * GHR_LEN + 1400)) / 10000;
    localparam int unsigned TERM_W = WEIGHT_BITS + 1;              // holds -w_min
    localparam int unsigned Y_W    = TERM_W + $clog2(NUM_W);       // exact |y|

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        S_IDLE     = 3'd0,
        S_IDX      = 3'd1,
        S_RD       = 3'd2,
        S_PRED     = 3'd3,
        S_WAIT_UPD = 3'd4,
        S_UPD      = 3'd5
    } state_e;

    state_e             state_q, state_d;
    logic [GHR_LEN-1:0] ghr_q;     // global history, newest outcome in bit 0
    logic [  IDX_W-1:0] idx_q;     // row of the branch in flight
    logic [    Y_W-1:0] y_q;       // its perceptron output (signed)
    logic               taken_q;   // its resolved direction

    // -------------------------------------------------------------------------
    // Row index: folded_xor32(pc >> PC_SHIFT) % NUM_PERCEPTRONS
    // -------------------------------------------------------------------------
    logic [ PC_W-1:0] pc_shifted;
    logic [     31:0] pc_fold;
    logic [IDX_W-1:0] pred_idx;

    assign pc_shifted = pred_req_pc_i >> PC_SHIFT;
    assign pc_fold    = pc_shifted[31:0] ^ pc_shifted[63:32];

    bp_mod_const #(
        .IN_W   (32),
        .DIVISOR(NUM_PERCEPTRONS),
        .RECIP  (MOD_RECIP)
    ) i_idx_mod (
        .x_i    (pc_fold),
        .rem_o  (pred_idx)
    );

    // -------------------------------------------------------------------------
    // Dot product: y = w0 + sum_i (ghr[i] ? +w(i+1) : -w(i+1))
    // -------------------------------------------------------------------------
    logic [NUM_W*TERM_W-1:0] terms;
    logic [         Y_W-1:0] y_comb;

    // Each weight is sign-extended by one bit first, so negating the most
    // negative weight cannot overflow.
    assign terms[0 +: TERM_W] = TERM_W'($signed(wt_rdata_i[0 +: WEIGHT_BITS]));

    for (genvar i = 0; i < GHR_LEN; i++) begin : g_term
        logic [TERM_W-1:0] w;
        assign w = TERM_W'($signed(wt_rdata_i[(i+1)*WEIGHT_BITS +: WEIGHT_BITS]));
        assign terms[(i+1)*TERM_W +: TERM_W] = ghr_q[i] ? w : (~w + TERM_W'(1));
    end

    bp_add_tree #(
        .N    (NUM_W),
        .IN_W (TERM_W)
    ) i_dot (
        .in_i (terms),
        .sum_o(y_comb)
    );

    // -------------------------------------------------------------------------
    // Training decision (at the update handshake, from y_q and the outcome)
    // -------------------------------------------------------------------------
    logic           pred_dir_q;   // the prediction that was returned
    logic [Y_W-1:0] abs_y;
    logic           train;

    assign pred_dir_q = ~y_q[Y_W-1];
    assign abs_y      = y_q[Y_W-1] ? (~y_q + Y_W'(1)) : y_q;
    assign train      = (pred_dir_q != upd_taken_i) || (32'(abs_y) <= THETA);

    // -------------------------------------------------------------------------
    // Handshakes
    // -------------------------------------------------------------------------
    logic pred_fire;       // predict request accepted
    logic upd_cond_fire;   // conditional update accepted

    assign pred_req_ready_o  = (state_q == S_IDLE);
    assign upd_ready_o       = (state_q == S_IDLE) || (state_q == S_WAIT_UPD);

    if (Y_REG) begin : g_resp_reg
        assign pred_resp_valid_o = (state_q == S_PRED);
        assign pred_resp_taken_o = ~y_q[Y_W-1];
    end else begin : g_resp_comb
        assign pred_resp_valid_o = (state_q == S_RD);
        assign pred_resp_taken_o = ~y_comb[Y_W-1];
    end

    assign pred_fire     = pred_req_valid_i && pred_req_ready_o;
    assign upd_cond_fire = upd_valid_i && upd_is_cond_i && (state_q == S_WAIT_UPD);

    // -------------------------------------------------------------------------
    // Weight table access: read at predict (from the registered index),
    // re-read and write when training
    // -------------------------------------------------------------------------
    assign wt_re_o    = (state_q == S_IDX) || (upd_cond_fire && train);
    assign wt_raddr_o = idx_q;
    assign wt_we_o    = (state_q == S_UPD);
    assign wt_waddr_o = idx_q;

    for (genvar j = 0; j < NUM_W; j++) begin : g_step
        logic inc;
        if (j == 0) begin : g_bias
            assign inc = taken_q;
        end else begin : g_hist
            assign inc = (ghr_q[j-1] == taken_q);
        end
        bp_sat_ctr #(
            .WIDTH (WEIGHT_BITS),
            .SIGNED(1'b1)
        ) i_sat (
            .ctr_i (wt_rdata_i[j*WEIGHT_BITS +: WEIGHT_BITS]),
            .inc_i (inc),
            .ctr_o (wt_wdata_o[j*WEIGHT_BITS +: WEIGHT_BITS])
        );
    end

    // -------------------------------------------------------------------------
    // Control
    // -------------------------------------------------------------------------
    always_comb begin
        state_d = state_q;
        case (state_q)
            S_IDLE:     if (pred_fire)     state_d = S_IDX;
            S_IDX:                         state_d = S_RD;
            S_RD:                          state_d = Y_REG ? S_PRED : S_WAIT_UPD;
            S_PRED:                        state_d = S_WAIT_UPD;
            S_WAIT_UPD: if (upd_cond_fire) state_d = train ? S_UPD : S_IDLE;
            S_UPD:                         state_d = S_IDLE;
            default:                       state_d = S_IDLE;
        endcase
    end

    // History push: at the update handshake when not training, after the
    // write-back when training. Either way after the training decision.
    logic hist_push;
    logic hist_bit;

    assign hist_push = (upd_cond_fire && !train) || (state_q == S_UPD);
    assign hist_bit  = (state_q == S_UPD) ? taken_q : upd_taken_i;

    // Control state and history: reset.
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q <= S_IDLE;
            ghr_q   <= '0;
        end else begin
            state_q <= state_d;
            // shift in the outcome; truncation keeps GHR_LEN bits
            if (hist_push) ghr_q <= GHR_LEN'({ghr_q, hist_bit});
        end
    end

    // Datapath registers: no reset (always written before they are used).
    always_ff @(posedge clk_i) begin
        if (pred_fire)         idx_q   <= pred_idx;
        if (state_q == S_RD)   y_q     <= y_comb;
        if (upd_cond_fire)     taken_q <= upd_taken_i;
    end

    // -------------------------------------------------------------------------
    // Unused interface signals (shared interface)
    // -------------------------------------------------------------------------
    /* verilator lint_off UNUSEDSIGNAL */
    logic unused_ok;
    assign unused_ok = ^{upd_pc_i, upd_target_i};
    /* verilator lint_on UNUSEDSIGNAL */

    // -------------------------------------------------------------------------
    // Protocol checks (simulation only)
    // -------------------------------------------------------------------------
`ifdef VERILATOR
    always_ff @(posedge clk_i) begin
        if (rst_ni && upd_valid_i) begin
            if (upd_is_cond_i && state_q != S_WAIT_UPD)
                $error("bp_gp_core: conditional update without a pending prediction");
            if (!upd_is_cond_i && state_q != S_IDLE)
                $error("bp_gp_core: unconditional update while a branch is in flight");
        end
        if (rst_ni && pred_req_valid_i && !pred_req_ready_o)
            $error("bp_gp_core: predict request while a branch is in flight");
    end
`endif

endmodule