// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Global-history perceptron core with a FIXED training threshold (Jimenez &
//   Lin), the RTL counterpart of g_perceptron_rtl.h in the CBP5 simulator
//   (g_perceptron.h restricted to a power-of-two table and a divider-free
//   index). This is the unit that is synthesized: global history, row index,
//   dot product, training decision, weight update and control. The weight
//   table is an SRAM outside this module (one row per perceptron, read and
//   written whole); its ports are this module's I/O.
//
//   Row index (as g_perceptron_rtl.h, p = pc >> PC_SHIFT, IDX_W = log2 N):
//     INDEX_FOLD = 0  idx = p[IDX_W-1:0]                  (a bit select)
//     INDEX_FOLD = 1  idx = XOR of the IDX_W-bit chunks of the 64-bit p
//   The index is computed from the PC in the predict-handshake cycle and the
//   row read starts on that edge (no index stage, as bp_gshare_core).
//
//   Row layout in the SRAM: weight j at bits [j*WEIGHT_BITS +: WEIGHT_BITS],
//   j = 0 the bias, j = i+1 paired with GHR bit i (bit 0 = newest outcome).
//
//   Prediction: y = w0 + sum_i (ghr[i] ? +w(i+1) : -w(i+1)); predict taken
//   iff y >= 0. All-zero initial weights give y = 0, taken. The negations
//   are done in ones' complement: for a two's-complement w, ~w = -w - 1, so
//     y = w0 + sum_i (ghr[i] ? w(i+1) : ~w(i+1)) + zcnt
//   where zcnt is the number of 0 bits in the GHR: the +1 of every negated
//   term, added once. zcnt_q is a counter next to the GHR, updated on every
//   history push, so no incrementer and no popcount sits on the path from
//   the SRAM. Every term then fits WEIGHT_BITS bits.
//
//   Adder tree: the NUM_W terms are split into NG groups of GN (the last
//   one zero-padded); each group is summed by a bp_add_tree, and a second
//   bp_add_tree adds the NG group sums and zcnt_q. With PSUM_REG = 1 the
//   group sums are registered (S_RD -> S_SUM), which splits SRAM clock-to-Q
//   + the group trees from the second tree + the decision. Every width is
//   exact for the worst case (no saturation, no truncation of a reachable
//   value).
//
//   Training (UpdatePredictor): when mispredicted or |y| <= THETA, every
//   weight steps once toward agreement with the outcome (bias toward taken,
//   w(i+1) toward ghr[i] == taken), saturating at the WEIGHT_BITS two's-
//   complement range (bp_sat_ctr, SIGNED). Otherwise the table is untouched.
//   THETA = (THETA_ALPHA_PCT * (193*GHR_LEN + 1400)) / 10000, the C++ integer
//   formula. The history shifts in the outcome after training, on every
//   conditional branch.
//
//   Predict -> update checkpoint (flops):
//     idx_q    the row index
//     decision DEC_CKPT = 1: the prediction and the |y| <= THETA flag (2 bits),
//              evaluated in the cycle y is complete (-THETA <= y <= THETA,
//              two compares in parallel after the tree).
//              DEC_CKPT = 0: y_q (Y_W bits); |y| <= THETA is evaluated at the
//              update handshake.
//     taken_q  the outcome, for the write one cycle after the handshake
//   The weight row is NOT checkpointed: that would be ROW_W flops captured on
//   every prediction, against one extra row read per training event. When
//   training, the row is re-read at idx_q. With several branches in flight
//   (later) the re-read also sees the writes of older branches, so no update
//   is lost to a stale copy; the training direction then needs the
//   predict-time history, which here is ghr_q (one branch in flight).
//   Unchanged rows are written back anyway: a training step leaves a row
//   unchanged only if every weight is saturated toward the outcome.
//
//   Protocol (non-speculative, ONE branch in flight):
//     S_IDLE     - pred_req_ready_o = 1. A predict handshake reads the row at
//                  the index of the PC and registers the index. An
//                  UNCONDITIONAL update (CBP5 TrackOtherInst) is also accepted
//                  here and ignored, as g_perceptron_rtl.h does.
//     S_RD       - the row is valid. PSUM_REG = 0: y is computed (S_DEC).
//                  PSUM_REG = 1: the group sums are registered.
//     S_SUM      - only with PSUM_REG = 1: y from the registered group sums
//                  (S_DEC).
//     (S_DEC)    - the cycle y is complete: the decision checkpoint is
//                  latched; with Y_REG = 0 the prediction is returned.
//     S_PRED     - only with Y_REG = 1: the prediction is returned from the
//                  checkpoint (one cycle later).
//     S_WAIT_UPD - waits for the CONDITIONAL update. Its handshake decides
//                  training from the checkpoint and the outcome. No
//                  training: the history shifts and the core returns to
//                  S_IDLE (no SRAM access). Training: the row is re-read at
//                  idx_q.
//     S_UPD      - the re-read row is valid: the stepped row is written back
//                  and the history shifts.
//   Prediction latency (predict handshake -> pred_resp_valid_o):
//   1 + PSUM_REG + Y_REG cycles. Cycles per conditional branch with
//   back-to-back handshakes: 3 + PSUM_REG + Y_REG, plus 1 when training.
//   Unconditional branch: 1. Per conditional branch: one row read, plus one
//   read and one write per training event.
//
//   Bit-exact with g_perceptron_rtl.h: the re-read returns what
//   UpdatePredictor reads (one branch in flight: nothing writes the row in
//   between), and the GHR used for training (the C++ last_ghr) equals ghr_q
//   until the update ends. The cycle-accurate C++ model of this core was
//   checked against the header (predictions, final table, history) for every
//   PSUM_REG / PSUM_GROUPS / Y_REG / DEC_CKPT / INDEX_FOLD combination tried.
//
//   C++ <-> RTL:  get_perceptron_index() pred_idx; last_index idx_q;
//   get_perceptron_output() terms + i_grp + i_dot (y_comb); last_output
//   y_q or the decision bits; ghr / last_ghr ghr_q; training g_step.
//
//   Interface signals the global perceptron does not use (upd_pc_i,
//   upd_target_i) are kept so all predictor cores share one interface.
//
//   bp_add_tree contract relied on (as the earlier core): N inputs of IN_W-bit
//   two's complement, sum_o = their exact signed sum on IN_W + $clog2(N) bits.
//
// Parameters:
//   GHR_LEN         - global history length (1..64)
//   NUM_PERCEPTRONS - table rows, a power of two (2..2^20)
//   WEIGHT_BITS     - bits per weight (2..8)
//   THETA_ALPHA_PCT - threshold scale in percent (1..1000)
//   PC_SHIFT        - right shift of the PC before indexing (0..3)
//   INDEX_FOLD      - row index: 0 the low bits of the shifted PC, 1 the XOR
//                     fold of the whole 64-bit shifted PC (g_perceptron_rtl.h)
//   PSUM_REG        - 1: register the group sums (one more cycle of latency,
//                     shorter critical path); 0: one combinational tree
//   PSUM_GROUPS     - number of group trees (>= 1; capped at NUM_W / 2 so
//                     every group adds at least 2 terms)
//   Y_REG           - 0: predict in the cycle y is complete; 1: register the
//                     decision first and predict one cycle later
//   DEC_CKPT        - 1: checkpoint 2 decision bits, threshold compare when y
//                     is complete; 0: checkpoint y, compare at the update
//   PC_W            - PC / target width of the shared predictor interface (64)
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_gp_core #(
    parameter int unsigned  GHR_LEN         = 18,
    parameter int unsigned  NUM_PERCEPTRONS = 256,
    parameter int unsigned  WEIGHT_BITS     = 6,
    parameter int unsigned  THETA_ALPHA_PCT = 25,
    parameter int unsigned  PC_SHIFT        = 2,
    parameter bit           INDEX_FOLD      = 1'b0,
    parameter bit           PSUM_REG        = 1'b1,
    parameter int unsigned  PSUM_GROUPS     = 8,
    parameter bit           Y_REG           = 1'b0,
    parameter bit           DEC_CKPT        = 1'b1,
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
    // Elaboration checks (as g_perceptron_rtl.h)
    // -------------------------------------------------------------------------
    if (GHR_LEN < 1 || GHR_LEN > 64) begin : g_check_ghr
        $error("bp_gp_core: GHR_LEN must be in 1..64");
    end
    if (NUM_PERCEPTRONS < 2 || NUM_PERCEPTRONS > (1 << 20)) begin : g_check_rows
        $error("bp_gp_core: NUM_PERCEPTRONS must be in 2..2^20");
    end
    if ((NUM_PERCEPTRONS & (NUM_PERCEPTRONS - 1)) != 0) begin : g_check_pow2
        $error("bp_gp_core: NUM_PERCEPTRONS must be a power of two (g_perceptron_rtl.h)");
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
    if (PSUM_GROUPS < 1) begin : g_check_groups
        $error("bp_gp_core: PSUM_GROUPS must be >= 1");
    end
    if (PC_W != 64) begin : g_check_pcw
        $error("bp_gp_core: the index is defined on a 64-bit PC (PC_W = 64)");
    end

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------
    localparam int unsigned THETA  = (THETA_ALPHA_PCT * (193 * GHR_LEN + 1400)) / 10000;
    localparam int unsigned Y_W    = WEIGHT_BITS + 1 + $clog2(NUM_W);      // holds every y
    // adder tree: NG groups of GN terms (each >= 2), no group only padding
    localparam int unsigned G0     = (PSUM_GROUPS <= NUM_W / 2) ? PSUM_GROUPS
                                   : ((NUM_W / 2 > 0) ? NUM_W / 2 : 1);
    localparam int unsigned GN     = (NUM_W + G0 - 1) / G0;
    localparam int unsigned NG     = (NUM_W + GN - 1) / GN;
    localparam int unsigned NPAD   = NG * GN;                               // terms + padding
    localparam int unsigned GS_W   = WEIGHT_BITS + $clog2(GN);              // exact group sum
    localparam int unsigned CNT_W  = $clog2(GHR_LEN + 1) + 1;               // zcnt, MSB 0
    localparam int unsigned L2_W   = (GS_W > CNT_W) ? GS_W : CNT_W;         // 2nd tree inputs
    localparam int unsigned L2_N   = NG + 1;                                // group sums + zcnt
    localparam int unsigned L2_SW  = L2_W + $clog2(L2_N);                   // 2nd tree sum
    // XOR fold: the IDX_W-bit chunks of the 64 - PC_SHIFT significant bits
    localparam int unsigned FOLD_CHUNKS = (PC_W - PC_SHIFT + IDX_W - 1) / IDX_W;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        S_IDLE     = 3'd0,
        S_RD       = 3'd1,
        S_SUM      = 3'd2,
        S_PRED     = 3'd3,
        S_WAIT_UPD = 3'd4,
        S_UPD      = 3'd5
    } state_e;

    state_e             state_q, state_d;
    logic [GHR_LEN-1:0] ghr_q;     // global history, newest outcome in bit 0
    logic [  CNT_W-1:0] zcnt_q;    // number of 0 bits in ghr_q
    logic [  IDX_W-1:0] idx_q;     // checkpoint: row of the branch in flight
    logic               taken_q;   // its resolved direction
    logic               dec_now;   // y is complete this cycle (S_DEC in the notes)

    assign dec_now = PSUM_REG ? (state_q == S_SUM) : (state_q == S_RD);

    // -------------------------------------------------------------------------
    // Row index (g_perceptron_rtl.h get_perceptron_index)
    // -------------------------------------------------------------------------
    logic [ PC_W-1:0] pc_shifted;
    logic [IDX_W-1:0] pred_idx;

    assign pc_shifted = pred_req_pc_i >> PC_SHIFT;

    if (INDEX_FOLD) begin : g_idx_fold
        always_comb begin
            pred_idx = '0;
            for (int unsigned c = 0; c < FOLD_CHUNKS; c++) begin
                pred_idx ^= IDX_W'(pc_shifted >> (c * IDX_W));
            end
        end
    end else begin : g_idx_low
        assign pred_idx = pc_shifted[IDX_W-1:0];
    end

    // -------------------------------------------------------------------------
    // Dot product (g_perceptron_rtl.h get_perceptron_output)
    //   y = w0 + sum_i (ghr[i] ? w(i+1) : ~w(i+1)) + zcnt
    // -------------------------------------------------------------------------
    logic [NPAD*WEIGHT_BITS-1:0] terms;

    assign terms[0 +: WEIGHT_BITS] = wt_rdata_i[0 +: WEIGHT_BITS];      // bias: + w0

    for (genvar i = 0; i < GHR_LEN; i++) begin : g_term
        // ghr[i] = 1: + w; ghr[i] = 0: ~w (= -w - 1; the +1 is in zcnt_q)
        assign terms[(i+1)*WEIGHT_BITS +: WEIGHT_BITS] =
            wt_rdata_i[(i+1)*WEIGHT_BITS +: WEIGHT_BITS] ^ {WEIGHT_BITS{~ghr_q[i]}};
    end

    if (NPAD > NUM_W) begin : g_pad
        assign terms[NPAD*WEIGHT_BITS-1 : NUM_W*WEIGHT_BITS] = '0;
    end

    // first level: one tree per group
    logic [NG*GS_W-1:0] gsum;

    for (genvar g = 0; g < NG; g++) begin : g_grp
        bp_add_tree #(
            .N    (GN),
            .IN_W (WEIGHT_BITS)
        ) i_grp (
            .in_i (terms[g*GN*WEIGHT_BITS +: GN*WEIGHT_BITS]),
            .sum_o(gsum[g*GS_W +: GS_W])
        );
    end

    // optional pipeline register on the group sums
    logic [NG*GS_W-1:0] gsum_dec;   // the group sums the second tree adds

    if (PSUM_REG) begin : g_psum_reg
        logic [NG*GS_W-1:0] psum_q;

        always_ff @(posedge clk_i) begin
            if (state_q == S_RD) psum_q <= gsum;
        end

        assign gsum_dec = psum_q;
    end else begin : g_psum_comb
        assign gsum_dec = gsum;
    end

    // second level: the group sums (sign-extended) and the zero count
    logic [L2_N*L2_W-1:0] l2_in;
    logic [    L2_SW-1:0] l2_sum;
    logic [      Y_W-1:0] y_comb;

    for (genvar g = 0; g < NG; g++) begin : g_l2_in
        assign l2_in[g*L2_W +: L2_W] = L2_W'($signed(gsum_dec[g*GS_W +: GS_W]));
    end
    assign l2_in[NG*L2_W +: L2_W] = L2_W'(zcnt_q);                     // 0..GHR_LEN

    bp_add_tree #(
        .N    (L2_N),
        .IN_W (L2_W)
    ) i_dot (
        .in_i (l2_in),
        .sum_o(l2_sum)
    );

    assign y_comb = Y_W'($signed(l2_sum));   // exact: y fits Y_W

    // -------------------------------------------------------------------------
    // Decision checkpoint (latched in the cycle y is complete, dec_now) and the
    // training decision (at the update handshake)
    // -------------------------------------------------------------------------
    logic pred_dir;   // the prediction that was returned
    logic low_conf;   // |y| <= THETA
    logic train;

    if (DEC_CKPT) begin : g_ckpt_dec
        localparam int THETA_S = int'(THETA);

        logic signed [31:0] y_s;
        logic               pred_dir_q;
        logic               low_conf_q;

        assign y_s = 32'($signed(y_comb));

        always_ff @(posedge clk_i) begin
            if (dec_now) begin
                pred_dir_q <= ~y_comb[Y_W-1];
                low_conf_q <= (y_s <= THETA_S) && (y_s >= -THETA_S);
            end
        end

        assign pred_dir = pred_dir_q;
        assign low_conf = low_conf_q;
    end else begin : g_ckpt_y
        logic [Y_W-1:0] y_q;     // perceptron output (signed)
        logic [Y_W-1:0] abs_y;

        always_ff @(posedge clk_i) begin
            if (dec_now) y_q <= y_comb;
        end

        assign pred_dir = ~y_q[Y_W-1];
        assign abs_y    = y_q[Y_W-1] ? (~y_q + Y_W'(1)) : y_q;
        assign low_conf = (32'(abs_y) <= THETA);
    end

    assign train = (pred_dir != upd_taken_i) || low_conf;

    // -------------------------------------------------------------------------
    // Handshakes
    // -------------------------------------------------------------------------
    logic pred_fire;       // predict request accepted
    logic upd_cond_fire;   // conditional update accepted

    assign pred_req_ready_o  = (state_q == S_IDLE);
    assign upd_ready_o       = (state_q == S_IDLE) || (state_q == S_WAIT_UPD);

    if (Y_REG) begin : g_resp_reg
        assign pred_resp_valid_o = (state_q == S_PRED);
        assign pred_resp_taken_o = pred_dir;
    end else begin : g_resp_comb
        assign pred_resp_valid_o = dec_now;
        assign pred_resp_taken_o = ~y_comb[Y_W-1];
    end

    assign pred_fire     = pred_req_valid_i && pred_req_ready_o;
    assign upd_cond_fire = upd_valid_i && upd_is_cond_i && (state_q == S_WAIT_UPD);

    // -------------------------------------------------------------------------
    // Weight table access: read at the predict handshake (index from the PC),
    // re-read and write when training (index from the checkpoint)
    // -------------------------------------------------------------------------
    assign wt_re_o    = pred_fire || (upd_cond_fire && train);
    assign wt_raddr_o = (state_q == S_IDLE) ? pred_idx : idx_q;
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
            S_IDLE:     if (pred_fire) state_d = S_RD;
            S_RD: begin
                if (PSUM_REG)   state_d = S_SUM;
                else if (Y_REG) state_d = S_PRED;
                else            state_d = S_WAIT_UPD;
            end
            S_SUM:      state_d = Y_REG ? S_PRED : S_WAIT_UPD;
            S_PRED:     state_d = S_WAIT_UPD;
            S_WAIT_UPD: if (upd_cond_fire) state_d = train ? S_UPD : S_IDLE;
            S_UPD:      state_d = S_IDLE;
            default:    state_d = S_IDLE;
        endcase
    end

    // History push: at the update handshake when not training, after the
    // write-back when training. Either way after the training decision.
    logic hist_push;
    logic hist_bit;
    logic zero_in;    // a 0 enters the history
    logic zero_out;   // a 0 leaves it

    assign hist_push = (upd_cond_fire && !train) || (state_q == S_UPD);
    assign hist_bit  = (state_q == S_UPD) ? taken_q : upd_taken_i;
    assign zero_in   = ~hist_bit;
    assign zero_out  = ~ghr_q[GHR_LEN-1];

    // Control state, history and its zero count: reset.
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q <= S_IDLE;
            ghr_q   <= '0;
            zcnt_q  <= CNT_W'(GHR_LEN);          // the reset history is all 0
        end else begin
            state_q <= state_d;
            if (hist_push) begin
                // shift in the outcome; truncation keeps GHR_LEN bits
                ghr_q  <= GHR_LEN'({ghr_q, hist_bit});
                zcnt_q <= zcnt_q + CNT_W'(zero_in) - CNT_W'(zero_out);
            end
        end
    end

    // Datapath registers: no reset (always written before they are used).
    always_ff @(posedge clk_i) begin
        if (pred_fire)     idx_q   <= pred_idx;
        if (upd_cond_fire) taken_q <= upd_taken_i;
    end

    // -------------------------------------------------------------------------
    // Unused interface signals (shared interface) and the shifted-PC bits the
    // low-bits index does not use
    // -------------------------------------------------------------------------
    /* verilator lint_off UNUSEDSIGNAL */
    logic unused_ok;
    assign unused_ok = ^{upd_pc_i, upd_target_i, pc_shifted};
    /* verilator lint_on UNUSEDSIGNAL */

    // -------------------------------------------------------------------------
    // Protocol and invariant checks (simulation only)
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
        if (rst_ni && zcnt_q != CNT_W'($countones(~ghr_q)))
            $error("bp_gp_core: zero count %0d != zeros in the history", zcnt_q);
    end
`endif

endmodule