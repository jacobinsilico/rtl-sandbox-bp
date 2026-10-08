// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Hashed perceptron core, the RTL counterpart of hashed_perceptron.h +
//   hp/hp_core.h in the CBP5 simulator (plain HP: HP_HASH=1, no loop, no
//   agree), bit-exact. This is the unit that is synthesized: global and path
//   history, folded histories, index hash, sum, training decision, weight
//   update and adaptive threshold. The bias table and the NUM_TABLES weight
//   tables are SRAMs outside this module; their ports are its I/O.
//
//   Prediction (GetPrediction): every index is hashed from the PC and the
//   history (bp_hp_hash per table; bias: XOR-fold of pcmix(pc)) and
//   registered before the table read, as in bp_gp_core. Then
//     y = bias + sum_t W_t[idx_t]   (bp_add_tree), predict taken iff y >= 0.
//
//   Update (UpdatePredictor), in the C++ order:
//     train = mispredicted || |y| <= theta       (old theta)
//     if train: every consulted weight steps toward the outcome (re-read,
//               bp_sat_ctr per table width, written back)
//     O-GEHL theta adaptation (TC counter), still with the old theta
//     history push: global history, path history, folded histories.
//   Unconditional branches (TrackOtherInst) change nothing, as in the C++.
//
//   Table ports: table t uses address bits [AOFF(t) +: LOG_t] and data bits
//   [DOFF(t) +: WBITS_t] of the packed wt_* vectors (AOFF / DOFF = sum of
//   the widths of the tables before t), so the port widths are exact.
//
//   State kept between predict and update (project-wide rule): the indices
//   and y in flops; the weights are re-read when training.
//
//   Protocol (non-speculative, ONE branch in flight, as bp_gp_core):
//     S_IDLE - predict handshake registers every index; an unconditional
//              update is accepted and ignored.
//     S_IDX  - every table is read at the registered indices.
//     S_RD   - y is computed and latched (Y_REG = 0: prediction returned).
//     S_PRED - only with Y_REG = 1: prediction from the registered y.
//     S_WAIT - conditional update handshake: no training -> theta update
//              and history push now, back to S_IDLE; training -> re-read.
//     S_UPD  - stepped weights written back; theta update, history push.
//   Cycles per conditional branch: 4 (no training) or 5 (training), plus one
//   with Y_REG = 1. Unconditional branch: 1.
//
// Parameters:
//   NUM_TABLES         - weight tables (1..8)
//   T0_LOG..T7_LOG     - log2 entries of each table (2..24; 0 for t >=
//                        NUM_TABLES)  (C++ TABLE_LOGS)
//   T0_WBITS..T7_WBITS - weight width of each table (2..8; 0 for t >=
//                        NUM_TABLES)  (C++ TABLE_WBITS)
//   T0_HIST..T7_HIST   - history length of each table in bits, from the
//                        PREDICTOR_HIST_LENS line of the CBP5 .log (0 unused)
//   BIAS_ENTRIES       - bias table entries (power of two >= 2)
//   BIAS_WEIGHT_BITS   - bias weight width (2..8)
//   THETA_BITS         - theta register width
//   TC_BITS            - threshold counter width (signed)
//   HP_FOLDS           - folded histories per table (1..3)
//   HP_PATH_BITS       - path history width (0..31; 0 = none)
//   HP_PCMIX           - 1: q ^ (q >> 3), q = pc >> HP_PC_SHIFT; 0: pc ^ pc>>2
//   HP_PC_SHIFT        - PC shift of HP_PCMIX = 1 (0..4)
//   Y_REG              - 0: predict in the cycle the weights arrive; 1: one
//                        cycle later from the registered y
//   PC_W               - PC / target width of the shared interface (64)
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_hp_core #(
    parameter int unsigned  NUM_TABLES       = 6,
    parameter int unsigned  T0_LOG           = 8,
    parameter int unsigned  T1_LOG           = 8,
    parameter int unsigned  T2_LOG           = 8,
    parameter int unsigned  T3_LOG           = 8,
    parameter int unsigned  T4_LOG           = 8,
    parameter int unsigned  T5_LOG           = 8,
    parameter int unsigned  T6_LOG           = 0,
    parameter int unsigned  T7_LOG           = 0,
    parameter int unsigned  T0_WBITS         = 3,
    parameter int unsigned  T1_WBITS         = 3,
    parameter int unsigned  T2_WBITS         = 3,
    parameter int unsigned  T3_WBITS         = 3,
    parameter int unsigned  T4_WBITS         = 3,
    parameter int unsigned  T5_WBITS         = 3,
    parameter int unsigned  T6_WBITS         = 0,
    parameter int unsigned  T7_WBITS         = 0,
    parameter int unsigned  T0_HIST          = 2,
    parameter int unsigned  T1_HIST          = 3,
    parameter int unsigned  T2_HIST          = 5,
    parameter int unsigned  T3_HIST          = 7,
    parameter int unsigned  T4_HIST          = 11,
    parameter int unsigned  T5_HIST          = 16,
    parameter int unsigned  T6_HIST          = 0,
    parameter int unsigned  T7_HIST          = 0,
    parameter int unsigned  BIAS_ENTRIES     = 256,
    parameter int unsigned  BIAS_WEIGHT_BITS = 5,
    parameter int unsigned  THETA_BITS       = 12,
    parameter int unsigned  TC_BITS          = 7,
    parameter int unsigned  HP_FOLDS         = 3,
    parameter int unsigned  HP_PATH_BITS     = 16,
    parameter int unsigned  HP_PCMIX         = 1,
    parameter int unsigned  HP_PC_SHIFT      = 2,
    parameter bit           Y_REG            = 1'b0,
    parameter int unsigned  PC_W             = 64,
    localparam int unsigned BIAS_LOG         = $clog2(BIAS_ENTRIES),
    localparam int unsigned ADDR_TOT         = T0_LOG + T1_LOG + T2_LOG + T3_LOG
                                             + T4_LOG + T5_LOG + T6_LOG + T7_LOG,
    localparam int unsigned DATA_TOT         = T0_WBITS + T1_WBITS + T2_WBITS + T3_WBITS
                                             + T4_WBITS + T5_WBITS + T6_WBITS + T7_WBITS
) (
    input  logic                        clk_i,
    input  logic                        rst_ni,

    // predict channel (CBP5 GetPrediction)
    input  logic                        pred_req_valid_i,
    output logic                        pred_req_ready_o,
    input  logic [            PC_W-1:0] pred_req_pc_i,
    output logic                        pred_resp_valid_o,
    output logic                        pred_resp_taken_o,

    // update channel (CBP5 UpdatePredictor / TrackOtherInst)
    input  logic                        upd_valid_i,
    output logic                        upd_ready_o,
    input  logic                        upd_is_cond_i,
    input  logic [            PC_W-1:0] upd_pc_i,
    input  logic                        upd_taken_i,
    input  logic [            PC_W-1:0] upd_target_i,

    // bias table SRAM (1R1W, synchronous read)
    output logic                        bias_re_o,
    output logic [        BIAS_LOG-1:0] bias_raddr_o,
    input  logic [BIAS_WEIGHT_BITS-1:0] bias_rdata_i,
    output logic                        bias_we_o,
    output logic [        BIAS_LOG-1:0] bias_waddr_o,
    output logic [BIAS_WEIGHT_BITS-1:0] bias_wdata_o,

    // weight table SRAMs (1R1W, synchronous read), packed per table
    output logic [      NUM_TABLES-1:0] wt_re_o,
    output logic [        ADDR_TOT-1:0] wt_raddr_o,
    input  logic [        DATA_TOT-1:0] wt_rdata_i,
    output logic [      NUM_TABLES-1:0] wt_we_o,
    output logic [        ADDR_TOT-1:0] wt_waddr_o,
    output logic [        DATA_TOT-1:0] wt_wdata_o
);

    // -------------------------------------------------------------------------
    // Per-table constants (elaboration time)
    // -------------------------------------------------------------------------
    function automatic int unsigned t_log(input int unsigned t);
        case (t)
            0:       t_log = T0_LOG;
            1:       t_log = T1_LOG;
            2:       t_log = T2_LOG;
            3:       t_log = T3_LOG;
            4:       t_log = T4_LOG;
            5:       t_log = T5_LOG;
            6:       t_log = T6_LOG;
            7:       t_log = T7_LOG;
            default: t_log = 0;
        endcase
    endfunction

    function automatic int unsigned t_wbits(input int unsigned t);
        case (t)
            0:       t_wbits = T0_WBITS;
            1:       t_wbits = T1_WBITS;
            2:       t_wbits = T2_WBITS;
            3:       t_wbits = T3_WBITS;
            4:       t_wbits = T4_WBITS;
            5:       t_wbits = T5_WBITS;
            6:       t_wbits = T6_WBITS;
            7:       t_wbits = T7_WBITS;
            default: t_wbits = 0;
        endcase
    endfunction

    function automatic int unsigned t_hist(input int unsigned t);
        case (t)
            0:       t_hist = T0_HIST;
            1:       t_hist = T1_HIST;
            2:       t_hist = T2_HIST;
            3:       t_hist = T3_HIST;
            4:       t_hist = T4_HIST;
            5:       t_hist = T5_HIST;
            6:       t_hist = T6_HIST;
            7:       t_hist = T7_HIST;
            default: t_hist = 0;
        endcase
    endfunction

    // offset of table t in the packed address / data vectors
    function automatic int unsigned a_off(input int unsigned t);
        a_off = 0;
        for (int unsigned j = 0; j < t; j++) a_off = a_off + t_log(j);
    endfunction

    function automatic int unsigned d_off(input int unsigned t);
        d_off = 0;
        for (int unsigned j = 0; j < t; j++) d_off = d_off + t_wbits(j);
    endfunction

    // compressed width of folded history k of table t: as Core::init(),
    // log + 1 + offset, the offset moved on while it clashes with an
    // earlier fold of the same table
    function automatic int unsigned f_clen(input int unsigned t, input int unsigned k);
        int unsigned lg, span, off, c0, c1, c2, cand;
        bit          done, clash;
        lg   = t_log(t);
        span = 31 - lg;
        c0   = 0;
        c1   = 0;
        c2   = 0;
        for (int unsigned kk = 0; kk <= k; kk++) begin
            off  = (t * 13 + kk * 7) % span;
            done = 1'b0;
            for (int unsigned tries = 0; tries < span; tries++) begin
                if (!done) begin
                    cand  = lg + 1 + off;
                    clash = ((kk > 0) && (c0 == cand)) || ((kk > 1) && (c1 == cand));
                    if (!clash) done = 1'b1;
                    else        off  = (off + 1) % span;
                end
            end
            if (kk == 0)      c0 = lg + 1 + off;
            else if (kk == 1) c1 = lg + 1 + off;
            else              c2 = lg + 1 + off;
        end
        if (k == 0)      f_clen = c0;
        else if (k == 1) f_clen = c1;
        else             f_clen = c2;
    endfunction

    // rotation of folded history k of table t into the index
    function automatic int unsigned f_rot(input int unsigned t, input int unsigned k);
        f_rot = ((k * t_log(t)) / HP_FOLDS + (t % 3)) % t_log(t);
    endfunction

    function automatic int unsigned max_wbits();
        max_wbits = BIAS_WEIGHT_BITS;
        for (int unsigned t = 0; t < NUM_TABLES; t++)
            if (t_wbits(t) > max_wbits) max_wbits = t_wbits(t);
    endfunction

    localparam int unsigned GH_LEN     = t_hist(NUM_TABLES - 1);   // longest history
    localparam int unsigned PH_W       = (HP_PATH_BITS > 0) ? HP_PATH_BITS : 1;
    localparam int unsigned NUM_W      = NUM_TABLES + 1;            // summed weights
    localparam int unsigned TERM_W     = max_wbits();
    localparam int unsigned Y_W        = TERM_W + $clog2(NUM_W);
    localparam int unsigned THETA_INIT = (193 * NUM_W) / 100 + 14;
    localparam int unsigned THETA_MAX  = (1 << THETA_BITS) - 1;
    localparam logic [TC_BITS-1:0] TC_MAX = {1'b0, {(TC_BITS - 1){1'b1}}};
    localparam logic [TC_BITS-1:0] TC_MIN = {1'b1, {(TC_BITS - 1){1'b0}}};

    // -------------------------------------------------------------------------
    // Elaboration checks (as hp_core.h, plus the RTL limits)
    // -------------------------------------------------------------------------
    if (NUM_TABLES < 1 || NUM_TABLES > 8) begin : g_check_tables
        $error("bp_hp_core: NUM_TABLES must be in 1..8");
    end
    for (genvar t = 0; t < 8; t++) begin : g_check_t
        if (t < NUM_TABLES) begin : g_used
            if (t_log(t) < 2 || t_log(t) > 24 || t_wbits(t) < 2 || t_wbits(t) > 8
                || t_hist(t) < 1) begin : g_bad_geom
                $error("bp_hp_core: every used table needs LOG 2..24, WBITS 2..8, HIST >= 1");
            end
            if (t > 0 && t_hist(t) <= t_hist(t - 1)) begin : g_bad_hist
                $error("bp_hp_core: T<i>_HIST must be strictly increasing (C++ deduplicates)");
            end
        end else begin : g_unused
            if (t_log(t) != 0 || t_wbits(t) != 0) begin : g_bad_unused
                $error("bp_hp_core: T<i>_LOG / T<i>_WBITS of unused tables must be 0");
            end
        end
    end
    if (BIAS_ENTRIES < 2 || (BIAS_ENTRIES & (BIAS_ENTRIES - 1)) != 0) begin : g_check_bias
        $error("bp_hp_core: BIAS_ENTRIES must be a power of two >= 2");
    end
    if (BIAS_WEIGHT_BITS < 2 || BIAS_WEIGHT_BITS > 8) begin : g_check_bw
        $error("bp_hp_core: BIAS_WEIGHT_BITS must be in 2..8");
    end
    if (THETA_BITS < 1 || THETA_BITS > 30 || THETA_INIT > THETA_MAX) begin : g_check_theta
        $error("bp_hp_core: THETA_BITS too small for THETA_INIT (or out of 1..30)");
    end
    if (TC_BITS < 2 || TC_BITS > 30) begin : g_check_tc
        $error("bp_hp_core: TC_BITS must be in 2..30");
    end
    if (HP_FOLDS < 1 || HP_FOLDS > 3) begin : g_check_folds
        $error("bp_hp_core: HP_FOLDS must be in 1..3");
    end
    if (HP_PATH_BITS > 31) begin : g_check_path
        $error("bp_hp_core: HP_PATH_BITS must be in 0..31");
    end
    if (HP_PCMIX > 1 || HP_PC_SHIFT > 4) begin : g_check_pcmix
        $error("bp_hp_core: HP_PCMIX must be 0/1 and HP_PC_SHIFT 0..4");
    end
    if (PC_W != 64) begin : g_check_pcw
        $error("bp_hp_core: the PC mix uses a 64-bit PC (PC_W = 64)");
    end

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        S_IDLE = 3'd0,
        S_IDX  = 3'd1,
        S_RD   = 3'd2,
        S_PRED = 3'd3,
        S_WAIT = 3'd4,
        S_UPD  = 3'd5
    } state_e;

    state_e                  state_q, state_d;
    logic [  GH_LEN-1:0]     gh_q;        // global history, gh_q[0] = newest
    logic [    PH_W-1:0]     phist_q;     // path history (HP_PATH_BITS > 0)
    logic [THETA_BITS-1:0]   theta_q;
    logic [ TC_BITS-1:0]     tc_q;        // two's complement
    logic [BIAS_LOG-1:0]     bidx_q;      // indices of the branch in flight
    logic [ADDR_TOT-1:0]     idx_q;
    logic [     Y_W-1:0]     y_q;         // its sum (signed)
    logic                    taken_q;     // its outcome (training only)
    logic                    pbit_q;      // its path bit (training only)

    // -------------------------------------------------------------------------
    // Handshakes
    // -------------------------------------------------------------------------
    logic pred_fire, upd_cond_fire;

    assign pred_req_ready_o = (state_q == S_IDLE);
    assign upd_ready_o      = (state_q == S_IDLE) || (state_q == S_WAIT);
    assign pred_fire        = pred_req_valid_i && pred_req_ready_o;
    assign upd_cond_fire    = upd_valid_i && upd_is_cond_i && (state_q == S_WAIT);

    // -------------------------------------------------------------------------
    // History push (theta update at the same time): at the update handshake
    // when not training, after the write-back when training
    // -------------------------------------------------------------------------
    logic train;
    logic hist_push;
    logic hist_bit;     // outcome pushed into the global history
    logic path_bit;     // pc[0] ^ pc[2] ^ pc[5] pushed into the path history
    logic upd_pbit;

    assign upd_pbit  = upd_pc_i[0] ^ upd_pc_i[2] ^ upd_pc_i[5];
    assign hist_push = (upd_cond_fire && !train) || (state_q == S_UPD);
    assign hist_bit  = (state_q == S_UPD) ? taken_q : upd_taken_i;
    assign path_bit  = (state_q == S_UPD) ? pbit_q  : upd_pbit;

    // -------------------------------------------------------------------------
    // Folded histories (HP_FOLDS per table) and the index hash
    // -------------------------------------------------------------------------
    logic [63:0] pc_mix;   // hp::pcmix
    if (HP_PCMIX != 0) begin : g_pcmix1
        logic [63:0] q;
        assign q      = pred_req_pc_i >> HP_PC_SHIFT;
        assign pc_mix = q ^ (q >> 3);
    end else begin : g_pcmix0
        assign pc_mix = pred_req_pc_i ^ (pred_req_pc_i >> 2);
    end

    logic [30:0] phist_ext;
    assign phist_ext = (HP_PATH_BITS > 0) ? 31'(phist_q) : 31'd0;

    logic [ADDR_TOT-1:0] idx_d;

    for (genvar t = 0; t < NUM_TABLES; t++) begin : g_tbl
        localparam int unsigned LG = t_log(t);
        localparam int unsigned HL = t_hist(t);

        logic [2:0][30:0] fc;   // folded histories, zero-extended

        for (genvar k = 0; k < 3; k++) begin : g_fold
            if (k < HP_FOLDS) begin : g_on
                localparam int unsigned CL = f_clen(t, k);
                logic [CL-1:0] comp;
                bp_folded_hist #(
                    .OLEN  (HL),
                    .CLEN  (CL),
                    .NPUSH (1)
                ) i_fold (
                    .clk_i (clk_i),
                    .rst_ni(rst_ni),
                    .push_i(hist_push),
                    .in_i  (hist_bit),
                    .out_i (gh_q[HL-1]),     // bit leaving the HL-bit window
                    .comp_o(comp)
                );
                assign fc[k] = 31'(comp);
            end else begin : g_off
                assign fc[k] = '0;
            end
        end

        bp_hp_hash #(
            .TBL      (t),
            .LG       (LG),
            .FOLDS    (HP_FOLDS),
            .ROT0     (f_rot(t, 0)),
            .ROT1     ((HP_FOLDS > 1) ? f_rot(t, 1) : 0),
            .ROT2     ((HP_FOLDS > 2) ? f_rot(t, 2) : 0),
            .HLEN     (HL),
            .PATH_BITS(HP_PATH_BITS)
        ) i_hash (
            .a_i    (pc_mix),
            .f0_i   (fc[0]),
            .f1_i   (fc[1]),
            .f2_i   (fc[2]),
            .phist_i(phist_ext),
            .idx_o  (idx_d[a_off(t) +: LG])
        );
    end

    // bias index: XOR of every BIAS_LOG-bit chunk of pcmix(pc)
    logic [BIAS_LOG-1:0] bidx_d;
    always_comb begin
        bidx_d = '0;
        for (int unsigned c = 0; c * BIAS_LOG < 64; c++)
            bidx_d ^= BIAS_LOG'(pc_mix >> (c * BIAS_LOG));
    end

    // -------------------------------------------------------------------------
    // Sum: y = bias + sum of the table weights (each sign-extended)
    // -------------------------------------------------------------------------
    logic [NUM_W*TERM_W-1:0] terms;
    logic [         Y_W-1:0] y_comb;

    assign terms[0 +: TERM_W] = TERM_W'($signed(bias_rdata_i));
    for (genvar t = 0; t < NUM_TABLES; t++) begin : g_term
        assign terms[(t+1)*TERM_W +: TERM_W] =
            TERM_W'($signed(wt_rdata_i[d_off(t) +: t_wbits(t)]));
    end

    bp_add_tree #(
        .N    (NUM_W),
        .IN_W (TERM_W)
    ) i_sum (
        .in_i (terms),
        .sum_o(y_comb)
    );

    // -------------------------------------------------------------------------
    // Training decision and theta adaptation (both with the old theta)
    // -------------------------------------------------------------------------
    logic           pred_dir_q;     // perceptron prediction of the branch
    logic [Y_W-1:0] abs_y;
    logic           low_conf;       // |y| <= theta
    logic           mispred;        // against the pushed outcome

    assign pred_dir_q = ~y_q[Y_W-1];
    assign abs_y      = y_q[Y_W-1] ? (~y_q + Y_W'(1)) : y_q;
    assign low_conf   = (32'(abs_y) <= 32'(theta_q));
    assign train      = (pred_dir_q != upd_taken_i) || low_conf;
    assign mispred    = (pred_dir_q != hist_bit);

    logic [THETA_BITS-1:0] theta_d;
    logic [   TC_BITS-1:0] tc_d, tc_inc, tc_dec;

    assign tc_inc = tc_q + TC_BITS'(1);
    assign tc_dec = tc_q - TC_BITS'(1);

    always_comb begin
        theta_d = theta_q;
        tc_d    = tc_q;
        if (hist_push) begin
            if (mispred) begin
                if (tc_inc == TC_MAX) begin
                    if (32'(theta_q) < THETA_MAX) theta_d = theta_q + THETA_BITS'(1);
                    tc_d = '0;
                end else begin
                    tc_d = tc_inc;
                end
            end else if (low_conf) begin
                if (tc_dec == TC_MIN) begin
                    if (theta_q != '0) theta_d = theta_q - THETA_BITS'(1);
                    tc_d = '0;
                end else begin
                    tc_d = tc_dec;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Weight tables: read at predict (registered indices), re-read and
    // written back when training
    // -------------------------------------------------------------------------
    logic rd_en, wr_en;

    assign rd_en = (state_q == S_IDX) || (upd_cond_fire && train);
    assign wr_en = (state_q == S_UPD);

    assign bias_re_o    = rd_en;
    assign bias_raddr_o = bidx_q;
    assign bias_we_o    = wr_en;
    assign bias_waddr_o = bidx_q;

    bp_sat_ctr #(
        .WIDTH (BIAS_WEIGHT_BITS),
        .SIGNED(1'b1)
    ) i_bias_step (
        .ctr_i (bias_rdata_i),
        .inc_i (taken_q),
        .ctr_o (bias_wdata_o)
    );

    assign wt_re_o    = {NUM_TABLES{rd_en}};
    assign wt_raddr_o = idx_q;
    assign wt_we_o    = {NUM_TABLES{wr_en}};
    assign wt_waddr_o = idx_q;

    for (genvar t = 0; t < NUM_TABLES; t++) begin : g_step
        bp_sat_ctr #(
            .WIDTH (t_wbits(t)),
            .SIGNED(1'b1)
        ) i_step (
            .ctr_i (wt_rdata_i[d_off(t) +: t_wbits(t)]),
            .inc_i (taken_q),
            .ctr_o (wt_wdata_o[d_off(t) +: t_wbits(t)])
        );
    end

    // -------------------------------------------------------------------------
    // Prediction response
    // -------------------------------------------------------------------------
    if (Y_REG) begin : g_resp_reg
        assign pred_resp_valid_o = (state_q == S_PRED);
        assign pred_resp_taken_o = ~y_q[Y_W-1];
    end else begin : g_resp_comb
        assign pred_resp_valid_o = (state_q == S_RD);
        assign pred_resp_taken_o = ~y_comb[Y_W-1];
    end

    // -------------------------------------------------------------------------
    // Control
    // -------------------------------------------------------------------------
    always_comb begin
        state_d = state_q;
        case (state_q)
            S_IDLE: if (pred_fire)     state_d = S_IDX;
            S_IDX:                     state_d = S_RD;
            S_RD:                      state_d = Y_REG ? S_PRED : S_WAIT;
            S_PRED:                    state_d = S_WAIT;
            S_WAIT: if (upd_cond_fire) state_d = train ? S_UPD : S_IDLE;
            S_UPD:                     state_d = S_IDLE;
            default:                   state_d = S_IDLE;
        endcase
    end

    // Control state, histories and threshold: reset.
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q <= S_IDLE;
            gh_q    <= '0;
            phist_q <= '0;
            theta_q <= THETA_BITS'(THETA_INIT);
            tc_q    <= '0;
        end else begin
            state_q <= state_d;
            theta_q <= theta_d;
            tc_q    <= tc_d;
            if (hist_push) begin
                gh_q <= GH_LEN'({gh_q, hist_bit});
                if (HP_PATH_BITS > 0) phist_q <= PH_W'({phist_q, path_bit});
            end
        end
    end

    // Datapath registers: no reset (always written before they are used).
    always_ff @(posedge clk_i) begin
        if (pred_fire) begin
            idx_q  <= idx_d;
            bidx_q <= bidx_d;
        end
        if (state_q == S_RD) y_q <= y_comb;
        if (upd_cond_fire) begin
            taken_q <= upd_taken_i;
            pbit_q  <= upd_pbit;
        end
    end

    // -------------------------------------------------------------------------
    // Unused interface signals (shared interface)
    // -------------------------------------------------------------------------
    /* verilator lint_off UNUSEDSIGNAL */
    logic unused_ok;
    assign unused_ok = ^{pred_req_pc_i, upd_pc_i[PC_W-1:6], upd_pc_i[4:3], upd_pc_i[1],
                         upd_target_i, phist_q};
    /* verilator lint_on UNUSEDSIGNAL */

    // -------------------------------------------------------------------------
    // Protocol checks (simulation only)
    // -------------------------------------------------------------------------
`ifdef VERILATOR
    always_ff @(posedge clk_i) begin
        if (rst_ni && upd_valid_i) begin
            if (upd_is_cond_i && state_q != S_WAIT)
                $error("bp_hp_core: conditional update without a pending prediction");
            if (!upd_is_cond_i && state_q != S_IDLE)
                $error("bp_hp_core: unconditional update while a branch is in flight");
        end
        if (rst_ni && pred_req_valid_i && !pred_req_ready_o)
            $error("bp_hp_core: predict request while a branch is in flight");
    end
`endif

endmodule
