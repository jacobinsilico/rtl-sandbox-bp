// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Cookbook TAGE predictor core ("TAGE: an engineering cookbook", Seznec,
//   INRIA RR-9561, 2024), bit-exact with tage_cb.h built with CB_SC=0,
//   AHEAD=0, CB_OPTTAGE=1, CB_ILEN_CAP=1 (STATIC_TARGET_FIX=1 traces). This
//   is the unit that is synthesized; every table is an SRAM outside it and
//   the SRAM ports are its I/O. Stage 1: direct-mapped (LOGASSOC = 0) only.
//
//   Block-based prediction: a fetch block ends at a taken branch or after 4
//   branches. The index of every odd logical table and the tag of every
//   table are hashed once per block (S_HASH, after the history update) and
//   held in flops; branch number Numero (num_q) is XORed in per branch. The
//   branch PC itself is not used.
//
//   Storage (C++ arrays -> banks). CB_ADJACENT: T(2k-1) and T(2k) share an
//   index, so their entries sit side by side in one row (low entry = odd
//   table). CB_SHARED: the first NHIST-SH_OFF arrays are doubled and shared
//   with T(i+SH_OFF); the doubled arrays are split into two banks by the
//   index LSB (X for the low tables, X^1 for their partners), so both are
//   read in one cycle. Bank order: doubled pairs (half 0, half 1), then the
//   other pairs. Entry = {tag, u[1:0], ctr[2:0]} (ctr two's complement).
//   Bimodal: pred bits (2^LOGB x 1) and hysteresis (2^(LOGB-1) x 2).
//
//   Protocol (non-speculative, one branch in flight, as gshare):
//     S_IDLE  predict handshake reads every bank -> S_PRED (response), or an
//             unconditional update (TrackOtherInst) -> S_HIST.
//     S_WAIT  conditional update handshake -> the update FSM, which mirrors
//             UpdatePredictorCore statement by statement, one MYRANDOM call
//             per cycle at most. Table contents are re-read at update:
//             every bank is marked stale at the handshake and re-read on
//             first use (and after each write to it, so read-modify-writes
//             of a shared row always see the previous write).
//     S_SWEEP the global u decrement (TICK >= 4096): all banks, one row per
//             cycle, read/write pipelined; 2^LOGG + 1 cycles.
//     S_HIST  history update (block end: path history, 4 global history
//             bits, folded histories) -> S_HASH (block hash) -> S_IDLE.
//   upd_ready_o / pred_req_ready_o are low while the update runs.
//
// Parameters:
//   NHIST              - logical tagged tables (even, 4..14)
//   LOGT               - log2 entries of a logical table (C++ LOGT)
//   LOGASSOC           - 0 (stage 1)
//   LOGB               - log2 bimodal entries
//   TBITS              - tag bits
//   CB_LMP             - 1: predict LongestMatchPred (C++ CB_LMP with CB_SC=0)
//   ILEN_CAP           - 1: index folds capped at 3*index width (CB_ILEN_CAP)
//   T1_HIST..T14_HIST  - history length of each table in bits, as printed by
//                        the CBP5 run (PREDICTOR_HIST_LENS); unused ones 0
//   PC_W               - PC / target width of the shared interface
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_tage_core #(
    parameter int unsigned  NHIST    = 12,
    parameter int unsigned  LOGT     = 6,
    parameter int unsigned  LOGASSOC = 0,
    parameter int unsigned  LOGB     = 11,
    parameter int unsigned  TBITS    = 10,
    parameter int unsigned  CB_LMP   = 0,
    parameter int unsigned  ILEN_CAP = 1,
    parameter int unsigned  T1_HIST  = 4,
    parameter int unsigned  T2_HIST  = 8,
    parameter int unsigned  T3_HIST  = 20,
    parameter int unsigned  T4_HIST  = 24,
    parameter int unsigned  T5_HIST  = 28,
    parameter int unsigned  T6_HIST  = 32,
    parameter int unsigned  T7_HIST  = 36,
    parameter int unsigned  T8_HIST  = 40,
    parameter int unsigned  T9_HIST  = 44,
    parameter int unsigned  T10_HIST = 48,
    parameter int unsigned  T11_HIST = 80,
    parameter int unsigned  T12_HIST = 104,
    parameter int unsigned  T13_HIST = 0,
    parameter int unsigned  T14_HIST = 0,
    parameter int unsigned  PC_W     = 64,
    localparam int unsigned LOGG     = LOGT - LOGASSOC,
    localparam int unsigned SH_OFF   = 2 * ((NHIST / 2 + 1) / 2),
    localparam int unsigned NB       = (NHIST - SH_OFF) / 2 + SH_OFF / 2,
    localparam int unsigned ROW_W    = 2 * (TBITS + 5)
) (
    input  logic                       clk_i,
    input  logic                       rst_ni,

    // predict channel (CBP5 GetPrediction)
    input  logic                       pred_req_valid_i,
    output logic                       pred_req_ready_o,
    input  logic [           PC_W-1:0] pred_req_pc_i,
    output logic                       pred_resp_valid_o,
    output logic                       pred_resp_taken_o,

    // update channel (CBP5 UpdatePredictor / TrackOtherInst)
    input  logic                       upd_valid_i,
    output logic                       upd_ready_o,
    input  logic                       upd_is_cond_i,
    input  logic [           PC_W-1:0] upd_pc_i,
    input  logic                       upd_taken_i,
    input  logic [           PC_W-1:0] upd_target_i,

    // tagged-table banks (1R1W, synchronous read): 2^LOGG rows x ROW_W
    output logic [NB-1:0]              tb_re_o,
    output logic [NB-1:0][ LOGG-1:0]   tb_raddr_o,
    input  logic [NB-1:0][ROW_W-1:0]   tb_rdata_i,
    output logic [NB-1:0]              tb_we_o,
    output logic [NB-1:0][ LOGG-1:0]   tb_waddr_o,
    output logic [NB-1:0][ROW_W-1:0]   tb_wdata_o,

    // bimodal prediction bits: 2^LOGB x 1
    output logic                       bp_re_o,
    output logic [           LOGB-1:0] bp_raddr_o,
    input  logic                       bp_rdata_i,
    output logic                       bp_we_o,
    output logic [           LOGB-1:0] bp_waddr_o,
    output logic                       bp_wdata_o,

    // bimodal hysteresis: 2^(LOGB-1) x 2
    output logic                       bh_re_o,
    output logic [           LOGB-2:0] bh_raddr_o,
    input  logic [                1:0] bh_rdata_i,
    output logic                       bh_we_o,
    output logic [           LOGB-2:0] bh_waddr_o,
    output logic [                1:0] bh_wdata_o
);

    // -------------------------------------------------------------------------
    // Elaboration-time geometry (constant functions)
    // -------------------------------------------------------------------------
    function automatic int unsigned hist_len(input int unsigned t);
        case (t)
            1:       hist_len = T1_HIST;
            2:       hist_len = T2_HIST;
            3:       hist_len = T3_HIST;
            4:       hist_len = T4_HIST;
            5:       hist_len = T5_HIST;
            6:       hist_len = T6_HIST;
            7:       hist_len = T7_HIST;
            8:       hist_len = T8_HIST;
            9:       hist_len = T9_HIST;
            10:      hist_len = T10_HIST;
            11:      hist_len = T11_HIST;
            12:      hist_len = T12_HIST;
            13:      hist_len = T13_HIST;
            14:      hist_len = T14_HIST;
            default: hist_len = 0;
        endcase
    endfunction

    // C++ array (gtable index) of logical table t
    function automatic int unsigned f_arr(input int unsigned t);
        f_arr = (t > SH_OFF) ? (t - SH_OFF) : t;
    endfunction

    // 1 = high entry of the row (even array), 0 = low entry
    function automatic int unsigned f_slot(input int unsigned t);
        f_slot = ((f_arr(t) % 2) == 0) ? 1 : 0;
    endfunction

    // bank holding logical table t when the half-select bit X is x
    function automatic int unsigned f_bank(input int unsigned t, input int unsigned x);
        int unsigned a, k, nd;
        a  = f_arr(t);
        k  = (a + 1) / 2;
        nd = (NHIST - SH_OFF) / 2;
        if (a <= NHIST - SH_OFF) f_bank = 2 * (k - 1) + (x ^ ((t > SH_OFF) ? 1 : 0));
        else                     f_bank = 2 * nd + (k - nd - 1);
    endfunction

    // index fold width of table t (25/27, capped at 3 x index width)
    function automatic int unsigned f_ciw(input int unsigned t);
        int unsigned c, lg;
        c  = 25 + ((2 * ((t - 1) / 2)) % 4);
        lg = LOGG + ((t == 1) ? 1 : 0);
        if (ILEN_CAP != 0 && c > 3 * lg) c = 3 * lg;
        f_ciw = c;
    endfunction

    localparam int unsigned NDPAIR = (NHIST - SH_OFF) / 2;    // doubled pairs
    localparam int unsigned LG1    = LOGG + 1;
    localparam int unsigned NPH    = NHIST / 2;               // odd tables
    localparam int unsigned E_W    = TBITS + 5;
    localparam int unsigned N_CNT  = (NHIST + 1) / 4 + 1;     // COUNT50 / COUNT16_31
    localparam int unsigned CNT_W  = (N_CNT > 2) ? 2 : 1;
    localparam int unsigned GH_LEN = hist_len(NHIST);

    // -------------------------------------------------------------------------
    // Elaboration checks
    // -------------------------------------------------------------------------
    if (LOGASSOC != 0) begin : g_check_assoc
        $error("bp_tage_core: stage 1 supports LOGASSOC = 0 only");
    end
    if (NHIST < 4 || NHIST > 14 || (NHIST % 2) != 0) begin : g_check_nhist
        $error("bp_tage_core: NHIST must be even and in 4..14");
    end
    if (LOGG < 3 || LOGG > 14) begin : g_check_logg
        $error("bp_tage_core: LOGT - LOGASSOC must be in 3..14");
    end
    if (LOGB < 3 || LOGB > 24) begin : g_check_logb
        $error("bp_tage_core: LOGB must be in 3..24");
    end
    if (TBITS < 4 || TBITS > 16) begin : g_check_tbits
        $error("bp_tage_core: TBITS must be in 4..16");
    end
    if (PC_W < 36) begin : g_check_pcw
        $error("bp_tage_core: PC_W must be >= 36");
    end
    for (genvar t = 1; t <= NHIST; t++) begin : g_check_hist
        if (hist_len(t) < 4 || (t > 1 && hist_len(t) <= hist_len(t - 1))) begin : g_bad
            $error("bp_tage_core: T<i>_HIST must be >= 4 and strictly increasing");
        end
    end

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    typedef enum logic [4:0] {
        S_IDLE   = 5'd0,
        S_PRED   = 5'd1,
        S_WAIT   = 5'd2,
        S_CLC    = 5'd3,    // CountLowConf, use_alt_on_na, ALLOC
        S_CM11   = 5'd4,    // CountMiss11
        S_CNT    = 5'd5,    // COUNT50 / COUNT16_31 loop, one index per cycle
        S_FILT   = 5'd6,    // allocation filter on COUNT50 / COUNT16_31
        S_DEP1   = 5'd7,    // DEP, first random bit
        S_DEP2   = 5'd8,    // DEP, second random bit
        S_AHEAD  = 5'd9,    // allocation loop head (bound, SHARED filter)
        S_AJ     = 5'd10,   // j = MYRANDOM() % ASSOC; read table i
        S_ACHK   = 5'd11,   // allocate, or FORCEU + Penalty
        S_AR1    = 5'd12,   // i -= ...
        S_AR2    = 5'd13,   // i += ...
        S_AR3    = 5'd14,   // i += ..., MaxNALLOC check
        S_TICK   = 5'd15,
        S_SWEEP  = 5'd16,   // global u decrement
        S_F_ALT  = 5'd17,   // UPDATEALT: alternate counter
        S_F_HC   = 5'd18,   // UPDATEALT: HCpred counter or bimodal
        S_F_PROV = 5'd19,   // provider counter + u, or bimodal
        S_HIST   = 5'd20,   // history update
        S_HASH   = 5'd21    // block index / tag hash
    } state_e;

    state_e state_q, state_d;

    // history
    logic [GH_LEN-1:0]           gh_q, gh_d;       // gh_q[0] = newest bit
    logic [      26:0]           phist_q, phist_d;
    logic [      31:0]           ptg_q, ptg_d;     // C++ ptghist (RNG input)
    logic [      31:0]           pcb_q, pcb_d;     // PCBLOCK[31:0]
    logic [       1:0]           num_q, num_d;     // Numero
    logic                        hist_push;
    logic [       3:0]           hist_bits;

    // block hash results (index of odd table 2p-1 in row_q[p])
    logic [NPH:1][LOGG-1:0]      row_q;
    logic                        x_q;              // SHARED half select
    logic [NHIST:1][TBITS-1:0]   tag_q;
    logic                        hash_en;

    // prediction decisions of the branch in flight
    logic [3:0]                  hit_q, alt_q, hc_q, hit_d, alt_d, hc_d;
    logic                        lmp_q, hcp_q, altt_q, tpred_q, pweak_q;
    logic                        lmp_d, hcp_d, altt_d, tpred_d, pweak_d;

    // update
    logic                        dir_q;
    logic [35:0]                 pc_q, tgt_q;
    logic                        alloc_q, alloc_d;
    logic [4:0]                  i_q, i_d;
    logic [4:0]                  maxna_q, maxna_d;  // two's complement
    logic [3:0]                  na_q, na_d, pen_q, pen_d;
    logic                        first_q, first_d, test_q, test_d;
    logic [CNT_W-1:0]            cnt_q, cnt_d;
    logic [LOGG:0]               sw_q, sw_d;
    logic [NB:0]                 stale_q, stale_d;  // bit NB: bimodal

    // architectural predictor state
    logic [31:0]                 seed_q, seed_d;
    logic [11:0]                 tick_q, tick_d;
    logic [ 7:0]                 cm11_q, cm11_d;    // CountMiss11
    logic [ 6:0]                 clc_q, clc_d;      // CountLowConf
    logic [ 4:0]                 uaon_q, uaon_d;    // use_alt_on_na
    logic [N_CNT-1:0][6:0]       c50_q, c50_d, c1631_q, c1631_d;

    // -------------------------------------------------------------------------
    // Handshakes
    // -------------------------------------------------------------------------
    logic pred_fire, unc_fire, upd_cond_fire;

    assign pred_req_ready_o  = (state_q == S_IDLE);
    assign pred_resp_valid_o = (state_q == S_PRED);
    assign upd_ready_o       = (state_q == S_IDLE) || (state_q == S_WAIT);

    assign pred_fire     = pred_req_valid_i && (state_q == S_IDLE);
    assign unc_fire      = upd_valid_i && !upd_is_cond_i && !pred_req_valid_i && (state_q == S_IDLE);
    assign upd_cond_fire = upd_valid_i && upd_is_cond_i && (state_q == S_WAIT);

    // -------------------------------------------------------------------------
    // Folded histories and block hash, per logical table
    // -------------------------------------------------------------------------
    logic [NHIST:1][LOGG:0]      h_idx;            // T1: LOGG+1 bits, others LOGG
    logic [NHIST:1][TBITS-1:0]   h_tag;

    for (genvar t = 1; t <= NHIST; t++) begin : g_tbl
        localparam int unsigned L   = hist_len(t);
        localparam bit          ODD = ((t % 2) == 1);
        localparam int unsigned CIW = ODD ? f_ciw(t) : 1;
        localparam int unsigned LGI = LOGG + ((t == 1) ? 1 : 0);

        logic [ CIW-1:0] ci;
        logic [    12:0] ct0;
        logic [    10:0] ct1;
        logic [     3:0] outb;
        logic [ LGI-1:0] idx;

        // bit leaving the L-bit window at push step k: gh_q[L-1-k]
        assign outb = {gh_q[L-4], gh_q[L-3], gh_q[L-2], gh_q[L-1]};

        if (ODD) begin : g_ci
            bp_folded_hist #(
                .OLEN  (L),
                .CLEN  (CIW),
                .NPUSH (4)
            ) i_ci (
                .clk_i (clk_i),
                .rst_ni(rst_ni),
                .push_i(hist_push),
                .in_i  (hist_bits),
                .out_i (outb),
                .comp_o(ci)
            );
        end else begin : g_no_ci
            // CB_ADJACENT discards the index of even tables
            assign ci = '0;
        end

        bp_folded_hist #(
            .OLEN  (L),
            .CLEN  (13),
            .NPUSH (4)
        ) i_ct0 (
            .clk_i (clk_i),
            .rst_ni(rst_ni),
            .push_i(hist_push),
            .in_i  (hist_bits),
            .out_i (outb),
            .comp_o(ct0)
        );

        bp_folded_hist #(
            .OLEN  (L),
            .CLEN  (11),
            .NPUSH (4)
        ) i_ct1 (
            .clk_i (clk_i),
            .rst_ni(rst_ni),
            .push_i(hist_push),
            .in_i  (hist_bits),
            .out_i (outb),
            .comp_o(ct1)
        );

        bp_tage_hash #(
            .TBL   (t),
            .LOGG  (LOGG),
            .LOGG_I(LGI),
            .OLEN  (L),
            .CI_W  (CIW),
            .TBITS (TBITS),
            .DO_IDX(ODD)
        ) i_hash (
            .pc_i   (pcb_q),
            .phist_i(phist_q),
            .ci_i   (ci),
            .ct0_i  (ct0),
            .ct1_i  (ct1),
            .idx_o  (idx),
            .tag_o  (h_tag[t])
        );

        assign h_idx[t] = LG1'(idx);
    end

    // Block hash registers: written once per block (S_HASH). Reset value 0
    // is the hash of the all-zero reset history, as in the C++.
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            row_q <= '0;
            x_q   <= 1'b0;
            tag_q <= '0;
        end else if (hash_en) begin
            for (int unsigned p = 1; p <= NPH; p++) begin
                if (p == 1) row_q[p] <= h_idx[1][LOGG:1];
                else        row_q[p] <= h_idx[2 * p - 1][LOGG-1:0];
            end
            x_q   <= h_idx[1][0];
            tag_q <= h_tag;
        end
    end

    // -------------------------------------------------------------------------
    // Bank addresses of the current branch (Numero XORed into 2 row bits)
    // -------------------------------------------------------------------------
    logic [NB-1:0][LOGG-1:0] baddr;

    for (genvar b = 0; b < NB; b++) begin : g_baddr
        if (b < 2 * NDPAIR) begin : g_dbl
            // half H of doubled pair b/2+1: low tables when H == X, else partners
            localparam int unsigned PLO = b / 2 + 1;
            localparam int unsigned PHI = b / 2 + 1 + SH_OFF / 2;
            localparam bit          H   = ((b % 2) == 1);
            assign baddr[b] = ((H == x_q) ? row_q[PLO] : row_q[PHI])
                              ^ (LOGG'(num_q) << (LOGG - 3));
        end else begin : g_single
            localparam int unsigned P = b - NDPAIR + 1;
            assign baddr[b] = row_q[P] ^ (LOGG'(num_q) << (LOGG - 2));
        end
    end

    logic [LOGB-1:0] bim_idx;
    assign bim_idx = pcb_q[LOGB-1:0] ^ (LOGB'(num_q) << (LOGB - 2));

    // -------------------------------------------------------------------------
    // Entries of the current branch, per logical table (index 0: bimodal)
    // -------------------------------------------------------------------------
    logic [NHIST:0][E_W-1:0]   ent;
    logic [NHIST:0][TBITS-1:0] gtag_cur;
    logic [NHIST:0]            hitv, dirv, weakv;
    logic [NHIST:0][3:0]       bank0_l, bank1_l;
    logic [NHIST:0]            slot_l;

    assign ent[0]      = '0;
    assign gtag_cur[0] = '0;
    assign hitv[0]     = 1'b0;
    assign dirv[0]     = bp_rdata_i;     // no hit: the bimodal decides
    assign weakv[0]    = 1'b0;
    assign bank0_l[0]  = '0;
    assign bank1_l[0]  = '0;
    assign slot_l[0]   = 1'b0;

    for (genvar t = 1; t <= NHIST; t++) begin : g_ent
        localparam int unsigned B0 = f_bank(t, 0);
        localparam int unsigned B1 = f_bank(t, 1);
        localparam int unsigned SL = f_slot(t);

        assign ent[t]      = x_q ? tb_rdata_i[B1][SL*E_W +: E_W] : tb_rdata_i[B0][SL*E_W +: E_W];
        assign gtag_cur[t] = tag_q[t] ^ TBITS'(num_q);
        assign hitv[t]     = (ent[t][E_W-1:5] == gtag_cur[t]);
        assign dirv[t]     = ~ent[t][2];
        assign weakv[t]    = (ent[t][2:0] == 3'b000) || (ent[t][2:0] == 3'b111);
        assign bank0_l[t]  = 4'(B0);
        assign bank1_l[t]  = 4'(B1);
        assign slot_l[t]   = 1'(SL);
    end

    // -------------------------------------------------------------------------
    // Prediction (valid in S_PRED)
    // -------------------------------------------------------------------------
    logic [3:0] p_hit, p_alt, p_hc0, p_hc;
    logic       p_lmp, p_altt, p_pweak, p_hcp, p_tpred;

    always_comb begin
        p_hit = '0;
        p_alt = '0;
        p_hc0 = '0;
        for (int unsigned t = 1; t <= NHIST; t++) begin
            if (hitv[t]) p_hit = 4'(t);
        end
        for (int unsigned t = 1; t <= NHIST; t++) begin
            if (hitv[t] && 4'(t) < p_hit)              p_alt = 4'(t);
            if (hitv[t] && !weakv[t] && 4'(t) < p_hit) p_hc0 = 4'(t);
        end
        p_lmp   = dirv[p_hit];
        p_altt  = dirv[p_alt];
        p_pweak = weakv[p_hit];
        p_hc    = (p_hit == '0) ? '0 : (p_pweak ? p_hc0 : p_hit);
        p_hcp   = dirv[p_hc];
        if (p_hit == '0)                 p_tpred = bp_rdata_i;
        else if (uaon_q[4] || !p_pweak)  p_tpred = p_lmp;      // use_alt_on_na < 0
        else                             p_tpred = p_hcp;
    end

    assign pred_resp_taken_o = (CB_LMP != 0) ? p_lmp : p_tpred;

    // -------------------------------------------------------------------------
    // Update datapath
    // -------------------------------------------------------------------------
    logic [31:0] rnd;          // MYRANDOM() if called this cycle
    logic        rng_en;

    bp_tage_rng #(
        .TBITS(TBITS)
    ) i_rng (
        .seed_i   (seed_q),
        .phist_i  (phist_q),
        .ptghist_i(ptg_q),
        .gtag4_i  (gtag_cur[4]),
        .seed_o   (rnd)
    );

    // entry operated on in the current state
    logic [3:0]     sel_t;
    logic [E_W-1:0] sel_e;
    logic [3:0]     sel_bank;
    logic           sel_slot;
    logic           sel_weak;
    logic [2:0]     sel_ctr_upd;

    always_comb begin
        case (state_q)
            S_AJ, S_ACHK: sel_t = i_q[3:0];
            S_F_ALT:      sel_t = alt_q;
            S_F_HC:       sel_t = hc_q;
            default:      sel_t = hit_q;
        endcase
    end

    assign sel_e    = ent[sel_t];
    assign sel_bank = x_q ? bank1_l[sel_t] : bank0_l[sel_t];
    assign sel_slot = slot_l[sel_t];
    assign sel_weak = weakv[sel_t];

    bp_sat_ctr #(.WIDTH(3), .SIGNED(1'b1)) i_ent_ctr (
        .ctr_i(sel_e[2:0]),
        .inc_i(dir_q),
        .ctr_o(sel_ctr_upd)
    );

    // bimodal counter: BIM = pred ? hyst : -1 - hyst (3-bit)
    logic [2:0] bim_val, bim_upd;
    assign bim_val = bp_rdata_i ? {1'b0, bh_rdata_i} : {1'b1, ~bh_rdata_i};

    bp_sat_ctr #(.WIDTH(3), .SIGNED(1'b1)) i_bim_ctr (
        .ctr_i(bim_val),
        .inc_i(dir_q),
        .ctr_o(bim_upd)
    );

    // scalar counters
    logic [6:0] clc_sat, c50_sat, c1631_sat;
    logic [7:0] cm11_sat;
    logic [4:0] uaon_sat;

    bp_sat_ctr #(.WIDTH(7), .SIGNED(1'b1)) i_clc_ctr (
        .ctr_i(clc_q),
        .inc_i(pweak_q),
        .ctr_o(clc_sat)
    );
    bp_sat_ctr #(.WIDTH(8), .SIGNED(1'b1)) i_cm11_ctr (
        .ctr_i(cm11_q),
        .inc_i(tpred_q != dir_q),
        .ctr_o(cm11_sat)
    );
    bp_sat_ctr #(.WIDTH(5), .SIGNED(1'b1)) i_uaon_ctr (
        .ctr_i(uaon_q),
        .inc_i(hcp_q == dir_q),
        .ctr_o(uaon_sat)
    );
    bp_sat_ctr #(.WIDTH(7), .SIGNED(1'b1)) i_c50_ctr (
        .ctr_i(c50_q[cnt_q]),
        .inc_i(dir_q == lmp_q),
        .ctr_o(c50_sat)
    );
    bp_sat_ctr #(.WIDTH(7), .SIGNED(1'b1)) i_c1631_ctr (
        .ctr_i(c1631_q[cnt_q]),
        .inc_i(dir_q == lmp_q),
        .ctr_o(c1631_sat)
    );

    // provider u update (PROTECTU): 0,1 -> 2; 2,3 -> 3 when the provider was
    // right and the alternate wrong; -1 when wrong and tage_pred right
    logic [1:0] u_new;
    always_comb begin
        u_new = sel_e[4:3];
        if (lmp_q != altt_q) begin
            if (lmp_q == dir_q)                          u_new = sel_e[4] ? 2'b11 : 2'b10;
            else if (sel_e[4:3] != 2'b00 && tpred_q == dir_q) u_new = sel_e[4:3] - 2'b01;
        end
    end

    // TICK += Penalty - (2 + 2 * (CountMiss11 >= 0)) * NA, 14-bit two's complement
    logic [13:0] tick_sum;
    assign tick_sum = 14'(tick_q) + 14'(pen_q)
                      - (cm11_q[7] ? (14'(na_q) << 1) : (14'(na_q) << 2));

    // allocation filter counter index (HitBank + 1) / 4
    logic [4:0]       hit_p1;
    logic [CNT_W-1:0] kf;
    assign hit_p1 = 5'(hit_q) + 5'd1;
    assign kf     = CNT_W'(hit_p1 >> 2);

    // first final-update state: UPDATEALT applies when the provider is weak
    // and wrong
    logic   upd_alt;
    state_e ff_state;
    assign upd_alt  = (hit_q != '0) && pweak_q && (lmp_q != dir_q);
    assign ff_state = (upd_alt && (alt_q != hc_q)) ? S_F_ALT : (upd_alt ? S_F_HC : S_F_PROV);

    // -------------------------------------------------------------------------
    // Control
    // -------------------------------------------------------------------------
    logic           lat_upd;     // latch the update inputs
    logic [NB:0]    rd_req;      // re-read requests (bit NB: bimodal)
    logic           wr_en;       // write entry wr_e of table sel_t
    logic [E_W-1:0] wr_e;
    logic           bim_wr;
    logic           sweep_rd, sweep_wr;
    logic           blk_end;

    assign blk_end = (num_q == 2'd3) || dir_q;

    always_comb begin
        state_d  = state_q;
        hit_d    = hit_q;
        alt_d    = alt_q;
        hc_d     = hc_q;
        lmp_d    = lmp_q;
        hcp_d    = hcp_q;
        altt_d   = altt_q;
        tpred_d  = tpred_q;
        pweak_d  = pweak_q;
        alloc_d  = alloc_q;
        i_d      = i_q;
        maxna_d  = maxna_q;
        na_d     = na_q;
        pen_d    = pen_q;
        first_d  = first_q;
        test_d   = test_q;
        cnt_d    = cnt_q;
        sw_d     = sw_q;
        stale_d  = stale_q;
        tick_d   = tick_q;
        cm11_d   = cm11_q;
        clc_d    = clc_q;
        uaon_d   = uaon_q;
        c50_d    = c50_q;
        c1631_d  = c1631_q;
        rng_en   = 1'b0;
        lat_upd  = 1'b0;
        rd_req   = '0;
        wr_en    = 1'b0;
        wr_e     = sel_e;
        bim_wr   = 1'b0;
        sweep_rd = 1'b0;
        sweep_wr = 1'b0;
        hash_en  = 1'b0;

        case (state_q)
            S_IDLE: begin
                if (pred_fire) begin
                    state_d = S_PRED;
                end else if (unc_fire) begin
                    lat_upd = 1'b1;
                    state_d = S_HIST;
                end
            end

            S_PRED: begin
                hit_d   = p_hit;
                alt_d   = p_alt;
                hc_d    = p_hc;
                lmp_d   = p_lmp;
                hcp_d   = p_hcp;
                altt_d  = p_altt;
                tpred_d = p_tpred;
                pweak_d = p_pweak;
                state_d = S_WAIT;
            end

            S_WAIT: begin
                if (upd_cond_fire) begin
                    lat_upd = 1'b1;
                    stale_d = '1;              // re-read every table at update
                    state_d = S_CLC;
                end
            end

            S_CLC: begin
                if (hit_q != '0) begin
                    if (pweak_q) begin
                        clc_d = clc_sat;
                    end else begin
                        rng_en = 1'b1;
                        if (rnd[1:0] == 2'b00) clc_d = clc_sat;
                    end
                    if (pweak_q && (lmp_q != hcp_q)) uaon_d = uaon_sat;
                end
                alloc_d = (hit_q < 4'(NHIST)) && (lmp_q != dir_q) && (tpred_q != dir_q);
                state_d = S_CM11;
            end

            S_CM11: begin
                if (tpred_q != dir_q) begin
                    cm11_d = cm11_sat;
                end else begin
                    rng_en = 1'b1;
                    if (rnd[4:0] < 5'd4) cm11_d = cm11_sat;
                end
                if ((hit_q != '0) && pweak_q) begin
                    cnt_d   = CNT_W'(hit_q >> 2);
                    state_d = S_CNT;
                end else begin
                    state_d = S_FILT;
                end
            end

            S_CNT: begin
                c50_d[cnt_q] = c50_sat;
                if (lmp_q != dir_q) begin
                    c1631_d[cnt_q] = c1631_sat;
                end else begin
                    rng_en = 1'b1;
                    if (rnd[4:0] > 5'd1) c1631_d[cnt_q] = c1631_sat;
                end
                if (cnt_q == CNT_W'(NHIST / 4)) state_d = S_FILT;
                else                            cnt_d   = cnt_q + CNT_W'(1);
            end

            S_FILT: begin
                // the right-hand side of ALLOC &= ... is evaluated (and
                // MYRANDOM called) even when ALLOC is already false
                if (c50_q[kf][6]) begin
                    rng_en  = 1'b1;
                    alloc_d = alloc_q && (rnd[2:0] == 3'd0);
                end else if (c1631_q[kf][6]) begin
                    rng_en  = 1'b1;
                    alloc_d = alloc_q && !rnd[0];
                end
                state_d = alloc_d ? S_DEP1 : ff_state;
            end

            S_DEP1: begin
                rng_en  = 1'b1;
                i_d     = 5'(hit_q) + 5'd1 + 5'(!rnd[0]);
                maxna_d = 5'(cm11_q[7]) + (clc_q[6] ? 5'd0 : 5'd8);
                na_d    = '0;
                pen_d   = '0;
                first_d = 1'b1;
                test_d  = 1'b0;
                state_d = S_DEP2;
            end

            S_DEP2: begin
                rng_en  = 1'b1;
                i_d     = i_q + 5'(rnd[1:0] == 2'b00);
                state_d = S_AHEAD;
            end

            S_AHEAD: begin
                if (i_q > 5'(NHIST)) begin
                    state_d = S_TICK;
                end else if ((i_q > 5'(SH_OFF)) && !test_q) begin
                    // long tables (sharing storage with short ones) when the
                    // misprediction rate is high
                    test_d = 1'b1;
                    if (!cm11_q[7]) begin
                        rng_en  = 1'b1;
                        state_d = (rnd[2:0] != 3'd0) ? S_TICK : S_AJ;
                    end else begin
                        state_d = S_AJ;
                    end
                end else begin
                    state_d = S_AJ;
                end
            end

            S_AJ: begin
                rng_en = 1'b1;                 // j = MYRANDOM() % ASSOC
                if (stale_q[sel_bank]) begin
                    rd_req[sel_bank]  = 1'b1;
                    stale_d[sel_bank] = 1'b0;
                end
                state_d = S_ACHK;
            end

            S_ACHK: begin
                if (sel_e[4:3] == 2'b00) begin
                    // allocate: tag, u = First (FORCEU), weak counter
                    wr_en             = 1'b1;
                    wr_e              = {gtag_cur[sel_t], 1'b0, first_q, dir_q ? 3'b000 : 3'b111};
                    stale_d[sel_bank] = 1'b1;
                    na_d              = na_q + 4'd1;
                    if ((i_q >= 5'd3) || !first_q) maxna_d = maxna_q - 5'd1;
                    first_d           = 1'b0;
                    i_d               = i_q + 5'd2;
                    state_d           = S_AR1;
                end else begin
                    // FORCEU: maybe clear u of a weak entry with u == 1
                    rng_en = 1'b1;
                    if (!rnd[0] && sel_weak && (sel_e[4:3] == 2'b01)) begin
                        wr_en             = 1'b1;
                        wr_e              = {sel_e[E_W-1:5], 2'b00, sel_e[2:0]};
                        stale_d[sel_bank] = 1'b1;
                    end
                    pen_d   = pen_q + 4'd1;
                    i_d     = i_q + 5'd1;
                    state_d = S_AHEAD;
                end
            end

            S_AR1: begin
                rng_en  = 1'b1;
                i_d     = i_q - 5'(!rnd[0]);
                state_d = S_AR2;
            end

            S_AR2: begin
                rng_en  = 1'b1;
                i_d     = i_q + 5'(!rnd[0]);
                state_d = S_AR3;
            end

            S_AR3: begin
                rng_en = 1'b1;
                if (maxna_q[4]) begin          // MaxNALLOC < 0
                    state_d = S_TICK;
                end else begin
                    i_d     = i_q + 5'(rnd[1:0] == 2'b00) + 5'd1;
                    state_d = S_AHEAD;
                end
            end

            S_TICK: begin
                if (tick_sum[13]) begin            // < 0: clamp
                    tick_d  = '0;
                    state_d = ff_state;
                end else if (tick_sum[12]) begin   // >= BORNTICK: reset u
                    tick_d  = '0;
                    sw_d    = '0;
                    state_d = S_SWEEP;
                end else begin
                    tick_d  = tick_sum[11:0];
                    state_d = ff_state;
                end
            end

            S_SWEEP: begin
                // read row sw while writing back row sw - 1
                sweep_rd = !sw_q[LOGG];
                sweep_wr = (sw_q != '0);
                sw_d     = sw_q + LG1'(1);
                if (sw_q[LOGG]) begin
                    stale_d[NB-1:0] = '1;
                    state_d         = ff_state;
                end
            end

            S_F_ALT: begin
                if (stale_q[sel_bank]) begin
                    rd_req[sel_bank]  = 1'b1;
                    stale_d[sel_bank] = 1'b0;
                end else begin
                    wr_en             = 1'b1;
                    wr_e              = {sel_e[E_W-1:3], sel_ctr_upd};
                    stale_d[sel_bank] = 1'b1;
                    state_d           = S_F_HC;
                end
            end

            S_F_HC: begin
                if (hc_q != '0) begin
                    if (stale_q[sel_bank]) begin
                        rd_req[sel_bank]  = 1'b1;
                        stale_d[sel_bank] = 1'b0;
                    end else begin
                        wr_en             = 1'b1;
                        wr_e              = {sel_e[E_W-1:3], sel_ctr_upd};
                        stale_d[sel_bank] = 1'b1;
                        state_d           = S_F_PROV;
                    end
                end else begin
                    if (stale_q[NB]) begin
                        rd_req[NB]  = 1'b1;
                        stale_d[NB] = 1'b0;
                    end else begin
                        bim_wr      = 1'b1;
                        stale_d[NB] = 1'b1;
                        state_d     = S_F_PROV;
                    end
                end
            end

            S_F_PROV: begin
                if (hit_q != '0) begin
                    if (stale_q[sel_bank]) begin
                        rd_req[sel_bank]  = 1'b1;
                        stale_d[sel_bank] = 1'b0;
                    end else begin
                        wr_en             = 1'b1;
                        wr_e              = {sel_e[E_W-1:5], u_new, sel_ctr_upd};
                        stale_d[sel_bank] = 1'b1;
                        state_d           = S_HIST;
                    end
                end else begin
                    if (stale_q[NB]) begin
                        rd_req[NB]  = 1'b1;
                        stale_d[NB] = 1'b0;
                    end else begin
                        bim_wr      = 1'b1;
                        stale_d[NB] = 1'b1;
                        state_d     = S_HIST;
                    end
                end
            end

            S_HIST: begin
                state_d = blk_end ? S_HASH : S_IDLE;
            end

            S_HASH: begin
                hash_en = 1'b1;
                state_d = S_IDLE;
            end

            default: state_d = S_IDLE;
        endcase
    end

    // -------------------------------------------------------------------------
    // History update (S_HIST). At a block end, with PC = PCBLOCK ^ (Numero<<5)
    // and N = 2*Numero + taken:
    //   T    = (PC ^ PC>>2) ^ N ^ (target >> 3)              -> 4 history bits
    //   PATH = PC ^ PC>>2 ^ PC>>4 ^ target ^ (N << 3)        -> phist
    //   PCBLOCK = X ^ X>>4,  X = taken ? target : pc + 1
    // -------------------------------------------------------------------------
    logic [31:0] pcx;
    logic [ 2:0] nn;
    logic [26:0] path;
    logic [35:0] nxt;

    always_comb begin
        pcx       = pcb_q ^ {25'd0, num_q, 5'd0};
        nn        = {num_q, dir_q};
        hist_bits = pcx[3:0] ^ pcx[5:2] ^ {1'b0, nn} ^ tgt_q[6:3];
        path      = pcx[26:0] ^ pcx[28:2] ^ pcx[30:4] ^ tgt_q[26:0] ^ {21'd0, nn, 3'd0};
        nxt       = dir_q ? tgt_q : (pc_q + 36'd1);
        hist_push = (state_q == S_HIST) && blk_end;

        gh_d    = gh_q;
        phist_d = phist_q;
        ptg_d   = ptg_q;
        pcb_d   = pcb_q;
        num_d   = num_q;
        if (state_q == S_HIST) begin
            if (blk_end) begin
                gh_d    = {gh_q[GH_LEN-5:0], hist_bits[0], hist_bits[1], hist_bits[2], hist_bits[3]};
                phist_d = {phist_q[22:0], 4'd0} ^ path;
                ptg_d   = ptg_q - 32'd4;
                pcb_d   = nxt[31:0] ^ nxt[35:4];
                num_d   = 2'd0;
            end else begin
                num_d   = num_q + 2'd1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // SRAM ports
    // -------------------------------------------------------------------------
    logic [LOGG-1:0] sw_m1;
    assign sw_m1 = LOGG'(sw_q - LG1'(1));

    for (genvar b = 0; b < NB; b++) begin : g_port
        logic [E_W-1:0] lo, hi, lo_dec, hi_dec;
        assign lo     = tb_rdata_i[b][E_W-1:0];
        assign hi     = tb_rdata_i[b][ROW_W-1:E_W];
        // u - 1 when u > 0 (global u reset), ctr and tag unchanged
        assign lo_dec = {lo[E_W-1:5], (lo[4:3] != 2'b00) ? (lo[4:3] - 2'b01) : 2'b00, lo[2:0]};
        assign hi_dec = {hi[E_W-1:5], (hi[4:3] != 2'b00) ? (hi[4:3] - 2'b01) : 2'b00, hi[2:0]};

        assign tb_re_o[b]    = pred_fire || sweep_rd || rd_req[b];
        assign tb_raddr_o[b] = sweep_rd ? sw_q[LOGG-1:0] : baddr[b];
        assign tb_we_o[b]    = sweep_wr || (wr_en && (sel_bank == 4'(b)));
        assign tb_waddr_o[b] = sweep_wr ? sw_m1 : baddr[b];
        // sweep: decrement u in both entries; else replace one entry
        assign tb_wdata_o[b] = sweep_wr ? {hi_dec, lo_dec}
                             : (sel_slot ? {wr_e, lo} : {hi, wr_e});
    end

    assign bp_re_o    = pred_fire || rd_req[NB];
    assign bp_raddr_o = bim_idx;
    assign bp_we_o    = bim_wr;
    assign bp_waddr_o = bim_idx;
    assign bp_wdata_o = ~bim_upd[2];
    assign bh_re_o    = pred_fire || rd_req[NB];
    assign bh_raddr_o = bim_idx[LOGB-1:1];
    assign bh_we_o    = bim_wr;
    assign bh_waddr_o = bim_idx[LOGB-1:1];
    assign bh_wdata_o = bim_upd[2] ? ~bim_upd[1:0] : bim_upd[1:0];

    // -------------------------------------------------------------------------
    // Registers
    // -------------------------------------------------------------------------
    always_comb begin
        seed_d = rng_en ? rnd : seed_q;
    end

    // control, history and architectural state: reset
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q <= S_IDLE;
            gh_q    <= '0;
            phist_q <= '0;
            ptg_q   <= '0;
            pcb_q   <= '0;
            num_q   <= '0;
            seed_q  <= '0;
            tick_q  <= '0;
            cm11_q  <= 8'hc0;          // -64
            clc_q   <= '0;
            uaon_q  <= '0;
            c50_q   <= '0;
            c1631_q <= '0;
        end else begin
            state_q <= state_d;
            gh_q    <= gh_d;
            phist_q <= phist_d;
            ptg_q   <= ptg_d;
            pcb_q   <= pcb_d;
            num_q   <= num_d;
            seed_q  <= seed_d;
            tick_q  <= tick_d;
            cm11_q  <= cm11_d;
            clc_q   <= clc_d;
            uaon_q  <= uaon_d;
            c50_q   <= c50_d;
            c1631_q <= c1631_d;
        end
    end

    // per-branch datapath registers: no reset (written before they are used)
    always_ff @(posedge clk_i) begin
        hit_q   <= hit_d;
        alt_q   <= alt_d;
        hc_q    <= hc_d;
        lmp_q   <= lmp_d;
        hcp_q   <= hcp_d;
        altt_q  <= altt_d;
        tpred_q <= tpred_d;
        pweak_q <= pweak_d;
        alloc_q <= alloc_d;
        i_q     <= i_d;
        maxna_q <= maxna_d;
        na_q    <= na_d;
        pen_q   <= pen_d;
        first_q <= first_d;
        test_q  <= test_d;
        cnt_q   <= cnt_d;
        sw_q    <= sw_d;
        stale_q <= stale_d;
        if (lat_upd) begin
            dir_q <= upd_taken_i;
            pc_q  <= upd_pc_i[35:0];
            tgt_q <= upd_target_i[35:0];
        end
    end

    // -------------------------------------------------------------------------
    // Unused interface signals (the prediction does not use the branch PC)
    // -------------------------------------------------------------------------
    /* verilator lint_off UNUSEDSIGNAL */
    logic unused_ok;
    assign unused_ok = ^{pred_req_pc_i, upd_pc_i[PC_W-1:36], upd_target_i[PC_W-1:36],
                         pcx[31], hit_p1[4], hit_p1[1:0], h_idx, bank0_l, bank1_l};
    /* verilator lint_on UNUSEDSIGNAL */

    // -------------------------------------------------------------------------
    // Protocol checks (simulation only)
    // -------------------------------------------------------------------------
`ifdef VERILATOR
    always_ff @(posedge clk_i) begin
        if (rst_ni && upd_valid_i) begin
            if (upd_is_cond_i && state_q != S_WAIT)
                $error("bp_tage_core: conditional update without a pending prediction");
            if (!upd_is_cond_i && state_q != S_IDLE)
                $error("bp_tage_core: unconditional update while a branch is in flight");
            if (!upd_is_cond_i && pred_req_valid_i)
                $error("bp_tage_core: predict request and unconditional update in the same cycle");
            // tage_cb.h computes a block's indices at its first CONDITIONAL
            // branch; a not-taken unconditional first branch would make the
            // C++ read a 10-block-old AHGI slot. Not modeled.
            if (!upd_is_cond_i && state_q == S_IDLE && !upd_taken_i && num_q == 2'd0)
                $error("bp_tage_core: not-taken unconditional branch at block start (not modeled)");
        end
        if (rst_ni && pred_req_valid_i && !pred_req_ready_o)
            $error("bp_tage_core: predict request while a branch is in flight");
    end
`endif

endmodule