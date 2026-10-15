// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Cookbook TAGE predictor core ("TAGE: an engineering cookbook", Seznec,
//   INRIA RR-9561, 2024), bit-exact with tage_cb.h built with CB_SC=0,
//   AHEAD=0, CB_OPTTAGE=1, CB_ILEN_CAP=1, LOGASSOC=0 (direct-mapped tagged
//   tables; STATIC_TARGET_FIX=1 traces). This is the unit that is
//   synthesized; every table is an SRAM outside it and the SRAM ports are
//   its I/O.
//
//   Block-based prediction: a fetch block ends at a taken branch or after 4
//   branches. The index of every odd logical table and the tag of every
//   table are hashed once per block (S_HASH, after the history update);
//   branch number Numero (num_q) is XORed in per branch. The branch PC
//   itself is not used.
//
//   Storage (C++ arrays -> banks). CB_ADJACENT gives T(2k-1) and T(2k) the
//   same index, so their entries sit side by side in one row (low entry =
//   odd table). CB_SHARED: the doubled arrays are split into two banks by
//   the index LSB (X for the low tables, X^1 for their partners
//   T(i+SH_OFF)), so both are read in one cycle. NB banks of 2^LOGT rows,
//   2 entries per row; entry = {tag, u[1:0], ctr[2:0]} (ctr two's
//   complement); bank b's data in the tb_* data vectors: [2*b*E_W +: 2*E_W].
//   Within one branch every bank is accessed at one row only (its branch
//   row), apart from the u-reset sweep.
//   Bimodal: pred bits (2^LOGB x 1) and hysteresis (2^(LOGB-1) x 2).
//
//   Timing structure (none of it changes a prediction):
//   - Per-bank state instead of per-table state. The hash is stored as what
//     each bank needs this block: its row address (baddr_q) and the tags of
//     the two tables it holds (et_q), already permuted by X and with Numero
//     XORed in (loaded in S_HASH, adjusted when Numero changes). The SRAM
//     read address comes straight from a flop, and every tag compare is one
//     bank entry against one flop, with no X multiplexer in front of it; X
//     (x_q) only selects between the two halves of a doubled pair on 1-bit
//     per-table signals. Same flop count as the per-table rows and tags.
//   - One-hot selection: provider = highest hit, alternate = highest hit
//     below it, HCpred = highest non-weak hit below it, each by prefix ORs
//     (log depth); directions and the provider's row are then selected by
//     AND-OR. The encoded table numbers (checkpoint, bench) are OR-encoded
//     from the one-hots.
//   - The selection in two stages. Stage 1, on the read data: tag compares,
//     provider, the hits and non-weak hits below it, its weak flag. Stage 2:
//     alternate and HCpred, the directions, the response. PRED_REG = 1:
//     stage 1 runs in the read cycle (S_TAG) and is registered with the
//     rows and the bimodal entry, stage 2 runs in S_PRED from those flops
//     (two-cycle prediction); the SRAM read data is still used only in the
//     cycle right after its read. PRED_REG = 0: both in one cycle.
//   - MYRANDOM lookahead: rnd_q holds MYRANDOM(seed_q), so the update
//     decisions read a flop, and the generator is fed from rnd_q alone:
//     rnd_q is reloaded with seed_q in the states before S_PRED and in
//     S_HIST / S_HASH (where phist, ptghist, GTAG[4] change), advanced once
//     in S_PRED and at every call. Only rnd_q -> MYRANDOM -> rnd_q goes
//     through the adders.
//   - Update entry operations: the operated table's bank (one-hot) and
//     slot are registered ahead (provider, alternate, HCpred in S_PRED;
//     the allocation index whenever it changes), so the bank multiplexer's
//     select is ready before the read data; a write enable comes from its
//     cause (counter not at the limit of the outcome's direction, u
//     changing, allocated entry different), not from comparing the new
//     entry with the old one.
//   In simulation (Verilator) the original per-table registers and
//   formulas run alongside as a reference and every cycle is checked.
//
//   SRAM read data is used only in the cycle right after its read (no
//   reliance on a macro holding its output). What the update needs later
//   comes from the predict -> update checkpoint (flops), latched in S_PRED:
//     ck_row_q  the provider's row (both entries, 2 x E_W bits): the
//               provider update (counter + u) is a read-modify-write of
//               that row; valid until a write of this update hits the
//               provider's bank (then the row is read again)
//     ck_bp_q, ck_bh_q  the bimodal entry (pred bit, hysteresis): the
//               bimodal is written at most once per update, never re-read
//   plus the prediction decisions (provider, alternate, HCpred, ...).
//
//   Protocol (non-speculative, one branch in flight, as gshare):
//     S_IDLE  predict handshake reads every bank -> S_PRED (response,
//             checkpoint; PRED_REG = 1: S_TAG first), or an unconditional
//             update (TrackOtherInst) -> S_HIST.
//     S_TAG   only with PRED_REG = 1: tag compares on the read data,
//             registered with the rows and the bimodal entry.
//     S_WAIT  conditional update handshake -> the update FSM, which mirrors
//             UpdatePredictorCore statement by statement, one MYRANDOM call
//             per cycle at most.
//     S_AW    allocation into table i: its bank is read, and MYRANDOM is
//             called for j (ASSOC = 1); S_AWD: u == 0 -> the new entry is
//             written, else FORCEU (MYRANDOM, maybe clear u) and Penalty.
//     S_F_ALT, S_F_HC (UPDATEALT) and S_F_PROV without a valid checkpoint
//             read their bank first (one extra cycle), then write.
//     S_SWEEP the global u decrement (TICK >= 4096): all banks, one row per
//             cycle, read/write pipelined; 2^LOGT + 1 cycles. A row is
//             written only if one of its entries has u > 0.
//     S_HIST  history update (block end: path history, 4 global history
//             bits, folded histories) -> S_HASH (block hash) -> S_IDLE.
//   An entry or bimodal write is skipped when it would store the value the
//   SRAM already holds (saturated counter, unchanged u).
//   upd_ready_o / pred_req_ready_o are low while the update runs.
//   Prediction latency (predict handshake -> pred_resp_valid_o): 1 + PRED_REG
//   cycles.
//
// Parameters:
//   NHIST              - logical tagged tables (even, 4..14)
//   LOGT               - log2 entries of a logical table (C++ LOGT = LOGG)
//   LOGB               - log2 bimodal entries
//   TBITS              - tag bits
//   CB_LMP             - 1: predict LongestMatchPred (C++ CB_LMP with CB_SC=0)
//   ILEN_CAP           - 1: index folds capped at 3*index width (CB_ILEN_CAP)
//   T1_HIST..T14_HIST  - history length of each table in bits, as printed by
//                        the CBP5 run (PREDICTOR_HIST_LENS); unused ones 0
//   PRED_REG           - 1: two-cycle prediction (tag compares registered, see
//                        above); 0: one cycle, read data to response
//   PC_W               - PC / target width of the shared interface
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_tage_core #(
    parameter int unsigned  NHIST    = 12,
    parameter int unsigned  LOGT     = 6,
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
    parameter bit           PRED_REG = 1'b1,
    parameter int unsigned  PC_W     = 64,
    localparam int unsigned SH_OFF   = 2 * ((NHIST / 2 + 1) / 2),
    localparam int unsigned NB       = (NHIST - SH_OFF) / 2 + SH_OFF / 2,
    localparam int unsigned DATA_W   = 2 * NB * (TBITS + 5)
) (
    input  logic                     clk_i,
    input  logic                     rst_ni,

    // predict channel (CBP5 GetPrediction)
    input  logic                     pred_req_valid_i,
    output logic                     pred_req_ready_o,
    input  logic [         PC_W-1:0] pred_req_pc_i,
    output logic                     pred_resp_valid_o,
    output logic                     pred_resp_taken_o,

    // update channel (CBP5 UpdatePredictor / TrackOtherInst)
    input  logic                     upd_valid_i,
    output logic                     upd_ready_o,
    input  logic                     upd_is_cond_i,
    input  logic [         PC_W-1:0] upd_pc_i,
    input  logic                     upd_taken_i,
    input  logic [         PC_W-1:0] upd_target_i,

    // tagged-table banks (1R1W, synchronous read), 2^LOGT rows of 2 entries;
    // bank b's data at [2*b*E_W +: 2*E_W]
    output logic [NB-1:0]            tb_re_o,
    output logic [NB-1:0][LOGT-1:0]  tb_raddr_o,
    input  logic [       DATA_W-1:0] tb_rdata_i,
    output logic [NB-1:0]            tb_we_o,
    output logic [NB-1:0][LOGT-1:0]  tb_waddr_o,
    output logic [       DATA_W-1:0] tb_wdata_o,

    // bimodal prediction bits: 2^LOGB x 1
    output logic                     bp_re_o,
    output logic [         LOGB-1:0] bp_raddr_o,
    input  logic                     bp_rdata_i,
    output logic                     bp_we_o,
    output logic [         LOGB-1:0] bp_waddr_o,
    output logic                     bp_wdata_o,

    // bimodal hysteresis: 2^(LOGB-1) x 2
    output logic                     bh_re_o,
    output logic [         LOGB-2:0] bh_raddr_o,
    input  logic [              1:0] bh_rdata_i,
    output logic                     bh_we_o,
    output logic [         LOGB-2:0] bh_waddr_o,
    output logic [              1:0] bh_wdata_o
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

    // 1 = the array of table t is doubled (CB_SHARED), split by X
    function automatic int unsigned f_dbl(input int unsigned t);
        f_dbl = (f_arr(t) <= NHIST - SH_OFF) ? 1 : 0;
    endfunction

    // bank of table t when the half-select bit X is x
    function automatic int unsigned f_bank(input int unsigned t, input int unsigned x);
        int unsigned a, k, nd;
        a  = f_arr(t);
        k  = (a + 1) / 2;
        nd = (NHIST - SH_OFF) / 2;
        if (a <= NHIST - SH_OFF) f_bank = 2 * (k - 1) + (x ^ ((t > SH_OFF) ? 1 : 0));
        else                     f_bank = 2 * nd + (k - nd - 1);
    endfunction

    // tables held by bank b when X = x (bit t = table t)
    function automatic logic [NHIST:1] bank_mask(input int unsigned b, input int unsigned x);
        bank_mask = '0;
        for (int unsigned t = 1; t <= NHIST; t++) bank_mask[t] = (f_bank(t, x) == b);
    endfunction

    // tables in the high entry of their row (bit t = table t)
    function automatic logic [NHIST:1] slot_mask();
        slot_mask = '0;
        for (int unsigned t = 1; t <= NHIST; t++) slot_mask[t] = (f_slot(t) != 0);
    endfunction

    // index fold width of table t (25/27, capped at 3 x index width)
    function automatic int unsigned f_ciw(input int unsigned t);
        int unsigned c, lg;
        c  = 25 + ((2 * ((t - 1) / 2)) % 4);
        lg = LOGT + ((t == 1) ? 1 : 0);
        if (ILEN_CAP != 0 && c > 3 * lg) c = 3 * lg;
        f_ciw = c;
    endfunction

    localparam int unsigned SH_N   = NHIST - SH_OFF;            // doubled arrays
    localparam int unsigned NDPAIR = SH_N / 2;                  // doubled pairs
    localparam int unsigned NPH    = NHIST / 2;                 // odd tables
    localparam int unsigned E_W    = TBITS + 5;
    localparam int unsigned N_CNT  = (NHIST + 1) / 4 + 1;       // COUNT50 / COUNT16_31
    localparam int unsigned CNT_W  = (N_CNT > 2) ? 2 : 1;
    localparam int unsigned GH_LEN = hist_len(NHIST);
    localparam int unsigned LG1    = LOGT + 1;
    localparam int unsigned BANK_W = (NB > 1) ? $clog2(NB) : 1;
    localparam int unsigned NSH_D  = LOGT - 3;                  // Numero in a half row
    localparam int unsigned NSH_S  = LOGT - 2;                  // Numero in a full row

    // -------------------------------------------------------------------------
    // Elaboration checks
    // -------------------------------------------------------------------------
    if (NHIST < 4 || NHIST > 14 || (NHIST % 2) != 0) begin : g_check_nhist
        $error("bp_tage_core: NHIST must be even and in 4..14");
    end
    if (LOGT < 3 || LOGT > 14) begin : g_check_logt
        $error("bp_tage_core: LOGT must be in 3..14");
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
        S_AW     = 5'd10,   // read table i's bank; j = MYRANDOM() % 1
        S_AWD    = 5'd11,   // u == 0: allocate; else FORCEU + Penalty
        S_AR1    = 5'd12,   // i -= ...
        S_AR2    = 5'd13,   // i += ...
        S_AR3    = 5'd14,   // i += ..., MaxNALLOC check
        S_TICK   = 5'd15,
        S_SWEEP  = 5'd16,   // global u decrement
        S_F_ALT  = 5'd17,   // UPDATEALT: alternate counter
        S_F_HC   = 5'd18,   // UPDATEALT: HCpred counter or bimodal
        S_F_PROV = 5'd19,   // provider counter + u, or bimodal
        S_HIST   = 5'd20,   // history update
        S_HASH   = 5'd21,   // block index / tag hash
        S_TAG    = 5'd22    // PRED_REG: tag compares on the read data, registered
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

    // per-bank state of the current branch: row address and the tags of the
    // two tables the bank holds this block (Numero included)
    logic [NB-1:0][LOGT-1:0]       baddr_q;
    logic [NB-1:0][1:0][TBITS-1:0] et_q;            // [bank][slot], slot 0 = low entry
    logic                          x_q;             // SHARED half select
    logic                          hash_en;

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
    logic [LOGT:0]               sw_q, sw_d;

    // predict -> update checkpoint (see the header) and read tracking
    logic [2*(TBITS+5)-1:0]      ck_row_q, ck_row_d;   // provider's row
    logic                        ck_v_q, ck_v_d;       // ... still matches the SRAM
    logic                        ck_bp_q, ck_bp_d;     // bimodal pred bit
    logic [1:0]                  ck_bh_q, ck_bh_d;     // bimodal hysteresis
    logic                        rd_q, rd_req;         // operated bank read last cycle

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
    logic [NHIST:1][LOGT:0]      h_idx;            // T1: LOGT+1 bits, others LOGT
    logic [NHIST:1][TBITS-1:0]   h_tag;

    for (genvar t = 1; t <= NHIST; t++) begin : g_tbl
        localparam int unsigned L   = hist_len(t);
        localparam bit          ODD = ((t % 2) == 1);
        localparam int unsigned CIW = ODD ? f_ciw(t) : 1;
        localparam int unsigned LGI = LOGT + ((t == 1) ? 1 : 0);

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
            .LOGG  (LOGT),
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

    // -------------------------------------------------------------------------
    // Block hash -> per-bank state of the next block (Numero 0). Bank b of a
    // doubled pair holds the low tables (TLO, TLO+1) when its half H equals
    // X, else their partners (THI, THI+1); a single bank always holds (T,
    // T+1). The row of a pair is the index of its odd table (CB_ADJACENT);
    // T1's index has one more bit, its LSB is X.
    // -------------------------------------------------------------------------
    logic                          x_h;
    logic [NB-1:0][LOGT-1:0]       baddr_h;
    logic [NB-1:0][1:0][TBITS-1:0] et_h;

    assign x_h = h_idx[1][0];

    for (genvar b = 0; b < NB; b++) begin : g_bh
        if (b < 2 * NDPAIR) begin : g_dbl
            localparam int unsigned TLO = b - (b % 2) + 1;
            localparam int unsigned THI = TLO + SH_OFF;
            localparam bit          H   = ((b % 2) == 1);

            logic [LOGT-1:0] r_lo, r_hi;

            if (TLO == 1) begin : g_t1
                assign r_lo = h_idx[1][LOGT:1];
            end else begin : g_tn
                assign r_lo = h_idx[TLO][LOGT-1:0];
            end
            assign r_hi       = h_idx[THI][LOGT-1:0];
            assign baddr_h[b] = (H == x_h) ? r_lo : r_hi;
            assign et_h[b][0] = (H == x_h) ? h_tag[TLO]     : h_tag[THI];
            assign et_h[b][1] = (H == x_h) ? h_tag[TLO + 1] : h_tag[THI + 1];
        end else begin : g_single
            localparam int unsigned T = 2 * (b - NDPAIR) + 1;

            if (T == 1) begin : g_t1
                assign baddr_h[b] = h_idx[1][LOGT:1];
            end else begin : g_tn
                assign baddr_h[b] = h_idx[T][LOGT-1:0];
            end
            assign et_h[b][0] = h_tag[T];
            assign et_h[b][1] = h_tag[T + 1];
        end
    end

    // Written once per block (S_HASH); when Numero changes (S_HIST), its
    // change is XORed into the row (bits NSH +: 2) and the tags (bits 1:0),
    // as GI ^ (Numero << NSH) and GTAG ^ Numero in the C++. Reset value 0 is
    // the hash of the all-zero reset history, as in the C++.
    logic [1:0] num_dif;
    assign num_dif = num_q ^ num_d;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            x_q     <= 1'b0;
            baddr_q <= '0;
            et_q    <= '0;
        end else if (hash_en) begin
            x_q     <= x_h;
            baddr_q <= baddr_h;
            et_q    <= et_h;
        end else if (num_dif != 2'b00) begin
            for (int unsigned b = 0; b < NB; b++) begin
                baddr_q[b] <= baddr_q[b] ^ (LOGT'(num_dif) << ((b < 2 * NDPAIR) ? NSH_D : NSH_S));
                et_q[b][0] <= et_q[b][0] ^ TBITS'(num_dif);
                et_q[b][1] <= et_q[b][1] ^ TBITS'(num_dif);
            end
        end
    end

    logic [LOGB-1:0] bim_idx;
    assign bim_idx = pcb_q[LOGB-1:0] ^ (LOGB'(num_q) << (LOGB - 2));

    // -------------------------------------------------------------------------
    // Read data per bank (low / high entry) and the per-entry decisions:
    // tag hit against the bank's own tag flop, direction, weak
    // -------------------------------------------------------------------------
    logic [NB-1:0][E_W-1:0] bank_lo, bank_hi;
    logic [NB-1:0][1:0]     hbs, dbs, wbs;

    for (genvar b = 0; b < NB; b++) begin : g_bank
        assign bank_lo[b] = tb_rdata_i[2 * b * E_W +: E_W];
        assign bank_hi[b] = tb_rdata_i[2 * b * E_W + E_W +: E_W];

        for (genvar s = 0; s < 2; s++) begin : g_s
            logic [E_W-1:0] e;
            assign e         = (s == 0) ? bank_lo[b] : bank_hi[b];
            assign hbs[b][s] = (e[E_W-1:5] == et_q[b][s]);
            assign dbs[b][s] = ~e[2];
            assign wbs[b][s] = (e[2:0] == 3'b000) || (e[2:0] == 3'b111);
        end
    end

    // -------------------------------------------------------------------------
    // Per logical table: its bank for X = 0 / 1 and its slot (reference), and
    // its hit / direction / weak flags from this cycle's read data. x_q only
    // chooses between the two halves of a doubled pair.
    // -------------------------------------------------------------------------
    logic [NHIST:0][BANK_W-1:0] bx0_l, bx1_l;
    logic [NHIST:0]             slot_l;
    logic [NHIST:1]             hv, dv, wv;

    assign bx0_l[0]  = '0;
    assign bx1_l[0]  = '0;
    assign slot_l[0] = 1'b0;

    for (genvar t = 1; t <= NHIST; t++) begin : g_tmap
        localparam int unsigned BX0 = f_bank(t, 0);
        localparam int unsigned BX1 = f_bank(t, 1);
        localparam int unsigned SL  = f_slot(t);

        assign bx0_l[t]  = BANK_W'(BX0);
        assign bx1_l[t]  = BANK_W'(BX1);
        assign slot_l[t] = (SL != 0);
        assign hv[t]     = x_q ? hbs[BX1][SL] : hbs[BX0][SL];
        assign dv[t]     = x_q ? dbs[BX1][SL] : dbs[BX0][SL];
        assign wv[t]     = x_q ? wbs[BX1][SL] : wbs[BX0][SL];
    end

    // -------------------------------------------------------------------------
    // Prediction, one-hot:
    //   provider  h1  = highest hit
    //   alternate a1  = highest hit below the provider
    //   HCpred    hc1 = highest non-weak hit below a weak provider, else the
    //                   provider
    // With no hit / no alternate / no HCpred the bimodal decides (C++ index 0).
    // Stage 1, on the read data: the provider, the hits below it (hv2), the
    // non-weak hits below it (hcc), its weak flag. abv[t] = any hit above t
    // (= t is below the provider, if t hits).
    // -------------------------------------------------------------------------
    logic [NHIST:1] abv, h1_c, hv2_c, hcc_c;
    logic           pweak_c, anyh_c;

    for (genvar t = 1; t <= NHIST; t++) begin : g_above1
        if (t == NHIST) begin : g_top
            assign abv[t] = 1'b0;
        end else begin : g_mid
            assign abv[t] = |hv[NHIST:t+1];
        end
    end

    assign h1_c    = hv & ~abv;
    assign hv2_c   = hv & abv;
    assign hcc_c   = hv & ~wv & abv;
    assign pweak_c = |(h1_c & wv);
    assign anyh_c  = |hv;

    // -------------------------------------------------------------------------
    // Stage 2 works on stage 1 of this cycle (PRED_REG = 0) or on its copy
    // registered in S_TAG, the cycle right after the read (PRED_REG = 1),
    // together with the rows read and the bimodal entry
    // -------------------------------------------------------------------------
    logic [NHIST:1]         h1_s, hv2_s, hcc_s, dv_s;
    logic                   pweak_s, anyh_s;
    logic [NB-1:0][E_W-1:0] row_lo_s, row_hi_s;
    logic                   bp_s;
    logic [1:0]             bh_s;

    if (PRED_REG) begin : g_pred_reg
        logic [NHIST:1]         h1_r, hv2_r, hcc_r, dv_r;
        logic                   pwk_r, anyh_r;
        logic [NB-1:0][E_W-1:0] lo_r, hi_r;
        logic                   bp_r;
        logic [1:0]             bh_r;

        always_ff @(posedge clk_i) begin
            if (state_q == S_TAG) begin
                h1_r   <= h1_c;
                hv2_r  <= hv2_c;
                hcc_r  <= hcc_c;
                dv_r   <= dv;
                pwk_r  <= pweak_c;
                anyh_r <= anyh_c;
                lo_r   <= bank_lo;
                hi_r   <= bank_hi;
                bp_r   <= bp_rdata_i;
                bh_r   <= bh_rdata_i;
            end
        end

        assign h1_s     = h1_r;
        assign hv2_s    = hv2_r;
        assign hcc_s    = hcc_r;
        assign dv_s     = dv_r;
        assign pweak_s  = pwk_r;
        assign anyh_s   = anyh_r;
        assign row_lo_s = lo_r;
        assign row_hi_s = hi_r;
        assign bp_s     = bp_r;
        assign bh_s     = bh_r;
    end else begin : g_pred_comb
        assign h1_s     = h1_c;
        assign hv2_s    = hv2_c;
        assign hcc_s    = hcc_c;
        assign dv_s     = dv;
        assign pweak_s  = pweak_c;
        assign anyh_s   = anyh_c;
        assign row_lo_s = bank_lo;
        assign row_hi_s = bank_hi;
        assign bp_s     = bp_rdata_i;
        assign bh_s     = bh_rdata_i;
    end

    // -------------------------------------------------------------------------
    // Stage 2 (S_PRED): alternate and HCpred, the directions, the response
    // -------------------------------------------------------------------------
    logic [NHIST:1] abv2, a1, abv3, hc0, hc1;
    logic           any_alt;
    logic [3:0]     p_hit, p_alt, p_hc;
    logic           p_lmp, p_altt, p_pweak, p_hcp, p_tpred;

    for (genvar t = 1; t <= NHIST; t++) begin : g_above2
        if (t == NHIST) begin : g_top
            assign abv2[t] = 1'b0;
            assign abv3[t] = 1'b0;
        end else begin : g_mid
            assign abv2[t] = |hv2_s[NHIST:t+1];
            assign abv3[t] = |hcc_s[NHIST:t+1];
        end
    end

    assign a1      = hv2_s & ~abv2;
    assign hc0     = hcc_s & ~abv3;
    assign any_alt = |hv2_s;
    assign p_pweak = pweak_s;
    assign hc1     = p_pweak ? hc0 : h1_s;  // no hit: h1 = 0, p_pweak = 0

    assign p_lmp   = anyh_s ? |(h1_s & dv_s) : bp_s;
    assign p_altt  = any_alt ? |(a1 & dv_s) : bp_s;
    assign p_hcp   = (|hc1) ? |(hc1 & dv_s) : bp_s;
    // use_alt_on_na < 0, or a confident provider: the longest match
    assign p_tpred = !anyh_s ? bp_s : ((uaon_q[4] || !p_pweak) ? p_lmp : p_hcp);

    assign pred_resp_taken_o = (CB_LMP != 0) ? p_lmp : p_tpred;

    // table numbers (checkpoint, bench), OR-encoded from the one-hots
    always_comb begin
        p_hit = '0;
        p_alt = '0;
        p_hc  = '0;
        for (int unsigned t = 1; t <= NHIST; t++) begin
            p_hit = p_hit | ({4{h1_s[t]}} & 4'(t));
            p_alt = p_alt | ({4{a1[t]}}   & 4'(t));
            p_hc  = p_hc  | ({4{hc1[t]}}  & 4'(t));
        end
    end

    // -------------------------------------------------------------------------
    // Bank one-hots: of the provider (its row is the checkpoint), and, for
    // the update, of the provider, alternate, HCpred (registered in S_PRED)
    // and of the allocation index (registered whenever i changes)
    // -------------------------------------------------------------------------
    localparam logic [NHIST:1] SLOTM = slot_mask();

    logic [NHIST:1]   iv_d;           // i_d as a table one-hot (0 outside 1..NHIST)
    logic [NB-1:0]    pbank, abank, hbank, ibank;
    logic [2*E_W-1:0] prov_row;

    always_comb begin
        for (int unsigned t = 1; t <= NHIST; t++) iv_d[t] = (i_d == 5'(t));
    end

    for (genvar b = 0; b < NB; b++) begin : g_boh
        localparam logic [NHIST:1] M0 = bank_mask(b, 0);
        localparam logic [NHIST:1] M1 = bank_mask(b, 1);
        assign pbank[b] = x_q ? |(h1_s & M1) : |(h1_s & M0);
        assign abank[b] = x_q ? |(a1   & M1) : |(a1   & M0);
        assign hbank[b] = x_q ? |(hc1  & M1) : |(hc1  & M0);
        assign ibank[b] = x_q ? |(iv_d & M1) : |(iv_d & M0);
    end

    always_comb begin
        prov_row = '0;
        for (int unsigned b = 0; b < NB; b++)
            prov_row = prov_row | ({(2 * E_W){pbank[b]}} & {row_hi_s[b], row_lo_s[b]});
    end

    // -------------------------------------------------------------------------
    // Update datapath
    // -------------------------------------------------------------------------
    // MYRANDOM, one call per cycle at most. rnd_q holds the value the next
    // call returns, MYRANDOM(seed_q), and the generator is fed from rnd_q
    // only (no multiplexer in front of it):
    //   rng_load (S_IDLE, S_TAG, S_HIST, S_HASH): rnd_q <= seed_q
    //   rng_adv  (S_PRED, exactly one cycle):     rnd_q <= MYRANDOM(rnd_q)
    //   a call:  seed_q <= rnd_q,                 rnd_q <= MYRANDOM(rnd_q)
    //   otherwise held.
    // phist, ptghist and GTAG[4] change only in S_HIST / S_HASH (at their
    // edges), which are followed by S_IDLE, so rnd_q = seed_q is reloaded
    // after the last change and advanced once with the final values before
    // the first call (S_WAIT -> update FSM).
    localparam int unsigned G4_B0 = f_bank(4, 0);
    localparam int unsigned G4_B1 = f_bank(4, 1);
    localparam int unsigned G4_S  = f_slot(4);

    logic [TBITS-1:0] gtag4;        // GTAG[4] (with Numero), the RNG input
    logic [     31:0] rnd;          // MYRANDOM() of this cycle's call
    logic [     31:0] rnd_q, rnd_d, rng_out;
    logic             rng_en;       // MYRANDOM() is called this cycle
    logic             rng_load, rng_adv;

    assign gtag4    = x_q ? et_q[G4_B1][G4_S] : et_q[G4_B0][G4_S];
    assign rng_load = (state_q == S_IDLE) || (state_q == S_TAG) || (state_q == S_HIST)
                      || (state_q == S_HASH);
    assign rng_adv  = (state_q == S_PRED);

    bp_tage_rng #(
        .TBITS(TBITS)
    ) i_rng (
        .seed_i   (rnd_q),
        .phist_i  (phist_q),
        .ptghist_i(ptg_q),
        .gtag4_i  (gtag4),
        .seed_o   (rng_out)
    );

    assign rnd   = rnd_q;
    assign rnd_d = rng_load ? seed_q : ((rng_adv || rng_en) ? rng_out : rnd_q);

    // Entry operation of the current state: table i (S_AW, S_AWD), the
    // alternate (S_F_ALT), HCpred (S_F_HC) or the provider, at its branch
    // row. Its bank (one-hot) and slot come from flops registered ahead, so
    // the bank multiplexer's select is ready before the read data. Its row
    // comes from the checkpoint (provider, still valid) or from the bank's
    // read data of the previous cycle.
    logic [NB-1:0]     prov_oh_q, alt_oh_q, hc_oh_q, i_oh_q, op_oh;
    logic              prov_sl_q, alt_sl_q, hc_sl_q, i_sl_q, op_slot;
    logic              use_ck;
    logic [2*E_W-1:0]  op_rowb, op_row;
    logic [E_W-1:0]    op_ent;
    logic [TBITS-1:0]  op_tag;      // GTAG of the operated table (with Numero)
    logic              op_weak, op_ok;
    logic [2:0]        op_ctr_upd;

    always_ff @(posedge clk_i) begin
        if (state_q == S_PRED) begin
            prov_oh_q <= pbank;
            alt_oh_q  <= abank;
            hc_oh_q   <= hbank;
            prov_sl_q <= |(h1_s & SLOTM);
            alt_sl_q  <= |(a1 & SLOTM);
            hc_sl_q   <= |(hc1 & SLOTM);
        end
        i_oh_q <= ibank;                     // i_q <= i_d every cycle
        i_sl_q <= |(iv_d & SLOTM);
    end

    always_comb begin
        case (state_q)
            S_AW, S_AWD: begin op_oh = i_oh_q;    op_slot = i_sl_q;    end
            S_F_ALT:     begin op_oh = alt_oh_q;  op_slot = alt_sl_q;  end
            S_F_HC:      begin op_oh = hc_oh_q;   op_slot = hc_sl_q;   end
            default:     begin op_oh = prov_oh_q; op_slot = prov_sl_q; end
        endcase
        op_rowb = '0;
        op_tag  = '0;
        for (int unsigned b = 0; b < NB; b++) begin
            op_rowb = op_rowb | ({(2 * E_W){op_oh[b]}} & {bank_hi[b], bank_lo[b]});
            op_tag  = op_tag  | ({TBITS{op_oh[b]}} & (op_slot ? et_q[b][1] : et_q[b][0]));
        end
        use_ck  = (state_q == S_F_PROV) && ck_v_q;
        op_row  = use_ck ? ck_row_q : op_rowb;
        op_ent  = op_slot ? op_row[E_W +: E_W] : op_row[0 +: E_W];
        op_weak = (op_ent[2:0] == 3'b000) || (op_ent[2:0] == 3'b111);
        op_ok   = use_ck || rd_q;       // the row data is valid this cycle
    end

    bp_sat_ctr #(.WIDTH(3), .SIGNED(1'b1)) i_ent_ctr (
        .ctr_i(op_ent[2:0]),
        .inc_i(dir_q),
        .ctr_o(op_ctr_upd)
    );

    // bimodal counter: BIM = pred ? hyst : -1 - hyst (3-bit)
    logic [2:0] bim_val, bim_upd;
    logic       bim_pred_new;
    logic [1:0] bim_hyst_new;
    assign bim_val      = ck_bp_q ? {1'b0, ck_bh_q} : {1'b1, ~ck_bh_q};
    assign bim_pred_new = ~bim_upd[2];
    assign bim_hyst_new = bim_upd[2] ? ~bim_upd[1:0] : bim_upd[1:0];

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
        u_new = op_ent[4:3];
        if (lmp_q != altt_q) begin
            if (lmp_q == dir_q)                                 u_new = op_ent[4] ? 2'b11 : 2'b10;
            else if (op_ent[4:3] != 2'b00 && tpred_q == dir_q)  u_new = op_ent[4:3] - 2'b01;
        end
    end

    // Causes of a write, from the old entry's fields (no compare of the new
    // entry with the old one):
    //   ctr_chg    the counter steps: not at the limit of the outcome's direction
    //   u_chg      PROTECTU changes u (see u_new)
    //   alloc_same the allocated entry equals the one it replaces
    logic ctr_chg, u_chg, alloc_same;
    assign ctr_chg    = dir_q ? (op_ent[2:0] != 3'b011) : (op_ent[2:0] != 3'b100);
    assign u_chg      = (lmp_q != altt_q)
                        && ((lmp_q == dir_q) ? (op_ent[4:3] != 2'b11)
                                             : ((op_ent[4:3] != 2'b00) && (tpred_q == dir_q)));
    assign alloc_same = (op_ent[E_W-1:5] == op_tag) && (op_ent[4:3] == {1'b0, first_q})
                        && (op_ent[2:0] == (dir_q ? 3'b000 : 3'b111));

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
    logic           wr_en;       // entry wr_e for the operated table ...
    logic           wr_chg;      // ... and it differs from the stored one (cause)
    logic           wr_do;       // the write happens
    logic [E_W-1:0] wr_e;
    logic           bim_wr;
    logic           bp_do, bh_do;
    logic           sweep_rd, sweep_wr;
    logic           blk_end;

    assign blk_end = (num_q == 2'd3) || dir_q;
    assign bp_do   = bim_wr && (bim_pred_new != ck_bp_q);
    assign bh_do   = bim_wr && (bim_hyst_new != ck_bh_q);

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
        ck_row_d = ck_row_q;
        ck_v_d   = ck_v_q;
        ck_bp_d  = ck_bp_q;
        ck_bh_d  = ck_bh_q;
        tick_d   = tick_q;
        cm11_d   = cm11_q;
        clc_d    = clc_q;
        uaon_d   = uaon_q;
        c50_d    = c50_q;
        c1631_d  = c1631_q;
        rng_en   = 1'b0;
        lat_upd  = 1'b0;
        rd_req   = 1'b0;
        wr_en    = 1'b0;
        wr_chg   = 1'b0;
        wr_e     = op_ent;
        bim_wr   = 1'b0;
        sweep_rd = 1'b0;
        sweep_wr = 1'b0;
        hash_en  = 1'b0;

        // a final-update entry operation without valid row data reads its
        // bank this cycle and repeats the state (data valid next cycle)
        if (!op_ok && (state_q == S_F_ALT || (state_q == S_F_HC && hc_q != '0)
                       || (state_q == S_F_PROV && hit_q != '0))) begin
            rd_req = 1'b1;
        end else begin
            case (state_q)
                S_IDLE: begin
                    if (pred_fire) begin
                        state_d = PRED_REG ? S_TAG : S_PRED;
                    end else if (unc_fire) begin
                        lat_upd = 1'b1;
                        state_d = S_HIST;
                    end
                end

                S_TAG: begin
                    state_d = S_PRED;
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
                    // checkpoint: the provider's row and the bimodal entry
                    ck_row_d = prov_row;
                    ck_v_d   = 1'b1;
                    ck_bp_d  = bp_s;
                    ck_bh_d  = bh_s;
                    state_d  = S_WAIT;
                end

                S_WAIT: begin
                    if (upd_cond_fire) begin
                        lat_upd = 1'b1;
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
                            state_d = (rnd[2:0] != 3'd0) ? S_TICK : S_AW;
                        end else begin
                            state_d = S_AW;
                        end
                    end else begin
                        state_d = S_AW;
                    end
                end

                S_AW: begin
                    // read table i's row; j = MYRANDOM() % ASSOC (ASSOC = 1: j = 0)
                    rd_req  = 1'b1;
                    rng_en  = 1'b1;
                    state_d = S_AWD;
                end

                S_AWD: begin
                    if (op_ent[4:3] == 2'b00) begin
                        // new entry: tag, u = First (FORCEU), weak counter
                        wr_en   = 1'b1;
                        wr_chg  = !alloc_same;
                        wr_e    = {op_tag, 1'b0, first_q, dir_q ? 3'b000 : 3'b111};
                        na_d    = na_q + 4'd1;
                        if ((i_q >= 5'd3) || !first_q) maxna_d = maxna_q - 5'd1;
                        first_d = 1'b0;
                        i_d     = i_q + 5'd2;
                        state_d = S_AR1;
                    end else begin
                        // FORCEU: maybe clear u of a weak entry with u == 1
                        rng_en = 1'b1;
                        if (!rnd[0] && op_weak && (op_ent[4:3] == 2'b01)) begin
                            wr_en  = 1'b1;
                            wr_chg = 1'b1;             // u 1 -> 0
                            wr_e   = {op_ent[E_W-1:5], 2'b00, op_ent[2:0]};
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
                        ck_v_d  = 1'b0;                // the sweep rewrites rows
                        state_d = S_SWEEP;
                    end else begin
                        tick_d  = tick_sum[11:0];
                        state_d = ff_state;
                    end
                end

                S_SWEEP: begin
                    // read row sw while writing back row sw - 1
                    sweep_rd = !sw_q[LOGT];
                    sweep_wr = (sw_q != '0);
                    sw_d     = sw_q + LG1'(1);
                    if (sw_q[LOGT]) state_d = ff_state;
                end

                S_F_ALT: begin
                    wr_en   = 1'b1;
                    wr_chg  = ctr_chg;
                    wr_e    = {op_ent[E_W-1:3], op_ctr_upd};
                    state_d = S_F_HC;
                end

                S_F_HC: begin
                    if (hc_q != '0) begin
                        wr_en  = 1'b1;
                        wr_chg = ctr_chg;
                        wr_e   = {op_ent[E_W-1:3], op_ctr_upd};
                    end else begin
                        bim_wr = 1'b1;
                    end
                    state_d = S_F_PROV;
                end

                S_F_PROV: begin
                    if (hit_q != '0) begin
                        wr_en  = 1'b1;
                        wr_chg = ctr_chg || u_chg;
                        wr_e   = {op_ent[E_W-1:5], u_new, op_ctr_upd};
                    end else begin
                        bim_wr = 1'b1;
                    end
                    state_d = S_HIST;
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

        // a write is skipped when it would store the value the row holds; a
        // write to the provider's bank makes the checkpoint stale
        wr_do = wr_en && wr_chg;
        if (wr_do && |(op_oh & prov_oh_q)) ck_v_d = 1'b0;
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
    // SRAM ports (every bank is read and written at its branch row baddr_q,
    // the sweep at row sw / sw - 1)
    // -------------------------------------------------------------------------
    logic [LOGT-1:0] sw_m1;
    assign sw_m1 = LOGT'(sw_q - LG1'(1));

    for (genvar b = 0; b < NB; b++) begin : g_port
        logic           sel;      // the entry operation targets this bank
        logic           sw_chg;   // sweep: an entry of the row has u > 0
        logic [E_W-1:0] lo_dec, hi_dec;

        assign sel    = op_oh[b];
        assign sw_chg = (bank_lo[b][4:3] != 2'b00) || (bank_hi[b][4:3] != 2'b00);
        // u - 1 when u > 0 (global u reset), ctr and tag unchanged
        assign lo_dec = {bank_lo[b][E_W-1:5],
                         (bank_lo[b][4:3] != 2'b00) ? (bank_lo[b][4:3] - 2'b01) : 2'b00,
                         bank_lo[b][2:0]};
        assign hi_dec = {bank_hi[b][E_W-1:5],
                         (bank_hi[b][4:3] != 2'b00) ? (bank_hi[b][4:3] - 2'b01) : 2'b00,
                         bank_hi[b][2:0]};

        assign tb_re_o[b]    = pred_fire || sweep_rd || (rd_req && sel);
        assign tb_raddr_o[b] = sweep_rd ? sw_q[LOGT-1:0] : baddr_q[b];
        assign tb_we_o[b]    = (sweep_wr && sw_chg) || (wr_do && sel);
        assign tb_waddr_o[b] = sweep_wr ? sw_m1 : baddr_q[b];
        assign tb_wdata_o[2 * b * E_W +: 2 * E_W] =
            sweep_wr ? {hi_dec, lo_dec}
                     : (op_slot ? {wr_e, op_row[0 +: E_W]} : {op_row[E_W +: E_W], wr_e});
    end

    assign bp_re_o    = pred_fire;
    assign bp_raddr_o = bim_idx;
    assign bp_we_o    = bp_do;
    assign bp_waddr_o = bim_idx;
    assign bp_wdata_o = bim_pred_new;
    assign bh_re_o    = pred_fire;
    assign bh_raddr_o = bim_idx[LOGB-1:1];
    assign bh_we_o    = bh_do;
    assign bh_waddr_o = bim_idx[LOGB-1:1];
    assign bh_wdata_o = bim_hyst_new;

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
            rnd_q   <= '0;
            rd_q    <= 1'b0;
            ck_v_q  <= 1'b0;
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
            rnd_q   <= rnd_d;
            rd_q    <= rd_req;
            ck_v_q  <= ck_v_d;
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
        ck_row_q <= ck_row_d;
        ck_bp_q <= ck_bp_d;
        ck_bh_q <= ck_bh_d;
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
                         pcx[31], hit_p1[4], hit_p1[1:0], h_idx};
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
        // a write needs this cycle's row data (checkpoint or last cycle's read)
        if (rst_ni && wr_en && !op_ok)
            $error("bp_tage_core: entry write without valid row data");
    end

    // -------------------------------------------------------------------------
    // Reference (simulation only): the original per-table registers and
    // formulas. The per-bank addresses / tags, the one-hot selection, the
    // provider's row and the precomputed MYRANDOM (against the original
    // five-step formula, so bp_tage_rng is checked too) must equal them
    // every cycle (also proved in C++; this catches a transliteration
    // error).
    // -------------------------------------------------------------------------
    logic [NPH:1][LOGT-1:0]    ref_row_q;
    logic [NHIST:1][TBITS-1:0] ref_tag_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            ref_row_q <= '0;
            ref_tag_q <= '0;
        end else if (hash_en) begin
            for (int unsigned p = 1; p <= NPH; p++) begin
                if (p == 1) ref_row_q[p] <= h_idx[1][LOGT:1];
                else        ref_row_q[p] <= h_idx[2 * p - 1][LOGT-1:0];
            end
            ref_tag_q <= h_tag;
        end
    end

    logic [NHIST:1][TBITS-1:0] ref_gtag;
    logic [NHIST:1][ LOGT-1:0] ref_rowv;
    logic [NB-1:0][LOGT-1:0]   ref_baddr;
    logic [NHIST:0]            ref_hit, ref_dir, ref_weak;

    assign ref_hit[0]  = 1'b0;
    assign ref_dir[0]  = bp_s;
    assign ref_weak[0] = 1'b0;

    for (genvar t = 1; t <= NHIST; t++) begin : g_ref_t
        localparam int unsigned P   = (t + 1) / 2;
        localparam bit          DBL = (f_dbl(t) != 0);
        localparam int unsigned BX0 = f_bank(t, 0);
        localparam int unsigned BX1 = f_bank(t, 1);
        localparam int unsigned SL  = f_slot(t);

        logic [E_W-1:0] ent;

        assign ref_gtag[t] = ref_tag_q[t] ^ TBITS'(num_q);
        assign ref_rowv[t] = ref_row_q[P] ^ (LOGT'(num_q) << (DBL ? NSH_D : NSH_S));
        assign ent         = x_q ? (SL != 0 ? row_hi_s[BX1] : row_lo_s[BX1])
                                 : (SL != 0 ? row_hi_s[BX0] : row_lo_s[BX0]);
        assign ref_hit[t]  = (ent[E_W-1:5] == ref_gtag[t]);
        assign ref_dir[t]  = ~ent[2];
        assign ref_weak[t] = (ent[2:0] == 3'b000) || (ent[2:0] == 3'b111);
    end

    for (genvar b = 0; b < NB; b++) begin : g_ref_b
        if (b < 2 * NDPAIR) begin : g_dbl
            localparam int unsigned TLO = b - (b % 2) + 1;
            localparam int unsigned THI = TLO + SH_OFF;
            localparam bit          H   = ((b % 2) == 1);
            assign ref_baddr[b] = (H == x_q) ? ref_rowv[TLO] : ref_rowv[THI];
        end else begin : g_single
            localparam int unsigned T = 2 * (b - NDPAIR) + 1;
            assign ref_baddr[b] = ref_rowv[T];
        end
    end

    logic [3:0]        r_hit, r_alt, r_hc0, r_hc;
    logic              r_lmp, r_altt, r_pweak, r_hcp, r_tpred;
    logic [BANK_W-1:0] r_ckb;

    always_comb begin
        r_hit = '0;
        r_alt = '0;
        r_hc0 = '0;
        for (int unsigned t = 1; t <= NHIST; t++) begin
            if (ref_hit[t]) r_hit = 4'(t);
        end
        for (int unsigned t = 1; t <= NHIST; t++) begin
            if (ref_hit[t] && 4'(t) < r_hit)                 r_alt = 4'(t);
            if (ref_hit[t] && !ref_weak[t] && 4'(t) < r_hit) r_hc0 = 4'(t);
        end
        r_lmp   = ref_dir[r_hit];
        r_altt  = ref_dir[r_alt];
        r_pweak = ref_weak[r_hit];
        r_hc    = (r_hit == '0) ? '0 : (r_pweak ? r_hc0 : r_hit);
        r_hcp   = ref_dir[r_hc];
        if (r_hit == '0)                 r_tpred = bp_s;
        else if (uaon_q[4] || !r_pweak)  r_tpred = r_lmp;
        else                             r_tpred = r_hcp;
        r_ckb   = x_q ? bx1_l[r_hit] : bx0_l[r_hit];
    end

    // MYRANDOM in its original five-step form (C int: wrapping adds,
    // arithmetic shifts as sign replication), from the reference GTAG[4]
    function automatic logic [31:0] myrandom_ref(input logic [31:0]      seed,
                                                 input logic [26:0]      ph,
                                                 input logic [31:0]      ptg,
                                                 input logic [TBITS-1:0] g4);
        logic [31:0] v;
        v = seed + 32'd1 + 32'(ph);
        v = {{21{v[31]}}, v[31:21]} + {v[20:0], 11'd0};
        v = v + ptg;
        v = {{10{v[31]}}, v[31:10]} + {v[9:0], 22'd0};
        return v + 32'(g4);
    endfunction

    logic [31:0] rnd_ref;           // MYRANDOM(seed_q), computed directly
    assign rnd_ref = myrandom_ref(seed_q, phist_q, ptg_q, ref_gtag[4]);

    // the original entry-operation select: table number -> bank, slot
    logic [3:0] r_sel;
    logic       r_op_on;     // an entry operation is possible in this state
    always_comb begin
        case (state_q)
            S_AW, S_AWD: r_sel = i_q[3:0];
            S_F_ALT:     r_sel = alt_q;
            S_F_HC:      r_sel = hc_q;
            default:     r_sel = hit_q;
        endcase
        r_op_on = (state_q == S_AW) || (state_q == S_AWD) || (state_q == S_F_ALT)
                  || (state_q == S_F_HC && hc_q != '0) || (state_q == S_F_PROV && hit_q != '0);
    end

    always_ff @(posedge clk_i) begin
        if (rst_ni) begin
            if (r_op_on && (op_oh != (NB'(1) << (x_q ? bx1_l[r_sel] : bx0_l[r_sel]))
                            || op_slot != slot_l[r_sel]))
                $error("bp_tage_core: entry operation bank / slot differs from the reference");
            if (wr_en && op_ok && (wr_chg != (wr_e != op_ent)))
                $error("bp_tage_core: write enable (cause) differs from comparing the entries");
            for (int unsigned b = 0; b < NB; b++) begin
                if (baddr_q[b] != ref_baddr[b])
                    $error("bp_tage_core: bank %0d row %h != reference %h", b, baddr_q[b], ref_baddr[b]);
            end
            for (int unsigned t = 1; t <= NHIST; t++) begin
                if (et_q[x_q ? bx1_l[t] : bx0_l[t]][slot_l[t]] != ref_gtag[t])
                    $error("bp_tage_core: tag of table %0d differs from the reference", t);
            end
            if (state_q == S_PRED) begin
                if ({p_hit, p_alt, p_hc} != {r_hit, r_alt, r_hc}
                    || {p_lmp, p_altt, p_pweak, p_hcp, p_tpred}
                       != {r_lmp, r_altt, r_pweak, r_hcp, r_tpred})
                    $error("bp_tage_core: one-hot selection differs from the reference (hit %0d/%0d alt %0d/%0d hc %0d/%0d)",
                           p_hit, r_hit, p_alt, r_alt, p_hc, r_hc);
                if (r_hit != '0 && prov_row != {row_hi_s[r_ckb], row_lo_s[r_ckb]})
                    $error("bp_tage_core: provider row differs from the reference");
            end
            if (rng_en && rnd_q != rnd_ref)
                $error("bp_tage_core: precomputed MYRANDOM %h != %h", rnd_q, rnd_ref);
        end
    end
`endif

endmodule