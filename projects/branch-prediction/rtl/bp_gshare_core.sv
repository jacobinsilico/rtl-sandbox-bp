// -----------------------------------------------------------------------------
// Author: Jakub Dawid Szkudlarek
// SPDX-License-Identifier: Apache-2.0
//
// Description:
//   Gshare branch predictor core (McFarling, 1993), the RTL counterpart of
//   gshare.h in the CBP5 simulator. This is the unit that is synthesized: it
//   holds the global history register, the index hash, the predict/update
//   control and the counter update. The pattern history table (PHT) is an
//   SRAM outside this module; its read and write ports are this module's I/O.
//
//   Index (as gshare.h, power-of-two PHT only):
//     idx = pc[PC_SHIFT +: INDEX_BITS] ^ fold(ghr)
//   fold() XORs successive INDEX_BITS-wide chunks of the GHR together
//   (gshare.h FoldHistory); for GHR_BITS <= INDEX_BITS it is the GHR itself.
//   Prediction = MSB of the counter (counter > 1 for 2 bits, as gshare.h).
//
//   Predict -> update checkpoint (flops): the PHT index (idx_q) and the
//   counter read at predict time (ctr_q). The update does not re-read the
//   PHT: one read per conditional branch.
//
//   Protocol (stage 1: non-speculative, ONE branch in flight, as the CBP5
//   harness calls GetPrediction and UpdatePredictor back to back):
//     S_IDLE     - pred_req_ready_o = 1. A predict handshake reads the PHT at
//                  idx and latches idx. An UNCONDITIONAL update
//                  (upd_is_cond_i = 0, CBP5 TrackOtherInst) is also accepted
//                  here; gshare ignores it, as gshare.h does.
//     S_PRED     - the read data is valid: pred_resp_valid_o = 1 for one
//                  cycle, pred_resp_taken_o is the prediction, and the
//                  counter is latched into ctr_q.
//     S_WAIT_UPD - waits for the CONDITIONAL update (upd_is_cond_i = 1) of
//                  the same branch. In its handshake cycle the saturated
//                  counter is written to idx_q and the resolved direction is
//                  shifted into the GHR; then back to S_IDLE.
//   The write is skipped when the counter does not change (already saturated
//   in the resolved direction). A conditional branch takes 3 cycles with
//   back-to-back handshakes (1 PHT read, at most 1 PHT write); an
//   unconditional one takes 1.
//
//   Bit-exact with gshare.h: with one branch in flight nothing writes the
//   entry between predict and update, so ctr_q is what UpdatePredictor reads,
//   and the GHR (hence the index) only changes in the update cycle. A skipped
//   write would have stored the value the entry already holds.
//
//   Not modeled (stage 1): speculative history update; several branches in
//   flight (ctr_q would go stale if an older in-flight branch updated the
//   same entry); a PHT init FSM (the SRAM model starts at weakly taken, as
//   gshare.h's constructor).
//
//   Interface signals gshare does not use (upd_pc_i, upd_target_i, the upper
//   PC bits) are kept so all four predictor cores share one interface.
//
// Parameters:
//   GHR_BITS   - global history length (1..32, as gshare.h)
//   INDEX_BITS - log2(PHT entries) (1..31); the PHT has 2^INDEX_BITS entries
//   PC_SHIFT   - right shift of the PC before indexing (0..3); 2 for the
//                4-byte-aligned wireless CSV traces
//   CTR_BITS   - PHT counter width; 2 matches gshare.h
//   PC_W       - PC / target width of the shared predictor interface
// -----------------------------------------------------------------------------

`timescale 1 ns/1 ps

module bp_gshare_core #(
    parameter int unsigned GHR_BITS   = 14,
    parameter int unsigned INDEX_BITS = 14,
    parameter int unsigned PC_SHIFT   = 2,
    parameter int unsigned CTR_BITS   = 2,
    parameter int unsigned PC_W       = 64
) (
    input  logic                  clk_i,
    input  logic                  rst_ni,

    // predict channel (CBP5 GetPrediction)
    input  logic                  pred_req_valid_i,
    output logic                  pred_req_ready_o,
    input  logic [      PC_W-1:0] pred_req_pc_i,
    output logic                  pred_resp_valid_o,
    output logic                  pred_resp_taken_o,

    // update channel (CBP5 UpdatePredictor / TrackOtherInst)
    input  logic                  upd_valid_i,
    output logic                  upd_ready_o,
    input  logic                  upd_is_cond_i,
    input  logic [      PC_W-1:0] upd_pc_i,
    input  logic                  upd_taken_i,
    input  logic [      PC_W-1:0] upd_target_i,

    // PHT SRAM ports (1R1W, synchronous read)
    output logic                  pht_re_o,
    output logic [INDEX_BITS-1:0] pht_raddr_o,
    input  logic [  CTR_BITS-1:0] pht_rdata_i,
    output logic                  pht_we_o,
    output logic [INDEX_BITS-1:0] pht_waddr_o,
    output logic [  CTR_BITS-1:0] pht_wdata_o
);

    // -------------------------------------------------------------------------
    // Elaboration checks (same limits as gshare.h)
    // -------------------------------------------------------------------------
    if (GHR_BITS < 1 || GHR_BITS > 32) begin : g_check_ghr
        $error("bp_gshare_core: GHR_BITS must be in 1..32");
    end
    if (INDEX_BITS < 1 || INDEX_BITS > 31) begin : g_check_index
        $error("bp_gshare_core: INDEX_BITS must be in 1..31");
    end
    if (PC_SHIFT > 3) begin : g_check_shift
        $error("bp_gshare_core: PC_SHIFT must be in 0..3");
    end
    if (PC_SHIFT + INDEX_BITS > PC_W) begin : g_check_pc
        $error("bp_gshare_core: PC_SHIFT + INDEX_BITS exceeds PC_W");
    end

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        S_IDLE     = 2'd0,
        S_PRED     = 2'd1,
        S_WAIT_UPD = 2'd2
    } state_e;

    state_e                state_q, state_d;
    logic [  GHR_BITS-1:0] ghr_q;     // global history, newest outcome in bit 0
    logic [INDEX_BITS-1:0] idx_q;     // checkpoint: PHT index of the branch in flight
    logic [  CTR_BITS-1:0] ctr_q;     // checkpoint: its counter as read at predict

    // -------------------------------------------------------------------------
    // Index: inline history fold (gshare.h FoldHistory) XOR shifted PC
    // -------------------------------------------------------------------------
    localparam int unsigned NUM_CHUNKS = (GHR_BITS + INDEX_BITS - 1) / INDEX_BITS;

    logic [INDEX_BITS-1:0] ghr_fold;
    logic [INDEX_BITS-1:0] pred_idx;

    always_comb begin
        ghr_fold = '0;
        for (int unsigned i = 0; i < NUM_CHUNKS; i++) begin
            ghr_fold ^= INDEX_BITS'(ghr_q >> (i * INDEX_BITS));
        end
    end

    assign pred_idx = pred_req_pc_i[PC_SHIFT +: INDEX_BITS] ^ ghr_fold;

    // -------------------------------------------------------------------------
    // Handshakes
    // -------------------------------------------------------------------------
    logic pred_fire;       // predict request accepted
    logic upd_cond_fire;   // conditional update accepted

    assign pred_req_ready_o  = (state_q == S_IDLE);
    assign pred_resp_valid_o = (state_q == S_PRED);
    assign pred_resp_taken_o = pht_rdata_i[CTR_BITS-1];
    assign upd_ready_o       = (state_q == S_IDLE) || (state_q == S_WAIT_UPD);

    assign pred_fire     = pred_req_valid_i && pred_req_ready_o;
    assign upd_cond_fire = upd_valid_i && upd_is_cond_i && (state_q == S_WAIT_UPD);

    // -------------------------------------------------------------------------
    // PHT access: read at predict only; write in the update-handshake cycle
    // from the checkpointed counter, skipped when the counter does not change
    // -------------------------------------------------------------------------
    logic [CTR_BITS-1:0] ctr_upd;

    bp_sat_ctr #(
        .WIDTH (CTR_BITS),
        .SIGNED(1'b0)
    ) i_sat_ctr (
        .ctr_i (ctr_q),
        .inc_i (upd_taken_i),
        .ctr_o (ctr_upd)
    );

    assign pht_re_o    = pred_fire;
    assign pht_raddr_o = pred_idx;
    assign pht_we_o    = upd_cond_fire && (ctr_upd != ctr_q);
    assign pht_waddr_o = idx_q;
    assign pht_wdata_o = ctr_upd;

    // -------------------------------------------------------------------------
    // Control
    // -------------------------------------------------------------------------
    always_comb begin
        state_d = state_q;
        case (state_q)
            S_IDLE:     if (pred_fire)     state_d = S_PRED;
            S_PRED:                        state_d = S_WAIT_UPD;
            S_WAIT_UPD: if (upd_cond_fire) state_d = S_IDLE;
            default:                       state_d = S_IDLE;
        endcase
    end

    // Control state and history: reset.
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q <= S_IDLE;
            ghr_q   <= '0;
        end else begin
            state_q <= state_d;
            // shift in the resolved direction; truncation keeps GHR_BITS bits
            if (upd_cond_fire) ghr_q <= GHR_BITS'({ghr_q, upd_taken_i});
        end
    end

    // Checkpoint registers: no reset (always written before they are used).
    always_ff @(posedge clk_i) begin
        if (pred_fire)           idx_q <= pred_idx;
        if (state_q == S_PRED)   ctr_q <= pht_rdata_i;
    end

    // -------------------------------------------------------------------------
    // Unused interface signals (shared interface; gshare needs none of them)
    // -------------------------------------------------------------------------
    /* verilator lint_off UNUSEDSIGNAL */
    logic unused_ok;
    assign unused_ok = ^{upd_pc_i, upd_target_i, pred_req_pc_i};
    /* verilator lint_on UNUSEDSIGNAL */

    // -------------------------------------------------------------------------
    // Protocol checks (simulation only)
    // -------------------------------------------------------------------------
`ifdef VERILATOR
    always_ff @(posedge clk_i) begin
        if (rst_ni && upd_valid_i) begin
            if (upd_is_cond_i && state_q != S_WAIT_UPD)
                $error("bp_gshare_core: conditional update without a pending prediction");
            if (!upd_is_cond_i && state_q != S_IDLE)
                $error("bp_gshare_core: unconditional update while a branch is in flight");
        end
        if (rst_ni && pred_req_valid_i && !pred_req_ready_o)
            $error("bp_gshare_core: predict request while a branch is in flight");
    end
`endif

endmodule