#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Accuracy, logic PPA and latency of the four predictors vs storage budget,
#   from the results evaluate.py wrote (one row per predictor x budget x
#   trace). Each figure has one legend, shared by all its columns, and the x
#   label under every column.
#
#     mpkbr_<scale>    MPKBr, one column per trace. Default (--mpkbr steady):
#                      the CBP5 steady MPKBr the selection was made on --
#                      per trace from the CBP5 ledger, for the row's
#                      config_id, pooled over the window rtl_selection.csv
#                      names (41-50, or 6-10 for configs with 10 passes); its
#                      geomean over the traces is checked against the
#                      selection's steady_mpkbr. --mpkbr rtl: the RTL check's
#                      count instead (pass 1: a cold predictor). The plotted
#                      MPKBr is also written to <ppa>/mpkbr_<mode>.csv
#                      (predictor, budget_kb, trace, config_id, mpkbr,
#                      window), which ppa_combine.py reads by default.
#     energy_<scale>   logic energy per conditional branch, one column per
#                      trace: evaluate.py's e_logic_pj_per_cond -- power,
#                      cycles and branches all of the activity window
#                      (P * window cycles * T_clk / window cond. branches).
#     ppa_<scale>      logic area, logic power, SRAM reads and SRAM writes per
#                      conditional branch (power and accesses: mean over the
#                      traces, over the activity window).
#     latency_<scale>  minimum clock period (the STA critical path), the
#                      prediction latency in ns at that period, the
#                      prediction-path delay of the last stage, and the busy
#                      cycles per conditional branch.
#     area_power_latency_logic_<scale>
#                      before SRAM modelling: logic area, logic power and the
#                      end-to-end prediction latency (prediction cycles x the
#                      critical path, the SRAM read at the STA's 200 ps
#                      placeholder clock-to-Q).
#   Once ppa_combine.py has written <ppa>/ppa_table.csv (logic + SRAM):
#     area_power_latency_total_<scale>
#                      after SRAM modelling, same columns: logic + SRAM area;
#                      logic + SRAM power, at the
#                      same rate as the logic power (one branch per busy-cycle
#                      count of the bench: measured logic power + SRAM dynamic
#                      energy over busy cycles x clock + SRAM leakage); and the
#                      end-to-end prediction latency with the CACTI access
#                      time in place of the placeholder (prediction cycles x
#                      ppa_combine.py's cycle, pessimistic bound). Lines only,
#                      like the other figures; --apl-bands shades the area
#                      corners and the optimistic-to-pessimistic latency range.
#                      Each figure is scaled on its own; --apl-sharey gives
#                      the pair the same y range column by column.
#     total_<scale>    total energy per conditional branch (logic + SRAM
#                      reads / writes + SRAM leakage) and total area (logic +
#                      SRAM), in ppa_combine.py's baseline scenario; the band
#                      is the range over its sensitivity corners (scenarios,
#                      host cycles per branch, SRAM energy / area factors).
#                      Also the SRAM share of the energy, and the cycle the
#                      logic needs with the CACTI access time in place of the
#                      200 ps placeholder (pessimistic bound; dashed: target).
#     tradeoff_<scale> total energy and total area against MPKBr (mean over
#                      the traces), one curve per predictor through its
#                      budgets (labelled in KB / AU).
#
#   Prediction latency = cycles from the request handshake to pred_resp_valid
#   (LATENCY below, from the cores' pipelining knobs) x clock period. At a
#   design's own minimum period: cycles x critical path. The critical path
#   includes the SRAM read with a PLACEHOLDER clock-to-Q (sta_reports.tcl,
#   200 ps) until CACTI access times replace it. Busy cycles per branch is
#   the bench's occupancy with one branch in flight (prediction + update +
#   handshakes): an upper bound on a pipelined front end's cost, not its
#   throughput.
#
#   Every figure is written twice: <scale> = log (log y axis) and linear (y
#   axis from 0). x axis: storage budget in area units, 1 AU = 1 KB.
#
#   Every figure is also drawn for each predictor set in PLOT_SETS: all four
#   predictors in <out>/, and the same figures without the hashed perceptron
#   in <out>/no_hp/. A predictor keeps its colour and marker in every set;
#   the printed tables and the MPKBr CSV always cover all four.
#
#   mpkbr / energy / ppa / latency are logic only: the SRAMs are outside the
#   synthesized core, so neither the power nor the energy includes them (the
#   total / tradeoff figures do). Rows of the old flow (no activity
#   window: empty gls_win_lines) are skipped for power, energy and accesses,
#   and named. The plotted values are printed as tables, plus the sequential
#   share of the logic energy (flops, mostly clocking: no clock gating).
#
# Usage (after `source sourceme.sh`):
#   /usr/bin/python3 projects/branch-prediction/scripts/plot_ppa.py [options]
#     --csv CSV ...      result CSVs (default: eval/<predictor>_results.csv of
#                        every predictor that has one)
#     --mpkbr MODE       steady (default) or rtl
#     --cbp DIR          CBP5 repo (default /home/j50065913/cbp5-wireless/cbp5-new-golden)
#     --ledger CSV       CBP5 ledger (default <cbp>/scripts/bp-comparison/results/
#                        results_traces_csv_long.csv)
#     --selection CSV    rtl_selection.csv (default <cbp>/scripts/bp-comparison/
#                        analysis/traces_csv_long/rtl_compare/rtl_selection.csv)
#     --traces T ...     MPKBr and energy columns, and the traces averaged for
#                        power and SRAM accesses (default: fdd_su_v1_0
#                        tdd_dl_mu_v1_0 tdd_ul_mu_v1_0)
#     --sharey           one y range across the MPKBr columns, and one across
#                        the energy columns
#     --apl-sharey       one y range per column across the two
#                        area_power_latency figures (default: each its own)
#     --apl-bands        shade the area corners and the latency bounds on
#                        area_power_latency_total (default: lines only)
#     --max-kb KB        plot only budgets up to KB (default: all budgets)
#     --out DIR          default eval/plots/
#     --ppa DIR          ppa_combine.py's output (default eval/ppa/): where the
#                        MPKBr CSV goes and the total / tradeoff data comes from
#     --mpkbr-csv PATH   MPKBr CSV (default <ppa>/mpkbr_<mode>.csv; none = not
#                        written; not written with --max-kb unless given)
#     --format FMT       png | pdf | svg (default png)
#
#   Examples:
#     /usr/bin/python3 projects/branch-prediction/scripts/plot_ppa.py
#     /usr/bin/python3 projects/branch-prediction/scripts/plot_ppa.py --format pdf --sharey
#     /usr/bin/python3 projects/branch-prediction/scripts/plot_ppa.py --mpkbr rtl
#     /usr/bin/python3 projects/branch-prediction/scripts/plot_ppa.py --max-kb 16
#
# Output (every file once in <out>/ and once in <out>/no_hp/):
#   <out>/mpkbr_log.<fmt>    <out>/mpkbr_linear.<fmt>
#   <out>/energy_log.<fmt>   <out>/energy_linear.<fmt>
#   <out>/ppa_log.<fmt>      <out>/ppa_linear.<fmt>
#   <out>/latency_log.<fmt>  <out>/latency_linear.<fmt>
#   <out>/area_power_latency_logic_log.<fmt>   <out>/area_power_latency_logic_linear.<fmt>
#   <out>/area_power_latency_total_log.<fmt>   <out>/area_power_latency_total_linear.<fmt>
#                                                          (with ppa_combine.py's output)
#   <out>/total_log.<fmt>    <out>/total_linear.<fmt>      (with ppa_combine.py's output)
#   <out>/tradeoff_log.<fmt> <out>/tradeoff_linear.<fmt>   (with ppa_combine.py's output)
#   <out>/no_hp/...          the same figures without the hashed perceptron
#   <ppa>/mpkbr_<mode>.csv
#
#   Order for the logic + SRAM figures: plot_ppa.py (writes the MPKBr CSV),
#   ppa_combine.py, plot_ppa.py again (draws total / tradeoff).
# -----------------------------------------------------------------------------

import argparse
import csv
import math
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.ticker import (FixedLocator, FuncFormatter, LogLocator, MaxNLocator,  # noqa: E402
                               NullFormatter, NullLocator)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
EVAL_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "eval")
CBP_DEFAULT = "/home/j50065913/cbp5-wireless/cbp5-new-golden"

TRACES = ["fdd_su_v1_0", "tdd_dl_mu_v1_0", "tdd_ul_mu_v1_0"]

# plotting and legend order: predictor -> (legend label, colour, marker)
STYLE = {
    "gshare":            ("Gshare",            "#E69F00", "o"),
    "g_perceptron":      ("Perceptron",        "#56B4E9", "s"),
    "hashed_perceptron": ("Hashed Perceptron", "#009E73", "^"),
    "tage_cb":           ("TAGE",              "#D55E00", "D"),
}

# Predictor sets every figure is drawn for: (subdirectory of --out, "" = --out
# itself; predictors left out of that set)
PLOT_SETS = [
    ("",      ()),
    ("no_hp", ("hashed_perceptron",)),
]

# Prediction latency in cycles (request handshake -> pred_resp_valid), from
# the cores' pipelining knobs; a knob missing from the row's rtl_params takes
# the core's default. Check against bp_<predictor>_core.sv if a default moves.
LATENCY = {
    "gshare":            lambda k: 1,
    "g_perceptron":      lambda k: 1 + k.get("PSUM_REG", 1) + k.get("Y_REG", 0),
    "hashed_perceptron": lambda k: 1 + k.get("IDX_REG", 0) + k.get("Y_REG", 0),
    "tage_cb":           lambda k: 1 + k.get("PRED_REG", 1),
}

X_LABEL = "Storage budget (area units)"
Y_ENERGY = "Logic energy [pJ / cond. branch]"

# PPA columns: (CSV column, scale, column title, y label, mean over traces, table format)
PPA = [
    ("area_um2",         1.0, "Logic area",  "Area [µm²]",            False, ".1f"),
    ("p_total_w",        1e6, "Logic power", "Power [µW]",            True,  ".2f"),
    ("sram_rd_per_cond", 1.0, "SRAM reads",  "Reads / cond. branch",  True,  ".3f"),
    ("sram_wr_per_cond", 1.0, "SRAM writes", "Writes / cond. branch", True,  ".3f"),
]


# =============================================================================
# Data
# =============================================================================

def num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def passed(v):
    return str(v).strip().lower() in ("true", "1")


def windowed(r):
    """A row of the current flow: passed gate-level run with an activity window."""
    return passed(r.get("gls_pass")) and num(r.get("gls_win_lines")) is not None


def knobs(r):
    out = {}
    for tok in (r.get("rtl_params") or "").replace(";", " ").split():
        k, _, v = tok.partition("=")
        if v.lstrip("-").isdigit():
            out[k] = int(v)
    return out


def load(paths):
    """{(predictor, budget_kb, trace): row}; a later CSV overrides an earlier one."""
    rows = {}
    for path in paths:
        with open(path, newline="") as f:
            for r in csv.DictReader(f):
                kb = num(r.get("budget_kb"))
                if r.get("predictor") in STYLE and kb is not None and r.get("trace"):
                    rows[(r["predictor"], kb, r["trace"])] = r
    return rows


def short(trace):
    return trace.split(".")[0]


def label_of(r, kb):
    return r.get("budget") or f"{kb:g}KB"


def geomean(vals):
    return math.exp(sum(math.log(v) for v in vals) / len(vals)) if vals and min(vals) > 0 else None


# ---- CBP5 steady MPKBr -------------------------------------------------------

def load_selection(path):
    """config_id -> {window: (first, last), steady: geomean} from rtl_selection.csv."""
    out = {}
    if not path or not os.path.exists(path):
        return out
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            w = (r.get("window") or "").split("-")
            win = (int(w[0]), int(w[1])) if len(w) == 2 and all(x.isdigit() for x in w) else None
            out[r.get("config_id")] = {"window": win, "steady": num(r.get("steady_mpkbr"))}
    return out


def load_ledger(path, ids, traces):
    """(config_id, short trace) -> [(mispred, cond)] per pass: the ok row with
    the most passes (pass lists 'a|b|...', as pipelib reads them)."""
    out = {}
    want = set(traces)
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            cid, t = r.get("config_id"), short(r.get("trace") or "")
            if cid not in ids or t not in want or (r.get("status") or "ok") != "ok":
                continue
            m, c = r.get("pass_mispred") or "", r.get("pass_cond") or ""
            if not m or not c:
                continue
            passes = [(int(a), int(b)) for a, b in zip(m.split("|"), c.split("|"))]
            if (cid, t) not in out or len(passes) >= len(out[(cid, t)]):
                out[(cid, t)] = passes
    return out


def steady(passes, window):
    """Pooled MPKBr over passes window[0]..window[1]; without a window, the
    sweeps' rule on the longest standard cap present (50, else 10): the last
    min(10, cap / 2) passes."""
    if window is None:
        cap = 50 if len(passes) >= 50 else 10
        window = (cap - min(10, cap // 2) + 1, cap)
    lo, hi = window
    if len(passes) < hi:
        return None, window
    sel = passes[lo - 1:hi]
    c = sum(x[1] for x in sel)
    return (1000.0 * sum(x[0] for x in sel) / c if c else None), window


def kb_text(kb):
    return int(kb) if float(kb).is_integer() else kb


def mpkbr_series(rows, traces, a):
    """-> ({trace: {predictor: [(kb, mpkbr)]}}, [CSV record per predictor x budget x trace])"""
    out = {t: defaultdict(list) for t in traces}
    recs = []
    if a.mpkbr == "rtl":
        for (p, kb, t), r in sorted(rows.items()):
            m = num(r.get("mpkbr"))
            if t in out and m is not None:
                out[t][p].append((kb, m))
                recs.append({"predictor": p, "budget_kb": kb_text(kb), "trace": t,
                             "config_id": r.get("config_id"), "mpkbr": m, "window": "rtl pass 1"})
        print("  MPKBr: the RTL check's count (pass 1, a cold predictor)")
        return out, recs

    if not os.path.exists(a.ledger):
        sys.exit(f"no CBP5 ledger {a.ledger}: give --ledger / --cbp, or --mpkbr rtl")
    sel = load_selection(a.selection)
    if not sel:
        print(f"WARNING no {a.selection}: steady windows from each run's pass count")
    ids = {r.get("config_id") for r in rows.values() if r.get("config_id")}
    led = load_ledger(a.ledger, ids, traces)
    windows = defaultdict(list)
    for p, kb in sorted({(p, kb) for p, kb, _ in rows}):
        cid = next((r.get("config_id") for (q, k, _), r in rows.items() if (q, k) == (p, kb)), None)
        s = sel.get(cid, {})
        vals = {}
        for t in traces:
            passes = led.get((cid, t))
            if passes is None:
                print(f"  skip MPKBr {p} {kb:g}KB {t}: {cid} not in the ledger")
                continue
            v, win = steady(passes, s.get("window"))
            if v is None:
                print(f"  skip MPKBr {p} {kb:g}KB {t}: {len(passes)} passes, window {win[0]}-{win[1]}")
                continue
            out[t][p].append((kb, v))
            vals[t] = v
            windows[f"{win[0]}-{win[1]}"].append(f"{p}:{kb:g}KB")
            recs.append({"predictor": p, "budget_kb": kb_text(kb), "trace": t, "config_id": cid,
                         "mpkbr": v, "window": f"{win[0]}-{win[1]}"})
        g = geomean(list(vals.values())) if len(vals) == len(traces) else None
        if g and s.get("steady") and abs(g / s["steady"] - 1) > 0.005:
            print(f"WARNING {p} {kb:g}KB: geomean {g:.4f} != the selection's {s['steady']:.4f}")
    for w, runs in sorted(windows.items()):
        print(f"  MPKBr: CBP5 steady, passes {w}: {len(set(runs))} configs"
              + ("" if len(windows) == 1 else f" ({' '.join(sorted(set(runs)))})"))
    return out, recs


def write_mpkbr_csv(path, recs):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["predictor", "budget_kb", "trace", "config_id", "mpkbr",
                                          "window"], lineterminator="\n")
        w.writeheader()
        w.writerows(recs)
    print(f"  MPKBr per trace -> {path} (read by ppa_combine.py)")


# ---- logic + SRAM (ppa_combine.py) -------------------------------------------

def load_combined(ppa_dir, max_kb):
    """ppa_combine.py's baseline table and sensitivity -> ({predictor: {kb: row}},
    {(predictor, kb): sensitivity row}), or None before ppa_combine.py ran."""
    tp, sp = os.path.join(ppa_dir, "ppa_table.csv"), os.path.join(ppa_dir, "sensitivity.csv")
    if not os.path.exists(tp):
        return None
    base, sens = defaultdict(dict), {}
    with open(tp, newline="") as f:
        for r in csv.DictReader(f):
            kb = num(r.get("budget_kb"))
            if r.get("predictor") in STYLE and kb is not None and (max_kb is None or kb <= max_kb):
                base[r["predictor"]][kb] = r
    if os.path.exists(sp):
        with open(sp, newline="") as f:
            for r in csv.DictReader(f):
                kb = num(r.get("budget_kb"))
                if r.get("predictor") in STYLE and kb is not None:
                    sens[(r["predictor"], kb)] = r
    return base, sens


def combined_series(base, sens):
    """-> panels and bands of the total figure, from ppa_combine.py's output."""
    keys = ("energy", "area", "sram_share", "period")
    out = {k: defaultdict(list) for k in keys}
    bands = {"energy": defaultdict(list), "area": defaultdict(list)}
    for p, by_kb in base.items():
        for kb, r in sorted(by_kb.items()):
            e, ar = num(r.get("e_total_pj")), num(r.get("area_um2"))
            sh, per = num(r.get("e_sram_share")), num(r.get("period_pes_ns"))
            if e is not None:
                out["energy"][p].append((kb, e))
            if ar is not None:
                out["area"][p].append((kb, ar))
            if sh is not None:
                out["sram_share"][p].append((kb, 100.0 * sh))
            if per is not None:
                out["period"][p].append((kb, 1000.0 * per))
            s = sens.get((p, kb))
            if s:
                lo, hi = num(s.get("e_min_pj")), num(s.get("e_max_pj"))
                if lo is not None and hi is not None:
                    bands["energy"][p].append((kb, lo, hi))
                lo, hi = num(s.get("area_min_um2")), num(s.get("area_max_um2"))
                if lo is not None and hi is not None:
                    bands["area"][p].append((kb, lo, hi))
    return out, bands


def logic_apl_panels(ppa, lat):
    """Before SRAM modelling: logic area, logic power and the end-to-end
    prediction latency (the SRAM read at the STA's placeholder)."""
    return [("Logic area", "Area [µm²]", ppa["area_um2"]),
            ("Logic power", "Power [µW]", ppa["p_total_w"]),
            ("Prediction latency", "Latency [ns]", lat["lat_ns"])]


def total_apl(base, bands, ppa, cycles, clk_ns):
    """After SRAM modelling (ppa_combine.py's baseline): -> (panels, bands).
      area     logic + SRAM; band: the area corners
      power    the measured logic power (µW, mean over the traces) + the SRAM
               dynamic energy spread over the same time per branch (busy
               cycles x clock) + the SRAM leakage power
      latency  prediction cycles x the cycle with the CACTI access time;
               line: pessimistic bound, band: optimistic to pessimistic"""
    p_logic = {p: dict(pts) for p, pts in ppa["p_total_w"].items()}
    area, power, latency, lat_band = (defaultdict(list) for _ in range(4))
    for p, by_kb in base.items():
        for kb, r in sorted(by_kb.items()):
            ar = num(r.get("area_um2"))
            if ar is not None:
                area[p].append((kb, ar))
            pl, busy = p_logic.get(p, {}).get(kb), num(r.get("busy_cyc"))
            rd, wr, leak = (num(r.get(k)) for k in ("e_sram_rd_pj", "e_sram_wr_pj", "sram_leak_uw"))
            if None not in (pl, busy, rd, wr, leak) and busy > 0 and clk_ns:
                power[p].append((kb, pl + 1000.0 * (rd + wr) / (busy * clk_ns) + leak))  # pJ/ns = mW
            n = cycles.get(p, {}).get(kb)
            lo, hi = num(r.get("period_opt_ns")), num(r.get("period_pes_ns"))
            if n and hi is not None:
                latency[p].append((kb, n * hi))
                if lo is not None:
                    lat_band[p].append((kb, n * lo, n * hi))
    panels = [("Logic + SRAM area", "Area [µm²]", area),
              ("Logic + SRAM power", "Power [µW]", power),
              ("Prediction latency with SRAM", "Latency [ns]", latency)]
    return panels, {0: dict(bands.get("area", {})), 2: lat_band}


def common_ylims(panel_sets, log, band_sets=()):
    """{panel index: (low, high)} covering that panel in every set (and its
    bands), so a before / after pair shares its y ranges column by column."""
    vals = defaultdict(list)
    for panels in panel_sets:
        for i, (_, _, series) in enumerate(panels):
            vals[i] += [y for pts in series.values() for _, y in pts
                        if y is not None and (not log or y > 0)]
    for bands in band_sets:
        for i, b in (bands or {}).items():
            vals[i] += [v for pts in b.values() for _, lo, hi in pts for v in (lo, hi)
                        if v is not None and (not log or v > 0)]
    return {i: ((min(v) / 1.25, max(v) * 1.25) if log else (0.0, max(v) * 1.08))
            for i, v in vals.items() if v}


# ---- logic energy, PPA, latency ----------------------------------------------

def check_windows(what, windows):
    """windows: {trace: {window lines: [run, ...]}}: warns when one trace was
    measured over different activity windows."""
    for t, by in windows.items():
        if not by:
            continue
        common = max(by, key=lambda c: len(by[c]))
        print(f"  {what} of {t}: window of {common} branch lines")
        odd = [f"{run} ({c})" for c, runs in sorted(by.items()) if c != common for run in runs]
        if odd:
            print(f"WARNING {what} of {t}: different window for {' '.join(odd)}")


def energy_series(rows, traces):
    """{trace: {predictor: [(kb, pJ per cond. branch)]}}: evaluate.py's
    e_logic_pj_per_cond (activity window)."""
    out = {t: defaultdict(list) for t in traces}
    skipped = defaultdict(list)
    for (p, kb, t), r in sorted(rows.items()):
        if t not in out:
            continue
        e = num(r.get("e_logic_pj_per_cond"))
        if not windowed(r) or e is None:
            skipped[t].append(f"{p}:{label_of(r, kb)}")
            continue
        out[t][p].append((kb, e))
    for t, runs in skipped.items():
        print(f"  skip energy of {t} (no windowed gate-level run): {' '.join(runs)}")
    return out


def ppa_series(rows, traces):
    """{column: {predictor: [(kb, value)]}}: area per budget, the rest the
    mean over the traces (windowed rows only)."""
    out = {col: defaultdict(list) for col, *_ in PPA}
    windows = {t: defaultdict(list) for t in traces}
    for p, kb in sorted({(p, kb) for p, kb, _ in rows}):
        runs = [r for (q, k, _), r in rows.items() if (q, k) == (p, kb)]
        label = label_of(runs[0], kb)
        ok = {t: rows[(p, kb, t)] for t in traces if (p, kb, t) in rows and windowed(rows[(p, kb, t)])}
        for t, r in ok.items():
            windows[t][r.get("gls_win_lines")].append(f"{p}:{label}")
        for col, scale, _title, _ylabel, per_trace, _fmt in PPA:
            if not per_trace:
                v = next((num(r.get(col)) for r in runs if num(r.get(col)) is not None), None)
                if v is not None:
                    out[col][p].append((kb, scale * v))
                continue
            vals = [num(ok[t].get(col)) if t in ok else None for t in traces]
            if None in vals:
                miss = [t for t, v in zip(traces, vals) if v is None]
                print(f"  skip {col} {p} {label}: no windowed gate-level value for {' '.join(miss)}")
                continue
            out[col][p].append((kb, scale * sum(vals) / len(vals)))
    check_windows("power / energy / SRAM accesses", windows)
    return out


def latency_series(rows, traces):
    """-> (panels {key: {predictor: [(kb, value)]}}, cycles {predictor: {kb: n}})."""
    keys = ("period", "lat_ns", "pred_ps", "busy")
    out = {k: defaultdict(list) for k in keys}
    cyc = defaultdict(dict)
    for p, kb in sorted({(p, kb) for p, kb, _ in rows}):
        runs = [r for (q, k, t), r in rows.items() if (q, k) == (p, kb) and t in traces]
        if not runs:
            continue
        crit = next((num(r.get("crit_delay_ps")) for r in runs if num(r.get("crit_delay_ps"))), None)
        pred = next((num(r.get("pred_path_ps")) for r in runs if num(r.get("pred_path_ps"))), None)
        n = LATENCY[p](knobs(runs[0]))
        cyc[p][kb] = n
        if crit:
            out["period"][p].append((kb, crit))
            out["lat_ns"][p].append((kb, n * crit / 1000.0))
        if pred:
            out["pred_ps"][p].append((kb, pred))
        busy = [num(r.get("cycles_per_cond")) for r in runs if windowed(r)]
        if len(busy) == len(traces) and None not in busy:
            out["busy"][p].append((kb, sum(busy) / len(busy)))
    return out, cyc


def seq_share(rows, traces):
    """{predictor: [(kb, % of the logic energy in sequential cells)]}"""
    out = defaultdict(list)
    for p, kb in sorted({(p, kb) for p, kb, _ in rows}):
        sh = []
        for t in traces:
            r = rows.get((p, kb, t))
            s, e = (num(r.get("e_seq_pj_per_cond")), num(r.get("e_logic_pj_per_cond"))) if r else (None, None)
            if r and windowed(r) and s is not None and e:
                sh.append(100.0 * s / e)
        if len(sh) == len(traces):
            out[p].append((kb, sum(sh) / len(sh)))
    return out


def table(title, series, fmt):
    print(title)
    for p, (label, _, _) in STYLE.items():
        pts = sorted(series.get(p, []))
        if pts:
            print(f"  {label:18s} " + "  ".join(f"{x:g}AU={y:{fmt}}" for x, y in pts))


# =============================================================================
# Plotting
# =============================================================================

def without(panels, excl):
    """panels [(title, y label, {predictor: points})] minus the predictors in excl."""
    return [(title, ylabel, {p: v for p, v in series.items() if p not in excl})
            for title, ylabel, series in panels]


def without_bands(bands, excl):
    """{panel index: {predictor: band}} minus the predictors in excl."""
    return {i: {p: v for p, v in b.items() if p not in excl} for i, b in (bands or {}).items()}


def plain(v, _pos=None):
    """Tick label in plain numbers: 2, 0.5, 20000 (no 1e+04)."""
    return f"{v:.0f}" if v >= 1 and float(v).is_integer() else f"{v:g}"


def budget_axis(ax, budgets):
    ax.set_xscale("log", base=2)
    ax.xaxis.set_major_locator(FixedLocator(budgets))
    ax.xaxis.set_minor_locator(NullLocator())
    ax.xaxis.set_major_formatter(FuncFormatter(plain))
    ax.grid(True, which="major", alpha=0.3)


def log_y(ax):
    """Log y axis in plain numbers: the sparsest of 1-2-5, 1-2-3-5 and 1..9 steps
    that labels at least 3 ticks, else evenly spaced ticks (very narrow range)."""
    ax.set_yscale("log")
    lo, hi = ax.get_ylim()
    loc = None
    for subs in ((1.0, 2.0, 5.0), (1.0, 2.0, 3.0, 5.0), tuple(float(s) for s in range(1, 10))):
        cand = LogLocator(base=10, subs=subs)
        if sum(lo <= t <= hi for t in cand.tick_values(lo, hi)) >= 3:
            loc = cand
            break
    ax.yaxis.set_major_locator(loc or MaxNLocator(nbins=4))
    ax.yaxis.set_major_formatter(FuncFormatter(plain))
    ax.yaxis.set_minor_formatter(NullFormatter())
    ax.grid(True, which="minor", alpha=0.1)


def log_x(ax):
    """Log x axis in plain numbers, ticks as log_y picks them."""
    ax.set_xscale("log")
    lo, hi = ax.get_xlim()
    loc = None
    for subs in ((1.0, 2.0, 5.0), (1.0, 2.0, 3.0, 5.0), tuple(float(s) for s in range(1, 10))):
        cand = LogLocator(base=10, subs=subs)
        if sum(lo <= t <= hi for t in cand.tick_values(lo, hi)) >= 3:
            loc = cand
            break
    ax.xaxis.set_major_locator(loc or MaxNLocator(nbins=4))
    ax.xaxis.set_major_formatter(FuncFormatter(plain))
    ax.xaxis.set_minor_formatter(NullFormatter())


def linear_y(ax):
    ax.set_ylim(bottom=0)
    ax.ticklabel_format(axis="y", style="plain", useOffset=False)


def draw(ax, series, log):
    """One line per predictor. -> the predictors drawn."""
    drawn = set()
    for p, (_, colour, marker) in STYLE.items():
        pts = sorted((x, y) for x, y in series.get(p, []) if not log or y > 0)
        if pts:
            ax.plot([x for x, _ in pts], [y for _, y in pts], color=colour, marker=marker,
                    lw=1.8, ms=6)
            drawn.add(p)
    return drawn


def figure(panels, log, sharey, path, hline=None, bands=None, ylims=None):
    """panels: [(column title, y label or None, {predictor: [(kb, y)]})].
    hline: {panel index: (y, label)} reference lines. bands: {panel index:
    {predictor: [(kb, low, high)]}} shaded ranges. ylims: {panel index:
    (low, high)} fixed y ranges (e.g. shared by a before / after pair)."""
    budgets = sorted({x for _, _, s in panels for pts in s.values() for x, _ in pts})
    if not budgets:
        print(f"  no data, skipped {path}")
        return
    h = 3.6
    fig, axes = plt.subplots(1, len(panels), figsize=(3.9 * len(panels), h),
                             squeeze=False, sharey=sharey)
    drawn = set()
    for i, (ax, (_, _, series)) in enumerate(zip(axes[0], panels)):  # data first (shared y range)
        for p, pts in ((bands or {}).get(i) or {}).items():
            pts = sorted(q for q in pts if not log or q[1] > 0)
            if pts:
                ax.fill_between([x for x, _, _ in pts], [lo for _, lo, _ in pts],
                                [hi for _, _, hi in pts], color=STYLE[p][1], alpha=0.15, lw=0)
        drawn |= draw(ax, series, log)
    for i, (ax, (title, ylabel, _)) in enumerate(zip(axes[0], panels)):
        if hline and i in hline:
            y, lab = hline[i]
            ax.axhline(y, color="0.4", lw=1.0, ls="--")
            ax.annotate(lab, (0.98, y), xycoords=("axes fraction", "data"), fontsize=8,
                        color="0.3", va="top", ha="right")
        budget_axis(ax, budgets)
        if ylims and i in ylims:
            ax.set_yscale("log" if log else "linear")
            ax.set_ylim(*ylims[i])
        log_y(ax) if log else linear_y(ax)
        ax.set_title(title, fontsize=11)
        ax.set_xlabel(X_LABEL)
        if ylabel:
            ax.set_ylabel(ylabel)
    fig.tight_layout(rect=(0, 0.4 / h, 1, 1))             # bottom 0.4 in: legend
    handles = [Line2D([], [], color=c, marker=m, lw=1.8, ms=6, label=lab)
               for p, (lab, c, m) in STYLE.items() if p in drawn]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, 0.0),
               ncol=len(handles), frameon=False, fontsize=10)
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> {path}")


def tradeoff_figure(base, log, path, title_note):
    """Total energy and total area against MPKBr, one curve per predictor
    through its budgets."""
    panels = [("e_total_pj", "Logic + SRAM energy", "Energy [pJ / cond. branch]"),
              ("area_um2", "Logic + SRAM area", "Area [µm²]")]
    h = 4.2
    fig, axes = plt.subplots(1, len(panels), figsize=(5.2 * len(panels), h), squeeze=False)
    drawn = set()
    for ax, (col, title, ylabel) in zip(axes[0], panels):
        for p, (_, colour, marker) in STYLE.items():
            pts = sorted((kb, num(r.get("mpkbr")), num(r.get(col))) for kb, r in base.get(p, {}).items())
            pts = [q for q in pts if q[1] is not None and q[2] is not None and q[1] > 0
                   and (not log or q[2] > 0)]
            if not pts:
                continue
            ax.plot([x for _, x, _ in pts], [y for _, _, y in pts], color=colour, marker=marker,
                    lw=1.5, ms=6)
            for kb, x, y in pts:
                ax.annotate(f"{kb:g}", (x, y), textcoords="offset points", xytext=(4, 4),
                            fontsize=7, color=colour)
            drawn.add(p)
        log_x(ax)
        ax.grid(True, which="major", alpha=0.3)
        log_y(ax) if log else linear_y(ax)
        ax.set_title(f"{title} vs accuracy", fontsize=11)
        ax.set_xlabel("MPKBr, mean over traces (labels: budget, AU)")
        ax.set_ylabel(ylabel)
    if not drawn:
        plt.close(fig)
        print(f"  no data, skipped {path}")
        return
    fig.suptitle(title_note, fontsize=9, y=0.995)
    fig.tight_layout(rect=(0, 0.4 / h, 1, 0.97))
    handles = [Line2D([], [], color=c, marker=m, lw=1.5, ms=6, label=lab)
               for p, (lab, c, m) in STYLE.items() if p in drawn]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, 0.0),
               ncol=len(handles), frameon=False, fontsize=10)
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> {path}")


# =============================================================================
# Main
# =============================================================================

def main():
    ap = argparse.ArgumentParser(description="MPKBr, energy, PPA and latency plots (see the header).")
    ap.add_argument("--csv", nargs="+", default=None)
    ap.add_argument("--mpkbr", choices=["steady", "rtl"], default="steady")
    ap.add_argument("--cbp", default=CBP_DEFAULT)
    ap.add_argument("--ledger", default=None)
    ap.add_argument("--selection", default=None)
    ap.add_argument("--traces", nargs="+", default=TRACES)
    ap.add_argument("--sharey", action="store_true",
                    help="one y range across the MPKBr columns, and one across the energy columns")
    ap.add_argument("--apl-sharey", action="store_true",
                    help="one y range per column across the two area_power_latency figures")
    ap.add_argument("--apl-bands", action="store_true",
                    help="shade the area corners and the latency bounds on area_power_latency_total")
    ap.add_argument("--max-kb", type=float, default=None,
                    help="plot only budgets up to this many KB (default: all)")
    ap.add_argument("--out", default=os.path.join(EVAL_DIR, "plots"))
    ap.add_argument("--ppa", default=os.path.join(EVAL_DIR, "ppa"),
                    help="ppa_combine.py's output: MPKBr CSV destination, total / tradeoff data")
    ap.add_argument("--mpkbr-csv", default=None,
                    help="MPKBr CSV (default <ppa>/mpkbr_<mode>.csv; 'none': not written)")
    ap.add_argument("--format", default="png", choices=["png", "pdf", "svg"])
    a = ap.parse_args()
    bpc = os.path.join(a.cbp, "scripts", "bp-comparison")
    a.ledger = a.ledger or os.path.join(bpc, "results", "results_traces_csv_long.csv")
    a.selection = a.selection or os.path.join(bpc, "analysis", "traces_csv_long", "rtl_compare",
                                              "rtl_selection.csv")

    paths = a.csv or [p for p in (os.path.join(EVAL_DIR, f"{q}_results.csv") for q in STYLE)
                      if os.path.exists(p)]
    if not paths:
        sys.exit(f"no <predictor>_results.csv in {EVAL_DIR}; run evaluate.py first or pass --csv")
    bad = [p for p in paths if not os.path.exists(p)]
    if bad:
        sys.exit(f"no such CSV: {' '.join(bad)}")
    rows = load(paths)
    if a.max_kb is not None:
        rows = {k: r for k, r in rows.items() if k[1] <= a.max_kb}
    if not rows:
        sys.exit(f"no usable rows in {' '.join(paths)}"
                 + (f" up to {a.max_kb:g} KB" if a.max_kb is not None else ""))
    have = sorted({t for _, _, t in rows})
    traces = [t for t in a.traces if t in have]
    if len(traces) < len(a.traces):
        print(f"WARNING no rows for {' '.join(t for t in a.traces if t not in have)} "
              f"(have: {' '.join(have)})")
    if not traces:
        sys.exit("none of the requested traces is in the CSVs")
    os.makedirs(a.out, exist_ok=True)

    acc, recs = mpkbr_series(rows, traces, a)
    mp_csv = a.mpkbr_csv or (None if a.max_kb is not None
                             else os.path.join(a.ppa, f"mpkbr_{a.mpkbr}.csv"))
    if mp_csv and mp_csv != "none" and recs:
        write_mpkbr_csv(mp_csv, recs)
    elif a.max_kb is not None and not a.mpkbr_csv:
        print("  MPKBr CSV not written (--max-kb: it would miss budgets; give --mpkbr-csv)")
    ppa = ppa_series(rows, traces)
    energy = energy_series(rows, traces)
    lat, cycles = latency_series(rows, traces)
    seq = seq_share(rows, traces)

    y_mpkbr = ("MPKBr, CBP5 steady" if a.mpkbr == "steady" else "MPKBr, RTL pass 1 (cold)")
    for t in traces:
        table(f"{y_mpkbr} {t}", acc[t], ".3f")
    for t in traces:
        table(f"Logic energy {t}: pJ / cond. branch", energy[t], ".4f")
    for col, _, title, ylabel, per_trace, fmt in PPA:
        table(f"{title}: {ylabel}" + (" (mean over traces)" if per_trace else ""), ppa[col], fmt)
    table("Sequential share of the logic energy [%] (mean over traces)", seq, ".1f")
    print("Prediction latency [cycles]")
    for p, (label, _, _) in STYLE.items():
        if cycles.get(p):
            print(f"  {label:18s} " + "  ".join(f"{kb:g}AU={n}" for kb, n in sorted(cycles[p].items())))
    table("Minimum clock period (critical path) [ps]", lat["period"], ".1f")
    table("Prediction latency at that period [ns]", lat["lat_ns"], ".3f")
    table("Prediction-path delay, last stage [ps]", lat["pred_ps"], ".1f")
    table("Busy cycles / cond. branch (mean over traces)", lat["busy"], ".3f")

    clk = next((num(r.get("clk_ns")) for r in rows.values() if num(r.get("clk_ns"))), None)
    acc_panels = [(t, f"{y_mpkbr} [misp. / 1K cond. br.]" if i == 0 else None, acc[t])
                  for i, t in enumerate(traces)]
    energy_panels = [(t, Y_ENERGY if i == 0 else None, energy[t]) for i, t in enumerate(traces)]
    ppa_panels = [(title, ylabel, ppa[col]) for col, _, title, ylabel, _, _ in PPA]
    lat_panels = [("Min. clock period", "Critical path [ps]", lat["period"]),
                  ("Prediction latency", "At its own period [ns]", lat["lat_ns"]),
                  ("Prediction path", "Last stage [ps]", lat["pred_ps"]),
                  ("Occupancy", "Busy cycles / cond. branch", lat["busy"])]
    lat_ref = {0: (1000.0 * clk, f"target {clk:g} ns")} if clk else None
    apl_logic = logic_apl_panels(ppa, lat)          # before SRAM modelling

    # logic + SRAM, once ppa_combine.py has run
    comb = load_combined(a.ppa, a.max_kb)
    tot_panels = apl_total = apl_bands = None
    if comb and comb[0]:
        base, sens = comb
        tot, bands = combined_series(base, sens)
        r0 = next(r for by in base.values() for r in by.values())
        note = (f"logic + SRAM, baseline {r0.get('scenario')}, {num(r0.get('cpb')):g} host cycles per "
                f"cond. branch; MPKBr: {r0.get('mpkbr_src') or 'see ppa_combine.py'}")
        print(f"Logic + SRAM ({note})")
        table("Total energy [pJ / cond. branch] (mean over traces)", tot["energy"], ".3f")
        table("  ... low end over the corners", {p: [(kb, lo) for kb, lo, _ in v]
                                                 for p, v in bands["energy"].items()}, ".3f")
        table("  ... high end over the corners", {p: [(kb, hi) for kb, _, hi in v]
                                                  for p, v in bands["energy"].items()}, ".3f")
        table("Total area [µm²]", tot["area"], ".0f")
        table("SRAM share of the energy [%]", tot["sram_share"], ".1f")
        table("Cycle with the SRAM (CACTI access time, pessimistic) [ps]", tot["period"], ".1f")
        tot_panels = [("Logic + SRAM energy", "Energy [pJ / cond. branch]", tot["energy"]),
                      ("Logic + SRAM area", "Area [µm²]", tot["area"]),
                      ("SRAM share of the energy", "Share [%]", tot["sram_share"]),
                      ("Cycle with the SRAM", "Logic + SRAM access [ps]", tot["period"])]
        tot_bands = {0: bands["energy"], 1: bands["area"]}
        tot_ref = {3: (1000.0 * clk, f"target {clk:g} ns")} if clk else None
        apl_total, apl_bands = total_apl(base, bands, ppa, cycles, clk)
        table("Logic + SRAM power [µW] (logic power + SRAM dynamic over the busy cycles "
              "+ SRAM leakage)", apl_total[1][2], ".2f")
        table("Prediction latency with the SRAM (CACTI access time, pessimistic) [ns]",
              apl_total[2][2], ".3f")
    else:
        print(f"  no {os.path.join(a.ppa, 'ppa_table.csv')} yet: run ppa_combine.py, then this "
              f"script again, for the logic + SRAM figures")

    for sub, excl in PLOT_SETS:
        out = os.path.join(a.out, sub) if sub else a.out
        os.makedirs(out, exist_ok=True)
        left_out = ", ".join(STYLE[p][0] for p in excl if p in STYLE)
        print(f"Figures -> {out}" + (f" (without {left_out})" if left_out else " (all predictors)"))
        for log, scale in ((True, "log"), (False, "linear")):
            figure(without(acc_panels, excl), log, a.sharey,
                   os.path.join(out, f"mpkbr_{scale}.{a.format}"))
            figure(without(energy_panels, excl), log, a.sharey,
                   os.path.join(out, f"energy_{scale}.{a.format}"))
            figure(without(ppa_panels, excl), log, False,
                   os.path.join(out, f"ppa_{scale}.{a.format}"))
            figure(without(lat_panels, excl), log, False,
                   os.path.join(out, f"latency_{scale}.{a.format}"), lat_ref)
            before = without(apl_logic, excl)
            after = without(apl_total, excl) if apl_total else None
            after_bands = without_bands(apl_bands, excl) if apl_total and a.apl_bands else None
            ylims = (common_ylims([before, after], log, [after_bands])
                     if after and a.apl_sharey else None)
            figure(before, log, False,
                   os.path.join(out, f"area_power_latency_logic_{scale}.{a.format}"),
                   ylims=ylims)
            if after:
                figure(after, log, False,
                       os.path.join(out, f"area_power_latency_total_{scale}.{a.format}"),
                       bands=after_bands, ylims=ylims)
            if tot_panels:
                figure(without(tot_panels, excl), log, False,
                       os.path.join(out, f"total_{scale}.{a.format}"),
                       tot_ref, without_bands(tot_bands, excl))
                tradeoff_figure({p: v for p, v in base.items() if p not in excl}, log,
                                os.path.join(out, f"tradeoff_{scale}.{a.format}"), note)


if __name__ == "__main__":
    main()