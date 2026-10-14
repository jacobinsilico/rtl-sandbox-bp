#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Accuracy and logic PPA of the four predictors vs storage budget, from the
#   results evaluate.py / campaign.py wrote (one row per predictor x budget x
#   trace). Each figure has one legend, shared by all its columns.
#
#     mpkbr_<scale>   MPKBr, one column per trace (no bands)
#     energy_<scale>  logic energy per conditional branch, one column per
#                     trace, from that trace's passed gate-level run:
#                       E = p_total_w * cycles * T_clk / gls_cond_br
#                     cycles = the bench's TB_CYCLES (first present column of
#                     CYCLES_COLS), T_clk = --period-ns or the first present
#                     column of PERIOD_COLS. Energy, not power, because the
#                     cores take different cycles per branch.
#     ppa_<scale>     logic area, logic power, SRAM reads and SRAM writes per
#                     conditional branch, one column each. Area is per
#                     budget; power and SRAM accesses are the mean over the
#                     traces, and a budget is plotted only when every trace
#                     has a passed gate-level value
#
#   Every figure is written twice: <scale> = log (log y axis) and linear (y
#   axis from 0). x axis: storage budget in area units, 1 AU = 1 KB.
#
#   Logic only: the SRAMs are outside the synthesized core, so neither the
#   power nor the energy includes them. MPKBr is the RTL check's count; the
#   number of conditional branches it covered is read from each RTL record
#   (eval/<predictor>/<budget>/rtl_<trace>.json), and a WARNING names the
#   runs when one trace was measured over different windows. The same check
#   runs on the gate-level window (gls_cond_br) of power, energy and SRAM
#   accesses. The plotted values are printed as tables.
#
# Usage (after `source sourceme.sh`):
#   /usr/bin/python3 projects/branch-prediction/scripts/plot_ppa.py [options]
#     --csv CSV ...    result CSVs (default: eval/<predictor>_results.csv of
#                      every predictor that has one)
#     --traces T ...   MPKBr and energy columns, and the traces averaged for
#                      power and SRAM accesses (default: fdd_su_v1_0
#                      tdd_dl_mu_v1_0 tdd_ul_mu_v1_0)
#     --sharey         one y range across the MPKBr columns, and one across
#                      the energy columns
#     --period-ns NS   clock period of the gate-level runs for the energy
#                      (default: read from each row, see PERIOD_COLS)
#     --out DIR        default eval/plots/
#     --format FMT     png | pdf | svg (default png)
#
#   Examples:
#     /usr/bin/python3 projects/branch-prediction/scripts/plot_ppa.py
#     /usr/bin/python3 projects/branch-prediction/scripts/plot_ppa.py --format pdf --sharey
#     /usr/bin/python3 projects/branch-prediction/scripts/plot_ppa.py --period-ns 1.0
#     /usr/bin/python3 projects/branch-prediction/scripts/plot_ppa.py --csv projects/branch-prediction/eval/campaign_full.csv
#
# Output:
#   <out>/mpkbr_log.<fmt>    <out>/mpkbr_linear.<fmt>
#   <out>/energy_log.<fmt>   <out>/energy_linear.<fmt>
#   <out>/ppa_log.<fmt>      <out>/ppa_linear.<fmt>
# -----------------------------------------------------------------------------

import argparse
import csv
import json
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

TRACES = ["fdd_su_v1_0", "tdd_dl_mu_v1_0", "tdd_ul_mu_v1_0"]

# plotting and legend order: predictor -> (legend label, colour, marker)
STYLE = {
    "gshare":            ("Gshare",            "#E69F00", "o"),
    "g_perceptron":      ("Perceptron",        "#56B4E9", "s"),
    "hashed_perceptron": ("Hashed Perceptron", "#009E73", "^"),
    "tage_cb":           ("TAGE",              "#D55E00", "D"),
}

X_LABEL = "Storage budget (area units)"
Y_MPKBR = "MPKBr (misp. / 1K cond. branches)"
Y_ENERGY = "Logic energy [pJ / cond. branch]"

# PPA columns: (CSV column, scale, column title, y label, mean over traces, table format)
PPA = [
    ("area_um2",         1.0, "Logic area",  "Area [µm²]",            False, ".1f"),
    ("p_total_w",        1e6, "Logic power", "Power [µW]",            True,  ".2f"),
    ("sram_rd_per_cond", 1.0, "SRAM reads",  "Reads / cond. branch",  True,  ".3f"),
    ("sram_wr_per_cond", 1.0, "SRAM writes", "Writes / cond. branch", True,  ".3f"),
]

# Energy inputs: the first column present in a row is used. Rename here if
# evaluate.py writes them under other names.
CYCLES_COLS = ("gls_cycles", "tb_cycles", "cycles")   # gate-level TB_CYCLES
PERIOD_COLS = ("clk_period_ns", "clk_ns")             # gate-level clock period


# =============================================================================
# Data
# =============================================================================

def num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def first_num(r, cols):
    """The value of the first of cols that row r holds as a number, else None."""
    for c in cols:
        v = num(r.get(c))
        if v is not None:
            return v
    return None


def passed(v):
    return str(v).strip().lower() in ("true", "1")


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


def rtl_cond(r):
    """Conditional branches the RTL check of this row counted, from its record."""
    path = os.path.join(EVAL_DIR, r["predictor"], r.get("budget", ""), f"rtl_{r['trace']}.json")
    try:
        with open(path) as f:
            return json.load(f)["metrics"]["cond_br"]
    except (OSError, ValueError, KeyError, TypeError):
        return None


def check_windows(what, windows):
    """windows: {trace: {cond. branches: [run, ...]}}. Warns when one trace was
    measured over different windows, so its points are not comparable, and
    names the runs that differ from the most common window."""
    for t, by in windows.items():
        if not by:
            continue
        common = max(by, key=lambda c: len(by[c]))
        print(f"  {what} of {t}: {common} cond. branches per run")
        odd = [f"{run} ({c})" for c, runs in sorted(by.items()) if c != common for run in runs]
        if odd:
            print(f"WARNING {what} of {t}: different window for {' '.join(odd)}")


def mpkbr_series(rows, traces):
    """{trace: {predictor: [(kb, mpkbr)]}}"""
    out = {t: defaultdict(list) for t in traces}
    windows = {t: defaultdict(list) for t in traces}
    for (p, kb, t), r in sorted(rows.items()):
        m = num(r.get("mpkbr"))
        if t not in out or m is None:
            continue                                # other trace, or no passed RTL check
        out[t][p].append((kb, m))
        windows[t][str(rtl_cond(r) or "?")].append(f"{p}:{r.get('budget')}")
    check_windows("MPKBr", windows)
    return out


def energy_series(rows, traces, period_ns):
    """{trace: {predictor: [(kb, pJ per cond. branch)]}}, logic only, from each
    trace's passed gate-level run (its window is checked in ppa_series)."""
    out = {t: defaultdict(list) for t in traces}
    missing = defaultdict(list)
    for (p, kb, t), r in sorted(rows.items()):
        if t not in out or not passed(r.get("gls_pass")):
            continue                                # other trace, or no passed gate-level run
        label = r.get("budget") or f"{kb:g}KB"
        p_w = num(r.get("p_total_w"))
        cycles = first_num(r, CYCLES_COLS)
        cond = num(r.get("gls_cond_br"))
        t_ns = period_ns if period_ns is not None else first_num(r, PERIOD_COLS)
        if None in (p_w, cycles, cond, t_ns) or cond <= 0:
            missing[t].append(f"{p}:{label}")
            continue
        # W * ns = 1e-9 J = 1e3 pJ
        out[t][p].append((kb, 1e3 * p_w * cycles * t_ns / cond))
    for t, runs in missing.items():
        print(f"  skip energy of {t}: no p_total_w, cycles ({'/'.join(CYCLES_COLS)}), "
              f"gls_cond_br or clock period ({'/'.join(PERIOD_COLS)} or --period-ns) "
              f"for {' '.join(runs)}")
    return out


def ppa_series(rows, traces):
    """{column: {predictor: [(kb, value)]}}: area per budget, the rest the
    mean over the traces."""
    out = {col: defaultdict(list) for col, *_ in PPA}
    windows = {t: defaultdict(list) for t in traces}
    for p, kb in sorted({(p, kb) for p, kb, _ in rows}):
        runs = [r for (q, k, _), r in rows.items() if (q, k) == (p, kb)]
        label = runs[0].get("budget") or f"{kb:g}KB"
        ok = {t: rows[(p, kb, t)] for t in traces
              if (p, kb, t) in rows and passed(rows[(p, kb, t)].get("gls_pass"))}
        for t, r in ok.items():
            windows[t][r.get("gls_cond_br") or "?"].append(f"{p}:{label}")
        for col, scale, _title, _ylabel, per_trace, _fmt in PPA:
            if not per_trace:
                v = next((num(r.get(col)) for r in runs if num(r.get(col)) is not None), None)
                if v is not None:
                    out[col][p].append((kb, scale * v))
                continue
            vals = [num(ok[t].get(col)) if t in ok else None for t in traces]
            if None in vals:
                miss = [t for t, v in zip(traces, vals) if v is None]
                print(f"  skip {col} {p} {label}: no passed gate-level value for {' '.join(miss)}")
                continue
            out[col][p].append((kb, scale * sum(vals) / len(vals)))
    check_windows("power / energy / SRAM accesses", windows)
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


def figure(panels, log, sharey, path):
    """panels: [(column title, y label or None, {predictor: [(kb, y)]})]."""
    budgets = sorted({x for _, _, s in panels for pts in s.values() for x, _ in pts})
    if not budgets:
        print(f"  no data, skipped {path}")
        return
    h = 3.6
    fig, axes = plt.subplots(1, len(panels), figsize=(3.9 * len(panels), h),
                             squeeze=False, sharey=sharey)
    drawn = set()
    for ax, (_, _, series) in zip(axes[0], panels):        # all data first (shared y range)
        drawn |= draw(ax, series, log)
    for ax, (title, ylabel, _) in zip(axes[0], panels):
        budget_axis(ax, budgets)
        log_y(ax) if log else linear_y(ax)
        ax.set_title(title, fontsize=11)
        if ylabel:
            ax.set_ylabel(ylabel)
    fig.tight_layout(rect=(0, 0.62 / h, 1, 1))            # bottom 0.62 in: x label + legend
    fig.text(0.5, 0.45 / h, X_LABEL, ha="center", va="center", fontsize=11)
    handles = [Line2D([], [], color=c, marker=m, lw=1.8, ms=6, label=lab)
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
    ap = argparse.ArgumentParser(description="MPKBr, energy and logic PPA plots (see the header).")
    ap.add_argument("--csv", nargs="+", default=None)
    ap.add_argument("--traces", nargs="+", default=TRACES)
    ap.add_argument("--sharey", action="store_true",
                    help="one y range across the MPKBr columns, and one across the energy columns")
    ap.add_argument("--period-ns", type=float, default=None,
                    help="clock period of the gate-level runs (default: from each row)")
    ap.add_argument("--out", default=os.path.join(EVAL_DIR, "plots"))
    ap.add_argument("--format", default="png", choices=["png", "pdf", "svg"])
    a = ap.parse_args()
    if a.period_ns is not None and a.period_ns <= 0:
        sys.exit("--period-ns must be positive")

    paths = a.csv or [p for p in (os.path.join(EVAL_DIR, f"{q}_results.csv") for q in STYLE)
                      if os.path.exists(p)]
    if not paths:
        sys.exit(f"no <predictor>_results.csv in {EVAL_DIR}; run evaluate.py first or pass --csv")
    bad = [p for p in paths if not os.path.exists(p)]
    if bad:
        sys.exit(f"no such CSV: {' '.join(bad)}")
    rows = load(paths)
    if not rows:
        sys.exit(f"no usable rows in {' '.join(paths)}")
    have = sorted({t for _, _, t in rows})
    traces = [t for t in a.traces if t in have]
    if len(traces) < len(a.traces):
        print(f"WARNING no rows for {' '.join(t for t in a.traces if t not in have)} "
              f"(have: {' '.join(have)})")
    if not traces:
        sys.exit("none of the requested traces is in the CSVs")
    os.makedirs(a.out, exist_ok=True)

    acc = mpkbr_series(rows, traces)
    ppa = ppa_series(rows, traces)
    energy = energy_series(rows, traces, a.period_ns)
    for t in traces:
        table(f"MPKBr {t}", acc[t], ".3f")
    for t in traces:
        table(f"Logic energy {t}: pJ / cond. branch", energy[t], ".4f")
    for col, _, title, ylabel, per_trace, fmt in PPA:
        table(f"{title}: {ylabel}" + (" (mean over traces)" if per_trace else ""), ppa[col], fmt)

    acc_panels = [(t, Y_MPKBR if i == 0 else None, acc[t]) for i, t in enumerate(traces)]
    energy_panels = [(t, Y_ENERGY if i == 0 else None, energy[t]) for i, t in enumerate(traces)]
    ppa_panels = [(title, ylabel, ppa[col]) for col, _, title, ylabel, _, _ in PPA]
    for log, scale in ((True, "log"), (False, "linear")):
        figure(acc_panels, log, a.sharey, os.path.join(a.out, f"mpkbr_{scale}.{a.format}"))
        figure(energy_panels, log, a.sharey, os.path.join(a.out, f"energy_{scale}.{a.format}"))
        figure(ppa_panels, log, False, os.path.join(a.out, f"ppa_{scale}.{a.format}"))


if __name__ == "__main__":
    main()