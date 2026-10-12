#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   PPA plots of a campaign.py run, from eval/campaign_<mode>.csv (one row per
#   predictor x budget x trace). Per predictor and budget, MPKBr is the
#   geometric mean over the traces (min-max as a band), every other metric
#   the mean. Logic only: the SRAMs are outside the synthesized core.
#
#     accuracy.png       MPKBr vs budget
#     area.png           logic area and flip-flops vs budget
#     timing.png         critical delay and prediction-path delay vs budget
#     energy.png         logic energy per conditional branch vs budget
#     energy_split.png   sequential / combinational / clock share of the logic
#                        power (needs the p_*_w columns; skipped otherwise)
#     pareto.png         MPKBr vs logic energy and vs logic area
#     sram_access.png    SRAM reads / writes per conditional branch
#     overview.png       the main panels on one page
#
# Usage:
#   /usr/bin/python3 projects/branch-prediction/scripts/plot_campaign.py [--csv CSV] [--out DIR]
#     --csv   default eval/campaign_full.csv
#     --out   default eval/plots/
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
from matplotlib.ticker import FixedLocator, FuncFormatter, LogLocator, NullFormatter, NullLocator  # noqa: E402

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
EVAL_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "eval")

ORDER = ["gshare", "g_perceptron", "hashed_perceptron", "tage_cb"]
LABEL = {"gshare": "gshare", "g_perceptron": "global perceptron",
         "hashed_perceptron": "hashed perceptron", "tage_cb": "TAGE (cookbook)"}
COLOR = {"gshare": "#E69F00", "g_perceptron": "#56B4E9",          # Okabe-Ito
         "hashed_perceptron": "#009E73", "tage_cb": "#D55E00"}
MARKER = {"gshare": "o", "g_perceptron": "s", "hashed_perceptron": "^", "tage_cb": "D"}
MEAN_COLS = ["area_um2", "flops", "crit_delay_ps", "pred_path_ps", "e_logic_pj_per_cond",
             "cycles_per_cond", "sram_rd_per_cond", "sram_wr_per_cond", "p_total_w",
             "p_sequential_w", "p_combinational_w", "p_clock_w", "clk_ns", "gls_cond_br"]


def num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def load(path):
    """-> {pred: {kb: {metric: value, 'mpkbr': gm, 'mpkbr_lo': min, 'mpkbr_hi': max}}}"""
    acc = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            p, kb = r.get("predictor"), num(r.get("budget_kb"))
            if p is None or kb is None:
                continue
            for c in ["mpkbr"] + MEAN_COLS:
                v = num(r.get(c))
                if v is not None:
                    acc[p][kb][c].append(v)
    out = defaultdict(dict)
    for p, budgets in acc.items():
        for kb, cols in budgets.items():
            d = {}
            m = [v for v in cols.get("mpkbr", []) if v > 0]
            if m:
                d["mpkbr"] = math.exp(sum(map(math.log, m)) / len(m))
                d["mpkbr_lo"], d["mpkbr_hi"] = min(m), max(m)
            for c in MEAN_COLS:
                if cols.get(c):
                    d[c] = sum(cols[c]) / len(cols[c])
            out[p][kb] = d
    return out


def series(data, pred, key):
    pts = sorted((kb, d[key]) for kb, d in data.get(pred, {}).items() if d.get(key) is not None)
    return [p[0] for p in pts], [p[1] for p in pts]


def nice_log(axis):
    """Log axis labeled at 1-2-5 steps in plain numbers."""
    axis.set_major_locator(LogLocator(base=10, subs=(1.0, 2.0, 5.0)))
    axis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
    axis.set_minor_formatter(NullFormatter())


def budget_axis(ax, budgets):
    ax.set_xscale("log", base=2)
    ax.xaxis.set_major_locator(FixedLocator(budgets))
    ax.xaxis.set_minor_locator(NullLocator())
    ax.xaxis.set_major_formatter(FuncFormatter(lambda x, _: f"{x:g} KB"))
    ax.set_xlabel("storage budget")
    ax.grid(True, which="major", alpha=0.3)


def plot_metric(ax, data, key, ylabel, budgets, log=False, band=None, title=None):
    for p in ORDER:
        x, y = series(data, p, key)
        if not x:
            continue
        ax.plot(x, y, color=COLOR[p], marker=MARKER[p], label=LABEL[p], lw=1.8, ms=6)
        if band:
            _, lo = series(data, p, band[0])
            _, hi = series(data, p, band[1])
            if len(lo) == len(x) == len(hi):
                ax.fill_between(x, lo, hi, color=COLOR[p], alpha=0.15, lw=0)
    budget_axis(ax, budgets)
    if log:
        ax.set_yscale("log")
        nice_log(ax.yaxis)
        ax.grid(True, which="minor", alpha=0.1)
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title, fontsize=11)


def freq_lines(ax):
    for ps, lab in ((1000, "1 GHz"), (500, "2 GHz")):
        ax.axhline(ps, color="0.4", ls=":", lw=1)
        ax.text(ax.get_xlim()[0] * 1.05, ps, lab, va="bottom", fontsize=8, color="0.3")


def plot_pareto(ax, data, xkey, xlabel):
    for p in ORDER:
        pts = sorted((kb, d.get(xkey), d.get("mpkbr")) for kb, d in data.get(p, {}).items()
                     if d.get(xkey) is not None and d.get("mpkbr") is not None)
        if not pts:
            continue
        ax.plot([q[1] for q in pts], [q[2] for q in pts], color=COLOR[p], marker=MARKER[p],
                label=LABEL[p], lw=1.2, ms=6)
        for kb, x, y in pts:
            ax.annotate(f"{kb:g}K", (x, y), textcoords="offset points", xytext=(4, 3), fontsize=7,
                        color=COLOR[p])
    ax.set_xscale("log")
    ax.set_yscale("log")
    nice_log(ax.xaxis)
    nice_log(ax.yaxis)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("MPKBr (geomean over traces)")
    ax.grid(True, which="both", alpha=0.2)


def note(data):
    clk = [d.get("clk_ns") for b in data.values() for d in b.values() if d.get("clk_ns")]
    gls = [d.get("gls_cond_br") for b in data.values() for d in b.values() if d.get("gls_cond_br")]
    s = "logic only (SRAMs excluded)"
    if clk:
        s += f", clock {clk[0]:g} ns"
    if gls:
        s += f", power from ~{sum(gls) / len(gls) / 1000:.0f}k gate-level cond. branches per trace"
    return s


def save(fig, out, name):
    path = os.path.join(out, name)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("wrote", path)


def main():
    ap = argparse.ArgumentParser(description="PPA plots of a campaign run (see the header).")
    ap.add_argument("--csv", default=os.path.join(EVAL_DIR, "campaign_full.csv"))
    ap.add_argument("--out", default=os.path.join(EVAL_DIR, "plots"))
    a = ap.parse_args()
    if not os.path.exists(a.csv):
        sys.exit(f"no such CSV: {a.csv}")
    data = load(a.csv)
    if not data:
        sys.exit(f"no usable rows in {a.csv}")
    os.makedirs(a.out, exist_ok=True)
    budgets = sorted({kb for b in data.values() for kb in b})
    sub = note(data)

    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    plot_metric(ax, data, "mpkbr", "MPKBr (geomean, band = min-max over traces)", budgets, log=True,
                band=("mpkbr_lo", "mpkbr_hi"), title="Accuracy")
    ax.legend(fontsize=9)
    save(fig, a.out, "accuracy.png")

    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    plot_metric(axs[0], data, "area_um2", "logic area [µm²]", budgets, log=True, title="Logic area")
    plot_metric(axs[1], data, "flops", "flip-flops", budgets, log=True, title="Flip-flops")
    axs[0].legend(fontsize=9)
    fig.suptitle(sub, fontsize=9, y=0.02)
    save(fig, a.out, "area.png")

    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    plot_metric(axs[0], data, "crit_delay_ps", "critical delay [ps]", budgets,
                title="Critical path (period - worst slack)")
    plot_metric(axs[1], data, "pred_path_ps", "arrival at pred_resp_taken_o [ps]", budgets,
                title="Prediction path")
    for ax in axs:
        freq_lines(ax)
    axs[0].legend(fontsize=9)
    save(fig, a.out, "timing.png")

    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    plot_metric(axs[0], data, "e_logic_pj_per_cond", "logic energy [pJ / cond. branch]", budgets, log=True,
                title="Logic energy per branch")
    plot_metric(axs[1], data, "cycles_per_cond", "cycles / cond. branch", budgets,
                title="Core cycles per branch")
    axs[0].legend(fontsize=9)
    fig.suptitle(sub, fontsize=9, y=0.02)
    save(fig, a.out, "energy.png")

    have_split = any(d.get("p_sequential_w") is not None for b in data.values() for d in b.values())
    if have_split:
        fig, ax = plt.subplots(figsize=(11, 4.2))
        labels, seq, comb, clk = [], [], [], []
        for p in ORDER:
            for kb in sorted(data.get(p, {})):
                d = data[p][kb]
                tot = d.get("p_total_w")
                if not tot:
                    continue
                labels.append(f"{LABEL[p].split()[0]}\n{kb:g}K")
                seq.append(100 * (d.get("p_sequential_w") or 0) / tot)
                comb.append(100 * (d.get("p_combinational_w") or 0) / tot)
                clk.append(100 * (d.get("p_clock_w") or 0) / tot)
        x = range(len(labels))
        ax.bar(x, seq, label="sequential", color="#0072B2")
        ax.bar(x, comb, bottom=seq, label="combinational", color="#CC79A7")
        ax.bar(x, clk, bottom=[s + c for s, c in zip(seq, comb)], label="clock", color="#F0E442")
        ax.set_xticks(list(x))
        ax.set_xticklabels(labels, fontsize=7)
        ax.set_ylabel("share of logic power [%]")
        ax.set_ylim(0, 105)
        ax.legend(fontsize=9, ncol=3)
        ax.set_title("Logic power split (a large sequential share = clocking, i.e. no clock gating)",
                     fontsize=11)
        ax.grid(True, axis="y", alpha=0.3)
        save(fig, a.out, "energy_split.png")
    else:
        print("skipped energy_split.png (no p_sequential_w column)")

    fig, axs = plt.subplots(1, 2, figsize=(11, 4.6))
    plot_pareto(axs[0], data, "e_logic_pj_per_cond", "logic energy [pJ / cond. branch]")
    plot_pareto(axs[1], data, "area_um2", "logic area [µm²]")
    axs[0].set_title("Accuracy vs logic energy (lower-left is better)", fontsize=11)
    axs[1].set_title("Accuracy vs logic area (lower-left is better)", fontsize=11)
    axs[0].legend(fontsize=9)
    fig.suptitle(sub, fontsize=9, y=0.01)
    save(fig, a.out, "pareto.png")

    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    plot_metric(axs[0], data, "sram_rd_per_cond", "SRAM reads / cond. branch", budgets,
                title="SRAM reads (all arrays)")
    plot_metric(axs[1], data, "sram_wr_per_cond", "SRAM writes / cond. branch", budgets,
                title="SRAM writes (all arrays)")
    axs[0].legend(fontsize=9)
    fig.suptitle("gate-level window; summed over arrays of different sizes (not yet usable for SRAM energy)",
                 fontsize=9, y=0.02)
    save(fig, a.out, "sram_access.png")

    fig, axs = plt.subplots(2, 3, figsize=(16, 9))
    plot_metric(axs[0][0], data, "mpkbr", "MPKBr (geomean)", budgets, log=True,
                band=("mpkbr_lo", "mpkbr_hi"), title="Accuracy")
    plot_metric(axs[0][1], data, "area_um2", "logic area [µm²]", budgets, log=True, title="Logic area")
    plot_metric(axs[0][2], data, "crit_delay_ps", "critical delay [ps]", budgets, title="Critical path")
    freq_lines(axs[0][2])
    plot_metric(axs[1][0], data, "e_logic_pj_per_cond", "pJ / cond. branch", budgets, log=True,
                title="Logic energy per branch")
    plot_pareto(axs[1][1], data, "e_logic_pj_per_cond", "logic energy [pJ / cond. branch]")
    axs[1][1].set_title("Accuracy vs logic energy", fontsize=11)
    plot_pareto(axs[1][2], data, "area_um2", "logic area [µm²]")
    axs[1][2].set_title("Accuracy vs logic area", fontsize=11)
    axs[0][0].legend(fontsize=9)
    fig.suptitle("Branch predictor PPA, " + sub, fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    save(fig, a.out, "overview.png")


if __name__ == "__main__":
    main()