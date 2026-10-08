#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Plots the results evaluate.py wrote (eval/<predictor>_results.csv) for
#   every predictor that has them, one line per predictor:
#
#     area_vs_budget     logic area of the synthesized core vs the predictor
#                        budget (x in AU, 1 AU = 1 KB of predictor storage)
#     power_vs_budget    logic power from the gate-level activity vs budget,
#                        one column per trace
#
#   Logic only: the SRAM tables are outside the synthesized core, so their
#   area and energy (CACTI) are not in these numbers. Power points come only
#   from gate-level runs that passed; area points only from finished
#   synthesis runs. The values plotted are also printed as a table.
#
# Usage (after `source sourceme.sh`):
#   /usr/bin/python3 projects/branch-prediction/scripts/plot_eval.py
#   /usr/bin/python3 projects/branch-prediction/scripts/plot_eval.py --predictors gshare tage_cb
#   /usr/bin/python3 projects/branch-prediction/scripts/plot_eval.py --out /tmp/plots --format pdf
#
# Output:
#   projects/<project>/eval/plots/area_vs_budget.<fmt>
#   projects/<project>/eval/plots/power_vs_budget.<fmt>
# -----------------------------------------------------------------------------

import argparse
import csv
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJ_DIR = os.path.dirname(SCRIPT_DIR)
EVAL_DIR = os.path.join(PROJ_DIR, "eval")

# plotting order, label, colour and marker per predictor
STYLE = {
    "gshare":            ("gshare",            "tab:gray",   "o"),
    "g_perceptron":      ("global perceptron", "tab:blue",   "s"),
    "hashed_perceptron": ("hashed perceptron", "tab:green",  "^"),
    "tage_cb":           ("TAGE (cookbook)",   "tab:red",    "D"),
}


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def trace_title(trace):
    return re.sub(r"_v\d+_\d+$", "", trace)          # fdd_su_v1_0 -> fdd_su


def load(predictors):
    """{predictor: rows} for every predictor with a results CSV."""
    data = {}
    for p in predictors:
        path = os.path.join(EVAL_DIR, f"{p}_results.csv")
        if not os.path.exists(path):
            continue
        with open(path, newline="") as f:
            data[p] = list(csv.DictReader(f))
    return data


def budget_axis(ax, budgets):
    ax.set_xscale("log", base=2)
    ax.set_xticks(budgets)
    ax.set_xticklabels([f"{b:g}" for b in budgets])
    ax.minorticks_off()
    ax.grid(True, which="major", alpha=0.3)
    ax.set_xlabel("Predictor budget [AU]  (1 AU = 1 KB)")


def area_points(rows):
    """[(budget_kb, area_um2)], one per budget."""
    pts = {}
    for r in rows:
        kb, a = num(r.get("budget_kb")), num(r.get("area_um2"))
        if kb is not None and a is not None:
            pts[kb] = a
    return sorted(pts.items())


def power_points(rows, trace):
    """[(budget_kb, power_uW)] of one trace, passed gate-level runs only."""
    pts = []
    for r in rows:
        if r.get("trace") != trace:
            continue
        kb, p = num(r.get("budget_kb")), num(r.get("p_total_w"))
        if kb is None or p is None:
            continue
        if r.get("gls_pass") != "True":
            print(f"  skip power {r['predictor']} {r['budget']} {trace}: gate-level run did not pass")
            continue
        pts.append((kb, p * 1e6))
    return sorted(pts)


def main():
    ap = argparse.ArgumentParser(description="Area and power plots from evaluate.py results.")
    ap.add_argument("--predictors", nargs="+", default=list(STYLE), choices=list(STYLE))
    ap.add_argument("--out", default=os.path.join(EVAL_DIR, "plots"))
    ap.add_argument("--format", default="png", choices=["png", "pdf", "svg"])
    args = ap.parse_args()

    data = load(args.predictors)
    if not data:
        sys.exit(f"no results CSV in {EVAL_DIR} for {' '.join(args.predictors)}")
    os.makedirs(args.out, exist_ok=True)
    budgets = sorted({num(r["budget_kb"]) for rows in data.values() for r in rows
                      if num(r.get("budget_kb")) is not None})
    traces = sorted({r["trace"] for rows in data.values() for r in rows if r.get("trace")})

    # ---- logic area vs budget ----
    fig, ax = plt.subplots(figsize=(6.0, 4.2))
    print("logic area [um2]")
    for p, rows in data.items():
        label, colour, marker = STYLE[p]
        pts = area_points(rows)
        if pts:
            ax.plot(*zip(*pts), marker=marker, color=colour, label=label)
            print(f"  {label:18s} " + "  ".join(f"{kb:g}AU={a:.1f}" for kb, a in pts))
    budget_axis(ax, budgets)
    ax.set_ylabel("Logic area [µm²]")
    ax.set_title("Predictor logic area (SRAM excluded)")
    ax.legend()
    fig.tight_layout()
    path = os.path.join(args.out, f"area_vs_budget.{args.format}")
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  -> {path}")

    # ---- logic power vs budget, one column per trace ----
    fig, axes = plt.subplots(1, len(traces), figsize=(4.2 * len(traces), 4.0),
                             sharey=True, squeeze=False)
    print("logic power [uW]")
    handles = {}
    for ax, trace in zip(axes[0], traces):
        for p, rows in data.items():
            label, colour, marker = STYLE[p]
            pts = power_points(rows, trace)
            if pts:
                (h,) = ax.plot(*zip(*pts), marker=marker, color=colour, label=label)
                handles[label] = h
                print(f"  {trace_title(trace):10s} {label:18s} "
                      + "  ".join(f"{kb:g}AU={pw:.2f}" for kb, pw in pts))
        budget_axis(ax, budgets)
        ax.set_title(trace_title(trace))
    axes[0][0].set_ylabel("Logic power [µW]")
    fig.suptitle("Predictor logic power (gate-level activity, SRAM excluded)")
    if handles:
        fig.legend(handles.values(), handles.keys(), loc="lower center",
                   ncol=len(handles), frameon=False)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    path = os.path.join(args.out, f"power_vs_budget.{args.format}")
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  -> {path}")


if __name__ == "__main__":
    main()