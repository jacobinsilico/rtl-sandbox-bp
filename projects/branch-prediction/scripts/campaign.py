#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   RTL evaluation campaign of the four predictors (gshare, global
#   perceptron, hashed perceptron, cookbook TAGE), each through evaluate.py:
#   RTL vs golden dump (bit-exact), synthesis, STA, gate-level simulation
#   with a capped VCD, and DPA.
#
#     smoke  the 1 KB budget only: checks the whole flow end to end fast
#     full   every budget with dumps (1 KB records of a smoke run with the
#            same settings are reused, not recomputed)
#
#   Before anything runs, a preflight checks the environment (sourceme.sh,
#   verilator / yosys / sta on PATH), that every predictor has dumps for the
#   budgets of the mode and no duplicate budget directories, and the free
#   disk space. The predictors run one after another in one process holding
#   the project's evaluation lock. A failing predictor does not stop the
#   others (--stop-on-fail does); within a predictor the pilot budget stops
#   it, as in evaluate.py.
#
#   Disk: as evaluate.py. Every gate-level run is capped at --gls-lines
#   branches, its VCD is deleted once its DPA passed, sim/ run directories
#   of passed steps are deleted, and a gls step refuses to start below
#   --min-free-gb free space. At most one VCD exists at a time.
#
# Output:
#   eval/<predictor>/<budget>/<step>.{log,json}, eval/<predictor>_results.csv
#   (evaluate.py), plus eval/campaign_<mode>.csv (all predictors) and a
#   summary table on stdout. Exit status 0 only if every step passed.
#
# Usage (after `source sourceme.sh` in the repository root):
#   /usr/bin/python3 projects/branch-prediction/scripts/campaign.py smoke [options]
#   /usr/bin/python3 projects/branch-prediction/scripts/campaign.py full  [options]
#     --predictors P ..   subset (default: gshare g_perceptron hashed_perceptron tage_cb)
#     --steps rtl syn ..  subset of steps (default: rtl syn sta gls dpa)
#     --rtl-lines N       branch lines per RTL check (default 0 = whole trace)
#     --gls-lines N       branch lines per gate-level run = VCD cap (default 20000)
#     --clk NS            clock period (default 3.0)
#     --io-delay-pct P    STA/DPA I/O delay, percent of the period (default 0)
#     --keep-vcd          keep the VCDs after DPA
#     --keep-sim          keep the sim/ run directories
#     --min-free-gb G     free space required before a gls step (default 5)
#     --stop-on-fail      stop at the first predictor that fails
#     --rerun             ignore finished steps
#     --dry-run           print the make commands only
#
#   Examples:
#     /usr/bin/python3 projects/branch-prediction/scripts/campaign.py smoke
#     /usr/bin/python3 projects/branch-prediction/scripts/campaign.py smoke --predictors tage_cb --steps rtl
#     /usr/bin/python3 projects/branch-prediction/scripts/campaign.py full
# -----------------------------------------------------------------------------

import argparse
import glob
import os
import shutil
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import evaluate as ev  # noqa: E402

ORDER = ["gshare", "g_perceptron", "hashed_perceptron", "tage_cb"]
MODES = {"smoke": ["1KB"], "full": None}       # None: every budget with dumps
TOOLS = ("verilator", "yosys", "sta")


def preflight(preds, budgets, opts):
    """-> list of problems that would make the campaign fail."""
    problems = []
    if not os.environ.get("REPO_HOME"):
        problems.append("REPO_HOME is not set: run `source sourceme.sh` in the repository root")
    for tool in TOOLS:
        if shutil.which(tool) is None:
            problems.append(f"'{tool}' is not on PATH")
    for p in preds:
        stim = os.path.join(ev.PROJ_DIR, "stim", p)
        labels = [os.path.basename(d).split("__", 1)[0]
                  for d in glob.glob(os.path.join(stim, "*__*"))
                  if glob.glob(os.path.join(d, "*.golden.txt"))]
        if not labels:
            problems.append(f"{p}: no golden dumps in {stim}")
            continue
        for lab in sorted(set(labels)):
            if labels.count(lab) > 1:
                problems.append(f"{p}: several dump directories for {lab} in {stim}")
        for lab in budgets or []:
            if lab not in labels:
                problems.append(f"{p}: no dumps for {lab} in {stim}")
    if "gls" in opts.steps and ev.free_gb(ev.PROJ_DIR) < opts.min_free_gb:
        problems.append(f"only {ev.free_gb(ev.PROJ_DIR):.1f} GB free under {ev.PROJ_DIR} "
                        f"(--min-free-gb {opts.min_free_gb})")
    return problems


def main():
    ap = argparse.ArgumentParser(description="RTL evaluation campaign of all predictors (see the header).")
    ap.add_argument("mode", choices=sorted(MODES))
    ap.add_argument("--predictors", nargs="+", default=ORDER, choices=ORDER)
    ap.add_argument("--stop-on-fail", action="store_true")
    ev.add_common_args(ap)
    args = ap.parse_args()
    ev.check_common_args(ap, args)

    preds = [p for p in ORDER if p in args.predictors]
    budgets = MODES[args.mode]
    opts = ev.make_opts(budgets=budgets, **{k: v for k, v in vars(args).items()
                                            if k in ev.DEFAULTS})

    problems = preflight(preds, budgets, opts)
    if problems:
        print("PREFLIGHT FAILED:\n  " + "\n  ".join(problems), file=sys.stderr)
        sys.exit(2)

    t0 = time.time()
    status, rows = {}, []
    with ev.Lock():
        for p in preds:
            ev.log("\n" + "#" * 100 + f"\n# {args.mode}: {p}\n" + "#" * 100)
            ok, prow = ev.evaluate_predictor(p, opts)
            status[p] = ok
            rows += [r for r in prow if budgets is None or r["budget"] in budgets]
            if not ok and args.stop_on_fail and not args.dry_run:
                ev.log(f"\n{p} FAILED: stopping (--stop-on-fail)")
                break
    if args.dry_run:
        return

    out_csv = os.path.join(ev.EVAL_DIR, f"campaign_{args.mode}.csv")
    if rows:
        ev.write_csv(out_csv, rows)
        ev.print_summary(rows, f"campaign {args.mode}   {out_csv}")
    ev.log(f"\n{args.mode} campaign, {(time.time() - t0) / 60:.1f} min:")
    for p in preds:
        st = "not run" if p not in status else ("ok" if status[p] else "FAILED")
        ev.log(f"  {p:18s} {st}")
    sys.exit(0 if status and all(status.values()) and len(status) == len(preds) else 1)


if __name__ == "__main__":
    main()