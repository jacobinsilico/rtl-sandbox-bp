#!/usr/bin/python3
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
# -----------------------------------------------------------------------------
"""fmax_sweep.py - the highest clock frequency each predictor core reaches in
this flow (Yosys + ABC with the period as delay target, OpenSTA with the
SRAM-port constraints of sta_reports.tcl), per budget, and the logic area it
costs.

For every predictor and budget it runs
    evaluate.py <predictor> --budgets <B> --steps syn sta --clk <P> --rerun [--extra ...]
with the environment of --env added (e.g. SYN_ABC=speed, a flow variant of
run.tcl). --rerun because evaluate.py's step keys do not see environment
variables or the flow scripts: every probe is a fresh synthesis.
at a sequence of periods. Synthesis is timing-driven (ABC map -D <period>), so
every probe is a new synthesis: a tighter target can give a faster netlist and
more area. A netlist does not change with the period STA checks it at, and
the SRAM-port delays are absolute (front-end I/O delay 0 by default), so a
netlist with critical delay D meets every period >= D, whatever target it
was synthesized for: its frequency is 1 / D. fmax is the fastest netlist.

Two modes, per (predictor, budget):
  search (default): synthesize at --start (0.9 ns), then each time at
    --push (0.85) x the shortest critical delay so far, on a grid of --res ns,
    until a tighter target no longer gives a netlist at least one grid step
    faster (--patience times in a row), --floor or --max-runs: the limit of
    the tool for that design.
  grid (--periods P1 P2 ...): one synthesis at every listed period, for an
    evenly sampled area-frequency curve.

Every probe is kept: one row in <out>/fmax_runs.csv and its reports in
<out>/<predictor>/<budget>/[<tag>_]<P>ps/. After every sweep, and with
--report, the probes are summarized:
  - fmax table: the fastest netlist per predictor / budget / tag, the target
    it was synthesized for, its logic area, the area of the netlist
    synthesized at the reference period --ref-clk (0.9 ns) and the area
    increase between the two;
  - <out>/area_vs_freq.csv: one row per synthesis, frequency achieved
    (1 / critical delay, met or not: every netlist is a real design point)
    against logic area, and whether it is on the Pareto front (no other
    netlist of the same predictor / budget / tag is both faster and smaller);
  - <out>/area_vs_freq_<budget>.png (if matplotlib is installed): those
    points and the Pareto front, one series per predictor (and tag).
Area is synthesized logic only (SRAMs excluded, as in evaluate.py).

evaluate.py keeps one record per step and budget (the last clock it ran),
so the sweep overwrites the 0.9 ns records. At the end every swept budget
is synthesized again at --restore-clk (default 0.9; 0 = leave the last probe)
without the --extra parameters, so eval/ shows the reference point again.

Predictors run in parallel with --jobs (each has its own result CSV and
imp/ directories); the budgets of one predictor run one after the other.

Usage (from the repository root, after `source sourceme.sh`):
  python3 projects/branch-prediction/scripts/fmax_sweep.py \
      gshare hashed_perceptron --budgets 1KB 16KB --jobs 2
  # an evenly sampled curve (adds to the same CSV):
  python3 projects/branch-prediction/scripts/fmax_sweep.py hashed_perceptron \
      --budgets 16KB --periods 1.2 1.0 0.9 0.8 0.7 0.6 0.5
  # a pipelining knob for one predictor only:
  python3 projects/branch-prediction/scripts/fmax_sweep.py hashed_perceptron \
      --budgets 16KB --extra hashed_perceptron:Y_REG=1 --tag yreg
  # a flow variant (delay-oriented ABC script, see run.tcl):
  python3 projects/branch-prediction/scripts/fmax_sweep.py gshare hashed_perceptron \
      --budgets 1KB 16KB --env SYN_ABC=speed --tag abc_speed
  # tables, CSV and plots again from fmax_runs.csv (log area axis: gshare's
  # logic is ~50x smaller than the perceptrons'):
  python3 projects/branch-prediction/scripts/fmax_sweep.py --report --log-area

What the numbers mean: post-synthesis, ideal clock, no wire delay, the SRAM
clock-to-Q / setup placeholders of sta_reports.tcl, front-end I/O delays from
IO_DELAY_PCT (0 by default). They say how short a cycle the logic between the
SRAM ports and the front-end ports fits in, and its area; nothing about the
SRAM macros' own cycle time or area.
"""

import argparse
import concurrent.futures
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import threading
import time

PROJECT = "branch-prediction"

RUN_FIELDS = ["time", "tag", "predictor", "budget", "clk_ns", "extra", "syn", "sta",
              "slack_ps", "crit_ps", "pred_ps", "area_um2", "flops", "met",
              "crit_start", "crit_end", "log"]
CURVE_FIELDS = ["tag", "predictor", "budget", "extra", "target_ns", "met", "crit_ps",
                "achieved_ghz", "area_um2", "flops", "pareto"]

_print_lock = threading.Lock()
_csv_lock = threading.Lock()


def say(msg):
    with _print_lock:
        print(msg, flush=True)


def repo_home():
    rh = os.environ.get("REPO_HOME", "")
    if not rh or not os.path.isdir(os.path.join(rh, "projects", PROJECT)):
        sys.exit("fmax_sweep: REPO_HOME is not set (or wrong): run `source sourceme.sh` "
                 "in the repository root first")
    return rh


def load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def series_name(tag, pred, extra):
    s = pred
    if tag:
        s += f" [{tag}]"
    elif extra:
        s += f" [{extra}]"
    return s


# -----------------------------------------------------------------------------
# Sweep
# -----------------------------------------------------------------------------
class Sweep:
    def __init__(self, args, rh):
        self.a = args
        self.rh = rh
        self.proj = os.path.join(rh, "projects", PROJECT)
        self.evaluate = args.evaluate or os.path.join(self.proj, "scripts", "evaluate.py")
        self.out = os.path.abspath(args.out or os.path.join(self.proj, "eval", "fmax"))
        os.makedirs(os.path.join(self.out, "logs"), exist_ok=True)
        self.runs_csv = os.path.join(self.out, "fmax_runs.csv")
        self.res_ps = int(round(args.res * 1000))
        if self.res_ps < 1:
            sys.exit("fmax_sweep: --res must be >= 0.001 ns")

    # ---- grid -------------------------------------------------------------
    def snap(self, ps, how):
        r = self.res_ps
        q = ps / r
        k = math.floor(q + 1e-9) if how == "down" else (math.ceil(q - 1e-9) if how == "up"
                                                         else round(q))
        return int(k * r)

    # ---- one probe --------------------------------------------------------
    def run_point(self, pred, budget, clk_ps, extras, restore=False):
        clk = clk_ps / 1000.0
        name = f"{pred}_{budget}_{clk_ps}ps" + ("_restore" if restore else "")
        if self.a.tag and not restore:
            name = f"{self.a.tag}_{name}"
        log = os.path.join(self.out, "logs", name + ".log")
        cmd = [sys.executable, self.evaluate, pred, "--budgets", budget,
               "--steps", "syn", "sta", "--clk", f"{clk:.3f}", "--rerun"]
        if extras:
            cmd += ["--extra"] + list(extras)
        env = dict(os.environ)
        envs = [] if restore else list(self.a.env)
        for kv in envs:
            k, v = kv.split("=", 1)
            env[k] = v
        t0 = time.time()
        with open(log, "w") as lf:
            lf.write("$ " + " ".join(([f"{kv}" for kv in envs]) + cmd) + "\n\n")
            lf.flush()
            rc = subprocess.call(cmd, cwd=self.rh, stdout=lf, stderr=subprocess.STDOUT, env=env)

        d = os.path.join(self.proj, "eval", pred, budget)
        syn = load_json(os.path.join(d, "syn.json")) or {}
        sta = load_json(os.path.join(d, "sta.json")) or {}
        sta_path = os.path.join(d, "sta.json")
        fresh = os.path.exists(sta_path) and os.path.getmtime(sta_path) >= t0 - 1
        sm, tm = syn.get("metrics", {}) or {}, sta.get("metrics", {}) or {}
        crit = fnum(tm.get("crit_delay_ps"))
        slack = fnum(tm.get("worst_slack_ps"))
        if slack is None and crit is not None:
            slack = clk_ps - crit
        if crit is None and slack is not None:
            crit = clk_ps - slack
        ok = (syn.get("status") == "ok" and sta.get("status") == "ok" and fresh
              and crit is not None and abs((crit + slack) - clk_ps) < 1.0)

        rec = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "tag": self.a.tag,
               "predictor": pred, "budget": budget, "clk_ns": f"{clk:.3f}",
               "extra": " ".join(list(extras) + [f"env:{kv}" for kv in envs]),
               "syn": syn.get("status", "-"),
               "sta": sta.get("status", "-") if fresh else "stale",
               "slack_ps": f"{slack:.1f}" if ok else "", "crit_ps": f"{crit:.1f}" if ok else "",
               "pred_ps": tm.get("pred_path_ps", "") if ok else "",
               "area_um2": sm.get("area_um2", "") if ok else "",
               "flops": sm.get("flops", "") if ok else "",
               "met": ("yes" if slack >= 0 else "no") if ok else "error",
               "crit_start": tm.get("crit_start", "") if ok else "",
               "crit_end": tm.get("crit_end", "") if ok else "",
               "log": os.path.relpath(log, self.rh)}
        if not restore:
            self.append_run(rec)
            if ok:
                self.keep_reports(pred, budget, clk_ps)
        if not ok:
            say(f"  {pred:17s} {budget:>5s} {clk:6.3f} ns  ERROR (rc {rc}, syn {rec['syn']}, "
                f"sta {rec['sta']}): see {rec['log']}")
            return None
        rec["_slack"], rec["_crit"] = slack, crit
        if not restore:
            say(f"  {pred:17s} {budget:>5s} {clk:6.3f} ns  slack {slack:+8.1f} ps  "
                f"crit {crit:7.1f}  pred {fnum(rec['pred_ps']) or 0:7.1f}  "
                f"area {fnum(rec['area_um2']) or 0:8.1f}  {'MET' if slack >= 0 else 'fail'}")
        return rec

    def append_run(self, rec):
        with _csv_lock:
            new = not os.path.exists(self.runs_csv)
            with open(self.runs_csv, "a", newline="") as f:
                w = csv.DictWriter(f, fieldnames=RUN_FIELDS, extrasaction="ignore")
                if new:
                    w.writeheader()
                w.writerow(rec)

    def keep_reports(self, pred, budget, clk_ps):
        src = os.path.join(self.proj, "imp", f"sta_{pred}_{budget}", "report")
        dst = os.path.join(self.out, pred, budget,
                           (f"{self.a.tag}_" if self.a.tag else "") + f"{clk_ps}ps")
        os.makedirs(dst, exist_ok=True)
        for n in ("worst_path.rpt", "pred_path.rpt", "worst_endpoints.rpt", "worst_slack.rpt"):
            p = os.path.join(src, n)
            if os.path.exists(p):
                shutil.copy2(p, dst)

    # ---- search for one (predictor, budget) --------------------------------
    def search(self, pred, budget, extras):
        a = self.a
        floor_ps = int(round(a.floor * 1000))
        p = self.snap(int(round(a.start * 1000)), "near")
        best, ran, stale, tried = None, 0, 0, set()
        while ran < a.max_runs and p >= floor_ps and p not in tried:
            tried.add(p)
            rec = self.run_point(pred, budget, p, extras)
            if rec is None:
                break
            ran += 1
            c = rec["_crit"]
            if best is not None and c > best - self.res_ps:
                stale += 1                # no netlist a grid step faster
                if stale >= a.patience:
                    break
            else:
                stale = 0
            best = c if best is None else min(best, c)
            nxt = max(self.snap(best * a.push, "down"), floor_ps)
            if nxt in tried:
                nxt -= self.res_ps
            p = nxt
        return ran

    def grid(self, pred, budget, extras):
        ran = 0
        for P in self.a.periods:
            if self.run_point(pred, budget, self.snap(int(round(P * 1000)), "near"), extras):
                ran += 1
        return ran

    def run_predictor(self, pred):
        extras = [e.split(":", 1)[1] for e in self.a.extra if e.split(":", 1)[0] == pred]
        swept = []
        for b in self.a.budgets:
            say(f"== {pred} {b}" + (f"  (extra {' '.join(extras)})" if extras else "")
                + (f"  (env {' '.join(self.a.env)})" if self.a.env else ""))
            ran = self.grid(pred, b, extras) if self.a.periods else self.search(pred, b, extras)
            if ran:
                swept.append(b)
        if self.a.restore_clk > 0:
            for b in swept:
                say(f"   restoring {pred} {b} at {self.a.restore_clk} ns")
                self.run_point(pred, b, int(round(self.a.restore_clk * 1000)), [], restore=True)


# -----------------------------------------------------------------------------
# Report: fmax table, area-frequency CSV, plots
# -----------------------------------------------------------------------------
def load_runs(path):
    runs = []
    if not os.path.exists(path):
        return runs
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            r["_clk"] = fnum(r["clk_ns"])
            r["_crit"] = fnum(r["crit_ps"])
            r["_area"] = fnum(r["area_um2"])
            r["_flops"] = fnum(r["flops"])
            r["_ok"] = r["met"] in ("yes", "no") and r["_crit"] and r["_area"] is not None
            runs.append(r)
    return runs


def group_key(r):
    return (r["tag"], r["predictor"], r["budget"], r["extra"])


def budget_order(b):
    try:
        return int(b.upper().rstrip("KB"))
    except ValueError:
        return 0


def pareto_flags(points):
    """points: list of (ghz, area). True where no other point is at least as
    fast and at least as small (one strictly); of identical points (the same
    netlist found at several targets) only the first counts."""
    pts = [(round(g, 4), round(a, 3)) for g, a in points]
    flags = []
    for i, (g, a) in enumerate(pts):
        dom = any(((g2 >= g and a2 <= a and (g2 > g or a2 < a)) or ((g2, a2) == (g, a) and j < i))
                  for j, (g2, a2) in enumerate(pts) if j != i)
        flags.append(not dom)
    return flags


def summarize(runs, ref_clk):
    groups = {}
    for r in runs:
        groups.setdefault(group_key(r), []).append(r)
    rows, curve = [], []
    for key in sorted(groups, key=lambda k: (budget_order(k[2]), k[1], k[0], k[3])):
        tag, pred, b, ex = key
        rs = [r for r in groups[key] if r["_ok"]]
        # one point per period: the latest probe at it
        latest = {}
        for r in rs:
            latest[r["_clk"]] = r
        rs = sorted(latest.values(), key=lambda r: -r["_clk"])
        # fastest netlist (shortest critical delay; the smaller one on a tie)
        best = min(rs, key=lambda r: (round(r["_crit"], 1), r["_area"])) if rs else None
        ref = next((r for r in rs if abs(r["_clk"] - ref_clk) < 1e-6), None)
        row = {"tag": tag, "predictor": pred, "budget": b, "extra": ex, "runs": len(groups[key])}
        if best:
            row.update(fmax_ghz=1000.0 / best["_crit"], crit_best=best["_crit"],
                       target_best=best["_clk"], area_best=best["_area"], flops_best=best["_flops"])
        if ref:
            row.update(area_ref=ref["_area"], fref_ghz=1000.0 / ref["_crit"])
        if best and ref and ref["_area"]:
            row["area_delta_pct"] = 100.0 * (best["_area"] - ref["_area"]) / ref["_area"]
        if not rs:
            row["note"] = "no netlist: synthesis or STA failed (see the log column of fmax_runs.csv)"
        rows.append(row)
        pts = [(1000.0 / r["_crit"], r["_area"]) for r in rs]
        for r, pf in zip(rs, pareto_flags(pts)):
            curve.append({"tag": tag, "predictor": pred, "budget": b, "extra": ex,
                          "target_ns": f"{r['_clk']:.3f}", "met": r["met"],
                          "crit_ps": f"{r['_crit']:.1f}", "achieved_ghz": f"{1000.0 / r['_crit']:.3f}",
                          "area_um2": f"{r['_area']:.2f}",
                          "flops": "" if r["_flops"] is None else f"{r['_flops']:.0f}",
                          "pareto": "yes" if pf else ""})
    return rows, curve


def fmt(v, spec):
    return "" if v is None else format(v, spec)


def print_table(rows, ref_clk):
    print(f"\nfmax and logic area (fastest netlist; post-synthesis, SRAM placeholders, SRAMs "
          f"excluded from area; reference: the netlist synthesized at {ref_clk} ns)")
    hdr = (f"{'predictor':17s} {'budget':>6s} {'tag/extra':14s} {'fmax GHz':>8s} {'crit ps':>8s} "
           f"{'target':>7s} {'area':>8s} {'flops':>6s} {'ref GHz':>8s} {'area ref':>9s} "
           f"{'d area':>7s} {'runs':>4s}")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        te = r["tag"] or r["extra"]
        d = r.get("area_delta_pct")
        print(f"{r['predictor']:17s} {r['budget']:>6s} {te[:14]:14s} "
              f"{fmt(r.get('fmax_ghz'), '8.2f'):>8s} {fmt(r.get('crit_best'), '8.1f'):>8s} "
              f"{fmt(r.get('target_best'), '7.3f'):>7s} {fmt(r.get('area_best'), '8.1f'):>8s} "
              f"{fmt(r.get('flops_best'), '6.0f'):>6s} {fmt(r.get('fref_ghz'), '8.2f'):>8s} "
              f"{fmt(r.get('area_ref'), '9.1f'):>9s} "
              f"{(format(d, '+6.1f') + '%') if d is not None else '':>7s} {r['runs']:>4d}"
              + (f"  {r['note']}" if r.get("note") else ""))
    print("  fmax: 1 / shortest critical delay of any netlist (it meets every period >= its delay);"
          "\n  target: the period it was synthesized for;  ref GHz / area ref: the netlist "
          "synthesized at the reference;\n  d area: area at fmax vs area at the reference")


def print_pareto(curve):
    print("\nPareto front (no other netlist of the series is both faster and smaller):")
    by = {}
    for c in curve:
        if c["pareto"]:
            by.setdefault((c["budget"], series_name(c["tag"], c["predictor"], c["extra"])), []).append(c)
    for (b, s) in sorted(by, key=lambda k: (budget_order(k[0]), k[1])):
        pts = sorted(by[(b, s)], key=lambda c: float(c["achieved_ghz"]))
        print(f"  {b:>5s} {s:30s} " + "  ".join(f"{float(c['achieved_ghz']):.2f} GHz/"
                                                f"{float(c['area_um2']):.0f}" for c in pts))


def write_curve(curve, path):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CURVE_FIELDS)
        w.writeheader()
        w.writerows(curve)


def plot_curve(curve, outdir, ref_clk, log_area=False):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not installed: no plots; area_vs_freq.csv has the data)")
        return []
    files = []
    budgets = sorted({c["budget"] for c in curve}, key=budget_order)
    for b in budgets:
        fig, ax = plt.subplots(figsize=(7.5, 4.8))
        series = {}
        for c in curve:
            if c["budget"] == b:
                series.setdefault(series_name(c["tag"], c["predictor"], c["extra"]), []).append(c)
        for i, (s, cs) in enumerate(sorted(series.items())):
            col = f"C{i % 10}"
            g = [float(c["achieved_ghz"]) for c in cs]
            a = [float(c["area_um2"]) for c in cs]
            ax.scatter(g, a, s=18, color=col, alpha=0.45)
            front = sorted((float(c["achieved_ghz"]), float(c["area_um2"])) for c in cs if c["pareto"])
            if front:
                ax.plot([p[0] for p in front], [p[1] for p in front], "-o", color=col, ms=4, label=s)
        ax.axvline(1.0 / ref_clk, color="0.6", ls="--", lw=0.8)
        ax.text(1.0 / ref_clk, ax.get_ylim()[1], f" {ref_clk} ns", color="0.4", va="top", fontsize=8)
        if log_area:
            ax.set_yscale("log")
        ax.set_xlabel("achieved frequency, 1 / critical delay (GHz)")
        ax.set_ylabel("logic area (um^2, SRAMs excluded)")
        ax.set_title(f"Area vs frequency, {b} (post-synthesis, SRAM placeholders)", fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        p = os.path.join(outdir, f"area_vs_freq_{b}.png")
        fig.savefig(p, dpi=140)
        plt.close(fig)
        files.append(p)
    return files


def report(out, ref_clk, rh, log_area=False):
    runs_csv = os.path.join(out, "fmax_runs.csv")
    runs = load_runs(runs_csv)
    if not runs:
        print(f"fmax_sweep: no probes in {runs_csv}")
        return
    rows, curve = summarize(runs, ref_clk)
    print_table(rows, ref_clk)
    print_pareto(curve)
    cpath = os.path.join(out, "area_vs_freq.csv")
    write_curve(curve, cpath)
    pngs = plot_curve(curve, out, ref_clk, log_area)
    rel = lambda p: os.path.relpath(p, rh)
    print(f"\nevery probe: {rel(runs_csv)}\narea vs frequency: {rel(cpath)}"
          + "".join(f"\nplot: {rel(p)}" for p in pngs)
          + f"\nreports: {rel(out)}/<predictor>/<budget>/[<tag>_]<period>ps/")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("predictors", nargs="*", help="evaluate.py predictor names")
    ap.add_argument("--budgets", nargs="+", default=["1KB", "16KB"])
    ap.add_argument("--periods", nargs="+", type=float, metavar="NS",
                    help="grid mode: synthesize at exactly these periods (no search)")
    ap.add_argument("--start", type=float, default=0.9, help="search: first period, ns (0.9)")
    ap.add_argument("--floor", type=float, default=0.2, help="search: shortest period, ns (0.2)")
    ap.add_argument("--res", type=float, default=0.02, help="period grid, ns (0.02)")
    ap.add_argument("--push", type=float, default=0.85,
                    help="search: next target = push x shortest critical delay so far (0.85)")
    ap.add_argument("--max-runs", type=int, default=8, help="search: syntheses per budget (8)")
    ap.add_argument("--patience", type=int, default=1,
                    help="search: stop after this many probes in a row without a netlist one "
                         "grid step faster (1)")
    ap.add_argument("--extra", action="append", default=[], metavar="PRED:KEY=VALUE",
                    help="core parameter for one predictor, passed as evaluate.py --extra")
    ap.add_argument("--env", action="append", default=[], metavar="KEY=VALUE",
                    help="environment for every probe, e.g. SYN_ABC=speed (not for the restore)")
    ap.add_argument("--tag", default="", help="label for this sweep in the CSV, report dirs, plots")
    ap.add_argument("--ref-clk", type=float, default=0.9,
                    help="reference period for the area comparison, ns (0.9)")
    ap.add_argument("--restore-clk", type=float, default=0.9,
                    help="re-synthesize every swept budget at this period at the end (0 = no)")
    ap.add_argument("--jobs", type=int, default=1, help="predictors in parallel (1)")
    ap.add_argument("--out", help="output directory (eval/fmax)")
    ap.add_argument("--evaluate", help="path of evaluate.py (projects/<p>/scripts/evaluate.py)")
    ap.add_argument("--log-area", action="store_true", help="plots: logarithmic area axis")
    ap.add_argument("--report", action="store_true",
                    help="only the tables, area_vs_freq.csv and plots from fmax_runs.csv")
    a = ap.parse_args()

    rh = repo_home()
    sw = Sweep(a, rh)
    if a.report:
        report(sw.out, a.ref_clk, rh, a.log_area)
        return
    if not a.predictors:
        ap.error("give at least one predictor (or --report)")
    for e in a.extra:
        if ":" not in e or "=" not in e.split(":", 1)[1]:
            ap.error(f"--extra {e}: expected PREDICTOR:KEY=VALUE")
    for e in a.env:
        if "=" not in e:
            ap.error(f"--env {e}: expected KEY=VALUE")
    if not os.path.exists(sw.evaluate):
        sys.exit(f"fmax_sweep: no {sw.evaluate}")

    mode = (f"grid {' '.join(str(p) for p in a.periods)} ns" if a.periods else
            f"search from {a.start} ns, push {a.push}, grid {a.res} ns, patience {a.patience}, "
            f"floor {a.floor} ns, <= {a.max_runs} runs each")
    print(f"fmax sweep: {' '.join(a.predictors)} | budgets {' '.join(a.budgets)} | {mode} | "
          f"out {os.path.relpath(sw.out, rh)}")
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, a.jobs)) as ex:
        list(ex.map(sw.run_predictor, a.predictors))
    report(sw.out, a.ref_clk, rh, a.log_area)


if __name__ == "__main__":
    main()