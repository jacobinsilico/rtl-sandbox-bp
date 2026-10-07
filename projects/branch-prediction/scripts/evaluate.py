#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Full-flow evaluation of one predictor across budgets. For every budget
#   that has golden dumps under projects/<project>/stim/<predictor>/:
#
#     rtl  make sim on every trace's dump (RTL vs CBP5, bit-exact)
#     syn  make syn of the core (the SRAMs stay outside, in the bench)
#     sta  make post-syn-sta
#     gls  make post-syn-sim VCD=1 on one trace (--gls-trace, --gls-lines):
#          the netlist checked against the dump again, and the activity
#     dpa  make post-syn-dpa on that activity
#
#   The RTL parameters come from the dump's .json (the CBP5 -D defines), so
#   every step is built with exactly the config that produced the dumps.
#   Steps run in this order per budget; a failed step skips the ones that
#   depend on it. Budgets run one after another (the gate-level flow writes
#   activity.vcd into a shared run directory, so runs must not overlap).
#
#   Resumable: each finished step leaves eval/<predictor>/<budget>/<step>.json
#   (status, metrics, the exact command key). A step whose json is ok and
#   whose key is unchanged is skipped; --rerun forces everything.
#
# Output:
#   projects/<project>/eval/<predictor>/<budget>/<step>.{log,json}
#   projects/<project>/eval/<predictor>_results.csv     one row per budget
#   a summary table on stdout
#
#   Energy per conditional branch (logic only, SRAM excluded):
#     E = P_total * TB_CYCLES * T_clk / TB_COND_BR   (from the gate-level run)
#   SRAM traffic per conditional branch: TB_SRAM_READS / TB_COND_BR, ...
#   The SRAM structures (PREDICTOR_STRUCT lines of the CBP5 log) are copied
#   into the CSV for the CACTI step.
#
# Usage (after `source sourceme.sh`):
#   python3 projects/branch-prediction/scripts/evaluate.py PREDICTOR [options]
#     PREDICTOR          gshare | g_perceptron
#     --budgets 1KB 4KB  subset of budgets (default: all with dumps)
#     --steps rtl syn    subset of steps (default: rtl syn sta gls dpa)
#     --rtl-lines N      branch lines per RTL check (default 0 = whole trace)
#     --gls-trace PAT    trace for the gate-level run (default 'fdd*')
#     --gls-lines N      branch lines for the gate-level run (default 50000)
#     --clk NS           clock period (default 1.0)
#     --extra K=V ...    extra RTL parameters, e.g. --extra Y_REG=1
#     --hier             also synthesize with KEEP_HIERARCHY=1 (area per module)
#     --rerun            ignore finished steps
#     --dry-run          print the make commands only
#
#   Adding a predictor: one entry in PREDICTORS below.
#   Report parsing is tolerant: a value that cannot be found is left empty
#   and a WARNING names the report file.
# -----------------------------------------------------------------------------

import argparse
import csv
import fnmatch
import glob
import json
import math
import os
import re
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJ_DIR = os.path.dirname(SCRIPT_DIR)
PROJECT = os.path.basename(PROJ_DIR)
REPO = os.environ.get("REPO_HOME") or os.path.dirname(os.path.dirname(PROJ_DIR))

STEPS = ["rtl", "syn", "sta", "gls", "dpa"]
NEEDS = {"rtl": [], "syn": [], "sta": ["syn"], "gls": ["syn"], "dpa": ["gls"]}


# =============================================================================
# Predictors: CBP5 defines -> RTL top and parameters
# =============================================================================

def _need(d, key, default=None):
    if key in d:
        return int(d[key])
    if default is not None:
        return default
    raise ValueError(f"-D{key} missing")


def map_gshare(d):
    ghr, idx, ent = _need(d, "GHR_BITS"), _need(d, "PHT_INDEX_BITS"), _need(d, "PHT_ENTRIES")
    if ent != 1 << idx:
        raise ValueError(f"PHT_ENTRIES {ent} != 2^PHT_INDEX_BITS")
    return {"GHR_BITS": ghr, "INDEX_BITS": idx, "PC_SHIFT": _need(d, "PC_SHIFT", 0)}


def map_g_perceptron(d):
    h, n = _need(d, "GHR_LEN"), _need(d, "NUM_PERCEPTRONS")
    if not 1 <= h <= 64:
        raise ValueError(f"GHR_LEN {h} outside the RTL range 1..64")
    if n < 2:
        raise ValueError(f"NUM_PERCEPTRONS {n} not supported by the RTL")
    return {"GHR_LEN": h, "NUM_PERCEPTRONS": n, "WEIGHT_BITS": _need(d, "WEIGHT_BITS"),
            "THETA_ALPHA_PCT": _need(d, "THETA_ALPHA_PCT", 100),
            "PC_SHIFT": _need(d, "PC_SHIFT", 0)}


# stim directory name -> (RTL core = synthesis top = bench DUT, mapping)
PREDICTORS = {
    "gshare":       ("bp_gshare_core", map_gshare),
    "g_perceptron": ("bp_gp_core",     map_g_perceptron),
}


# =============================================================================
# Helpers
# =============================================================================

def log(msg=""):
    print(msg, flush=True)


def params_str(p):
    return " ".join(f"{k}={v}" for k, v in p.items())


def budget_key(label):
    m = re.match(r"(\d+)KB", label)
    return (int(m.group(1)) if m else 1 << 30, label)


def read(path):
    try:
        with open(path, errors="replace") as f:
            return f.read()
    except OSError:
        return ""


def first(patterns, text, cast=float):
    for pat in patterns:
        m = re.search(pat, text, re.MULTILINE | re.IGNORECASE)
        if m:
            try:
                return cast(m.group(1))
            except ValueError:
                continue
    return None


def report_text(out_dir, prefer=None):
    """Concatenated report files of a flow run (preferred names first)."""
    files = sorted(glob.glob(os.path.join(PROJ_DIR, "imp", out_dir, "report", "*")))
    if prefer:
        files.sort(key=lambda f: 0 if any(p in os.path.basename(f) for p in prefer) else 1)
    return "\n".join(read(f) for f in files if os.path.isfile(f)), files


def tb_metrics(text):
    keys = ["TB_LINES", "TB_COND_BR", "TB_UNCOND_BR", "TB_RTL_MISPRED", "TB_REF_MISPRED",
            "TB_CYCLES", "TB_SRAM_READS", "TB_SRAM_WRITES"]
    m = {k.lower()[3:]: first([rf"^{k}\s*:\s*(\d+)"], text, int) for k in keys}
    m["passed"] = "PASSED" in text
    return m


def warn_missing(step, metrics, keys, files):
    missing = [k for k in keys if metrics.get(k) is None]
    if missing:
        where = ", ".join(os.path.relpath(f, REPO) for f in files) or "(no report files)"
        log(f"    WARNING {step}: could not parse {', '.join(missing)} from {where}")


# =============================================================================
# Report parsers
# =============================================================================

def parse_area(out_dir):
    text, files = report_text(out_dir, prefer=["area"])
    m = {
        "area_um2": first([r"Chip area for (?:top )?module\s+'?\\?[^':]*'?\s*:\s*([\d.]+)",
                           r"Chip area\s*:\s*([\d.]+)",
                           r"Design area\s+([\d.]+)"], text),
        "cells": first([r"Number of cells\s*:?\s*(\d+)", r"^\s*(\d+)\s+(?:[\d.]+\s+)?cells\s*$"],
                       text, int),
    }
    # Flip-flops: every cell-type line naming a DFF; count = its integer
    # column (works for "NAME count" and "count [area] NAME" layouts).
    flops, seen = 0, False
    for line in text.splitlines():
        if "DFF" not in line.upper() or ":" in line:
            continue
        m1 = re.match(r"^\s*(\S*DFF\S*)\s+(\d+)\s*$", line, re.IGNORECASE)
        m2 = re.match(r"^\s*(\d+)\s+(?:[\d.]+\s+)?(\S*DFF\S*)\s*$", line, re.IGNORECASE)
        if m1:
            flops += int(m1.group(2)); seen = True
        elif m2:
            flops += int(m2.group(1)); seen = True
    m["flops"] = flops if seen else None
    warn_missing("syn", m, ["area_um2", "cells", "flops"], files)
    return m


def parse_area_hier(out_dir):
    text, _ = report_text(out_dir, prefer=["area"])
    return {name: float(a) for name, a in
            re.findall(r"Chip area for (?:top )?module\s+'?\\?([^':\s]+)'?\s*:\s*([\d.]+)", text)}


def parse_sta(out_dir):
    text, files = report_text(out_dir, prefer=["wns", "slack", "timing", "path"])
    m = {
        "wns": first([r"^\s*wns\s+(?:max\s+)?(-?[\d.]+)", r"worst slack\s+(?:max\s+)?(-?[\d.]+)",
                      r"\bWNS\b[^-\d\n]*(-?[\d.]+)"], text),
        "tns": first([r"^\s*tns\s+(?:max\s+)?(-?[\d.]+)", r"\bTNS\b[^-\d\n]*(-?[\d.]+)"], text),
        "crit_start": first([r"Startpoint:\s*(\S+)"], text, str),
        "crit_end": first([r"Endpoint:\s*(\S+)"], text, str),
    }
    warn_missing("sta", m, ["wns"], files)
    return m


def parse_power(out_dir):
    text, files = report_text(out_dir, prefer=["power"])
    num = r"([\d.]+(?:[eE][+-]?\d+)?)"
    m = {}
    for group in ("Sequential", "Combinational", "Clock", "Total"):
        g = re.search(rf"^\s*{group}\s+{num}\s+{num}\s+{num}\s+{num}", text, re.MULTILINE)
        m[f"p_{group.lower()}_w"] = float(g.group(4)) if g else None
        if group == "Total" and g:
            m["p_internal_w"], m["p_switching_w"], m["p_leakage_w"] = \
                float(g.group(1)), float(g.group(2)), float(g.group(3))
    ann = read(os.path.join(PROJ_DIR, "imp", out_dir, "report", "vcd_annotated.rpt"))
    una = read(os.path.join(PROJ_DIR, "imp", out_dir, "report", "vcd_unannotated.rpt"))
    m["vcd_annotated"] = first([r"(\d+)"], ann, int)
    m["vcd_unannotated"] = first([r"(\d+)"], una, int)
    warn_missing("dpa", m, ["p_total_w"], files)
    return m


# =============================================================================
# Steps
# =============================================================================

def make(args, target, variables, logfile):
    cmd = ["make", "-C", REPO, target, f"PROJECT={PROJECT}"] + [f"{k}={v}" for k, v in variables]
    if args.dry_run:
        log("    " + " ".join(cmd))
        return 0, ""
    with open(logfile, "w") as f:
        rc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT).returncode
    return rc, read(logfile)


def stim_param(path):
    return f'STIM_FILE="{path}"'          # Verilator wants the quotes (no spaces in paths)


class Budget:
    def __init__(self, args, top, mapping, label, dump_dir):
        self.args, self.top, self.label, self.dir = args, top, label, dump_dir
        self.cid = os.path.basename(dump_dir).split("__", 1)[1]
        self.dumps = sorted(glob.glob(os.path.join(dump_dir, "*.golden.txt")))
        jsons = [d[:-len(".golden.txt")] + ".json" for d in self.dumps]
        meta = json.load(open(jsons[0]))
        self.defines = meta["defines"]
        self.core = mapping(dict(re.findall(r"-D(\w+)=(\S+)", self.defines)))
        for kv in args.extra:
            k, _, v = kv.partition("=")
            self.core[k] = v
        cbp_log = read(self.dumps[0][:-len(".golden.txt")] + ".log")
        self.size_bits = first([r"PREDICTOR_CONFIG.*\bSIZE_BITS (\d+)"], cbp_log, int)
        self.structs = ";".join(f"{n}:{e}x{w}" for n, e, w in re.findall(
            r"^PREDICTOR_STRUCT (\S+) ENTRIES (\d+) WIDTH (\d+)", cbp_log, re.MULTILINE))
        self.edir = os.path.join(PROJ_DIR, "eval", args.predictor, label)
        os.makedirs(self.edir, exist_ok=True)
        tag = f"{args.predictor}_{label}"
        self.out = {s: f"{s}_{tag}" for s in ("syn", "sta", "gls", "dpa")}
        self.out["syn_hier"] = f"syn_{tag}_hier"

    # ---- bookkeeping ----
    def rec_path(self, step):
        return os.path.join(self.edir, f"{step}.json")

    def load(self, step):
        try:
            return json.load(open(self.rec_path(step)))
        except (OSError, ValueError):
            return None

    def done(self, step, key):
        rec = self.load(step)
        return rec is not None and rec.get("status") == "ok" and rec.get("key") == key \
            and not self.args.rerun

    def save(self, step, key, status, metrics, t0):
        rec = {"step": step, "key": key, "status": status, "metrics": metrics,
               "time_s": round(time.time() - t0, 1)}
        if not self.args.dry_run:
            with open(self.rec_path(step), "w") as f:
                json.dump(rec, f, indent=1)
        return rec

    # ---- steps ----
    def run_rtl(self):
        key = f"{self.top}|{params_str(self.core)}|{self.args.rtl_lines}|{self.args.clk}"
        if self.done("rtl", key):
            return "ok (kept)"
        t0, per_trace, ok = time.time(), {}, True
        for dump in self.dumps:
            trace = os.path.basename(dump)[:-len(".golden.txt")]
            short = trace.split(".")[0]
            logf = os.path.join(self.edir, f"rtl_{short}.log")
            rc, text = make(self.args, "sim", [
                ("TOP_LEVEL", self.top), ("CLK_PERIOD_NS", self.args.clk),
                ("OUT_DIR", f"sim_{self.args.predictor}_{self.label}_{short}"),
                ("PARAMS", f"{params_str(self.core)} MAX_LINES={self.args.rtl_lines} {stim_param(dump)}")],
                logf)
            if self.args.dry_run:
                continue
            m = tb_metrics(text)
            m["ok"] = rc == 0 and m["passed"]
            per_trace[trace] = m
            ok &= m["ok"]
            log(f"    rtl {short:12s} {'PASS' if m['ok'] else 'FAIL'}  cond={m['cond_br']} "
                f"mispred={m['rtl_mispred']}" + ("" if m["ok"] else f"  (see {logf})"))
        if self.args.dry_run:
            return "dry"
        self.save("rtl", key, "ok" if ok else "failed", {"traces": per_trace}, t0)
        return "ok" if ok else "FAILED"

    def run_syn(self, hier=False):
        step = "syn_hier" if hier else "syn"
        key = f"{self.top}|{params_str(self.core)}|{self.args.clk}|{hier}"
        if self.done(step, key):
            return "ok (kept)"
        t0 = time.time()
        variables = [("TOP_LEVEL", self.top), ("CLK_PERIOD_NS", self.args.clk),
                     ("OUT_DIR", self.out[step]), ("PARAMS", params_str(self.core))]
        if hier:
            variables.append(("KEEP_HIERARCHY", 1))
        rc, _ = make(self.args, "syn", variables, os.path.join(self.edir, f"{step}.log"))
        if self.args.dry_run:
            return "dry"
        metrics = (parse_area_hier if hier else parse_area)(self.out[step]) if rc == 0 else {}
        self.save(step, key, "ok" if rc == 0 else "failed", metrics, t0)
        return "ok" if rc == 0 else f"FAILED (exit {rc})"

    def run_sta(self):
        key = f"{self.load('syn') and self.load('syn').get('key')}|{self.args.clk}"
        if self.done("sta", key):
            return "ok (kept)"
        t0 = time.time()
        rc, _ = make(self.args, "post-syn-sta", [
            ("TOP_LEVEL", self.top), ("CLK_PERIOD_NS", self.args.clk),
            ("OUT_DIR", self.out["sta"]), ("NETLIST_DIR", self.out["syn"])],
            os.path.join(self.edir, "sta.log"))
        if self.args.dry_run:
            return "dry"
        metrics = parse_sta(self.out["sta"]) if rc == 0 else {}
        self.save("sta", key, "ok" if rc == 0 else "failed", metrics, t0)
        return "ok" if rc == 0 else f"FAILED (exit {rc})"

    def gls_dump(self):
        for d in self.dumps:
            if fnmatch.fnmatch(os.path.basename(d), self.args.gls_trace + ".golden.txt") or \
               fnmatch.fnmatch(os.path.basename(d), self.args.gls_trace):
                return d
        return self.dumps[0]

    def run_gls(self):
        dump = self.gls_dump()
        key = f"{self.load('syn') and self.load('syn').get('key')}|{dump}|{self.args.gls_lines}"
        if self.done("gls", key):
            return "ok (kept)"
        t0 = time.time()
        logf = os.path.join(self.edir, "gls.log")
        rc, text = make(self.args, "post-syn-sim", [
            ("TOP_LEVEL", self.top), ("CLK_PERIOD_NS", self.args.clk),
            ("OUT_DIR", self.out["gls"]), ("NETLIST_DIR", self.out["syn"]), ("VCD", 1),
            ("PARAMS", f"{params_str(self.core)} MAX_LINES={self.args.gls_lines} {stim_param(dump)}")],
            logf)
        if self.args.dry_run:
            return "dry"
        m = tb_metrics(text)
        m["trace"] = os.path.basename(dump)[:-len(".golden.txt")]
        ok = rc == 0 and m["passed"]
        self.save("gls", key, "ok" if ok else "failed", m, t0)
        return "ok" if ok else f"FAILED (see {logf})"

    def run_dpa(self):
        key = f"{self.load('gls') and self.load('gls').get('key')}"
        if self.done("dpa", key):
            return "ok (kept)"
        t0 = time.time()
        rc, _ = make(self.args, "post-syn-dpa", [
            ("TOP_LEVEL", self.top), ("CLK_PERIOD_NS", self.args.clk),
            ("OUT_DIR", self.out["dpa"]), ("NETLIST_DIR", self.out["syn"]),
            ("VCD_DIR", self.out["gls"])], os.path.join(self.edir, "dpa.log"))
        if self.args.dry_run:
            return "dry"
        metrics = parse_power(self.out["dpa"]) if rc == 0 else {}
        self.save("dpa", key, "ok" if rc == 0 else "failed", metrics, t0)
        return "ok" if rc == 0 else f"FAILED (exit {rc})"

    # ---- result row ----
    def row(self):
        def met(step):
            rec = self.load(step)
            return (rec or {}).get("metrics", {}) if (rec or {}).get("status") == "ok" else {}
        rtl, syn, sta, gls, dpa = (met(s) for s in STEPS)
        r = {"predictor": self.args.predictor, "budget": self.label, "config_id": self.cid,
             "rtl_params": params_str(self.core), "size_bits": self.size_bits,
             "sram_structs": self.structs, "clk_ns": self.args.clk}
        traces = (rtl or {}).get("traces", {})
        r["rtl_pass"] = f"{sum(t['ok'] for t in traces.values())}/{len(self.dumps)}" if traces else ""
        mpk = [1000.0 * t["rtl_mispred"] / t["cond_br"] for t in traces.values()
               if t.get("ok") and t.get("cond_br")]
        r["mpkbr_geomean"] = round(math.exp(sum(map(math.log, mpk)) / len(mpk)), 4) \
            if mpk and all(v > 0 for v in mpk) else ""
        for k in ("area_um2", "cells", "flops"):
            r[k] = syn.get(k, "")
        for k in ("wns", "tns", "crit_start", "crit_end"):
            r[k] = sta.get(k, "")
        r["gls_pass"] = gls.get("passed", "")
        for k in ("cond_br", "cycles", "sram_reads", "sram_writes"):
            r[f"gls_{k}"] = gls.get(k, "")
        for k in ("p_total_w", "p_sequential_w", "p_combinational_w", "p_clock_w",
                  "p_internal_w", "p_switching_w", "p_leakage_w", "vcd_annotated", "vcd_unannotated"):
            r[k] = dpa.get(k, "")
        cond, cyc = gls.get("cond_br"), gls.get("cycles")
        if cond and cyc:
            r["cycles_per_cond"] = round(cyc / cond, 4)
            r["sram_rd_per_cond"] = round(gls["sram_reads"] / cond, 4)
            r["sram_wr_per_cond"] = round(gls["sram_writes"] / cond, 4)
            if dpa.get("p_total_w") is not None:
                r["e_logic_pj_per_cond"] = round(
                    dpa["p_total_w"] * cyc * float(self.args.clk) * 1e-9 / cond * 1e12, 4)
        hier = met("syn_hier")
        r["area_by_module"] = ";".join(f"{k}:{v}" for k, v in hier.items()) if hier else ""
        return r


# =============================================================================
# Main
# =============================================================================

def main():
    ap = argparse.ArgumentParser(description=__doc__ if __doc__ else "",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("predictor", choices=sorted(PREDICTORS))
    ap.add_argument("--budgets", nargs="+", default=None)
    ap.add_argument("--steps", nargs="+", default=STEPS, choices=STEPS)
    ap.add_argument("--rtl-lines", type=int, default=0)
    ap.add_argument("--gls-trace", default="fdd*")
    ap.add_argument("--gls-lines", type=int, default=50000)
    ap.add_argument("--clk", default="1.0")
    ap.add_argument("--extra", nargs="+", default=[], metavar="K=V")
    ap.add_argument("--hier", action="store_true")
    ap.add_argument("--rerun", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    try:
        float(args.clk)
    except ValueError:
        ap.error(f"--clk must be a number of nanoseconds, got '{args.clk}'")

    top, mapping = PREDICTORS[args.predictor]
    stim = os.path.join(PROJ_DIR, "stim", args.predictor)
    dirs = sorted(glob.glob(os.path.join(stim, "*__*")), key=lambda d: budget_key(os.path.basename(d)))
    budgets = []
    for d in dirs:
        label = os.path.basename(d).split("__", 1)[0]
        if args.budgets and label not in args.budgets:
            continue
        if not glob.glob(os.path.join(d, "*.golden.txt")):
            continue
        try:
            budgets.append(Budget(args, top, mapping, label, d))
        except (ValueError, KeyError, OSError) as exc:
            log(f"SKIP {label}: {exc}")
    if not budgets:
        sys.exit(f"No usable dumps for '{args.predictor}' in {stim}")

    log(f"project {PROJECT}, predictor {args.predictor} ({top}), clk {args.clk} ns, "
        f"steps {' '.join(args.steps)}{' + syn_hier' if args.hier else ''}")
    for b in budgets:
        log(f"\n=== {b.label}  {b.cid}\n    params {params_str(b.core)}")
        status = {}
        for step in STEPS:
            if step not in args.steps:
                continue
            blocked = [n for n in NEEDS[step]
                       if (status.get(n) or "").startswith("FAILED")
                       or (n not in args.steps and not args.dry_run
                           and (b.load(n) or {}).get("status") != "ok")]
            if blocked:
                status[step] = f"FAILED (needs {', '.join(blocked)})"
            else:
                status[step] = getattr(b, f"run_{step}")()
            log(f"    {step:4s} {status[step]}")
            if step == "syn" and args.hier and not status[step].startswith("FAILED"):
                log(f"    hier {b.run_syn(hier=True)}")

    if args.dry_run:
        return

    rows = [b.row() for b in budgets]
    out_csv = os.path.join(PROJ_DIR, "eval", f"{args.predictor}_results.csv")
    old = []
    if os.path.exists(out_csv):                    # keep budgets not run this time
        with open(out_csv, newline="") as f:
            old = [r for r in csv.DictReader(f) if r.get("budget") not in {x["budget"] for x in rows}]
    allrows = sorted(old + rows, key=lambda r: budget_key(r["budget"]))
    fields = list(dict.fromkeys(k for r in allrows for k in r))
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(allrows)

    def fmt(v, spec):
        try:
            return format(float(v), spec)
        except (TypeError, ValueError):
            return "-"

    log("\n" + "=" * 112)
    log(f"SUMMARY {args.predictor}  (logic only; SRAM energy needs CACTI)   {out_csv}")
    log("=" * 112)
    log(f"{'budget':>7s} {'rtl':>5s} {'MPKBr':>7s} {'area um2':>9s} {'flops':>6s} {'WNS':>9s} "
        f"{'P uW':>8s} {'cyc/br':>7s} {'pJ/br':>7s} {'rd/br':>6s} {'wr/br':>6s}  gls  crit path")
    for r in allrows:
        crit = f"{r.get('crit_start') or '-'} -> {r.get('crit_end') or '-'}"
        log(f"{r['budget']:>7s} {r.get('rtl_pass') or '-':>5s} {fmt(r.get('mpkbr_geomean'), '7.2f')} "
            f"{fmt(r.get('area_um2'), '9.2f')} {fmt(r.get('flops'), '6.0f')} {fmt(r.get('wns'), '9.1f')} "
            f"{fmt((float(r['p_total_w']) * 1e6) if r.get('p_total_w') not in (None, '') else None, '8.2f')} "
            f"{fmt(r.get('cycles_per_cond'), '7.3f')} {fmt(r.get('e_logic_pj_per_cond'), '7.3f')} "
            f"{fmt(r.get('sram_rd_per_cond'), '6.3f')} {fmt(r.get('sram_wr_per_cond'), '6.3f')}  "
            f"{str(r.get('gls_pass') or '-'):4s} {crit}")


if __name__ == "__main__":
    main()