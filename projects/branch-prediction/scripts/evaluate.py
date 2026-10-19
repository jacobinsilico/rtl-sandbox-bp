#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Full-flow evaluation of one predictor across budgets, from its golden
#   dumps under projects/<project>/stim/<predictor>/<budget>__<config_id>/:
#
#     rtl  make sim on every trace's dump: RTL vs CBP5, bit-exact. When a
#          <trace>.state file sits next to the dump (TAGE, hashed perceptron)
#          it is passed as STATE_FILE and every branch's internal state is
#          compared too (as far as the state file goes: a state trace capped
#          at fewer passes than the dump ends the state checks, not the run).
#          --rtl-passes N runs the first N passes of a multi-pass dump
#          (branches_per_pass lines each, from the dump's .json); otherwise
#          --rtl-lines (0 = the whole dump).
#     act  make sim (RTL, no VCD, no state check) from reset to the end of the
#          activity window of the gls step (same --gls-warmup-passes /
#          --gls-window-pct): the window counters, and per bp_sram instance
#          its depth, width, reads, writes and same-cycle read + write counts
#          (TB_SRAM_INST lines), for the whole run and the window. The
#          per-instance sums must equal the bench's TB_SRAM_* and
#          TB_WIN_SRAM_* totals, or the step fails. RTL and netlist run the
#          same cycles, so where a gls record exists its window counters must
#          match too (act_vs_gls in the CSV).
#     syn  make syn of the core (the SRAMs stay outside, in the bench)
#     sta  make post-syn-sta with IO_DELAY_PCT (default 0) and the extra
#          reports of sta_reports.tcl (SDC hook): unclamped worst slack,
#          critical delay = period - worst slack, prediction-path delay, and
#          the period the worst path from an SRAM read port needs
#          (rdata_crit_ps, with the SRAM clock-to-Q placeholder in it)
#     gls  make post-syn-sim VCD=1 on every trace: the netlist checked against
#          the dump again, and its activity. Activity window:
#            --gls-warmup-passes W (> 0): the first W passes warm the
#              predictor up unrecorded; the VCD and the window counters
#              (cycles, conditional branches, SRAM accesses) cover the first
#              --gls-window-pct percent of pass W+1, then the core drains.
#              The bench gets VCD_START = first line of the window and
#              MAX_LINES = its last line.
#            0 (default): the whole run, capped at --gls-lines branches.
#     dpa  make post-syn-dpa on each trace's activity (power per trace)
#
#   Energy per conditional branch is computed over the activity window, at
#   the clock the gate-level run and its DPA ran at (recorded with them):
#     E = P * TB_WIN_CYCLES * T_clk / TB_WIN_COND_BR
#
#   Disk: the VCD is deleted once its DPA passed; the sim/ run directory of a
#   passed rtl step, and of a gls step whose DPA passed, is deleted too (the
#   step's make output stays in eval/). Failed runs keep everything for
#   debugging (an act step's sim/ directory goes like an rtl step's).
#   --keep-vcd / --keep-sim disable the cleanup. A gls step
#   refuses to start below --min-free-gb free space. A DPA that has to run
#   again after its VCD was deleted reruns the gls step.
#
#   Dumps may be deleted after a budget is evaluated (dump_eval.py does):
#   a dump directory that keeps its .json / .log files still counts. Its
#   records stay in the CSV and synthesis / STA still run; rtl, act and gls steps
#   report that the dump is gone instead of running (they never overwrite a
#   finished record). Only budgets with dumps are run unless --budgets
#   names them.
#
#   The PILOT budget (--pilot, default the smallest) runs first, all steps on
#   all traces. If any of its steps fails, this predictor stops, so a broken
#   config is found before hours are spent on the other budgets. Budgets run
#   one after another; a lock (eval/.lock) keeps two evaluate.py /
#   campaign.py runs of this project from overlapping.
#
#   Resumable: each finished step leaves eval/<predictor>/<budget>/<step>.json
#   (status, metrics, the exact command key). A step whose json is ok and
#   whose key is unchanged is skipped; --rerun forces everything. The keys
#   include a hash of the project's rtl/ and tb/ sources and the size and
#   mtime of each dump, so editing the RTL or regenerating a dump reruns what
#   depends on it (edits of the template's flow scripts are not tracked: use
#   --rerun). The CSV is rebuilt from these records on every run.
#
#   RTL parameters come from the dump's metadata, so every step is built with
#   exactly the config that produced the dumps:
#     gshare, g_perceptron  the .json (-D defines; INDEX_FOLD too when set)
#     hashed_perceptron     the .log (PREDICTOR_CONFIG, PREDICTOR_HIST_LENS)
#     tage_cb               the .log (PREDICTOR_CONFIG, PREDICTOR_KNOBS,
#                           PREDICTOR_HIST_LENS); direct-mapped only
#                           (LOGASSOC 0)
#   Two dump directories of one budget (e.g. an old and a new config) are an
#   error: they would share eval/<predictor>/<budget>/.
#
# Output:
#   projects/<project>/eval/<predictor>/<budget>/<step>.{log,json}
#   projects/<project>/eval/<predictor>_results.csv   one row per budget x trace
#   projects/<project>/eval/<predictor>_sram_counts.csv  one row per budget x
#     trace x bp_sram instance (act step): name = <predictor>/<budget>/<instance>
#     as in eval/cacti/arrays.csv, depth, width, rd, wr, rw (same-cycle read
#     + write), rwa (... to the same address), the same in the window
#     (win_*), the window's conditional branches and cycles, and per
#     conditional branch rd_per_cond, wr_per_cond, rw_per_cond
#   a summary table on stdout
#
#   Per conditional branch, per trace (gate-level run, SRAMs excluded):
#     E = P * TB_WIN_CYCLES * T_clk / TB_WIN_COND_BR, for the total power and
#     split into sequential, combinational and leakage
#     SRAM traffic: TB_WIN_SRAM_READS / TB_WIN_COND_BR, TB_WIN_SRAM_WRITES / ...
#   The SRAM structures (PREDICTOR_STRUCT lines of the CBP5 log) are copied
#   into the CSV for the SRAM costing step.
#   campaign.py runs all four predictors through evaluate_predictor().
#
# Usage (after `source sourceme.sh`):
#   /usr/bin/python3 projects/branch-prediction/scripts/evaluate.py PREDICTOR [options]
#     PREDICTOR          gshare | g_perceptron | hashed_perceptron | tage_cb
#     --budgets 1KB 4KB  subset of budgets (default: all with dumps)
#     --pilot 4KB        budget run first; stop if it fails (default: smallest)
#     --steps rtl syn    subset of steps (default: rtl act syn sta gls dpa)
#     --rtl-passes N     passes per RTL check (multi-pass dumps; 0 = use --rtl-lines)
#     --rtl-lines N      branch lines per RTL check (default 0 = whole dump)
#     --gls-warmup-passes W  activity window in pass W+1 (0 = from the start)
#     --gls-window-pct P     window length, percent of a pass (default 1)
#     --gls-lines N      branch lines per gate-level run without a window (20000)
#     --clk NS           clock period (default 3.0)
#     --io-delay-pct P   STA/DPA I/O delay, percent of the period (default 0)
#     --extra K=V ...    extra RTL parameters, e.g. --extra Y_REG=1
#     --hier             also synthesize with KEEP_HIERARCHY=1 (area per module)
#     --keep-vcd         keep the VCDs after DPA
#     --keep-sim         keep the sim/ run directories
#     --min-free-gb G    free space required before a gls step (default 5)
#     --print-params     print each budget's top and PARAMS, then exit
#     --rerun            ignore finished steps
#     --dry-run          print the make commands only
#
#   Examples:
#     /usr/bin/python3 projects/branch-prediction/scripts/evaluate.py tage_cb --print-params
#     /usr/bin/python3 projects/branch-prediction/scripts/evaluate.py tage_cb --steps rtl --rtl-passes 1
#     /usr/bin/python3 projects/branch-prediction/scripts/evaluate.py tage_cb --steps act --budgets 1KB \
#         --clk 0.9 --gls-warmup-passes 5 --gls-window-pct 1
#     /usr/bin/python3 projects/branch-prediction/scripts/evaluate.py hashed_perceptron --budgets 1KB \
#         --clk 0.9 --gls-warmup-passes 5 --gls-window-pct 1
#
#   Adding a predictor: a params function and one entry in PREDICTORS.
#   Report parsing is tolerant: a value that cannot be found is left empty
#   and a WARNING names the report file.
# -----------------------------------------------------------------------------

import argparse
import csv
import fcntl
import glob
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJ_DIR = os.path.dirname(SCRIPT_DIR)
PROJECT = os.path.basename(PROJ_DIR)
REPO = os.environ.get("REPO_HOME") or os.path.dirname(os.path.dirname(PROJ_DIR))
EVAL_DIR = os.path.join(PROJ_DIR, "eval")
SIM_DIR = os.path.join(PROJ_DIR, "sim")
IMP_DIR = os.path.join(PROJ_DIR, "imp")
STA_TCL = os.path.join(SCRIPT_DIR, "sta_reports.tcl")

STEPS = ["rtl", "act", "syn", "sta", "gls", "dpa"]

# Defaults shared by evaluate.py and campaign.py: a campaign and a manual
# run with the same settings produce the same step keys, so they reuse each
# other's records.
DEFAULTS = {
    "budgets": None, "pilot": None, "steps": STEPS, "rtl_lines": 0, "rtl_passes": 0,
    "gls_lines": 20000, "gls_warmup_passes": 0, "gls_window_pct": 1.0,
    "clk": "3.0", "io_delay_pct": 0, "extra": [], "hier": False, "keep_vcd": False,
    "keep_sim": False, "min_free_gb": 5.0, "print_params": False, "rerun": False,
    "dry_run": False,
}


# =============================================================================
# Dump metadata (<stem> = dump path without .golden.txt)
# =============================================================================

def read(path):
    try:
        with open(path, errors="replace") as f:
            return f.read()
    except OSError:
        return ""


def json_defines(stem):
    """-D defines of the CBP5 build, from the dump's .json."""
    return dict(re.findall(r"-D(\w+)=(\S+)", json.load(open(stem + ".json"))["defines"]))


def log_pairs(stem, tag, name=None):
    """KEY VALUE pairs of a '<tag> [name] K V K V ...' line of the dump's .log."""
    pat = rf"^{tag} {re.escape(name)} (.*)$" if name else rf"^{tag} (.*)$"
    m = re.search(pat, read(stem + ".log"), re.M)
    if not m:
        return None
    tok = m.group(1).split()
    return dict(zip(tok[0::2], tok[1::2]))


def hist_lens(stem):
    m = re.search(r"^PREDICTOR_HIST_LENS((?: \d+)+)\s*$", read(stem + ".log"), re.M)
    if not m:
        raise ValueError(f"no PREDICTOR_HIST_LENS line in {stem}.log")
    return [int(x) for x in m.group(1).split()]


def need(d, key, default=None):
    if key in d:
        return int(d[key])
    if default is not None:
        return default
    raise ValueError(f"{key} missing")


# =============================================================================
# Predictors: dump metadata -> RTL parameters of the core
# =============================================================================

def params_gshare(stem):
    d = json_defines(stem)
    ghr, idx, ent = need(d, "GHR_BITS"), need(d, "PHT_INDEX_BITS"), need(d, "PHT_ENTRIES")
    if ent != 1 << idx:
        raise ValueError(f"PHT_ENTRIES {ent} != 2^PHT_INDEX_BITS")
    return {"GHR_BITS": ghr, "INDEX_BITS": idx, "PC_SHIFT": need(d, "PC_SHIFT", 0)}


def params_g_perceptron(stem):
    d = json_defines(stem)
    h, n = need(d, "GHR_LEN"), need(d, "NUM_PERCEPTRONS")
    if not 1 <= h <= 64:
        raise ValueError(f"GHR_LEN {h} outside the RTL range 1..64")
    if n < 2:
        raise ValueError(f"NUM_PERCEPTRONS {n} not supported by the RTL")
    p = {"GHR_LEN": h, "NUM_PERCEPTRONS": n, "WEIGHT_BITS": need(d, "WEIGHT_BITS"),
         "THETA_ALPHA_PCT": need(d, "THETA_ALPHA_PCT", 100),
         "PC_SHIFT": need(d, "PC_SHIFT", 0)}
    if "INDEX_FOLD" in d:                 # g_perceptron_rtl.h dumps
        p["INDEX_FOLD"] = need(d, "INDEX_FOLD")
    return p


def params_hashed_perceptron(stem):
    kv = log_pairs(stem, "PREDICTOR_CONFIG", "hashed_perceptron")
    if kv is None:
        raise ValueError(f"no 'PREDICTOR_CONFIG hashed_perceptron' line in {stem}.log")
    if kv.get("VARIANT") != "HP" or kv.get("HP_HASH") != "1" or "TABLE_LOGS" not in kv:
        raise ValueError("RTL supports plain HP with HP_HASH=1 and TABLE_LOGS/TABLE_WBITS only "
                         f"(VARIANT {kv.get('VARIANT')}, HP_HASH {kv.get('HP_HASH')})")
    n = need(kv, "NUM_TABLES")
    logs = [int(x) for x in kv["TABLE_LOGS"].strip("{}").split(",")]
    wbits = [int(x) for x in kv["TABLE_WBITS"].strip("{}").split(",")]
    hist = hist_lens(stem)
    if n > 8 or not (len(logs) == len(wbits) == len(hist) == n):
        raise ValueError(f"need NUM_TABLES <= 8 and one LOG/WBITS/HIST per table (NUM_TABLES {n})")
    p = {"NUM_TABLES": n}
    p.update({f"T{t}_LOG": logs[t] if t < n else 0 for t in range(8)})
    p.update({f"T{t}_WBITS": wbits[t] if t < n else 0 for t in range(8)})
    p.update({f"T{t}_HIST": hist[t] if t < n else 0 for t in range(8)})
    for rtl_key, log_key in (("BIAS_ENTRIES", "BIAS_ENTRIES"), ("BIAS_WEIGHT_BITS", "BIAS_WEIGHT_BITS"),
                             ("THETA_BITS", "THETA_BITS"), ("TC_BITS", "TC_BITS"),
                             ("HP_FOLDS", "HP_FOLDS"), ("HP_PATH_BITS", "PATH_BITS"),
                             ("HP_PCMIX", "HP_PCMIX"), ("HP_PC_SHIFT", "HP_PC_SHIFT")):
        p[rtl_key] = need(kv, log_key)
    return p


# tage_cb.h knobs the RTL hard-codes (checked when PREDICTOR_KNOBS is logged)
TAGE_FIXED = {"CWIDTH": 3, "UWIDTH": 2, "BIMWIDTH": 3, "HYSTSHIFT": 1, "MAXBR": 4,
              "NBREADPERTABLE": 4, "CB_ADJACENT": 1, "CB_SHARED": 1, "CB_FILTERALLOC": 1,
              "CB_FORCEU": 1, "CB_PROTECTU": 1, "CB_UPDATEALT": 1, "CB_RANDINIT": 0}


def params_tage_cb(stem):
    kv = log_pairs(stem, "PREDICTOR_CONFIG", "tage_cb")
    if kv is None:
        raise ValueError(f"no 'PREDICTOR_CONFIG tage_cb' line in {stem}.log (CB_SC must be 0)")
    for key, want in (("CB_SC", 0), ("AHEAD", 0), ("CB_OPTTAGE", 1), ("LOGASSOC", 0)):
        if need(kv, key) != want:
            raise ValueError(f"RTL needs {key}={want}, the dump has {kv[key]}")
    knobs = log_pairs(stem, "PREDICTOR_KNOBS") or {}
    bad = [f"{k}={knobs[k]}" for k, v in TAGE_FIXED.items() if k in knobs and int(knobs[k]) != v]
    if bad:
        raise ValueError(f"RTL hard-codes {TAGE_FIXED}; the dump has {' '.join(bad)}")
    n = need(kv, "NHIST")
    hist = hist_lens(stem)
    if len(hist) != n or n > 14:
        raise ValueError(f"need NHIST <= 14 and one history length per table (NHIST {n})")
    p = {"NHIST": n, "LOGT": need(kv, "LOGT"), "LOGB": need(kv, "LOGB"),
         "TBITS": need(kv, "TBITS"), "CB_LMP": need(kv, "CB_LMP"),
         "ILEN_CAP": need(kv, "CB_ILEN_CAP")}
    p.update({f"T{t}_HIST": hist[t - 1] if t <= n else 0 for t in range(1, 15)})
    return p


# stim directory name -> RTL core (= synthesis top = bench DUT), parameters,
# and whether its bench takes a STATE_FILE
PREDICTORS = {
    "gshare":            {"top": "bp_gshare_core", "params": params_gshare,            "state": False},
    "g_perceptron":      {"top": "bp_gp_core",     "params": params_g_perceptron,      "state": False},
    "hashed_perceptron": {"top": "bp_hp_core",     "params": params_hashed_perceptron, "state": True},
    "tage_cb":           {"top": "bp_tage_core",   "params": params_tage_cb,           "state": True},
}


# =============================================================================
# Helpers
# =============================================================================

def log(msg=""):
    print(msg, flush=True)


def params_str(p):
    return " ".join(f"{k}={v}" for k, v in p.items())


def budget_kb(label):
    m = re.match(r"(\d+)KB$", label)
    return int(m.group(1)) if m else None


def budget_key(label):
    kb = budget_kb(label)
    return (kb if kb is not None else 1 << 30, label)


def first(patterns, text, cast=float):
    for pat in patterns:
        m = re.search(pat, text, re.MULTILINE | re.IGNORECASE)
        if m:
            try:
                return cast(m.group(1))
            except ValueError:
                continue
    return None


def warn_missing(step, metrics, keys, files):
    missing = [k for k in keys if metrics.get(k) is None]
    if missing:
        where = ", ".join(os.path.relpath(f, REPO) for f in files) or "(no report files)"
        log(f"    WARNING {step}: could not parse {', '.join(missing)} from {where}")


def tb_metrics(text):
    keys = ["TB_LINES", "TB_COND_BR", "TB_UNCOND_BR", "TB_RTL_MISPRED", "TB_REF_MISPRED",
            "TB_CYCLES", "TB_SRAM_READS", "TB_SRAM_WRITES", "TB_STATE_LINES", "TB_STATE_CKSUMS",
            "TB_WIN_START", "TB_WIN_LINES", "TB_WIN_COND_BR", "TB_WIN_CYCLES",
            "TB_WIN_SRAM_READS", "TB_WIN_SRAM_WRITES"]
    m = {k.lower()[3:]: first([rf"^{k}\s*:\s*(\d+)"], text, int) for k in keys}
    m["passed"] = "PASSED" in text
    inst = sram_instances(text)
    if inst:
        m["sram_inst"] = inst
    return m


SRAM_INST_KEYS = ("DEPTH", "WIDTH", "RD", "WR", "RW", "RWA",
                  "WIN_RD", "WIN_WR", "WIN_RW", "WIN_RWA")


def inst_name(path):
    """Bench-relative instance path of a %m name:
    'TOP.tb_bp_hp_core.g_wt[0].i_wt' -> 'g_wt[0].i_wt' (sram_arrays.py's)."""
    p = path.replace("__BRA__", "[").replace("__KET__", "]").replace("__DOT__", ".")
    parts = p.split(".")
    for i, s in enumerate(parts):
        if s.startswith("tb_"):
            return ".".join(parts[i + 1:])
    return p[4:] if p.startswith("TOP.") else p


def sram_instances(text):
    """TB_SRAM_INST lines of bp_sram.sv -> {instance: {depth, width, rd, ...}}."""
    out = {}
    for path, rest in re.findall(r"^TB_SRAM_INST\s+(\S+)\s+(.*)$", text, re.MULTILINE):
        tok = rest.split()
        kv = dict(zip(tok[0::2], tok[1::2]))
        try:
            out[inst_name(path)] = {k.lower(): int(kv[k]) for k in SRAM_INST_KEYS}
        except (KeyError, ValueError):
            continue
    return out


def sram_inst_check(m):
    """None if the per-instance counts add up to the bench's totals, else why not."""
    inst = m.get("sram_inst")
    if not inst:
        return "no TB_SRAM_INST lines (is the per-instance bp_sram.sv compiled?)"
    bad = []
    for k_inst, k_tb in (("rd", "sram_reads"), ("wr", "sram_writes"),
                         ("win_rd", "win_sram_reads"), ("win_wr", "win_sram_writes")):
        tot = sum(v[k_inst] for v in inst.values())
        if m.get(k_tb) is not None and tot != m[k_tb]:
            bad.append(f"sum {k_inst} {tot} != TB_{k_tb.upper()} {m[k_tb]}")
    return "; ".join(bad) or None


def file_sig(path):
    """Size and mtime of a file: a regenerated dump changes its step keys."""
    try:
        st = os.stat(path)
        return f"{st.st_size}:{st.st_mtime_ns}"
    except OSError:
        return "missing"


def source_hash():
    """Hash of the project's RTL and testbench sources."""
    h = hashlib.sha1()
    for pat in ("rtl/*.sv", "rtl/*.svh", "rtl/*.v", "tb/*.sv", "tb/*.svh"):
        for f in sorted(glob.glob(os.path.join(PROJ_DIR, pat))):
            h.update(os.path.relpath(f, PROJ_DIR).encode())
            with open(f, "rb") as fh:
                h.update(fh.read())
    return h.hexdigest()[:12]


def remove_tree(path):
    if os.path.isdir(path):
        shutil.rmtree(path, ignore_errors=True)


def free_gb(path):
    return shutil.disk_usage(path).free / 1e9


class Lock:
    """Exclusive lock of this project's evaluation (eval/.lock)."""

    def __enter__(self):
        os.makedirs(EVAL_DIR, exist_ok=True)
        self.path = os.path.join(EVAL_DIR, ".lock")
        self.f = open(self.path, "a+")
        try:
            fcntl.flock(self.f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.f.seek(0)
            sys.exit(f"Another evaluate.py / campaign.py run holds {self.path} "
                     f"({self.f.read().strip() or 'pid unknown'}); runs must not overlap.")
        self.f.seek(0)
        self.f.truncate()
        self.f.write(f"pid {os.getpid()} since {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        self.f.flush()
        return self

    def __exit__(self, *exc):
        fcntl.flock(self.f, fcntl.LOCK_UN)
        self.f.close()
        return False


# =============================================================================
# Report parsers
# =============================================================================

ASAP7_CELL = re.compile(r"^\\?[A-Za-z]\w*_ASAP7_\w+$")


def top_section(text, top):
    """The '=== <top> ===' section of a Yosys stat report (whole text if none)."""
    parts = re.split(r"^\s*===\s*\\?(\S+)\s*===\s*$", text, flags=re.MULTILINE)
    if len(parts) < 3:
        return text
    sections = dict(zip(parts[1::2], parts[2::2]))
    return sections.get(top, text)


def parse_area(out_dir, top):
    """Area, cell count and flip-flop count of a flat synthesis run. The
    cells are counted from the per-cell-type lines of report/area.rpt, in
    whatever column order the Yosys version prints them: a line with one
    ASAP7 cell name and an integer is that many cells of that type."""
    path = os.path.join(IMP_DIR, out_dir, "report", "area.rpt")
    text = top_section(read(path), top)
    m = {"area_um2": first([r"Chip area for (?:top )?module\s+'?\\?[^':]*'?\s*:\s*([\d.]+)",
                            r"Chip area\s*:\s*([\d.]+)",
                            r"Design area\s+([\d.]+)"], text)}
    cells = flops = 0
    seen = False
    for line in text.splitlines():
        tok = line.split()
        names = [t for t in tok if ASAP7_CELL.match(t)]
        ints = [t for t in tok if t.isdigit()]
        if len(names) != 1 or not ints:
            continue
        n = int(ints[0])
        seen = True
        cells += n
        if re.search(r"DFF|SDF", names[0], re.IGNORECASE):
            flops += n
    m["cells"] = cells if seen else first([r"Number of cells\s*:?\s*(\d+)",
                                           r"^\s*(\d+)\s+(?:[\d.]+\s+)?cells\s*$"], text, int)
    m["flops"] = flops if seen else None
    warn_missing("syn", m, ["area_um2", "cells", "flops"], [path])
    return m


def parse_area_hier(out_dir):
    text = read(os.path.join(IMP_DIR, out_dir, "report", "area.rpt"))
    return {name: float(a) for name, a in
            re.findall(r"Chip area for (?:top )?module\s+'?\\?([^':\s]+)'?\s*:\s*([\d.]+)", text)}


def parse_sta(out_dir, clk_ns):
    rep = os.path.join(IMP_DIR, out_dir, "report")
    files = {n: os.path.join(rep, n + ".rpt")
             for n in ("wns", "tns", "worst_slack", "worst_path", "pred_path", "rdata_path")}
    txt = {n: read(p) for n, p in files.items()}
    m = {
        "wns": first([r"^\s*wns\s+(?:max\s+)?(-?[\d.]+)"], txt["wns"]),
        "tns": first([r"^\s*tns\s+(?:max\s+)?(-?[\d.]+)"], txt["tns"]),
        "worst_slack_ps": first([r"worst slack\s+(?:max\s+)?(-?[\d.]+)"], txt["worst_slack"]),
        "crit_start": first([r"Startpoint:\s*(\S+)"], txt["worst_path"], str),
        "crit_end": first([r"Endpoint:\s*(\S+)"], txt["worst_path"], str),
        "pred_path_ps": first([r"^\s*(-?[\d.]+)\s+data arrival time"], txt["pred_path"]),
        # worst path from an SRAM read port (sta_reports.tcl): its arrival
        # includes the SRAM clock-to-Q placeholder
        "rdata_slack_ps": first([r"^\s*(-?[\d.]+)\s+slack"], txt["rdata_path"]),
        "rdata_end": first([r"Endpoint:\s*(\S+)"], txt["rdata_path"], str),
    }
    # the period that path alone needs: arrival + setup (or + output delay)
    m["rdata_crit_ps"] = (round(float(clk_ns) * 1000.0 - m["rdata_slack_ps"], 2)
                          if m["rdata_slack_ps"] is not None else None)
    m["crit_delay_ps"] = (round(float(clk_ns) * 1000.0 - m["worst_slack_ps"], 2)
                          if m["worst_slack_ps"] is not None else None)
    warn_missing("sta", m, ["worst_slack_ps", "crit_start", "pred_path_ps"],
                 [files["worst_slack"], files["worst_path"], files["pred_path"]])
    return m


def parse_power(out_dir):
    rep = os.path.join(IMP_DIR, out_dir, "report")
    files = sorted(glob.glob(os.path.join(rep, "*power*")))
    text = "\n".join(read(f) for f in files)
    num = r"([\d.]+(?:[eE][+-]?\d+)?)"
    m = {}
    for group in ("Sequential", "Combinational", "Clock", "Total"):
        g = re.search(rf"^\s*{group}\s+{num}\s+{num}\s+{num}\s+{num}", text, re.MULTILINE)
        m[f"p_{group.lower()}_w"] = float(g.group(4)) if g else None
        if group == "Total" and g:
            m["p_internal_w"], m["p_switching_w"], m["p_leakage_w"] = \
                float(g.group(1)), float(g.group(2)), float(g.group(3))
    # both reports start with the same two summary lines ("vcd N" and
    # "unannotated M"), then list the pins: read the counts off those lines
    ann = read(os.path.join(rep, "vcd_annotated.rpt")) or read(os.path.join(rep, "vcd_unannotated.rpt"))
    m["vcd_annotated"] = first([r"^\s*vcd\s+(\d+)\s*$"], ann, int)
    m["vcd_unannotated"] = first([r"^\s*unannotated\s+(\d+)\s*$"], ann, int)
    warn_missing("dpa", m, ["p_total_w"], files)
    return m


# =============================================================================
# One budget: its dumps, its steps and their records
# =============================================================================

# =============================================================================
# VCD post-processing for the power analysis
# =============================================================================

VCD_MAX_BUS = 64    # OpenSTA's VCD reader holds a bus value in a 64-bit integer


def _vcd_ids():
    """Fresh VCD identifiers (printable ASCII 33..126), shortest first."""
    n = 0
    while True:
        k, s = n, ""
        while True:
            s = chr(33 + k % 94) + s
            k = k // 94 - 1
            if k < 0:
                break
        yield s
        n += 1


def split_wide_vcd(path, max_bus=VCD_MAX_BUS):
    """Rewrite a VCD so that no bus is wider than max_bus: each wider variable
    becomes one 1-bit variable per element (name [i]), and each of its value
    changes becomes changes of only the bits that changed. The switching
    activity per bit is unchanged; only the representation differs. In place
    (via a temporary file); returns the number of variables split (0: the file
    is left alone)."""
    wide, defs = {}, []
    with open(path, errors="replace") as f:
        for line in f:
            if line.startswith("$enddefinitions") or "$enddefinitions" in line:
                break
            t = line.split()
            if len(t) >= 5 and t[0] == "$var" and t[2].isdigit() and int(t[2]) > max_bus:
                wide.setdefault(t[3], int(t[2]))
    if not wide:
        return 0
    used, gen = set(), _vcd_ids()
    with open(path, errors="replace") as f:          # every identifier in use
        for line in f:
            if "$enddefinitions" in line:
                break
            t = line.split()
            if len(t) >= 4 and t[0] == "$var":
                used.add(t[3])

    def fresh():
        while True:
            i = next(gen)
            if i not in used:
                used.add(i)
                return i

    bits = {}                                        # wide id -> [new id per character]
    prev = {}                                        # wide id -> last full value
    tmp = path + ".split.tmp"
    with open(path, errors="replace") as f, open(tmp, "w", buffering=1 << 22) as o:
        for line in f:                               # header
            t = line.split()
            if len(t) >= 5 and t[0] == "$var" and t[3] in wide:
                vid, width = t[3], wide[t[3]]
                name, rng = t[4], (t[5] if len(t) > 6 and t[5].startswith("[") else "")
                if not rng and "[" in name:
                    name, rng = name[:name.index("[")], name[name.index("["):]
                msb, lsb = width - 1, 0
                if rng.startswith("[") and ":" in rng:
                    msb, lsb = (int(x) for x in rng.strip("[]").split(":"))
                ids = bits.setdefault(vid, [fresh() for _ in range(width)])
                for c in range(width):               # character c = bit msb -/+ c
                    idx = msb - c if msb >= lsb else msb + c
                    o.write(f"$var {t[1]} 1 {ids[c]} {name} [{idx}] $end\n")
                continue
            o.write(line)
            if "$enddefinitions" in line:
                break
        write = o.write
        for line in f:                               # value changes
            if line[0] not in "bB":
                write(line)
                continue
            val, _, vid = line[1:].rstrip("\n").rpartition(" ")
            ids = bits.get(vid.strip())
            if ids is None:
                write(line)
                continue
            vid = vid.strip()
            width = len(ids)
            if len(val) < width:                     # IEEE left extension
                val = (val[0] if val[0] in "xXzZ" else "0") * (width - len(val)) + val
            old = prev.get(vid)
            prev[vid] = val
            if old is None:
                write("".join([val[c] + ids[c] + "\n" for c in range(width)]))
            elif "x" in val or "z" in val or "x" in old or "z" in old \
                    or "X" in val or "Z" in val or "X" in old or "Z" in old:
                write("".join([val[c] + ids[c] + "\n" for c in range(width) if val[c] != old[c]]))
            else:                                    # 0/1 only: walk the changed bits
                d = int(val, 2) ^ int(old, 2)
                out = []
                while d:
                    low = d & -d
                    c = width - low.bit_length()     # character position of that bit
                    out.append(val[c] + ids[c] + "\n")
                    d ^= low
                write("".join(out))
    os.replace(tmp, path)
    return len(wide)


def make(args, target, variables, logfile):
    cmd = ["make", "-C", REPO, target, f"PROJECT={PROJECT}"] + [f"{k}={v}" for k, v in variables]
    if args.dry_run:
        log("    " + " ".join(cmd))
        return 0, ""
    with open(logfile, "w") as f:
        rc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT).returncode
    return rc, read(logfile)


def quoted(name, path):
    return f'{name}="{path}"'           # Verilator wants the quotes (no spaces in paths)


class Budget:
    def __init__(self, args, spec, label, dump_dir, src_hash):
        self.args, self.spec, self.top, self.label = args, spec, spec["top"], label
        self.kb = budget_kb(label)
        self.cid = os.path.basename(dump_dir).split("__", 1)[1]
        self.src = src_hash
        # A trace is known from its .json + .log pair (written with every valid
        # dump and kept when dump_eval.py deletes the dump itself); any other
        # .json in the directory (e.g. a dump_eval.py marker) is not a trace.
        metas = dump_metas(dump_dir)
        # trace short name (fdd_su_v1_0) -> dump stem (path without .golden.txt)
        self.traces = {os.path.basename(m).split(".")[0]: m[:-len(".json")] for m in metas}
        if not self.traces:
            raise ValueError("no dump metadata (.json) in the directory")
        self.meta = {}
        for t, stem in self.traces.items():
            try:
                self.meta[t] = json.load(open(stem + ".json"))
            except (OSError, ValueError):
                self.meta[t] = {}
        stem0 = next(iter(self.traces.values()))
        self.core = spec["params"](stem0)
        for kv in args.extra:
            k, _, v = kv.partition("=")
            self.core[k] = v
        cbp_log = read(stem0 + ".log")
        self.size_bits = first([r"PREDICTOR_CONFIG.*\bSIZE_BITS (\d+)"], cbp_log, int)
        self.structs = ";".join(f"{n}:{e}x{w}" for n, e, w in re.findall(
            r"^PREDICTOR_STRUCT (\S+) ENTRIES (\d+) WIDTH (\d+)", cbp_log, re.MULTILINE))
        self.edir = os.path.join(EVAL_DIR, args.predictor, label)
        self.tag = f"{args.predictor}_{label}"

    # ---- dumps and their line ranges ----
    def has_dump(self, trace):
        return os.path.exists(self.traces[trace] + ".golden.txt")

    def has_dumps(self):
        return any(self.has_dump(t) for t in self.traces)

    def bpp(self, trace):
        v = self.meta[trace].get("branches_per_pass")
        if not v:
            raise ValueError(f"{trace}: no branches_per_pass in the dump's .json "
                             f"(a dump of 01_golden_dump.py --trace-kind long)")
        return int(v)

    def rtl_lines(self, trace):
        """Branch lines of the RTL check (MAX_LINES; 0 = whole dump)."""
        a = self.args
        if a.rtl_passes:
            n = a.rtl_passes * self.bpp(trace)
            total = self.meta[trace].get("lines")
            if total is not None and n > total:
                raise ValueError(f"{trace}: --rtl-passes {a.rtl_passes} needs {n} lines, "
                                 f"the dump has {total}")
            return n
        return a.rtl_lines

    def gls_window(self, trace):
        """-> (VCD_START, MAX_LINES) of the gate-level run: (0, --gls-lines)
        without a window; else the first --gls-window-pct % of pass W+1."""
        a = self.args
        if not a.gls_warmup_passes:
            return 0, a.gls_lines
        bpp = self.bpp(trace)
        start = a.gls_warmup_passes * bpp + 1
        end = start - 1 + max(1, int(round(bpp * a.gls_window_pct / 100.0)))
        total = self.meta[trace].get("lines")
        if total is not None and end > total:
            raise ValueError(f"{trace}: window ends at line {end}, the dump has {total} "
                             f"(needs {a.gls_warmup_passes + 1} passes)")
        return start, end

    # ---- run directories ----
    def sim_dir(self, kind, trace):
        return os.path.join(SIM_DIR, f"{kind}_{self.tag}_{trace}")

    def vcds(self, trace):
        return glob.glob(os.path.join(self.sim_dir("gls", trace), "**", "*.vcd"), recursive=True)

    # ---- bookkeeping ----
    def load(self, step):
        try:
            return json.load(open(os.path.join(self.edir, f"{step}.json")))
        except (OSError, ValueError):
            return None

    def ok(self, step):
        return (self.load(step) or {}).get("status") == "ok"

    def done(self, step, key=None):
        rec = self.load(step)
        try:
            key = self.key(step) if key is None else key
        except ValueError:
            return False
        if rec is not None and step.startswith("dpa_") \
                and (rec.get("metrics") or {}).get("p_total_w") is None:
            return False                     # an older flow accepted a DPA without power
        return rec is not None and rec.get("status") == "ok" and rec.get("key") == key \
            and not self.args.rerun

    def save(self, step, key, ok, metrics, t0):
        if not self.args.dry_run:
            os.makedirs(self.edir, exist_ok=True)
            with open(os.path.join(self.edir, f"{step}.json"), "w") as f:
                json.dump({"step": step, "key": key, "status": "ok" if ok else "failed",
                           "metrics": metrics, "time_s": round(time.time() - t0, 1)}, f, indent=1)

    def key(self, step):
        """Command key of a step: a step reruns when this changes."""
        a, p = self.args, params_str(self.core)
        if step.startswith("rtl_"):
            stem = self.traces[step[4:]]
            state = stem + ".state" if self.spec["state"] and os.path.exists(stem + ".state") else ""
            return (f"{self.top}|{p}|src {self.src}|dump {file_sig(stem + '.golden.txt')}|"
                    f"{self.rtl_lines(step[4:])}|{a.clk}|{state} {file_sig(state) if state else ''}")
        if step.startswith("act_"):
            stem = self.traces[step[4:]]
            start, end = self.gls_window(step[4:])
            return (f"{self.top}|{p}|src {self.src}|dump {file_sig(stem + '.golden.txt')}|"
                    f"act {start}-{end}|{a.clk}")
        if step in ("syn", "syn_hier"):
            return f"{self.top}|{p}|src {self.src}|{a.clk}|{step == 'syn_hier'}"
        syn_key = (self.load("syn") or {}).get("key")
        if step == "sta":
            sdc = file_sig(STA_TCL) if os.path.exists(STA_TCL) else "none"
            return f"{syn_key}|{a.clk}|io {a.io_delay_pct}|sdc {sdc}"
        if step.startswith("gls_"):
            stem = self.traces[step[4:]]
            start, end = self.gls_window(step[4:])
            return f"{syn_key}|{stem}|dump {file_sig(stem + '.golden.txt')}|{start}-{end}"
        if step.startswith("dpa_"):
            return f"{(self.load('gls_' + step[4:]) or {}).get('key')}|io {a.io_delay_pct}"
        raise KeyError(step)

    def run_step(self, step, force=False):
        """Runs one step unless finished; returns a status string."""
        t0, a, kind = time.time(), self.args, step.split("_")[0]
        trace = step.split("_", 1)[1] if step.startswith(("rtl_", "act_", "gls_", "dpa_")) else None
        if kind in ("rtl", "act", "gls") and not self.has_dump(trace):
            # never overwrite a finished record because its dump was deleted
            rec = self.load(step) or {}
            return ("FAILED (dump deleted; regenerate it with dump_eval.py)"
                    + (" -- the earlier record stays" if rec.get("status") == "ok" else ""))
        try:
            key = self.key(step)
        except ValueError as exc:
            return f"FAILED ({exc})"      # a wrong option: no record is overwritten
        if not force and self.done(step, key):
            return "ok (kept)"
        logf = os.path.join(self.edir, f"{step}.log")
        if not a.dry_run:
            os.makedirs(self.edir, exist_ok=True)
        base = [("TOP_LEVEL", self.top), ("CLK_PERIOD_NS", a.clk)]
        p = params_str(self.core)

        if kind == "rtl":
            stem = self.traces[trace]
            par = f"{p} MAX_LINES={self.rtl_lines(trace)} {quoted('STIM_FILE', stem + '.golden.txt')}"
            if self.spec["state"] and os.path.exists(stem + ".state"):
                par += " " + quoted("STATE_FILE", stem + ".state")
            rc, text = make(a, "sim", base + [("OUT_DIR", f"sim_{self.tag}_{trace}"),
                                              ("PARAMS", par)], logf)
            m = tb_metrics(text)
            m["max_lines"] = self.rtl_lines(trace)
            ok = rc == 0 and m["passed"]
            if ok and not a.keep_sim and not a.dry_run:
                remove_tree(self.sim_dir("sim", trace))
        elif kind == "act":
            stem = self.traces[trace]
            start, end = self.gls_window(trace)
            par = f"{p} MAX_LINES={end} {quoted('STIM_FILE', stem + '.golden.txt')}"
            if start:
                par += f" VCD_START={start}"
            rc, text = make(a, "sim", base + [("OUT_DIR", f"act_{self.tag}_{trace}"),
                                              ("PARAMS", par)], logf)
            m = tb_metrics(text)
            m.update(clk_ns=a.clk, vcd_start=start, max_lines=end)
            ok = rc == 0 and m["passed"]
            err = None if a.dry_run else sram_inst_check(m)
            if ok and err:
                ok, m["error"] = False, err
            if ok and start and m.get("win_start") not in (None, start):
                ok = False
                m["error"] = f"bench window starts at line {m.get('win_start')}, expected {start}"
            if ok and not a.keep_sim and not a.dry_run:
                remove_tree(self.sim_dir("act", trace))
        elif kind == "syn":
            out = f"syn_{self.tag}" + ("_hier" if step == "syn_hier" else "")
            var = base + [("OUT_DIR", out), ("PARAMS", p)]
            if step == "syn_hier":
                var.append(("KEEP_HIERARCHY", 1))
            rc, _ = make(a, "syn", var, logf)
            ok = rc == 0
            m = ((parse_area_hier(out) if step == "syn_hier" else parse_area(out, self.top))
                 if ok and not a.dry_run else {})
            m["clk_ns"] = a.clk
        elif kind == "sta":
            var = base + [("OUT_DIR", f"sta_{self.tag}"), ("NETLIST_DIR", f"syn_{self.tag}"),
                          ("IO_DELAY_PCT", a.io_delay_pct)]
            if os.path.exists(STA_TCL):
                var.append(("SDC", os.path.relpath(STA_TCL, REPO)))
            rc, _ = make(a, "post-syn-sta", var, logf)
            ok = rc == 0
            m = parse_sta(f"sta_{self.tag}", a.clk) if ok and not a.dry_run else {}
            m["clk_ns"] = a.clk
        elif kind == "gls":
            if not a.dry_run and free_gb(PROJ_DIR) < a.min_free_gb:
                msg = (f"only {free_gb(PROJ_DIR):.1f} GB free (< --min-free-gb {a.min_free_gb}); "
                       f"not starting a VCD run")
                self.save(step, key, False, {"error": msg}, t0)
                return f"FAILED ({msg})"
            stem = self.traces[trace]
            start, end = self.gls_window(trace)
            if not a.dry_run:
                remove_tree(self.sim_dir("gls", trace))     # no stale VCD from an older run
            par = f"{p} MAX_LINES={end} {quoted('STIM_FILE', stem + '.golden.txt')}"
            if start:
                par += f" VCD_START={start}"
            rc, text = make(a, "post-syn-sim", base + [
                ("OUT_DIR", f"gls_{self.tag}_{trace}"), ("NETLIST_DIR", f"syn_{self.tag}"),
                ("VCD", 1), ("PARAMS", par)], logf)
            m = tb_metrics(text)
            m.update(clk_ns=a.clk, vcd_start=start, max_lines=end)
            m["vcd_mb"] = round(sum(os.path.getsize(v) for v in self.vcds(trace)) / 1e6, 1) \
                if not a.dry_run else None
            ok = rc == 0 and m["passed"]
            if ok and start and m.get("win_start") not in (None, start):
                ok = False
                m["error"] = f"bench window starts at line {m.get('win_start')}, expected {start}"
        else:  # dpa
            var = base + [("OUT_DIR", f"dpa_{self.tag}_{trace}"), ("NETLIST_DIR", f"syn_{self.tag}"),
                          ("VCD_DIR", f"gls_{self.tag}_{trace}"), ("IO_DELAY_PCT", a.io_delay_pct)]
            n_split = 0
            if not a.dry_run:                        # buses > 64 bits break OpenSTA's read_vcd
                n_split = sum(split_wide_vcd(v) for v in self.vcds(trace))
            rc, text = make(a, "post-syn-dpa", var, logf)
            ok = rc == 0
            m = parse_power(f"dpa_{self.tag}_{trace}") if ok and not a.dry_run else {}
            m["clk_ns"] = a.clk
            m["vcd_split_vars"] = n_split
            if ok and not a.dry_run and m.get("p_total_w") is None:
                # make can exit 0 although the power tool died (its exit status
                # is lost in the pipe to tee): no power, no ok -- the VCD stays
                ok = False
                errs = [l.strip() for l in (text or "").splitlines() if l.startswith("Error")]
                m["error"] = errs[0] if errs else "no power in the report (the power tool did not finish?)"
            if ok and not a.dry_run and not a.keep_vcd:
                if a.keep_sim:
                    for v in self.vcds(trace):
                        os.remove(v)
                else:
                    remove_tree(self.sim_dir("gls", trace))

        if a.dry_run:
            return "dry"
        self.save(step, key, ok, m, t0)
        if kind in ("rtl", "act", "gls"):
            extra = f"  state lines={m.get('state_lines')}" if m.get("state_lines") else ""
            extra += (f"  window lines {m.get('win_start')}+{m.get('win_lines')}"
                      if kind in ("act", "gls") and m.get("vcd_start") else "")
            extra += f"  {len(m['sram_inst'])} SRAMs" if kind == "act" and m.get("sram_inst") else ""
            extra += f"  vcd {m['vcd_mb']} MB" if m.get("vcd_mb") else ""
            extra += f"  {m['error']}" if m.get("error") else ""
            return (f"{'ok' if ok else 'FAILED'}  lines={m.get('lines')} cond={m.get('cond_br')} "
                    f"mispred={m.get('rtl_mispred')}{extra}" + ("" if ok else f"  (see {logf})"))
        if ok:
            return "ok"
        return f"FAILED ({m['error']}, see {logf})" if m.get("error") else f"FAILED (exit {rc}, see {logf})"

    def run(self):
        """All requested steps of this budget; True if all of them are ok."""
        a, all_ok = self.args, True
        order = []
        if "rtl" in a.steps:
            order += [f"rtl_{t}" for t in self.traces]
        if "act" in a.steps:
            order += [f"act_{t}" for t in self.traces]
        if "syn" in a.steps:
            order += ["syn"] + (["syn_hier"] if a.hier else [])
        if "sta" in a.steps:
            order += ["sta"]
        for t in self.traces:
            order += [f"gls_{t}"] if "gls" in a.steps else []
            order += [f"dpa_{t}"] if "dpa" in a.steps else []
        for step in order:
            kind = step.split("_")[0]
            dep = {"sta": "syn", "gls": "syn", "dpa": "gls_" + step[4:]}.get(kind)
            if dep and not a.dry_run and not self.ok(dep):
                status = f"FAILED (needs {dep})"
            elif kind == "dpa" and not a.dry_run and not self.done(step) and not self.vcds(step[4:]):
                # its VCD was deleted after an earlier DPA: regenerate it first
                gls = "gls_" + step[4:]
                gstat = self.run_step(gls, force=True)
                log(f"    {gls:24s} {gstat}  (VCD regenerated for DPA)")
                status = self.run_step(step) if gstat.startswith("ok") else f"FAILED (needs {gls})"
            else:
                status = self.run_step(step)
            log(f"    {step:24s} {status}")
            all_ok &= status.startswith(("ok", "dry"))
        return all_ok

    # ---- CSV rows: one per trace ----
    def rows(self):
        def met(step):
            rec = self.load(step) or {}
            return rec.get("metrics", {}) if rec.get("status") == "ok" else {}
        syn, sta, hier = met("syn"), met("sta"), met("syn_hier")
        out = []
        for trace in self.traces:
            rtl, gls, dpa = met(f"rtl_{trace}"), met(f"gls_{trace}"), met(f"dpa_{trace}")
            r = {"predictor": self.args.predictor, "budget": self.label, "budget_kb": self.kb,
                 "trace": trace, "config_id": self.cid, "rtl_params": params_str(self.core),
                 "size_bits": self.size_bits, "sram_structs": self.structs,
                 "clk_ns": sta.get("clk_ns", self.args.clk),
                 "io_delay_pct": self.args.io_delay_pct,
                 "rtl_pass": rtl.get("passed", ""), "rtl_lines": rtl.get("lines", ""),
                 "rtl_state_lines": rtl.get("state_lines", "")}
            cond = rtl.get("cond_br")
            r["mpkbr"] = round(1000.0 * rtl["rtl_mispred"] / cond, 4) if cond else ""
            for k in ("area_um2", "cells", "flops"):
                r[k] = syn.get(k, "")
            for k in ("crit_delay_ps", "pred_path_ps", "worst_slack_ps", "wns", "tns",
                      "crit_start", "crit_end", "rdata_crit_ps", "rdata_end"):
                r[k] = sta.get(k, "")
            r["gls_pass"] = gls.get("passed", "")
            r["gls_clk_ns"] = gls.get("clk_ns", "")
            r["gls_win_start"] = gls.get("win_start", "")
            r["gls_win_lines"] = gls.get("win_lines", "")
            for k in ("cond_br", "cycles", "sram_reads", "sram_writes"):
                r[f"gls_{k}"] = gls.get(k, "")
            for k in ("p_total_w", "p_sequential_w", "p_combinational_w", "p_clock_w",
                      "p_internal_w", "p_switching_w", "p_leakage_w",
                      "vcd_annotated", "vcd_unannotated"):
                r[k] = dpa.get(k, "")
            # the activity window (the whole run for benches without one)
            gc = gls.get("win_cond_br") or gls.get("cond_br")
            cyc = gls.get("win_cycles") or gls.get("cycles")
            rd = gls.get("win_sram_reads") if gls.get("win_cond_br") else gls.get("sram_reads")
            wr = gls.get("win_sram_writes") if gls.get("win_cond_br") else gls.get("sram_writes")
            if gc and cyc:
                r["cycles_per_cond"] = round(cyc / gc, 4)
                r["sram_rd_per_cond"] = round((rd or 0) / gc, 4)
                r["sram_wr_per_cond"] = round((wr or 0) / gc, 4)
                # power was measured at the clock of the gls / dpa run, not --clk
                clk = float(dpa.get("clk_ns") or gls.get("clk_ns") or self.args.clk)
                t_per_cond = cyc * clk * 1e-9 / gc                     # s per branch
                for src, dst in (("p_total_w", "e_logic_pj_per_cond"),
                                 ("p_sequential_w", "e_seq_pj_per_cond"),
                                 ("p_combinational_w", "e_comb_pj_per_cond"),
                                 ("p_leakage_w", "e_leak_pj_per_cond")):
                    if dpa.get(src) is not None:
                        r[dst] = round(dpa[src] * t_per_cond * 1e12, 4)
            # RTL activity run (act): window counters, and whether they equal the
            # gate-level run's (same design, same cycles)
            act = met(f"act_{trace}")
            r["act_pass"] = act.get("passed", "")
            r["act_win_cond_br"] = act.get("win_cond_br", "")
            if act.get("win_cond_br"):
                ac = act["win_cond_br"]
                r["act_cycles_per_cond"] = round((act.get("win_cycles") or 0) / ac, 4)
                r["act_sram_rd_per_cond"] = round((act.get("win_sram_reads") or 0) / ac, 4)
                r["act_sram_wr_per_cond"] = round((act.get("win_sram_writes") or 0) / ac, 4)
            keys = ("win_start", "win_lines", "win_cond_br", "win_cycles", "win_sram_reads",
                    "win_sram_writes")
            if act.get("win_cond_br") and gls.get("win_cond_br"):
                r["act_vs_gls"] = ("match" if all(act.get(k) == gls.get(k) for k in keys)
                                   else "DIFFERS")
            r["area_by_module"] = ";".join(f"{k}:{v}" for k, v in hier.items())
            out.append(r)
        return out

    def sram_rows(self):
        """One row per trace x bp_sram instance, from the act records."""
        out = []
        for trace in self.traces:
            rec = self.load(f"act_{trace}") or {}
            if rec.get("status") != "ok":
                continue
            m = rec.get("metrics") or {}
            wc = m.get("win_cond_br") or 0
            for inst, v in sorted((m.get("sram_inst") or {}).items()):
                r = {"predictor": self.args.predictor, "budget": self.label, "budget_kb": self.kb,
                     "trace": trace, "config_id": self.cid,
                     "name": f"{self.args.predictor}/{self.label}/{inst}", "instance": inst}
                r.update(v)
                r.update(win_start=m.get("win_start"), win_lines=m.get("win_lines"),
                         win_cond_br=m.get("win_cond_br"), win_cycles=m.get("win_cycles"))
                for k in ("rd", "wr", "rw"):
                    r[f"{k}_per_cond"] = round(v[f"win_{k}"] / wc, 6) if wc else ""
                out.append(r)
        return out


# =============================================================================
# One predictor, all its budgets (also called by campaign.py)
# =============================================================================

def make_opts(**kw):
    """Options namespace with the DEFAULTS, overridden by kw."""
    bad = set(kw) - set(DEFAULTS)
    if bad:
        raise KeyError(f"unknown options {sorted(bad)}")
    return argparse.Namespace(**dict(DEFAULTS, **kw))


def dump_metas(dump_dir):
    """The .json files of a dump directory that belong to a trace dump: those
    with a matching .log (01_golden_dump.py writes both)."""
    return sorted(m for m in glob.glob(os.path.join(dump_dir, "*.json"))
                  if os.path.exists(m[:-len(".json")] + ".log"))


def load_budgets(predictor, opts):
    """Budget objects of every dump directory (with or without its dump
    files); exits on duplicates."""
    spec = PREDICTORS[predictor]
    stim = os.path.join(PROJ_DIR, "stim", predictor)
    dirs = sorted(glob.glob(os.path.join(stim, "*__*")),
                  key=lambda d: budget_key(os.path.basename(d).split("__", 1)[0]))
    src = source_hash()
    budgets = []
    for d in dirs:
        label = os.path.basename(d).split("__", 1)[0]
        if not dump_metas(d):
            continue
        try:
            budgets.append(Budget(opts, spec, label, d, src))
        except (ValueError, KeyError, OSError) as exc:
            log(f"SKIP {label}: {exc}")
    dup = sorted({b.label for b in budgets if [c.label for c in budgets].count(b.label) > 1})
    if dup:
        sys.exit(f"Several dump directories for budget(s) {' '.join(dup)} in {stim}: "
                 + ", ".join(f"{b.label}__{b.cid}" for b in budgets if b.label in dup)
                 + ". Remove the old ones (they would share eval/<predictor>/<budget>/).")
    return budgets


def evaluate_predictor(predictor, opts):
    """Runs the requested steps of one predictor. -> (all ok, CSV rows)."""
    opts = argparse.Namespace(**dict(vars(opts), predictor=predictor))
    spec = PREDICTORS[predictor]
    budgets = load_budgets(predictor, opts)
    if not budgets:
        log(f"No usable dumps for '{predictor}' in {os.path.join(PROJ_DIR, 'stim', predictor)}")
        return False, []
    if opts.print_params:
        for b in budgets:
            log(f"{b.label:>5s} {b.top} traces={','.join(b.traces)}"
                f"{'' if b.has_dumps() else '  (dumps deleted)'}\n      {params_str(b.core)}")
        return True, []

    # without --budgets: only budgets that still have their dumps
    todo = [b for b in budgets if (b.label in opts.budgets if opts.budgets else b.has_dumps())]
    missing = sorted(set(opts.budgets or []) - {b.label for b in todo}, key=budget_key)
    if missing:
        log(f"WARNING {predictor}: no usable dumps for {' '.join(missing)} "
            f"(have: {' '.join(b.label for b in budgets)})")
    if not todo:
        return False, []
    pilot = opts.pilot if opts.pilot in [b.label for b in todo] else todo[0].label
    todo.sort(key=lambda b: (b.label != pilot, budget_key(b.label)))

    window = (f"gls window: {opts.gls_window_pct}% of pass {opts.gls_warmup_passes + 1}"
              if opts.gls_warmup_passes else f"gls lines {opts.gls_lines}")
    rtl_n = f"rtl passes {opts.rtl_passes}" if opts.rtl_passes else f"rtl lines {opts.rtl_lines or 'all'}"
    log(f"project {PROJECT}, predictor {predictor} ({spec['top']}), clk {opts.clk} ns, "
        f"steps {' '.join(opts.steps)}{' + syn_hier' if opts.hier else ''}, pilot {pilot}, "
        f"{rtl_n}, {window}")
    all_ok = not missing
    for b in todo:
        log(f"\n=== {predictor} {b.label}{' (pilot)' if b.label == pilot else ''}  {b.cid}"
            f"{'' if b.has_dumps() else '  (dumps deleted)'}\n    params {params_str(b.core)}")
        ok = b.run()
        all_ok &= ok
        if b.label == pilot and not ok and not opts.dry_run:
            log(f"\nPILOT {pilot} FAILED: fix it before running the other budgets "
                f"(rerunning resumes where it stopped).")
            break
    if opts.dry_run:
        return all_ok, []

    # The CSV is rebuilt from the records of every budget, run now or before.
    rows = [r for b in budgets for r in b.rows()
            if any(r.get(k) not in ("", None)
                   for k in ("rtl_pass", "act_pass", "area_um2", "p_total_w"))]
    write_csv(os.path.join(EVAL_DIR, f"{predictor}_results.csv"), rows)
    srows = [r for b in budgets for r in b.sram_rows()]
    if srows:
        write_csv(os.path.join(EVAL_DIR, f"{predictor}_sram_counts.csv"), srows)
    differs = sorted({f"{r['budget']} {r['trace']}" for r in rows if r.get("act_vs_gls") == "DIFFERS"},
                     key=lambda x: budget_key(x.split()[0]))
    if differs:
        log(f"WARNING {predictor}: RTL (act) and netlist (gls) window counters differ for "
            + ", ".join(differs))
    return all_ok, rows


def write_csv(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def print_summary(rows, title):
    def fmt(v, spec_):
        try:
            return format(float(v), spec_)
        except (TypeError, ValueError):
            return "-".rjust(int(spec_.split(".")[0]))

    log("\n" + "=" * 128)
    log(f"SUMMARY {title}  (logic only, SRAMs excluded; delays in ps, energy in pJ per cond. branch)")
    log("=" * 128)
    log(f"{'predictor':>17s} {'budget':>6s} {'trace':>12s} {'rtl':>5s} {'MPKBr':>7s} {'area um2':>9s} "
        f"{'flops':>6s} {'crit ps':>8s} {'pred ps':>8s} {'gls':>5s} {'P uW':>8s} {'cyc/br':>7s} "
        f"{'pJ/br':>7s} {'R/br':>6s} {'W/br':>6s}")
    for r in rows:
        p_uw = float(r["p_total_w"]) * 1e6 if r.get("p_total_w") not in ("", None) else None
        log(f"{r['predictor']:>17s} {r['budget']:>6s} {r['trace'][:12]:>12s} "
            f"{str(r.get('rtl_pass') or '-'):>5s} {fmt(r.get('mpkbr'), '7.2f')} "
            f"{fmt(r.get('area_um2'), '9.1f')} {fmt(r.get('flops'), '6.0f')} "
            f"{fmt(r.get('crit_delay_ps'), '8.1f')} {fmt(r.get('pred_path_ps'), '8.1f')} "
            f"{str(r.get('gls_pass') or '-'):>5s} {fmt(p_uw, '8.2f')} "
            f"{fmt(r.get('cycles_per_cond'), '7.3f')} {fmt(r.get('e_logic_pj_per_cond'), '7.3f')} "
            f"{fmt(r.get('sram_rd_per_cond'), '6.2f')} {fmt(r.get('sram_wr_per_cond'), '6.2f')}")


def add_common_args(ap):
    """Options shared by evaluate.py and campaign.py."""
    d = DEFAULTS
    ap.add_argument("--steps", nargs="+", default=d["steps"], choices=STEPS)
    ap.add_argument("--rtl-lines", type=int, default=d["rtl_lines"],
                    help="branch lines per RTL check (0 = whole dump)")
    ap.add_argument("--rtl-passes", type=int, default=d["rtl_passes"],
                    help="passes per RTL check of a multi-pass dump (0 = use --rtl-lines)")
    ap.add_argument("--gls-lines", type=int, default=d["gls_lines"],
                    help="branch lines per gate-level run without a window: the VCD cap")
    ap.add_argument("--gls-warmup-passes", type=int, default=d["gls_warmup_passes"],
                    help="gate-level activity window in pass W+1 after W warm-up passes "
                         "(0 = no window: from the start, --gls-lines)")
    ap.add_argument("--gls-window-pct", type=float, default=d["gls_window_pct"],
                    help="window length in percent of a pass (default 1)")
    ap.add_argument("--clk", default=d["clk"], help="clock period in ns")
    ap.add_argument("--io-delay-pct", type=int, default=d["io_delay_pct"])
    ap.add_argument("--extra", nargs="+", default=d["extra"], metavar="K=V")
    ap.add_argument("--hier", action="store_true")
    ap.add_argument("--keep-vcd", action="store_true", help="keep the VCDs after DPA")
    ap.add_argument("--keep-sim", action="store_true", help="keep the sim/ run directories")
    ap.add_argument("--min-free-gb", type=float, default=d["min_free_gb"])
    ap.add_argument("--rerun", action="store_true")
    ap.add_argument("--dry-run", action="store_true")


def check_common_args(ap, args):
    try:
        float(args.clk)
    except ValueError:
        ap.error(f"--clk must be a number of nanoseconds, got '{args.clk}'")
    if args.gls_warmup_passes < 0 or args.rtl_passes < 0:
        ap.error("--gls-warmup-passes and --rtl-passes must be >= 0")
    if not 0 < args.gls_window_pct <= 100:
        ap.error("--gls-window-pct must be in (0, 100]")
    if not args.gls_warmup_passes and args.gls_lines <= 0 and "gls" in args.steps:
        ap.error("--gls-lines must be > 0: an uncapped gate-level VCD can fill the disk")


# =============================================================================
# Main
# =============================================================================

def main():
    ap = argparse.ArgumentParser(description="Full-flow evaluation of one predictor (see the header).")
    ap.add_argument("predictor", choices=sorted(PREDICTORS))
    ap.add_argument("--budgets", nargs="+", default=None)
    ap.add_argument("--pilot", default=None)
    ap.add_argument("--print-params", action="store_true")
    add_common_args(ap)
    args = ap.parse_args()
    check_common_args(ap, args)
    opts = make_opts(**{k: v for k, v in vars(args).items() if k != "predictor"})

    with Lock():
        ok, rows = evaluate_predictor(args.predictor, opts)
    if rows:
        print_summary(rows, f"{args.predictor}   {os.path.join(EVAL_DIR, args.predictor + '_results.csv')}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()