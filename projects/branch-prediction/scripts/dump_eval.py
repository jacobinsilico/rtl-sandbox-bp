#!/usr/bin/python3
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
# -----------------------------------------------------------------------------
"""
dump_eval.py -- golden dumps made, evaluated and deleted again, one predictor
and budget at a time, for the configs compare_rtl.py selected
(rtl_selection.csv). Disk use peaks at one budget's dumps.

PER SELECTED CONFIG (family, budget)
    1. dump      01_golden_dump.py (CBP5 repo) on the three core traces,
                 --max-passes PASSES (default 6), with the predictor's
                 internal-state trace (TAGE: TAGE_CB_STATE, hashed
                 perceptron: HP_STATE), into a staging directory on the same
                 disk (stim/.staging/). Skipped when stim/ already holds a
                 complete dump of this config with the same pass count.
    2. place     every other dump directory of this budget is removed (
                 evaluate.py refuses two), the new one is moved to
                 stim/<predictor>/<budget>KB__<config_id>/; the state trace
                 becomes <trace>.state, cut to its first --state-passes
                 passes (default 1: a TAGE state line is ~120 bytes, six
                 passes would be several GB per trace)
    3. evaluate  evaluate.py <predictor> --budgets <budget>KB with --steps,
                 --clk, --rtl-passes, --gls-warmup-passes, --gls-window-pct
    4. check     evaluate.py's exit status, and in its records: every step ok,
                 the RTL run covered the expected lines, the state was checked
                 as far as the state trace goes, the activity window starts
                 and ends where it should
    5. clean     the dumps and state traces are deleted (--keep-dumps keeps
                 them); the .json / .log stay, so evaluate.py still lists the
                 budget and its records stay in the CSV; a marker
                 (dump_eval.done) records the options of the finished run
    A failed step keeps everything (staging, dumps) and stops (--keep-going:
    go on with the next config). A config whose marker matches the current
    options is skipped; --redo runs it again (the dump is regenerated from the
    ledger config, deterministically).

PATHS  (CBP5 repo = --cbp, default /home/j50065913/cbp5-wireless/cbp5-new-golden)
    selection   <cbp>/scripts/bp-comparison/analysis/traces_csv_long/rtl_compare/
                rtl_selection.csv
    ledger      <cbp>/scripts/bp-comparison/results/results_traces_csv_long.csv
    configs     <cbp>/scripts/bp-comparison/configs/<predictor>__<experiment>.csv
    traces      <cbp>/traces/traces_csv_long
    dump script <cbp>/scripts/*/01_golden_dump.py
    logs        projects/<project>/eval/dump_eval/

USAGE  (RTL repo root, after `source sourceme.sh`)
    first test: one config, RTL check of pass 1 only, dumps kept
      /usr/bin/python3 projects/branch-prediction/scripts/dump_eval.py \\
          --families gshare --budgets 1 --steps rtl --rtl-passes 1 --keep-dumps
    everything, 1 GHz, power on 1 % of pass 6 after 5 warm-up passes
      /usr/bin/python3 projects/branch-prediction/scripts/dump_eval.py --clk 0.9
    one family:  --families tcb      the plan only:  --dry-run
"""

import argparse
import csv
import glob
import json
import os
import shutil
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJ_DIR = os.path.dirname(SCRIPT_DIR)
EVALUATE = os.path.join(SCRIPT_DIR, "evaluate.py")
PY = "/usr/bin/python3" if os.path.exists("/usr/bin/python3") else sys.executable
CBP_DEFAULT = "/home/j50065913/cbp5-wireless/cbp5-new-golden"

FAMILIES = {                       # compare_rtl.py family -> stim dir, state trace
    "gshare": {"stim": "gshare", "state_env": None},
    "gp":     {"stim": "g_perceptron", "state_env": None},
    "hp":     {"stim": "hashed_perceptron", "state_env": "HP_STATE"},
    "tcb":    {"stim": "tage_cb", "state_env": "TAGE_CB_STATE"},
}
ORDER = ["gshare", "gp", "hp", "tcb"]
CORE_TRACES = ["fdd_su*", "tdd_dl_mu*", "tdd_ul_mu*"]
MARKER = "dump_eval.done"            # not *.json: evaluate.py reads the *.json + *.log pairs
OLD_MARKER = "dump_eval.json"        # written by the first version; removed when seen


def log(msg=""):
    print(msg, flush=True)


def run(cmd, logfile, dry, echo=False):
    """Runs cmd, output to logfile (and the terminal with echo). -> exit code."""
    log("    $ " + " ".join(cmd))
    if dry:
        return 0
    os.makedirs(os.path.dirname(logfile), exist_ok=True)
    with open(logfile, "w") as f:
        f.write("$ " + " ".join(cmd) + "\n\n")
        f.flush()
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in p.stdout:
            f.write(line)
            if echo:
                print("    | " + line, end="", flush=True)
        return p.wait()


def load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def copy_head(src, dst, n_lines):
    """Copies the '#' lines and the first n_lines other lines of src to dst."""
    kept = 0
    with open(src) as fi, open(dst, "w") as fo:
        for line in fi:
            if line.startswith("#"):
                fo.write(line)
                continue
            if kept >= n_lines:
                break
            fo.write(line)
            kept += 1
    return kept


class Job:
    def __init__(self, a, sel):
        self.a, self.sel = a, sel
        self.fam = sel["family"]
        self.F = FAMILIES[self.fam]
        self.pred, self.cid = sel["predictor"], sel["config_id"]
        self.label = f"{int(sel['budget_kb'])}KB"
        self.stim_dir = os.path.join(PROJ_DIR, "stim", self.F["stim"])
        self.dest = os.path.join(self.stim_dir, f"{self.label}__{self.cid}")
        self.staging = os.path.join(PROJ_DIR, "stim", ".staging", f"{self.F['stim']}_{self.label}")
        self.edir = os.path.join(PROJ_DIR, "eval", self.F["stim"], self.label)
        self.logs = os.path.join(PROJ_DIR, "eval", "dump_eval")
        self.state = bool(self.F["state_env"]) and a.state_passes != 0
        self.opts = {"config_id": self.cid, "passes": a.passes,
                     "state_passes": a.state_passes if self.state else 0,
                     "steps": sorted(a.steps), "clk": a.clk, "rtl_passes": a.rtl_passes,
                     "gls_warmup_passes": a.gls_warmup_passes, "gls_window_pct": a.gls_window_pct}

    def name(self):
        return f"{self.fam} {self.label} {self.cid}"

    # ---- state on disk ----
    def finished(self):
        m = load_json(os.path.join(self.dest, MARKER))
        return bool(m) and m.get("complete") and m.get("options") == self.opts

    def metas(self, d):
        """trace -> dump metadata: the .json files with a matching .log."""
        return {os.path.basename(p)[:-len(".json")]: load_json(p)
                for p in sorted(glob.glob(os.path.join(d, "*.json")))
                if os.path.exists(p[:-len(".json")] + ".log")}

    def dumps_present(self):
        """stim/ holds a complete dump of this config with this pass count."""
        metas = self.metas(self.dest)
        if len(metas) != len(CORE_TRACES):
            return False
        for t, m in metas.items():
            if not m or m.get("config_id") != self.cid or m.get("max_passes") != self.a.passes:
                return False
            if not os.path.exists(os.path.join(self.dest, t + ".golden.txt")):
                return False
            if self.state and not os.path.exists(os.path.join(self.dest, t + ".state")):
                return False
        return True

    # ---- steps ----
    def dump(self):
        a = self.a
        cfg = os.path.join(a.bpc, "configs", f"{self.pred}__{self.sel['experiment']}.csv")
        if not os.path.exists(cfg) and not a.dry_run:
            return f"no config file {cfg}"
        if not a.dry_run:
            shutil.rmtree(self.staging, ignore_errors=True)
            os.makedirs(self.staging)
        cmd = [PY, a.dump_script, "--predictor", self.pred, "--config", cfg,
               "--config-id", self.cid, "--results", a.ledger, "--traces", a.traces,
               "--trace-kind", "long", "--max-passes", str(a.passes),
               "--only-traces", *CORE_TRACES, "--out-dir", self.staging,
               "--jobs", str(a.dump_jobs)]
        if self.state:
            cmd += ["--trace-env", self.F["state_env"]]
        rc = run(cmd, os.path.join(self.logs, f"{self.F['stim']}_{self.label}_dump.log"), a.dry_run)
        if a.dry_run:
            return None
        src = os.path.join(self.staging, self.pred, f"{self.label}__{self.cid}")
        metas = self.metas(src)
        if rc != 0 or len(metas) != len(CORE_TRACES):
            return (f"01_golden_dump.py exit {rc}, {len(metas)} of {len(CORE_TRACES)} traces "
                    f"dumped (log eval/dump_eval/{self.F['stim']}_{self.label}_dump.log, "
                    f"staging kept: {src})")
        # place: no other dump directory of this budget may stay
        for old in glob.glob(os.path.join(self.stim_dir, f"{self.label}__*")):
            shutil.rmtree(old)
        os.makedirs(self.dest)
        for t, m in metas.items():
            for ext in (".golden.txt", ".log", ".json"):
                os.replace(os.path.join(src, t + ext), os.path.join(self.dest, t + ext))
            if self.state:
                tr = os.path.join(src, t + ".trace.txt")
                st = os.path.join(self.dest, t + ".state")
                if self.a.state_passes >= self.a.passes:
                    os.replace(tr, st)
                else:
                    copy_head(tr, st, self.a.state_passes * int(m["branches_per_pass"]))
                    os.remove(tr)
        shutil.rmtree(self.staging, ignore_errors=True)
        return None

    def evaluate(self):
        a = self.a
        cmd = [PY, EVALUATE, self.F["stim"], "--budgets", self.label, "--steps", *a.steps,
               "--clk", a.clk, "--rtl-passes", str(a.rtl_passes),
               "--gls-warmup-passes", str(a.gls_warmup_passes),
               "--gls-window-pct", str(a.gls_window_pct)]
        return run(cmd, os.path.join(self.logs, f"{self.F['stim']}_{self.label}_evaluate.log"),
                   a.dry_run, echo=True)

    def check(self, rc):
        """Problems found in evaluate.py's records ([] = all good)."""
        a, probs = self.a, []
        if rc != 0:
            probs.append(f"evaluate.py exit {rc}")
        rec = lambda s: load_json(os.path.join(self.edir, f"{s}.json")) or {}  # noqa: E731
        for s in [x for x in ("syn", "sta") if x in a.steps]:
            if rec(s).get("status") != "ok":
                probs.append(f"{s}: {rec(s).get('status', 'no record')}")
        for t, m in self.metas(self.dest).items():
            short, bpp = t.split(".")[0], int(m["branches_per_pass"])
            if "rtl" in a.steps:
                r = rec(f"rtl_{short}")
                mm = r.get("metrics", {})
                want = a.rtl_passes * bpp if a.rtl_passes else m.get("lines")
                if r.get("status") != "ok":
                    probs.append(f"rtl_{short}: {r.get('status', 'no record')}")
                elif mm.get("lines") != want:
                    probs.append(f"rtl_{short}: {mm.get('lines')} lines checked, expected {want}")
                elif self.state:
                    st = min(want, a.state_passes * bpp) if a.state_passes < a.passes else want
                    if mm.get("state_lines") != st:
                        probs.append(f"rtl_{short}: state checked for {mm.get('state_lines')} "
                                     f"branches, expected {st}")
            if "gls" in a.steps:
                r = rec(f"gls_{short}")
                mm = r.get("metrics", {})
                if r.get("status") != "ok":
                    probs.append(f"gls_{short}: {r.get('status', 'no record')}")
                elif a.gls_warmup_passes:
                    start = a.gls_warmup_passes * bpp + 1
                    n = max(1, int(round(bpp * a.gls_window_pct / 100.0)))
                    if (mm.get("win_start"), mm.get("win_lines")) != (start, n):
                        probs.append(f"gls_{short}: window {mm.get('win_start')}+{mm.get('win_lines')}, "
                                     f"expected {start}+{n}")
            if "dpa" in a.steps and rec(f"dpa_{short}").get("status") != "ok":
                probs.append(f"dpa_{short}: {rec(f'dpa_{short}').get('status', 'no record')}")
        return probs

    def clean(self):
        removed = 0
        for pat in ("*.golden.txt", "*.state"):
            for p in glob.glob(os.path.join(self.dest, pat)):
                removed += os.path.getsize(p)
                os.remove(p)
        return removed

    def mark(self, kept):
        with open(os.path.join(self.dest, MARKER), "w") as f:
            json.dump({"complete": True, "options": self.opts, "dumps_kept": kept,
                       "time": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=1)

    # ---- the whole cycle ----
    def go(self):
        a = self.a
        log(f"\n=== {self.name()}")
        old = os.path.join(self.dest, OLD_MARKER)
        if os.path.exists(old) and not a.dry_run:
            os.remove(old)                   # it made evaluate.py skip this budget
        if not a.redo and self.finished():
            if not a.keep_dumps and not a.dry_run and \
                    glob.glob(os.path.join(self.dest, "*.golden.txt")):
                freed = self.clean()             # finished earlier with --keep-dumps
                self.mark(False)
                return f"done (kept); {freed / 1e9:.1f} GB of dumps deleted now"
            return "done (kept)"
        if not a.dry_run:
            free = shutil.disk_usage(PROJ_DIR).free / 1e9
            if free < a.min_free_gb:
                return f"FAILED: only {free:.0f} GB free (< --min-free-gb {a.min_free_gb})"
        t0 = time.time()
        if not a.redo and self.dumps_present():
            log("    dumps already in stim/ (same config and passes): not regenerated")
        else:
            err = self.dump()
            if err:
                return f"FAILED (dump): {err}"
        rc = self.evaluate()
        if a.dry_run:
            return "dry"
        probs = self.check(rc)
        if probs:
            for p in probs:
                log(f"    PROBLEM {p}")
            return f"FAILED (evaluate): {len(probs)} problem(s); dumps kept in {self.dest}"
        freed = 0 if a.keep_dumps else self.clean()
        self.mark(a.keep_dumps)
        return (f"ok in {(time.time() - t0) / 60:.1f} min"
                + (", dumps kept" if a.keep_dumps else f", {freed / 1e9:.1f} GB of dumps deleted"))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cbp", default=CBP_DEFAULT, help="CBP5 repo root")
    ap.add_argument("--selection", help="rtl_selection.csv (compare_rtl.py)")
    ap.add_argument("--ledger", help="results ledger of the selection")
    ap.add_argument("--traces", help="long-trace directory")
    ap.add_argument("--dump-script", help="01_golden_dump.py")
    ap.add_argument("--families", nargs="+", choices=ORDER, default=ORDER)
    ap.add_argument("--budgets", nargs="+", type=int, help="KB (default: all selected)")
    ap.add_argument("--passes", type=int, default=6, help="passes per dump (6)")
    ap.add_argument("--state-passes", type=int, default=1,
                    help="passes kept in the state traces (1; 0 = no state trace)")
    ap.add_argument("--steps", nargs="+", default=["rtl", "syn", "sta", "gls", "dpa"],
                    choices=["rtl", "syn", "sta", "gls", "dpa"])
    ap.add_argument("--clk", default="0.9", help="clock period, ns (0.9)")
    ap.add_argument("--rtl-passes", type=int, default=0,
                    help="passes per RTL check (0 = the whole dump)")
    ap.add_argument("--gls-warmup-passes", type=int, default=5)
    ap.add_argument("--gls-window-pct", type=float, default=1.0)
    ap.add_argument("--dump-jobs", type=int, default=3, help="01_golden_dump.py --jobs (3)")
    ap.add_argument("--min-free-gb", type=float, default=40.0)
    ap.add_argument("--keep-dumps", action="store_true", help="do not delete the dumps")
    ap.add_argument("--redo", action="store_true", help="run finished configs again")
    ap.add_argument("--keep-going", action="store_true", help="go on after a failed config")
    ap.add_argument("--allow-provisional", action="store_true",
                    help="accept a selection made before the rtl winners existed")
    ap.add_argument("--dry-run", action="store_true", help="print the commands only")
    a = ap.parse_args()

    if not os.environ.get("REPO_HOME"):
        sys.exit("dump_eval: REPO_HOME is not set: run `source sourceme.sh` in the RTL repo first")
    a.cbp = os.path.abspath(a.cbp)
    a.bpc = os.path.join(a.cbp, "scripts", "bp-comparison")
    a.selection = os.path.abspath(a.selection or os.path.join(
        a.bpc, "analysis", "traces_csv_long", "rtl_compare", "rtl_selection.csv"))
    a.ledger = os.path.abspath(a.ledger or os.path.join(a.bpc, "results",
                                                          "results_traces_csv_long.csv"))
    a.traces = os.path.abspath(a.traces or os.path.join(a.cbp, "traces", "traces_csv_long"))
    if not a.dump_script:
        found = sorted(glob.glob(os.path.join(a.cbp, "scripts", "*", "01_golden_dump.py")))
        if not found:
            sys.exit(f"dump_eval: no 01_golden_dump.py under {a.cbp}/scripts/*/ (--dump-script)")
        a.dump_script = found[0]
    for what, p in (("selection", a.selection), ("ledger", a.ledger), ("traces", a.traces),
                    ("dump script", a.dump_script), ("evaluate.py", EVALUATE)):
        if not os.path.exists(p):
            sys.exit(f"dump_eval: no {what} {p}")
    if a.gls_warmup_passes and a.gls_warmup_passes >= a.passes and "gls" in a.steps:
        sys.exit(f"dump_eval: the window needs pass {a.gls_warmup_passes + 1}, "
                 f"the dumps have {a.passes}")
    if a.rtl_passes > a.passes:
        sys.exit(f"dump_eval: --rtl-passes {a.rtl_passes} > --passes {a.passes}")

    with open(a.selection, newline="") as f:
        sel = [r for r in csv.DictReader(f)
               if r["family"] in a.families and (not a.budgets or int(r["budget_kb"]) in a.budgets)]
    prov = [r for r in sel if r.get("source") == "provisional"]
    if prov and not a.allow_provisional:
        sys.exit(f"dump_eval: {len(prov)} selected config(s) are provisional (chosen before the rtl "
                 f"winners existed). Rerun compare_rtl.py once the campaign is done, or pass "
                 f"--allow-provisional.")
    sel.sort(key=lambda r: (ORDER.index(r["family"]), int(r["budget_kb"])))
    if not sel:
        sys.exit("dump_eval: nothing selected")

    log(f"dump_eval: {len(sel)} config(s) from {a.selection}")
    log(f"  dumps: {a.passes} passes, "
        f"{f'state traces {a.state_passes} pass(es)' if a.state_passes else 'no state traces'}; "
        f"evaluate: steps {' '.join(a.steps)}, clk {a.clk} ns, rtl passes "
        f"{a.rtl_passes or 'all'}, gls window {a.gls_window_pct}% of pass {a.gls_warmup_passes + 1}; "
        f"dumps {'kept' if a.keep_dumps else 'deleted after a passing run'}")
    results = []
    for r in sel:
        job = Job(a, r)
        status = job.go()
        log(f"    -> {status}")
        results.append((job.name(), status))
        if status.startswith("FAILED") and not a.keep_going:
            log("\nstopped at the first failure (--keep-going goes on)")
            break

    log("\n" + "=" * 100)
    for name, status in results:
        log(f"  {name:90s}  {status}")
    sys.exit(1 if any(s.startswith("FAILED") for _, s in results) else 0)


if __name__ == "__main__":
    main()