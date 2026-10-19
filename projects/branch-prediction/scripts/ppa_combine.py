#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Full PPA of the evaluated predictors: the logic (evaluate.py: area, energy
#   per conditional branch, timing) plus the SRAM arrays (cacti_cost.py: 22 nm
#   CACTI scaled to 7 nm), each charged with its own accesses (the act step's
#   eval/<predictor>_sram_counts.csv), per budget, with scenarios and a
#   sensitivity sweep.
#
#   Per predictor, budget and trace (and the mean over the traces):
#     area      logic area + the SRAM areas (and with the unused bits' cell
#               area taken out: area_used)
#     energy    per conditional branch = E_logic + E_sram_dyn + E_sram_leak
#                 E_sram_dyn  = sum_i (rd_i * E_rd,i + wr_i * E_wr,i), rd_i / wr_i
#                               per cond. branch in the activity window
#                 E_sram_leak = sum_i P_leak,i * CPB * T_host
#               CPB = host-core cycles per conditional branch (an assumption,
#               swept with --cpb)
#     timing    the worst SRAM access time (both bounds) against --clk-ns, and
#               the cycle the logic needs with the SRAM in front of it: the
#               worse of the critical path and the worst path from an SRAM
#               read port (rdata_crit_ps, sta_reports.tcl's rdata_path.rpt),
#               with the STA's placeholder clock-to-Q on the read ports
#               (--sram-delay-ps, 200 ps) replaced by the access time; STA
#               runs older than that report only adjust a critical path that
#               starts at a read port (*rdata*)
#     MPKBr     plot_ppa.py's eval/ppa/mpkbr_steady.csv (CBP5 steady state)
#               when it exists, else evaluate.py's RTL value (first pass, cold
#               tables); --mpkbr CSV / rtl chooses
#   Ports per instance (--ports auto): 1RW when the act runs saw no cycle with
#   a read and a write (rw = 0 on every trace), else 1R1W.
#   Scenarios: every CACTI directory x cell type it costed, named dir:cell; a
#   directory fixes the row packing (byte / exact) and the periphery (hp /
#   lstp).
#
#   Sensitivity (does the per-budget picture survive?): energy over every
#   scenario x CPB x SRAM energy factor (--e-factors), area over every
#   scenario x area factor (Stillmaker's 22 -> 7 nm factor and --a-factors).
#   Per budget: how often each predictor is on the (MPKBr, energy) and the
#   (MPKBr, area) Pareto front, whether the baseline's energy and area
#   rankings hold in every corner, and for each pair (more accurate, more
#   expensive) the misprediction energy above which the more accurate one
#   saves energy overall (break-even: a misprediction must cost the host core
#   more than this; "dominates" when it is also the cheaper one).
#
#   Not modelled (keep with the table): the logic has no clock gating, so a
#   multi-cycle core (TAGE: ~11 busy cycles per branch) pays clock power on
#   idle flops; a core busy for more cycles per branch than CPB cannot keep up
#   with the host (flagged); logic leakage is charged over the busy cycles
#   only (< 0.1 % of the logic energy).
#
# Output (--out, default projects/branch-prediction/eval/ppa/):
#   ppa_long.csv     scenario x CPB x predictor x budget x trace (+ mean)
#   ppa_table.csv    the baseline scenario at --cpb-base, mean over traces
#   sensitivity.csv  per budget and predictor: Pareto counts, energy and area
#                    over the corners, rank ranges
#   breakeven.csv    per budget and pair: break-even misprediction energy,
#                    baseline and range over the corners
#   the per-budget tables on stdout
#
# Usage (after `source sourceme.sh`):
#   /usr/bin/python3 projects/branch-prediction/scripts/ppa_combine.py [options]
#     --cacti DIR ...     CACTI directories (default: every eval/cacti*/ with a
#                         cacti_arrays.csv)
#     --baseline S        scenario of the main table (default
#                         cacti_exact_plstp:lstp if costed, else the first)
#     --cpb N ...         host cycles per cond. branch swept (default 2 4 8)
#     --cpb-base N        CPB of the main table (default 4)
#     --host-clk-ns T     host clock period for the leakage (default 0.9)
#     --clk-ns T          predictor clock for the timing check (default 0.9)
#     --ports M           auto (default), 1r1w or 1rw
#     --e-factors F ...   SRAM energy factors (default 0.5 1 2)
#     --a-factors F ...   22 -> 7 nm area factors besides Stillmaker's
#                         (default 0.2 0.4)
#     --mpkbr CSV|rtl     MPKBr source: a CSV with the columns predictor,
#                         budget_kb (or budget), trace, mpkbr, or rtl (default:
#                         eval/ppa/mpkbr_steady.csv if it exists, else rtl)
#     --sram-delay-ps P   SRAM clock-to-Q placeholder of the STA (default 200)
#     --eval DIR          evaluate.py's eval/ (default: this project's)
#     --out DIR           output directory
#
#   Example:
#     /usr/bin/python3 projects/branch-prediction/scripts/ppa_combine.py --cpb 2 4 8 --cpb-base 4
# -----------------------------------------------------------------------------

import argparse
import csv
import glob
import json
import os
import re
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJ_DIR = os.path.dirname(SCRIPT_DIR)
EVAL_DIR = os.path.join(PROJ_DIR, "eval")

PREDICTORS = ["gshare", "g_perceptron", "hashed_perceptron", "tage_cb"]
ALIAS = {"gshare": "gshare", "g_perceptron": "g_perceptron", "gp": "g_perceptron",
         "perceptron": "g_perceptron", "hashed_perceptron": "hashed_perceptron",
         "hp": "hashed_perceptron", "hashed perceptron": "hashed_perceptron",
         "tage_cb": "tage_cb", "tcb": "tage_cb", "tage": "tage_cb"}
SHORT = {"gshare": "gshare", "g_perceptron": "GP", "hashed_perceptron": "HP", "tage_cb": "TAGE"}


def warn(msg):
    print(f"WARNING: {msg}", file=sys.stderr)


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, restval="", lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def kb_of(r):
    if r.get("budget_kb") not in (None, ""):
        return int(float(r["budget_kb"]))
    m = re.match(r"(\d+)\s*(?:KB|AU)?$", (r.get("budget") or "").strip(), re.IGNORECASE)
    return int(m.group(1)) if m else None


# =============================================================================
# Inputs
# =============================================================================

def load_logic(eval_dir):
    """(predictor, KB, trace) -> evaluate.py results row."""
    out = {}
    for p in PREDICTORS:
        path = os.path.join(eval_dir, f"{p}_results.csv")
        if not os.path.exists(path):
            warn(f"no {path}")
            continue
        for r in read_csv(path):
            out[(p, kb_of(r), r["trace"])] = r
    return out


def load_counts(eval_dir):
    """(predictor, KB, trace) -> {array name: act-step count row}."""
    out = {}
    for p in PREDICTORS:
        path = os.path.join(eval_dir, f"{p}_sram_counts.csv")
        if os.path.exists(path):
            for r in read_csv(path):
                out.setdefault((p, kb_of(r), r["trace"]), {})[r["name"]] = r
    return out


def load_mpkbr(path):
    """(predictor, KB, trace) -> MPKBr from a user CSV."""
    out = {}
    for r in read_csv(path):
        p = ALIAS.get((r.get("predictor") or "").strip().lower())
        kb, v = kb_of(r), fnum(r.get("mpkbr"))
        if p and kb and v is not None:
            out[(p, kb, (r.get("trace") or "").strip())] = v
    return out


def mpkbr_of(over, key, logic_row):
    """MPKBr of (predictor, KB, trace): the override (trace matched exactly or
    by prefix), else the RTL's."""
    if over is not None:
        if key in over:
            return over[key], "csv"
        p, kb, t = key
        for (q, k, u), v in over.items():
            if q == p and k == kb and u and (t.startswith(u) or u.startswith(t)):
                return v, "csv"
        return None, "csv missing"
    return fnum(logic_row.get("mpkbr")), "rtl pass 1"


class Scenario:
    """One CACTI directory and one cell type it costed."""

    def __init__(self, d, cell, meta, rows):
        self.dir, self.cell, self.rows = d, cell, rows
        self.name = f"{os.path.basename(os.path.normpath(d))}:{cell}"
        self.periph = meta.get("periph", "hp")
        self.pack = meta.get("pack_mode", "byte")
        self.temp_k = meta.get("temp_k", "")
        self.area_f = meta["factors"]["area"]


def load_scenarios(dirs):
    out = []
    for d in dirs:
        meta_p, rows_p = os.path.join(d, "cacti_scaling.json"), os.path.join(d, "cacti_arrays.csv")
        if not (os.path.exists(meta_p) and os.path.exists(rows_p)):
            warn(f"{d}: no cacti_scaling.json / cacti_arrays.csv, skipped")
            continue
        meta = json.load(open(meta_p))
        rows = {(r["name"], r["ports"], r["cell"]): r for r in read_csv(rows_p)}
        for cell in meta.get("cells", ["hp", "lstp"]):
            out.append(Scenario(d, cell, meta, rows))
    return out


def choose_ports(counts, mode):
    """(predictor, KB) -> {array name: '1rw' | '1r1w'}."""
    seen = {}
    for (p, kb, _), inst in counts.items():
        for name, r in inst.items():
            seen.setdefault((p, kb), {}).setdefault(name, []).append(r)
    out = {}
    for cfg, inst in seen.items():
        out[cfg] = {n: mode if mode != "auto" else
                    ("1rw" if all(int(float(r["rw"])) == 0 for r in rs) else "1r1w")
                    for n, rs in inst.items()}
    return out


# =============================================================================
# Costs
# =============================================================================

NEEDED = ("e_rd_7", "e_wr_7", "leak_7", "area_22", "area_7", "t_opt_7")


def sram_terms(sc, inst_counts, ports):
    """SRAM terms of one config and trace in one scenario (nominal factors):
    -> (dict, [missing arrays])."""
    acc = {"e_rd": 0.0, "e_wr": 0.0, "leak_uw": 0.0, "area22": 0.0, "area_used22": 0.0,
           "t_opt": 0.0, "t_pes": 0.0, "n": 0, "n_1rw": 0, "rwa": 0}
    missing = []
    for name, c in inst_counts.items():
        port = ports[name]
        r = sc.rows.get((name, port, sc.cell))
        v = {k: fnum(r.get(k)) for k in NEEDED + ("area_7_used", "t_pes_7")} if r else {}
        if not r or r.get("status", "").startswith("fail") or any(v[k] is None for k in NEEDED):
            missing.append(f"{name} [{port} {sc.cell}]")
            continue
        acc["e_rd"] += (fnum(c.get("rd_per_cond")) or 0.0) * v["e_rd_7"]
        acc["e_wr"] += (fnum(c.get("wr_per_cond")) or 0.0) * v["e_wr_7"]
        acc["leak_uw"] += v["leak_7"]
        acc["area22"] += v["area_22"]
        used = v["area_7_used"] if v["area_7_used"] is not None else v["area_7"]
        acc["area_used22"] += v["area_22"] * (used / v["area_7"] if v["area_7"] else 1.0)
        acc["t_opt"] = max(acc["t_opt"], v["t_opt_7"])
        acc["t_pes"] = max(acc["t_pes"], v["t_pes_7"] if v["t_pes_7"] is not None else v["t_opt_7"])
        acc["n"] += 1
        acc["n_1rw"] += port == "1rw"
        acc["rwa"] += int(float(c.get("rwa") or 0))
    return acc, missing


def ppa(sc, acc, logic, mpkbr, cpb, a, k_e=1.0, a_f=None):
    """One PPA point. k_e scales the SRAM energy, a_f replaces the area factor."""
    a_f = sc.area_f if a_f is None else a_f
    e_logic = fnum(logic.get("e_logic_pj_per_cond"))
    e_dyn = (acc["e_rd"] + acc["e_wr"]) * k_e
    e_leak = acc["leak_uw"] * cpb * a.host_clk_ns * 1e-3 * k_e      # uW * ns = 1e-3 pJ
    area_logic = fnum(logic.get("area_um2"))
    area_sram = acc["area22"] * a_f
    crit = fnum(logic.get("crit_delay_ps"))
    rcrit = fnum(logic.get("rdata_crit_ps"))   # worst path from a read port (newer STA runs)
    from_sram = bool(re.search(r"rdata", logic.get("crit_start") or ""))
    ph = a.sram_delay_ps / 1000.0          # the STA's placeholder on the rdata inputs
    busy = fnum(logic.get("cycles_per_cond"))
    r = {"mpkbr": mpkbr, "e_logic_pj": e_logic, "e_sram_rd_pj": acc["e_rd"] * k_e,
         "e_sram_wr_pj": acc["e_wr"] * k_e, "e_sram_leak_pj": e_leak,
         "e_total_pj": (e_logic + e_dyn + e_leak) if e_logic is not None else None,
         "sram_leak_uw": acc["leak_uw"], "area_logic_um2": area_logic,
         "area_sram_um2": area_sram, "area_sram_used_um2": acc["area_used22"] * a_f,
         "area_um2": (area_logic + area_sram) if area_logic is not None else None,
         "t_sram_opt_ns": acc["t_opt"], "t_sram_pes_ns": acc["t_pes"],
         "crit_logic_ns": crit / 1000.0 if crit is not None else None,
         "crit_from_sram": int(from_sram), "busy_cyc": busy,
         "n_sram": acc["n"], "n_1rw": acc["n_1rw"], "rwa": acc["rwa"]}
    r["rdata_crit_ns"] = rcrit / 1000.0 if rcrit is not None else None
    if crit is not None:
        for b in ("opt", "pes"):
            t = acc[f"t_{b}"]
            cand = [t, crit / 1000.0 + ((t - ph) if from_sram else 0.0)]
            if rcrit is not None:
                cand.append(rcrit / 1000.0 - ph + t)
            r[f"period_{b}_ns"] = max(cand)
    if r["e_total_pj"] is not None:
        r["e_sram_share"] = (e_dyn + e_leak) / r["e_total_pj"]
    if r["area_um2"]:
        r["area_sram_share"] = area_sram / r["area_um2"]
    return r


def mean_rows(rows):
    out = {}
    for k in rows[0]:
        vals = [r[k] for r in rows if isinstance(r.get(k), (int, float)) and r.get(k) is not None]
        out[k] = sum(vals) / len(vals) if len(vals) == len(rows) else None
    return out


def pareto(points):
    """points {name: (x, y)}, lower is better -> set of names on the front."""
    front = set()
    for n, (x, y) in points.items():
        if not any((x2 <= x and y2 <= y and (x2 < x or y2 < y))
                   for m, (x2, y2) in points.items() if m != n):
            front.add(n)
    return front


def ranking(values):
    return tuple(n for n, _ in sorted(values.items(), key=lambda kv: kv[1]))


# =============================================================================
# Main
# =============================================================================

def main():
    ap = argparse.ArgumentParser(description="Logic + SRAM PPA per budget (see the header)")
    ap.add_argument("--eval", default=EVAL_DIR)
    ap.add_argument("--cacti", nargs="+", default=None)
    ap.add_argument("--baseline", default=None)
    ap.add_argument("--cpb", nargs="+", type=float, default=[2.0, 4.0, 8.0])
    ap.add_argument("--cpb-base", type=float, default=4.0)
    ap.add_argument("--host-clk-ns", type=float, default=0.9)
    ap.add_argument("--clk-ns", type=float, default=0.9)
    ap.add_argument("--ports", default="auto", choices=["auto", "1r1w", "1rw"])
    ap.add_argument("--e-factors", nargs="+", type=float, default=[0.5, 1.0, 2.0])
    ap.add_argument("--a-factors", nargs="+", type=float, default=[0.2, 0.4])
    ap.add_argument("--mpkbr", default=None)
    ap.add_argument("--sram-delay-ps", type=float, default=200.0)
    ap.add_argument("--out", default=os.path.join(EVAL_DIR, "ppa"))
    a = ap.parse_args()
    if a.cpb_base not in a.cpb:
        a.cpb = sorted(set(a.cpb) | {a.cpb_base})

    dirs = a.cacti or sorted(d for d in glob.glob(os.path.join(a.eval, "cacti*"))
                             if os.path.exists(os.path.join(d, "cacti_arrays.csv")))
    scen = load_scenarios(dirs)
    if not scen:
        sys.exit("no CACTI results (cacti_cost.py --out eval/cacti*)")
    names = [s.name for s in scen]
    base_name = a.baseline or ("cacti_exact_plstp:lstp" if "cacti_exact_plstp:lstp" in names
                               else names[0])
    if base_name not in names:
        sys.exit(f"--baseline {base_name} is not one of {', '.join(names)}")
    base = names.index(base_name)

    logic = load_logic(a.eval)
    counts = load_counts(a.eval)
    if a.mpkbr is None:
        dflt = os.path.join(a.eval, "ppa", "mpkbr_steady.csv")
        a.mpkbr = dflt if os.path.exists(dflt) else "rtl"
    over = None if a.mpkbr == "rtl" else load_mpkbr(a.mpkbr)
    ports = choose_ports(counts, a.ports)
    configs = sorted({(p, kb) for (p, kb, _) in logic}, key=lambda c: (c[1], PREDICTORS.index(c[0])))
    traces = {c: sorted(t for (p, kb, t) in logic if (p, kb) == c) for c in configs}

    # ---- per scenario, config, trace: SRAM terms (independent of CPB and factors) ----
    terms, skipped = {}, set()
    for c in configs:
        for t in traces[c]:
            if (c[0], c[1], t) not in counts:
                skipped.add(c)
    for c in sorted(skipped, key=lambda c: (c[1], c[0])):
        warn(f"{c[0]} {c[1]} KB: no per-instance counts for every trace (act step): left out")
    configs = [c for c in configs if c not in skipped]
    for si, sc in enumerate(scen):
        for c in configs:
            for t in traces[c]:
                acc, miss = sram_terms(sc, counts[(c[0], c[1], t)], ports[c])
                if miss:
                    warn(f"{sc.name} {c[0]} {c[1]} KB: no CACTI cost for {', '.join(miss[:3])}"
                         f"{' ...' if len(miss) > 3 else ''}: left out of this scenario")
                    break
                terms[(si, c, t)] = acc

    def point(si, c, cpb, k_e=1.0, a_f=None):
        """Mean over the traces (None if a trace is missing)."""
        rows = []
        for t in traces[c]:
            if (si, c, t) not in terms:
                return None
            m, _ = mpkbr_of(over, (c[0], c[1], t), logic[(c[0], c[1], t)])
            rows.append(ppa(scen[si], terms[(si, c, t)], logic[(c[0], c[1], t)], m, cpb, a, k_e, a_f))
        return mean_rows(rows)

    os.makedirs(a.out, exist_ok=True)

    # ---- ppa_long.csv ----
    long_rows = []
    for si, sc in enumerate(scen):
        for cpb in a.cpb:
            for c in configs:
                per = []
                for t in traces[c]:
                    if (si, c, t) not in terms:
                        continue
                    m, src = mpkbr_of(over, (c[0], c[1], t), logic[(c[0], c[1], t)])
                    r = ppa(sc, terms[(si, c, t)], logic[(c[0], c[1], t)], m, cpb, a)
                    per.append(r)
                    long_rows.append({"scenario": sc.name, "pack": sc.pack, "periph": sc.periph,
                                      "cell": sc.cell, "cpb": cpb, "predictor": c[0],
                                      "budget_kb": c[1], "trace": t, "mpkbr_src": src, **r})
                if per and len(per) == len(traces[c]):
                    long_rows.append({"scenario": sc.name, "pack": sc.pack, "periph": sc.periph,
                                      "cell": sc.cell, "cpb": cpb, "predictor": c[0],
                                      "budget_kb": c[1], "trace": "mean", **mean_rows(per)})
    write_csv(os.path.join(a.out, "ppa_long.csv"), long_rows)

    # ---- per budget: baseline table, sensitivity, break-even ----
    budgets = sorted({kb for _, kb in configs})
    e_corners = [(si, cpb, k) for si in range(len(scen)) for cpb in a.cpb for k in a.e_factors]
    a_corners = [(si, f) for si in range(len(scen)) for f in [None] + a.a_factors]
    table, sens, brk = [], [], []
    mp_src = os.path.basename(a.mpkbr) if over is not None else "RTL pass 1 (cold)"
    mp_label = f"MPKBr from {mp_src}"
    print(f"baseline {base_name}, CPB {a.cpb_base:g}, host {a.host_clk_ns} ns, predictor clock "
          f"{a.clk_ns} ns, ports {a.ports}; mean over the traces; energy per cond. branch")
    for kb in budgets:
        cs = [c for c in configs if c[1] == kb]
        bp = {c[0]: point(base, c, a.cpb_base) for c in cs}
        bp = {p: v for p, v in bp.items() if v and v["mpkbr"] is not None and v["e_total_pj"] is not None}
        if not bp:
            continue
        # sensitivity corners
        e_pts = []
        for si, cpb, k in e_corners:
            pts = {c[0]: point(si, c, cpb, k) for c in cs}
            pts = {p: v for p, v in pts.items() if v and v["mpkbr"] is not None
                   and v["e_total_pj"] is not None}
            if set(pts) == set(bp):
                e_pts.append(((si, cpb, k), pts))
        a_pts = []
        for si, f in a_corners:
            pts = {c[0]: point(si, c, a.cpb_base, 1.0, f) for c in cs}
            pts = {p: v for p, v in pts.items() if v and v["mpkbr"] is not None and v["area_um2"]}
            if set(pts) == set(bp):
                a_pts.append(((si, f), pts))
        front_e = pareto({p: (v["mpkbr"], v["e_total_pj"]) for p, v in bp.items()})
        front_a = pareto({p: (v["mpkbr"], v["area_um2"]) for p, v in bp.items()})
        rank_e = ranking({p: v["e_total_pj"] for p, v in bp.items()})
        rank_a = ranking({p: v["area_um2"] for p, v in bp.items()})
        hold_e = sum(ranking({p: v["e_total_pj"] for p, v in pts.items()}) == rank_e for _, pts in e_pts)
        hold_a = sum(ranking({p: v["area_um2"] for p, v in pts.items()}) == rank_a for _, pts in a_pts)
        on_e = {p: sum(p in pareto({q: (v["mpkbr"], v["e_total_pj"]) for q, v in pts.items()})
                       for _, pts in e_pts) for p in bp}
        on_a = {p: sum(p in pareto({q: (v["mpkbr"], v["area_um2"]) for q, v in pts.items()})
                       for _, pts in a_pts) for p in bp}

        print(f"\n=== {kb} KB  {mp_label}")
        print(f"  {'':6s} {'MPKBr':>7s} {'area um2':>9s} {'SRAM':>5s} {'E pJ':>7s} {'logic':>7s} "
              f"{'SRAM dyn':>8s} {'leak':>6s} {'t_SRAM ns':>11s} {'period ns':>11s} {'busy':>5s} "
              f"{'1RW':>5s} {'Pareto E/A':>10s}")
        for p in [q for q in PREDICTORS if q in bp]:
            v = bp[p]
            flags = ("E" if p in front_e else "-") + "/" + ("A" if p in front_a else "-")
            slow = "!" if v["busy_cyc"] is not None and v["busy_cyc"] > a.cpb_base else " "
            late = "!" if v.get("period_pes_ns") and v["period_pes_ns"] > a.clk_ns else " "
            print(f"  {SHORT[p]:6s} {v['mpkbr']:7.2f} {v['area_um2']:9.0f} {100 * v['area_sram_share']:4.0f}% "
                  f"{v['e_total_pj']:7.2f} {v['e_logic_pj']:7.2f} "
                  f"{v['e_sram_rd_pj'] + v['e_sram_wr_pj']:8.2f} {v['e_sram_leak_pj']:6.2f} "
                  f"{v['t_sram_opt_ns']:5.3f}-{v['t_sram_pes_ns']:5.3f} "
                  f"{v.get('period_opt_ns', 0):5.3f}-{v.get('period_pes_ns', 0):5.3f}{late}"
                  f"{v['busy_cyc']:5.1f}{slow} {int(v['n_1rw']):2d}/{int(v['n_sram']):<2d} {flags:>10s}")
            table.append({"budget_kb": kb, "predictor": p, "scenario": base_name, "cpb": a.cpb_base,
                          "mpkbr_src": mp_src,
                          "pareto_energy": int(p in front_e), "pareto_area": int(p in front_a), **v})
            es = [pts[p]["e_total_pj"] for _, pts in e_pts]
            ar = [pts[p]["area_um2"] for _, pts in a_pts]
            re_ = [ranking({q: w["e_total_pj"] for q, w in pts.items()}).index(p) + 1 for _, pts in e_pts]
            ra = [ranking({q: w["area_um2"] for q, w in pts.items()}).index(p) + 1 for _, pts in a_pts]
            sens.append({"budget_kb": kb, "predictor": p, "mpkbr": v["mpkbr"],
                         "e_base_pj": v["e_total_pj"], "e_min_pj": min(es), "e_max_pj": max(es),
                         "rank_e_base": rank_e.index(p) + 1, "rank_e_min": min(re_), "rank_e_max": max(re_),
                         "pareto_e_corners": on_e[p], "e_corners": len(e_pts),
                         "area_base_um2": v["area_um2"], "area_min_um2": min(ar), "area_max_um2": max(ar),
                         "rank_a_base": rank_a.index(p) + 1, "rank_a_min": min(ra), "rank_a_max": max(ra),
                         "pareto_a_corners": on_a[p], "a_corners": len(a_pts)})
        print(f"  energy ranking {' < '.join(SHORT[p] for p in rank_e)} holds in {hold_e}/{len(e_pts)} "
              f"corners; area ranking {' < '.join(SHORT[p] for p in rank_a)} in {hold_a}/{len(a_pts)}")
        print("  on the (MPKBr, energy) front in " + ", ".join(
            f"{SHORT[p]} {on_e[p]}/{len(e_pts)}" for p in bp) + " corners; (MPKBr, area) "
            + ", ".join(f"{SHORT[p]} {on_a[p]}/{len(a_pts)}" for p in bp))
        # break-even misprediction energy, pairs (more accurate x, less accurate y)
        for x in bp:
            for y in bp:
                if bp[x]["mpkbr"] >= bp[y]["mpkbr"]:
                    continue
                vals = []
                for _, pts in [(None, bp)] + e_pts:
                    dm = (pts[y]["mpkbr"] - pts[x]["mpkbr"]) / 1000.0
                    de = pts[x]["e_total_pj"] - pts[y]["e_total_pj"]
                    vals.append(None if dm <= 0 else max(0.0, de / dm) / 1000.0)   # nJ
                base_v, rng = vals[0], [v for v in vals[1:] if v is not None]
                if base_v is None:
                    continue
                brk.append({"budget_kb": kb, "accurate": x, "cheaper": y,
                            "d_mpkbr": bp[y]["mpkbr"] - bp[x]["mpkbr"],
                            "d_e_pj": bp[x]["e_total_pj"] - bp[y]["e_total_pj"],
                            "breakeven_nj": base_v, "breakeven_min_nj": min(rng) if rng else "",
                            "breakeven_max_nj": max(rng) if rng else ""})
                if base_v > 0:
                    span = f" ({min(rng):.2f}-{max(rng):.2f} over the corners)" if rng else ""
                    print(f"  {SHORT[x]} over {SHORT[y]}: pays off above {base_v:.2f} nJ per "
                          f"misprediction{span}")
                else:
                    print(f"  {SHORT[x]} dominates {SHORT[y]} (more accurate and cheaper)")

    write_csv(os.path.join(a.out, "ppa_table.csv"), table)
    write_csv(os.path.join(a.out, "sensitivity.csv"), sens)
    if brk:
        write_csv(os.path.join(a.out, "breakeven.csv"), brk)
    print(f"\nflags: busy '!' = busy cycles per branch > CPB (cannot keep up with the host); period '!' "
          f"= logic + SRAM over {a.clk_ns} ns in the pessimistic bound\n"
          f"not modelled: clock gating (the logic clocks idle flops on every busy cycle)\n"
          f"-> {a.out}/ppa_table.csv, ppa_long.csv, sensitivity.csv, breakeven.csv "
          f"({len(scen)} scenarios: {', '.join(names)})")


if __name__ == "__main__":
    main()