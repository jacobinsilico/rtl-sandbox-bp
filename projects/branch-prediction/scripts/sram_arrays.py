#!/usr/bin/env python3
"""
sram_arrays.py -- the physical SRAM arrays of the RTL selection, as the input
of cacti_cost.py (name,depth,width,ports).

One row per bp_sram instance of each selected config, with the geometry the
benches instantiate (the same formulas as tb_bp_<predictor>_core.sv):

    gshare             i_pht           2^PHT_INDEX_BITS x CTR_BITS (2)
    global perceptron  i_weights       NUM_PERCEPTRONS x (GHR_LEN + 1) * WEIGHT_BITS
    hashed perceptron  i_bias          BIAS_ENTRIES x BIAS_WEIGHT_BITS
                       g_wt[t].i_wt    2^TABLE_LOGS[t] x TABLE_WBITS[t], t < NUM_TABLES
    cookbook TAGE      g_tb[b].i_tb    2^LOGT x 2 * (TBITS + 5), b < NB (two entries per row;
                                       NB = NHIST/2 for the doubled pairs + shared arrays)
                       i_bp            2^LOGB x 1 (bimodal prediction bits)
                       i_bh            2^(LOGB - 1) x 2 (bimodal hysteresis)

Names are <predictor>/<budget>KB/<instance>, with the predictor names of
evaluate.py and the bench's instance names, so per-instance access counts
can be joined to the CACTI costs later. Every array is listed as 1r1w (the
port model of bp_sram); add 1rw rows by hand where an array never reads and
writes in the same cycle.

Also prints, per config, the bits of its arrays against the budget: the
tables should fill 90-100 % of it (the rest is history / counter registers
in the logic, which the C++ budget also counts).

USAGE (RTL repo root, after source sourceme.sh)
    /usr/bin/python3 projects/branch-prediction/scripts/sram_arrays.py
    /usr/bin/python3 projects/branch-prediction/scripts/sram_arrays.py --families tcb --budgets 1 2
    then:  cacti_cost.py projects/branch-prediction/eval/cacti/arrays.csv --vdd-hp <V> --vdd-lstp <V>
"""

import argparse
import csv
import os
import re
import sys

CBP_DEFAULT = "/home/j50065913/cbp5-wireless/cbp5-new-golden"
PRED_OF = {"gshare": "gshare", "gp": "g_perceptron", "hp": "hashed_perceptron", "tcb": "tage_cb"}
ORDER = ["gshare", "gp", "hp", "tcb"]


def defines(s):
    """-DK=V tokens -> {K: V string}."""
    out = {}
    for tok in (s or "").split():
        if tok.startswith("-D"):
            k, _, v = tok[2:].partition("=")
            out[k] = v
    return out


def need_int(d, k, cid, default=None):
    if k not in d:
        if default is not None:
            return default
        sys.exit(f"{cid}: no {k} in its defines")
    return int(d[k])


def int_list(d, k, cid):
    m = re.fullmatch(r"\{([\d,\s]+)\}", d.get(k, ""))
    if not m:
        sys.exit(f"{cid}: {k} is not a {{a,b,...}} list: {d.get(k)!r}")
    return [int(x) for x in m.group(1).split(",")]


def arrays(family, cid, d):
    """[(instance, depth, width)] of one config."""
    if family == "gshare":
        k = need_int(d, "PHT_INDEX_BITS", cid)
        return [("i_pht", 1 << k, need_int(d, "CTR_BITS", cid, 2))]
    if family == "gp":
        n, h, w = (need_int(d, k, cid) for k in ("NUM_PERCEPTRONS", "GHR_LEN", "WEIGHT_BITS"))
        return [("i_weights", n, (h + 1) * w)]
    if family == "hp":
        nt = need_int(d, "NUM_TABLES", cid)
        logs, wbits = int_list(d, "TABLE_LOGS", cid), int_list(d, "TABLE_WBITS", cid)
        if len(logs) < nt or len(wbits) < nt:
            sys.exit(f"{cid}: TABLE_LOGS / TABLE_WBITS shorter than NUM_TABLES={nt}")
        out = [("i_bias", need_int(d, "BIAS_ENTRIES", cid), need_int(d, "BIAS_WEIGHT_BITS", cid))]
        out += [(f"g_wt[{t}].i_wt", 1 << logs[t], wbits[t]) for t in range(nt)]
        return out
    if family == "tcb":
        nhist, logt, logb, tbits = (need_int(d, k, cid) for k in ("NHIST", "LOGT", "LOGB", "TBITS"))
        if need_int(d, "LOGASSOC", cid, 0) != 0:
            sys.exit(f"{cid}: LOGASSOC != 0, but bp_tage_core is direct-mapped")
        sh_off = 2 * ((nhist // 2 + 1) // 2)          # tb_bp_tage_core.sv localparams
        nb = (nhist - sh_off) // 2 + sh_off // 2
        out = [(f"g_tb[{b}].i_tb", 1 << logt, 2 * (tbits + 5)) for b in range(nb)]
        out += [("i_bp", 1 << logb, 1), ("i_bh", 1 << (logb - 1), 2)]
        return out
    sys.exit(f"unknown family {family!r}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selection", default=None, help="rtl_selection.csv (default: the CBP5 repo's)")
    ap.add_argument("--cbp", default=CBP_DEFAULT, help="CBP5 repo root")
    ap.add_argument("--out", default=None,
                    help="default: $REPO_HOME/projects/branch-prediction/eval/cacti/arrays.csv")
    ap.add_argument("--families", nargs="+", default=ORDER, choices=ORDER)
    ap.add_argument("--budgets", nargs="+", type=int, default=None)
    a = ap.parse_args()
    sel = a.selection or os.path.join(a.cbp, "scripts", "bp-comparison", "analysis",
                                      "traces_csv_long", "rtl_compare", "rtl_selection.csv")
    if not os.path.exists(sel):
        sys.exit(f"no selection {sel}: run compare_rtl.py --pick ... first")
    out = a.out
    if out is None:
        home = os.environ.get("REPO_HOME")
        if not home:
            sys.exit("REPO_HOME is not set: source sourceme.sh, or give --out")
        out = os.path.join(home, "projects", "branch-prediction", "eval", "cacti", "arrays.csv")

    rows = [r for r in csv.DictReader(open(sel, newline=""))
            if r["family"] in a.families and (not a.budgets or int(r["budget_kb"]) in a.budgets)]
    if not rows:
        sys.exit(f"no selection rows for {a.families} {a.budgets or ''} in {sel}")
    rows.sort(key=lambda r: (ORDER.index(r["family"]), int(r["budget_kb"])))

    lines, report, geoms = [], [], set()
    for r in rows:
        fam, kb, cid = r["family"], int(r["budget_kb"]), r["config_id"]
        if r.get("source") == "provisional":
            print(f"WARNING {cid}: provisional selection (before the rtl winners)")
        arr = arrays(fam, cid, defines(r["defines"]))
        bits = sum(dp * w for _, dp, w in arr)
        for inst, dp, w in arr:
            lines.append((f"{PRED_OF[fam]}/{kb}KB/{inst}", dp, w, "1r1w"))
            geoms.add((dp, w))
        report.append((fam, f"{kb} KB", len(arr), bits, f"{100.0 * bits / (kb * 8192):.1f}%",
                       ", ".join(f"{dp}x{w}" for _, dp, w in arr[:3]) + (" ..." if len(arr) > 3 else "")))

    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["name", "depth", "width", "ports"])
        w.writerows(lines)

    head = ("family", "budget", "arrays", "bits", "of budget", "geometry (depth x width)")
    wid = [max(len(str(x[i])) for x in [head] + report) for i in range(len(head))]
    print("  " + "  ".join(str(h).ljust(wid[i]) for i, h in enumerate(head)))
    print("  " + "  ".join("-" * x for x in wid))
    for x in report:
        print("  " + "  ".join(str(c).rjust(wid[i]) if i in (2, 3, 4) else str(c).ljust(wid[i])
                               for i, c in enumerate(x)))
    print(f"\n{len(lines)} arrays ({len(geoms)} distinct geometries) of {len(rows)} configs -> {out}")


if __name__ == "__main__":
    main()