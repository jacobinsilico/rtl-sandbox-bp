#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Prints the PARAMS string for bp_hp_core / tb_bp_hp_core from the CBP5 .log
#   next to a hashed_perceptron golden dump. Everything comes from the log:
#   PREDICTOR_CONFIG holds the effective knob values (defaults included) and
#   PREDICTOR_HIST_LENS the history length of each table. Unused table slots
#   (t >= NUM_TABLES) get LOG = WBITS = HIST = 0, as the core requires.
#   Only plain HP with HP_HASH=1 (power-of-two tables) is accepted.
#
# Usage:
#   /usr/bin/python3 projects/branch-prediction/scripts/hp_params.py <dump>.log
#   P=$(/usr/bin/python3 projects/branch-prediction/scripts/hp_params.py "${STIM%.golden.txt}.log")
# -----------------------------------------------------------------------------

import re
import sys


def hp_params(log_text):
    """CBP5 .log of a hashed_perceptron run -> ordered {PARAM: value} for bp_hp_core."""
    m = re.search(r"^PREDICTOR_CONFIG hashed_perceptron (.*)$", log_text, re.M)
    if not m:
        sys.exit("hp_params: no 'PREDICTOR_CONFIG hashed_perceptron' line in the log")
    cfg = m.group(1).split()
    kv = dict(zip(cfg[0::2], cfg[1::2]))
    if kv.get("VARIANT") != "HP" or kv.get("HP_HASH") != "1":
        sys.exit(f"hp_params: plain HP with HP_HASH=1 only (VARIANT {kv.get('VARIANT')}, "
                 f"HP_HASH {kv.get('HP_HASH')})")
    if "TABLE_LOGS" not in kv:
        sys.exit("hp_params: the run must use TABLE_LOGS / TABLE_WBITS")

    n = int(kv["NUM_TABLES"])
    logs = [int(x) for x in kv["TABLE_LOGS"].strip("{}").split(",")]
    wbits = [int(x) for x in kv["TABLE_WBITS"].strip("{}").split(",")]
    h = re.search(r"^PREDICTOR_HIST_LENS((?: \d+)+)\s*$", log_text, re.M)
    if not h:
        sys.exit("hp_params: no PREDICTOR_HIST_LENS line in the log")
    hist = [int(x) for x in h.group(1).split()]
    if n > 8 or not (len(logs) == len(wbits) == len(hist) == n):
        sys.exit(f"hp_params: need NUM_TABLES <= 8 and one LOG/WBITS/HIST per table (NUM_TABLES {n})")

    p = {"NUM_TABLES": n}
    for t in range(8):
        p[f"T{t}_LOG"] = logs[t] if t < n else 0
    for t in range(8):
        p[f"T{t}_WBITS"] = wbits[t] if t < n else 0
    for t in range(8):
        p[f"T{t}_HIST"] = hist[t] if t < n else 0
    for rtl_key, log_key in (("BIAS_ENTRIES", "BIAS_ENTRIES"),
                             ("BIAS_WEIGHT_BITS", "BIAS_WEIGHT_BITS"),
                             ("THETA_BITS", "THETA_BITS"),
                             ("TC_BITS", "TC_BITS"),
                             ("HP_FOLDS", "HP_FOLDS"),
                             ("HP_PATH_BITS", "PATH_BITS"),
                             ("HP_PCMIX", "HP_PCMIX"),
                             ("HP_PC_SHIFT", "HP_PC_SHIFT")):
        p[rtl_key] = int(kv[log_key])
    return p


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: hp_params.py <dump>.log")
    with open(sys.argv[1]) as f:
        params = hp_params(f.read())
    print(" ".join(f"{k}={v}" for k, v in params.items()))


if __name__ == "__main__":
    main()