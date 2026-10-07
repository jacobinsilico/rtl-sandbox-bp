#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Runs a predictor's RTL bench (make sim) on its golden dumps under
#   projects/<project>/stim/<predictor>/ and prints one PASS/FAIL line per
#   dump. The RTL parameters are derived from each dump's .json (the CBP5 -D
#   defines), so the RTL is always built with exactly the config that
#   produced the dump. <project> is the project this script lives in
#   (projects/<project>/scripts/).
#
#   Predictors (stim directory name -> RTL top, parameters from the defines):
#     gshare        -> bp_gshare_core  GHR_BITS INDEX_BITS(=PHT_INDEX_BITS) PC_SHIFT
#     g_perceptron  -> bp_gp_core      GHR_LEN NUM_PERCEPTRONS WEIGHT_BITS
#                                      THETA_ALPHA_PCT PC_SHIFT
#   Add a predictor by adding an entry to params_from_json below.
#
# Usage (from anywhere, after `source sourceme.sh`):
#   bash projects/<project>/scripts/run_golden.sh PREDICTOR [MAX_LINES] [BUDGET_GLOB] [TRACE_GLOB]
#     PREDICTOR    gshare | g_perceptron
#     MAX_LINES    branch lines per run, 0 = whole trace (default 0)
#     BUDGET_GLOB  e.g. 4KB (default: all budgets)
#     TRACE_GLOB   e.g. 'fdd*' (default: all traces)
#   Environment:
#     CLK_PERIOD_NS  clock period (default 1.0)
#     EXTRA_PARAMS   appended to PARAMS, e.g. EXTRA_PARAMS="Y_REG=1"
#
#   Example, quick smoke test of one dump:
#     bash projects/branch-prediction/scripts/run_golden.sh g_perceptron 1000 4KB 'fdd*'
#
# Output:
#   projects/<project>/sim/sim_<predictor>_<budget>_<trace>/  (make sim run)
#   projects/<project>/sim/golden_logs/<run>.log              (full output)
# -----------------------------------------------------------------------------

set -u

if [ $# -lt 1 ]; then
    echo "usage: $0 PREDICTOR [MAX_LINES] [BUDGET_GLOB] [TRACE_GLOB]   (PREDICTOR: gshare | g_perceptron)" >&2
    exit 2
fi

PRED="$1"
MAX_LINES="${2:-0}"
BUDGET_GLOB="${3:-*}"
TRACE_GLOB="${4:-*}"
CLK="${CLK_PERIOD_NS:-1.0}"
EXTRA="${EXTRA_PARAMS:-}"

REPO="${REPO_HOME:?run 'source sourceme.sh' from the repository root first}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT="$(basename "$PROJ_DIR")"
STIM_DIR="$PROJ_DIR/stim/$PRED"
LOG_DIR="$PROJ_DIR/sim/golden_logs"
mkdir -p "$LOG_DIR"

# How the stimulus path reaches the bench's STIM_FILE string parameter:
# Verilator needs -GSTIM_FILE="/abs/path" with the quotes (paths: no spaces).
stim_param() { printf 'STIM_FILE="%s"' "$1"; }

# "<top>|<PARAMS>" for one dump, from its .json defines. Exits non-zero with
# a message when the defines cannot be mapped onto the RTL.
params_from_json() {
    python3 - "$1" "$2" <<'EOF'
import json, re, sys
pred, path = sys.argv[1], sys.argv[2]
d = dict(re.findall(r"-D(\w+)=(\S+)", json.load(open(path))["defines"]))

def need(key, default=None):
    if key in d:
        return int(d[key])
    if default is not None:
        return default
    sys.exit(f"{path}: -D{key} missing")

if pred == "gshare":
    ghr, idx, ent = need("GHR_BITS"), need("PHT_INDEX_BITS"), need("PHT_ENTRIES")
    sh = need("PC_SHIFT", 0)
    if ent != 1 << idx:
        sys.exit(f"PHT_ENTRIES {ent} != 2^PHT_INDEX_BITS ({1 << idx})")
    print(f"bp_gshare_core|GHR_BITS={ghr} INDEX_BITS={idx} PC_SHIFT={sh}")
elif pred == "g_perceptron":
    h, n, w = need("GHR_LEN"), need("NUM_PERCEPTRONS"), need("WEIGHT_BITS")
    a, sh = need("THETA_ALPHA_PCT", 100), need("PC_SHIFT", 0)   # g_perceptron.h defaults
    if not 1 <= h <= 64:
        sys.exit(f"GHR_LEN {h} outside the RTL range 1..64")
    if n < 2:
        sys.exit(f"NUM_PERCEPTRONS {n} not supported by the RTL (needs >= 2)")
    print(f"bp_gp_core|GHR_LEN={h} NUM_PERCEPTRONS={n} WEIGHT_BITS={w} "
          f"THETA_ALPHA_PCT={a} PC_SHIFT={sh}")
else:
    sys.exit(f"unknown predictor '{pred}' (known: gshare, g_perceptron)")
EOF
}

if [ ! -d "$STIM_DIR" ]; then
    echo "ERROR: no dumps for '$PRED': $STIM_DIR does not exist" >&2
    exit 2
fi
echo "project $PROJECT, predictor $PRED, dumps in $STIM_DIR"

value() { grep -m1 "^$1 " "$2" | awk '{print $NF}'; }

results=()
for dir in "$STIM_DIR"/${BUDGET_GLOB}__*/; do
    [ -d "$dir" ] || continue
    budget=$(basename "$dir"); budget=${budget%%__*}
    for dump in "$dir"${TRACE_GLOB}.golden.txt; do
        [ -f "$dump" ] || continue
        trace=$(basename "$dump" .golden.txt)
        json="${dump%.golden.txt}.json"
        if ! mapped=$(params_from_json "$PRED" "$json"); then
            results+=("FAIL  $budget  $trace  (cannot map the defines in $json)")
            continue
        fi
        top=${mapped%%|*}
        params=${mapped#*|}
        [ -n "$EXTRA" ] && params="$params $EXTRA"
        run="sim_${PRED}_${budget}_${trace%%.*}"
        log="$LOG_DIR/$run.log"
        echo "=== $budget  $trace  $top [$params]  -> $log"
        make -C "$REPO" sim PROJECT="$PROJECT" TOP_LEVEL="$top" CLK_PERIOD_NS="$CLK" OUT_DIR="$run" \
             PARAMS="$params MAX_LINES=$MAX_LINES $(stim_param "$dump")" > "$log" 2>&1
        rc=$?
        if [ $rc -eq 0 ] && grep -q "PASSED" "$log"; then
            line="PASS  $budget  $trace  cond=$(value TB_COND_BR "$log")"
            line="$line mispred=$(value TB_RTL_MISPRED "$log") cycles=$(value TB_CYCLES "$log")"
            line="$line rd=$(value TB_SRAM_READS "$log") wr=$(value TB_SRAM_WRITES "$log")"
            results+=("$line")
        else
            results+=("FAIL  $budget  $trace  (exit $rc, see $log)")
        fi
    done
done

if [ ${#results[@]} -eq 0 ]; then
    echo "ERROR: no dump matched $STIM_DIR/${BUDGET_GLOB}__*/${TRACE_GLOB}.golden.txt" >&2
    echo "       budget dirs present:" >&2
    ls -1 "$STIM_DIR" >&2
    exit 2
fi

echo
echo "================ SUMMARY  $PRED  (MAX_LINES=$MAX_LINES) ================"
printf '%s\n' "${results[@]}"
printf '%s\n' "${results[@]}" | grep -q "^FAIL" && exit 1
exit 0