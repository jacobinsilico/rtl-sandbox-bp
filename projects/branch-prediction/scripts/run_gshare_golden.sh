#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Runs tb_bp_gshare_core (make sim) on the gshare golden dumps under
#   projects/<project>/stim/gshare/ and prints one PASS/FAIL line per dump.
#   <project> is the project this script lives in (projects/<project>/scripts/). The RTL
#   parameters are taken from each dump's .json (the CBP5 -D defines), so the
#   RTL is always built with exactly the config that produced the dump.
#
# Usage (from the repository root, after `source sourceme.sh`):
#   bash projects/<project>/scripts/run_gshare_golden.sh [MAX_LINES] [BUDGET_GLOB] [TRACE_GLOB]
#     MAX_LINES    branch lines per run, 0 = whole trace (default 0)
#     BUDGET_GLOB  e.g. 4KB (default: all budgets)
#     TRACE_GLOB   e.g. 'fdd*' (default: all traces)
#   CLK_PERIOD_NS from the environment (default 1.0).
#
#   Example, quick smoke test of one dump:
#     bash projects/<project>/scripts/run_gshare_golden.sh 100000 4KB 'fdd*'
#
# Output:
#   projects/<project>/sim/sim_gshare_<budget>_<trace>/   (make sim run directory)
#   projects/<project>/sim/golden_logs/<run>.log          (full make + sim output)
# -----------------------------------------------------------------------------

set -u

REPO="${REPO_HOME:?run 'source sourceme.sh' from the repository root first}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT="$(basename "$PROJ_DIR")"
STIM_DIR="$PROJ_DIR/stim/gshare"
MAX_LINES="${1:-0}"
BUDGET_GLOB="${2:-*}"
TRACE_GLOB="${3:-*}"
CLK="${CLK_PERIOD_NS:-1.0}"
LOG_DIR="$PROJ_DIR/sim/golden_logs"
mkdir -p "$LOG_DIR"

# How the stimulus path reaches the bench's STIM_FILE string parameter.
# Verilator needs the value in double quotes (-GSTIM_FILE="/abs/path").
# If scripts/sim/run.sh strips or re-quotes PARAMS differently, change
# only this function.
stim_param() { printf 'STIM_FILE="%s"' "$1"; }

# RTL parameters from the dump's .json defines.
params_from_json() {
    python3 - "$1" <<'EOF'
import json, re, sys
d = dict(re.findall(r"-D(\w+)=(\S+)", json.load(open(sys.argv[1]))["defines"]))
ghr, idx = int(d["GHR_BITS"]), int(d["PHT_INDEX_BITS"])
ent, sh = int(d["PHT_ENTRIES"]), int(d.get("PC_SHIFT", 0))
if ent != 1 << idx:
    sys.exit(f"PHT_ENTRIES {ent} != 2^PHT_INDEX_BITS ({1 << idx})")
print(f"GHR_BITS={ghr} INDEX_BITS={idx} PC_SHIFT={sh}")
EOF
}

if [ ! -d "$STIM_DIR" ]; then
    echo "ERROR: no gshare dumps: $STIM_DIR does not exist" >&2
    exit 2
fi
echo "project $PROJECT, dumps in $STIM_DIR"

results=()
for dir in "$STIM_DIR"/${BUDGET_GLOB}__*/; do
    [ -d "$dir" ] || continue
    budget=$(basename "$dir"); budget=${budget%%__*}
    for dump in "$dir"${TRACE_GLOB}.golden.txt; do
        [ -f "$dump" ] || continue
        trace=$(basename "$dump" .golden.txt)
        json="${dump%.golden.txt}.json"
        if ! params=$(params_from_json "$json"); then
            results+=("FAIL  $budget  $trace  (cannot read params from $json)")
            continue
        fi
        run="sim_gshare_${budget}_${trace%%.*}"
        log="$LOG_DIR/$run.log"
        echo "=== $budget  $trace  [$params]  -> $log"
        make -C "$REPO" sim PROJECT="$PROJECT" TOP_LEVEL=bp_gshare_core CLK_PERIOD_NS="$CLK" OUT_DIR="$run" \
             PARAMS="$params MAX_LINES=$MAX_LINES $(stim_param "$dump")" > "$log" 2>&1
        rc=$?
        if [ $rc -eq 0 ] && grep -q "PASSED" "$log"; then
            mis=$(grep -m1 "TB_RTL_MISPRED" "$log" | awk '{print $NF}')
            cond=$(grep -m1 "TB_COND_BR" "$log" | awk '{print $NF}')
            results+=("PASS  $budget  $trace  cond=$cond mispred=$mis")
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
echo "================ SUMMARY (MAX_LINES=$MAX_LINES) ================"
printf '%s\n' "${results[@]}"
printf '%s\n' "${results[@]}" | grep -q "^FAIL" && exit 1
exit 0