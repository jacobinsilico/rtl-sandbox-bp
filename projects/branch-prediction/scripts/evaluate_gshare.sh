#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Full flow (RTL check, synthesis, timing, gate-level simulation, power) for
#   the gshare predictor on every budget with golden dumps. A thin entry point
#   to evaluate.py; every evaluate.py option is passed through, e.g.
#     bash projects/branch-prediction/scripts/evaluate_gshare.sh --budgets 4KB
#     bash projects/branch-prediction/scripts/evaluate_gshare.sh --rtl-lines 100000
#     bash projects/branch-prediction/scripts/evaluate_gshare.sh --dry-run
#   Results: projects/branch-prediction/eval/gshare_results.csv
# -----------------------------------------------------------------------------
exec python3 "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/evaluate.py" gshare "$@"