#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Full flow (RTL check, synthesis, timing, gate-level simulation, power) for
#   the g_perceptron predictor on every budget with golden dumps. A thin entry point
#   to evaluate.py; every evaluate.py option is passed through, e.g.
#     bash projects/branch-prediction/scripts/evaluate_gp.sh --budgets 4KB
#     bash projects/branch-prediction/scripts/evaluate_gp.sh --rtl-lines 100000
#     bash projects/branch-prediction/scripts/evaluate_gp.sh --dry-run
#   Results: projects/branch-prediction/eval/g_perceptron_results.csv
# -----------------------------------------------------------------------------
exec python3 "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/evaluate.py" g_perceptron "$@"