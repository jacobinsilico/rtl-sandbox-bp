#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Block diagrams of the global perceptron predictor: bp_gp_top (bp_gp_core
#   and the weight-table bp_sram as boxes) and bp_gp_core.
#   Parameters are the RTL defaults unless overridden with -G (applied to both
#   views); output goes to yosys_block_diagram/gp/ (see yosys_show_common.sh
#   for OUT_DIR, FLAT, YOSYS).
#
# Usage:
#   scripts/yosys_block_diagram/show_gp.sh [-G NAME=VALUE ...]
#   e.g. scripts/yosys_block_diagram/show_gp.sh -G GHR_LEN=16 -G NUM_PERCEPTRONS=256
#        FLAT=1 scripts/yosys_block_diagram/show_gp.sh
# -----------------------------------------------------------------------------

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/yosys_show_common.sh"

bp_show gp bp_gp_top bp_gp_core "$@"