#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Block diagrams of the hashed perceptron predictor: bp_hp_top (bp_hp_core,
#   the bias table and the per-table bp_srams as boxes) and bp_hp_core.
#   Fewer tables give a much more readable core.
#   Parameters are the RTL defaults unless overridden with -G (applied to both
#   views); output goes to yosys_block_diagram/hp/ (see yosys_show_common.sh
#   for OUT_DIR, FLAT, YOSYS).
#
# Usage:
#   scripts/yosys_block_diagram/show_hp.sh [-G NAME=VALUE ...]
#   e.g. scripts/yosys_block_diagram/show_hp.sh -G NUM_TABLES=4
#        FLAT=1 scripts/yosys_block_diagram/show_hp.sh
# -----------------------------------------------------------------------------

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/yosys_show_common.sh"

bp_show hp bp_hp_top bp_hp_core "$@"