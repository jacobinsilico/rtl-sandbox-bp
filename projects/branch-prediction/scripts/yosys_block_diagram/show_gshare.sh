#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Block diagrams of the gshare predictor: bp_gshare_top (core and PHT
#   bp_sram as boxes; skipped if rtl/ has no bp_gshare_top.sv) and
#   bp_gshare_core (bp_sat_ctr as a box).
#   Parameters are the RTL defaults unless overridden with -G (applied to both
#   views); output goes to yosys_block_diagram/gshare/ (see yosys_show_common.sh
#   for OUT_DIR, FLAT, YOSYS).
#
# Usage:
#   scripts/yosys_block_diagram/show_gshare.sh [-G NAME=VALUE ...]
#   e.g. scripts/yosys_block_diagram/show_gshare.sh -G GHR_BITS=8 -G INDEX_BITS=12 -G PC_SHIFT=2
#        FLAT=1 scripts/yosys_block_diagram/show_gshare.sh
# -----------------------------------------------------------------------------

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/yosys_show_common.sh"

top=""
if [[ -f "$BP_PROJ_DIR/rtl/bp_gshare_top.sv" ]]; then
    top=bp_gshare_top
fi

bp_show gshare "$top" bp_gshare_core "$@"