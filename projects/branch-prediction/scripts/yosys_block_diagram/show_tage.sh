#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Block diagrams of the direct-mapped cookbook TAGE: bp_tage_top
#   (bp_tage_core, the tagged-bank bp_srams and the bimodal prediction and
#   hysteresis bp_srams as boxes) and bp_tage_core. The core is large at the
#   default NHIST; a small NHIST keeps it readable.
#   Parameters are the RTL defaults unless overridden with -G (applied to both
#   views); output goes to yosys_block_diagram/tage/ (see yosys_show_common.sh
#   for OUT_DIR, FLAT, YOSYS).
#
# Usage:
#   scripts/yosys_block_diagram/show_tage.sh [-G NAME=VALUE ...]
#   e.g. scripts/yosys_block_diagram/show_tage.sh -G NHIST=4
#        FLAT=1 scripts/yosys_block_diagram/show_tage.sh
# -----------------------------------------------------------------------------

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/yosys_show_common.sh"

bp_show tage bp_tage_top bp_tage_core "$@"