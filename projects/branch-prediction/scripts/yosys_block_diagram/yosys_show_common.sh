#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Shared helper for the show_*.sh scripts (sourced, not executed). Renders
#   two block diagrams per predictor with the yosys-slang frontend into
#   yosys_block_diagram/<name>/:
#     <top>.svg  - the integration top with its instances (core, bp_sram
#                  tables) as boxes; bp_sram is blackboxed, so its reset loop
#                  is never unrolled
#     <core>.svg - the core's own logic, its submodules (e.g. bp_sat_ctr) as
#                  boxes (FLAT=1: fully flattened)
#   Only the named modules are rendered, never every specialized submodule.
#   Each view: opt; wreduce; opt_clean, then show -width -stretch -colors 2.
#
# Environment:
#   YOSYS   - yosys binary (default: yosys)
#   OUT_DIR - output directory (default: <project>/yosys_block_diagram/<name>)
#   FLAT    - 1: flatten the core view (default: 0)
# -----------------------------------------------------------------------------

BP_PROJ_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# bp_show <name> <top or ""> <core> [-G NAME=VALUE ...]
bp_show() {
    local name="$1" top="$2" core="$3"
    shift 3
    local yosys="${YOSYS:-yosys}"
    local out_dir="${OUT_DIR:-$BP_PROJ_DIR/yosys_block_diagram/$name}"
    local hier="--best-effort-hierarchy"
    if [[ "${FLAT:-0}" == 1 ]]; then
        hier=""
    fi
    local show_opts="-width -stretch -colors 2"

    if ! command -v "$yosys" > /dev/null; then
        echo "error: '$yosys' not found" >&2
        return 1
    fi
    if ! command -v dot > /dev/null; then
        echo "error: graphviz 'dot' not found (sudo apt install graphviz)" >&2
        return 1
    fi
    if [[ "$BP_PROJ_DIR $out_dir" == *" "*" "* ]]; then
        echo "error: paths containing spaces are not supported" >&2
        return 1
    fi

    local srcs=("$BP_PROJ_DIR"/rtl/*.sv)
    if [[ ! -e "${srcs[0]}" ]]; then
        echo "error: no sources in $BP_PROJ_DIR/rtl" >&2
        return 1
    fi

    mkdir -p "$out_dir"
    rm -f "$out_dir"/*.dot "$out_dir"/*.svg "$out_dir"/modules.txt

    # one command per line: a newline ends a yosys command
    {
        if [[ -n "$top" ]]; then
            echo "read_slang --best-effort-hierarchy --blackboxed-module bp_sram --top $top $* ${srcs[*]}"
            echo "opt; wreduce; opt_clean"
            echo "show -format svg -prefix $out_dir/$top $show_opts $top"
            echo "design -reset"
        fi
        echo "read_slang $hier --top $core $* ${srcs[*]}"
        echo "opt; wreduce; opt_clean"
        echo "show -format svg -prefix $out_dir/$core $show_opts $core"
    } > "$out_dir/show.ys"

    echo "[$name] ${top:+$top + }$core $*"
    if ! "$yosys" -m slang -q -l "$out_dir/yosys.log" -s "$out_dir/show.ys"; then
        echo "error: yosys failed, see $out_dir/yosys.log" >&2
        return 1
    fi
    rm -f "$out_dir"/*.dot

    local f
    for f in "$out_dir"/*.svg; do
        echo "  $f"
    done
}