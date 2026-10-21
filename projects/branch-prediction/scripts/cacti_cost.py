#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Costs each SRAM array with CACTI at 22 nm (the smallest node CACTI accepts)
#   and scales the results to 7 nm with stillmaker_scale.py (same directory),
#   so they can be added to the ASAP7 logic numbers.
#
#   Input: arrays.csv of sram_arrays.py, one row per physical bp_sram instance:
#     name,depth,width[,ports]     width in bits; ports 1r1w (default) or 1rw
#   name is <predictor>/<budget>KB/<bench instance>, the join key of the
#   per-instance access counts.
#
#   Per array, port model and cell type (--cells) the script
#     1. sets the geometry: depth up to a power of two; arrays narrower than
#        --pack-bits get floor(pack/width) entries per row; rows up to whole
#        bytes. --pack-mode exact first tries 8/gcd(width, 8) entries per row
#        instead (a 5-bit table: 8 entries in a 5-byte row, no unused bits;
#        a read then senses the whole row), and falls back to the byte rows.
#        util = stored bits / costed bits shows the overestimate;
#     2. writes a config from the template (CACTI's cache.cfg): "ram", UCA,
#        1-way, one row per access, 1R1W (0 RW + 1 R + 1 W ports) or 1RW,
#        22 nm, itrs-<cell> cells, itrs-<--periph> periphery, no ECC,
#        DETAILED, --temp-k, and CACTI's own supplies (a CACTI 7
#        "-<type> Vdd" line is set to "default");
#     3. runs CACTI. If it finds no organization, it pads the row to a
#        power-of-two number of bytes, then doubles the depth up to --max-pad
#        times, and records the padding (an overestimate, like a compiler's
#        minimum macro size). An array that still fails is status=fail: cost
#        it as flops in the combine step;
#     4. parses access time and its components (H-tree in / out, bitline),
#        read and write energy, leakage (subthreshold + gate), area and area
#        efficiency. The cells' share of the leakage: when both cell types are
#        costed with the same geometry, leak(hp cells) - leak(lstp cells),
#        i.e. LSTP cell leakage taken as ~0 (leak_split says so). CACTI's own
#        breakdown lines are never used: CACTI 7.0 copies them into its
#        result only when power gating is on, otherwise they are
#        uninitialized memory (zeros, or garbage);
#     5. scales to 7 nm at --dst-vdd: dynamic energy and the periphery's
#        leakage with the periphery's row, the cells' leakage with the cell
#        type's row, area with the Table 3 geometric mean. Delay: the bitline
#        part (set by the cell's read current) with the cell type's row, the
#        rest with the periphery's row (opt); the pessimistic bound (pes)
#        also holds the H-tree part (wires) unscaled. CACTI's 22 nm is
#        Stillmaker's 20 nm multi-gate row. --calib-fo4-ps replaces
#        Stillmaker's 7 nm FO4 of the periphery with a measured ASAP7 FO4
#        (delay only).
#   --ports both (default) costs every array as 1r1w AND 1rw, so the combine
#   step can take 1rw where the bench saw no same-cycle read + write.
#   Identical geometries are run once (--jobs in parallel). Raw configs and
#   outputs stay in OUT/raw/; a raw output is reused while its config is
#   unchanged (--rerun forces new runs).
#
#   Output:
#     OUT/cacti_arrays.csv   one row per array x port model x cell type.
#       *_22 = CACTI at 22 nm, *_7 = scaled (t_bitline_22 / t_htree_22: the
#       access time's bitline and H-tree parts). Time in ns, energy in pJ per row
#       access, leakage in uW, area in um^2. cyc_opt / cyc_pes = cycles at
#       --clk-ns. *_7_used: the unused bits' cell area / cell leakage taken
#       out (periphery kept), a lower estimate next to the costed one
#       (leak_7_used only when the cells' share is known).
#       cell_um2_22 = cell area per costed bit (check it against a real
#       22 nm cell).
#     OUT/cacti_scaling.json  supplies, factors, CACTI binary and template
#   Parsing is tolerant: a value that cannot be found is left empty and a
#   WARNING names the raw file.
#
# Usage (after `source sourceme.sh`, with CACTI_BIN set):
#   /usr/bin/python3 projects/branch-prediction/scripts/cacti_cost.py [ARRAYS.csv] \
#       --vdd-hp V --vdd-lstp V [options]
#     ARRAYS.csv           default: projects/branch-prediction/eval/cacti/arrays.csv
#     --vdd-hp, --vdd-lstp CACTI's 22 nm itrs-hp and itrs-lstp supplies: the
#                          first two values of '-Vdd (V)' in tech_params/22nm.dat
#                          (CACTI 7) or vdd[0], vdd[1] of technology.cc (6.5);
#                          cross-checked against the files next to the binary
#     --cells hp lstp      data-array cell types (default: both)
#     --ports both|csv     both port models (default), or the CSV's column
#     --periph hp|lstp     periphery device type (default hp)
#     --temp-k K           CACTI temperature, 300-400 K in steps of 10 (default
#                          300, ~ the 25 C of the ASAP7 TT libraries; CACTI's
#                          own default is 360)
#     --out DIR            default: projects/branch-prediction/eval/cacti
#     --dst-vdd V          7 nm supply (default 0.7, ASAP7 nominal)
#     --clk-ns T           cycle time for cyc_* (default 0.9)
#     --calib-fo4-ps P     measured ASAP7 FO4 at --dst-vdd (default: Table 5)
#     --pack-bits B        pack arrays narrower than B bits (default 8, 0 = off)
#     --pack-mode M        byte (default) or exact (see 1.)
#     --max-pad N          depth doublings tried after row padding (default 6)
#     --jobs N             parallel CACTI runs (default 4)
#     --timeout S          seconds per CACTI run (default 900)
#     --cacti PATH         CACTI binary (default: $CACTI_BIN)
#     --template PATH      base config (default: cache.cfg next to the binary)
#     --rerun              ignore raw outputs from earlier runs
#
#   Example:
#     /usr/bin/python3 projects/branch-prediction/scripts/sram_arrays.py
#     /usr/bin/python3 projects/branch-prediction/scripts/cacti_cost.py \
#         --vdd-hp 0.8 --vdd-lstp 0.8 --cacti ~/eda/cacti/cacti
#     exact-bit rows for the sub-byte tables, into its own directory:
#     /usr/bin/python3 projects/branch-prediction/scripts/cacti_cost.py \
#         --vdd-hp 0.8 --vdd-lstp 0.8 --pack-mode exact \
#         --out projects/branch-prediction/eval/cacti_exact
#     leakage bracket with LSTP periphery, into its own directory:
#     /usr/bin/python3 projects/branch-prediction/scripts/cacti_cost.py \
#         --vdd-hp 0.8 --vdd-lstp 0.8 --periph lstp \
#         --out projects/branch-prediction/eval/cacti_plstp
# -----------------------------------------------------------------------------
import argparse
import csv
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJ_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, SCRIPT_DIR)
from stillmaker_scale import factors, vdd_range  # noqa: E402

SRC_NODE, DST_NODE = 20, 7          # CACTI 22 nm = Stillmaker's 20 nm multi-gate row
PORTS = ("1r1w", "1rw")

NUM = r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
FIELDS = {  # name: patterns, first match wins (the summary block comes first)
    "t_acc": [r"Access time \(ns\):\s*" + NUM],
    "t_cyc": [r"Cycle time \(ns\):\s*" + NUM],
    "t_htree_in": [r"H-tree input delay \(ns\):\s*" + NUM],
    "t_htree_out": [r"H-tree output delay \(ns\):\s*" + NUM],
    "t_bitline": [r"Bitline delay \(ns\):\s*" + NUM],
    "e_rd": [r"Total dynamic read energy per access \(nJ\):\s*" + NUM,
             r"dynamic read energy[^:\n]*\(nJ\):\s*" + NUM],
    "e_wr": [r"Total dynamic write energy per access \(nJ\):\s*" + NUM,
             r"dynamic write energy[^:\n]*\(nJ\):\s*" + NUM],
    "leak": [r"Total leakage power of a bank \(mW\):\s*" + NUM,
             r"Total leakage power of a bank without power gating[^:\n]*\(mW\):\s*" + NUM,
             r"Total leakage power of a bank[^:\n]*\(mW\):\s*" + NUM],
    "leak_gate": [r"Total gate leakage power of a bank[^:\n]*\(mW\):\s*" + NUM],
    "leak_cells": [r"Total leakage power in cells[^:\n]*\(mW\):\s*" + NUM],
    "eff_pct": [r"Area efficiency[^\n]*?-\s*" + NUM + r"\s*%"],
}
AREA_RES = [r"height x width \(mm\):\s*" + NUM + r"\s*x\s*" + NUM,
            r"Data array: Area \(mm2\):\s*" + NUM]
NEEDED = ("t_acc", "e_rd", "e_wr", "leak", "area_mm2")

# Template keys replaced by the lines of make_cfg (commented out in the template)
KEYS = ["-size (bytes)", "-block size (bytes)", "-associativity", "-read-write port",
        "-exclusive read port", "-exclusive write port", "-single ended read ports",
        "-UCA bank count", "-technology (u)", "-cache type", "-Cache model",
        "-output/input bus width", "-Data array cell type", "-Data array peripheral type",
        "-access mode", "-print level", "-Add ECC", "-Force cache config",
        "-operating temperature"]
VDD_RE = re.compile(r"^(\s*-\w+ Vdd(?: \(V\))?)\s*(.*)$")     # CACTI 7 user supplies


def pow2(n):
    return 1 << max(0, math.ceil(math.log2(n)))


def make_cfg(template, depth, row_bytes, ports, cell, periph="hp", temp_k=300):
    rw, er, ew = (0, 1, 1) if ports == "1r1w" else (1, 0, 0)
    ours = [f"-size (bytes) {depth * row_bytes}",
            f"-block size (bytes) {row_bytes}",
            "-associativity 1",
            f"-read-write port {rw}",
            f"-exclusive read port {er}",
            f"-exclusive write port {ew}",
            "-single ended read ports 0",
            "-UCA bank count 1",
            "-technology (u) 0.022",
            '-cache type "ram"',
            '-Cache model (NUCA, UCA)  - "UCA"',
            f"-output/input bus width {row_bytes * 8}",
            f'-Data array cell type - "itrs-{cell}"',
            f'-Data array peripheral type - "itrs-{periph}"',
            f"-operating temperature (K) {temp_k}",
            '-access mode (normal, sequential, fast) - "normal"',
            '-print level (DETAILED, CONCISE) - "DETAILED"',
            '-Add ECC - "false"',
            '-Force cache config - "false"']
    out = []
    for line in template:
        s = line.lstrip()
        m = VDD_RE.match(line)
        if any(s.startswith(k) for k in KEYS):
            out.append("//" + line)
        elif m and s.startswith("-"):
            out.append(m.group(1) + ' "default"')          # CACTI's own vdd[]
        else:
            out.append(line)
    return "\n".join(out + ["", "// cacti_cost.py overrides"] + ours) + "\n"


def template_vdd_overrides(template):
    """CACTI 7 '-<type> Vdd' lines of the template that are not "default"."""
    out = []
    for line in template:
        m = VDD_RE.match(line)
        if m and line.lstrip().startswith("-") and "default" not in m.group(2):
            out.append(line.strip())
    return out


def tech_vdd_22(cacti_dir):
    """CACTI's 22 nm supplies per device type (index = ram_cell_tech_type:
    0 itrs-hp, 1 itrs-lstp, 2 itrs-lop, ...) -> ({i: V}, path). CACTI 7:
    the '-Vdd (V)' line of tech_params/22nm.dat; CACTI 6.5: vdd[i] = V in the
    first 22 nm branch of technology.cc."""
    dat = os.path.join(cacti_dir, "tech_params", "22nm.dat")
    try:
        for ln in open(dat, errors="replace"):
            if re.match(r"\s*-Vdd\b", ln):
                vals = [float(x) for x in re.findall(r"(?<![\w.])\d*\.?\d+", ln.split("Vdd", 1)[1])]
                if vals:
                    return dict(enumerate(vals)), dat
    except OSError:
        pass
    path = os.path.join(cacti_dir, "technology.cc")
    try:
        lines = open(path, errors="replace").read().splitlines()
    except OSError:
        return {}, path
    out, on = {}, False
    for ln in lines:
        s = ln.split("//")[0]
        m = re.search(r"\btech\s*==\s*(\d+)", s)
        if m:
            if on and out:
                break
            on = m.group(1) == "22"
            continue
        if on:
            m = re.search(r"\bvdd\[(\d)\]\s*=\s*([\d.]+)", s)
            if m:
                out.setdefault(int(m.group(1)), float(m.group(2)))
    return out, path


def parse(text, path):
    vals, missing = {}, []
    for name, pats in FIELDS.items():
        for p in pats:
            m = re.search(p, text, re.IGNORECASE)
            if m:
                vals[name] = float(m.group(1))
                break
        else:
            missing.append(name)
    m = re.search(AREA_RES[0], text, re.IGNORECASE)
    if m:
        vals["area_mm2"] = float(m.group(1)) * float(m.group(2))
    else:
        m = re.search(AREA_RES[1], text, re.IGNORECASE)
        if m:
            vals["area_mm2"] = float(m.group(1))
        else:
            missing.append("area")
    if missing:
        print(f"WARNING: {path}: not found: {', '.join(missing)}", file=sys.stderr)
    return vals


class Runner:
    """Runs CACTI once per distinct config; reuses raw outputs of the same config."""

    def __init__(self, a, template, raw_dir):
        self.a, self.template, self.raw_dir = a, template, raw_dir
        self.locks, self.guard = {}, threading.Lock()
        self.ran = self.reused = 0

    def lock(self, tag):
        with self.guard:
            return self.locks.setdefault(tag, threading.Lock())

    def run_one(self, depth, row_bytes, ports, cell):
        tag = f"{cell}_{ports}_{depth}x{row_bytes}B"
        cfg_path = os.path.join(self.raw_dir, tag + ".cfg")
        out_path = os.path.join(self.raw_dir, tag + ".txt")
        cfg = make_cfg(self.template, depth, row_bytes, ports, cell, self.a.periph,
                       self.a.temp_k)
        with self.lock(tag):
            old = open(cfg_path).read() if os.path.exists(cfg_path) else None
            if self.a.rerun or old != cfg or not os.path.exists(out_path):
                with open(cfg_path, "w") as f:
                    f.write(cfg)
                try:
                    r = subprocess.run([self.a.cacti, "-infile", cfg_path], capture_output=True,
                                       text=True, cwd=os.path.dirname(self.a.cacti),
                                       timeout=self.a.timeout)
                    text = r.stdout + r.stderr + f"\n// cacti_cost.py: exit {r.returncode}\n"
                except subprocess.TimeoutExpired:
                    text = f"// cacti_cost.py: TIMEOUT after {self.a.timeout} s\n"
                with open(out_path, "w") as f:
                    f.write(text)
                with self.guard:
                    self.ran += 1
            else:
                with self.guard:
                    self.reused += 1
            text = open(out_path, errors="replace").read()
        if not re.search(r"Access time \(ns\):\s*" + NUM, text):
            return None, out_path
        return parse(text, out_path), out_path

    def ladder(self, job):
        """The start geometries as given, then the last one's row padded to a
        power of two, then deeper. -> (vals, rows, bytes, entries per row,
        index of the step that worked, path)."""
        starts, ports, cell = job
        k, depth, row_bytes = starts[-1]
        steps = [(d, b, kk) for kk, d, b in starts]
        if pow2(row_bytes) != row_bytes:
            steps.append((depth, pow2(row_bytes), k))
        rb = steps[-1][1]
        steps += [(depth << n, rb, k) for n in range(1, self.a.max_pad + 1)]
        path = None
        for i, (d, b, kk) in enumerate(steps):
            vals, path = self.run_one(d, b, ports, cell)
            if vals is not None:
                return vals, d, b, kk, i, path
        return None, None, None, None, None, path


def geometry(depth, width, pack_bits, mode="byte"):
    """-> start geometries ((entries per row, rows, bytes per row), ...) to try
    in order: the exact-bit rows (--pack-mode exact, if they differ), then the
    byte rows."""
    k = pack_bits // width if 0 < width < pack_bits else 1
    byte = (k, pow2(math.ceil(depth / k)), math.ceil(k * width / 8))
    if mode == "exact" and 0 < width < pack_bits:
        kx = 8 // math.gcd(width, 8)
        exact = (kx, pow2(math.ceil(depth / kx)), kx * width // 8)
        if exact[1:] != byte[1:]:
            return (exact, byte)
    return (byte,)


def read_arrays(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(l for l in f if l.strip() and not l.lstrip().startswith("#")))
    seen, out = set(), []
    for r in rows:
        name = (r.get("name") or "").strip()
        try:
            depth, width = int(r["depth"]), int(r["width"])
        except (KeyError, TypeError, ValueError):
            sys.exit(f"{path}: bad depth/width in row {r}")
        ports = (r.get("ports") or "1r1w").strip().lower()
        if not name or depth < 1 or width < 1 or ports not in PORTS:
            sys.exit(f"{path}: bad row {r} (need name, depth >= 1, width >= 1, ports 1r1w|1rw)")
        if name in seen:
            sys.exit(f"{path}: duplicate name {name}")
        seen.add(name)
        out.append({"name": name, "depth": depth, "width": width, "ports": ports})
    if not out:
        sys.exit(f"{path}: no arrays")
    return out


def total_leak(vals):
    if vals is None or vals.get("leak") is None:
        return None
    return vals["leak"] + (vals.get("leak_gate") or 0.0)


def leak_parts(vals, costed, cell, done, job):
    """-> (leakage mW, cells' share mW or None, how the share was found)."""
    tot = total_leak(vals)
    if tot is None:
        return None, None, "none"
    # CACTI's breakdown lines are uninitialized without power gating: not used
    other = done.get(job[:2] + ("lstp" if cell == "hp" else "hp",))
    if other and other[0] is not None and (other[1], other[2]) == costed \
            and total_leak(other[0]) is not None:
        lstp_tot = tot if cell == "lstp" else total_leak(other[0])
        hp_tot = tot if cell == "hp" else total_leak(other[0])
        return tot, (max(0.0, hp_tot - lstp_tot) if cell == "hp" else 0.0), "hp-lstp"
    return tot, None, "none"


def rnd(v, f=1.0):
    return "" if v is None else round(v * f, 6)


def main():
    ap = argparse.ArgumentParser(description="CACTI 22 nm costing of the SRAM arrays, "
                                             "scaled to 7 nm (see the header)")
    ap.add_argument("arrays", nargs="?",
                    default=os.path.join(PROJ_DIR, "eval", "cacti", "arrays.csv"))
    ap.add_argument("--vdd-hp", type=float, required=True,
                    help="CACTI 22 nm vdd[0] (itrs-hp) from technology.cc")
    ap.add_argument("--vdd-lstp", type=float, required=True,
                    help="CACTI 22 nm vdd[1] (itrs-lstp) from technology.cc")
    ap.add_argument("--cells", nargs="+", default=["hp", "lstp"], choices=["hp", "lstp"])
    ap.add_argument("--ports", default="both", choices=["both", "csv"])
    ap.add_argument("--periph", default="hp", choices=["hp", "lstp"])
    ap.add_argument("--temp-k", type=int, default=300)
    ap.add_argument("--out", default=os.path.join(PROJ_DIR, "eval", "cacti"))
    ap.add_argument("--dst-vdd", type=float, default=0.7)
    ap.add_argument("--clk-ns", type=float, default=0.9)
    ap.add_argument("--calib-fo4-ps", type=float, default=None)
    ap.add_argument("--pack-bits", type=int, default=8)
    ap.add_argument("--pack-mode", default="byte", choices=["byte", "exact"])
    ap.add_argument("--max-pad", type=int, default=6)
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--timeout", type=float, default=900.0)
    ap.add_argument("--cacti", default=os.environ.get("CACTI_BIN"))
    ap.add_argument("--template")
    ap.add_argument("--rerun", action="store_true")
    a = ap.parse_args()
    if not a.cacti or not os.path.isfile(os.path.expanduser(a.cacti)):
        sys.exit("CACTI binary not found: set CACTI_BIN or pass --cacti")
    a.cacti = os.path.abspath(os.path.expanduser(a.cacti))
    if not (300 <= a.temp_k <= 400 and a.temp_k % 10 == 0):
        sys.exit("--temp-k must be 300..400 in steps of 10 (CACTI's range)")
    cacti_dir = os.path.dirname(a.cacti)
    tpl_path = os.path.abspath(a.template or os.path.join(cacti_dir, "cache.cfg"))
    if not os.path.isfile(tpl_path):
        sys.exit(f"no template {tpl_path} (--template)")
    template = open(tpl_path).read().splitlines()
    out_dir = os.path.abspath(a.out)          # CACTI runs in its own directory
    raw_dir = os.path.join(out_dir, "raw")
    os.makedirs(raw_dir, exist_ok=True)

    # ---- source supplies: the flags, cross-checked against CACTI's source ----
    src_vdd = {"hp": a.vdd_hp, "lstp": a.vdd_lstp}
    tech, tech_path = tech_vdd_22(cacti_dir)
    if tech:
        for i, cell in ((0, "hp"), (1, "lstp")):
            if i in tech and abs(tech[i] - src_vdd[cell]) > 1e-6:
                print(f"WARNING: --vdd-{cell} {src_vdd[cell]} V, but {tech_path} has "
                      f"{tech[i]} V for device type {i}", file=sys.stderr)
            elif i in tech:
                print(f"--vdd-{cell} {src_vdd[cell]} V matches {tech_path}")
    else:
        print(f"note: no 22 nm supplies found next to {a.cacti}: --vdd-hp / --vdd-lstp "
              f"unchecked", file=sys.stderr)
    for line in template_vdd_overrides(template):
        print(f"note: template line '{line}' is set to \"default\" (CACTI's own vdd[])",
              file=sys.stderr)
    for cell, v in list(src_vdd.items()) + [("hp", a.dst_vdd), ("lstp", a.dst_vdd)]:
        lo, hi = vdd_range(cell, SRC_NODE)
        if not lo <= v <= hi:
            print(f"WARNING: {cell} {v} V outside Stillmaker's simulated {lo}-{hi} V",
                  file=sys.stderr)

    # ---- Stillmaker factors, 20 nm MG (= CACTI 22 nm) -> 7 nm at dst-vdd ----
    f_hp = factors(((a.periph, SRC_NODE), src_vdd[a.periph]), ((a.periph, DST_NODE), a.dst_vdd),
                   a.calib_fo4_ps)
    kd, ke, ka, kl_hp = f_hp["delay"], f_hp["energy"], f_hp["area"], f_hp["leakage"]
    kl = {c: factors(((c, SRC_NODE), src_vdd[c]), ((c, DST_NODE), a.dst_vdd))["leakage"]
          for c in a.cells}
    kd_cell = {c: factors(((c, SRC_NODE), src_vdd[c]), ((c, DST_NODE), a.dst_vdd))["delay"]
               for c in a.cells}
    print(f"factors 22->7 nm ({a.periph} periphery, {a.temp_k} K): delay x{kd:.4f} "
          + "(" + ", ".join(f"bitline[{c}] x{kd_cell[c]:.4f}" for c in a.cells) + ")  "
          + f"energy x{ke:.4f}  area x{ka:.4f}  leak periphery x{kl_hp:.4f}  "
          + "  ".join(f"leak cells[{c}] x{kl[c]:.4f}" for c in a.cells))
    if a.calib_fo4_ps:
        print(f"  7 nm FO4 calibrated: {a.calib_fo4_ps} ps (Table 5: "
              f"{f_hp['fo4_dst_table5_ps']:.3f} ps)")

    # ---- arrays -> distinct CACTI jobs ----
    if not os.path.isfile(a.arrays):
        sys.exit(f"no {a.arrays}: run sram_arrays.py first")
    arrays = read_arrays(a.arrays)
    plan = []
    for r in arrays:
        starts = geometry(r["depth"], r["width"], a.pack_bits, a.pack_mode)
        for ports in (PORTS if a.ports == "both" else (r["ports"],)):
            for cell in a.cells:
                plan.append((r, ports, cell, (starts, ports, cell)))
    jobs = sorted({p[3] for p in plan})
    runner = Runner(a, template, raw_dir)
    t0 = time.time()
    print(f"{len(arrays)} arrays, {len(plan)} costings, {len(jobs)} distinct CACTI ladders, "
          f"{a.jobs} in parallel")
    with ThreadPoolExecutor(max_workers=max(1, a.jobs)) as ex:
        done = dict(zip(jobs, ex.map(runner.ladder, jobs)))
    print(f"CACTI: {runner.ran} runs, {runner.reused} reused, {time.time() - t0:.0f} s")

    # ---- results ----
    results, flagged = [], []
    for r, ports, cell, job in plan:
        vals, d, b, k, step, path = done[job]
        res = {"name": r["name"], "depth": r["depth"], "width": r["width"],
               "bits": r["depth"] * r["width"], "ports": ports, "cell": cell, "pack": k or "",
               "raw": os.path.relpath(path, out_dir) if path else ""}
        if vals is None:
            res["status"] = "fail"
            results.append(res)
            flagged.append(f"{r['name']} [{ports} {cell}]: no organization up to "
                           f"{job[0][-1][1] << a.max_pad}x{pow2(job[0][-1][2])}B (see {path}); "
                           f"cost as flops")
            continue
        missing = [x for x in NEEDED if vals.get(x) is None]
        bits_c = d * b * 8
        util = r["depth"] * r["width"] / bits_c
        t = vals.get("t_acc")
        tree = [vals.get(x) for x in ("t_htree_in", "t_htree_out")]
        htree = sum(p for p in tree if p is not None) if any(p is not None for p in tree) else None
        bitl = vals.get("t_bitline")
        leak, cells_lk, split = leak_parts(vals, (d, b), cell, done, job)
        if leak is None:
            leak7 = None
        elif cells_lk is not None:              # cells with their row, the rest with the periphery's
            leak7 = cells_lk * kl[cell] + (leak - cells_lk) * kl_hp
        else:
            leak7 = leak * kl_hp                # mostly periphery (see the header)
        area = vals.get("area_mm2")
        eff = vals.get("eff_pct")
        # bitline with the cell's row, the rest with the periphery's (opt);
        # the H-tree part also held unscaled (pes)
        t_bl, t_ht = (bitl or 0.0), (htree or 0.0)
        t_opt = (t - t_bl) * kd + t_bl * kd_cell[cell] if t is not None else None
        t_pes = (t - t_bl - t_ht) * kd + t_bl * kd_cell[cell] + t_ht if t is not None else None
        area7 = area * ka if area is not None else None
        area7_used = (area7 * (1 - eff / 100.0 * (1 - util))
                      if area7 is not None and eff is not None else None)
        leak7_used = (leak7 - cells_lk * kl[cell] * (1 - util)
                      if leak7 is not None and cells_lk is not None else None)
        res.update({
            "status": ("ok" if step == 0 else "byte-rows" if step < len(job[0]) else "padded")
            + ("-partial" if missing else ""),
            "depth_costed": d, "row_bytes_costed": b, "util": round(util, 4),
            "t_acc_22": rnd(t), "t_bitline_22": rnd(bitl), "t_htree_22": rnd(htree),
            "t_cyc_22": rnd(vals.get("t_cyc")),
            "e_rd_22": rnd(vals.get("e_rd"), 1e3), "e_wr_22": rnd(vals.get("e_wr"), 1e3),
            "leak_22": rnd(leak, 1e3), "leak_cells_22": rnd(cells_lk, 1e3), "leak_split": split,
            "area_22": rnd(area, 1e6), "eff_pct": rnd(eff),
            "cell_um2_22": rnd(area * eff / 100.0 / bits_c, 1e6)
            if area is not None and eff is not None else "",
            "t_opt_7": rnd(t_opt), "t_pes_7": rnd(t_pes),
            "cyc_opt": math.ceil(t_opt / a.clk_ns - 1e-9) if t_opt else "",
            "cyc_pes": math.ceil(t_pes / a.clk_ns - 1e-9) if t_pes else "",
            "e_rd_7": rnd(vals.get("e_rd"), ke * 1e3), "e_wr_7": rnd(vals.get("e_wr"), ke * 1e3),
            "leak_7": rnd(leak7, 1e3), "area_7": rnd(area7, 1e6),
            "leak_7_used": rnd(leak7_used, 1e3), "area_7_used": rnd(area7_used, 1e6),
        })
        results.append(res)
        if missing:
            flagged.append(f"{r['name']} [{ports} {cell}]: {', '.join(missing)} not parsed "
                           f"from {path}")
        elif util < 0.5:
            flagged.append(f"{r['name']} [{ports} {cell}]: util {util:.2f} "
                           f"({r['depth']}x{r['width']} costed as {d}x{b}B)")

    cols = ["name", "depth", "width", "bits", "ports", "cell", "pack", "status",
            "depth_costed", "row_bytes_costed", "util", "t_acc_22", "t_bitline_22", "t_htree_22",
            "t_cyc_22",
            "e_rd_22", "e_wr_22", "leak_22", "leak_cells_22", "leak_split", "area_22", "eff_pct",
            "cell_um2_22", "t_opt_7", "t_pes_7", "cyc_opt", "cyc_pes", "e_rd_7", "e_wr_7",
            "leak_7", "area_7", "leak_7_used", "area_7_used", "raw"]
    out_csv = os.path.join(out_dir, "cacti_arrays.csv")
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, restval="", lineterminator="\n")
        w.writeheader()
        w.writerows(results)
    meta = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "cacti": a.cacti, "template": tpl_path,
            "template_sha1": hashlib.sha1("\n".join(template).encode()).hexdigest(),
            "tech_vdd_22": {str(k): v for k, v in tech.items()},
            "src": {"node_nm": SRC_NODE, "vdd_hp": a.vdd_hp, "vdd_lstp": a.vdd_lstp},
            "dst": {"node_nm": DST_NODE, "vdd": a.dst_vdd, "calib_fo4_ps": a.calib_fo4_ps},
            "factors": {"delay": kd, "energy": ke, "area": ka, "leak_periphery": kl_hp,
                        **{f"leak_cells_{c}": kl[c] for c in a.cells}},
            "periph": a.periph, "temp_k": a.temp_k,
            "clk_ns": a.clk_ns, "pack_bits": a.pack_bits, "pack_mode": a.pack_mode,
            "max_pad": a.max_pad,
            "ports": a.ports, "cells": a.cells}
    with open(os.path.join(out_dir, "cacti_scaling.json"), "w") as f:
        json.dump(meta, f, indent=1)

    # ---- summary ----
    head = ("name", "ports", "cell", "status", "costed", "util", "t7 ns", "cyc", "rd7 pJ",
            "wr7 pJ", "leak7 uW", "area7 um2")
    lines = [(x["name"], x["ports"], x["cell"], x["status"],
              f"{x['depth_costed']}x{x['row_bytes_costed']}B" if "depth_costed" in x else "-",
              x.get("util", ""), f"{x.get('t_opt_7', '')}-{x.get('t_pes_7', '')}",
              f"{x.get('cyc_opt', '')}-{x.get('cyc_pes', '')}", x.get("e_rd_7", ""),
              x.get("e_wr_7", ""), x.get("leak_7", ""), x.get("area_7", ""))
             for x in results]
    wid = [max(len(str(l[i])) for l in [head] + lines) for i in range(len(head))]
    print("  " + "  ".join(str(h).ljust(wid[i]) for i, h in enumerate(head)))
    for l in lines:
        print("  " + "  ".join(str(c).ljust(wid[i]) for i, c in enumerate(l)))
    n_fail = sum(1 for x in results if x["status"] == "fail")
    n_pad = sum(1 for x in results if not x["status"].startswith(("ok", "fail")))
    slow = sorted({x["name"] for x in results if isinstance(x.get("cyc_pes"), int)
                   and x["cyc_pes"] > 1})
    print(f"\n{len(results)} rows: {n_pad} padded or on byte rows, {n_fail} failed -> {out_csv}")
    if slow:
        print(f"{len(slow)} array(s) need more than one {a.clk_ns} ns cycle in the pessimistic "
              f"bound: {', '.join(slow[:6])}{' ...' if len(slow) > 6 else ''}")
    ratios = sorted(x["leak_22"] / y["leak_22"] for x in results for y in results
                    if x["cell"] == "hp" and y["cell"] == "lstp" and x["name"] == y["name"]
                    and x["ports"] == y["ports"] and x.get("leak_22") and y.get("leak_22"))
    if ratios:
        print(f"leakage hp cells / lstp cells, same array: min x{ratios[0]:.1f}, "
              f"median x{ratios[len(ratios) // 2]:.1f} (the hp-lstp split assumes LSTP cells leak "
              f"far less than HP cells: with {a.periph} periphery this ratio "
              + ("shows it directly)" if a.periph == "lstp" else
                 "includes the periphery; check it in a --periph lstp run)"))
    for msg in flagged:
        print("  FLAG " + msg)
    sys.exit(0)


if __name__ == "__main__":
    main()