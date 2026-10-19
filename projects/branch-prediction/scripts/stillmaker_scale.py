#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
# -----------------------------------------------------------------------------
"""
stillmaker_scale.py - scale delay / dynamic energy / leakage / area between CMOS
nodes and supply voltages with Stillmaker & Baas, Integration VLSI J. 58 (2017)
and its 2019 corrigendum: Table 5 polynomials (Eqs. 5-10) and Table 3
geometry (area). cacti_cost.py imports factors() from here.

Notes
  * Factors come from Table 5 only. Table 2's 16/14/10/7 nm rows do not match
    their listed VDD (Table 5 reproduces them at ~0.95 V / 1.0 V instead);
    the 20 nm row does.
  * Area is the geometric mean of the Table 3 ratios, i.e. how Table 4 was
    built; Table 4's row/column labels are swapped as printed (20 -> 7 nm
    reads 3.6 there; the factor is 0.28).
  * FO4 chains carry no wire load. Pass the wire-dominated part of a CACTI
    access time (H-tree, bitline) as --wire-delay to get a pessimistic bound
    in which that part stays unscaled.
  * Eq. 10 (PowerFactor) is mostly standby power: use it for leakage only.
    Eq. 11 (power with frequency) is not used.

Usage
  /usr/bin/python3 stillmaker_scale.py --selftest
  /usr/bin/python3 stillmaker_scale.py --src 20:hp:0.8 --dst 7:hp:0.7
  /usr/bin/python3 stillmaker_scale.py --src 20:hp:0.8 --dst 7:hp:0.7 \\
      --delay 0.21 --wire-delay 0.07 --energy 1.35 --leakage 42 --area 1200
  /usr/bin/python3 stillmaker_scale.py --src 20:lstp:0.8 --dst 7:lstp:0.7 --leakage 42
  /usr/bin/python3 stillmaker_scale.py --src 20:hp:0.8 --dst 7:hp:0.7 --calib-fo4-ps 4.1

Spec format NODE:FLAVOR:VDD, FLAVOR in {bulk (180-65 nm), hp, lp (45/32 nm only),
lstp (20-7 nm only)}; hp is high-k at 45/32 nm and multi-gate at 20-7 nm.
Values keep their units (ps, pJ, uW, um^2, ...); only the ratio is applied.
"""
import argparse
import hashlib
import sys

# Table 5: (ad3, ad2, ad1, ad0, ae2, ae1, ae0, ap2, ap1, ap0); '-' entries -> 0.
# Delay in ps, energy in fJ per transition, power in nW (FO4 inverter, no wire).
T5 = {
    ("bulk", 180): (0, 97.09, -356.7, 406.5, 0, 24.64, -17.98, 0, 101000, -79720),
    ("bulk", 130): (-76.65, 334.9, -493.4, 275.8, 7.171, -6.709, 2.904, 27020, -15450, 5630),
    ("bulk", 90): (-60.34, 262.5, -384.2, 210.9, 4.762, -4.781, 2.092, 17320, -11230, 4328),
    ("bulk", 65): (-53.3, 230.4, -333.9, 178.6, 3.755, -4.398, 1.975, 12890, -10510, 4362),
    ("hp", 45): (-501.6, 1567, -1619, 566.1, 1.018, -0.3107, 0.1539, 5462, -1760, 522.4),
    ("hp", 32): (-1047, 2982, -2797, 873.5, 0.8367, -0.4341, 0.1701, 4001, -1733, 533.6),
    ("lp", 45): (-285.7, 1239, -1795, 898.8, 1.103, -0.362, 0.2767, 6297, -3009, 1124),
    ("lp", 32): (-325.9, 1374, -1922, 913.2, 0.9559, -0.7823, 0.471, 4557, -3037, 1323),
    ("hp", 20): (0, 34.63, -66.37, 41.15, 0.373, -0.1582, 0.04104, 2922, -1286, 299.9),
    ("hp", 16): (0, 24.8, -47.52, 28.87, 0.2958, -0.1241, 0.03024, 2133, -882.6, 197.7),
    ("hp", 14): (-40.66, 109.2, -100.6, 35.92, 0.2363, -0.09675, 0.02239, 1675, -711, 159),
    ("hp", 10): (-34.95, 93.65, -85.99, 30.4, 0.2068, -0.09311, 0.02375, 1456, -621.6, 143.8),
    ("hp", 7): (-28.58, 76.6, -70.26, 24.69, 0.1776, -0.09097, 0.02447, 1179, -515.7, 123.4),
    ("lstp", 20): (-160.5, 514.1, -558.6, 217.5, 0.2632, -0.14, 0.06841, 2096, -962.4, 287.1),
    ("lstp", 16): (-114.6, 366.7, -397.4, 153.6, 0.2139, -0.1187, 0.05639, 1609, -715.5, 205.7),
    ("lstp", 14): (-85.37, 271.6, -292.2, 111.4, 0.1556, -0.06472, 0.03066, 1259, -554.1, 152.3),
    ("lstp", 10): (-71.76, 228.6, -246.3, 93.91, 0.1261, -0.0518, 0.02769, 1046, -422.7, 118.9),
    ("lstp", 7): (-61.79, 196.1, -210.3, 79.55, 0.09365, -0.03409, 0.02043, 815.2, -307.3, 87.54),
}
# sha1 of repr(sorted T5), taken after checking every coefficient against the
# PDF's Table 5 text (2026-10-16): --selftest fails if the table is edited.
T5_SHA1 = "db9f65a50730c798919e7ac30b6795172b9cf0d4"

# Table 3: min feature size (nm), metal-1 half pitch (nm), 4T logic gate (um^2).
T3 = {180: (180, 230, 57), 130: (130, 150, 10.4), 90: (90, 90, 5.2), 65: (65, 68, 2.6),
      45: (45, 59, 2.1), 32: (32, 45, 0.71), 20: (20, 32, 0.35), 16: (16, 40, 0.248),
      14: (14, 40, 0.248), 10: (10, 31.8, 0.157), 7: (7, 25.3, 0.099)}


def vdd_range(flavor, node):
    """Simulated VDD range per curve, read off Fig. 3 (approximate). Outside
    it the paper advises against using the polynomials."""
    if flavor == "bulk":
        return (1.0, 1.8) if node == 180 else (0.65, 1.8)
    if node in (45, 32):
        return {("hp", 45): (0.55, 1.3), ("hp", 32): (0.5, 1.2),
                ("lp", 45): (0.8, 1.75), ("lp", 32): (0.75, 1.75)}[(flavor, node)]
    return (0.5, 1.0) if flavor == "hp" else (0.65, 1.2)


def poly(key, v):
    """(delay ps, energy fJ, power nW) of the FO4 inverter at VDD v (Eqs. 5-7)."""
    c = T5[key]
    d = c[0] * v**3 + c[1] * v**2 + c[2] * v + c[3]
    e = c[4] * v**2 + c[5] * v + c[6]
    p = c[7] * v**2 + c[8] * v + c[9]
    return d, e, p


def area_factor(n_from, n_to):
    """Geometric mean of the three Table 3 area ratios."""
    f, t = T3[n_from], T3[n_to]
    r = ((t[0] / f[0]) ** 2) * ((t[1] / f[1]) ** 2) * (t[2] / f[2])
    return r ** (1.0 / 3.0)


def factors(src, dst, calib_fo4_ps=None):
    """src, dst = ((flavor, node), vdd). -> dict of the delay, energy, leakage
    and area factors (Eqs. 8-10, Table 3) plus the FO4 values behind them.
    calib_fo4_ps replaces the Table 5 FO4 delay at dst (measured ASAP7)."""
    (ks, vs), (kd, vd) = src, dst
    ds, es, ps = poly(ks, vs)
    dd, ed, pd = poly(kd, vd)
    fo4_dst = calib_fo4_ps if calib_fo4_ps else dd
    return {"delay": fo4_dst / ds, "energy": ed / es, "leakage": pd / ps,
            "area": area_factor(ks[1], kd[1]), "fo4_src_ps": ds, "fo4_dst_ps": fo4_dst,
            "fo4_dst_table5_ps": dd}


def table_sha1():
    return hashlib.sha1(repr(sorted(T5.items())).encode()).hexdigest()


def parse_spec(s):
    node, flavor, vdd = s.split(":")
    key, v = (flavor.lower(), int(node)), float(vdd)
    if key not in T5:
        sys.exit(f"no Table 5 row for {s} (valid: {sorted(T5)})")
    lo, hi = vdd_range(*key)
    if not lo - 1e-9 <= v <= hi + 1e-9:
        print(f"WARNING: {s}: {v} V outside simulated range {lo}-{hi} V (Fig. 3)",
              file=sys.stderr)
    return key, v


def selftest():
    ok = True

    def check(name, got, want, tol):
        nonlocal ok
        good = abs(got / want - 1) <= tol
        ok &= good
        print(f"{'PASS' if good else 'FAIL'}  {name}: {got:.4g} (expect {want:.4g})")

    sha = table_sha1()
    good = sha == T5_SHA1
    ok &= good
    print(f"{'PASS' if good else 'FAIL'}  Table 5 unchanged since checked against the PDF "
          f"(sha1 {sha[:12]})")
    # Corrigendum example: 1 pJ at 65 nm bulk 1.3 V -> 0.1755 pJ at 32 nm HP 0.9 V.
    check("corrigendum 65nm@1.3V -> 32nm HP@0.9V, 1 pJ",
          poly(("hp", 32), 0.9)[1] / poly(("bulk", 65), 1.3)[1], 0.1755, 0.005)
    # Table 2's 20 nm row at its listed 0.9 V (the source row cacti_cost.py uses).
    d, e, p = poly(("hp", 20), 0.9)
    check("Table 2, 20nm HP@0.9V delay (ps)", d, 9.66, 0.03)
    check("Table 2, 20nm HP@0.9V energy (fJ)", e, 0.198, 0.03)
    check("Table 2, 20nm HP@0.9V power (nW)", p, 1510, 0.03)
    # Table 2's 7 nm row is reproduced at 1.0 V, not at its listed 0.8 V.
    d, e, p = poly(("hp", 7), 1.0)
    check("Table 2, 7nm HP row at 1.0 V: delay (ps)", d, 2.47, 0.05)
    check("Table 2, 7nm HP row at 1.0 V: energy (fJ)", e, 0.111, 0.05)
    check("Table 2, 7nm HP row at 1.0 V: power (nW)", p, 789, 0.05)
    check("Table 4, area 65nm -> 32nm", area_factor(65, 32), 0.31, 0.02)
    check("Table 4, area 20nm -> 7nm", area_factor(20, 7), 0.28, 0.02)
    # LSTP has no Table 2 row: plausibility only (slower, less leaky than HP).
    for node, v in ((20, 0.8), (7, 0.7)):
        dh, _, ph = poly(("hp", node), v)
        dl, _, pl = poly(("lstp", node), v)
        good = dl > dh and pl < ph
        ok &= good
        print(f"{'PASS' if good else 'FAIL'}  LSTP {node}nm@{v}V slower and less leaky than HP "
              f"({dl:.2f} vs {dh:.2f} ps, {pl:.0f} vs {ph:.0f} nW)")
    sys.exit(0 if ok else 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true",
                    help="check the transcription against the paper and exit")
    ap.add_argument("--src", help="NODE:FLAVOR:VDD of the input numbers, e.g. 20:hp:0.8")
    ap.add_argument("--dst", help="NODE:FLAVOR:VDD to scale to, e.g. 7:hp:0.7")
    ap.add_argument("--delay", type=float, help="access/cycle time to scale (Eq. 8)")
    ap.add_argument("--wire-delay", type=float, default=0.0,
                    help="part of --delay held unscaled in the pessimistic bound")
    ap.add_argument("--energy", type=float, help="dynamic energy per access (Eq. 9)")
    ap.add_argument("--leakage", type=float, help="leakage power (Eq. 10)")
    ap.add_argument("--area", type=float, help="area (Table 3 geometric mean)")
    ap.add_argument("--calib-fo4-ps", type=float,
                    help="measured FO4 delay at dst (e.g. ASAP7 in OpenSTA); "
                         "replaces the Table 5 dst delay")
    a = ap.parse_args()
    if a.selftest:
        selftest()
    if not (a.src and a.dst):
        ap.error("--src and --dst are required (or use --selftest)")

    ks, vs = parse_spec(a.src)
    kd, vd = parse_spec(a.dst)
    f = factors((ks, vs), (kd, vd), a.calib_fo4_ps)
    ds, es, ps = poly(ks, vs)
    dd, ed, pd = poly(kd, vd)
    print(f"src {ks[1]} nm {ks[0]} @ {vs} V: FO4 {ds:.3f} ps, {es:.4f} fJ, {ps:.1f} nW")
    print(f"dst {kd[1]} nm {kd[0]} @ {vd} V: FO4 {dd:.3f} ps, {ed:.4f} fJ, {pd:.1f} nW")
    if a.calib_fo4_ps:
        print(f"calibrated dst FO4: {a.calib_fo4_ps:.3f} ps "
              f"(Table 5 gives {dd:.3f} ps, ratio {a.calib_fo4_ps / dd:.2f})")
    print(f"factors: delay x{f['delay']:.4f}  energy x{f['energy']:.4f}  "
          f"leakage x{f['leakage']:.4f}  area x{f['area']:.4f}")

    if a.delay is not None:
        opt = a.delay * f["delay"]
        pes = (a.delay - a.wire_delay) * f["delay"] + a.wire_delay
        print(f"delay   {a.delay:g} -> {opt:.6g} (all scaled) .. {pes:.6g} (wire unscaled)")
    if a.energy is not None:
        print(f"energy  {a.energy:g} -> {a.energy * f['energy']:.6g}")
    if a.leakage is not None:
        print(f"leakage {a.leakage:g} -> {a.leakage * f['leakage']:.6g}")
    if a.area is not None:
        print(f"area    {a.area:g} -> {a.area * f['area']:.6g}")


if __name__ == "__main__":
    main()