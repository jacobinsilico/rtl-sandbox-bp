# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Delay-oriented ABC script, selected with SYN_ABC=dch (see run.tcl). The
# mapper of abc.tcl (map), but given structural choices: dch -f computes
# several decompositions of the same logic and map picks the fastest. Same
# sizing tail as abc.tcl. The fallback if SYN_ABC=speed (&nf) misbehaves.
# -----------------------------------------------------------------------------

strash
dc2
dch -f
map {D} -B 0.9
topo
stime -c
buffer -c
upsize -c
dnsize -c