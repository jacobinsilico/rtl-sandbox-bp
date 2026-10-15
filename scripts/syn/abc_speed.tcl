# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Delay-oriented ABC script, selected with SYN_ABC=speed (see run.tcl).
# Mapping as in Yosys's own liberty recipe, then the sizing tail of abc.tcl:
#   &fraig -x, dc2   AIG cleanup and light rewriting
#   &syn2            delay-aware AIG resynthesis (shortens logic depth)
#   &dch -f          structural choices: the mapper can pick among several
#                    decompositions of the same logic, not only cells for
#                    one fixed structure
#   &nf {D}          delay-oriented standard-cell mapper, the clock period as
#                    target ({D} becomes -D <period in ps>)
#   buffer / upsize / dnsize   as in abc.tcl
# -----------------------------------------------------------------------------

strash
&get -n
&fraig -x
&put
dc2
strash
&get -n
&syn2
&dch -f
&nf {D}
&put
topo
stime -c
buffer -c
upsize -c
dnsize -c