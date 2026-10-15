# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   SRAM-port constraints and extra timing reports for evaluate.py. The
#   flow's post-syn-sta script sources this file through its SDC hook
#     make post-syn-sta ... SDC=projects/branch-prediction/scripts/sta_reports.tcl
#   after the generated constraints (clk_i, virtual clock vclk, I/O delays
#   from IO_DELAY_PCT) and before its own reports, in the same Tcl scope
#   ($REPORT_DIR is the run's report directory).
#
#   SRAM ports. Every table is an SRAM macro outside the core, clocked by
#   the same clock. Its read data reaches the core SRAM_CLK2Q_PS after the
#   edge (input delay on *_rdata_i*), and its enables, addresses and write
#   data must arrive SRAM_SETUP_PS before the edge (output delay on
#   *_re_o*, *_raddr_o*, *_we_o*, *_waddr_o*, *_wdata_o*). These replace the
#   generated IO_DELAY_PCT delays on those ports only; the front-end ports
#   (predict / update channels) keep IO_DELAY_PCT. Every core names its
#   SRAM ports this way (pht_, wt_, bias_, tb_, bp_, bh_ prefixes). The two
#   values are PLACEHOLDERS until the ASAP7 SRAM macro timing is in.
#
#   Reports:
#     worst_slack.rpt      worst setup slack, NOT clamped at 0 (report_wns
#                          reports 0 whenever timing is met)
#     worst_path.rpt       the single worst path over all path groups, with
#                          input pins and net names
#     pred_path.rpt        the worst path ending at pred_resp_taken_o (the
#                          prediction path), if the design has that port
#     worst_endpoints.rpt  the 20 worst endpoints, one line each
#   evaluate.py parses "Startpoint:" / "Endpoint:" in worst_path.rpt and
#   "data arrival time" in pred_path.rpt. Options an older OpenSTA might
#   lack sit inside catch: a missing option drops back to the plain report
#   (or skips the extra one) instead of failing the STA run.
#   Critical delay (evaluate.py) = period - worst slack: the shortest period
#   that would meet timing with these I/O budgets. The prediction-path
#   arrival includes the SRAM clock-to-Q when the path starts at read data.
#   Times are in the library unit (ps for ASAP7).
# -----------------------------------------------------------------------------

# --- SRAM macro timing (placeholders) ----------------------------------------
set SRAM_CLK2Q_PS 200.0
set SRAM_SETUP_PS  50.0

set io_clk [get_clocks -quiet vclk]
if {[llength $io_clk] == 0} {
    set io_clk [get_clocks clk_i]
}
set sram_in  [get_ports -quiet {*_rdata_i*}]
set sram_out [get_ports -quiet {*_re_o* *_raddr_o* *_we_o* *_waddr_o* *_wdata_o*}]
if {[llength $sram_in] > 0} {
    set_input_delay $SRAM_CLK2Q_PS -clock $io_clk $sram_in
}
if {[llength $sram_out] > 0} {
    set_output_delay $SRAM_SETUP_PS -clock $io_clk $sram_out
}
puts "sta_reports.tcl: SRAM clock-to-Q $SRAM_CLK2Q_PS ps on [llength $sram_in] input ports,\
 setup $SRAM_SETUP_PS ps on [llength $sram_out] output ports (placeholders)"

# --- reports -----------------------------------------------------------------
report_worst_slack -max -digits 4 > $REPORT_DIR/worst_slack.rpt

if {[catch {report_checks -path_delay max -sort_by_slack -group_path_count 1 -digits 4 \
                -fields {input_pin net} > $REPORT_DIR/worst_path.rpt} err]} {
    puts "sta_reports.tcl: worst path without net names ($err)"
    report_checks -path_delay max -sort_by_slack -group_path_count 1 -digits 4 \
        > $REPORT_DIR/worst_path.rpt
}

if {[llength [get_ports -quiet pred_resp_taken_o]] > 0} {
    if {[catch {report_checks -path_delay max -to [get_ports pred_resp_taken_o] -digits 4 \
                    -fields {input_pin net} > $REPORT_DIR/pred_path.rpt} err]} {
        puts "sta_reports.tcl: prediction path without net names ($err)"
        report_checks -path_delay max -to [get_ports pred_resp_taken_o] -digits 4 \
            > $REPORT_DIR/pred_path.rpt
    }
}

if {[catch {report_checks -path_delay max -group_path_count 20 -format end -digits 4 \
                > $REPORT_DIR/worst_endpoints.rpt} err]} {
    puts "sta_reports.tcl: worst_endpoints.rpt skipped ($err)"
}