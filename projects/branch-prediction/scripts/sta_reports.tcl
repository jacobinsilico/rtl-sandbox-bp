# -----------------------------------------------------------------------------
# Author: Jakub Dawid Szkudlarek
# SPDX-License-Identifier: Apache-2.0
#
# Description:
#   Extra timing reports for evaluate.py. The flow's post-syn-sta script
#   sources this file through its SDC hook
#     make post-syn-sta ... SDC=projects/branch-prediction/scripts/sta_reports.tcl
#   after the generated constraints and before its own reports, in the same
#   Tcl scope ($REPORT_DIR is the run's report directory). It adds no
#   constraint, only reports:
#     worst_slack.rpt  worst setup slack, NOT clamped at 0 (report_wns
#                      reports 0 whenever timing is met)
#     worst_path.rpt   the single worst path over all path groups
#     pred_path.rpt    the worst path ending at pred_resp_taken_o (the
#                      prediction path), if the design has that port
#   With IO_DELAY_PCT=0 every path class (in->reg, reg->out, in->out,
#   reg->reg) gets the whole period, so critical delay = period - worst slack.
#   Times are in the library unit (ps for ASAP7).
# -----------------------------------------------------------------------------

report_worst_slack -max -digits 4 > $REPORT_DIR/worst_slack.rpt
report_checks -path_delay max -sort_by_slack -group_path_count 1 -digits 4 \
    > $REPORT_DIR/worst_path.rpt
if {[llength [get_ports -quiet pred_resp_taken_o]] > 0} {
    report_checks -path_delay max -to [get_ports pred_resp_taken_o] -digits 4 \
        > $REPORT_DIR/pred_path.rpt
}