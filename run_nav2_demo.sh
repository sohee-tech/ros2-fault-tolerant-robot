#!/bin/bash
# Nav2 warehouse 자동 데모: Nav2 waypoint mission + Fault 1/2/3 -> results/nav2_summary.{json,csv}
# 인자 예: gui:=false dashboard:=false
SIM_SCRIPT=~/capstone_fault_ws/run_nav2.sh exec ~/capstone_fault_ws/run_test.sh nav2_demo "$@"
