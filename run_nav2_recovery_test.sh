#!/bin/bash
# Nav2 lifecycle fault-recovery 자동 테스트 (warehouse, 5-waypoint mission)
# -> results/nav2_recovery_summary.{json,csv}. 인자 예: gui:=false dashboard:=false
SIM_SCRIPT=~/capstone_fault_ws/run_nav2.sh exec ~/capstone_fault_ws/run_test.sh nav2_recovery_test mission_config:=nav2_mission_recovery.yaml "$@"
