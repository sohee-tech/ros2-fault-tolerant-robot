#!/bin/bash
# 통합 데모: Gazebo(GUI) + 대시보드 + Fault 1~4 순차 실행 -> results/final_summary.{json,csv}
# 인자 예: gui:=false (Gazebo 창 없이)
exec ~/capstone_fault_ws/run_test.sh integrated_demo demo:=true dashboard:=true "$@"
