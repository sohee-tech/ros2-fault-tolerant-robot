#!/bin/bash
# Safe Resume Gate 자동 테스트 (warehouse, TEST 0~8) -> results/safe_resume_summary.{json,csv}
# 게이트는 테스트가 직접 늦게 시작한다 (노드 시작 순서 검증). 인자 예: gui:=true
SIM_SCRIPT=~/capstone_fault_ws/run_nav2.sh exec ~/capstone_fault_ws/run_test.sh safe_resume_test mission_config:=nav2_mission_recovery.yaml safe_resume_gate:=false gui:=false dashboard:=true "$@"
