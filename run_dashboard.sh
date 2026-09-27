#!/bin/bash
# 대시보드만 실행 (이미 실행 중인 시뮬레이션에 연결). 시뮬레이션과 함께 띄우려면:
#   ./run_sim.sh demo:=true dashboard:=true
source ~/capstone_fault_ws/env.sh
exec ros2 run fault_bringup fault_dashboard
