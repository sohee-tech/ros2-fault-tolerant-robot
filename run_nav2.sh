#!/bin/bash
# Warehouse + fault system + Nav2 (+ dashboard, mission runner). 인자 예: gui:=false dashboard:=false autostart_mission:=true
# 자신의 process group을 새로 만들고 PGID를 기록 -> cleanup.sh가 이 그룹만 종료.
if [ "$(ps -o pgid= $$ | tr -d ' ')" != "$$" ]; then
  exec setsid "$0" "$@"
fi
source ~/capstone_fault_ws/env.sh
~/capstone_fault_ws/cleanup.sh               # 이전 실행에서 기록된 그룹만 정리
if pgrep -x gzserver >/dev/null; then
  echo "[run_nav2] 다른 gzserver가 실행 중입니다 (이 스크립트가 만든 것이 아님). 종료 후 다시 실행하세요." >&2
  exit 1
fi
mkdir -p ~/capstone_fault_ws/.run
echo $$ > ~/capstone_fault_ws/.run/sim_$$.pgid
exec ros2 launch fault_bringup nav2_warehouse.launch.py "$@"
