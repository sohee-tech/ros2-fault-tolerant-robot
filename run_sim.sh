#!/bin/bash
# 전체 fault 시뮬레이션 (Gazebo + LiDAR/odom injector + health monitor + speed_limiter + twist_mux + idle_stop).
# 인자: gui:=false 가능
# 자신의 process group을 새로 만들고 PGID를 기록 -> cleanup.sh가 이 그룹만 종료.
if [ "$(ps -o pgid= $$ | tr -d ' ')" != "$$" ]; then
  exec setsid "$0" "$@"
fi
source ~/capstone_fault_ws/env.sh
~/capstone_fault_ws/cleanup.sh               # 이전 실행에서 기록된 그룹만 정리
if pgrep -x gzserver >/dev/null; then
  echo "[run_sim] 다른 gzserver가 실행 중입니다 (이 스크립트가 만든 것이 아님). 종료 후 다시 실행하세요." >&2
  exit 1
fi
mkdir -p ~/capstone_fault_ws/.run
echo $$ > ~/capstone_fault_ws/.run/sim_$$.pgid
exec ros2 launch fault_bringup fault_sim.launch.py "$@"
