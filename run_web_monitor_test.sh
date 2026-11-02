#!/bin/bash
# 웹 모니터 통합 테스트 (Gazebo + 실제 고장 주입 + 실제 브라우저) -> results/web_monitor_summary.json, results/web_monitor/*.png
source ~/capstone_fault_ws/env.sh
LOG=~/capstone_fault_ws/logs; STAMP=$(date +%Y%m%d_%H%M%S); mkdir -p "$LOG"
setsid ~/capstone_fault_ws/run_nav2.sh gui:=false dashboard:=false mission_config:=nav2_mission_recovery.yaml "$@" > "$LOG/sim_$STAMP.log" 2>&1 &
SIM=$!
cleanup() {
  kill -INT -- -$SIM 2>/dev/null
  for _ in $(seq 20); do pgrep -g $SIM >/dev/null || break; sleep 0.25; done
  pgrep -g $SIM >/dev/null && kill -KILL -- -$SIM 2>/dev/null
  rm -f ~/capstone_fault_ws/.run/sim_$SIM.pgid
}
trap cleanup EXIT
sleep 12
~/capstone_fault_ws/.venv_web/bin/python -m fault_web.web_monitor_test 2>&1 | tee "$LOG/web_monitor_test_$STAMP.log"
exit ${PIPESTATUS[0]}
