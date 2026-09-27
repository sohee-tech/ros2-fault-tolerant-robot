#!/bin/bash
# Fault 1 자동 테스트: 시뮬레이션 실행 -> fault1_test -> 이번에 띄운 process group만 종료.
source ~/capstone_fault_ws/env.sh
LOG=~/capstone_fault_ws/logs; mkdir -p $LOG
STAMP=$(date +%Y%m%d_%H%M%S)
setsid ~/capstone_fault_ws/run_fault1_sim.sh "$@" > $LOG/sim_$STAMP.log 2>&1 &
SIM_PGID=$!
cleanup() {
  kill -INT -- -$SIM_PGID 2>/dev/null
  for _ in $(seq 20); do pgrep -g $SIM_PGID >/dev/null || break; sleep 0.25; done
  pgrep -g $SIM_PGID >/dev/null && kill -KILL -- -$SIM_PGID 2>/dev/null
  rm -f ~/capstone_fault_ws/.run/sim_$SIM_PGID.pgid
}
trap cleanup EXIT
sleep 12
if ! pgrep -g $SIM_PGID >/dev/null; then echo "simulation failed to start, see $LOG/sim_$STAMP.log"; exit 1; fi
ros2 run fault_bringup fault1_test 2>&1 | tee $LOG/test_$STAMP.log
RC=${PIPESTATUS[0]}
echo "--- node log ---"
grep -E 'health_monitor|lidar_fault_injector|twist_mux|idle_stop' $LOG/sim_$STAMP.log | grep -E 'NORMAL|WARNING|DEGRADED|CRITICAL|fault|CSV|idle'
exit $RC
