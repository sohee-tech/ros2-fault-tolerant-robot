#!/bin/bash
# 자동 테스트: 시뮬레이션 실행 -> <test> 실행 -> 이번에 띄운 process group만 종료.
# 사용: run_test.sh fault1_test|fault2_test|fault3_test|fault4_test [gui:=false]
TEST=${1:?usage: run_test.sh fault1_test|fault2_test|fault3_test|fault4_test [launch args]}; shift
source ~/capstone_fault_ws/env.sh
LOG=~/capstone_fault_ws/logs; mkdir -p $LOG
STAMP=$(date +%Y%m%d_%H%M%S)
setsid ~/capstone_fault_ws/run_sim.sh "$@" > $LOG/sim_$STAMP.log 2>&1 &
SIM_PGID=$!
export SIM_PGID   # lets a test signal processes of THIS simulation only
cleanup() {
  kill -INT -- -$SIM_PGID 2>/dev/null
  for _ in $(seq 20); do pgrep -g $SIM_PGID >/dev/null || break; sleep 0.25; done
  pgrep -g $SIM_PGID >/dev/null && kill -KILL -- -$SIM_PGID 2>/dev/null
  for _ in $(seq 20); do pgrep -g $SIM_PGID >/dev/null || break; sleep 0.1; done
  rm -f ~/capstone_fault_ws/.run/sim_$SIM_PGID.pgid
}
trap cleanup EXIT
sleep 12
if ! pgrep -g $SIM_PGID >/dev/null; then echo "simulation failed to start, see $LOG/sim_$STAMP.log"; exit 1; fi
ros2 run fault_bringup $TEST 2>&1 | tee $LOG/${TEST}_$STAMP.log
RC=${PIPESTATUS[0]}
echo "--- node log ---"
grep -E 'health_monitor|fault_injector|speed_limiter|control_delay|nav_command_source|nav_fault' $LOG/sim_$STAMP.log | grep -E 'NORMAL|WARNING|DEGRADED|CRITICAL|ODOM|CONTROL|delay|fault|spike|limit|CSV|system|died|NAV|pid|crash|respawn'
exit $RC
