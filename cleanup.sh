#!/bin/bash
# 이번 프로젝트 실행이 만든 process group만 종료한다.
# run_fault1_sim.sh가 새 session/process group으로 실행하며 PGID를 .run/*.pgid에 기록한다.
# 이름 기반(pkill -f) 종료는 사용하지 않는다.
RUN_DIR=~/capstone_fault_ws/.run

group_alive() { pgrep -g "$1" >/dev/null; }

stop_group() {
  local pgid=$1
  # PID 재사용 방지: 그룹 멤버가 이 프로젝트/Gazebo 프로세스인지 확인
  if ! ps -o args= -p "$(pgrep -d, -g "$pgid")" 2>/dev/null | grep -qE 'capstone_fault_ws|fault_bringup|gzserver|gzclient'; then
    return 0
  fi
  echo "[cleanup] stopping process group $pgid"
  kill -INT -- -"$pgid" 2>/dev/null
  for _ in $(seq 20); do group_alive "$pgid" || return 0; sleep 0.25; done
  kill -TERM -- -"$pgid" 2>/dev/null; sleep 1
  group_alive "$pgid" && kill -KILL -- -"$pgid" 2>/dev/null
  return 0
}

shopt -s nullglob
for f in "$RUN_DIR"/*.pgid; do
  pgid=$(cat "$f")
  [[ "$pgid" =~ ^[0-9]+$ ]] || { rm -f "$f"; continue; }
  group_alive "$pgid" && stop_group "$pgid"
  if group_alive "$pgid"; then
    echo "[cleanup] WARNING: process group $pgid still alive, keeping $f" >&2
  else
    rm -f "$f"
  fi
done
exit 0
