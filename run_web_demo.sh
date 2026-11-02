#!/bin/bash
# 시연: Gazebo + Nav2 + 고장 대시보드 + 임무 자동 시작, 웹 모니터 + QR 페이지. 고장 주입은 대시보드 버튼을 사용한다.
# 인자는 run_nav2.sh로 전달 (예: gui:=false).
#
# 종료: Ctrl+C 한 번이면 이 실행이 만든 프로세스 그룹만 정리한다 (다른 프로그램은 건드리지 않음).
#   SIGINT(launch가 노드를 차례로 종료) -> STOP_TIMEOUT초 후 SIGTERM -> 3초 후 SIGKILL.
#   정리 중 Ctrl+C를 한 번 더 누르면 바로 다음 단계로 넘어간다.
#
# 백그라운드 작업은 set -m으로 시작한다: 자기 프로세스 그룹을 갖고 SIGINT 무시 상태를 물려받지 않는다.
# 시작 후 끄는 이유: 켜 두면 정리 중의 sleep/pgrep이 터미널 전경 그룹이 되어 Ctrl+C를 대신 받고 스크립트가 끝나 버린다.
# 그래서 터미널의 Ctrl+C는 이 스크립트에만 오고, 스크립트가 각 그룹에 순서대로 전달한다.
WS=~/capstone_fault_ws
STOP_TIMEOUT=${STOP_TIMEOUT:-15}
SIM=; WEB=; STOPPING=0; SKIP=0

alive() { pgrep -g "$1" >/dev/null 2>&1; }

wait_group() {   # pgid, seconds: true when the group is gone. Another Ctrl+C ends the wait early.
  local end=$((SECONDS + $2))
  SKIP=0
  while alive "$1"; do
    [ "$SKIP" = 1 ] || [ "$SECONDS" -ge "$end" ] && return 1
    sleep 0.2
  done
}

stop_group() {   # pgid, first signal, seconds to wait after it
  [ -n "$1" ] && alive "$1" || return 0
  kill "-$2" -- "-$1" 2>/dev/null
  wait_group "$1" "$3" && return 0
  kill -TERM -- "-$1" 2>/dev/null
  wait_group "$1" 3 && return 0
  echo "[run_web_demo] 그룹 $1 강제 종료"
  kill -KILL -- "-$1" 2>/dev/null
  wait_group "$1" 2
}

shutdown() {
  [ "$STOPPING" = 1 ] && return
  STOPPING=1
  echo; echo "[run_web_demo] 종료 중… (바로 끝내려면 Ctrl+C 한 번 더)"
  stop_group "$WEB" TERM 3
  stop_group "$SIM" INT "$STOP_TIMEOUT"
  rm -f "$WS/.run/sim_$SIM.pgid" "$WS/.run/web_$WEB.pgid"
  echo "[run_web_demo] 종료 완료"
}

on_signal() {   # during the cleanup a further signal only skips the current wait; the cleanup always finishes
  [ "$STOPPING" = 1 ] && { SKIP=1; return; }
  shutdown
  exit "$1"
}
trap 'on_signal 130' INT
trap 'on_signal 143' TERM HUP
trap 'shutdown' EXIT

set -m
"$WS/run_nav2.sh" autostart_mission:=true mission_config:=nav2_mission_recovery.yaml "$@" &
SIM=$!
set +m
# run_nav2.sh의 cleanup.sh가 끝난 뒤(.pgid 기록 후) 웹 서버를 시작해야 웹 서버가 정리 대상이 되지 않는다.
until [ -e "$WS/.run/sim_$SIM.pgid" ]; do
  kill -0 "$SIM" 2>/dev/null || { echo "[run_web_demo] 시뮬레이션 시작 실패"; exit 1; }
  sleep 0.2
done
set -m
"$WS/run_web_monitor.sh" &
WEB=$!
set +m
echo "$WEB" > "$WS/.run/web_$WEB.pgid"   # 터미널이 강제로 닫혀도 다음 실행의 cleanup.sh가 정리한다

wait "$SIM"     # 웹 서버가 죽어도 로봇 시스템은 계속 실행한다
