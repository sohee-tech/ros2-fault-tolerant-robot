#!/bin/bash
# 읽기 전용 웹 모니터 (로봇 시스템과 별도 프로세스). 종료해도 로봇 제어에 영향이 없다.
# 인자 예: --port 8080 --public-host 192.168.0.10   /   NO_BROWSER=1 이면 QR 페이지를 자동으로 열지 않음
source ~/capstone_fault_ws/env.sh
PY=~/capstone_fault_ws/.venv_web/bin/python
[ -x "$PY" ] || { echo "먼저 ./setup_web_monitor.sh 를 실행하세요."; exit 1; }
PORT=8080; for ((i=1;i<$#;i++)); do [ "${!i}" = "--port" ] && { j=$((i+1)); PORT=${!j}; }; done
# 이전 실행이 남아 있으면 옛 코드가 계속 응답하므로 교체한다. 다른 프로그램이 쓰는 포트는 건드리지 않는다.
OLD=$(ss -ltnp 2>/dev/null | grep ":$PORT " | sed -nE 's/.*pid=([0-9]+).*/\1/p' | head -1)
if [ -n "$OLD" ]; then
  if tr '\0' ' ' < /proc/$OLD/cmdline 2>/dev/null | grep -q 'fault_web.server'; then
    echo "[web_monitor] 이전 웹 모니터(PID $OLD)를 종료하고 다시 시작합니다."
    kill "$OLD"; for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 "$OLD" 2>/dev/null || break; sleep 0.3; done
  else
    echo "[web_monitor] 포트 $PORT 을 다른 프로그램(PID $OLD)이 사용 중입니다. --port 로 다른 포트를 지정하세요."; exit 1
  fi
elif ss -ltn 2>/dev/null | grep -q ":$PORT "; then
  echo "[web_monitor] 포트 $PORT 이 이미 사용 중입니다. --port 로 다른 포트를 지정하세요."; exit 1
fi
[ -n "$NO_BROWSER" ] || (sleep 2; setsid -f xdg-open "http://127.0.0.1:$PORT/qr" >/dev/null 2>&1) &   # 브라우저는 서버 종료와 함께 닫히지 않음
exec "$PY" -m fault_web.server "$@"
