#!/bin/bash
# 남은 시뮬레이션 노드 개수 확인
for p in gzserver gzclient twist_mux robot_state_publisher; do echo "$p: $(pgrep -c -x ${p:0:15})"; done
echo "python nodes: $(pgrep -fc "lib/fault_")"
