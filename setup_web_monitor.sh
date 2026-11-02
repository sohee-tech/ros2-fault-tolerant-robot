#!/bin/bash
# 웹 모니터 전용 Python 환경(.venv_web) 생성. 시스템 Python과 기존 ROS2 환경은 변경하지 않는다.
set -e
cd ~/capstone_fault_ws
[ -x .venv_web/bin/python ] || python3 -m venv --system-site-packages .venv_web
.venv_web/bin/python -m pip install -q --disable-pip-version-check -r src/fault_web/requirements.txt
[ "$1" = "--test" ] && .venv_web/bin/python -m pip install -q --disable-pip-version-check -r src/fault_web/requirements-test.txt
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fault_web
echo "완료: ./run_web_monitor.sh"
