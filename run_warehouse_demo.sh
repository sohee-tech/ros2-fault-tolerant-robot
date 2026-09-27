#!/bin/bash
# Warehouse AMR world에서 통합 데모 실행 (Gazebo GUI + 대시보드 + Fault 1~4)
exec ~/capstone_fault_ws/run_integrated_demo.sh world:=warehouse "$@"
