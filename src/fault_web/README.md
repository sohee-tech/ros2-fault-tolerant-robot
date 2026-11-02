# fault_web: 관리자 알림 및 QR 모바일 모니터 (읽기 전용)

로봇 시스템과 별도 프로세스로 실행된다. ROS 토픽을 구독만 하며 로봇에 어떤 명령도 보내지 않는다.

## 설치 (최초 1회)

    ./setup_web_monitor.sh          # .venv_web 생성, requirements.txt 설치. 시스템 Python은 변경하지 않음
    ./setup_web_monitor.sh --test   # 통합 테스트 도구(selenium, zxing-cpp)까지 설치

## 실행

    ./run_web_demo.sh               # Gazebo + Nav2 + tkinter 대시보드 + 임무 자동 시작 + 웹 모니터 + QR 페이지
    ./run_web_monitor.sh            # 이미 실행 중인 시스템에 웹 모니터만 연결
    ./run_web_monitor.sh --port 8081 --public-host 192.168.0.10   # 포트 또는 QR 주소 직접 지정

QR 페이지(`http://127.0.0.1:8080/qr`)가 자동으로 열린다. 터미널에도 주소와 QR코드가 출력된다.
고장 주입은 tkinter 대시보드의 버튼을 사용한다.

## 휴대폰 접속

1. PC와 휴대폰을 같은 네트워크에 연결한다.
2. PC 화면의 QR코드를 휴대폰 카메라로 스캔한다. 앱 설치와 회원가입은 필요 없다.

페이지가 열리지 않으면 네트워크가 기기 간 통신을 막고 있을 가능성이 크다(대학교 Wi-Fi에서 흔함).

- 휴대폰 핫스팟: 휴대폰 핫스팟을 켜고 PC를 연결한다. 3초 안에 QR코드가 새 주소로 갱신된다.
- 공유기: PC와 휴대폰을 같은 공유기에 연결한다. 게스트 네트워크는 기기 간 통신이 막혀 있을 수 있다.
- 방화벽: ufw를 켠 경우 `sudo ufw allow 8080/tcp`.

서버는 LAN에만 열리며 외부 터널을 만들지 않는다. 시연 네트워크 안의 누구나 볼 수 있으므로 신뢰할 수 있는 네트워크에서 사용한다.

## 범위

- 브라우저를 열어 둔 동안의 실시간 경고가 대상이다. 잠금 화면 푸시 알림은 제외했다.
- 복구 소요 시간은 실제 시간(wall clock) 기준이다. 웹 서버가 꺼진 동안 시작된 고장은 "최소 N초"로 표시한다.

## 테스트

    source env.sh
    .venv_web/bin/python -m pytest -q -p no:anyio -p no:launch_testing_ros_pytest_entrypoint src/fault_web/test
    ./run_web_monitor_test.sh       # Gazebo + 실제 고장 주입 + 실제 브라우저, 결과: results/web_monitor_summary.json
