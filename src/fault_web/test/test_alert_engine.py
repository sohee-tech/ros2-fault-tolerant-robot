"""Alert logic with synthetic status (no ROS, no web)."""
import math

from fault_web.alert_engine import AlertEngine, diagnose, gate_reason_ko, zone_of
from fault_web import network

ZONES = [{'name': '동쪽 통로', 'x': [1, 7], 'y': [-1, 1]}]
OK_CHECKS = {k: {'ok': True, 'detail': ''} for k in ('system', 'lidar', 'odom', 'nav2', 'localization', 'obstacle', 'control')}


def status(t, system='NORMAL', lidar='NORMAL', nav2='NORMAL', gate='APPROVED', recovery='IDLE', mission='RUNNING', checks=None,
           pose_t=None, cov=0.01, x=3.0, safety=False, scan_age=0.1, attempts=0, nodes=None):
    gate_status = {'state': gate, 'reason': '', 'checks': {**OK_CHECKS, **(checks or {})}, 'verify_elapsed_s': None,
                   'stable_time_s': 3.0}
    return {'health': {'state': system, 'lidar_state': lidar, 'odom_state': 'NORMAL', 'control_state': 'NORMAL',
                       'nav_state': 'NORMAL', 'nav2_state': nav2},
            'health_age': 0.05, 'scan_age': scan_age, 'gate_status': gate_status, 'gate_status_age': 0.1,
            'recovery_state': recovery, 'nav2_age': 0.1, 'recovery_attempts': attempts, 'nav2_nodes': nodes or {},
            'mission': {'state': mission, 'index': 3, 'total': 5, 'name': 'wp', 'reached': 2}, 'mission_age': 0.1,
            'pose': {'x': x, 'y': 0.0, 'cov_xx': cov, 'cov_yy': cov, 'frame': 'map', 't': t if pose_t is None else pose_t},
            'safety_age': 0.05 if safety else None, 'speed_limit': 0.0}


def engine():
    e = AlertEngine(zones=ZONES)
    e.update(status(0.0), 0.0)
    return e


def kinds(e, kind):
    return [a for a in e.alerts if a['kind'] == kind]


LOST = dict(system='CRITICAL', lidar='CRITICAL', gate='WAITING', mission='PAUSED', safety=True, scan_age=5.0)


def test_normal_view():
    v = engine().update(status(1.0), 1.0)
    assert v['banner']['level'] == 'ok' and '정상 운행' in v['banner']['title'] and not v['fault']['active']
    assert v['position']['zone'] == '동쪽 통로' and v['mission']['text'] == '배송 3/5 (wp)' and not v['alerts']


def test_critical_alert_is_immediate_and_once():
    e = engine()
    for t in range(1, 40):
        v = e.update(status(float(t), pose_t=1.0, **LOST), float(t))
        if t == 1:
            assert v['banner']['level'] == 'critical'      # no waiting for the recovery
            assert v['fault']['type'] == 'LiDAR 데이터 수신 중단' and v['safe_stop'] and v['response'] == '안전 정지'
    assert len(kinds(e, 'critical')) == 1 and len(e.incidents) == 1
    assert v['action']['required'] and '30초' in v['action']['reasons'][0] and len(kinds(e, 'action')) == 1
    alert = kinds(e, 'critical')[0]
    for key in ('robot_id', 'time', 'fault_type', 'system', 'position', 'safe_stop', 'recovery', 'safe_resume', 'action_required'):
        assert key in alert


def test_escalation_keeps_one_incident():
    e = engine()
    e.update(status(1.0, system='WARNING', lidar='WARNING', scan_age=0.4), 1.0)
    e.update(status(1.4, system='DEGRADED', lidar='DEGRADED', scan_age=0.8), 1.4)
    e.update(status(1.8, **LOST), 1.8)
    assert [a['kind'] for a in e.alerts] == ['warning', 'degraded', 'critical'] and len(e.incidents) == 1
    assert e.incidents[0]['causes'][0]['label'] == 'LiDAR 데이터 수신 중단'


def test_full_recovery_sequence_and_duration():
    e = engine()
    e.update(status(10.0, **LOST), 10.0)
    v = e.update(status(12.0, system='CRITICAL', lidar='CRITICAL', gate='BLOCKED', mission='PAUSED', safety=True, scan_age=0.1,
                        checks={'lidar': {'ok': False, 'detail': 'health lidar CRITICAL'}}), 12.0)
    assert v['fault']['type'] == 'LiDAR 데이터 수신 중단'      # the settling text does not replace the cause
    v = e.update(status(14.0, gate='VERIFYING', mission='PAUSED'), 14.0)
    assert v['fault']['active'] and v['banner']['level'] == 'recovery'
    v = e.update(status(17.0, gate='APPROVED', mission='PAUSED'), 17.0)
    assert v['fault']['active'], 'stays open until the mission runs'
    v = e.update(status(17.4), 17.4)
    assert not v['fault']['active'] and v['resolved']['show'] and v['resolved']['duration_s'] == 7.4
    assert '정상 복구' in v['banner']['title'] and kinds(e, 'resolved') and kinds(e, 'approved')
    v = e.update(status(40.0), 40.0)
    assert not v['resolved']['show'] and '정상 운행' in v['banner']['title']
    assert v['incidents'][0]['resolved']


def test_recovery_failed_requires_action_once():
    e = engine()
    for t in (1.0, 2.0, 3.0):
        v = e.update(status(t, system='CRITICAL', nav2='CRITICAL', gate='BLOCKED', mission='PAUSED', recovery='FAILED', safety=True,
                            attempts=3, nodes={'controller_server': {'state': 'INACTIVE'}},
                            checks={'nav2': {'ok': False, 'detail': 'not ACTIVE: controller_server (recovery FAILED)'}}), t)
    assert v['action']['required'] and '관리자 조치' in v['banner']['title'] and '3회' in v['action']['reasons'][0]
    assert len(kinds(e, 'recovery_failed')) == 1 and 'Nav2 서버 비활성' in v['fault']['type']
    assert v['safe_resume']['reasons'][0]['text'] == 'Nav2 서버 비활성'


def test_blocked_reason_without_system_fault_dedupes_on_changing_numbers():
    e = engine()
    for t, d in ((1.0, 0.20), (2.0, 0.21), (3.0, 0.19)):
        v = e.update(status(t, gate='BLOCKED', mission='PAUSED',
                            checks={'obstacle': {'ok': False, 'detail': f'obstacle {d:.2f} m ahead < required 0.30 m'}}), t)
    assert v['banner']['level'] == 'blocked' and '전방 장애물 근접' in v['banner']['text']
    assert v['fault']['type'] == '전방 장애물 근접' and len(kinds(e, 'blocked')) == 1


def test_startup_block_is_not_an_incident():
    e = AlertEngine(zones=ZONES)
    v = e.update(status(0.0, gate='BLOCKED', mission='IDLE', checks={'nav2': {'ok': False, 'detail': 'not ACTIVE: x'}}), 0.0)
    assert not v['fault']['active'] and not e.alerts and v['banner']['level'] == 'recovery'


def test_stale_health_is_never_normal():
    e = engine()
    s = status(1.0)
    s['health_age'] = 10.0
    v = e.update(s, 1.0)
    assert v['system_state'] == 'UNKNOWN' and v['banner']['level'] == 'unknown' and '정상' not in v['banner']['title']
    assert v['response'] == '확인 불가' and len(kinds(e, 'link_lost')) == 1
    e.update(s, 2.0)
    assert len(kinds(e, 'link_lost')) == 1
    e.update(status(3.0), 3.0)
    assert kinds(e, 'link_restored')


def test_stale_gate_and_mission_are_unknown():
    s = status(1.0)
    s['gate_status_age'] = 9.0
    s['mission_age'] = 9.0
    v = engine().update(s, 1.0)
    assert v['safe_resume']['state'] == 'UNKNOWN' and not v['safe_resume']['approved']
    assert v['mission']['text'] == '확인 불가' and v['banner']['level'] == 'warning'


def test_invalid_pose_shows_last_position_with_time():
    e = engine()
    for bad in (status(10.0, pose_t=1.0), status(10.0, cov=5.0), status(10.0, cov=math.nan)):
        p = e.update(bad, 10.0)['position']
        assert not p['valid'] and p['text'].startswith('마지막 확인 위치: 동쪽 통로') and '기준' in p['text']
    s = status(10.0)
    del s['pose']
    assert AlertEngine(zones=ZONES).update(s, 10.0)['position']['text'].startswith('위치 확인 불가')


def test_recurrence_reopens_and_limits_alerts():
    e = engine()
    t = 1.0
    for _ in range(8):
        e.update(status(t, **LOST), t)
        e.update(status(t + 1, ), t + 1)
        t += 3
    assert len(e.incidents) == 1 and e.incidents[0]['repeats'] == 7
    assert len(kinds(e, 'critical')) == 1 and len(kinds(e, 'recurred')) == 3


def test_different_cause_soon_after_recovery_is_a_new_incident():
    e = engine()
    e.update(status(1.0, **LOST), 1.0)
    e.update(status(5.0), 5.0)
    assert e.incidents[0]['resolved']
    # a Nav2 failure is first visible only as a recovery state, without any cause yet
    e.update(status(6.0, recovery='DETECTED', gate='BLOCKED', mission='PAUSED'), 6.0)
    v = e.update(status(7.0, system='CRITICAL', nav2='CRITICAL', recovery='RECOVERING', gate='BLOCKED', mission='PAUSED',
                        nodes={'controller_server': {'state': 'MISSING'}}), 7.0)
    assert len(e.incidents) == 2 and v['fault']['type'] == 'Nav2 서버 비활성' and v['fault']['elapsed_s'] == 1.0
    assert len(kinds(e, 'critical')) == 2


def test_fault_older_than_the_server_is_not_given_a_false_start_time():
    e = AlertEngine(zones=ZONES)
    v = e.update(status(100.0, **LOST), 100.0)
    assert '이전' in v['fault']['opened']
    e.update(status(101.0, gate='APPROVED'), 104.0)
    v = e.update(status(105.0), 105.0)
    assert v['resolved']['at_least'] and v['banner']['text'] == '복구 소요 최소 4.0초'
    v = engine().update(status(50.0, **LOST), 50.0)
    assert v['fault']['opened'] and '이전' not in v['fault']['opened']


def test_persist_restores_open_incident():
    e = engine()
    e.update(status(1.0, **LOST), 1.0)
    r = AlertEngine(zones=ZONES)
    r.load(e.dump())
    v = r.update(status(5.0, **LOST), 5.0)
    assert v['fault']['id'] == 1 and v['fault']['opened'] and len(kinds(r, 'critical')) == 1


def test_helpers():
    assert zone_of(9, 9, ZONES) == '창고 구역 외'
    assert gate_reason_ko('lidar', 'no scan received') == 'LiDAR 데이터 미수신'
    assert gate_reason_ko('localization', 'xy covariance 1.2 > 1.0 m^2') == '위치 추정 불확실 (공분산 초과)'
    assert gate_reason_ko('localization', 'pose or covariance is not finite') == 'AMCL 위치 데이터 무효'
    assert diagnose({'health': {'lidar_state': 'CRITICAL'}, 'scan_age': 3.0})[0]['label'] == 'LiDAR 데이터 수신 중단'
    assert '고장' not in diagnose({'health': {'lidar_state': 'CRITICAL'}, 'scan_age': 3.0})[0]['label']


def test_network_never_uses_loopback_or_virtual():
    addrs = [('lo', '127.0.0.1'), ('docker0', '172.17.0.1'), ('wlan0', '192.168.0.7'), ('eth0', '10.0.0.5'), ('wlan1', '169.254.1.1')]
    assert network.candidates(addrs, primary='10.0.0.5') == ['10.0.0.5', '192.168.0.7']
    assert network.candidates([('lo', '127.0.0.1')], primary='127.0.0.1') == []
