"""Administrator alert logic: raw status -> what the page shows + deduplicated alerts. No ROS, no web.

Incident: one fault episode, from the first abnormal status until the system is NORMAL, Safe Resume is
APPROVED and the mission runs again. Each alert kind fires once per incident (key `dedupe`), so a persisting
fault never repeats. A fault that returns within `reopen_window` s with the same cause reopens the incident.

Causes come only from diagnostic data: "LiDAR 데이터 수신 중단" is what /health/lidar_state and the scan age show;
nothing here concludes a hardware fault. Old data is never shown as current: a silent health_monitor gives the
state UNKNOWN, and an invalid or old pose is shown as the last confirmed position with its time.
"""
import math
import re
import time

ACTIVE_RECOVERY = ('DETECTED', 'RECOVERING', 'VERIFY', 'WAIT_RETRY')
LEVELS = {'NORMAL': 0, 'WARNING': 1, 'DEGRADED': 2, 'CRITICAL': 3}
RECOVERY_TEXT = {'DETECTED': '장애 감지, 복구 준비', 'RECOVERING': 'Nav2 자동 복구 진행 중', 'VERIFY': 'Nav2 복구 결과 확인 중',
                 'WAIT_RETRY': '복구 실패, 재시도 대기', 'RECOVERED': 'Nav2 복구 완료, 안정화 확인 중',
                 'FAILED': 'Nav2 자동 복구 실패'}
MISSION_TEXT = {'IDLE': '대기', 'RUNNING': '주행 중', 'PAUSED': '일시 정지', 'CHECK_ABORT': '중단 원인 확인 중',
                'RETRY_WAIT': '재시도 대기', 'RECOVERING': '복구 대기', 'WAIT_NAV2': 'Nav2 대기', 'COMPLETE': '완료',
                'FAILED': '실패'}
CHECK_LABEL = {'system': '시스템', 'lidar': 'LiDAR', 'odom': 'Odometry', 'nav2': 'Nav2', 'localization': '위치 추정',
               'obstacle': '전방 장애물', 'control': '제어 지연', 'gate': '판단 노드'}
GATE_PATTERNS = [
    ('lidar', r'scan age|no scan|stamp', 'LiDAR 데이터 미수신'), ('lidar', r'invalid scan', 'LiDAR 데이터 무효'),
    ('lidar', r'scan rate', 'LiDAR 수신 주기 부족'), ('lidar', r'health lidar', 'LiDAR 안정화 확인 중'),
    ('odom', r'not running', 'Odometry 검증 데이터 없음'), ('odom', r'health odom', 'Odometry 이상 상태'),
    ('odom', r'implausible', '최근 Odometry 이상값 감지'), ('odom', r'age|rate', 'Odometry 데이터 부족'),
    ('nav2', r'not ACTIVE', 'Nav2 서버 비활성'), ('nav2', r'unavailable', 'Nav2 상태 확인 불가'),
    ('nav2', r'recovery|health nav2', 'Nav2 복구 안정화 확인 중'),
    ('localization', r'no /amcl|age', 'AMCL 위치 데이터 미수신'), ('localization', r'covariance \d', '위치 추정 불확실 (공분산 초과)'),
    ('localization', r'', 'AMCL 위치 데이터 무효'),
    ('obstacle', r'unknown', '전방 안전거리 확인 불가'), ('obstacle', r'ahead', '전방 장애물 근접'),
    ('control', r'latency \d', '제어 지연 허용치 초과'), ('control', r'sample', '제어 지연 측정값 없음'),
    ('control', r'health', '제어 지연 안정화 확인 중'),
    ('system', r'unavailable', '고장 진단 노드 상태 확인 불가'), ('system', r'', '시스템 정상화 대기'),
]
CRITICAL_ICON = '\U0001F6A8'


def clock(ts):
    return time.strftime('%H:%M:%S', time.localtime(ts)) if ts else None


def zone_of(x, y, zones):
    for z in zones:
        if z['x'][0] <= x <= z['x'][1] and z['y'][0] <= y <= z['y'][1]:
            return z['name']
    return '창고 구역 외'


def gate_reason_ko(check, detail):
    for name, pattern, text in GATE_PATTERNS:
        if name == check and re.search(pattern, detail or ''):
            return text
    return f'{CHECK_LABEL.get(check, check)} 조건 미충족'


def diagnose(s):
    """[{key, level, label, detail}] from the per-subsystem health states: only what the data shows."""
    out = []
    h = s.get('health', {})
    level = lambda k: LEVELS.get(h.get(k), 0)
    age = s.get('scan_age')
    if level('lidar_state'):
        receiving = age is not None and age < 0.3
        text = ('LiDAR 수신 재개, 안정화 확인 중' if receiving
                else 'LiDAR 데이터 수신 중단' if level('lidar_state') == 3 else 'LiDAR 데이터 수신 지연')
        out.append({'key': 'lidar', 'level': level('lidar_state'), 'label': text, 'settling': receiving,
                    'detail': f'마지막 수신 {age:.1f}초 전' if age is not None else ''})
    if level('odom_state'):
        out.append({'key': 'odom', 'level': level('odom_state'), 'label': 'Odometry 비정상 값 감지',
                    'detail': '타당성 검사 실패 (속도·가속도·명령 대비)'})
    if level('control_state'):
        lat = s.get('latency')
        ok = lat is not None and lat < 0.10
        out.append({'key': 'control', 'level': level('control_state'), 'settling': ok,
                    'label': '제어 지연 정상화, 안정화 확인 중' if ok else '제어 명령 지연 초과',
                    'detail': f'측정 지연 {lat * 1000:.0f} ms' if lat is not None else ''})
    if level('nav_state'):
        out.append({'key': 'nav', 'level': level('nav_state'), 'label': '주행 명령 노드 응답 중단',
                    'detail': f'heartbeat 끊김, 재시작 {s.get("nav_restarts", 0)}회'})
    if level('nav2_state'):
        nodes = s.get('nav2_nodes') or {}
        bad = [n for n, v in nodes.items() if v.get('state') != 'ACTIVE']
        out.append({'key': 'nav2', 'level': level('nav2_state'), 'settling': not bad,
                    'label': 'Nav2 서버 비활성' if bad else 'Nav2 안정화 확인 중',
                    'detail': ', '.join(f'{n}={nodes[n].get("state")}' for n in bad)})
    return sorted(out, key=lambda d: -d['level'])


class AlertEngine:
    def __init__(self, zones=(), robot_id='AMR-01', health_timeout=2.0, gate_timeout=2.0, mission_timeout=3.0,
                 pose_max_age=6.0, pose_max_cov=1.0, reopen_window=10.0, resolved_hold=20.0, action_after=30.0,
                 max_alerts=50, max_incidents=20, max_repeat_alerts=3):
        self.max_repeat_alerts = max_repeat_alerts
        self.zones = list(zones)
        self.robot_id = robot_id
        self.health_timeout = health_timeout
        self.gate_timeout = gate_timeout
        self.mission_timeout = mission_timeout
        self.pose_max_age = pose_max_age
        self.pose_max_cov = pose_max_cov
        self.reopen_window = reopen_window
        self.resolved_hold = resolved_hold
        self.action_after = action_after
        self.max_alerts = max_alerts
        self.max_incidents = max_incidents
        self.alerts = []
        self.incidents = []
        self.current = None           # open incident
        self.next_alert = 1
        self.next_incident = 1
        self.seen_approved = False
        self.link_lost_alerted = False
        self.ever_link = False
        self.first_link = None
        self.start_grace = 2.0
        self.last_pose = None         # last valid pose
        self.mission_alerted = None
        self.dirty = False

    # --- persistence: the history survives a web server restart ---
    def dump(self):
        return {'alerts': self.alerts, 'incidents': self.incidents, 'next_alert': self.next_alert,
                'next_incident': self.next_incident, 'seen_approved': self.seen_approved,
                'last_pose': self.last_pose, 'saved_at': time.time()}

    def load(self, data):
        self.alerts = data.get('alerts', [])
        self.incidents = data.get('incidents', [])
        self.next_alert = data.get('next_alert', 1)
        self.next_incident = data.get('next_incident', 1)
        self.seen_approved = data.get('seen_approved', False)
        self.last_pose = data.get('last_pose')
        self.current = next((i for i in self.incidents if i['closed_at'] is None), None)
        if self.current:
            self.current['restored'] = True

    # --- pose ---
    def position(self, s, now):
        p = s.get('pose')
        problem = None
        if p is None:
            problem = '위치 데이터 없음'
        elif p.get('frame') != 'map' or not all(math.isfinite(p[k]) for k in ('x', 'y', 'cov_xx', 'cov_yy')):
            problem = '위치 데이터 무효'
        elif max(p['cov_xx'], p['cov_yy']) > self.pose_max_cov:
            problem = '위치 추정 불확실'
        elif now - p['t'] > self.pose_max_age:
            problem = '위치 데이터 오래됨'
        elif s.get('gate_checks', {}).get('localization', {}).get('ok') is False and s.get('gate_fresh'):
            problem = '위치 추정 조건 미충족'
        if problem is None:
            self.last_pose = {'x': p['x'], 'y': p['y'], 't': p['t'], 'zone': zone_of(p['x'], p['y'], self.zones)}
            return {'valid': True, 'x': round(p['x'], 2), 'y': round(p['y'], 2), 'zone': self.last_pose['zone'],
                    'as_of': clock(p['t']), 'problem': None, 'text': self.last_pose['zone']}
        lp = self.last_pose
        if lp is None:
            return {'valid': False, 'x': None, 'y': None, 'zone': None, 'as_of': None, 'problem': problem,
                    'text': f'위치 확인 불가 ({problem})'}
        return {'valid': False, 'x': round(lp['x'], 2), 'y': round(lp['y'], 2), 'zone': lp['zone'],
                'as_of': clock(lp['t']), 'problem': problem,
                'text': f'마지막 확인 위치: {lp["zone"]} ({clock(lp["t"])} 기준, {problem})'}

    # --- alerts ---
    def emit(self, now, kind, severity, title, text, view_ctx, dedupe=True):
        inc = self.current
        if inc is not None and dedupe:
            key = kind if dedupe is True else f'{kind}:{dedupe}'
            if key in inc['alerted']:
                return
            inc['alerted'].append(key)
        alert = {'id': self.next_alert, 'ts': now, 'time': clock(now), 'incident': inc['id'] if inc else None,
                 'kind': kind, 'severity': severity, 'title': title, 'text': text, 'robot_id': self.robot_id,
                 **view_ctx}
        self.next_alert += 1
        self.alerts.append(alert)
        del self.alerts[:-self.max_alerts]
        self.dirty = True

    def open_incident(self, now, pos, keys):
        """keys: subsystems known to be failing. Only the same cause reopens the previous incident."""
        last = self.incidents[-1] if self.incidents else None
        if (last and last['closed_at'] and now - last['closed_at'] < self.reopen_window
                and keys & {c['key'] for c in last['causes']}):
            last.update(closed_at=None, closed=None, resolved=False, duration_s=None, repeats=last.get('repeats', 0) + 1)
            self.current = last
            return
        # already abnormal when this server first saw the robot: it began earlier, at an unknown time
        before_start = self.first_link is not None and now - self.first_link <= self.start_grace
        self.current = {'id': self.next_incident, 'opened_at': now, 'closed_at': None, 'before_start': before_start,
                        'opened': f'{clock(now)} 이전 (웹 서버 중단 중 발생)' if before_start else clock(now),
                        'closed': None, 'level': 'NORMAL', 'causes': [], 'location': pos['text'],
                        'location_valid': pos['valid'], 'resolved': False, 'duration_s': None, 'alerted': [],
                        'action_reason': None, 'repeats': 0, 'blocked': []}
        self.next_incident += 1
        self.incidents.append(self.current)
        del self.incidents[:-self.max_incidents]

    @staticmethod
    def merge_causes(inc, diag, gate_failed):
        """Keep the worst thing seen per subsystem; a 'settling' label never replaces a real one."""
        known = {c['key']: c for c in inc['causes']}
        for d in diag:
            c = known.get(d['key'])
            if c is None:
                c = {'key': d['key'], 'level': d['level'], 'label': d['label'], 'detail': d['detail']}
                inc['causes'].append(c)
                known[d['key']] = c
            elif not d.get('settling') and d['level'] >= c['level']:
                c.update(level=d['level'], label=d['label'], detail=d['detail'])
        if not inc['causes']:
            for g in gate_failed:
                if g['check'] != 'system':
                    inc['causes'].append({'key': g['check'], 'level': 0, 'label': g['text'], 'detail': g['detail']})

    # --- main ---
    def update(self, s, now=None):
        """s: raw status (see ros_bridge.StatusNode.snapshot). Returns the view sent to the browsers."""
        now = time.time() if now is None else now
        link = s.get('health_age') is not None and s['health_age'] <= self.health_timeout
        self.ever_link |= link
        if link and self.first_link is None:
            self.first_link = now
        health = s.get('health', {})
        system = health.get('state') if link else None
        s = dict(s)
        gate_fresh = s.get('gate_status_age') is not None and s['gate_status_age'] <= self.gate_timeout
        s['gate_fresh'] = gate_fresh
        gate_status = s.get('gate_status') or {}
        s['gate_checks'] = gate_status.get('checks', {}) if gate_fresh else {}
        gate_state = gate_status.get('state') if gate_fresh else None
        recovery_known = s.get('nav2_age') is not None and s['nav2_age'] <= self.gate_timeout
        recovery = s.get('recovery_state') if recovery_known else None
        mission_fresh = s.get('mission_age') is not None and s['mission_age'] <= self.mission_timeout
        mission = (s.get('mission') or {}) if mission_fresh else {}
        mstate = mission.get('state')

        pos = self.position(s, now)
        diag = diagnose(s) if link else []
        gate_failed = [{'check': k, 'label': CHECK_LABEL.get(k, k), 'text': gate_reason_ko(k, v.get('detail')),
                        'detail': v.get('detail', '')} for k, v in s['gate_checks'].items() if not v.get('ok')]
        if gate_fresh and gate_status.get('reason', '').startswith('gate:'):
            gate_failed.insert(0, {'check': 'gate', 'label': CHECK_LABEL['gate'], 'text': '판단 노드 평가 중단 감지',
                                   'detail': gate_status['reason']})
        self.seen_approved |= gate_state == 'APPROVED'
        safety_on = s.get('safety_age') is not None and s['safety_age'] < 0.5
        limit = s.get('speed_limit') or 0.0

        # --- open / close the incident ---
        abnormal = (system is not None and system != 'NORMAL') or recovery in ACTIVE_RECOVERY + ('FAILED',) \
            or (gate_state == 'BLOCKED' and self.seen_approved)
        if abnormal and self.current is None:
            self.open_incident(now, pos, {d['key'] for d in diag}
                               | {g['check'] for g in gate_failed if g['check'] not in ('system', 'gate')})
            self.dirty = True
        inc = self.current
        if inc is not None and abnormal:
            self.merge_causes(inc, diag, gate_failed)
            if system in LEVELS and LEVELS[system] > LEVELS[inc['level']]:
                inc['level'] = system

        response = ('안전 정지' if safety_on else
                    '재출발 차단으로 정지 유지' if gate_state in ('BLOCKED', 'WAITING', 'VERIFYING') else
                    f'속도 제한 {limit:.2f} m/s' if limit > 0 else
                    '정상 주행' if mstate == 'RUNNING' else '주행 명령 대기')
        if not link:
            response = '확인 불가'
        rec_text = RECOVERY_TEXT.get(recovery)
        if rec_text is None:
            if inc and any(d['key'] == 'nav' for d in diag):
                rec_text = '주행 노드 재시작 대기'
            elif diag:
                rec_text = '정상화 대기'
            elif gate_state == 'VERIFYING' and inc:
                rec_text = '재출발 안전 조건 검증 중'
            elif gate_state in ('BLOCKED', 'WAITING') and inc:
                rec_text = '안전 조건 충족 대기'
            elif not recovery_known:
                rec_text = '확인 불가'
            else:
                rec_text = '해당 없음'
        attempts = s.get('recovery_attempts') or 0
        if recovery in ('RECOVERING', 'VERIFY', 'WAIT_RETRY') and attempts:
            rec_text += f' ({attempts}회 시도)'

        # --- required action ---
        reasons = []
        if recovery == 'FAILED':
            reasons.append(f'Nav2 자동 복구 실패 ({attempts}회 시도): {s.get("failed_node") or "서버"} 확인 필요')
        if mstate == 'FAILED':
            reasons.append('임무 실패: 재시작 판단 필요')
        if inc and now - inc['opened_at'] >= self.action_after and (safety_on or gate_state in ('BLOCKED', 'WAITING')):
            reasons.append(f'{self.action_after:.0f}초 이상 자동 정상화되지 않음: 현장 확인 필요')

        fault_type = ' / '.join(c['label'] for c in inc['causes']) if inc and inc['causes'] else None
        ctx = {'fault_type': fault_type, 'system': system or 'UNKNOWN', 'position': pos['text'],
               'position_valid': pos['valid'], 'safe_stop': safety_on, 'recovery': rec_text,
               'safe_resume': gate_state or 'UNKNOWN', 'action_required': bool(reasons)}

        # --- alerts ---
        if not link and self.ever_link and not self.link_lost_alerted:
            self.link_lost_alerted = True
            self.emit(now, 'link_lost', 'critical', '로봇 상태 수신 중단',
                      '고장 진단 노드 데이터가 들어오지 않아 현재 상태를 확인할 수 없습니다.', ctx, dedupe=False)
        elif link and self.link_lost_alerted:
            self.link_lost_alerted = False
            self.emit(now, 'link_restored', 'info', '로봇 상태 수신 재개', f'현재 상태: {system}', ctx, dedupe=False)
        if inc:
            if 0 < inc['repeats'] <= self.max_repeat_alerts:
                self.emit(now, 'recurred', 'critical', f'{CRITICAL_ICON} 동일 장애 재발 ({inc["repeats"]}회)',
                          f'{fault_type or "원인 확인 중"} / {response}', ctx, dedupe=inc['repeats'])
            if system == 'CRITICAL':
                self.emit(now, 'critical', 'critical', f'{CRITICAL_ICON} 긴급 장애 발생',
                          f'{fault_type or "원인 확인 중"} / {response}', ctx)
            elif system == 'DEGRADED':
                self.emit(now, 'degraded', 'degraded', '성능 저하: 감속 운행', f'{fault_type or "원인 확인 중"} / {response}', ctx)
            elif system == 'WARNING':
                self.emit(now, 'warning', 'warning', '이상 징후 감지', fault_type or '원인 확인 중', ctx)
            if recovery in ('RECOVERING', 'DETECTED'):
                self.emit(now, 'recovery_start', 'recovery', 'Nav2 자동 복구 시작', rec_text, ctx)
            if recovery == 'RECOVERED':
                self.emit(now, 'recovery_done', 'recovery', 'Nav2 서버 복구 완료', '안정화 확인 후 재출발을 검증합니다.', ctx)
            if recovery == 'FAILED':
                self.emit(now, 'recovery_failed', 'critical', '자동 복구 실패: 관리자 조치 필요', reasons[0], ctx)
            if gate_state == 'BLOCKED' and gate_failed:
                names = ','.join(sorted(g['check'] for g in gate_failed))
                self.emit(now, 'blocked', 'blocked', 'Safe Resume 재출발 차단',
                          '; '.join(g['text'] for g in gate_failed), ctx, dedupe=names)
            if reasons:
                self.emit(now, 'action', 'critical', '관리자 조치 필요', reasons[-1], ctx,
                          dedupe=reasons[-1][:6])
                inc['action_reason'] = reasons[-1]
            if gate_state == 'APPROVED' and not abnormal:
                self.emit(now, 'approved', 'ok', '재출발 승인', 'Safe Resume 안전 조건을 모두 충족했습니다.', ctx)

            # closed only when the mission really runs again
            if not abnormal and link and gate_state == 'APPROVED' and system == 'NORMAL' \
                    and recovery in ('IDLE', 'RECOVERED', None) and mstate in ('RUNNING', 'COMPLETE', 'IDLE', None):
                inc['closed_at'] = now
                inc['closed'] = clock(now)
                inc['resolved'] = True
                inc['duration_s'] = round(now - inc['opened_at'], 1)
                inc['location_now'] = pos['text']
                mtext = f'임무 재개: {mission_label(mission)}' if mstate == 'RUNNING' else '정상 복귀'
                least = '최소 ' if inc.get('before_start') else ''
                self.emit(now, 'resolved', 'ok', '✅ 정상 복구', f'{mtext} (복구 소요 {least}{inc["duration_s"]}초)', ctx)
                self.current = None
                inc = None
                self.dirty = True

        if mstate in ('COMPLETE', 'FAILED') and self.mission_alerted != mstate:
            self.mission_alerted = mstate
            if mstate == 'COMPLETE':
                self.emit(now, 'mission_complete', 'ok', '임무 완료', mission_label(mission), ctx, dedupe=False)
            else:
                self.emit(now, 'mission_failed', 'critical', '임무 실패: 관리자 조치 필요', mission_label(mission), ctx, dedupe=False)
        elif mstate == 'RUNNING':
            self.mission_alerted = None

        last_closed = next((i for i in reversed(self.incidents) if i['closed_at']), None)
        show_resolved = bool(last_closed and not inc and now - last_closed['closed_at'] <= self.resolved_hold)
        return {
            'robot_id': self.robot_id, 'time': clock(now), 'link_ok': link,
            'system_state': system or 'UNKNOWN',
            'subsystems': {k: (health.get(f'{k}_state') if link else None)
                           for k in ('lidar', 'odom', 'control', 'nav', 'nav2')},
            'banner': self.banner(link, system, gate_state, recovery, reasons, inc, fault_type, diag, gate_failed,
                                  mission, show_resolved, last_closed),
            'fault': {'active': inc is not None, 'id': inc['id'] if inc else None, 'type': fault_type,
                      'level': inc['level'] if inc else None, 'opened': inc['opened'] if inc else None,
                      'elapsed_s': round(now - inc['opened_at'], 1) if inc else None,
                      'location': inc['location'] if inc else None,
                      'location_valid': inc['location_valid'] if inc else None,
                      'causes': [{'label': c['label'], 'detail': c['detail']} for c in inc['causes']] if inc else []},
            'position': pos, 'response': response, 'safe_stop': safety_on, 'speed_limit': limit,
            'cmd_vx': s.get('cmd_vx'),
            'recovery': {'state': recovery, 'text': rec_text, 'attempts': attempts, 'failed_node': s.get('failed_node'),
                         'active': recovery in ACTIVE_RECOVERY},
            'safe_resume': {'known': gate_fresh, 'state': gate_state or 'UNKNOWN', 'approved': gate_state == 'APPROVED',
                            'reasons': gate_failed, 'verify_elapsed_s': gate_status.get('verify_elapsed_s'),
                            'stable_time_s': gate_status.get('stable_time_s'),
                            'obstacle_m': gate_status.get('obstacle_distance_m'),
                            'required_m': gate_status.get('required_clearance_m')},
            'mission': {'known': mission_fresh, 'state': mstate or 'UNKNOWN',
                        'state_text': MISSION_TEXT.get(mstate, '확인 불가') if mission_fresh else '확인 불가',
                        'text': mission_label(mission) if mission_fresh else '확인 불가',
                        'reached': mission.get('reached'), 'total': mission.get('total')},
            'action': {'required': bool(reasons), 'reasons': reasons},
            'resolved': ({'show': True, 'id': last_closed['id'], 'type': ' / '.join(c['label'] for c in last_closed['causes']),
                          'duration_s': last_closed['duration_s'], 'closed': last_closed['closed'],
                          'mission': mission_label(mission) if mission_fresh else None,
                          'restored': last_closed.get('restored', False),
                          'at_least': last_closed.get('before_start', False)} if show_resolved else {'show': False}),
            'alerts': list(reversed(self.alerts[-30:])),
            'incidents': [{k: i.get(k) for k in ('id', 'opened', 'closed', 'level', 'location', 'location_valid', 'resolved',
                                                 'duration_s', 'repeats', 'action_reason', 'before_start')}
                          | {'type': ' / '.join(c['label'] for c in i['causes']) or '원인 확인 중'}
                          for i in reversed(self.incidents)],
        }

    def banner(self, link, system, gate, recovery, reasons, inc, fault_type, diag, gate_failed, mission, show_resolved,
               last_closed):
        rid = self.robot_id
        stop = '안전 정지'
        if not link:
            return {'level': 'unknown', 'title': '로봇 상태 확인 불가',
                    'text': f'{rid} 고장 진단 데이터가 수신되지 않습니다. 정상 상태로 간주하지 마세요.'}
        if reasons:
            return {'level': 'critical', 'title': f'{CRITICAL_ICON} 관리자 조치 필요', 'text': reasons[0]}
        if system == 'CRITICAL':
            return {'level': 'critical', 'title': f'{CRITICAL_ICON} 긴급 장애 발생',
                    'text': f'{rid} {stop}: {fault_type or "원인 확인 중"}'}
        if recovery in ACTIVE_RECOVERY:
            return {'level': 'recovery', 'title': 'Nav2 자동 복구 진행 중', 'text': RECOVERY_TEXT[recovery]}
        if gate == 'BLOCKED' and self.seen_approved:
            return {'level': 'blocked', 'title': '재출발 차단',
                    'text': '; '.join(g['text'] for g in gate_failed) or '안전 조건 미충족'}
        if system == 'DEGRADED':
            return {'level': 'degraded', 'title': f'{rid} 감속 운행', 'text': fault_type or '원인 확인 중'}
        if system == 'WARNING':
            return {'level': 'warning', 'title': f'{rid} 이상 징후', 'text': fault_type or '원인 확인 중'}
        if inc:
            return {'level': 'recovery', 'title': '정상 복귀 진행 중',
                    'text': '재출발 안전 조건 검증 중' if gate == 'VERIFYING' else '임무 재개 대기'}
        if gate is None:
            return {'level': 'warning', 'title': 'Safe Resume 상태 확인 불가', 'text': '재출발 승인 여부를 확인할 수 없습니다.'}
        if gate != 'APPROVED':
            return {'level': 'recovery', 'title': f'{rid} 시작 준비 중', 'text': f'Safe Resume {gate}: 주행 조건 확인 중'}
        if show_resolved:
            least = '최소 ' if last_closed.get('before_start') else ''
            return {'level': 'ok', 'title': '✅ 정상 복구', 'text': f'복구 소요 {least}{last_closed["duration_s"]}초'}
        if mission.get('state') == 'COMPLETE':
            return {'level': 'ok', 'title': '✅ 임무 완료', 'text': mission_label(mission)}
        return {'level': 'ok', 'title': f'\U0001F7E2 {rid} 정상 운행', 'text': ''}


def mission_label(m):
    if not m or m.get('total') is None:
        return '임무 정보 없음'
    state = m.get('state')
    if state == 'IDLE':
        return '임무 대기'
    if state == 'COMPLETE':
        return f'배송 {m.get("reached")}/{m["total"]} 완료'
    return f'배송 {m.get("index")}/{m["total"]} ({m.get("name")})'
