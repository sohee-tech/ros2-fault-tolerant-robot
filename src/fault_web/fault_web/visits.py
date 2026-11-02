"""Which devices reached the monitor and how far: page request -> JavaScript ran -> WebSocket connected.

The PC cannot probe a phone, so this is what separates "the request never arrived" (network) from "it arrived but
did not render" (browser). Own addresses of this machine do not count. Only kept in memory, nothing touches the robot.
"""
import time

MAX_DEVICES = 10
STALL = 6.0     # s after a stage before the next one counts as failed


class Visits:
    def __init__(self):
        self.devices = {}

    def record(self, ip, stage, agent='', error=None, now=None):
        """stage: page | js | ws | ws_closed | error. Returns True when this device reached a new stage."""
        now = time.time() if now is None else now
        d = self.devices.setdefault(ip, {'ip': ip, 'page': None, 'js': None, 'ws': None, 'ws_open': 0, 'error': None,
                                         'agent': '', 'last': now})
        d['last'] = now
        if agent:
            d['agent'] = agent[:120]
        new = False
        if stage in ('page', 'js', 'ws'):
            new = d[stage] is None
            d[stage] = now
            d['ws_open'] += stage == 'ws'
            if stage == 'page':     # a reload restarts the sequence
                d['js'] = None
                d['error'] = None
        elif stage == 'ws_closed':
            d['ws_open'] = max(0, d['ws_open'] - 1)
        elif stage == 'error':
            new = d['error'] is None
            d['error'] = (error or 'unknown')[:200]
        if len(self.devices) > MAX_DEVICES:
            oldest = min(self.devices.values(), key=lambda x: x['last'])
            del self.devices[oldest['ip']]
        return new

    def view(self, now=None):
        now = time.time() if now is None else now
        return {'diagnosis': self.diagnose(now),
                'devices': sorted(self.devices.values(), key=lambda x: -x['last'])}

    def diagnose(self, now):
        """Result for the newest device that matters: a device connected right now wins over an old failure."""
        if not self.devices:
            return {'level': 'none', 'title': '아직 휴대폰 접속 기록이 없습니다',
                    'text': '스캔 후 이 표시가 그대로라면 휴대폰 요청이 PC에 도착하지 못한 것입니다. '
                            '브라우저의 흰 화면이나 접속 오류도 이 경우에 나타납니다. '
                            '휴대폰이 LTE/5G이거나 다른 Wi-Fi에 있는지 확인하고, 같은 Wi-Fi인데도 그렇다면 네트워크의 기기 간 통신 차단을 의심하세요. '
                            '이때는 휴대폰 핫스팟을 사용하세요.'}
        connected = [d for d in self.devices.values() if d['ws_open'] > 0]
        if connected:
            return {'level': 'ok', 'title': f'접속 성공: 현재 {len(connected)}대 연결됨',
                    'text': '휴대폰에 대시보드가 실시간으로 표시되고 있습니다.'}
        d = max(self.devices.values(), key=lambda x: x['last'])
        if d['error']:
            return {'level': 'bad', 'title': f'{d["ip"]}: 페이지 오류', 'text': f'브라우저 오류: {d["error"]}'}
        if d['js'] is None:
            if now - d['page'] < STALL:
                return {'level': 'wait', 'title': f'{d["ip"]}: 페이지 요청 도착, 확인 중', 'text': '네트워크는 통과했습니다.'}
            return {'level': 'bad', 'title': f'{d["ip"]}: 페이지 요청은 왔지만 실행되지 않음',
                    'text': '네트워크는 정상입니다. 앱 내장 스캐너 브라우저 문제일 수 있으니 주소를 Chrome 또는 Safari에서 직접 여세요.'}
        if d['ws'] is None:
            if now - d['js'] < STALL:
                return {'level': 'wait', 'title': f'{d["ip"]}: 페이지 실행됨, 실시간 연결 중', 'text': ''}
            return {'level': 'bad', 'title': f'{d["ip"]}: 실시간 연결(WebSocket) 실패',
                    'text': '페이지는 열렸지만 WebSocket이 막혀 있습니다. VPN, 프록시, 데이터 절약 모드를 끄고 다시 시도하세요.'}
        return {'level': 'wait', 'title': f'{d["ip"]}: 연결이 끊어짐', 'text': '휴대폰 화면이 켜져 있으면 자동으로 다시 연결합니다.'}
