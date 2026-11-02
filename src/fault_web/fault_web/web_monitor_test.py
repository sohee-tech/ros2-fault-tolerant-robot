"""Web monitor integration test against the real system (Gazebo + fault system + Nav2 + Safe Resume Gate).

Requires nav2_warehouse.launch.py (run_web_monitor_test.sh). The test starts the web server itself.
  Q  QR code decodes to a LAN URL that serves the page; read-only server
  S1 LiDAR dropout: alert, cause, stop, Safe Resume reason, approval, mission resumes, recovered message
  S2 Nav2 process crash: recovery stages visible, recovered message
  S3 recovery impossible: safe stop stays, administrator action required, Safe Resume reason
  S4 web failure: server hung and killed -> robot control unaffected, phones show the disconnect;
     restart -> same browser re-syncs by itself without a reload
Views are checked on N WebSocket clients and on real headless Firefox pages (phone viewport, never reloaded).
Robot motion is checked independently of the monitor: /cmd_vel and Gazebo /odom_raw.
Writes results/web_monitor_summary.json, screenshots in results/web_monitor/.
"""
import asyncio
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime

import io

import numpy as np
import rclpy
import websockets
import zxingcpp
from PIL import Image
from selenium import webdriver
from selenium.webdriver.firefox.options import Options
from selenium.webdriver.firefox.service import Service
from std_srvs.srv import Trigger

from fault_bringup.nav2_demo import Nav2Demo, RESULTS

PORT = 8080
SHOTS = os.path.join(RESULTS, 'web_monitor')
STATE_FILE = os.path.join(SHOTS, 'test_state.json')
N_CLIENTS = 6


class WsClients(threading.Thread):
    """N independent WebSocket clients recording (arrival time, state), reconnecting like the page does."""

    def __init__(self, url, n):
        super().__init__(daemon=True)
        self.url, self.n = url, n
        self.latest = {}
        self.history = []
        self.lock = threading.Lock()
        self.connects = [0] * n

    def run(self):
        asyncio.run(self.main())

    async def main(self):
        await asyncio.gather(*(self.client(i) for i in range(self.n)))

    async def client(self, i):
        while True:
            try:
                async with websockets.connect(self.url, open_timeout=2) as ws:
                    self.connects[i] += 1
                    async for raw in ws:
                        v = json.loads(raw)
                        if v.get('type') != 'state':
                            continue
                        with self.lock:
                            self.latest[i] = (time.time(), v)
                            if i == 0:
                                self.history.append((time.time(), v))
            except (OSError, asyncio.TimeoutError, websockets.WebSocketException):
                with self.lock:
                    self.latest.pop(i, None)
                await asyncio.sleep(0.5)

    def view(self, i=0):
        with self.lock:
            item = self.latest.get(i)
        return item[1] if item else {}

    def all_views(self):
        with self.lock:
            return [v for _, v in self.latest.values()]

    def mark(self):
        with self.lock:
            return len(self.history)

    def since(self, mark):
        with self.lock:
            return self.history[mark:]


class WebMonitorTest(Nav2Demo):
    def __init__(self):
        super().__init__()
        for n in ('nav2_fault/controller_crash', 'nav2_fault/recovery_block_enable', 'nav2_fault/recovery_block_disable',
                  'nav2_fault/controller_deactivate'):
            self.cli[n] = self.create_client(Trigger, n)
        self.server = None
        self.browsers = []
        self.ws = None
        self.lan = None
        self.table = {}

    # --- server / browsers ---
    def start_server(self):
        env = dict(os.environ)
        self.server = subprocess.Popen([sys.executable, '-m', 'fault_web.server', '--port', str(PORT),
                                        '--state-file', STATE_FILE], env=env,
                                       stdout=open(os.path.join(SHOTS, 'server.log'), 'a'), stderr=subprocess.STDOUT)

    def stop_server(self, sig=signal.SIGKILL):
        if self.server and self.server.poll() is None:
            self.server.send_signal(signal.SIGCONT)
            self.server.send_signal(sig)
            self.server.wait(timeout=10)

    def new_browser(self):
        os.environ['TMPDIR'] = os.path.expanduser('~/snap/firefox/common/tmp')
        os.makedirs(os.environ['TMPDIR'], exist_ok=True)
        o = Options()
        o.add_argument('-headless')
        o.add_argument('--width=390')
        o.add_argument('--height=844')
        o.binary_location = '/snap/firefox/current/usr/lib/firefox/firefox'
        return webdriver.Firefox(options=o, service=Service('/snap/bin/geckodriver'))

    def dom(self, b, element):
        return b.find_element('id', element).text

    def shot(self, name):
        if self.browsers:
            self.browsers[0].save_screenshot(os.path.join(SHOTS, f'{name}.png'))

    def ws_until(self, cond, timeout):
        def safe():
            v = self.ws.view()
            try:
                return bool(v) and cond(v)
            except (KeyError, TypeError):
                return False
        return self.wait_until(safe, timeout)

    def all_browsers(self, cond, timeout=4.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            try:
                if all(cond(b) for b in self.browsers):
                    return True
            except Exception:       # page mid-update
                pass
            time.sleep(0.1)
        return False

    def stopped(self, label, dur=1.5):
        """Independent of the monitor: final /cmd_vel and Gazebo ground truth."""
        t_a = self.now()
        self.spin_for(dur)
        out = self.window(self.out, t_a, self.now())
        _, d = self.speed(t_a, self.now())
        self.check(f'{label}: robot stopped (/cmd_vel 0, ground truth)', bool(out) and max(map(abs, out)) == 0.0 and d < 0.01,
                   f'(n={len(out)}, moved {d * 1000:.1f} mm)')

    def driving(self):
        self.wait_until(lambda: self.moving(0.08) and self.status.get('state') == 'RUNNING', 40.0)
        self.spin_for(1.5)

    def alerts(self, kind, incident=None):
        return [a for a in self.ws.view().get('alerts', []) if a['kind'] == kind and (incident is None or a['incident'] == incident)]

    def expect_recovered(self, label, t0):
        ok = self.ws_until(lambda v: v['resolved']['show'], 45.0)
        v = self.ws.view()
        self.check(f'{label}: recovered message pushed', ok, f'({v.get("banner", {}).get("title")}, {v.get("resolved", {}).get("duration_s")} s)')
        self.check(f'{label}: mission RUNNING and Safe Resume APPROVED when recovered',
                   v.get('mission', {}).get('state') in ('RUNNING', 'COMPLETE') and v.get('safe_resume', {}).get('state') == 'APPROVED',
                   f'({v.get("mission", {}).get("text")}, {v.get("safe_resume", {}).get("state")})')
        ok = self.all_browsers(lambda b: '정상 복구' in self.dom(b, 'b-title') and self.dom(b, 'gate') == 'APPROVED'
                               and self.dom(b, 'r-time') != '')
        self.check(f'{label}: every browser shows it without a reload', ok,
                   f'({[self.dom(b, "b-title") for b in self.browsers]})')
        self.check(f'{label}: robot moving again (ground truth)', self.wait_until(lambda: self.moving(0.05), 20.0))
        duration = v.get('resolved', {}).get('duration_s')
        wall = time.time() - t0
        self.check(f'{label}: reported duration plausible', duration is not None and 0 < duration <= wall + 1,
                   f'({duration} s, {wall:.1f} s since injection)')
        self.shot(f'{label}_recovered')
        return duration

    @staticmethod
    def decode_qr(png):
        codes = zxingcpp.read_barcodes(np.asarray(Image.open(io.BytesIO(png)).convert('RGB')))
        return codes[0].text if codes else None

    # --- tests ---
    def test_qr(self):
        self.phase('Q: QR code and access')
        for _ in range(40):
            try:
                net = json.load(urllib.request.urlopen(f'http://127.0.0.1:{PORT}/api/network', timeout=1))
                break
            except OSError:
                self.spin_for(0.5)
        else:
            self.check('Q: server started', False)
            return False
        self.check('Q: address found', net['ok'], f'({net["urls"]})')
        png = urllib.request.urlopen(f'http://127.0.0.1:{PORT}/qr.png', timeout=2).read()
        data = self.decode_qr(png)
        self.check('Q: QR code decodes to the announced URL', data == net['url'], f'({data})')
        if not self.check('Q: not localhost', bool(re.match(r'http://\d+\.\d+\.\d+\.\d+:\d+/$', data or ''))
                          and '127.' not in data):
            return False
        self.lan = data
        page = urllib.request.urlopen(data, timeout=3).read().decode()
        self.check('Q: URL from the QR code serves the page on the LAN address', 'AMR MONITOR' in page)
        self.check('Q: page needs no external resources', not re.search(r'(src|href)=["\']https?:', page))
        for path in ('api/state', 'ws', 'qr.png'):
            req = urllib.request.Request(data + path, data=b'{}', method='POST')
            try:
                urllib.request.urlopen(req, timeout=2)
                code = 200
            except urllib.error.HTTPError as e:
                code = e.code
            self.check(f'Q: POST /{path} refused', code in (404, 405), f'({code})')
        self.spin_for(1.0)
        pubs = {t for t, _ in self.get_publisher_names_and_types_by_node('web_monitor_bridge', '/')}
        clients = {t for t, _ in self.get_client_names_and_types_by_node('web_monitor_bridge', '/')}
        self.check('Q: bridge publishes nothing to the robot', pubs <= {'/rosout', '/parameter_events'}, f'({sorted(pubs)})')
        self.check('Q: bridge calls no robot service', not [c for c in clients if not c.startswith('/web_monitor_bridge')], f'({sorted(clients)})')
        self.ws = WsClients(data.replace('http://', 'ws://') + 'ws', N_CLIENTS)
        self.ws.start()
        self.browsers = [self.new_browser() for _ in range(2)]
        pc = self.browsers[0]
        pc.set_window_size(1280, 900)
        pc.get(f'http://127.0.0.1:{PORT}/qr')    # the PC page opened on localhost must still show the LAN address
        self.spin_for(2.0)
        shown = pc.get_screenshot_as_png()
        self.check('Q: QR code displayed on the PC page decodes to the LAN URL (screenshot)', self.decode_qr(shown) == data,
                   f'({self.decode_qr(shown)})')
        pc.save_screenshot(os.path.join(SHOTS, 'qr_page.png'))
        pc.set_window_size(390, 844)
        for b in self.browsers:
            b.get(data)
        ok = self.all_browsers(lambda b: self.dom(b, 'conn') == 'CONNECTED', 10.0)
        self.check('Q: two real browsers and WebSocket clients connected at once',
                   ok and self.wait_until(lambda: len(self.ws.all_views()) == N_CLIENTS, 5.0),
                   f'({len(self.ws.all_views())}+{len(self.browsers)} clients)')
        self.check('Q: mobile layout has no horizontal overflow',
                   self.browsers[0].execute_script('return document.documentElement.scrollWidth<=window.innerWidth'))
        return True

    def test_normal(self):
        self.phase('Normal driving')
        self.svc('mission/start')
        ok = self.ws_until(lambda v: v['system_state'] == 'NORMAL' and v['safe_resume']['state'] == 'APPROVED'
                           and v['mission']['state'] == 'RUNNING', 60.0)
        self.check('N: real ROS state reaches the web (NORMAL, APPROVED, RUNNING)', ok, f'({self.ws.view().get("mission", {}).get("text")})')
        self.driving()
        v = self.ws.view()
        self.check('N: position and zone from AMCL', v['position']['valid'] and v['position']['zone'], f'({v["position"]["text"]})')
        self.check('N: browser shows normal', self.all_browsers(lambda b: '정상 운행' in self.dom(b, 'b-title')))
        self.shot('normal')

    def s1_lidar(self):
        self.phase('S1 LiDAR dropout')
        self.driving()
        m = self.ws.mark()
        pos_before = self.ws.view()['position']['zone']
        t_wall, t0 = time.time(), self.svc('lidar_fault/enable')
        self.check('S1: robot system CRITICAL', self.wait_until(lambda: self.state() == 'CRITICAL', 6.0))
        t_crit = time.time()
        ok = self.ws_until(lambda v: v['system_state'] == 'CRITICAL', 2.0)
        latency = time.time() - t_crit
        self.check('S1: CRITICAL pushed within 1 s', ok and latency <= 1.0, f'({latency * 1000:.0f} ms)')
        ok = self.all_browsers(lambda b: '긴급 장애' in self.dom(b, 'b-title') and self.dom(b, 'system') == 'CRITICAL', 2.0)
        self.check('S1: emergency banner in every browser', ok, f'({time.time() - t_crit:.2f} s after CRITICAL)')
        self.ws_until(lambda v: v['safe_stop'], 2.0)
        v = self.ws.view()
        self.check('S1: cause = LiDAR 데이터 수신 중단 (no hardware claim)', v['fault']['type'] == 'LiDAR 데이터 수신 중단', f'({v["fault"]["type"]})')
        self.check('S1: location and response shown', v['fault']['location'] and v['response'] == '안전 정지',
                   f'({v["fault"]["location"]}, before {pos_before}, {v["response"]})')
        self.check('S1: emergency alert has the required fields', all(k in self.alerts('critical')[0] for k in (
            'robot_id', 'time', 'fault_type', 'system', 'position', 'safe_stop', 'recovery', 'safe_resume', 'action_required')),
            f'({self.alerts("critical")[0]["text"]})')
        self.stopped('S1')
        ok = self.ws_until(lambda v: any(r['check'] == 'lidar' for r in v['safe_resume']['reasons'])
                           and v['safe_resume']['state'] != 'APPROVED', 3.0)
        self.check('S1: Safe Resume reason shown', ok, f'({[r["text"] for r in self.ws.view()["safe_resume"]["reasons"]]})')
        ok = self.ws_until(lambda v: not v['position']['valid'], 8.0)
        self.check('S1: pose not presented as current once stale', ok and '마지막 확인 위치' in self.ws.view()['position']['text'],
                   f'({self.ws.view()["position"]["text"]})')
        fg, bg = self.browsers[0].execute_script(
            'const c=getComputedStyle(document.getElementById("system"));return [c.color,c.backgroundColor]')
        self.check('S1: state chip readable (white on red)', fg == 'rgb(255, 255, 255)' and bg == 'rgb(214, 43, 43)', f'({fg} on {bg})')
        self.shot('S1_critical')
        n = self.alerts('critical')
        incident = n[0]['incident']
        self.spin_for(2.0)
        self.check('S1: one emergency alert for one fault', len(self.alerts('critical', incident)) == 1 and all(
            len({a['incident'] for a in self.alerts('critical')}) == len(self.alerts('critical')) for _ in [0]),
            f'({len(self.alerts("critical", incident))})')
        self.svc('lidar_fault/disable')
        duration = self.expect_recovered('S1', t_wall)
        states = [v['safe_resume']['state'] for _, v in self.ws.since(m)]
        self.check('S1: verification visible on the way (VERIFYING -> APPROVED)', 'VERIFYING' in states and states[-1] == 'APPROVED',
                   f'({sorted(set(states))})')
        self.check('S1: all clients got the same final state', len({v['safe_resume']['state'] for v in self.ws.all_views()}) == 1)
        self.table['S1'] = {'push_latency_ms': round(latency * 1000), 'reported_recovery_s': duration}

    def s2_nav2(self):
        self.phase('S2 Nav2 process crash')
        self.driving()
        m = self.ws.mark()
        t_wall = time.time()
        self.svc('nav2_fault/controller_crash')
        ok = self.ws_until(lambda v: v['recovery']['active'], 8.0)
        self.check('S2: automatic recovery visible', ok, f'({self.ws.view()["recovery"]["text"]})')
        ok = self.ws_until(lambda v: v['system_state'] == 'CRITICAL', 8.0)
        v = self.ws.view()
        self.check('S2: cause names only the Nav2 server, not the previous fault',
                   ok and 'Nav2' in (v['fault']['type'] or '') and 'LiDAR' not in v['fault']['type'],
                   f'({v["fault"]["type"]}: {[c["detail"] for c in v["fault"]["causes"]]})')
        ok = self.all_browsers(lambda b: 'Nav2' in self.dom(b, 'rec') or '정상화' in self.dom(b, 'rec'), 4.0)
        self.check('S2: recovery stage shown in the browsers', ok, f'({self.dom(self.browsers[0], "rec")})')
        self.shot('S2_recovering')
        duration = self.expect_recovered('S2', t_wall)
        stages = []
        for _, x in self.ws.since(m):
            if x['recovery']['state'] and (not stages or stages[-1] != x['recovery']['state']):
                stages.append(x['recovery']['state'])
        self.check('S2: recovery stages pushed in order', 'RECOVERING' in stages and 'RECOVERED' in stages
                   and stages.index('RECOVERING') < stages.index('RECOVERED'), f'({stages})')
        self.table['S2'] = {'reported_recovery_s': duration, 'stages': stages}

    def s3_failed(self):
        self.phase('S3 recovery impossible')
        self.driving()
        t_wall = time.time()
        self.svc('nav2_fault/recovery_block_enable')
        self.svc('nav2_fault/controller_deactivate')
        ok = self.ws_until(lambda v: v['recovery']['state'] == 'FAILED', 90.0)
        self.check('S3: recovery FAILED shown', ok, f'({time.time() - t_wall:.1f} s)')
        ok = self.ws_until(lambda v: v['action']['required'] and v['safe_resume']['state'] == 'BLOCKED', 6.0)
        v = self.ws.view()
        self.check('S3: administrator action required', ok and '관리자 조치' in v['banner']['title'], f'({v["action"]["reasons"]})')
        self.check('S3: Safe Resume reason names Nav2', any(r['check'] == 'nav2' for r in v['safe_resume']['reasons']),
                   f'({[r["text"] for r in v["safe_resume"]["reasons"]]})')
        self.check('S3: failure alert once', len(self.alerts('recovery_failed', v['fault']['id'])) == 1)
        ok = self.all_browsers(lambda b: '관리자 조치' in self.dom(b, 'b-title') and self.dom(b, 'action') == '필요'
                               and self.dom(b, 'gate') == 'BLOCKED', 3.0)
        self.check('S3: every browser shows it', ok)
        self.stopped('S3', 3.0)
        self.check('S3: still CRITICAL (safe stop stays)', self.state() == 'CRITICAL')
        self.shot('S3_action_required')
        self.svc('nav2_fault/recovery_block_disable')
        self.expect_recovered('S3 release', t_wall)     # the duration counts from the fault, not from the release

    def s4_web(self):
        self.phase('S4 web failure: server hung')
        self.driving()
        self.server.send_signal(signal.SIGSTOP)
        t_a = self.now()
        self.spin_for(3.0)
        _, d = self.speed(t_a, self.now())
        self.check('S4: robot keeps driving while the server is hung', d > 0.2, f'({d:.2f} m in 3 s)')
        ok = self.all_browsers(lambda b: self.dom(b, 'conn') == 'DISCONNECTED', 5.0)
        self.check('S4: hung server detected by the page watchdog', ok)
        self.server.send_signal(signal.SIGCONT)
        ok = self.all_browsers(lambda b: self.dom(b, 'conn') == 'CONNECTED', 10.0)
        self.check('S4: page reconnects after the hang', ok)

        self.phase('S4 web failure: server killed')
        self.driving()
        n_conn = list(self.ws.connects)
        self.stop_server()
        t_kill = time.time()
        ok = self.all_browsers(lambda b: self.dom(b, 'conn') == 'DISCONNECTED' and '오래된 정보' in self.dom(b, 'offline'), 5.0)
        self.check('S4: phones show the disconnect, data marked old', ok,
                   f'({time.time() - t_kill:.2f} s: {self.dom(self.browsers[0], "offline")})')
        self.check('S4: last state no longer displayed as live', self.browsers[0].execute_script(
            'return document.body.classList.contains("offline")'))
        self.shot('S4_disconnected')
        t_a = self.now()
        self.spin_for(2.0)
        _, d = self.speed(t_a, self.now())
        self.check('S4: robot keeps driving without the server', d > 0.15 and self.state() == 'NORMAL', f'({d:.2f} m, {self.state()})')
        t0 = self.svc('lidar_fault/enable')
        ok = self.wait_until(lambda: self.state() == 'CRITICAL', 6.0)
        self.check('S4: fault still detected without the server', ok, f'({self.now() - t0:.2f} s)')
        self.spin_for(0.5)
        self.stopped('S4 no server')
        self.start_server()
        ok = self.all_browsers(lambda b: self.dom(b, 'conn') == 'CONNECTED' and self.dom(b, 'system') == 'CRITICAL', 20.0)
        self.check('S4: restart -> same pages re-sync to the current CRITICAL, no reload', ok,
                   f'({self.dom(self.browsers[0], "b-title")}, retries {[c - o for c, o in zip(self.ws.connects, n_conn)]})')
        self.check('S4: WebSocket clients reconnected', self.wait_until(
            lambda: len(self.ws.all_views()) == N_CLIENTS and all(c > o for c, o in zip(self.ws.connects, n_conn)), 8.0))
        v = self.ws.view()
        self.check('S4: restarted server knows the cause', v.get('fault', {}).get('type') == 'LiDAR 데이터 수신 중단', f'({v.get("fault", {}).get("type")})')
        self.check('S4: fault during the outage has no false start time', '이전' in (v.get('fault', {}).get('opened') or ''),
                   f'({v.get("fault", {}).get("opened")})')
        self.shot('S4_resynced')
        t_wall = time.time()
        self.svc('lidar_fault/disable')
        self.expect_recovered('S4', t_wall)
        self.check('S4: recovery time reported as a lower bound', self.ws.view()['resolved'].get('at_least') and
                   '최소' in self.dom(self.browsers[0], 'r-time'), f'({self.dom(self.browsers[0], "r-time")})')
        self.check('S4: history kept across the restart', len(self.ws.view().get('incidents', [])) >= 4,
                   f'({len(self.ws.view().get("incidents", []))} incidents)')

    def run(self):
        os.makedirs(SHOTS, exist_ok=True)
        if os.path.exists(STATE_FILE):
            os.remove(STATE_FILE)
        self.start_server()
        ok = self.wait_until(lambda: self.status.get('mode') == 'NAV2' and self.state() == 'NORMAL'
                             and self.cli['mission/start'].service_is_ready(), 90.0)
        if not self.check('sim ready', ok) or not self.test_qr():
            return False
        self.test_normal()
        self.s1_lidar()
        self.s2_nav2()
        self.s3_failed()
        self.s4_web()
        self.phase('Web monitor test complete')
        return all(r[1] for r in self.results)

    def finish(self, passed):
        for b in self.browsers:
            try:
                b.quit()
            except Exception:
                pass
        self.stop_server(signal.SIGTERM)
        doc = {'generated': datetime.now().isoformat(timespec='seconds'), 'test_passed': passed,
               'checks_passed': sum(r[1] for r in self.results), 'checks_total': len(self.results),
               'failed_checks': [r[0] for r in self.results if not r[1]], 'url': self.lan, 'scenarios': self.table,
               'external_phone_tested': False}
        with open(os.path.join(RESULTS, 'web_monitor_summary.json'), 'w') as fp:
            json.dump(doc, fp, indent=2, ensure_ascii=False)


def main():
    rclpy.init()
    node = WebMonitorTest()
    passed = False
    try:
        passed = node.run()
    finally:
        node.finish(passed)
        node.destroy_node()
        rclpy.try_shutdown()
    print(f'\n==== WEB MONITOR TEST: {"PASSED" if passed else "FAILED"} '
          f'({sum(r[1] for r in node.results)}/{len(node.results)} checks) ====')
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
