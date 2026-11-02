"""Read-only web monitor: FastAPI + WebSocket, separate process from the robot system.

Pages:  /  mobile dashboard   /qr  PC page with the QR code
API:    GET /api/state, /api/network, /api/beacon, /qr.png, WebSocket /ws. No endpoint changes anything on the robot.
Run:    ./run_web_monitor.sh [--port 8080] [--public-host ADDR]
"""
import argparse
import asyncio
import json
import os
import time
from contextlib import asynccontextmanager

import uvicorn
import yaml
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse

from fault_web import network, ros_bridge
from fault_web.alert_engine import AlertEngine
from fault_web.visits import Visits

TICK = 0.2          # s between status evaluations
HEARTBEAT = 1.0     # s between pushes when nothing changed
SEND_TIMEOUT = 5.0  # a client that does not take data for this long is dropped
NO_CACHE = {'Cache-Control': 'no-store'}


class Hub:
    """Each client has a one-slot queue holding only the newest state, so a slow phone cannot delay anything."""

    def __init__(self):
        self.clients = set()

    def add(self):
        q = asyncio.Queue(maxsize=1)
        self.clients.add(q)
        return q

    def remove(self, q):
        self.clients.discard(q)

    def publish(self, text):
        for q in self.clients:
            if q.full():
                q.get_nowait()
            q.put_nowait(text)


def fingerprint(view):
    """Everything whose change must reach the phones at once (excludes ages and clocks)."""
    return json.dumps([view['banner'], view['system_state'], view['subsystems'], view['response'], view['recovery'],
                       view['safe_resume']['state'], [r['check'] for r in view['safe_resume']['reasons']],
                       view['mission']['state'], view['mission']['text'], view['action'], view['resolved'],
                       view['position']['valid'], view['position']['zone'],
                       view['alerts'][0]['id'] if view['alerts'] else 0], ensure_ascii=False)


def create_app(args):
    share = get_package_share_directory('fault_web')
    static = os.path.join(share, 'static')
    with open(os.path.join(share, 'config', 'warehouse_zones.yaml'), encoding='utf-8') as fp:
        zones = yaml.safe_load(fp)['zones']
    engine = AlertEngine(zones=zones, robot_id=args.robot_id)
    state_file = os.path.expanduser(args.state_file)
    if os.path.exists(state_file):
        try:
            with open(state_file, encoding='utf-8') as fp:
                engine.load(json.load(fp))
        except (OSError, ValueError) as e:
            print(f'[web_monitor] history not loaded: {e}', flush=True)
    hub = Hub()
    visits = Visits()
    stage_text = {'page': '페이지 요청 도착', 'js': '페이지 실행됨', 'ws': '실시간 연결 성공', 'error': '브라우저 오류'}

    def visit(client, stage, agent='', error=None):
        """Device diagnostics; own addresses do not count, so testing on the PC cannot look like a phone success."""
        ip = client.host if client else None
        if ip is None or (ip in shared['net']['own_ips'] and not args.count_local_visits):
            return
        if visits.record(ip, stage, agent, error) and stage in stage_text:
            print(f'[web_monitor] 기기 {ip}: {stage_text[stage]}' + (f' ({error})' if error else ''), flush=True)

    shared = {'view': None, 'text': '{}', 'seq': 0, 'net': network.describe(args.port, args.public_host)}

    def save(data):
        os.makedirs(os.path.dirname(state_file), exist_ok=True)
        tmp = state_file + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as fp:
            json.dump(data, fp, ensure_ascii=False)
        os.replace(tmp, state_file)

    def announce(net):
        if net['ok']:
            print(f'\n[web_monitor] 휴대폰 접속 주소: {net["url"]}\n{network.qr_ascii(net["url"])}', flush=True)
        else:
            print('[web_monitor] 사용 가능한 네트워크 주소가 없습니다. Wi-Fi 또는 핫스팟에 연결하세요.', flush=True)

    async def loop(node):
        last_fp, last_push, last_net = None, 0.0, 0.0
        while True:
            now = time.time()
            view = engine.update(node.snapshot(), now)
            if now - last_net >= 3.0:
                last_net = now
                net = await asyncio.to_thread(network.describe, args.port, args.public_host)
                if net['urls'] != shared['net']['urls']:
                    shared['net'] = net
                    announce(net)
            shared['view'] = view
            fp = fingerprint(view)
            if fp != last_fp or now - last_push >= HEARTBEAT:
                last_fp, last_push = fp, now
                shared['seq'] += 1
                view = dict(view, type='state', seq=shared['seq'], viewers=len(hub.clients), url=shared['net']['url'])
                shared['text'] = json.dumps(view, ensure_ascii=False)
                hub.publish(shared['text'])
            if engine.dirty:
                engine.dirty = False
                try:
                    await asyncio.to_thread(save, engine.dump())
                except OSError as e:
                    print(f'[web_monitor] history not saved: {e}', flush=True)
            await asyncio.sleep(TICK)

    @asynccontextmanager
    async def lifespan(_app):
        node, stop = ros_bridge.start()
        announce(shared['net'])
        task = asyncio.create_task(loop(node))
        try:
            yield
        finally:
            task.cancel()
            stop()

    app = FastAPI(title='AMR monitor', docs_url=None, redoc_url=None, openapi_url=None, lifespan=lifespan)

    @app.get('/')
    async def index(request: Request):
        visit(request.client, 'page', request.headers.get('user-agent', ''))
        return FileResponse(os.path.join(static, 'index.html'), headers=NO_CACHE)

    @app.get('/qr')
    async def qr_page():
        return FileResponse(os.path.join(static, 'qr.html'), headers=NO_CACHE)

    @app.get('/api/state')
    async def state():
        return Response(shared['text'], media_type='application/json', headers=NO_CACHE)

    @app.get('/api/network')
    async def net():
        return JSONResponse({**shared['net'], 'visits': visits.view()}, headers=NO_CACHE)

    @app.get('/api/beacon')
    async def beacon(request: Request, stage: str = 'js', msg: str = ''):
        if stage in ('js', 'error'):    # diagnostics only, stored in memory
            visit(request.client, stage, request.headers.get('user-agent', ''), msg[:200] if stage == 'error' else None)
        return Response(status_code=204, headers=NO_CACHE)

    @app.get('/qr.png')
    async def qr_png(i: int = 0):
        urls = shared['net']['urls']
        if not 0 <= i < len(urls):      # only addresses of this machine, never a caller-supplied URL
            return JSONResponse({'error': 'no reachable address'}, status_code=404, headers=NO_CACHE)
        return Response(network.qr_png(urls[i]), media_type='image/png', headers=NO_CACHE)

    @app.websocket('/ws')
    async def ws(socket: WebSocket):
        await socket.accept()
        visit(socket.client, 'ws')
        q = hub.add()

        async def drain():        # read-only: anything a client sends is ignored
            while True:
                await socket.receive()

        reader = asyncio.create_task(drain())
        try:
            await socket.send_text(shared['text'])      # newest state immediately after (re)connecting
            while not reader.done():
                try:
                    text = await asyncio.wait_for(q.get(), HEARTBEAT * 2)
                except asyncio.TimeoutError:
                    continue
                await asyncio.wait_for(socket.send_text(text), SEND_TIMEOUT)
        except (WebSocketDisconnect, asyncio.TimeoutError, RuntimeError):
            pass
        finally:
            visit(socket.client, 'ws_closed')
            hub.remove(q)
            reader.cancel()

    return app


def main():
    ap = argparse.ArgumentParser(description='AMR read-only web monitor')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--bind', default='0.0.0.0')
    ap.add_argument('--public-host', default=None, help='address for the QR code (default: detected)')
    ap.add_argument('--robot-id', default='AMR-01')
    ap.add_argument('--state-file', default='~/capstone_fault_ws/logs/web_monitor_state.json')
    ap.add_argument('--count-local-visits', action='store_true', help='test only: count this machine as a device')
    args = ap.parse_args()
    uvicorn.run(create_app(args), host=args.bind, port=args.port, log_level='warning')


if __name__ == '__main__':
    main()
