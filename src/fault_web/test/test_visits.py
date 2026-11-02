"""Device diagnostics (no ROS, no web)."""
from fault_web.visits import Visits


def level(v, now):
    return v.diagnose(now)['level']


def test_no_visit_points_to_the_network():
    d = Visits().diagnose(0.0)
    assert d['level'] == 'none' and '핫스팟' in d['text']


def test_stages_in_order():
    v = Visits()
    v.record('10.0.0.9', 'page', now=0.0)
    assert level(v, 1.0) == 'wait'
    assert level(v, 7.0) == 'bad' and '내장' in v.diagnose(7.0)['text']     # page without JS: in-app browser
    v.record('10.0.0.9', 'js', now=7.0)
    assert level(v, 8.0) == 'wait'
    assert level(v, 14.0) == 'bad' and 'WebSocket' in v.diagnose(14.0)['title']
    v.record('10.0.0.9', 'ws', now=14.0)
    assert level(v, 15.0) == 'ok'
    v.record('10.0.0.9', 'ws_closed', now=16.0)
    assert level(v, 17.0) == 'wait'


def test_connected_device_wins_over_another_failure():
    v = Visits()
    v.record('10.0.0.1', 'page', now=0.0)
    v.record('10.0.0.2', 'page', now=1.0)
    v.record('10.0.0.2', 'js', now=1.0)
    v.record('10.0.0.2', 'ws', now=1.0)
    assert level(v, 20.0) == 'ok'


def test_browser_error_is_shown_and_reload_clears_it():
    v = Visits()
    v.record('10.0.0.1', 'page', now=0.0)
    assert v.record('10.0.0.1', 'error', error='boom', now=1.0)
    assert level(v, 2.0) == 'bad' and 'boom' in v.diagnose(2.0)['text']
    v.record('10.0.0.1', 'page', now=3.0)
    assert level(v, 3.5) == 'wait'


def test_device_list_is_bounded():
    v = Visits()
    for i in range(30):
        v.record(f'10.0.0.{i}', 'page', now=float(i))
    assert len(v.devices) == 10
