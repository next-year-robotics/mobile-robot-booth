import uuid

from booth_bridge.ops import Ops
from booth_bridge.state import BoothState
from test_state import CFG


class FakeNode:
    def __init__(self):
        self.sent = []

    def can_trip(self):
        return True

    def send_trip(self, **kw):
        self.sent.append(kw)


def make():
    state = BoothState(CFG)
    node = FakeNode()
    return Ops(CFG, state, node), state, node


def test_start_sends_uuid_trip_with_booth_places():
    ops, state, node = make()
    ops.handle('start')
    kw = node.sent[-1]
    assert kw['start'] is True and str(uuid.UUID(kw['trip_id'])) == kw['trip_id']
    assert (kw['from_place'], kw['to_place']) == ('place_home', 'place_senior_center')
    assert state.snapshot(0.0)['ops']['text'] == '출발 보냄'


def test_stop_cancels_current_trip_or_empty():
    ops, state, node = make()
    ops.handle('stop')
    assert node.sent[-1] == {'start': False, 'trip_id': ''}
    state.on_app_status(1.0, {'mission_state': 2, 'tracking_state': 2, 'trip_id': 'abc', 'last_error': ''})
    ops.handle('stop')
    assert node.sent[-1] == {'start': False, 'trip_id': 'abc'}


def test_no_ros_reports_instead_of_crashing():
    ops, state, _ = make()
    ops.node = None
    ops.handle('start')
    assert state.snapshot(0.0)['ops']['level'] == 'warn'
