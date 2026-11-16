import asyncio
import json

from booth_bridge.server import BoothServer


class WS:
    def __init__(self): self.sent = []
    async def send(self, data): self.sent.append(json.loads(data))


def run(server, msg):
    ws = WS()
    asyncio.run(server.arm_command(ws, msg))
    return ws.sent[0]


def test_reply_carries_id_and_result():
    server = object.__new__(BoothServer)
    class Arm:
        def command(self, body): return {'ok': False, 'error': '지금은 들고 있는 물건이 없어요.'} if body['cmd'] == 'give' else {'ok': True, 'started': True}
    server.arm = Arm()
    assert run(server, {'cmd': 'arm', 'id': 7, 'msg': {'cmd': 'fetch', 'name': '귤'}}) == {'type': 'reply', 'id': 7, 'ok': True, 'started': True}
    assert run(server, {'cmd': 'arm', 'id': 8, 'msg': {'cmd': 'give'}})['error'].startswith('지금은')


def test_no_arm_or_bad_body_is_refused():
    server = object.__new__(BoothServer)
    server.arm = None
    assert run(server, {'cmd': 'arm', 'id': 1, 'msg': {'cmd': 'give'}})['ok'] is False
    class Arm:
        def command(self, body): raise AssertionError('불려서는 안 된다')
    server.arm = Arm()
    assert run(server, {'cmd': 'arm', 'id': 2, 'msg': 'give'})['ok'] is False


def test_sim_bridge_never_commands_a_real_arm():
    from booth_bridge.arm_link import ArmLink
    link = ArmLink({'url': 'http://127.0.0.1:9'}, state=None, sim=True)
    link.up, link.real = True, True
    assert '진짜 팔' in link.command({'cmd': 'fetch', 'name': '귤'})['error']
    link.up, link.real = False, False               # 아직 무엇이 떠 있는지 모른다
    assert '진짜 팔' in link.command({'cmd': 'throw'})['error']
