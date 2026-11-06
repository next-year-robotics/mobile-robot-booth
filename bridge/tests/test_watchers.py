from booth_bridge.watchers import parse_pi


def test_parse_pi_flags():
    p = parse_pi("temp=58.3'C\nthrottled=0x50005\n1.52 1.20 0.98 2/345 12345\n4\n")
    assert p['ok'] and p['temp_c'] == 58.3 and p['load1'] == 1.52 and p['ncpu'] == 4
    assert p['undervoltage_now'] and p['throttled_now'] and p['undervoltage_seen']


def test_parse_pi_healthy_and_empty():
    p = parse_pi("temp=45.0'C\nthrottled=0x0\n0.10 0.20 0.30 1/200 999\n4\n")
    assert not p['undervoltage_now'] and not p['undervoltage_seen']
    assert parse_pi('')['ok'] is False


def test_parse_pi_stacks():
    units = ['follow', 'phone_bridge', 'mission', 'telemetry', 'pad']
    out = "temp=50.0'C\nthrottled=0x0\n0.10 0.20 0.30 1/200 999\n4\nunits:\nactive\nactive\nfailed\ninactive\nactivating\n"
    p = parse_pi(out, units)
    assert p['load1'] == 0.10 and p['ncpu'] == 4
    assert p['stacks'] == {'follow': 'active', 'phone_bridge': 'active', 'mission': 'failed',
                           'telemetry': 'inactive', 'pad': 'activating'}
    # 유닛 줄이 잘리면 모름으로
    assert parse_pi(out.split('units:')[0] + 'units:\nactive\n', units)['stacks']['pad'] == 'unknown'
