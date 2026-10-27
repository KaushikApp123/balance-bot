import math
import struct

from balancebot import protocol as P


def test_crc_check_value():
    assert P.crc16_ccitt(b"123456789") == 0x29B1     # same vector as the C test


def test_struct_sizes_match_firmware():
    # sizes asserted with _Static_assert in firmware/core/inc/protocol.h
    assert P.TELEMETRY_FMT.size == 54
    assert P.GAINS_FMT.size == 40
    assert P.SET_GAINS_FMT.size == 13


def test_roundtrip_through_split_stream_with_garbage():
    tel = {k: 0.0 for k in P.TELEMETRY_FIELDS}
    tel.update(t_ms=1234, angle=math.radians(3), state=int(P.State.ARMED), faults=0x05)
    stream = b"\x00\xAA\x13" + P.encode_telemetry(tel) + b"\xAA" + P.cmd_set_gains(2, 1.5, 0.25, 3.0)
    parser = P.FrameParser()
    got = []
    for i in range(0, len(stream), 7):           # arbitrary BLE-ish packet size
        got += list(parser.feed(stream[i:i + 7]))
    assert [m for m, _ in got] == [P.Msg.TELEMETRY, P.Msg.SET_GAINS]
    t = P.decode_telemetry(got[0][1])
    assert t["t_ms"] == 1234 and t["faults"] == 0x05
    assert abs(t["angle"] - math.radians(3)) < 1e-6
    assert P.SET_GAINS_FMT.unpack(got[1][1]) == (2, 1.5, 0.25, 3.0)
    assert P.fault_names(0x05) == ["TILT", "BATT"]


def test_corrupt_frame_is_dropped_and_parser_recovers():
    good = P.cmd_arm(True)
    bad = bytearray(good)
    bad[4] ^= 0xFF
    parser = P.FrameParser()
    out = list(parser.feed(bytes(bad) + good))
    assert len(out) == 1 and out[0] == (P.Msg.ARM, b"\x01")
    assert parser.crc_errors == 1


def test_gains_roundtrip():
    g = P.Gains([100.0, 8.0, 0.05], [0.0, 100.0, 0.02], [2.0, 0.0, 0.0], 0.01)
    (msg, payload), = P.FrameParser().feed(P.encode_gains(g))
    back = P.decode_gains(payload)
    assert msg == P.Msg.GAINS
    assert [round(x, 4) for x in back.kp] == g.kp and abs(back.trim - 0.01) < 1e-6
