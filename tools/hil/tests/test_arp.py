"""Analysis helpers of hil.py arp (hil/arp.py), with synthetic frames and pcap data."""

import struct

import numpy as np

from hil import arp

BOARD = "60:95:ce:10:06:00"
NIC_A = "04:ab:18:14:26:8e"
NIC_B = "04:ab:18:6f:c5:cf"
NAMES = {NIC_A: "NIC A", NIC_B: "NIC B", BOARD: "board"}


def mac(s: str) -> bytes:
    return bytes(int(x, 16) for x in s.split(":"))


def ip(s: str) -> bytes:
    return bytes(int(x) for x in s.split("."))


def arp_frame(op, sha, spa, tha, tpa, dst="ff:ff:ff:ff:ff:ff"):
    body = struct.pack(">HHBBH", 1, 0x0800, 6, 4, op) + mac(sha) + ip(spa) + mac(tha) + ip(tpa)
    return mac(dst) + mac(sha) + struct.pack(">H", 0x0806) + body


def udp_frame(src, dst, ip_src, ip_dst, sport, dport, payload):
    udp = struct.pack(">HHHH", sport, dport, 8 + len(payload), 0) + payload
    hdr = struct.pack(">BBHHHBBH", 0x45, 0, 20 + len(udp), 0, 0, 64, 17, 0) + ip(ip_src) + ip(ip_dst)
    return mac(dst) + mac(src) + struct.pack(">H", 0x0800) + hdr + udp


def pcap(frames, nano=False, big=False):
    e = ">" if big else "<"
    magic = 0xA1B23C4D if nano else 0xA1B2C3D4
    out = struct.pack(e + "IHHiIII", magic, 2, 4, 0, 0, 65535, 1)
    for t, pkt in frames:
        sec = int(t)
        frac = round((t - sec) * (1e9 if nano else 1e6))
        out += struct.pack(e + "IIII", sec, frac, len(pkt), len(pkt)) + pkt
    return out


def osc(address: str) -> bytes:
    a = address.encode() + b"\0"
    return a + b"\0" * (-len(a) % 4) + b",\0\0\0"


def test_parse_pcap_arp_and_osc():
    data = pcap([
        (100.5, arp_frame(1, BOARD, "10.0.0.100", "00:00:00:00:00:00", "10.0.0.10")),
        (100.501, arp_frame(2, NIC_A, "10.0.0.10", BOARD, "10.0.0.100", dst=BOARD)),
        (100.6, udp_frame(BOARD, NIC_A, "10.0.0.100", "10.0.0.10", 50000, 50100, osc("/positionList"))),
    ])
    f = arp.parse_pcap(data, "en9")
    assert len(f) == 3
    assert f[0].t == 100.5 and f[0].arp_op == 1 and f[0].arp_tpa == "10.0.0.10" and f[0].src == BOARD
    assert f[1].arp_op == 2 and f[1].arp_sha == NIC_A and f[1].arp_tha == BOARD
    assert f[2].osc == "/positionList" and f[2].udp_dport == 50100 and f[2].iface == "en9"
    assert abs(f[2].t - 100.6) < 1e-6


def test_parse_pcap_nanosecond_big_endian_and_truncated():
    pkt = udp_frame(BOARD, NIC_A, "10.0.0.100", "10.0.0.10", 50000, 50100, osc("/version"))
    data = pcap([(5.000000123, pkt), (6.0, pkt)], nano=True, big=True)
    f = arp.parse_pcap(data[:-5])  # the last record is cut
    assert len(f) == 1 and abs(f[0].t - 5.000000123) < 1e-9 and f[0].osc == "/version"


def test_parse_ifconfig_and_transitions():
    text = ("en7: flags=8863<UP> mtu 1500\n\tether 04:ab:18:6f:c5:cf\n\tinet 10.0.0.10 netmask 0xffffff00\n"
            "\tinet 10.0.0.11 netmask 0xffffffff\n\tstatus: active\n")
    assert arp.parse_ifconfig(text) == {"mac": NIC_B, "status": "active", "inet": ["10.0.0.10", "10.0.0.11"]}
    s = [(0, "en9", "active"), (0, "en7", "inactive"), (1, "en9", "active"), (2, "en9", "inactive"),
         (3, "en7", "active"), (4, "en7", "active")]
    assert arp.link_transitions(s) == [(0, "en9", "active"), (0, "en7", "inactive"), (2, "en9", "inactive"),
                                       (3, "en7", "active")]


def frames_old_mac():
    """Board keeps sending to NIC A after the swap; NIC B ARPs for the board."""
    F = arp.Frame
    return [
        F(10.0, BOARD, "ff:ff:ff:ff:ff:ff", 0x0806, "en9", arp_op=1, arp_sha=BOARD, arp_spa="10.0.0.100",
          arp_tha="00:00:00:00:00:00", arp_tpa="10.0.0.10"),
        F(10.001, NIC_A, BOARD, 0x0806, "en9", arp_op=2, arp_sha=NIC_A, arp_spa="10.0.0.10", arp_tha=BOARD,
          arp_tpa="10.0.0.100"),
        F(10.1, BOARD, NIC_A, 0x0800, "en9", osc="/positionList"),
        F(10.2, BOARD, NIC_A, 0x0800, "en9", osc="/positionList"),
        # after the swap: gratuitous ARP of NIC B, its request for the board, the board's reply
        F(20.0, NIC_B, "ff:ff:ff:ff:ff:ff", 0x0806, "en7", arp_op=1, arp_sha=NIC_B, arp_spa="10.0.0.10",
          arp_tha="00:00:00:00:00:00", arp_tpa="10.0.0.10"),
        F(21.0, NIC_B, "ff:ff:ff:ff:ff:ff", 0x0806, "en7", arp_op=1, arp_sha=NIC_B, arp_spa="10.0.0.10",
          arp_tha="00:00:00:00:00:00", arp_tpa="10.0.0.100"),
        F(21.001, BOARD, NIC_B, 0x0806, "en7", arp_op=2, arp_sha=BOARD, arp_spa="10.0.0.100", arp_tha=NIC_B,
          arp_tpa="10.0.0.10"),
        F(21.002, NIC_B, BOARD, 0x0800, "en7", udp_sport=50100, udp_dport=50000, osc="/getVersion"),
        F(21.003, BOARD, NIC_A, 0x0800, "en7", osc="/version"),  # seen in promiscuous mode (T2)
    ]


def test_frame_summary_and_h1():
    fr = frames_old_mac()
    before = arp.frame_summary(fr, BOARD, "10.0.0.100", NAMES, 9.0, 15.0, "en7")
    assert before["board_arp_requests"] == [{"ms": 1000.0, "who_has": "10.0.0.10", "iface": "en9"}]
    assert before["board_ipv4_by_dst"] == {"NIC A": 2}
    assert before["board_osc_by_dst"] == {"NIC A /positionList": 2}
    assert before["arp_to_board"][0]["op"] == "reply" and before["arp_to_board"][0]["from"] == "NIC A"
    after = arp.frame_summary(fr, BOARD, "10.0.0.100", NAMES, 15.0, 30.0, "en7")
    assert after["board_arp_requests"] == []
    assert after["board_arp_replies"] == [{"ms": 6001.0, "to": "NIC B", "iface": "en7"}]
    assert after["arp_other"][0]["gratuitous"] is True and after["arp_other"][0]["from"] == "NIC B"
    assert after["arp_to_board"][0]["from"] == "NIC B" and after["arp_to_board"][0]["tpa"] == "10.0.0.100"
    assert after["udp_to_board_by_src"] == {"NIC B": 1}
    assert after["board_ipv4_by_dst"] == {"NIC A": 1}
    assert after["nic_b_frames_to_nic_b"] == 0
    v = arp.h1_verdict(arp.merge_summaries([after]), "10.0.0.10")
    assert v == {"board_arp_requests_for_client": 0, "frames_to_nic_b": 0, "frames_to_nic_a_seen": 1, "h1": True}


def test_h1_not_observed_when_the_board_asks_again():
    fr = frames_old_mac() + [
        arp.Frame(22.0, BOARD, "ff:ff:ff:ff:ff:ff", 0x0806, "en7", arp_op=1, arp_sha=BOARD, arp_spa="10.0.0.100",
                  arp_tha="00:00:00:00:00:00", arp_tpa="10.0.0.10"),
        arp.Frame(22.1, BOARD, NIC_B, 0x0800, "en7", osc="/positionList"),
    ]
    s = arp.frame_summary(fr, BOARD, "10.0.0.100", NAMES, 15.0, 30.0, "en7")
    assert s["nic_b_frames_to_nic_b"] == 1
    v = arp.h1_verdict(arp.merge_summaries([s]), "10.0.0.10")
    assert v["board_arp_requests_for_client"] == 1 and v["frames_to_nic_b"] == 1 and v["h1"] is False


def test_board_send_times_counts_ipv4_only():
    t = arp.board_send_times(frames_old_mac(), BOARD)
    assert list(t) == [10.1, 10.2, 21.003]


def test_match_times_with_offset_and_missing():
    p5 = np.arange(0.0, 1.0, 0.1) + 100.0
    frames = p5[[0, 1, 2, 4, 5, 6, 7, 8, 9]] - 0.003 + 0.0001  # one frame not seen, clock offset 3 ms
    m = arp.match_times(frames, p5)
    assert m["a"] == 9 and m["b"] == 10 and m["matched"] == 9
    assert abs(m["offset_ms"] - 2.9) < 0.01
    # a frame without a send nearby is not paired
    m = arp.match_times(np.append(frames, 100.95), p5)
    assert m["matched"] == 9
    assert arp.match_times(np.zeros(0), p5)["matched"] == 0


def test_covered_and_phase_windows():
    assert list(arp.covered(np.array([0.5, 1.5, 2.5]), [(0, 1), (2, 3)])) == [True, False, True]
    ev = [{"t": 1.0, "phase": "1"}, {"t": 1.5, "phase": None}, {"t": 2.0, "phase": "2"}]
    assert arp.phase_windows(ev, 5.0) == [("1", 1.0, 2.0), ("2", 2.0, 5.0)]
    ev = [{"t": 1.0, "phase": "a"}, {"t": 2.0, "phase": "a"}, {"t": 3.0, "phase": "a"}]
    assert [w[0] for w in arp.phase_windows(ev, 5.0)] == ["a", "a #2", "a #3"]


def test_to_monotonic():
    f = [arp.Frame(1000.5, BOARD, NIC_A, 0x0800)]
    assert arp.to_monotonic(f, 990.0)[0].t == 10.5
    # wall clock slewed by 1 ms/s: offset 990.000 at mono 0, 990.010 at mono 10
    clock = [(0.0, 990.0), (10.0, 990.01)]
    f = [arp.Frame(990.0, BOARD, NIC_A, 0x0800), arp.Frame(995.005, BOARD, NIC_A, 0x0800)]
    t = [x.t for x in arp.to_monotonic(f, clock)]
    assert abs(t[0] - 0.0) < 1e-9 and abs(t[1] - 5.0) < 1e-6


# --- resolution of the MAC address of destIp (gap step) ---

C = "10.0.0.10"


def test_expected_resolutions_follows_the_firmware_rule():
    # every /setDestIp, also the heartbeat from the destIp already set
    pk = [(0.0, C, "/setDestIp"), (0.1, C, "/setPositionListReportInterval")]
    pk += [(1.0 + i, C, "/setDestIp") for i in range(30)]  # heartbeat 1.0 s
    assert arp.expected_resolutions(pk) == [0.0] + [1.0 + i for i in range(30)]
    # gaps in the packets start none
    pk2 = [(32.0 + 2 * i, C, "/getVersion") for i in range(10)] + [(60.0, C, "/x"), (63.0, C, "/x")]
    assert arp.expected_resolutions(pk2) == []
    # /setDestIp from another address too
    assert arp.expected_resolutions([(0, C, "/setDestIp"), (0.5, "10.0.0.11", "/setDestIp"),
                                     (0.6, "10.0.0.11", "/x")]) == [0, 0.5]


def udp(t, src, dst, ip_src, ip_dst, dport, osc_addr=None):
    return arp.Frame(t, src, dst, 0x0800, "en9", ip_src=ip_src, ip_dst=ip_dst, udp_sport=50000, udp_dport=dport,
                     osc=osc_addr)


def arpf(t, op, sha, spa, tha, tpa, dst="ff:ff:ff:ff:ff:ff"):
    return arp.Frame(t, sha, dst, 0x0806, "en9", arp_op=op, arp_sha=sha, arp_spa=spa, arp_tha=tha, arp_tpa=tpa)


def refresh_frames(swap_reply_and_arp=False, failed_probe=False):
    fr = [
        udp(1.0000, NIC_A, BOARD, C, "10.0.0.100", 50000, "/setDestIp"),
        udp(1.0006, BOARD, "ff:ff:ff:ff:ff:ff", "10.0.0.100", "255.255.255.255", 9),
        arpf(1.0010, 1, BOARD, "10.0.0.100", "00:00:00:00:00:00", C),
        arpf(1.0011, 2, NIC_A, C, BOARD, "10.0.0.100", dst=BOARD),
        udp(1.0015, BOARD, NIC_A, "10.0.0.100", C, 9),  # the probe
        udp(1.0020, BOARD, NIC_A, "10.0.0.100", C, 50100, "/destIp"),
        udp(1.0500, BOARD, NIC_A, "10.0.0.100", C, 50100, "/positionList"),
    ]
    if swap_reply_and_arp:
        fr[2], fr[5] = fr[5], fr[2]
        fr[2].t, fr[5].t = 1.0010, 1.0020
    if failed_probe:
        # the first probe got no ARP reply (two requests, 40 ms), the second one did
        fr[2:2] = [arpf(1.0008, 1, BOARD, "10.0.0.100", "00:00:00:00:00:00", C),
                   arpf(1.0009, 1, BOARD, "10.0.0.100", "00:00:00:00:00:00", C)]
    return fr


def test_refresh_sequences_order():
    sq = arp.refresh_sequences(refresh_frames(), BOARD, "10.0.0.100", C)
    assert len(sq) == 1
    s0 = sq[0]
    assert s0["trigger"] == "/setDestIp" and s0["reply"] == "/destIp" and s0["order_ok"] is True
    assert s0["dummy_ms"] == 0.6 and s0["arp_request_ms"] == 1.0 and s0["arp_reply_ms"] == 1.1
    assert s0["probe_ms"] == 1.5 and s0["reply_ms"] == 2.0 and s0["arp_requests"] == 1
    assert arp.is_refresh_dummy(refresh_frames()[1], BOARD) and not arp.is_refresh_dummy(refresh_frames()[4], BOARD)
    assert arp.is_probe(refresh_frames()[4], BOARD, C) and not arp.is_probe(refresh_frames()[5], BOARD, C)


def test_refresh_sequences_counts_the_arp_requests():
    sq = arp.refresh_sequences(refresh_frames(failed_probe=True), BOARD, "10.0.0.100", C)
    assert sq[0]["arp_requests"] == 3 and sq[0]["arp_request_ms"] == 0.8 and sq[0]["order_ok"] is True


def test_refresh_sequences_reply_before_arp_is_out_of_order():
    sq = arp.refresh_sequences(refresh_frames(swap_reply_and_arp=True), BOARD, "10.0.0.100", C)
    assert sq[0]["order_ok"] is False


def test_refresh_sequences_without_probe_is_out_of_order():
    fr = [f for f in refresh_frames() if not arp.is_probe(f, BOARD, C)]
    sq = arp.refresh_sequences(fr, BOARD, "10.0.0.100", C)
    assert sq[0]["probe_ms"] is None and sq[0]["order_ok"] is False


def test_nearest_pulse():
    rise = np.array([1.0, 1.0006, 1.002])
    width = np.array([0.0003, 0.0002, 0.0012])
    assert arp.nearest_pulse(rise, width, 1.0007) == (1, 0.0002)
    assert arp.nearest_pulse(rise, width, 1.5) == (None, None)
    assert arp.nearest_pulse(np.zeros(0), np.zeros(0), 1.0) == (None, None)


def test_swap_recovery():
    B10 = "10.0.0.100"
    fr = [
        udp(0.5, BOARD, NIC_A, B10, C, 50100, "/positionList"),  # old MAC, before the first packet
        udp(2.0, NIC_B, BOARD, C, B10, 50000, "/setDestIp"),
        udp(2.0005, BOARD, "ff:ff:ff:ff:ff:ff", B10, "255.255.255.255", 9),
        arpf(2.0009, 1, BOARD, B10, "00:00:00:00:00:00", C),
        arpf(2.0010, 2, NIC_B, C, BOARD, B10, dst=BOARD),
        udp(2.0015, BOARD, NIC_B, B10, C, 9),  # the probe
        udp(2.0020, BOARD, NIC_B, B10, C, 50100, "/destIp"),
        udp(2.0500, BOARD, NIC_B, B10, C, 50100, "/positionList"),
        udp(9.0, BOARD, NIC_B, B10, C, 50100, "/positionList"),  # after the window
    ]
    r = arp.swap_recovery(fr, BOARD, B10, C, NIC_B, 0.0, 5.0)
    assert r["first_rx"] == "/setDestIp" and r["first_rx_s"] == 2.0
    assert r["dummy_ms"] == 0.5 and r["arp_request_ms"] == 0.9 and r["probe_ms"] == 1.5
    assert r["first_to_new_ms"] == 2.0 and r["first_to_new"] == "/destIp" and r["dummies"] == 1
    assert r["osc_to_new"] == {"/destIp": 1, "/positionList": 1}
    none = arp.swap_recovery(fr[:1], BOARD, B10, C, NIC_B, 0.0, 5.0)
    assert none["first_rx"] is None and none["dummy_ms"] is None and none["probe_ms"] is None
    assert none["dummies"] == 0


def fix_res(tag, perf_b, perf_a, macs):
    ev = [{"t": 1.0, "name": f"{tag} B swap to NIC B requested", "phase": f"{tag} B swap to B"},
          {"t": 1.5, "name": f"{tag} B 10.0.0.10 on NIC B", "phase": None},
          {"t": 5.0, "name": f"{tag} A swap to NIC A requested", "phase": f"{tag} A swap to A"},
          {"t": 5.5, "name": f"{tag} A 10.0.0.10 on NIC A", "phase": None},
          {"t": 9.0, "name": "last", "phase": None}]
    return {"board_mac": BOARD, "board_ip": "10.0.0.100", "client_ip": C, "events": ev,
            "perf": {f"{tag} on B": perf_b, f"{tag} on A": perf_a}, "dest_macs": macs}


def resolved_perf():
    return {"arpResolve": 1, "arpResolved": 1, "arpTimeout": 0, "arpDropped": 0, "udpSendFailed": 0}


def repaired(t, nic):
    B10 = "10.0.0.100"
    return [udp(t, nic, BOARD, C, B10, 50000, "/setDestIp"),
            udp(t + 0.0005, BOARD, "ff:ff:ff:ff:ff:ff", B10, "255.255.255.255", 9),
            arpf(t + 0.0009, 1, BOARD, B10, "00:00:00:00:00:00", C),
            arpf(t + 0.0010, 2, nic, C, BOARD, B10, dst=BOARD),
            udp(t + 0.0015, BOARD, nic, B10, C, 9),
            udp(t + 0.0020, BOARD, nic, B10, C, 50100, "/destIp"),
            udp(t + 0.1, BOARD, nic, B10, C, 50100, "/positionList")]


def test_fix_items_f1_repaired_by_set_dest_ip():
    macs = [{"label": "F1 on B before the first packet", "s": NIC_A, "T": NIC_A},
            {"label": "F1 on B after /setDestIp", "s": NIC_B, "T": NIC_B},
            {"label": "F1 on A before the first packet", "s": NIC_B, "T": NIC_B},
            {"label": "F1 on A after /setDestIp", "s": NIC_A, "T": NIC_A}]
    res = fix_res("F1", resolved_perf(), resolved_perf(), macs)
    items = arp.fix_items(res, repaired(3.0, NIC_B) + repaired(7.0, NIC_A), NAMES)
    assert [i.verdict for i in items] == ["PASS"] * 6, [i.detail for i in items]
    # no probe: not repaired by the resolution
    fr = [f for f in repaired(3.0, NIC_B) if not arp.is_probe(f, BOARD, C)] + repaired(7.0, NIC_A)
    assert arp.fix_items(fix_res("F1", resolved_perf(), resolved_perf(), []), fr, NAMES)[0].verdict == "FAIL"


def test_fix_items_f2_not_repaired_by_a_command():
    B10 = "10.0.0.100"
    none = {"arpResolve": 0, "arpResolved": 0, "arpTimeout": 0, "arpDropped": 0, "udpSendFailed": 0}
    macs = [{"label": "F2 on B before the first packet", "s": NIC_A, "T": NIC_A},
            {"label": "F2 on B after /getVersion", "s": NIC_A, "T": NIC_A},
            {"label": "F2 on A before the first packet", "s": NIC_A, "T": NIC_A},
            {"label": "F2 on A after /getVersion", "s": NIC_A, "T": NIC_A}]
    fr = [udp(3.0, NIC_B, BOARD, C, B10, 50000, "/getVersion"),
          udp(3.002, BOARD, NIC_A, B10, C, 50100, "/version"),  # still to NIC A
          udp(7.0, NIC_A, BOARD, C, B10, 50000, "/getVersion"),
          udp(7.002, BOARD, NIC_A, B10, C, 50100, "/version"),
          udp(7.1, BOARD, NIC_A, B10, C, 50100, "/positionList")]
    items = arp.fix_items(fix_res("F2", none, none, macs), fr, NAMES)
    assert [i.verdict for i in items] == ["PASS"] * 6, [i.detail for i in items]
    # a frame to NIC B: repaired, not as specified
    fr2 = fr + [udp(3.5, BOARD, NIC_B, B10, C, 50100, "/positionList")]
    assert arp.fix_items(fix_res("F2", none, none, []), fr2, NAMES)[0].verdict == "FAIL"


def test_assign_replies():
    s = 1_000_000_000
    sent = [0, 4 * s, 8 * s]
    # a late reply (1.5 s) still counts; none for the second; two for the third: the first
    assert arp.assign_replies(sent, [int(1.5 * s), 8 * s + 5, 8 * s + 9]) == [int(1.5 * s), None, 8 * s + 5]
    assert arp.assign_replies([], [1]) == []


def test_reach_summary():
    p = lambda probes, ok=True: {"arpResolve": 1, "arpProbe": probes, "arpProbeFailed": probes - ok,  # noqa: E731
                                 "arpResolved": int(ok), "arpTimeout": int(not ok), "arpDropped": 0,
                                 "udpSendFailed": 0}
    trials = [{"reply_ms": 2.0, "perf": p(1)}, {"reply_ms": 310.0, "perf": p(3)}, {"reply_ms": 1500.0, "perf": p(9)},
              {"reply_ms": None, "perf": p(14, ok=False)}]
    s = arp.reach_summary(trials, 1.0)
    assert s["trials"] == 4 and s["replies"] == 3 and s["replies_in_window"] == 2
    assert s["reply_ms"]["min"] == 2.0 and s["reply_ms"]["max"] == 1500.0
    assert s["resolutions"] == 4 and s["resolved"] == 3 and s["timeouts"] == 1
    assert s["probes_to_resolve"] == {1: 1, 3: 1, 9: 1}
    assert "probes to resolve {1: 1, 3: 1, 9: 1}" in arp.reach_line(s)
    # the firmware before the resolution: no arp* counters
    old = arp.reach_summary([{"reply_ms": None, "perf": {"udpSendFailed": 1}}], 1.0)
    assert "resolutions" not in old and old["replies"] == 0 and "-" in arp.reach_line(old)


def test_gap_captures_nic_a_only():
    nic = {"A": "en9", "B": "en7"}
    assert arp.capture_nics("gap", nic) == {"A": "en9"}
    for step in arp.STEPS:
        if step != "gap":
            assert arp.capture_nics(step, nic) == nic


def test_nic_names_without_nic_b():
    # `ifconfig` of an adapter that is not attached: no output, no MAC address.
    info = {"A": arp.parse_ifconfig(f"en9: flags=8863<UP>\n\tether {NIC_A}\n\tstatus: active\n"),
            "B": arp.parse_ifconfig("")}
    assert arp.nic_names(info) == {NIC_A: "NIC A"}
    info["B"] = arp.parse_ifconfig(f"en7: flags=8863<UP>\n\tether {NIC_B}\n\tstatus: inactive\n")
    assert arp.nic_names(info) == {NIC_A: "NIC A", NIC_B: "NIC B"}
