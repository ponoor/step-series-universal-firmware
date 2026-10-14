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


def test_to_monotonic():
    f = [arp.Frame(1000.5, BOARD, NIC_A, 0x0800)]
    assert arp.to_monotonic(f, 990.0)[0].t == 10.5
    # wall clock slewed by 1 ms/s: offset 990.000 at mono 0, 990.010 at mono 10
    clock = [(0.0, 990.0), (10.0, 990.01)]
    f = [arp.Frame(990.0, BOARD, NIC_A, 0x0800), arp.Frame(995.005, BOARD, NIC_A, 0x0800)]
    t = [x.t for x in arp.to_monotonic(f, clock)]
    assert abs(t[0] - 0.0) < 1e-9 and abs(t[1] - 5.0) < 1e-6
