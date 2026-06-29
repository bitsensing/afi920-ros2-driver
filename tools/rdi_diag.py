#!/usr/bin/env python3
"""AFI920 RDI diagnostic tool - SOME/IP, E2E, and RDI byte verification."""
import argparse
import math
import socket
import struct

SOMEIP_HDR = 16
TP_HDR = 4
E2E_HDR = 20
RDI_HDR = 36
DET_SIZE = 51
SERVICE_ID = 0x6000
EVENT_RDI = 0x8002
E2E_DATA_ID_RDI = 0x60008002


def crc64_xz(data):
    crc = 0xFFFFFFFFFFFFFFFF
    poly = 0xC96C5795D7870F42
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc >> 1) ^ poly) if (crc & 1) else (crc >> 1)
    return crc ^ 0xFFFFFFFFFFFFFFFF


def parse_someip(data):
    if len(data) < SOMEIP_HDR:
        return None
    msg_id, length, req_id = struct.unpack_from(">III", data, 0)
    return {
        "service_id": (msg_id >> 16) & 0xFFFF,
        "method_id": msg_id & 0xFFFF,
        "length": length,
        "session_id": req_id & 0xFFFF,
        "message_type": data[14],
        "is_tp": (data[14] & 0x20) != 0,
    }


def parse_tp(data, off):
    raw = struct.unpack_from(">I", data, off)[0]
    return (raw >> 4) * 16, (raw & 1) != 0


def consume_e2e(payload):
    if len(payload) < E2E_HDR:
        return None, {"error": f"short E2E header: {len(payload)}B"}
    e2e = {
        "crc": struct.unpack_from(">Q", payload, 0)[0],
        "length": struct.unpack_from(">I", payload, 8)[0],
        "counter": struct.unpack_from(">I", payload, 12)[0],
        "data_id": struct.unpack_from(">I", payload, 16)[0],
    }
    actual_crc = crc64_xz(payload[8:])
    e2e["crc_ok"] = (actual_crc == e2e["crc"])
    e2e["actual_crc"] = actual_crc
    e2e["length_ok"] = (e2e["length"] == len(payload) - E2E_HDR)
    e2e["data_id_ok"] = (e2e["data_id"] == E2E_DATA_ID_RDI)
    return payload[E2E_HDR:], e2e


class TpReasm:
    def __init__(self):
        self.buf = bytearray()
        self.sid = None
        self.n = 0

    def feed(self, seg, off, more, sid):
        if self.sid is not None and self.sid != sid:
            self.reset()
        if self.sid is None:
            self.sid = sid
        end = off + len(seg)
        if end > len(self.buf):
            self.buf.extend(b"\x00" * (end - len(self.buf)))
        self.buf[off:end] = seg
        self.n += 1
        if not more:
            result = bytes(self.buf[:end])
            count = self.n
            self.reset()
            return result, count
        return None, self.n

    def reset(self):
        self.buf = bytearray()
        self.sid = None
        self.n = 0


def parse_rdi_hdr(p):
    if len(p) < RDI_HDR:
        return None
    vel_begin, vel_end = struct.unpack_from("<ff", p, 24)
    return {
        "ver": f"{p[0]}.{p[1]}.{p[2]}",
        "iface_id": p[3],
        "sensor_id": p[5],
        "msg_counter": struct.unpack_from("<I", p, 14)[0],
        "dq": p[23],
        "vel_amb": (vel_begin, vel_end),
        "det_cap": struct.unpack_from("<H", p, 32)[0],
        "num_det": struct.unpack_from("<H", p, 34)[0],
    }


def parse_det(p, i):
    b = RDI_HDR + i * DET_SIZE
    if len(p) < b + DET_SIZE:
        return None
    d = p[b:b + DET_SIZE]
    rng = struct.unpack_from("<f", d, 19)[0]
    azi = struct.unpack_from("<f", d, 23)[0]
    ele = struct.unpack_from("<f", d, 27)[0]
    vel = struct.unpack_from("<f", d, 43)[0]
    rcs = struct.unpack_from("<f", d, 9)[0]
    snr = struct.unpack_from("<f", d, 13)[0]
    return {
        "i": i,
        "range": rng,
        "azi": azi,
        "ele": ele,
        "vel": vel,
        "rcs": rcs,
        "snr": snr,
        "exist": d[0],
        "det_id": struct.unpack_from("<H", d, 1)[0],
        "rng_hex": d[19:23].hex(),
        "azi_hex": d[23:27].hex(),
        "ele_hex": d[27:31].hex(),
        "raw": d.hex(),
    }


def ff(v, unit=""):
    if math.isnan(v):
        return "\033[91mNaN\033[0m"
    if math.isinf(v):
        return "\033[91mInf\033[0m"
    return f"{v:.4f}{unit}"


def show(fnum, hdr, rh, payload, e2e=None, tpn=None):
    print(f'\n{"=" * 72}\n FRAME #{fnum}\n{"=" * 72}')
    tp = f" (TP, {tpn} segs)" if tpn else " (non-TP)"
    print(
        f'  SOMEIP: svc=0x{hdr["service_id"]:04X} '
        f'method=0x{hdr["method_id"]:04X} type=0x{hdr["message_type"]:02X}{tp}'
    )
    if e2e:
        if "error" in e2e:
            print(f'  \033[91mE2E: {e2e["error"]}\033[0m')
            return
        print(
            f'  E2E: crc=0x{e2e["crc"]:016X} len={e2e["length"]} '
            f'counter={e2e["counter"]} data_id=0x{e2e["data_id"]:08X}'
        )
        if not e2e["crc_ok"]:
            print(f'       \033[93mCRC mismatch: actual=0x{e2e["actual_crc"]:016X}\033[0m')
        if not e2e["length_ok"]:
            print(f'       \033[93mlength mismatch: actual={len(payload)}\033[0m')
        if not e2e["data_id_ok"]:
            print(f'       \033[93mdata_id mismatch: expected=0x{E2E_DATA_ID_RDI:08X}\033[0m')
    print(f"  ISO payload: {len(payload)} bytes")
    if rh is None:
        print("  \033[91mRDI HEADER PARSE FAILED\033[0m")
        print(f"  First 64B hex: {payload[:64].hex()}")
        return
    print(
        f'  RDI: ver={rh["ver"]} sensor={rh["sensor_id"]} '
        f'counter={rh["msg_counter"]} dq={rh["dq"]}'
    )
    print(f'  NumDet={rh["num_det"]} cap={rh["det_cap"]} vel_amb={rh["vel_amb"]}')
    exp = RDI_HDR + rh["num_det"] * DET_SIZE
    if len(payload) < exp:
        print(
            f"  \033[93mWARN: payload {len(payload)} < expected {exp} "
            f"(short by {exp - len(payload)}B)\033[0m"
        )

    nan_c = 0
    total = 0
    for i in range(min(rh["num_det"], 4096)):
        det = parse_det(payload, i)
        if det is None:
            break
        total += 1
        if math.isnan(det["range"]):
            nan_c += 1

    for i in range(min(rh["num_det"], 10)):
        det = parse_det(payload, i)
        if det is None:
            break
        ad = math.degrees(det["azi"]) if not math.isnan(det["azi"]) else float("nan")
        ed = math.degrees(det["ele"]) if not math.isnan(det["ele"]) else float("nan")
        print(
            f'  [{i:4d}] range={ff(det["range"], "m"):>14s} '
            f'azi={ff(ad, "deg"):>12s} ele={ff(ed, "deg"):>12s} '
            f'vel={ff(det["vel"], "m/s"):>12s} snr={ff(det["snr"], "dB"):>10s} '
            f'exist={det["exist"]}'
        )
        print(
            f'         rng_hex={det["rng_hex"]} '
            f'azi_hex={det["azi_hex"]} ele_hex={det["ele_hex"]}'
        )
        if i == 0:
            print("         det[0] raw bytes:")
            for o in range(0, len(det["raw"]), 32):
                print(f'           +{o // 2:3d}: {det["raw"][o:o + 32]}')
    if rh["num_det"] > 10:
        print(f'  ... ({rh["num_det"] - 10} more)')
    print(f"\n  \033[1mResult: {total} parsed, {nan_c} NaN range ({100 * nan_c / max(total, 1):.1f}%)\033[0m")
    if nan_c > 0:
        print("  \033[93m--- NaN DIAG ---\033[0m")
        if nan_c == total:
            print("  ALL range=NaN! Check: E2E strip, TP gaps, or offset shift")
        s = max(0, RDI_HDR - 4)
        e = min(len(payload), RDI_HDR + 32)
        print(f"  Hex @header/det boundary ({s}-{e}):")
        for o in range(0, e - s, 16):
            h = " ".join(f"{b:02x}" for b in payload[s + o:s + o + 16])
            print(f"    @{s + o:4d}: {h}")


def handle_payload(fc, hdr, payload, tpn=None):
    iso, e2e = consume_e2e(payload)
    show(fc, hdr, parse_rdi_hdr(iso) if iso is not None else None, iso, e2e=e2e, tpn=tpn)


def run_udp(port, maxf):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(10)
    print(f"Listening UDP:{port} ...\n")
    ra = TpReasm()
    fc = 0
    while fc < maxf:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            print("Timeout. No data received.")
            break
        hdr = parse_someip(data)
        if hdr is None or hdr["service_id"] != SERVICE_ID or hdr["method_id"] != EVENT_RDI:
            continue
        off = SOMEIP_HDR
        if hdr["is_tp"]:
            tpo, tpm = parse_tp(data, off)
            off += TP_HDR
            seg = data[off:]
            if ra.n == 0:
                print(f'  TP seg: offset={tpo} more={tpm} len={len(seg)} sid={hdr["session_id"]}')
            payload, tn = ra.feed(seg, tpo, tpm, hdr["session_id"])
            if payload is None:
                continue
            fc += 1
            handle_payload(fc, hdr, payload, tpn=tn)
        else:
            fc += 1
            handle_payload(fc, hdr, data[off:])
    sock.close()
    print(f"\nDone. {fc} frames.")


def run_tcp(ip, port, maxf):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    print(f"Connecting TCP {ip}:{port}...")
    try:
        sock.connect((ip, port))
    except OSError as exc:
        print(f"Failed: {exc}")
        return
    print("Connected!\n")
    buf = b""
    fc = 0
    ra = TpReasm()
    while fc < maxf:
        try:
            chunk = sock.recv(65535)
        except socket.timeout:
            print("Timeout.")
            break
        if not chunk:
            print("Connection closed.")
            break
        buf += chunk
        while len(buf) >= SOMEIP_HDR:
            hdr = parse_someip(buf)
            if hdr is None:
                break
            tlen = 8 + hdr["length"]
            if len(buf) < tlen:
                break
            pkt = buf[:tlen]
            buf = buf[tlen:]
            if hdr["service_id"] != SERVICE_ID or hdr["method_id"] != EVENT_RDI:
                continue
            off = SOMEIP_HDR
            if hdr["is_tp"]:
                tpo, tpm = parse_tp(pkt, off)
                off += TP_HDR
                payload, tn = ra.feed(pkt[off:], tpo, tpm, hdr["session_id"])
                if payload is None:
                    continue
                fc += 1
                handle_payload(fc, hdr, payload, tpn=tn)
            else:
                fc += 1
                handle_payload(fc, hdr, pkt[off:])
            if fc >= maxf:
                break
    sock.close()
    print(f"\nDone. {fc} frames.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=30509)
    ap.add_argument("--tcp", action="store_true")
    ap.add_argument("--sensor-ip", default="192.168.10.150")
    ap.add_argument("--max-frames", type=int, default=3)
    args = ap.parse_args()
    print(f'AFI920 RDI Diag | {"TCP" if args.tcp else "UDP"}:{args.port} | max={args.max_frames}\n')
    if args.tcp:
        run_tcp(args.sensor_ip, args.port, args.max_frames)
    else:
        run_udp(args.port, args.max_frames)
