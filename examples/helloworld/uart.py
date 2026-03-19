#!/usr/bin/env python3
"""
UART protocol client for Buildroot Linux host.

No third-party dependencies are required (no pyserial).
It uses Linux termios + select + SLIP framing.

Protocol (inside one SLIP frame):
    +---------+---------+--------------+----------+----------+
    | type(1) | seq(1)  | payload_len(2)| payload  | crc16(2) |
    +---------+---------+--------------+----------+----------+
    payload_len is little-endian.
    crc16 = CRC-16/CCITT-FALSE on bytes: type..payload

Request command IDs (same semantic as spi.py):
    0x01 INIT
    0x02 GET_CAMERA_INFO
    0x03 GET_CAMERA_FRAME

Response IDs:
    request_id | 0x80  (ACK/response for specific request)
    0x90              (frame data chunk for GET_CAMERA_FRAME)

Examples:
    python3 uart.py --dev /dev/ttyS1 ping
    python3 uart.py --dev /dev/ttyS1 info
    python3 uart.py --dev /dev/ttyS1 frame --out /tmp/esp_frame.jpg
"""

import argparse
import os
import select
import struct
import sys
import termios
import time

# Command constants (aligned with include/control.h)
CMD_INIT = 0x01
CMD_GET_CAMERA_INFO = 0x02
CMD_GET_CAMERA_FRAME = 0x03
CMD_PING = 0x7F

# Response/constants
RESP_MASK = 0x80
RESP_FRAME_CHUNK = 0x90
MAX_FRAME_SIZE = 2 * 1024 * 1024
FRAME_CHUNK_TIMEOUT_S = 1.0
DEFAULT_TIMEOUT_S = 2.0

# SLIP constants
SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def slip_encode(payload: bytes) -> bytes:
    out = bytearray([SLIP_END])
    for b in payload:
        if b == SLIP_END:
            out.extend((SLIP_ESC, SLIP_ESC_END))
        elif b == SLIP_ESC:
            out.extend((SLIP_ESC, SLIP_ESC_ESC))
        else:
            out.append(b)
    out.append(SLIP_END)
    return bytes(out)


def slip_decode(frame: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(frame):
        b = frame[i]
        if b == SLIP_ESC:
            if i + 1 >= len(frame):
                raise ValueError("invalid SLIP escape at end of frame")
            n = frame[i + 1]
            if n == SLIP_ESC_END:
                out.append(SLIP_END)
            elif n == SLIP_ESC_ESC:
                out.append(SLIP_ESC)
            else:
                raise ValueError(f"invalid SLIP escape value 0x{n:02X}")
            i += 2
            continue
        out.append(b)
        i += 1
    return bytes(out)


def hexdump(data: bytes, width: int = 16, prefix: str = "") -> None:
    for i in range(0, len(data), width):
        chunk = data[i : i + width]
        hexs = " ".join(f"{b:02x}" for b in chunk)
        text = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"{prefix}{i:04x}: {hexs:<{width*3}} {text}")


class UartPort:
    def __init__(self, dev: str, baud: int):
        self.dev = dev
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self._configure(baud)
        self._rx_buf = bytearray()

    def _configure(self, baud: int) -> None:
        attrs = termios.tcgetattr(self.fd)

        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0

        baud_map = {
            9600: termios.B9600,
            19200: termios.B19200,
            38400: termios.B38400,
            57600: termios.B57600,
            115200: termios.B115200,
            230400: termios.B230400,
            460800: termios.B460800,
            921600: termios.B921600,
        }
        for extra in (1000000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000):
            sym = getattr(termios, f"B{extra}", None)
            if sym is not None:
                baud_map[extra] = sym
        if baud not in baud_map:
            raise ValueError(f"unsupported baudrate: {baud}")

        speed = baud_map[baud]
        attrs[4] = speed  # ispeed
        attrs[5] = speed  # ospeed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self) -> None:
        os.close(self.fd)

    def write_all(self, data: bytes) -> None:
        view = memoryview(data)
        sent = 0
        while sent < len(view):
            n = os.write(self.fd, view[sent:])
            if n <= 0:
                raise RuntimeError("uart write failed")
            sent += n

    def read_slip_frame(self, timeout_s: float) -> bytes:
        deadline = time.monotonic() + timeout_s
        # print("read slip frame dead line: ")

        while True:
            end_idx = self._rx_buf.find(bytes([SLIP_END]))
            if end_idx != -1:
                # Drop leading END bytes.
                while self._rx_buf and self._rx_buf[0] == SLIP_END:
                    del self._rx_buf[0]
                    end_idx = self._rx_buf.find(bytes([SLIP_END]))
                if end_idx != -1:
                    frame = bytes(self._rx_buf[:end_idx])
                    del self._rx_buf[: end_idx + 1]
                    if frame:
                        return frame

            remain = deadline - time.monotonic()
            if remain <= 0:
                raise TimeoutError("read frame timeout")

            rlist, _, _ = select.select([self.fd], [], [], remain)
            if not rlist:
                continue
            chunk = os.read(self.fd, 16384)
            if chunk:
                self._rx_buf.extend(chunk)


class UartProtocol:
    def __init__(self, port: UartPort, timeout_s: float = DEFAULT_TIMEOUT_S):
        self.port = port
        self.timeout_s = timeout_s
        self.seq = 0

    def _build_packet(self, ptype: int, seq: int, payload: bytes) -> bytes:
        header = struct.pack("<BBH", ptype, seq, len(payload))
        crc = crc16_ccitt_false(header + payload)
        return header + payload + struct.pack("<H", crc)

    def _parse_packet(self, raw: bytes):
        if len(raw) < 6:
            raise ValueError("packet too short")
        ptype, seq, plen = struct.unpack("<BBH", raw[:4])
        if len(raw) != 4 + plen + 2:
            raise ValueError("packet length mismatch")
        payload = raw[4 : 4 + plen]
        recv_crc = struct.unpack("<H", raw[4 + plen : 6 + plen])[0]
        calc_crc = crc16_ccitt_false(raw[: 4 + plen])
        if recv_crc != calc_crc:
            raise ValueError(
                f"crc mismatch recv=0x{recv_crc:04X} calc=0x{calc_crc:04X}"
            )
        return ptype, seq, payload

    def send_packet(self, ptype: int, payload: bytes = b"") -> int:
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        pkt = self._build_packet(ptype, seq, payload)
        self.port.write_all(slip_encode(pkt))
        return seq

    def recv_packet(self, timeout_s: float = None):
        raw = self.port.read_slip_frame(timeout_s or self.timeout_s)
        decoded = slip_decode(raw)
        return self._parse_packet(decoded)

    def request(self, cmd: int, payload: bytes = b"", timeout_s: float = None) -> bytes:
        seq = self.send_packet(cmd, payload)
        rsp_type, rsp_seq, rsp_payload = self.recv_packet(timeout_s)
        expected_rsp = cmd | RESP_MASK
        if rsp_type != expected_rsp or rsp_seq != seq:
            raise RuntimeError(
                f"unexpected response type/seq: type=0x{rsp_type:02X} seq={rsp_seq}, "
                f"expected type=0x{expected_rsp:02X} seq={seq}"
            )
        return rsp_payload


def cmd_ping(proto: UartProtocol) -> None:
    rsp = proto.request(CMD_PING, b"ping")
    print("[ping] response payload:")
    hexdump(rsp, prefix="  ")


def cmd_init(proto: UartProtocol) -> None:
    rsp = proto.request(CMD_INIT)
    print(f"[init] ok, payload_len={len(rsp)}")
    if rsp:
        hexdump(rsp, prefix="  ")


def cmd_info(proto: UartProtocol) -> None:
    rsp = proto.request(CMD_GET_CAMERA_INFO)
    if len(rsp) < 6:
        raise RuntimeError(f"invalid camera info length: {len(rsp)}")
    width, height, fmt, connected = struct.unpack("<HHBB", rsp[:6])
    print(f"[info] width={width} height={height} format={fmt} connected={connected}")
    if len(rsp) > 6:
        print("[info] extra bytes:")
        hexdump(rsp[6:], prefix="  ")


def cmd_frame(proto: UartProtocol, out_path: str) -> None:
    rsp = proto.request(CMD_GET_CAMERA_FRAME)
    if len(rsp) < 4:
        raise RuntimeError("frame response too short, need 4-byte length")
    frame_len = struct.unpack("<I", rsp[:4])[0]
    print(f"[frame] expected length={frame_len}")
    if frame_len == 0 or frame_len > MAX_FRAME_SIZE:
        raise RuntimeError(f"invalid frame length {frame_len}")

    data = bytearray()
    if len(rsp) > 4:
        # Allow firmware to append first chunk in ACK payload.
        data.extend(rsp[4:])

    while len(data) < frame_len:
        ptype, seq, payload = proto.recv_packet(FRAME_CHUNK_TIMEOUT_S)
        if ptype != RESP_FRAME_CHUNK:
            raise RuntimeError(
                f"unexpected packet while reading frame: type=0x{ptype:02X}, seq={seq}"
            )
        data.extend(payload)
        if len(data) > frame_len:
            data = data[:frame_len]
        pct = (len(data) * 100) // frame_len
        print(f"\r[frame] receiving {len(data)}/{frame_len} ({pct}%)", end="", flush=True)

    print()
    with open(out_path, "wb") as f:
        f.write(data)
    print(f"[frame] saved to {out_path}")


def cmd_listen(proto: UartProtocol) -> None:
    print("[listen] Ctrl+C to stop")
    while True:
        ptype, seq, payload = proto.recv_packet(timeout_s=60.0)
        ts = time.strftime("%H:%M:%S")
        print(f"{ts} type=0x{ptype:02X} seq={seq} len={len(payload)}")
        if payload:
            hexdump(payload, prefix="  ")


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="UART protocol client (Buildroot host)")
    p.add_argument("--dev", default="/dev/ttyS1", help="UART device, e.g. /dev/ttyS1")
    p.add_argument("--baud", type=int, default=115200, help="baudrate")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="request timeout")

    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ping", help="send ping packet")
    sub.add_parser("init", help="send init command")
    sub.add_parser("info", help="get camera info")
    f = sub.add_parser("frame", help="get camera frame")
    f.add_argument("--out", default="/tmp/esp_frame.jpg", help="output file path")
    sub.add_parser("listen", help="print all incoming packets")
    return p


def main() -> int:
    args = build_arg_parser().parse_args()
    port = UartPort(args.dev, args.baud)
    proto = UartProtocol(port, timeout_s=args.timeout)

    try:
        if args.cmd == "ping":
            cmd_ping(proto)
        elif args.cmd == "init":
            cmd_init(proto)
        elif args.cmd == "info":
            cmd_info(proto)
        elif args.cmd == "frame":
            cmd_frame(proto, args.out)
        elif args.cmd == "listen":
            cmd_listen(proto)
        else:
            raise RuntimeError(f"unknown command: {args.cmd}")
        return 0
    except KeyboardInterrupt:
        print("\n[exit] interrupted")
        return 130
    except Exception as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1
    finally:
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
