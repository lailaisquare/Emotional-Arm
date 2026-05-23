#!/usr/bin/env python3
"""Control a single Feetech ST3215 servo angle over USB serial.

Supports two write styles:
1) Position-only write (default): write 2-byte goal position.
2) Position+time+speed write (--use-pos-ex): write 6 bytes starting from a base address.

Protocol uses Feetech packet format: 0xFF 0xFF ID LEN INST PARAMS... CHK.
"""

import argparse
import sys
import time
from typing import Optional, Tuple

import serial


HEADER = b"\xFF\xFF"
INST_READ = 0x02
INST_WRITE = 0x03


def checksum(data: bytes) -> int:
    return (~sum(data) & 0xFF)


class FeetechBus:
    def __init__(self, port: str, baudrate: int, timeout: float):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=timeout,
        )

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()

    def _build_packet(self, servo_id: int, instruction: int, params: bytes) -> bytes:
        length = len(params) + 2
        body = bytes([servo_id, length, instruction]) + params
        return HEADER + body + bytes([checksum(body)])

    def _read_exact(self, size: int) -> Optional[bytes]:
        data = self.ser.read(size)
        if len(data) != size:
            return None
        return data

    def write_register(self, servo_id: int, addr: int, data: bytes, wait_status: bool = True) -> Tuple[bool, str]:
        packet = self._build_packet(servo_id, INST_WRITE, bytes([addr]) + data)

        self.ser.reset_input_buffer()
        self.ser.write(packet)
        self.ser.flush()

        if not wait_status:
            return True, "ok(no status wait)"

        # Expected status: FF FF ID LEN ERR CHK
        header = self._read_exact(2)
        if header != HEADER:
            return False, "status header timeout/invalid"

        head_rest = self._read_exact(2)
        if head_rest is None:
            return False, "status id/len timeout"

        resp_id, length = head_rest[0], head_rest[1]
        if resp_id != servo_id:
            return False, f"status id mismatch: got {resp_id}"

        payload = self._read_exact(length)
        if payload is None:
            return False, "status payload timeout"

        chk_expected = checksum(bytes([resp_id, length]) + payload[:-1])
        chk_recv = payload[-1]
        if chk_expected != chk_recv:
            return False, "status checksum mismatch"

        err = payload[0]
        if err != 0:
            return False, f"servo error code: 0x{err:02X}"

        return True, "ok"

    def read_register(self, servo_id: int, addr: int, size: int) -> Tuple[Optional[bytes], Optional[str]]:
        packet = self._build_packet(servo_id, INST_READ, bytes([addr, size]))

        self.ser.reset_input_buffer()
        self.ser.write(packet)
        self.ser.flush()

        header = self._read_exact(2)
        if header != HEADER:
            return None, "response header timeout/invalid"

        head_rest = self._read_exact(2)
        if head_rest is None:
            return None, "response id/len timeout"

        resp_id, length = head_rest[0], head_rest[1]
        if resp_id != servo_id:
            return None, f"response id mismatch: got {resp_id}"

        payload = self._read_exact(length)
        if payload is None:
            return None, "response payload timeout"

        chk_expected = checksum(bytes([resp_id, length]) + payload[:-1])
        chk_recv = payload[-1]
        if chk_expected != chk_recv:
            return None, "checksum mismatch"

        err = payload[0]
        data = payload[1:-1]
        if err != 0:
            return None, f"servo error code: 0x{err:02X}"

        if len(data) != size:
            return None, f"data size mismatch: expected {size}, got {len(data)}"

        return data, None


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def degree_to_raw(degree: float, raw_max: int) -> int:
    degree = degree % 360.0
    raw = int(round((degree / 360.0) * raw_max))
    return clamp(raw, 0, raw_max)


def raw_to_degree(raw: int, raw_max: int) -> float:
    return (raw / float(raw_max)) * 360.0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Set one ST3215 servo target angle")
    p.add_argument("--port", default="/dev/ttyUSB0", help="USB serial port, e.g. /dev/ttyUSB0")
    p.add_argument("--baudrate", type=int, default=1000000, help="Servo bus baudrate")
    p.add_argument("--timeout", type=float, default=0.05, help="Serial timeout seconds")

    p.add_argument("--id", type=int, required=True, help="Servo id (e.g. 1)")
    p.add_argument("--angle", type=float, required=True, help="Target angle in degrees [0, 360)")

    p.add_argument("--raw-max", type=int, default=4095, help="Raw max value mapping to 360 deg")
    p.add_argument("--endian", choices=["little", "big"], default="little", help="Register byte order")

    # Position-only write: address + 2-byte position.
    p.add_argument("--position-addr", type=int, default=42, help="Goal position register address (position-only mode)")

    # PosEx write: address + position(2) + time(2) + speed(2).
    p.add_argument("--use-pos-ex", action="store_true", help="Use position+time+speed write (6-byte payload)")
    p.add_argument("--pos-ex-addr", type=int, default=42, help="Base address for PosEx payload")
    p.add_argument("--time-ms", type=int, default=500, help="Move time for PosEx write")
    p.add_argument("--speed", type=int, default=0, help="Move speed for PosEx write (0 means servo default)")

    p.add_argument("--verify", action="store_true", help="Read back present position after command")
    p.add_argument("--present-addr", type=int, default=56, help="Present position register address")
    p.add_argument("--present-size", type=int, default=2, help="Present position register byte size")
    p.add_argument("--verify-delay", type=float, default=0.2, help="Seconds to wait before verify read")
    p.add_argument("--no-status", action="store_true", help="Do not wait for status packet after write")

    return p


def main() -> int:
    args = build_parser().parse_args()

    if args.id < 0 or args.id > 253:
        print("illegal --id, must be 0..253", file=sys.stderr)
        return 2

    target_raw = degree_to_raw(args.angle, args.raw_max)

    try:
        bus = FeetechBus(args.port, args.baudrate, args.timeout)
    except Exception as exc:
        print(f"open serial failed: {exc}", file=sys.stderr)
        return 1

    try:
        if args.use_pos_ex:
            t = clamp(args.time_ms, 0, 0xFFFF)
            s = clamp(args.speed, 0, 0xFFFF)
            payload = (
                target_raw.to_bytes(2, byteorder=args.endian, signed=False)
                + t.to_bytes(2, byteorder=args.endian, signed=False)
                + s.to_bytes(2, byteorder=args.endian, signed=False)
            )
            ok, msg = bus.write_register(args.id, args.pos_ex_addr, payload, wait_status=not args.no_status)
            mode = "PosEx"
            addr = args.pos_ex_addr
        else:
            payload = target_raw.to_bytes(2, byteorder=args.endian, signed=False)
            ok, msg = bus.write_register(args.id, args.position_addr, payload, wait_status=not args.no_status)
            mode = "PositionOnly"
            addr = args.position_addr

        if not ok:
            print(f"write failed: {msg}", file=sys.stderr)
            return 3

        print(
            f"write ok: id={args.id}, mode={mode}, addr={addr}, "
            f"angle={args.angle:.2f} deg, raw={target_raw}, status={msg}"
        )

        if args.verify:
            time.sleep(max(0.0, args.verify_delay))
            data, err = bus.read_register(args.id, args.present_addr, args.present_size)
            if err is not None:
                print(f"verify failed: {err}", file=sys.stderr)
                return 4

            raw = int.from_bytes(data, byteorder=args.endian, signed=False)
            deg = raw_to_degree(raw, args.raw_max)
            print(f"present: id={args.id}, raw={raw}, angle={deg:.2f} deg")

        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())

