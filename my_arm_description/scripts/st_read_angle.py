#!/usr/bin/env python3
"""Read Feetech ST3215 servo positions over USB serial.

Protocol: Feetech protocol 1.0 style packet (0xFF 0xFF ...), used by ST/SCS series.
Default present position register is 56 (2 bytes), which matches common STS/ST3215 layouts.
"""

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import serial


HEADER = b"\xFF\xFF"
INST_READ = 0x02


def checksum(data: bytes) -> int:
    return (~sum(data) & 0xFF)


@dataclass
class ServoReading:
    servo_id: int
    name: str
    raw: Optional[int]
    degree: Optional[float]
    error: Optional[str]


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
        # Length includes instruction + params + checksum, excluding 0xFF 0xFF ID LENGTH.
        length = len(params) + 2
        body = bytes([servo_id, length, instruction]) + params
        return HEADER + body + bytes([checksum(body)])

    def _read_exact(self, size: int) -> Optional[bytes]:
        data = self.ser.read(size)
        if len(data) != size:
            return None
        return data

    def read_register(self, servo_id: int, addr: int, size: int) -> (Optional[bytes], Optional[str]):
        packet = self._build_packet(servo_id, INST_READ, bytes([addr, size]))

        self.ser.reset_input_buffer()
        self.ser.write(packet)
        self.ser.flush()

        # Expected: FF FF ID LEN ERR DATA... CHK
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

        # payload = ERR + DATA + CHK
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


def parse_ids(text: str) -> List[int]:
    ids: List[int] = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if "-" in item:
            left, right = item.split("-", 1)
            a, b = int(left), int(right)
            step = 1 if b >= a else -1
            ids.extend(list(range(a, b + step, step)))
        else:
            ids.append(int(item))

    unique_ids = sorted(set(ids))
    for sid in unique_ids:
        if sid < 0 or sid > 253:
            raise ValueError(f"illegal servo id: {sid}")
    return unique_ids


def build_servo_name_map() -> Dict[int, str]:
    return {
        1: "base",
        2: "joint2",
        3: "joint3",
        4: "joint4",
        5: "joint5",
        6: "gripper",
    }


def read_positions(
    bus: FeetechBus,
    ids: List[int],
    addr: int,
    size: int,
    raw_max: int,
    endian: str,
    names: Dict[int, str],
) -> List[ServoReading]:
    results: List[ServoReading] = []
    for sid in ids:
        data, err = bus.read_register(sid, addr, size)
        if err is not None:
            results.append(ServoReading(servo_id=sid, name=names.get(sid, f"id{sid}"), raw=None, degree=None, error=err))
            continue

        raw = int.from_bytes(data, byteorder=endian, signed=False)
        degree = (raw / float(raw_max)) * 360.0
        results.append(ServoReading(servo_id=sid, name=names.get(sid, f"id{sid}"), raw=raw, degree=degree, error=None))
    return results


def print_results(readings: List[ServoReading]) -> None:
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"\n[{ts}] ST3215 angles")
    print("ID  Name     Raw    Degree")
    print("--  -------  -----  --------")
    for r in readings:
        if r.error:
            print(f"{r.servo_id:<2}  {r.name:<7}  {'-':>5}  ERR: {r.error}")
        else:
            print(f"{r.servo_id:<2}  {r.name:<7}  {r.raw:>5}  {r.degree:>7.2f} deg")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read ST3215 servo angles from USB serial")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="USB serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=1000000, help="Servo bus baudrate (default: 1000000)")
    parser.add_argument("--timeout", type=float, default=0.05, help="Serial read timeout in seconds")
    parser.add_argument("--ids", default="1-6", help="Servo ids, e.g. 1-6 or 1,2,3,6")
    parser.add_argument("--position-addr", type=int, default=56, help="Present position register address")
    parser.add_argument("--position-size", type=int, default=2, help="Position register byte size")
    parser.add_argument("--raw-max", type=int, default=4095, help="Max raw position for 360 deg")
    parser.add_argument("--endian", choices=["little", "big"], default="little", help="Register byte order")
    parser.add_argument("--interval", type=float, default=0.2, help="Read interval seconds when looping")
    parser.add_argument("--once", action="store_true", help="Read one frame and exit")
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    try:
        ids = parse_ids(args.ids)
    except Exception as exc:
        print(f"parse --ids failed: {exc}", file=sys.stderr)
        return 2

    names = build_servo_name_map()

    try:
        bus = FeetechBus(args.port, args.baudrate, args.timeout)
    except Exception as exc:
        print(f"open serial failed: {exc}", file=sys.stderr)
        return 1

    print(
        f"connected: port={args.port}, baud={args.baudrate}, ids={ids}, "
        f"addr={args.position_addr}, size={args.position_size}"
    )

    try:
        while True:
            readings = read_positions(
                bus=bus,
                ids=ids,
                addr=args.position_addr,
                size=args.position_size,
                raw_max=args.raw_max,
                endian=args.endian,
                names=names,
            )
            print_results(readings)

            if args.once:
                break
            time.sleep(max(0.01, args.interval))
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

