"""AERO frame protocol v1 (pit side / tools). See docs/protocol.md.

Mirrors common/protocol/c/aero_frame.{h,c}. Standard library only.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass, field
from typing import Iterator

SYNC = b"\xA5\x5A"
PROTO_VERSION = 1
HEADER_SIZE = 22
CRC_SIZE = 4
MAX_PAYLOAD = 1024

FLAG_RETRANSMIT = 0x01

SRC_VEHICLE_LOGGER = 0x01
SRC_POWERTRAIN = 0x02
SRC_GATEWAY = 0x0F
SRC_PIT = 0x10

MSG_HEARTBEAT = 0x01
MSG_SUSPENSION = 0x10
MSG_IMU = 0x11
MSG_GPS = 0x12
MSG_CAN_RAW = 0x20
MSG_NACK = 0x80
MSG_TIME_SYNC = 0x81

# version, source_id, msg_type, flags, payload_len, boot_id, seq, timestamp_us
_HDR = struct.Struct("<BBBBHHIQ")
_NACK = struct.Struct("<BxHIH")


@dataclass(frozen=True)
class Frame:
    source_id: int
    msg_type: int
    boot_id: int
    seq: int
    timestamp_us: int
    payload: bytes = b""
    flags: int = 0

    @property
    def is_retransmit(self) -> bool:
        return bool(self.flags & FLAG_RETRANSMIT)

    @property
    def key(self) -> tuple[int, int, int]:
        """Globally unique id shared by the SD log and the radio stream."""
        return (self.source_id, self.boot_id, self.seq)


def encode(frame: Frame) -> bytes:
    if len(frame.payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    body = _HDR.pack(
        PROTO_VERSION,
        frame.source_id,
        frame.msg_type,
        frame.flags,
        len(frame.payload),
        frame.boot_id,
        frame.seq & 0xFFFFFFFF,
        frame.timestamp_us,
    ) + frame.payload
    return SYNC + body + struct.pack("<I", zlib.crc32(body))


class StreamDecoder:
    """Byte-stream decoder with resync; same behaviour as aero_parser_t."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.frames_ok = 0
        self.crc_errors = 0
        self.header_errors = 0
        self.bytes_discarded = 0

    def _resync(self) -> None:
        k = self._buf.find(SYNC[:1], 1)
        if k < 0:
            k = len(self._buf)
        self.bytes_discarded += k
        del self._buf[:k]

    def feed(self, data: bytes) -> Iterator[Frame]:
        self._buf += data
        buf = self._buf
        while True:
            if len(buf) >= 1 and buf[0] != SYNC[0]:
                self._resync()
                continue
            if len(buf) >= 2 and buf[1] != SYNC[1]:
                self._resync()
                continue
            if len(buf) < HEADER_SIZE:
                return
            ver, src, typ, flags, plen, boot, seq, ts = _HDR.unpack_from(buf, 2)
            if ver != PROTO_VERSION or plen > MAX_PAYLOAD:
                self.header_errors += 1
                self._resync()
                continue
            body_end = HEADER_SIZE + plen
            total = body_end + CRC_SIZE
            if len(buf) < total:
                return
            (crc,) = struct.unpack_from("<I", buf, body_end)
            if zlib.crc32(buf[2:body_end]) != crc:
                self.crc_errors += 1
                self._resync()
                continue
            self.frames_ok += 1
            frame = Frame(src, typ, boot, seq, ts, bytes(buf[HEADER_SIZE:body_end]), flags)
            del buf[:total]
            yield frame


def decode_datagram(data: bytes) -> Frame | None:
    """Decode exactly one frame (UDP / LoRa packet). None if invalid."""
    frames = list(StreamDecoder().feed(data))
    return frames[0] if len(frames) == 1 else None


# ---- loss detection -------------------------------------------------------


@dataclass
class GapTracker:
    """Tracks missing seq ranges for one (source_id, boot_id) session.

    Unlike the C tracker (used on the car), this keeps the full set of holes
    so the pit can NACK them and mark them filled when backfill arrives.
    """

    boot_id: int | None = None
    expected: int = 0
    received: int = 0
    duplicates: int = 0
    # sorted, non-overlapping half-open ranges [first, end) in *unwrapped* seq
    missing: list[list[int]] = field(default_factory=list)
    _base: int = 0  # unwrapping offset (multiples of 2**32)

    def _unwrap(self, seq: int) -> int:
        cand = self._base + seq
        # choose the representation closest to expected
        if cand - self.expected > 2**31:
            cand -= 2**32
        elif self.expected - cand > 2**31:
            cand += 2**32
        return cand

    def update(self, boot_id: int, seq: int) -> str:
        """Returns 'new_session', 'in_order', 'gap', 'filled' or 'duplicate'."""
        if self.boot_id != boot_id:
            self.boot_id = boot_id
            self._base = 0
            self.expected = seq + 1
            self.received = 1
            self.duplicates = 0
            self.missing = []
            return "new_session"

        s = self._unwrap(seq)
        self._base = s - seq
        if s == self.expected:
            self.expected += 1
            self.received += 1
            return "in_order"
        if s > self.expected:
            self.missing.append([self.expected, s])
            self.expected = s + 1
            self.received += 1
            return "gap"
        if self._fill(s):
            self.received += 1
            return "filled"
        self.duplicates += 1
        return "duplicate"

    def _fill(self, s: int) -> bool:
        for i, (a, b) in enumerate(self.missing):
            if a <= s < b:
                parts = [r for r in ([a, s], [s + 1, b]) if r[0] < r[1]]
                self.missing[i : i + 1] = parts
                return True
        return False

    @property
    def lost(self) -> int:
        return sum(b - a for a, b in self.missing)

    def missing_ranges(self) -> list[tuple[int, int]]:
        """(first_seq, count) with seq wrapped back to 32 bits."""
        return [(a & 0xFFFFFFFF, b - a) for a, b in self.missing]


def encode_nack(ranges: list[tuple[int, int, int, int]]) -> bytes:
    """ranges: (source_id, boot_id, first_seq, count); count is split at 65535."""
    out = bytearray()
    for src, boot, first, count in ranges:
        while count > 0:
            n = min(count, 0xFFFF)
            out += _NACK.pack(src, boot, first & 0xFFFFFFFF, n)
            first += n
            count -= n
    return bytes(out)


def decode_nack(payload: bytes) -> list[tuple[int, int, int, int]]:
    n = len(payload) // _NACK.size
    return [_NACK.unpack_from(payload, i * _NACK.size) for i in range(n)]
