"""python -m unittest (run from this directory)"""

import unittest

import aero_protocol as ap

# Same vector as docs/protocol.md and c/test_aero_frame.c
GOLDEN = bytes.fromhex(
    "a55a0101100004000201040302010807060504030201deadbeef32c927d5"
)
GOLDEN_FRAME = ap.Frame(
    source_id=ap.SRC_VEHICLE_LOGGER,
    msg_type=ap.MSG_SUSPENSION,
    boot_id=0x0102,
    seq=0x01020304,
    timestamp_us=0x0102030405060708,
    payload=bytes.fromhex("deadbeef"),
)


class FrameTest(unittest.TestCase):
    def test_encode_golden(self):
        self.assertEqual(ap.encode(GOLDEN_FRAME), GOLDEN)

    def test_datagram_roundtrip(self):
        self.assertEqual(ap.decode_datagram(GOLDEN), GOLDEN_FRAME)
        self.assertIsNone(ap.decode_datagram(GOLDEN[:-1]))
        bad = bytearray(GOLDEN)
        bad[23] ^= 0xFF
        self.assertIsNone(ap.decode_datagram(bytes(bad)))

    def test_stream_resync(self):
        bad = bytearray(GOLDEN)
        bad[23] ^= 0xFF
        stream = b"\x00\xa5\x11\xa5\xa5" + GOLDEN + bytes(bad) + GOLDEN
        dec = ap.StreamDecoder()
        frames = []
        for b in stream:  # byte by byte
            frames += dec.feed(bytes([b]))
        self.assertEqual(frames, [GOLDEN_FRAME, GOLDEN_FRAME])
        self.assertEqual(dec.crc_errors, 1)


class GapTrackerTest(unittest.TestCase):
    def test_gap_and_backfill(self):
        t = ap.GapTracker()
        self.assertEqual(t.update(1, 10), "new_session")
        self.assertEqual(t.update(1, 11), "in_order")
        self.assertEqual(t.update(1, 15), "gap")
        self.assertEqual(t.update(1, 20), "gap")
        self.assertEqual(t.missing_ranges(), [(12, 3), (16, 4)])
        self.assertEqual(t.lost, 7)
        self.assertEqual(t.update(1, 13), "filled")
        self.assertEqual(t.missing_ranges(), [(12, 1), (14, 1), (16, 4)])
        self.assertEqual(t.update(1, 13), "duplicate")
        for s in (12, 14, 16, 17, 18, 19):
            self.assertEqual(t.update(1, s), "filled")
        self.assertEqual(t.missing_ranges(), [])
        self.assertEqual(t.update(2, 0), "new_session")

    def test_wraparound(self):
        t = ap.GapTracker()
        t.update(1, 0xFFFFFFFE)
        self.assertEqual(t.update(1, 1), "gap")
        self.assertEqual(t.missing_ranges(), [(0xFFFFFFFF, 2)])
        self.assertEqual(t.update(1, 0), "filled")
        self.assertEqual(t.update(1, 0xFFFFFFFF), "filled")
        self.assertEqual(t.missing_ranges(), [])

    def test_nack(self):
        payload = ap.encode_nack([(1, 3, 1000, 70000)])
        self.assertEqual(
            ap.decode_nack(payload),
            [(1, 3, 1000, 65535), (1, 3, 1000 + 65535, 70000 - 65535)],
        )


if __name__ == "__main__":
    unittest.main()
