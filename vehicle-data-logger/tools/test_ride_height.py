"""python -m unittest (run from this directory)"""

import csv
import math
import tempfile
import unittest
from pathlib import Path

import ride_height as rh
from ride_height import ap

HERE = Path(__file__).resolve().parent


def base_config(**corner_overrides):
    return {
        "geometry": {"wheelbase_mm": 1500.0, "track_front_mm": 1200.0, "track_rear_mm": 1200.0},
        "defaults": {
            "static_pot_mm": 25.0,
            "spring_rate_n_per_mm": 30.0,
            "tire_rate_n_per_mm": 100.0,
            "motion_ratio": 1.0,
            **corner_overrides,
        },
        "corners": {
            "FL": {"static_ride_height_mm": 30.0},
            "FR": {"static_ride_height_mm": 30.0},
            "RL": {"static_ride_height_mm": 40.0},
            "RR": {"static_ride_height_mm": 40.0},
        },
    }


def run(cfg, pots, t=0.0):
    cal = rh.calibration_from_dict(cfg)
    return rh.RideHeightModel(cal).process(t, pots)


class ModelTest(unittest.TestCase):
    def test_static_reference(self):
        r = run(base_config(), [25.0] * 4)
        self.assertAlmostEqual(r["h_fl_mm"], 30.0)
        self.assertAlmostEqual(r["h_rr_mm"], 40.0)
        self.assertAlmostEqual(r["heave_mm"], 0.0)
        self.assertAlmostEqual(r["warp_mm"], 0.0)
        self.assertAlmostEqual(r["roll_deg"], 0.0)
        # 뒤가 10 mm 높음 → + pitch (nose down)
        self.assertAlmostEqual(r["pitch_deg"], math.degrees(math.atan(10 / 1500)))

    def test_uniform_compression(self):
        # d = 10, MR 1 → wheel 10, load 300 N → tire 3 mm → 13 mm 하강
        r = run(base_config(), [35.0] * 4)
        for c in ("fl", "fr", "rl", "rr"):
            self.assertAlmostEqual(r[f"wheel_{c}_mm"], 10.0)
            self.assertAlmostEqual(r[f"dload_{c}_n"], 300.0)
        self.assertAlmostEqual(r["h_fl_mm"], 17.0)
        self.assertAlmostEqual(r["heave_mm"], -13.0)

    def test_motion_ratio_and_sign(self):
        # MR 0.5, pot 감소 = bump → d = 5, wheel 10, wheel rate = 30*0.25 → 75 N
        r = run(base_config(motion_ratio=0.5, pot_sign=-1.0), [20.0] * 4)
        self.assertAlmostEqual(r["wheel_fl_mm"], 10.0)
        self.assertAlmostEqual(r["dload_fl_n"], 75.0)
        self.assertAlmostEqual(r["h_fl_mm"], 30.0 - 10.0 - 0.75)

    def test_table_matches_constant(self):
        cfg = base_config()
        cfg["corners"]["FL"]["motion_ratio_table"] = [[-20.0, -40.0], [0.0, 0.0], [20.0, 40.0]]
        const = run(base_config(motion_ratio=0.5), [30.0] * 4)
        table = run(cfg, [30.0, 25.0, 25.0, 25.0])
        self.assertAlmostEqual(table["h_fl_mm"], const["h_fl_mm"])
        self.assertAlmostEqual(table["dload_fl_n"], const["dload_fl_n"])

    def test_table_offset_and_extrapolation(self):
        cfg = base_config()
        # 기준점이 0 을 지나지 않아도 0 에서 wheel = 0 이 되도록 보정
        cfg["corners"]["FL"]["motion_ratio_table"] = [[-10.0, -9.0], [10.0, 11.0]]
        r = run(cfg, [25.0, 25.0, 25.0, 25.0])
        self.assertAlmostEqual(r["wheel_fl_mm"], 0.0)
        r = run(cfg, [55.0, 25.0, 25.0, 25.0])  # d = 30, 범위 밖 → 선형 외삽
        self.assertAlmostEqual(r["wheel_fl_mm"], 30.0)

    def test_both_motion_ratio_forms_rejected(self):
        cfg = base_config()
        cfg["corners"]["FL"].update(motion_ratio=1.0, motion_ratio_table=[[0, 0], [1, 1]])
        with self.assertRaises(rh.CalibrationError):
            rh.calibration_from_dict(cfg)

    def test_unknown_key_rejected(self):
        cfg = base_config()
        cfg["corners"]["FL"]["sprng_rate"] = 1
        with self.assertRaises(rh.CalibrationError):
            rh.calibration_from_dict(cfg)

    def test_arb_and_roll(self):
        cfg = base_config()
        cfg["arb"] = {"front_n_per_mm": 20.0}
        # 왼쪽 앞만 10 mm 압축
        r = run(cfg, [35.0, 25.0, 25.0, 25.0])
        self.assertAlmostEqual(r["dload_fl_n"], 300.0 + 200.0)
        self.assertAlmostEqual(r["dload_fr_n"], -200.0)
        self.assertLess(r["roll_deg"], 0.0)  # 왼쪽이 낮음
        self.assertNotAlmostEqual(r["warp_mm"], 0.0)

    def test_wheel_lift(self):
        cfg = base_config()
        cfg["corners"]["FR"]["static_load_n"] = 100.0
        r = run(cfg, [25.0, 15.0, 25.0, 25.0])  # FR 10 mm droop → -300 N
        self.assertEqual(r["lift"], "FR")
        # 타이어 변화는 무부하(-100 N → +1 mm) 로 제한
        self.assertAlmostEqual(r["h_fr_mm"], 30.0 + 10.0 + 1.0)

    def test_points(self):
        cfg = base_config()
        cfg["points"] = [{"name": "wing", "x_mm": -500.0, "y_mm": 0.0, "static_height_mm": 50.0}]
        r = run(cfg, [25.0] * 4)
        self.assertAlmostEqual(r["pt_wing_mm"], 50.0)
        r = run(cfg, [35.0, 35.0, 25.0, 25.0])  # 앞만 13 mm 하강
        # 평면: 앞차축 -13, 뒤차축 0 → x=-500 에서 -13 * (1 + 500/1500)
        self.assertAlmostEqual(r["pt_wing_mm"], 50.0 - 13.0 * (2000 / 1500))

    def test_lowpass_and_gap_reset(self):
        cfg = base_config()
        cfg["filter"] = {"load_lowpass_hz": 5.0, "gap_reset_s": 0.05}
        model = rh.RideHeightModel(rh.calibration_from_dict(cfg))
        model.process(0.0, [25.0] * 4)
        r = model.process(0.002, [35.0] * 4)
        self.assertLess(r["dload_fl_n"], 300.0 * 0.2)  # 계단 입력이 바로 반영되지 않음
        for i in range(2, 1000):
            r = model.process(i * 0.002, [35.0] * 4)
        self.assertAlmostEqual(r["dload_fl_n"], 300.0, places=3)
        r = model.process(10.0, [25.0] * 4)  # 긴 공백 뒤 → 필터 초기화
        self.assertAlmostEqual(r["dload_fl_n"], 0.0)


class InputTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_example_calibration_loads(self):
        cal = rh.load_calibration(HERE / "calibration.example.toml")
        self.assertEqual([p.name for p in cal.points], ["front_wing", "diffuser_exit"])
        self.assertEqual(cal.corners["RL"].spring_rate_n_per_mm, 40.0)

    def test_aero_roundtrip_with_duplicate_and_loss(self):
        cfg = base_config(adc_mm_per_count=0.01)
        cal = rh.calibration_from_dict(cfg)
        raw = 2500  # 25.0 mm

        def frame(seq, flags=0):
            payload = rh.encode_suspension_payload(2000, [[raw] * 4, [raw + 1000] * 4])
            return ap.encode(ap.Frame(ap.SRC_VEHICLE_LOGGER, ap.MSG_SUSPENSION, 7, seq,
                                      seq * 4000, payload, flags))

        other = ap.encode(ap.Frame(ap.SRC_VEHICLE_LOGGER, ap.MSG_IMU, 7, 0, 0, b"\x00" * 12))
        # seq 2 누락, seq 1 재전송 중복, 순서 뒤섞임, 앞쪽 쓰레기 바이트
        data = b"\x00\x11" + frame(1) + other + frame(0) + frame(3) + frame(1, ap.FLAG_RETRANSMIT)
        path = self.dir / "s.aero"
        path.write_bytes(data)

        stats = rh.AeroReadStats()
        samples = list(rh.read_aero(path, cal, stats))
        self.assertEqual(stats.frames, 3)
        self.assertEqual(stats.duplicates, 1)
        self.assertEqual(stats.missing_frames, {7: 1})
        self.assertEqual([round(t, 6) for _, t, _ in samples],
                         [0.0, 0.002, 0.004, 0.006, 0.012, 0.014])
        self.assertAlmostEqual(samples[0][2][0], 25.0)
        self.assertAlmostEqual(samples[1][2][0], 35.0)

    def test_cli_csv(self):
        inp = self.dir / "log.csv"
        with open(inp, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "pot_fl_mm", "pot_fr_mm", "pot_rl_mm", "pot_rr_mm"])
            w.writerow([0.0, 25.0, 25.2, 24.8, 25.1])
            w.writerow([0.002, "", 25.2, 24.8, 25.1])  # 결측 행은 건너뜀
            w.writerow([0.004, 30.0, 30.2, 29.8, 30.1])
        out = self.dir / "out.csv"
        rc = rh.main([str(HERE / "calibration.example.toml"), str(inp), "-o", str(out)])
        self.assertEqual(rc, 0)
        with open(out) as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 2)
        self.assertAlmostEqual(float(rows[0]["h_fl_mm"]), 35.0)
        self.assertAlmostEqual(float(rows[0]["pt_front_wing_mm"]), 55.0)
        self.assertLess(float(rows[1]["heave_mm"]), 0.0)

    def test_cli_bad_calibration(self):
        bad = self.dir / "bad.toml"
        bad.write_text("[geometry]\nwheelbase_mm = 1500\n")
        self.assertEqual(rh.main([str(bad), str(self.dir / "x.csv")]), 2)


if __name__ == "__main__":
    unittest.main()
