#!/usr/bin/env python3
"""리니어 포텐셔미터 4채널 → 코너 지상고 / heave / pitch / roll / warp 변환.

모델 (코너별, 모든 값은 정지 기준 상태 대비 변화량):

    damper   d  = pot_sign * (pot - static_pot)          [mm, + = bump]
    wheel    z  = f(d)          (모션비 상수 또는 실측 테이블)
    load    dF  = k_spring * d * MR_local + ARB          [N]
    tire     δ  = dF / k_tire                            [mm]
    height   h  = static_ride_height - z - δ             [mm]

4개 코너 h 로 차체 평면(최소제곱)을 구해 heave/pitch/roll 과 임의 지점의
지상고를 계산한다. 자세한 설명은 README.md 참고.

사용법:
    python ride_height.py calibration.toml session.aero -o out.csv
    python ride_height.py calibration.toml export.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
import tomllib
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Iterable, Iterator, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "common" / "protocol" / "python"))
import aero_protocol as ap  # noqa: E402

CORNERS = ("FL", "FR", "RL", "RR")

# SUSPENSION payload (docs/protocol.md): u32 sample_period_us + N x 4 x u16 raw ADC
SUSPENSION_HDR = struct.Struct("<I")
SUSPENSION_SAMPLE = struct.Struct("<4H")


class CalibrationError(ValueError):
    pass


# ---------------------------------------------------------------- calibration


@dataclass
class Corner:
    name: str
    static_ride_height_mm: float
    static_pot_mm: float
    spring_rate_n_per_mm: float
    tire_rate_n_per_mm: float
    pot_sign: float = 1.0
    motion_ratio: float | None = None  # damper travel / wheel travel
    motion_ratio_table: list[tuple[float, float]] | None = None  # (damper_mm, wheel_mm)
    static_load_n: float | None = None
    adc_mm_per_count: float | None = None
    adc_offset_mm: float = 0.0

    def __post_init__(self) -> None:
        if (self.motion_ratio is None) == (self.motion_ratio_table is None):
            raise CalibrationError(
                f"{self.name}: motion_ratio 또는 motion_ratio_table 중 하나만 지정하세요"
            )
        if self.motion_ratio is not None and self.motion_ratio <= 0:
            raise CalibrationError(f"{self.name}: motion_ratio 는 양수여야 합니다")
        if self.tire_rate_n_per_mm <= 0:
            raise CalibrationError(f"{self.name}: tire_rate_n_per_mm 는 양수여야 합니다")
        if self.motion_ratio_table is not None:
            table = sorted((float(d), float(w)) for d, w in self.motion_ratio_table)
            if len(table) < 2:
                raise CalibrationError(f"{self.name}: motion_ratio_table 은 2점 이상 필요합니다")
            for (d0, w0), (d1, w1) in zip(table, table[1:]):
                if not (d1 > d0 and w1 > w0):
                    raise CalibrationError(
                        f"{self.name}: motion_ratio_table 은 damper/wheel 모두 단조 증가해야 합니다"
                    )
            self.motion_ratio_table = table
            self._wheel_at_zero = self._interp(0.0)[0]

    def _interp(self, d: float) -> tuple[float, float]:
        t = self.motion_ratio_table
        assert t is not None
        i = 0
        while i < len(t) - 2 and d > t[i + 1][0]:
            i += 1
        (d0, w0), (d1, w1) = t[i], t[i + 1]
        slope = (w1 - w0) / (d1 - d0)
        return w0 + (d - d0) * slope, 1.0 / slope  # wheel, local MR (damper/wheel)

    def wheel_travel(self, damper_mm: float) -> tuple[float, float]:
        """damper 변위 → (wheel 변위, 국부 모션비). 테이블 범위 밖은 선형 외삽."""
        if self.motion_ratio is not None:
            return damper_mm / self.motion_ratio, self.motion_ratio
        wheel, mr = self._interp(damper_mm)
        return wheel - self._wheel_at_zero, mr

    def pot_from_raw(self, raw: int) -> float:
        if self.adc_mm_per_count is None:
            raise CalibrationError(f"{self.name}: .aero 입력에는 adc_mm_per_count 가 필요합니다")
        return raw * self.adc_mm_per_count + self.adc_offset_mm


@dataclass
class Point:
    name: str
    x_mm: float  # 앞차축 기준, + = 뒤쪽
    y_mm: float  # 차량 중심선 기준, + = 왼쪽
    static_height_mm: float


@dataclass
class Calibration:
    corners: dict[str, Corner]
    wheelbase_mm: float
    track_front_mm: float
    track_rear_mm: float
    arb_front_n_per_mm: float = 0.0
    arb_rear_n_per_mm: float = 0.0
    load_lowpass_hz: float = 0.0
    gap_reset_s: float = 0.05
    points: list[Point] = field(default_factory=list)
    csv_time_column: str = "time_s"
    csv_pot_columns: dict[str, str] = field(
        default_factory=lambda: {c: f"pot_{c.lower()}_mm" for c in CORNERS}
    )

    def corner_xy(self, c: str) -> tuple[float, float]:
        x = 0.0 if c[0] == "F" else self.wheelbase_mm
        half = (self.track_front_mm if c[0] == "F" else self.track_rear_mm) / 2
        return x, half if c[1] == "L" else -half


def _build(cls, data: dict, where: str):
    allowed = {f.name for f in fields(cls)}
    unknown = set(data) - allowed
    if unknown:
        raise CalibrationError(f"{where}: 알 수 없는 항목 {sorted(unknown)}")
    try:
        return cls(**data)
    except TypeError as e:
        raise CalibrationError(f"{where}: {e}") from None


def calibration_from_dict(data: dict) -> Calibration:
    defaults = data.get("defaults", {})
    corner_data = data.get("corners", {})
    missing = [c for c in CORNERS if c not in corner_data]
    if missing:
        raise CalibrationError(f"[corners] 에 {missing} 가 없습니다")
    corners = {}
    for c in CORNERS:
        own = corner_data[c]
        merged = {"name": c, **defaults, **own}
        # 코너에서 모션비 방식을 지정하면 defaults 의 다른 방식은 무시
        for mine, other in (("motion_ratio_table", "motion_ratio"), ("motion_ratio", "motion_ratio_table")):
            if mine in own and other not in own:
                merged.pop(other, None)
        corners[c] = _build(Corner, merged, f"[corners.{c}]")
    geo = data.get("geometry", {})
    arb = data.get("arb", {})
    flt = data.get("filter", {})
    csv_cfg = data.get("csv", {})
    try:
        cal = Calibration(
            corners=corners,
            wheelbase_mm=float(geo["wheelbase_mm"]),
            track_front_mm=float(geo["track_front_mm"]),
            track_rear_mm=float(geo["track_rear_mm"]),
            arb_front_n_per_mm=float(arb.get("front_n_per_mm", 0.0)),
            arb_rear_n_per_mm=float(arb.get("rear_n_per_mm", 0.0)),
            load_lowpass_hz=float(flt.get("load_lowpass_hz", 0.0)),
            gap_reset_s=float(flt.get("gap_reset_s", 0.05)),
            points=[_build(Point, p, "[[points]]") for p in data.get("points", [])],
        )
    except KeyError as e:
        raise CalibrationError(f"[geometry] 에 {e} 가 없습니다") from None
    if "time_column" in csv_cfg:
        cal.csv_time_column = csv_cfg["time_column"]
    if "pot_columns" in csv_cfg:
        cal.csv_pot_columns.update(csv_cfg["pot_columns"])
    return cal


def load_calibration(path: str | Path) -> Calibration:
    with open(path, "rb") as f:
        return calibration_from_dict(tomllib.load(f))


# ---------------------------------------------------------------- model


class PlaneFit:
    """h(x, y) = a + b*x + c*y 최소제곱 적합 (고정 지오메트리 → 가중치 사전계산)."""

    def __init__(self, xy: Sequence[tuple[float, float]]):
        rows = [(1.0, x, y) for x, y in xy]
        ata = [[sum(r[i] * r[j] for r in rows) for j in range(3)] for i in range(3)]
        inv = _inv3(ata)
        self._p = [[sum(inv[i][k] * rows[n][k] for k in range(3)) for n in range(len(rows))]
                   for i in range(3)]

    def fit(self, h: Sequence[float]) -> tuple[float, float, float]:
        a, b, c = (sum(w * v for w, v in zip(row, h)) for row in self._p)
        return a, b, c


def _inv3(m: list[list[float]]) -> list[list[float]]:
    (a, b, c), (d, e, f), (g, h, i) = m
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        raise CalibrationError("지오메트리가 잘못되었습니다 (wheelbase/track 확인)")
    return [
        [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
        [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
        [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det],
    ]


OUTPUT_COLUMNS = (
    ["time_s"]
    + [f"h_{c.lower()}_mm" for c in CORNERS]
    + [f"wheel_{c.lower()}_mm" for c in CORNERS]
    + [f"dload_{c.lower()}_n" for c in CORNERS]
    + ["heave_mm", "pitch_deg", "roll_deg", "warp_mm", "lift"]
)


class RideHeightModel:
    def __init__(self, cal: Calibration):
        self.cal = cal
        self._c = [cal.corners[c] for c in CORNERS]
        self._plane = PlaneFit([cal.corner_xy(c) for c in CORNERS])
        self._ref = self._plane.fit([c.static_ride_height_mm for c in self._c])
        self._mid = (cal.wheelbase_mm / 2, 0.0)
        self.reset()

    def reset(self) -> None:
        self._filt: list[float] | None = None
        self._last_t: float | None = None

    def _lowpass(self, t: float, dF: list[float]) -> list[float]:
        fc = self.cal.load_lowpass_hz
        last, self._last_t = self._last_t, t
        if fc <= 0:
            return dF
        dt = None if last is None else t - last
        if self._filt is None or dt is None or dt <= 0 or dt > self.cal.gap_reset_s:
            self._filt = list(dF)  # 시작 / 데이터 끊김 → 필터 초기화
        else:
            alpha = dt / (dt + 1.0 / (2 * math.pi * fc))
            self._filt = [f + alpha * (x - f) for f, x in zip(self._filt, dF)]
        return self._filt

    @staticmethod
    def _plane_at(p: tuple[float, float, float], x: float, y: float) -> float:
        return p[0] + p[1] * x + p[2] * y

    def process(self, t: float, pots_mm: Sequence[float]) -> dict:
        d = [c.pot_sign * (p - c.static_pot_mm) for c, p in zip(self._c, pots_mm)]
        wm = [c.wheel_travel(di) for c, di in zip(self._c, d)]
        z = [w for w, _ in wm]
        dF = [c.spring_rate_n_per_mm * di * mr for c, di, (_, mr) in zip(self._c, d, wm)]

        # ARB (휠 기준 강성): 좌우 휠 변위 차이에 비례, 좌우 반대 방향
        for (l, r), k in (((0, 1), self.cal.arb_front_n_per_mm), ((2, 3), self.cal.arb_rear_n_per_mm)):
            f = k * (z[l] - z[r])
            dF[l] += f
            dF[r] -= f

        dF = self._lowpass(t, dF)

        lifted = []
        tire = []
        for c, f in zip(self._c, dF):
            if c.static_load_n is not None and c.static_load_n + f < 0:
                lifted.append(c.name)
                f = -c.static_load_n  # 타이어 완전 무부하 (휠 들림)
            tire.append(f / c.tire_rate_n_per_mm)

        h = [c.static_ride_height_mm - zi - ti for c, zi, ti in zip(self._c, z, tire)]
        plane = self._plane.fit(h)
        row = {"time_s": t}
        for c, hi, zi, fi in zip(CORNERS, h, z, dF):
            row[f"h_{c.lower()}_mm"] = hi
            row[f"wheel_{c.lower()}_mm"] = zi
            row[f"dload_{c.lower()}_n"] = fi
        row["heave_mm"] = self._plane_at(plane, *self._mid) - self._plane_at(self._ref, *self._mid)
        row["pitch_deg"] = math.degrees(math.atan(plane[1]))  # + = 뒤가 높음 (nose down)
        row["roll_deg"] = math.degrees(math.atan(plane[2]))  # + = 왼쪽이 높음
        s = [c.static_ride_height_mm for c in self._c]
        row["warp_mm"] = (h[0] + h[3] - h[1] - h[2]) - (s[0] + s[3] - s[1] - s[2])
        row["lift"] = "|".join(lifted)
        for p in self.cal.points:
            row[f"pt_{p.name}_mm"] = p.static_height_mm + (
                self._plane_at(plane, p.x_mm, p.y_mm) - self._plane_at(self._ref, p.x_mm, p.y_mm)
            )
        return row


# ---------------------------------------------------------------- inputs

# 입력 반복자: (session, time_s, [FL, FR, RL, RR] pot_mm). session 이 바뀌면 필터 초기화.
Sample = tuple[int, float, list[float]]


def read_csv(path: str | Path, cal: Calibration) -> Iterator[Sample]:
    with open(path, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        cols = [cal.csv_time_column] + [cal.csv_pot_columns[c] for c in CORNERS]
        missing = [c for c in cols if c not in (reader.fieldnames or [])]
        if missing:
            raise CalibrationError(f"CSV 에 {missing} 열이 없습니다 ([csv] 설정 확인)")
        for row in reader:
            try:
                vals = [float(row[c]) for c in cols]
            except (TypeError, ValueError):
                continue  # 빈 칸 / 잘못된 행은 건너뜀
            yield 0, vals[0], vals[1:]


@dataclass
class AeroReadStats:
    frames: int = 0
    duplicates: int = 0
    crc_errors: int = 0
    missing_frames: dict[int, int] = field(default_factory=dict)  # boot_id → 개수


def read_aero(path: str | Path, cal: Calibration, stats: AeroReadStats | None = None) -> Iterator[Sample]:
    """SD 로그/수신 로그에서 SUSPENSION 프레임을 읽는다.

    재전송 등 중복은 (boot_id, seq) 로 제거하고 seq 순서로 정렬한다.
    """
    stats = stats if stats is not None else AeroReadStats()
    dec = ap.StreamDecoder()
    frames: dict[tuple[int, int], ap.Frame] = {}
    with open(path, "rb") as f:
        while chunk := f.read(1 << 16):
            for fr in dec.feed(chunk):
                if fr.source_id != ap.SRC_VEHICLE_LOGGER or fr.msg_type != ap.MSG_SUSPENSION:
                    continue
                key = (fr.boot_id, fr.seq)
                if key in frames:
                    stats.duplicates += 1
                    continue
                frames[key] = fr
    stats.frames = len(frames)
    stats.crc_errors = dec.crc_errors

    trackers: dict[int, ap.GapTracker] = {}
    for boot, seq in sorted(frames):
        trackers.setdefault(boot, ap.GapTracker()).update(boot, seq)
    stats.missing_frames = {b: t.lost for b, t in trackers.items() if t.lost}

    corners = [cal.corners[c] for c in CORNERS]
    for boot, seq in sorted(frames):
        fr = frames[(boot, seq)]
        if len(fr.payload) < SUSPENSION_HDR.size:
            continue
        (period_us,) = SUSPENSION_HDR.unpack_from(fr.payload)
        body = fr.payload[SUSPENSION_HDR.size:]
        for i, raw in enumerate(SUSPENSION_SAMPLE.iter_unpack(body[: len(body) // 8 * 8])):
            t = (fr.timestamp_us + i * period_us) / 1e6
            yield boot, t, [c.pot_from_raw(r) for c, r in zip(corners, raw)]


def encode_suspension_payload(period_us: int, samples: Iterable[Sequence[int]]) -> bytes:
    """펌웨어/테스트용: SUSPENSION payload 생성."""
    return SUSPENSION_HDR.pack(period_us) + b"".join(SUSPENSION_SAMPLE.pack(*s) for s in samples)


# ---------------------------------------------------------------- driver


def convert(samples: Iterable[Sample], cal: Calibration) -> Iterator[dict]:
    model = RideHeightModel(cal)
    session = None
    for sess, t, pots in samples:
        if sess != session:
            model.reset()
            session = sess
        row = model.process(t, pots)
        row["session"] = sess
        yield row


def write_csv(rows: Iterable[dict], path: str | Path, cal: Calibration) -> int:
    cols = ["session"] + OUTPUT_COLUMNS + [f"pt_{p.name}_mm" for p in cal.points]
    n = 0
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for row in rows:
            w.writerow(f"{row[c]:.4f}" if isinstance(row[c], float) else row[c] for c in cols)
            n += 1
    return n


def main(argv: Sequence[str] | None = None) -> int:
    ap_ = argparse.ArgumentParser(description="리니어 포텐셔미터 → 지상고 변환")
    ap_.add_argument("calibration", help="캘리브레이션 TOML")
    ap_.add_argument("input", help=".aero (SD/수신 로그) 또는 .csv")
    ap_.add_argument("-o", "--output", help="출력 CSV (기본: <input>_ride_height.csv)")
    args = ap_.parse_args(argv)

    inp = Path(args.input)
    out = Path(args.output) if args.output else inp.with_name(inp.stem + "_ride_height.csv")
    try:
        cal = load_calibration(args.calibration)
        stats = None
        if inp.suffix.lower() == ".aero":
            stats = AeroReadStats()
            samples = read_aero(inp, cal, stats)
        else:
            samples = read_csv(inp, cal)
        n = write_csv(convert(samples, cal), out, cal)
    except CalibrationError as e:
        print(f"오류: {e}", file=sys.stderr)
        return 2

    print(f"{n} 샘플 → {out}", file=sys.stderr)
    if stats is not None:
        print(
            f"프레임 {stats.frames}, 중복 {stats.duplicates}, CRC 오류 {stats.crc_errors}",
            file=sys.stderr,
        )
        for boot, lost in stats.missing_frames.items():
            print(f"  경고: boot {boot} 에서 프레임 {lost}개 누락 (해당 구간 데이터 없음)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
