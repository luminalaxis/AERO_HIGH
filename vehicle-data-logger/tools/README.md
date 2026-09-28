# vehicle-data-logger/tools

Python 3.11 이상, 표준 라이브러리만 사용 (별도 설치 불필요).

## ride_height.py — 리니어 포텐셔미터 → 지상고 변환

```bash
python ride_height.py calibration.toml session.aero -o session_rh.csv   # SD / 피트 수신 로그
python ride_height.py calibration.toml export.csv                       # CSV (시간 + pot 4ch, mm)
```

`.aero` 입력은 재전송 중복을 제거하고 seq 순서로 정렬하며, 누락된 프레임 수를 경고로 출력합니다.
누락 구간은 채우지 않고 그대로 비워 둡니다 (보간하지 않음).

### 출력 열

| 열 | 설명 |
| --- | --- |
| `session` | `.aero` 의 boot_id (CSV 입력은 0) |
| `h_fl_mm` … `h_rr_mm` | 코너 지상고 (앞/뒤 차축 위치의 섀시 기준면) |
| `wheel_*_mm` | 휠 변위 (정지 기준 대비, + = bump) |
| `dload_*_n` | 코너 하중 변화 추정값 (스프링 + ARB, 로우패스 후) |
| `heave_mm` | 휠베이스 중앙 차체 높이 변화 (− = 내려감) |
| `pitch_deg` | 차체 피치 (+ = 뒤가 높음 / 앞이 낮음, 정적 레이크 포함) |
| `roll_deg` | 차체 롤 (+ = 왼쪽이 높음) |
| `warp_mm` | 대각 비틀림 (FL+RR − FR+RL, 정지 기준 대비) |
| `lift` | 하중이 0 이하로 추정된 코너 (예: `FR`) — `static_load_n` 입력 시 |
| `pt_<이름>_mm` | `[[points]]` 로 지정한 지점의 지상고 |

### 모델

```
d  = pot_sign × (pot − static_pot)                 댐퍼 변위 (+ = bump)
z  = d / MR   또는  motion_ratio_table 보간          휠 변위
ΔF = k_spring × d × MR_local  ±  k_arb × (z_L − z_R)   코너 하중 변화
δ  = ΔF / k_tire                                   타이어 눌림 변화
h  = static_ride_height − z − δ                    코너 지상고
```

- 정지 기준 상태의 지상고를 실측해서 넣으므로 **정적 하중은 몰라도 됩니다** (변화량만 사용). `static_load_n` 은 휠 들림 감지용 선택 항목.
- 네 코너 h 에 평면을 최소제곱으로 맞춰 heave/pitch/roll 과 임의 지점 높이를 계산합니다.
- 코일오버(스프링과 댐퍼가 같은 축)를 가정합니다. 스프링이 다른 위치에 있으면 `spring_rate_n_per_mm` 을 댐퍼 축 기준 환산값으로 넣으세요.

### 무시하는 효과 (오차 요인)

- 댐퍼 힘, 비스프링 질량 관성 → `load_lowpass_hz` 로 완화
- 타이어 공기압·온도에 따른 강성 변화, 고속 시 타이어 반경 증가
- 노면 요철 (결과는 평탄 노면 기준 지상고)

## 캘리브레이션 절차

[`calibration.example.toml`](calibration.example.toml) 을 복사해서 채웁니다 (예시 숫자는 전부 교체).

1. **정지 기준 상태**: 평평한 바닥, 드라이버 탑승, 레이스 공기압. 서스펜션을 몇 번 눌렀다 놓아 안착시킨 뒤
   각 코너 지상고(앞/뒤 차축 위치)와 그때의 pot 값을 기록 → `static_ride_height_mm`, `static_pot_mm`.
2. **pot 방향**: 차를 눌렀을 때 pot 값이 증가하면 `pot_sign = 1`, 감소하면 `-1`.
3. **ADC 환산** (`.aero` 입력 시): pot 전체 스트로크와 ADC count 로 `adc_mm_per_count`, `adc_offset_mm`.
4. **모션비**: 스프링을 빼거나 차를 들고 휠을 단계적으로 올리며 (휠 변위, pot 변위) 기록.
   거의 직선이면 `motion_ratio` 상수, 아니면 `motion_ratio_table`.
5. **스프링·ARB 강성**: 스펙 또는 실측. ARB 는 **휠 기준** 강성 (좌우 휠 변위 차 1 mm 당 N).
6. **타이어 수직 강성**: TTC 데이터를 쓰거나, 차에 무게추를 올리며 코너 하중 변화와 지상고 변화를 측정.
   타이어 눌림 = 지상고 변화 − 휠 변위(pot 로 계산) 이고, 강성 = 하중 변화 / 타이어 눌림. 공기압별로 기록.
7. **검증**: 차에 알려진 무게(예: 50 kg × 4)를 올리고 실측 지상고와 스크립트 결과를 비교.
