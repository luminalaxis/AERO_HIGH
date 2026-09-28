# Orion BMS 2 CAN

| 항목 | 값 |
| --- | --- |
| 비트레이트 | **500 kbps** |
| DBC | [`Orion_CANBUS.dbc`](Orion_CANBUS.dbc) |
| BMS 설정 프로파일 | [`orion_bms2_profile_2026-08-14.o2bms`](orion_bms2_profile_2026-08-14.o2bms) (Orion BMS 2 Utility 로 열기, DBC 아님) |

## DBC 메시지 (`Orion_CANBUS.dbc`)

모든 신호는 Motorola(big-endian) 바이트 순서입니다.

### `0x6B0` — 8 ms 주기

| 바이트 | 신호 | 스케일 | 단위 |
| --- | --- | --- | --- |
| 0–1 | Pack_Current | 0.1 | A |
| 2–3 | Pack_Inst_Voltage | 0.1 | V |
| 4 | Pack_SOC | 0.5 | % |
| 5–6 | Relay_State | 1 | 비트 플래그 |
| 7 | CRC_Checksum | — | — |

### `0x6B1` — 104 ms 주기

| 바이트 | 신호 | 스케일 | 단위 |
| --- | --- | --- | --- |
| 0–1 | Pack_DCL | 1 | A |
| 2 | Pack_CCL | 1 | A |
| 3 | (Blank) | — | — |
| 4 | High_Temperature | 1 | °C |
| 5 | Low_Temperature | 1 | °C |
| 6 | (Blank) | — | — |
| 7 | CRC_Checksum | — | — |

## 주의 사항

1. **DBC 와 프로파일의 CAN ID 가 다릅니다.** 프로파일(2026-08-14)의 커스텀 메시지는 `0x6B0`/`0x6B1` 이 아니라 아래와 같습니다.
   실제 차량 BMS 에 어떤 설정이 들어 있는지 버스를 떠서 확인한 뒤, 맞는 쪽 기준으로 DBC 를 다시 뽑아야 합니다.

   | CAN ID | 종류 | 비고 |
   | --- | --- | --- |
   | `0x101`, `0x111`, `0x121` | 표준 11-bit, 8 B | BMS 브로드캐스트 |
   | `0x1806E5F4`, `0x1806E7F4`, `0x1806E9F4` | 확장 29-bit, 100 ms | 충전기 명령 (Elcon/TC 계열 형식) |
   | `0x18FF50E5` | 확장 29-bit | 충전기 상태 (수신) |

2. **부호 없는(unsigned) 신호 정의.** `Pack_Current`, `High_Temperature`, `Low_Temperature` 가 `@0+`(unsigned) 로 되어 있어,
   충전 전류(음수)나 영하 온도가 큰 양수(예: -1 A → 6553.5 A, -1 °C → 255 °C)로 디코딩됩니다. 필요하면 `@0-` 로 바꿔야 합니다.
3. **`CRC_Checksum` 의 offset(1720/1721)** 은 Orion 체크섬(`(ID + 길이 + 데이터 바이트 합) & 0xFF`)을 표현하려는 값이라
   디코딩된 숫자 자체는 의미가 없습니다. 검증은 코드에서 따로 해야 합니다.
4. `0x6B1` 에 이름이 같은 `Blank` 신호가 두 개 있어, `cantools` 같은 도구는 기본(strict) 모드에서 로드를 거부합니다
   (`strict=False` 로 로드하거나 `Blank_1`/`Blank_2` 로 이름 변경).
