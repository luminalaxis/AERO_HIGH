# wireless-powertrain-control

배터리 BMS(Orion BMS 2)와 모터 컨트롤러(Sevcon Gen4)의 데이터를 수집해 피트로 무선 전송하고, 제한된 원격 제어를 제공하는 프로젝트입니다.
전체 구조는 [docs/architecture.md](../docs/architecture.md), 데이터 형식은 [docs/protocol.md](../docs/protocol.md) 참고.

## 대상 장치

| 장치 | 통신 | 비고 |
| --- | --- | --- |
| **Orion BMS 2** | CAN 2.0 (BMS2 는 CAN 포트 2개) | Orion BMS 유틸리티에서 브로드캐스트 메시지(ID·주기·내용: 팩 전압/전류/SOC/온도/셀 전압 등) 설정. OBD-II PID 요청 방식 조회도 지원 |
| **Sevcon Gen4** | CANopen | TPDO 로 속도/토크/전류/온도/폴트 송신, SDO 로 파라미터 읽기·쓰기. 설정은 DVT 소프트웨어. 비트레이트는 현재 설정값 확인 필요 |

## 통신 방식 권장

- 두 장치 모두 CAN 이므로 **CAN 버스를 수신(listen-only) 모드로 탭**해서 모든 프레임을 `CAN_RAW` 로 SD 기록·전송하는 것을 기본으로 합니다.
  - listen-only 모드는 버스에 ACK 조차 보내지 않으므로, 텔레메트리 장치가 고장나도 **파워트레인 CAN 에 영향을 주지 않습니다.**
  - 디코딩(DBC / CANopen 객체 사전)은 피트에서 하므로, 메시지 구성이 바뀌어도 펌웨어 수정이 필요 없습니다.
- BMS 와 모터 컨트롤러가 다른 버스/비트레이트에 있으면 CAN 채널 2개가 있는 MCU(STM32F4 의 CAN1/CAN2 등)를 사용.
- 하드웨어: STM32(bxCAN/FDCAN) 또는 ESP32(TWAI) + CAN 트랜시버(SN65HVD230 / TJA1051 등), 버스 종단 저항 위치 확인.

## 원격 "제어" 에 대한 안전 원칙

무선으로 파워트레인에 **쓰기**를 하는 기능은 신중하게 설계해야 합니다.

1. **주행 중에는 읽기 전용.** 파라미터 변경(SDO 쓰기 등)은 차량 정지 + Ready-to-Drive 해제 상태에서만 허용.
2. 토크/속도 명령 같은 **구동 제어는 무선 경로로 절대 보내지 않음.**
3. 모든 명령은 인증(공유 키 기반 HMAC) + seq 기반 재전송 공격 방지 + 피트 쪽 명시적 확인.
4. 통신 두절 시 어떤 명령도 "진행 중" 상태로 남지 않도록(페일세이프) 설계.
5. **FSK 규정(원격 제어/무선 통신 관련 조항)** 과 기술검차 요구사항을 먼저 확인.

## 폴더 구조 (예정)

```
wireless-powertrain-control/
├── firmware/      # CAN 수집 노드 (STM32 또는 ESP32)
├── gateway/       # 차량 무선 게이트웨이 (ESP32) — 주행 로거와 공유 가능
├── pit/           # 피트 수신기 (Python / Raspberry Pi)
└── docs/          # CAN ID 표, DBC, 배선도
```
