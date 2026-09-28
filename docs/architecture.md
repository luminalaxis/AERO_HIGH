# AERO_HIGH 시스템 아키텍처 (초안)

## 목표

> 손실이 일어나더라도 이를 **정확히 감지**하고 **추후 보강**이 가능한 구조로, 데이터를 **온전히 피트에 전달**한다.

이를 위해 모든 노드가 다음을 지킵니다.

1. 모든 데이터는 [AERO 프레임](protocol.md)으로 만들고, **먼저 SD 에 기록한 뒤** 무선으로 보낸다.
2. 프레임마다 `(source_id, boot_id, seq)` 와 타임스탬프를 붙여 손실·재부팅·순서 뒤바뀜을 감지한다.
3. 피트는 빠진 seq 를 NACK 로 요청해 백필하고, 세션 후 SD 동기화로 최종 완성한다.
4. 무선 링크가 끊기거나 느려져도 **센서 수집과 SD 기록은 절대 멈추지 않는다** (수집 ↔ 전송 분리).

## 전체 구성 (권장안)

```
 ┌──────────────────── 차량 ────────────────────┐
 │                                              │
 │  [vehicle-data-logger]   STM32               │
 │   포텐셔미터×4 (ADC+DMA)                      │
 │   IMU 6축 (SPI)          ─► microSD (SDIO)   │
 │   GPS (UART, PPS)                            │──UART──┐
 │   지상고×4                                    │        │
 │   (서보×2~4, PWM)                             │        ▼
 │                                              │   [텔레메트리 게이트웨이]
 │  [wireless-powertrain-control] STM32/ESP32   │   ESP32(-S3), 외장 안테나
 │   CAN (listen-only) ◄── Orion BMS2           │──►  주 링크 (Wi-Fi/LTE)
 │                     ◄── Sevcon Gen4          │   + LoRa 보조 링크
 │                          ─► microSD          │──UART──┘   + RAM 링버퍼
 └──────────────────────────────────────────────┘
                         │ 무선 (≤300 m)
                         ▼
 [피트]  수신기(Raspberry Pi 또는 노트북) → GapTracker/NACK → 저장 → 피트 대시보드
```

- **수집 노드와 무선 게이트웨이를 분리**하면, 무선 스택(Wi-Fi 재연결, 버퍼링)이 센서 샘플링 타이밍에 영향을 주지 않습니다.
- 두 프로젝트가 **게이트웨이 하나를 공유**하면 차량에 무선 모듈이 한 세트만 있으면 됩니다.
- 초기 프로토타입은 ESP32 하나로 수집+전송을 같이 해도 되지만, 구조(프레임/SD/링버퍼)는 위와 동일하게 유지합니다.

## MCU 역할 추천

| 보드 | 추천 역할 | 이유 |
| --- | --- | --- |
| **STM32** (F4/G4 계열, 예: NUCLEO-F446RE, STM32F405) | 센서 수집 노드, CAN 노드 | 정확한 ADC+DMA, 타이머(PPS 캡처, 서보 PWM), SDIO, bxCAN/FDCAN 내장 |
| **ESP32 / ESP32-S3** (외장 안테나 모델) | 무선 게이트웨이 | Wi-Fi/ESP-NOW 내장, TWAI(CAN) 도 가능. 단 ADC 가 비선형이고 Wi-Fi 사용 시 ADC2 사용 불가 → 정밀 아날로그 측정에는 비추천 |
| **Raspberry Pi** | 피트 수신기/서버 | 리눅스, Python, 대시보드 연동. 차량 탑재는 부팅 시간(수십 초)·전원 차단 시 SD 손상 위험으로 비추천 (쓴다면 UPS HAT 필수) |
| **Arduino** | 초기 센서 동작 확인용 | 빠른 프로토타이핑. 최종 차량용으로는 성능/신뢰성 부족 |

## 개발 환경 추천

| 대상 | 도구 |
| --- | --- |
| STM32 | **STM32CubeMX**(핀/클럭/주변장치 설정, 코드 생성) + **CMake** + **VS Code**(STM32Cube for VS Code 확장). 미들웨어: FreeRTOS, FatFS (CubeMX 제공) |
| ESP32 | **VS Code + PlatformIO** (Arduino-ESP32 프레임워크, 필요 시 ESP-IDF API 직접 사용) |
| 피트 / 분석 / Raspberry Pi | **Python 3** (표준 라이브러리 기반 `aero_protocol`, 필요 시 FastAPI/MQTT) |
| 공통 프로토콜 | C99 (`common/protocol/c`) — STM32/ESP32 양쪽에 그대로 포함 |
| 버전 관리 / CI | Git 모노레포 + GitHub Actions (공통 프로토콜 테스트 자동 실행) |

모든 도구가 VS Code 하나에서 동작하므로 팀원 온보딩이 쉽습니다.

## SD 카드 기록 규칙

- 부팅마다 `boot_id` 를 +1 하고 새 파일 생성: `LOG_<source>_<boot_id>.aero` (프레임을 그대로 이어 쓴 바이너리).
- 파일을 **미리 크게 할당(pre-allocate)** 하고 512 B 배수 블록 단위로 기록 → 지연 스파이크 감소.
- 주기적으로(예: 0.5–1 s) `f_sync` → 전원이 갑자기 끊겨도 잃는 양이 최대 1초 이내. 잘린 마지막 프레임은 CRC 로 걸러짐.
- 가능하면 전원 차단 감지 + 슈퍼커패시터로 마지막 버퍼를 flush.
- 산업용(SLC/pSLC) microSD 권장, 진동 대비 카드 고정.

## 시간 동기화

- 주행 로거는 **GPS PPS** 를 타이머 입력 캡처로 받아 µs 타임스탬프를 GPS 시간에 맞춥니다.
- 파워트레인 노드는 게이트웨이의 `TIME_SYNC` 메시지로 보정합니다.
- 피트에서 두 노드의 데이터를 같은 시간축으로 정렬할 수 있게 됩니다.

## 무선 링크

[wireless-link-options.md](wireless-link-options.md) 참고. 요약: **주 링크(Wi-Fi 또는 LTE) + LoRa 보조 링크 + SD** 3중 구조를 권장하며, 트랙 실측 후 결정합니다.
