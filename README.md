# AERO_HIGH
DEVELOP AERO AND VEHICLE DATA LOGGING SYSTEM IN ORDER TO ENHANCE DRIVING INFORMATION ANALYSIS ABILITY

FSK(Formula Student Korea) 차량용 데이터 시스템 모노레포입니다.

**목표:** 손실이 일어나더라도 이를 정확히 감지하고 추후 보강이 가능한 구조로, 데이터를 온전히 피트에 전달한다.

## 구성

| 디렉터리 | 내용 |
| --- | --- |
| [`vehicle-data-logger/`](vehicle-data-logger/) | 센서(서스펜션·GPS·IMU) 기반 주행 정보 수집·SD 백업·전송 |
| [`wireless-powertrain-control/`](wireless-powertrain-control/) | Orion BMS2 / Sevcon Gen4 CAN 수집·SD 백업·무선 모니터링 및 제한적 제어 |
| [`common/protocol/`](common/protocol/) | 두 프로젝트 공통 AERO 프레임 프로토콜 (C: 차량 MCU, Python: 피트) |
| [`docs/`](docs/) | [아키텍처](docs/architecture.md) · [프로토콜](docs/protocol.md) · [무선 링크 비교](docs/wireless-link-options.md) |
| `pit-dashboard/` | 피트 대시보드(분석 도구) — 기존 완성본, 추후 이관 예정 |
