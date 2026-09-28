# common/protocol

AERO 프레임 프로토콜 v1 구현. 명세는 [docs/protocol.md](../../docs/protocol.md).

| 경로 | 용도 | 테스트 |
| --- | --- | --- |
| `c/` | 차량 MCU(STM32/ESP32)용 C99, 동적 할당 없음 | `make -C common/protocol/c test` |
| `python/` | 피트 수신기·분석 도구용, 표준 라이브러리만 사용 | `cd common/protocol/python && python3 -m unittest` |

두 구현은 같은 테스트 벡터로 바이트 단위 호환성을 검증합니다.
