# ESP32-S3 Drone Firmware — Rev 1.3-a

> 현재 개발 버전입니다. ESP-NOW를 ELRS UART/CRSF로 교체하고 명령 안전 처리와 telemetry를 통합했습니다.

## 프로젝트 개요

이 프로젝트는 Seeed Studio XIAO ESP32-S3 기반 2.5인치 쿼드콥터 비행 제어 펌웨어입니다. Rev 1.3-a는 Rev 1.2의 multi-rate 제어 구조를 유지하면서 조종 링크를 ESP-NOW에서 ELRS/CRSF로 전환한 버전입니다. 장거리 통신 가능성, 명령 timeout, 스위치 edge 처리 및 제한된 CRSF telemetry를 실제 비행 제어 파이프라인에 통합하는 데 초점을 맞췄습니다.

## 현재 상태

- 현재 개발 단계: **Rev 1.3-a — ELRS/CRSF integration**
- 빌드: 확인됨
- 목표 PID 주기: 2 kHz
- ARMED 상태 실측 주기: 약 1880–1905 Hz
- Motor/DSHOT 목표 주기: 1 kHz
- 실제 RadioMaster/ELRS 송수신기 환경의 최종 채널 검증: 진행 필요

## 개발 환경

| 항목 | 내용 |
| --- | --- |
| 비행 제어 보드 | Seeed Studio XIAO ESP32-S3 |
| 프레임워크 | ESP-IDF 6.0 |
| IMU | ICM-42688-P, SPI, 8 kHz ODR |
| 조종기 통신 | ELRS UART / CRSF |
| 모터 프로토콜 | RMT 기반 DSHOT |
| 제어 모드 | RATE/ACRO, ANGLE/SELF-LEVEL |
| 언어 | C / C++ |

## CRSF 구현 범위

| 방향 | Frame | 용도 |
| --- | --- | --- |
| RX | `0x16 RC Channels Packed` | 16채널 조종 입력 수신 및 정규화 |
| RX | `0x14 Link Statistics` | 링크 상태 수신 |
| TX | `0x08 Battery Sensor` | 필터링된 전압과 배터리 상태 전송 |
| TX | `0x21 Flight Mode` | 현재 비행 모드 전송 |

GPS frame(`0x02`)은 현재 범위에서 제외했습니다.

## Rev 1.2 대비 주요 변경

- CRSF 16채널 decoding 및 stick 값 정규화
- ARM/DISARM, flight mode, emergency kill switch 처리
- 최신 `ControlPacket` timestamp 기반 통신 timeout 판정
- Control sequence를 이용한 최신 명령 cache 및 command edge/repeat 처리
- 송신기 재시작을 전제로 하던 ESP-NOW sequence reset 예외 제거
- CRSF telemetry 오류를 비행 중단으로 연결하지 않는 best-effort 처리
- 필터링된 배터리 전압과 잔량을 battery telemetry에 반영
- `BatteryMonitor::Update()`에 실제 누적 검사 시간 전달
- 초기 switch 상태를 동기화하고 ARM LOW 확인 후 `LOW -> HIGH` edge에서만 ARM 허용
- 공중 또는 LANDING 상태에서 일반 DISARM 요청 차단
- Emergency stop은 상태와 관계없이 모터를 즉시 차단하고 DISARMED로 전환
- Rev 1.1 리팩터링 중 소실되었던 TPA 출력 적용 복구

## 현재 ARMED 파이프라인

1. **Command / Safety / Flight state** — CRSF snapshot, timeout, ARM/DISARM, soft landing, emergency stop, mode, target 및 failsafe 처리
2. **Sensor / Estimation** — IMU LPF, quaternion predict, gated gravity correction, attitude/vertical state 및 tilt safety 처리
3. **Attitude control** — angle-to-rate 변환, rate PID, authority scaling, yaw priority/boost 및 TPA
4. **Thrust compensation** — thrust curve, battery voltage compensation, high-throttle limit 및 vertical-speed damping
5. **Mixer / Motor output** — quad mixing, bias/multiplier, normalization, DSHOT 변환 및 RMT 출력

## Multi-rate 실행 주기

| 처리 항목 | 목표 또는 현재 설정 |
| --- | ---: |
| Main ARMED control / IMU fast processing / PID | 2 kHz target |
| 자세 및 상태 추정 | 1 kHz |
| Motor / DSHOT 출력 | 1 kHz |
| Command / target 처리 | 500 Hz |
| 느린 compensation/cache | 250 Hz |
| 배터리 모니터링 | 20 Hz |
| CRSF telemetry 합계 | 10 Hz |
| Battery / Flight mode telemetry | 각각 5 Hz |

## 측정 결과

- ARMED 상태에서 약 1880–1905 Hz가 측정되었습니다.
- TPA 적용 전후의 주기 차이는 약 2–3 Hz로, 전체 병목에 미치는 영향은 작았습니다.
- RMT DSHOT 출력은 약 256 us로 여전히 가장 큰 시간 제약 중 하나입니다.

## 확인이 필요한 항목

- ELRS 활성 상태에서 평균 주기, 최대 실행 시간 및 500 us deadline 초과 횟수 측정
- 실제 RadioMaster/ELRS 환경에서 채널 번호, 방향, 중심값 및 endpoint 검증
- `CMD_SOFT_LANDING`과 `CMD_LEVEL_CALIBRATE` AUX 채널 최종 매핑
- 수신 중인 Link Statistics의 비행 제어 활용 여부 결정
- 통신 거리 및 실제 비행 환경 검증

## 알려진 한계

- 현재 blocking RMT DSHOT 구조에서는 4 kHz 제어가 현실적으로 어렵습니다.
- 기능을 여러 주기로 분산했지만 2 kHz main loop의 시간 여유가 작습니다.
- Optical Flow와 ToF가 없어 XY 위치 및 고도 유지 기능이 없습니다.
- CRSF telemetry는 flight-critical 경로가 아닌 best-effort 방식입니다.

## 다음 단계 — Rev 1.3-b

Optical Flow와 ToF 인터페이스, 센서 유효성/timeout, XY 속도 및 높이 추정, hover 보정 모드를 추가할 예정입니다. 바닥 무늬·조명·반사율과 ESP32 연산 자원 제약을 함께 검증해야 합니다.

---

# ESP32-S3 Drone Firmware — Rev 1.3-a

> The current development revision. It replaces ESP-NOW with ELRS UART/CRSF and integrates command safety handling and telemetry.

## Overview

This project is flight-control firmware for a 2.5-inch quadcopter based on the Seeed Studio XIAO ESP32-S3. Rev 1.3-a retains the Rev 1.2 multi-rate control architecture while migrating the control link from ESP-NOW to ELRS/CRSF. The revision focuses on long-range communication potential, command timeouts, safe switch-edge handling, and limited CRSF telemetry integrated into the flight-control pipeline.

## Current Status

- Development stage: **Rev 1.3-a — ELRS/CRSF integration**
- Build: confirmed
- Target PID rate: 2 kHz
- Measured ARMED rate: approximately 1880–1905 Hz
- Motor/DSHOT target rate: 1 kHz
- Final channel validation with physical RadioMaster/ELRS hardware: pending

## Development Environment

| Item | Details |
| --- | --- |
| Flight controller | Seeed Studio XIAO ESP32-S3 |
| Framework | ESP-IDF 6.0 |
| IMU | ICM-42688-P over SPI, 8 kHz ODR |
| Control link | ELRS UART / CRSF |
| Motor protocol | RMT-based DSHOT |
| Flight modes | RATE/ACRO and ANGLE/SELF-LEVEL |
| Languages | C / C++ |

## CRSF Scope

| Direction | Frame | Purpose |
| --- | --- | --- |
| RX | `0x16 RC Channels Packed` | Receive and normalize 16 RC channels |
| RX | `0x14 Link Statistics` | Receive link status |
| TX | `0x08 Battery Sensor` | Transmit filtered voltage and battery state |
| TX | `0x21 Flight Mode` | Transmit the current flight mode |

The GPS frame (`0x02`) is intentionally excluded from the current scope.

## Main Changes from Rev 1.2

- Decoded 16 CRSF channels and normalized stick values
- Added ARM/DISARM, flight-mode, and emergency-kill switch handling
- Determined communication timeout from the latest `ControlPacket` timestamp
- Added command caching and edge/repeat handling based on the control sequence
- Removed ESP-NOW sequence-reset exceptions intended for transmitter restarts
- Treated CRSF telemetry as best effort so telemetry errors do not stop flight control
- Reported filtered battery voltage and remaining capacity through telemetry
- Passed actual accumulated inspection time to `BatteryMonitor::Update()`
- Synchronized initial switch states and allowed ARM only on a `LOW -> HIGH` edge after confirming ARM LOW
- Blocked ordinary DISARM requests while airborne or in LANDING state
- Made emergency stop cut motor output immediately and enter DISARMED regardless of flight state
- Restored the TPA output path lost during the Rev 1.1 refactor

## Current ARMED Pipeline

1. **Command / Safety / Flight state** — CRSF snapshot, timeout, ARM/DISARM, soft landing, emergency stop, mode, target, and failsafe handling
2. **Sensor / Estimation** — IMU LPF, quaternion prediction, gated gravity correction, attitude/vertical state, and tilt safety
3. **Attitude control** — angle-to-rate conversion, rate PID, authority scaling, yaw priority/boost, and TPA
4. **Thrust compensation** — thrust curve, battery-voltage compensation, high-throttle limiting, and vertical-speed damping
5. **Mixer / Motor output** — quad mixing, bias/multipliers, normalization, DSHOT conversion, and RMT output

## Multi-rate Schedule

| Processing stage | Target or current setting |
| --- | ---: |
| Main ARMED control / IMU fast processing / PID | 2 kHz target |
| Attitude and state estimation | 1 kHz |
| Motor / DSHOT output | 1 kHz |
| Command / target processing | 500 Hz |
| Slow compensation/cache | 250 Hz |
| Battery monitoring | 20 Hz |
| Total CRSF telemetry | 10 Hz |
| Battery / flight-mode telemetry | 5 Hz each |

## Measured Results

- The measured ARMED rate is approximately 1880–1905 Hz.
- Enabling TPA reduced the measured rate by only about 2–3 Hz, indicating negligible impact on the main bottleneck.
- RMT DSHOT output still takes approximately 256 us and remains one of the largest timing constraints.

## Items to Validate

- Measure average rate, worst-case execution time, and count of missed 500 us deadlines with ELRS enabled
- Verify channel indices, directions, center values, and endpoints with physical RadioMaster/ELRS hardware
- Finalize AUX channel mapping for `CMD_SOFT_LANDING` and `CMD_LEVEL_CALIBRATE`
- Decide whether received Link Statistics should affect flight-control behavior
- Validate communication range and behavior in real flight conditions

## Known Limitations

- A 4 kHz control rate is impractical with the current blocking RMT DSHOT implementation.
- Timing margin remains small in the 2 kHz main loop despite distributing features across multiple rates.
- XY position hold and altitude hold are unavailable without Optical Flow and ToF sensors.
- CRSF telemetry is best effort and is not part of the flight-critical path.

## Next Stage — Rev 1.3-b

The planned next stage adds Optical Flow and ToF interfaces, sensor validity and timeout handling, XY velocity and height estimation, and hover-correction modes. Floor texture, lighting, surface reflectivity, and ESP32 processing limits must be evaluated together.