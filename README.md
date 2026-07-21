# ESP32-S3 Drone Firmware — Rev 1.1

> WDT 문제를 완화하고 실제 1 kHz 비행 제어 주기를 달성한 버전입니다.

## 프로젝트 개요

Rev 1.1은 Rev 1.0의 통합 기능을 유지하면서 ARMED 실행 흐름을 단계별 파이프라인으로 분리한 실시간 구조 개선 버전입니다. 실행 시간 보호 로직과 부하 분산을 통해 1 kHz 제어 주기를 확보하는 데 초점을 맞췄습니다.

## 개발 환경

| 항목 | 내용 |
| --- | --- |
| 비행 제어 보드 | Seeed Studio XIAO ESP32-S3 |
| 프레임워크 | ESP-IDF 6.0 |
| IMU | ICM-42688-P, SPI |
| 조종기 통신 | ESP-NOW |
| 모터 프로토콜 | RMT 기반 DSHOT |
| 언어 | C / C++ |

## Rev 1.0 대비 주요 변경

- WDT guard 및 실행 시간 보호 로직 추가
- ARMED 관련 코드를 기능별 파일과 단계별 함수로 분리
- 파이프라인 형태로 실행 주기와 연산 부하 분산
- TPA 감쇠 곡선 개선 시도
- 제어 루프 실행 시간과 task starvation 가능성 점검

## 파이프라인 구조

```text
Command/Safety -> Sensor/Attitude -> PID -> Compensation
               -> Mixer/Output shaping -> DSHOT
```

기능 분리는 이후 버전에서 서로 다른 주기로 실행되는 multi-rate 구조의 기반이 되었습니다.

## 검증 결과

- 실제 제어 주기 1 kHz를 달성했습니다.
- Rev 1.0에서 발생하던 WDT 문제를 완화했습니다.
- 분리된 ARMED 파이프라인을 통해 실행 부하를 추적하기 쉬워졌습니다.

## 알려진 한계 및 회귀

- 2 kHz 제어 주기 달성에는 실패했습니다.
- 높은 실행 주기에서는 WDT 또는 task starvation 가능성이 남아 있습니다.
- 리팩터링 과정에서 TPA 출력 적용 코드가 소실되었습니다. 설정 일부는 남았으며 실제 적용 코드는 Rev 1.3에서 복구됩니다.
- Atomic index와 double buffer를 사용한 snapshot은 빠른 읽기/쓰기 시점에 데이터 일관성 문제가 발생했습니다.
- RMT DSHOT의 blocking 실행 시간이 고속 제어 주기의 주요 제약입니다.

## 버전 위치

Rev 1.1은 1 kHz 실시간 구조를 확립한 버전입니다. SharedSnapshot과 multi-rate 2 kHz 제어는 Rev 1.2에서 적용됩니다.

---

# ESP32-S3 Drone Firmware — Rev 1.1

> This revision mitigates watchdog issues and achieves an actual 1 kHz flight-control rate.

## Overview

Rev 1.1 retains the integrated functionality of Rev 1.0 while splitting the ARMED execution flow into staged pipeline functions. Its primary goal is to secure a 1 kHz control rate through execution-time guards and distributed processing load.

## Development Environment

| Item | Details |
| --- | --- |
| Flight controller | Seeed Studio XIAO ESP32-S3 |
| Framework | ESP-IDF 6.0 |
| IMU | ICM-42688-P over SPI |
| Control link | ESP-NOW |
| Motor protocol | RMT-based DSHOT |
| Languages | C / C++ |

## Main Changes from Rev 1.0

- Added watchdog guards and execution-time protection
- Split ARMED processing into feature-specific files and staged functions
- Distributed update rates and processing load through a pipeline structure
- Experimented with an improved TPA attenuation curve
- Investigated loop execution time and task-starvation risks

## Pipeline Structure

```text
Command/Safety -> Sensor/Attitude -> PID -> Compensation
               -> Mixer/Output shaping -> DSHOT
```

This separation became the foundation of the multi-rate pipeline introduced in later revisions.

## Verified Results

- Achieved an actual 1 kHz control rate.
- Mitigated the watchdog issue observed in Rev 1.0.
- Improved visibility into execution load through the separated ARMED pipeline.

## Known Limitations and Regressions

- A 2 kHz control rate was not achieved.
- Watchdog or task-starvation risks remained at higher update rates.
- The TPA output application was lost during refactoring. Some configuration remained, and the output path was restored in Rev 1.3.
- Snapshots based on atomic indices and double buffering showed consistency problems during fast concurrent reads and writes.
- Blocking RMT DSHOT execution remained a major constraint on higher control rates.

## Revision Context

Rev 1.1 establishes the 1 kHz real-time structure. Rev 1.2 introduces critical-section snapshots and a 2 kHz multi-rate pipeline.