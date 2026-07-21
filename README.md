# ESP32-S3 Drone Firmware — Rev 1.2

> SharedSnapshot으로 데이터 일관성을 확보하고 multi-rate 구조로 2 kHz 제어를 달성한 실내 비행 기준 버전입니다.

## 프로젝트 개요

Rev 1.2는 고속 task 사이의 공유 데이터 구조와 전체 실행 주기를 재설계한 버전입니다. Atomic index 기반 double buffer를 `portMUX` critical section 기반 `SharedSnapshot<T>`로 교체하고, IMU ODR 및 ARMED 파이프라인을 multi-rate 방식으로 구성했습니다.

## 개발 환경

| 항목 | 내용 |
| --- | --- |
| 비행 제어 보드 | Seeed Studio XIAO ESP32-S3 |
| 프레임워크 | ESP-IDF 6.0 |
| IMU | ICM-42688-P, SPI, 8 kHz ODR |
| 조종기 통신 | ESP-NOW |
| 모터 프로토콜 | RMT 기반 DSHOT |
| 제어 모드 | RATE/ACRO, ANGLE/SELF-LEVEL |
| 언어 | C / C++ |

## Rev 1.1 대비 주요 변경

- ESP-NOW 및 IMU 공유 데이터에 `SharedSnapshot<T>` 적용
- Atomic read/write index와 double buffer 제거
- `portMUX` critical section으로 snapshot 읽기/쓰기의 일관성 확보
- IMU ODR을 1 kHz에서 8 kHz로 변경하고 최신 센서 데이터를 사용
- PID와 주요 제어를 2 kHz로 실행하는 multi-rate ARMED 파이프라인 구성
- 가속도 크기가 허용 범위에 있을 때만 자세 보정 수행
- ARM 전 정지 상태에서 gyro bias를 측정하여 IMU 입력에 적용
- LEVEL CALIBRATE를 통한 roll/pitch 자세 offset 지원

## 자세 추정

- Gyro와 `dt`로 quaternion 자세를 예측합니다.
- Accel norm이 엄격한 유효 범위에 있을 때만 중력 방향으로 roll/pitch를 보정합니다.
- Gyro bias는 ARM 전 정지 보정에서 결정하며 비행 중 온라인 갱신하지 않습니다.
- 코드의 클래스 이름은 EKF이지만, 실제 동작은 quaternion 기반 상보 필터에 가깝습니다.
- 공분산 `P`는 유지되지만 correction gain 계산에는 직접 사용되지 않습니다.

## Multi-rate 실행 주기

| 처리 항목 | 목표 주기 |
| --- | ---: |
| Main ARMED / IMU fast processing / PID | 2 kHz |
| 자세 및 일부 상태 추정 | 1 kHz |
| Motor / DSHOT 출력 | 1 kHz |
| Command / target 처리 | 500 Hz |
| 느린 보상값 계산 | 250 Hz |
| 배터리 및 통신 telemetry | 저속 주기 |

## 검증 결과

- 분할된 파이프라인으로 목표 제어 주기 2 kHz를 달성했습니다.
- 실내 비행과 자세 안정화 동작을 확인했습니다.
- SharedSnapshot 적용 후 고속 task 사이의 주기 및 데이터 전달 문제가 관찰되지 않았습니다.

## 알려진 한계

- RMT DSHOT 출력에 약 256 us가 소요됩니다. 4 kHz 주기는 250 us이므로 현재 blocking 출력 구조에서는 4 kHz 달성이 어렵습니다.
- PID 2 kHz와 Motor/DSHOT 1 kHz 구성도 실행 시간 여유가 크지 않습니다.
- Optical Flow와 ToF가 없어 XY 위치 및 고도 유지 기능이 없습니다.
- ESP-NOW 기반 통신의 장거리 안정성은 검증되지 않았으며 50 m 이상 시험 기록이 없습니다.

## 버전 위치

Rev 1.2는 ESP-NOW 기반 펌웨어의 안정적인 실내 동작 기준입니다. Rev 1.3-a에서는 제어 링크를 ELRS UART/CRSF로 교체합니다.

---

# ESP32-S3 Drone Firmware — Rev 1.2

> The indoor-flight baseline that improves data consistency with SharedSnapshot and achieves 2 kHz control through a multi-rate pipeline.

## Overview

Rev 1.2 redesigns shared data exchange and execution scheduling between high-rate tasks. It replaces the atomic-index double buffer with a `portMUX` critical-section-based `SharedSnapshot<T>`, increases the IMU ODR, and organizes the ARMED pipeline as a multi-rate system.

## Development Environment

| Item | Details |
| --- | --- |
| Flight controller | Seeed Studio XIAO ESP32-S3 |
| Framework | ESP-IDF 6.0 |
| IMU | ICM-42688-P over SPI, 8 kHz ODR |
| Control link | ESP-NOW |
| Motor protocol | RMT-based DSHOT |
| Flight modes | RATE/ACRO and ANGLE/SELF-LEVEL |
| Languages | C / C++ |

## Main Changes from Rev 1.1

- Applied `SharedSnapshot<T>` to ESP-NOW and IMU shared data
- Removed atomic read/write indices and double buffering
- Used `portMUX` critical sections for consistent snapshot reads and writes
- Increased the IMU ODR from 1 kHz to 8 kHz and consumed the latest sample
- Introduced a multi-rate ARMED pipeline with PID and primary control at 2 kHz
- Restricted attitude correction to valid accelerometer-magnitude conditions
- Measured gyro bias while stationary before ARM and applied it to IMU input
- Added roll/pitch attitude offsets through LEVEL CALIBRATE

## Attitude Estimation

- Quaternion attitude is predicted from gyro data and `dt`.
- Roll and pitch are corrected using gravity only while the accelerometer norm is within a strict valid range.
- Gyro bias is determined during stationary pre-arm calibration and is not updated online during flight.
- Although the class is named EKF, the implemented behavior is closer to a quaternion complementary filter.
- The covariance matrix `P` is retained but is not used directly to calculate the correction gain.

## Multi-rate Schedule

| Processing stage | Target rate |
| --- | ---: |
| Main ARMED / IMU fast processing / PID | 2 kHz |
| Attitude and partial state estimation | 1 kHz |
| Motor / DSHOT output | 1 kHz |
| Command / target processing | 500 Hz |
| Slow compensation calculation | 250 Hz |
| Battery and communication telemetry | Low-rate tasks |

## Verified Results

- Achieved the 2 kHz target control rate using the split pipeline.
- Verified stable indoor operation and attitude stabilization.
- No periodicity or data-transfer issue was observed between high-rate tasks after applying SharedSnapshot.

## Known Limitations

- RMT DSHOT output takes approximately 256 us. Because a 4 kHz period is 250 us, 4 kHz control is impractical with the current blocking output path.
- Timing margin remains limited even with PID at 2 kHz and Motor/DSHOT at 1 kHz.
- XY position hold and altitude hold are unavailable without Optical Flow and ToF sensors.
- Long-range ESP-NOW reliability has not been validated, including operation beyond 50 m.

## Revision Context

Rev 1.2 is the stable indoor baseline for the ESP-NOW firmware. Rev 1.3-a replaces the control link with ELRS UART/CRSF.