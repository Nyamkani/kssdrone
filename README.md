# ESP32-S3 Drone Firmware — Rev 1.0

> 최초의 전체 비행 제어 파이프라인을 통합하고 실내 기본 동작을 확인한 버전입니다.

## 프로젝트 개요

이 프로젝트는 Seeed Studio XIAO ESP32-S3를 비행 제어기로 사용하는 2.5인치 쿼드콥터 펌웨어입니다. Rev 1.0은 조종기 통신, IMU 자세 추정, PID 제어, 모터 믹싱 및 DSHOT 출력을 하나의 비행 가능한 시스템으로 처음 통합한 기준 버전입니다.

## 개발 환경

| 항목 | 내용 |
| --- | --- |
| 비행 제어 보드 | Seeed Studio XIAO ESP32-S3 |
| 프레임워크 | ESP-IDF 6.0 |
| IMU | ICM-42688-P, SPI |
| 조종기 통신 | ESP-NOW |
| 모터 프로토콜 | RMT 기반 DSHOT |
| 언어 | C / C++ |

## 주요 기능

- ESP-NOW 기반 조종 명령 수신 및 통신 failsafe
- SPI 기반 ICM-42688-P 인터페이스
- Gyro 적분과 가속도 중력 방향 보정을 사용하는 quaternion 자세 추정
- `RATE/ACRO` 및 `ANGLE/SELF-LEVEL` 비행 모드
- Rate/angle PID 제어기와 쿼드 모터 믹서
- 저스로틀 구간 PID authority scaling
- TPA(Throttle PID Attenuation)
- ARM, DISARM 및 비상 정지 처리
- RMT 기반 4채널 DSHOT 출력

## 제어 흐름

```text
ESP-NOW command -> IMU acquisition -> attitude estimation
                -> PID control -> motor mixing -> RMT DSHOT
```

## 검증 결과

- 전체 비행 제어 기능의 연결과 기본 동작을 확인했습니다.
- 실내에서 자세 안정화와 모터 제어가 동작함을 확인했습니다.

## 알려진 한계

- 목표 제어 주기 1 kHz를 안정적으로 달성하지 못했습니다.
- 실행 부하가 증가할 때 WDT가 발생할 수 있습니다.
- ARMED 상태의 처리가 하나의 큰 실행 흐름에 집중되어 있습니다.
- ESP-NOW 처리와 비행 제어 task 사이에 실행 시간 간섭 가능성이 있습니다.
- Optical Flow와 ToF가 없어 XY 위치 및 고도 유지 기능을 제공하지 않습니다.

## 버전 위치

Rev 1.0은 최초 통합 기준점입니다. 실시간 실행 구조와 WDT 대응은 Rev 1.1에서 개선됩니다.

---

# ESP32-S3 Drone Firmware — Rev 1.0

> The first revision to integrate the complete flight-control pipeline and verify basic indoor operation.

## Overview

This project is flight-control firmware for a 2.5-inch quadcopter built around the Seeed Studio XIAO ESP32-S3. Rev 1.0 is the initial baseline that combines radio communication, IMU-based attitude estimation, PID control, motor mixing, and DSHOT output into a flyable system.

## Development Environment

| Item | Details |
| --- | --- |
| Flight controller | Seeed Studio XIAO ESP32-S3 |
| Framework | ESP-IDF 6.0 |
| IMU | ICM-42688-P over SPI |
| Control link | ESP-NOW |
| Motor protocol | RMT-based DSHOT |
| Languages | C / C++ |

## Main Features

- ESP-NOW command reception and communication failsafe
- SPI interface for the ICM-42688-P
- Quaternion attitude estimation using gyro integration and accelerometer gravity correction
- `RATE/ACRO` and `ANGLE/SELF-LEVEL` flight modes
- Rate/angle PID controllers and quad-motor mixer
- Low-throttle PID authority scaling
- Throttle PID Attenuation (TPA)
- ARM, DISARM, and emergency-stop handling
- Four-channel DSHOT output using RMT

## Control Flow

```text
ESP-NOW command -> IMU acquisition -> attitude estimation
                -> PID control -> motor mixing -> RMT DSHOT
```

## Verified Results

- Verified integration and basic operation of the complete flight-control pipeline.
- Confirmed indoor attitude stabilization and motor control.

## Known Limitations

- The 1 kHz target control rate was not achieved reliably.
- Increased processing load could trigger the watchdog timer.
- ARMED-state processing remained concentrated in one large execution flow.
- ESP-NOW processing could interfere with flight-control execution time.
- XY position hold and altitude hold are unavailable without Optical Flow and ToF sensors.

## Revision Context

Rev 1.0 is the initial integration baseline. Rev 1.1 improves real-time scheduling and watchdog handling.