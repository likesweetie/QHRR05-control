# CAN HAL 동작 규약

실행·GUI·설정 위치·SHM 작성 규칙은 [README](../README.md)를 기준으로 합니다. 이 문서는 패킷 처리, 계측 해석과 후속 영점 보정 결정을 관리합니다.

## 설정과 통신 경로

- [daemon 설정](../config/daemon/daemon.yaml): 공유 영역 이름 `shm_name`과 자식 실행 인자. `"{shm_name}"`은 daemon이 치환합니다.
- [HAL 설정](../config/can_hal/can_hal.yaml): 인터페이스·실행/통계 주기·점유율 기준 비트레이트·관절별 CAN ID와 모터 설정 경로.
- [시뮬레이션 설정](../config/simulation/simulation.yaml): 실제 Kt·감속비·전기각과 Iq 센서 오차. 시뮬레이터의 `--simulation-config`로 전달하며 HAL은 읽지 않습니다.
- [모터 규약](../config/motor/ak10_9_v3.yaml): V_MAX/T_MAX·Kt_out·감속비·Kd scale. `motor_config` 경로는 HAL YAML 디렉터리 기준입니다.
- HAL은 `can_hal HAL_CONFIG.yaml SHM_NAME`, 가상 모터는 `simulation --hal-config HAL_CONFIG.yaml`로 같은 설정을 받습니다. 각 생성자가 직접 읽고, 누락·불일치는 명시적으로 실패합니다.

모터 명령·피드백은 Linux SocketCAN의 `PF_CAN/SOCK_RAW/CAN_RAW`를 통과합니다. 기본 런처는 호스트 vcan0를 준비하며 네임스페이스 생성은 명시적 `--isolated-vcan` 디버깅에서만 수행합니다. daemon은 공유 영역을 생성·초기화하고 모든 자식 종료 후 해제합니다. 각 프로그램에 별도 터미널을 열고 고정 통계 영역과 로그 영역을 나눕니다. 표시 주기·로그 저장·SHM 지표의 정의와 계측 한계는 [README의 관측 설명](../README.md#프로세스-터미널과-shm-통계)을 참고하십시오. HAL·가상 모터는 기존 영역에 연결하며 소켓은 각각 소유합니다. HAL 설정의 예전 `shm_name`은 오류로 거부합니다. 직접 HAL을 실행할 때도 daemon이 만든 영역 이름을 인자로 지정해야 합니다.

프로토콜 구현 기준은 [OpenRobot MIT v17](https://dongilc.github.io/openrobot-motor-tool/)이며 [AK10-9 V3.0 KV60](https://www.cubemars.com/product/ak10-9-v3-0-kv60-robotic-actuator.html) 프로파일을 사용합니다. CubeMars 기본 펌웨어와의 호환과 실물 드라이버 동작은 검증 전입니다.

## 패킷과 상태 전이

| 명령 | HAL / 가상 모터 동작 |
| --- | --- |
| C4 | V_MAX/T_MAX·Kt_out·감속비·Kd scale 조회·일치 확인 |
| C1 | 프로파일 검증 후 enable 요청·확인 |
| C0 | MIT 목표 송신, 상대각·속도·Iq·온도 응답 |
| C2 | release. 이후 C0 송신 중단, 종료 시에도 송신 |

ID 1/2/3은 SID 0x141/0x142/0x143에 대응합니다. 아직 검증되지 않은 C4와 미확인 C1은 100 HAL 주기마다 재요청합니다. 이는 준비 확인용 반복이며 응답 지연 감시가 아닙니다. 모터 응답이 없어도 제어 루프는 설정 주기로 계속 동작합니다.

[control_types.hpp](../src/ipc/control_types.hpp)의 `MitCommand`는 position·velocity·kp·kd·tau_ff를 가집니다. enable 요청은 `MotorData.command_enabled[i]`로 분리합니다. SHM과 TaskController는 OpenRobot 코덱을 참조하지 않고, 코덱이 이 공통 타입을 참조합니다. position은 출력축 상대 좌표 rad입니다. Homing은 보간 q/dq와 자체 PD 게인을 보내며, Gravity/Magnetic은 계산한 토크를 다음 위치 목표로 환산합니다.

```text
q_motor_target = q_joint - position_offset + (tau + kd_motor * dq) / kp_motor
```

kd 항은 이미 토크 계산에 포함한 감쇠를 모터 PD에서 중복 적용하지 않기 위한 항입니다. 가상 모터의 내부 PD 출력은 모터 프로파일 토크 한계와 XML actuator 한계의 교집합으로 제한합니다.

**tau_ff는 0만 허용하며 비영·비유한 입력은 명시적으로 거부합니다.** 송신 바이트는 논리적 0인 0x80으로 고정합니다. 가상 모터는 이 값만 허용하고 FF 없이 PD를 계산합니다. 실물 드라이버가 대칭 8-bit 필드를 수치로 복원하면 반 LSB 오차가 생길 수 있어 zero-code 해석은 실기 캡처로 확인해야 합니다.

피드백은 v14 이후 signed 출력축 MIT 좌표로 해석합니다. 구형 입력축 표를 자동 감지하거나 대체하지 않습니다. 위치는 int16 × 12.56 / 32768 rad, 속도는 int16 deg/s를 rad/s로 변환하며, Iq는 int16 × 33 / 2048 A입니다. 가상 모터는 센서 OFF일 때 직전 물리 스텝의 실제 actuator torque / C4 Kt를 반환합니다. 센서 ON일 때는 실제 모터 Kt·감속비로 true Iq를 역산한 뒤 센서 오차를 적용해 반환합니다. CAN 패킷 형식은 그대로이며 토크 필드를 추가하지 않습니다. HAL은 실제 CAN Iq와 검증한 C4 Kt로 SHM의 측정 토크를 계산합니다. 조정 항목과 물리 시각 기반 샘플링은 [README](../README.md#가상-iq-센서)를 참고하십시오.

수동 명령은 TaskController를 실행하지 않는 별도 디버깅 세션에서만 사용합니다. 기본 세션에서는 TaskController가 명령을 계속 발행합니다.

```bash
./build/can_hal --target /qhrr05 0 0.2 0.0 40.0 0.3
./build/can_hal --release /qhrr05
```

## 터미널 채널 점유율

`BUS LOAD est.`는 통계 전용 수신 소켓이 관측한 Classic CAN 프레임의 점유율을 바·백분율로 표시합니다. 모터 3개의 ID 필터와 독립적이므로 같은 채널의 다른 ID도 포함합니다. 로컬 송신도 loopback으로 한 번만 세며 TX/RX 누계를 더해 중복 집계하지 않습니다. 통계 스레드가 지속 수신하므로 제어 루프에 수집 작업을 추가하지 않습니다.

```text
BUS LOAD est. [############..................] 40.5% | reference 1000000 bit/s
```

HAL YAML의 `bus_load_bitrate`는 1~1,000,000 bit/s 범위의 필수 표시 기준입니다. 실제 인터페이스 속도를 설정하거나 자동 조회하는 값이 아닙니다. vcan은 이 기준 속도의 물리 버스로 환산한 값을 표시합니다. `statistics_hz`마다 실제 경과 시간으로 갱신합니다. 기본값은 0.5 Hz(2초)이며 관리 터미널의 화면 재표시 주기는 daemon의 `monitoring.display_interval_sec`입니다.

```text
점유율 [%] = 100 × 관측 프레임의 추정 비트 합 / (기준 bit/s × 관측 시간 [s])
표준 ID 프레임 = 55 + 10 × 데이터 바이트 수 [bit]
확장 ID 프레임 = 80 + 10 × 데이터 바이트 수 [bit]
```

비트 수는 [can-utils의 Classic CAN 최대 스터핑 계산 기준](https://github.com/linux-can/can-utils/blob/master/canframelen.c)을 사용하며 ACK·EOF·intermission을 포함합니다. RTR은 DLC와 관계없이 데이터 길이 0으로 계산합니다. 실제 스터핑 수·오류/재전송의 물리 시간은 복원하지 않으므로 계측 장비의 실측 점유율과는 차이가 있습니다. CAN FD는 집계 대상이 아닙니다.

60%부터 노랑, 80%부터 빨강으로 표시합니다. 100% 초과 수치는 그대로 유지하며 바만 최대 길이로 제한하고 `OVER CAPACITY`를 표시합니다. 수신 큐 누락은 `monitor drops`와 `INCOMPLETE`로 노출하며 오류 알림 수를 별도로 표시합니다. 다른 송신 프로그램이 로컬 loopback을 끄거나 커널/장치에서 프레임을 누락하면 관측값에 포함할 수 없습니다. 점유율·관측 누락으로 제어 명령을 바꾸거나 응답 지연 판정을 추가하지 않습니다.

## SHM과 계측 해석

`shared_memory.hpp`에 `MotorData`·`ModelData`·`ControlData`·`SimulationData`와 동기화·더블버퍼 배치를 함께 정의합니다. `control_types.hpp`에는 재사용하는 명령·피드백·Magnetic 값 타입만 둡니다. `MotorData`에는 축별 배열과 공통 통계를 직접 둡니다. command/feedback 발행 번호는 독립적이며 `received_sequence[i]`는 축별 응답 번호입니다. HAL은 잠금 아래 최신 슬롯의 피드백·측정 토크·CAN 배열·공통 통계만 갱신해 제어기 명령을 보존합니다.

`MotorProfile` 구조체와 SHM의 프로파일 복사본은 제거했습니다. HAL은 YAML에서 기대 C4 바이트와 인코딩 범위를 초기화하고 모터의 실제 C4 응답과 비교합니다. 불일치는 enable을 거부하는 명시적 오류입니다. ROM 값을 쓰거나 SHM 초기값으로 모터 설정을 대체하지 않습니다. 가상 모터도 자신의 C4 응답을 로컬에서 보관하고 CAN으로 응답하며 SHM의 설정값을 참조하지 않습니다.

HAL은 확인한 C4 응답의 Kt를 프로세스 내부에 보관하고 `motor_torque[i] = Iq * Kt`를 피드백과 함께 발행합니다. 제어기는 Kt나 모터 설정을 읽지 않습니다. 직접 토크 디버깅(`--without-can`)은 MuJoCo actuator torque를 전달하고 전류는 계측하지 않습니다. 이 모드의 current 필드는 사용하지 않습니다.

수동 `--target`은 유한한 목표 값을 SHM에 발행합니다. 프로토콜 범위 검증과 실제 패킷 전송은 실행 중인 HAL이 수행합니다.

모델 M/h/J·중력·발끝 정보는 CAN으로 측정한 관절 상태에서 별도 scratch `mjData`로 계산합니다. G/M의 `gravity_only`는 순수 중력과 실제 속도의 전체 bias 중 보상항을 선택합니다. 중력 계산용 `mjData`는 초기화 때 한 번 만들고, qvel=0에서 `mj_fwdPosition()`·`mj_fwdVelocity()`를 호출해 재사용합니다. 이 선택은 CAN 명령 형식·물리 마찰·RFOB 모델을 바꾸지 않습니다. TaskController의 q/dq/토크와 차분 가속도도 CAN 피드백에서 얻습니다. 물체 위치·접촉/해제 이벤트는 시뮬레이션 SHM을 사용하며 GT 힘은 RFOB 검증용입니다.

```bash
./build/shm_logger --can /qhrr05 /tmp/can-telemetry-new.csv
```

- CAN CSV의 age·관측 간격·SHM 발행 누락 평가는 로거에서만 수행합니다. 미수신 축의 age/interval은 빈 칸입니다.
- 최신 목표 sequence와 HAL이 소비한 목표 sequence는 다를 수 있습니다. 패킷에 sequence가 없어 요청별 왕복 지연·전체 CAN 손실·정확한 적용 시점을 식별하지 않습니다.
- 일반 CSV는 simulation/control/motors의 최신 완성본을 함께 읽습니다. `state_seq`는 소비한 모델 프레임 번호이고 `control_latency_us`는 모델 발행에서 명령 발행까지의 시간입니다.
- `sim_contact_event_time`은 시뮬레이션 시간, RFOB `event_time`은 독립 제어 시간입니다.

HAL이 소유한 SHM은 정상 종료 때 해제합니다. SIGKILL 뒤 남은 객체나 크기가 다른 객체는 자동 재사용·삭제하지 않고 충돌로 보고합니다. SHM 배치를 변경하면 모든 프로그램을 새 바이너리로 재시작합니다.

## 스토퍼 영점 보정 — 미구현

아래는 **미구현 설정안**입니다. 관절마다 한쪽 기준 스토퍼를 `lower` 또는 `upper`로 선택하고, 알려진 하드웨어 기준각을 설정합니다. 순서도 설정으로 변경합니다. 예시 각도는 현재 XML의 하한에서 가져온 **시뮬레이션 초기값**이며 실제 기구 측정값이 아닙니다.

```yaml
homing:
  calibration:
    order: [KP, HP, HR]
    joints:
      HR: {stop_side: lower, stop_angle_rad: -1.0472}
      HP: {stop_side: lower, stop_angle_rad: -1.5708}
      KP: {stop_side: lower, stop_angle_rad: -2.7227}
```

TaskController가 런타임 보정값을 소유합니다. 방향 `s=±1`, 상대 엔코더값 `r`, 접촉 시 측정값 `r_stop`, 알려진 스토퍼 각도 `q_stop`에 대해 다음 관계를 사용합니다.

```text
offset   = q_stop - s * r_stop
q_joint  = s * r + offset
r_target = s * (q_target - offset)
```

낮은 kp/kd·제한된 위치 오차·느린 위치 목표로 접근하고, 새 피드백에서 정지를 확인한 뒤 후퇴/재접근 재현성을 검사합니다. 보정 후 스토퍼에서 물러나 일반 자세 호밍으로 이어갑니다. tau_ff로 누르거나 절대 엔코더·C3 원점 재설정에 의존하지 않습니다. 이전 세션 offset을 자동 재사용하지 않으며, 알려진 원점·세션 변경 시 재호밍합니다.

오래된 피드백을 반복해서 읽은 것만으로 접촉을 확정하지 않습니다. 센서리스 정지는 장애물·마찰·중력 평형과 구별되지 않을 수 있으므로 실제 기준각과 비어 있는 탐색 경로가 필요합니다. 거리·시간 한계와 재현성 실패는 호밍 실패로 처리하며, CAN 응답 지연 감시를 추가하지 않습니다.
