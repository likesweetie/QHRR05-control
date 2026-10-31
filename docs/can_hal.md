# CAN HAL 동작 규약

실행·GUI·설정 위치·SHM 작성 규칙은 [README](../README.md)를 기준으로 합니다. 이 문서는 패킷 처리, 계측 해석과 영점 보정 규약을 관리합니다.

## 설정과 통신 경로

- [daemon 설정](../config/daemon/daemon.yaml): 공유 영역 이름 `shm_name`과 자식 실행 인자. `"{shm_name}"`은 daemon이 치환합니다.
- [HAL 설정](../config/can_hal/can_hal.yaml): 인터페이스·실행/통계 주기·점유율 기준 비트레이트·관절별 CAN ID와 모터 설정 경로.
- [시뮬레이션 설정](../config/simulation/simulation.yaml): 실제 Kt·감속비·전기각과 Iq 센서 오차. 시뮬레이터의 `--simulation-config`로 전달하며 HAL은 읽지 않습니다.
- [모터 규약](../config/motor/ak10_9_v3.yaml): V_MAX/T_MAX·Kt_out·감속비·Kd scale. `motor_config` 경로는 HAL YAML 디렉터리 기준입니다.
- HAL은 `can_hal HAL_CONFIG.yaml SHM_NAME`, 가상 모터는 `simulation --hal-config HAL_CONFIG.yaml`로 같은 설정을 받습니다. 각 생성자가 직접 읽고, 누락·불일치는 명시적으로 실패합니다.

모터 명령·피드백은 Linux SocketCAN의 `PF_CAN/SOCK_RAW/CAN_RAW`를 통과합니다. 기본 런처는 준비된 물리 CAN을 사용하고, run_simulation.sh만 호스트 vcan0를 준비하며 네임스페이스 생성은 명시적 `--isolated-vcan` 디버깅에서만 수행합니다. daemon은 공유 영역을 생성·초기화하고 모든 자식 종료 후 해제합니다. 각 프로그램에 별도 터미널을 열고 고정 통계 영역과 로그 영역을 나눕니다. 표시 주기·로그 저장·SHM 지표의 정의와 계측 한계는 [README의 관측 설명](../README.md#프로세스-터미널과-shm-통계)을 참고하십시오. HAL·가상 모터는 기존 영역에 연결하며 소켓은 각각 소유합니다. HAL 설정의 예전 `shm_name`은 오류로 거부합니다. 직접 HAL을 실행할 때도 daemon이 만든 영역 이름을 인자로 지정해야 합니다.

프로토콜 구현 기준은 [OpenRobot MIT v17](https://dongilc.github.io/openrobot-motor-tool/)이며 [AK10-9 V3.0 KV60](https://www.cubemars.com/product/ak10-9-v3-0-kv60-robotic-actuator.html) 프로파일을 사용합니다. CubeMars 기본 펌웨어와의 호환과 실물 드라이버 동작은 검증 전입니다.

## 패킷과 상태 전이

| 명령 | HAL / 가상 모터 동작 |
| --- | --- |
| C4 | V_MAX/T_MAX·Kt_out·감속비·Kd scale 조회·일치 확인 |
| C1 | HAL은 프로파일 검증 후 enable 요청·확인. 가상 모터는 enable 대기로 진입하며 C0 수신 전 출력은 0 |
| C0 | enable 상태에서 MIT 목표 적용, MIT 원점 기준 상대각·속도·Iq·온도 응답. 해제 상태·제로셋 보호 구간에서는 목표 적용 없이 피드백만 응답 |
| C2 | release 및 목표 초기화. HAL은 이후 C0 송신 중단, 종료 시에도 C2 송신 |
| C3 | 가상 모터의 MIT 런타임 원점 설정. DATA[6:7] signed int16 LE, 출력축 0.01 deg/LSB, 응답에 오프셋 에코 |

ID 1/2/3은 SID 0x141/0x142/0x143에 대응합니다. 아직 검증되지 않은 C4와 미확인 C1은 100 HAL 주기마다 재요청합니다. 이는 준비 확인용 반복이며 응답 지연 감시가 아닙니다. 모터 응답이 없어도 제어 주기는 유지하지만, HAL의 설정 기한을 넘으면 출력 해제와 latched fault를 기록합니다.

가상 모터는 축마다 `Released → Enabled → MitControl` 상태를 가집니다. C1만으로는 토크가 발생하지 않고, 다음 유효 C0부터 목표를 유지하며 PD를 계산합니다. 이미 enable된 모터에 반복한 C1은 원점·목표를 바꾸지 않습니다. C2는 즉시 출력을 0으로 만들고 이전 목표를 지우므로 재-enable 후 새 C0가 필요합니다.

가상 모터의 시작 원점은 `simulation.yaml`의 `motor.require_zero_position_setup`으로 정합니다. **false**이면 원점이 보정된 모델 좌표로 시작하여 C0 위치 피드백이 MuJoCo 관절각과 일치합니다. **true**이면 축마다 독립적인 임의 엔코더 원점으로 시작하며 TaskController의 순차 스토퍼 탐색으로 런타임 `position_offset`을 보정합니다. 이 자동 절차는 C3/ROM 쓰기를 전송하지 않습니다. 임의 값은 모터 객체 생성 때 한 번만 정하고 매 스텝 다시 뽑지 않습니다. 현재 가상 모터의 두 시작 정책은 초기 원점을 이미 정하므로 C1에서 원점을 다시 잡지 않습니다. 펌웨어 문서의 미설정 상태 최초 진입 자동 캡처를 그대로 적용하면 enable 전 움직인 실제 관절과 호스트 원점이 어긋났기 때문에 이 시작 정책으로 분리했습니다.

C3으로 명시한 원점은 C2/C1 사이에도 보존합니다. 예를 들어 C3 offset=3000이면 현재 관절의 MIT 피드백이 +30°가 되며, MuJoCo qpos는 바꾸지 않습니다. C0 피드백과 PD 위치 오차는 같은 MIT 좌표를 사용합니다. MuJoCo qpos는 이미 출력축 rad이므로 감속비로 다시 나누지 않습니다. 알려진 관절각에서 C3 오프셋을 그 각도로 지정하면 모델 좌표와 맞출 수 있습니다. C3 offset=0은 현재 자세를 0도로 정하는 명령이며 실제 관절각의 자동 추정은 아닙니다.

C3는 기존 목표를 지우고 **시뮬레이션 시간 20ms** 동안 수신한 C0 목표를 버립니다. 보호 구간에도 피드백은 새 원점을 반영하며, 시간이 지난 뒤 수신한 새 C0부터 제어합니다. 일시정지 중에는 보호 시간이 줄지 않고, MuJoCo 시간 리셋 시 남은 보호 시간과 런타임 원점을 유지합니다. 가상 모터 객체 재생성(전원 재시작에 해당) 시에는 런타임 원점을 초기화합니다. C3는 모터를 enable하거나 ROM을 수정하지 않습니다.

현재 HAL의 자동 경로는 C4/C1/C0/C2이며 C3 송신용 SHM/CLI는 추가하지 않았습니다. C3는 외부 CAN 명령으로 가상 모터가 처리하고, HAL은 예상하지 않은 C3 응답을 기존 invalid-frame 로그에 표시합니다. 운용 중 원점을 바꾸는 호출자는 `MotorData.position_offset`의 관절 좌표 변환도 함께 관리해야 합니다. 시뮬레이터가 제로셋을 근거로 이 값을 자동 보정하지 않습니다. C5 ROM 쓰기, 물리적 엔코더 영점 명령 0x19, 모터 펌웨어의 fault clear 및 RMD 제어는 지원하지 않습니다. 아래의 HAL 통신 fault 해제는 호스트 상태의 해제이며 펌웨어 명령을 대신하지 않습니다. 지원하지 않는 명령은 명시적 오류입니다.

[control_types.hpp](../src/ipc/control_types.hpp)의 `MitCommand`는 position·velocity·kp·kd·tau_ff를 가집니다. enable 요청은 `MotorData.command_enabled[i]`로 분리합니다. SHM과 TaskController는 OpenRobot 코덱을 참조하지 않고, 코덱이 이 공통 타입을 참조합니다. position은 출력축 상대 좌표 rad입니다. Homing은 보간 q/dq와 자체 PD 게인을 보내며, Gravity/Magnetic/Tracking/Ball은 계산한 토크를 다음 위치 목표로 환산합니다.

```text
q_motor_target = q_joint - position_offset + (tau + kd_motor * dq) / kp_motor
```

kd 항은 이미 토크 계산에 포함한 감쇠를 모터 PD에서 중복 적용하지 않기 위한 항입니다. 가상 모터의 내부 PD 출력은 모터 프로파일 토크 한계와 XML actuator 한계의 교집합으로 제한합니다.

**tau_ff는 0만 허용하며 비영·비유한 입력은 명시적으로 거부합니다.** 송신 바이트는 논리적 0인 0x80으로 고정합니다. 가상 모터는 이 값만 허용하고 FF 없이 PD를 계산합니다. 실물 드라이버가 대칭 8-bit 필드를 수치로 복원하면 반 LSB 오차가 생길 수 있어 zero-code 해석은 실기 캡처로 확인해야 합니다.

피드백은 v14 이후 signed 출력축 MIT 좌표로 해석합니다. 구형 입력축 표를 자동 감지하거나 대체하지 않습니다. 위치는 int16 × 12.56 / 32768 rad, 속도는 int16 deg/s를 rad/s로 변환하며, Iq는 int16 × 33 / 2048 A입니다. 가상 모터는 센서 OFF일 때 직전 물리 스텝의 실제 actuator torque / C4 Kt를 반환합니다. 센서 ON일 때는 실제 모터 Kt·감속비로 true Iq를 역산한 뒤 센서 오차를 적용해 반환합니다. CAN 패킷 형식은 그대로이며 토크 필드를 추가하지 않습니다. HAL은 실제 CAN Iq와 검증한 C4 Kt로 SHM의 측정 토크를 계산합니다. 조정 항목과 물리 시각 기반 샘플링은 [제어·시뮬레이션 설정](control.md#가상-iq-센서)를 참고하십시오.

직접 SHM에 쓰던 HAL --target/--release 경로는 제거했습니다. 모든 운영 명령은 Input을 통해 승인받습니다.

```bash
./build/input --command /qhrr05 arm
./build/input --command /qhrr05 release
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

`shared_memory.hpp`에 `MotorData`·`ModelData`·`ControlData`와 동기화·더블버퍼 배치를 정의합니다. 시뮬레이션 전용 상태는 별도 `ipc/simulation`에서 교환합니다. `control_types.hpp`에는 재사용하는 명령·피드백·구동 상태 타입을 둡니다. `MotorData`에는 축별 배열과 공통 통계를 직접 둡니다. command/feedback 발행 번호는 독립적이며 `received_sequence[i]`는 축별 응답 번호입니다. HAL은 잠금 아래 최신 슬롯의 피드백·측정 토크·CAN 배열·공통 통계만 갱신해 제어기 명령을 보존합니다.

`MotorProfile` 구조체와 SHM의 프로파일 복사본은 제거했습니다. HAL은 YAML에서 기대 C4 바이트와 인코딩 범위를 초기화하고 모터의 실제 C4 응답과 비교합니다. 불일치는 enable을 거부하는 명시적 오류입니다. ROM 값을 쓰거나 SHM 초기값으로 모터 설정을 대체하지 않습니다. 가상 모터도 자신의 C4 응답을 로컬에서 보관하고 CAN으로 응답하며 SHM의 설정값을 참조하지 않습니다.

HAL은 확인한 C4 응답의 Kt를 프로세스 내부에 보관하고 `motor_torque[i] = Iq * Kt`를 피드백과 함께 발행합니다. 제어기는 Kt나 모터 설정을 읽지 않습니다. 직접 토크 디버깅(`--without-can`)은 MuJoCo actuator torque를 전달하고 전류는 계측하지 않습니다. 이 모드의 current 필드는 사용하지 않습니다.

모델 M/h/J·중력·발끝 정보는 CAN으로 측정한 관절 상태에서 별도 scratch `mjData`로 계산합니다. G/M의 `gravity_only`는 순수 중력과 실제 속도의 전체 bias 중 보상항을 선택합니다. 중력 계산용 `mjData`는 초기화 때 한 번 만들고, qvel=0에서 `mj_fwdPosition()`·`mj_fwdVelocity()`를 호출해 재사용합니다. 이 선택은 CAN 명령 형식·물리 마찰·RFOB 모델을 바꾸지 않습니다. TaskController의 q/dq/토크와 차분 가속도도 CAN 피드백에서 얻습니다. 물체 위치·접촉/해제 이벤트·GT 힘은 시뮬레이션 IPC의 관측/검증 정보이며 모션 제어 입력으로 사용하지 않습니다.

```bash
./build/shm_logger --can /qhrr05 /tmp/can-telemetry-new.csv
```

- CAN CSV의 age·관측 간격·SHM 발행 누락 평가는 로거에서만 수행합니다. 미수신 축의 age/interval은 빈 칸입니다.
- 최신 목표 sequence와 HAL이 소비한 목표 sequence는 다를 수 있습니다. 패킷에 sequence가 없어 요청별 왕복 지연·전체 CAN 손실·정확한 적용 시점을 식별하지 않습니다.
- 일반 CSV는 model/control/motors의 최신 완성본을 함께 읽고, 시뮬레이션 세션에서만 별도 IPC 정보를 추가합니다. `state_seq`는 소비한 모델 프레임 번호이고 `control_latency_us`는 모델 발행에서 명령 발행까지의 시간입니다.
- `sim_contact_event_time`은 시뮬레이션 시간, RFOB `event_time`은 독립 제어 시간입니다.

daemon이 소유한 SHM은 모든 자식이 종료된 뒤 daemon이 해제합니다. SIGKILL 뒤 남은 객체나 크기가 다른 객체는 자동 재사용·삭제하지 않고 충돌로 보고합니다. SHM 배치를 변경하면 모든 프로그램을 새 바이너리로 재시작합니다.

## 스토퍼 영점 보정

Z 모드는 hip → thigh → calf 순서로 작은 위치 증분을 보내고, 설정 방향의 토크 임계값과 저속 유지 조건으로 스토퍼 접촉을 판정합니다. 탐색 방향·기준 관절각·속도·토크·시간/이동 한계는 `control.yaml`의 `zero_set`에 있습니다. Arm이 별도로 필요합니다.

TaskController가 `q_joint = encoder + position_offset`의 런타임 offset과 축별 `coordinate_valid`를 저장합니다. 축 보정 후 설정 거리만큼 후퇴하고 다음 축을 진행하며, 완료하면 후퇴 자세를 유지합니다. C3/ROM 쓰기나 자동 호밍은 하지 않습니다. 이전 프로세스의 offset은 자동 재사용하지 않습니다.

새 피드백 없이 접촉을 확정하지 않으며 탐색 거리·시간·데이터 실패는 출력 해제와 fault로 처리합니다. 토크만으로 임의 장애물과 스토퍼를 구별할 수는 없습니다. 세부 동작과 설정은 [제어 문서](control.md)를 참고하십시오.

## 하드웨어 데이터와 통신 기한

HAL은 지원하는 신호 마스크와 유효 마스크, 축별 수신 시각/측정 시각을 게시합니다. 원시 엔코더 유효성과 `coordinate_valid`는 별개이며, 관절 영점은 제어기의 보정 결과로만 유효해집니다. `motor_torque`는 직접 센서가 아닌 Iq×Kt 환산값입니다.

`command_timeout_sec`(기본 0.10s), `feedback_timeout_sec`(0.25s), `enable_timeout_sec`(1.0s)는 HAL 한 곳에서 판정합니다. 오래된 명령을 반복 전송해 유효 기간을 늘리지 않습니다. SHM이 잠겨도 기한 판정을 계속하고 release를 시도합니다. 명령/피드백/enable 오류는 latched fault이며 clear-fault 후에도 DISARMED 상태를 유지합니다.

`poll_released`는 release 상태에서 무출력 C0 조회를 보낼지 명시합니다. 가상 모터에서는 true이며, 실제 펌웨어 지원을 가정하지 않도록 물리 기본값은 false입니다. 이 경우 Arm 후 무출력 준비 구간에서 첫 피드백을 기다립니다. 실제 CAN 단절에서는 release 전달을 보장할 수 없으므로 장치 자체 통신 타임아웃은 별도 확인해야 합니다.
