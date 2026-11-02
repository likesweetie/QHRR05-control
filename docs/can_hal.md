# CAN HAL 동작 규약

실행·GUI·설정 위치·SHM 작성 규칙은 [README](../README.md)를 기준으로 합니다. 이 문서는 패킷 처리, 계측 해석과 영점 보정 규약을 관리합니다.

## 설정과 통신 경로

- [daemon 설정](../config/daemon/daemon.yaml): 공유 영역 이름 `shm_name`과 자식 실행 인자. `"{shm_name}"`은 daemon이 치환합니다.
- [HAL 설정](../config/can_hal/can_hal.yaml): 인터페이스·실행/통계 주기·점유율 기준 비트레이트·관절별 CAN ID와 모터 설정 경로.
- [시뮬레이션 설정](../config/simulation/simulation.yaml): 실제 Kt·감속비·전기각과 Iq 센서 오차. 시뮬레이터의 `--simulation-config`로 전달하며 HAL은 읽지 않습니다.
- [모터 규약](../config/motor/ak10_9_v3.yaml): V_MAX/T_MAX·Kt_out·감속비·Kd scale. `motor_config` 경로는 HAL YAML 디렉터리 기준입니다.
- HAL은 `can_hal HAL_CONFIG.yaml SHM_NAME`, 가상 모터는 `simulation --hal-config HAL_CONFIG.yaml`로 같은 설정을 받습니다. 각 생성자가 직접 읽고, 누락·불일치는 명시적으로 실패합니다.

모터 명령·피드백은 Linux SocketCAN의 `PF_CAN/SOCK_RAW/CAN_RAW`를 통과합니다. 기본 런처는 HAL 설정의 기존 CAN 채널에 비트레이트를 적용하고 UP으로 전환합니다. 이미 설정이 일치하면 변경하지 않으며, 설정 권한이 필요하면 `ip link`에만 sudo를 사용합니다. 장치 생성과 호스트 vcan0 준비는 run_simulation.sh가 담당하며 네임스페이스 생성은 명시적 `--isolated-vcan` 디버깅에서만 수행합니다. daemon은 공유 영역을 생성·초기화하고 모든 자식 종료 후 해제합니다. 각 프로그램에 별도 터미널을 열고 고정 통계 영역과 로그 영역을 나눕니다. 표시 주기·로그 저장·SHM 지표의 정의와 계측 한계는 [README의 관측 설명](../README.md#프로세스-터미널과-shm-통계)을 참고하십시오. HAL·가상 모터는 기존 영역에 연결하며 소켓은 각각 소유합니다. HAL 설정의 예전 `shm_name`은 오류로 거부합니다. 직접 HAL을 실행할 때도 daemon이 만든 영역 이름을 인자로 지정해야 합니다.

프로토콜 구현 기준은 [OpenRobot MIT v17](https://dongilc.github.io/openrobot-motor-tool/)이며 [AK10-9 V3.0 KV60](https://www.cubemars.com/product/ak10-9-v3-0-kv60-robotic-actuator.html) 프로파일을 사용합니다. CubeMars 기본 펌웨어와의 호환과 실물 드라이버 동작은 검증 전입니다.

## 패킷과 상태 전이

| 명령 | HAL / 가상 모터 동작 |
| --- | --- |
| C4 | V_MAX/T_MAX·Kt_out·감속비·Kd scale 조회·일치 확인 |
| C5 | C4가 motor YAML과 다를 때만 모터별 한 번 ROM에 기록하고 C4로 검증 |
| C1 | HAL은 프로파일 검증 후 enable 요청·확인. 가상 모터는 enable 대기로 진입하며 C0 수신 전 출력은 0 |
| C0 | enable 상태에서 MIT 목표 적용, MIT 원점 기준 상대각·속도·Iq·온도 응답. 해제 상태·제로셋 보호 구간에서는 목표 적용 없이 피드백만 응답 |
| C2 | release 및 목표 초기화. HAL은 이후 C0 송신 중단, 종료 시에도 C2 송신 |
| C3 | 스토퍼 위치를 모터 MIT 좌표로 설정. DATA[6:7] signed int16 LE, 출력축 0.01 deg/LSB, 응답에 오프셋 에코 |

ID 1/2/3은 SID 0x141/0x142/0x143에 대응합니다. 초기화는 C2 해제 응답 → C4 조회 순서입니다. C4가 motor YAML의 표현값 또는 C5 양자화 후 표현값과 바이트 단위로 일치하면 C5를 생략합니다. 다를 때만 C5를 한 번 보내고, 쓰기 후에는 C5 양자화 후 표현값과 정확히 일치해야 합니다. 완료 전에는 해당 모터의 C0/C1을 보내지 않습니다. 장치가 늦게 연결되면 C2를 100 HAL 주기마다 재요청합니다. 초기 C4 조회와 C5 이후 C4 검증도 100 주기마다 재시도하며, 각각 `enable_timeout_sec` 안에 응답·검증되지 않으면 HAL을 오류 종료합니다. C5 쓰기 자체는 반복하지 않습니다. 미확인 C1도 100 주기마다 재요청합니다.

가상 모터는 축마다 `Released → Enabled → MitControl` 상태를 가집니다. C1만으로는 토크가 발생하지 않고, 다음 유효 C0부터 목표를 유지하며 PD를 계산합니다. 이미 enable된 모터에 반복한 C1은 원점·목표를 바꾸지 않습니다. C2는 즉시 출력을 0으로 만들고 이전 목표를 지우므로 재-enable 후 새 C0가 필요합니다.

가상 모터의 시작 원점은 `simulation.yaml`의 `motor.require_zero_position_setup`으로 정합니다. **false**이면 원점이 보정된 모델 좌표로 시작하여 C0 위치 피드백이 MuJoCo 관절각과 일치합니다. **true**이면 축마다 독립적인 임의 엔코더 원점으로 시작하며 TaskController의 순차 스토퍼 탐색과 HAL의 C3 교환으로 모터의 런타임 좌표를 보정합니다. 이 원점 설정 자체는 ROM을 쓰지 않으며, 모터 파라미터는 별도의 시작 시 C5로 설정합니다. 임의 값은 모터 객체 생성 때 한 번만 정하고 매 스텝 다시 뽑지 않습니다. 현재 가상 모터의 두 시작 정책은 초기 원점을 이미 정하므로 C1에서 원점을 다시 잡지 않습니다. 펌웨어 문서의 미설정 상태 최초 진입 자동 캡처를 그대로 적용하면 enable 전 움직인 실제 관절과 호스트 원점이 어긋났기 때문에 이 시작 정책으로 분리했습니다.

C3으로 명시한 원점은 C2/C1 사이에도 보존합니다. 예를 들어 C3 offset=3000이면 현재 관절의 MIT 피드백이 +30°가 되며, MuJoCo qpos는 바꾸지 않습니다. C0 피드백과 PD 위치 오차는 같은 MIT 좌표를 사용합니다. MuJoCo qpos는 이미 출력축 rad이므로 감속비로 다시 나누지 않습니다. 알려진 관절각에서 C3 오프셋을 그 각도로 지정하면 모델 좌표와 맞출 수 있습니다. C3 offset=0은 현재 자세를 0도로 정하는 명령이며 실제 관절각의 자동 추정은 아닙니다.

C3는 기존 목표를 지우고 **시뮬레이션 시간 20ms** 동안 수신한 C0 목표를 버립니다. 보호 구간에도 피드백은 새 원점을 반영하며, 시간이 지난 뒤 수신한 새 C0부터 제어합니다. 일시정지 중에는 보호 시간이 줄지 않고, MuJoCo 시간 리셋 시 남은 보호 시간과 런타임 원점을 유지합니다. 가상 모터 객체 재생성(전원 재시작에 해당) 시에는 런타임 원점을 초기화합니다. C3는 모터를 enable하거나 ROM을 수정하지 않습니다.

현재 HAL은 시작 시 C5 쓰기와 C4 검증, C1/C0/C2 및 제로셋 전용 C3 요청을 처리합니다. 제어기는 `zero_position`에 스토퍼 각도를 rad로 저장하고 `zero_request_sequence`를 증가시킵니다. HAL은 C3를 한 번 전송하고 에코를 확인한 뒤 20ms를 기다리며, 이후 중립 C0의 새 피드백을 받아 `zero_completed_sequence`, `zero_applied_position`, `coordinate_valid`를 발행합니다. 제어기가 새 좌표의 목표와 같은 `zero_resume_sequence`를 함께 발행하기 전까지 이전 좌표의 위치 명령은 전송하지 않습니다. 에코/피드백 시간 제한은 HAL의 feedback/enable timeout 중 짧은 값이며, 누락·불일치는 출력을 해제하는 fault입니다. 외부에서 임의로 보낸 C3 응답도 좌표 불일치 fault로 처리합니다. 호스트 위치 오프셋은 없습니다. C5는 시작 시 모터 파라미터 쓰기에 사용하며 운전 중에는 전송하지 않습니다. 물리적 엔코더 영점 명령 0x19, 모터 펌웨어의 fault clear 및 RMD 제어는 지원하지 않습니다. 아래의 HAL 통신 fault 해제는 호스트 상태의 해제이며 펌웨어 명령을 대신하지 않습니다. 지원하지 않는 명령은 명시적 오류입니다.

[control_types.hpp](../src/ipc/control_types.hpp)의 `MitCommand`는 position·velocity·kp·kd·tau_ff를 가집니다. enable 요청은 `MotorData.command_enabled[i]`로 분리합니다. SHM과 TaskController는 OpenRobot 코덱을 참조하지 않고, 코덱이 이 공통 타입을 참조합니다. position은 출력축 상대 좌표 rad입니다. Homing/Gravity/Magnetic/Tracking/Ball은 `control.yaml` 최상단의 `torque_reference_mode`로 토크 전달 방식을 선택합니다.

```text
torque_ref_as_position: q_motor_target = q_reference + tau_reference / kp_motor, tau_ff = 0
native_torque_ref:      q_motor_target = q_reference, tau_ff = tau_reference
rmd_torque_ref:         Iq = tau_reference / Kt_out -> RMD 0xA1
v_motor_target = 0
tau_motor = kp_motor*(q_reference - q_measured) + tau_reference - kd_motor*dq
# 공통 변환은 P+토크 합산 노력에 위치 오차 제한을 적용; 식은 비포화·양자화 무시 기준
```

MIT 경로의 관절 감쇠는 모터 내부 MIT D에서만 적용하며 위치 목표에서 상쇄하지 않습니다. 작업공간 속도 피드백은 소프트웨어 토크에 유지합니다. 가상 모터의 내부 PD 출력은 모터 프로파일 토크 한계와 XML actuator 한계의 교집합으로 제한합니다.

**Damping 테스트 모드는 출력 방식 설정과 무관하게 kp=kd=0으로 두고 tau_ff에 출력축 토크 명령을 보냅니다.** MIT 경로의 다른 모드는 `native_torque_ref`를 선택하면 토크를 tau_ff로 전송합니다. MIT 경로의 HAL은 검증된 C4 T_MAX로 토크를 제한하고 마지막 바이트에 `round((tau_ff+T_MAX)/(2*T_MAX)*255)`를 넣습니다. 가상 모터도 `raw*2*T_MAX/255-T_MAX`로 복원해 P/D 출력에 합산합니다. T_MAX=33일 때 토크 간격은 약 0.2588 N·m이고 0x80은 +0.1294 N·m입니다. 기존 송신의 0 명령 바이트는 동일하지만, 가상 모터는 더 이상 0x80을 수치 0으로 특별 취급하지 않습니다. 비유한 토크는 오류입니다.

피드백은 v14 이후 signed 출력축 MIT 좌표로 해석합니다. 구형 입력축 표를 자동 감지하거나 대체하지 않습니다. 위치는 int16 × 12.56 / 32768 rad, 속도는 int16 deg/s를 rad/s로 변환하며, Iq는 int16 × 33 / 2048 A입니다. 가상 모터는 센서 OFF일 때 직전 물리 스텝의 실제 actuator torque / C4 Kt를 반환합니다. 센서 ON일 때는 실제 모터 Kt·감속비로 true Iq를 역산한 뒤 센서 오차를 적용해 반환합니다. CAN 패킷 형식은 그대로이며 토크 필드를 추가하지 않습니다. HAL은 실제 CAN Iq와 검증한 C4 Kt로 SHM의 측정 토크를 계산합니다. 조정 항목과 물리 시각 기반 샘플링은 [제어·시뮬레이션 설정](control.md#가상-iq-센서)를 참고하십시오.

Damping도 HAL이 MIT C0 또는 RMD A1에서 구성한 C3 기준 누적각과 `received_sequence`·`measurement_ns`를 사용합니다. 새 위치 샘플의 차분으로 속도를 계산하며 2*pi 싱글턴 래핑을 적용하지 않습니다. 별도 출력 엔코더 조회와 SHM 필드는 없습니다.

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

`MotorProfile` 구조체와 SHM의 프로파일 복사본은 제거했습니다. HAL은 YAML에서 C4 비교값·C5 쓰기 바이트·인코딩 범위를 초기화합니다. C2 해제 확인 후 V_MAX/T_MAX/Kt/GR/Kd scale을 C4로 먼저 확인하고, 필요한 경우에만 C5를 한 번 기록합니다. 쓰기 후 불일치는 enable을 거부하는 명시적 오류입니다. SHM에는 파라미터를 복사하지 않습니다. 가상 모터도 자신의 C4 응답을 로컬에서 보관하고 CAN으로 응답하며 SHM의 설정값을 참조하지 않습니다.

C5는 입력축 `Kt_input`을 0.001 Nm/A 단위로 받지만 YAML은 출력축 `kt_output_nm_a`를 유지합니다. 양자화한 GR로 Kt_out/GR을 계산해 C5에 넣고, 그 입력축 Kt와 GR의 곱을 C4 기대값으로 사용합니다. 예를 들어 Kt_out=1.298, GR=9이면 Kt_input=0.144, C4 Kt_out=1.296입니다. 요청값·전송값·예상 응답을 시작 로그에 표시하고 측정 토크에는 확인된 C4 값을 사용합니다. 표현 불가능한 값은 CAN 송신 전에 거부합니다. 가상 모터는 C5를 축별로 적용하고, 구동 중 쓰기를 거부하며 기존 C4 값을 반환합니다.

위 예시에서 이미 C4가 1.298을 반환하거나, 앞선 C5로 1.296이 저장되어 있으면 다른 모든 필드까지 정확히 일치할 때 재기록하지 않습니다. 임의 허용 오차로 비교하지 않으며, 시작 후 C4는 처음 검증한 응답과 일치해야 합니다. 펌웨어는 FOC 구동 중 C5를 거부할 수 있고, C2 응답만으로 FOC OFF가 보장되지는 않습니다. 특히 `cc_min_current=0`인 장치에서 관측된 현상이며, 실제 설정 변경이 거부되면 HAL은 검증을 통과시키거나 MCU를 재부팅하지 않고 오류로 종료합니다.

C4 불일치 로그에는 인터페이스, 축(HR/HP/KP), 모터 ID, CAN SID와 모터 설정 파일 경로를 표시합니다. 각 항목의 `actual`은 응답에서 해석한 값, `expected`는 C5 쓰기의 양자화를 반영한 C4 값이며, `[MISMATCH]`가 다른 항목을 가리킵니다. Kt는 Nm/A, 속도는 rad/s, 토크는 Nm로 표시하고 실제·기대 원시 8바이트도 함께 기록합니다. 이 로그는 `logs/run-*/can_hal.log`에서 확인할 수 있습니다.

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

TaskController는 접촉한 축의 `stop_position_rad`를 C3 목표로 요청합니다. HAL이 에코·새 좌표의 피드백을 확인한 뒤 해당 축의 `coordinate_valid`를 설정합니다. 제어기는 모터 피드백을 관절각으로 직접 사용하며, 설정 거리만큼 후퇴하고 다음 축을 진행합니다. 완료하면 후퇴 자세를 유지합니다. 제로셋 자체는 ROM 쓰기나 자동 호밍을 하지 않습니다.

새 피드백 없이 접촉을 확정하지 않으며 탐색 거리·시간·데이터 실패는 출력 해제와 fault로 처리합니다. 토크만으로 임의 장애물과 스토퍼를 구별할 수는 없습니다. 세부 동작과 설정은 [제어 문서](control.md)를 참고하십시오.

## 하드웨어 데이터와 통신 기한

HAL은 지원하는 신호 마스크와 유효 마스크, 축별 수신 시각/측정 시각을 게시합니다. 원시 엔코더 유효성과 `coordinate_valid`는 별개이며, 관절 영점은 제어기의 보정 결과로만 유효해집니다. `motor_torque`는 직접 센서가 아닌 Iq×Kt 환산값입니다.

`command_timeout_sec`(기본 0.10s), `feedback_timeout_sec`(0.25s), `enable_timeout_sec`(1.0s)는 HAL 한 곳에서 판정합니다. 오래된 명령을 반복 전송해 유효 기간을 늘리지 않습니다. SHM이 잠겨도 기한 판정을 계속하고 release를 시도합니다. 명령/피드백/enable 오류는 latched fault이며 clear-fault 후에도 DISARMED 상태를 유지합니다.

`poll_released`는 release 상태에서 무출력 C0 조회를 보낼지 명시합니다. 가상 모터에서는 true이며, 실제 펌웨어 지원을 가정하지 않도록 물리 기본값은 false입니다. 이 경우 Arm 후 무출력 준비 구간에서 첫 피드백을 기다립니다. 실제 CAN 단절에서는 release 전달을 보장할 수 없으므로 장치 자체 통신 타임아웃은 별도 확인해야 합니다.

## RMD Torque Closed-Loop 출력

`control.yaml`의 `torque_reference_mode: rmd_torque_ref`는 Gravity·Magnetic·Tracking 실행·Damping의 토크를 `0xA1`로 전송합니다. 기본 Homing·Ball·ZeroSet과 Tracking 사전 검사/실패 HOLD는 MIT C0 위치 출력을 유지합니다. `homing.torque_control: true`이면 호밍도 호스트 PD 토크를 A1으로 보내며 damping_kd는 기존 `rmd_damping_constant` 설정값을 그대로 사용합니다. 나머지 RMD 경로에는 호스트 관절 P/D를 추가하지 않으며 A1의 damping_kd에는 rmd_damping_constant를 보냅니다. 기본값 5, 범위 0..100 정수이며 MIT kd 단위와 다릅니다. 작업공간 피드백과 코깅 보상은 기존 torque_command에 포함됩니다.

HAL은 C4에서 검증한 출력축 Kt로 `Iq=tau/Kt_out`을 계산하고 ±33 A로 제한한 뒤 `round(Iq*2048/33)`을 signed int16 little-endian DATA[4:5]에 넣습니다. 0 Nm는 정확히 0입니다. MIT의 C5 T_MAX는 이 전류 범위에 적용되지 않으며, 제어기의 `limits.torque_nm`은 계속 적용됩니다.

A1 응답의 14-bit 입력축 엔코더와 입력축 deg/s를 감속비로 나눕니다. MIT→RMD 전환 시 한 번의 C0/9C 응답 쌍으로 현재 C3 좌표에 연결하며, 두 응답 사이의 짧은 이동은 측정 속도로 보간합니다. 이후 A1 응답만으로 각도를 누적하고 매 주기 추가 읽기 요청은 보내지 않습니다. 좌표 연결 오차는 초기 C0 양자화와 두 측정 시각 차이에 영향을 받습니다. 초기 C0/9C 응답이 누락되면 10 ms 간격으로 재시도하고, 설정된 feedback_timeout_sec 안에 완료되지 않으면 기존 피드백 타임아웃으로 정지합니다. RMD 수신에도 공통 feedback_timeout_sec를 적용합니다.


MIT→RMD 전환에는 C2 후 A1, RMD→MIT 전환에는 C1 후 C0를 전송합니다. 종료·Release·fault는 기존 C2로 출력을 해제합니다. 명령의 `rmd_torque` 선택은 SHM ABI 22 및 MCAP에 기록됩니다.
