# qhrr-05 제어 검증

고정 베이스 3자유도 다리(HR / HP / KP)의 호밍·중력보상·Magnetic Touch를 검증하는 C++ 프로그램입니다. 모델은 Go2 몸통과 FL 다리만 사용합니다.

기본 제어 경로는 **TaskController → SHM → CAN HAL → vcan → MuJoCo 가상 모터**입니다. 모터는 수신한 MIT 위치·속도 목표와 kp/kd로 내부 PD를 계산합니다. 관절 위치·속도·Iq는 CAN 응답으로 돌아오며 제어기는 이를 사용합니다. tau_ff는 사용하지 않습니다. 기존 SHM 토크 구동은 `--without-can` 디버깅 모드에만 남겼습니다.

## 빌드와 실행

Linux, CMake, C++17 컴파일러, pkg-config, yaml-cpp, GLFW/OpenGL이 필요합니다. MuJoCo는 `third_party/mujoco`의 헤더·라이브러리를 사용합니다. GUI 실행에는 디스플레이가 필요합니다.

저장소 루트에서 실행합니다.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 2
./scripts/run_daemon.sh
```

daemon은 [프로세스 목록](config/app_config/daemon_config.yaml)에 따라 **CAN HAL·시뮬레이터·TaskController를 직접 자식으로 실행**하고 종료·자식 회수를 관리합니다. CAN 통계는 별도 GNOME 창에 표시합니다. 창에서는 동일 실행 파일의 출력 전용 함수만 실행하고, 프로세스 수명 관리는 원래 daemon 하나가 담당합니다. Ctrl+C, 통계 창 또는 시뮬레이터 종료 시 나머지 프로세스도 정리합니다.

기본 런처는 격리된 사용자/네트워크 네임스페이스에 **vcan0를 생성하고 UP으로 올린 뒤** daemon을 실행합니다. 호스트 네트워크는 변경하지 않습니다. vcan 준비나 CAN HAL 초기화에 실패하면 실행을 중단하며 CAN 없는 구성으로 넘어가지 않습니다. 모터 응답이 없으면 HAL은 WAIT 상태로 계속 동작합니다.

| 실행 | 구성 |
| --- | --- |
| `./scripts/run_daemon.sh` | 기본: vcan 자동 준비 + CAN HAL·시뮬레이터·제어기 |
| `./scripts/run_daemon.sh --existing-can` | 이미 준비된 호스트 CAN 사용. 인터페이스는 HAL YAML에서 지정 |
| `./scripts/run_daemon.sh --without-can` | 디버깅 전용: CAN 준비와 HAL을 생략하고 `debug_processes` 실행 |

호스트 CAN이 준비되어 있으면 `./build/daemon`을 직접 실행할 수도 있습니다. CAN 없는 디버깅은 `./build/daemon --without-can`으로도 선택합니다. 선택한 모드는 로그에 표시합니다.

프로그램별 옵션은 목록의 `args`에 지정합니다. 다른 목록은 `./build/daemon --config FILE.yaml`로 선택합니다. CAN HAL의 소켓·SHM은 HAL 자체가 소유합니다.

필요하면 다른 터미널에서 계측을 기록합니다. 기존 CSV는 덮어쓰지 않습니다.

```bash
./build/shm_logger /qhrr05 /tmp/qhrr05-new-run.csv
```

## GUI 조작

| 입력 | 동작 |
| --- | --- |
| H | 5차 보간 호밍 후 순수 PD 자세 유지 |
| G | 중력보상. 자세 강성·감쇠는 설정값 사용 |
| M | Magnetic Touch: FREE → 접촉 후보 → 힘 추종 → 해제 |
| Target [N] | 목표 접촉력 변경. 접촉 상태는 유지 |
| Space | 물리 일시정지·재개. 제어 루프는 계속 동작 |
| R | 물리 초기화와 재호밍 요청 |
| Esc | 종료 |

몸체를 왼쪽 더블클릭하여 선택한 뒤 Ctrl+오른쪽 드래그로 외력을 가합니다. 주황색 물체도 같은 방식으로 선택·이동하며, Ctrl+Shift+오른쪽 드래그는 수평면 이동입니다. Magnetic 모드에서 물체를 발끝에 접촉시키고 천천히 이동하면 자동으로 힘 추종에 진입합니다. 접촉 해제·작업영역·추종 거리 등 설정 한계를 넘으면 FREE로 복귀합니다.

H/G 화면은 관절각과 실제 모터 토크를 표시합니다. M 화면은 추정/실제/목표/명령 힘, 발끝 실제·목표 위치, 추종 오차와 전환 사유를 표시합니다.

![Magnetic Touch 화면](docs/magnetic_touch_preview.png)

## 설정과 코드 위치

| 파일·디렉터리 | 역할 |
| --- | --- |
| [control_config.yaml](config/control_config/control_config.yaml) | 제어 주파수, 각 모션의 게인·목표·한계, RFOB, 그래프 주기 |
| [daemon_config.yaml](config/app_config/daemon_config.yaml) | 기본 `processes`와 명시적 CAN 없는 `debug_processes` |
| [can_hal.yaml](config/motor_config/can_hal.yaml) | CAN 인터페이스·주기·모터 ID·프로파일·SHM |
| [app/task_controller/](app/task_controller/) | 제어기 핸들 소유, 입력 → 제어 → 출력 → 주기 대기 |
| [app/motion_controllers/](app/motion_controllers/) | 호밍·중력보상·Magnetic Touch와 외력 관측기 |
| [app/mujoco_simulate/](app/mujoco_simulate/) | 물리·모델 계산·GUI·계측 |
| [app/can_hal/](app/can_hal/) / [app/motor_can/](app/motor_can/) | SocketCAN 루프·통계 / MIT 코덱 |
| [app/shm_logger/](app/shm_logger/) | CSV와 지연·누락 평가 |

TaskController는 각 모션 클래스의 초기화 인자로 YAML **경로**를 넘깁니다. 각 클래스가 자기 항목을 파싱·검증합니다. 설정을 바꾼 뒤 프로그램을 재시작합니다. 관절 배열 순서는 HR(FL hip) / HP(thigh) / KP(calf)입니다. 게인과 목표값은 위 YAML을 기준으로 합니다.

Homing은 보간 위치·속도와 자신의 kp/kd를 MIT 목표로 보냅니다. Gravity는 원래 surrogate 위치 목표를, Magnetic은 `q + (tau + kd_motor*dq)/kp_motor`를 보냅니다. 후자의 kd 항은 이미 계산된 감쇠를 모터 PD에서 중복 적용하지 않기 위한 항입니다. Magnetic의 내부 서보 게인은 `magnetic_touch.kp_motor/kd_motor`에서 설정합니다.

가상 엔코더는 시작 자세에서 상대각 0으로 시작합니다. TaskController는 시뮬레이션 초기 자세를 좌표 offset으로 사용합니다. **실기 영점 보정은 아니며 스토퍼 호밍은 미구현**입니다.

## SHM과 주기

기존 시뮬레이션은 시뮬레이터가 `/qhrr05`를 생성·해제합니다. 독립 HAL은 `/qhrr05_can`을 소유합니다. daemon은 SHM을 관리하지 않습니다. 기존 객체가 있으면 명시적으로 실패하며 자동 삭제·재사용하지 않습니다.

채널은 [shared_memory.hpp](include/shared_memory.hpp)에 정의된 **시퀀스 번호가 있는 더블버퍼**이며, 슬롯 복사와 발행은 짧은 프로세스 공유 robust mutex로 보호합니다. 계산과 파일 쓰기는 잠금 밖에서 수행합니다. 잠금 소유자 사망은 명시적 오류입니다.

| 채널 | 작성자 → 용도 |
| --- | --- |
| `states` | 시뮬레이터 → CAN으로 디코딩한 자세에서 계산한 동역학·발끝 모델 정보 |
| `requests` | GUI → 모드·재시작·목표 힘 |
| `commands` | TaskController → 표시용 목표·모션 상태. 토크 구동은 디버깅 모드에서만 사용 |
| `samples` | 시뮬레이터 → 그래프·CSV 검증 계측 |
| `joystick` | 선택적 aux_reader → 조이스틱 축·버튼 |
| `motor_targets` / `can_states` | TaskController의 MIT 위치 명령 / HAL의 CAN 피드백·통계 |

제어 주파수는 `task_controller.frequency_hz`로 설정합니다. 제어기는 monotonic 시간으로 동작하며 새 측정이나 물리 run/pause를 기다리지 않고 최신 입력을 사용합니다. 물리 간격은 모델의 `timestep`, 그래프·계측 주기는 `plots.sample_hz`의 시뮬레이션 시간 기준입니다. 물리 일시정지 중에도 제어 시간은 진행합니다. 응답 지연·관측 누락 평가는 **shm_logger에서만** 수행합니다. 더블버퍼는 모든 중간 프레임의 기록을 보장하지 않습니다.

RFOB는 새 물리 측정에서만 필터를 갱신합니다. 접촉 GT 힘·모멘트와 물체 좌표는 `samples`에만 있으며 제어 입력으로 사용하지 않습니다. 현재 관측기는 모델 잔차 기반 발끝 등가 외력 추정입니다. `condim=6`의 접촉 모멘트를 3관절 관측으로 힘과 분리할 수 없으며, Momentum Observer는 미구현입니다.

CAN 모드에서는 새 CAN 피드백에서 속도를 차분하고 RFOB를 갱신합니다. 모델 정보는 별도 주기로 발행되므로 최신 엔코더보다 한 주기 늦을 수 있습니다. 물체 이동·접촉·해제 같은 시뮬레이션 이벤트는 SHM으로 직접 전달해도 되는 영역입니다. 이 이벤트 때문에 모터 CAN 패킷을 추가하지 않습니다.

## CAN HAL

HAL은 기본 daemon의 필수 프로세스입니다. 중복 HAL 전용 daemon YAML은 제거했습니다. 기본 vcan 준비에는 `unshare`, iproute2, D-Bus가 필요합니다. 통계 창은 manifest의 `terminal_exec`·`terminal_title`로 지정합니다. 현재 환경은 FD 전달을 지원하는 `/usr/bin/gnome-terminal.real`을 명시했습니다. 실행 파일·인터페이스·창 오류를 다른 경로로 자동 대체하지 않습니다.

명령은 MIT q target·velocity·kp/kd이며 tau_ff는 논리적 0으로 고정합니다. 기본 시뮬레이터가 C4 프로파일 조회·C1 enable·C0 제어/피드백·C2 release에 응답합니다. 시작 시 가상 모터가 늦게 준비되어도 100 HAL 주기마다 프로파일/enable을 재요청합니다. 응답 지연 판정이나 시간 초과 정지는 없습니다. 명령 API·로깅·프로토콜 범위·스토퍼 보정 결정은 [CAN HAL 핸드오버](docs/vcan_motor_handover.md)에 정리했습니다.

## 검증과 유지 문서

```bash
ctest --test-dir build --output-on-failure
```

CTest에는 격리된 vcan의 코덱 시험과 실제 MuJoCo 가상 모터 폐루프 시험이 있습니다. 후자는 호밍·G/M·물체 접촉/해제·CAN 정지 중 우회 금지·Release를 검사합니다. 실제 AK10-9 하드웨어 검증은 아닙니다. 회귀 시험은 사용자 튜닝과 분리한 [고정 설정](tests/fixtures/control_config.yaml)을 사용합니다.

- [최근 작업과 다음 우선순위](WORK_NOTE.md)
- [코드 작성·포맷 규칙](docs/code_style.md)
- [모델 구조·출처·라이선스](models/qhrr-05/README.md)
- [MuJoCo GUI 복사 출처와 변경 범위](app/mujoco_simulate/SOURCE.md)
