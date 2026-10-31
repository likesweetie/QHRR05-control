# qhrr-05 제어 검증

고정 베이스 3자유도 다리(HR / HP / KP)의 호밍·중력보상·Magnetic Touch·작업공간 Tracking·조이스틱 공 추종을 검증하는 C++ 프로그램입니다. 모델은 제공된 SolidWorks CAD 기반 QHRR-05 몸통과 FL 다리를 사용합니다.

기본 제어 경로는 **TaskController → SHM → CAN HAL → 물리 CAN 모터**입니다. vcan과 MuJoCo 물리 시뮬레이터는 별도 시뮬레이션 스크립트에서 선택합니다. 실제/가상 모터 모두 같은 관측·명령 계약을 사용하며 모션 제어식은 데이터 출처를 구분하지 않습니다. MuJoCo의 측정 상태 기반 기구학·동역학 계산은 두 경로에서 유지합니다.

## 빌드와 실행

Linux, CMake 3.22 이상, C++17 컴파일러(DataTamer 내부는 C++20), yaml-cpp, GLFW/OpenGL, OpenSSL·zstd·lz4 개발 라이브러리와 스크립트용 Python 3/PyYAML을 사용합니다. 같은 컴퓨터에서 시뮬레이션도 할 수 있도록 기존 빌드 타깃을 유지합니다.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 4
./scripts/run_daemon.sh          # 기본: 준비된 물리 can0, 시뮬레이터 실행 안 함
# 웹 패널도 자동 시작: http://127.0.0.1:8080
```

물리 인터페이스와 비트레이트는 [CAN 설정](config/can_hal/can_hal.yaml)을 확인하십시오. 기본 스크립트는 장치가 없거나 DOWN/비트레이트 불일치이면 명시적으로 실패하며, 네트워크 설정을 바꾸거나 가상 장치로 대체하지 않습니다. daemon은 HAL/control 및 선택적 input/logger/web_gui의 수명을 관리합니다. `--no-terminals`는 자식 출력을 현재 터미널로 모읍니다.

### vcan 시뮬레이션

```bash
./scripts/run_simulation.sh                     # vcan0 + MuJoCo OpenGL GUI + 웹 GUI
./scripts/run_simulation.sh --isolated-vcan     # 독립 네임스페이스 vcan0
./scripts/run_simulation.sh --without-can       # 직접 effort 디버그; 전류는 취득 불가
./scripts/run_visualization.sh --simulation    # vcan + headless physics + Filament
```

실행 목록과 CAN 설정은 기본 [daemon.yaml](config/daemon/daemon.yaml)·[can_hal.yaml](config/can_hal/can_hal.yaml)에서 런처가 임시로 생성합니다. 복제한 simulation용 daemon/HAL YAML은 두지 않습니다. [config/simulation/simulation.yaml](config/simulation/simulation.yaml)은 기존 가상 모터의 원점·Iq 센서 특성만 설정합니다. 제어 설정은 [control.yaml](config/control/control.yaml) 하나이며, 모든 경로에서 같은 `simulation` 실행 파일을 사용합니다. 호스트 vcan 준비는 시뮬레이션 스크립트만 수행하며 종료 후 호스트 vcan은 유지됩니다.

### HW_deploy 가상 모터 시험

```bash
./scripts/run_hw_deploy_test.sh               # 기본: 호스트 vcan0 + MuJoCo OpenGL 창
./scripts/run_hw_deploy_test.sh --verify      # 창 없이 자동 검증 후 종료
./scripts/run_hw_deploy_test.sh --verify --isolated-vcan  # 독립 vcan 자동 검증
```

이 시험은 물리용 daemon/HAL/control 설정을 사용하고 **CAN 인터페이스만 vcan0로 바꾼 뒤 기존 `simulation` 프로세스를 시작합니다.** 같은 물리 루프·가상 모터·OpenGL GUI를 재사용합니다. 제어기는 하드웨어 세션으로 동작하므로 시뮬레이션 IPC/GT/자동 보정 정책을 사용하지 않습니다. 기본 실행은 `/qhrr05` 세션과 MuJoCo OpenGL 창을 엽니다. Z로 제로셋 선택, A로 Arm, X로 Release, H로 호밍을 요청합니다. 창의 제어 명령은 공통 Input을 거치며 창을 닫으면 시험 세션을 종료합니다. `--interactive`는 기본 동작과 같습니다. GUI를 열 수 없으면 오류로 종료합니다.

`--verify`는 DISARMED 시작, 시뮬레이션 명령 거부, 명시적 Arm, 제로셋, 호밍, Release, 제어기 일시 중단에 대한 HAL 명령 만료, fault 해제를 확인하고 종료합니다. 실제 모터로 검증했다는 의미는 아닙니다.

## 조작과 데이터 상태

모든 세션은 **DISARMED**로 시작합니다. 모드 선택만으로 모터 출력이 켜지지 않습니다. 웹 또는 GUI에서 Arm/Release를 사용하며, 터미널에서도 같은 Input 인터페이스로 요청할 수 있습니다.

```bash
./build/input --command /qhrr05 mode 6 0      # Zero set 선택
./build/input --command /qhrr05 arm          # 출력 허용
./build/input --command /qhrr05 release      # 출력 해제
./build/input --command /qhrr05 clear-fault  # 원인 해소 후 DISARMED로 복귀
```

실제 장치 세션은 엔코더를 수신했다고 보정 완료로 간주하지 않습니다. 일반 모션 전에 영점 보정이 필요합니다. 제로셋은 엔코더·속도·Iq×Kt 환산 토크를 사용하고 ROM을 쓰지 않습니다. 자세한 설정은 [제어 문서](docs/control.md)를 참고하십시오. 자동 시뮬레이션 시작 정책도 일반 작업 선택으로 전달하며 출력에는 Arm이 필요합니다.

| 키 | 기능 |
| --- | --- |
| A / X | Arm / Release |
| Z / D / H | 제로셋 / 감쇠 / 호밍 |
| G / M | 중력 보상 / 접촉 추종 |
| T / 1 / 2 / 3 | 궤적 / XY / XZ / YZ |
| B / W | 공 추종 / 작업공간 표시 |
| Space / R / Ctrl 드래그 | 시뮬레이션에서만 pause / reset / 외란 |

취득 가능, 미수신, 오래됨, 미보정, 유효한 0을 구분합니다. 원시 엔코더와 보정된 관절 좌표, Iq×Kt 환산 토크와 직접 토크 측정, 모델 FK와 실측/GT는 같은 의미로 표시하지 않습니다. 시뮬레이션 전용 데이터는 기존 별도 IPC에 남고 공통 SHM에는 관측·계산·제어 상태와 유효성을 저장합니다. **SHM ABI 16**이므로 참여 프로세스를 모두 다시 빌드하고 재시작해야 합니다.

## Filament PBR 시각화

```bash
./scripts/build_visualization.sh
./scripts/run_visualization.sh              # 실행 중인 물리/가상 장치 세션 관측
./scripts/run_visualization.sh --simulation # 별도 시뮬레이션 세션 시작
./scripts/run_visualization.sh --preview    # 정적 모델 미리보기
```

물리 세션에서는 CAN 관절 좌표로 렌더링 전용 데이터의 FK만 갱신합니다. 시뮬레이션 GT가 없어도 갱신되며 물리 적분은 하지 않습니다. 아직 관절 좌표가 유효하지 않으면 자세가 확정되지 않았음을 표시합니다. 기존 카메라·재질·8x MSAA·고정 내부 해상도 기능을 유지합니다. 시뮬레이션 전용 조작은 해당 기능이 제공되는 세션에서만 활성화합니다.

필요한 native 실행 파일이나 렌더링 자산이 없으면 해당 런처가 자동 빌드합니다. 소스를 수정한 뒤에는 직접 다시 빌드하십시오. 웹 런처에는 C++ 자동 빌드를 추가하지 않습니다.

## 설정과 코드 위치

| 파일·디렉터리 | 역할 |
| --- | --- |
| [control.yaml](config/control/control.yaml) | 제어 주파수, 각 모션의 게인·목표·한계, RFOB, 그래프 주기 |
| [input.yaml](config/input/input.yaml) | 조이스틱 장치·축·트리거·데드밴드 |
| [daemon.yaml](config/daemon/daemon.yaml) | SHM 이름·기본 물리 CAN 프로세스 |
| [can_hal.yaml](config/can_hal/can_hal.yaml) | HAL 인터페이스·주기·CAN ID 배치·모터 설정 경로 |
| [simulation.yaml](config/simulation/simulation.yaml) | 시뮬레이션 실제 모터 특성·가상 Iq 센서 ON/OFF·오차 파라미터 |
| [ak10_9_v3.yaml](config/motor/ak10_9_v3.yaml) | 모터 MIT 규약: V_MAX·T_MAX·Kt·감속비·Kd scale |
| [shm_logger.yaml](config/shm_logger/shm_logger.yaml) | 500 Hz MCAP·Flight Recorder·저장 한계·건전성·세션 설정 |
| [apps/](apps/) | daemon·TaskController·CAN HAL·MuJoCo·로거·입력 프로그램의 실행 진입점 |
| [src/ipc/](src/ipc/) | SHM 배치·동기화·공유 타입과 simulation IPC |
| [src/control/](src/control/) | `TaskController`의 제어기 핸들 소유, 입력 → 제어 → 출력 → 주기 대기 |
| [src/motion_controllers/](src/motion_controllers/) | 호밍·중력보상·Magnetic·Tracking/ILC·공 추종·외력 관측기 |
| [src/motion_controllers/numerics/](src/motion_controllers/numerics/) | 모션 컨트롤러 전용 행렬·역행렬·DLS·수치 함수 |
| [src/robot_model/](src/robot_model/) | 측정 상태 기반 MuJoCo 기구학·동역학 백엔드 |
| [src/simulation/](src/simulation/) | 물리·가상 모터 IO·가상 Iq 센서·그래프 |
| [src/motor_can_protocol/](src/motor_can_protocol/) | OpenRobot MIT 코덱 |
| [src/daemon/](src/daemon/) | 기존 `Daemon` 프로세스 관리·터미널 릴레이·SHM 통계 |
| [src/interfaces/](src/interfaces/) | Input 명령·조이스틱·웹 상태 브리지 |
| [apps/interfaces/](apps/interfaces/) | 입력 실행 진입점·웹 서버와 React 프런트엔드 |
| [src/shm_logger/](src/shm_logger/) | SHM 복사·명시적 스키마·MCAP/RAM 이력·세션/저장 정책·관리 소켓 |
| [third_party/mujoco_simulate/](third_party/mujoco_simulate/) | 프로젝트에서 사용하는 MuJoCo GUI 복사본·출처·해시 |
| [tests/](tests/) | 기능·CAN·SHM·프로세스·GUI 회귀 시험 |

실행 진입점은 `apps/<프로그램>/main.cpp`, 구현·헤더는 `src/<모듈>/`에 둡니다. `TaskController`는 별도 런타임 래퍼 없이 입력 → 제어 → 출력 → 주기 대기를 수행하며 모션 제어기를 소유합니다. 각 모션 클래스는 전달받은 YAML 경로에서 자기 설정을 읽습니다. 공통 조건수 상한은 `workspace.maximum_condition_number`, 계산 모델은 최상위 `model_path` 하나를 사용합니다.

## 프로세스 터미널과 SHM 통계

기본 실행은 HAL·TaskController·로거 등 설정된 프로세스마다 터미널을 하나씩 엽니다. 시뮬레이션 프로파일에서는 시뮬레이터 창도 추가됩니다. 창 상단에는 PID·상태·직전 관측 간격의 CPU 사용률·RSS·스레드 수·가동 시간을 **2초마다** 갱신합니다. CPU는 여러 코어를 사용하면 100%를 넘을 수 있습니다. CAN 창에는 기존 모터 상태와 점유율도 같은 고정 영역에 표시합니다.

통계 갱신은 로그 영역을 스크롤하지 않습니다. 실제 stdout/stderr는 하단 로그 영역과 실행별 `logs/run-<epoch_ms>-<daemon_pid>/<process>.log`에 함께 기록합니다. 통계 패널은 이 파일에 섞지 않습니다. 시작 실패 메시지도 기록하며, 정상 종료 시 프로그램을 먼저 종료하고 릴레이가 EOF까지 읽은 뒤 창을 닫습니다. 창 크기가 아직 0이거나 너무 작으면 명시적인 대기/잘림 안내를 표시합니다. 로그는 계속 전달됩니다. 기본 실행은 아래 자동 배치로 충분한 작업 영역을 확보합니다.

화면은 터미널의 별도 화면 버퍼를 사용합니다. 정상 종료·Ctrl+C·오류 종료 시 원래 화면·제목·커서·스크롤 영역을 복원하므로 통계가 셸에 남지 않습니다. 핵심 프로세스 창의 Ctrl+C는 daemon에, 선택적 로거 창의 Ctrl+C는 로거에 전달하며 마지막 종료 로그까지 읽습니다. 화면 크기를 변경하면 최근 메시지 최대 200줄을 다시 배치하고, 전체 로그는 파일에 보존합니다.

설정은 [daemon.yaml](config/daemon/daemon.yaml)에 모았습니다. 경로는 해당 YAML 디렉터리 기준입니다.

```yaml
terminal_exec: /usr/bin/gnome-terminal.real
log_directory: ../../logs
terminal_layout:
  monitor: auto
  columns: 2
  gap_px: 12
  margin_px: 12
  zoom: 0.85
  include_daemon: true
monitoring:
  display_interval_sec: 2.0  # 1초 초과, 최대 60초
  window_sec: 10.0          # 0초 초과, 최대 60초
  sample_hz: 100.0          # 1~1000 Hz; 제어 주기와 무관
```

기본 `terminal_layout`은 **보조 모니터가 있으면 첫 보조 모니터**, 없으면 주 모니터를 선택하며 선택 결과를 `[layout]` 로그에 출력합니다. 운영체제 패널을 제외한 작업 영역에 2열로 배치하고, 마지막 줄에 창 하나만 남으면 그 줄의 전체 폭을 사용합니다. daemon을 일반 GNOME 터미널에서 실행하면 그 창도 배치에 포함합니다. stdout을 파일로 연결하면 실제 호출 창이 없으므로 자식 터미널만 배치합니다.

`monitor`는 `auto`, `primary`, 또는 `xrandr --listmonitors`에 표시되는 이름(예: `DP-2`)입니다. `columns`는 1~8, 여백·간격은 0~200 px, 자식 터미널 `zoom`은 0.5~2.0으로 조정합니다. 기존 호출 터미널의 글꼴 설정은 유지합니다. IDE 내장 터미널처럼 독립된 호출 창을 찾을 수 없는 경우에는 `include_daemon: false`를 명시합니다. 배치는 **시작 시에만** 적용하므로 이후 사용자가 창을 자유롭게 옮길 수 있습니다.

이 기능은 **GNOME/X11 + XRandR** 기준입니다. 고정 문자 수만 요청하지 않고 실제 창 장식·문자 셀 크기를 반영해 위치와 크기를 검증합니다. 창 식별에는 실행별 고유 `WM_WINDOW_ROLE`을 사용해 같은 제목의 다른 창을 이동하지 않습니다. 지정 모니터 없음·지원하지 않는 창 관리자·배치 거부·읽기 어려운 작은 타일(최소 600×360 px)은 오류로 알리며 겹친 창이나 임의의 다른 배치로 대체하지 않습니다. 창 관리자가 없는 시험용 X11은 작업 영역 정보가 없음을 알리고 모니터 경계를 사용합니다. native Wayland 창의 위치 제어는 지원하지 않습니다.

`terminal_title`과 프로그램별 `terminal_exec`은 필요할 때만 덮어씁니다. 터미널을 사용할 수 없으면 실행을 실패로 보고하며 `--no-terminals`로 자동 전환하지 않습니다. 이 옵션을 직접 지정하면 로그 파일/개별 창 대신 현재 stdout/stderr를 사용하고, daemon의 SHM 표는 계속 출력합니다.

Daemon은 기존 SHM 필드만 읽습니다. mutex는 trylock으로 시도하고 사용 중이면 표본을 건너뛰어 `busy skips`에 표시합니다. 통계 계산·화면 출력·파일 쓰기는 잠금 밖에서 수행합니다. SHM 레이아웃과 제어 루프 계측은 추가하지 않았습니다. 이 관측도 짧은 mutex 획득과 CPU 비용은 발생시킵니다.

| 표시 | 계산 및 해석 |
| --- | --- |
| `Age us / command, feedback, model, control` | daemon 관측 시각 − 각 발행 시각. feedback은 HAL 발행 Age이며 실제 모터 수신 Age와 구분합니다. CAN 없는 모드에서는 시뮬레이터가 직접 발행한 피드백입니다. |
| `RX worst age us` | 세 모터 `received_ns` 중 가장 오래된 수신 시각의 Age. 세 축 모두 수신 이력이 있어야 표시합니다. |
| `Obs skips #` | 이전 daemon 관측 이후 증가한 시퀀스 − 1. **관측기가 못 본 발행 수**이며 소비자 손실·CAN 패킷 유실이 아닙니다. |
| `Cmd unconsumed distance #` | 최신 `command_sequence` − HAL의 `consumed_command_sequence`. 최신값 방식의 시퀀스 거리이며 큐 깊이가 아닙니다. CAN 없는 모드에서는 `N/A`입니다. |
| `Pub mean us` | 관측 사이 발행 시각 차이 ÷ 해당 발행 시퀀스 증가량. 다중 작성자의 루트 `motor_sequence`와 명령/피드백 시각을 잘못 짝짓지 않습니다. |
| `Model->control lag us` | `control.published_ns` − `input_model_published_ns`. 입력 모델의 오래됨·제어 계산·스케줄링이 포함되므로 IPC 비용으로 해석하지 않습니다. |

각 행의 최소·최대·평균·sigma는 모두 **현재 시간 윈도우 안의 관측 표본** 기준입니다. sigma는 모집단 표준편차이며, 발행 간격 행에서도 **관측 구간 평균 간격들의 변동**만 나타냅니다. 100 Hz 관측으로 500 Hz의 모든 발행 간격과 실제 주기 지터를 복원할 수 없습니다. 빈 윈도우·아직 발행/수신되지 않은 값은 `N/A`입니다. 관측 횟수·mutex 사용 중 건너뜀·역행한 카운터/시각·잘못된 시각의 개수는 실행 누계입니다. Age로 제어 정지나 응답 지연 판정을 내리지 않습니다.

표는 데이터 Age·발행/입력·시퀀스 거리로 나누고, 각 행에 최신값과 시간 윈도우를 10구간으로 나눈 추세를 표시합니다. 추세 높이는 해당 행의 최소~최대 범위에 맞춘 상대값이며 경고 임계값을 뜻하지 않습니다. 터미널에서는 상태와 구역 제목에 색상을 사용하고, 파일로 출력하면 색상 코드를 넣지 않습니다.

다음 항목은 기존 데이터로 알 수 없어 수치에 넣지 않았습니다.

| 제외한 값 | 정확히 측정하려면 필요한 변경 — 이번에는 미구현 |
| --- | --- |
| Mutex Wait Time | 각 `shm_utils::Lock`의 잠금 요청 직전·획득 직후 시각을 기록해야 합니다. daemon 자신의 trylock 건너뜀으로 다른 프로세스의 대기 시간을 추정할 수 없습니다. |
| Mutex Hold Time | 획득 직후~unlock 직전 구간을 계측해야 합니다. 조건변수 대기는 mutex를 놓으므로 hold 구간에서 제외하고 재획득 후 새 구간으로 처리해야 합니다. |
| IPC Overhead | 작성자의 copy/publish/unlock 비용과 소비자의 lock/read/copy 비용을 각각 계측해야 합니다. 전달 지연을 따로 구하려면 동일 시퀀스의 발행 완료·수신 완료 시각도 연결해야 합니다. 현재 타임스탬프 차이는 순수 IPC 비용이 아닙니다. |
| 발행별 실제 지터·소비자별 누락 수 | 작성자/소비자에서 모든 발행·읽기 이벤트를 계측하고, 로컬 집계값이나 별도 trace ring으로 내보내야 합니다. 최신값 두 슬롯만으로 중간 이벤트를 복원할 수 없습니다. |

## SHM과 주기

공유 영역은 기본 **`/qhrr05` 하나**이며 이름은 daemon 설정의 `shm_name`으로 지정합니다. CAN 사용 여부와 관계없이 daemon이 자식 실행 전에 생성·초기화하고, 모든 자식을 종료한 뒤 해제합니다. HAL·시뮬레이터·제어기·로거는 연결만 하며, 영역이 없으면 명시적으로 실패합니다. 기존 객체는 재사용·삭제하지 않습니다. daemon이 SIGKILL되면 정리할 수 없으므로 남은 객체는 다음 실행에서 충돌로 보고합니다.

[shared_memory.hpp](src/ipc/shared_memory.hpp)에 magic·ABI 버전·버퍼 수·권한·시계·컴파일된 전체 바이트 크기와 `MotorData`·`ModelData`·`ControlData`·`ShmData`를 함께 정의합니다. 연결 시 실제 객체 크기와 헤더의 magic·ABI 버전·크기·버퍼 수를 검사하고 불일치하면 동기화 객체를 사용하기 전에 거부합니다. 현재 ABI는 **16**입니다. 레이아웃이나 필드 의미를 변경하면 `kShmAbiVersion`을 올리고 모든 프로세스를 함께 재빌드·재시작합니다. [control_types.hpp](src/ipc/control_types.hpp)에는 여러 곳에서 재사용하는 `MitCommand`·`MotorFeedback`·Tracking 평면·제어 방식·ILC 및 Magnetic 상태/계측만 남깁니다. 두 헤더는 OpenRobot 코덱을 포함하지 않습니다.

| 영역 | 데이터와 작성자 |
| --- | --- |
| `motors` | `MotorData`: 제어기는 command·command_enabled·reference·debug_torque 배열, HAL은 feedback·motor_torque(Nm)·축별 CAN 상태 배열 갱신 |
| `model` | 제어기가 수신한 엔코더 상태로 MuJoCo 백엔드를 갱신해 M/h/J·중력·마찰·발끝 계산값 발행 |
| `control` | Input·안전레이어를 거친 모드·목표 힘, 제어기의 상태·RFOB 결과, 선택적 조이스틱 입력 |

같은 영역의 작성자는 **mutex 안에서 최신 슬롯을 반대 슬롯으로 복사하고 자기 필드만 갱신한 뒤 발행**합니다. 명령·피드백 sequence와 시각은 별도입니다. robust mutex 소유자 사망은 명시적 오류입니다. 계산·렌더링·파일 쓰기는 잠금 밖에서 수행합니다.

관절 명령·측정값은 `MotorData` 한 곳을 원본으로 사용합니다. `MitCommand`는 position·velocity·kp·kd·tau_ff를 담고 enable 요청은 별도 `command_enabled[i]`에 둡니다. tau_ff는 0만 허용합니다. TaskController는 패킷 인코딩을 호출하지 않으며 프로토콜 범위·양자화 검증은 HAL이 수행합니다. `MotorProfile`과 설정값 복사본은 SHM에 두지 않습니다. HAL이 C4 모터 응답을 설정과 대조하고, 응답의 Kt와 측정 Iq로 `motor_torque[i]`를 계산합니다. 제어기는 이 토크 값을 바로 사용합니다. 그래프는 로컬 표시 샘플을 만들고 로거는 필요한 영역을 한 번의 잠금으로 읽어 CSV 행을 조합합니다.

제어 주파수는 `control.frequency_hz`로 설정합니다. 제어기는 monotonic 시간으로 동작하며 새 측정이나 물리 run/pause를 기다리지 않고 최신 입력을 사용합니다. 물리 간격은 모델의 `timestep`, 그래프·계측 주기는 `plots.sample_hz`의 시뮬레이션 시간 기준입니다. 물리 일시정지 중에도 제어 시간은 진행합니다. 제어 경로는 응답 지연·관측 누락으로 동작을 바꾸지 않습니다. CSV 평가는 **shm_logger**, 읽기 전용 실시간 SHM 관측은 daemon이 담당합니다. 더블버퍼는 모든 중간 프레임의 기록을 보장하지 않습니다.

시뮬레이션 전용 교환은 모두 [simulation IPC](src/ipc/simulation/channel.hpp)의 버전형 Unix `SOCK_SEQPACKET` 경로(`/tmp/qhrr05-simulation-<uid>/<세션>.sock`)를 사용합니다. 실제 위치·속도·토크, 접촉 이벤트·합력, 물체 위치·회전, pause·finished 상태와 승인된 시뮬레이션 조작은 SHM에 넣지 않습니다. 뷰어·로거·웹 브리지가 IPC 상태와 하드웨어 SHM을 로컬에서 조합하며, 느린 구독자는 물리를 막지 않고 누락 수를 확인할 수 있습니다. 자세한 구조는 [시각화 안내](docs/visualization.md)를 참고하십시오.

RFOB는 새 측정에서만 필터를 갱신합니다. 접촉 GT 힘·모멘트와 물체 좌표는 simulation IPC에만 있으며 제어 입력으로 사용하지 않습니다. 현재 관측기는 모델 잔차 기반 발끝 등가 외력 추정입니다. 발끝의 `condim=6` 접촉 모멘트를 3관절 관측으로 힘과 분리할 수 없으며, Momentum Observer는 미구현입니다.

CAN 모드에서는 새 CAN 피드백에서 속도를 차분하고 RFOB를 갱신합니다. 제어기는 읽은 엔코더 상태로 같은 제어 주기 안에서 모델을 갱신합니다. 시뮬레이터가 없어도 이 경로로 계산하며, 외부 `ModelData`는 제어 입력으로 읽지 않습니다. 물체 이동·접촉·해제 같은 시뮬레이션 이벤트는 통일된 simulation IPC로 전달합니다. 수집 주기마다 `contact`·`contact_event_sequence`·`contact_event_time`을 갱신하며 CSV에도 기록합니다. 이 이벤트 때문에 모터 CAN 패킷을 추가하지 않습니다.

## CAN HAL

HAL은 기본 daemon의 필수 프로세스입니다. 통계 창의 `BUS LOAD est.`는 전체 Classic CAN ID를 별도 수신 소켓으로 관측한 점유율 추정치입니다. [HAL 설정](config/can_hal/can_hal.yaml)의 `bus_load_bitrate`는 표시 기준 비트레이트이며 기본값은 1,000,000 bit/s입니다. 인터페이스 속도는 변경하지 않습니다. vcan에서도 같은 기준으로 계산하며 비트 스터핑 최댓값·프레임 오버헤드를 포함합니다. 100%를 넘으면 수치는 유지하고 바만 끝까지 채웁니다.

기본 자동 배치 빌드에는 X11/XRandR 개발 라이브러리(`pkg-config x11 xrandr`)가 필요합니다. 호스트 vcan 준비에는 iproute2·Python 3와 생성/활성화 권한이 필요합니다. 권한이 없으면 sudo 인증을 요청합니다. `--isolated-vcan` 디버깅에는 `unshare`·D-Bus가 추가로 필요합니다. 터미널 실행 파일은 최상위 `terminal_exec`, 창 이름은 프로그램별 `terminal_title`로 지정합니다. 현재 환경은 FD 전달을 지원하는 `/usr/bin/gnome-terminal.real`을 명시했습니다. 실행 파일·인터페이스·창 오류를 다른 경로로 자동 대체하지 않습니다.

명령은 MIT q target·velocity·kp/kd이며 tau_ff는 논리적 0으로 고정합니다. 기본 시뮬레이터가 C4 프로파일 조회·C1 enable·C0 제어/피드백·C2 release에 응답합니다. 시작 시 가상 모터가 늦게 준비되어도 100 HAL 주기마다 프로파일/enable을 재요청합니다. 응답 지연 판정이나 시간 초과 정지는 없습니다. 명령 API·로깅·프로토콜 범위·스토퍼 보정 결정은 [CAN HAL 동작 규약](docs/can_hal.md)에 정리했습니다.

## 검증과 유지 문서

```bash
ctest --test-dir build --output-on-failure
```

CTest에는 격리된 vcan의 코덱 시험과 실제 MuJoCo 가상 모터 폐루프 시험이 있습니다. 후자는 호밍·G/M/T·물체 접촉/해제·CAN 정지 중 우회 금지·Release를 검사합니다. 실제 AK10-9 하드웨어 검증은 아닙니다. 회귀 시험은 사용자 튜닝과 분리한 [고정 설정](tests/fixtures/control.yaml)을 사용합니다.

GUI 시험도 위 명령에 포함합니다. `simulator_gui`는 H/G/M/T/B·1/2/3 조작·목표 힘 변경·일시정지·Ctrl+드래그를, `can_gui`는 CAN 실행·실패·전체 종료·선택적 로거 창 재시작 경로를 묶어 검사합니다. `logging_core/service/daemon`은 MCAP 스키마·5초/2초 덤프·저장 장애·오프라인 분석·독립적 재시작을 검사합니다. `terminal_layout`과 `terminal_statistics`는 창 배치와 화면 복원·로그 보존을 검사합니다. 공통 [GUI 실행 환경](tests/run_gui_test.sh)과 [X11 도우미](tests/gui_test_utils.py)를 사용하며, Xvfb·D-Bus·xdotool·GNOME Terminal이 필요합니다. 시험은 별도 세션과 임시 디렉터리를 사용하며 프로젝트 문서에 스크린샷을 남기지 않습니다.

- [최근 작업과 다음 우선순위](WORK_NOTE.md)
- [제어 모드·궤적·작업공간·가상 Iq 센서](docs/control.md)
- [MCAP·Flight Recorder·CLI·분석과 검증 범위](docs/logging.md)
- [CAN HAL 패킷·계측·영점 보정](docs/can_hal.md)
- [코드 작성·포맷 규칙](docs/code_style.md)
- [모델 구조·출처·라이선스](models/qhrr-05/README.md)
- [MuJoCo GUI 복사 출처와 변경 범위](third_party/mujoco_simulate/SOURCE.md)
