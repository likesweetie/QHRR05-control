# 웹 대시보드

프로젝트 루트의 `dashboard.zip`에 들어 있는 프런트엔드 레이아웃과 CSS를 React 화면에 옮겼습니다. 압축본의 Python 서버·SocketCAN 송신·SHM 구현은 사용하지 않습니다. 기존 Node WebSocket 서버와 C++ 읽기 전용 브리지를 유지합니다.

## 화면

- **Overview:** 실제 구동 상태, 현재/선택 모션, 모터별 Enable, HAL이 소비한 MIT 명령, CAN 통계, 관절각 다이얼, 모델 FK와 명령 이력.
- **Processes:** daemon이 관리하는 실제 프로세스의 PID·CPU·메모리·가동 시간·실행 명령과 시작/중지.
- **SHM Data:** ABI·크기·발행 번호, Motor/Model/Control 데이터와 Raw JSON. 잠금이나 내부 포인터는 전송하지 않습니다.

새 기능은 모터 Enable/Disable과 프로세스 관리로 한정합니다. Raw CAN 송신, 수동 C3/영점 오프셋·프리셋, 임의 모터 폴링, IMU와 별도 ESTOP 상태 머신은 추가하지 않습니다. 순차 스토퍼 제로셋, 기존 모션·Tracking 평면·목표 힘 입력은 유지합니다.

취득 불가·미수신·오래됨·미보정 상태를 숫자 0으로 대신하지 않습니다. CAN Load는 설정된 모터 ID의 TX/RX 카운터와 Classic CAN 최대 스터핑 기준의 추정치입니다. 전체 버스의 다른 노드·오류·재전송은 포함하지 않습니다. Applied MIT Command 표의 HAL 소비 번호도 모터 실제 적용 ACK와 구분합니다.

## 실행

```bash
cmake --build build --parallel 4
./scripts/run_daemon.sh                 # 준비된 물리 CAN 세션
# 또는 ./scripts/run_simulation.sh      # vcan + MuJoCo OpenGL GUI
# 또는 ./scripts/run_hw_deploy_test.sh  # 가상 모터 + OpenGL
# 웹 GUI도 자동 시작: http://127.0.0.1:8080
```

최초 의존성 준비는 `./scripts/run_web_gui.sh --setup`입니다. 프런트엔드는 실행 때 빌드하며 보통 시작에서는 네트워크 설치를 하지 않습니다. URL은 `http://127.0.0.1:8080`입니다. 기본 물리·시뮬레이션·시각화 daemon 설정에 선택 프로세스 `web_gui`를 등록했습니다. 웹 오류는 로그로 표시하며 로봇 세션을 종료하지 않습니다. 수동 실행이 필요하면 `run_web_gui.sh`를 별도로 사용할 수 있습니다.

```bash
./scripts/run_web_gui.sh --shm-name /qhrr05 --port 8080 \
  --control-config config/control/control.yaml
```

데몬이 종료되어 SHM이 삭제되거나 같은 이름의 새 세션으로 교체되면 수동 실행한 웹 서버도 이유를 출력하고 종료합니다. 스택만 중지한 경우에는 같은 SHM 세션이 유지되므로 웹에서 다시 시작할 수 있습니다. 이미 다른 웹 서버가 8080 포트를 사용 중이면 명시적인 오류로 중단합니다. 이전 웹 서버를 종료하거나 `--port`로 다른 포트를 지정해야 하며, 자동 포트 변경이나 기존 서버 강제 종료는 하지 않습니다.

프로세스 관리와 축별 요청 필드 추가로 **SHM ABI 16**을 사용합니다. 이전 세션은 모든 참여 프로그램을 새 바이너리로 다시 시작해야 합니다. `run_visualization.sh`와 웹 서버 실행 파일 경로는 유지합니다.

## 모터 Enable과 제어 입력

`Z`로 Zero set을 선택하고 `A`로 전체 Arm하면 기존대로 탐색합니다. `H/G/M/T/B/D`, `1/2/3`, 목표 힘 제어도 기존 Input 경로를 사용합니다. `X` 또는 Release는 전체 출력을 해제합니다.

개별 Enable/Enable All은 선택한 모터에 C1과 중립 MIT 명령(kp=kd=tau=0)을 전달하고 **ARMED**로 대기합니다. 실제 모션은 전체 Arm/Run으로 시작합니다. 활성 모션에서 한 축을 Disable하면 모션을 중단하고 남은 축도 중립 출력으로 바뀝니다. FAULT 중 재-enable은 거부하며 Clear fault 후 다시 명령해야 합니다.

웹 모터 명령은 `WebSocket → 기존 Node 서버 → Input → Safety → TaskController → SHM → CAN HAL` 경로입니다. 서버는 SHM이나 CAN에 직접 명령을 쓰지 않습니다. 승인·제어기 반영·동작 완료를 구분하며 연결이 끊긴 요청을 자동 재전송하지 않습니다.

## 프로세스 관리

프로세스 관리는 해당 프로세스가 멈춰도 사용할 수 있어야 하므로 기존 daemon에 별도 관리 엔드포인트를 둡니다. 경로는 `/tmp/qhrr05-daemon-<uid>/<session>.sock`이며 사용자 소유 0700 디렉터리와 SO_PEERCRED로 같은 UID만 허용합니다. PID·실행 파일·셸 명령을 받지 않고 daemon 설정에 등록된 프로세스 이름만 받습니다.

- 선택 프로세스(input/logger 등)는 개별 Start/Stop을 지원합니다. 수동 Stop은 자동 재시작을 억제합니다.
- 핵심 프로세스(control/CAN/가상 장치 등)는 전체 스택 단위로 시작·중지합니다. 개별 중지는 거부합니다.
- 스택 중지 전 모든 모터에 Release가 반영되어 있어야 합니다. daemon이 출력 활성화를 우회하지 않습니다.
- 스택을 중지해도 daemon·SHM·웹은 유지합니다. `keep_on_stack_stop: true`인 관리 UI는 재시작 때 중복 실행하지 않습니다. 다시 시작하면 새 관측/보정 상태에서 **DISARMED**로 시작합니다.
- 예상치 못한 핵심 프로세스 실패의 기존 전체 종료 정책은 유지합니다.

관리 엔드포인트가 없는 구버전 daemon에서는 이유를 표시하고 관리 버튼을 비활성화합니다. 다른 PID에 신호를 보내는 방식으로 대체하지 않습니다.

## 검증

```bash
npm --prefix apps/interfaces/web_gui/frontend test
npm --prefix apps/interfaces/web_gui/frontend run test:browser
ctest --test-dir build --output-on-failure -R '^(hardware_deploy|safety_layer|control_io|daemon_lifecycle|can_hal_lifecycle|logging_service)$'
```

웹 통합 시험은 실제 Input·TaskController와 실제 daemon 자식 프로세스로 명령·프로세스 관리를 확인합니다. 브라우저 시험은 320/390/768/1280/1440px 화면, 키보드, 입력 편집 중 단축키 차단, 미적용 목표 힘, 연결 끊김을 확인합니다. 실제 모터를 구동하지 않습니다.
