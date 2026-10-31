> `logging.runtime_mode: session`은 daemon이 선언한 시뮬레이션 기능을 따릅니다. 연결 실패로 모드를 바꾸지 않습니다. supported/valid/coordinate_valid, 모델 입력 sequence, 구동 상태와 fault도 함께 기록합니다.

# 텔레메트리 기록과 분석

`shm_logger`는 기본 daemon의 선택적 프로세스입니다. **500 Hz 상태·10 Hz 건전성·이벤트·세션 정보를 DataTamer 2.0.0 MCAP에 기록**하고, 기본 5초 이전/2초 이후 Flight Recorder를 함께 운영합니다. 하드웨어 상태는 SHM ABI 16, 시뮬레이션 전용 상태는 별도 simulation IPC로 받으며 기록을 위해 제어 루프에 계측을 추가하지 않습니다. `config/shm_logger/shm_logger.yaml`에서 켜기·끄기·주기·버퍼·저장 정책을 조정합니다.

## 빌드와 시작

CMake 3.22 이상, 기존 C++17 툴체인과 OpenSSL·zstd·lz4 개발 라이브러리가 필요합니다. DataTamer 내부만 C++20으로 빌드하며 프로젝트의 공개 코드는 C++17을 유지합니다. [고정 의존성](../third_party/logging_dependencies.json)을 저장소에 포함하므로 CMake가 네트워크에서 소스를 가져오지 않습니다.

분석 도구와 해당 통합 테스트에는 별도 Python 환경을 권장합니다. 기존 사용자 환경을 변경하지 않습니다.

```bash
python3 -m venv build/analysis_env
build/analysis_env/bin/pip install -r tools/analysis/requirements.txt
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug \
  -DQHRR_ANALYSIS_PYTHON="$PWD/build/analysis_env/bin/python"
cmake --build build --parallel 2
./scripts/run_daemon.sh
```

`venv/ensurepip`가 없고 `uv`가 설치된 환경에서는 위 환경 생성·설치 두 줄을 명시적으로 다음 명령으로 대체합니다.

```bash
uv venv --python /usr/bin/python3 build/analysis_env
uv pip install --python build/analysis_env/bin/python -r tools/analysis/requirements.txt
```

기본 daemon은 실제 사용한 daemon 설정 경로와 SHM 이름을 로거에 전달합니다. SHM 생성·해제는 계속 daemon만 담당합니다. `--without-can` 경로에서도 MCAP 기록이 동작합니다. 수동 실행은 다음과 같습니다.

```bash
./build/shm_logger --config config/shm_logger/shm_logger.yaml \
  --shm-name /qhrr05 --daemon-config config/daemon/daemon.yaml
./build/qhrr-logctl status
./build/qhrr-logctl mark "contact-test"
./build/qhrr-logctl trigger
./build/qhrr-logctl health
./build/qhrr-logctl stop
./build/qhrr-logctl start
./build/qhrr-logctl sessions
```

`stop`은 서비스를 종료하지 않고 현재 세션을 마무리합니다. 이후 `start`는 새 디렉터리와 새 파일을 만듭니다. `logging.enabled: false`이면 `start`도 명확히 거부합니다. 기본 CLI는 기본 daemon 설정의 SHM 이름을 읽습니다. 사용자 설정·다른 SHM을 사용하면 시작 로그의 소켓 경로를 `--socket PATH`로 지정합니다. 소켓은 `/tmp/qhrr05-log-<uid>-<shm_name>.sock`, 소유자 권한 0600이며 같은 UID만 명령을 보낼 수 있습니다. 중복 서버는 거부하고, 강제 종료 후 남은 소유자 소켓을 제거할 때는 메시지를 출력합니다.

기존 CSV는 명시적인 진단 경로로 유지합니다. MCAP 실패를 CSV로 자동 대체하지 않습니다.

```bash
./build/shm_logger --csv /qhrr05 /tmp/new-telemetry.csv
./build/shm_logger --can /qhrr05 /tmp/new-can.csv
```

## 저장과 장애

기본 저장소는 `logs/telemetry/`입니다. 세션 이름은 **UTC 시각·실험 라벨·PID·monotonic 시각**으로 구분합니다.

```text
<session>/
  session.json              # 빌드 소스 버전·dirty·시각 대응·설정 해시·종료 결과
  config/                   # 실제 사용 설정의 복사본
  model_manifest.json       # 모델 디렉터리 XML/자산 SHA256
  schemas.txt, units.json    # 명시적 자료형과 단위
  recording.mcap
  recording_1.mcap           # 시간 또는 크기에 따른 분할
  faults/fault_1.mcap
  faults/fault_1.mcap.json    # 트리거·성공 여부·실제 기록 범위·잘림 여부
  quality_report.json
  active.lock               # 다른 로거가 사용 중인 세션을 삭제하지 않는 파일 잠금
```

기본 분할 한계는 **600초 또는 512 MiB**, 총 예산은 **10 GiB**, 최소 여유 공간은 **5 GiB**입니다. 보존 기간 제한은 없으며, 용량과 여유 공간 조건을 만족하면 오래된 기록도 유지합니다. 활성 파일·다른 로거의 활성 세션·`.protect` 파일이 있는 세션을 보호합니다. 기본적으로 장애 MCAP도 보호합니다. 오래된 닫힌 MCAP을 FIFO로 삭제하고, 공간 확보가 필요하면 기록 파일이 없는 종료 세션의 메타데이터도 정리합니다. 보호 파일 때문에 예산을 지킬 수 없으면 저장 오류를 표시하고 연속 기록을 중단하며, 새 장애 덤프도 거부합니다. 다른 프로세스의 제어 상태는 바꾸지 않습니다.

크기/여유 공간 검사는 관리 주기로 수행하므로 마지막 MCAP 청크·진행 중 덤프·검사 사이에 기록된 데이터만큼 한계를 일시적으로 초과할 수 있습니다. 파일 분할은 `MCAPSink::restartRecording()`으로 수행하며 활성 경로가 retention에 의해 삭제되지 않게 관리합니다. 압축 기본값은 꺼져 있습니다. 자료형·필드 해석이 가능한 **비압축 바이너리 직렬화**이며, C++ 구조체의 메모리 바이트를 그대로 덤프하지 않습니다.

로거는 `INITIALIZING → READY → RECORDING`, 오류 시 `DEGRADED/FAULT`로 표시합니다. 일반 sink 오류가 5초 이상 지속되면 FAULT로 표시합니다. 상태 확인 소켓과 가능한 RAM 이력은 계속 유지합니다. 파일 오류는 stderr·이벤트·건전성·가능한 세션 메타데이터에 남깁니다. 저장 장치 자체가 쓰기를 거부하면 그 오류를 같은 장치에 보존할 수는 없습니다.

daemon의 로거 항목은 `critical: false`, `restart_limit: 3`, `restart_delay_ms: 1000`입니다. 로거 실행 실패·비정상 종료·로거 창 종료는 다른 프로세스를 종료시키지 않으며, 새 프로세스로 제한된 횟수만큼 재시작합니다. 재시작은 새 세션으로 기록하고 기존 파일을 덮어쓰지 않습니다. 핵심 CAN·제어·시뮬레이터 프로세스의 종료 정책은 유지합니다.

## 수집·시각·지표의 의미

시뮬레이션 모드에서는 통일된 simulation IPC를 nonblocking으로 구독해 GT·물체·접촉·pause/finished를 받습니다. 하드웨어 모드는 이 구독을 생성하지 않습니다. 별도 수집 스레드가 `pthread_mutex_trylock()` 후 현재 게시 슬롯의 논리 데이터만 복사하고 즉시 해제합니다. 등록·직렬화·파일/소켓 처리는 SHM 잠금 밖에서 수행합니다. 500 Hz 정상 수집 경로는 고정 로컬 객체와 사전 할당한 DataTamer 풀을 사용합니다. 느린 sink가 풀을 모두 점유하면 스냅샷을 버리고 계측합니다. 로거가 잠근 동안 제어기가 기다리는 짧은 시간까지 없애는 설계는 아닙니다.

| 채널 | 내용 |
| --- | --- |
| `qhrr/state` | Motor/Model/Control/Simulation 필드·시퀀스·원본 게시/수신 시각·수집 시각·SHM 플래그 |
| `qhrr/health` | 로거 자신의 mutex 시간·수집 누락·시퀀스 불연속·DataTamer 풀/sink·디스크/RAM 이력 상태 |
| `qhrr/events` | 모드·접촉·관측 오류·트리거 수락/거절·회전·사용자 마커·파일 오류 |
| `qhrr/session` | 세션 시작 메타데이터 JSON |

MCAP 메시지 시각은 `CLOCK_MONOTONIC` 수집 시각입니다. SHM의 원본 시각은 별도 필드에 보존하며, 세션의 UTC↔monotonic 대응 관계로 실험 간 비교를 할 수 있습니다. 서로 다른 채널이 같은 제어 주기에 만들어졌다고 가정하지 않습니다.

`motor/HR/...`, `motor/HP/...`, `motor/KP/...` 별칭에는 단위를 이름에 넣었습니다. 원본 구조체 필드와 배열도 모두 기록합니다. SHM에는 Motor/Model/Control만 있으며, simulation 필드는 별도 IPC에서 받은 로컬 스냅샷입니다. `simulation/ipc_sequence`와 `simulation/finished`를 기록하며 기존 녹화 파일의 `shm/simulation_sequence`는 분석 도구에서 계속 읽을 수 있습니다. 수집 후 원본이 바뀌어도 이미 복사한 프레임은 영향을 받지 않습니다. `runtime_mode: hardware`이면 GT 구조체를 스키마에서 제외합니다. simulation 모드에서도 첫 유효 게시 전에는 GT 값을 비활성화하며 `simulation/valid`를 제공합니다.

건전성의 mutex 수치는 **로거의 trylock 시도 시간과 복사+unlock 시간**입니다. 제어기/HAL의 mutex wait/hold는 아닙니다. `source_sequence_gaps`는 motor 루트/model/control command/simulation/motor command/motor feedback 순서의 **이 관측기가 놓친 게시 수**이며 CAN 손실률이 아닙니다. 데이터 Age는 아직 값이 없거나 하드웨어 GT이면 무효로 표시합니다. `accepted_output_rate_hz`는 sink 큐가 수락한 state 비율, `sink_delivered`는 모든 채널·sink의 전달 합계입니다. 실제 파일별 sample rate는 아래 inspect로 확인합니다.

자동 트리거는 CAN 피드백 지연, 필수 채널 stale, robust mutex 복구, Magnetic 관측 무효, 관절 한계 위반, 공유 실패 플래그를 관측할 때 작동합니다. 제어 모드 변경이나 정지 명령을 내리지는 않습니다. 정확한 `CONTROL_DEADLINE`, 독립 heartbeat, 실패 프로세스가 원인인 `SIMULATION_FAILURE`는 현재 ABI로 구분할 수 없어 추정값을 해당 이름으로 기록하지 않습니다. 이 기능에는 생산자 측 명시적 이벤트/계측 게시가 필요합니다.

SHM보다 로거가 먼저 시작되면 최대 10초 기다린 후 명확한 오류 상태를 표시합니다. ABI 불일치는 기록을 거부합니다. 실행 중 SHM이 교체되면 기존 객체를 새 세션으로 오인하지 않고 수집을 중단하며 로거 재시작을 요구합니다. robust mutex owner death에서는 잠재적으로 불완전한 표본을 버리고 복구 사실을 기록하며 제어 플래그를 변경하지 않습니다.

Flight Recorder의 기본 데이터 용량 64 MiB는 내부 덤프 버퍼 때문에 약 두 배의 RAM을 사용합니다. 풀이 차면 오래된 표본이 빠질 수 있으므로 실제 시작·종료 시각, 용량에 의한 잘림, 요청 거절을 확인해야 합니다. 종료 시 진행 중인 post-trigger 구간은 그때까지의 자료로 마무리하며 `post_interval_complete`에 표시합니다. 강제 종료·전원 차단은 RAM 이력을 잃을 수 있습니다.

## 오프라인 분석과 재생

```bash
./build/qhrr-log-inspect logs/telemetry/<session> --fields
./build/qhrr-log-inspect logs/telemetry/<session> --faults
./build/qhrr-log-analyze logs/telemetry/<session> --fields
./build/qhrr-log-analyze logs/telemetry/<session> \
  --signals motor/HR/position_rad motor/HR/command/position \
  --align-hz 200 --csv /tmp/aligned.csv --plot /tmp/position.png \
  --report /tmp/analysis.json
./build/qhrr-log-analyze <session-a> --compare <session-b>
```

inspect는 MCAP·스키마를 읽고 채널별 개수/실제 시간 범위/기록률, 시퀀스/시각 역행, 설정 복사본 해시를 검사합니다. analyze는 Pandas/NumPy 변환, 선택 신호의 공통 시간축 보간, 모터 목표/실제 오차, 유효한 RFOB-GT 힘 오차, 관측된 인접 제어 시퀀스의 타이밍 통계, CSV·PNG·세션 비교를 제공합니다. 출력 파일은 덮어쓰지 않습니다. 보간은 외삽하지 않으며 enum/상태 신호를 연속량으로 보간하는 용도로 사용하지 않습니다.

PlotJuggler 4+에서 MCAP 채널을 연 뒤 [기본 레이아웃](../tools/plotjuggler/layouts/qhrr05.xml)을 불러옵니다. Motor·Observer·Magnetic Touch·System Health 탭이 있습니다. MCAP loader와 DataTamer parser 플러그인이 필요합니다. 공식 플러그인의 `채널/필드` 명명 규약과 XML 구조를 확인했지만, **이 개발 환경에는 PlotJuggler가 없어 실제 GUI 로드·그래프 표시는 미검증**입니다. hardware 로그에는 GT 곡선이 없습니다.

이 분석 경로는 실제 SHM·CAN에 접근하지 않습니다. 오프라인 파일 재생은 PlotJuggler 또는 Python에서 수행합니다. 옵저버 알고리즘 재실행, 생산자 무손실 링, 원시 CAN 프레임 전체 기록, 실제 프로세스/클록을 재현하는 System Replay와 웹 GUI는 구현 범위에서 제외했습니다.

## 검증과 다른 장비에서의 성능 확인

```bash
ctest --test-dir build --output-on-failure
./build/qhrr-log-benchmark --duration 60 --output /tmp/qhrr-log-benchmark
./build/qhrr-log-benchmark --rig simulation --duration 60 --output /tmp/qhrr-sim-benchmark
./build/qhrr-log-benchmark --rig can --duration 60 --output /tmp/qhrr-can-benchmark
```

벤치마크는 같은 CSV 관측기를 둔 baseline / continuous / continuous+flight를 비교합니다. synthetic은 고정 500 Hz 생산자, simulation은 명시적 CAN 없는 MuJoCo, can은 **격리된 vcan**의 MuJoCo/CAN 폐루프입니다. `--cases continuous_flight`로 일부만 선택할 수 있으며 synthetic의 `--inject-contention`은 생산자 mutex를 의도적으로 오래 점유합니다. 임시 설정은 출력 디렉터리에 남기며 원래 튜닝 파일은 수정하지 않습니다. synthetic 시험에는 `BUILD_TESTING=ON`으로 빌드한 시험 생산자가 필요합니다.

2026-10-26 synthetic continuous+flight 1분 시험: state **30,027개**, 파일 기록률 **499.9995 Hz**, SHM 잠금 누락·pool/drop·sink 오류 **0회**. 로거 자신의 trylock 평균 **0.438 μs**, 복사+unlock 평균 **0.884 μs**, 최대 **30.947 μs**였습니다. 이 한 번의 결과는 실제 하드웨어의 상한 보장이 아닙니다.

통합 시험은 실제 시간의 5초/2초 덤프·중복 요청·파일 분할·mutex 경합/stale·저장 예산/권한 오류·GT 생략·CLI·메타데이터·Python CSV/PNG 출력·강제 종료 후 새 세션 재시작을 검사합니다. C++ 시험은 모든 도메인 스키마·중첩 배열/enum 왕복, bounded pool, 링 용량 부족과 종료 중 덤프, FIFO·보호 파일, ABI·SHM 교체·owner death를 검사합니다.

기록된 인접 게시만으로 모든 제어 주기의 P99/P99.9나 생산자 mutex 시간을 복원할 수 없습니다. 실제 플랫폼의 로깅 전후 지터 예산·디스크 stall 상한·PlotJuggler 동시 부하 시험은 이후 대상 장비에서 수행합니다. 헤드리스 MuJoCo 시험은 물리 시간을 빠르게 진행할 수 있으므로 합성 생산자의 wall-clock 500 Hz 시험과 구분합니다.

참조: [DataTamer 매뉴얼](https://github.com/facontidavide/data_tamer/blob/0c48ed05536699ca761bea9b46365c6fe805358d/docs/USER_MANUAL.md), [MCAP 사양](https://mcap.dev/spec), [PlotJuggler 공식 플러그인](https://github.com/PlotJuggler/PlotJuggler/tree/main/plotjuggler_plugins).
