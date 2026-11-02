# Interfaces

- `input/`: Input 명령 수신/응답, 클라이언트, 조이스틱 입력.
- `web_gui/`: 웹 표시용 읽기 전용 상태 브리지.

Cogging Identification 구독 중에는 웹 브리지가 기존 MotorData를 500 Hz로 읽고 새 C0 샘플을 Node 서버로 전달합니다. 실제 시각 기반 q 차분·LPF 후 `감속비 × q`에 대한 균일 각도 보간·차수 FFT를 Node 서버에서 수행하며 브라우저에는 4 Hz 분석 결과만 보냅니다. 제어기·SHM ABI·CAN 명령 경로는 바꾸지 않습니다.

두 입력 경로는 `interfaces` CMake 라이브러리로 빌드합니다. 승인 경로는 Input → Safety → TaskController를 유지합니다. 시뮬레이션 메시지 타입과 전송은 `ipc/simulation`에서 가져오며 별도 IPC 구현을 두지 않습니다.

실행 진입점은 `apps/interfaces/input/main.cpp`, `apps/interfaces/web_gui/main.cpp`이고 Node/React 프로젝트는 `apps/interfaces/web_gui/frontend`에 있습니다. 런처와 실행 파일 이름은 유지합니다.
