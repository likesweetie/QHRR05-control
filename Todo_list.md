# Todo_list

## 의존성 및 실행 환경 준비

2026-10-31 대화에서 확인된 문제를 기준으로 정리했습니다. 설치 명령을 안내한 것만으로는 완료로 표시하지 않습니다.

- [x] **zstd/lz4 개발 패키지 누락 해결 확인.** `Could NOT find zstd` 오류 당시 `libzstd-dev`, `liblz4-dev`가 모두 없었습니다. 이후 CMake 설정과 전체 빌드는 통과했습니다.
- [ ] **신규 환경용 네이티브 의존성 설치 안내 보완.** README에 `sudo apt-get install libzstd-dev liblz4-dev`와 설치 후 CMake 재설정·빌드 명령을 추가합니다. 일반 런타임 라이브러리만으로는 헤더와 링크용 파일이 준비되지 않는다는 점을 명시합니다.
- [x] **Git 메타데이터 없는 배포 폴더의 빌드 중단 해결.** `git rev-parse HEAD`의 종료 코드 128을 치명적 오류로 처리하던 CMake를 수정했습니다. Git 정보를 얻지 못하면 경고와 `git_commit=unknown`으로 빌드를 계속합니다.
- [ ] **Node.js 22.12 이상과 npm 설치·버전 확인.** `web_gui.log`에서 `required executable not found: node`를 확인했습니다. `node --version`, `npm --version`으로 확인하고, 미설치 또는 낮은 버전이면 설치합니다. 실제 설치 완료 여부는 아직 확인하지 않았습니다.
- [ ] **daemon 실행 환경의 Node.js PATH 확인.** nvm을 사용하는 경우 해당 셸에서 nvm을 로드하고 Node를 선택한 뒤 daemon을 실행하는 절차를 README와 웹 GUI 문서에 추가합니다. daemon이 시작한 `web_gui`에서도 `node`, `npm`을 찾는지 확인합니다.
- [ ] **웹 프런트엔드 의존성 설치·검증.** 프로젝트 루트에서 `npm --prefix apps/interfaces/web_gui/frontend ci --no-audit --no-fund`로 잠금 파일 기준 설치 후, `npm --prefix apps/interfaces/web_gui/frontend ls --depth=0`과 `npm --prefix apps/interfaces/web_gui/frontend run build`를 확인합니다. 현재 설치·빌드 완료 여부는 아직 확인하지 않았습니다.
- [ ] **웹 GUI 최초 준비 절차 안내 보완.** Node.js와 npm을 먼저 준비해야 한다는 점을 명시합니다. `./scripts/run_web_gui.sh --setup`은 브리지 빌드·npm 의존성 설치 후 웹 서버까지 실행하며, 실행 시 활성 daemon 세션이 필요하다는 점도 안내합니다.
- [ ] **조이스틱 장치 준비·경로 확인.** `input.log`에서 `/dev/input/js0`의 `No such file or directory` 오류를 확인했습니다. 조이스틱 연결과 Linux 장치 인식을 확인하고, 실제 장치 경로가 다르면 `config/input/input.yaml`의 `device`를 맞춥니다. 조이스틱을 사용하지 않는 운용에서는 선택 프로세스 `input`의 실행 여부를 명시적으로 설정하는 절차를 정리합니다.

CAN 인터페이스의 DOWN/비트레이트 미설정과 C4 모터 프로파일 불일치는 패키지 의존성 문제가 아니므로 이 목록의 설치 작업에는 포함하지 않았습니다.
