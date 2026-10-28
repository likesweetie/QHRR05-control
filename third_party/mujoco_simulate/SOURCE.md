# MuJoCo simulate GUI copy

현재 디렉터리의 GUI 파일은 이 저장소의 [third_party/mujoco/simulate](../mujoco/simulate/README.md)에서 복사했습니다. 원본의 저작권·Apache-2.0 고지와 [라이선스](../mujoco/LICENSE)를 유지합니다. `third_party` 원본은 수정하지 않았습니다.

원본 `main.cc`는 이전 프로젝트의 SHM·PD 코드가 섞여 있어 복사하지 않았습니다. GUI는 복사한 `Simulate`, GLFW adapter, 플랫폼 UI, dispatch 구현을 실제로 사용합니다. [apps/simulation/main.cpp](../../apps/simulation/main.cpp)는 뷰어의 모델·물리·SHM 연결과 계측 발행을 담당하고, [src/simulation/telemetry.hpp](../../src/simulation/telemetry.hpp)에는 그래프를 둡니다. CSV 기록·지연 평가는 별도 `shm_logger` 프로세스가 같은 SHM을 읽어 수행합니다.

복사본 변경은 다음과 같습니다.

- `simulate.h/cc`: 키·리셋·단일 스텝·그래프 표시·3D 장면 장식 연결점, Damping/호밍/중력보상/Magnetic/Tracking/Ball 선택 버튼·실시간 목표 힘 슬라이더와 콜백, 그래프 영역 확보, 창 종료 중 모델 로딩 대기 해제.
- 렌더링과 마우스 선택이 동일한 3D 영역을 사용하도록 맞춰 그래프 패널 추가 후의 선택 좌표 오류를 수정했습니다. 원래의 몸체 선택·Ctrl+드래그 외력 기능을 사용합니다.
- `glfw_adapter.cc`: 초기 창 크기와 제목.
- 3D 장면 갱신 후 뷰어가 최근 2초 발끝 경로를 반투명 장식으로 추가합니다. 그래프의 실제 위치 샘플을 재사용하고 물리 모델·접촉에는 반영하지 않습니다.
- Tracking 평면 단축키 1/2/3(XY/XZ/YZ)은 앱의 기존 키 콜백에서 처리합니다.
- 리셋·수동 단일 스텝도 동일한 IO 경로를 사용하며, GUI에서 제어 법칙을 계산하지 않습니다.

복사 원본·현재 파일 해시는 [source_manifest.json](source_manifest.json)에 기록했습니다. 태스크 컨트롤러용 런타임 래퍼는 사용하지 않습니다.

공 추종은 기존 키·장면 콜백에서 B 단축키와 장식용 반투명 구를 연결합니다. 공은 물리 충돌이나 MuJoCo 직접 제어 입력을 만들지 않습니다.

W 작업공간 오버레이는 앱의 키·장면 콜백과 `src/simulation/workspace_overlay.hpp`에서 처리합니다. FK 유효 셀의 반투명 경계를 비동기로 생성하며 복사한 GUI 원본에는 추가 변경이 없습니다.
