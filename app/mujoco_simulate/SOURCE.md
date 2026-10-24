# MuJoCo simulate GUI copy

`simulate/`는 이 저장소의 [third_party/mujoco/simulate](../../third_party/mujoco/simulate/README.md)에서 복사했습니다. 원본의 저작권·Apache-2.0 고지와 [라이선스](../../third_party/mujoco/LICENSE)를 유지합니다. `third_party` 원본은 수정하지 않았습니다.

원본 `main.cc`는 이전 프로젝트의 SHM·PD 코드가 섞여 있어 복사하지 않았습니다. GUI는 복사한 `Simulate`, GLFW adapter, 플랫폼 UI, dispatch 구현을 실제로 사용합니다. 새 `main.cpp`는 뷰어의 모델·물리·IO 연결만 담당하고, `telemetry.hpp`에 그래프·CSV 기능을 둡니다.

복사본 변경은 다음과 같습니다.

- `simulate.h/cc`: 키·리셋·단일 스텝·그래프 표시 연결점, 그래프 영역 확보, 창 종료 중 모델 로딩 대기 해제.
- `glfw_adapter.cc`: 초기 창 크기와 제목.
- 리셋·수동 단일 스텝도 동일한 IO 경로를 사용하며, GUI에서 제어 법칙을 계산하지 않습니다.

복사 원본·현재 파일 해시는 [source_manifest.json](source_manifest.json)에 기록했습니다. 태스크 컨트롤러용 런타임 래퍼는 사용하지 않습니다.
