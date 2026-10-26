# 코드 작성·포맷 규칙

C++은 LLVM 기반 [설정](../.clang-format), Python 시험 코드는 [Black 설정](../pyproject.toml)을 사용합니다. 도구 버전은 clang-format 18.1.8 / Black 24.10.0입니다. 빌드·실행 의존성에는 포함하지 않습니다.

## 형식

- 공백 4칸, 100자 기준 줄바꿈. 문자열·주석은 초과할 수 있습니다.
- 중괄호는 여는 문장과 같은 줄, 포인터·참조는 `mjData*`, `const MotorData&`처럼 타입에 붙입니다.
- 짧은 함수·분기·반복문도 본문을 다음 줄에 둡니다. include 순서와 주석 내용은 유지합니다.
- 변수 설정·검증·계산·출력·SHM 발행 등 역할이 바뀌면 빈 줄 하나를 둡니다. 반복·분기가 끝난 뒤의 다음 처리도 구분합니다.
- 같은 역할의 연속 대입과 `if / else`, `try / catch`, 조건·반복 헤더와 본문은 붙여둡니다.

의미에 따른 빈 줄은 편집할 때 직접 넣으며 포맷 도구가 유지합니다.

```cpp
double value = 0;

for (int i = 0; i < 3; ++i)
    value += a[i] * b[i];

return value;
```

## 구조

`apps/`는 실행 진입점, `src/`는 기능별 헤더·구현을 함께 둡니다. 헤더는 `ipc/shared_memory.hpp`, `control/control.hpp`처럼 `src/` 기준 모듈 경로로 포함합니다. `qhrr05_common`은 검색 경로만 제공하고 실제 라이브러리는 CMake에서 명시적으로 연결합니다.

앱 디렉터리와 실행 파일은 `daemon`, `control`, `simulation`, `input`, `can_hal`, `shm_logger`로 맞춥니다. 명세의 관리 CLI `qhrr-logctl`은 `apps/logctl/`에 둡니다. 분리된 구현은 같은 이름의 `src/<모듈>/`에 두고 라이브러리 타깃에는 `_core`를 붙입니다. 공용 모듈 `ipc`, `motion_controllers`, `motor_can_protocol`은 기능 이름을 유지합니다. 설정 경로는 `config/<모듈>/<모듈>.yaml`, 모터 규약은 `config/motor/<모델>.yaml`을 사용합니다. 설정이 필요 없는 앱에는 파일이나 빈 구현 디렉터리를 만들지 않습니다.

TaskController는 모션 컨트롤러 인스턴스를 소유하고 입력·제어·출력·주기 대기를 호출합니다. 각 모션 컨트롤러는 생성자에서 받은 YAML 경로로 자신의 설정을 파싱·검증합니다. 공용 설정 파서나 작은 데이터 전달만을 위한 계약 구조체는 추가하지 않습니다.

SHM 상위 데이터·헤더·버퍼 수·전체 크기·ABI 규약은 `shared_memory.hpp`에 함께 둡니다. 레이아웃이나 필드 의미가 바뀌면 ABI 버전을 올립니다. SHM 이름은 daemon 설정에서 한 번 정하고, daemon이 생성·초기화·해제를 담당합니다. 각 자식은 전달받은 이름으로 연결만 합니다. 여러 곳에서 재사용하는 값 타입·상태 enum만 `control_types.hpp`에 남깁니다. 설정 구조체를 런타임 SHM 데이터 안에 넣지 않습니다. OpenRobot 코덱은 공통 타입을 참조하며 SHM·제어기는 코덱을 참조하지 않습니다.

모터 SHM은 `MotorData`의 직접 배열로 관리합니다. 축별 데이터·CAN 상태를 중첩 구조체로 다시 감싸지 않습니다. daemon의 준비 통지·터미널 출력 같은 작은 기능도 기존 클래스 함수로 둡니다. 설정·자원·프로토콜 오류는 명시적으로 보고하며 다른 실행 경로로 자동 대체하지 않습니다.

## 적용 범위와 실행

직접 작성한 apps/src/tests 코드에 적용합니다. third_party, 복사한 MuJoCo simulate 소스, build 생성물은 [.clang-format-ignore](../.clang-format-ignore)에서 제외합니다. Python은 기존 문자열 따옴표를 유지합니다.

저장소 루트에서 해당 버전의 도구로 실행합니다.

```bash
rg --files apps src tests -g '*.cpp' -g '*.hpp' -g '*.cc' -g '*.h' -0 |
    xargs -0 clang-format -i

for path in tests/*.py; do
    python3 -m black "$path"
done
```
