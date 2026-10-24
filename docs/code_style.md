# 코드 포맷 규칙

2026-10-24 · 적용 전에 기존 코드와 차이를 비교하여 안내한 규칙입니다.

C++은 [clang-format의 LLVM 스타일](https://clang.llvm.org/docs/ClangFormatStyleOptions.html)을 바탕으로 프로젝트의 공백 4칸 들여쓰기를 유지합니다. Python 시험 코드는 [Black](https://black.readthedocs.io/en/stable/the_black_code_style/current_style.html)을 사용합니다.

| 항목 | 기존 코드 | 적용 규칙 |
| --- | --- | --- |
| 들여쓰기 | 주로 공백 4칸 | 공백 4칸으로 통일 |
| 줄 길이 | 긴 조건식·인자 목록이 한 줄에 혼재 | 100자 기준 줄바꿈 |
| 문장 배치 | 여러 대입·처리를 한 줄에 압축 | 문장마다 줄바꿈 |
| 연산자 | `a*b`, `time-start` 등 간격 혼재 | `a * b`, `time - start` |
| 함수·분기 | 짧은 본문을 한 줄로 압축 | 본문을 다음 줄로 펼침 |
| 빈 줄 | 처리 단계가 연속해서 붙어 있음 | 반복·분기 종료와 변수 설정·검증·계산·출력 경계마다 한 줄 |
| 긴 멤버 선언 | 여러 설정 항목을 한 선언에 나열 | 컨트롤러의 긴 설정 선언을 항목별로 분리 |

중괄호는 여는 문장과 같은 줄에 두며, 포인터·참조는 `mjData*`, `const ShmState&`처럼 타입에 붙입니다. include 순서와 주석 내용은 유지하며, 긴 문자열은 자동 분할하지 않습니다. 문자열과 주석 등은 100자를 넘을 수 있습니다.

예를 들어 다음과 같이 바뀝니다.

```cpp
// 이전
const double dt = time-previous_time_; previous_time_ = time;
for (int i = 0; i < 3; ++i) value += a[i]*b[i];

// 적용 후
const double dt = time - previous_time_;
previous_time_ = time;

for (int i = 0; i < 3; ++i)
    value += a[i] * b[i];
```

## 개념 블록 사이의 빈 줄

- 같은 역할의 변수 선언·초기화·대입을 묶고, 다음 역할이 시작되기 전에 한 줄을 띄웁니다.
- `for`·`while`·조건 분기가 끝나고 다음 처리를 시작할 때 한 줄을 띄웁니다. 중괄호 없는 단일 문장 본문에도 적용합니다.
- 입력 검증, 상태 설정, 수식 계산, 출력 저장, SHM 발행을 각각 빈 줄로 구분합니다. 중첩된 반복문 안에서도 처리 단계가 달라지는 곳에 적용합니다.
- `if / else if / else`와 `try / catch`의 연결 문장은 하나의 흐름으로 유지합니다. 조건·반복 헤더와 그 본문도 붙여둡니다.
- 같은 목적의 연속 대입은 한 블록으로 유지하며, 블록 사이에는 빈 줄 하나를 둡니다.

```cpp
double value = 0;

for (int i = 0; i < 3; ++i)
    value += a[i] * b[i];

return value;
```

의미 경계는 코드를 편집할 때 직접 정리합니다. 기존 clang-format·Black 설정으로 다시 포맷해도 이러한 빈 줄을 유지합니다.

## 범위와 재실행

직접 작성한 `app/`, `include/`, `tests/`의 C++과 `tests/*.py`에 적용했습니다. `third_party/`, MuJoCo 원본 복사본 `app/mujoco_simulate/simulate/`, 생성물 `build/`는 `.clang-format-ignore`에서 제외합니다. Python은 기존 문자열 따옴표를 유지합니다.

규칙은 루트의 `.clang-format`, `.clang-format-ignore`, `pyproject.toml`에 저장했습니다. 사용한 버전은 **clang-format 18.1.8**, **Black 24.10.0**입니다.

해당 버전이 설치된 환경에서 저장소 루트를 기준으로 실행합니다.

```bash
rg --files app include tests -g '*.cpp' -g '*.hpp' -g '*.cc' -g '*.h' -0 |
    xargs -0 clang-format -i

for path in tests/*.py; do
    python3 -m black "$path"
done
```

이 작업에서는 도구만 `/tmp/qhrr05-format-packages`에 설치했습니다. 프로젝트 실행·빌드 의존성에는 추가하지 않았습니다.
