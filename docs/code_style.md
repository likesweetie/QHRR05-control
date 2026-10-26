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

TaskController는 모션 컨트롤러 인스턴스를 소유하고 입력·제어·출력·주기 대기를 호출합니다. 각 모션 컨트롤러는 생성자에서 받은 YAML 경로로 자신의 설정을 파싱·검증합니다. 공용 설정 파서나 작은 데이터 전달만을 위한 계약 구조체는 추가하지 않습니다.

모터 SHM은 `MotorData`의 직접 배열로 관리합니다. 축별 데이터·CAN 상태를 중첩 구조체로 다시 감싸지 않습니다. daemon의 준비 통지·터미널 출력 같은 작은 기능도 기존 클래스 함수로 둡니다. 설정·자원·프로토콜 오류는 명시적으로 보고하며 다른 실행 경로로 자동 대체하지 않습니다.

## 적용 범위와 실행

직접 작성한 app/include/tests 코드에 적용합니다. third_party, 복사한 MuJoCo simulate 소스, build 생성물은 [.clang-format-ignore](../.clang-format-ignore)에서 제외합니다. Python은 기존 문자열 따옴표를 유지합니다.

저장소 루트에서 해당 버전의 도구로 실행합니다.

```bash
rg --files app include tests -g '*.cpp' -g '*.hpp' -g '*.cc' -g '*.h' -0 |
    xargs -0 clang-format -i

for path in tests/*.py; do
    python3 -m black "$path"
done
```
