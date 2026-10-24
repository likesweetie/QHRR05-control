# MuJoCo 3.3.2

이 디렉터리는 MuJoCo 런타임·헤더·시뮬레이터 소스를 포함합니다. 라이선스와 외부 고지는 `LICENSE`, `THIRD_PARTY_NOTICES`에 있습니다.

`simulate/main.cc`에는 과거 프로젝트의 SHM 연결과 선택적 PD 코드가 남아 있습니다. 변경 없는 벤더 코드가 아니며 현재 빌드·실행 경로에는 포함하지 않습니다. 현재 GUI는 `app/mujoco_simulate/simulate/`로 복사해 사용하며, `app/task_controller/`는 IO·제어 수퍼루프만 담당합니다.

이전 프로젝트 로봇과 로봇 예제는 제거했으며, 현재 프로젝트 모델은 [models/qhrr-05](../../models/qhrr-05/README.md)에 있습니다. `model/`에는 일반 물리 예제만 유지합니다.
