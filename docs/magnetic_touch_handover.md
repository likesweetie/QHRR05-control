# Magnetic Touch Mode 핸드오버

2026-10-24 · 초기 기능 구현 · `zero_gravity`

## 실행과 조작

실행 중인 기존 daemon을 종료하고 다시 시작합니다. SHM 구조가 확장되었으므로 이전 프로세스와 혼용하지 않습니다.

```bash
./build/daemon --csv /tmp/magnetic_touch.csv
```

CSV 파일이 이미 있으면 명시적으로 실패합니다. 새 경로를 지정하십시오.

1. 기본 호밍이 끝나면 **Magnetic [M]** 버튼 또는 **M**을 누릅니다. `FREE`에서는 위치 유지 없이 중력보상과 작은 관절 감쇠만 적용합니다.
2. 주황색 물체를 **왼쪽 더블클릭**하여 선택합니다. **Ctrl+오른쪽 드래그**로 이동하고, **Ctrl+Shift+오른쪽 드래그**로 수평면에서 이동합니다. 물체를 발끝에 접촉시킨 뒤 조금 더 누르면 자동으로 `CONTACT CANDIDATE → FORCE FOLLOWING`이 됩니다.
3. 접촉 후에는 물체를 천천히 조금씩 움직이십시오. 추종 범위는 접촉 시작점에서 15 mm입니다. 계속 크게 밀면 범위 초과로 FREE로 해제되는 것이 정상입니다.
4. **Target [N]** 슬라이더로 목표 힘을 변경합니다. 기본 3 N, 범위 0..설정 최대값(기본 5 N)입니다. 슬라이더 변경은 관측기·접촉 상태를 초기화하지 않습니다.
5. 물체를 떼면 추정 접촉력이 0.5 N 미만이 되는 시점부터 위치 적분을 즉시 동결합니다. 100 ms 지속되면 FREE로 복귀합니다. 범위·관절 한계·관측 무효 사유도 화면과 로그에 표시합니다.
6. **G**는 자동 전환 없는 기존 중력보상, **H**는 호밍, **Space**는 물리 일시정지, **R**은 물리 리셋과 호밍 요청, **Esc**는 종료입니다. 제어기는 물리 일시정지 중에도 독립 주기로 계속 실행합니다.

![Magnetic Touch GUI](magnetic_touch_preview.png)

## 구현 범위

- 기존 torque actuator와 발끝 **condim=6 유지**. 모델의 기존 damping·frictionloss·질량도 유지했습니다.
- TaskController가 `MagneticTouchController` 핸들을 소유합니다. `ReactionForceObserver`는 MagneticTouchController가 소유합니다. MuJoCo 의존성은 시뮬레이터 쪽에만 있습니다.
- 제어 출력은 **Nm 토크 목표**입니다. `g − Dq̇ + Jᵀ n f_cmd`를 출력하며 접촉 중에만 힘 피드백·제한 admittance·태스크 공간 임피던스 항을 더합니다.
- 관측기는 `M*qddot_est + h − motor_torque − passive_model` 잔차와 명시적 smooth dry-friction 모델을 사용합니다. 가속도는 물리 측정 간격의 qvel 차분이며 `mjData.qacc`를 읽지 않습니다. 새 측정 시퀀스에서만 필터를 갱신합니다.
- 실제 모터 토크는 제한이 적용된 `qfrc_actuator`를 토크 추정치 역할로 사용합니다. q/dq와 같은 프레임의 모델 M/h/passive/J를 발행합니다.
- `mj_contactForce()`의 실제 접촉 힘·모멘트와 물체 좌표는 **ShmSample 검증 채널에만** 있습니다. 제어 입력 `ShmState`에 없으며 감지·방향·힘 제어에 사용하지 않습니다.
- 물체→로봇의 추정 외력 반대 방향을 로봇이 누르는 방향 n으로 래치합니다. GT는 contact frame에서 world frame으로 변환하고 발끝-물체 접촉만 합산합니다.
- 독립 제어 주기와 시퀀스 더블버퍼를 유지합니다. 모드/재시작 시퀀스와 힘 파라미터 갱신을 분리했습니다. 응답 지연 판정은 로거에만 있습니다.

## 화면과 기록

Magnetic 모드의 오른쪽 그래프는 추정/GT/목표/명령 힘, 발끝 실제·목표 위치, 위치 오차·기준 이동량입니다. FOLLOW에서는 래치된 법선 성분을 비교하고 FREE에서는 힘 크기를 표시합니다. 화면에는 상태·해제 사유·실제 접촉 모멘트 크기·이벤트 번호가 표시됩니다.

CSV에는 `magnetic_phase`, `magnetic_reason`, `observer_valid`, `force_est_n`, `force_gt_n`, `force_target_n`, `force_command_n`, `tracking_error_m`, `displacement_m`, `event_seq`, `event_time`, 포화·특이값, 발끝/힘/GT 모멘트/법선/물체 좌표가 추가되었습니다. 로거가 상태 전환도 출력합니다.

## 튜닝 위치

`config/control_config/control_config.yaml`의 **magnetic_touch** 블록을 수정하고 재실행합니다. 각 클래스가 YAML 경로를 받아 자신의 항목을 읽고 검증합니다.

| 항목 | 현재 초기값 / 역할 |
| --- | --- |
| `target_force_n`, `maximum_force_n` | 3 / 5 N |
| `contact_on_n`, `contact_cancel_n` | 1.5 / 1.0 N, 접촉 후보 히스테리시스 |
| `contact_off_n`, `contact_recover_n` | 0.5 / 0.8 N, 해제/재접촉 히스테리시스 |
| `contact_debounce_sec`, `release_sec` | 0.02 / 0.1 s |
| `force_ramp_sec` | 0.15 s, 목표 힘의 부드러운 변경 |
| `force_gain` | 1.0, 힘 오차 피드백 |
| `admittance_m_per_ns` | 0.005 m/(N·s) |
| `tracking_stiffness_n_m`, `tracking_damping_ns_m` | 20 N/m / 5 N·s/m, 접촉 전용 임피던스 |
| `maximum_velocity_m_s`, `maximum_displacement_m` | 0.03 m/s / 0.015 m |
| `joint_damping_nm_s_rad`, `torque_limit_nm` | 관절 감쇠·토크 한계 |
| `workspace_min_m`, `workspace_max_m`, `joint_margin_rad` | 작업영역·관절 한계 접근 제한 |
| `observer.bandwidth_hz` | 20 Hz |
| `observer.regularization_m`, `minimum_singular_value_m` | 감쇠 의사역행렬·특이 자세 판정 |
| `observer.maximum_estimate_n` | 비정상 외력 추정 무효화 한계 |
| `observer.friction_velocity_rad_s` | smooth dry-friction 모델의 속도 스케일 |

## 확인한 결과와 남은 튜닝

- 빌드와 CTest 11건 통과. 관측기 부호/필터/반복 프레임/특이점, 자동 감지·해제, 해제 대기 중 적분 동결, 작업영역·관절·추종 범위, 힘·토크 한계와 live 목표 변경을 검사했습니다.
- 실제 condim=6 모델의 물체 접근·정지·소폭 왕복·제거 시험에서 목표 3 N 구간의 **GT 평균 3.31 N**, 추정 평균 **3.18 N**, GT 힘 RMSE **0.62 N**, 전체 시험 peak **4.91 N**을 관측했습니다. 특정 1 kHz 시험 궤적의 결과이며 일반적인 정확도 보장값은 아닙니다. 사용자의 지시에 따라 추가 게인 탐색은 중단했습니다.
- 실제 GUI 버튼·슬라이더·마우스 물체 이동으로 **FORCE FOLLOWING 진입과 범위 초과 해제**를 확인했습니다. 기존 H/G·리셋·종료·SHM 정리도 검증했습니다.
- **3관절 관측으로 6축 wrench를 분리할 수 없습니다.** condim=6의 접촉 모멘트와 마찰은 등가 힘 추정에 섞일 수 있습니다. GT 모멘트를 함께 표시하는 이유입니다. 현재 값은 발끝 기준 등가 외력입니다.
- QDD 실기 마찰·토크 센싱 오차는 아직 식별하지 않았습니다. Momentum observer는 후속 작업이며 현재는 모델 잔차 방식입니다. 5 N은 힘 명령 한계이며 실제 충격의 상한을 보장하지 않습니다.
- 첫 발행 이전/측정이 없는 상태에서는 제어 루프를 기다리게 하지 않고 중력보상 FREE와 `no measurement` 상태를 표시합니다. headless/일시정지에서도 독립 시간 정책을 유지하므로 정상 실시간 전시와 같은 성능을 가정하지 않습니다.

## 재검증 명령

```bash
ctest --test-dir build --output-on-failure
xvfb-run -a -s '-screen 0 1800x1000x24' python3 tests/gui_magnetic_smoke.py
```

`magnetic_touch_test`는 물리 시뮬레이션 기반 기능/힘 지표를 출력합니다. GUI 시험은 실제 마우스 접촉과 슬라이더를 조작하며 위 미리보기 이미지를 저장합니다.
