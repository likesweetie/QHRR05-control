# 제어 모드와 시뮬레이션

현재 실행/Arm/Release와 데이터 상태는 [README](../README.md)를 우선 확인하십시오. 빌드·기본 실행은 [README](../README.md), 패킷 규약은 [CAN HAL](can_hal.md)을 참고하십시오. 설정을 변경하면 데몬을 재시작합니다. 아래 알고리즘 설명과 예시 수치는 현재 사용자 튜닝 파일의 값을 대신하지 않습니다.

## 시뮬레이션 엔코더 원점

[simulation.yaml](../config/simulation/simulation.yaml)의 `motor.require_zero_position_setup`을 사용합니다. 기본값 `false`는 제로셋 없이 모델 관절각을 그대로 CAN 위치로 반환합니다(패킷 양자화 오차 포함). 이때 호스트 `position_offset`은 0이며 enable 전에 관절이 움직여도 좌표가 바뀌지 않습니다.

`true`는 모터 생성 시 축별 초기 표시 각도를 −π~π rad에서 독립적으로 뽑습니다. 시뮬레이션 입력 인터페이스가 이 정책을 **일반 Zero set(모드 6) 요청**으로 전달합니다. 모든 세션은 DISARMED로 시작하며 Arm 이후에 탐색합니다. enable/release·일시정지·물리 리셋은 임의 원점을 다시 뽑거나 런타임 보정값을 지우지 않습니다. 세 축 보정 전에는 H/G/M/T/B 요청을 거부하며, 거부한 요청을 보정 후 뒤늦게 실행하지 않습니다. D로 중단하고 Z로 처음부터 재시도할 수 있습니다.

### 스토퍼 기반 Zero set

[control.yaml](../config/control/control.yaml)의 `zero_set`에서 HR(hip) → HP(thigh) → KP(calf) 순서로 탐색합니다. 원시 CAN 엔코더 좌표에서 선택한 축의 위치 목표만 `direction × speed_rad_s × dt`씩 이동시키며 나머지 축은 진입/후퇴 자세를 유지합니다. 중력·모델 참값·역기구학을 이용해 미지의 원점을 추측하지 않습니다.

접촉은 **탐색 방향의 측정 토크 ≥ contact_torque_nm**, **|관절 속도| ≤ contact_velocity_rad_s** 조건이 `contact_duration_sec` 동안 새 피드백에서 유지될 때 확정합니다. 측정 토크는 CAN Iq × 검증된 Kt이며, 목표 토크나 시뮬레이션 접촉 GT를 사용하지 않습니다. 접촉 후 `backoff_rad`만큼 물러나 정착한 뒤 다음 축으로 넘어갑니다. 완료하면 후퇴한 자세를 유지하며 H 등 일반 모션을 선택할 수 있습니다.

- 기본 탐색 방향: `[-1, -1, -1]`.
- 스토퍼 관절각 `stop_position_rad`: `[-0.872665, -2.984513, -2.722714]` rad. 현재 MJCF의 해당 관절 한계에 맞춘 값입니다. 스토퍼를 모든 축의 0 rad로 취급하지 않습니다.
- 런타임 변환: `position_offset = stop_position_rad - measured_encoder`, `joint_position = measured_encoder + position_offset`. MIT 목표에는 반대 변환을 적용합니다.
- `maximum_push_nm`은 보정 중 접촉/유지 노력 상한으로 공통 `limits.torque_nm` 이하여야 합니다. 위치 오차도 제한하여 스토퍼에서 목표가 무한히 누적되지 않게 합니다.
- 이동량·축별 시간 초과, 피드백 중단, 비유한 입력은 FAULT로 표시하고 모든 모터 출력을 해제합니다. 일시정지 중에는 탐색/접촉 판정 시간도 정지합니다.

**저장은 실행 중 `MotorData.position_offset`에만 합니다.** CAN C3나 실제 모터 ROM/EEPROM 쓰기를 전송하지 않으며 설정 파일도 수정하지 않습니다. SHM 세션을 새로 만들면 보정값이 사라지고 다시 탐색합니다. 가상 모터의 수동 C3 구현은 그대로 유지됩니다. 직접 effort 디버그는 가상 관절 관측과 effort를 제공하며, 전류·온도는 취득 불가입니다. 모션은 출처 대신 필요한 관측의 유효성을 확인합니다.

현재 기본 장면에서는 초기 발끝이 주황색 접촉 물체와 겹칩니다. 자동 보정에서 이 물체와의 접촉이 감지되면 `ExternalContact` 오류로 출력을 해제합니다. Ctrl 드래그/물체 이동으로 경로 밖에 놓고 Z를 눌러 재시도하십시오. 이 시뮬레이션 정보는 보정을 **거부**하는 데만 쓰며, 영점 산출에는 쓰지 않습니다.

토크와 정지만으로 기계 스토퍼와 임의의 장애물을 구별할 수 없습니다. 방향·알려진 스토퍼 각도·접촉 토크를 실제 구조에 맞게 설정하고 탐색 경로를 확보해야 합니다. 관절의 탄성 스토퍼 침투, 엔코더/CAN 양자화와 전류 노이즈 때문에 보정 오차가 남을 수 있습니다. 물리 XML·질량·마찰·기존 모션 게인은 변경하지 않습니다.

## GUI 조작

기존 Classic GUI와 독립 Filament 창에서 아래 프로젝트 조작을 사용할 수 있습니다. Filament 패널·카메라 추가 키는 [시각화 안내](visualization.md)를 참고하십시오. 시뮬레이션 전용 데이터와 승인 조작은 `simulation_ipc`로 교환하며 하드웨어 SHM에는 넣지 않습니다.

**시작은 DISARMED, 선택된 기본 모션은 Damping**입니다. Arm 요청 전에는 출력하지 않습니다. `require_zero_position_setup: true`인 새 세션은 먼저 Zero set을 수행합니다. 자동 호밍하지 않습니다. 관절 `kp=0`, `kd=1`, 기본 목표 속도·`tau_ff=0`으로 `tau=-dq`를 계산하고 공통 `limits.torque_nm`으로 제한합니다. CAN에는 이를 MIT 명령으로 직접 보내며(kp로 나누는 surrogate 변환 없음), 제한이 걸릴 때만 동일한 제한 토크가 나오도록 목표 속도를 보정합니다. 첫 유효 모델 상태에서 현재 관절각과 발끝 위치를 목표로 한 번 저장합니다. 이후 위치 오차에 따른 복원력이나 중력 보상은 없으므로 중력·외력에 의해 움직일 수 있습니다. 모델이 아직 없으면 제어 주기는 계속 감쇠 출력을 내고, 목표 미확정 상태를 표시한 뒤 첫 모델 샘플에서 목표를 확정합니다. D로 다시 진입할 때도 그때의 위치를 캡처하며 GUI와 `ControlData.damping_toe_reference`·`damping_reference_valid` 및 MCAP에 표시·기록합니다. 호밍하려면 H를 누릅니다.

| 입력 | 동작 |
| --- | --- |
| A / X | 명시적 Arm / Release. fault 해제는 별도 Clear fault 버튼 또는 Input 명령 |
| Z | hip → thigh → calf 스토퍼 탐색·런타임 영점 보정 시작/재시도 |
| D | 기본 Damping: kp=0, kd=1, 중력 보상 없이 관절 속도 감쇠 |
| H | 명시적으로 5차 보간 호밍 후 순수 PD 자세 유지 |
| G | 중력보상. 자세 강성·감쇠는 설정값 사용 |
| M | Magnetic Touch: FREE → 접촉 후보 → 힘 추종 → 해제 |
| Target [N] | 목표 접촉력 변경. 접촉 상태는 유지 |
| Space | 물리 일시정지·재개. 제어 루프는 계속 동작 |
| R | 물리 초기화 후 Damping으로 복귀 |
| Esc | 종료 |

몸체를 왼쪽 더블클릭하여 선택한 뒤 Ctrl+오른쪽 드래그로 외력을 가합니다. 주황색 물체도 같은 방식으로 선택·이동하며, Ctrl+Shift+오른쪽 드래그는 수평면 이동입니다. Magnetic 모드에서 물체를 발끝에 접촉시키고 천천히 이동하면 자동으로 힘 추종에 진입합니다. 접촉 해제·작업영역·추종 거리 등 설정 한계를 넘으면 FREE로 복귀합니다.

D/H/G 화면은 관절각과 피드백 측정 토크를 표시합니다. M/T/B 새 진입 때 그래프와 잔상 이력을 초기화하여 이전 진입 자료가 섞이지 않게 합니다. CAN 모드의 토크 그래프는 반환 Iq와 C4 Kt로 환산한 값을 사용하므로 가상 Iq 센서의 오차·노이즈·CAN 양자화가 반영됩니다. 검증용 CSV의 실제 토크(GT)는 물리 출력으로 별도 기록합니다. M 화면은 추정/실제/목표/명령 힘, 발끝 실제·목표 위치, 추종 오차와 전환 사유를 표시합니다.

## 제어 설정과 공통 입력

TaskController는 각 모션 클래스의 초기화 인자로 YAML **경로**를 넘깁니다. 각 클래스가 자기 항목을 파싱·검증합니다. 설정을 바꾼 뒤 프로그램을 재시작합니다. 관절 배열 순서는 HR(FL hip) / HP(thigh) / KP(calf)입니다. 게인과 목표값은 위 YAML을 기준으로 합니다.

제어 입력은 TaskController의 이름 있는 상태 배열 한 벌로 보관합니다. `ReadInput()`이 같은 SHM 잠금 안에서 각 모델·관절 값을 명시적으로 읽고, 미분용 이전 샘플은 별도로 유지합니다. Magnetic의 `Start()`는 발끝 위치만 받으며 `Compute()`와 RFOB의 `Update()`는 사용하는 배열·측정 번호·주기를 각각 인자로 받습니다. 모션 제어기는 SHM 데이터 구조를 포함하거나 상속하지 않습니다.

모든 모션의 최종 토크 상한은 최상위 `limits.torque_nm` 하나로 설정합니다. 현재 개별 제한 중 최대값인 **60.7 N·m**로 통일했습니다.

```yaml
limits:
  torque_nm: 60.7  # 스칼라 또는 HR/HP/KP 순서의 3개 배열
```

Homing/Gravity/Magnetic/Tracking/Ball과 TaskController 최종 출력, Classic/Filament 그래프가 이 값을 사용합니다. 호밍과 Damping의 MIT 명령에도 제한된 토크가 실제로 표현됩니다. 개별 모션에 옛 `torque_limit_nm`/`output_torque_limit_nm`을 남기면 오류로 안내하며 자동으로 우선순위를 정하지 않습니다. 물리 MJCF 및 CAN 모터의 독립적인 제한은 유지하므로 공통 설정이 그 한계를 높이지 않습니다. ILC의 `correction_limit_nm`(보정 자체 상한)과 `update_limit_nm`(한 주기 갱신량 상한)은 학습 정책이므로 유지하며, 합산 최종 출력에 공통 한도를 적용합니다.

Homing은 보간 위치·속도와 자신의 kp/kd를 MIT 목표로 보냅니다. Gravity와 Magnetic은 `q + (tau + kd_motor*dq)/kp_motor`를 보냅니다. kd 항은 이미 계산된 감쇠를 모터 PD에서 중복 적용하지 않기 위한 항입니다. 각 모드의 내부 서보 게인은 해당 `kp_motor/kd_motor`에서 설정합니다.

두 모드의 동역학 보상항은 각 설정의 `gravity_only`로 따로 선택합니다. 설정 변경 후 데몬을 재시작합니다.

```yaml
gravity_compensation:
  gravity_only: true
magnetic_touch:
  gravity_only: true
```

`true`는 별도 `mjData`에 qpos를 복사하고 qvel을 0으로 둔 뒤 `mj_fwdPosition()`·`mj_fwdVelocity()`로 구한 순수 중력을 사용합니다. `false`는 실제 속도에서 계산한 전체 `qfrc_bias`(중력+코리올리·원심력)를 사용합니다. Gravity의 `alpha`는 선택한 보상항 전체에 적용합니다. 물리 모델의 마찰·감쇠는 바꾸지 않으며, Gravity에서만 아래의 별도 제어 토크로 일부 보상합니다. Magnetic의 마찰·감쇠 처리는 그대로입니다. RFOB는 설정과 관계없이 실제 속도에서의 전체 동역학을 사용합니다. 옵션 누락·잘못된 bool 값은 오류로 처리합니다.

### Gravity의 마찰 보상과 잔여 감쇠

`config/control/control.yaml`에서 아래 값을 바꾸고 **제어기/데몬을 재시작**합니다. 스칼라 또는 HR/HP/KP 순서 배열을 사용할 수 있습니다. 실행 중 자동 재로드나 GUI 슬라이더는 없습니다.

```yaml
gravity_compensation:
  damping_ratio: [1.0, 1.0, 1.0]
  friction_compensation:
    enabled: true  # false: 건마찰·점성감쇠 보상 OFF
    coulomb_ratio: [0.95, 0.95, 0.95]
    velocity_scale_rad_s: 0.03
```

`damping_ratio`는 **모델 점성 감쇠를 남기는 비율**이며 임계감쇠비 ζ가 아닙니다. 0이면 모델 점성 감쇠 전량 보상, 0.5면 절반 보상, 1이면 점성 감쇠 보상 없음입니다. 현재 설정 1.0은 모델 감쇠 2.0을 그대로 남깁니다. 0.5로 설정하면 1.0을 남기며, 별도 `kd_motor=0.01`를 포함한 총 점성 감쇠는 각각 약 2.01 / 1.01 N·m·s/rad입니다. `task_stiffness=0`이므로 진입 위치로 복원하지 않습니다.

`friction_compensation.enabled=false`이면 건마찰과 모델 점성감쇠 보상 항을 모두 끕니다. 설정된 비율은 유지하며, 중력/bias·자세 강성·모터 kd는 그대로 적용합니다. OFF 토크는 `alpha*h + K*(q_entry-q) - kd_motor*dq`입니다. 스위치 누락·잘못된 bool은 시작 오류입니다.

ON일 때 기본 토크는 `alpha*h + K*(q_entry-q) + (1-r)*D*dq + beta*Fc*tanh(dq/v0) - kd_motor*dq`입니다. h는 위의 중력/bias 선택, r은 damping_ratio, beta는 coulomb_ratio입니다. D/Fc는 측정 관절 상태로 갱신하는 MuJoCo 백엔드의 모델 값입니다. 전체 passive 토크를 뒤집지 않으므로 스프링 등 다른 수동항까지 임의로 상쇄하지 않습니다. 현재 건마찰 보상 95%와 tanh 전환은 정지 부근의 갑작스러운 토크 반전을 줄이며, 속도가 정확히 0이면 이 보상도 0입니다. 정지 마찰을 완전히 없애거나 아주 작은 힘만으로 반드시 움직이게 만드는 설정은 아닙니다. 실제 하드웨어에서는 모델 D/Fc의 식별 정확도에 따라 보상량이 달라집니다.

가상 관성 제어는 제거했습니다. 외력 추정 기반의 추가 보조 토크는 계산하지 않습니다. 이전 YAML의 `virtual_inertia` 항목은 삭제해야 하며, 남아 있으면 시작 오류로 안내합니다. 중력/bias 선택, 자세 강성, 마찰·감쇠 보상 및 최종 토크 제한은 유지합니다.

가상 엔코더는 시작 자세에서 상대각 0으로 시작합니다. `MotorData.position_offset[i]`은 시뮬레이션 시작 자세로 초기화하고 제어기가 이를 사용합니다. **실기 영점 보정은 아니며 스토퍼 호밍은 미구현**입니다.

## 무한대 궤적 Tracking 데모

기본 실행 후 **Homing [H]로 HOLD까지 기다린 다음** 아래 단축키로 Tracking을 선택합니다. `H`·`G`·`M`으로 기존 모드에 돌아갑니다.

| 단축키 | 평면 | 고정하는 목표 좌표 |
| --- | --- | --- |
| `1` | XY | 진입 시 Z |
| `2` | XZ | 진입 시 Y |
| `3` | YZ | 진입 시 X |
| `T` 또는 Tracking 버튼 | 마지막으로 선택한 평면 (최초 XY) | 해당 평면 밖 좌표 |

평면을 바꾸면 현재 발끝 위치를 중심으로 궤적과 속도 램프를 다시 시작하며, 이전 그래프·잔상을 지웁니다. 같은 평면 키를 반복해서 눌러도 궤적을 재시작하지 않습니다. 숫자 편집란 입력 중에는 평면 단축키를 처리하지 않습니다. 선택 평면은 GUI 상단 상태와 경로 그래프 제목에 표시됩니다.

[tracking 설정](../config/control/control.yaml)의 `trajectory.period_sec`로 진입 이후 주기, `trajectory.amplitude_m`의 월드 X/Y/Z 배열로 각 축의 반폭을 조절합니다. 전체 폭은 진폭의 2배입니다. 선택 평면의 두 축을 `(u, v)`로 두고 `A[u]*sin(φ), A[v]*sin(2φ)`의 무한대 모양을 유지합니다. `trajectory.timing`은 시간 배분 방식을 명시적으로 선택합니다.

- `uniform_acceleration` (현재 기본): 진입 이후 **발끝 목표 가속도 크기의 편차와 피크를 줄이는 가변 속력** 프로파일입니다. 경로 곡률을 반영하여 접선 가속도와 법선 가속도의 합성 크기를 제한하는 순방향·역방향 속력 포락선을 초기화 시 계산합니다. `a² = (du/ds / 2)² + (κu)²`, `u = v²`를 사용하며 단위 가속도에서 구한 모양을 설정 주기에 맞춰 정규화합니다. 2048개 기하 샘플과 로그 위상 속력의 24개 코사인 조화항(가우시안 평활 폭 0.02rad)으로 가감속 전환을 매끄럽게 합니다. 속력은 양수이고 주기 경계에서도 위치·속도·가속도가 이어집니다. 교차점 부근의 짧은 가속도 감소는 남으며 완전한 등가속도나 지정한 절대 가속도 상한을 보장하는 방식은 아닙니다.
- `constant_speed`: 기존 **Arc-length Reparameterization**입니다. 진입 이후 목표 속력은 `전체 경로 길이 / period_sec`이며, 곡률에 따라 가속도가 변합니다.

두 방식 모두 설정 주기와 경로 모양을 유지합니다. `trajectory.arc_length_samples`는 Simpson 적분 표의 구간 수이며 기본 4096, 정속은 64..65536, 가속도 프로파일은 256..65536입니다. 호 길이 또는 위상-시간 적분 표를 한 번 만들고, 제어 루프에서는 이진 검색과 구간 내 Newton 보정만 수행합니다. 프로파일 생성 오류·잘못된 `timing`·필수 설정 누락은 오류로 보고하며 다른 방식으로 자동 대체하지 않습니다.

`trajectory.ramp_sec` 동안에는 5차 함수로 **프로파일 시계의 진행 속도**를 0부터 높입니다. 진폭은 축소하지 않으며 정지 상태의 중심 교차점에서 출발합니다. 램프 이후 프로파일 시간은 `elapsed - ramp_sec/2`이므로 ILC의 시간 위상과 완주 경계는 유지됩니다. 목표 속도·가속도는 시간 변환과 램프의 미분을 포함합니다. 실제 발끝의 가속도는 추종 오차·IK 근사·토크 제한의 영향을 받습니다.

```text
(u, v) = XY: (X,Y), XZ: (X,Z), YZ: (Y,Z)
profile_clock = integral(quintic_speed_ramp(t), dt)
phi = inverse_time_profile(profile_clock mod period_sec)  # uniform_acceleration
# constant_speed: inverse_arc_length(path_length * (profile_clock mod period_sec) / period_sec)
x_ref[u] = center[u] + A[u]*sin(phi)
x_ref[v] = center[v] + A[v]*sin(2*phi)
x_ref[remaining axis] = center[remaining axis]
v_toe = J*dq
F = K*(x_ref - x_toe) + D*(v_ref - v_toe)
q_d = IK(x_ref)
dq_d = J(q_d)# * v_ref
ddq_d = J(q_d)# * (a_ref - Jdot(q_d,dq_d)*dq_d)
tau_id = M(q_d)*ddq_d + bias(q_d,dq_d)
tau = gravity + alpha*(tau_id - gravity) + J^T*F - kd_motor*dq
q_MIT = q + (tau + kd_motor*dq)/kp_motor - encoder_origin
```

`J#`는 조건에 따라 실제 역행렬 또는 Adaptive DLS 연산을 뜻합니다. 이 로봇의 발끝 위치 자코비안은 3×3 정방행렬이므로 `σ_min >= singularity_threshold_m`이고 조건수가 `workspace.maximum_condition_number` 이하이면 스케일을 정규화한 실제 `J^-1`을 사용합니다. 그 밖에는 `J^T*(J*J^T + λ²I)^-1`을 Cholesky 분해로 풉니다. 역행렬 계산 실패를 다른 방식으로 숨기지 않으며, 잘못된 수치는 실패로 반환해 기존 HOLD/오류 처리를 거칩니다. 전체 궤적 조건수 제한은 그대로 적용되므로 DLS가 안전 검사를 우회하지 않습니다.

```text
threshold = max(singularity_threshold_m, sigma_max / maximum_condition_number)
z = min(1, sigma_min / threshold)
lambda^2 = lambda_min^2 + (lambda_max^2 - lambda_min^2) * (1-z^2)^2
```

`src/motion_controllers/numerics`는 모션 컨트롤러 내부 구현입니다. MuJoCo 백엔드는 이 모듈을 참조하지 않고 `std::array`와 MuJoCo 자체 행렬 연산을 사용합니다. 이 하위 모듈에 3×3 행렬·특이값·역행렬·DLS·벡터·삼각형 거리·조화급수 계산을 모았습니다. 같은 자코비안의 분해는 재사용하여 목표 속도와 가속도를 구합니다. 조건이 좋은 영역에서는 `lambda=0`, DLS 영역에서는 설정된 감쇠를 사용하므로 경계에는 `lambda_min`에 따른 작은 차이가 있습니다. RFOB도 같은 역행렬/전치 DLS 풀이를 쓰되 기존 최소 특이값 무효 판정과 고정 정규화 계수는 유지합니다. 특이점 부근의 증폭을 줄이는 대신 DLS 영역의 작업공간 재현 오차는 커질 수 있습니다.

`impedance.stiffness_n_m/damping_ns_m`은 월드 X/Y/Z 피드백 게인, `motor.kp_nm_rad/kd_nm_s_rad`는 HR/HP/KP MIT 내부 서보 게인입니다. `limits.torque_nm`은 기본 제어와 ILC를 합산한 최종 명령의 관절별 한계입니다. 전체 토크를 제한한 뒤 기존 MIT 위치 명령 경로로 보내며 `tau_ff=0`을 유지합니다. 설정 누락·잘못된 값은 시작 오류로 보고합니다. 설정 변경 후 재시작합니다.

제어 설정 최상위의 `model_path` 하나를 Tracking 역동역학·공 추종 FK·GUI 작업공간 계산에 공통으로 사용합니다. 상대 경로는 이 YAML 파일 위치를 기준으로 해석합니다. TaskController가 백엔드를 한 번 생성하여 Tracking과 Ball에 주입하며, 하위 항목의 예전 모델 경로로 대체하지 않습니다. 독립 뷰어의 작업공간 작업 스레드는 별도 백엔드를 소유합니다. `tracking.inverse_dynamics`에는 해당 모드 전용 보간 비율 `alpha`만 둡니다.

설정의 소속은 다음과 같습니다. 전체 필수 항목과 현재 튜닝값은 [control.yaml](../config/control/control.yaml)에 있습니다.

| 경로 (`tracking.` 아래) | 역할 |
| --- | --- |
| `mode` | `inverse_dynamics` / `ctc` / `impedance` 중 기본 제어 방식 선택 |
| `trajectory` | 시간 배분 방식·주기·진입 램프·XYZ 진폭·적분 표 |
| `ik` | 전체 경로 사전 검사 및 IDFF/CTC/ILC 관절 목표 생성과 Adaptive DLS |
| `motor` | MIT 내부 서보의 위치/속도 게인 |
| `impedance` | IDFF 또는 순수 임피던스의 작업공간 피드백 |
| `inverse_dynamics` | 목표 상태 역동역학과 현재 중력 사이의 `alpha` 보간 |
| `ctc` | 가속도 피드백 `kp_per_s2`·`kd_per_s` |
| `ilc` | 반복 학습·위상 필터·보상 및 갱신 한계 |

`mode: impedance`는 중력+작업공간 임피던스를 사용합니다. ILC를 끄면 실행 중 관절 목표 IK/RNEA 없이 중력+작업공간 임피던스를 사용합니다. 실행 전 전체 경로 검사용 IK 모델은 모든 방식에서 필요합니다. CTC는 IDFF·임피던스 설정을 요구하지 않고, ILC를 끈 임피던스도 사전 검사용 `ik` 설정은 필요하며 CTC·IDFF 설정은 요구하지 않습니다. 임피던스에서도 ILC를 켤 수 있으며, 이때 `ik`와 최상위 `model_path`로 학습용 목표 관절각·속도를 계산합니다. IDFF/CTC 토크는 계산하거나 더하지 않습니다. 모델 경로는 계속 제어 YAML 최상위의 `model_path` 하나입니다.

`ik.damping_min_m`은 기본 감쇠 `λ_min`, `ik.damping_max_m`은 상한 `λ_max`, `ik.singularity_threshold_m`은 감쇠를 높이기 시작하는 최소 특이값입니다. 두 감쇠를 같게 설정하면 DLS 영역에서 고정 감쇠를 사용합니다. 정상 조건의 실제 역행렬 분기는 유지합니다. 누락되거나 잘못된 설정은 시작 오류로 보고하며 이전 고정 감쇠로 자동 대체하지 않습니다.

`tracking.mode: ctc`를 선택하면 **관절 공간 Computed Torque Control**로 전환합니다. 같은 시간 배분 궤적과 Adaptive DLS로 `q_d/dq_d/ddq_d`를 만들고, 관절 오차로 가속도를 보정한 뒤 **현재 측정 상태**에서 동역학을 계산합니다. 이 방식은 [Modern Robotics의 computed-torque 제어](https://modernrobotics.northwestern.edu/nu-gm-book-resource/11-4-motion-control-with-torque-or-force-inputs-part-3-of-3/)에서 적분항을 제외한 형태입니다.

```text
ddq_cmd = ddq_d + Kp_ctc*(q_d-q) + Kd_ctc*(dq_d-dq)
tau_ctc = M(q)*ddq_cmd + bias(q,dq)
tau = clip(tau_ctc, -limits.torque_nm, limits.torque_nm)
q_MIT = q + (tau + kd_motor*dq)/kp_motor - encoder_origin
```

`ctc.kp_per_s2/kd_per_s`는 가속도 단위의 피드백 게인이며 MIT 내부 서보의 `motor.kp_nm_rad/kd_nm_s_rad`와 다릅니다. CTC에서는 작업공간 `impedance`, IDFF의 `alpha`, 별도 중력·모터 D 토크를 합산하지 않습니다. 내부 D는 기존 MIT surrogate 변환에서 보정하므로 출력은 위의 CTC 토크가 됩니다. `ik` 항목의 Adaptive DLS/IK 파라미터는 IDFF와 CTC가 함께 사용합니다. `mode`로 방식을 선택하며, 알 수 없는 모드·잘못된 CTC 게인은 시작 오류입니다. CTC가 실패해도 IDFF로 자동 전환하지 않고 기존 IK 실패 자세 유지 상태로 들어갑니다.

CTC도 같은 계산용 모델·`mjData`를 재사용하고 `mj_rne()` 결과에 armature 관성을 더합니다. 물리 감쇠·건마찰·스프링·접촉력은 보상 모델에 포함하지 않습니다. 초기 CTC 게인은 시험용이며 실제 추종 오차와 토크 여유를 보며 조정합니다.

세 Tracking 방식 모두에 **토크 ILC(Iterative Learning Control)**를 독립적으로 더할 수 있습니다. `tracking.ilc.enabled: true`로 켜고 `false`로 끕니다. 설정 변경 후 재시작합니다. [TrackingILC](../src/motion_controllers/tracking_ilc.cpp)는 TrackingController가 소유하고 자체 YAML 경로로 설정을 읽습니다. `mode: impedance`에서도 ILC를 지원합니다. 이 조합은 학습 오차용 관절 목표만 IK로 계산하며 기본 출력은 중력+작업공간 임피던스입니다. ILC를 자동으로 끄거나 IDFF로 전환하지 않습니다. IK 실패 시에는 다른 방식과 동일하게 오류를 표시하고 측정 자세 유지 상태로 들어갑니다.

```yaml
tracking:
  mode: ctc  # inverse_dynamics/impedance에서도 ILC 사용 가능
  ilc:
    enabled: true
```

ILC를 켰을 때 설정 검증상 궤적 주기 하한은 `max(phase_bins / control.frequency_hz, 4 × blend_sec)`입니다. 이는 물리적인 추종 가능 주기를 보장하지 않습니다.

정상 궤적의 위상을 `phase_bins`개로 나누어 주기별 관절 위치·속도 오차를 모으고, **완주한 이전 주기**로만 다음 주기의 보상 표를 갱신합니다. [PD-type ILC와 Q 필터](https://www.diag.uniroma1.it/deluca/rob2_en/A_survey_of_iterative_learning_control_CSMag_2006.pdf)의 형태를 사용하며, 여기서 오차는 목표−측정입니다.

```text
tau_ilc[k+1] = Q(retention*tau_ilc[k] + Kp_ilc*e_q[k] + Kd_ilc*e_dq[k])
tau_cmd = clip(tau_base + tau_ilc, -limits.torque_nm, limits.torque_nm)
```

`ilc.position_gain_nm_rad/velocity_gain_nm_s_rad`는 HR/HP/KP 순서의 유효 학습 토크 게인이고 `ilc.retention`은 기존 학습값 유지율입니다. `phase_lead_bins`만큼 뒤쪽 위상의 오차를 앞쪽 보상에 반영하고, `smoothing_passes`번 순환 대칭 3점 필터(`1/4,1/2,1/4`)를 적용합니다. `ilc.update_limit_nm`은 표 한 지점의 주기별 변화, `ilc.correction_limit_nm`은 ILC 보상 자체의 토크 한계입니다. 최종 `limits.torque_nm`과는 별개입니다. 주기 경계에서는 `blend_sec` 동안 5차 보간으로 이전·새 보상을 연결하고 위상 표도 순환 선형 보간하므로 보상값이 경계에서 뛰지 않습니다.

속도 램프를 마친 뒤 첫 위상 교차점과 `warmup_cycles`를 기다립니다. 구간 하나라도 샘플이 없거나 제어기 최종 토크가 포화된 주기는 학습 갱신을 건너뛰며 GUI·터미널에 이유를 표시합니다. 이 경우 기존 보상 표는 유지합니다. 포화 판정은 **제어기 토크 제한** 기준이며 가상 모터·실기 내부 포화를 모두 알아내는 것은 아닙니다. 제어기는 기존처럼 최신 관절 상태로 독립 실행하며 응답 지연 감시나 시뮬레이터 일시정지 연동은 추가하지 않았습니다.

모드 재진입·평면 전환·리셋 시 학습 표를 지웁니다. IK 실패 시 ILC 출력도 0으로 하고 기존 자세 유지 상태로 들어갑니다. 학습값은 파일에 저장하지 않습니다. GUI에는 ILC 상태·갱신/건너뜀 횟수·관절별 보상 Nm·마지막 완주 주기 발끝 RMS를 표시하고 SHM/MCAP의 `tracking_ilc_*` 필드에도 기록합니다. 기존 IDFF/CTC 토크 그래프는 **기본 제어 토크**이며, 별도로 표시하는 최종 명령 여유에는 ILC가 포함됩니다. 고정 시험 설정에서의 추종 개선은 시험 조건의 결과이며 모든 궤적·게인·외란에서의 수렴을 보장하지 않습니다.

`src/robot_model/ModelBackend`는 MuJoCo 헤더와 분리된 기구학·동역학 인터페이스입니다. `control.cpp::ReadInput`이 CAN 또는 명시적 디버그 피드백의 `q/dq`로 백엔드를 갱신하고 결과를 `ModelData`에 발행합니다. 시뮬레이터는 런타임 모델 작성자가 아니며 제어기는 외부 `ModelData`와 시뮬레이션 GT를 계산 입력으로 쓰지 않습니다. 백엔드는 스레드 하나에서 사용하고 측정 상태, 목표 계산용 scratch, 중력 계산용 `mjData`를 분리합니다. 목표 IK를 계산해도 측정 상태는 변하지 않습니다.

IK는 이전 목표에서 시작하는 기존 반복 알고리즘을 유지하며, MuJoCo의 FK와 `mj_jacSite`를 이용합니다. `mj_inverse`는 역동역학 API이므로 위치 IK의 대체 함수로 사용하지 않습니다. 전체 `mj_forward` 대신 `mj_kinematics`/`mj_comPos`, 속도 단계, `mj_crb`/`mj_rne` 등 필요한 계산만 실행합니다. `Jdot*dq`는 `mj_jacDot`으로 구합니다. 호출의 선행 단계는 [MuJoCo 3.3.2 API](https://mujoco.readthedocs.io/en/3.3.2/APIreference/APIfunctions.html)를 따릅니다. 수동 감쇠·스프링·마찰·접촉력은 Feedforward에 포함하지 않으며 armature는 질량행렬 또는 RNE 결과에 한 번만 포함합니다.

측정 자세가 같으면 질량·중력·자코비안을 재사용합니다. 측정 `q/dq`가 모두 같으면 전체 모델 갱신도 재사용합니다. 고해상도 렌더 메시 제거는 명시적 inertial이 있고 `inertiafromgeom=true`가 아닌 몸체의 비접촉 시각 지오메트리에만 적용합니다. 자동 관성 추론에 필요한 메시는 남겨 원본 질량·관성을 보존합니다. 모델 경로·관절 순서·차원·유한값 검증에 실패하면 명시적으로 오류를 내며 다른 모델로 대체하지 않습니다. 모델·게인·물리 시뮬레이션 계산은 변경하지 않았습니다.

`inverse_dynamics` 모드에서 `alpha: 1`은 역동역학 전체 토크, `alpha: 0`은 기존 현재 자세 중력 보상이며 중간값은 둘 사이를 보간합니다. `mode: impedance`는 기존 중력+작업공간 임피던스를 사용합니다. ILC를 끄면 실행 중 관절 목표 IK/RNEA를 계산하지 않고, 켜면 IK로 학습용 관절 목표만 생성합니다. 사전 검사와 실제 자세 조건수 검사는 두 경우 모두 수행합니다. 기존 임피던스가 위치 피드백을 담당하므로 별도의 관절 위치 강성이나 RL residual은 추가하지 않았습니다. 계산 토크는 기존처럼 MIT **위치 surrogate**에 반영하며 패킷의 **tau_ff는 0**입니다. 목표 궤적은 관절 범위에 맞춰 자동 축소하거나 투영하지 않습니다. 기하학적으로 해가 있어도 목표 관절각이 물리 범위를 넘으면 GUI에 `q_d=OUTSIDE LIMITS`를 표시합니다. 실제 모델의 관절 범위와 기존 토크 한계는 유지합니다.

**두 모드에서 실행 중 IK 미수렴·수치 계산 실패 시** 궤적과 IDFF/CTC 동역학 출력을 중단하고 **실패 순간의 측정 관절각**을 고정합니다. 모터를 끄지 않고 `중력 + kp_motor × 관절 위치 오차 − kd_motor × 관절 속도`로 자세를 유지하며, 기존 토크 제한과 MIT 위치 명령 경로를 사용합니다. GUI에는 `IK / CONDITION BLOCKED - HOLD`, 제어 터미널에는 실패 원인·시각·목표를 표시합니다. SHM/MCAP의 `tracking_ik_failed`와 로거의 `TRACKING_IK_HOLD` 이벤트에도 기록합니다. 도달 가능한 목표로 돌아와도 자동 재개하지 않으며, **다른 모드로 전환한 뒤 T로 재진입하거나 다른 궤적 평면을 선택**하면 재시도합니다. 현재 평면 키를 다시 누르는 것만으로는 재시작하지 않습니다. 모델·설정 오류나 유효하지 않은 측정 상태는 계속 명시적 오류로 처리합니다.

3D 화면에는 최근 2초의 실제 발끝 경로를 가는 반투명 하늘색 잔상으로 표시합니다. 오래된 부분부터 옅어지고 리셋 시 지워집니다. 일시정지 중에는 잔상도 유지합니다.

GUI 오른쪽에는 선택 평면의 목표/실제 경로, XYZ 위치 비교, 위치 오차 그래프를 표시하고 상단에는 오차·토크 포화·선택한 IDFF/CTC의 ON/OFF·IK 오차를 표시합니다. `ControlData.tracking_position_reference`·`tracking_velocity_reference`·`tracking_torque_saturated` 및 요청/실제 평면(`requested_tracking_plane`·`tracking_plane`, XY=0/XZ=1/YZ=2)은 MCAP에도 기록됩니다. 목표 발끝 가속도·목표 관절 속도/가속도·역동역학 토크·활성화 여부·IK 오차도 기록하며, 목표 관절각은 기존 `MotorData.reference`를 사용합니다. 선택 방식(`tracking_method`: inverse_dynamics=0, CTC=1, impedance=2)과 포화 전 CTC 토크(`tracking_ctc_torque`)도 기록합니다. SHM ABI는 **16**이므로 모든 프로세스를 함께 재빌드·재시작해야 합니다.

Tracking 화면에는 **IDFF/CTC 토크 여유**도 표시합니다. HR/HP/KP별 `limit / now / |peak| / left Nm (%)`는 유효 제한·현재 선택한 방식의 raw 토크·최근 절댓값 피크·피크 기준 잔여 토크와 비율입니다. `left = limit - |peak|`이며 초과하면 음수와 `OVER`를 표시합니다. 유효 제한은 Tracking 설정과 MuJoCo actuator 양방향 제한 중 작은 값이고, CAN 실행에서는 가상 모터의 토크 제한도 포함합니다. 현재 기본 CAN 설정에서는 **53Nm**입니다. 비대칭 제한은 양방향 중 작은 절댓값으로 보수적으로 평가합니다.

통계는 기존 `plots.sample_hz`(기본 **100Hz**)로 관측한 최신 SHM 값을 `plots.history_sec`(기본 **10초**, 시뮬레이션 시간) 동안 보관한 결과입니다. 제어기의 모든 주기 피크를 수집한 것은 아닙니다. 모드 재진입·평면 변경·리셋 시 이전 실행의 피크를 제외하고, IDFF/CTC OFF/HOLD에서는 현재값을 `N/A`로 표시하면서 같은 실행의 시간창 안에 남은 과거 피크는 유지합니다. 네 번째 그래프는 선택한 IDFF 또는 CTC 토크의 부호 있는 제한 대비 사용률과 ±100% 기준선이며 제목·표에도 방식을 구분해 표시합니다.

Raw IDFF는 **alpha 적용·임피던스 합산·포화 전** 값이며, raw CTC는 **관절 피드백을 포함한 CTC 전체 토크의 포화 전** 값입니다. `Ctrl cmd left now`는 최종 제어 명령(`MotorData.debug_torque`, 제어기 토크 제한 적용 후)에 대한 현재 여유를 HR/HP/KP 순서로 별도 표시합니다. 두 값 모두 설정된 출력 토크 예산과의 비교이며, 전류 센서 측정 토크나 실제 구동기의 연속 정격·열 여유를 의미하지 않습니다. 계측 자체는 제어 토크를 변경하지 않습니다.

Tracking도 기존 독립 monotonic 제어 주기를 사용합니다. 시뮬레이터 일시정지 중에도 궤적 시간은 진행합니다. 기본 호밍 위치에서 시험합니다. 사전 검사는 유한 표본의 IK 수렴과 조건수를 검사하며, 실행 중에도 위반이 생기면 자세 유지 상태로 전환합니다. 속도와 외력·토크 제한 때문에 물리적으로 즉시 정지하거나 위치 오차가 완전히 없어지는 것은 보장하지 않습니다.

Tracking 진입 시 `tracking.ik.trajectory_samples`(기본 **512구간**)로 한 바퀴 전체를 IK로 검사합니다. 각 목표 자세의 위치 야코비안 조건수 `σ_max/σ_min`이 공통 `workspace.maximum_condition_number`(기본 **10**)를 넘거나 IK가 수렴하지 않으면, 유효한 앞부분만 움직이지 않고 궤적 전체를 거부합니다. 관절각을 물리 범위 안으로 자동 투영하는 검사는 아닙니다.

검사는 매 제어 틱에 최대 **4점**씩 수행합니다. 검사 중에는 중력 보상+관절 PD로 진입 자세를 유지하고 IDFF/CTC/ILC를 출력하지 않습니다. GUI에는 `CHECKING TRAJECTORY - HOLD`와 진행률을 표시합니다. 기본 500Hz에서는 513점을 약 129틱, **0.258초**에 검사하며, 전체 통과 후 궤적 시계와 진입 램프를 시작합니다. 제어 루프는 검사 중에도 SHM·CAN 출력을 계속 갱신합니다.

거부 시에는 `IK / CONDITION BLOCKED - HOLD`를 표시하고 터미널에 평면·실패 위상·조건수/상한·IK 오차를 남깁니다. 실행 중 실제 자세의 조건수도 매 틱 검사하고, 관절 목표 IK를 사용하는 방식은 목표 자세도 검사합니다. 실패하면 현재 측정 자세를 유지하고 ILC 출력을 중단하며 자동 재개하지 않습니다. H/D 등 다른 모드로 전환 후 T/1/2/3으로 다시 진입하거나 평면을 바꾸면 새로 검사합니다. 유한 표본에 의한 근사 검사이므로 샘플 사이의 모든 연속 자세를 수학적으로 인증하는 것은 아니며, 물리적인 즉시 정지를 보장하지 않습니다. 검사 상태·진행률은 `ControlData.tracking_checking/tracking_check_progress`로 SHM/MCAP에 기록합니다.

## 작업공간 오버레이

GUI에서 **W 키**로 작업공간 표시를 켜고 끕니다. 기본은 꺼짐이며 모든 제어 모드에서 사용할 수 있습니다. 반투명 초록색 폴리곤은 공 추종의 **위치 야코비안 조건수·2cm 여유·샘플링 추가 여유를 적용한 FK 유효 영역**입니다. 관절 격자를 FK로 샘플링하고 `κ(J)=σ_max/σ_min ≤ workspace.maximum_condition_number`를 만족하는 셀의 경계를 거칠게 표시합니다. 기본 상한은 **10**이며, 낮추면 조건이 좋은 자세만 남고 높이면 민감한 자세까지 포함합니다. 기존 `minimum_singular_value_m` 대신 무차원 조건수를 사용합니다. 격자 근사이므로 계단 모양이 보일 수 있으며, 경계가 정확한 IK 수렴 한계나 토크·속도 한계를 뜻하지는 않습니다.

최초 표시 시 `BUILDING...`을 보여 주며 별도 스레드에서 한 번 계산합니다. 이후 W 전환은 캐시를 재사용합니다. 상태는 기존 상단 정보 아래에 표시하며, 생성 도중 종료해도 계산을 취소하고 정리합니다. 격자가 너무 커서 장면 용량을 넘거나 생성에 실패하면 GUI와 터미널에 명시적으로 오류를 표시합니다. 설정을 바꾸면 데몬을 재시작합니다.

표시는 장식용 도형이므로 물리 충돌·조이스틱 입력·제어 모드·SHM 구조를 바꾸지 않습니다. Tracking은 같은 조건수 상한으로 전체 경로를 검사합니다. 궤적을 자동 축소·투영하지 않으며, 검사 실패 시 진입 자세 유지 상태로 거부합니다. 공통 조건수 상한은 [control.yaml](../config/control/control.yaml)의 `workspace.maximum_condition_number`입니다. 격자·여유 설정은 `ball_following`을 사용하며 활성화되어 있어야 합니다. 이전 ball_following 하위 조건수 항목은 제거했습니다.

## 조이스틱 공 추종

데몬 재시작 후 **Homing [H]의 HOLD 확인 → Ball follow 버튼 또는 B**를 누릅니다. 작은 반투명 하늘색 공이 발끝 위치에 생성됩니다. 다른 모드에서 B로 돌아오면 공은 그때의 발끝 위치로 다시 초기화됩니다.

| 조작 | 공 움직임 |
| --- | --- |
| 왼쪽 스틱 상하 / 좌우 | 월드 X / Y |
| 오른쪽 스틱 상하 | 월드 Z |
| 트리거 없음 | 보통 속도, 기본 0.10m/s |
| R2/RT | 빠르게, 기본 0.30m/s |
| L2/LT | 느리게, 기본 0.03m/s; 양쪽을 누르면 느린 속도 우선 |

[입력 설정](../config/input/input.yaml)에서 장치(`/dev/input/js0`)·축 번호·반전·중앙 데드밴드·트리거 원시 범위를 조정합니다. 기본 축 번호는 Linux Xbox 계열을 가정하므로 다른 장치는 확인 후 수정합니다. 입력 프로세스는 기본 daemon이 별도 터미널에서 실행합니다. 장치가 없거나 읽기에 실패하면 명시적으로 오류를 기록하고 GUI에 **NO JOYSTICK**을 표시합니다. 다른 입력으로 자동 대체하지 않습니다. 장치를 연결하거나 설정을 변경한 뒤 데몬을 재시작합니다. 실제 조이스틱이 없어도 GUI·다른 모드는 사용할 수 있습니다. 입력 프로세스 종료 시 축·트리거 상태를 지우며 응답 지연 감시 기능은 추가하지 않았습니다.

[제어 설정](../config/control/control.yaml)의 `ball_following`에서 공의 세 속도, 발끝 속도, 관절 PD·토크 한계와 작업공간 격자를 조절합니다. 발끝의 **참조 경로 속도는 공 속도와 독립적으로 기본 0.08m/s**입니다. 도착하면 정지합니다. 실제 발끝 속도·오차는 PD와 물리 모델의 영향을 받습니다.

목표 관절각은 **FK 격자 검색과 연결 경로**로 생성하며 IK나 역야코비안으로 풀지 않습니다. 모델의 전체 관절 범위를 FK로 샘플링하고, 호밍과 같은 비특이 분기의 연결 작업공간을 구성합니다. 경계 메시에서 기본 **2cm**와 샘플링 추가 여유 **2mm**를 제외하고, 조건수 상한을 넘는 자세·중간 경로도 같은 기준으로 제외합니다. 최근접점은 이 **샘플링된 작업공간에서의 근사 최근접점**이며 연속 공간의 엄밀한 최적해는 아닙니다. 격자를 늘리면 근사 오차가 줄지만 초기화 시간이 늘어납니다.

공은 범위 밖으로 자유롭게 움직이고 발끝만 가까운 유효 FK 지점까지 뻗은 채 유지합니다. 공이 불연속적으로 바뀌어도 발끝 참조는 기존 짧은 연결 구간을 마친 후 새 경로로 이어져 **위치가 점프하지 않습니다**. 관절 보간의 FK 호장 기준으로 정속 이동하며, 경로 모서리에서 속도 방향까지 연속인 C1 궤적을 보장하는 방식은 아닙니다. 진입 자세가 2cm 여유 밖이면 현재 위치에서 유효 영역으로 연속 이동하고 `ENTERING SAFE WORKSPACE`로 표시합니다. 반대 운동학 분기·특이 자세에서는 H로 돌아가도록 명시적 오류를 보고합니다.

기존 MIT 위치 명령·`tau_ff=0` 경로를 사용합니다. 공 위치·발끝 참조·최근접점·작업공간 제한·입력 연결/트리거 상태는 SHM/MCAP에 기록합니다. 선택적 외부 목표 점프는 `requested_ball_position`을 쓰고 `ball_target_sequence`를 증가시키며, 모드 재진입 시 이전 요청은 다시 적용하지 않습니다. SHM ABI는 **16**이므로 모든 프로그램을 함께 재빌드·재시작합니다.

## 가상 Iq 센서

[simulation.yaml](../config/simulation/simulation.yaml)의 `iq_sensor.enabled: true`로 켭니다. 현재 기본값은 `true`이며 시작 로그에 ENABLED/DISABLED를 표시합니다. 설정을 바꾼 뒤 시뮬레이션 세션을 재시작합니다. 시뮬레이션 프로파일은 시뮬레이터에 `--simulation-config` 경로를 전달합니다. 가상 CAN 모드에서 이 경로는 필수이며, 직접 토크 디버깅 `--without-can`에는 적용하지 않습니다. 물리 CAN 기본 세션은 이 가상 센서 설정을 읽지 않습니다.

```yaml
iq_sensor:
  enabled: true
  gain: 1.0
  offset_a: 0.0
  noise_std_a: 0.02
  ripple_amplitude_a: 0.01
  bandwidth_hz: 200.0
  quantization_a: 0.005
  current_limit_a: 40.0
```

위 항목을 기존 파일에서 조정합니다. 이득·오프셋·가우시안 노이즈·전기각 리플·1차 저역통과 필터·센서 양자화·측정 범위를 설정할 수 있습니다. `enabled` 외 센서 값은 스칼라 또는 HR/HP/KP 세 값 배열을 받습니다. `random_seed`는 같은 입력 샘플 순서에서 노이즈를 재현합니다. 대역폭·양자화가 0이면 해당 단계만 생략합니다. CAN 자체의 전류 양자화는 항상 적용됩니다.

물리 스텝마다 MuJoCo 실제 actuator torque에서 `iq_true = tau_actual / (gear_actual * Kt_motor_actual)`를 구합니다. 전기각은 `pole_pairs * gear_actual * q_joint + electrical_zero_rad`입니다. 센서 출력은 CAN Iq 필드로 반환하고 HAL이 `tau_measured = Kt_output_C4 * iq_reported_CAN`을 계산합니다. 센서값을 물리 토크에 직접 주입하지 않습니다. 제어기가 이 피드백에 반응하면서 이후 궤적·토크는 달라질 수 있습니다.

`motor` 항목은 시뮬레이션 실제 특성이며 모터 규약 파일의 C4 보고값과 독립적입니다. 기본은 Kt_motor=0.16 Nm/A·감속비=9로 C4의 1.44 Nm/A와 맞췄습니다. Kt 오차도 별도로 시험할 수 있습니다. `pole_pairs: 1`은 시뮬레이션 위상 설정이며 실기 식별값이 아닙니다. 센서를 끄면 기존 `actuator torque / C4 Kt`의 이상적 CAN 피드백으로 복귀합니다.

센서는 CAN 요청 횟수와 독립적으로 물리 시각마다 한 번 샘플링합니다. 동일 시각의 반복 요청·일시정지는 값을 유지하고, 시뮬레이션 시간 리셋은 필터와 난수 상태도 초기화합니다. CAN HAL 터미널·`shm_logger --can`에서 보고 Iq를 확인할 수 있습니다.

## 수치 백엔드 검증

`ctest --test-dir build -R "^(motion_numerics|robot_model)$" --output-on-failure`로 컨트롤러 수치 연산과 독립 모델 백엔드를 각각 확인합니다. `motion_numerics`는 역행렬/DLS·특이값을, `robot_model`은 200개 자세의 MuJoCo 대조·관성 보존·상태 격리를 검증합니다. `build-filament/native/bin/visualization_model_test config/control/control.yaml models/qhrr-05/qhrr-05.xml`은 같은 검증을 MuJoCo 3.16.0에서 실행합니다. 테스트가 출력하는 microbenchmark는 해당 빌드/호스트의 참고값이며 실제 제어 주기 보장을 뜻하지 않습니다. Release와 Debug 값을 혼합 비교하지 않습니다.

현재 CAD와 기존 튜닝에서 원본 추종 회귀의 XZ IDFF+ILC 10% 개선 기준은 미달합니다(11.6697 → 11.2324 mm). 변경 전 추종 구현에서도 동일하게 재현되며, 이번 리팩토링에서 게인이나 합격 기준을 변경하지 않았습니다.
