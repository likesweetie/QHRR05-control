# Magnetic Touch Mode 구현 계획

상태: **초기 계획 이력** · 2026-10-24

> 사용자 수정 지시에 따라 **condim=6과 torque actuator를 유지**한 초기 기능을 구현했습니다. 아래의 position actuator 전환·condim 축소 제안은 채택하지 않았습니다. 현재 구현·실행·튜닝·검증 결과는 [핸드오버](magnetic_touch_handover.md)를 기준으로 확인하십시오. 힘 차이는 튜닝 항목으로 남기고 기능 구현을 우선한다는 최신 지시를 반영했습니다.

## 1. 구현 방향과 유지할 구조

고정형 qhrr-05와 기존 FL_foot site를 사용합니다. 관절 순서 FL hip/thigh/calf를 HR/HP/KP로 표시합니다. 새 MagneticTouchController가 FREE부터 접촉 검출·힘 추종·해제까지 소유하고, TaskController는 해당 핸들과 IO·주기 호출을 소유합니다.

- 제어기는 MuJoCo API/실행·일시정지 상태를 알지 않습니다.
- 독립 monotonic 제어 주기, 시퀀스 더블버퍼, 최신값 사용 정책을 유지합니다.
- 지연·누락 평가는 shm_logger만 수행합니다. 응답 지연으로 제어를 중단하거나 시뮬레이터와 기다리는 로직을 추가하지 않습니다.
- 각 모션 클래스는 YAML 경로를 받고 자기 항목을 직접 파싱·검증합니다. 새 설정 구조체·별도 파서·runtime/IO 클래스는 만들지 않습니다.
- 이번 구현의 센서리스 범위는 **발끝 단일 접촉의 3차원 힘**입니다. 다중 링크 접촉/6축 wrench의 분리 관측은 보장하지 않습니다.

## 2. 먼저 해결할 현재 구현과의 차이

1. 현재 XML은 torque motor이며 TaskController에서 내부 위치 서보를 대수적으로 모사합니다. 요구사항대로 실제 위치 목표가 IO를 통과하고 MuJoCo의 위치 actuator가 PD를 수행하는 경로로 전환해야 합니다.
2. 현재 SHM에는 q/dq/g만 있습니다. 관측에 필요한 모터 토크 추정치, 일관된 모델 항, 발끝 위치/Jacobian과 측정 시간 정보가 필요합니다.
3. 현재 damping=2, frictionloss=0.2는 QDD의 저마찰 실측값이 아닙니다. 특히 MuJoCo frictionloss는 qfrc_passive의 단순 점성항이 아니라 별도 제약입니다. 그대로 잔차에 포함시키면 접촉으로 오인할 수 있습니다.
4. Force/Admittance 요구 식에서 x_ref를 적분하기만 하면 최종 q_target에 영향을 주지 않습니다. 접촉 상태에서만 작동하는 작은 가상 임피던스로 이를 출력에 연결할 것을 제안합니다.
5. 현재 발끝 condim=6은 접촉 모멘트도 생성할 수 있습니다. 최초 검증은 단일 법선 접촉으로 제한하고, 전시용 접촉 쌍은 condim=1부터 확인합니다. 접선 힘 확장은 condim=3으로 검증하며 6축 모멘트를 3축 힘으로 오인하지 않도록 합니다.

## 3. 위치 서보와 모델 계층

- MuJoCo actuator를 position servo로 바꾸고 외부 입력 단위를 rad로 통일합니다. 기존 Nm ctrlrange는 그대로 옮기지 않고 토크 한계를 forcerange로 설정합니다. 관절 목표각/등가 오프셋 한계는 별도로 둡니다.
- 내부 물리/서보 초기 1 kHz, 외부 RFOB/힘 제어 기본 1 kHz, 비교 시험 500 Hz. 외부 목표각은 다음 발행까지 ZOH 유지합니다.
- 실제 서보의 법칙을 먼저 고정합니다: tau_m = Kp_m(q_target−q) − Kd_m*dq. D 게인을 몰래 상쇄하지 않으며 실제 드라이버의 D 정의가 다르면 모델을 명시적으로 맞춥니다.
- Kp_m/Kd_m는 실제 내부 서보와 surrogate 환산에 반드시 동일해야 합니다. 물리 드라이버 설정은 motor_servo 공통 항목을 단일 출처로 두는 방향을 제안합니다. 모션별 외부 게인은 각 블록에 유지합니다. 이 전환 때 기존 호밍/중력보상도 동일한 위치 명령 인터페이스로 이행하여 단위를 섞지 않습니다.
- 중력은 기존처럼 별도 mjData에 qpos 복사/qvel=0/mj_forward 후 qfrc_bias로 얻습니다.
- M/h/passive/J/x는 q/dq만 설정한 모델 계산에서 얻습니다. 실제 장면의 접촉·constraint 해를 모델 항에 섞지 않습니다. armature를 포함한 관성 정의를 일치시킵니다.
- MuJoCo 계산과 실제 위치 actuator 설정은 기존 simulator IO 함수에 둡니다. TaskController와 motion 클래스는 MuJoCo 타입/링크를 추가하지 않습니다.
- 접촉 전시 scene에 마우스로 움직일 물체를 추가합니다. 우선 mocap 구동 물체를 사용하고 움직임을 보간합니다. 물체 좌표/접촉 여부는 제어 입력으로 보내지 않습니다. 발끝 외 다른 접촉으로 시작하는 시험은 분리합니다.

## 4. RFOB 단독 검증

새 ReactionForceObserver를 MagneticTouchController 내부 구성요소로 둡니다. 처음에는 가속도 잔차 방식, 다음 확장으로 momentum observer를 구현합니다.

1. q/dq, **실제 제한 적용 후 모터 토크 추정치**와 같은 측정 시점의 모델 항을 사용합니다. 명령 토크를 실현 토크로 간주하지 않습니다.
2. qddot는 dq 차분과 필터로 추정합니다. MuJoCo qacc를 제어 입력으로 사용하지 않습니다. qacc는 분석/테스트의 별도 기준으로만 사용할 수 있습니다.
3. 같은 측정 시퀀스를 다시 읽었다고 새 차분 샘플로 처리하지 않습니다. 입력 측정 간격과 제어 tick 간격을 구분합니다. 센서 dt 메타데이터는 관측 미분 전용이며 제어 루프를 맞추거나 기다리게 하지 않습니다. MuJoCo backend는 물리 측정 간격, 하드웨어 backend는 센서 간격을 제공하도록 정의합니다.
4. tau_ext = M*qddot_est + h − tau_m − tau_passive_model을 계산합니다. 모델 항/토크/가속도 추정의 시간 정렬과 필터 지연을 함께 검증합니다.
5. F_ext는 감쇠 의사역행렬로 계산하고, Jacobian의 최소 특이값/조건을 사용해 관측 유효성을 판정합니다. 특이점에서 임의로 크게 증폭한 힘으로 접촉을 선언하지 않습니다.
6. 20 Hz 저역통과 필터를 초기값으로 사용합니다. 무접촉 정지·자유 운동·단일 발끝 외력에서 영점/부호/편향/대역폭을 확인합니다.
7. 점성 마찰과 모델링할 수 있는 마찰은 명시적으로 보정합니다. dry friction과 관절 한계 반력을 실제 qfrc_constraint로 빼서 정답을 만들지 않습니다. 최초 모델은 dry friction을 명시적으로 낮춘 기준 시험과 원래 마찰을 넣은 민감도 시험으로 구분하며 UI/로그에 설정을 남깁니다. QDD 특성이라고 주장할 값은 향후 실측으로 교체합니다.

**Ground Truth 분리:** mj_contactForce, qfrc_constraint, qfrc_applied/xfrc_applied의 알려진 외력, 물체 위치/접촉 normal은 시험·표시 채널에만 둡니다. 제어기/RFOB가 이 값으로 감지·방향·힘·해제를 결정하지 않습니다. GT 접촉력은 발끝-대상 물체 접촉만 합산하고 contact frame→기준 좌표 변환과 geom 순서 부호를 검증합니다.

## 5. 힘 부호와 상태 머신

F_ext를 **환경이 로봇에 가하는 힘**, n을 **로봇이 물체를 누르는 방향**으로 정의합니다.

- 접촉 확정 시 n = −F_ext_est / ||F_ext_est||, x_contact=x_toe를 저장합니다.
- 압축력 추정은 F_c_est = −n^T F_ext_est입니다.
- 제어 토크는 g + J^T(n*f_cmd)입니다. 알려진 방향의 정적 접촉 시험으로 부호를 먼저 고정합니다.
- 감지 후 n은 우선 고정합니다. 물체의 접촉 normal을 읽어 갱신하지 않습니다.

| 상태 | 동작/전이 |
| --- | --- |
| FREE | g 보상, 외부 위치 강성 0, RFOB 관측. 추정 힘이 1.5 N 초과하면 접촉 후보 |
| CONTACT_CANDIDATE | 디바운스와 유효성 확인. 제안 초기 확인 시간 20 ms, 미달하면 FREE. 확정하면 방향/시작점 저장 |
| FORCE_FOLLOWING | 3 N 목표를 제안 초기 200 ms 동안 ramp. 접촉 방향 힘 제어와 제한된 admittance 수행 |
| RELEASE_PENDING | 추정 접촉력 0.5 N 미만이면 즉시 x_ref 적분 동결. 힘 명령을 완화하고 100 ms 미만 재접촉 시 연속성 있게 복귀 |
| FREE 복귀 | 0.5 N 미만 100 ms 확정 또는 작업영역/추종 범위 이탈 시 접촉 힘 명령 0, 적분/래치 초기화, g 보상으로 즉시 복귀 |

후보 취소·재접촉 경계에는 별도 히스테리시스를 설정합니다. 감지 확인 20 ms, ramp 200 ms는 요구사항에 없던 **제안 초기값**이며 시험 후 조정합니다. invalid observer/Jacobian에서는 force-following을 해제하고 이유를 UI·로그에 명시합니다. q 자체가 무효이면 명시적 FAULT로 출력 중단합니다. 상태 전환을 조용히 숨기지 않습니다.

## 6. Force Feedback와 제한된 Admittance

초기에는 스칼라 법선 힘 루프를 먼저 검증하고, 통과 후 admittance를 연결합니다.

- f_force = Fd_ramp + Kf*(Fd_ramp−F_c_est)
- x_ref_dot = sat_velocity(ka*(Fd_ramp−F_c_est)*n), 속도 상한 0.03 m/s
- 접촉 시작점 기준 법선 변위 s_ref를 ±0.015 m로 제한합니다. 접촉 불확실/상실·포화 상태에서는 누적을 동결하거나 되감는 anti-windup을 적용합니다.
- **제안 연결 항:** f_adm = Kx*n^T(x_ref−x) − Dx*n^T*x_dot. 이 항은 FORCE_FOLLOWING에서만 활성화하고 FREE에서는 0입니다. 접선 방향 위치 고정은 초기에는 넣지 않습니다.
- 최종 f_cmd = sat_[0,5 N](f_force + f_adm), F_cmd=n*f_cmd. 힘 루프와 가상 임피던스가 중복 증폭하지 않도록 낮은 게인부터 단계적으로 조정합니다.
- q_target = q_now + Kp_m^-1*(g + J^T*F_cmd).
- 힘·목표각 변화율·등가 각도 오프셋·토크·관절 위치·workspace를 제한합니다. 5 N은 명령 상한이며 실제 접촉 충격의 상한을 보장하는 숫자로 취급하지 않습니다.
- ±15 mm 안에서 접촉하며 움직이는 물체를 따라갑니다. 접촉 상실 후 보이지 않는 물체를 찾아 무제한 이동하지 않습니다. 공간 한계에 닿으면 force-following을 해제합니다.

Kf/ka/Kx/Dx와 실제 위치 서보 gain·접촉 compliance는 아직 수치가 정해지지 않았습니다. 임의의 최종 게인을 확정하지 않고, 정적 힘 시험→저속 이동 시험 순으로 선정합니다.

## 7. 설정/파일별 변경 범위

- config/control_config/control_config.yaml: motor_servo 물리 설정, magnetic_touch의 목표 힘/임계값/디바운스/히스테리시스/ramp/필터/게인/속도·변위·workspace·특이점·출력 한계. 요구 초기값 1.5/3/5/0.5 N, release100 ms, filter20 Hz, speed0.03 m/s, stroke±0.015 m를 반영합니다. SI 단위를 사용합니다.
- app/motion_controllers/magnetic_touch_controller.{hpp,cpp}: 파일 경로 기반 초기화, FSM, 힘 루프, 제한 admittance, surrogate 목표각.
- app/motion_controllers/reaction_force_observer.{hpp,cpp}: 잔차/필터/힘 변환/유효성. MagneticTouchController가 소유하며 같은 YAML 경로로 자기 설정을 읽습니다.
- app/task_controller: 새 컨트롤러 핸들과 모드 선택 호출, 필요한 명명된 입력/출력만 추가. 기존 독립 주기 유지.
- include/shared_memory.hpp: 관절 토크 추정·모델 항·발끝 운동학, 목표각 명령, observer/FSM 계측. 관절 3개/3x3 행렬의 명명된 필드로 유지하고 새 메시지 프레임워크를 만들지 않습니다. GT 값은 검증 채널로 분리합니다.
- app/mujoco_simulate: 실제 position actuator, 모델 항 산출, 물체 조작, GT 계산, UI/그래프. 모델/렌더 source 수정 시 복사 출처 해시 갱신.
- app/shm_logger: observer/GT/force error/좌표/전이 이벤트/명령·실제 출력/포화·관측 무효 사유를 기록하고 오차·지연·누락을 평가합니다.

## 8. UI

기존 호밍을 유지하고, Zero-G와 Magnetic Touch 활성화를 추가합니다. Magnetic Touch 안에서는 FREE/후보/FOLLOW/해제대기 상태를 구분합니다.

- 목표 힘 슬라이더: 0..5 N. 입력 변경도 ramp/명령 한계를 거칩니다.
- 추정 법선 힘·GT 법선 힘·목표 힘 비교 그래프. 필요 시 3축 벡터도 표시합니다.
- 발끝 실제/목표 위치, 오차, 접촉 시작점, ±15 mm 추종 범위를 표시합니다.
- 마우스 물체 이동, 설정된 이동 속도 제한/보간. 물체 위치는 UI·시험용으로만 사용합니다.
- 감지/해제/한계 이탈/관측 무효/포화 이벤트와 이유를 표시합니다.

## 9. 검증 순서와 완료 조건

1. **위치 서보:** q_target(rad)→실제 토크(Nm), Kp/Kd, saturation, outer500/1000Hz+inner1kHz ZOH를 검증합니다. 자유 모드의 잔류 저항을 damping/friction/서보 D/ZOH로 나누어 관측합니다.
2. **RFOB 단독:** 무접촉 영점·정지/자유 운동·알려진 힘·마찰/모델 오차를 시험합니다. GT를 제어 채널에서 제거해도 제어 출력이 달라지지 않는지 검사합니다.
3. **정적 접촉:** 물체 정지 상태에서 접촉 감지, 3 N ramp/유지, 해제100ms와 적분 동결을 확인합니다.
4. **이동 추종:** 0..30 mm/s, ±15 mm 안의 왕복 이동에서 힘 오차/추종 오차/지연을 측정합니다. 범위 이탈 및 빠른 물체 이탈 시 FREE 복귀를 검증합니다.
5. **예외:** 특이 자세, 관절 한계 접근, 포화, observer 노이즈/오류, 비발끝 접촉, 재접촉, 리셋/모드 전환을 시험합니다.
6. **상호작용:** 실제 GUI 마우스 조작과 자동 스크립트 물체 궤적을 모두 시험합니다. 기록 항목은 force RMSE/정상상태 오차/observer bias·RMSE/감지·해제 시간/전환 peak force·impulse/추종 지연/포화·누락 횟수입니다.

정확도·충격·추종 지연의 수치 합격선은 baseline 측정 후 명시합니다. 요구된 힘·속도·거리·토크 한계, 해제 시간과 GT 비사용은 먼저 검증 조건으로 고정합니다. 500/1000Hz 사이에서 자동으로 주기를 바꾸거나 지연으로 루프를 정지시키지 않습니다.

일시정지와 무제한 속도 headless는 센서 시간/제어 시간의 관계가 실시간 전시와 다릅니다. 독립 제어 동작은 유지하며 이를 숨기지 않습니다. 초기 제어 성능 판정은 정상 물리 측정 간격을 가진 실시간 조건에서 수행하고, headless 성능 시험에는 측정 시계 의미를 명시합니다. 물체 접촉 해석의 시간 정렬 없이 빠른 headless 결과를 실시간 성능으로 간주하지 않습니다.

## 10. 권장 구현 단계

**1차 PR/검토 단위:** 위치 서보 전환 + 모델/관측 데이터 경로 + RFOB와 GT 비교. 이 단계에서는 자동 힘 추종을 활성화하지 않습니다.

**2차:** FREE/감지/해제 상태 머신 + 정적 3 N 힘 유지 + 명령 ramp/한계.

**3차:** 제한 admittance + 마우스 물체 추종 + 전시 UI/예외 시험. Momentum observer는 1차 잔차 방식의 정확도·시간 정렬을 확인한 뒤 별도 확장합니다.

## 근거 문서

- MuJoCo 3.3.2 [position actuator](https://mujoco.readthedocs.io/en/3.3.2/XMLreference.html#actuator-position): 위치 서보 gain/damping과 입력 단위·힘 한계.
- MuJoCo 3.3.2 [동역학·friction loss](https://mujoco.readthedocs.io/en/3.3.2/computation/#friction-loss): dry friction의 별도 제약 처리.
- MuJoCo 3.3.2 [접촉력과 좌표계](https://mujoco.readthedocs.io/en/3.3.2/programming/simulation.html#contact): contact frame의 힘/토크와 변환.
