# 제어 모드와 시뮬레이션

현재 실행/Arm/Release와 데이터 상태는 [README](../README.md)를 우선 확인하십시오. 빌드·기본 실행은 [README](../README.md), 패킷 규약은 [CAN HAL](can_hal.md)을 참고하십시오. 설정을 변경하면 데몬을 재시작합니다. 아래 알고리즘 설명과 예시 수치는 현재 사용자 튜닝 파일의 값을 대신하지 않습니다.

## 시뮬레이션 엔코더 원점

[simulation.yaml](../config/simulation/simulation.yaml)의 `motor.require_zero_position_setup`을 사용합니다. 기본값 `false`는 제로셋 없이 모델 관절각을 그대로 CAN 위치로 반환합니다(패킷 양자화 오차 포함). 호스트 오프셋을 더하지 않으며 enable 전에 관절이 움직여도 좌표가 바뀌지 않습니다.

`true`는 모터 생성 시 축별 초기 표시 각도를 −π~π rad에서 독립적으로 뽑습니다. 시뮬레이션 입력 인터페이스가 이 정책을 **일반 Zero set(모드 6) 요청**으로 전달합니다. 모든 세션은 DISARMED로 시작하며 Arm 이후에 탐색합니다. enable/release·일시정지·물리 리셋은 임의 원점을 다시 뽑거나 런타임 보정값을 지우지 않습니다. 세 축 보정 전에는 H/G/M/T/B 요청을 거부하며, 거부한 요청을 보정 후 뒤늦게 실행하지 않습니다. D로 중단하고 Z로 처음부터 재시도할 수 있습니다.

### 스토퍼 기반 Zero set

[control.yaml](../config/control/control.yaml)의 `zero_set`에서 HR(hip) → HP(thigh) → KP(calf) 순서로 탐색합니다. 원시 CAN 엔코더 좌표에서 선택한 축의 위치 목표만 `direction × speed_rad_s × dt`씩 이동시키며 나머지 축은 진입/후퇴 자세를 유지합니다. 중력·모델 참값·역기구학을 이용해 미지의 원점을 추측하지 않습니다.

접촉은 **탐색 방향의 측정 토크 ≥ contact_torque_nm**, **|관절 속도| ≤ contact_velocity_rad_s** 조건이 `contact_duration_sec` 동안 새 피드백에서 유지될 때 확정합니다. 측정 토크는 CAN Iq × 검증된 Kt이며, 목표 토크나 시뮬레이션 접촉 GT를 사용하지 않습니다. 접촉 후 해당 축의 `stop_position_rad`를 C3로 전송하고, 에코와 새 좌표의 피드백을 확인한 뒤 `backoff_rad`만큼 물러나 정착하면 다음 축으로 넘어갑니다. 완료하면 후퇴한 자세를 유지하며 H 등 일반 모션을 선택할 수 있습니다.

- 탐색 방향과 스토퍼 관절각은 현재 `zero_set.direction`과 `zero_set.stop_position_rad`를 사용합니다. 스토퍼를 모든 축의 0 rad로 취급하지 않습니다.
- C3 오프셋은 `round(stop_position_rad × 18000 / π)`를 signed int16 little-endian으로 전송합니다. 이 값은 현재 스토퍼 위치에 부여할 출력축 각도이며, 현재 피드백과의 차이값이 아닙니다.
- 관측과 MIT 목표는 모터의 C3 좌표를 그대로 사용합니다. 호스트의 추가 위치 오프셋은 없습니다.
- `maximum_push_nm`은 보정 중 접촉/유지 노력 상한으로 공통 `limits.torque_nm` 이하여야 합니다. 위치 오차도 제한하여 스토퍼에서 목표가 무한히 누적되지 않게 합니다.
- 이동량·축별 시간 초과, 피드백 중단, 비유한 입력은 FAULT로 표시하고 모든 모터 출력을 해제합니다. 일시정지 중에는 탐색/접촉 판정 시간도 정지합니다.

**영점은 모터의 C3 런타임 좌표에 설정합니다.** HAL은 C3를 한 번 전송하며 에코를 확인한 뒤 20ms 동안 위치 명령을 차단합니다. 이후 중립 C0로 새 피드백을 받고, 제어기가 새 좌표의 명령을 발행했을 때만 위치 제어를 재개합니다. 에코 불일치·에코 또는 피드백 시간 초과는 출력을 해제하는 통신 fault입니다. ROM/EEPROM과 설정 파일은 수정하지 않습니다. 모터 전원 재시작 시 영점은 사라집니다. 새 SHM 세션에서도 좌표 확인을 위해 다시 제로셋합니다. 직접 effort 디버그는 처음부터 모델 관절각을 사용하며 C3 제로셋 요청을 지원하지 않습니다.

현재 기본 장면에서는 초기 발끝이 주황색 접촉 물체와 겹칩니다. 자동 보정에서 이 물체와의 접촉이 감지되면 `ExternalContact` 오류로 출력을 해제합니다. Ctrl 드래그/물체 이동으로 경로 밖에 놓고 Z를 눌러 재시도하십시오. 이 시뮬레이션 정보는 보정을 **거부**하는 데만 쓰며, 영점 산출에는 쓰지 않습니다.

토크와 정지만으로 기계 스토퍼와 임의의 장애물을 구별할 수 없습니다. 방향·알려진 스토퍼 각도·접촉 토크를 실제 구조에 맞게 설정하고 탐색 경로를 확보해야 합니다. 관절의 탄성 스토퍼 침투, 엔코더/CAN 양자화와 전류 노이즈 때문에 보정 오차가 남을 수 있습니다. 물리 XML·질량·마찰·기존 모션 게인은 변경하지 않습니다.

## GUI 조작

기존 Classic GUI와 독립 Filament 창에서 아래 프로젝트 조작을 사용할 수 있습니다. Filament 패널·카메라 추가 키는 [시각화 안내](visualization.md)를 참고하십시오. 시뮬레이션 전용 데이터와 승인 조작은 `simulation_ipc`로 교환하며 하드웨어 SHM에는 넣지 않습니다.

**시작은 DISARMED, 선택된 기본 모션은 Damping**입니다. Arm 요청 전에는 출력하지 않습니다. `require_zero_position_setup: true`인 새 세션은 먼저 Zero set을 수행합니다. Damping은 테스트용 직접 토크 제어입니다. HAL이 MIT `C0` 또는 RMD `A1`에서 구성한 C3 기준 누적각과 샘플 시각으로 `dq_raw=(q_now-q_prev)/dt`를 계산하고, **30 Hz의 1차 LPF**를 거친 속도로 `tau_cmd=-damping.gain_nm_s_rad*dq_filtered`를 계산하여 `limits.torque_nm`으로 제한합니다. 축별 실제 샘플 간격에 따라 `alpha=1-exp(-2*pi*30*dt)`, `dq_filtered += alpha*(dq_raw-dq_filtered)`로 갱신합니다. 누적각에는 2*pi 래핑을 적용하지 않습니다. `kp=0`, `kd=0`, `v_des=0`, `tau_ff=tau_cmd`를 보내며 모터 속도 피드백은 이 감쇠 계산에 사용하지 않습니다. 최초 샘플·재진입·샘플 시각 리셋에서는 필터 속도와 토크를 0으로 시작하고, 새 샘플이 없으면 직전 필터 값을 유지합니다. MIT 위치 피드백이 무효이거나 오래되면 토크를 중단하고 필터를 초기화합니다. 발끝 진입 위치는 GUI·로그용으로 캡처하며 위치 복원력이나 중력 보상은 더하지 않습니다.

```yaml
damping:
  gain_nm_s_rad: [1.0, 1.0, 1.0]  # 소프트웨어 D, 모터 MIT kd는 0
```

MIT 경로에서 HAL은 C4로 검증한 `T_MAX`에도 토크를 제한하고 8-bit `[-T_MAX,+T_MAX]`로 양자화합니다. 현재 `T_MAX=33`이면 간격은 약 0.2588 N·m이며, 0 명령은 `0x80`으로 인코딩되어 선형 디코딩 시 약 +0.1294 N·m입니다. 가상 CAN 모터도 이 오차를 반영합니다. 직접 디버그 시뮬레이션은 패킷 양자화 전 명령을 사용합니다.

| 입력 | 동작 |
| --- | --- |
| A / X | 명시적 Arm / Release. fault 해제는 별도 Clear fault 버튼 또는 Input 명령 |
| Z | hip → thigh → calf 스토퍼 탐색·런타임 영점 보정 시작/재시도 |
| D | 테스트용 토크 Damping: 위치 차분, kp=kd=0, tau_ff 직접 출력 |
| H | 명시적으로 5차 보간 호밍 후 위치 P + 모터 MIT D 자세 유지 |
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

모든 모션의 소프트웨어 토크 타깃 상한은 최상위 `limits.torque_nm` 하나로 설정합니다.

```yaml
limits:
  torque_nm: 60.7  # 스칼라 또는 HR/HP/KP 순서의 3개 배열
```

Homing/Gravity/Magnetic/Tracking/Ball과 TaskController가 계산한 토크 타깃을 이 값으로 제한합니다. MIT 내부 D가 추가하는 감쇠는 이 소프트웨어 제한에 포함되지 않으며, 실제 모터 출력에는 모터 프로파일과 물리 actuator의 한계가 적용됩니다. 개별 모션의 옛 `torque_limit_nm`/`output_torque_limit_nm`은 오류로 안내합니다. ILC의 `correction_limit_nm`과 `update_limit_nm`은 학습 정책이며, 기본 제어와 ILC 합산 타깃에 공통 한도를 적용합니다.

모든 모션은 `position_reference`와 `torque_command`를 분리해 출력합니다. `control.yaml` 최상단의 `torque_reference_mode`로 `control.cpp`에서 모터 명령을 만드는 방식을 선택합니다. 설정 변경 후 데몬을 재시작합니다. 누락되거나 아래 두 값 이외의 설정이면 시작 오류입니다.

```yaml
torque_reference_mode: torque_ref_as_position  # 또는 native_torque_ref
```

| 설정 | 위치 명령 `q_MIT` | 토크 명령 `tau_ff` |
| --- | --- | --- |
| `torque_ref_as_position` (기본) | `q_ref + tau_ref/kp_motor` | 0 |
| `native_torque_ref` | `q_ref` | `tau_ref` |

두 방식 모두 각 모션의 `kp/kd`를 유지하고 `v_MIT=0`을 보냅니다. 위치 P와 토크 보상의 합산 `kp*(q_MIT-q_measured)+tau_ff`가 `±limits.torque_nm`을 넘으면 위치 명령을 제한합니다. native 방식에서는 `tau_ff`를 보존하며 위치 명령의 허용 범위를 `q_measured+(-limit-tau_ff)/kp`부터 `q_measured+(limit-tau_ff)/kp`까지로 잡습니다. 따라서 양자화·모터 프로파일 제한 전의 합산 노력은 두 방식이 같습니다. 모터 내부 D는 이 한도와 별개입니다. native 토크에는 HAL의 C4 `T_MAX` 제한과 CAN 토크 양자화가 추가로 적용됩니다.

Homing·Gravity·Magnetic·Tracking·Ball 생성자는 `torque_ref_as_position`에서 한 축이라도 MIT `kp=0`이면 시작을 거부합니다. `native_torque_ref`에서는 축별 `kp=0`을 허용하며, 그 축은 위치 나눗셈·보정을 거치지 않고 `q_ref`, `kd`, 제한된 `tau_ff`를 보냅니다. 위의 native 위치 제한식은 `kp>0`인 축에만 적용됩니다. 음수·비유한 게인은 두 방식 모두 거부합니다. CTC의 모델 기반 피드백 게인은 MIT kp와 별개입니다.

Damping 테스트는 이 선택과 무관하게 `kp=kd=0`, `tau_ff=tau_cmd`를 유지합니다. Zero set은 토크 성분이 0인 별도 위치 제어로, `zero_set.kp>0` 조건과 기존 `maximum_push_nm` 제한·C3 처리를 유지합니다.

| 모션 | 위치 성분 `q_ref` | 토크 성분 `tau_ref` |
| --- | --- | --- |
| Homing (`torque_control: false`) | 보간한 호밍 위치 | 0 |
| Homing (`torque_control: true`) | 보간한 호밍 위치를 기록; 출력 변환에서 P 중복 방지 | `kp*(q_ref-q)-kd*dq` |
| Gravity | 현재 측정각 | 중력/bias·건마찰 보상 |
| Magnetic | 현재 측정각 | 중력/bias + 작업공간 접촉 토크 |
| Tracking 실행 | 현재 측정각 | IDFF·작업공간 피드백·ILC 또는 CTC 토크 |
| Tracking HOLD | 캡처한 유지 위치 | 중력 보상 |
| Ball | FK 경로의 목표 관절각 | 중력 보상 |
| Zero set | 탐색·후퇴 위치 | 0; 별도 `maximum_push_nm`으로 모터 위치 오차 제한 |
| Damping | 현재 위치, kp=kd=0 | 위치 차분으로 계산한 `-D*dq`, tau_ff 전송 |

### 호밍 토크 추종

`homing.torque_control`의 기본값은 `false`이며 기존 모터 MIT 위치 제어를 사용합니다. `true`이면 같은 5차 보간 궤적을 따라 호스트에서 `tau = kp*(q_ref-q) - kd*dq`를 계산합니다. `homing.kp/kd`를 그대로 사용하고, 궤적의 속도 레퍼런스는 사용하지 않습니다. 완료 후에도 최종 위치에 대한 PD 토크를 유지하며, 중력 보상을 추가하지 않습니다.

```yaml
torque_reference_mode: native_torque_ref
homing_velocity_lpf:  # YAML 최상단, 호밍 호스트 D항에만 적용
  enabled: true
  cutoff_hz: 20.0
homing:
  torque_control: true
  differentiate_position: true
  # 기존 kp, kd, position_rad, duration_sec 사용
```

`homing.differentiate_position: true`이면 MIT 누적 위치를 축별 실제 측정 시간 간격으로 차분해 `dq`를 계산합니다. `false`이면 모터가 보고한 `dq`를 사용합니다. 두 입력 모두 최상단 `homing_velocity_lpf.enabled`로 1차 LPF를 선택하며, `cutoff_hz`는 활성화 시 양수여야 합니다. LPF 계수는 `1-exp(-2*pi*cutoff_hz*dt)`입니다. 새 엔코더 샘플에서만 차분·필터를 갱신하고 중복 샘플에서는 이전 값을 유지합니다. 호밍 재진입이나 샘플 시간/순서 초기화 시 차분 이력을 초기화해 첫 샘플의 속도를 0으로 둡니다. 새 설정이 생략된 기존 파일은 모터 `dq`를 필터 없이 사용합니다. 다른 모션과 모터 내부 감쇠 설정에는 적용하지 않습니다.

`native_torque_ref`에서는 MIT `kp=kd=v_des=0`, `tau_ff=tau`로 출력합니다. `rmd_torque_ref`에서는 A1으로 출력하며 `rmd_damping_constant` 설정값을 그대로 보냅니다. 호밍의 호스트 D와 펌웨어 RMD 감쇠는 각각 적용됩니다. `torque_ref_as_position`에서는 `q_MIT=q_measured+tau/kp`, `kd=v_des=tau_ff=0`으로 변환합니다. 이미 토크에 들어간 호밍 위치 오차를 다시 더하지 않습니다. 모든 경로에 `limits.torque_nm`을 적용하며, 코깅 보상도 기존 공통 합산 경로를 사용합니다. 설정 변경 후 제어기를 재시작해야 합니다.

### 21·36차 코깅 보상

`src/motion_controllers/two_order_anticogging.*`의 `TwoOrderAnticogging`이 모터 기계각의 21·36차 외란을 계산합니다. `control.yaml`의 `anticogging.enabled: true`로 활성화합니다. 이 섹션이 없으면 OFF이며, `scale` 생략 시 0.1을 사용합니다. 현재 적용값은 control YAML을 따릅니다. 감속비 G와 출력축 Kt는 `anticogging.motor_config`가 가리키는 모터 YAML의 `mit.gear_ratio`, `mit.kt_output_nm_a`에서 읽습니다. 경로는 control YAML 기준입니다.

```text
θm = G*q + motor_zero_rad
τcog = amp21_motor_nm*sin(21*θm + phase21_rad)
     + amp36_motor_nm*sin(36*θm + phase36_rad)
τopposing = clamp(-τcog, ±max_motor_comp_nm)
Iq = scale * clamp(τopposing/(Kt_output/G), ±max_comp_current_a)
τanti = Kt_output * Iq
τcommand = clamp(τcontroller + τanti, ±limits.torque_nm)
```

`motor_zero_rad`와 두 진폭·위상은 HR/HP/KP별 배열 또는 공통 스칼라로 설정합니다. 진폭은 **모터축 외란 토크 Nm**, 위상은 rad이며 양의 외란에 음의 보상을 냅니다. 웹 GUI의 속도 리플 계수(rad/s)를 이 토크 진폭에 그대로 넣지는 않습니다. 제공된 0.05 Nm와 위상 0은 예시이므로 각 축의 측정값으로 바꿉니다.

보상은 각 모션 계산 뒤 `torque_command_`에 한 번만 더하고 기존 출력 변환을 거칩니다. `torque_ref_as_position`에서는 합산 토크를 `kp`로 나눠 위치 목표에 넣고 `tau_ff=0`을 유지합니다. `native_torque_ref`에서는 합산 토크를 `tau_ff`로 보냅니다. Damping은 기존처럼 `kp=kd=0`이며 감쇠 토크와 보상 토크의 합을 직접 보냅니다. 최종 토크 및 P+FF 제한은 그대로 적용됩니다.

보상은 활성 운전의 Homing·Gravity·Magnetic·Tracking·Ball·Damping에 적용합니다. Zero set 중, 미보정 좌표, 출력 해제, 무효 피드백에는 적용하지 않습니다. SHM 구조나 CAN 프로토콜을 추가하지 않으며 기존 `debug_torque`에 보상까지 더한 최종 토크가 기록됩니다. 설정 변경 후 제어기 재시작이 필요합니다.

웹 Overview의 **State Machine 바로 아래**에서 축별 `phase21_rad`·`phase36_rad`를 조절합니다. 0.1 / 0.25 / 0.5 rad 스텝 버튼(기본 0.5)으로 증분을 선택합니다. ▲/▼ 또는 위상 값에 포커스를 둔 ↑/↓ 키는 선택한 스텝만큼 증감 요청을 보냅니다. 스텝 선택 자체는 모터 명령을 보내지 않습니다. 기본 위상은 YAML의 0이며, 목표값과 실제 적용값을 별도로 표시합니다. 조작은 현재 제어기 메모리에만 적용되고 YAML 파일을 덮어쓰지 않습니다. 재연결 시 제어기의 현재 값을 읽고, 제어기 재시작 시 YAML 값으로 돌아갑니다.

`anticogging.phase_slew_rad_s`의 기본값은 **5 rad/s**입니다. 목표 수신 시 적용 위상을 즉시 바꾸지 않고 제어 주기마다 최단 각도 방향으로 이동합니다. 기본 0.5 rad 요청은 약 0.1초에 적용됩니다. 같은 속도에서 0.1 rad는 약 0.02초, 0.25 rad는 약 0.05초입니다. 한 번의 갱신에 사용하는 경과 시간은 최대 10 ms로 제한하여 지연된 루프가 위상을 한꺼번에 따라잡지 않도록 합니다. 목표·적용 위상을 모두 `0 ≤ φ < 2π`로 정규화하고 2π는 0으로 취급합니다. 0/2π 경계에서도 최단 경로로 연속 적용합니다.

고정 관절각에서 한 차수의 위상 0.5 rad를 즉시 바꾸면 출력축 토크 변화는 최대 `2*G*scale*A*sin(0.25)`입니다. G=9, scale=1, 모터 진폭 A=0.05 Nm이면 **0.2226636 Nm**입니다. 5 rad/s로 전환하면 한 성분의 1 ms 변화는 **0.00225 Nm 이하**, 두 성분 동시 변경은 합계 **0.0045 Nm 이하**입니다. 이는 위상 변경에 따른 계산 토크 변화이며, 실제 관절 운동에 따른 변화와 CAN 양자화는 별도입니다. native 토크의 8비트 인코딩은 `2*T_MAX/255` 단위이므로 T_MAX=3 Nm에서 **0.0235294 Nm** 단계가 남습니다.

GUI 진폭 ▲/▼는 `amp21_motor_nm`·`amp36_motor_nm` 목표를 **0.01 모터축 Nm**씩 조절합니다. `amplitude_slew_nm_s` 기본값은 0.1 모터 Nm/s로, 고정 q에서 한 단계는 약 0.1초에 적용됩니다. 두 차수 진폭은 0 이상이어야 하며, 목표 변경 자체가 적용 진폭을 즉시 바꾸지는 않습니다. 위상·진폭 모두 현재 실행에만 반영되고 YAML은 자동 저장하지 않습니다.

클리핑 상태는 실제 보상 계산 시 기록합니다. 모터축 토크 제한 전 값, 전류 제한 전 값, 최종 소프트웨어 합산/P+FF 한계, native CAN `torque_max_nm` 범위를 구분합니다. 마지막 클리핑 원인을 1초 유지하여 웹의 4 Hz 상태 갱신 사이에서 생긴 제한도 표시합니다. Zero set 또는 비활성 출력에서는 보상 계산을 하지 않습니다. 웹의 출력축 보상 계산값은 모터축 진폭에 감속비·scale·보상 한계가 반영된 값이며, 최종 모션 전체의 출력 토크와 구분합니다.

라이브 요청과 상태 읽기는 기존 `WebSocket → Input → TaskController` 경로를 사용합니다. Input은 축 0..2, 차수 21/36, 위상 ±0.1 / ±0.25 / ±0.5 rad 또는 진폭 ±0.01 모터 Nm만 허용하고, 상태 읽기는 모션·구동 요청을 변경하지 않습니다. 웹 서버는 4 Hz로 목표/적용 위상을 읽으며 SHM 구조·ABI는 변경하지 않습니다.


기본 Homing·Ball·HOLD는 관절 위치 오차를 토크로 다시 계산하지 않습니다. `homing.torque_control: true`는 명시적으로 선택한 호스트 PD 예외이며 MIT P/D는 중복 적용하지 않습니다. RMD 감쇠는 사용자가 설정한 값을 유지합니다. CTC는 `M(q)*(Kp_ctc*(q_d-q)+Kd_ctc*(dq_d-dq))` 모델 기반 피드백을 사용하며 위치 성분을 현재 측정각으로 두어 별도 MIT 위치 복원력이 중복되지 않게 합니다. Tracking·Magnetic의 작업공간 속도 피드백, 궤적의 속도·가속도 계산, 역동역학과 RFOB도 유지합니다.

두 모드의 동역학 보상항은 각 설정의 `gravity_only`로 따로 선택합니다. 설정 변경 후 데몬을 재시작합니다.

```yaml
gravity_compensation:
  gravity_only: true
magnetic_touch:
  gravity_only: true
```

`true`는 별도 `mjData`에 qpos를 복사하고 qvel을 0으로 둔 뒤 `mj_fwdPosition()`·`mj_fwdVelocity()`로 구한 순수 중력을 사용합니다. `false`는 실제 속도에서 계산한 전체 `qfrc_bias`(중력+코리올리·원심력)를 사용합니다. Gravity의 `alpha`는 선택한 보상항 전체에 적용합니다. Gravity의 선택적 건마찰 보상과 RFOB의 전체 동역학 계산은 유지하며 물리 모델의 마찰·감쇠는 바꾸지 않습니다. 옵션 누락·잘못된 bool 값은 오류로 처리합니다.

### Magnetic의 GM 반력 추정

`ReactionForceObserver`는 가속도 잔차 대신 일반화 운동량 `p=M(q)*dq`를 사용합니다. 입력은 측정 모터 토크, 관절 위치·속도, 모델의 질량행렬·코리올리 행렬·bias·passive 및 기존 건마찰 모델입니다. 관절 가속도는 옵저버 입력에서 제거했습니다.

```text
u = tau_motor + tau_passive - tau_friction - bias + (C + C^T)*dq
innovation = p[k] - p[k-1] - 0.5*(u[k] + u[k-1])*dt
alpha = 1 - exp(-2*pi*bandwidth_hz*dt)
tau_external[k] = (1-alpha)*tau_external[k-1] + alpha/dt*innovation
F_external = J^(-T)*tau_external
```

기존 외력 부호를 유지하며, 힘 변환에는 기존 역행렬/DLS와 특이점·최대 추정 힘 검증을 사용합니다. 코리올리 행렬은 같은 MuJoCo 모델의 RNE 속도 이차항에서 계산하고 캐시합니다. `C*dq = bias-gravity`, `C+C^T = Mdot`을 모델 시험으로 검증합니다. 시뮬레이터 상태를 읽거나 C를 0으로 가정하지 않습니다.

첫 유효 샘플은 운동량 이력만 초기화합니다. 중복 관측은 적분하지 않으며 시퀀스 역행, 무효 입력, `maximum_sample_gap_sec`(기본 0.02초)를 넘는 공백에서는 이력을 비웁니다. CAN 세션은 개별 축 간격 대신 새 관절 관측을 구성한 단조시간 간격을 사용하고, 직접 시뮬레이션 디버그 모드는 생산자의 가상 측정 시각을 사용합니다. 무효 또는 재초기화 중인 값으로 접촉 토크를 만들지 않습니다.

선택적인 `magnetic_touch.observer.harmonic_compensation`은 기본 OFF입니다. 켜면 `motor_config`의 감속비로 `theta_motor=GR*q+motor_zero_rad`를 계산하고, 축별 `coefficients_nm`의 `[sin21, cos21, sin36, cos36]` 성분을 GM 잔차에서 뺍니다. 계수 단위는 **출력축 잔차 토크 Nm**입니다. 구동용 anticogging과 별도이며, 그 모터축 계수를 그대로 재사용하지 않습니다. C3 기준 q와 모터 위상 오프셋, 필터 적용 상태에 맞추어 식별한 계수가 필요합니다. 이 옵션 자체가 모터 토크를 추가하지는 않습니다.

### Magnetic의 곡면 접촉 추종

접촉이 확정되면 반력의 반대 방향으로 최초 법선을 잡고, `Following` 중에만 천천히 갱신합니다. 기존 법선으로 계산한 반력 투영값과 반력 크기가 모두 `contact_recover_n`보다 커야 하며, 새 방향이 현재 법선에서 `normal_max_deviation_rad` 이상 벗어나면 갱신하지 않습니다. 따라서 큰 접선 힘이 들어왔다는 이유만으로 법선을 그 방향으로 돌려 접촉 해제를 가리는 동작을 피합니다.

`normal_bandwidth_hz`의 기본값은 2 Hz입니다. 구면 보간으로 법선을 정규화하면서 `normal_max_rate_rad_s`(기본 π/2 rad/s, 90도/s) 이하로 회전시킵니다. 방향 편차 한계 기본값은 π/6 rad(30도)입니다. 세 항목은 `magnetic_touch` 아래에서 설정하고 제어기 재시작 후 적용합니다. 반력으로부터 얻는 방향이므로 미끄럼 마찰의 영향을 받으며, 정확한 표면 기하 법선을 보장하지는 않습니다.

접촉 앵커는 매 주기 `Δp_t = Δp - n*(n·Δp)`만큼 접선 방향으로 이동합니다. 목표는 `p_ref = p_anchor + n*d`이고, 강성·댐핑·힘 제어 및 `Jᵀ` 토크 변환은 모두 법선 방향에만 적용합니다. 접선 방향 위치 복원력은 추가하지 않습니다.

`maximum_displacement_m`은 앵커 기준의 실제 **법선 거리**와 목표 법선 변위 `d`를 제한합니다. 표면을 따라 이동한 전체 거리는 이 제한에 누적하지 않으며, 기존 전체 작업공간·관절·출력 토크 제한은 계속 적용합니다. `Releasing`에서는 법선 갱신과 어드미턴스 적분을 멈추고 기존 힘 램프·접촉 복구·해제 시간을 따릅니다. SHM 구조와 CAN 명령 형식은 변경하지 않습니다.

### Gravity의 건마찰 보상

`config/control/control.yaml`에서 아래 값을 바꾸고 **제어기/데몬을 재시작**합니다. 스칼라 또는 HR/HP/KP 순서 배열을 사용할 수 있습니다. 실행 중 자동 재로드나 GUI 슬라이더는 없습니다.

```yaml
gravity_compensation:
  friction_compensation:
    enabled: true  # false: 건마찰 보상 OFF
    coulomb_ratio: [0.95, 0.95, 0.95]
    velocity_scale_rad_s: 0.03
```

관절 속도에 비례하는 점성 보상과 `damping_ratio` 설정을 제거했습니다. 진입 위치를 유지하는 `task_stiffness`는 제거했으며 위치 성분은 항상 현재 측정각입니다. `kd_motor`는 모터 내부 MIT D에서만 적용합니다.

`friction_compensation.enabled=false`이면 건마찰 보상을 끕니다. 이때 소프트웨어 토크는 `alpha*h`입니다. 스위치 누락·잘못된 bool은 시작 오류입니다.

ON일 때 소프트웨어 토크는 `alpha*h + beta*Fc*tanh(dq/v0)`입니다. h는 위의 중력/bias 선택, beta는 `coulomb_ratio`, Fc는 모델 건마찰 값입니다. 이 항은 운동을 거스르는 감쇠가 아니라 건마찰 보상이며, 속도가 정확히 0이면 0입니다. 전체 passive 토크를 뒤집지 않으므로 스프링이나 점성 감쇠까지 상쇄하지 않습니다.

가상 관성 제어는 제거했습니다. 외력 추정 기반의 추가 보조 토크는 계산하지 않습니다. 이전 YAML의 `virtual_inertia` 항목은 삭제해야 하며, 남아 있으면 시작 오류로 안내합니다. 중력/bias 선택, 건마찰 보상 및 최종 노력 제한은 유지합니다.

가상 모터의 초기 좌표는 위의 `motor.require_zero_position_setup` 정책을 따릅니다. 스토퍼 제로셋에서는 물리 CAN과 같은 C3 요청·응답 경로로 모터 좌표를 변경하며, 제어기는 별도의 위치 보정값을 저장하지 않습니다.

## 원·타원 궤적 Tracking 데모

기본 실행 후 **Homing [H]로 HOLD까지 기다린 다음** 아래 단축키로 Tracking을 선택합니다. `H`·`G`·`M`으로 기존 모드에 돌아갑니다.

| 단축키 | 평면 | 고정하는 목표 좌표 |
| --- | --- | --- |
| `1` | XY | 진입 시 Z |
| `2` | XZ | 진입 시 Y |
| `3` | YZ | 진입 시 X |
| `T` 또는 Tracking 버튼 | 마지막으로 선택한 평면 (최초 XY) | 해당 평면 밖 좌표 |

평면을 바꾸면 현재 발끝 위치를 시작점으로 궤적과 속도 램프를 다시 시작하며, 이전 그래프·잔상을 지웁니다. 같은 평면 키를 반복해서 눌러도 궤적을 재시작하지 않습니다. 숫자 편집란 입력 중에는 평면 단축키를 처리하지 않습니다. 선택 평면은 GUI 상단 상태와 경로 그래프 제목에 표시됩니다.

[tracking 설정](../config/control/control.yaml)의 `trajectory.period_sec`로 진입 이후 주기, `trajectory.amplitude_m`의 월드 X/Y/Z 배열로 각 축의 반경을 조절합니다. 전체 폭은 반경의 2배입니다. 선택 평면의 두 축을 `(u, v)`로 두고 같은 위상의 `A[u]*sin(φ), A[v]*cos(φ)`를 사용합니다. 두 반경이 같으면 원, 다르면 타원입니다. 시작 위치가 튀지 않도록 기하학적 중심은 진입 위치에서 두 번째 축의 음수 방향으로 `A[v]`만큼 떨어진 곳이며, 이동 범위는 첫 번째 축 `entry[u]±A[u]`, 두 번째 축 `entry[v]-2*A[v]..entry[v]`입니다. `trajectory.timing`은 시간 배분 방식을 명시적으로 선택합니다.

- `uniform_acceleration` (현재 기본): 진입 이후 **발끝 목표 가속도 크기의 편차와 피크를 줄이는 속력** 프로파일입니다. 경로 곡률을 반영하여 접선 가속도와 법선 가속도의 합성 크기를 제한하는 순방향·역방향 속력 포락선을 초기화 시 계산합니다. `a² = (du/ds / 2)² + (κu)²`, `u = v²`를 사용하며 단위 가속도에서 구한 모양을 설정 주기에 맞춰 정규화합니다. 2048개 기하 샘플과 로그 위상 속력의 24개 코사인 조화항(가우시안 평활 폭 0.02rad)으로 가감속 전환을 매끄럽게 합니다. 속력은 양수이고 주기 경계에서도 위치·속도·가속도가 이어집니다. 원에서는 곡률이 일정하므로 정상 구간의 속력과 구심 가속도 크기도 일정합니다. 타원에서는 완전한 등가속도나 지정한 절대 가속도 상한을 보장하는 방식은 아닙니다.
- `constant_speed`: 기존 **Arc-length Reparameterization**입니다. 진입 이후 목표 속력은 `전체 경로 길이 / period_sec`이며, 곡률에 따라 가속도가 변합니다.

두 방식 모두 설정 주기와 경로 모양을 유지합니다. `trajectory.arc_length_samples`는 Simpson 적분 표의 구간 수이며 기본 4096, 정속은 64..65536, 가속도 프로파일은 256..65536입니다. 호 길이 또는 위상-시간 적분 표를 한 번 만들고, 제어 루프에서는 이진 검색과 구간 내 Newton 보정만 수행합니다. 프로파일 생성 오류·잘못된 `timing`·필수 설정 누락은 오류로 보고하며 다른 방식으로 자동 대체하지 않습니다.

`trajectory.ramp_sec` 동안에는 5차 함수로 **프로파일 시계의 진행 속도**를 0부터 높입니다. 반경은 축소하지 않으며 진입 위치에서 정지 상태로 출발합니다. 램프 이후 프로파일 시간은 `elapsed - ramp_sec/2`이므로 ILC의 시간 위상과 완주 경계는 유지됩니다. 목표 속도·가속도는 시간 변환과 램프의 미분을 포함합니다. 실제 발끝의 가속도는 추종 오차·IK 근사·토크 제한의 영향을 받습니다.

```text
(u, v) = XY: (X,Y), XZ: (X,Z), YZ: (Y,Z)
profile_clock = integral(quintic_speed_ramp(t), dt)
phi = inverse_time_profile(profile_clock mod period_sec)  # uniform_acceleration
# constant_speed: inverse_arc_length(path_length * (profile_clock mod period_sec) / period_sec)
x_ref[u] = entry[u] + A[u]*sin(phi)
x_ref[v] = entry[v] + A[v]*(cos(phi)-1)
x_ref[remaining axis] = entry[remaining axis]
v_toe = J*dq
F = K*(x_ref - x_toe) + D*(v_ref - v_toe)
q_d = IK(x_ref)
dq_d = J(q_d)# * v_ref
ddq_d = J(q_d)# * (a_ref - Jdot(q_d,dq_d)*dq_d)
tau_id = M(q_d)*ddq_d + bias(q_d,dq_d)
tau = gravity + alpha*(tau_id - gravity) + J^T*F
q_MIT = q + tau/kp_motor
v_MIT = 0
```

`J#`는 조건에 따라 실제 역행렬 또는 Adaptive DLS 연산을 뜻합니다. 이 로봇의 발끝 위치 자코비안은 3×3 정방행렬이므로 `σ_min >= singularity_threshold_m`이고 조건수가 `workspace.maximum_condition_number` 이하이면 스케일을 정규화한 실제 `J^-1`을 사용합니다. 그 밖에는 `J^T*(J*J^T + λ²I)^-1`을 Cholesky 분해로 풉니다. 역행렬 계산 실패를 다른 방식으로 숨기지 않으며, 잘못된 수치는 실패로 반환해 기존 HOLD/오류 처리를 거칩니다. 전체 궤적 조건수 제한은 그대로 적용되므로 DLS가 안전 검사를 우회하지 않습니다.

```text
threshold = max(singularity_threshold_m, sigma_max / maximum_condition_number)
z = min(1, sigma_min / threshold)
lambda^2 = lambda_min^2 + (lambda_max^2 - lambda_min^2) * (1-z^2)^2
```

`src/motion_controllers/numerics`는 모션 컨트롤러 내부 구현입니다. MuJoCo 백엔드는 이 모듈을 참조하지 않고 `std::array`와 MuJoCo 자체 행렬 연산을 사용합니다. 이 하위 모듈에 3×3 행렬·특이값·역행렬·DLS·벡터·삼각형 거리·조화급수 계산을 모았습니다. 같은 자코비안의 분해는 재사용하여 목표 속도와 가속도를 구합니다. 조건이 좋은 영역에서는 `lambda=0`, DLS 영역에서는 설정된 감쇠를 사용하므로 경계에는 `lambda_min`에 따른 작은 차이가 있습니다. RFOB도 같은 역행렬/전치 DLS 풀이를 쓰되 기존 최소 특이값 무효 판정과 고정 정규화 계수는 유지합니다. 특이점 부근의 증폭을 줄이는 대신 DLS 영역의 작업공간 재현 오차는 커질 수 있습니다.

`impedance.stiffness_n_m/damping_ns_m`은 월드 X/Y/Z 피드백 게인, `motor.kp_nm_rad/kd_nm_s_rad`는 HR/HP/KP MIT 내부 서보 게인입니다. `limits.torque_nm`은 기본 제어와 ILC를 합산한 최종 명령의 관절별 한계입니다. 전체 토크를 제한한 뒤 `torque_reference_mode`에 따라 위치 오프셋 또는 `tau_ff`로 보냅니다. 설정 누락·잘못된 값은 시작 오류로 보고합니다. 설정 변경 후 재시작합니다.

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
| `ctc` | 위치·속도 오차→가속도 피드백 `kp_per_s2`·`kd_per_s` |
| `ilc` | 반복 학습·위상 필터·보상 및 갱신 한계 |

`mode: impedance`는 중력+작업공간 임피던스를 사용합니다. ILC를 끄면 실행 중 관절 목표 IK/RNEA 없이 중력+작업공간 임피던스를 사용합니다. 실행 전 전체 경로 검사용 IK 모델은 모든 방식에서 필요합니다. CTC는 IDFF·임피던스 설정을 요구하지 않고, ILC를 끈 임피던스도 사전 검사용 `ik` 설정은 필요하며 CTC·IDFF 설정은 요구하지 않습니다. 임피던스에서도 ILC를 켤 수 있으며, 이때 `ik`와 최상위 `model_path`로 학습용 목표 관절각·속도를 계산합니다. IDFF/CTC 토크는 계산하거나 더하지 않습니다. 모델 경로는 계속 제어 YAML 최상위의 `model_path` 하나입니다.

`ik.damping_min_m`은 기본 감쇠 `λ_min`, `ik.damping_max_m`은 상한 `λ_max`, `ik.singularity_threshold_m`은 감쇠를 높이기 시작하는 최소 특이값입니다. 두 감쇠를 같게 설정하면 DLS 영역에서 고정 감쇠를 사용합니다. 정상 조건의 실제 역행렬 분기는 유지합니다. 누락되거나 잘못된 설정은 시작 오류로 보고하며 이전 고정 감쇠로 자동 대체하지 않습니다.

`tracking.mode: ctc`를 선택하면 **관절 공간 PD Computed Torque Control**로 전환합니다. 같은 시간 배분 궤적과 Adaptive DLS로 `q_d/dq_d/ddq_d`를 만들고, 관절 위치·속도 오차로 가속도를 보정한 뒤 **현재 측정 상태**에서 동역학을 계산합니다.

```text
ddq_cmd = ddq_d + Kp_ctc*(q_d-q) + Kd_ctc*(dq_d-dq)
tau_ctc = M(q)*ddq_cmd + bias(q,dq)
tau = clip(tau_ctc + tau_ILC, -limits.torque_nm, limits.torque_nm)
q_MIT = q + tau/kp_motor
v_MIT = 0
```

`ctc.kp_per_s2`의 단위는 s⁻²이며 양수여야 합니다. `ctc.kd_per_s`의 단위는 s⁻¹이며 0 이상의 유한한 값이어야 합니다. 두 항목 모두 HR/HP/KP 순서의 3개 배열입니다. `kd_per_s: [0,0,0]`이면 기존 위치 피드백만 적용됩니다. `kp`·`kd`라는 이전 키 대신 단위가 명시된 키를 사용합니다. 속도 오차 항은 질량행렬 곱 안에서 계산되며, 위치·속도 입력은 모터에서 받은 측정값을 사용합니다.

CTC에서는 작업공간 `impedance`나 IDFF의 `alpha`를 합산하지 않습니다. 모터 MIT D와 RMD damping constant는 기존 사용자 설정을 유지하므로 CTC의 D항과 각각 적용됩니다. `ik`의 Adaptive DLS/IK 파라미터는 IDFF와 CTC가 함께 사용합니다. CTC 실패 시 IDFF로 자동 전환하지 않고 측정 자세 유지 상태로 들어갑니다.

이상적인 비포화 오차 동역학에서 `Kp=omega_n²`, `Kd=2*zeta*omega_n`입니다. 예를 들어 `omega_n=10 rad/s`, `zeta=0.7`이면 `kp_per_s2: [100,100,100]`, `kd_per_s: [14,14,14]`입니다. 현재 런타임 Kp는 YAML에 저장된 값을 따르며, D항 추가가 기존 Kp를 자동 변경하지 않습니다.

CTC도 같은 계산용 모델·`mjData`를 재사용하고 `mj_rne()` 결과에 armature 관성을 더합니다. 물리 감쇠·건마찰·스프링·접촉력은 보상 모델에 포함하지 않습니다. 초기 CTC 게인은 시험용이며 실제 추종 오차와 토크 여유를 보며 조정합니다.

세 Tracking 방식 모두에 **토크 ILC(Iterative Learning Control)**를 독립적으로 더할 수 있습니다. `tracking.ilc.enabled: true`로 켜고 `false`로 끕니다. YAML 설정 변경 후에는 재시작합니다. `phase_lead_bins`, `smoothing_passes`, `retention`은 웹 GUI에서 실행 중에도 변경할 수 있습니다. [TrackingILC](../src/motion_controllers/tracking_ilc.cpp)는 TrackingController가 소유하고 자체 YAML 경로로 설정을 읽습니다. `mode: impedance`에서도 ILC를 지원합니다. 이 조합은 학습 오차용 관절 목표만 IK로 계산하며 기본 출력은 중력+작업공간 임피던스입니다. ILC를 자동으로 끄거나 IDFF로 전환하지 않습니다. IK 실패 시에는 다른 방식과 동일하게 오류를 표시하고 측정 자세 유지 상태로 들어갑니다.

```yaml
tracking:
  mode: ctc  # inverse_dynamics/impedance에서도 ILC 사용 가능
  ilc:
    enabled: true
```

ILC를 켰을 때 설정 검증상 궤적 주기 하한은 `max(phase_bins / control.frequency_hz, 4 × blend_sec)`입니다. 이는 물리적인 추종 가능 주기를 보장하지 않습니다.

정상 궤적의 위상을 `phase_bins`개로 나누어 주기별 관절 위치 오차를 모으고, **완주한 이전 주기**로만 다음 주기의 보상 표를 갱신합니다. P형 ILC와 Q 필터를 사용하며, 오차는 목표−측정입니다. 관절 속도 오차 학습 항은 제거했습니다.

```text
tau_ilc[k+1] = Q(retention*tau_ilc[k] + Kp_ilc*e_q[k])
tau_cmd = clip(tau_base + tau_ilc, -limits.torque_nm, limits.torque_nm)
```

`ilc.position_gain_nm_rad`는 HR/HP/KP 순서의 유효 학습 토크 게인이고 `ilc.retention`은 기존 학습값 유지율입니다. `phase_lead_bins`만큼 뒤쪽 위상의 오차를 앞쪽 보상에 반영하고, `smoothing_passes`번 순환 대칭 3점 필터(`1/4,1/2,1/4`)를 적용합니다. `ilc.update_limit_nm`은 표 한 지점의 주기별 변화, `ilc.correction_limit_nm`은 ILC 보상 자체의 토크 한계입니다. 최종 `limits.torque_nm`과는 별개입니다. 주기 경계에서는 `blend_sec` 동안 5차 보간으로 이전·새 보상을 연결하고 위상 표도 순환 선형 보간하므로 보상값이 경계에서 뛰지 않습니다.

웹 GUI의 ILC **반영**은 세 파라미터를 기존 Input 소켓으로 보내 제어 스레드에서 한 번에 검증·변경합니다. 범위는 `phase_lead_bins: 0..phase_bins/4`, `smoothing_passes: 0..32`, `retention: (0,1]`입니다. 새 값은 다음 학습 갱신에 사용하며, 현재 출력이나 기존 학습 표를 즉시 변경하지 않습니다. 모드 재진입으로 학습 표가 초기화되어도 이 런타임 설정은 유지하고 프로세스 재시작 시 YAML로 복원합니다. SHM 필드는 추가하지 않습니다.

속도 램프를 마친 뒤 첫 위상 교차점과 `warmup_cycles`를 기다립니다. 구간 하나라도 샘플이 없거나 제어기 최종 토크가 포화된 주기는 학습 갱신을 건너뛰며 GUI·터미널에 이유를 표시합니다. 이 경우 기존 보상 표는 유지합니다. 포화 판정은 **제어기 토크 제한** 기준이며 가상 모터·실기 내부 포화를 모두 알아내는 것은 아닙니다. 제어기는 기존처럼 최신 관절 상태로 독립 실행하며 응답 지연 감시나 시뮬레이터 일시정지 연동은 추가하지 않았습니다.

모드 재진입·평면 전환·리셋 시 학습 표를 지웁니다. IK 실패 시 ILC 출력도 0으로 하고 기존 자세 유지 상태로 들어갑니다. 학습값은 파일에 저장하지 않습니다. GUI에는 ILC 상태·갱신/건너뜀 횟수·관절별 보상 Nm·마지막 완주 주기 발끝 RMS를 표시하고 SHM/MCAP의 `tracking_ilc_*` 필드에도 기록합니다. 기존 IDFF/CTC 토크 그래프는 **기본 제어 토크**이며, 별도로 표시하는 최종 명령 여유에는 ILC가 포함됩니다. 고정 시험 설정에서의 추종 개선은 시험 조건의 결과이며 모든 궤적·게인·외란에서의 수렴을 보장하지 않습니다.

`src/robot_model/ModelBackend`는 MuJoCo 헤더와 분리된 기구학·동역학 인터페이스입니다. `control.cpp::ReadInput`이 CAN 또는 명시적 디버그 피드백의 `q/dq`로 백엔드를 갱신하고 결과를 `ModelData`에 발행합니다. 시뮬레이터는 런타임 모델 작성자가 아니며 제어기는 외부 `ModelData`와 시뮬레이션 GT를 계산 입력으로 쓰지 않습니다. 백엔드는 스레드 하나에서 사용하고 측정 상태, 목표 계산용 scratch, 중력 계산용 `mjData`를 분리합니다. 목표 IK를 계산해도 측정 상태는 변하지 않습니다.

IK는 이전 목표에서 시작하는 기존 반복 알고리즘을 유지하며, MuJoCo의 FK와 `mj_jacSite`를 이용합니다. `mj_inverse`는 역동역학 API이므로 위치 IK의 대체 함수로 사용하지 않습니다. 전체 `mj_forward` 대신 `mj_kinematics`/`mj_comPos`, 속도 단계, `mj_crb`/`mj_rne` 등 필요한 계산만 실행합니다. `Jdot*dq`는 `mj_jacDot`으로 구합니다. 호출의 선행 단계는 [MuJoCo 3.3.2 API](https://mujoco.readthedocs.io/en/3.3.2/APIreference/APIfunctions.html)를 따릅니다. 수동 감쇠·스프링·마찰·접촉력은 Feedforward에 포함하지 않으며 armature는 질량행렬 또는 RNE 결과에 한 번만 포함합니다.

측정 자세가 같으면 질량·중력·자코비안을 재사용합니다. 측정 `q/dq`가 모두 같으면 전체 모델 갱신도 재사용합니다. 고해상도 렌더 메시 제거는 명시적 inertial이 있고 `inertiafromgeom=true`가 아닌 몸체의 비접촉 시각 지오메트리에만 적용합니다. 자동 관성 추론에 필요한 메시는 남겨 원본 질량·관성을 보존합니다. 모델 경로·관절 순서·차원·유한값 검증에 실패하면 명시적으로 오류를 내며 다른 모델로 대체하지 않습니다. 모델·게인·물리 시뮬레이션 계산은 변경하지 않았습니다.

`inverse_dynamics` 모드에서 `alpha: 1`은 역동역학 전체 토크, `alpha: 0`은 기존 현재 자세 중력 보상이며 중간값은 둘 사이를 보간합니다. `mode: impedance`는 기존 중력+작업공간 임피던스를 사용합니다. ILC를 끄면 실행 중 관절 목표 IK/RNEA를 계산하지 않고, 켜면 IK로 학습용 관절 목표만 생성합니다. 사전 검사와 실제 자세 조건수 검사는 두 경우 모두 수행합니다. 기존 임피던스가 위치 피드백을 담당하므로 별도의 관절 위치 강성이나 RL residual은 추가하지 않았습니다. 계산 토크는 기존처럼 MIT **위치 surrogate**에 반영하며 패킷의 **tau_ff는 0**입니다. 목표 궤적은 관절 범위에 맞춰 자동 축소하거나 투영하지 않습니다. 기하학적으로 해가 있어도 목표 관절각이 물리 범위를 넘으면 GUI에 `q_d=OUTSIDE LIMITS`를 표시합니다. 실제 모델의 관절 범위와 기존 토크 한계는 유지합니다.

**두 모드에서 실행 중 IK 미수렴·수치 계산 실패 시** 궤적과 IDFF/CTC 동역학 출력을 중단하고 **실패 순간의 측정 관절각**을 고정합니다. 위치 성분을 캡처한 각도로 유지하고 토크 성분에는 중력만 넣습니다. 모터 내부 위치 P와 D가 자세를 유지하며 공통 모터 변환에서 합산 노력을 제한합니다. GUI에는 `IK / CONDITION BLOCKED - HOLD`, 제어 터미널에는 실패 원인·시각·목표를 표시합니다. SHM/MCAP의 `tracking_ik_failed`와 로거의 `TRACKING_IK_HOLD` 이벤트에도 기록합니다. 도달 가능한 목표로 돌아와도 자동 재개하지 않으며, **다른 모드로 전환한 뒤 T로 재진입하거나 다른 궤적 평면을 선택**하면 재시도합니다. 현재 평면 키를 다시 누르는 것만으로는 재시작하지 않습니다. 모델·설정 오류나 유효하지 않은 측정 상태는 계속 명시적 오류로 처리합니다.

3D 화면에는 최근 2초의 실제 발끝 경로를 가는 반투명 하늘색 잔상으로 표시합니다. 오래된 부분부터 옅어지고 리셋 시 지워집니다. 일시정지 중에는 잔상도 유지합니다.

GUI 오른쪽에는 선택 평면의 목표/실제 경로, XYZ 위치 비교, 위치 오차 그래프를 표시하고 상단에는 오차·토크 포화·선택한 IDFF/CTC의 ON/OFF·IK 오차를 표시합니다. `ControlData.tracking_position_reference`·`tracking_velocity_reference`·`tracking_torque_saturated` 및 요청/실제 평면(`requested_tracking_plane`·`tracking_plane`, XY=0/XZ=1/YZ=2)은 MCAP에도 기록됩니다. 목표 발끝 가속도·목표 관절 속도/가속도·역동역학 토크·활성화 여부·IK 오차도 기록하며, `MotorData.reference`는 모터 변환 전 위치 성분입니다. 실행 중인 토크 방식에서는 현재 측정각이며, 궤적 관절각은 TrackingController 내부의 IK 결과로 별도 유지합니다. 선택 방식(`tracking_method`: inverse_dynamics=0, CTC=1, impedance=2)과 포화 전 CTC 토크(`tracking_ctc_torque`)도 기록합니다. SHM ABI는 **22**이므로 모든 프로세스를 함께 재빌드·재시작해야 합니다.

Tracking 화면에는 **IDFF/CTC 토크 여유**도 표시합니다. HR/HP/KP별 `limit / now / |peak| / left Nm (%)`는 유효 제한·현재 선택한 방식의 raw 토크·최근 절댓값 피크·피크 기준 잔여 토크와 비율입니다. `left = limit - |peak|`이며 초과하면 음수와 `OVER`를 표시합니다. 유효 제한은 Tracking 설정과 MuJoCo actuator 양방향 제한 중 작은 값이고, CAN 실행에서는 가상 모터의 토크 제한도 포함합니다. 현재 기본 CAN 설정에서는 **53Nm**입니다. 비대칭 제한은 양방향 중 작은 절댓값으로 보수적으로 평가합니다.

통계는 기존 `plots.sample_hz`(기본 **100Hz**)로 관측한 최신 SHM 값을 `plots.history_sec`(기본 **10초**, 시뮬레이션 시간) 동안 보관한 결과입니다. 제어기의 모든 주기 피크를 수집한 것은 아닙니다. 모드 재진입·평면 변경·리셋 시 이전 실행의 피크를 제외하고, IDFF/CTC OFF/HOLD에서는 현재값을 `N/A`로 표시하면서 같은 실행의 시간창 안에 남은 과거 피크는 유지합니다. 네 번째 그래프는 선택한 IDFF 또는 CTC 토크의 부호 있는 제한 대비 사용률과 ±100% 기준선이며 제목·표에도 방식을 구분해 표시합니다.

Raw IDFF는 **alpha 적용·임피던스 합산·포화 전** 값이며, raw CTC는 **관절 피드백을 포함한 CTC 전체 토크의 포화 전** 값입니다. `Ctrl cmd left now`는 토크 성분(`MotorData.debug_torque`, 독립 토크 제한 적용 후; 위치 성분의 P 노력은 제외)에 대한 현재 여유를 HR/HP/KP 순서로 별도 표시합니다. 두 값 모두 설정된 출력 토크 예산과의 비교이며, 전류 센서 측정 토크나 실제 구동기의 연속 정격·열 여유를 의미하지 않습니다. 계측 자체는 제어 토크를 변경하지 않습니다.

Tracking도 기존 독립 monotonic 제어 주기를 사용합니다. 시뮬레이터 일시정지 중에도 궤적 시간은 진행합니다. 기본 호밍 위치에서 시험합니다. 사전 검사는 유한 표본의 IK 수렴과 조건수를 검사하며, 실행 중에도 위반이 생기면 자세 유지 상태로 전환합니다. 속도와 외력·토크 제한 때문에 물리적으로 즉시 정지하거나 위치 오차가 완전히 없어지는 것은 보장하지 않습니다.

Tracking 진입 시 `tracking.ik.trajectory_samples`(기본 **512구간**)로 한 바퀴 전체를 IK로 검사합니다. 각 목표 자세의 위치 야코비안 조건수 `σ_max/σ_min`이 공통 `workspace.maximum_condition_number`(기본 **10**)를 넘거나 IK가 수렴하지 않으면, 유효한 앞부분만 움직이지 않고 궤적 전체를 거부합니다. 관절각을 물리 범위 안으로 자동 투영하는 검사는 아닙니다.

검사는 매 제어 틱에 최대 **4점**씩 수행합니다. 검사 중에는 캡처한 진입 위치와 중력 보상 토크를 분리해 자세를 유지하고 IDFF/CTC/ILC를 출력하지 않습니다. GUI에는 `CHECKING TRAJECTORY - HOLD`와 진행률을 표시합니다. 기본 500Hz에서는 513점을 약 129틱, **0.258초**에 검사하며, 전체 통과 후 궤적 시계와 진입 램프를 시작합니다. 제어 루프는 검사 중에도 SHM·CAN 출력을 계속 갱신합니다.

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

[입력 설정](../config/input/input.yaml)에서 장치(`/dev/input/js0`)·축 번호·반전·중앙 데드밴드·트리거 원시 범위를 조정합니다. 기본 축 번호는 Linux Xbox 계열을 가정하므로 다른 장치는 확인 후 수정합니다. 입력 프로세스는 기본 daemon이 별도 터미널에서 실행합니다. 장치가 없거나 읽기에 실패해도 프로세스는 종료하지 않고 **1초마다 같은 장치에 재연결**합니다. 끊긴 동안 GUI에 **NO JOYSTICK**을 표시하고 축·버튼·트리거를 0으로 지웁니다. 장치를 다시 연결하면 데몬 재시작 없이 자동으로 입력을 받으며, 이전 연결의 눌림 상태를 재사용하지 않습니다. 설정 파일을 변경했을 때만 입력 프로세스를 재시작합니다. 실제 조이스틱이 없어도 GUI·다른 모드는 사용할 수 있습니다. 입력 이벤트가 없는 정상적인 정지 상태는 연결 끊김으로 판단하지 않습니다.

[제어 설정](../config/control/control.yaml)의 `ball_following`에서 공의 세 속도, 발끝 속도 상한, 관절 위치 P·MIT D와 작업공간 격자를 조절합니다. `smoothing_sec`는 FK 보간 목표를 따라가는 속도의 두 단계 1차 필터 시정수이며 기본 0.10초입니다. 크게 하면 출발·정지·방향 전환이 더 완만해지고 응답도 느려집니다. 실제 발끝의 추종 오차는 모터 게인과 물리 모델의 영향을 받습니다.

목표 관절각은 **FK 테이블의 인접 셀 보간**으로 생성합니다. 모델의 관절 범위를 FK로 샘플링하고 호밍과 같은 비특이 분기의 연결 작업공간을 구성하는 기존 방식은 유지합니다. 검증된 셀을 공유 면이 일치하는 사면체로 나누고, 시작할 때 FK 좌표의 보간 계수만 계산해 둡니다. 실행 중에는 찾은 사면체의 네 관절 자세를 합이 1인 비음수 가중치로 보간합니다. 반복식 IK·역야코비안·위치 오차 수렴 반복은 사용하지 않습니다. 격자점이 바뀔 때까지 기다리는 대신 셀 내부에서도 목표가 연속적으로 바뀝니다.

보간 결과와 이동 구간도 실제 FK로 확인합니다. 기존 **2cm 경계 여유 + 2mm 샘플링 여유**, 관절 한계, 분기, 조건수 상한을 유지합니다. 직선 연결이 검사에 실패하면 기존의 검증된 격자 연결 경로로 우회합니다. 두 단계 속도 필터를 적분해 출발·정지·방향 변경을 완만하게 하고, 다음 위치와 정지 예상 구간을 검사하여 제한에 접근하면 제동합니다. 안전 제한이 작동할 때는 평활화보다 제한 준수를 우선합니다. 보간 목표의 차분 속도는 위치 궤적 생성에만 사용하며 모터 속도 목표는 계속 0입니다. 모터 게인·토크 계산·SHM 구조는 변경하지 않습니다.

공은 범위 밖으로 자유롭게 움직이고 발끝만 가까운 유효 FK 지점까지 이동한 뒤 유지합니다. 보간은 FK 표의 근사이므로 격자 해상도에 따른 위치 오차가 남으며, 격자를 늘리면 오차가 줄지만 초기화 비용이 증가합니다. 보간 가능한 셀 밖에서는 기존의 유효 격자점 투영을 사용합니다. 진입 자세가 경계 여유 밖이면 현재 위치에서 유효 영역으로 이동하고 `ENTERING SAFE WORKSPACE`로 표시합니다. 반대 운동학 분기·특이 자세에서는 H로 돌아가도록 명시적 오류를 보고합니다.

모터 출력은 `torque_reference_mode`에 따라 위치 참조와 토크를 합칩니다. 공 위치·발끝 참조·최근접점·작업공간 제한·입력 연결/트리거 상태는 SHM/MCAP에 기록합니다. 선택적 외부 목표 점프는 `requested_ball_position`을 쓰고 `ball_target_sequence`를 증가시키며, 모드 재진입 시 이전 요청은 다시 적용하지 않습니다. SHM ABI는 **22**이므로 모든 프로그램을 함께 재빌드·재시작합니다.

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

## rmd_torque_ref

최상단 `torque_reference_mode: rmd_torque_ref`를 선택하면 Gravity·Magnetic·Tracking 실행·Damping은 계산된 토크를 RMD A1 전류 명령으로 보냅니다. 이 경로의 MIT `kp/kd/v_des`는 0이며 호스트 관절 PD를 추가하지 않습니다. 펌웨어 RMD 감쇠 레벨은 `rmd_damping_constant`로 설정합니다(기본 5, 정수 0..100). 기본 Homing·Ball·ZeroSet과 Tracking 사전 검사/실패 시 위치 유지는 기존 MIT를 사용합니다. `homing.torque_control: true`일 때는 호밍도 A1으로 전송하며 RMD 감쇠는 동일한 `rmd_damping_constant` 설정값을 유지합니다. 따라서 기본 위치 모드의 kp/kd는 그대로 적용되고, 호밍 토크 옵션의 kp/kd는 호스트 계산에 사용됩니다. 작업공간 PD, 역동역학, ILC, 코깅 보상 및 공통 토크 제한은 유지됩니다. 패킷 단위·좌표 연결은 [CAN HAL 문서](can_hal.md#rmd-torque-closed-loop-출력)를 참고하십시오.
