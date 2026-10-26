# CAN HAL 동작 규약

실행·GUI·설정 위치·SHM 작성 규칙은 [README](../README.md)를 기준으로 합니다. 이 문서는 패킷 처리, 계측 해석과 후속 영점 보정 결정을 관리합니다.

## 설정과 통신 경로

- [HAL 설정](../config/hal_config/can_hal.yaml): 인터페이스·실행/통계 주기·SHM·관절별 CAN ID와 모터 설정 경로.
- [모터 규약](../config/motor_config/ak10_9_v3.yaml): V_MAX/T_MAX·Kt_out·감속비·Kd scale. `motor_config` 경로는 HAL YAML 디렉터리 기준입니다.
- HAL은 `can_hal HAL_CONFIG.yaml`, 가상 모터는 `mujoco_simulate --hal-config HAL_CONFIG.yaml`로 같은 설정을 받습니다. 각 생성자가 직접 읽고, 누락·불일치는 명시적으로 실패합니다.

모터 명령·피드백은 Linux SocketCAN의 `PF_CAN/SOCK_RAW/CAN_RAW`를 통과합니다. 기본 런처는 호스트 vcan0를 준비하며 네임스페이스 생성은 명시적 `--isolated-vcan` 디버깅에서만 수행합니다. daemon은 소켓·SHM을 해석하지 않고 프로세스와 출력 창의 수명만 관리합니다.

프로토콜 구현 기준은 [OpenRobot MIT v17](https://dongilc.github.io/openrobot-motor-tool/)이며 [AK10-9 V3.0 KV60](https://www.cubemars.com/product/ak10-9-v3-0-kv60-robotic-actuator.html) 프로파일을 사용합니다. CubeMars 기본 펌웨어와의 호환과 실물 드라이버 동작은 검증 전입니다.

## 패킷과 상태 전이

| 명령 | HAL / 가상 모터 동작 |
| --- | --- |
| C4 | V_MAX/T_MAX·Kt_out·감속비·Kd scale 조회·일치 확인 |
| C1 | 프로파일 검증 후 enable 요청·확인 |
| C0 | MIT 목표 송신, 상대각·속도·Iq·온도 응답 |
| C2 | release. 이후 C0 송신 중단, 종료 시에도 송신 |

ID 1/2/3은 SID 0x141/0x142/0x143에 대응합니다. 아직 검증되지 않은 C4와 미확인 C1은 100 HAL 주기마다 재요청합니다. 이는 준비 확인용 반복이며 응답 지연 감시가 아닙니다. 모터 응답이 없어도 제어 루프는 설정 주기로 계속 동작합니다.

`PositionTarget`은 enabled·position·velocity·kp·kd만 가집니다. position은 출력축 상대 좌표 rad입니다. Homing은 보간 q/dq와 자체 PD 게인을 보내며, Gravity/Magnetic은 계산한 토크를 다음 위치 목표로 환산합니다.

```text
q_motor_target = q_joint - position_offset + (tau + kd_motor * dq) / kp_motor
```

kd 항은 이미 토크 계산에 포함한 감쇠를 모터 PD에서 중복 적용하지 않기 위한 항입니다. 가상 모터의 내부 PD 출력은 모터 프로파일 토크 한계와 XML actuator 한계의 교집합으로 제한합니다.

**tau_ff는 사용하지 않습니다.** 송신 바이트는 논리적 0인 0x80으로 고정합니다. 가상 모터는 이 값만 허용하고 FF 없이 PD를 계산합니다. 실물 드라이버가 대칭 8-bit 필드를 수치로 복원하면 반 LSB 오차가 생길 수 있어 zero-code 해석은 실기 캡처로 확인해야 합니다.

피드백은 v14 이후 signed 출력축 MIT 좌표로 해석합니다. 구형 입력축 표를 자동 감지하거나 대체하지 않습니다. 위치는 int16 × 12.56 / 32768 rad, 속도는 int16 deg/s를 rad/s로 변환하며, Iq는 int16 × 33 / 2048 A입니다. 가상 모터는 직전 물리 스텝의 실제 actuator torque / Kt로 Iq를 구성합니다.

수동 명령은 TaskController를 실행하지 않는 별도 디버깅 세션에서만 사용합니다. 기본 세션에서는 TaskController가 명령을 계속 발행합니다.

```bash
./build/can_hal --target /qhrr05 0 0.2 0.0 40.0 0.3
./build/can_hal --release /qhrr05
```

## SHM과 계측 해석

`MotorData`에는 축별 배열과 공통 통계를 직접 둡니다. command/feedback 발행 번호는 독립적이며 `received_sequence[i]`는 축별 응답 번호입니다. HAL은 잠금 아래 최신 슬롯의 피드백·CAN 배열·공통 통계만 갱신해 제어기 명령을 보존합니다.

모델 M/h/J·중력·발끝 정보는 CAN으로 측정한 관절 상태에서 별도 scratch `mjData`로 계산합니다. TaskController의 q/dq/토크와 차분 가속도도 CAN 피드백에서 얻습니다. 물체 위치·접촉/해제 이벤트는 시뮬레이션 SHM을 사용하며 GT 힘은 RFOB 검증용입니다.

```bash
./build/shm_logger --can /qhrr05 /tmp/can-telemetry-new.csv
```

- CAN CSV의 age·관측 간격·SHM 발행 누락 평가는 로거에서만 수행합니다. 미수신 축의 age/interval은 빈 칸입니다.
- 최신 목표 sequence와 HAL이 소비한 목표 sequence는 다를 수 있습니다. 패킷에 sequence가 없어 요청별 왕복 지연·전체 CAN 손실·정확한 적용 시점을 식별하지 않습니다.
- 일반 CSV는 simulation/control/motors의 최신 완성본을 함께 읽습니다. `state_seq`는 소비한 모델 프레임 번호이고 `control_latency_us`는 모델 발행에서 명령 발행까지의 시간입니다.
- `sim_contact_event_time`은 시뮬레이션 시간, RFOB `event_time`은 독립 제어 시간입니다.

HAL이 소유한 SHM은 정상 종료 때 해제합니다. SIGKILL 뒤 남은 객체나 크기가 다른 객체는 자동 재사용·삭제하지 않고 충돌로 보고합니다. SHM 배치를 변경하면 모든 프로그램을 새 바이너리로 재시작합니다.

## 스토퍼 영점 보정 — 미구현

아래는 **미구현 설정안**입니다. 관절마다 한쪽 기준 스토퍼를 `lower` 또는 `upper`로 선택하고, 알려진 하드웨어 기준각을 설정합니다. 순서도 설정으로 변경합니다. 예시 각도는 현재 XML의 하한에서 가져온 **시뮬레이션 초기값**이며 실제 기구 측정값이 아닙니다.

```yaml
homing:
  calibration:
    order: [KP, HP, HR]
    joints:
      HR: {stop_side: lower, stop_angle_rad: -1.0472}
      HP: {stop_side: lower, stop_angle_rad: -1.5708}
      KP: {stop_side: lower, stop_angle_rad: -2.7227}
```

TaskController가 런타임 보정값을 소유합니다. 방향 `s=±1`, 상대 엔코더값 `r`, 접촉 시 측정값 `r_stop`, 알려진 스토퍼 각도 `q_stop`에 대해 다음 관계를 사용합니다.

```text
offset   = q_stop - s * r_stop
q_joint  = s * r + offset
r_target = s * (q_target - offset)
```

낮은 kp/kd·제한된 위치 오차·느린 위치 목표로 접근하고, 새 피드백에서 정지를 확인한 뒤 후퇴/재접근 재현성을 검사합니다. 보정 후 스토퍼에서 물러나 일반 자세 호밍으로 이어갑니다. tau_ff로 누르거나 절대 엔코더·C3 원점 재설정에 의존하지 않습니다. 이전 세션 offset을 자동 재사용하지 않으며, 알려진 원점·세션 변경 시 재호밍합니다.

오래된 피드백을 반복해서 읽은 것만으로 접촉을 확정하지 않습니다. 센서리스 정지는 장애물·마찰·중력 평형과 구별되지 않을 수 있으므로 실제 기준각과 비어 있는 탐색 경로가 필요합니다. 거리·시간 한계와 재현성 실패는 호밍 실패로 처리하며, CAN 응답 지연 감시를 추가하지 않습니다.
