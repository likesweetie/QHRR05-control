# 작업 요약

갱신: 2026-10-29 · 브랜치: `refactoring` · 기준 커밋: `2f27d0e` (main 병합·원격 반영 완료)

## 마찰 보상 스위치·공통 출력 토크 제한 — 2026-10-29

- `gravity_compensation.friction_compensation.enabled`를 추가했습니다(현재 true). false는 건마찰과 모델 점성감쇠 보상을 함께 끄며 중력/bias·자세 강성·motor kd를 유지합니다. 보상 비율·속도 파라미터는 저장값을 유지하고, bool 누락/오류는 시작 오류입니다.
- 각 모션의 최종 토크 제한을 최상위 `limits.torque_nm: 60.7`로 통일했습니다. 현재 YAML의 최대 최종 출력값 60.7을 모든 관절에 적용하며, 스칼라/3개 배열을 지원합니다. 공통 파서는 src/control/limits.hpp입니다. 이전 모션별 torque_limit_nm/output_torque_limit_nm은 거부합니다.
- Homing/Gravity/Magnetic/Tracking/Ball, TaskController 최종 토크와 MIT 목표, Classic/Filament 그래프가 공통 제한을 읽습니다. Homing/Damping도 제한된 토크를 실제 MIT 목표로 표현하도록 변환했습니다. 물리 MJCF/CAN의 제한, ILC 보정 자체 한도·주기별 갱신 한도는 역할이 달라 유지합니다.
- runtime의 다른 사용자 튜닝(damping_ratio1, coulomb.95, kd.01, 추종 파라미터 등)은 유지했습니다. native 전체/Filament 빌드, 마찰 ON/OFF·공통 한도·실제 MIT 변환·오류 거부와 관련 모션 7개 시험 통과. Filament 정적 미리보기의 새 설정 로딩도 통과했습니다.
- 가상 CAN IDFF/CTC/Impedance 3/3 통과(84.60s), 총 관련 회귀 10개 통과. 별도 공통 설정 유효성 재검증도 통과했습니다. 로그: /tmp/qhrr-shared-limits-tests.log, /tmp/qhrr-shared-limits-can.log. diff/서식 검사 통과, 변경은 refactoring 브랜치에 미커밋 상태입니다.

## 가상 관성 제어 제거 — 2026-10-29

- 사용자 요청으로 가상 관성 보조 토크, 외력 추정 필터, 전용 상태/피드백/측정 dt를 제거했습니다. Gravity의 입력은 측정 상태 기반 RobotModelState만 사용하며 중력/bias, 자세 강성, 마찰·감쇠 보상과 최종 토크 제한을 유지합니다.
- 현재 사용자가 조정한 damping_ratio=[1,1,1], coulomb_ratio=[.95,.95,.95]를 보존했습니다. YAML의 virtual_inertia 섹션과 활성 문서·전용 시험은 제거했습니다. 구 설정에 해당 항목이 남아 있으면 제거 안내 오류를 내며 조용히 무시하지 않습니다.
- 이전 가상 관성 구현 기록은 아래 이력으로만 남아 있으며 현재 기능이 아닙니다. 렌더링 재질 작업은 그대로 보존했습니다.
- control/simulation 및 관련 시험 빌드 완료. safety_layer/control_io/motion_controller_config/qhrr05_gravity 4/4 통과(16.67s), diff 검사 통과. 실행 중 사용자 세션은 변경하지 않았으며 재시작부터 적용됩니다.

## 재질 기준값·거칠기 맵 재보정 — 2026-10-29

- 사용자 첨부 진단을 기준으로 실제 맵을 확인했습니다. 이전 roughness 맵 평균은 .891로, 은색 유효값은 .232였습니다. OBJ에 authored UV가 없어 texrepeat25는 0~1 UV에서 25타일이라는 의미가 아니었습니다. C++ 재질 재할당과 배경/IBL 분리는 이미 적용 중입니다.
- 동일 GPU/카메라로 기존, 무텍스처 상수, 약한 roughness, repeat1, 사진 IBL을 비교했습니다. repeat1만의 차이는 작은 반면 사진 IBL은 넓은 반사 띠를 강화하여 기본 환경은 기존 analytic studio IBL로 유지했습니다. 비교는 `artifacts/material-calibration/`에 보관합니다.
- 은색/몸체의 색상 브러시 무늬를 제거하고 상수 RGB .78/.80/.82와 .48/.50/.53으로 변경했습니다. 별도 4K roughness 맵에 유효값을 직접 저장(계수1), 평균 .348834/.380253, 범위 .322~.380/.357~.404입니다. texrepeat10(200mm projected tile), max anisotropy.20. 검정 하우징은 별도 무텍스처 재질(RGB.12/.13/.14, metallic.85, roughness.42)로 등방 반사합니다. 흰 커버는 도장 재질, 바닥 거칠기는 고정.90입니다. 미사용 텍스처 선언을 제거했습니다.
- 생성 도구/메타데이터/문서를 갱신했습니다. 새 shader 빌드, 실제 RTX5090에서 MSAA1/4/8x·2xSSAA·홀수폭 캡처·잘못된 옵션 거부 시험 통과. 활성 texture 파일 로딩/해시 검증 통과, GPU 등록 texture13→9. 최종 이미지 `artifacts/material-calibration/final.png`, 비교 `before-after.png`. 물리/제어/모델 형상과 기존 배경·광량은 변경하지 않았습니다. refactoring 브랜치의 미커밋 작업입니다.

## 통합 커밋·main 병합 준비 — 2026-10-29

- 사용자 요청으로 현재 GUI/IPC/렌더러/모델/수치 연산/중력보상 변경을 함께 커밋하고 main 병합·원격 푸시 후 refactoring 브랜치를 생성합니다. 로그·빌드·캐시·원본 전달 ZIP은 제외하며, CAD 메타데이터/URDF의 원본 CRLF 바이트는 .gitattributes로 보존합니다.
- 전체 CTest 41종을 실행하고 실패 항목을 확인했습니다. 분석 Python 환경은 문서의 uv 방식으로 build/analysis_env에 복원했고, renderer --render-scale 옵션이 뒤의 두 인수를 버리던 shift 4→2 오류를 수정했습니다. 로그 분석/자동 빌드 재검증 통과, 웹 실제 브라우저 포함 3/3 및 실제 RTX5090 품질 시험 통과입니다.
- daemon SHM smoke는 이제 controller model counter(state_seq)가 아닌 physics counter(sample_seq)와 command_seq를 비교합니다. 신규 CAD의 지연 enable 시험은 특정 HR 축 절대값 대신 시작 관절각 대비 실제 최대 변위를 검사합니다. 두 검증 재실행 통과했습니다. Native GUI 슬라이더 시험은 물리 접촉을 잠시 고정해 자연 상태 전이를 GUI 재시작으로 오인하지 않게 했으며 이 부분은 통과했습니다.
- **미통과 2건을 숨기지 않습니다:** Tracking XZ IDFF+ILC 개선율 약3.75%(기준10%)는 앞서 변경 전 구현에서도 재현한 기존 항목입니다. Native Ctrl-drag는 현재 모델에서 관절 이동량 약0.0105rad로 시험 기준0.03rad에 못 미칩니다. 좌표 투영/긴 드래그의 진단에서도 최대 약0.0188rad였고, 시험 기준은 낮추지 않았습니다. 해당 진단용 변경은 제외했습니다. 전체 항목 중 재검증 포함39/41 통과이며, 이 두 건은 후속 조사 대상으로 남깁니다.

## Gravity 마찰 보상·선택적 가상 관성 — 이전 구현 이력 (가상 관성은 이후 제거)

- 사용자 최종 요청대로 모델 감쇠의 50%를 G 제어기에서 보상합니다. `gravity_compensation.damping_ratio=[.5,.5,.5]`는 남기는 모델 감쇠 비율이며 임계감쇠비 ζ가 아닙니다. 모델 D=2.0은 그대로, 잔여 D=1.0에 기존 motor kd=.05가 추가됩니다. 건마찰은 85%만 tanh(dq/.03)로 부드럽게 보상합니다. 조절은 control.yaml 편집 후 제어기 재시작입니다.
- ModelBackend는 모델 damping을 제공하고 Gravity는 측정 상태의 RobotModelState를 받습니다. 물리 MJCF, SHM ABI와 다른 모션 제어식은 바꾸지 않았습니다. 모델 passive 전체를 뒤집지 않으므로 스프링까지 임의로 보상하지 않습니다.
- `virtual_inertia.enabled=false`가 기본입니다. 활성화 시 외력 토크 LPF(M*ddq+bias-passive+smooth friction-measured torque)에 (1/scale-1)을 곱해 보조합니다. scale=.5, 8Hz, deadband=.15Nm, 진입ramp=.3s, 보조±3Nm/총±53.7Nm, 속도2rad/s에서 진행 방향 보조0입니다. 필터는 새 측정의 실제 dt로만 갱신하고 재진입 때 초기화합니다. 명시적 설정 검증, 잘못된 활성 측정은 오류이며 대체 제어로 숨기지 않습니다. 위치 어드미턴스/절대 가상질량이 아닌 관절 관성 비율 제어이고 실기 검증 전 OFF 유지입니다.
- 전체 native 빌드와 Filament viewer/model-test 빌드 완료. MuJoCo3.3.2/3.16 백엔드 일치 검사, G 순수중력/bias/50%감쇠/건마찰/동일모델 밀기 응답, 가상 관성 .5의 가속 반응/포화/노이즈/중복샘플/재진입/비정상 입력, control.cpp의 MIT 전달 경로를 검사했습니다. 추가 실제 control 프로세스 시험에서 관성 ON의 측정 토크 기반 보조도 통과했습니다. 실행 중 사용자 세션은 건드리지 않았습니다.
- 중단 후 재검증 완료: 관련 10개 CTest 전부 통과, 별도 가상 CAN IDFF/CTC/Impedance 3개 전부 통과(84.23s), 추가 활성 관성 control_io 과정도 통과했습니다. 로그는 `/tmp/qhrr-gravity-resume-tests.log`, `/tmp/qhrr-gravity-resume-can.log`, `/tmp/qhrr-gravity-inertia-process.log`입니다. 서식/diff 검사 통과, 커밋·푸시는 하지 않았습니다.

## 자연스러운 회색 배경·조명 재조정 — 2026-10-29

- 흰색으로 뜨던 바닥을 중간 회색 sRGB 135/136/137로, skybox를 0.29/0.30/0.31로 조정했습니다. 주광 30000→22000, IBL 48000→40000, rim 1800→1500이며 fill 3000·노출 0 EV는 유지했습니다. 기존 금속 재질·4K 텍스처·IBL 파일·물리·제어에는 변경이 없습니다.
- 실제 RTX5090/Vulkan에서 동일 카메라 3안과 최종 1920×1080을 비교했습니다. 최종 기본 8x MSAA/2560×1440 내부 렌더 확인, 별도 품질 검사 1/4/8x MSAA·2x SSAA·홀수폭 캡처·잘못된 설정 거부 모두 통과했습니다. `artifacts/lighting-study/final.png`, `before-after.png`에 결과를 보관했습니다. 사용자 실행 세션은 건드리지 않았으며 새 창부터 적용됩니다.

## 금속 반사·4K PBR 보강 — 2026-10-29

- 이전 줄무늬 완화 설정은 IBL이 지나치게 균일하고 roughness가 높아 무광 도장처럼 보였습니다. 단순히 해상도를 올리는 대신 금속 BRDF/반사 대비/방향성을 함께 조정했습니다.
- ambientCG Metal009의 4K PNG 원본(202,436,424 bytes, CC0)을 공식 링크에서 받았습니다. zip SHA256 `2e740afb0818258d177a71280076a9a18903e23fff5e2b7563322f6c0e99c6e4`. 원본 zip/미사용 NormalGL은 Git 제외 `.cache/materials`에 두고, 중립 albedo/좁은 roughness로 재매핑한 4096×4096 PNG 2장(~5.5MB)을 runtime 자산으로 포함했습니다. `tools/prepare_metal_materials.py --download`로 검증/재생성, 메타데이터는 `textures/brushed_metal.json`.
- 금속 shader `visualization/materials/brushed_pbr.mat` 추가: upstream PBR 입력 채널/Apache 헤더 유지, UV/위치 미분으로 브러시 접선 방향을 구해 이방성 반사(최대.38), specular AA 적용. UV퇴화면은 정의된 방향이 없어 등방성으로 계산합니다. `qhrr_pbr.filamat`를 별도로 빌드하고 렌더러 provider가 명시적으로 사용합니다. 없으면 자동 복구 빌드 또는 명시적 오류이며 기존 shader로 몰래 대체하지 않습니다. pinned MuJoCo/Filament 소스는 변경하지 않았습니다.
- 타일80mm/texrepeat25, 은색metallic1·roughness유효약.23, 그래파이트metallic.95·roughness약.28, 검정metallic.9·roughness약.26. Normal/Displacement 맵으로 커다란 요철을 만들지 않습니다. 물리·제어·CAD 버텍스/법선 그대로입니다.
- IBL은 512px cube로 보강하고 경계가 부드러운 3개 직사각형 소프트박스+어두운 반사 영역으로 재구성했습니다. 주광 방향/보조광도 조정해 곡면 하이라이트와 가장자리 금속 반사를 드러냅니다. 기존 AA8x+고정4/3 해상도는 유지.
- 실제 RTX5090/Vulkan에서 4K맵/새shader/서로다른카메라각도 확인. AA1/4/8x+2배SSAA·홀수폭readback·옵션오류거부 통과. customshader누락복구/소스구조시험 통과. GPU 200ms지연 전후 physics ratio0.99917→0.99983, controller460.03→459.50Hz로 독립성 유지. 최종이미지 `artifacts/metal-material-study/final-metal.png`, 각도별/이전설정 비교및logs 같은폴더. 빌드완료, 사용자세션에연결하지않았고커밋/푸시없음.

## 은색 알루미늄 표면 반사 정리 — 2026-10-29

- 사용자 지적의 넓은 줄무늬를 동일 카메라/모델 A/B로 분리했습니다. Normal Map 제거 시 은색 ROI 평균 RGB 차이0.51/255, 은색 모든 텍스처 제거0.64/255, IBL 제거65.2/255로 고대비 Studio Small09 반사가 주원인이었습니다. `artifacts/silver-finish-diagnosis/`에 원본/비교/최종 이미지를 보관합니다.
- 새 `tools/prepare_product_studio_ibl.py`는 넓고 부드러운 3개 반사 카드를 수학적으로 만들고 고정 Filament cmgen으로256px GGX/SH KTX를 생성합니다. 네트워크는 사용하지 않습니다. `product_studio_ibl.ktx`가 새 기본이며 이전 CC0 환경은 명시적 --ibl 선택용으로 남겼습니다. 입력HDR/출력KTX 해시는 JSON기록에 있습니다.
- 은색 pbr_metal은 RGB[.67,.69,.71], metallic .96, roughness .44 × 전용 aluminum_roughness 맵(평균247/255) = 약.426입니다. 은색 Normal Map은 제거하고, 별도의 낮은 진폭 비주기 roughness를 넣었습니다. texuniform=false, texrepeat320으로 bbox크기 중복 적용을 없앴고 ProjectUv 0.5*p 기준6.25mm타일로 고정했습니다. MuJoCo는 RGBtexture가 있어야UVscale을 설정하므로 단색 RGBtexture는 유지합니다. 원본 CAD/메시 법선/물리/제어는 그대로입니다.
- 기본IBL의 낮은 주파수 반사에 맞춰 environment intensity48000으로 조절했습니다. 초기화/실행은 새KTX의존성을 명시적으로 확인하며 대체환경 fallback은 없습니다. 이전 HDRI도 기본이 아닌선택자산으로 보존했습니다.
- 고해상도원본/같은크기 비교 `final.png`, `final-comparison.png`, 확대 `final-comparison-detail.png`. 실제RTX5090/Vulkan에서1x/4x/8xAA·2xSSAA·홀수폭RGB저장 및 잘못된옵션거부 시험, 런처자동빌드/소스구조검사 통과. 빌드완료, 기존실행세션은건드리지않았습니다. 커밋/푸시없음.

## 모듈 구조 정리 — 2026-10-29

- 사용자 요청대로 `src/numerics`를 `src/motion_controllers/numerics`로 옮겼습니다. 별도 공개 numerics 라이브러리는 제거하고 모션 컨트롤러 소스로만 빌드합니다. 유일한 외부 사용자였던 `robot_model`은 public 타입을 `std::array`로, 두 행렬 곱을 MuJoCo `mju_mulMatVec`로 바꿔 수치 모듈 의존성을 제거했습니다. 모델 상태·수치 결과 비교 검증은 통과했습니다.
- Input/조이스틱과 웹 상태 브리지는 `src/interfaces/{input,web_gui}` 및 하나의 `interfaces` 라이브러리로 묶었습니다. 실행 진입점은 `apps/interfaces/{input,web_gui}`, Node/React는 `apps/interfaces/web_gui/frontend`입니다. 기존 `input`, `web_gui_bridge` 실행 파일과 런처 명령은 유지합니다. npm 경로·테스트·서버의 저장소 기준 경로·gitignore를 새 위치로 갱신했습니다.
- 시뮬레이션 전용 IPC는 `src/ipc/simulation`으로 옮기고 타깃을 `ipc_simulation`으로 명명했습니다. 와이어 프로토콜·소켓 이름·SHM ABI13·명령 승인 경로는 그대로입니다. 예전 소스 경로를 대신하는 링크/래퍼는 남기지 않았습니다. 기존 사용자 세션은 건드리지 않았습니다.
- 모션 수치 시험(`motion_numerics`)과 모델 백엔드 시험(`robot_model`)을 분리했습니다. `source_layout`은 모션 외부의 numerics 참조와 예전 경로 재생성을 검사합니다. 시각화에서도 모델과 모션 컨트롤러 라이브러리를 나누었습니다.
- 검증: native 전체 빌드, React 빌드, 시각화 3개 타깃 빌드, 핵심 CTest10개 + 제어기 단독/가상 CAN2개, 웹 통합2개 + 실제 브라우저1개 모두 통과. MuJoCo3.16 독립 모델 수치 시험 및 RTX5090/Vulkan 프리뷰 통과. 로그 `artifacts/module-layout/`. 이번 구조 정리에서 모델·게인·렌더링/제어 알고리즘을 변경하지 않았고 커밋/푸시는 하지 않았습니다.

## 밝은 스튜디오·안티앨리어싱 개선 — 2026-10-29

- 사용자 요청에 따라 Filament 기본 품질을 8x MSAA + 고정 4/3배 SSAA로 변경했습니다. `--msaa 1|2|4|8`, `--render-scale 1..2`를 새 세션/attach/preview에 지원하며 잘못된 값이나 GPU 미지원은 명시적 오류입니다. 자동 해상도 하향, FXAA/TAA, sharpen은 없고 16F HDR/custom resolve를 사용합니다. Classic은 4x 기본이며 기존 OpenGL 개발 런처와 물리 모델은 유지합니다.
- mjrf의 render-target 경로가 MSAA를 꺼서 저장 화면이 거칠어지던 문제를 발견했습니다. 저장 시에만 읽기 가능한 headless swapchain과 같은 SceneView를 쓰도록 바꿔 UI를 포함한 동일 AA/SSAA를 보존합니다. 고정 MuJoCo 3.16의 색보정 LUT가 이전 설정으로 만들어지는 문제는 현재 설정 재적용으로 보정했습니다. 내부 API 사용은 문서화했고 upstream 소스는 변경하지 않았습니다.
- 밝은 중립 배경/무광 바닥, 그래파이트 몸체·검정 금속 힙·새틴 알루미늄 허벅지, 약한 Normal/Roughness 변화를 적용했습니다. 기존 CC0 Studio Small 09 IBL을 재사용하고 key/fill/rim 스튜디오 조명, ACES·PCSS·AO를 사용합니다. headlight/bloom/vignette/fog는 끕니다. 새 bevel·CAD 재분할은 추가하지 않았습니다.
- 검증 완료: 실제 RTX 5090/Vulkan의 1x/4x/8x + 2배 SSAA, 홀수 폭 RGB readback, 잘못된 설정 거부, Classic 4x 출력, 자동 빌드, 기존 그래프/단축키/Ctrl force·rotation/mocap/작업공간/스크린샷/독립 종료. 1080p viewport→2560×1440 및 3840×2160 실제 내부 해상도 로그 확인. `tests/visualization_quality_test.py` 추가.
- `tests/visualization_operator_test.py`의 이전 CAD 기준 near-foot 고정 좌표는 상자에 가려져 잘못 선택됐으므로 테스트의 Projection을 실제 ray hit 확인으로 개선했습니다. 재개 시 perturb가 초기화되는 안전 동작을 고려해 재개 후 최신 위치를 다시 투영합니다. `tests/visualization_gpu_test.py`는 ABI13 이후 모델 발행 주기가 제어기 약500Hz임을 검사하도록 갱신했습니다. 제품 조작/안전 정책은 그대로입니다.
- 렌더 200ms 지연 전후 physics ratio 0.99982→0.99983, 제어 469.83→470.66Hz, 최대 SHM 잠금18.8μs. 최종 이미지 `artifacts/studio-quality/studio-1080p.png`; 비교 before/after 및 시험 로그도 같은 디렉터리입니다. 빌드 완료, 사용자 기존 세션에는 연결/변경하지 않았습니다. 커밋/푸시하지 않았습니다.

## 수치 연산·실측 상태 백엔드 리팩토링 — 2026-10-29

- `src/motion_controllers/numerics`에 3×3 실제 역행렬, 고유값 전용 특이값 계산, adaptive DLS/Cholesky, 전치 외력 변환, 벡터/삼각형 거리, 조화급수 평가를 모았습니다. 정상 조건은 실제 역행렬, 불량 조건은 명시적인 DLS 정책을 사용하며 실패 시 다른 알고리즘으로 숨겨 전환하지 않습니다. 동일 J의 속도/가속도 분해와 RFOB 분해를 재사용합니다.
- `src/robot_model`에 MuJoCo 헤더와 분리된 인터페이스를 도입했습니다. TaskController가 한 백엔드를 소유하고 Tracking/Ball에 주입합니다. `control.cpp`의 수신 q/dq로 M, J, bias, 중력, 수동력을 갱신하고 SHM ModelData를 발행합니다. 시뮬레이터는 모델 작성자가 아니며 실하드웨어에서도 시뮬레이터 없이 계산합니다. 목표 계산용/실측용/중력용 데이터는 분리했습니다. SHM 작성자 의미 변경으로 ABI **13**이며 모든 바이너리 재빌드 완료, 사용자의 기존 세션은 건드리지 않았습니다.
- IK 반복의 전체 `mj_forward`를 FK/J 필수 단계로 줄이고 CTC는 캐시된 M/bias를 재사용합니다. q가 같으면 자세 항, q/dq가 같으면 모델 갱신을 재사용합니다. 명시적 inertial이 있고 inertiafromgeom=true가 아닌 몸체의 비접촉 시각 메시만 제거하며, 물리 모델·게인·토크/조건수 안전 한계는 유지합니다. Ball A* 작업 버퍼 재사용, 궤적 조화급수 recurrence와 중복 평가 제거도 적용했습니다. Eigen 추가 없이 고정 크기 연산으로 구현했습니다.
- 검증: 수치/백엔드 200 자세 대조, rank-deficient DLS, 스펙트럼/전치/조화급수, 자동 관성 추론 보존, 캐시·scratch 격리 통과. MuJoCo **3.3.2와 3.16.0 모두** 동일 검증 통과. 9개 기본 회귀와 제어기 단독/가상 CAN IDFF·CTC·임피던스 4개 통과. CAN에서는 GT/외부 ModelData 오염 무시, 실제 특이 측정 자세 HOLD, 복귀, 접촉력 시험까지 확인했습니다. 새 CAD 발끝이 기존 접촉 박스에 겹치는 테스트 조건은 자유공간 시험 중 박스를 치우고 접촉 시험에서 명시적으로 배치하도록 수정했습니다. 실제 RTX 5090 Vulkan 프리뷰 3프레임 출력 통과.
- Release 참고값(단일 호스트 microbenchmark, 주기 보장 아님): 3.3.2 전체 mj_forward 약31.5 μs, 백엔드 M/J/bias/중력/수동력 갱신 약0.74 μs, IK FK/J 약0.24 μs. 계산용 모델 로드 약0.85 ms, 렌더 메시 포함 원본 약4.54 s. 3.16.0은 전체 forward 약2.93 μs, 백엔드 약0.97 μs. 3.3.2 Jacobian 분해+2회 풀이 약0.34 μs (고유벡터+반복 DLS 대조 약0.35 μs). Debug에서는 새 소스만 비최적화라 역행렬 벤치가 더 느릴 수 있습니다.
- **남은 기존 실패:** 원본 `tracking_controller` 회귀의 XZ IDFF+ILC 10% 개선 기준이 실패합니다. 변경 전 HEAD 추종 구현을 현재 CAD 모델로 별도 빌드한 결과와 이번 구현 모두 baseline 11.6697 mm → ILC 11.2324 mm(약3.75%)로 동일합니다. 저장소의 10% 합격 기준·제어 게인은 낮추지 않았습니다. /tmp의 별도 진단 실행에서 개선 여부만 확인해 이후 YZ/CTC/특이점/재시작 검증도 끝까지 통과했지만, 이를 원본 전체 회귀 통과로 집계하지 않습니다.
- 근거 로그는 `artifacts/numerics-refactor/`에 보관합니다. 문서 `docs/control.md`와 SHM 설명 갱신. 이번 작업 커밋/푸시는 하지 않았습니다.

## 최신 모델 교체 — qhrr-05_new.zip

- 기존 `models/qhrr-05`와 XML 진입점, FL body/joint/actuator/foot/site 및 재질 이름을 유지하면서 새 CAD 메시 4개(1,080,012 삼각형), 링크 치수·원점·질량·COM·full inertia·관절 범위·충돌 형상을 반영했습니다. 기존 OBJ 9개는 압축과 LF 정규화 후 동일하여 파일을 변경하지 않았고 새 MJCF에서는 사용하지 않습니다. 새 OBJ는 원본 해시와 바이트가 일치합니다.
- 제공 MJCF의 `hip/thigh/calf`만 기존 `FL_hip/FL_thigh/FL_calf`로 맞췄습니다. 재질 `qhrr_metal`은 기존 `gray`에 연결했습니다. 모든 로봇 관성은 명시값 그대로이며 `inertiafromgeom=auto`로 기존 장면의 mocap 물체 계산을 유지했습니다. 원본 URDF·질량/CAD 근거·미리보기와 갱신한 provenance를 보관했습니다. `scene.xml`·PBR 장면·제어 게인·계산식은 변경하지 않았습니다.
- 고해상도 메시의 초기 로딩을 위해 시뮬레이션 시작 대기만 20초로 설정했습니다. 물리 timestep은 1 ms 그대로입니다. 자유 공간 호밍 검증은 기존 장면의 접촉 물체를 포함하지 않는 로봇 XML로 수행합니다. 시작 가드 시험은 동일한 새 관성·충돌 형상을 쓰되 시각 메시만 제외해 로딩 시간을 잘못된 제어/종료 오류로 오인하지 않도록 했습니다.
- 모델/장면 2,000스텝·호밍·중력보상·외란·시작 가드 6개 시험 통과. 실제 격리 vcan에서도 새 모델 시작→Input/안전레이어를 통한 호밍 완료→중력보상 전환을 확인했습니다. Classic/PBR 모두 실제 GPU 출력 성공. 최종 화면 `artifacts/model-replacement/new-model-pbr.png`, 교체 전 백업 `artifacts/model-replacement/20261029-211843/original`에 있습니다.

## 웹 GUI 개편·자동 빌드·시작 오류 수정

- `docs/flip7-card-game-DESIGN.md`를 참고해 웹 GUI를 청록/크림/금색 카드 기반으로 개편했습니다. 현재 모드·연결·접촉을 크게 표시하고 명령 처리 단계를 조작부 위에 배치했습니다. ±0.5 N 입력, 미적용 값·범위 오류, 실제 키보드 단축키, 모바일 관절 카드, 세로축 범위가 있는 그래프를 추가했습니다. Input/안전계층·통신·제어 모델은 디자인 변경에서 유지합니다. 실제 브라우저 포함 웹 시험 3개 통과, 320/390/768/1280/1440px 확인.
- 사용자 요청에 따라 **데몬/비주얼라이저에만** 누락 빌드 자동 복구를 추가했습니다. `scripts/ensure_build.sh`를 공유하고 동시 실행은 파일 잠금으로 직렬화합니다. 웹 런처의 C++ 준비 정책은 유지합니다. 누락 자식 실행 파일·전체 빌드 디렉터리 삭제·동시 실행·빌드 실패 중단을 임시 체크아웃에서 검증했습니다. 셰이더는 매 복구 빌드 후 복사하고 manifest로 누락을 검사합니다.
- 사용자 시작 오류 원인은 `logs/run-1793275909845-102463/simulation.log`의 잔여 `/tmp/qhrr05-simulation-1000/qhrr05.sock` 충돌입니다. `Broken pipe`는 ready 응답 전에 시뮬레이터가 종료된 결과였습니다. simulation IPC에 세션별 소유권 flock과 리스너 검사 후 안전한 잔여 소켓 복구를 추가했습니다. 활성 서버/일반 파일은 보존합니다. 데몬 시작 오류에 자식 로그 경로를 추가했습니다. IPC·외란·daemon lifecycle/SHM·control IO·자동 빌드 회귀 시험 6개 통과.

- **최종 실기 검증:** 삭제된 `build-filament/`를 `run_visualization.sh --preview`가 자동으로 내려받아 재구성했고, RTX 5090 / Vulkan에서 실제 5프레임 출력까지 성공했습니다. 셰이더 manifest 26개 자산 확인, Classic 미리보기와 읽기 전용 스냅샷 시험도 통과했습니다. 실제 MuJoCo를 SIGKILL한 뒤 같은 시험 세션으로 재시작하는 추가 시험을 통과했으며, 기존 CAN 런처 선택 시험과 웹 3개 시험도 재통과했습니다. UI 캡처는 `artifacts/web-redesign/{desktop,mobile}.png`에 보관했습니다.

## 최신 변경 — GUI 기능 통합과 시뮬레이션 IPC 분리

- **사용자 아키텍처 지시 반영:** SHM은 실제 하드웨어용 Motor/Model/Control과 공통 수명 플래그만 유지합니다. `SimulationInput` 및 GT·접촉·물체·pause/finished를 SHM에서 제거했고 ABI를 **12**로 올렸습니다. 기본 실행 파일과 뷰어를 모두 재빌드하고 기존 세션을 종료한 뒤 재시작해야 합니다. 기존 사용자 세션은 건드리지 않았습니다.
- **통일된 전용 IPC:** `src/ipc/simulation`의 버전 1 Unix `SOCK_SEQPACKET` 채널 하나로 시뮬레이션 실제 상태와 승인 명령을 교환합니다. 사용자별 0700 디렉터리/세션별 소켓, 버전·길이·역할 검사, nonblocking 송수신, 느린 구독자별 누락 수, 정상 종료 시 정리를 제공합니다. logger·CSV·웹 상태 브리지·뷰어를 모두 이 경로로 이전했습니다. 하드웨어 로거는 시뮬레이션 구독을 만들지 않습니다. 구형 MCAP 분석은 유지합니다.
- **제어 경로:** 새 뷰어는 SHM 명령을 쓰지 않고 Input → 기본 안전레이어 → TaskController를 거칩니다. 시뮬레이션 조작은 승인 후 전용 IPC로 물리 스레드가 적용합니다. 외란은 단일 뷰어 소유와 500 ms lease를 사용하며 해제·포커스 상실·UI 정지 시 잔류 힘을 제거합니다. 단일 스텝은 pause에서만 허용합니다. 기존 모델·게인·모션 계산식은 유지했습니다.
- **새 창 조작:** D/H/G/M/T/B, 1/2/3 평면, 목표 힘 슬라이더, Space/R/Right, Ctrl 드래그 힘·토크 및 mocap 이동·회전, 발끝 잔상·공 목표·W 작업공간과 기존 모드별 플롯/진단을 지원합니다. Classic의 LivePlots/ToeTrail/WorkspaceOverlay를 재사용하고 ImGui UI를 별도 합성합니다. Tab 패널, L 잔상, F/Home 카메라, Alt+0..5 geom, F12 캡처를 제공합니다.
- **검증:** CTest **36/36**, 웹 실제 브라우저 포함 **3/3**, 뷰어 스냅샷 동시성·owner death·ABI 시험 통과. 실제 RTX 5090 Filament·독립 Classic 두 창의 슬라이더·모드·평면·정확한 1 ms 스텝·힘/토크·물체 이동/회전·작업공간·리셋·독립 종료 시험 통과. IPC backpressure·버전 거부와 외란 lease 만료 후 힘 0도 자동 시험합니다. GPU 200 ms 지연 비교에서 모델 998.33→999.00 Hz, 제어 470.37→469.52 Hz(설정 500 Hz), SHM hold 최대 11.4 µs였습니다. 빌드와 셸/Python 문법·diff 검사도 통과했습니다.

## 이전 단계 기록

- **실사 느낌 조명·재질:** 로봇을 매트 그래파이트 아노다이징(금속성0.8~0.85, 최종 거칠기약0.65·약한 비드블라스트)으로 조정했습니다. 로고·고무를 구분하고 격자 바닥을 중립색 마이크로시멘트로 교체했습니다. CC0 Studio Small 09(Sergej Majboroda / Poly Haven)를 SHA256 확인 후 Filament cmgen으로 256px IBL KTX(약2MiB)로 변환·포함했습니다. 사선 key·약한 fill·PCSS·ACES +0.6EV·중립 배경을 적용했습니다. 실행 중 다운로드/자동 대체는 없습니다. 재현 도구와 출처 기록을 추가했으며 원본 물리 MJCF·제어 로직은 이번에 변경하지 않았습니다.
- **재질 검증:** 실제 RTX5090 이미지 비교·Classic 호환·GPU 지연 시험을 통과했습니다. 200ms 지연 중 물리999.65Hz, 제어480.32Hz, SHM hold 최대33.4µs였습니다. 최종 실제 출력은 ignored artifacts/pbr-lookdev/studio-final.png이며 소스 HDR·비교 이미지·로그는 Git 제외, 런타임 KTX/텍스처는 의도한 자산입니다.
- **조정 후 성능/재현:** 1600×1000에서 300프레임/5.27초(준비·종료 포함 평균56.9FPS). 텍스처 7개와 IBL KTX 모두 재생성 후 바이트 SHA256 일치를 확인했고 포맷·문법·diff 검사를 통과했습니다.

- **시각화 런처 배치 수정:** 실제 실행에서 6개 로그 터미널+daemon의 2열/4행 배치가 1440p 작업 영역의 행당 최소 높이에 미달했습니다. 시각화 YAML만 3열로 수정했습니다. 호스트에서 창 이동 없는 기존 planner probe로 2열 오류를 재현하고 3열 813×453px 배치를 검증했습니다. `--no-terminals`는 명시적 대안이며 자동 적용하지 않습니다.

- **독립 PBR 시각화:** 공식 MuJoCo 3.16.0(`a892750`) / upstream Filament rc/1.77.3을 고정한 `qhrr_visualizer`와 별도 빌드·실행·attach/preview 런처를 추가했습니다. `MUJOCO_USE_FILAMENT=ON`, `MJR_COMPAT=OFF`, RTX 5090 Vulkan 강제 검사이며 Classic `mjr` 백엔드도 제공합니다. 물리·제어는 기존 MuJoCo 3.3.2, 모델 XML·게인·제어 계산을 유지합니다. 원래 simulation의 추가 변경은 headless에서 duration 없이 daemon 종료까지 실행할 수 있도록 한 CLI 검사뿐입니다.
- **시각화 자원/동기화:** 별도 MJCF PBR 자산(금속/거칠기·색/노멀/AO), 환경 IBL/HDR KTX 선택·그림자·ACES, 카메라/resize/디버그 geometry를 구현했습니다. 관찰 스레드는 기존 SHM 두 슬롯에서 짧게 복사하고 GPU 작업은 별도 프로세스에서 수행합니다. 전체 접촉점을 임의 복원하지 않고 실제 전달된 발끝 접촉 합력·토크만 표시합니다. Clang 18.1.8은 SHA256 고정 공식 패키지를 ignored build-filament 안에만 풀었으며 시스템 패키지는 변경하지 않았습니다.
- **시각화 검증:** 실제 X11 / RTX 5090 / NVIDIA 580.82.09에서 Vulkan과 Classic 이미지·resize·orbit/pan/zoom·tracking·디버그 조작을 확인했습니다. 200ms/프레임 렌더 지연에도 물리 발행률 999.0→998.3Hz, 제어 480.7→480.0Hz(설정 500Hz), SHM hold 최대29.8µs였습니다. 스냅샷 동시성/owner death/ABI 검사, CPU GPU 거부·자원/인자 오류, 기존 GUI/CAN/MIT/제로셋 4개 회귀 시험이 통과했습니다. 저장소 밖 런처 호출·duration 없는 세션·attach 창 독립 종료·SHM 정리도 확인했습니다. 자세한 실행과 현 SHM 범위는 docs/visualization.md에 정리했습니다. 미커밋 상태입니다.
- **출력 성능:** 기본 1600×1000·60FPS 제한의 정적 미리보기에서 300프레임/5.43초(준비·종료 포함 평균55.3FPS)를 확인했습니다.

- **Git 제외 검토:** 로그 약 7.1GiB·build 약 457MiB·웹 의존성/산출물은 이미 제외되어 있었고, 1MiB 이상 미추적 파일 중 제외 누락은 0개였습니다. 압축/회전 로그·영상·다른 build 경로·Python/npm 캐시·웹 시험 산출물 제외를 보강했습니다. 모델 OBJ/미리보기·외부 MCAP/CSV fixture·package-lock 유지 등 33개 규칙 검사를 통과했습니다. 현재 미사용 ONNX Runtime은 추적된 정리 후보이며 자동 추적 해제하지 않았습니다.
- **웹 실행 스크립트:** `scripts/run_web_gui.sh`를 추가했습니다. 기본 실행은 준비 상태 확인 → 프런트엔드 빌드 → Node 서버 exec이며 `--setup`에서만 브리지 빌드와 npm ci를 수행합니다. 저장소 밖에서 호출·옵션 전달·신호 종료를 실제 Input/TaskController+브라우저 경로로 검사해 웹 시험 3개 통과했습니다.

- **제어 이상 원인:** 기존 C1 자동 원점 캡처와 호스트 `position_offset` 불일치가 확인됐습니다. 19:04 실행 MCAP에서 정지 구간 HR 위치 오차 중앙값은 +0.547712 rad였습니다. 모델 XML·게인·모션 계산식은 변경하지 않았습니다.
- **원점 옵션:** `simulation.yaml`의 `motor.require_zero_position_setup`을 추가했습니다. 기본 false는 CAN 위치=모델 관절각, 호스트 offset=0입니다. true는 시작 시 임의 고정 원점이며 명시적 C3 보정이 필요합니다. C1은 초기 원점을 다시 잡지 않습니다.
- **추가 수정:** GUI 상태 동기화 때문에 목표 힘이 설정 10N에서 SHM 초기값 3N으로 바뀌던 문제를 수정했습니다. 시작 시 설정값을 Input 경로로 발행합니다.
- **검증:** 모델/랜덤 원점·Iq·지연 enable·CAN 제어 3종·GUI 2종 총 8개 시험 통과. 실제 프로세스 시작을 1초 지연해 HR이 −0.58755 rad 움직인 시험에서 원점 오차 중앙값은 −0.0000564 rad, 목표 힘은 10N으로 확인했습니다.
- **GUI 진행:** React/Vite/WebSocket, 읽기 전용 상태 브리지, Input 소켓, 기본 안전레이어를 구현했습니다. 서버는 SHM에 명령을 쓰지 않고, MuJoCo GUI·조이스틱도 공통 Input/안전 판정을 사용합니다. Homing 선행 강제는 추가하지 않았습니다. 브라우저·통합 시험은 통과했고 사용자 참고 HTML은 보존했습니다. 변경은 아직 GUI 브랜치에서 미커밋 상태입니다.

## MIT 상태 전이 초판 — 2026-10-29

- **MIT 상태 전이:** 가상 모터를 Released / Enabled / MitControl로 구분했습니다. C1 진입 시 자동 원점 캡처, C2 release·목표 초기화, C3 signed 출력축 오프셋과 재-enable 시 사용자 원점 보존을 구현했습니다.
- **제로셋 보호:** C3 후 시뮬레이션 시간 20ms 동안 C0 목표를 버리고 새 원점으로 피드백합니다. 일시정지·시간 리셋·축별 독립 동작을 검사했습니다.
- **검증:** 빌드, clang-format 18.1.8, diff 검사 및 관련 시험 8개 통과(새 MIT CAN 시험, 센서 2개, HAL, 제어 통합 3개, CAN GUI).
- **범위:** C3는 CAN 수신으로 처리합니다. 기존 tau_ff=0 제한을 유지하며 C5 ROM 쓰기·fault 처리·C3 송신용 CLI는 미구현입니다. 초판의 C1 자동 캡처는 위 원점 정책으로 대체했습니다.

## 이전 X11 오류 분석 — 2026-10-28

- **최근 오류:** `logs/run-1793174205663-126352/simulation.log:37`의 X11 `BadWindow`, 요청 `X_ChangeProperty`, 대상 `0x5e00130`입니다. 해당 창 ID에 대한 프로퍼티 변경 요청이 거부됐습니다.
- **발생 순서(KST):** 17:05:39.619경 X11 오류 기록과 함께 모델·시뮬레이션·가상 모터 응답 갱신이 멈췄습니다. 17:05:39.792 로거가 CAN_TIMEOUT/SHM_STALE을 검출했습니다. 따라서 관측된 타임아웃은 갱신 중단 이후입니다.
- **제어 상태:** 장애 덤프에서 Tracking/XZ, IK 실패 없음, 비유한 수치 없음. ILC 토크 포화는 별도로 반복됐지만 X11 오류를 유발했다는 증거는 없습니다. 제어 명령은 기존 독립 주기로 계속 갱신됐고 17:06:14경 실행이 중단됐습니다.
- **남은 불확실성:** 오류의 창 ID가 MuJoCo 창인지 외부 클립보드 요청자 창인지, 구체적인 GUI 조작·호출 스택은 기록되어 있지 않습니다. 정상 종료 메시지·simulation_finished가 없어 종료 처리 경로도 추가 확인이 필요합니다.
- **범위:** 실행 로그·fault_10.mcap·관련 소스를 분석했습니다. 실행 중인 프로그램이나 제어 설정·코드는 변경하지 않았습니다. 영상은 계속 Git 제외 상태입니다.

## 다음 작업 — 우선순위 3개

1. 새 PBR 시각화·웹 제어 패널·기본 false 원점 설정의 사용자 운용 피드백을 확인합니다.
2. 일반 모델/자유 관절·개별 접촉점 전체가 필요하면 기존 simulation IPC 프로토콜을 명시적으로 확장합니다. QHRR mocap 회전은 현재 지원합니다.
3. 미사용 ONNX Runtime 추적 정리와 true 설정의 제로셋 운용 절차를 별도 작업으로 결정합니다.
