import React, { useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import "./style.css";

const modes = [
  {
    id: 6, name: "Zero set", ko: "영점 보정", key: "Z", icon: "home",
    description: "hip → thigh → calf 순서로 스토퍼를 찾아 영점을 보정합니다.",
  },
  {
    id: 5,
    name: "Damping",
    ko: "감쇠",
    key: "D",
    description: "위치 유지 없이 관절 속도를 감쇠합니다.",
    icon: "wave",
  },
  {
    id: 0,
    name: "Homing",
    ko: "호밍",
    key: "H",
    description: "설정된 기준 자세로 이동합니다.",
    icon: "home",
  },
  {
    id: 1,
    name: "Gravity",
    ko: "중력 보상",
    key: "G",
    description: "설정에 따라 중력·동역학을 보상합니다.",
    icon: "up",
  },
  {
    id: 2,
    name: "Magnetic",
    ko: "접촉 추종",
    key: "M",
    description: "접촉을 감지하고 목표 힘을 추종합니다.",
    icon: "magnet",
  },
  {
    id: 3,
    name: "Tracking",
    ko: "궤적 추종",
    key: "T",
    description: "선택한 평면의 궤적을 검사하고 추종합니다.",
    icon: "path",
  },
  {
    id: 4,
    name: "Ball",
    ko: "공 추종",
    key: "B",
    description: "조이스틱으로 공 목표를 움직입니다.",
    icon: "ball",
  },
];
const planes = ["XY", "XZ", "YZ"];
const fmt = (value, digits = 3) =>
  Number.isFinite(value) ? value.toFixed(digits) : "—";
const modeName = (id) => modes.find((mode) => mode.id === id)?.name ?? "—";

function Icon({ name, size = 22 }) {
  const paths = {
    wave: <path d="M2 12c3-12 5 12 8 0s5 12 8 0 3-4 4 0" />,
    home: (
      <>
        <path d="m3 10 9-7 9 7v10H3Z" />
        <path d="M9 20v-7h6v7" />
      </>
    ),
    up: (
      <>
        <path d="M12 21V4m-6 6 6-6 6 6" />
        <path d="M4 20h16" />
      </>
    ),
    magnet: (
      <>
        <path d="M5 4v9a7 7 0 0 0 14 0V4h-5v9a2 2 0 0 1-4 0V4Z" />
        <path d="M5 8h5m4 0h5" />
      </>
    ),
    path: (
      <>
        <path d="M12 12C1-1-3 25 12 12s19 13 0 0Z" />
      </>
    ),
    ball: (
      <>
        <circle cx="12" cy="12" r="8" />
        <path d="M3 12h18M12 4c-5 4-5 12 0 16 5-4 5-12 0-16Z" />
      </>
    ),
    arrow: <path d="M5 12h14m-6-6 6 6-6 6" />,
    panel: (
      <>
        <rect x="3" y="3" width="18" height="18" rx="3" />
        <path d="M9 3v18m4-12h4m-4 6h4" />
      </>
    ),
    chart: (
      <>
        <path d="M3 3v18h18M6 15l4-5 4 3 6-8" />
      </>
    ),
  };
  return (
    <svg
      width={size}
      height={size}
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="1.5"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      {paths[name]}
    </svg>
  );
}

function Sparkline({ history, axis }) {
  const values = history.map((sample) => sample[axis]);
  const finite = values.filter(Number.isFinite);
  if (finite.length < 2)
    return (
      <div className="chart-empty">새로운 측정값을 기다리고 있습니다.</div>
    );
  const low = Math.min(...finite),
    high = Math.max(...finite);
  const span = Math.max(high - low, 0.02);
  const bottom = (high + low - span) / 2,
    top = bottom + span;
  let previousValid = false;
  const path = values
    .map((value, i) => {
      if (!Number.isFinite(value)) {
        previousValid = false;
        return "";
      }
      const x = 8 + (i * 304) / (values.length - 1),
        y = 82 - ((value - bottom) / span) * 66;
      const segment = `${previousValid ? "L" : "M"}${x.toFixed(2)},${y.toFixed(2)}`;
      previousValid = true;
      return segment;
    })
    .join(" ");
  return (
    <div className="chart-plot">
      <div className="chart-scale">
        <span>{fmt(top)}</span>
        <span>{fmt(bottom)}</span>
      </div>
      <svg
        className="sparkline"
        viewBox="0 0 320 96"
        preserveAspectRatio="none"
        role="img"
        aria-label={`${["HR", "HP", "KP"][axis]} 관절 위치 추이, ${fmt(bottom)}에서 ${fmt(top)} rad`}
      >
        <path
          d="M0 16H320M0 49H320M0 82H320M80 8V90M160 8V90M240 8V90"
          className="chart-gridlines"
        />
        <path
          d={path}
          fill="none"
          stroke="currentColor"
          strokeWidth="2.5"
          vectorEffect="non-scaling-stroke"
        />
      </svg>
      <div className="chart-time">
        <span>이전</span>
        <span>최신</span>
      </div>
    </div>
  );
}

function RequestFeedback({ request }) {
  const status = request?.status ?? "idle";
  const description = !request
    ? "모드나 설정을 선택하면 처리 결과가 여기에 표시됩니다."
    : status === "unknown"
      ? "결과 미확인 · 자동 재전송하지 않습니다. 현재 상태를 확인하세요."
      : status === "rejected"
        ? `요청 거부 · ${request.reason}`
        : status === "applied"
          ? "TaskController 모드 반영을 확인했습니다. 실제 동작 완료를 뜻하지 않습니다."
          : status === "approved"
            ? request.type === "force"
              ? "목표 힘 요청이 승인되었습니다. 실제 접촉력 도달을 뜻하지 않습니다."
              : "안전 승인 완료 · 제어기에 모드가 반영되는지 확인하고 있습니다."
            : "요청을 전송했습니다. 안전 판정을 기다리고 있습니다.";
  return (
    <section
      className={`panel request-panel request-${status}`}
      aria-label="명령 처리 현황"
      aria-live="polite"
    >
      <div className="request-overview">
        <span className="request-symbol" aria-hidden="true">
          {status === "applied" || status === "approved"
            ? "✓"
            : status === "rejected" || status === "unknown"
              ? "!"
              : "→"}
        </span>
        <div>
          <div className="request-caption">
            최근 명령{" "}
            <span className="request-id">
              {request ? `#${String(request.id).padStart(4, "0")}` : "대기"}
            </span>
          </div>
          <strong className="request-title">
            {request?.label ?? "조작할 준비가 되셨나요?"}
          </strong>
        </div>
      </div>
      <div className="request-stages">
        {[
          ["requested", "요청 전송"],
          ["approved", "안전 승인"],
          ["applied", "모드 반영"],
        ].map(([stage, label], index) => {
          const completed =
            request &&
            ["requested", "approved", "applied"].indexOf(status) >= index;
          return (
            <span
              key={stage}
              className={`${completed ? "stage-complete" : ""} ${request?.type === "force" && stage === "applied" ? "stage-na" : ""}`}
            >
              <b>{completed ? "✓" : index + 1}</b>
              {request?.type === "force" && stage === "applied"
                ? "모드 전용"
                : label}
            </span>
          );
        })}
      </div>
      <p role="status">{description}</p>
    </section>
  );
}

function App() {
  const [state, setState] = useState(null);
  const [connection, setConnection] = useState("connecting");
  const [connectionKey, setConnectionKey] = useState(0);
  const [notice, setNotice] = useState(null);
  const [pending, setPending] = useState(false);
  const [plane, setPlane] = useState(0);
  const [force, setForce] = useState("");
  const [history, setHistory] = useState([]);
  const [latestRequest, setLatestRequest] = useState(null);
  const [events, setEvents] = useState([]);
  const socketRef = useRef(null);
  const nextId = useRef(0);
  const requestRef = useRef(null);
  const forceEdited = useRef(false);
  const planeEdited = useRef(false);
  const latestRef = useRef(null);
  const record = (title, status) =>
    setEvents((previous) =>
      [
        {
          title,
          status,
          time: new Date().toLocaleTimeString("en-GB", {
            hour12: false,
            timeZone: "Asia/Seoul",
          }),
        },
        ...previous,
      ].slice(0, 12),
    );
  const updateRequest = (value) => {
    latestRef.current = value;
    setLatestRequest(value);
  };

  useEffect(() => {
    setConnection("connecting");
    setState(null);
    setHistory([]);
    forceEdited.current = false;
    planeEdited.current = false;
    setForce("");
    setPlane(0);
    const ws = new WebSocket(
      `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`,
    );
    socketRef.current = ws;
    let disposed = false;
    ws.onopen = () => {
      if (!disposed) {
        setConnection("connected");
        record("제어 서버 연결", "CONNECTED");
      }
    };
    ws.onmessage = (event) => {
      if (disposed) return;
      try {
        const message = JSON.parse(event.data);
        if (message.type === "state") {
          setState(message);
          setHistory((previous) => [
            ...previous.slice(-99),
            message.motors.map((m) => m.position),
          ]);
          if (!forceEdited.current)
            setForce(String(message.target_force ?? ""));
          if (!planeEdited.current) setPlane(message.requested_tracking_plane);
          const latest = latestRef.current;
          if (
            latest?.status === "approved" &&
            latest.type === "mode" &&
            latest.motion_sequence === message.active_motion_sequence &&
            latest.mode === message.mode &&
            (latest.mode !== 3 || latest.plane === message.tracking_plane)
          ) {
            updateRequest({ ...latest, status: "applied" });
            record(`${latest.label} · TaskController 반영 확인`, "APPLIED");
          }
        } else if (message.id === requestRef.current || message.id === null) {
          setPending(false);
          requestRef.current = null;
          const latest = latestRef.current;
          if (latest) {
            const status =
              message.type === "ack"
                ? "approved"
                : message.uncertain
                  ? "unknown"
                  : "rejected";
            updateRequest({
              ...latest,
              status,
              motion_sequence: message.motion_sequence,
              reason: message.message,
            });
            record(
              `${latest.label} · ${message.type === "ack" ? "안전레이어 승인" : message.message}`,
              status.toUpperCase(),
            );
          }
          setNotice(
            message.type === "ack"
              ? null
              : {
                  kind: "error",
                  text: `${message.uncertain ? "결과 미확인" : "요청 거부"}: ${message.message}`,
                },
          );
        }
      } catch {
        setNotice({ kind: "error", text: "서버 응답을 읽지 못했습니다." });
        ws.close();
      }
    };
    ws.onerror = () => {
      if (!disposed)
        setNotice({
          kind: "error",
          text: "제어 서버에 연결할 수 없습니다. 서버 실행 상태를 확인하세요.",
        });
    };
    ws.onclose = () => {
      if (disposed) return;
      setConnection("disconnected");
      setState(null);
      setPending(false);
      setHistory([]);
      setForce("");
      setPlane(0);
      forceEdited.current = false;
      planeEdited.current = false;
      record("제어 서버 연결 끊김", "DISCONNECTED");
      const latest = latestRef.current;
      if (latest && ["requested", "approved"].includes(latest.status))
        updateRequest({ ...latest, status: "unknown" });
      if (requestRef.current !== null)
        setNotice({
          kind: "error",
          text: "연결이 끊겨 요청 결과를 확인하지 못했습니다. 재연결 후 상태를 확인하세요.",
        });
      requestRef.current = null;
    };
    return () => {
      disposed = true;
      ws.close();
    };
  }, [connectionKey]);

  const available = connection === "connected" && state?.ready && !pending;
  const activeMode = modes.find((mode) => mode.id === state?.mode);
  const request = (command) => {
    if (
      !available ||
      socketRef.current?.readyState !== WebSocket.OPEN ||
      requestRef.current !== null
    )
      return;
    const id = ++nextId.current;
    requestRef.current = id;
    const label =
      command.type === "mode"
        ? `${modeName(command.mode)}${command.mode === 3 ? ` · ${planes[command.plane]}` : ""}`
        : `목표 힘 ${command.value} N`;
    updateRequest({ ...command, id, label, status: "requested" });
    record(`${label} · 요청 전송`, "REQUESTED");
    setPending(true);
    setNotice(null);
    socketRef.current.send(JSON.stringify({ ...command, id }));
  };
  const forceValid =
    force.trim() !== "" &&
    Number.isFinite(Number(force)) &&
    Number(force) >= 0 &&
    Number(force) <= state?.maximum_force;
  const changeMode = (mode) => request({ type: "mode", mode, plane });
  const applied =
    state && state.motion_sequence === state.active_motion_sequence;

  const changeForce = (value) => {
    forceEdited.current = true;
    setForce(String(value));
  };
  const forceDirty = forceValid && Number(force) !== state?.target_force;
  const forceInvalid =
    connection === "connected" &&
    state?.ready &&
    forceEdited.current &&
    !forceValid;

  useEffect(() => {
    const onKey = (event) => {
      if (
        event.repeat ||
        event.altKey ||
        event.ctrlKey ||
        event.metaKey ||
        event.target.closest?.(
          'input, textarea, select, [contenteditable="true"]',
        )
      )
        return;
      const mode = modes.find((item) => item.key === event.key.toUpperCase());
      const selectedPlane = ["1", "2", "3"].indexOf(event.key);
      if (!mode && selectedPlane < 0) return;
      event.preventDefault();
      if (selectedPlane >= 0) {
        planeEdited.current = true;
        setPlane(selectedPlane);
        request({ type: "mode", mode: 3, plane: selectedPlane });
      } else changeMode(mode.id);
    };
    document.addEventListener("keydown", onKey);
    return () => document.removeEventListener("keydown", onKey);
  });

  return (
    <div className="app-shell">
      <a className="skip-link" href="#control">
        제어 패널로 바로가기
      </a>
      <header className="topbar">
        <a className="brand" href="#" aria-label="QHRR 05 홈">
          <span className="brand-mark" aria-hidden="true">
            <i />
            <i />
            <i />
            <b>Q</b>
          </span>
          <span>
            QHRR<span className="brand-model">05</span>
            <small>ROBOT OPERATIONS</small>
          </span>
        </a>
        <nav aria-label="주 메뉴">
          <a href="#control">
            <Icon name="panel" size={18} />
            제어
          </a>
          <a href="#telemetry">
            <Icon name="chart" size={18} />
            실시간 상태
          </a>
          <a href="#events">이벤트</a>
        </nav>
        <div className="connection-box">
          <span className={`connection ${connection}`} role="status">
            <i />
            {connection === "connected"
              ? "서버 연결됨"
              : connection === "connecting"
                ? "연결 중"
                : "연결 끊김"}
          </span>
          {connection === "disconnected" && (
            <button
              className="reconnect-button"
              onClick={() => {
                setNotice(null);
                setConnectionKey((key) => key + 1);
              }}
            >
              다시 연결
            </button>
          )}
        </div>
      </header>

      <main>
        <header className="page-header">
          <div>
            <div className="eyebrow">
              QHRR-05 <span>/</span> CONTROL WORKSPACE
            </div>
            <h1>로봇 제어</h1>
            <p>제어 모드와 목표 힘을 설정하고, 실시간 상태를 확인하세요.</p>
          </div>
          <span className="session-tag">
            <i />3 DOF <span>HR · HP · KP</span>
          </span>
        </header>

        {notice?.kind === "error" && (
          <div
            role={notice.kind === "error" ? "alert" : "status"}
            className={`notice ${notice.kind}`}
          >
            <span className="notice-symbol" aria-hidden="true">
              {notice.kind === "error"
                ? "!"
                : notice.kind === "success"
                  ? "✓"
                  : "i"}
            </span>
            <span>{notice.text}</span>
            <button aria-label="알림 닫기" onClick={() => setNotice(null)}>
              ×
            </button>
          </div>
        )}
        {connection !== "connected" && (
          <div className="notice warning" role="status">
            <span className="notice-symbol" aria-hidden="true">
              !
            </span>
            <span>
              {connection === "connecting"
                ? "서버에 연결하고 있습니다. 상태를 받은 뒤 조작할 수 있습니다."
                : "연결이 끊겼습니다. 다시 연결해 상태를 확인하세요. 모든 제어 버튼이 비활성화되었습니다."}
            </span>
          </div>
        )}
        {connection === "connected" && !state?.ready && (
          <div role="status" className="notice warning">
            <span className="notice-symbol" aria-hidden="true">
              !
            </span>
            <span>
              {state?.failed
                ? "제어 세션에 오류가 있습니다."
                : state?.stopping
                  ? "제어 세션이 종료 중입니다."
                  : "제어기와 모델의 첫 상태를 기다리고 있습니다."}
            </span>
          </div>
        )}

        <section className="summary-grid" aria-label="세션 요약">
          <div className="summary-card current-mode">
            <div className="summary-label">
              <span>현재 제어 모드</span>
              <span className="live-label">
                <i />
                {state?.ready ? "LIVE" : "대기"}
              </span>
            </div>
            <strong>{activeMode?.name ?? "—"}</strong>
            <small>
              {state
                ? applied
                  ? activeMode?.ko
                  : `${modeName(state.requested_mode)} 적용 대기`
                : "세션 연결 대기"}
            </small>
            <div className="summary-icon" aria-hidden="true">
              <Icon name={activeMode?.icon ?? "panel"} size={64} />
            </div>
          </div>
          <div className="summary-card">
            <span className="summary-label">모터 연결</span>
            <strong>
              {state
                ? state.can_enabled
                  ? `${state.motors.filter((m) => m.enabled).length} / 3`
                  : "DEBUG"
                : "—"}
            </strong>
            <small>
              {state
                ? state.can_enabled
                  ? "CAN · Enabled 모터"
                  : "직접 토크 모드"
                : "측정 대기"}
            </small>
          </div>
          <div
            className={`summary-card ${state?.contact ? "has-contact" : ""}`}
          >
            <span className="summary-label">발끝 접촉</span>
            <strong>
              {state?.contact == null
                ? "—"
                : state.contact
                  ? "접촉 중"
                  : "비접촉"}
            </strong>
            <small>
              {state?.mode === 2
                ? state.magnetic_phase
                : state?.contact == null
                  ? "시뮬레이션 데이터 없음"
                  : "시뮬레이션 실제 접촉"}
            </small>
          </div>
          <div className="summary-card">
            <span className="summary-label">시뮬레이션 시간</span>
            <strong className="mono">
              {fmt(state?.time, 2)}
              <em>s</em>
            </strong>
            <small>
              {Number.isFinite(state?.time)
                ? "시뮬레이션 시계 기준"
                : "시뮬레이션 데이터 없음"}
            </small>
          </div>
        </section>

        <RequestFeedback request={latestRequest} />

        <div className="control-grid" id="control">
          <section className="panel mode-panel" aria-labelledby="mode-title">
            <div className="section-heading">
              <div>
                <span className="section-number">01</span>
                <h2 id="mode-title">어떻게 움직일까요?</h2>
              </div>
              <span className="section-meta">제어 모드</span>
            </div>
            <p className="section-description">
              선택하면 해당 모드로 전환을 요청합니다.
            </p>
            <div className="mode-grid">
              {modes.map((mode) => {
                const selected = state?.mode === mode.id;
                const waiting =
                  latestRequest?.type === "mode" &&
                  latestRequest.mode === mode.id &&
                  ["requested", "approved"].includes(latestRequest.status);
                return (
                  <button
                    key={mode.id}
                    className={`mode-card ${selected ? "selected" : ""} ${waiting ? "waiting" : ""}`}
                    disabled={!available}
                    onClick={() => changeMode(mode.id)}
                    aria-pressed={selected}
                    aria-keyshortcuts={mode.key}
                  >
                    <span className="mode-card-top">
                      <span className="mode-icon">
                        <Icon name={mode.icon} size={25} />
                      </span>
                      <span
                        className={
                          selected
                            ? "active-label"
                            : waiting
                              ? "pending-label"
                              : "mode-code"
                        }
                      >
                        {waiting
                          ? "반영 대기"
                          : selected
                            ? "현재 모드"
                            : mode.key}
                      </span>
                    </span>
                    <strong>
                      {mode.name}
                      <span>{mode.ko}</span>
                    </strong>
                    <p>{mode.description}</p>
                    <span className="mode-action">
                      {selected ? "다시 요청" : "선택하기"}
                      <Icon name="arrow" size={17} />
                    </span>
                  </button>
                );
              })}
            </div>
            {state?.zero_set && (state.zero_set.required || state.mode === 6) && (
              <div className="inline-warning" role={state.zero_set.phase === "FAULT" ? "alert" : "status"}>
                영점 보정: {state.zero_set.axis} / {state.zero_set.phase}
                {state.zero_set.phase === "FAULT" && ` — ${state.zero_set.failure} (모터 출력 해제)`}
                <br />Z: 처음부터 재시도 · D: 중단. 보정 완료 전에는 일반 모션이 차단됩니다.
              </div>
            )}
            {state?.tracking_checking && (
              <div className="tracking-progress" role="status">
                <span>
                  궤적 검사 중{" "}
                  <b>{fmt(state.tracking_check_progress * 100, 0)}%</b>
                </span>
                <progress max="1" value={state.tracking_check_progress} />
              </div>
            )}
            {state?.tracking_ik_failed && (
              <div role="alert" className="inline-warning">
                궤적 검사 또는 IK 한계로 자세를 유지하고 있습니다. 상태를 확인해
                주세요.
              </div>
            )}
            {state?.tracking_torque_saturated && (
              <div className="inline-warning">
                Tracking 토크 포화가 감지되었습니다.
              </div>
            )}
            <div className="keyboard-hint">
              <span>단축키</span>
              {modes.map((mode) => (
                <kbd key={mode.key}>{mode.key}</kbd>
              ))}
              <small>입력란 편집 중에는 비활성</small>
            </div>
            <div className="mode-footnote">
              영점 설정이 필요하면 Zero set으로 시작하며, 아니면 Damping으로 시작합니다.
              영점 보정 후 기준 자세 이동은 Homing을 선택하세요.
            </div>
          </section>

          <section
            className="panel settings-panel"
            aria-labelledby="settings-title"
          >
            <div className="section-heading">
              <div>
                <span className="section-number">02</span>
                <h2 id="settings-title">세부 설정</h2>
              </div>
            </div>
            <div className="setting-block plane-setting">
              <div className="setting-title">
                <label id="plane-label">추종 평면</label>
                <span className="setting-tag">Tracking</span>
              </div>
              <div
                className="segmented"
                role="group"
                aria-labelledby="plane-label"
              >
                {planes.map((name, i) => (
                  <button
                    key={name}
                    aria-pressed={plane === i}
                    className={plane === i ? "chosen" : ""}
                    disabled={!available}
                    onClick={() => {
                      planeEdited.current = true;
                      setPlane(i);
                    }}
                  >
                    <span>{name}</span>
                    <kbd>{i + 1}</kbd>
                  </button>
                ))}
              </div>
              <div className="setting-caption">
                현재 추종 평면{" "}
                <strong>
                  {state?.mode === 3 ? planes[state.tracking_plane] : "—"}
                </strong>
              </div>
              <button
                className="apply-button secondary"
                disabled={!available}
                onClick={() => changeMode(3)}
              >
                Tracking 시작 · {planes[plane]}
                <Icon name="arrow" size={18} />
              </button>
            </div>
            <div className="setting-block force-setting">
              <div className="setting-title">
                <label htmlFor="target-force">목표 접촉력</label>
                <span className="setting-tag">Magnetic</span>
              </div>
              <p>값을 조절한 뒤 적용 버튼을 눌러주세요.</p>
              <div className="force-control">
                <button
                  className="counter minus"
                  aria-label="목표 접촉력 0.5 N 줄이기"
                  disabled={!available || !forceValid || Number(force) <= 0}
                  onClick={() =>
                    changeForce(Math.max(0, +(Number(force) - 0.5).toFixed(1)))
                  }
                >
                  −
                </button>
                <div className="force-input">
                  <input
                    id="target-force"
                    type="number"
                    inputMode="decimal"
                    min="0"
                    max={state?.maximum_force ?? 0}
                    step="0.1"
                    value={force}
                    disabled={!available}
                    aria-invalid={Boolean(forceInvalid)}
                    aria-describedby="force-help"
                    onChange={(event) => changeForce(event.target.value)}
                  />
                  <span>N</span>
                </div>
                <button
                  className="counter plus"
                  aria-label="목표 접촉력 0.5 N 늘리기"
                  disabled={
                    !available ||
                    !forceValid ||
                    Number(force) >= state?.maximum_force
                  }
                  onClick={() =>
                    changeForce(
                      Math.min(
                        state.maximum_force,
                        +(Number(force) + 0.5).toFixed(1),
                      ),
                    )
                  }
                >
                  +
                </button>
              </div>
              <input
                className="force-range"
                type="range"
                aria-label="목표 접촉력 슬라이더"
                min="0"
                max={state?.maximum_force ?? 0}
                step="0.1"
                value={forceValid ? force : 0}
                disabled={!available}
                onChange={(event) => changeForce(event.target.value)}
              />
              <div className="range-labels">
                <span>0 N</span>
                <span>{fmt(state?.maximum_force, 1)} N</span>
              </div>
              <div
                id="force-help"
                className={forceInvalid ? "field-error" : "force-help"}
              >
                {forceInvalid
                  ? `0~${fmt(state?.maximum_force, 1)} N 범위의 숫자를 입력하세요.`
                  : forceDirty
                    ? "변경한 값이 아직 적용되지 않았습니다."
                    : "설정 범위 안에서 0.1 N 단위로 조절합니다."}
              </div>
              <button
                className="apply-button primary"
                disabled={!available || !forceValid}
                onClick={() => request({ type: "force", value: Number(force) })}
              >
                목표 힘 적용
                <Icon name="arrow" size={18} />
              </button>
              <div className="current-force">
                적용 요청값 <strong>{fmt(state?.target_force, 1)} N</strong>
                {forceDirty && <span className="draft-badge">미적용 변경</span>}
              </div>
            </div>
          </section>
        </div>

        <section
          className="panel telemetry-panel"
          id="telemetry"
          aria-labelledby="telemetry-title"
        >
          <div className="section-heading">
            <div>
              <span className="section-number">03</span>
              <h2 id="telemetry-title">로봇은 지금</h2>
            </div>
            <span className="section-meta">
              <i
                className={`live-dot ${connection === "connected" && state?.ready ? "live" : ""}`}
              />
              20 Hz 상태 조회
            </span>
          </div>
          <div className="telemetry-intro">
            <p>관절별 측정값과 최근 위치 변화를 확인하세요.</p>
            <div className="toe-position">
              <span>
                발끝 <small>m</small>
              </span>
              {["X", "Y", "Z"].map((axis, i) => (
                <span key={axis}>
                  <b>{axis}</b>
                  <strong>{fmt(state?.toe_position[i])}</strong>
                </span>
              ))}
            </div>
          </div>
          <div className="table-scroll">
            <table>
              <caption className="sr-only">
                HR, HP, KP 관절의 측정값과 CAN 상태
              </caption>
              <thead>
                <tr>
                  <th scope="col">관절</th>
                  <th scope="col">
                    위치 <span>rad</span>
                  </th>
                  <th scope="col">
                    속도 <span>rad/s</span>
                  </th>
                  <th scope="col">
                    전류 <span>A</span>
                  </th>
                  <th scope="col">
                    토크 <span>N·m</span>
                  </th>
                  <th scope="col">
                    온도 <span>°C</span>
                  </th>
                  <th scope="col">CAN 상태</th>
                </tr>
              </thead>
              <tbody>
                {["HR", "HP", "KP"].map((axis, i) => {
                  const motor = state?.motors[i];
                  return (
                    <tr key={axis}>
                      <th scope="row">
                        <span className={`axis-dot axis-${i}`} />
                        <strong>{axis}</strong>
                        <small>
                          {["Hip roll", "Hip pitch", "Knee pitch"][i]}
                        </small>
                      </th>
                      <td data-label="위치 · rad">{fmt(motor?.position)}</td>
                      <td data-label="속도 · rad/s">{fmt(motor?.velocity)}</td>
                      <td data-label="전류 · A">{fmt(motor?.current, 2)}</td>
                      <td data-label="토크 · N·m">{fmt(motor?.torque, 2)}</td>
                      <td data-label="온도 · °C">
                        {fmt(motor?.temperature, 0)}
                      </td>
                      <td data-label="CAN 상태">
                        <span
                          className={`motor-status ${state?.can_enabled && motor?.enabled ? "enabled" : ""}`}
                        >
                          {!state
                            ? "미연결"
                            : !state.can_enabled
                              ? "DEBUG"
                              : !motor.profile_verified
                                ? "프로파일 대기"
                                : motor.enabled
                                  ? "Enabled"
                                  : "Released"}
                        </span>
                      </td>
                    </tr>
                  );
                })}
              </tbody>
            </table>
          </div>
          <div className="chart-grid">
            {["HR", "HP", "KP"].map((axis, i) => (
              <div className={`chart-card chart-${i}`} key={axis}>
                <div className="chart-title">
                  <span>
                    <i className={`axis-dot axis-${i}`} />
                    <b>{axis}</b> 위치
                  </span>
                  <strong>
                    {fmt(state?.motors[i].position)}
                    <em>rad</em>
                  </strong>
                </div>
                <Sparkline
                  history={connection === "connected" ? history : []}
                  axis={i}
                />
              </div>
            ))}
          </div>
          <p className="chart-note">
            최근 최대 100개 수신 표본 · 각 그래프의 세로축 범위는 별도로
            표시합니다.
          </p>
        </section>

        <section
          className="panel event-panel"
          id="events"
          aria-labelledby="event-title"
        >
          <div className="section-heading">
            <div>
              <span className="section-number">04</span>
              <h2 id="event-title">작업 기록</h2>
            </div>
            <span className="section-meta">최근 12건 · KST</span>
          </div>
          <ol className="event-list">
            {events.map((event, i) => (
              <li key={`${event.time}-${i}`} data-status={event.status}>
                <time>{event.time}</time>
                <span>{event.title}</span>
                <small>
                  {{
                    CONNECTED: "연결",
                    DISCONNECTED: "연결 끊김",
                    REQUESTED: "요청 전송",
                    APPROVED: "안전 승인",
                    APPLIED: "모드 반영",
                    REJECTED: "거부",
                    UNKNOWN: "미확인",
                  }[event.status] ?? event.status}
                </small>
              </li>
            ))}
          </ol>
          {!events.length && (
            <p className="event-empty">
              연결과 명령 처리 기록이 여기에 표시됩니다.
            </p>
          )}
        </section>
        <footer>
          <span>
            <i className="footer-dot" />
            QHRR-05 OPERATIONS
          </span>
          <span>
            측정값과 명령 결과는 현재 연결된 제어 세션을 기준으로 표시합니다.
          </span>
        </footer>
      </main>
    </div>
  );
}

createRoot(document.getElementById("root")).render(<App />);
