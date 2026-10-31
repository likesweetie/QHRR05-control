import React, { useEffect, useState } from "react";
import { createRoot } from "react-dom/client";
import {
  useSession,
  modes,
  modeName,
  planes,
  fmt,
  age,
  vector,
} from "./session.jsx";
import "./style.css";

const axes = ["HR · hip", "HP · thigh", "KP · calf"];
const hex = (value) =>
  Number.isInteger(value) ? `0x${value.toString(16).toUpperCase()}` : "—";
const statusValue = (value, status, digits = 3) =>
  Number.isFinite(value)
    ? fmt(value, digits)
    : ({
        unsupported: "취득 불가",
        waiting: "미수신",
        stale: "오래됨",
        uncalibrated: "미보정",
        invalid: "무효",
      }[status] ?? "—");
const tone = (value) =>
  ["ACTIVE", "online", "RUNNING", "COMPLETE"].includes(value)
    ? "ok"
    : ["FAULT", "error", "stale", "disconnected"].includes(value)
      ? "danger"
      : ["ARMED", "DAMPING", "ZERO SET"].includes(value)
        ? "warn"
        : "muted";
function Badge({ children, status, className = "" }) {
  return (
    <span className={`badge ${status ?? tone(children)} ${className}`}>
      {children}
    </span>
  );
}
function Panel({ eyebrow, title, className = "", extra, children }) {
  return (
    <section className={`panel ${className}`}>
      <div className="panel-heading">
        <div>
          <p className="eyebrow">{eyebrow}</p>
          <h2>{title}</h2>
        </div>
        {extra}
      </div>
      {children}
    </section>
  );
}
function Metrics({ entries, className = "metric-grid compact" }) {
  return (
    <div className={className}>
      {entries.map(([label, value]) => (
        <div key={label}>
          <span>{label}</span>
          <strong title={typeof value === "string" ? value : undefined}>
            {value ?? "—"}
          </strong>
        </div>
      ))}
    </div>
  );
}
function RequestStatus({ session }) {
  const req = session.latest;
  const steps = ["요청 전송", "안전 승인", "TaskController 반영"];
  const count = !req
    ? 0
    : req.status === "applied"
      ? 3
      : req.status === "approved"
        ? 2
        : 1;
  return (
    <div
      className={`request-panel request-${req?.status ?? "idle"}`}
      aria-live="polite"
    >
      <div className="request-caption">
        {req ? `#${req.id} ${req.label}` : "명령 대기"}
      </div>
      <div className="request-stages">
        {steps.map((step, i) => (
          <span key={step} className={count > i ? "stage-complete" : ""}>
            {i + 1}.{" "}
            {req && ["stack", "process"].includes(req.type)
              ? ["요청 전송", "Daemon 판정", "프로세스 반영"][i]
              : step}
          </span>
        ))}
      </div>
      <p>
        {req?.status === "rejected"
          ? `요청 거부 · ${req.message}`
          : req?.status === "unknown"
            ? "결과 미확인 · 자동 재전송하지 않습니다."
            : req?.status === "applied"
              ? ["stack", "process"].includes(req.type)
                ? "Daemon 프로세스 관리 반영 완료"
                : "TaskController 모드 반영을 확인했습니다. 동작 완료와는 구분합니다."
              : req?.status === "approved"
                ? req.type === "force"
                  ? "목표 힘 요청이 승인되었습니다."
                  : "안전 승인 완료 · 제어기 반영 대기"
                : req
                  ? "명령의 판정 결과를 기다립니다."
                  : "모드 선택 → Arm(A). 개별 Enable은 중립 출력으로 대기합니다."}
      </p>
    </div>
  );
}
function StateMachine({ session, plane }) {
  const { state: s, available, send } = session;
  const drive = s?.drive_state;
  const current =
    drive !== "ACTIVE"
      ? drive
      : s.mode === 5
        ? "DAMPING"
        : s.mode === 6
          ? "ZERO SET"
          : "ACTIVE";
  const stages = [
    "DISARMED",
    "ARMED",
    "ACTIVE",
    "DAMPING",
    "ZERO SET",
    "FAULT",
  ];
  const reason =
    drive === "DISARMED"
      ? `출력 해제 상태입니다. ${modeName(s?.requested_mode)} 선택 후 Arm(A)으로 시작합니다.`
      : drive === "ARMED"
        ? "개별 모터가 중립 출력으로 Enable되어 있습니다. Arm / Run으로 전체 모션을 시작합니다."
        : drive === "FAULT"
          ? s?.drive_fault
          : s?.mode === 6
            ? `${s.zero_set.axis} · ${s.zero_set.phase} · ${fmt(s.zero_set.travel_rad)} rad / ${fmt(s.zero_set.elapsed_sec, 1)} s`
            : s?.holding
              ? "설정 자세를 유지하고 있습니다."
              : "현재 모션이 실행 중입니다.";
  return (
    <Panel
      eyebrow="Safety"
      title="State Machine"
      className="machine-panel"
      extra={<Badge>{current ?? "waiting"}</Badge>}
    >
      <div className="state-machine" aria-label="구동 상태">
        {stages.map((item) => (
          <div
            key={item}
            className={`state-block ${item === current ? `active ${tone(item)}` : "idle"}`}
            data-state={item}
          >
            <span />
            <strong>{item}</strong>
          </div>
        ))}
      </div>
      <Metrics
        className="machine-detail-grid"
        entries={[
          [
            "Controller",
            <span id="machineControllerState">{modeName(s?.mode)}</span>,
          ],
          ["Selected", modeName(s?.requested_mode)],
          ["Fault", s?.transport_fault ?? "—"],
          ["Age", age(s?.controller_age_ms)],
        ]}
      />
      <p className="machine-reason" title={reason}>
        {s ? reason : "제어 서버 연결 대기"}
      </p>
      <div className="operator-actions">
        <button
          id="armButton"
          className="button primary"
          disabled={!available || ["ACTIVE", "FAULT"].includes(drive)}
          onClick={() => send({ type: "arm" }, "Arm · 전체 모션 시작")}
        >
          Arm
        </button>
        <button
          id="runButton"
          className="button primary"
          disabled={!available || !["ARMED", "ACTIVE"].includes(drive)}
          onClick={() =>
            drive === "ARMED"
              ? send({ type: "arm" }, "Run · 전체 모션 시작")
              : send(
                  { type: "mode", mode: s.requested_mode, plane },
                  `Run ${modeName(s.requested_mode)}`,
                )
          }
        >
          Run
        </button>
        <button
          id="dampingButton"
          className="button secondary"
          disabled={!available}
          onClick={() => send({ type: "mode", mode: 5, plane }, "Damping")}
        >
          Damping
        </button>
        <button
          id="faultClearButton"
          className="button secondary"
          disabled={!available || drive !== "FAULT"}
          onClick={() => send({ type: "clear-fault" }, "Fault Clear")}
        >
          Fault Clear
        </button>
        <button
          id="releaseButton"
          className="button danger"
          disabled={!available}
          onClick={() => send({ type: "release" }, "Release · 전체 출력 해제")}
        >
          Release / Stop
        </button>
      </div>
      <RequestStatus session={session} />
    </Panel>
  );
}
function ModeControl({ session, plane, setPlane }) {
  const { state: s, available, send } = session;
  const [force, setForce] = useState(""),
    [edited, setEdited] = useState(false);
  useEffect(() => {
    if (!edited) setForce(s ? String(s.target_force) : "");
  }, [s?.target_force, !!s, edited]);
  useEffect(() => {
    if (!s) {
      setEdited(false);
      setForce("");
    }
  }, [!!s]);
  const valid =
    force.trim() !== "" &&
    Number.isFinite(Number(force)) &&
    Number(force) >= 0 &&
    Number(force) <= s?.maximum_force;
  const dirty = edited && Number(force) !== s?.target_force;
  return (
    <Panel
      eyebrow="Control"
      title="Motion & Force"
      className="command-panel"
      extra={<Badge>{modeName(s?.requested_mode)}</Badge>}
    >
      <div className="mode-grid">
        {modes.map((m) => (
          <button
            key={m.id}
            className={`button mode-card ${s?.requested_mode === m.id ? "selected" : "secondary"}`}
            disabled={!available}
            onClick={() => send({ type: "mode", mode: m.id, plane }, m.name)}
          >
            <span>{m.name}</span>
            <kbd>{m.key}</kbd>
          </button>
        ))}
      </div>
      <div className="plane-control">
        <label>Tracking plane</label>
        <div className="segmented">
          {planes.map((p, i) => (
            <button
              key={p}
              className={`button ${plane === i ? "selected" : "secondary"}`}
              disabled={!available}
              onClick={() => setPlane(i)}
            >
              {p}
            </button>
          ))}
        </div>
      </div>
      <div className="transmit-subhead">
        <strong>Target force</strong>
        <span className="current-force">
          <strong>{fmt(s?.target_force, 1)} N</strong>
        </span>
        {dirty && (
          <Badge status="warn" className="draft-badge">
            미적용
          </Badge>
        )}
      </div>
      <form
        className="force-form"
        onSubmit={(e) => {
          e.preventDefault();
          if (valid) {
            send({ type: "force", value: Number(force) }, `목표 힘 ${force} N`);
            setEdited(false);
          }
        }}
      >
        <label htmlFor="target-force">
          Force [N]
          <input
            id="target-force"
            type="number"
            min="0"
            max={s?.maximum_force ?? 0}
            step="0.5"
            value={force}
            disabled={!available}
            aria-invalid={edited && !valid}
            onChange={(e) => {
              setEdited(true);
              setForce(e.target.value);
            }}
          />
        </label>
        <button
          className="button counter"
          type="button"
          disabled={!available}
          aria-label="목표 접촉력 0.5 N 줄이기"
          onClick={() => {
            setEdited(true);
            setForce(String(Math.max(0, Number(force || 0) - 0.5)));
          }}
        >
          −
        </button>
        <button
          className="button counter"
          type="button"
          disabled={!available}
          aria-label="목표 접촉력 0.5 N 늘리기"
          onClick={() => {
            setEdited(true);
            setForce(
              String(Math.min(s.maximum_force, Number(force || 0) + 0.5)),
            );
          }}
        >
          +
        </button>
        <button
          className="button primary apply-button"
          disabled={!available || !valid}
        >
          목표 힘 적용
        </button>
      </form>
      {edited && !valid && (
        <p className="error-line">
          0–{s?.maximum_force} N 범위의 유한한 값을 입력하세요.
        </p>
      )}
      <p className="message-line">
        Zero set: hip → thigh → calf. 모드 선택만으로 출력이 켜지지 않습니다.
      </p>
    </Panel>
  );
}
function traffic(history, axis) {
  const first = history[0],
    last = history.at(-1);
  if (!first || !last || last.observed_ns <= first.observed_ns) return null;
  const dt = (last.observed_ns - first.observed_ns) / 1e9;
  const read = (state, key) =>
    axis == null
      ? state.motors.reduce((sum, m) => sum + m[key], 0)
      : state.motors[axis]?.[key];
  const rx = (read(last, "rx_frames") - read(first, "rx_frames")) / dt;
  const tx = (read(last, "tx_frames") - read(first, "tx_frames")) / dt;
  return Number.isFinite(rx) && Number.isFinite(tx) && rx >= 0 && tx >= 0
    ? { rx, tx }
    : null;
}
function CanLoad({ session }) {
  const s = session.state,
    rate = traffic(session.history),
    bitrate = s?.runtime?.bitrate;
  const load =
    rate && bitrate ? (100 * 135 * (rate.rx + rate.tx)) / bitrate : null;
  return (
    <Panel eyebrow="Bus" title="CAN Load" className="bus-panel">
      <div className="can-channel-grid">
        <article className="can-channel-card">
          <header>
            <div>
              <strong>Motor CAN</strong>
              <code>{s?.runtime?.interface || "인터페이스 정보 대기"}</code>
            </div>
            <Badge>{s?.transport_running ? "online" : "waiting"}</Badge>
          </header>
          <div className="can-channel-load">
            <strong>{fmt(load, 1)}%</strong>
            <div className="progress" aria-label="설정된 CAN ID의 추정 점유율">
              <div
                className={`progress-fill ${load >= 75 ? "danger" : load >= 45 ? "warn" : ""}`}
                style={{ width: `${Math.min(100, load ?? 0)}%` }}
              />
            </div>
          </div>
          <Metrics
            className="can-channel-metrics"
            entries={[
              ["RX", `${fmt(rate?.rx, 1)} fps`],
              ["TX", `${fmt(rate?.tx, 1)} fps`],
              ["Bitrate", bitrate ? `${bitrate / 1000} kbps` : "—"],
              [
                "Total RX / TX",
                s
                  ? `${s.motors.reduce((v, m) => v + (m.rx_frames ?? 0), 0)} / ${s.motors.reduce((v, m) => v + (m.tx_frames ?? 0), 0)}`
                  : "—",
              ],
            ]}
          />
        </article>
      </div>
      <p className="message-line">
        설정된 모터 ID의 송수신 카운터와 Classic CAN 최대 스터핑 기준
        추정치입니다. 다른 노드·오류·재전송은 포함하지 않습니다.
      </p>
    </Panel>
  );
}
function Commands({ session }) {
  const s = session.state;
  return (
    <Panel
      eyebrow="Actuator Output"
      title="Applied MIT Command"
      className="policy-command-panel"
      extra={<Badge>{s?.command_sequence ? "published" : "waiting"}</Badge>}
    >
      <Metrics
        entries={[
          ["Source", "TaskController → CAN HAL"],
          ["Targets", s?.motors.filter((m) => m.command_enabled).length],
          ["Age", age(s?.command_age_ms)],
          [
            "Sequence / HAL",
            s ? `${s.command_sequence} / ${s.consumed_command_sequence}` : "—",
          ],
        ]}
      />
      <div className="table-wrap compact-table">
        <table>
          <thead>
            <tr>
              {[
                "Actuator / CAN",
                "P target [rad]",
                "V target [rad/s]",
                "KP",
                "KD",
                "Tau target [Nm]",
                "Output",
              ].map((x) => (
                <th key={x}>{x}</th>
              ))}
            </tr>
          </thead>
          <tbody>
            {(s?.motors ?? []).map((m, i) => (
              <tr key={i}>
                <td>
                  {axes[i]} <code>{hex(s.runtime?.motor_ids?.[i])}</code>
                </td>
                {["position", "velocity", "kp", "kd", "tau_ff"].map((k) => (
                  <td key={k}>
                    {s.command_sequence ? fmt(m.command?.[k], 3) : "—"}
                  </td>
                ))}
                <td>
                  <Badge>{m.command_enabled ? "ACTIVE" : "OFF"}</Badge>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      <p className="message-line">
        HAL 소비 번호는 명령 전달 상태이며, 모터의 실제 적용 완료 ACK를 뜻하지
        않습니다.
      </p>
    </Panel>
  );
}
function Robot({ session }) {
  const s = session.state;
  return (
    <Panel
      eyebrow="Joint Encoders & Model"
      title="Robot State"
      className="robot-panel"
      extra={
        <div className="panel-badges">
          <Badge>{s?.drive_state ?? "waiting"}</Badge>
          <Badge status={s?.model?.valid ? "ok" : "muted"}>
            FK {s?.model?.valid ? "valid" : "unavailable"}
          </Badge>
        </div>
      }
    >
      <div className="enabled-motors">
        {axes.map((name, i) => {
          const m = s?.motors[i],
            q = m?.position;
          return (
            <article
              className={`enabled-motor-card ${m?.enabled ? "" : "inactive"}`}
              key={name}
            >
              <div className="enabled-motor-copy">
                <Badge status={m?.enabled ? "ok" : "muted"}>
                  {hex(s?.runtime?.motor_ids?.[i])}
                </Badge>
                <strong>{name}</strong>
                <span>{m?.enabled ? "ENABLED" : "RELEASED"}</span>
              </div>
              <div
                className={`angle-dial ${Number.isFinite(q) ? "" : "muted"}`}
                style={{ "--angle": `${((q ?? 0) * 180) / Math.PI - 90}deg` }}
              >
                {Number.isFinite(q) && <span className="dial-arrow" />}
                <span className="dial-center" />
              </div>
              <Metrics
                className="enabled-motor-metrics"
                entries={[
                  ["Angle", statusValue(q, m?.position_status)],
                  [
                    "Deg",
                    Number.isFinite(q)
                      ? `${fmt((q * 180) / Math.PI, 1)}°`
                      : "미보정",
                  ],
                  [
                    "Speed [rad/s]",
                    statusValue(m?.velocity, m?.velocity_status),
                  ],
                  [
                    "Current [A]",
                    statusValue(m?.current, m?.current_status, 2),
                  ],
                ]}
              />
            </article>
          );
        })}
      </div>
      <Metrics
        className="vector-grid"
        entries={[
          ["Toe FK [m]", vector(s?.toe_position)],
          ["Zero set", s ? `${s.zero_set.axis} · ${s.zero_set.phase}` : "—"],
          [
            "Coordinate / source",
            s
              ? `${s.motors.filter((m) => m.coordinate_valid).length}/3 · ${s.observation_origin}`
              : "—",
          ],
        ]}
      />
    </Panel>
  );
}
function Actuators({ session }) {
  const { state: s, available, send } = session;
  const any = s?.motors.some((m) => m.enabled || m.command_enabled);
  return (
    <Panel
      eyebrow="Actuators"
      title="MIT"
      className="motor-panel"
      extra={
        <button
          id="allActuatorToggle"
          className={`button ${any ? "danger" : "secondary"}`}
          disabled={!available || (!any && s?.drive_state === "FAULT")}
          onClick={() =>
            send(
              any ? { type: "release" } : { type: "motor-enable", axis: -1 },
              any ? "Disable All" : "Enable All · 중립 출력",
            )
          }
        >
          {any ? "Disable All" : "Enable All"}
        </button>
      }
    >
      <div className="table-wrap">
        <table>
          <thead>
            <tr>
              {[
                "Actuator",
                "CAN",
                "State",
                "Age",
                "Temp [°C]",
                "Current [A]",
                "Speed [rad/s]",
                "Pos [rad]",
                "Encoder [rad]",
                "Actions",
              ].map((x) => (
                <th key={x}>{x}</th>
              ))}
            </tr>
          </thead>
          <tbody>
            {axes.map((name, i) => {
              const m = s?.motors[i];
              return (
                <tr key={name}>
                  <td>{name}</td>
                  <td>
                    <code>{hex(s?.runtime?.motor_ids?.[i])}</code>
                  </td>
                  <td>
                    <span
                      className={`status-dot ${m?.enabled ? "online" : m?.position_status === "stale" ? "stale" : ""}`}
                    >
                      {m?.enabled ? "ENABLED" : "RELEASED"}
                    </span>
                  </td>
                  <td>{age(m?.age_ms)}</td>
                  <td>
                    {statusValue(m?.temperature, m?.temperature_status, 0)}
                  </td>
                  <td>{statusValue(m?.current, m?.current_status, 2)}</td>
                  <td>{statusValue(m?.velocity, m?.velocity_status)}</td>
                  <td>{statusValue(m?.position, m?.position_status)}</td>
                  <td>{fmt(m?.encoder_position, 3)}</td>
                  <td>
                    <button
                      className={`button compact ${m?.enabled || m?.command_enabled ? "danger" : "secondary"}`}
                      disabled={
                        !available ||
                        (!["DISARMED", "ARMED"].includes(s?.drive_state) &&
                          !m?.enabled)
                      }
                      onClick={() =>
                        send(
                          {
                            type:
                              m.enabled || m.command_enabled
                                ? "motor-disable"
                                : "motor-enable",
                            axis: i,
                          },
                          `${name} ${m.enabled || m.command_enabled ? "Disable" : "Enable"}`,
                        )
                      }
                    >
                      {m?.enabled || m?.command_enabled ? "Disable" : "Enable"}
                    </button>
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
      <p className="message-line">
        개별 Enable은 토크 0으로 대기합니다. 실행 중 한 축을 Disable하면 모션이
        멈추고 나머지 축도 중립 출력으로 전환됩니다.
      </p>
    </Panel>
  );
}
function Nodes({ session }) {
  const s = session.state;
  return (
    <Panel eyebrow="Nodes" title="Feedback" className="node-panel">
      <div className="table-wrap">
        <table>
          <thead>
            <tr>
              {[
                "Name",
                "CAN",
                "Role",
                "Hz",
                "Last Seen",
                "RX / Invalid",
                "Status",
              ].map((x) => (
                <th key={x}>{x}</th>
              ))}
            </tr>
          </thead>
          <tbody>
            {axes.map((name, i) => {
              const m = s?.motors[i];
              return (
                <tr key={name}>
                  <td>{name}</td>
                  <td>{hex(s?.runtime?.motor_ids?.[i])}</td>
                  <td>MIT actuator</td>
                  <td>{fmt(traffic(session.history, i)?.rx, 1)}</td>
                  <td>{age(m?.age_ms)}</td>
                  <td>{m ? `${m.rx_frames} / ${m.invalid_frames}` : "—"}</td>
                  <td>
                    <Badge>
                      {m?.profile_verified
                        ? m.age_ms < 250
                          ? "online"
                          : "stale"
                        : "waiting"}
                    </Badge>
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </Panel>
  );
}
function Events({ session }) {
  return (
    <Panel eyebrow="Activity" title="Recent Commands" className="frames-panel">
      <div className="frames-list event-list">
        {session.events.length ? (
          session.events.slice(0, 12).map((e, i) => (
            <article className="frame-item" key={i}>
              <strong>
                <span>{e.status}</span>
                <code>{e.time}</code>
              </strong>
              <p title={e.text}>{e.text}</p>
            </article>
          ))
        ) : (
          <p className="message-line">아직 요청한 명령이 없습니다.</p>
        )}
      </div>
    </Panel>
  );
}
function Processes({ session }) {
  const { state: s, send } = session,
    runtime = s?.runtime,
    stopped = runtime?.state === "STOPPED";
  const outputs = s?.motors.some((m) => m.enabled || m.command_enabled);
  const available = session.connection === "connected" && runtime?.available;
  return (
    <main className="dashboard process-grid">
      <Panel
        eyebrow="Runtime"
        title="Process Status"
        className="process-panel"
        extra={<Badge>{runtime?.state ?? "unavailable"}</Badge>}
      >
        <div className="stack-actions">
          <button
            className="button primary"
            disabled={!available || !stopped}
            onClick={() =>
              send({ type: "stack", action: "start" }, "Start control stack")
            }
          >
            Start control stack
          </button>
          <button
            className="button danger"
            disabled={!available || stopped || outputs}
            title={outputs ? "먼저 Release로 모터 출력을 해제하세요." : ""}
            onClick={() =>
              send({ type: "stack", action: "stop" }, "Stop control stack")
            }
          >
            Stop control stack
          </button>
          <p className="message-line">
            핵심 프로세스는 묶음으로 시작·중지합니다. 중지 전에 Release가
            필요하며, 시작 상태는 DISARMED입니다.
          </p>
        </div>
        {!runtime?.available && (
          <p className="error-line">
            {runtime?.error ?? "현재 세션의 Daemon 관리 연결을 기다립니다."}
          </p>
        )}
        <div id="processTiles" className="process-tiles">
          {runtime?.processes?.map((p) => (
            <article
              className="process-tile"
              key={p.name}
              data-process={p.name}
            >
              <header>
                <h3>{p.name}</h3>
                <Badge>{p.running ? "online" : "STOPPED"}</Badge>
              </header>
              <Metrics
                className="process-meta"
                entries={[
                  ["Managed PID", p.pid > 0 ? p.pid : "—"],
                  [
                    "Return / signal",
                    `${p.exit_code ?? "—"} / ${p.signal ?? "—"}`,
                  ],
                  [
                    "CPU",
                    Number.isFinite(p.cpu_percent)
                      ? `${fmt(p.cpu_percent, 1)}%`
                      : "—",
                  ],
                  [
                    "Memory",
                    Number.isFinite(p.rss_mib)
                      ? `${fmt(p.rss_mib, 1)} MiB`
                      : "—",
                  ],
                  [
                    "Uptime",
                    Number.isFinite(p.uptime_sec)
                      ? `${fmt(p.uptime_sec, 1)} s`
                      : "—",
                  ],
                  ["Threads", p.threads],
                  ["Start order", p.order],
                  [
                    "Policy",
                    p.keep_on_stack_stop
                      ? "management UI"
                      : p.critical
                        ? "control stack"
                        : "independent",
                  ],
                ]}
              />
              <p
                className="command-line"
                title={[p.command, ...p.args].join(" ")}
              >
                {[p.command, ...p.args].join(" ")}
              </p>
              <p className="command-line" title={p.cwd || p.working_directory}>
                cwd: {p.cwd || p.working_directory || "Inherited"}
              </p>
              <div className="process-actions">
                <button
                  className="button secondary"
                  disabled={
                    !available || (p.critical ? !stopped : p.running || stopped)
                  }
                  onClick={() =>
                    send(
                      p.critical
                        ? { type: "stack", action: "start" }
                        : { type: "process", name: p.name, action: "start" },
                      `Start ${p.critical ? "stack" : p.name}`,
                    )
                  }
                >
                  {p.critical ? "Start stack" : "Start"}
                </button>
                <button
                  className="button danger"
                  disabled={
                    !available ||
                    (p.critical ? stopped || outputs : !p.running || stopped)
                  }
                  onClick={() =>
                    send(
                      p.critical
                        ? { type: "stack", action: "stop" }
                        : { type: "process", name: p.name, action: "stop" },
                      `Stop ${p.critical ? "stack" : p.name}`,
                    )
                  }
                >
                  {p.critical ? "Stop stack" : "Stop"}
                </button>
              </div>
            </article>
          ))}
        </div>
        <RequestStatus session={session} />
      </Panel>
    </main>
  );
}
function SharedMemory({ session }) {
  const s = session.state;
  return (
    <main className="dashboard shm-grid">
      <Panel
        eyebrow="Shared Memory"
        title="Control State SHM"
        className="shm-panel"
        extra={<Badge>{s?.ready ? "online" : "waiting"}</Badge>}
      >
        <Metrics
          className="metric-grid shm-summary-grid"
          entries={[
            ["Name", s?.session],
            ["Age", age(s?.controller_age_ms)],
            ["Controller", s?.drive_state],
            [
              "Schema",
              s?.shm ? `ABI ${s.shm.abi} · ${s.shm.bytes} bytes` : "—",
            ],
          ]}
        />
        <div className="shm-sections">
          <article className="shm-section">
            <h3>CAN / transport</h3>
            <Metrics
              className="key-value-grid"
              entries={[
                ["Interface", s?.runtime?.interface],
                ["Command sequence", s?.command_sequence],
                ["HAL consumed", s?.consumed_command_sequence],
                ["Fault", s?.transport_fault],
                ["Motor sequence", s?.shm?.motor_sequence],
                ["Control sequence", s?.shm?.control_sequence],
              ]}
            />
          </article>
          <article className="shm-section">
            <h3>Model / calculated state</h3>
            <Metrics
              className="key-value-grid"
              entries={[
                ["Valid", s?.model?.valid ? "true" : "false"],
                ["Input sample", s?.model?.input_measurement_sequence],
                ["Toe FK [m]", vector(s?.toe_position)],
                [
                  "Acceleration valid",
                  s?.model?.acceleration_valid ? "true" : "false",
                ],
                [
                  "Gravity [Nm]",
                  s?.model?.valid ? vector(s.model.gravity) : "취득 불가",
                ],
                [
                  "Acceleration [rad/s²]",
                  s?.model?.acceleration_valid
                    ? vector(s.model.acceleration)
                    : "취득 불가",
                ],
              ]}
            />
          </article>
        </div>
        <h3 className="section-title">Actuators</h3>
        <div className="shm-actuators">
          {(s?.motors ?? []).map((m, i) => (
            <article className="shm-actuator-card" key={i}>
              <header>
                <strong>{axes[i]}</strong>
                <Badge>{m.enabled ? "ACTIVE" : "OFF"}</Badge>
              </header>
              <Metrics
                className="key-value-grid"
                entries={[
                  ["CAN", hex(s.runtime?.motor_ids?.[i])],
                  ["Calibrated", String(m.coordinate_valid)],
                  [
                    "Position [rad]",
                    statusValue(m.position, m.position_status),
                  ],
                  ["Encoder [rad]", fmt(m.encoder_position, 3)],
                  ["Offset [rad]", fmt(m.position_offset, 3)],
                  [
                    "Velocity [rad/s]",
                    statusValue(m.velocity, m.velocity_status),
                  ],
                  ["Torque [Nm]", statusValue(m.torque, m.torque_status)],
                  ["Current [A]", statusValue(m.current, m.current_status)],
                  ["supported / valid", `${m.supported} / ${m.valid}`],
                  ["Sample", m.received_sequence],
                ]}
              />
            </article>
          ))}
        </div>
        <details className="json-details">
          <summary>Raw JSON</summary>
          <pre className="json-view">{JSON.stringify(s, null, 2)}</pre>
        </details>
      </Panel>
    </main>
  );
}
function App() {
  const session = useSession(),
    s = session.state;
  const [page, setPage] = useState(location.pathname),
    [plane, setPlane] = useState(0);
  useEffect(() => {
    const fn = () => setPage(location.pathname);
    addEventListener("popstate", fn);
    return () => removeEventListener("popstate", fn);
  }, []);
  useEffect(() => {
    if (s) setPlane(s.requested_tracking_plane);
  }, [s?.requested_tracking_plane]);
  useEffect(() => {
    const key = (e) => {
      if (
        e.repeat ||
        e.ctrlKey ||
        e.altKey ||
        e.metaKey ||
        e.target.closest?.('input,textarea,select,[contenteditable="true"]')
      )
        return;
      const mode = modes.find((m) => m.key === e.key.toUpperCase());
      if (mode) {
        e.preventDefault();
        session.send({ type: "mode", mode: mode.id, plane }, mode.name);
      } else if (e.key.toUpperCase() === "A")
        session.send({ type: "arm" }, "Arm");
      else if (e.key.toUpperCase() === "X")
        session.send({ type: "release" }, "Release");
      else if (["1", "2", "3"].includes(e.key)) {
        const p = Number(e.key) - 1;
        setPlane(p);
        session.send(
          { type: "mode", mode: 3, plane: p },
          `Tracking ${planes[p]}`,
        );
      }
    };
    document.addEventListener("keydown", key);
    return () => document.removeEventListener("keydown", key);
  });
  const navigate = (e, path) => {
    if (e.ctrlKey || e.metaKey) return;
    e.preventDefault();
    history.pushState({}, "", path);
    setPage(path);
    window.scrollTo(0, 0);
  };
  const title =
    page === "/processes"
      ? "Processes"
      : page === "/shm"
        ? "SHM Data"
        : "Robot State";
  useEffect(() => {
    document.title = `QHRR-05 · ${title}`;
  }, [title]);
  return (
    <>
      <a className="skip-link" href="#dashboard">
        Skip to dashboard
      </a>
      <header className="topbar">
        <div>
          <p className="eyebrow">QHRR Control</p>
          <h1>{title}</h1>
        </div>
        <div className="status-strip">
          <Badge>{s?.runtime?.interface || "CAN"}</Badge>
          <Badge
            status={session.connection === "connected" ? "ok" : "danger"}
            className="connection"
          >
            {session.connection === "connected"
              ? "서버 연결됨"
              : session.connection === "connecting"
                ? "연결 중"
                : "연결 끊김"}
          </Badge>
          <Badge status={s?.drive_state === "ACTIVE" ? "warn" : "muted"}>
            {s?.drive_state ?? "DISCONNECTED"}
          </Badge>
          {session.connection === "disconnected" && (
            <button className="button light-button" onClick={session.reconnect}>
              다시 연결
            </button>
          )}
        </div>
      </header>
      <nav className="menubar" aria-label="Dashboard views">
        {[
          ["/", "Overview"],
          ["/processes", "Processes"],
          ["/shm", "SHM Data"],
        ].map(([path, label]) => (
          <a
            key={path}
            className={`menu-link ${page === path ? "active" : ""}`}
            href={path}
            onClick={(e) => navigate(e, path)}
          >
            {label}
          </a>
        ))}
      </nav>
      {session.error && (
        <div className="notice-wrap">
          <div className="notice danger" role="alert">
            {session.error}
            <button aria-label="알림 닫기" onClick={() => session.setError("")}>
              ×
            </button>
          </div>
        </div>
      )}
      <div id="dashboard">
        {page === "/processes" ? (
          <Processes session={session} />
        ) : page === "/shm" ? (
          <SharedMemory session={session} />
        ) : (
          <main className="dashboard overview-grid">
            <StateMachine session={session} plane={plane} />
            <Commands session={session} />
            <CanLoad session={session} />
            <ModeControl session={session} plane={plane} setPlane={setPlane} />
            <Robot session={session} />
            <Actuators session={session} />
            <Nodes session={session} />
            <Events session={session} />
          </main>
        )}
      </div>
      <footer className="footer-note">
        QHRR-05 · {s?.session ?? "session pending"} · Input → Safety →
        TaskController ·{" "}
        {s?.simulation_available ? "Simulation session" : "Hardware session"}
      </footer>
    </>
  );
}

createRoot(document.getElementById("root")).render(<App />);
