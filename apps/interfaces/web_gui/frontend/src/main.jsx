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
                : req.type === "rmd-damping" ? "RMD Damping Constant 반영을 확인했습니다."
                : req.type === "tracking-ilc" ? "ILC 설정 반영 완료 · 다음 학습 갱신부터 적용됩니다."
                : ["anticogging-phase", "anticogging-amplitude"].includes(req.type) ? "보상 설정 적용을 확인했습니다."
                : "TaskController 모드 반영을 확인했습니다. 동작 완료와는 구분합니다."
              : req?.status === "approved"
                ? req.type === "force"
                  ? "목표 힘 요청이 승인되었습니다."
                  : ["anticogging-phase", "anticogging-amplitude"].includes(req.type) ? "보상 설정 승인 · 적용값 이동 중"
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
function RmdDamping({ session }) {
  const status = session.rmd;
  const available = session.available && status?.available && status.enabled;
  const [draft, setDraft] = useState(""), [edited, setEdited] = useState(false);
  useEffect(() => {
    if (!edited && status?.available) setDraft(String(status.damping_constant));
  }, [status?.available, status?.damping_constant, edited]);
  const value = Number(draft);
  const valid = draft.trim() !== "" && Number.isInteger(value) && value >= 0 && value <= 100;
  const dirty = valid && value !== status?.damping_constant;
  const pending = session.latest?.type === "rmd-damping" &&
    ["requested", "approved"].includes(session.latest.status);
  useEffect(() => {
    if (edited && valid && value === status?.damping_constant) setEdited(false);
  }, [edited, valid, value, status?.damping_constant]);
  return <Panel eyebrow="RMD" title="Damping Constant" className="rmd-panel"
    extra={<Badge>{status?.available ? status.enabled ? "RMD A1" : "MIT" : "연결 대기"}</Badge>}>
    <div className="rmd-damping-control">
      <label htmlFor="rmd-damping">Damping Constant (0–100)
        <input id="rmd-damping" type="number" min="0" max="100" step="1" value={draft}
          disabled={!available} aria-invalid={edited && !valid}
          onChange={(event) => { setEdited(true); setDraft(event.target.value); }} />
      </label>
      <button id="rmd-damping-apply" type="button" className="button" disabled={!available || !dirty || pending}
        onClick={() => session.send({ type: "rmd-damping", value }, `RMD Damping Constant ${value}`)}>반영</button>
      <output id="rmd-damping-applied">현재 적용: {status?.available ? status.damping_constant : "—"}</output>
      {dirty && <Badge status="warn">미반영</Badge>}
    </div>
    <p className="message-line">입력값은 반영 버튼을 눌러야 적용됩니다. 재시작 시 YAML 값으로 돌아갑니다.</p>
    {edited && !valid && <p className="analysis-warning" role="alert">0부터 100까지의 정수를 입력하세요.</p>}
    {status?.available && !status.enabled && <p className="message-line">rmd_torque_ref 모드에서 사용할 수 있습니다.</p>}
  </Panel>;
}

function AnticoggingTuning({ session }) {
  const [phaseStep, setPhaseStep] = useState(.5);
  const status = session.anticogging;
  const available = session.available && status?.available && status.enabled;
  return <Panel eyebrow="Anticogging" title="코깅 위상 튜닝" className="anticogging-panel"
    extra={<Badge>{status?.available ? status.enabled ? "ON" : "OFF" : "연결 대기"}</Badge>}>
    <div className="phase-step-selector" role="group" aria-label="위상 스텝 선택">
      <span>스텝 [rad]</span>
      <div className="segmented">
        {[.1, .25, .5].map((value) => <button key={value} type="button" data-phase-step={value}
          aria-pressed={phaseStep === value} className={`button secondary ${phaseStep === value ? "selected" : ""}`}
          onClick={() => setPhaseStep(value)}>{value}</button>)}
      </div>
    </div>
    <div className="phase-grid">
      {axes.map((axis, index) => <div className="phase-axis" key={axis}>
        <strong>{axis}</strong>
        {[21, 36].map((order) => {
          const target = status?.[`target_phase${order}_rad`]?.[index] ?? 0;
          const applied = status?.[`phase${order}_rad`]?.[index] ?? 0;
          const step = (delta) => session.send({ type: "anticogging-phase", axis: index, order, delta },
            `${axis} ${order}차 위상 ${delta > 0 ? "+" : ""}${delta.toFixed(2)} rad`);
          return <div className="phase-control" key={order} data-axis={index} data-order={order}>
            <label>{order}차 · phase{order}_rad</label>
            <div className="phase-stepper">
              <output tabIndex={available ? 0 : -1} aria-label={`${axis} ${order}차 위상 목표`}
                onKeyDown={(event) => {
                  if (available && ["ArrowUp", "ArrowDown"].includes(event.key)) {
                    event.preventDefault(); step(event.key === "ArrowUp" ? phaseStep : -phaseStep);
                  }
                }}>{fmt(target, 2)} <small>rad</small></output>
              <div className="phase-arrows">
                <button type="button" disabled={!available} aria-label={`${axis} ${order}차 위상 ${phaseStep} rad 증가`} onClick={() => step(phaseStep)}>▲</button>
                <button type="button" disabled={!available} aria-label={`${axis} ${order}차 위상 ${phaseStep} rad 감소`} onClick={() => step(-phaseStep)}>▼</button>
              </div>
            </div>
            <small className="phase-applied">적용 {fmt(applied, 3)} rad</small>
            <label>보상 진폭 · 모터축 Nm</label>
            <div className="phase-stepper amplitude-stepper">
              <output aria-label={`${axis} ${order}차 보상 진폭`}>{fmt(status?.[`target_amp${order}_motor_nm`]?.[index] ?? 0, 2)} <small>Nm</small></output>
              <div className="phase-arrows amplitude-arrows">
                <button type="button" disabled={!available} aria-label={`${axis} ${order}차 보상 0.01 Nm 증가`}
                  onClick={() => session.send({ type: "anticogging-amplitude", axis: index, order, delta: .01 }, `${axis} ${order}차 진폭 +0.01 Nm`)}>▲</button>
                <button type="button" disabled={!available || (status?.[`target_amp${order}_motor_nm`]?.[index] ?? 0) < .01 - 1e-12}
                  aria-label={`${axis} ${order}차 보상 0.01 Nm 감소`}
                  onClick={() => session.send({ type: "anticogging-amplitude", axis: index, order, delta: -.01 }, `${axis} ${order}차 진폭 -0.01 Nm`)}>▼</button>
              </div>
            </div>
            <small className="amplitude-applied">적용 {fmt(status?.[`amp${order}_motor_nm`]?.[index] ?? 0, 3)} Nm</small>
          </div>;
        })}
        <p className="message-line">보상 계산값 {fmt(status?.compensation_output_nm?.[index], 4)} Nm · 출력축</p>
        {status?.available && !!status.clipping?.[index] && <p className="analysis-warning amplitude-clipping" role="alert">
          클리핑: {[[1, "모터축 보상 토크 한계"], [2, "보상 전류 한계"], [4, "합산 토크 한계"], [8, "CAN 토크 범위"]]
            .filter(([flag]) => status.clipping[index] & flag).map(([, label]) => label).join(" · ")} (최근 1초)
        </p>}
      </div>)}
    </div>
    <p className="message-line">진폭은 0.01 모터축 Nm씩 조절하며, 적용값을 서서히 변경합니다.</p>
    <p className="message-line">화살표 ±{phaseStep} rad · 범위 0 ≤ φ &lt; 2π · 적용 속도 {fmt(status?.phase_slew_rad_s ?? 5, 2)} rad/s · 재시작 시 YAML 위상으로 돌아갑니다.</p>
    {!status?.available ? <p className="message-line">제어기의 위상 상태를 기다립니다.</p>
      : !status.enabled && <p className="message-line">control.yaml의 anticogging.enabled를 켜면 조절할 수 있습니다.</p>}
  </Panel>;
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
                  {" "}<small>{m.command?.rmd_torque ? "RMD A1" : "MIT C0"}</small>
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
function ReactionForce({ session }) {
  const force = session.state?.reaction_force;
  const valid = force?.valid === true;
  const status = !session.state ? "연결 대기" : ({
    valid: "추정 중", inactive: "Magnetic 실행 대기", stale: "갱신 중단", invalid: "추정 무효",
  }[force?.status] ?? "추정 대기");
  let start = session.history.length;
  while (start > 0 && session.history[start - 1].reaction_force?.valid) --start;
  const samples = valid ? session.history.slice(start) : [];
  const origin = samples[0]?.observed_ns ?? 0;
  const colors = ["#2c7a61", "#bb6724", "#4c6bc0", "#9b3d64"];
  const names = ["Fx", "Fy", "Fz", "|F|"];
  const newtons = (value) => valid && Number.isFinite(value) ? `${fmt(value, 2)} N` : "—";
  return <Panel eyebrow="GM Reaction Force" title="추정 반력" className="reaction-force-panel"
    extra={<Badge status={valid ? "ok" : force?.status === "stale" ? "warn" : "muted"}>{status}</Badge>}>
    <p className="panel-note">모델 XYZ 좌표계에서 발끝에 작용하는 외력 · 표시 20 Hz · 최근 10초</p>
    <div className="metric-grid compact">
      {names.map((name, i) => <div key={name}><span>{name}</span>
        <strong id={`reaction-force-${i}`}>{newtons(i < 3 ? force?.xyz_n?.[i] : force?.magnitude_n)}</strong></div>)}
      <div><span>법선 방향 −n·F</span><strong id="reaction-force-normal">{newtons(force?.normal_n)}</strong></div>
    </div>
    {samples.length > 0 && <>
      <AnalysisChart title="반력 XYZ / 크기" xLabel="수집 구간 경과 [s]" yLabel="힘 [N]"
        traces={names.map((name, i) => ({ name: `reaction-${name}`, color: colors[i],
          points: samples.map((row) => ({ x: (row.observed_ns - origin) / 1e9,
            y: i < 3 ? row.reaction_force.xyz_n[i] : row.reaction_force.magnitude_n })) }))} />
      <p className="tracking-legend">{names.map((name, i) => <span key={name} style={{ color: colors[i] }}>━ {name}</span>)}</p>
    </>}
    <p className="panel-note">법선 방향 힘은 접촉 추종·해제 중에만 표시합니다. 추정 무효·갱신 중단 시 이전 값은 표시하지 않습니다.</p>
  </Panel>;
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
                  ["C3 position [rad]", m.zero_completed_sequence ? fmt(m.zero_applied_position, 3) : "—"],
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
function AnalysisChart({ traces, title, xLabel, yLabel, positive = false, spatial = false }) {
  const width = 600, height = 250, left = 66, right = 18, top = 25, bottom = 42;
  const points = traces.flatMap((trace) => trace.points);
  let xmin = spatial && points.length ? Math.min(...points.map((p) => p.x)) : 0;
  let xmax = Math.max(xmin + .001, ...points.map((p) => p.x));
  if (spatial) { const padding = Math.max(.001, (xmax - xmin) * .08); xmin -= padding; xmax += padding; }
  let ymin = positive ? 0 : Math.min(...(spatial && points.length ? [] : [0]), ...points.map((p) => p.y));
  let ymax = Math.max(...(spatial && points.length ? [] : [0]), ...points.map((p) => p.y));
  if (ymax - ymin < 1e-9) { ymax += .01; if (!positive) ymin -= .01; }
  else { const padding = (ymax - ymin) * .08; ymax += padding; if (!positive) ymin -= padding; }
  const x = (v) => left + (v - xmin) / (xmax - xmin) * (width - left - right);
  const y = (v) => top + (ymax - v) / (ymax - ymin) * (height - top - bottom);
  const tick = (v) => Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 1 ? v.toFixed(1) : v.toFixed(3);
  return <figure className="analysis-chart">
    <figcaption>{title}</figcaption>
    <svg viewBox={`0 0 ${width} ${height}`} role="img" aria-label={title}>
      <title>{title}</title>
      {[0, 1, 2, 3, 4].map((i) => <g key={i} className="analysis-grid">
        <line x1={left} x2={width - right} y1={y(ymin + (ymax - ymin) * i / 4)} y2={y(ymin + (ymax - ymin) * i / 4)} />
        <text x={left - 9} y={y(ymin + (ymax - ymin) * i / 4) + 4} textAnchor="end">{tick(ymin + (ymax - ymin) * i / 4)}</text>
        <text x={x(xmin + (xmax - xmin) * i / 4)} y={height - 22} textAnchor="middle">{tick(xmin + (xmax - xmin) * i / 4)}</text>
      </g>)}
      {traces.map((trace) => <g key={trace.name}>
        <path data-series={trace.name} stroke={trace.color}
          fill="none" strokeWidth="1.7" vectorEffect="non-scaling-stroke"
          d={trace.points.map((p, i) => `${i ? "L" : "M"}${x(p.x).toFixed(2)},${y(p.y).toFixed(2)}`).join(" ")} />
        {spatial && trace.points.length > 0 && <circle
          cx={x(trace.points.at(-1).x)} cy={y(trace.points.at(-1).y)} r="3.5" fill={trace.color} />}
      </g>)}
      <text className="analysis-axis-label" x={width / 2} y={height - 3} textAnchor="middle">{xLabel}</text>
      <text className="analysis-axis-label" x="5" y="12">{yLabel}</text>
    </svg>
  </figure>;
}

function TrackingIlc({ session }) {
  const status = session.ilc;
  const available = session.available && status?.available && status.enabled;
  const fields = [
    { key: "phase_lead_bins", label: "phase_lead_bins", min: 0, max: Math.floor((status?.phase_bins ?? 0) / 4), step: 1 },
    { key: "smoothing_passes", label: "smoothing_passes", min: 0, max: 32, step: 1 },
    { key: "retention", label: "retention", min: 0, max: 1, step: .01 },
  ];
  const [draft, setDraft] = useState({ phase_lead_bins: "", smoothing_passes: "", retention: "" });
  const [edited, setEdited] = useState(false);
  useEffect(() => {
    if (!edited && status?.available) setDraft({
      phase_lead_bins: String(status.phase_lead_bins), smoothing_passes: String(status.smoothing_passes),
      retention: String(status.retention),
    });
  }, [edited, status?.available, status?.phase_lead_bins, status?.smoothing_passes, status?.retention]);
  const values = Object.fromEntries(fields.map(({ key }) => [key, Number(draft[key])]));
  const valid = fields.every(({ key, min, max }) => draft[key].trim() !== "" &&
    Number.isFinite(values[key]) && values[key] <= max &&
    (key === "retention" ? values[key] > 0 : Number.isInteger(values[key]) && values[key] >= min));
  const dirty = valid && fields.some(({ key }) => values[key] !== status?.[key]);
  const pending = session.latest?.type === "tracking-ilc" && ["requested", "approved"].includes(session.latest.status);
  useEffect(() => {
    if (edited && valid && !dirty) setEdited(false);
  }, [edited, valid, dirty]);
  return <Panel eyebrow="Tracking ILC" title="ILC 파라미터" className="ilc-panel"
    extra={<Badge status={available ? "ok" : "muted"}>{status?.available ? status.enabled ? "ILC ON" : "ILC OFF" : "연결 대기"}</Badge>}>
    <div className="ilc-controls">
      {fields.map(({ key, label, min, max, step }) => <label key={key} htmlFor={`ilc-${key}`}>
        {label} <span>({key === "retention" ? "0 초과~1" : `${min}~${max}`})</span>
        <input id={`ilc-${key}`} type="number" min={min} max={max} step={step} value={draft[key]}
          disabled={!available} aria-invalid={edited && !valid}
          onChange={(event) => { const value = event.target.value; setEdited(true); setDraft((old) => ({ ...old, [key]: value })); }} />
        <output id={`ilc-applied-${key}`}>현재 설정: {status?.available ? status[key] : "—"}</output>
      </label>)}
      <button id="ilc-apply" type="button" className="button" disabled={!available || !dirty || pending}
        onClick={() => session.send({ type: "tracking-ilc", ...values }, "ILC 파라미터 반영")}>반영</button>
    </div>
    <p className="message-line">반영 후 다음 학습 갱신부터 적용됩니다. 기존 학습값은 유지하며, 재시작 시 YAML 값으로 돌아갑니다.</p>
    {dirty && <Badge status="warn">미반영</Badge>}
    {edited && !valid && <p className="analysis-warning" role="alert">위상 선행과 필터 반복은 표시 범위의 정수, retention은 0 초과 1 이하로 입력하세요.</p>}
    {status?.available && !status.enabled && <p className="message-line">tracking.ilc.enabled가 꺼져 있습니다.</p>}
  </Panel>;
}

function TrackingError({ session }) {
  const { tracking: data, setTrackingStream } = session;
  useEffect(() => {
    setTrackingStream(true);
    return () => setTrackingStream(false);
  }, [setTrackingStream]);
  const valid = data?.active && !data.stale && data.latest;
  const status = data?.stale ? "피드백 수집 중단" : data?.problem || (valid ? "수집 중" : "트래킹 실행 대기");
  const mm = (value) => valid ? `${fmt(value, 2)} mm` : "—";
  const plane = planes[data?.plane] ?? "XY";
  const coordinates = { XY: [0, 1], XZ: [0, 2], YZ: [1, 2] }[plane];
  const wave = data?.wave ?? [];
  const colors = ["#2c7a61", "#bb6724", "#4c6bc0", "#9b3d64"];
  return <Panel eyebrow="Tracking Error" title="궤적 추종 오차" className="tracking-error-panel"
    extra={<Badge status={valid ? "ok" : data?.stale ? "warn" : "muted"}>{status}</Badge>}>
    <p className="panel-note">목표 궤적 − FK(실제 관절각) · 수집 500 Hz / 표시 4 Hz · 최근 10초 · 단위 mm</p>
    <div className="metric-grid compact tracking-error-summary">
      <div><span>현재 3D 오차</span><strong id="tracking-error-norm">{mm(data?.latest?.norm_mm)}</strong></div>
      <div><span>3D RMS</span><strong id="tracking-error-rms">{mm(data?.norm_rms_mm)}</strong></div>
      <div><span>최대 3D 오차</span><strong id="tracking-error-max">{mm(data?.max_norm_mm)}</strong></div>
      <div><span>수집 구간 / 표본</span><strong>{data?.count ? `${fmt(data.span_seconds, 1)} s / ${data.count}` : "—"}</strong></div>
    </div>
    <Metrics entries={["X", "Y", "Z"].map((axis, index) =>
      [`${axis} 오차 / RMS`, valid ? `${fmt(data.latest.error_mm[index], 2)} / ${fmt(data.rms_mm[index], 2)} mm` : "—"])} />
    {!!wave.length && <div className={`tracking-error-charts${valid ? "" : " stale"}`}>
      <div>
        <AnalysisChart title={`목표 / 실제 궤적 · ${plane} 평면`} spatial
          xLabel={`${plane[0]} [mm]`} yLabel={`${plane[1]} [mm]`} traces={[
            { name: "tracking-target", color: colors[3], points: wave.map((row) => ({ x: row[4 + coordinates[0]], y: row[4 + coordinates[1]] })) },
            { name: "tracking-actual", color: colors[0], points: wave.map((row) => ({ x: row[1 + coordinates[0]], y: row[1 + coordinates[1]] })) },
          ]} />
        <p className="tracking-legend"><span style={{ color: colors[3] }}>━ 목표</span><span style={{ color: colors[0] }}>━ 실제 FK</span></p>
      </div>
      <div>
        <AnalysisChart title="XYZ 오차 / 3D 크기" xLabel="수집 구간 경과 [s]" yLabel="오차 [mm]"
          traces={["X", "Y", "Z", "3D"].map((name, index) => ({ name: `tracking-error-${name}`,
            color: colors[index], points: wave.map((row) => ({ x: row[0], y: row[7 + index] })) }))} />
        <p className="tracking-legend">{["X", "Y", "Z", "3D"].map((name, index) => <span key={name} style={{ color: colors[index] }}>━ {name}</span>)}</p>
      </div>
    </div>}
    <p className="panel-note">{valid ? `실제 XYZ [${vector(data.latest.actual_mm, 2)}] · 목표 XYZ [${vector(data.latest.reference_mm, 2)}] mm` :
      "트래킹이 실행되고 보정된 관절 피드백이 들어오면 표시합니다."}</p>
  </Panel>;
}

const CoggingIdentification = React.memo(function CoggingIdentification({ connection, subscribeCogging, setCoggingStream }) {
  const [running, setRunning] = useState(false), [results, setResults] = useState([]);
  const [axis, setAxis] = useState("all"), [size, setSize] = useState(1024);
  const [lpf, setLpf] = useState(true), [cutoff, setCutoff] = useState(10);
  const [maxOrder, setMaxOrder] = useState(100), [gearRatio, setGearRatio] = useState(9);
  const [revolutions, setRevolutions] = useState(1);
  const [stale, setStale] = useState(false), [readMisses, setReadMisses] = useState(0);
  useEffect(() => subscribeCogging((message) => {
    if (message.type === "reset") {
      setResults([]); setStale(true);
    } else if (message.type === "cogging-analysis") {
      setResults(message.results.map((result) => ({ ...result,
        derivedSpectrum: result.derivedSpectrum?.map(([amplitude, sine, cosine], i) => ({ order: i * result.resolution, amplitude, sine, cosine })),
        directSpectrum: result.directSpectrum?.map(([amplitude, sine, cosine], i) => ({ order: i * result.resolution, amplitude, sine, cosine })),
      }))); setStale(message.stale);
      setReadMisses(message.read_misses ?? 0);
    }
  }), [subscribeCogging]);
  useEffect(() => {
    setCoggingStream(running, { size, lpf, cutoff, maxOrder, gearRatio, revolutions });
  }, [running, size, lpf, cutoff, maxOrder, gearRatio, revolutions, setCoggingStream]);
  useEffect(() => () => setCoggingStream(false), [setCoggingStream]);
  const clear = () => {
    setResults([]);
    setCoggingStream(running, { size, lpf, cutoff, maxOrder, gearRatio, revolutions }, true);
  };
  const download = () => {
    const lines = ["axis,motor_order_cycles_per_rev,q_difference_amplitude_output_rad_s,dq_amplitude_output_rad_s,q_difference_sine_output_rad_s,q_difference_cosine_output_rad_s,dq_sine_output_rad_s,dq_cosine_output_rad_s"];
    results.forEach((result, i) => result.derivedSpectrum?.forEach((point, k) =>
      lines.push([axes[i], point.order, point.amplitude, result.directSpectrum?.[k]?.amplitude ?? "",
        point.sine, point.cosine, result.directSpectrum?.[k]?.sine ?? "",
        result.directSpectrum?.[k]?.cosine ?? ""].join(","))));
    const url = URL.createObjectURL(new Blob([lines.join("\n")], { type: "text/csv;charset=utf-8" }));
    const link = document.createElement("a"); link.href = url; link.download = "cogging-spectrum.csv"; link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  return <main className="dashboard cogging-grid">
    <Panel eyebrow="Speed ripple analysis" title="측정 설정" extra={<Badge status={running && !stale ? "ok" : "muted"}>{running ? stale ? "새 샘플 대기" : "측정 중" : "일시정지"}</Badge>}>
      <p className="analysis-intro">θ_motor = 감속비 × MIT q를 기준으로 속도 리플을 균일한 모터 각도 간격에 보간한 뒤 FFT를 계산합니다. q 차분과 dq를 같은 각도 구간에서 비교합니다.</p>
      <div className="analysis-controls">
        <label>표시 축<select value={axis} onChange={(e) => setAxis(e.target.value)}><option value="all">전체 3축</option>{axes.map((a, i) => <option key={a} value={i}>{a}</option>)}</select></label>
        <label>회전 구간의 FFT 점 수<select value={size} onChange={(e) => setSize(Number(e.target.value))}>{[128, 256, 512, 1024, 2048, 4096].map((n) => <option key={n}>{n}</option>)}</select></label>
        <label>표시 상한 [차수]<input type="number" min="1" max="2048" value={maxOrder} onChange={(e) => setMaxOrder(Math.max(1, Math.min(2048, Number(e.target.value) || 1)))} /></label>
        <label>감속비 · θ_motor = q × 비율<input type="number" min="0.01" max="1000" step="0.1" value={gearRatio} onChange={(e) => setGearRatio(Math.max(.01, Math.min(1000, Number(e.target.value) || 1)))} /></label>
        <label>분석 구간 · 출력축 이동각<select value={revolutions} onChange={(e) => setRevolutions(Number(e.target.value))}>{[.125, .25, .5, 1, 2, 4, 8, 16].map((n) => <option key={n} value={n}>{n} 모터 회전 · 출력축 {fmt(360 * n / gearRatio, 1)}°</option>)}</select></label>
        <label className="analysis-check"><input type="checkbox" checked={lpf} onChange={(e) => setLpf(e.target.checked)} /> q 차분에 1차 LPF</label>
        <label>LPF 차단 주파수 [Hz]<input type="number" min="0.1" max="249" step="0.1" disabled={!lpf} value={cutoff} onChange={(e) => setCutoff(Math.max(.1, Math.min(249, Number(e.target.value) || .1)))} /></label>
      </div>
      <div className="analysis-actions">
        <button className={`button ${running ? "secondary" : "primary"}`} disabled={connection !== "connected"} onClick={() => setRunning((v) => !v)}>{running ? "측정 일시정지" : "측정 시작"}</button>
        <button className="button secondary" onClick={clear}>버퍼 초기화</button>
        <button className="button secondary" disabled={!results.some((r) => r.ready)} onClick={download}>스펙트럼 CSV</button>
      </div>
      <div className="analysis-legend"><span><i style={{ background: "#6650d9" }} /> q 차분{lpf ? ` · ${cutoff} Hz LPF` : " · 필터 OFF"}</span><span><i style={{ background: "#008b8b" }} /> 기존 dq 피드백</span></div>
      <p className="message-line">서버 수집 500 Hz · 각도 FFT / 그래프 4 Hz · 유효 차수는 원본 각도 간격으로 제한 · 읽기 잠금 건너뜀 {readMisses}회</p>
      {stale && <p className="analysis-warning" role="status">새로운 C0 피드백이 없습니다. 연결과 모터 피드백 상태를 확인하세요.</p>}
    </Panel>
    {axes.map((label, i) => {
      if (axis !== "all" && Number(axis) !== i) return null;
      const result = results[i] ?? { ready: false, progress: 0, count: 0, revolutions, resolution: 1 / revolutions };
      const maximum = Math.min(maxOrder, result.nyquistOrder ?? maxOrder);
      const peaks = [
        ...(result.derivedPeaks ?? []).map((p) => ({ ...p, source: "q 차분" })),
        ...(result.directPeaks ?? []).map((p) => ({ ...p, source: "dq" })),
      ];
      return <Panel key={label} eyebrow={`Axis ${i + 1}`} title={label} className="cogging-axis" extra={<Badge>{result.ready ? result.holding ? "최근 유효 구간" : "FFT ready" : "버퍼 수집"}</Badge>}>
        <Metrics entries={[
          ["실제 수신률", result.rate ? `${fmt(result.rate, 1)} Hz` : "—"],
          ["유효 최대 차수", result.nyquistOrder ? `${fmt(result.nyquistOrder, 1)} 차` : "—"],
          ["차수 간격", result.resolution ? `${fmt(result.resolution, 3)} 차` : "—"],
          ["각도 구간 / 소요 시간", result.ready ? `${result.revolutions} rev / ${fmt(result.duration, 2)} s` : `${fmt(result.availableRevolutions ?? 0, 2)} / ${result.revolutions} rev`],
          ["q 차분 / dq 평균", result.ready ? `${fmt(result.derivedMean)} / ${fmt(result.directMean)} rad/s` : "—"],
          ["q 차분 / dq 리플 RMS", result.ready ? `${fmt(result.derivedRms, 4)} / ${fmt(result.directRms, 4)}` : "—"],
          ["구간 내 건너뛴 C0", result.missing ?? "—"],
          ["방향 / 원본 각도 샘플", result.ready ? `${result.direction > 0 ? "정방향" : "역방향"} / ${result.sourceSamples}` : "—"],
        ]} />
        {!result.ready ? <div className="analysis-wait" role="status"><p>{result.problem || `${Math.round(100 * result.progress)}% · 각도 구간을 채우는 중`}</p><progress value={result.progress} max="1" /></div> : <>
          {(result.missing > 0 || result.jitter > 20) && <p className="analysis-warning">샘플 누락 또는 시각 지터가 있습니다. 각도 보간 결과와 함께 수신 상태를 확인하세요.</p>}
          {result.holding && <p className="message-line">정지·방향 전환 후에는 마지막 완료 구간을 유지합니다. 새 방향의 분석 구간이 채워지면 갱신합니다.</p>}
          {!result.hasDirect && <p className="analysis-warning">이 구간에는 유효한 dq가 없어 위치 차분만 표시합니다.</p>}
          <div className="analysis-chart-grid">
            <AnalysisChart title="각도별 속도 리플 · 평균 제거" xLabel="구간 시작으로부터 모터 회전 [rev]" yLabel="출력축 rad/s" traces={[
              { name: "q-difference", color: "#6650d9", points: result.wave.map((p) => ({ x: p[0], y: p[1] })) },
              ...(result.hasDirect ? [{ name: "dq-feedback", color: "#008b8b", points: result.wave.map((p) => ({ x: p[0], y: p[2] })) }] : []),
            ]} />
            <AnalysisChart title="각도 FFT · 단측 진폭" xLabel="모터 회전 차수 [cycles/rev]" yLabel="출력축 rad/s · peak" positive traces={[
              { name: "q-difference", color: "#6650d9", points: result.derivedSpectrum.filter((p) => p.order > 0 && p.order <= maximum).map((p) => ({ x: p.order, y: p.amplitude })) },
              ...(result.directSpectrum ? [{ name: "dq-feedback", color: "#008b8b", points: result.directSpectrum.filter((p) => p.order > 0 && p.order <= maximum).map((p) => ({ x: p.order, y: p.amplitude })) }] : []),
            ]} />
          </div>
          <div className="table-scroll"><table className="analysis-peaks"><caption>모터 각도 기준 리플 성분 · 감속비 {result.gearRatio}</caption><thead><tr><th>신호</th><th>차수 [cycles/rev]</th><th>진폭 [출력축 rad/s]</th><th>sin 계수 [rad/s]</th><th>cos 계수 [rad/s]</th></tr></thead><tbody>{peaks.map((p) => <tr key={`${p.source}-${p.order}`}><td>{p.source}</td><td>{fmt(p.order, 3)}</td><td>{fmt(p.amplitude, 5)}</td><td>{fmt(p.sine, 5)}</td><td>{fmt(p.cosine, 5)}</td></tr>)}</tbody></table></div>
          <p className="message-line">각도 원점은 현재 MIT q=0입니다. θ=감속비×q [rad], Δωₙ(θ)=sin계수×sin(nθ)+cos계수×cos(nθ). 두 계수와 진폭의 단위는 출력축 rad/s입니다.{lpf ? " q 차분 계수에는 LPF의 위상 지연이 포함됩니다." : ""}</p>
          <p className="message-line">1차는 모터 1회전당 1주기입니다. 구간 {result.revolutions}회전의 차수 간격은 {fmt(result.resolution, 2)}차입니다. 짧은 각도 구간은 인접 차수를 구분하는 분해능이 낮아집니다.</p>
        </>}
      </Panel>;
    })}
  </main>;
});

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
      if (page === "/cogging") return;
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
        : page === "/cogging"
          ? "Cogging Identification"
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
          ["/cogging", "Cogging Identification"],
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
        ) : page === "/cogging" ? (
          <CoggingIdentification connection={session.connection} subscribeCogging={session.subscribeCogging} setCoggingStream={session.setCoggingStream} />
        ) : (
          <main className="dashboard overview-grid">
            <StateMachine session={session} plane={plane} />
            <RmdDamping session={session} />
            <AnticoggingTuning session={session} />
            <Commands session={session} />
            <CanLoad session={session} />
            <ModeControl session={session} plane={plane} setPlane={setPlane} />
            <Robot session={session} />
            <ReactionForce session={session} />
            <TrackingIlc session={session} />
            <TrackingError session={session} />
            <Actuators session={session} />
            <Nodes session={session} />
            <Events session={session} />
          </main>
        )}
      </div>
      <footer className="footer-note">
        QHRR-05 · {s?.session ?? "session pending"} · {page === "/cogging" ? "Server FFT → web plots" : "Input → Safety → TaskController"} ·{" "}
        {s?.simulation_available ? "Simulation session" : "Hardware session"}
      </footer>
    </>
  );
}

createRoot(document.getElementById("root")).render(<App />);
