import { useCallback, useEffect, useRef, useState } from "react";

export const modes = [
  { id: 0, name: "Homing", key: "H" },
  { id: 1, name: "Gravity", key: "G" },
  { id: 2, name: "Magnetic", key: "M" },
  { id: 3, name: "Tracking", key: "T" },
  { id: 4, name: "Ball", key: "B" },
  { id: 5, name: "Damping", key: "D" },
  { id: 6, name: "Zero set", key: "Z" },
];
export const modeName = (id) => modes.find((m) => m.id === id)?.name ?? "—";
export const planes = ["XY", "XZ", "YZ"];
export const fmt = (value, digits = 2) =>
  Number.isFinite(value) ? value.toFixed(digits) : "—";
export const age = (value) =>
  Number.isFinite(value)
    ? value < 1000
      ? `${Math.round(value)} ms`
      : `${fmt(value / 1000)} s`
    : "never";
export const vector = (values, digits = 3) =>
  Array.isArray(values) ? values.map((v) => fmt(v, digits)).join(", ") : "—";

export function useSession() {
  const [state, setState] = useState(null);
  const [anticogging, setAnticogging] = useState(null);
  const [rmd, setRmd] = useState(null);
  const [ilc, setIlc] = useState(null);
  const [tracking, setTracking] = useState(null);
  const [connection, setConnection] = useState("connecting");
  const [epoch, setEpoch] = useState(0);
  const [events, setEvents] = useState([]);
  const [latest, setLatest] = useState(null);
  const [error, setError] = useState("");
  const [history, setHistory] = useState([]);
  const socket = useRef(null),
    sequence = useRef(0),
    outstanding = useRef(new Map());
  const latestRef = useRef(null);
  const coggingListeners = useRef(new Set()), coggingWanted = useRef({ enabled: false });
  const trackingWanted = useRef(false);
  const setTrackingStream = useCallback((enabled) => {
    trackingWanted.current = enabled;
    if (!enabled) setTracking(null);
    if (socket.current?.readyState === WebSocket.OPEN)
      socket.current.send(JSON.stringify({ type: "tracking-subscribe", enabled }));
  }, []);
  const subscribeCogging = useCallback((listener) => {
    coggingListeners.current.add(listener);
    return () => coggingListeners.current.delete(listener);
  }, []);
  const setCoggingStream = useCallback((enabled, settings, reset = false) => {
    coggingWanted.current = { enabled, settings: settings ?? coggingWanted.current.settings };
    if (socket.current?.readyState === WebSocket.OPEN)
      socket.current.send(JSON.stringify({ type: "cogging-subscribe", ...coggingWanted.current, reset }));
  }, []);
  const record = (text, status) =>
    setEvents((old) =>
      [
        {
          text,
          status,
          time: new Date().toLocaleTimeString("en-GB", {
            hour12: false,
            timeZone: "Asia/Seoul",
          }),
        },
        ...old,
      ].slice(0, 40),
    );
  const update = (value) => {
    latestRef.current = value;
    setLatest(value);
  };
  useEffect(() => {
    setConnection("connecting");
    setState(null);
    setAnticogging(null);
    setRmd(null);
    setIlc(null);
    setTracking(null);
    setHistory([]);
    const ws = new WebSocket(
      `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`,
    );
    socket.current = ws;
    let disposed = false;
    ws.onopen = () => {
      if (!disposed) {
        setConnection("connected");
        record("제어 서버 연결", "CONNECTED");
        if (coggingWanted.current.enabled)
          ws.send(JSON.stringify({ type: "cogging-subscribe", ...coggingWanted.current }));
        if (trackingWanted.current)
          ws.send(JSON.stringify({ type: "tracking-subscribe", enabled: true }));
      }
    };
    ws.onmessage = (event) => {
      if (disposed) return;
      try {
        const message = JSON.parse(event.data);
        if (message.type === "tracking-ilc-state") {
          setIlc(message);
          const last = latestRef.current;
          if (message.available && last?.status === "approved" && last.type === "tracking-ilc" &&
              ["phase_lead_bins", "smoothing_passes", "retention"].every((key) => message[key] === last[key])) {
            update({ ...last, status: "applied" });
            record(`${last.label} · 다음 학습 갱신부터 적용`, "APPLIED");
          }
          return;
        }
        if (message.type === "tracking-analysis") {
          if (trackingWanted.current) setTracking(message);
          return;
        }
        if (message.type === "rmd-state") {
          setRmd(message);
          const last = latestRef.current;
          if (message.available && last?.status === "approved" && last.type === "rmd-damping" &&
              message.damping_constant === last.value) {
            update({ ...last, status: "applied" });
            record(`${last.label} · 반영 완료`, "APPLIED");
          }
          return;
        }
        if (message.type === "anticogging-state") {
          setAnticogging(message);
          const last = latestRef.current;
          if (message.available && last?.status === "approved" && ["anticogging-phase", "anticogging-amplitude"].includes(last.type)) {
            const amplitude = last.type === "anticogging-amplitude";
            const key = amplitude ? `amp${last.order}_motor_nm` : `phase${last.order}_rad`;
            const expected = last.data?.[`target_${key}`]?.[last.axis];
            const current = message[key]?.[last.axis], target = message[`target_${key}`]?.[last.axis];
            const near = (a, b) => Number.isFinite(a) && Number.isFinite(b) &&
              (amplitude ? Math.abs(a - b) : Math.abs(Math.atan2(Math.sin(a - b), Math.cos(a - b)))) < 1e-6;
            if (near(current, expected) && near(target, expected)) {
              update({ ...last, status: "applied" });
              record(`${last.label} · ${amplitude ? "진폭" : "위상"} 적용 완료`, "APPLIED");
            }
          }
          return;
        }
        if (["cogging-analysis", "cogging-status"].includes(message.type)) {
          for (const listener of coggingListeners.current) listener(message);
          return;
        }
        if (message.type === "state") {
          setState(message);
          setHistory((old) => [...old.slice(-199), message]);
          const last = latestRef.current;
          if (
            last?.status === "approved" &&
            ((last.type === "mode" &&
              last.motion_sequence === message.active_motion_sequence &&
              last.mode === message.mode &&
              (last.mode !== 3 || last.plane === message.tracking_plane)) ||
              (last.drive_sequence != null &&
                last.drive_sequence === message.drive_request_sequence))
          ) {
            update({ ...last, status: "applied" });
            record(`${last.label} · TaskController 반영`, "APPLIED");
          }
          return;
        }
        const request = outstanding.current.get(message.id);
        if (!request) return;
        outstanding.current.delete(message.id);
        const status =
          message.type === "ack"
            ? message.stage === "supervisor_applied"
              ? "applied"
              : "approved"
            : message.uncertain
              ? "unknown"
              : "rejected";
        update({ ...request, ...message, type: request.type, status });
        record(
          `${request.label} · ${message.type === "ack" ? (message.stage === "supervisor_applied" ? "감독 프로세스 반영" : "안전레이어 승인") : message.message}`,
          status.toUpperCase(),
        );
        setError(
          message.type === "ack"
            ? ""
            : `${message.uncertain ? "결과 미확인" : "요청 거부"}: ${message.message}`,
        );
      } catch {
        setError("서버 응답을 읽지 못했습니다.");
        ws.close();
      }
    };
    ws.onerror = () => {
      if (!disposed) setError("제어 서버에 연결할 수 없습니다.");
    };
    ws.onclose = () => {
      if (disposed) return;
      setConnection("disconnected");
      setState(null);
      setTracking(null);
      setIlc(null);
      setHistory([]);
      for (const listener of coggingListeners.current) listener({ type: "reset" });
      if (outstanding.current.size) {
        update({ ...latestRef.current, status: "unknown" });
        setError(
          "연결이 끊겨 요청 결과를 확인할 수 없습니다. 자동 재전송하지 않습니다.",
        );
      }
      outstanding.current.clear();
      record("서버 연결 끊김", "DISCONNECTED");
    };
    return () => {
      disposed = true;
      ws.close();
    };
  }, [epoch]);
  const send = (command, label) => {
    const admin = ["process", "stack"].includes(command.type);
    if (
      socket.current?.readyState !== WebSocket.OPEN ||
      (!admin && !state?.ready) ||
      outstanding.current.size >= 8
    )
      return;
    const id = ++sequence.current;
    const item = {
      ...command,
      id,
      label:
        label ??
        (command.type === "mode" ? modeName(command.mode) : command.type),
      status: "requested",
    };
    outstanding.current.set(id, item);
    update(item);
    setError("");
    record(`${item.label} · 요청 전송`, "REQUESTED");
    socket.current.send(JSON.stringify({ ...command, id }));
  };
  return {
    state,
    anticogging,
    rmd,
    ilc,
    tracking,
    setTrackingStream,
    connection,
    events,
    history,
    latest,
    error,
    setError,
    send,
    subscribeCogging,
    setCoggingStream,
    reconnect: () => setEpoch((x) => x + 1),
    available: connection === "connected" && state?.ready,
  };
}
