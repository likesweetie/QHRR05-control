import { useEffect, useRef, useState } from "react";

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
      }
    };
    ws.onmessage = (event) => {
      if (disposed) return;
      try {
        const message = JSON.parse(event.data);
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
      setHistory([]);
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
    connection,
    events,
    history,
    latest,
    error,
    setError,
    send,
    reconnect: () => setEpoch((x) => x + 1),
    available: connection === "connected" && state?.ready,
  };
}
