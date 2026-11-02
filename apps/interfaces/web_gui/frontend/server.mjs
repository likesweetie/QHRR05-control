import { spawn } from "node:child_process";
import { createServer } from "node:http";
import { readFile } from "node:fs/promises";
import { createInterface } from "node:readline";
import { dirname, extname, isAbsolute, relative, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { WebSocket, WebSocketServer } from "ws";
import { sendInputRequest } from "./input-client.mjs";
import { Supervisor } from "./supervisor.mjs";
import { createCapture, appendSamples, analyseAxis, spectralPeaks } from "./cogging-analysis.mjs";

const here = dirname(fileURLToPath(import.meta.url));
const root = resolve(here, "../../../..");
const options = {
  port: "8080",
  "shm-name": "/qhrr05",
  bridge: resolve(root, "build/web_gui_bridge"),
  "control-config": resolve(root, "config/control/control.yaml"),
  "input-socket": "",
};

for (let i = 2; i < process.argv.length; i += 2) {
  const name = process.argv[i].replace(/^--/, "");
  if (
    !process.argv[i].startsWith("--") ||
    !(name in options) ||
    !process.argv[i + 1]
  ) {
    console.error(
      "usage: node server.mjs [--port N] [--shm-name /NAME] [--bridge PATH] [--control-config PATH] [--input-socket PATH]",
    );
    process.exit(1);
  }
  options[name] = process.argv[i + 1];
}

const port = Number(options.port);
const inputSocket =
  options["input-socket"] ||
  `/tmp/qhrr05-input-${process.getuid()}/${options["shm-name"].slice(1)}.sock`;
if (!Number.isInteger(port) || port < 0 || port > 65535) {
  console.error("[web_gui] port must be an integer in [0, 65535]");
  process.exit(1);
}

const dist = resolve(here, "dist");
const supervisor = new Supervisor(options["shm-name"]);
supervisor.poll();
const supervisorTimer = setInterval(() => supervisor.poll(), 1000);
const clients = new Set();
const coggingClients = new Map();
const trackingClients = new Set();
let trackingStreaming = false;
let trackingCapture = { rows: [], active: false, problem: "트래킹 실행 대기", lastData: 0 };
const updateTrackingSubscription = () => {
  const enabled = trackingClients.size > 0;
  if (enabled !== trackingStreaming) {
    trackingStreaming = enabled;
    trackingCapture = { rows: [], active: false, problem: "트래킹 실행 대기", lastData: 0 };
    child.stdin.write(`tracking ${enabled ? 1 : 0}\n`);
  }
};
let coggingStreaming = false;
const updateCoggingSubscription = () => {
  const enabled = [...coggingClients.values()].some((entry) => entry.enabled);
  if (enabled !== coggingStreaming) {
    coggingStreaming = enabled;
    child.stdin.write(`cogging ${enabled ? 1 : 0}\n`);
  }
};
const pending = new Map();
let state = null;
let nextId = 0;
let shuttingDown = false;
let started = false;
let adminBusy = false;
const child = spawn(
  options.bridge,
  [options["shm-name"], options["control-config"]],
  { stdio: ["pipe", "pipe", "pipe"] },
);
child.stderr.pipe(process.stderr);
const wss = new WebSocketServer({ noServer: true, maxPayload: 4096 });
const allowedHost = (host) =>
  ["127.0.0.1", "localhost"].some(
    (h) => host === `${h}:${server.address()?.port}`,
  );
const send = (client, message) => {
  if (client.readyState !== WebSocket.OPEN) return;
  if (client.bufferedAmount > 262144) {
    client.close(1013, "telemetry consumer is too slow");
    return;
  }
  client.send(JSON.stringify(message));
};
let anticoggingState = null, phasePolling = false;
let rmdState = null, rmdPolling = false;
let ilcState = null, ilcPolling = false;
const publishIlc = (data) => {
  if (typeof data?.enabled !== "boolean" || !Number.isFinite(data.observed_ns) ||
      !Number.isInteger(data.phase_bins) || !Number.isInteger(data.phase_lead_bins) ||
      !Number.isInteger(data.smoothing_passes) || !Number.isFinite(data.retention))
    throw new Error("Invalid tracking ILC status from controller");
  if (ilcState?.available && data.observed_ns < ilcState.observed_ns) return;
  ilcState = { ...data, type: "tracking-ilc-state", available: true };
  for (const client of clients) send(client, ilcState);
};
const ilcTimer = setInterval(async () => {
  if (ilcPolling || !state?.ready || clients.size === 0 || pending.size >= 8) return;
  ilcPolling = true;
  try {
    const reply = await sendInputRequest(inputSocket, ++nextId, "tracking-ilc-status");
    if (reply.type !== "ack") throw new Error(reply.message);
    publishIlc(reply.data);
  } catch (error) {
    ilcState = { ...ilcState, type: "tracking-ilc-state", available: false, message: error.message };
    for (const client of clients) send(client, ilcState);
  } finally { ilcPolling = false; }
}, 250);
const publishRmd = (data) => {
  if (typeof data?.enabled !== "boolean" || !Number.isFinite(data.observed_ns) ||
      !Number.isInteger(data.damping_constant) || data.damping_constant < 0 || data.damping_constant > 100)
    throw new Error("Invalid RMD damping status from controller");
  if (rmdState?.available && data.observed_ns < rmdState.observed_ns) return;
  rmdState = { ...data, type: "rmd-state", available: true };
  for (const client of clients) send(client, rmdState);
};
const rmdTimer = setInterval(async () => {
  if (rmdPolling || !state?.ready || clients.size === 0 || pending.size >= 8) return;
  rmdPolling = true;
  try {
    const reply = await sendInputRequest(inputSocket, ++nextId, "rmd-status");
    if (reply.type !== "ack") throw new Error(reply.message);
    publishRmd(reply.data);
  } catch (error) {
    rmdState = { ...rmdState, type: "rmd-state", available: false, message: error.message };
    for (const client of clients) send(client, rmdState);
  } finally { rmdPolling = false; }
}, 250);
const publishPhases = (data) => {
  if (typeof data?.enabled !== "boolean" || !Number.isFinite(data.observed_ns) ||
      !Number.isFinite(data.phase_slew_rad_s) ||
      ["phase21_rad", "phase36_rad", "target_phase21_rad", "target_phase36_rad",
        "amp21_motor_nm", "amp36_motor_nm", "target_amp21_motor_nm", "target_amp36_motor_nm",
        "clipping", "compensation_output_nm"].some((key) =>
        !Array.isArray(data[key]) || data[key].length !== 3 || !data[key].every(Number.isFinite)))
    throw new Error("Invalid anticogging phase status from controller");
  if (anticoggingState?.available && data.observed_ns < anticoggingState.observed_ns) return;
  anticoggingState = { ...data, type: "anticogging-state", available: true };
  for (const client of clients) send(client, anticoggingState);
};
const phaseTimer = setInterval(async () => {
  if (phasePolling || !state?.ready || clients.size === 0 || pending.size >= 8) return;
  phasePolling = true;
  try {
    const reply = await sendInputRequest(inputSocket, ++nextId, "anticogging-status");
    if (reply.type !== "ack") throw new Error(reply.message);
    publishPhases(reply.data);
  } catch (error) {
    anticoggingState = { ...anticoggingState, type: "anticogging-state", available: false, message: error.message };
    for (const client of clients) send(client, anticoggingState);
  } finally { phasePolling = false; }
}, 250);
const coggingTimer = setInterval(() => {
  for (const [client, entry] of coggingClients) {
    if (!entry.enabled && !entry.dirty) continue;
    const results = entry.capture.axes.map((samples) => {
      let result;
      try { result = analyseAxis(samples, entry.settings); }
      catch (error) { return { ready: false, count: samples.length, progress: 0, problem: error.message }; }
      if (!result.ready) return result;
      const maximum = Math.min(entry.settings.maxOrder, result.nyquistOrder);
      const step = Math.max(1, Math.ceil(result.derived.length / 384));
      const wave = [];
      for (let i = 0; i < result.derived.length; i += step)
        wave.push([i * result.angleStepRev, result.derived[i] - result.derivedMean,
          result.direct ? result.direct[i] - result.directMean : null]);
      return { ...result, derived: undefined, direct: undefined, wave, hasDirect: !!result.direct,
        derivedSpectrum: result.derivedSpectrum.filter((p) => p.order <= maximum)
          .map((p) => [p.amplitude, p.sine, p.cosine].map((v) => Number(v.toPrecision(8)))),
        directSpectrum: result.directSpectrum?.filter((p) => p.order <= maximum)
          .map((p) => [p.amplitude, p.sine, p.cosine].map((v) => Number(v.toPrecision(8)))) ?? null,
        derivedPeaks: spectralPeaks(result.derivedSpectrum, maximum, 3),
        directPeaks: spectralPeaks(result.directSpectrum, maximum, 3) };
    });
    send(client, { type: "cogging-analysis", results, read_hz: 500, plot_hz: 4,
      read_misses: entry.readMisses, stale: entry.enabled && Date.now() - entry.lastData > 2000 });
    entry.dirty = false;
  }
}, 250);
// All statistics use the full 500 Hz capture. Only the plotted rows are reduced.
const trackingTimer = setInterval(() => {
  if (!trackingClients.size) return;
  const capture = trackingCapture, rows = capture.rows;
  const stale = capture.active && Date.now() - capture.lastData > 500;
  const latest = rows.at(-1);
  const sum = [0, 0, 0];
  let max = 0;
  for (const row of rows) {
    for (let i = 0; i < 3; ++i) sum[i] += row[7 + i] ** 2;
    max = Math.max(max, row[10]);
  }
  const rms = sum.map((value) => rows.length ? Math.sqrt(value / rows.length) : null);
  const indices = new Set();
  const step = Math.max(1, Math.ceil(rows.length / 240));
  for (let i = 0; i < rows.length; i += step) {
    indices.add(i);
    // Preserve each bucket's largest error even if it occurs between plotted points.
    let peak = i;
    for (let j = i + 1; j < Math.min(i + step, rows.length); ++j)
      if (rows[j][10] > rows[peak][10]) peak = j;
    indices.add(peak);
  }
  if (rows.length) indices.add(rows.length - 1);
  const message = {
    type: "tracking-analysis", read_hz: 500, plot_hz: 4, window_seconds: 10,
    active: capture.active, stale, problem: capture.problem, plane: capture.plane,
    motion_sequence: capture.motion_sequence, read_misses: capture.read_misses ?? 0,
    count: rows.length, span_seconds: rows.length > 1 ? (latest[0] - rows[0][0]) / 1000 : 0,
    latest: latest ? { actual_mm: latest.slice(1, 4), reference_mm: latest.slice(4, 7),
      error_mm: latest.slice(7, 10), norm_mm: latest[10] } : null,
    rms_mm: rms, norm_rms_mm: rows.length ? Math.hypot(...rms) : null,
    max_norm_mm: rows.length ? max : null,
    // [seconds, actual XYZ mm, target XYZ mm, error XYZ mm, error norm mm]
    wave: [...indices].sort((a, b) => a - b).map((index) =>
      [(rows[index][0] - rows[0][0]) / 1000, ...rows[index].slice(1)]),
  };
  for (const client of trackingClients) send(client, message);
}, 250);
const reject = (client, id, message, uncertain = false) =>
  send(client, { type: "error", id, message, uncertain });

const server = createServer(async (request, response) => {
  if (!allowedHost(request.headers.host)) {
    response.writeHead(403).end("Invalid host");
    return;
  }
  if (request.method !== "GET" && request.method !== "HEAD") {
    response.writeHead(405, { Allow: "GET, HEAD" }).end();
    return;
  }
  try {
    const url = new URL(request.url, "http://localhost");
    const path = decodeURIComponent(url.pathname);
    const file = resolve(
      dist,
      `.${["/", "/processes", "/shm", "/cogging"].includes(path) ? "/index.html" : path}`,
    );
    const childPath = relative(dist, file);
    if (childPath.startsWith("..") || isAbsolute(childPath)) {
      response.writeHead(403).end("Invalid path");
      return;
    }
    const bytes = await readFile(file);
    const types = {
      ".html": "text/html; charset=utf-8",
      ".js": "text/javascript; charset=utf-8",
      ".css": "text/css; charset=utf-8",
      ".svg": "image/svg+xml",
    };
    response.writeHead(200, {
      "Content-Type": types[extname(file)] || "application/octet-stream",
      "Cache-Control": "no-store",
      "X-Content-Type-Options": "nosniff",
    });
    response.end(request.method === "HEAD" ? undefined : bytes);
  } catch (error) {
    const status =
      error.code === "ENOENT" || error.code === "EISDIR" ? 404 : 400;
    response
      .writeHead(status)
      .end(
        status === 404
          ? "Asset not found. Run npm run build."
          : "Invalid request",
      );
  }
});

server.on("upgrade", (request, socket, head) => {
  const origins = [
    `http://127.0.0.1:${server.address()?.port}`,
    `http://localhost:${server.address()?.port}`,
    "http://127.0.0.1:5173",
    "http://localhost:5173",
  ];
  if (
    !allowedHost(request.headers.host) ||
    request.url !== "/ws" ||
    !origins.includes(request.headers.origin) ||
    clients.size >= 8
  ) {
    socket.end("HTTP/1.1 403 Forbidden\r\nConnection: close\r\n\r\n");
    return;
  }
  wss.handleUpgrade(request, socket, head, (client) =>
    wss.emit("connection", client),
  );
});

wss.on("connection", (client) => {
  clients.add(client);
  if (state) send(client, state);
  if (anticoggingState) send(client, anticoggingState);
  if (rmdState) send(client, rmdState);
  if (ilcState) send(client, ilcState);
  client.on("error", (error) =>
    console.error("[web_gui] WebSocket:", error.message),
  );
  client.on("close", () => {
    clients.delete(client);
    if (trackingClients.delete(client) && !shuttingDown)
      updateTrackingSubscription();
    if (coggingClients.delete(client) && !shuttingDown)
      updateCoggingSubscription();
    for (const [id, entry] of pending) {
      if (entry.client === client) {
        pending.delete(id);
      }
    }
  });
  client.on("message", (bytes, binary) => {
    let request;
    try {
      if (binary) throw new Error("JSON text messages are required");
      request = JSON.parse(bytes.toString());
      if (request?.type === "tracking-subscribe") {
        if (typeof request.enabled !== "boolean" ||
            Object.keys(request).some((key) => !["type", "enabled"].includes(key)))
          throw new Error("Invalid read-only tracking subscription");
        if (request.enabled) trackingClients.add(client);
        else trackingClients.delete(client);
        updateTrackingSubscription();
        return;
      }
      if (request?.type === "cogging-subscribe") {
        if (typeof request.enabled !== "boolean" ||
            Object.keys(request).some((key) => !["type", "enabled", "settings", "reset"].includes(key)) ||
            (request.reset != null && typeof request.reset !== "boolean"))
          throw new Error("Invalid read-only cogging subscription");
        const entry = coggingClients.get(client) ?? { capture: createCapture(), lastData: 0,
          readMisses: 0, settings: { size: 1024, lpf: true, cutoff: 10, maxOrder: 100, gearRatio: 9, revolutions: 1 } };
        if (request.settings != null) {
          if (typeof request.settings !== "object" || Array.isArray(request.settings) ||
              Object.keys(request.settings).some((key) => !["size", "lpf", "cutoff", "maxOrder", "gearRatio", "revolutions"].includes(key)))
            throw new Error("Invalid FFT settings");
          const settings = { ...entry.settings, ...request.settings };
          if (![128, 256, 512, 1024, 2048, 4096].includes(settings.size) ||
              typeof settings.lpf !== "boolean" || !Number.isFinite(settings.cutoff) ||
              settings.cutoff < .1 || settings.cutoff >= 250 ||
              !Number.isFinite(settings.maxOrder) || settings.maxOrder < 1 || settings.maxOrder > 2048 ||
              !Number.isFinite(settings.gearRatio) || settings.gearRatio < .01 || settings.gearRatio > 1000 ||
              ![.125, .25, .5, 1, 2, 4, 8, 16].includes(settings.revolutions))
            throw new Error("FFT settings are out of range");
          entry.settings = settings;
        }
        if (request.reset) { entry.capture = createCapture(); entry.lastData = 0; }
        entry.enabled = request.enabled; entry.dirty = true;
        coggingClients.set(client, entry);
        updateCoggingSubscription();
        send(client, { type: "cogging-status", enabled: request.enabled, read_hz: 500 });
        return;
      }
      if (
        !request ||
        !Number.isSafeInteger(request.id) ||
        request.id < 1 ||
        request.id > 2147483647
      )
        throw new Error("A positive integer request id is required");
      const allowed =
        request.type === "mode"
          ? ["id", "type", "mode", "plane"]
          : request.type === "tracking-ilc"
            ? ["id", "type", "phase_lead_bins", "smoothing_passes", "retention"]
          : ["anticogging-phase", "anticogging-amplitude"].includes(request.type)
            ? ["id", "type", "axis", "order", "delta"]
          : ["force", "rmd-damping"].includes(request.type)
            ? ["id", "type", "value"]
            : ["motor-enable", "motor-disable"].includes(request.type)
              ? ["id", "type", "axis"]
              : request.type === "process"
                ? ["id", "type", "name", "action"]
                : request.type === "stack"
                  ? ["id", "type", "action"]
                  : ["id", "type"];
      if (Object.keys(request).some((key) => !allowed.includes(key)))
        throw new Error("Unexpected command fields");
      if (["process", "stack"].includes(request.type)) {
        if (!["start", "stop"].includes(request.action))
          throw new Error("Invalid process action");
        if (
          request.type === "process" &&
          (typeof request.name !== "string" ||
            !/^[\w.-]{1,80}$/.test(request.name))
        )
          throw new Error("Invalid managed process name");
        if (!supervisor.state.available)
          throw new Error("Daemon supervisor is unavailable");
        if (adminBusy)
          throw new Error("A supervisor command is already pending");
        adminBusy = true;
        const command =
          request.type === "stack"
            ? `stack ${request.action}`
            : `process ${request.name} ${request.action}`;
        supervisor
          .request(command)
          .then((reply) =>
            send(client, {
              ...reply,
              id: request.id,
              stage: "supervisor_applied",
            }),
          )
          .catch((error) =>
            reject(client, request.id, error.message, error.uncertain),
          )
          .finally(() => {
            adminBusy = false;
          });
        return;
      }
      if (request.type === "tracking-ilc") {
        if (!ilcState?.available || !ilcState.enabled)
          throw new Error("Tracking ILC is unavailable or disabled in YAML");
        if (!Number.isInteger(request.phase_lead_bins) || request.phase_lead_bins < 0 ||
            request.phase_lead_bins > Math.floor(ilcState.phase_bins / 4) ||
            !Number.isInteger(request.smoothing_passes) || request.smoothing_passes < 0 ||
            request.smoothing_passes > 32 || !Number.isFinite(request.retention) ||
            request.retention <= 0 || request.retention > 1)
          throw new Error("ILC requires phase_lead_bins in [0, phase_bins/4], smoothing_passes in [0, 32], retention in (0, 1]");
      } else if (request.type === "rmd-damping") {
        if (!Number.isInteger(request.value) || request.value < 0 || request.value > 100)
          throw new Error("RMD damping constant must be an integer in [0, 100]");
      } else if (request.type === "anticogging-phase") {
        if (!Number.isInteger(request.axis) || request.axis < 0 || request.axis > 2 ||
            ![21, 36].includes(request.order) || ![-.5, -.25, -.1, .1, .25, .5].includes(request.delta))
          throw new Error("Expected axis 0..2, order 21/36 and phase step +/-0.1, +/-0.25 or +/-0.5 rad");
      } else if (request.type === "anticogging-amplitude") {
        if (!Number.isInteger(request.axis) || request.axis < 0 || request.axis > 2 ||
            ![21, 36].includes(request.order) || ![-.01, .01].includes(request.delta))
          throw new Error("Expected axis 0..2, order 21/36 and amplitude step +/-0.01 motor Nm");
      } else if (["motor-enable", "motor-disable"].includes(request.type)) {
        if (
          !Number.isInteger(request.axis) ||
          request.axis < -1 ||
          request.axis > 2
        )
          throw new Error("Invalid motor axis");
      } else if (request.type === "mode") {
        if (
          !Number.isInteger(request.mode) ||
          request.mode < 0 ||
          request.mode > 6 ||
          !Number.isInteger(request.plane) ||
          request.plane < 0 ||
          request.plane > 2
        )
          throw new Error("Mode or tracking plane is invalid");
      } else if (request.type === "force") {
        if (
          typeof request.value !== "number" ||
          !Number.isFinite(request.value) ||
          request.value < 0 ||
          !state ||
          request.value > state.maximum_force
        )
          throw new Error("Target force is outside configured limits");
      } else if (!["arm", "release", "clear-fault"].includes(request.type))
        throw new Error("Unsupported command");
      if (!state?.ready) throw new Error("Control session is not ready");
      if (pending.size >= 8) throw new Error("Command queue is full");
      if (
        [...pending.values()].some(
          (entry) => entry.client === client && entry.id === request.id,
        )
      )
        throw new Error("Request id is already pending");
      const id = ++nextId;
      pending.set(id, { client, id: request.id });
      const command =
        request.type === "mode"
          ? `mode ${request.mode} ${request.plane}`
          : request.type === "tracking-ilc"
            ? `tracking-ilc ${request.phase_lead_bins} ${request.smoothing_passes} ${request.retention}`
          : ["anticogging-phase", "anticogging-amplitude"].includes(request.type)
            ? `${request.type} ${request.axis} ${request.order} ${request.delta}`
          : ["force", "rmd-damping"].includes(request.type)
            ? `${request.type} ${request.value}`
            : ["motor-enable", "motor-disable"].includes(request.type)
              ? `${request.type} ${request.axis}`
              : request.type;
      sendInputRequest(inputSocket, id, command)
        .then((reply) => {
          if (["anticogging-phase", "anticogging-amplitude"].includes(request.type) && reply.type === "ack") publishPhases(reply.data);
          if (request.type === "rmd-damping" && reply.type === "ack") publishRmd(reply.data);
          if (request.type === "tracking-ilc" && reply.type === "ack") publishIlc(reply.data);
          send(client, { ...reply, id: request.id });
        })
        .catch((error) =>
          reject(client, request.id, error.message, error.uncertain),
        )
        .finally(() => pending.delete(id));
    } catch (error) {
      reject(
        client,
        Number.isSafeInteger(request?.id) ? request.id : null,
        error.message,
      );
    }
  });
});

function stop(code, message) {
  if (shuttingDown) return;
  shuttingDown = true;
  if (message) console.error(`[web_gui] ${message}`);
  clearTimeout(startup);
  clearInterval(supervisorTimer);
  clearInterval(coggingTimer);
  clearInterval(trackingTimer);
  clearInterval(phaseTimer);
  clearInterval(rmdTimer);
  clearInterval(ilcTimer);
  supervisor.close();
  pending.clear();
  for (const client of clients) client.terminate();
  child.stdin.end();
  child.kill("SIGTERM");
  wss.close();
  server.close();
  process.exitCode = code;
}

server.on("error", (error) =>
  stop(
    1,
    error.code === "EADDRINUSE"
      ? `Port ${port} is already in use. Stop the previous web GUI or explicitly select another port with --port. No port was changed automatically.`
      : error.message,
  ),
);
child.on("error", (error) => stop(1, `Bridge failed: ${error.message}`));
child.stdin.on("error", (error) =>
  stop(1, `Command pipe failed: ${error.message}`),
);
child.on("exit", (code, signal) => {
  if (!shuttingDown)
    stop(
      code === 0 ? 0 : 1,
      `Bridge exited (${signal || code}); session disconnected`,
    );
});
const startup = setTimeout(
  () => stop(1, "No initial SHM snapshot received"),
  5000,
);
createInterface({ input: child.stdout }).on("line", (line) => {
  try {
    const message = JSON.parse(line);
    if (message.type === "state") {
      state = {
        ...message,
        session: options["shm-name"],
        runtime: supervisor.state,
      };
      for (const client of clients) send(client, state);
      if (!started) {
        started = true;
        clearTimeout(startup);
        server.listen(port, "127.0.0.1", () => {
          console.log(
            `[web_gui] http://127.0.0.1:${server.address().port} | SHM ${options["shm-name"]}`,
          );
        });
      }
    } else if (message.type === "tracking-samples") {
      if (!trackingStreaming) return;
      const old = trackingCapture;
      const reset = !message.active || !old.active || old.generation !== message.generation ||
        old.motion_sequence !== message.motion_sequence || old.plane !== message.plane;
      trackingCapture = { ...message, rows: reset ? [] : old.rows,
        lastData: reset ? 0 : old.lastData };
      if (message.active) {
        for (const row of message.samples) {
          if (!Array.isArray(row) || row.length !== 7 || !row.every(Number.isFinite)) continue;
          const previous = trackingCapture.rows.at(-1);
          if (previous && row[0] <= previous[0]) continue;
          const values = row.slice(1).map((value) => value * 1000);
          const error = values.slice(3).map((value, axis) => value - values[axis]);
          trackingCapture.rows.push([row[0], ...values, ...error, Math.hypot(...error)]);
          trackingCapture.lastData = Date.now();
        }
      }
      const rows = trackingCapture.rows, cutoff = (rows.at(-1)?.[0] ?? 0) - 10000;
      let start = Math.max(0, rows.length - 6000);
      while (start < rows.length && rows[start][0] < cutoff) ++start;
      if (start) rows.splice(0, start);
      delete trackingCapture.samples;
    } else if (message.type === "cogging-samples") {
      for (const entry of coggingClients.values()) {
        if (!entry.enabled) continue;
        if (appendSamples(entry.capture, message.samples)) entry.lastData = Date.now();
        entry.readMisses = message.read_misses ?? 0;
      }
    } else throw new Error("Telemetry bridge returned a non-state message");
  } catch (error) {
    stop(1, `Invalid bridge message: ${error.message}`);
  }
});
process.on("SIGINT", () => stop(0));
process.on("SIGTERM", () => stop(0));
