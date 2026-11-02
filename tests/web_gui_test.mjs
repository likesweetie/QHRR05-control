import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import { once } from "node:events";
import { access, mkdtemp, readFile, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { createConnection, createServer } from "node:net";
import { createRequire } from "node:module";
import { get } from "node:http";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";

const root = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const { WebSocket } = createRequire(
  resolve(root, "apps/interfaces/web_gui/frontend/package.json"),
)("ws");
const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
async function waitFor(predicate, detail = () => "", timeout = 5000) {
  const deadline = Date.now() + timeout;
  while (!predicate()) {
    if (Date.now() > deadline) throw new Error(`Timed out: ${detail()}`);
    await delay(10);
  }
  return predicate();
}
function start(command, args, cwd = root) {
  const child = spawn(command, args, {
    cwd,
    stdio: ["pipe", "pipe", "pipe"],
  });
  child.logs = "";
  child.stdout.on("data", (bytes) => {
    child.logs += bytes;
  });
  child.stderr.on("data", (bytes) => {
    child.logs += bytes;
  });
  child.on("error", (error) => {
    child.logs += error.message;
  });
  child.done = once(child, "exit");
  return child;
}
async function stop(child) {
  if (!child || child.exitCode !== null || child.signalCode !== null) return;
  child.kill("SIGTERM");
  await child.done;
}
async function inputRequest(path, line) {
  return new Promise((resolve, reject) => {
    const socket = createConnection(path);
    let bytes = "";
    socket.setTimeout(2000, () => socket.destroy(new Error("Input timeout")));
    socket.on("connect", () => socket.write(line + "\n"));
    socket.on("error", reject);
    socket.on("data", (chunk) => {
      bytes += chunk;
      if (bytes.includes("\n")) {
        socket.destroy();
        resolve(JSON.parse(bytes.trim()));
      }
    });
  });
}

test("Read-only GUI shows 500 Hz FK tracking error and GM reaction force telemetry", { timeout: 20000 }, async () => {
  const directory = await mkdtemp(resolve(tmpdir(), "qhrr-tracking-web-"));
  const config = resolve(directory, "control.yaml");
  await writeFile(config, (await readFile(resolve(root, "tests/fixtures/control.yaml"), "utf8"))
    .replace(/^model_path:.*$/m, `model_path: ${JSON.stringify(resolve(root, "models/qhrr-05/qhrr-05.xml"))}`));
  const name = `/qhrr05-tracking-web-${process.pid}`;
  let fixture, server, ws;
  try {
    fixture = start(resolve(root, "build/web_gui_fixture"), [name, config]);
    await waitFor(() => fixture.logs.includes("ready"), () => fixture.logs);
    server = start(process.execPath, [resolve(root, "apps/interfaces/web_gui/frontend/server.mjs"),
      "--port", "0", "--shm-name", name, "--control-config", config,
      "--bridge", resolve(root, "build/web_gui_bridge")]);
    const url = await waitFor(() => server.logs.match(/http:\/\/127\.0\.0\.1:\d+/)?.[0], () => server.logs);
    let state, analysis;
    const raw = [], errors = [];
    ws = new WebSocket(url.replace("http:", "ws:") + "/ws", { origin: url });
    ws.on("message", (bytes) => {
      const message = JSON.parse(bytes);
      if (message.type === "state") state = message;
      if (message.type === "tracking-analysis") analysis = message;
      if (message.type === "tracking-samples") raw.push(message);
      if (message.type === "error") errors.push(message);
    });
    await once(ws, "open");
    await waitFor(() => state?.ready);
    const drive = state.drive_request_sequence, motion = state.active_motion_sequence;
    ws.send(JSON.stringify({ type: "tracking-subscribe", enabled: true }));
    await waitFor(() => analysis && !analysis.active);
    assert.equal(analysis.latest, null);
    assert.equal(state.drive_request_sequence, drive);
    assert.equal(state.active_motion_sequence, motion);
    ws.send(JSON.stringify({ type: "tracking-subscribe", enabled: true, mode: 3 }));
    await waitFor(() => errors.length);
    assert.match(errors[0].message, /read-only tracking/);

    fixture.stdin.write("tracking\n");
    await waitFor(() => analysis?.active && analysis.count === 1);
    assert.equal(analysis.read_hz, 500);
    assert.equal(analysis.plot_hz, 4);
    for (const [axis, value] of [3, -4, 12].entries()) {
      assert.ok(Math.abs(analysis.latest.error_mm[axis] - value) < 1e-8);
      assert.ok(Math.abs(analysis.rms_mm[axis] - Math.abs(value)) < 1e-8);
    }
    assert.ok(Math.abs(analysis.latest.norm_mm - 13) < 1e-8);
    assert.ok(Math.abs(analysis.norm_rms_mm - 13) < 1e-8);
    assert.ok(analysis.latest.actual_mm.every((value) => Math.abs(value) < 1000), "used poisoned SHM model position");
    for (let i = 0; i < 700; ++i) {
      fixture.stdin.write("tracking-sample\n");
      await delay(3);
    }
    await waitFor(() => analysis.count > 500);
    assert.ok(analysis.wave.length < analysis.count, "browser received the full high-rate capture");
    assert.ok(Math.abs(analysis.max_norm_mm - 13) < 1e-8);
    assert.equal(raw.length, 0);
    await waitFor(() => analysis.stale);
    const count = analysis.count;
    await delay(350);
    assert.equal(analysis.count, count, "duplicate feedback was counted as a fresh sample");
    const sequence = analysis.motion_sequence;
    fixture.stdin.write("tracking-reset\n");
    await waitFor(() => analysis.motion_sequence > sequence && analysis.count === 1);
    assert.ok(Math.abs(analysis.norm_rms_mm - 10) < 1e-8, "old trajectory contaminated the new RMS");
    fixture.stdin.write("tracking-invalid\n");
    await waitFor(() => !analysis.active && analysis.problem.includes("관절"));
    assert.equal(analysis.latest, null);
    fixture.stdin.write("tracking\ntracking-checking\n");
    await waitFor(() => analysis.problem.includes("검사"));
    fixture.stdin.write("tracking-stop\n");
    await waitFor(() => analysis.problem.includes("실행 대기"));
    assert.equal(analysis.count, 0);
    // Reuse the read-only bridge session to verify existing GM telemetry, including signs.
    fixture.stdin.write("reaction\n");
    await waitFor(() => state?.reaction_force?.valid);
    assert.deepEqual(state.reaction_force.xyz_n, [-3, 4, 0]);
    assert.equal(state.reaction_force.magnitude_n, 5);
    assert.equal(state.reaction_force.normal_n, 3);
    const forceSequence = state.shm.control_sequence;
    await delay(100);
    assert.equal(state.shm.control_sequence, forceSequence, "force display wrote to SHM");
    fixture.stdin.write("reaction-free\n");
    await waitFor(() => state.magnetic_phase === "FREE");
    assert.equal(state.reaction_force.valid, true);
    assert.equal(state.reaction_force.normal_n, null, "free-space magnitude mislabeled as normal force");
    for (const [command, status] of [["reaction-invalid", "invalid"], ["reaction-nan", "invalid"],
      ["reaction-inactive", "inactive"], ["reaction-disarmed", "inactive"]]) {
      const sequence = state.shm.control_sequence;
      fixture.stdin.write(command + "\n");
      await waitFor(() => state.shm.control_sequence > sequence && state.reaction_force.status === status);
      assert.equal(state.reaction_force.valid, false);
      assert.deepEqual(state.reaction_force.xyz_n, [null, null, null]);
      assert.equal(state.reaction_force.magnitude_n, null);
      assert.equal(state.reaction_force.normal_n, null);
    }
    fixture.stdin.write("reaction\n");
    await waitFor(() => state.reaction_force.valid);
    await waitFor(() => state.reaction_force.status === "stale");
    assert.equal(state.reaction_force.valid, false);
    assert.equal(state.reaction_force.magnitude_n, null);
    ws.close();
  } catch (error) {
    error.message += `\nFIXTURE:\n${fixture?.logs.slice(-2000)}\nSERVER:\n${server?.logs}`;
    throw error;
  } finally {
    ws?.terminate();
    await stop(server);
    if (fixture) { fixture.stdin.end(); await fixture.done; }
    await rm(directory, { recursive: true, force: true });
  }
});

test(
  "WebSocket → Input → safety → real TaskController; telemetry is read-only",
  { timeout: 25000 },
  async () => {
    const name = `/qhrr05-web-test-${process.pid}`;
    const endpoint = `/tmp/qhrr05-input-${process.getuid()}/${name.slice(1)}.sock`;
    let fixture, controller, server, ws, bridge;
    const phaseDirectory = await mkdtemp(resolve(tmpdir(), "qhrr-phase-web-"));
    const phaseConfig = resolve(phaseDirectory, "control.yaml");
    const fixtureConfig = (await readFile(resolve(root, "tests/fixtures/control.yaml"), "utf8"))
      .replace(/(  ilc:[^\n]*\n    enabled:) false/, "$1 true")
      .replace(/^model_path:.*$/m, `model_path: ${JSON.stringify(resolve(root, "models/qhrr-05/qhrr-05.xml"))}`);
    await writeFile(phaseConfig, fixtureConfig + `\nanticogging:\n  enabled: true\n  motor_config: ${JSON.stringify(resolve(root, "config/motor/ak10_9_v3.yaml"))}\n  scale: 0.1\n`);
    try {
      fixture = start(resolve(root, "build/web_gui_fixture"), [name]);
      await waitFor(
        () => fixture.logs.includes("ready"),
        () => fixture.logs,
      );
      controller = start(resolve(root, "build/control"), [
        phaseConfig,
        name,
      ]);
      await waitFor(
        () => controller.logs.includes("DAMPING"),
        () => controller.logs,
      );
      server = start(
        resolve(root, "scripts/run_web_gui.sh"),
        [
          "--port",
          "0",
          "--shm-name",
          name,
          "--control-config",
          phaseConfig,
        ],
        "/tmp",
      );
      await waitFor(
        () => server.logs.match(/http:\/\/127\.0\.0\.1:\d+/),
        () => server.logs,
      );
      const url = server.logs.match(/http:\/\/127\.0\.0\.1:\d+/)[0];
      const page = await fetch(url);
      assert.equal(page.status, 200);
      assert.match(await page.text(), /QHRR-05/);
      const denied = await new Promise((resolve, reject) => {
        get(url, { headers: { Host: "untrusted.example" } }, (response) => {
          response.resume();
          resolve(response.statusCode);
        }).on("error", reject);
      });
      assert.equal(denied, 403);

      let state, coggingStatus, phases, rmd, ilc;
      const coggingResults = [], rawBatches = [];
      const responses = new Map();
      ws = new WebSocket(url.replace("http:", "ws:") + "/ws", { origin: url });
      ws.on("message", (bytes) => {
        const message = JSON.parse(bytes);
        if (message.type === "state") state = message;
        else if (message.type === "anticogging-state") phases = message;
        else if (message.type === "rmd-state") rmd = message;
        else if (message.type === "tracking-ilc-state") ilc = message;
        else if (message.type === "cogging-status") coggingStatus = message;
        else if (message.type === "cogging-analysis") coggingResults.push(message);
        else if (message.type === "cogging-samples") rawBatches.push(message);
        else responses.set(message.id, message);
      });
      await once(ws, "open");
      await waitFor(() => state?.ready);
      assert.equal(state.mode, 5);
      assert.ok(Math.abs(state.motors[1].position - 0.5) < 1e-12);
      assert.equal(state.motors[0].current, 1.5);
      const driveRequest = state.drive_request_sequence, motionRequest = state.active_motion_sequence;
      ws.send(JSON.stringify({ type: "cogging-subscribe", enabled: true }));
      await waitFor(() => coggingStatus?.enabled && coggingResults.some((b) => b.results.every((r) => r.count === 1)));
      assert.equal(coggingStatus.read_hz, 500);
      assert.equal(rawBatches.length, 0, "raw 500 Hz samples were forwarded to the browser");
      assert.equal(state.drive_request_sequence, driveRequest, "read-only subscription issued a drive command");
      assert.equal(state.active_motion_sequence, motionRequest, "read-only subscription changed motion");
      ws.send(JSON.stringify({ type: "cogging-subscribe", enabled: false }));
      await waitFor(() => coggingStatus?.enabled === false);
      const request = async (command) => {
        ws.send(JSON.stringify(command));
        return waitFor(() => responses.get(command.id));
      };

      await waitFor(() => ilc?.available && ilc.enabled);
      assert.equal(ilc.phase_bins, 128);
      assert.equal(ilc.phase_lead_bins, 1);
      assert.equal(ilc.smoothing_passes, 2);
      assert.equal(ilc.retention, 1);
      const parameters = { phase_lead_bins: 3, smoothing_passes: 5, retention: .85 };
      assert.equal((await request({ id: 950, type: "tracking-ilc", ...parameters })).type, "ack");
      await waitFor(() => Object.keys(parameters).every((key) => ilc[key] === parameters[key]));
      for (const [index, invalid] of [
        { phase_lead_bins: -1 }, { phase_lead_bins: 33 }, { phase_lead_bins: 1.5 },
        { smoothing_passes: 33 }, { smoothing_passes: -.1 }, { retention: 0 },
        { retention: 1.01 }, { retention: "0.5" }, { retention: null },
      ].entries())
        assert.equal((await request({ id: 951 + index, type: "tracking-ilc", ...parameters, ...invalid })).type, "error");
      for (const line of ["970 tracking-ilc -1 5 0.85", "971 tracking-ilc 33 5 0.85",
        "972 tracking-ilc 3 33 0.85", "973 tracking-ilc 3 5 0", "974 tracking-ilc 1.5 5 0.85",
        "975 tracking-ilc 3 5 nan", "976 tracking-ilc 3 5 0.85 extra"])
        assert.equal((await inputRequest(endpoint, line)).type, "error");
      const ilcRead = (await inputRequest(endpoint, "977 tracking-ilc-status")).data;
      for (const key of Object.keys(parameters)) assert.equal(ilcRead[key], parameters[key]);
      assert.equal(state.active_motion_sequence, motionRequest);
      assert.equal(state.drive_request_sequence, driveRequest);

      await waitFor(() => rmd?.available);
      assert.equal(rmd.damping_constant, 5);
      const dampingReply = await request({ id: 901, type: "rmd-damping", value: 9 });
      assert.equal(dampingReply.type, "ack");
      await waitFor(() => rmd.damping_constant === 9);
      for (const [index, value] of [-1, 101, 5.5, "5", null].entries())
        assert.equal((await request({ id: 902 + index, type: "rmd-damping", value })).type, "error");
      for (const line of ["910 rmd-damping -1", "911 rmd-damping 101", "912 rmd-damping 5.5"])
        assert.equal((await inputRequest(endpoint, line)).type, "error");
      assert.equal((await inputRequest(endpoint, "913 rmd-status")).data.damping_constant, 9);
      assert.equal(state.active_motion_sequence, motionRequest);
      assert.equal(state.drive_request_sequence, driveRequest);

      await waitFor(() => phases?.available && phases.enabled);
      assert.deepEqual(phases.target_phase21_rad, [0, 0, 0]);
      assert.equal(phases.phase_slew_rad_s, 5);
      await Promise.all([request({ id: 21, type: "anticogging-phase", axis: 0, order: 21, delta: .5 }),
        request({ id: 22, type: "anticogging-phase", axis: 0, order: 21, delta: .5 })]);
      await waitFor(() => Math.abs(phases.phase21_rad[0] - 1) < 1e-9);
      assert.deepEqual(phases.phase36_rad, [0, 0, 0]);
      const anti = (phase) => -.1 * 9 * .05 * (Math.sin(21 * 9 * .1 + phase) + Math.sin(36 * 9 * .1));
      await waitFor(() => Math.abs(state.motors[0].command.tau_ff - anti(1)) < 1e-9);
      assert.equal(state.active_motion_sequence, motionRequest);
      assert.equal(state.drive_request_sequence, driveRequest);
      for (const line of ["201 anticogging-phase -1 21 0.5", "202 anticogging-phase 0 22 0.5",
          "203 anticogging-phase 0 21 0.2", "204 anticogging-phase 0 21 nan", "206 anticogging-phase 0 21 0.05"]) {
        const invalid = await inputRequest(endpoint, line);
        assert.equal(invalid.type, "error");
      }
      const phaseRead = await inputRequest(endpoint, "205 anticogging-status");
      assert.equal(phaseRead.data.target_phase21_rad[0], 1);
      let invalidPhase = await request({ id: 23, type: "anticogging-phase", axis: 2, order: 36, delta: 1 });
      assert.equal(invalidPhase.type, "error");
      await request({ id: 24, type: "anticogging-phase", axis: 1, order: 36, delta: -.5 });
      await waitFor(() => Math.abs(phases.phase36_rad[1] - (2 * Math.PI - .5)) < 1e-9);
      assert.deepEqual(phases.phase21_rad, [1, 0, 0]);

      let selectedPhase = 0;
      for (const [index, delta] of [.1, .25, -.25, -.1].entries()) {
        const phaseReply = await request({ id: 30 + index, type: "anticogging-phase", axis: 2, order: 21, delta });
        assert.equal(phaseReply.type, "ack", `step ${delta} was rejected`);
        selectedPhase += delta;
        await waitFor(() => Math.abs(Math.atan2(Math.sin(phases.phase21_rad[2] - selectedPhase),
          Math.cos(phases.phase21_rad[2] - selectedPhase))) < 1e-9);
      }

      assert.deepEqual(phases.target_amp36_motor_nm, [.05, .05, .05]);
      for (const [id, delta, expected] of [[40, .01, .06], [41, -.01, .05]]) {
        const amplitudeReply = await request({ id, type: "anticogging-amplitude", axis: 2, order: 36, delta });
        assert.equal(amplitudeReply.type, "ack");
        await waitFor(() => Math.abs(phases.amp36_motor_nm[2] - expected) < 1e-9);
      }
      const invalidAmplitude = await request({ id: 42, type: "anticogging-amplitude", axis: 0, order: 21, delta: .1 });
      assert.equal(invalidAmplitude.type, "error");
      assert.equal((await inputRequest(endpoint, "208 anticogging-amplitude 0 21 0.1")).type, "error");
      let reply = await request({ id: 1, type: "mode", mode: 1, plane: 0 });
      assert.equal(reply.type, "ack");
      assert.equal(reply.stage, "safety_approved");
      await waitFor(
        () =>
          state.mode === 1 &&
          state.active_motion_sequence === reply.motion_sequence,
      );
      const beforePhasePosition = state.motors[0].command.position;
      const phaseKp = state.motors[0].command.kp;
      await request({ id: 25, type: "anticogging-phase", axis: 0, order: 21, delta: -.5 });
      await waitFor(() => Math.abs(phases.phase21_rad[0] - .5) < 1e-9);
      await waitFor(() => Math.abs(state.motors[0].command.position - beforePhasePosition -
        (anti(.5) - anti(1)) / phaseKp) < 1e-9);
      assert.equal(state.motors[0].command.tau_ff, 0, "position mode leaked phase compensation into native FF");
      const force = Math.min(4, state.maximum_force);
      reply = await request({ id: 2, type: "force", value: force });
      assert.equal(reply.type, "ack");
      await waitFor(() => state.target_force === force);

      reply = await request({ id: 3, type: "force", value: -1 });
      assert.equal(reply.type, "error");
      assert.equal(state.target_force, force);
      const sequence = state.motion_sequence;
      reply = await inputRequest(endpoint, "100 mode 99 0");
      assert.equal(reply.type, "error");
      assert.match(reply.message, /safety rejected/);
      reply = await inputRequest(
        endpoint,
        `101 force ${state.maximum_force + 1}`,
      );
      assert.equal(reply.type, "error");
      reply = await inputRequest(endpoint, "102 mode 1 7");
      assert.equal(reply.type, "error");
      reply = await inputRequest(endpoint, "103 force nan");
      assert.equal(reply.type, "error");
      reply = await inputRequest(endpoint, "104 mode 1 0 unexpected");
      assert.equal(reply.type, "error");
      await delay(75);
      assert.equal(state.motion_sequence, sequence);
      assert.equal(state.mode, 1);
      assert.equal(state.target_force, force);

      fixture.stdin.write("gui-homing\n");
      await waitFor(() => state.mode === 0);
      fixture.stdin.write("gui-invalid\n");
      await delay(75);
      assert.equal(
        state.requested_mode,
        0,
        "rejected request was retained for later execution",
      );
      await waitFor(() =>
        controller.logs.includes("safety rejected request: invalid mode"),
      );
      assert.equal(state.mode, 0, "native GUI request bypassed safety");
      fixture.stdin.write("gui-homing\n");
      await waitFor(() => state.requested_mode === 0);
      fixture.stdin.write("joystick-invalid\n");
      await waitFor(() =>
        controller.logs.includes(
          "safety rejected request: invalid joystick axis",
        ),
      );
      assert.equal(
        state.mode,
        0,
        "invalid joystick input changed the approved mode",
      );
      fixture.stdin.write("joystick-disconnect\n");
      await waitFor(() => fixture.logs.includes("joystick-disconnect"));

      bridge = start(resolve(root, "build/web_gui_bridge"), [
        name,
        phaseConfig,
      ]);
      await waitFor(() => bridge.logs.includes('"type":"state"'));
      bridge.stdin.write("900 mode 1 0\n");
      assert.equal((await bridge.done)[0], 1);
      assert.match(bridge.logs, /read-only/);
      assert.equal(state.mode, 0, "telemetry bridge wrote a command");

      await stop(controller);
      await assert.rejects(access(endpoint));
      const oldSequence = state.motion_sequence;
      reply = await request({ id: 4, type: "mode", mode: 5, plane: 0 });
      assert.equal(reply.type, "error");
      assert.match(reply.message, /Input interface unavailable/);
      await delay(75);
      assert.equal(
        state.motion_sequence,
        oldSequence,
        "server fell back to a direct SHM write",
      );
      fixture.stdin.write("stop\n");
      await waitFor(() => state.stopping && !state.ready);
      reply = await request({ id: 5, type: "force", value: 1 });
      assert.equal(reply.type, "error");
      assert.match(reply.message, /not ready/);
    } catch (error) {
      error.message += `\nFIXTURE:\n${fixture?.logs}\nCONTROLLER:\n${controller?.logs}\nSERVER:\n${server?.logs}`;
      throw error;
    } finally {
      ws?.terminate();
      await stop(bridge);
      await stop(server);
      await stop(controller);
      if (fixture) {
        fixture.stdin.end();
        await fixture.done;
      }
    }
    await rm(phaseDirectory, { recursive: true, force: true });
    await assert.rejects(access(`/dev/shm/${name.slice(1)}`));
    await assert.rejects(access(endpoint));
  },
);

test(
  "Missing SHM fails explicitly without serving synthetic state",
  { timeout: 8000 },
  async () => {
    const server = start(process.execPath, [
      "apps/interfaces/web_gui/frontend/server.mjs",
      "--port",
      "0",
      "--shm-name",
      `/missing-web-gui-${process.pid}`,
    ]);
    const [code] = await server.done;
    assert.equal(code, 1);
    assert.match(server.logs, /shm_open/);
    assert.doesNotMatch(server.logs, /http:\/\/127/);
  },
);

test(
  "Port collision is explicit; closing the SHM session releases the web port",
  { timeout: 12000 },
  async () => {
    const name = `/qhrr05-web-lifecycle-${process.pid}`;
    let fixture, server, duplicate;
    try {
      fixture = start(resolve(root, "build/web_gui_fixture"), [name]);
      await waitFor(
        () => fixture.logs.includes("ready"),
        () => fixture.logs,
      );
      const launch = (port) =>
        start(process.execPath, [
          "apps/interfaces/web_gui/frontend/server.mjs",
          "--port",
          String(port),
          "--shm-name",
          name,
          "--control-config",
          "tests/fixtures/control.yaml",
        ]);
      server = launch(0);
      const url = await waitFor(
        () => server.logs.match(/http:\/\/127\.0\.0\.1:\d+/)?.[0],
        () => server.logs,
      );
      const port = Number(new URL(url).port);
      duplicate = launch(port);
      await waitFor(
        () => duplicate.exitCode !== null,
        () => duplicate.logs,
      );
      assert.equal(duplicate.exitCode, 1);
      assert.match(duplicate.logs, /already in use.*Stop the previous web GUI/);
      assert.doesNotMatch(duplicate.logs, /http:\/\/127/);
      assert.equal((await fetch(url)).status, 200);

      fixture.stdin.end();
      await fixture.done;
      await waitFor(
        () => server.exitCode !== null,
        () => server.logs,
      );
      assert.equal(server.exitCode, 1);
      assert.match(server.logs, /SHM session ended or was replaced/);
      // No orphaned listener remains for the next daemon's automatically started UI.
      const listener = createServer();
      const listening = once(listener, "listening");
      listener.listen(port, "127.0.0.1");
      await listening;
      await new Promise((resolve, reject) =>
        listener.close((error) => (error ? reject(error) : resolve())),
      );
    } finally {
      await stop(duplicate);
      await stop(server);
      if (fixture && fixture.exitCode === null) {
        fixture.stdin.end();
        await fixture.done;
      }
    }
  },
);

test(
  "Web supervisor manages actual registered processes and restarts a stopped stack",
  { timeout: 25000 },
  async () => {
    const name = `/qhrr-web-supervisor-${process.pid}`;
    const folder = await mkdtemp(resolve(tmpdir(), "qhrr-web-supervisor-"));
    const manifest = resolve(folder, "daemon.json");
    await writeFile(
      manifest,
      JSON.stringify({
        runtime_mode: "hardware",
        shm_name: name,
        monitoring: { display_interval_sec: 2, window_sec: 10, sample_hz: 100 },
        processes: [
          { name: "core", exec_path: "/bin/sleep", args: ["60"] },
          {
            name: "worker",
            exec_path: "/bin/sleep",
            args: ["60"],
            critical: false,
          },
        ],
      }),
    );
    let daemon, server, ws;
    try {
      daemon = start(resolve(root, "build/daemon"), [
        "--config",
        manifest,
        "--no-terminals",
      ]);
      await waitFor(
        () => daemon.logs.includes("worker started"),
        () => daemon.logs,
      );
      server = start(resolve(root, "scripts/run_web_gui.sh"), [
        "--port",
        "0",
        "--shm-name",
        name,
        "--control-config",
        "tests/fixtures/control.yaml",
      ]);
      const url = await waitFor(
        () => server.logs.match(/http:\/\/127\.0\.0\.1:\d+/)?.[0],
        () => server.logs,
      );
      for (const route of ["/", "/processes", "/shm"])
        assert.equal((await fetch(url + route)).status, 200);
      let state;
      const replies = new Map();
      ws = new WebSocket(url.replace("http:", "ws:") + "/ws", { origin: url });
      ws.on("message", (bytes) => {
        const m = JSON.parse(bytes);
        if (m.type === "state") state = m;
        else replies.set(m.id, m);
      });
      await once(ws, "open");
      await waitFor(() => state?.runtime?.available);
      assert.equal(
        state.ready,
        false,
        "Supervisor must remain available without a running controller",
      );
      let id = 0;
      const request = async (command) => {
        const n = ++id;
        ws.send(JSON.stringify({ ...command, id: n }));
        return waitFor(() => replies.get(n));
      };
      let reply = await request({
        type: "process",
        name: "worker",
        action: "stop",
      });
      assert.equal(reply.stage, "supervisor_applied");
      assert.equal(reply.type, "ack");
      await waitFor(
        () =>
          state.runtime.processes.find((p) => p.name === "worker").running ===
          false,
      );
      reply = await request({
        type: "process",
        name: "worker",
        action: "start",
      });
      assert.equal(reply.type, "ack");
      await waitFor(
        () =>
          state.runtime.processes.find((p) => p.name === "worker").running ===
          true,
      );
      reply = await request({ type: "process", name: "core", action: "stop" });
      assert.equal(reply.type, "error");
      reply = await request({
        type: "process",
        name: "../../control",
        action: "start",
      });
      assert.equal(reply.type, "error");
      reply = await request({ type: "stack", action: "stop" });
      assert.equal(reply.type, "ack");
      await waitFor(() => state.runtime.state === "STOPPED");
      reply = await request({ type: "stack", action: "start" });
      assert.equal(reply.type, "ack");
      await waitFor(
        () =>
          state.runtime.state === "RUNNING" &&
          state.runtime.processes.every((p) => p.running),
      );
    } finally {
      ws?.terminate();
      await stop(server);
      await stop(daemon);
      await rm(folder, { recursive: true, force: true });
    }
  },
);
