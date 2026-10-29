import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import { once } from "node:events";
import { access } from "node:fs/promises";
import { createConnection } from "node:net";
import { createRequire } from "node:module";
import { get } from "node:http";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";

const root = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const { WebSocket } = createRequire(resolve(root, "apps/interfaces/web_gui/frontend/package.json"))(
  "ws",
);
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

test(
  "WebSocket → Input → safety → real TaskController; telemetry is read-only",
  { timeout: 25000 },
  async () => {
    const name = `/qhrr05-web-test-${process.pid}`;
    const endpoint = `/tmp/qhrr05-input-${process.getuid()}/${name.slice(1)}.sock`;
    let fixture, controller, server, ws, bridge;
    try {
      fixture = start(resolve(root, "build/web_gui_fixture"), [name]);
      await waitFor(
        () => fixture.logs.includes("ready"),
        () => fixture.logs,
      );
      controller = start(resolve(root, "build/control"), [
        "tests/fixtures/control.yaml",
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
          "tests/fixtures/control.yaml",
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

      let state;
      const responses = new Map();
      ws = new WebSocket(url.replace("http:", "ws:") + "/ws", { origin: url });
      ws.on("message", (bytes) => {
        const message = JSON.parse(bytes);
        if (message.type === "state") state = message;
        else responses.set(message.id, message);
      });
      await once(ws, "open");
      await waitFor(() => state?.ready);
      assert.equal(state.mode, 5);
      assert.ok(Math.abs(state.motors[1].position - 0.5) < 1e-12);
      assert.equal(state.motors[0].current, 1.5);
      const request = async (command) => {
        ws.send(JSON.stringify(command));
        return waitFor(() => responses.get(command.id));
      };

      let reply = await request({ id: 1, type: "mode", mode: 1, plane: 0 });
      assert.equal(reply.type, "ack");
      assert.equal(reply.stage, "safety_approved");
      await waitFor(
        () =>
          state.mode === 1 &&
          state.active_motion_sequence === reply.motion_sequence,
      );
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
      await waitFor(() => state.requested_mode === 99);
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
        "tests/fixtures/control.yaml",
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
