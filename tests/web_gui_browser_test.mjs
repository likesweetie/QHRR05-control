import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import { once } from "node:events";
import { mkdtemp, rm, writeFile } from "node:fs/promises";
import { createRequire } from "node:module";
import { tmpdir } from "node:os";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";

const root = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const { WebSocket } = createRequire(
  resolve(root, "apps/interfaces/web_gui/frontend/package.json"),
)("ws");
const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
async function waitFor(predicate, detail = () => "", timeout = 6000) {
  const deadline = Date.now() + timeout;
  while (Date.now() < deadline) {
    const result = await predicate();
    if (result) return result;
    await delay(25);
  }
  throw new Error(`Browser test timed out: ${detail()}`);
}
function start(command, args) {
  const child = spawn(command, args, {
    cwd: root,
    stdio: ["pipe", "pipe", "pipe"],
  });
  child.logs = "";
  child.stdout.on("data", (bytes) => {
    child.logs += bytes;
  });
  child.stderr.on("data", (bytes) => {
    child.logs += bytes;
  });
  child.done = once(child, "exit");
  return child;
}
async function stop(child) {
  if (!child || child.exitCode !== null || child.signalCode !== null) return;
  child.kill("SIGTERM");
  await child.done;
}
async function connectCdp(url) {
  const ws = new WebSocket(url);
  await once(ws, "open");
  const pending = new Map();
  const exceptions = [];
  let sequence = 0;
  ws.on("message", (bytes) => {
    const message = JSON.parse(bytes);
    if (message.method === "Runtime.exceptionThrown")
      exceptions.push(message.params);
    const entry = pending.get(message.id);
    if (!entry) return;
    pending.delete(message.id);
    clearTimeout(entry.timer);
    if (message.error) entry.reject(new Error(JSON.stringify(message.error)));
    else entry.resolve(message.result);
  });
  const call = (method, params = {}) =>
    new Promise((resolve, reject) => {
      const id = ++sequence;
      const timer = setTimeout(() => {
        pending.delete(id);
        reject(new Error(`CDP timeout: ${method}`));
      }, 5000);
      pending.set(id, { resolve, reject, timer });
      ws.send(JSON.stringify({ id, method, params }));
    });
  const evaluate = async (expression) => {
    const result = await call("Runtime.evaluate", {
      expression,
      returnByValue: true,
      awaitPromise: true,
    });
    if (result.exceptionDetails)
      throw new Error(JSON.stringify(result.exceptionDetails));
    return result.result.value;
  };
  return { call, evaluate, exceptions, close: () => ws.close() };
}

test(
  "Real browser controls Input/safety, shows request stages, disconnects and fits mobile",
  { timeout: 35000 },
  async () => {
    const name = `/qhrr05-web-browser-${process.pid}`;
    const profile = await mkdtemp(resolve(tmpdir(), "qhrr05-browser-"));
    let fixture, controller, server, chrome, cdp;
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
      chrome = start(process.env.QHRR_CHROME || "/opt/google/chrome/chrome", [
        "--headless",
        "--no-sandbox",
        "--disable-gpu",
        "--disable-dev-shm-usage",
        "--no-first-run",
        "--no-default-browser-check",
        "--remote-debugging-port=0",
        `--user-data-dir=${profile}`,
        "about:blank",
      ]);
      const debuggerUrl = await waitFor(
        () =>
          chrome.logs.match(
            /ws:\/\/127\.0\.0\.1:\d+\/devtools\/browser\/\S+/,
          )?.[0],
        () => chrome.logs,
      );
      const browserHost = new URL(debuggerUrl).host;
      const pages = await (
        await fetch(`http://${browserHost}/json/list`)
      ).json();
      cdp = await connectCdp(
        pages.find((page) => page.type === "page").webSocketDebuggerUrl,
      );
      await cdp.call("Runtime.enable");
      await cdp.call("Page.enable");
      await cdp.call("Emulation.setDeviceMetricsOverride", {
        width: 1440,
        height: 1050,
        deviceScaleFactor: 1,
        mobile: false,
      });
      await cdp.call("Page.navigate", { url });
      await waitFor(() =>
        cdp.evaluate(
          `document.querySelector('button.mode-card')?.disabled === false`,
        ),
      );
      assert.match(
        await cdp.evaluate("document.body.innerText"),
        /서버 연결됨/,
      );

      await cdp.evaluate(
        `[...document.querySelectorAll('button.mode-card')].find(button => button.textContent.includes('Gravity')).click()`,
      );
      await waitFor(() =>
        cdp.evaluate(
          `document.querySelector('#machineControllerState')?.textContent.includes('Gravity') && document.querySelectorAll('.request-stages .stage-complete').length === 3`,
        ),
      );
      assert.match(
        await cdp.evaluate(
          `document.querySelector('.request-panel').innerText`,
        ),
        /TaskController 모드 반영/,
      );
      assert.match(
        await cdp.evaluate(`document.querySelector('.event-list').innerText`),
        /안전레이어 승인/,
      );

      // Real key events use the same Input/safety route, even after button focus.
      const press = async (key, code, virtualKey) => {
        await cdp.call("Input.dispatchKeyEvent", {
          type: "rawKeyDown",
          key,
          code,
          windowsVirtualKeyCode: virtualKey,
        });
        await cdp.call("Input.dispatchKeyEvent", {
          type: "keyUp",
          key,
          code,
          windowsVirtualKeyCode: virtualKey,
        });
      };
      await press("d", "KeyD", 68);
      await waitFor(() =>
        cdp.evaluate(
          `document.querySelector('#machineControllerState').textContent.includes('Damping') && document.querySelectorAll('.request-stages .stage-complete').length === 3`,
        ),
      );
      await press("g", "KeyG", 71);
      await waitFor(() =>
        cdp.evaluate(
          `document.querySelector('#machineControllerState').textContent.includes('Gravity')`,
        ),
      );

      await cdp.evaluate(
        `document.querySelector('#target-force').focus(); document.querySelector('#target-force').select()`,
      );
      await cdp.call("Input.insertText", { text: "4" });
      await press("m", "KeyM", 77);
      await delay(100);
      assert.match(
        await cdp.evaluate(
          `document.querySelector('#machineControllerState').textContent`,
        ),
        /Gravity/,
        "editing a number must not activate a motion shortcut",
      );
      await cdp.evaluate(
        `[...document.querySelectorAll('.apply-button')].find(button => button.textContent.includes('목표 힘 적용')).click()`,
      );
      await waitFor(() =>
        cdp.evaluate(
          `document.querySelector('.current-force strong')?.textContent === '4.0 N' && document.querySelector('.request-panel').innerText.includes('목표 힘 요청이 승인')`,
        ),
      );

      // Draft force adjustments must not send commands until Apply is pressed.
      await cdp.evaluate(
        `document.querySelector('[aria-label="목표 접촉력 0.5 N 늘리기"]').click()`,
      );
      assert.equal(
        await cdp.evaluate(`document.querySelector('#target-force').value`),
        "4.5",
      );
      assert.equal(
        await cdp.evaluate(
          `document.querySelector('.current-force strong').textContent`,
        ),
        "4.0 N",
      );
      assert.match(
        await cdp.evaluate(
          `document.querySelector('.draft-badge').textContent`,
        ),
        /미적용/,
      );
      await cdp.evaluate(
        `document.querySelector('#target-force').focus(); document.querySelector('#target-force').select()`,
      );
      await cdp.call("Input.insertText", { text: "100" });
      assert.equal(
        await cdp.evaluate(
          `document.querySelector('.apply-button.primary').disabled`,
        ),
        true,
      );
      assert.equal(
        await cdp.evaluate(
          `document.querySelector('#target-force').getAttribute('aria-invalid')`,
        ),
        "true",
      );
      await cdp.evaluate(`document.querySelector('#target-force').select()`);
      await cdp.call("Input.insertText", { text: "4" });
      assert.equal(
        await cdp.evaluate(
          "document.documentElement.scrollWidth <= innerWidth",
        ),
        true,
      );
      const desktop = await cdp.call("Page.captureScreenshot", {
        format: "png",
        captureBeyondViewport: true,
      });
      await writeFile(
        resolve(root, "build/web_gui_desktop.png"),
        Buffer.from(desktop.data, "base64"),
      );

      await cdp.call("Emulation.setDeviceMetricsOverride", {
        width: 390,
        height: 844,
        deviceScaleFactor: 1,
        mobile: true,
      });
      await delay(100);
      assert.equal(
        await cdp.evaluate(
          "document.documentElement.scrollWidth <= innerWidth",
        ),
        true,
      );
      assert.equal(
        await cdp.evaluate(
          `document.querySelector('button.mode-card').disabled`,
        ),
        false,
      );
      assert.equal(
        await cdp.evaluate(`
        [...document.querySelectorAll('.mode-card, .counter, .apply-button, .segmented button')]
          .every(button => button.getBoundingClientRect().height >= 44)
      `),
        true,
        "mobile controls must have usable touch targets",
      );
      const mobile = await cdp.call("Page.captureScreenshot", {
        format: "png",
        captureBeyondViewport: true,
      });
      await writeFile(
        resolve(root, "build/web_gui_mobile.png"),
        Buffer.from(mobile.data, "base64"),
      );

      for (const width of [320, 768, 1280]) {
        await cdp.call("Emulation.setDeviceMetricsOverride", {
          width,
          height: 900,
          deviceScaleFactor: 1,
          mobile: width < 768,
        });
        assert.equal(
          await cdp.evaluate(
            `document.documentElement.scrollWidth <= innerWidth`,
          ),
          true,
          `page overflows at ${width}px`,
        );
      }

      await stop(controller);
      await cdp.evaluate(
        `[...document.querySelectorAll('button.mode-card')].find(button => button.textContent.includes('Damping')).click()`,
      );
      await waitFor(() =>
        cdp.evaluate(
          `document.querySelector('[role=alert]')?.textContent.includes('Input interface unavailable')`,
        ),
      );
      assert.match(
        await cdp.evaluate(
          `document.querySelector('.request-panel').innerText`,
        ),
        /요청 거부/,
      );
      await stop(server);
      await waitFor(() =>
        cdp.evaluate(
          `document.querySelector('.connection')?.textContent.includes('연결 끊김')`,
        ),
      );
      assert.equal(
        await cdp.evaluate(
          `[...document.querySelectorAll('button.mode-card')].every(button => button.disabled)`,
        ),
        true,
      );
      assert.equal(
        await cdp.evaluate(
          `document.querySelector('#machineControllerState').textContent.includes('—')`,
        ),
        true,
      );
      assert.deepEqual(cdp.exceptions, []);
    } catch (error) {
      console.error(
        `CONTROLLER:\n${controller?.logs}\nSERVER:\n${server?.logs}\nCHROME:\n${chrome?.logs}`,
      );
      throw error;
    } finally {
      cdp?.close();
      await stop(chrome);
      await stop(server);
      await stop(controller);
      if (fixture) {
        fixture.stdin.end();
        await fixture.done;
      }
      await rm(profile, { recursive: true, force: true });
    }
  },
);

test(
  "Processes and SHM screens use the existing live supervisor",
  { timeout: 35000 },
  async () => {
    const name = `/qhrr-dashboard-processes-${process.pid}`;
    const profile = await mkdtemp(
      resolve(tmpdir(), "qhrr-dashboard-processes-"),
    );
    const manifest = resolve(profile, "daemon.json");
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
    let daemon, server, chrome, cdp;
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
      chrome = start(process.env.QHRR_CHROME || "/opt/google/chrome/chrome", [
        "--headless",
        "--no-sandbox",
        "--disable-gpu",
        "--disable-dev-shm-usage",
        "--no-first-run",
        "--remote-debugging-port=0",
        `--user-data-dir=${profile}`,
        "about:blank",
      ]);
      const debuggerUrl = await waitFor(
        () =>
          chrome.logs.match(
            /ws:\/\/127\.0\.0\.1:\d+\/devtools\/browser\/\S+/,
          )?.[0],
        () => chrome.logs,
      );
      const pages = await (
        await fetch(`http://${new URL(debuggerUrl).host}/json/list`)
      ).json();
      cdp = await connectCdp(
        pages.find((p) => p.type === "page").webSocketDebuggerUrl,
      );
      await cdp.call("Runtime.enable");
      await cdp.call("Page.enable");
      await cdp.call("Emulation.setDeviceMetricsOverride", {
        width: 1440,
        height: 1000,
        deviceScaleFactor: 1,
        mobile: false,
      });
      await cdp.call("Page.navigate", { url: url + "/processes" });
      await waitFor(() =>
        cdp.evaluate(
          `document.querySelectorAll('[data-process]').length === 2`,
        ),
      );
      await cdp.evaluate(
        `document.querySelector('[data-process="worker"] .button.danger').click()`,
      );
      await waitFor(() =>
        cdp.evaluate(
          `document.querySelector('[data-process="worker"] .badge').textContent==='STOPPED'`,
        ),
      );
      await cdp.evaluate(
        `document.querySelector('[data-process="worker"] .button.secondary').click()`,
      );
      await waitFor(() =>
        cdp.evaluate(
          `document.querySelector('[data-process="worker"] .badge').textContent==='online'`,
        ),
      );
      await cdp.evaluate(
        `document.querySelector('.stack-actions .button.danger').click()`,
      );
      await waitFor(() =>
        cdp.evaluate(
          `document.querySelector('.process-panel .panel-heading .badge').textContent==='STOPPED'`,
        ),
      );
      await cdp.evaluate(
        `document.querySelector('.stack-actions .button.primary').click()`,
      );
      await waitFor(() =>
        cdp.evaluate(
          `document.querySelector('.process-panel .panel-heading .badge').textContent==='RUNNING'`,
        ),
      );
      let shot = await cdp.call("Page.captureScreenshot", {
        format: "png",
        captureBeyondViewport: true,
      });
      await writeFile(
        resolve(root, "build/web_gui_processes.png"),
        Buffer.from(shot.data, "base64"),
      );
      await cdp.evaluate(`document.querySelector('a[href="/shm"]').click()`);
      await waitFor(() =>
        cdp.evaluate(`document.querySelector('h1').textContent==='SHM Data'`),
      );
      assert.match(await cdp.evaluate(`document.body.innerText`), /ABI 16/);
      await cdp.evaluate(`document.querySelector('summary').click()`);
      assert.match(
        await cdp.evaluate(`document.querySelector('.json-view').textContent`),
        /"command_sequence"/,
      );
      shot = await cdp.call("Page.captureScreenshot", {
        format: "png",
        captureBeyondViewport: true,
      });
      await writeFile(
        resolve(root, "build/web_gui_shm.png"),
        Buffer.from(shot.data, "base64"),
      );
      await cdp.call("Emulation.setDeviceMetricsOverride", {
        width: 390,
        height: 844,
        deviceScaleFactor: 1,
        mobile: true,
      });
      assert.equal(
        await cdp.evaluate(
          `document.documentElement.scrollWidth <= innerWidth`,
        ),
        true,
      );
      assert.deepEqual(cdp.exceptions, []);
    } finally {
      cdp?.close();
      await stop(chrome);
      await stop(server);
      await stop(daemon);
      await rm(profile, { recursive: true, force: true });
    }
  },
);
