import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import { once } from "node:events";
import { chmod, mkdtemp, readFile, rm, writeFile } from "node:fs/promises";
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

test("Cogging Identification plots synthetic speed ripple without any control requests", { timeout: 30000 }, async () => {
  const directory = await mkdtemp(resolve(tmpdir(), "qhrr05-cogging-browser-"));
  const commands = resolve(directory, "subscriptions.txt"), bridgePath = resolve(directory, "bridge.cjs");
  let server, chrome, cdp;
  try {
    await writeFile(bridgePath, `#!/usr/bin/env node
const {createInterface}=require('node:readline');
const {appendFileSync}=require('node:fs');
let enabled=false, sequence=0;
const positions=[10,10,10];
createInterface({input:process.stdin}).on('line',line=>{
 appendFileSync(${JSON.stringify(commands)},line+'\\n');
 if(line==='cogging 1')enabled=true;else if(line==='cogging 0')enabled=false;else process.exit(2);
});
setInterval(()=>{
 process.stdout.write(JSON.stringify({type:'state',ready:true,drive_state:'ACTIVE',mode:1,
 requested_tracking_plane:0,observed_ns:sequence*2000000,simulation_available:false,
 motors:[{},{},{}]})+'\\n');
 if(!enabled)return;
 const samples=[];
 for(let n=0;n<25;n++,sequence++)for(let axis=0;axis<3;axis++){
  const t=sequence/500,order=6+axis*2,amp=.3-axis*.05,speed=1+axis*.1;
  const velocity=q=>speed+amp*Math.cos(order*9*q);
  let q=positions[axis];
  if(sequence>0){
   const h=1/500,a=velocity(q),b=velocity(q+a*h/2),c=velocity(q+b*h/2),d=velocity(q+c*h);
   q+=h/6*(a+2*b+2*c+d);
  }
  positions[axis]=q;
  samples.push([axis,t*1000,sequence+1,q,velocity(q),1]);
 }
 process.stdout.write(JSON.stringify({type:'cogging-samples',read_hz:500,read_misses:0,samples})+'\\n');
},50);
`);
    await chmod(bridgePath, 0o700);
    server = start(process.execPath, ["apps/interfaces/web_gui/frontend/server.mjs", "--port", "0",
      "--bridge", bridgePath, "--shm-name", `/cogging-test-${process.pid}`]);
    const url = await waitFor(() => server.logs.match(/http:\/\/127\.0\.0\.1:\d+/)?.[0], () => server.logs);
    chrome = start(process.env.QHRR_CHROME || "/opt/google/chrome/chrome", ["--headless", "--no-sandbox", "--disable-gpu",
      "--disable-dev-shm-usage", "--no-first-run", "--remote-debugging-port=0", `--user-data-dir=${directory}/profile`, "about:blank"]);
    const debuggerUrl = await waitFor(() => chrome.logs.match(/ws:\/\/127\.0\.0\.1:\d+\/devtools\/browser\/\S+/)?.[0], () => chrome.logs);
    const pages = await (await fetch(`http://${new URL(debuggerUrl).host}/json/list`)).json();
    cdp = await connectCdp(pages.find((p) => p.type === "page").webSocketDebuggerUrl);
    await cdp.call("Runtime.enable"); await cdp.call("Page.enable");
    await cdp.call("Page.addScriptToEvaluateOnNewDocument", { source: `
      window.analysisRequests=[];
      const send=WebSocket.prototype.send;
      WebSocket.prototype.send=function(data){window.analysisRequests.push(JSON.parse(data));return send.call(this,data);};` });
    await cdp.call("Emulation.setDeviceMetricsOverride", { width: 1440, height: 1150, deviceScaleFactor: 1, mobile: false });
    await cdp.call("Page.navigate", { url: url + "/cogging" });
    await waitFor(() => cdp.evaluate(`document.querySelector('h1')?.textContent==='Cogging Identification' && [...document.querySelectorAll('button')].some(b=>b.textContent==='측정 시작'&&!b.disabled)`));
    await cdp.evaluate(`[...document.querySelectorAll('button')].find(b=>b.textContent==='측정 시작').click()`);
    await waitFor(() => cdp.evaluate(`document.querySelectorAll('.cogging-axis [data-series="dq-feedback"]').length===6`), () => server.logs, 10000);
    const getPeaks = () => cdp.evaluate(`[...document.querySelectorAll('.cogging-axis')[0].querySelectorAll('.analysis-peaks tbody tr')].map(r=>[...r.cells].map(c=>c.textContent))`);
    let peaks = await getPeaks();
    const qPeak = peaks.find((p) => p[0] === "q 차분"), dqPeak = peaks.find((p) => p[0] === "dq");
    assert.ok(Math.abs(Number(qPeak[1]) - 6) < .01);
    assert.ok(Number(qPeak[2]) > .20 && Number(qPeak[2]) < .27);
    assert.ok(Number(dqPeak[1]) === 6);
    assert.ok(Number(dqPeak[2]) > .29 && Number(dqPeak[2]) < .31);
    assert.equal(dqPeak.length, 5);
    assert.ok(Math.abs(Number(dqPeak[3])) < .004, "dq sine must be relative to q=0");
    assert.ok(Math.abs(Number(dqPeak[4]) - .3) < .004, "dq cosine must be relative to q=0");
    assert.equal(await cdp.evaluate(`document.querySelectorAll('.analysis-chart')[1].textContent.includes('cycles/rev')`), true);
    assert.equal(await cdp.evaluate(`analysisRequests.findLast(r=>r.enabled)?.settings.gearRatio`), 9);
    assert.equal(await cdp.evaluate(`analysisRequests.findLast(r=>r.enabled)?.settings.revolutions`), 1);
    assert.equal(await cdp.evaluate(`[...document.querySelectorAll('label')].find(l=>l.textContent.includes('분석 구간')).textContent.includes('출력축 5.0°')`), true);
    await cdp.evaluate(`{
      const select=[...document.querySelectorAll('label')].find(l=>l.textContent.includes('분석 구간')).querySelector('select');
      select.value='0.5'; select.dispatchEvent(new Event('change',{bubbles:true}));
    }`);
    await waitFor(()=>cdp.evaluate(`analysisRequests.findLast(r=>r.enabled)?.settings.revolutions===0.5 && document.querySelector('.cogging-axis').textContent.includes('0.5 rev')`));
    assert.ok(Math.abs(Number((await getPeaks()).find(p=>p[0]==='dq')[1])-6)<.01);
    await cdp.call("Input.dispatchKeyEvent", { type: "keyDown", key: "g", code: "KeyG", windowsVirtualKeyCode: 71 });
    await cdp.call("Input.dispatchKeyEvent", { type: "keyUp", key: "g", code: "KeyG", windowsVirtualKeyCode: 71 });
    await cdp.evaluate(`document.querySelector('.analysis-check input').click()`);
    await waitFor(async () => Number((await getPeaks()).find((p) => p[0] === "q 차분")?.[2]) > .29);
    const rawQPeak = (await getPeaks()).find((p) => p[0] === "q 차분");
    assert.ok(Math.abs(Number(rawQPeak[3])) < .004);
    assert.ok(Math.abs(Number(rawQPeak[4]) - .3) < .004);
    assert.equal(await cdp.evaluate(`document.querySelector('.cogging-axis').textContent.includes('MIT q=0')`), true);
    const screenshot = await cdp.call("Page.captureScreenshot", { format: "png", captureBeyondViewport: true });
    await writeFile(resolve(root, "build/web_gui_cogging.png"), Buffer.from(screenshot.data, "base64"));
    await cdp.call("Emulation.setDeviceMetricsOverride", { width: 390, height: 844, deviceScaleFactor: 1, mobile: true });
    assert.equal(await cdp.evaluate(`document.documentElement.scrollWidth <= innerWidth`), true);
    const mobile = await cdp.call("Page.captureScreenshot", { format: "png", captureBeyondViewport: true });
    await writeFile(resolve(root, "build/web_gui_cogging_mobile.png"), Buffer.from(mobile.data, "base64"));
    await cdp.evaluate(`[...document.querySelectorAll('button')].find(b=>b.textContent==='측정 일시정지').click()`);
    await waitFor(async () => (await readFile(commands, "utf8")).trim().endsWith("cogging 0"));
    await delay(350); // Let the server publish the final paused window.
    const frozen = await cdp.evaluate(`document.querySelector('.analysis-chart path').getAttribute('d')`);
    await delay(400);
    assert.equal(await cdp.evaluate(`document.querySelector('.analysis-chart path').getAttribute('d')`), frozen);
    await cdp.evaluate(`[...document.querySelectorAll('button')].find(b=>b.textContent==='버퍼 초기화').click()`);
    await waitFor(() => cdp.evaluate(`document.querySelectorAll('.analysis-peaks').length===0`));
    assert.equal(await cdp.evaluate(`analysisRequests.every(r=>r.type==='cogging-subscribe')`), true);
    assert.deepEqual(cdp.exceptions, []);
  } finally {
    cdp?.close(); await stop(chrome); await stop(server); await rm(directory, { recursive: true, force: true });
  }
});

test(
  "Real browser controls Input/safety, shows request stages, disconnects and fits mobile",
  { timeout: 35000 },
  async () => {
    const name = `/qhrr05-web-browser-${process.pid}`;
    const profile = await mkdtemp(resolve(tmpdir(), "qhrr05-browser-"));
    let fixture, controller, server, chrome, cdp;
    let trackingPump;
    const phaseConfig = resolve(profile, "control.yaml");
    const fixtureConfig = (await readFile(resolve(root, "tests/fixtures/control.yaml"), "utf8"))
      .replace(/(  ilc:[^\n]*\n    enabled:) false/, "$1 true")
      .replace(/^torque_reference_mode:.*$/m, "torque_reference_mode: rmd_torque_ref")
      .replace(/^model_path:.*$/m, `model_path: ${JSON.stringify(resolve(root, "models/qhrr-05/qhrr-05.xml"))}`);
    await writeFile(phaseConfig, fixtureConfig + `\nanticogging:\n  enabled: true\n  motor_config: ${JSON.stringify(resolve(root, "config/motor/ak10_9_v3.yaml"))}\n  scale: 0.1\n  amp21_motor_nm: 0.01\n  amp36_motor_nm: 0\n  motor_zero_rad: [-0.9, -4.5, 11.7]\n  max_motor_comp_nm: 0.005\n`);
    try {
      fixture = start(resolve(root, "build/web_gui_fixture"), [name, phaseConfig]);
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
      server = start(resolve(root, "scripts/run_web_gui.sh"), [
        "--port",
        "0",
        "--shm-name",
        name,
        "--control-config",
        phaseConfig,
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

      await waitFor(() => cdp.evaluate(`document.querySelector('#ilc-retention')?.disabled===false`));
      assert.equal(await cdp.evaluate(`document.querySelector('#ilc-phase_lead_bins').value`), "1");
      assert.equal(await cdp.evaluate(`document.querySelector('#ilc-smoothing_passes').value`), "2");
      assert.equal(await cdp.evaluate(`document.querySelector('#ilc-retention').value`), "1");
      assert.equal(await cdp.evaluate(`document.querySelector('.ilc-panel').getBoundingClientRect().bottom <= document.querySelector('.tracking-error-panel').getBoundingClientRect().top`), true);
      await cdp.evaluate(`(() => {
        window.ilcRequests=[];
        const send=WebSocket.prototype.send;
        WebSocket.prototype.send=function(value){ const message=JSON.parse(value); if(message.type==='tracking-ilc')ilcRequests.push(message); return send.call(this,value); };
        for(const [key,value] of Object.entries({phase_lead_bins:'3',smoothing_passes:'5',retention:'0.85'})){
          const input=document.querySelector('#ilc-'+key);
          Object.getOwnPropertyDescriptor(HTMLInputElement.prototype,'value').set.call(input,value);
          input.dispatchEvent(new Event('input',{bubbles:true}));
        }
        document.querySelector('#ilc-retention').focus();
      })()`);
      await delay(400);
      await cdp.call("Input.dispatchKeyEvent", { type: "keyDown", key: "Enter", code: "Enter", windowsVirtualKeyCode: 13 });
      await cdp.call("Input.dispatchKeyEvent", { type: "keyUp", key: "Enter", code: "Enter", windowsVirtualKeyCode: 13 });
      assert.equal(await cdp.evaluate(`ilcRequests.length`), 0);
      assert.equal(await cdp.evaluate(`document.querySelector('#ilc-applied-retention').textContent`), "현재 설정: 1");
      await cdp.evaluate(`document.querySelector('#ilc-apply').click()`);
      await waitFor(() => cdp.evaluate(`document.querySelector('#ilc-applied-retention').textContent==='현재 설정: 0.85'`));
      assert.equal(await cdp.evaluate(`document.querySelector('#ilc-applied-phase_lead_bins').textContent`), "현재 설정: 3");
      assert.equal(await cdp.evaluate(`document.querySelector('#ilc-applied-smoothing_passes').textContent`), "현재 설정: 5");
      assert.equal(await cdp.evaluate(`ilcRequests.length`), 1);
      const ilcBounds = await cdp.evaluate(`(() => {const r=document.querySelector('.ilc-panel').getBoundingClientRect();return {x:r.x+scrollX,y:r.y+scrollY,width:r.width,height:r.height,scale:1};})()`);
      const ilcShot = await cdp.call("Page.captureScreenshot", { format: "png", captureBeyondViewport: true, clip: ilcBounds });
      await writeFile(resolve(root, "build/web_gui_ilc.png"), Buffer.from(ilcShot.data, "base64"));
      await cdp.evaluate(`(() => {const input=document.querySelector('#ilc-retention');Object.getOwnPropertyDescriptor(HTMLInputElement.prototype,'value').set.call(input,'0');input.dispatchEvent(new Event('input',{bubbles:true}));})()`);
      await waitFor(() => cdp.evaluate(`document.querySelector('#ilc-apply').disabled && !!document.querySelector('.ilc-panel [role=alert]')`));
      assert.equal(await cdp.evaluate(`document.querySelector('#ilc-applied-retention').textContent`), "현재 설정: 0.85");

      await waitFor(() => cdp.evaluate(`document.querySelector('#rmd-damping')?.disabled===false`));
      assert.equal(await cdp.evaluate(`document.querySelector('#rmd-damping').value`), "5");
      await cdp.evaluate(`(() => {
        const input = document.querySelector('#rmd-damping');
        Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set.call(input, '9');
        input.dispatchEvent(new Event('input', {bubbles:true}));
        input.focus();
      })()`);
      await delay(400);
      assert.equal(await cdp.evaluate(`document.querySelector('#rmd-damping-applied').textContent`), "현재 적용: 5");
      await cdp.call("Input.dispatchKeyEvent", { type: "keyDown", key: "Enter", code: "Enter", windowsVirtualKeyCode: 13 });
      await cdp.call("Input.dispatchKeyEvent", { type: "keyUp", key: "Enter", code: "Enter", windowsVirtualKeyCode: 13 });
      assert.equal(await cdp.evaluate(`document.querySelector('#rmd-damping-applied').textContent`), "현재 적용: 5");
      await cdp.evaluate(`document.querySelector('#rmd-damping-apply').click()`);
      await waitFor(() => cdp.evaluate(`document.querySelector('#rmd-damping-applied').textContent==='현재 적용: 9'`));
      await waitFor(() => cdp.evaluate(`document.querySelector('.phase-arrows button')?.disabled===false`));
      assert.equal(await cdp.evaluate(`document.querySelector('.phase-control output').textContent.trim()`), "0.00 rad");
      assert.equal(await cdp.evaluate(`document.querySelector('.anticogging-panel').getBoundingClientRect().top >= document.querySelector('.machine-panel').getBoundingClientRect().bottom`), true);
      const phaseButtons = `.phase-control[data-axis="0"][data-order="21"]`;
      await cdp.evaluate(`document.querySelector('${phaseButtons} .phase-arrows button').click()`);
      await waitFor(() => cdp.evaluate(`document.querySelector('${phaseButtons} .phase-applied').textContent==='적용 0.500 rad'`));
      assert.equal(await cdp.evaluate(`document.querySelector('${phaseButtons} output').textContent.trim()`), "0.50 rad");
      await cdp.evaluate(`document.querySelector('${phaseButtons} .phase-arrows button:last-child').click()`);
      await waitFor(() => cdp.evaluate(`document.querySelector('${phaseButtons} .phase-applied').textContent==='적용 0.000 rad'`));
      await cdp.evaluate(`document.querySelector('.phase-control[data-axis="2"][data-order="36"] .phase-arrows button:last-child').click()`);
      await waitFor(() => cdp.evaluate(`document.querySelector('.phase-control[data-axis="2"][data-order="36"] .phase-applied').textContent==='적용 5.783 rad'`));
      assert.equal(await cdp.evaluate(`document.querySelector('${phaseButtons} output').textContent.trim()`), "0.00 rad");
      assert.equal(await cdp.evaluate(`document.querySelector('[data-phase-step="0.5"]').getAttribute('aria-pressed')`), "true");
      await cdp.evaluate(`document.querySelector('[data-phase-step="0.1"]').click()`);
      await cdp.evaluate(`document.querySelector('${phaseButtons} .phase-arrows button').click()`);
      await waitFor(() => cdp.evaluate(`document.querySelector('${phaseButtons} .phase-applied').textContent==='적용 0.100 rad'`));
      await cdp.evaluate(`document.querySelector('[data-phase-step="0.25"]').click(); document.querySelector('${phaseButtons} output').focus()`);
      await cdp.call("Input.dispatchKeyEvent", { type: "keyDown", key: "ArrowUp", code: "ArrowUp", windowsVirtualKeyCode: 38 });
      await cdp.call("Input.dispatchKeyEvent", { type: "keyUp", key: "ArrowUp", code: "ArrowUp", windowsVirtualKeyCode: 38 });
      await waitFor(() => cdp.evaluate(`document.querySelector('${phaseButtons} .phase-applied').textContent==='적용 0.350 rad'`));
      assert.equal(await cdp.evaluate(`document.querySelector('[data-phase-step="0.25"]').getAttribute('aria-pressed')`), "true");
      assert.equal(await cdp.evaluate(`document.querySelector('.anticogging-panel').textContent.includes('±0.25 rad')`), true);
      assert.equal(await cdp.evaluate(`document.querySelector('${phaseButtons} .amplitude-stepper output').textContent.trim()`), "0.01 Nm");
      assert.equal(await cdp.evaluate(`document.querySelector('.phase-axis .amplitude-clipping')===null`), true);
      await cdp.evaluate(`document.querySelector('${phaseButtons} .amplitude-arrows button').click()`);
      await waitFor(() => cdp.evaluate(`document.querySelector('${phaseButtons} .amplitude-applied').textContent==='적용 0.020 Nm'`));
      await waitFor(() => cdp.evaluate(`document.querySelector('.phase-axis .amplitude-clipping')?.textContent.includes('모터축 보상 토크 한계')`));
      assert.equal(await cdp.evaluate(`document.querySelector('.phase-control[data-axis="0"][data-order="36"] .amplitude-arrows button:last-child').disabled`), true);
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
        [...document.querySelectorAll('.mode-card, .counter, .apply-button, .segmented button, .phase-arrows button')]
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
      // Controller is stopped; publish read-only fixture samples with a known 13 mm error.
      fixture.stdin.write("tracking\n");
      trackingPump = setInterval(() => fixture.stdin.write("tracking-sample\n"), 5);
      await waitFor(() => cdp.evaluate(`document.querySelector('#tracking-error-norm')?.textContent==='13.00 mm'`), () => server.logs);
      assert.equal(await cdp.evaluate(`document.querySelector('#tracking-error-rms').textContent`), "13.00 mm");
      assert.equal(await cdp.evaluate(`document.querySelectorAll('.tracking-error-panel path[data-series]').length`), 6);
      assert.equal(await cdp.evaluate(`document.querySelector('.tracking-error-panel').textContent.includes('-4.00 / 4.00 mm')`), true);
      for (const width of [390, 1280]) {
        await cdp.call("Emulation.setDeviceMetricsOverride", { width, height: 1000, deviceScaleFactor: 1, mobile: width < 768 });
        assert.equal(await cdp.evaluate(`document.documentElement.scrollWidth <= innerWidth`), true);
      }
      const trackingBounds = await cdp.evaluate(`(() => { const r=document.querySelector('.tracking-error-panel').getBoundingClientRect(); return {x:r.x+scrollX,y:r.y+scrollY,width:r.width,height:r.height,scale:1}; })()`);
      const trackingShot = await cdp.call("Page.captureScreenshot", { format: "png", captureBeyondViewport: true, clip: trackingBounds });
      await writeFile(resolve(root, "build/web_gui_tracking.png"), Buffer.from(trackingShot.data, "base64"));
      clearInterval(trackingPump);
      await waitFor(() => cdp.evaluate(`document.querySelector('.tracking-error-panel').textContent.includes('피드백 수집 중단')`));
      assert.equal(await cdp.evaluate(`document.querySelector('#tracking-error-norm').textContent`), "—");
      fixture.stdin.write("reaction\n");
      trackingPump = setInterval(() => fixture.stdin.write("reaction\n"), 50);
      await waitFor(() => cdp.evaluate(`document.querySelector('#reaction-force-3')?.textContent==='5.00 N'`));
      assert.deepEqual(await cdp.evaluate(`[0,1,2,3].map(i=>document.querySelector('#reaction-force-'+i).textContent)`),
        ["-3.00 N", "4.00 N", "0.00 N", "5.00 N"]);
      assert.equal(await cdp.evaluate(`document.querySelector('#reaction-force-normal').textContent`), "3.00 N");
      assert.equal(await cdp.evaluate(`document.querySelectorAll('.reaction-force-panel path[data-series]').length`), 4);
      for (const width of [390, 1280]) {
        await cdp.call("Emulation.setDeviceMetricsOverride", { width, height: 1000, deviceScaleFactor: 1, mobile: width < 768 });
        assert.equal(await cdp.evaluate(`document.documentElement.scrollWidth <= innerWidth`), true);
      }
      const reactionBounds = await cdp.evaluate(`(() => { const r=document.querySelector('.reaction-force-panel').getBoundingClientRect(); return {x:r.x+scrollX,y:r.y+scrollY,width:r.width,height:r.height,scale:1}; })()`);
      const reactionShot = await cdp.call("Page.captureScreenshot", { format: "png", captureBeyondViewport: true, clip: reactionBounds });
      await writeFile(resolve(root, "build/web_gui_reaction_force.png"), Buffer.from(reactionShot.data, "base64"));
      clearInterval(trackingPump);
      fixture.stdin.write("reaction-free\n");
      await waitFor(() => cdp.evaluate(`document.querySelector('#reaction-force-normal')?.textContent==='—'`));
      assert.equal(await cdp.evaluate(`document.querySelector('#reaction-force-3').textContent`), "5.00 N");
      fixture.stdin.write("reaction-invalid\n");
      await waitFor(() => cdp.evaluate(`document.querySelector('.reaction-force-panel .panel-heading .badge')?.textContent==='추정 무효'`));
      assert.equal(await cdp.evaluate(`document.querySelector('#reaction-force-3').textContent`), "—");
      assert.equal(await cdp.evaluate(`document.querySelectorAll('.reaction-force-panel path[data-series]').length`), 0);
      fixture.stdin.write("reaction\n");
      await waitFor(() => cdp.evaluate(`document.querySelector('#reaction-force-3').textContent==='5.00 N'`));
      await waitFor(() => cdp.evaluate(`document.querySelector('.reaction-force-panel .panel-heading .badge')?.textContent==='갱신 중단'`));
      assert.equal(await cdp.evaluate(`document.querySelector('#reaction-force-3').textContent`), "—");
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
        `CONTROLLER:\n${controller?.logs}\nSERVER:\n${server?.logs}\nCHROME:\n${chrome?.logs}\nJS:\n${JSON.stringify(cdp?.exceptions)}`,
      );
      throw error;
    } finally {
      clearInterval(trackingPump);
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
      assert.match(await cdp.evaluate(`document.body.innerText`), /ABI 22/);
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
