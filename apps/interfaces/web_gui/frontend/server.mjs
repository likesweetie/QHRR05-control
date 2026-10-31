import { spawn } from "node:child_process";
import { createServer } from "node:http";
import { readFile } from "node:fs/promises";
import { createInterface } from "node:readline";
import { dirname, extname, isAbsolute, relative, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { WebSocket, WebSocketServer } from "ws";
import { sendInputRequest } from "./input-client.mjs";

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
const clients = new Set();
const pending = new Map();
let state = null;
let nextId = 0;
let shuttingDown = false;
let started = false;
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
    const file = resolve(dist, `.${path === "/" ? "/index.html" : path}`);
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
  client.on("error", (error) =>
    console.error("[web_gui] WebSocket:", error.message),
  );
  client.on("close", () => {
    clients.delete(client);
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
          : ["id", "type", "value"];
      if (Object.keys(request).some((key) => !allowed.includes(key)))
        throw new Error("Unexpected command fields");
      if (request.type === "mode") {
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
      } else throw new Error("Unsupported command");
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
          : `force ${request.value}`;
      sendInputRequest(inputSocket, id, command)
        .then((reply) => send(client, { ...reply, id: request.id }))
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
  pending.clear();
  for (const client of clients) client.terminate();
  child.stdin.end();
  child.kill("SIGTERM");
  wss.close();
  server.close();
  process.exitCode = code;
}

server.on("error", (error) => stop(1, error.message));
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
      state = message;
      for (const client of clients) send(client, message);
      if (!started) {
        started = true;
        clearTimeout(startup);
        server.listen(port, "127.0.0.1", () => {
          console.log(
            `[web_gui] http://127.0.0.1:${server.address().port} | SHM ${options["shm-name"]}`,
          );
        });
      }
    } else throw new Error("Telemetry bridge returned a non-state message");
  } catch (error) {
    stop(1, `Invalid bridge message: ${error.message}`);
  }
});
process.on("SIGINT", () => stop(0));
process.on("SIGTERM", () => stop(0));
