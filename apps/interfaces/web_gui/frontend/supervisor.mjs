import { createConnection } from "node:net";
import { readFile, readlink } from "node:fs/promises";
import { execFileSync } from "node:child_process";

export class Supervisor {
  constructor(session) {
    this.sockets = new Set();
    this.stopped = false;
    this.path = `/tmp/qhrr05-daemon-${process.getuid()}/${session.slice(1)}.sock`;
    this.state = { available: false, error: "Waiting for daemon supervisor" };
    this.previous = new Map();
    this.ticks = Number(
      execFileSync("getconf", ["CLK_TCK"], { encoding: "utf8" }).trim(),
    );
    if (!Number.isFinite(this.ticks) || this.ticks <= 0)
      throw new Error("Invalid process clock frequency");
  }
  request(command) {
    if (this.stopped)
      return Promise.reject(new Error("Supervisor client stopped"));
    return new Promise((resolve, reject) => {
      const socket = createConnection({ path: this.path });
      this.sockets.add(socket);
      socket.once("close", () => this.sockets.delete(socket));
      let bytes = "",
        sent = false,
        finished = false;
      const finish = (error, reply) => {
        if (finished) return;
        finished = true;
        clearTimeout(timeout);
        socket.destroy();
        if (error) {
          error.uncertain = sent;
          reject(error);
        } else resolve(reply);
      };
      const timeout = setTimeout(
        () =>
          finish(
            new Error(
              "Daemon response not confirmed; inspect process state before retrying",
            ),
          ),
        30000,
      );
      socket.on("connect", () => {
        sent = true;
        socket.write(`1 ${command}\n`);
      });
      socket.on("error", (error) =>
        finish(new Error(`Daemon supervisor unavailable: ${error.message}`)),
      );
      socket.on("end", () =>
        finish(new Error("Daemon closed before its decision")),
      );
      socket.on("data", (data) => {
        bytes += data;
        if (bytes.length > 262144)
          return finish(new Error("Daemon response too large"));
        if (!bytes.includes("\n")) return;
        try {
          const reply = JSON.parse(bytes.slice(0, bytes.indexOf("\n")));
          if (reply.id !== 1 || !["ack", "error"].includes(reply.type))
            throw new Error("Invalid daemon response");
          if (reply.runtime) this.state = reply.runtime;
          finish(null, reply);
        } catch (error) {
          finish(error);
        }
      });
    });
  }
  async poll() {
    if (this.polling || this.stopped) return;
    this.polling = true;
    try {
      const reply = await this.request("status");
      if (reply.type !== "ack") throw new Error(reply.message);
      const now = performance.now();
      const uptime = Number(
        (await readFile("/proc/uptime", "utf8")).split(" ")[0],
      );
      for (const item of this.state.processes) {
        if (!item.running) continue;
        try {
          const stat = await readFile(`/proc/${item.pid}/stat`, "utf8");
          const fields = stat.slice(stat.lastIndexOf(")") + 2).split(" ");
          const ticks = Number(fields[11]) + Number(fields[12]);
          const start = Number(fields[19]);
          const old = this.previous.get(item.name);
          item.cpu_percent =
            old?.start === start && now > old.time
              ? (100000 * (ticks - old.ticks)) / this.ticks / (now - old.time)
              : null;
          this.previous.set(item.name, { start, ticks, time: now });
          item.uptime_sec = Math.max(0, uptime - start / this.ticks);
          const status = await readFile(`/proc/${item.pid}/status`, "utf8");
          item.rss_mib = Number(status.match(/^VmRSS:\s+(\d+)/m)?.[1]) / 1024;
          item.threads = Number(status.match(/^Threads:\s+(\d+)/m)?.[1]);
          item.cwd = await readlink(`/proc/${item.pid}/cwd`);
        } catch (error) {
          item.metrics_error = error.message;
        }
      }
    } catch (error) {
      this.state = { available: false, error: error.message };
    } finally {
      this.polling = false;
    }
  }
  close() {
    this.stopped = true;
    for (const socket of this.sockets)
      socket.destroy(new Error("Supervisor client stopped"));
  }
}
