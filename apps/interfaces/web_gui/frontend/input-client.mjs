import { createConnection } from "node:net";

// The Input endpoint owns safety decisions. This client never publishes SHM commands.
export function sendInputRequest(socketPath, id, command) {
  return new Promise((resolve, reject) => {
    const socket = createConnection({ path: socketPath });
    let buffer = "";
    let finished = false;
    let sent = false;
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
            "Input/safety response was not confirmed; inspect current state before retrying",
          ),
        ),
      2000,
    );
    socket.on("connect", () => {
      sent = true;
      socket.write(`${id} ${command}\n`);
    });
    socket.on("error", (error) =>
      finish(new Error(`Input interface unavailable: ${error.message}`)),
    );
    socket.on("end", () =>
      finish(new Error("Input interface closed before a safety decision")),
    );
    socket.on("data", (bytes) => {
      buffer += bytes.toString();
      if (buffer.length > 4096)
        return finish(new Error("Input response exceeded size limit"));
      const end = buffer.indexOf("\n");
      if (end === -1) return;
      try {
        const reply = JSON.parse(buffer.slice(0, end));
        if (
          reply.id !== id ||
          !["ack", "error"].includes(reply.type) ||
          (reply.type === "ack" && reply.stage !== "safety_approved")
        )
          throw new Error("Input returned an invalid safety decision");
        finish(null, reply);
      } catch (error) {
        finish(error);
      }
    });
  });
}
