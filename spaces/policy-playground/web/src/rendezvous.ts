/**
 * JSON-RPC to a duck through the rendezvous, from the browser.
 *
 * A direct port of `sdk/duck/wire.py`, which is the transport that works from anywhere: no
 * WebRTC, no ICE, no relay candidate. The service forwards every key of a `peer` envelope except
 * `type` and `sessionId` verbatim to the session partner, so an envelope carrying `rpc` is a
 * control call and `mediad::relay`'s control lane answers it out of the routing table the
 * datachannel uses.
 *
 *     POST /send  {"type": "peer", "sessionId": S, "rpc": {"jsonrpc": "2.0", "id": 1, …}}
 *     SSE         {"type": "peer", "sessionId": S, "rpc": {"jsonrpc": "2.0", "id": 1, "result": …}}
 *
 * Four things this protocol punishes a reader for not knowing, all learned the hard way in the
 * Python version and all still true here:
 *
 * - **`POST /send` before `GET /events` is a 400.** Identity comes from the bearer token and the
 *   stream is what binds it, so the stream opens first and everything waits for the `welcome`.
 * - **`startSession` and `list` answer in the POST body**, not on the stream. Everything else
 *   arrives on the stream. This is the shape a reader gets wrong once.
 * - **CRLF is not ours to assume.** SSE permits `\r\n` and a proxy may rewrite them; splitting on
 *   `\n\n` alone then matches nothing and every message vanishes in silence.
 * - **One consumer per robot.** `sessionRejected` means somebody else holds it, and the robot's
 *   own console counts.
 *
 * And one thing the browser changes for the better: `EventSource` cannot send an `Authorization`
 * header, so this reads the stream with `fetch` and a reader. That is not a workaround — it is
 * what lets the token stay in a header rather than a query string the service only kept as a
 * deprecated fallback.
 */

export const DEFAULT_CENTRAL_URL = "https://pollen-robotics-reachy-mini-central.hf.space";

/** `meta.kind`, which is how one client tells a duck from a mini without opening a session. */
const DUCK = "microduck";

export class RendezvousError extends Error {}

export interface Robot {
  peerId: string;
  name: string;
  kind: string | null;
  release: string;
  busy: boolean;
  activeApp: string | null;
  ageSeconds: number | null;
}

interface Envelope {
  type?: string;
  [key: string]: unknown;
}

function robotFrom(entry: Record<string, unknown>): Robot {
  const meta = (entry.meta ?? {}) as Record<string, unknown>;
  return {
    peerId: String(entry.peerId ?? entry.id ?? ""),
    name: String(meta.name ?? entry.robotName ?? "a robot with no name"),
    kind: meta.kind == null ? null : String(meta.kind),
    release: String(meta.release ?? "release unknown"),
    busy: Boolean(entry.busy),
    activeApp: entry.activeApp == null ? null : String(entry.activeApp),
    ageSeconds: typeof entry.last_seen_age_seconds === "number" ? entry.last_seen_age_seconds : null,
  };
}

/**
 * The ducks this account can reach, and whatever else was listed.
 *
 * `GET /api/robot-status` is one request and opens no event stream, so it is safe to press
 * mid-session — unlike the console's `list`, which would open a second stream on the same token
 * and evict the peer the session is riding on.
 */
export async function listDucks(token: string, base = DEFAULT_CENTRAL_URL): Promise<{
  ducks: Robot[];
  others: string[];
}> {
  if (!token) throw new RendezvousError("no token to ask with");

  let answer: Response;
  try {
    answer = await fetch(`${base}/api/robot-status`, {
      headers: { Authorization: `Bearer ${token}` },
    });
  } catch (e) {
    throw new RendezvousError(`the rendezvous could not be reached: ${e}`);
  }

  if (answer.status === 401) {
    throw new RendezvousError(
      "the rendezvous did not recognise this sign-in. Signing out and back in is the fix.",
    );
  }
  if (!answer.ok) {
    throw new RendezvousError(
      `the rendezvous answered HTTP ${answer.status}: ${(await answer.text()).slice(0, 200)}`,
    );
  }

  const body = (await answer.json()) as { robots?: unknown };
  const listed = Array.isArray(body.robots) ? body.robots : [];
  const ducks: Robot[] = [];
  const others: string[] = [];
  for (const entry of listed) {
    if (typeof entry !== "object" || entry === null) continue;
    const robot = robotFrom(entry as Record<string, unknown>);
    if (!robot.peerId) continue;
    if (robot.kind === DUCK) ducks.push(robot);
    else others.push(`${robot.name} (${robot.kind ?? "no kind declared"})`);
  }
  return { ducks, others };
}

type Pending = {
  method: string;
  resolve: (value: unknown) => void;
  reject: (reason: Error) => void;
  timer: number;
};

export class RpcError extends Error {
  constructor(
    readonly method: string,
    message: string,
  ) {
    super(message);
  }
}

/** How long a call may go unanswered. `policy.fetch` overrides it — that one is a download. */
const CALL_TIMEOUT = 20_000;

/**
 * One control-only session with one duck.
 *
 * `onEvent` is how the page learns what is happening without polling: every envelope, every call
 * and every refusal is announced, which is what the log panel reads and what the status chip
 * reads. A transport that only reported success would leave four different silences looking
 * identical, which is the failure this whole file was written twice to avoid.
 */
export class Session {
  private stream: AbortController | null = null;
  private sessionId: string | null = null;
  private selfPeerId: string | null = null;
  private nextId = 1;
  private readonly pending = new Map<number, Pending>();
  private welcome: ((message: Envelope) => void) | null = null;

  constructor(
    private readonly token: string,
    private readonly peerId: string,
    private readonly label: string,
    private readonly onEvent: (line: string) => void,
    private readonly base = DEFAULT_CENTRAL_URL,
  ) {}

  get open(): boolean {
    return this.sessionId !== null;
  }

  async start(): Promise<void> {
    const welcomed = new Promise<Envelope>((resolve, reject) => {
      this.welcome = resolve;
      window.setTimeout(
        () => reject(new RendezvousError("the rendezvous accepted the stream and never said hello")),
        20_000,
      );
    });

    this.openStream();
    const hello = await welcomed;
    this.selfPeerId = typeof hello.peerId === "string" ? hello.peerId : null;
    this.onEvent(`welcome: peer ${(this.selfPeerId ?? "?").slice(0, 8)}`);

    // A name in the listing, so the owner's other devices can see who holds the robot — the
    // service reports a consumer's `meta.name` back as `activeApp`.
    await this.post({ type: "setPeerStatus", roles: ["listener"], meta: { name: this.label } });

    // The session is what makes a `peer` envelope routable: the service drops one naming a session
    // it does not know. Nothing about it commits either end to WebRTC.
    const answer = await this.post({ type: "startSession", peerId: this.peerId });
    const kind = answer?.type;
    if (kind === "sessionRejected") {
      throw new RendezvousError(
        `this duck is busy with ${String(answer?.activeApp ?? "something else")} — ` +
          "one at a time, and the robot's own console counts.",
      );
    }
    if (kind === "error") {
      throw new RendezvousError(String(answer?.details ?? "the rendezvous refused"));
    }
    if (kind !== "sessionStarted") {
      throw new RendezvousError(`startSession answered ${JSON.stringify(answer)}`);
    }
    this.sessionId = String(answer?.sessionId ?? "");
    this.onEvent(`session ${this.sessionId.slice(0, 8)} open, control only`);
  }

  async stop(): Promise<void> {
    const id = this.sessionId;
    this.sessionId = null;
    if (id) {
      // A teardown that cannot be delivered still ends this side: the service drops the robot's
      // lane on its own endSession broadcast either way.
      await this.post({ type: "endSession", sessionId: id }).catch(() => undefined);
    }
    this.stream?.abort();
    this.stream = null;
    this.abandon("disconnected");
  }

  /** One JSON-RPC request, answered on the stream. */
  async call(method: string, params: Record<string, unknown> = {}, timeout = CALL_TIMEOUT): Promise<unknown> {
    if (!this.sessionId) throw new RpcError(method, "not connected to a duck");
    const id = this.nextId++;
    this.onEvent(`→ ${method} ${JSON.stringify(params)}`.slice(0, 300));

    const answered = new Promise<unknown>((resolve, reject) => {
      const timer = window.setTimeout(() => {
        this.pending.delete(id);
        reject(new RpcError(method, `no answer in ${Math.round(timeout / 1000)}s`));
      }, timeout);
      this.pending.set(id, { method, resolve, reject, timer });
    });

    await this.post({
      type: "peer",
      sessionId: this.sessionId,
      rpc: { jsonrpc: "2.0", id, method, params },
    });
    return answered;
  }

  private openStream(): void {
    const controller = new AbortController();
    this.stream = controller;
    void (async () => {
      try {
        const answer = await fetch(`${this.base}/events`, {
          headers: { Authorization: `Bearer ${this.token}`, Accept: "text/event-stream" },
          signal: controller.signal,
        });
        if (answer.status === 401) throw new RendezvousError("the rendezvous refused this sign-in");
        if (!answer.ok || !answer.body) throw new RendezvousError(`the event stream answered HTTP ${answer.status}`);

        const reader = answer.body.getReader();
        const decoder = new TextDecoder();
        let buffer = "";
        for (;;) {
          const { done, value } = await reader.read();
          if (done) break;
          // CRLF normalised on arrival: the framing is not ours, and a proxy may rewrite it.
          buffer += decoder.decode(value, { stream: true }).replaceAll("\r\n", "\n");
          let cut = buffer.indexOf("\n\n");
          while (cut !== -1) {
            const frame = buffer.slice(0, cut);
            buffer = buffer.slice(cut + 2);
            const data = frame
              .split("\n")
              .filter((line) => line.startsWith("data:"))
              .map((line) => line.slice(5).trim())
              .join("");
            if (data) this.handle(data);
            cut = buffer.indexOf("\n\n");
          }
        }
        if (!controller.signal.aborted) this.ended("the rendezvous closed the event stream");
      } catch (e) {
        // Aborting mid-read is how `stop` works, and it throws. That is a clean teardown and not
        // something to report as a failure.
        if (!controller.signal.aborted) this.ended(`the event stream failed: ${e}`);
      }
    })();
  }

  private handle(data: string): void {
    let message: Envelope;
    try {
      message = JSON.parse(data) as Envelope;
    } catch {
      this.onEvent(`← unparseable frame: ${data.slice(0, 160)}`);
      return;
    }

    if (message.type === "welcome") {
      this.welcome?.(message);
      this.welcome = null;
      return;
    }

    if (message.type === "peer") {
      const payload = message.rpc as Record<string, unknown> | undefined;
      // An `sdp` or an `ice`: a media negotiation this transport never started. Not an error.
      if (!payload) return;
      const id = typeof payload.id === "number" ? payload.id : null;
      if (id === null) return;
      const waiting = this.pending.get(id);
      if (!waiting) return;
      this.pending.delete(id);
      window.clearTimeout(waiting.timer);
      const failure = payload.error as { message?: unknown } | undefined;
      if (failure) {
        this.onEvent(`← ${waiting.method} refused: ${String(failure.message ?? "no reason")}`);
        waiting.reject(new RpcError(waiting.method, String(failure.message ?? "refused")));
      } else {
        this.onEvent(`← ${waiting.method} ${JSON.stringify(payload.result).slice(0, 200)}`);
        waiting.resolve(payload.result);
      }
      return;
    }

    if (message.type === "endSession" || message.type === "sessionRejected") {
      this.ended(`the session ended: ${String(message.reason ?? "no reason given")}`);
    }
  }

  private ended(why: string): void {
    this.sessionId = null;
    this.onEvent(why);
    this.abandon(why);
  }

  private abandon(why: string): void {
    for (const [id, waiting] of this.pending) {
      window.clearTimeout(waiting.timer);
      waiting.reject(new RpcError(waiting.method, why));
      this.pending.delete(id);
    }
  }

  private async post(message: Envelope): Promise<Envelope | null> {
    let answer: Response;
    try {
      answer = await fetch(`${this.base}/send`, {
        method: "POST",
        headers: { Authorization: `Bearer ${this.token}`, "Content-Type": "application/json" },
        body: JSON.stringify(message),
      });
    } catch (e) {
      throw new RendezvousError(`POST /send: ${e}`);
    }
    if (answer.status === 429) {
      throw new RendezvousError(
        "the rendezvous is rate-limiting this sign-in (1200 requests a minute).",
      );
    }
    if (answer.status === 400) {
      throw new RendezvousError(
        "the rendezvous says this peer does not exist — its event stream is gone. Connect again.",
      );
    }
    if (!answer.ok) {
      throw new RendezvousError(`POST /send answered HTTP ${answer.status}`);
    }
    try {
      const body = (await answer.json()) as Envelope;
      // `startSession` and `list` answer here; everything else answers on the stream.
      return body && body.type ? body : null;
    } catch {
      return null;
    }
  }
}
