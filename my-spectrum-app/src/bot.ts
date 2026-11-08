// Robot-facing logic of the chat bridge: command parsing, Orin HTTP calls, status notifications.
// Kept free of Spectrum setup so it can be tested against tools/orin_run.py --sim (see src/selftest.ts).
import { attachment } from "spectrum-ts";

export let ROBOT_URL = (process.env.ROBOT_URL || "http://127.0.0.1:8090").replace(/\/+$/, "");
export function setRobotUrl(url: string) { ROBOT_URL = url.replace(/\/+$/, ""); }

const POLL_MS = 2000;
const HTTP_TIMEOUT_MS = 5000;

export type Space = { id: string; send: (...c: any[]) => Promise<unknown> };

// ------------------------------------------------------------------ robot API
async function robot(path: string, body?: object): Promise<any> {
  const res = await fetch(ROBOT_URL + path, {
    method: body === undefined ? "GET" : "POST",
    headers: body === undefined ? {} : { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
    signal: AbortSignal.timeout(HTTP_TIMEOUT_MS),
  });
  const ctype = res.headers.get("content-type") || "";
  const data = ctype.includes("application/json") ? await res.json() : Buffer.from(await res.arrayBuffer());
  if (!res.ok) throw new Error((data as any)?.error || `HTTP ${res.status}`);
  return data;
}

const skill = (name: string, args: object = {}) => robot("/skill", { name, args });

function splitItems(s: string): string[] {
  return s.split(/,|\band\b|\n/i).map((x) => x.trim()).filter(Boolean);
}

function summary(st: any): string {
  const sk = st.skill || {};
  const call = sk.call ? `${sk.call.name} ${JSON.stringify(sk.call.args)}` : "none";
  const lines = [
    `Skill: ${call} (${sk.state})${sk.detail ? ` - ${sk.detail}` : ""}`,
    `Pose: x=${st.pose?.x} y=${st.pose?.y} heading=${st.pose?.theta_deg} deg`,
    `Explored: ${Math.round((st.explored_frac ?? 0) * 100)}%`,
  ];
  if (st.safety_reason) lines.push(`SAFETY STOP: ${st.safety_reason}`);
  const items = (st.items || []) as any[];
  if (items.length) {
    lines.push("Items:");
    for (const i of items) {
      lines.push(`  ${i.confirmed ? "found" : "maybe"} ${i.name} at (${i.x}, ${i.y}) - ${i.sightings} sightings`);
    }
  }
  const p = st.brain?.pending;
  if (p) lines.push(`Waiting for approval: ${p.name} ${JSON.stringify(p.args)} - ${p.reason} (/approve or /reject)`);
  return lines.join("\n");
}

const HELP = `locatebot commands:
/find water bottle, red box  - search the room for items
/explore                     - map the room
/goto <item>                 - drive to a found item
/follow <item>               - follow an item
/look                        - slow look-around turn
/home                        - return to the start pose
/stop                        - STOP the car
/status                      - what the robot is doing
/map                         - map picture
/savemap <name>              - save the session map
/approve, /reject            - answer an LLM proposal
The slash is optional: "find my keys" or "stop" work too.`;

// ------------------------------------------------------------------ command handling
const STOP_WORDS = /\b(stop|halt|freeze|brake)\b/i;
const ITEM_COMMANDS = new Set(["find", "search", "locate", "goto", "go_to", "follow", "savemap", "save_map"]);

// Newest /status seen by the watcher, used for reply text only. The stop never waits on a status request.
let lastStatus: any = null;

async function stop(space: Space): Promise<void> {
  let was = "";
  const sk = lastStatus?.skill;
  if (sk?.call && sk.call.name !== "stop" && sk.state === "running") {
    const items = sk.call.args?.items ?? sk.call.args?.item;
    was = ` (cancelled ${sk.call.name}${items ? ` ${[].concat(items).join(", ")}` : ""})`;
  }
  try {
    await robot("/stop", {});
  } catch (e) {
    try { await robot("/stop", {}); } catch {
      return void (await space.send(`STOP FAILED: ${(e as Error).message}. Use the kill switch or the Pi page.`));
    }
  }
  await space.send(`Stopped${was}.`);
}

export async function handle(space: Space, raw: string): Promise<void> {
  const text = raw.trim();
  // "/find@MyBot keys" -> cmd "find", rest "keys"
  const m = text.match(/^\/?([a-z_]+)(?:@\S+)?\s*([\s\S]*)$/i);
  let cmd = (m?.[1] || "").toLowerCase();
  const rest = (m?.[2] || "").trim();
  // Safety: a stop word anywhere ("please stop", "can you halt") stops the car, unless the message
  // is an item request such as "find stop sign".
  if (STOP_WORDS.test(text) && !ITEM_COMMANDS.has(cmd)) cmd = "stop";

  switch (cmd) {
    case "stop":
      return stop(space);
    case "find":
    case "search":
    case "locate": {
      const items = splitItems(rest);
      if (!items.length) return void (await space.send("What should I look for? e.g. /find water bottle, keys"));
      await skill("search", { items });
      return void (await space.send(`Searching for: ${items.join(", ")}. I'll message you when I find them.`));
    }
    case "explore":
    case "map_room":
      await skill("explore");
      return void (await space.send("Exploring the room."));
    case "goto":
    case "go_to":
      if (!rest) return void (await space.send("Go to what? e.g. /goto keys"));
      await skill("go_to", { item: rest });
      return void (await space.send(`Driving to ${rest}.`));
    case "follow":
      if (!rest) return void (await space.send("Follow what? e.g. /follow person"));
      await skill("follow", { item: rest });
      return void (await space.send(`Following ${rest}.`));
    case "look":
    case "look_around":
      await skill("look_around");
      return void (await space.send("Looking around."));
    case "home":
    case "return_home":
      await skill("return_home");
      return void (await space.send("Going home."));
    case "status":
      return void (await space.send(summary(await robot("/status"))));
    case "map": {
      const png: Buffer = await robot("/map.png");
      return void (await space.send(attachment(png, { name: "map.png", mimeType: "image/png" })));
    }
    case "savemap":
    case "save_map": {
      const r = await robot("/save_map", { name: rest });
      return void (await space.send(`Map saved: ${(r.files || []).join(", ")}`));
    }
    case "approve":
      return void (await space.send((await robot("/approve", {})).ok ? "Approved." : "Nothing to approve."));
    case "reject":
      return void (await space.send((await robot("/reject", {})).ok ? "Rejected." : "Nothing to reject."));
    default:
      return void (await space.send(HELP));
  }
}

// ------------------------------------------------------------------ notifications
// Chats that sent a command get told when an item is confirmed, a skill ends, or a safety stop fires.
export const subscribers = new Map<string, Space>();

async function notify(msg: string) {
  for (const s of subscribers.values()) await s.send(msg).catch((e) => console.error("notify failed:", e));
}

export function startWatcher(): () => Promise<void> {
  const confirmed = new Set<string>();
  let lastSkillKey = "";
  let lastSafety: string | null = null;
  let lastPending = "";
  let down = false;

  const tick = async () => {
    let st: any;
    try {
      st = await robot("/status");
      lastStatus = st;
      if (down) await notify("Robot connection back.");
      down = false;
    } catch (e) {
      if (!down) await notify(`Can't reach the robot at ${ROBOT_URL}: ${(e as Error).message}`);
      down = true;
      return;
    }
    for (const i of st.items || []) {
      if (i.confirmed && !confirmed.has(i.name)) {
        confirmed.add(i.name);
        await notify(`Found ${i.name} at (${i.x}, ${i.y}) m.`);
      }
    }
    const sk = st.skill || {};
    const skillKey = `${sk.call?.name}|${sk.state}`;
    if (skillKey !== lastSkillKey && ["succeeded", "failed", "cancelled"].includes(sk.state) && sk.call &&
        sk.call.name !== "stop") {
      await notify(`${sk.call.name} ${sk.state}${sk.detail ? `: ${sk.detail}` : ""}`);
    }
    lastSkillKey = skillKey;
    if (st.safety_reason && st.safety_reason !== lastSafety) await notify(`Safety stop: ${st.safety_reason}`);
    lastSafety = st.safety_reason || null;
    const p = st.brain?.pending;
    const pendingKey = p ? JSON.stringify(p) : "";
    if (pendingKey && pendingKey !== lastPending) {
      await notify(`Robot wants to: ${p.name} ${JSON.stringify(p.args)}\n${p.reason}\n/approve or /reject`);
    }
    lastPending = pendingKey;
  };
  setInterval(tick, POLL_MS);
  return tick;
}

