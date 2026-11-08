// Telegram -> locatebot bridge.
//
// Telegram messages become calls to the Orin operator HTTP API (tools/orin_run.py):
//   POST /skill {name, args}, POST /stop, GET /status, GET /map.png, POST /save_map, /approve, /reject.
// The robot's own safety chain is unchanged: the Pi LiDAR stop still gates every motion command.
//
// Env (.env): SPECTRUM_PROJECT_ID, SPECTRUM_PROJECT_SECRET, SPECTRUM_TELEGRAM_BOT_TOKEN,
//   ROBOT_URL         Orin page, default http://127.0.0.1:8090
//   ALLOWED_CHAT_IDS  comma-separated Telegram chat ids allowed to command the robot (required)
import { Spectrum, attachment } from "spectrum-ts";
import { telegram } from "@spectrum-ts/telegram";

const ROBOT_URL = (process.env.ROBOT_URL || "http://127.0.0.1:8090").replace(/\/+$/, "");
const ALLOWED = new Set(
  (process.env.ALLOWED_CHAT_IDS || "").split(",").map((s) => s.trim()).filter(Boolean),
);
const POLL_MS = 2000;
const HTTP_TIMEOUT_MS = 5000;

type Space = { id: string; send: (...c: any[]) => Promise<unknown> };

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
Plain text like "find my keys" also works. "stop" always stops.`;

// ------------------------------------------------------------------ command handling
async function handle(space: Space, raw: string): Promise<void> {
  const text = raw.trim();
  // "/find@MyBot keys" -> cmd "find", rest "keys"
  const m = text.match(/^\/?([a-z_]+)(?:@\S+)?\s*([\s\S]*)$/i);
  const cmd = (m?.[1] || "").toLowerCase();
  const rest = (m?.[2] || "").trim();

  switch (cmd) {
    case "stop":
    case "halt":
      await robot("/stop", {});
      return void (await space.send("Stopped."));
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
const subscribers = new Map<string, Space>();

async function notify(msg: string) {
  for (const s of subscribers.values()) await s.send(msg).catch((e) => console.error("notify failed:", e));
}

function startWatcher() {
  const confirmed = new Set<string>();
  let lastSkillKey = "";
  let lastSafety: string | null = null;
  let lastPending = "";
  let down = false;

  setInterval(async () => {
    let st: any;
    try {
      st = await robot("/status");
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
    if (skillKey !== lastSkillKey && ["succeeded", "failed", "cancelled"].includes(sk.state) && sk.call) {
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
  }, POLL_MS);
}

// ------------------------------------------------------------------ main
if (ALLOWED.size === 0) {
  console.warn("ALLOWED_CHAT_IDS is empty: every chat is refused. Message the bot once to see your chat id.");
}

const app = await Spectrum({
  projectId: process.env.SPECTRUM_PROJECT_ID!,
  projectSecret: process.env.SPECTRUM_PROJECT_SECRET!,
  providers: [telegram.config({ botToken: process.env.SPECTRUM_TELEGRAM_BOT_TOKEN! })],
});

console.log(`locatebot Telegram bridge running, robot at ${ROBOT_URL}`);
startWatcher();

for await (const [space, message] of app.messages) {
  if (message.direction === "outbound" || message.content.type !== "text") continue;
  if (!ALLOWED.has(space.id)) {
    console.log(`refused chat ${space.id}`);
    await space.send(`Not authorised. Add this chat id to ALLOWED_CHAT_IDS: ${space.id}`).catch(() => {});
    continue;
  }
  subscribers.set(space.id, space as Space);
  try {
    await handle(space as Space, message.content.text);
  } catch (e) {
    console.error(e);
    await space.send(`Robot error: ${(e as Error).message}`).catch(() => {});
  }
}
