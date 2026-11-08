// Telegram / iMessage -> locatebot bridge.
//
// Chat messages become calls to the Orin operator HTTP API (tools/orin_run.py):
//   POST /skill {name, args}, POST /stop, GET /status, GET /map.png, POST /save_map, /approve, /reject.
// The robot's own safety chain is unchanged: the Pi LiDAR stop still gates every motion command.
//
// Env (.env): SPECTRUM_PROJECT_ID, SPECTRUM_PROJECT_SECRET (Photon dashboard)
//   IMESSAGE_ENABLED=1            turn on Spectrum Cloud iMessage
//   SPECTRUM_TELEGRAM_BOT_TOKEN   turn on Telegram (leave empty to disable)
//   ROBOT_URL                     Orin page, default http://127.0.0.1:8090
//   RESTRICT_SENDERS=1            only let ALLOWED_SENDERS use the bot (off by default: anyone can)
//   ALLOWED_SENDERS               comma-separated phone numbers (+15551234567), Apple ID emails
//                                 or Telegram chat ids allowed to command the robot (required)
import { Spectrum } from "spectrum-ts";
import { imessage } from "spectrum-ts/providers/imessage";
import { telegram } from "@spectrum-ts/telegram";
import { ROBOT_URL, handle, startWatcher, subscribers, type Space } from "./bot.js";

const RESTRICT = /^(1|true|yes)$/i.test(process.env.RESTRICT_SENDERS || "");
// ALLOWED_CHAT_IDS is the older name for the same list.
const ALLOWED = new Set(
  `${process.env.ALLOWED_SENDERS || ""},${process.env.ALLOWED_CHAT_IDS || ""}`
    .split(",").map((s) => s.trim().toLowerCase()).filter(Boolean),
);
// `npm run telegram` / `imessage` / `both` pass the provider as an argument; `npm run start` uses .env.
const ONLY = (process.argv[2] || "").toLowerCase();
if (ONLY && !["telegram", "imessage", "both"].includes(ONLY)) {
  console.error(`Unknown provider "${ONLY}": use telegram, imessage or both`);
  process.exit(1);
}
const IMESSAGE_ON = ONLY ? ONLY !== "telegram" : /^(1|true|yes)$/i.test(process.env.IMESSAGE_ENABLED || "");
const TELEGRAM_TOKEN = ONLY === "imessage" ? "" : process.env.SPECTRUM_TELEGRAM_BOT_TOKEN || "";
if ((ONLY === "telegram" || ONLY === "both") && !TELEGRAM_TOKEN) {
  console.error(`npm run ${ONLY} needs SPECTRUM_TELEGRAM_BOT_TOKEN in .env`);
  process.exit(1);
}
// ------------------------------------------------------------------ main
if (!RESTRICT) {
  console.warn("Open mode: anyone who messages the bot can drive the robot (set RESTRICT_SENDERS=1 to limit it).");
} else if (ALLOWED.size === 0) {
  console.warn("RESTRICT_SENDERS=1 but ALLOWED_SENDERS is empty: everyone is refused.");
}

const providers: any[] = [];
// Log once per live iMessage subscription, so a silent tab can be told apart from a dead stream.
const liveSubs = new Set<string>();
if (IMESSAGE_ON) providers.push(imessage.config({
  onHeartbeat: (hb) => {
    if (liveSubs.has(hb.subscriptionId)) return;
    liveSubs.add(hb.subscriptionId);
    console.log(`iMessage ${hb.stream} stream live (line ${hb.phone})`);
  },
}));
if (TELEGRAM_TOKEN) providers.push(telegram.config({ botToken: TELEGRAM_TOKEN }));
if (!providers.length) {
  console.error("No chat provider: set IMESSAGE_ENABLED=1 and/or SPECTRUM_TELEGRAM_BOT_TOKEN in .env");
  process.exit(1);
}

const app = await Spectrum({
  projectId: process.env.SPECTRUM_PROJECT_ID!,
  projectSecret: process.env.SPECTRUM_PROJECT_SECRET!,
  providers,
});

console.log(`locatebot bridge running (${[IMESSAGE_ON && "imessage", TELEGRAM_TOKEN && "telegram"]
  .filter(Boolean).join(" + ")}), robot at ${ROBOT_URL}`);
startWatcher();

for await (const [space, message] of app.messages) {
  const preview = message.content.type === "text" ? JSON.stringify(message.content.text.slice(0, 80)) : "";
  console.log(`[${new Date().toISOString()}] ${message.direction} ${message.platform} ` +
    `from=${message.sender?.id ?? "?"} chat=${space.id} ${message.content.type} ${preview}`);
  if (message.direction === "outbound" || message.content.type !== "text") continue;
  // iMessage: sender.id is the phone number / Apple ID email. Telegram: space.id is the chat id.
  const who = (message.platform === "telegram" ? space.id : message.sender?.id || "").toLowerCase();
  if (RESTRICT && (!who || !ALLOWED.has(who))) {
    console.log(`refused ${message.platform} sender ${who || "(unknown)"}`);
    await space.send(`Not authorised. Add this to ALLOWED_SENDERS: ${who || "(unknown sender)"}`).catch(() => {});
    continue;
  }
  subscribers.set(`${message.platform}:${space.id}`, space as Space);
  try {
    await handle(space as Space, message.content.text);
    console.log("  -> replied");
  } catch (e) {
    console.error(e);
    await space.send(`Robot error: ${(e as Error).message}`).catch(() => {});
  }
}
