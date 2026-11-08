// End-to-end check of the bridge logic without Telegram/iMessage: a fake chat talks to a running
// `tools/orin_run.py --sim`. Usage: npm run selftest [-- http://127.0.0.1:8090]
import { handle, setRobotUrl, startWatcher, subscribers, ROBOT_URL, type Space } from "./bot.js";

if (process.argv[2]) setRobotUrl(process.argv[2]);

const sent: string[] = [];
const chat: Space = {
  id: "selftest",
  send: async (...c: any[]) => {
    for (const x of c) sent.push(typeof x === "string" ? x : `[${x?.type ?? "content"}]`);
  },
};
subscribers.set("selftest", chat);

const status = async () => (await fetch(`${ROBOT_URL}/status`)).json() as Promise<any>;
const wait = (ms: number) => new Promise((r) => setTimeout(r, ms));
let failures = 0;
const check = (ok: boolean, what: string) => {
  console.log(`${ok ? "PASS" : "FAIL"}  ${what}`);
  if (!ok) failures++;
};

async function say(text: string): Promise<string[]> {
  const before = sent.length;
  try {
    await handle(chat, text);
  } catch (e) {
    sent.push(`Robot error: ${(e as Error).message}`);
  }
  return sent.slice(before);
}

const tick = startWatcher();
await tick(); // prime the watcher with the current state

// 1. Start a search, then stop it in the ways people type it.
for (const stopText of ["/stop", "stop", "Stop", "STOP!", "/stop@LocateAnythingBot", "stop the car", "please stop",
                        "can you halt", "freeze"]) {
  await say("find water bottle");
  await wait(2500);
  await tick(); // the watcher's poll, as the running bot does every 2 s
  const moving = (await status()).app.command.throttle;
  const reply = await say(stopText);
  await wait(300);
  const st = await status();
  const notes = sent.length;
  await tick();
  const extra = sent.slice(notes);
  check(st.skill.call?.name === "stop" && st.app.command.throttle === 0,
    `${JSON.stringify(stopText)} stops the car (throttle ${moving} -> ${st.app.command.throttle}); reply ${JSON.stringify(reply)}` +
    (extra.length ? `; then notified ${JSON.stringify(extra)}` : ""));
}

// 2. "find stop sign" searches for the item instead of stopping.
{
  const r = await say("find stop sign");
  await wait(300);
  const st = await status();
  check(st.skill.call?.name === "search" && st.skill.call.args.items[0] === "stop sign",
    `"find stop sign" searches for "stop sign"; reply ${JSON.stringify(r)}`);
  const r2 = await say("stop");
  await wait(300);
  const st2 = await status();
  check(st2.skill.call?.name === "stop", `then "stop" stops it again; reply ${JSON.stringify(r2)}, robot: ${st2.skill.call?.name} ${st2.skill.state}`);
}

// 3. Stop with the robot unreachable says so loudly.
{
  const real = ROBOT_URL;
  setRobotUrl("http://127.0.0.1:9");
  const r = await say("stop");
  setRobotUrl(real);
  check(r.length === 1 && r[0]!.startsWith("STOP FAILED"), `stop with robot down -> ${JSON.stringify(r)}`);
}

// 4. The other commands still answer.
for (const t of ["status", "map", "help"]) {
  const r = await say(t);
  check(r.length > 0 && !r[0]!.startsWith("Robot error"), `${JSON.stringify(t)} -> ${JSON.stringify(r[0]?.slice(0, 60))}`);
}

console.log(failures ? `${failures} FAILED` : "all passed");
process.exit(failures ? 1 : 0);
