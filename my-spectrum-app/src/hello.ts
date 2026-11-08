// Send one iMessage from the bot to a phone number, to find out which number the bot texts from.
// Usage: npm run imessage-hello -- +15551234567
// On Photon's shared pool the recipient must be added as a project user in the Photon dashboard.
import { Spectrum } from "spectrum-ts";
import { imessage } from "spectrum-ts/providers/imessage";

const to = (process.argv[2] || "").trim();
if (!/^\+\d{8,15}$/.test(to)) {
  console.error("Give a phone number with + and country code, e.g. npm run imessage-hello -- +15551234567");
  process.exit(1);
}

const app = await Spectrum({
  projectId: process.env.SPECTRUM_PROJECT_ID!,
  projectSecret: process.env.SPECTRUM_PROJECT_SECRET!,
  providers: [imessage.config()],
});

try {
  const im = imessage(app);
  const space = await im.space.create(await im.user(to));
  await space.send("Hi from locatebot. Reply to this conversation with 'status' to talk to the robot.");
  console.log(`Sent to ${to}. Reply in that conversation; the number it came from is the bot's number for you.`);
} catch (e) {
  console.error("Send failed:", (e as Error).message);
  process.exitCode = 1;
} finally {
  await app.stop();
}
