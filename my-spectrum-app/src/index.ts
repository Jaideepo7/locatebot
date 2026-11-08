import { Spectrum } from "spectrum-ts";
import { telegram } from "@spectrum-ts/telegram";

const app = await Spectrum({
  projectId: process.env.SPECTRUM_PROJECT_ID!,
  projectSecret: process.env.SPECTRUM_PROJECT_SECRET!,
  providers: [
    telegram.config({
      botToken: process.env.SPECTRUM_TELEGRAM_BOT_TOKEN!,
    }),
  ],
});

console.log("Bot is running");

for await (const [space, message] of app.messages) {
  if (message.content.type === "text") {
    console.log(`message from ${space.id}: ${message.content.text}`);
    await space.send(`echo: ${message.content.text}`);
  }
}
