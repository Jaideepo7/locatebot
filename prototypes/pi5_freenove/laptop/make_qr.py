#!/usr/bin/env python3
"""make_qr.py URL OUT.png [caption line ...] - a scannable QR card in the app's pixel style.

Dark ink on parchment (high contrast, so every phone camera reads it), title in the app's pixel font.
Needs the `qrcode` package (pip install qrcode); not a dependency of the locatebot package.
"""
import os, sys
import qrcode
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
FONT = os.path.join(HERE, "..", "pi", "web", "fonts", "PixelifySans.woff2")
NIGHT, PARCH, GOLD, EMBER, INK = (23, 13, 46), (242, 226, 182), (240, 191, 79), (165, 81, 42), (23, 13, 46)


def card(url, out, lines):
    qr = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M, border=4, box_size=1)
    qr.add_data(url); qr.make(fit=True)
    m = qr.make_image(fill_color=INK, back_color=PARCH).convert("RGB")
    scale = max(8, 520 // m.size[0])
    m = m.resize((m.size[0] * scale, m.size[1] * scale), Image.NEAREST)
    title_f = ImageFont.truetype(FONT, 64); body_f = ImageFont.truetype(FONT, 30)
    W = m.size[0] + 160
    H = 150 + m.size[1] + 60 + 46 * (len(lines) + 1) + 50
    im = Image.new("RGB", (W, H), NIGHT); d = ImageDraw.Draw(im)
    # stepped gold frame (notched corners) with an ember drop step, like the app
    x0, y0, x1, y1 = 40, 130, W - 40, 130 + m.size[1] + 40
    d.rectangle([x0 + 8, y1, x1 - 8, y1 + 16], fill=EMBER)
    d.rectangle([x0 + 8, y0 - 8, x1 - 8, y1 + 8], fill=GOLD); d.rectangle([x0, y0, x1, y1], fill=GOLD)
    d.rectangle([x0 + 8, y0 + 8, x1 - 8, y1 - 8], fill=PARCH)
    im.paste(m, ((W - m.size[0]) // 2, y0 + 20))
    t = "LocateBot"; tw = d.textlength(t, font=title_f); tx = (W - tw) / 2
    for dx, dy in ((4, 0), (-4, 0), (0, 4), (0, -4)):
        d.text((tx + dx, 30 + dy), t, font=title_f, fill=EMBER)
    d.text((tx, 30), t, font=title_f, fill=GOLD)
    y = y1 + 50
    for i, line in enumerate([url] + list(lines)):
        f = body_f; lw = d.textlength(line, font=f)
        while lw > W - 60 and f.size > 16:
            f = ImageFont.truetype(FONT, f.size - 2); lw = d.textlength(line, font=f)
        d.text(((W - lw) / 2, y), line, font=f, fill=(239, 232, 255) if i else GOLD); y += 46
    im.save(out)
    return out


if __name__ == "__main__":
    print(card(sys.argv[1], sys.argv[2], sys.argv[3:]))
