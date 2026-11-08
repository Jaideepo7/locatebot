"""Minimal drivers for the car board: PCA9685 (motors/servos), ADS7830 (ADC), SSD1306 OLED.
Only needs smbus2 + Pillow, both already on Raspberry Pi OS."""
import time
from smbus2 import SMBus

I2C_BUS = 1


class PCA9685:
    MODE1, MODE2, PRESCALE, LED0 = 0x00, 0x01, 0xFE, 0x06
    ALL_OFF_H = 0xFD

    def __init__(self, bus=None, addr=0x40, freq=50):
        self.bus = bus or SMBus(I2C_BUS)
        self.addr = addr
        self.all_off()
        self.set_freq(freq)

    def set_freq(self, freq):
        prescale = round(25_000_000 / (4096 * freq)) - 1
        old = self.bus.read_byte_data(self.addr, self.MODE1)
        self.bus.write_byte_data(self.addr, self.MODE1, (old & 0x7F) | 0x10)  # sleep
        self.bus.write_byte_data(self.addr, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.addr, self.MODE1, 0x20)                 # wake, auto-increment
        time.sleep(0.005)
        self.bus.write_byte_data(self.addr, self.MODE1, 0xA0)                 # restart
        self.freq = freq

    def set_duty(self, ch, duty):
        """duty 0..4095 (0 = fully off, 4095 = fully on)."""
        duty = max(0, min(4095, int(duty)))
        if duty == 0:
            data = [0, 0, 0, 0x10]
        elif duty >= 4095:
            data = [0, 0x10, 0, 0]
        else:
            data = [0, 0, duty & 0xFF, duty >> 8]
        self.bus.write_i2c_block_data(self.addr, self.LED0 + 4 * ch, data)

    def set_pulse_us(self, ch, us):
        """For servos: pulse width in microseconds (about 500-2500)."""
        self.set_duty(ch, us * self.freq * 4096 / 1_000_000)

    def all_off(self):
        self.bus.write_byte_data(self.addr, self.ALL_OFF_H, 0x10)


class ADS7830:
    def __init__(self, bus=None, addr=0x48):
        self.bus = bus or SMBus(I2C_BUS)
        self.addr = addr

    def raw(self, ch):
        cmd = 0x84 | ((((ch << 2) | (ch >> 1)) & 0x07) << 4)
        self.bus.write_byte(self.addr, cmd)
        self.bus.read_byte(self.addr)
        return self.bus.read_byte(self.addr)

    def all_raw(self):
        return [self.raw(c) for c in range(8)]


class OLED:
    """SSD1306 128x64 over I2C."""
    W, H = 128, 64

    def __init__(self, bus=None, addr=0x3C):
        self.bus = bus or SMBus(I2C_BUS)
        self.addr = addr
        for c in (0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
                  0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF):
            self.bus.write_byte_data(self.addr, 0x00, c)

    def show_image(self, img):
        img = img.convert("1")
        px = img.load()
        buf = []
        for page in range(8):
            for x in range(self.W):
                b = 0
                for bit in range(8):
                    if px[x, page * 8 + bit]:
                        b |= 1 << bit
                buf.append(b)
        for c in (0x21, 0, 127, 0x22, 0, 7):
            self.bus.write_byte_data(self.addr, 0x00, c)
        for i in range(0, len(buf), 16):
            self.bus.write_i2c_block_data(self.addr, 0x40, buf[i:i + 16])

    def text(self, big, small=""):
        from PIL import Image, ImageDraw, ImageFont
        img = Image.new("1", (self.W, self.H), 0)
        d = ImageDraw.Draw(img)
        fpath = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        try:
            fbig, fsmall = ImageFont.truetype(fpath, 30), ImageFont.truetype(fpath, 12)
        except OSError:
            fbig = fsmall = ImageFont.load_default()
        d.text((2, 2), big, font=fbig, fill=1)
        if small:
            d.text((2, 46), small, font=fsmall, fill=1)
        self.show_image(img)

    def off(self):
        self.bus.write_byte_data(self.addr, 0x00, 0xAE)
