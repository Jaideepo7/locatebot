#!/usr/bin/env python3
"""Emergency stop: turn every PCA9685 channel off."""
from smbus2 import SMBus
SMBus(1).write_byte_data(0x40, 0xFD, 0x10)
print("all motor/servo channels OFF")
