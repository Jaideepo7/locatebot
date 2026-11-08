"""Hardware-free regression: shared SMBus address selection must be serialized."""
import sys
import types
import threading
import time
import unittest
sys.modules.setdefault('smbus2', types.SimpleNamespace(SMBus=lambda *a: None))
from hw import PCA9685, ADS7830

class RaceBus:
    def __init__(self):
        self.address = None
        self.motor_selected = threading.Event()
        self.release = threading.Event()
        self.bad = []
    def write_i2c_block_data(self, addr, reg, data):
        self.address = addr
        self.motor_selected.set()
        self.release.wait(1)
        if self.address != addr:
            self.bad.append((addr, self.address))
    def write_byte(self, addr, value):
        self.address = addr
    def read_byte(self, addr):
        self.address = addr
        return 174

class ThreadingTests(unittest.TestCase):
    def test_adc_cannot_change_address_during_motor_transaction(self):
        bus = RaceBus()
        pwm = PCA9685.__new__(PCA9685)
        pwm.bus, pwm.addr = bus, 0x40
        adc = ADS7830(bus)
        motor = threading.Thread(target=lambda: pwm.set_duty(0, 0))
        motor.start()
        self.assertTrue(bus.motor_selected.wait(1))
        battery = threading.Thread(target=lambda: adc.raw(2))
        battery.start()
        time.sleep(0.05)
        bus.release.set()
        motor.join(1); battery.join(1)
        self.assertFalse(motor.is_alive() or battery.is_alive())
        self.assertEqual(bus.bad, [], 'battery read changed motor SMBus address mid-transfer')

if __name__ == '__main__':
    unittest.main()
