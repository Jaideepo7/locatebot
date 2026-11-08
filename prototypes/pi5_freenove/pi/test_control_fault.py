"""Motor failure must clear old motion intent; no hardware imports are exercised."""
import asyncio
import sys
import time
import types
import unittest
sys.modules.setdefault('smbus2', types.SimpleNamespace(SMBus=lambda *a: None))
sys.modules.setdefault('lidar', types.SimpleNamespace(RPLidar=object))
from motion import State, control_task

class ControlFaultTests(unittest.TestCase):
    def test_latched_fault_blocks_new_motion_requests(self):
        st = State()
        st.control_fault = True
        st.target, st.manual, st.manual_until = .3, (.3, .3), time.time()+10
        st.lidar_cm, st.lidar_t = 500, time.time()
        writes = []
        class Car:
            def _apply(self, *args):
                writes.append(args)
                st.running = False
            def stop(self):
                st.running = False
        asyncio.run(control_task(st, Car(), False))
        self.assertEqual(writes, [], 'faulted controller must not energize motors')

    def test_failed_motor_write_clears_motion_intent(self):
        st = State()
        st.target, st.turn, st.manual, st.manual_until = .3, .2, (.3, .3), time.time()+10
        st.lidar_cm, st.lidar_t = 500, time.time()
        class BrokenCar:
            def _apply(self, *args):
                raise OSError(121, 'Remote I/O error')
            def stop(self):
                st.running = False
        asyncio.run(control_task(st, BrokenCar(), False))
        self.assertEqual(st.control_errs, 1)
        self.assertEqual((st.target, st.cmd, st.turn, st.manual), (0.0, 0.0, 0.0, None))
        self.assertIn('Remote I/O error', st.control_err)

if __name__ == '__main__':
    unittest.main()
