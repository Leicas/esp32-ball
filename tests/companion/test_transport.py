import collections
import queue
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "companion"))
import visualizer


def bare_visualizer():
    obj = visualizer.Visualizer.__new__(visualizer.Visualizer)
    obj._state_lock = threading.RLock()
    obj._stop_event = threading.Event()
    obj._cmd_q = queue.Queue(maxsize=16)
    obj._pending_commands = collections.deque()
    obj._command_ack_timeout_s = 2.0
    obj._stale_after_s = 0.5
    obj._last_telem_at = time.monotonic()
    obj._connected_at = time.monotonic()
    obj.telemetry_age_s = 0.0
    obj.connection_state = "connected"
    obj.connection_detail = "USB telemetry active"
    obj.last_command_feedback = ""
    obj._connected = True
    obj._conn_label = "USB"
    obj._target = "COM_TEST"
    obj._baud = 115200
    obj._serial_handle = None
    obj._ble_loop = None
    obj._ble_client = None
    obj.mode = "ROLLING"
    obj.cavity_mm = 1000.0
    obj.rebound = None
    obj.haptic_gain = None
    obj._texture_state = ((0.0, 0.0),)
    obj._aud_impact = 0
    obj._prev_x_mm = 500.0
    obj._aud_impact_amp = 1.0
    obj._aud_x = 500.0
    obj._aud_v = 0.0
    obj._aud_play = True
    obj._aud_sample_index = 0
    return obj


class TransportTests(unittest.TestCase):
    def test_commands_require_fresh_telemetry_and_acknowledge_state(self):
        obj = bare_visualizer()
        obj._send_cmd(b"t")
        self.assertEqual(obj._cmd_q.get_nowait(), b"t")
        obj._mark_command_sent(b"t")
        obj._parse_line("# mode=TEXTURED rebound=1 cavity_mm=1050 haptic_gain=0.7")

        self.assertEqual(obj.mode, "TEXTURED")
        self.assertTrue(obj.rebound)
        self.assertEqual(obj.cavity_mm, 1050.0)
        self.assertEqual(obj.haptic_gain, 0.7)
        self.assertEqual(obj.last_command_feedback, "Confirmed mode TEXTURED")

        obj._last_telem_at = time.monotonic() - 0.7
        obj._refresh_transport_state()
        obj._send_cmd(b"+")
        self.assertTrue(obj._cmd_q.empty())
        self.assertIn("Not sent", obj.last_command_feedback)
        self.assertEqual(obj.connection_state, "stale")

    def test_bounded_queue_rejects_without_creating_pending_ack(self):
        obj = bare_visualizer()
        for _ in range(obj._cmd_q.maxsize):
            obj._cmd_q.put_nowait(b"+")
        obj._send_cmd(b"t")
        self.assertEqual(len(obj._pending_commands), 0)
        self.assertIn("queue full", obj.last_command_feedback)

    def test_invalid_telemetry_never_reaches_renderer_or_audio_state(self):
        obj = bare_visualizer()
        obj._on_telem = unittest.mock.Mock()
        obj._parse_line("$nan,500,0.2")
        self.assertFalse(obj._on_telem.called)

        out = np.ones((32, 1), dtype=np.float32)
        obj._connected = True
        obj.telemetry_age_s = 0.8
        obj._audio_cb(out, len(out), None, None)
        self.assertTrue(np.all(out == 0.0))

    def test_serial_error_closes_port_before_reopening_and_stops_cleanly(self):
        obj = bare_visualizer()
        opened_twice = threading.Event()
        closed = []
        open_count = 0

        class FakeSerial:
            def __init__(self, *args, **kwargs):
                nonlocal open_count
                open_count += 1
                self.index = open_count
                self.did_fail = False
                self.is_closed = False
                if open_count >= 2:
                    opened_twice.set()

            def write(self, data):
                return len(data)

            def readline(self):
                time.sleep(0.005)
                if self.index == 1 and not self.did_fail:
                    self.did_fail = True
                    raise OSError("simulated cable loss")
                return b""

            def close(self):
                self.is_closed = True
                closed.append(self.index)

        worker = threading.Thread(target=obj._serial_loop)
        with patch.object(visualizer.serial, "Serial", FakeSerial):
            worker.start()
            self.assertTrue(opened_twice.wait(3.0), "serial worker did not retry")
            obj._stop_event.set()
            worker.join(timeout=1.0)

        self.assertFalse(worker.is_alive(), "serial worker did not stop promptly")
        self.assertEqual(closed, [1, 2])
        self.assertFalse(obj._connected)
        self.assertTrue(obj._cmd_q.empty())


if __name__ == "__main__":
    unittest.main()
