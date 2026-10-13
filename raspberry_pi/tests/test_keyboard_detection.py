"""Keyboard discovery without accessing Linux input devices."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control_arm import detect_keyboard


class KeyboardDetectionTests(unittest.TestCase):
    def test_skips_non_keyboards_and_closes_handles(self):
        codes = SimpleNamespace(EV_KEY=1, KEY_W=2, KEY_E=3, KEY_Q=4,
                                KEY_UP=5, KEY_DOWN=6, KEY_LEFT=7, KEY_RIGHT=8)
        closed = []
        class Device:
            def __init__(self, path): self.path = path
            def close(self): closed.append(self.path)
            def capabilities(self):
                return {1: [2] if self.path == '/a-mouse' else list(range(2, 9))}
        found = detect_keyboard(Device, codes, lambda: ['/b-keyboard', '/a-mouse'])
        self.assertEqual(found, '/b-keyboard')
        self.assertEqual(closed, ['/a-mouse', '/b-keyboard'])

    def test_no_keyboard_explains_local_input_requirement(self):
        with self.assertRaisesRegex(ValueError, 'Connect a USB keyboard'):
            detect_keyboard(None, SimpleNamespace(KEY_W=1, KEY_E=2, KEY_Q=3,
                            KEY_UP=4, KEY_DOWN=5, KEY_LEFT=6, KEY_RIGHT=7), lambda: [])
