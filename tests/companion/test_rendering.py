"""Smoke the retained Python/BLE viewer after the desktop migration."""
import os
os.environ['MPLBACKEND'] = 'Agg'
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'companion'))
import visualizer

class RenderTests(unittest.TestCase):
    def test_tube_marble_and_return_render_without_artist_growth(self):
        with patch.object(visualizer.threading.Thread, 'start'), patch.object(visualizer.Visualizer, '_start_audio'):
            app = visualizer.Visualizer('COM_TEST')
        try:
            for mode in ['TEXTURED', 'MARBLES', 'ROLLING']:
                app._on_status(mode, 1000)
                app._draw_frame()
                app.fig.canvas.draw()
                artist_count = len(app.ax.patches) + len(app.ax.collections) + len(app.ax.lines)
                app._draw_frame()
                self.assertEqual(artist_count, len(app.ax.patches) + len(app.ax.collections) + len(app.ax.lines))
                if mode == 'MARBLES':
                    low, high = app.ax.get_zlim()
                    self.assertGreater(low, high, 'positive Z must display downward')
        finally:
            # The thread is deliberately not started in a renderer-only test.
            with patch.object(app._transport_thread, 'join'):
                app._on_close(None)
            visualizer.plt.close(app.fig)

if __name__ == '__main__':
    unittest.main()
