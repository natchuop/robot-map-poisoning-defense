from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path


def _install_controller_stub() -> None:
    if 'controller' in sys.modules:
        return

    controller = types.ModuleType('controller')

    class _Keyboard:
        UP = 1
        DOWN = 2
        LEFT = 3
        RIGHT = 4

        def enable(self, _timestep):
            return None

        def getKey(self):
            return -1

    class _Robot:
        def getName(self):
            return 'robot_2'

        def getCustomData(self):
            return ''

        def getBasicTimeStep(self):
            return 64

    controller.Keyboard = _Keyboard
    controller.Robot = _Robot
    sys.modules['controller'] = controller


def _load_module():
    _install_controller_stub()
    repo_root = Path(__file__).resolve().parents[3]
    module_path = repo_root / 'webots/robot_controllers/user_controlled_robot/user_controlled_robot.py'
    spec = importlib.util.spec_from_file_location('user_controlled_robot_under_test', module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_sequence_normalization_accepts_json_points_and_pairs() -> None:
    module = _load_module()

    points = module.normalize_fake_obstacle_sequence('[{"x": 3.1, "y": -0.111}, [3.1, 0.089], {"x": "3.1", "y": "0.289"}]')

    assert points == [(3.1, -0.111), (3.1, 0.089), (3.1, 0.289)]


def test_sequence_click_uses_sequence_source_and_coordinates() -> None:
    module = _load_module()

    click = module.build_fake_obstacle_click(
        {
            'fake_obstacle_frame_id': 'map',
            'fake_obstacle_sequence_source': 'observer_bot_sequence',
        },
        obstacle_point=(3.1, 2.41),
    )

    assert click['clicked_point']['x'] == 3.1
    assert click['clicked_point']['y'] == 2.41
    assert click['clicked_point']['frame_id'] == 'map'
    assert click['clicked_point']['source'] == 'observer_bot_sequence'
