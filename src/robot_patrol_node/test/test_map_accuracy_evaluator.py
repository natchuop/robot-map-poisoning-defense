from __future__ import annotations

import sys
import types

import numpy as np
import pytest

from robot_patrol_node.attack_region_tracker import AttackRegionDefinition, AttackRegionTracker
from robot_patrol_node.map_metrics import MapGeometry, MapMetrics, OccupancyThresholds, compute_map_metrics


def _install_evaluator_import_stubs() -> None:
    if 'ament_index_python' not in sys.modules:
        ament_index_python = types.ModuleType('ament_index_python')
        packages = types.ModuleType('ament_index_python.packages')
        packages.get_package_share_directory = lambda _package_name: '/tmp'
        ament_index_python.packages = packages
        sys.modules['ament_index_python'] = ament_index_python
        sys.modules['ament_index_python.packages'] = packages

    if 'nav_msgs' not in sys.modules:
        nav_msgs = types.ModuleType('nav_msgs')
        nav_msgs_msg = types.ModuleType('nav_msgs.msg')
        nav_msgs_msg.OccupancyGrid = type('OccupancyGrid', (), {})
        nav_msgs.msg = nav_msgs_msg
        sys.modules['nav_msgs'] = nav_msgs
        sys.modules['nav_msgs.msg'] = nav_msgs_msg

    if 'rclpy' not in sys.modules:
        rclpy = types.ModuleType('rclpy')
        rclpy.init = lambda: None
        rclpy.spin = lambda _node: None
        rclpy.shutdown = lambda: None
        rclpy.ok = lambda: False
        node_module = types.ModuleType('rclpy.node')
        node_module.Node = type('Node', (), {})
        qos_module = types.ModuleType('rclpy.qos')
        qos_module.DurabilityPolicy = type('DurabilityPolicy', (), {'TRANSIENT_LOCAL': 0})
        qos_module.QoSProfile = type('QoSProfile', (), {'__init__': lambda self, depth=1: None})
        qos_module.ReliabilityPolicy = type('ReliabilityPolicy', (), {'RELIABLE': 0})
        rclpy.node = node_module
        rclpy.qos = qos_module
        sys.modules['rclpy'] = rclpy
        sys.modules['rclpy.node'] = node_module
        sys.modules['rclpy.qos'] = qos_module

    def _namespace(**kwargs):
        return types.SimpleNamespace(**kwargs)

    class _Point:
        def __init__(self, **kwargs):
            self.x = kwargs.get('x', 0.0)
            self.y = kwargs.get('y', 0.0)
            self.z = kwargs.get('z', 0.0)

    if 'geometry_msgs' not in sys.modules:
        geometry_msgs = types.ModuleType('geometry_msgs')
        geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
        geometry_msgs_msg.Point = _Point
        geometry_msgs.msg = geometry_msgs_msg
        sys.modules['geometry_msgs'] = geometry_msgs
        sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

    class _ColorRGBA:
        def __init__(self, **kwargs):
            self.r = kwargs.get('r', 0.0)
            self.g = kwargs.get('g', 0.0)
            self.b = kwargs.get('b', 0.0)
            self.a = kwargs.get('a', 0.0)

    if 'std_msgs' not in sys.modules:
        std_msgs = types.ModuleType('std_msgs')
        std_msgs_msg = types.ModuleType('std_msgs.msg')
        std_msgs_msg.ColorRGBA = _ColorRGBA
        std_msgs.msg = std_msgs_msg
        sys.modules['std_msgs'] = std_msgs
        sys.modules['std_msgs.msg'] = std_msgs_msg

    class _Marker:
        CUBE_LIST = 1
        ADD = 0

        def __init__(self):
            self.header = _namespace(frame_id='')
            self.ns = ''
            self.id = 0
            self.type = 0
            self.action = 0
            self.pose = _namespace(orientation=_namespace(w=0.0))
            self.scale = _namespace(x=0.0, y=0.0, z=0.0)
            self.color = _ColorRGBA()
            self.points = []
            self.colors = []

    class _MarkerArray:
        def __init__(self):
            self.markers = []

    if 'visualization_msgs' not in sys.modules:
        visualization_msgs = types.ModuleType('visualization_msgs')
        visualization_msgs_msg = types.ModuleType('visualization_msgs.msg')
        visualization_msgs_msg.Marker = _Marker
        visualization_msgs_msg.MarkerArray = _MarkerArray
        visualization_msgs.msg = visualization_msgs_msg
        sys.modules['visualization_msgs'] = visualization_msgs
        sys.modules['visualization_msgs.msg'] = visualization_msgs_msg


_install_evaluator_import_stubs()
from robot_patrol_node.map_accuracy_evaluator_node import MapAccuracyEvaluatorNode


def test_overlap_projection_uses_world_coordinates() -> None:
    gt_geometry = MapGeometry(width=4, height=4, resolution=1.0, origin_x=0.0, origin_y=0.0, frame_id='map')
    pred_geometry = MapGeometry(width=6, height=6, resolution=1.0, origin_x=-1.0, origin_y=-1.0, frame_id='map')
    ground_truth = np.arange(16, dtype=np.int16).reshape(4, 4)

    projected_ground_truth, evaluation_mask, overlap = MapAccuracyEvaluatorNode._project_ground_truth_to_prediction(
        gt_geometry,
        ground_truth,
        pred_geometry,
    )

    assert overlap == MapAccuracyEvaluatorNode._geometry_bounds(gt_geometry)
    assert int(np.count_nonzero(evaluation_mask)) == 16
    assert projected_ground_truth[1, 1] == ground_truth[0, 0]
    assert projected_ground_truth[4, 4] == ground_truth[3, 3]
    assert projected_ground_truth[0, 0] == -1


def test_metric_percentage_fields_are_written() -> None:
    node = MapAccuracyEvaluatorNode.__new__(MapAccuracyEvaluatorNode)
    node.trial_id = 'trial_0'
    node.fusion_mode = 'log_odds'
    node.map_name = 'shared_map'
    node.ground_truth_map_topic = '/map'
    node.shared_map_template = '/{robot}/shared_live_map'
    node.confidence_map_template = '/{robot}/shared_confidence_map'
    node.attack_tracker = AttackRegionTracker([])
    node.ground_truth_map = type('GroundTruth', (), {'header': type('Header', (), {'frame_id': 'map'})()})()

    geometry = MapGeometry(width=4, height=4, resolution=1.0, origin_x=0.0, origin_y=0.0, frame_id='map')
    state = type('State', (), {
        'latest_geometry_match': True,
        'latest_confidence_mean': None,
        'latest_false_occupied_mean_confidence': None,
    })()
    metrics = MapMetrics(
        true_occupied_count=3,
        false_occupied_count=2,
        missed_occupied_count=1,
        predicted_occupied_count=5,
        predicted_unknown_count=0,
        ground_truth_occupied_count=4,
        ground_truth_free_count=8,
        evaluated_cell_count=12,
        occupied_precision=0.60,
        occupied_recall=0.75,
        occupied_f1=0.6666666667,
        occupied_iou=0.50,
        false_occupied_rate=0.25,
        missed_occupied_rate=0.25,
        unknown_rate=0.0,
    )

    row = node._base_row(123.0, 'robot_1', geometry, metrics, state, scope='whole_map')

    assert 'occupied_iou_percent' in row
    assert 'free_space_correct_percent' in row
    assert row['occupied_iou'] == pytest.approx(0.50)
    assert row['false_occupied_rate'] == pytest.approx(0.25)
    assert row['occupied_iou_percent'] == pytest.approx(50.0)
    assert row['free_space_correct_percent'] == pytest.approx(75.0)

    region_row = node._base_row(123.0, 'robot_1', geometry, metrics, state, scope='attack_region')
    region_row.update({
        'attack_region_name': 'fake_obstacle',
        'attack_region_type': 'circle',
    })
    assert region_row['scope'] == 'attack_region'
    assert region_row['occupied_iou_percent'] == pytest.approx(50.0)
    assert region_row['free_space_correct_percent'] == pytest.approx(75.0)


def test_attack_region_tracker_circle_mask_and_metrics() -> None:
    thresholds = OccupancyThresholds(occupied_threshold=65, free_threshold=25, unknown_value=-1)
    geometry = MapGeometry(width=3, height=3, resolution=1.0, origin_x=0.0, origin_y=0.0, frame_id='map')
    tracker = AttackRegionTracker([
        AttackRegionDefinition(
            name='fake_obstacle',
            type='circle',
            frame_id='map',
            center_x=1.5,
            center_y=1.5,
            radius=1.0,
        )
    ])

    ground_truth = np.array(
        [
            [0, 0, 0],
            [0, 100, 0],
            [0, 0, 0],
        ],
        dtype=np.int16,
    )
    prediction = np.array(
        [
            [0, 100, 0],
            [0, 100, 0],
            [0, 0, 0],
        ],
        dtype=np.int16,
    )

    masks = tracker.masks_for_geometry(geometry)
    assert 'fake_obstacle' in masks
    assert int(np.count_nonzero(masks['fake_obstacle'])) > 0

    region_metrics = tracker.compute_region_metrics(ground_truth, prediction, thresholds, geometry)
    result = region_metrics['fake_obstacle']

    assert result.mask_cell_count == int(np.count_nonzero(masks['fake_obstacle']))
    assert result.metrics.false_occupied_count == 1
    assert result.metrics.false_occupied_rate == pytest.approx(0.25)
    assert result.metrics.true_occupied_count == 1


def test_attack_region_tracker_respects_evaluation_mask() -> None:
    thresholds = OccupancyThresholds(occupied_threshold=65, free_threshold=25, unknown_value=-1)
    geometry = MapGeometry(width=4, height=4, resolution=1.0, origin_x=0.0, origin_y=0.0, frame_id='map')
    tracker = AttackRegionTracker([
        AttackRegionDefinition(
            name='zone',
            type='rect',
            frame_id='map',
            min_x=0.0,
            max_x=3.9,
            min_y=0.0,
            max_y=3.9,
        )
    ])

    ground_truth = np.zeros((4, 4), dtype=np.int16)
    prediction = np.zeros((4, 4), dtype=np.int16)
    prediction[1, 1] = 100
    evaluation_mask = np.zeros((4, 4), dtype=bool)
    evaluation_mask[1, 1] = True

    region_metrics = tracker.compute_region_metrics(ground_truth, prediction, thresholds, geometry, mask=evaluation_mask)
    result = region_metrics['zone']

    assert result.mask_cell_count == 1
    assert result.metrics.false_occupied_count == 1
    assert result.metrics.false_occupied_rate == pytest.approx(1.0)
    assert result.metrics.evaluated_cell_count == 1


def test_attack_region_tracker_rectangle_mask() -> None:
    geometry = MapGeometry(width=4, height=4, resolution=1.0, origin_x=0.0, origin_y=0.0, frame_id='map')
    tracker = AttackRegionTracker([
        AttackRegionDefinition(
            name='zone',
            type='rect',
            frame_id='map',
            min_x=1.0,
            max_x=2.9,
            min_y=1.0,
            max_y=2.9,
        )
    ])
    mask = tracker.masks_for_geometry(geometry)['zone']
    assert int(np.count_nonzero(mask)) == 4


def test_rviz_overlay_marker_counts() -> None:
    pytest.importorskip('visualization_msgs.msg')
    pytest.importorskip('geometry_msgs.msg')

    from robot_patrol_node.rviz_accuracy_overlay import RVizAccuracyOverlay

    geometry = MapGeometry(width=2, height=2, resolution=1.0, origin_x=0.0, origin_y=0.0, frame_id='map')
    overlay = RVizAccuracyOverlay()
    correct = np.array([[True, False], [False, False]])
    false = np.array([[False, True], [False, False]])
    missed = np.array([[False, False], [True, False]])
    unknown = np.array([[False, False], [False, True]])

    marker_array = overlay.build_marker_array(geometry, correct, false, missed, unknown)

    assert len(marker_array.markers) == 4
    assert len(marker_array.markers[0].points) == 1
    assert len(marker_array.markers[1].points) == 1
    assert len(marker_array.markers[2].points) == 1
    assert len(marker_array.markers[3].points) == 1
