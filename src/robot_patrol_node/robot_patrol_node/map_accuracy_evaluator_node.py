from __future__ import annotations

from dataclasses import dataclass
import csv
import math
from pathlib import Path
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory
import yaml
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray

from .attack_region_tracker import AttackRegionTracker
from .map_metrics import MapGeometry, MapMetrics, OccupancyThresholds, classify_occupancy, compute_map_metrics
from .rviz_accuracy_overlay import RVizAccuracyOverlay


def _package_share_path(*parts: str) -> str:
    return str(Path(get_package_share_directory('robot_patrol_node')).joinpath(*parts))


@dataclass
class EvaluatedRobotState:
    shared_map: OccupancyGrid | None = None
    confidence_map: OccupancyGrid | None = None
    latest_metrics: MapMetrics | None = None
    latest_region_metrics: dict[str, object] | None = None
    latest_confidence_mean: float | None = None
    latest_false_occupied_mean_confidence: float | None = None
    latest_geometry_match: bool = False
    last_geometry_warning_key: tuple[object, ...] | None = None


@dataclass(frozen=True)
class MapBounds:
    min_x: float
    min_y: float
    max_x: float
    max_y: float


class MapAccuracyEvaluatorNode(Node):
    """Compare shared robot maps against ground truth and log map accuracy."""

    FIELDNAMES = [
        'timestamp',
        'trial_id',
        'robot_name',
        'scope',
        'fusion_mode',
        'map_name',
        'attack_enabled',
        'attack_region_name',
        'attack_region_type',
        'attack_region_frame_id',
        'attack_region_mask_cell_count',
        'geometry_match',
        'ground_truth_topic',
        'shared_map_topic',
        'confidence_map_topic',
        'frame_id',
        'ground_truth_frame_id',
        'width',
        'height',
        'resolution',
        'origin_x',
        'origin_y',
        'true_occupied_count',
        'false_occupied_count',
        'missed_occupied_count',
        'predicted_occupied_count',
        'predicted_unknown_count',
        'ground_truth_occupied_count',
        'ground_truth_free_count',
        'evaluated_cell_count',
        'occupied_precision',
        'occupied_recall',
        'occupied_f1',
        'occupied_iou',
        'occupied_iou_percent',
        'false_occupied_rate',
        'free_space_correct_percent',
        'missed_occupied_rate',
        'unknown_rate',
        'confidence_mean',
        'false_occupied_mean_confidence',
        'attack_region_mean_confidence',
        'attack_region_false_occupied_mean_confidence',
        'status',
    ]

    def __init__(self) -> None:
        super().__init__('map_accuracy_evaluator')

        self.declare_parameter('yaml_config_file', _package_share_path('config', 'map_accuracy_evaluator.yaml'))
        self.declare_parameter('robots', ['robot_1', 'robot_2'])
        self.declare_parameter('robot_names', '')
        self.declare_parameter('trial_id', 'trial_0')
        self.declare_parameter('fusion_mode', 'log_odds')
        self.declare_parameter('map_name', 'shared_map')
        self.declare_parameter('ground_truth_topic', '/map')
        self.declare_parameter('ground_truth_map_topic', '/map')
        self.declare_parameter('shared_map_template', '/{robot}/shared_live_map')
        self.declare_parameter('confidence_map_template', '/{robot}/shared_confidence_map')
        self.declare_parameter('overlay_topic_template', '/{robot}/map_accuracy_overlay')
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('free_threshold', 25)
        self.declare_parameter('unknown_value', -1)
        self.declare_parameter('evaluation_period_sec', 2.0)
        self.declare_parameter('log_period_sec', 2.0)
        self.declare_parameter('output_directory', 'results/map_accuracy')
        self.declare_parameter('output_dir', 'results/map_accuracy')
        self.declare_parameter('publish_overlay', True)
        self.declare_parameter('timeseries_filename', 'raw/map_accuracy_timeseries.csv')
        self.declare_parameter('summary_filename', 'processed/summary_by_trial.csv')
        self.declare_parameter('write_timeseries', True)
        self.declare_parameter('write_final_summary', True)

        self.yaml_config_file = str(self.get_parameter('yaml_config_file').value).strip() or _package_share_path('config', 'map_accuracy_evaluator.yaml')
        self.yaml_config = self._load_yaml_config(self.yaml_config_file)
        robot_names = str(self.get_parameter('robot_names').value).strip()
        robots_param = self.get_parameter('robots').value
        self.robots = self._normalize_string_list(robot_names or robots_param)
        self.trial_id = str(self.get_parameter('trial_id').value).strip() or 'trial_0'
        self.fusion_mode = str(self.get_parameter('fusion_mode').value).strip() or 'log_odds'
        self.map_name = str(self.get_parameter('map_name').value).strip() or 'shared_map'
        self.ground_truth_topic = str(self.get_parameter('ground_truth_topic').value).strip() or str(self.get_parameter('ground_truth_map_topic').value).strip() or '/map'
        self.ground_truth_map_topic = self.ground_truth_topic
        self.shared_map_template = str(self.get_parameter('shared_map_template').value).strip() or '/{robot}/shared_live_map'
        self.confidence_map_template = str(self.get_parameter('confidence_map_template').value).strip() or '/{robot}/shared_confidence_map'
        self.overlay_topic_template = str(self.get_parameter('overlay_topic_template').value).strip() or '/{robot}/map_accuracy_overlay'
        self.thresholds = OccupancyThresholds(
            occupied_threshold=int(self.get_parameter('occupied_threshold').value),
            free_threshold=int(self.get_parameter('free_threshold').value),
            unknown_value=int(self.get_parameter('unknown_value').value),
        )
        self.evaluation_period_sec = max(0.1, float(self.get_parameter('evaluation_period_sec').value or self.get_parameter('log_period_sec').value))
        self.log_period_sec = self.evaluation_period_sec
        self.write_timeseries = bool(self.get_parameter('write_timeseries').value)
        self.write_final_summary_enabled = bool(self.get_parameter('write_final_summary').value)
        self.publish_overlay = bool(self.get_parameter('publish_overlay').value)

        output_directory = str(self.get_parameter('output_directory').value).strip() or str(self.get_parameter('output_dir').value).strip() or 'results/map_accuracy'
        self.output_directory = output_directory
        self.output_dir = output_directory
        self.timeseries_path = Path(output_directory) / str(self.get_parameter('timeseries_filename').value).strip()
        self.summary_path = Path(output_directory) / str(self.get_parameter('summary_filename').value).strip()
        self.timeseries_path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_path.parent.mkdir(parents=True, exist_ok=True)

        self.attack_tracker = AttackRegionTracker.from_raw(self._load_attack_regions(self.yaml_config))
        self.overlay_helper = RVizAccuracyOverlay()
        self.map_qos = self._build_map_qos()
        self.overlay_qos = self._build_overlay_qos()

        self.ground_truth_map: OccupancyGrid | None = None
        self.robot_states: dict[str, EvaluatedRobotState] = {
            robot_id: EvaluatedRobotState() for robot_id in self.robots
        }
        self.overlay_publishers = {
            robot_id: self.create_publisher(
                MarkerArray,
                self._format_topic(self.overlay_topic_template, robot_id),
                self.overlay_qos,
            )
            for robot_id in self.robots
        }
        self.topic_subscriptions = []
        self._last_summary_rows: dict[str, dict[str, object]] = {}
        self._destroyed = False

        self.topic_subscriptions.append(
            self.create_subscription(
                OccupancyGrid,
                self.ground_truth_map_topic,
                self.ground_truth_callback,
                self.map_qos,
            )
        )
        for robot_id in self.robots:
            self.topic_subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    self._format_topic(self.shared_map_template, robot_id),
                    lambda msg, robot_name=robot_id: self.shared_map_callback(robot_name, msg),
                    self.map_qos,
                )
            )
            if self.confidence_map_template:
                self.topic_subscriptions.append(
                    self.create_subscription(
                        OccupancyGrid,
                        self._format_topic(self.confidence_map_template, robot_id),
                        lambda msg, robot_name=robot_id: self.confidence_map_callback(robot_name, msg),
                        self.map_qos,
                    )
                )

        self.timer = self.create_timer(self.evaluation_period_sec, self.evaluate_and_publish)
        self.get_logger().info(
            'Map accuracy evaluator ready: '
            f'robots={self.robots} gt={self.ground_truth_topic} '
            f'shared={self.shared_map_template} confidence={self.confidence_map_template or "disabled"} '
            f'attack_regions={len(self.attack_tracker.regions)} output={self.timeseries_path} '
            f'period={self.evaluation_period_sec:.2f}s'
        )

    @staticmethod
    def _build_map_qos() -> QoSProfile:
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        return qos

    @staticmethod
    def _build_overlay_qos() -> QoSProfile:
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        return qos

    @staticmethod
    def _normalize_string_list(raw_value) -> list[str]:
        if isinstance(raw_value, (list, tuple, set)):
            return [str(value).strip() for value in raw_value if str(value).strip()]
        if raw_value is None:
            return []
        if isinstance(raw_value, str):
            return [part.strip() for part in raw_value.split(',') if part.strip()]
        return [str(raw_value).strip()]

    @staticmethod
    def _format_topic(template: str, robot_id: str) -> str:
        if '{robot}' in template:
            return template.format(robot=robot_id)
        return template

    def _load_yaml_config(self, config_path: str) -> dict:
        if not config_path:
            return {}

        path = Path(config_path)
        if not path.is_file():
            self.get_logger().error(
                f'YAML config file does not exist: {path}. Expected a map_accuracy_evaluator config file.'
            )
            return {}

        try:
            payload = yaml.safe_load(path.read_text(encoding='utf-8'))
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().error(f'Failed to load YAML config file {path}: {exc}')
            return {}

        if not isinstance(payload, dict):
            return {}

        config = payload.get('map_accuracy_evaluator')
        if isinstance(config, dict):
            params = config.get('ros__parameters')
            if isinstance(params, dict):
                return params
            return config

        return payload

    @staticmethod
    def _load_attack_regions(config: dict) -> list[dict]:
        regions = config.get('attack_regions', []) if isinstance(config, dict) else []
        if isinstance(regions, list):
            return regions
        return []

    def ground_truth_callback(self, msg: OccupancyGrid) -> None:
        self.ground_truth_map = msg

    @staticmethod
    def _geometry_bounds(geometry: MapGeometry) -> MapBounds:
        return MapBounds(
            min_x=float(geometry.origin_x),
            min_y=float(geometry.origin_y),
            max_x=float(geometry.origin_x) + float(geometry.width) * float(geometry.resolution),
            max_y=float(geometry.origin_y) + float(geometry.height) * float(geometry.resolution),
        )

    @staticmethod
    def _geometry_signature(geometry: MapGeometry) -> tuple[object, ...]:
        return (
            int(geometry.width),
            int(geometry.height),
            float(geometry.resolution),
            float(geometry.origin_x),
            float(geometry.origin_y),
            str(geometry.frame_id or ''),
        )

    @staticmethod
    def _metric_percentages(metrics: MapMetrics) -> dict[str, object]:
        return {
            'occupied_iou_percent': float(metrics.occupied_iou) * 100.0,
            'free_space_correct_percent': (1.0 - float(metrics.false_occupied_rate)) * 100.0,
        }

    @classmethod
    def _overlap_bounds(cls, first: MapGeometry, second: MapGeometry) -> MapBounds | None:
        first_bounds = cls._geometry_bounds(first)
        second_bounds = cls._geometry_bounds(second)
        overlap = MapBounds(
            min_x=max(first_bounds.min_x, second_bounds.min_x),
            min_y=max(first_bounds.min_y, second_bounds.min_y),
            max_x=min(first_bounds.max_x, second_bounds.max_x),
            max_y=min(first_bounds.max_y, second_bounds.max_y),
        )
        if overlap.min_x >= overlap.max_x or overlap.min_y >= overlap.max_y:
            return None
        return overlap

    @staticmethod
    def _world_to_grid_index(world_x: float, world_y: float, geometry: MapGeometry) -> tuple[int, int] | None:
        col = int(math.floor((float(world_x) - float(geometry.origin_x)) / float(geometry.resolution)))
        row = int(math.floor((float(world_y) - float(geometry.origin_y)) / float(geometry.resolution)))
        if row < 0 or col < 0 or row >= int(geometry.height) or col >= int(geometry.width):
            return None
        return row, col

    @classmethod
    def _project_ground_truth_to_prediction(
        cls,
        ground_truth_geometry: MapGeometry,
        ground_truth_values: np.ndarray,
        prediction_geometry: MapGeometry,
    ) -> tuple[np.ndarray, np.ndarray, MapBounds | None]:
        overlap = cls._overlap_bounds(ground_truth_geometry, prediction_geometry)
        projected_ground_truth = np.full(
            (int(prediction_geometry.height), int(prediction_geometry.width)),
            -1,
            dtype=np.int16,
        )
        evaluation_mask = np.zeros_like(projected_ground_truth, dtype=bool)
        if overlap is None:
            return projected_ground_truth, evaluation_mask, None

        for row in range(int(prediction_geometry.height)):
            world_y = float(prediction_geometry.origin_y) + (row + 0.5) * float(prediction_geometry.resolution)
            if world_y < overlap.min_y or world_y >= overlap.max_y:
                continue
            for col in range(int(prediction_geometry.width)):
                world_x = float(prediction_geometry.origin_x) + (col + 0.5) * float(prediction_geometry.resolution)
                if world_x < overlap.min_x or world_x >= overlap.max_x:
                    continue

                gt_index = cls._world_to_grid_index(world_x, world_y, ground_truth_geometry)
                if gt_index is None:
                    continue

                gt_row, gt_col = gt_index
                projected_ground_truth[row, col] = int(ground_truth_values[gt_row, gt_col])
                evaluation_mask[row, col] = True

        return projected_ground_truth, evaluation_mask, overlap

    def shared_map_callback(self, robot_id: str, msg: OccupancyGrid) -> None:
        self.robot_states[robot_id].shared_map = msg

    def confidence_map_callback(self, robot_id: str, msg: OccupancyGrid) -> None:
        self.robot_states[robot_id].confidence_map = msg

    def evaluate_and_publish(self) -> None:
        if self.ground_truth_map is None:
            return

        gt_geometry = MapGeometry.from_occupancy_grid(self.ground_truth_map)
        gt_values = np.asarray(self.ground_truth_map.data, dtype=np.int16).reshape(
            (int(gt_geometry.height), int(gt_geometry.width))
        )

        for robot_id in self.robots:
            state = self.robot_states[robot_id]
            if state.shared_map is None:
                continue

            shared_geometry = MapGeometry.from_occupancy_grid(state.shared_map)
            if not gt_geometry.matches(shared_geometry):
                warning_key = (
                    self._geometry_signature(gt_geometry),
                    self._geometry_signature(shared_geometry),
                )
                if state.last_geometry_warning_key != warning_key:
                    state.last_geometry_warning_key = warning_key
                    self.get_logger().warning(
                        f'[{robot_id}] ground truth and shared map geometries differ; evaluating the overlapping '
                        f'world region only. ground_truth={gt_geometry} shared_map={shared_geometry}'
                    )
            else:
                state.last_geometry_warning_key = None

            projected_gt_values, evaluation_mask, overlap = self._project_ground_truth_to_prediction(
                gt_geometry,
                gt_values,
                shared_geometry,
            )
            if overlap is None or not np.any(evaluation_mask):
                state.latest_geometry_match = False
                self.get_logger().warning(
                    f'[{robot_id}] no overlapping world region between ground_truth={gt_geometry} '
                    f'and shared_map={shared_geometry}; skipping evaluator tick for this robot.'
                )
                continue

            state.latest_geometry_match = True
            shared_values = np.asarray(state.shared_map.data, dtype=np.int16).reshape(
                (int(shared_geometry.height), int(shared_geometry.width))
            )
            metrics = compute_map_metrics(projected_gt_values, shared_values, self.thresholds, mask=evaluation_mask)
            state.latest_metrics = metrics

            confidence_values = self._confidence_array(state.confidence_map, shared_geometry)
            overall_confidence_mean, false_occupied_confidence_mean = self._confidence_stats(
                confidence_values,
                projected_gt_values,
                shared_values,
                evaluation_mask,
            )
            state.latest_confidence_mean = overall_confidence_mean
            state.latest_false_occupied_mean_confidence = false_occupied_confidence_mean

            region_metrics = {}
            if self.attack_tracker.is_enabled():
                region_metrics = self.attack_tracker.compute_region_metrics(
                    projected_gt_values,
                    shared_values,
                    self.thresholds,
                    shared_geometry,
                    confidence=confidence_values,
                    mask=evaluation_mask,
                )
            state.latest_region_metrics = region_metrics

            self._publish_overlay(robot_id, shared_geometry, projected_gt_values, shared_values, evaluation_mask)
            self._write_timeseries_rows(robot_id, shared_geometry, metrics, region_metrics, state)

        if self.write_final_summary_enabled:
            self._last_summary_rows = {
                robot_id: {
                    'geometry': MapGeometry.from_occupancy_grid(state.shared_map) if state.shared_map is not None else gt_geometry,
                    'metrics': state.latest_metrics,
                    'regions': state.latest_region_metrics or {},
                    'state': state,
                }
                for robot_id, state in self.robot_states.items()
                if state.latest_metrics is not None and state.latest_geometry_match
            }
            self.write_final_summary_rows()

    def _confidence_array(self, confidence_map: OccupancyGrid | None, geometry: MapGeometry) -> np.ndarray | None:
        if confidence_map is None:
            return None
        confidence_geometry = MapGeometry.from_occupancy_grid(confidence_map)
        if not geometry.matches(confidence_geometry):
            self.get_logger().warning(
                'Confidence map geometry does not match the shared map geometry; '
                'ignoring confidence diagnostics for this tick.'
            )
            return None
        return np.asarray(confidence_map.data, dtype=np.int16).reshape((int(geometry.height), int(geometry.width)))

    def _confidence_stats(
        self,
        confidence_values: np.ndarray | None,
        ground_truth_values: np.ndarray,
        shared_values: np.ndarray,
        evaluation_mask: np.ndarray,
    ) -> tuple[float | None, float | None]:
        if confidence_values is None:
            return None, None

        valid_confidence = confidence_values[(confidence_values >= 0) & evaluation_mask]
        overall_mean = float(np.mean(valid_confidence)) if valid_confidence.size else None
        _gt_occupied_mask, gt_free_mask, _gt_unknown_mask, _gt_known_mask = classify_occupancy(ground_truth_values, self.thresholds)
        pred_occupied_mask, _, _, _ = classify_occupancy(shared_values, self.thresholds)
        false_occupied_mask = pred_occupied_mask & gt_free_mask & evaluation_mask
        false_occupied_confidence = confidence_values[false_occupied_mask]
        false_occupied_valid = false_occupied_confidence[false_occupied_confidence >= 0]
        false_mean = float(np.mean(false_occupied_valid)) if false_occupied_valid.size else None
        return overall_mean, false_mean

    def _publish_overlay(
        self,
        robot_id: str,
        geometry: MapGeometry,
        gt_values: np.ndarray,
        shared_values: np.ndarray,
        evaluation_mask: np.ndarray,
    ) -> None:
        if not self.publish_overlay:
            return
        pred_occupied_mask, _, pred_unknown_mask, _ = classify_occupancy(shared_values, self.thresholds)
        gt_occupied_mask, gt_free_mask, _gt_unknown_mask, gt_known_mask = classify_occupancy(gt_values, self.thresholds)
        correct_occupied_mask = pred_occupied_mask & gt_occupied_mask & evaluation_mask
        false_occupied_mask = pred_occupied_mask & gt_free_mask & evaluation_mask
        missed_occupied_mask = (~pred_occupied_mask) & gt_occupied_mask & evaluation_mask
        unknown_mask = pred_unknown_mask & gt_known_mask & evaluation_mask

        overlay = self.overlay_helper.build_marker_array(
            geometry,
            correct_occupied_mask,
            false_occupied_mask,
            missed_occupied_mask,
            unknown_mask,
            frame_id=geometry.frame_id or self.ground_truth_map.header.frame_id,
        )
        self.overlay_publishers[robot_id].publish(overlay)

    def _write_timeseries_rows(
        self,
        robot_id: str,
        geometry: MapGeometry,
        metrics: MapMetrics,
        region_metrics: dict[str, object],
        state: EvaluatedRobotState,
    ) -> None:
        timestamp = time.time()
        if self.write_timeseries:
            base_row = self._base_row(timestamp, robot_id, geometry, metrics, state, scope='whole_map')
            self._append_row(self.timeseries_path, base_row)

            for region_name, region_result in region_metrics.items():
                region_row = self._base_row(
                    timestamp,
                    robot_id,
                    geometry,
                    region_result.metrics,
                    state,
                    scope='attack_region',
                )
                region_row.update(region_result.to_row())
                region_row['attack_region_name'] = region_name
                region_row['status'] = 'timeseries'
                self._append_row(self.timeseries_path, region_row)

    def _base_row(
        self,
        timestamp: float,
        robot_id: str,
        geometry: MapGeometry,
        metrics: MapMetrics,
        state: EvaluatedRobotState,
        *,
        scope: str,
    ) -> dict[str, object]:
        row = {
            'timestamp': timestamp,
            'trial_id': self.trial_id,
            'robot_name': robot_id,
            'scope': scope,
            'fusion_mode': self.fusion_mode,
            'map_name': self.map_name,
            'attack_enabled': bool(self.attack_tracker.is_enabled()),
            'attack_region_name': '',
            'attack_region_type': '',
            'attack_region_frame_id': '',
            'attack_region_mask_cell_count': '',
            'geometry_match': bool(state.latest_geometry_match),
            'ground_truth_topic': self.ground_truth_map_topic,
            'shared_map_topic': self._format_topic(self.shared_map_template, robot_id),
            'confidence_map_topic': self._format_topic(self.confidence_map_template, robot_id) if self.confidence_map_template else '',
            'frame_id': geometry.frame_id,
            'ground_truth_frame_id': self.ground_truth_map.header.frame_id,
            'width': geometry.width,
            'height': geometry.height,
            'resolution': geometry.resolution,
            'origin_x': geometry.origin_x,
            'origin_y': geometry.origin_y,
            'confidence_mean': state.latest_confidence_mean if state.latest_confidence_mean is not None else '',
            'false_occupied_mean_confidence': (
                state.latest_false_occupied_mean_confidence if state.latest_false_occupied_mean_confidence is not None else ''
            ),
            'attack_region_mean_confidence': '',
            'attack_region_false_occupied_mean_confidence': '',
            'status': 'timeseries' if scope != 'final' else 'final',
        }
        row.update(metrics.to_row())
        row.update(self._metric_percentages(metrics))
        return row

    def _append_row(self, path: Path, row: dict[str, object]) -> None:
        with path.open('a', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=self.FIELDNAMES)
            if handle.tell() == 0:
                writer.writeheader()
            writer.writerow({field: row.get(field, '') for field in self.FIELDNAMES})

    def write_final_summary_rows(self) -> None:
        if not self.write_final_summary_enabled:
            return
        if not self._last_summary_rows:
            return

        rows: list[dict[str, object]] = []
        for robot_id, snapshot in self._last_summary_rows.items():
            geometry = snapshot['geometry']
            metrics = snapshot['metrics']
            state = snapshot['state']
            if metrics is None:
                continue
            row = self._base_row(time.time(), robot_id, geometry, metrics, state, scope='final')
            row['status'] = 'final'
            rows.append(row)
            regions = snapshot['regions']
            for region_name, region_result in regions.items():
                region_row = self._base_row(
                    time.time(),
                    robot_id,
                    geometry,
                    region_result.metrics,
                    state,
                    scope='final_attack_region',
                )
                region_row.update(region_result.to_row())
                region_row['attack_region_name'] = region_name
                region_row['status'] = 'final'
                rows.append(region_row)

        if not rows:
            return

        with self.summary_path.open('w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=self.FIELDNAMES)
            writer.writeheader()
            for row in rows:
                writer.writerow({field: row.get(field, '') for field in self.FIELDNAMES})

    def destroy_node(self) -> None:
        if self._destroyed:
            return
        self._destroyed = True
        self.write_final_summary_rows()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MapAccuracyEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

