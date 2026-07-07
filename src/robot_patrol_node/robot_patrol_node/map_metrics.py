from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class MapGeometry:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str = ''

    @classmethod
    def from_occupancy_grid(cls, grid) -> 'MapGeometry':
        return cls(
            width=int(grid.info.width),
            height=int(grid.info.height),
            resolution=float(grid.info.resolution),
            origin_x=float(grid.info.origin.position.x),
            origin_y=float(grid.info.origin.position.y),
            frame_id=str(getattr(grid.header, 'frame_id', '') or ''),
        )

    def matches(self, other: 'MapGeometry', *, resolution_tol: float = 1e-9, origin_tol: float = 1e-6) -> bool:
        return (
            int(self.width) == int(other.width)
            and int(self.height) == int(other.height)
            and abs(float(self.resolution) - float(other.resolution)) <= resolution_tol
            and abs(float(self.origin_x) - float(other.origin_x)) <= origin_tol
            and abs(float(self.origin_y) - float(other.origin_y)) <= origin_tol
            and (
                not self.frame_id
                or not other.frame_id
                or str(self.frame_id) == str(other.frame_id)
            )
        )


@dataclass(frozen=True)
class OccupancyThresholds:
    occupied_threshold: int = 65
    free_threshold: int = 25
    unknown_value: int = -1


@dataclass(frozen=True)
class MapMetrics:
    true_occupied_count: int
    false_occupied_count: int
    missed_occupied_count: int
    predicted_occupied_count: int
    predicted_unknown_count: int
    ground_truth_occupied_count: int
    ground_truth_free_count: int
    evaluated_cell_count: int
    occupied_precision: float
    occupied_recall: float
    occupied_f1: float
    occupied_iou: float
    false_occupied_rate: float
    missed_occupied_rate: float
    unknown_rate: float

    def to_row(self, prefix: str = '') -> dict[str, object]:
        values = asdict(self)
        if not prefix:
            return values
        return {f'{prefix}{key}': value for key, value in values.items()}


def _ensure_numpy_array(values) -> np.ndarray:
    array = np.asarray(values, dtype=np.int16)
    if array.ndim == 1:
        return array
    return np.asarray(array, dtype=np.int16)


def classify_occupancy(values, thresholds: OccupancyThresholds) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    array = _ensure_numpy_array(values)
    occupied = array >= int(thresholds.occupied_threshold)
    unknown = array == int(thresholds.unknown_value)
    free = (~occupied) & (~unknown) & (array >= 0) & (array <= int(thresholds.free_threshold))
    known = occupied | free
    return occupied, free, unknown, known


def compute_map_metrics(
    ground_truth,
    prediction,
    thresholds: OccupancyThresholds,
    mask: np.ndarray | None = None,
) -> MapMetrics:
    gt_array = _ensure_numpy_array(ground_truth)
    pred_array = _ensure_numpy_array(prediction)

    if gt_array.shape != pred_array.shape:
        raise ValueError(f'Grid shapes do not match: {gt_array.shape} vs {pred_array.shape}')

    gt_occupied, gt_free, _gt_unknown, gt_known = classify_occupancy(gt_array, thresholds)
    pred_occupied, pred_free, pred_unknown, _pred_known = classify_occupancy(pred_array, thresholds)

    eval_mask = gt_known.copy()
    if mask is not None:
        eval_mask = eval_mask & np.asarray(mask, dtype=bool)

    true_occupied = pred_occupied & gt_occupied & eval_mask
    false_occupied = pred_occupied & gt_free & eval_mask
    missed_occupied = (~pred_occupied) & gt_occupied & eval_mask
    predicted_occupied = pred_occupied & eval_mask
    predicted_unknown = pred_unknown & eval_mask
    gt_occupied_eval = gt_occupied & eval_mask
    gt_free_eval = gt_free & eval_mask

    true_occupied_count = int(np.count_nonzero(true_occupied))
    false_occupied_count = int(np.count_nonzero(false_occupied))
    missed_occupied_count = int(np.count_nonzero(missed_occupied))
    predicted_occupied_count = int(np.count_nonzero(predicted_occupied))
    predicted_unknown_count = int(np.count_nonzero(predicted_unknown))
    ground_truth_occupied_count = int(np.count_nonzero(gt_occupied_eval))
    ground_truth_free_count = int(np.count_nonzero(gt_free_eval))
    evaluated_cell_count = int(np.count_nonzero(eval_mask))

    occupied_precision = true_occupied_count / max(predicted_occupied_count, 1)
    occupied_recall = true_occupied_count / max(ground_truth_occupied_count, 1)
    if occupied_precision + occupied_recall > 0.0:
        occupied_f1 = 2.0 * occupied_precision * occupied_recall / (occupied_precision + occupied_recall)
    else:
        occupied_f1 = 0.0
    occupied_iou = true_occupied_count / max(true_occupied_count + false_occupied_count + missed_occupied_count, 1)
    false_occupied_rate = false_occupied_count / max(ground_truth_free_count, 1)
    missed_occupied_rate = missed_occupied_count / max(ground_truth_occupied_count, 1)
    unknown_rate = predicted_unknown_count / max(evaluated_cell_count, 1)

    return MapMetrics(
        true_occupied_count=true_occupied_count,
        false_occupied_count=false_occupied_count,
        missed_occupied_count=missed_occupied_count,
        predicted_occupied_count=predicted_occupied_count,
        predicted_unknown_count=predicted_unknown_count,
        ground_truth_occupied_count=ground_truth_occupied_count,
        ground_truth_free_count=ground_truth_free_count,
        evaluated_cell_count=evaluated_cell_count,
        occupied_precision=occupied_precision,
        occupied_recall=occupied_recall,
        occupied_f1=occupied_f1,
        occupied_iou=occupied_iou,
        false_occupied_rate=false_occupied_rate,
        missed_occupied_rate=missed_occupied_rate,
        unknown_rate=unknown_rate,
    )

