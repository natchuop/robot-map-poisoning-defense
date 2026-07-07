from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .map_metrics import MapGeometry, MapMetrics, OccupancyThresholds, compute_map_metrics


@dataclass(frozen=True)
class AttackRegionDefinition:
    name: str
    type: str
    frame_id: str = 'map'
    center_x: float | None = None
    center_y: float | None = None
    radius: float | None = None
    min_x: float | None = None
    max_x: float | None = None
    min_y: float | None = None
    max_y: float | None = None


@dataclass(frozen=True)
class AttackRegionMetrics:
    region: AttackRegionDefinition
    mask_cell_count: int
    mean_confidence: float | None
    false_occupied_mean_confidence: float | None
    metrics: MapMetrics

    def to_row(self, prefix: str = '') -> dict[str, object]:
        row = {
            'attack_region_name': self.region.name,
            'attack_region_type': self.region.type,
            'attack_region_frame_id': self.region.frame_id,
            'attack_region_mask_cell_count': self.mask_cell_count,
            'attack_region_mean_confidence': self.mean_confidence if self.mean_confidence is not None else '',
            'attack_region_false_occupied_mean_confidence': (
                self.false_occupied_mean_confidence if self.false_occupied_mean_confidence is not None else ''
            ),
        }
        row.update(self.metrics.to_row(prefix=prefix))
        return row


class AttackRegionTracker:
    def __init__(self, regions: Iterable[AttackRegionDefinition] | None = None) -> None:
        self.regions = list(regions or [])

    @staticmethod
    def _normalize_region(raw_region) -> AttackRegionDefinition | None:
        if raw_region is None:
            return None
        if isinstance(raw_region, AttackRegionDefinition):
            return raw_region
        if not isinstance(raw_region, dict):
            return None

        name = str(raw_region.get('name', '')).strip()
        region_type = str(raw_region.get('type', 'circle')).strip().lower() or 'circle'
        frame_id = str(raw_region.get('frame_id', 'map')).strip() or 'map'
        if not name:
            return None

        return AttackRegionDefinition(
            name=name,
            type=region_type,
            frame_id=frame_id,
            center_x=AttackRegionTracker._maybe_float(raw_region.get('center_x')),
            center_y=AttackRegionTracker._maybe_float(raw_region.get('center_y')),
            radius=AttackRegionTracker._maybe_float(raw_region.get('radius')),
            min_x=AttackRegionTracker._maybe_float(raw_region.get('min_x')),
            max_x=AttackRegionTracker._maybe_float(raw_region.get('max_x')),
            min_y=AttackRegionTracker._maybe_float(raw_region.get('min_y')),
            max_y=AttackRegionTracker._maybe_float(raw_region.get('max_y')),
        )

    @staticmethod
    def _maybe_float(value) -> float | None:
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @classmethod
    def from_raw(cls, raw_regions) -> 'AttackRegionTracker':
        if raw_regions is None:
            return cls([])
        if isinstance(raw_regions, str):
            return cls([])
        if isinstance(raw_regions, dict):
            raw_regions = [raw_regions]
        regions = []
        for raw_region in raw_regions:
            normalized = cls._normalize_region(raw_region)
            if normalized is not None:
                regions.append(normalized)
        return cls(regions)

    def is_enabled(self) -> bool:
        return bool(self.regions)

    def masks_for_geometry(self, geometry: MapGeometry) -> dict[str, np.ndarray]:
        return {region.name: self.mask_for_region(region, geometry) for region in self.regions}

    def mask_for_region(self, region: AttackRegionDefinition, geometry: MapGeometry) -> np.ndarray:
        height = int(geometry.height)
        width = int(geometry.width)
        mask = np.zeros((height, width), dtype=bool)

        if region.type in {'circle', 'circular'}:
            if region.center_x is None or region.center_y is None or region.radius is None:
                return mask
            radius_sq = float(region.radius) ** 2
            for row in range(height):
                world_y = geometry.origin_y + (row + 0.5) * geometry.resolution
                for col in range(width):
                    world_x = geometry.origin_x + (col + 0.5) * geometry.resolution
                    if (world_x - float(region.center_x)) ** 2 + (world_y - float(region.center_y)) ** 2 <= radius_sq:
                        mask[row, col] = True
            return mask

        if region.type in {'rect', 'rectangle', 'box'}:
            if None in {region.min_x, region.max_x, region.min_y, region.max_y}:
                return mask
            min_x = float(region.min_x)
            max_x = float(region.max_x)
            min_y = float(region.min_y)
            max_y = float(region.max_y)
            for row in range(height):
                world_y = geometry.origin_y + (row + 0.5) * geometry.resolution
                if world_y < min_y or world_y > max_y:
                    continue
                for col in range(width):
                    world_x = geometry.origin_x + (col + 0.5) * geometry.resolution
                    if min_x <= world_x <= max_x:
                        mask[row, col] = True
            return mask

        return mask

    def compute_region_metrics(
        self,
        ground_truth,
        prediction,
        thresholds: OccupancyThresholds,
        geometry: MapGeometry,
        confidence: np.ndarray | None = None,
        mask: np.ndarray | None = None,
    ) -> dict[str, AttackRegionMetrics]:
        results: dict[str, AttackRegionMetrics] = {}
        for region in self.regions:
            region_mask = self.mask_for_region(region, geometry)
            evaluation_mask = region_mask if mask is None else (region_mask & np.asarray(mask, dtype=bool))
            metrics = compute_map_metrics(ground_truth, prediction, thresholds, mask=evaluation_mask)
            confidence_mean = None
            false_occupied_confidence = None
            if confidence is not None and confidence.shape == evaluation_mask.shape:
                region_confidence = confidence[evaluation_mask]
                if region_confidence.size:
                    valid_region_confidence = region_confidence[region_confidence >= 0]
                    if valid_region_confidence.size:
                        confidence_mean = float(np.mean(valid_region_confidence))

                gt_occupied, gt_free, gt_unknown, _ = self._classify_for_confidence(ground_truth, thresholds)
                pred_occupied, _, _, _ = self._classify_for_confidence(prediction, thresholds)
                false_occupied_mask = evaluation_mask & gt_free & pred_occupied & (~gt_unknown)
                false_confidence = confidence[false_occupied_mask]
                if false_confidence.size:
                    valid_false_confidence = false_confidence[false_confidence >= 0]
                    if valid_false_confidence.size:
                        false_occupied_confidence = float(np.mean(valid_false_confidence))

            results[region.name] = AttackRegionMetrics(
                region=region,
                mask_cell_count=int(np.count_nonzero(evaluation_mask)),
                mean_confidence=confidence_mean,
                false_occupied_mean_confidence=false_occupied_confidence,
                metrics=metrics,
            )
        return results

    @staticmethod
    def _classify_for_confidence(values, thresholds: OccupancyThresholds):
        array = np.asarray(values, dtype=np.int16)
        occupied = array >= int(thresholds.occupied_threshold)
        unknown = array == int(thresholds.unknown_value)
        free = (~occupied) & (~unknown) & (array >= 0) & (array <= int(thresholds.free_threshold))
        known = occupied | free
        return occupied, free, unknown, known

