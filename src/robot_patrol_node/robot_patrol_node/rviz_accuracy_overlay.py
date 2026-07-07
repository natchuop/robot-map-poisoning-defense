from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .map_metrics import MapGeometry


@dataclass(frozen=True)
class OverlayPalette:
    correct_occupied: ColorRGBA = field(default_factory=lambda: ColorRGBA(r=0.20, g=0.78, b=0.30, a=0.92))
    false_occupied: ColorRGBA = field(default_factory=lambda: ColorRGBA(r=0.95, g=0.20, b=0.20, a=0.92))
    missed_occupied: ColorRGBA = field(default_factory=lambda: ColorRGBA(r=0.18, g=0.45, b=1.00, a=0.92))
    unknown: ColorRGBA = field(default_factory=lambda: ColorRGBA(r=0.60, g=0.60, b=0.60, a=0.82))


class RVizAccuracyOverlay:
    def __init__(self, palette: OverlayPalette | None = None, z_height: float = 0.03) -> None:
        self.palette = palette or OverlayPalette()
        self.z_height = float(z_height)

    def build_marker_array(
        self,
        geometry: MapGeometry,
        correct_occupied_mask: np.ndarray,
        false_occupied_mask: np.ndarray,
        missed_occupied_mask: np.ndarray,
        unknown_mask: np.ndarray,
        *,
        frame_id: str | None = None,
    ) -> MarkerArray:
        markers = MarkerArray()
        frame = frame_id if frame_id is not None else geometry.frame_id or 'map'

        markers.markers.append(
            self._build_marker(0, 'correct_occupied', frame, geometry, correct_occupied_mask, self.palette.correct_occupied)
        )
        markers.markers.append(
            self._build_marker(1, 'false_occupied', frame, geometry, false_occupied_mask, self.palette.false_occupied)
        )
        markers.markers.append(
            self._build_marker(2, 'missed_occupied', frame, geometry, missed_occupied_mask, self.palette.missed_occupied)
        )
        markers.markers.append(self._build_marker(3, 'unknown', frame, geometry, unknown_mask, self.palette.unknown))
        return markers

    def _build_marker(
        self,
        marker_id: int,
        namespace: str,
        frame_id: str,
        geometry: MapGeometry,
        mask: np.ndarray,
        color: ColorRGBA,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(geometry.resolution)
        marker.scale.y = float(geometry.resolution)
        marker.scale.z = self.z_height
        marker.color = color

        height, width = mask.shape
        for row in range(height):
            for col in range(width):
                if not bool(mask[row, col]):
                    continue
                point = Point()
                point.x = geometry.origin_x + ((col + 0.5) * geometry.resolution)
                point.y = geometry.origin_y + ((row + 0.5) * geometry.resolution)
                point.z = self.z_height * 0.5
                marker.points.append(point)
                marker.colors.append(color)

        return marker
