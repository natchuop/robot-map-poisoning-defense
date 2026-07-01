from dataclasses import dataclass


@dataclass(frozen=True)
class MapEvidence:
    claim_id: str
    reporting_robot_id: str
    target_robot_id: str
    cell_x: int
    cell_y: int
    world_x: float
    world_y: float
    occupied: bool
    source: str
    attack_type: str
    is_attack_report: bool
    stamp_sec: float
