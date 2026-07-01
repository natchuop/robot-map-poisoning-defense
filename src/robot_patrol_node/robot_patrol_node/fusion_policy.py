from .fusion_modes import FusionMode
from .map_evidence import MapEvidence


class FusionPolicy:
    def __init__(self, fusion_mode: str):
        self.fusion_mode = fusion_mode

    def weight_for_local_map(self, reporting_robot_id: str, view_robot_id: str) -> float:
        if self.fusion_mode == FusionMode.LOG_ODDS.value:
            return 1.0

        if self.fusion_mode == FusionMode.MATE_LOG_ODDS.value:
            return 1.0

        if self.fusion_mode == FusionMode.MATE_CLAIM_VERIFICATION.value:
            return 1.0

        return 1.0

    def weight_for_claim(self, evidence: MapEvidence, view_robot_id: str) -> float:
        if self.fusion_mode == FusionMode.LOG_ODDS.value:
            return 1.0

        if self.fusion_mode == FusionMode.MATE_LOG_ODDS.value:
            return 1.0

        if self.fusion_mode == FusionMode.MATE_CLAIM_VERIFICATION.value:
            return 1.0

        return 1.0
