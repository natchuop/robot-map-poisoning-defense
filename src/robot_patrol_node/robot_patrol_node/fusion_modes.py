from enum import Enum


class FusionMode(str, Enum):
    LOG_ODDS = 'log_odds'
    MATE_LOG_ODDS = 'mate_log_odds'
    MATE_CLAIM_VERIFICATION = 'mate_claim_verification'
