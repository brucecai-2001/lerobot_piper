from dataclasses import dataclass
from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("so101_bimanual_leader")
@dataclass
class SO101BimanualLeaderConfig(TeleoperatorConfig):
    # Port to connect to the arms
    port_left: str
    port_right: str
    
    # calibration id
    left_id: str
    right_id: str
