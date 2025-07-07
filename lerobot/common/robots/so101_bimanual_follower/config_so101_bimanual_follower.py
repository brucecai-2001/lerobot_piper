from dataclasses import dataclass, field

from lerobot.common.cameras import CameraConfig

from ..config import RobotConfig

@RobotConfig.register_subclass("so101_bimanual_follower")
@dataclass
class SO101BimanualFollowerConfig(RobotConfig):
    id: str = "xlerobot"
    
    # Port to connect to the arm
    port_left: str
    port_right: str

    left_id: str
    right_id: str

    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    # cameras
    left_cameras: dict[str, CameraConfig] = field(default_factory=dict)
    right_cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = False