import logging
import time

from ..teleoperator import Teleoperator
from ..so101_leader import SO101Leader, SO101LeaderConfig
from .config_so101_bimanual_leader import SO101BimanualLeaderConfig

logger = logging.getLogger(__name__)


class SO101BimanualLeader(Teleoperator):
    """
    SO-101 Leader Arm designed by TheRobotStudio and Hugging Face.
    Bimanual Teleoperator Mode
    """
    config_class = SO101BimanualLeaderConfig
    name = "so101_bimanual_leader"

    def __init__(self, config: SO101BimanualLeaderConfig):
        self.config = config

        left_leader_config = SO101LeaderConfig(
            port= self.config.port_left,
            id=self.config.left_id,
        )

        right_leader_config = SO101LeaderConfig(
            port= self.config.port_right,
            id=self.config.right_id,
        )

        self.left_leader = SO101Leader(left_leader_config)
        self.right_leader = SO101Leader(right_leader_config)
    
    def __str__(self) -> str:
        return f"{self.config.left_id} {self.__class__.__name__}" + f"{self.config.right_id} {self.__class__.__name__}"

    @property
    def action_features(self) -> dict[str, type]:
        feature = {}
        left_feature = {f"left_{motor}.pos": float for motor in self.left_leader.bus.motors}
        right_feature = {f"right_{motor}.pos": float for motor in self.right_leader.bus.motors}
        feature.update(left_feature)
        feature.update(right_feature)
        return feature
    
    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return (self.left_leader.bus.is_connected and self.right_leader.bus.is_connected)
    
    @property
    def is_calibrated(self) -> bool:
        return (self.left_leader.bus.is_calibrated and self.right_leader.bus.is_calibrated)

    def connect(self, calibrate: bool = True) -> None:
        """
            Connect left arm first, then right arm
        """
        self.left_leader.connect(calibrate)
        self.right_leader.connect(calibrate)

    def calibrate(self) -> None:
        """
            Calibrate left arm first, then right arm
        """
        self.left_leader.calibrate()
        self.right_leader.calibrate()

    def configure(self) -> None:
        self.left_leader.configure()
        self.right_leader.configure()
    
    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()
        left_action = self.left_leader.bus.sync_read("Present_Position")
        right_action = self.right_leader.bus.sync_read("Present_Position")

        action = {}
        left_action = {f"left_{motor}.pos": val for motor, val in left_action.items()}
        right_action = {f"right_{motor}.pos": val for motor, val in right_action.items()}
        action.update(left_action)
        action.update(right_action)

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError
    
    def disconnect(self) -> None:
        self.left_leader.disconnect()
        self.right_leader.disconnect()