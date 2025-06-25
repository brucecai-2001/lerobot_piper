import logging
import time
from functools import cached_property
from typing import Any

from lerobot.common.errors import DeviceNotConnectedError

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from ..so101_follower import SO101Follower, SO101FollowerConfig
from .config_so101_bimanual_follower import SO101BimanualFollowerConfig

logger = logging.getLogger(__name__)

class SO101BimanualFollower(Robot):
    """
    SO-101 Follower Arm designed by TheRobotStudio and Hugging Face.
    Bimanual Follower mode
    """

    config_class = SO101BimanualFollowerConfig
    name = "so101_bimanual_follower"

    def __init__(self, config: SO101BimanualFollowerConfig):
        self.config = config

        left_follower_config = SO101FollowerConfig(
            port=self.config.port_left,
            id=self.config.left_id,
            cameras=self.config.left_cameras
        )

        right_follower_config = SO101FollowerConfig(
            port=self.config.port_right,
            id=self.config.right_id,
            cameras=self.config.right_cameras
        )

        self.left_follower = SO101Follower(left_follower_config)
        self.right_follower = SO101Follower(right_follower_config)

    @property
    def _motors_ft(self) -> dict[str, type]:
        ft = {}
        left_ft = {f"left_{motor}.pos": float for motor in self.left_follower.bus.motors}
        right_ft = {f"right_{motor}.pos": float for motor in self.right_follower.bus.motors}
        ft.update(left_ft)
        ft.update(right_ft)
        return ft
    
    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        left_cam_ft = {
            cam: (self.config.left_cameras[cam].height, self.config.left_cameras[cam].width, 3) for cam in self.left_follower.cameras
        }

        right_cam_ft = {
            cam: (self.config.right_cameras[cam].height, self.config.right_cameras[cam].width, 3) for cam in self.right_follower.cameras
        }

        cam_ft = {}
        cam_ft.update(left_cam_ft)
        cam_ft.update(right_cam_ft)

        return cam_ft
    
    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        left_connected = self.left_follower.bus.is_connected and all(cam.is_connected for cam in self.left_follower.cameras.values())
        right_connected = self.right_follower.bus.is_connected and all(cam.is_connected for cam in self.right_follower.cameras.values())

        return (left_connected and right_connected)
    
    @property
    def is_calibrated(self) -> bool:
        return (self.left_follower.bus.is_calibrated and self.right_follower.bus.is_calibrated)
    
    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        """
        self.left_follower.connect(calibrate)
        self.right_follower.connect(calibrate)

    def calibrate(self) -> None:
        self.left_follower.calibrate()
        self.right_follower.calibrate()
    
    def configure(self) -> None:
        self.left_follower.configure()
        self.right_follower.configure()
    
    def setup_motors(self) -> None:
        self.left_follower.setup_motors()
        self.right_follower.setup_motors()

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        obs_dict = {}

        # Read arm position
        start = time.perf_counter()

        left_obs_dict = self.left_follower.bus.sync_read("Present_Position")
        left_obs_dict = {f"left_{motor}.pos": val for motor, val in left_obs_dict.items()}
        right_obs_dict = self.right_follower.bus.sync_read("Present_Position")
        right_obs_dict = {f"right_{motor}.pos": val for motor, val in right_obs_dict.items()}

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from left cameras
        for cam_key, cam in self.left_follower.cameras.items():
            start = time.perf_counter()
            left_obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} left read {cam_key}: {dt_ms:.1f}ms")

        for cam_key, cam in self.right_follower.cameras.items():
            start = time.perf_counter()
            right_obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} right read {cam_key}: {dt_ms:.1f}ms")
        
        obs_dict.update(left_obs_dict)
        obs_dict.update(right_obs_dict)

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        # extract left actions
        left_actions = {k: v for k, v in action.items() if k.startswith("left_")}
        left_goal_position = {key.removesuffix(".pos").removeprefix("left_"): val for key, val in left_actions.items() if key.endswith(".pos")}
        if self.config.max_relative_target is not None:
            present_pos = self.left_follower.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in left_goal_position.items()}
            left_goal_position = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Send goal position to the left arm
        self.left_follower.bus.sync_write("Goal_Position", left_goal_position)

        # extract right actions
        right_actions = {k: v for k, v in action.items() if k.startswith("right_")}
        right_goal_position = {key.removesuffix(".pos").removeprefix("right_"): val for key, val in right_actions.items() if key.endswith(".pos")}
        if self.config.max_relative_target is not None:
            present_pos = self.right_follower.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in right_goal_position.items()}
            right_goal_position = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Send goal position to the right arm
        self.right_follower.bus.sync_write("Goal_Position", right_goal_position)

        return action

    def disconnect(self):
        self.left_follower.disconnect()
        self.right_follower.disconnect()
    
    
    