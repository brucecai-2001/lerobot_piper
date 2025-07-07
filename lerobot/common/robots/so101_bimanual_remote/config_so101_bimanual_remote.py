from dataclasses import dataclass, field

from lerobot.common.cameras import CameraConfig

from ..config import RobotConfig

@dataclass
class SO101BimanualHostConfig:
    # Network Configuration
    port_zmq_cmd: int = 5557
    port_zmq_observations: int = 5558

    # Duration of the application
    connection_time_s: int = 30

    # Watchdog: stop the robot if no command is received for over 0.5 seconds.
    watchdog_timeout_ms: int = 500

    # If robot jitters decrease the frequency and monitor cpu load with `top` in cmd
    max_loop_freq_hz: int = 30


@RobotConfig.register_subclass("so101_client")
@dataclass
class SO101BimanualClientConfig(RobotConfig):
    # Network Configuration
    remote_ip: str
    port_zmq_cmd: int = 5557
    port_zmq_observations: int = 5558

    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5