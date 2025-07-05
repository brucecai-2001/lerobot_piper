from dataclasses import dataclass

from ..config import TeleoperatorConfig

@TeleoperatorConfig.register_subclass("gamepad")
@dataclass
class XboxControllerConfig(TeleoperatorConfig):
    mock: bool = False
