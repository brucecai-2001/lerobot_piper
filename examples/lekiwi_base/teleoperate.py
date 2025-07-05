from lerobot.common.robots.lekiwi_base import LeKiwiClient, LeKiwiBaseClientConfig
from lerobot.common.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.common.teleoperators.xbox_controller import XboxControllerTeleop, XboxControllerConfig

# keyboard or xboxcontroller
teleop_type = "xboxcontroller"

robot_config = LeKiwiBaseClientConfig(remote_ip="192.168.31.122", id="lekiwi_base")

if teleop_type == "keyboard":
    teleop_keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard",)
    teleoperator = KeyboardTeleop(teleop_keyboard_config)
else:
    teleop_keyboard_config = XboxControllerConfig(id = "xbox_controller_wireless")
    teleoperator = XboxControllerTeleop(teleop_keyboard_config)

robot = LeKiwiClient(robot_config)
robot.connect()
teleoperator.connect()

while True:
    observation = robot.get_observation()

    teleop_state = teleoperator.get_action()
    if teleop_type == "keyboard":
        base_action = robot._from_keyboard_to_base_action(teleop_state)
    else:
        base_action = robot._from_xboxController_to_base_action(teleop_state)
    print(base_action)
    robot.send_action(base_action)
