from lerobot.common.robots.lekiwi_base import LeKiwiClient, LeKiwiBaseClientConfig
from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig

robot_config = LeKiwiBaseClientConfig(remote_ip="192.168.31.122", id="lekiwi_base")

teleop_keyboard_config = KeyboardTeleopConfig(
    id="my_laptop_keyboard",
)

robot = LeKiwiClient(robot_config)
telep_keyboard = KeyboardTeleop(teleop_keyboard_config)

robot.connect()
telep_keyboard.connect()

while True:
    observation = robot.get_observation()

    keyboard_keys = telep_keyboard.get_action()
    base_action = robot._from_keyboard_to_base_action(keyboard_keys)

    robot.send_action(base_action)
