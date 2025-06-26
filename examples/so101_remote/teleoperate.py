"""
Run this script on the laptop.
"""
from lerobot.common.robots.so101_remote import SO101Client, SO101ClientConfig
from lerobot.common.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig

robot_config = SO101ClientConfig(remote_ip="127.0.0.1") # remote ip is the ip of your client

teleop_arm_config = SO101LeaderConfig(
    port="/dev/tty.usbmodem5A680114791",
    id="left_leader_arm",
)

robot = SO101Client(robot_config)
teleop_arm = SO101Leader(teleop_arm_config)
robot.connect()
teleop_arm.connect()

while True:
    observation = robot.get_observation()

    arm_action = teleop_arm.get_action()
    arm_action = {f"{k}": v for k, v in arm_action.items()}

    robot.send_action(arm_action)
