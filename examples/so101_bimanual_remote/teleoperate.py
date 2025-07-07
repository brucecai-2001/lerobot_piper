"""
Run this script on the laptop.
"""
from lerobot.common.robots.so101_bimanual_remote import SO101BimanualClient, SO101BimanualClientConfig
from lerobot.common.teleoperators.so101_bimanual_leader import SO101BimanualLeader, SO101BimanualLeaderConfig

robot_config = SO101BimanualClientConfig(remote_ip="127.0.0.1") # remote ip is the ip of your client
teleop_arm_config = SO101BimanualLeaderConfig(
    port_left= "/dev/tty.usbmodem5A680114791",
    port_right="/dev/tty.usbmodem5A680126181",
    left_id="left_leader_arm",
    right_id="right_leader_arm"
)

robot = SO101BimanualClient(robot_config)
teleop_arm = SO101BimanualLeader(teleop_arm_config)
robot.connect()
teleop_arm.connect()

while True:
    observation = robot.get_observation()

    arm_action = teleop_arm.get_action()
    arm_action = {f"{k}": v for k, v in arm_action.items()}

    robot.send_action(arm_action)
