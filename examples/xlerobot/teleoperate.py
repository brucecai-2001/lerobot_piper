from lerobot.common.robots.so101_remote import SO101Client, SO101ClientConfig
from lerobot.common.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig

from lerobot.common.robots.so101_bimanual_remote import SO101BimanualClient, SO101BimanualClientConfig
from lerobot.common.teleoperators.so101_bimanual_leader import SO101BimanualLeader, SO101BimanualLeaderConfig

from lerobot.common.robots.lekiwi_base import LeKiwiClient, LeKiwiBaseClientConfig
from lerobot.common.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.common.teleoperators.xbox_controller import XboxControllerTeleop, XboxControllerConfig


if __name__ == '__main__':
    # config
    lekiwi_base_enable = True # use the lekiwi base or not
    lekiwi_teleop_type = "xboxcontroller" # keyboard or xboxcontroller

    so101_enable = True # use arm or not
    so101_teleop_type = "bimanual" # manipulate in solo mode(False) or bimanual mode(True)
    
    another_so101_as_camera = False # TODO: a feature that control another arm as the top view camera.

    # robots and teleoperators instances
    lekiwi_base_robot = None
    lekiwi_base_teleoperator = None
    so101_robot = None
    so101_teleoperator = None

    # init lekiwi_base
    if lekiwi_base_enable:
        lekiwi_base_config = LeKiwiBaseClientConfig(remote_ip="192.168.31.122", id="lekiwi_base", port_zmq_cmd=5555, port_zmq_observations=5556)
        lekiwi_base_robot = LeKiwiClient(lekiwi_base_config)

        if lekiwi_teleop_type == "keyboard":
            teleop_keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")
            lekiwi_base_teleoperator = KeyboardTeleop(teleop_keyboard_config)
        else:
            teleop_xbox_config = XboxControllerConfig(id = "xbox_controller_wireless")
            lekiwi_base_teleoperator = XboxControllerTeleop(teleop_xbox_config)

        print("Connect to lekiwi base robot......")
        lekiwi_base_robot.connect()
        print("lekiwi base robot connected")

        print("Connect to lekiwi teleoperator......")
        lekiwi_base_teleoperator.connect()
        print("lekiwi base teleoperator connected")

    # init so101
    if so101_enable:
        if so101_teleop_type:
            arm_config = SO101BimanualClientConfig(remote_ip="192.168.31.122", port_zmq_cmd=5557, port_zmq_observations=5558)
            so101_robot = SO101BimanualClient(arm_config)

            teleop_arm_config = SO101BimanualLeaderConfig(
                port_left= "/dev/tty.usbmodem5A680114791",
                port_right="/dev/tty.usbmodem5A680126181",
                left_id="left_leader_arm",
                right_id="right_leader_arm"
            )
            so101_teleoperator = SO101BimanualLeader(teleop_arm_config)

        else:
            arm_config = SO101ClientConfig(remote_ip="192.168.31.122", port_zmq_cmd=5557, port_zmq_observations=5558)
            so101_robot = SO101Client(arm_config)

            teleop_arm_config = SO101LeaderConfig(
                port="/dev/tty.usbmodem5A680114791",
                id="left_leader_arm",
            )
            so101_teleoperator = SO101Leader(teleop_arm_config)

        print("Connect to so101......")
        so101_robot.connect()
        print("so101 connected")
        
        print("Connect to so101 teleoperator......")
        so101_teleoperator.connect()
        print("so101 teleoperator connected")


    # control loop
    while True:
        if lekiwi_base_enable:
            lekiwi_base_teleoperator_action = lekiwi_base_teleoperator.get_action()
            if lekiwi_teleop_type == "keyboard":
                base_action = lekiwi_base_robot._from_keyboard_to_base_action(lekiwi_base_teleoperator_action)
            else:
                base_action = lekiwi_base_robot._from_xboxController_to_base_action(lekiwi_base_teleoperator_action)
            
            lekiwi_base_robot.send_action(base_action)

        if so101_enable:
            so101_teleoperator_action = so101_teleoperator.get_action()
            so101_teleoperator_action = {f"{k}": v for k, v in so101_teleoperator_action.items()}

            so101_robot.send_action(so101_teleoperator_action)