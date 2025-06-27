# Bimanual Teleopration without cameras

```shell
python -m lerobot.teleoperate \
    --robot.type=so101_bimanual_follower \
    --robot.port_left=/dev/tty.usbmodem5A680113741 \
    --robot.port_right=/dev/tty.usbmodem5A680125671 \
    --robot.left_id=left_follower_arm \
    --robot.right_id=right_follower_arm \
    --teleop.type=so101_bimanual_leader \
    --teleop.port_left=/dev/tty.usbmodem5A680114791 \
    --teleop.port_right=/dev/tty.usbmodem5A680126181 \
    --teleop.left_id=left_leader_arm \
    --teleop.right_id=right_leader_arm
```