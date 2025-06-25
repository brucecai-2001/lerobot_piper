# Prepare

SO101 leader + laptop
SO101 follower + Raspberry Pi

1. On Pi, run
```shell
hostname -I
```
Get the Raspberry Pi’s IP address

2. On laptop, ping
```shell
ping <pi_ip_address>
```

3. Install the lerobot on your Pi



# Teleoperate
1. SSH into your Raspberry Pi
```shell
python -m lerobot.common.robots.so101_follower_remote.so101_host
```

  
2. Then on your lapto
```shell
python examples/so101_remote/teleoperate.py
```