Prepare
Host: SO101 follower
Client: SO101 leader

1. Get the client’s IP address

2. On host, ping the client, make sure two computers are connected
```shell
ping <client_ip_address>
```

3. Install the lerobot on your host and client

Teleoperate
1. SSH into your host
```shell
python -m lerobot.common.robots.so101_remote.so101_host
```
  
2. Then on your client
```shell
python examples/so101_remote/teleoperate.py
```