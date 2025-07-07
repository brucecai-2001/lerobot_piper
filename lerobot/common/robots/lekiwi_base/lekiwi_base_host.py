import base64
import json
import logging
import time

import cv2
import zmq

from lerobot.common.constants import OBS_IMAGES

from .config_lekiwi_base import LeKiwiBaseConfig, LeKiwiBaseHostConfig
from .lekiwi_base import LekiwiBase


class LeKiwiHost:
    def __init__(self, config: LeKiwiBaseHostConfig):
        self.zmq_context = zmq.Context()
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PULL)
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_cmd_socket.bind(f"tcp://*:{config.port_zmq_cmd}")

        self.zmq_observation_socket = self.zmq_context.socket(zmq.PUSH)
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_observation_socket.bind(f"tcp://*:{config.port_zmq_observations}")

        self.connection_time_s = config.connection_time_s
        self.watchdog_timeout_ms = config.watchdog_timeout_ms
        self.max_loop_freq_hz = config.max_loop_freq_hz

    def disconnect(self):
        self.zmq_observation_socket.close()
        self.zmq_cmd_socket.close()
        self.zmq_context.term()


def main():
    logging.info("Configuring LeKiwi Base")
    robot_config = LeKiwiBaseConfig(port="/dev/ttyACM2")
    robot = LekiwiBase(robot_config)

    logging.info("Connecting LeKiwi Base")
    robot.connect()

    logging.info("Starting HostAgent")
    host_config = LeKiwiBaseHostConfig(port_zmq_cmd=5555, port_zmq_observations=5556)
    host = LeKiwiHost(host_config)

    last_cmd_time = time.time()
    watchdog_active = False
    logging.info("Waiting for commands...")
    try:
        # Business logic
        start = time.perf_counter()
        duration = 0
        while duration < host.connection_time_s:
            loop_start_time = time.time()
            try:
                msg = host.zmq_cmd_socket.recv_string(zmq.NOBLOCK)
                data = dict(json.loads(msg))
                _ = robot.send_action(data)
                last_cmd_time = time.time()
                watchdog_active = False
            except zmq.Again:
                if not watchdog_active:
                    logging.warning("No command available")
            except Exception as e:
                logging.error("Message fetching failed: %s", e)

            now = time.time()
            if (now - last_cmd_time > host.watchdog_timeout_ms / 1000) and not watchdog_active:
                logging.warning(
                    f"Command not received for more than {host.watchdog_timeout_ms} milliseconds. Stopping the base."
                )
                watchdog_active = True
                robot.stop_base()

            last_observation = robot.get_observation()

            # Send the observation to the remote agent
            try:
                host.zmq_observation_socket.send_string(json.dumps(last_observation), flags=zmq.NOBLOCK)
            except zmq.Again:
                logging.info("Dropping observation, no client connected")

            # Ensure a short sleep to avoid overloading the CPU.
            elapsed = time.time() - loop_start_time

            time.sleep(max(1 / host.max_loop_freq_hz - elapsed, 0))
            duration = time.perf_counter() - start
        print("Cycle time reached.")

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting...")
    finally:
        print("Shutting down Lekiwi Host.")
        robot.disconnect()
        host.disconnect()

    logging.info("Finished LeKiwi Base cleanly")


if __name__ == "__main__":
    main()
