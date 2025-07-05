import time
import numpy as np
import pygame
from lerobot.common.errors import DeviceAlreadyConnectedError
from ..teleoperator import Teleoperator
from .config_xboxcontroller import XboxControllerConfig

# Xbox 控制器按钮和轴的映射
XBOX_CONTROLLER = {
    "left_stick_x": 0,          # 左摇杆 X 轴
    "left_stick_y": 1,          # 左摇杆 Y 轴
    "right_stick_x": 2,         # 右摇杆 X 轴
    "right_stick_y": 3,         # 右摇杆 Y 轴
}

class XboxController(Teleoperator):
    """Xbox Controllrt Teleoperator
       V0.1 axes only    
    """

    config_class = XboxControllerConfig
    name = "XboxController"

    def __init__(self, config: XboxControllerConfig):
        super().__init__(config)

        self.config = config
        self.robot_type = self.config.type

        self.logs = {}
        self.controller = None
        self.axes = None
        #self.buttons = None
        #self.hats = None

    @property
    def action_features(self) -> dict:
        return {
            "dtype": "float32",
            "shape": (len(XBOX_CONTROLLER),),
            "names": {"buttons": list(XBOX_CONTROLLER.keys())},
        }

    @property
    def feedback_features(self) -> dict:
        return {}
    
    @property
    def is_connected(self) -> bool:
        if not pygame.joystick.get_init():
            return False
        
        # 检查控制器对象是否存在
        if self.controller is None:
            return False
        
        try:
            # 尝试获取控制器的名称，如果失败则认为未连接
            self.controller.get_name()
            return True
        except pygame.error:
            return False

    @property
    def is_calibrated(self) -> bool:
        pass

    def configure(self):
        pass

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(
                "XboxController is already connected. Do not run `connect()` twice."
            )

        try:
            pygame.init()
            pygame.joystick.init()
            
            # 检查控制器数量
            joystick_count = pygame.joystick.get_count()
            if joystick_count == 0:
                raise RuntimeError("未检测到游戏手柄")
                
            # 尝试找到 Xbox 控制器
            for i in range(joystick_count):
                self.controller = pygame.joystick.Joystick(i)
                self.controller.init()
                if "Xbox" in self.controller.get_name():
                    print(f"已连接 Xbox 控制器: {self.controller.get_name()}")
                    break
            else:
                # 如果没找到 Xbox 控制器，使用第一个检测到的
                self.controller = pygame.joystick.Joystick(0)
                self.controller.init()
                print(f"未找到 Xbox 控制器，使用第一个检测到的游戏手柄: {self.controller.get_name()}")
                
            self.axes = self.controller.get_numaxes()
            #self.buttons = self.controller.get_numbuttons()
            #self.hats = self.controller.get_numhats()
            
            print("Xbox 控制器连接成功")
            
        except Exception as e:
            print(f"连接 Xbox 控制器失败: {e}")
            raise

    def calibrate(self) -> None:
        # 可以添加校准逻辑，如校准摇杆死区
        pass

    def get_action(self) -> np.ndarray:
        """
            [left_stick_x, left_stick_y, right_stick_x, right_stick_y]
            stick_x: left to right: -1~1
            stick_y: up to down: -1~1
        """
        if not self.is_connected:
            raise RuntimeError("Xbox 控制器未连接")
            
        # 处理事件以更新控制器状态
        pygame.event.pump()
        
        # 读取控制器状态
        before_read_t = time.perf_counter()
        
        # 初始化状态字典
        state = {}
        
        # 读取轴状态
        for key, index in XBOX_CONTROLLER.items():
            if isinstance(index, int) and index < self.axes:
                state[key] = self.controller.get_axis(index)
            # elif isinstance(index, tuple) and len(index) == 2 and self.hats > 0:
            #     hat_value = self.controller.get_hat(0)
            #     state[key] = 1.0 if hat_value == index else 0.0
            # elif isinstance(index, int) and index < self.buttons:
            #     state[key] = 1.0 if self.controller.get_button(index) else 0.0
            
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t
        
        return state

    def send_feedback(self, feedback: np.ndarray) -> None:
        # 可以添加力反馈或振动反馈逻辑
        pass

    def print_logs(self) -> None:
        for key, value in self.logs.items():
            print(f"{key}: {value}")

    def disconnect(self) -> None:
        if self.is_connected:
            pygame.joystick.quit()
            pygame.quit()
            self.is_connected = False
            print("Xbox 控制器已断开连接")

if __name__ == '__main__':
    config = XboxControllerConfig(id = "xbox_controller_wireless")
    controller = XboxController(config)
    controller.connect()
    while True:
        axes_state = controller.get_action()
        print(axes_state)