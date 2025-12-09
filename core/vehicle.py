from __future__ import annotations

from typing import Optional, Dict, Any, List
import time
import numpy as np
import threading

from core.sensors import VehicleSensors
from core.comm import comm
from pal.products.qcar import QCar, IS_PHYSICAL_QCAR
from qvl.multi_agent import readRobots


class VehicleAgent:
    """Manage sensors and communication for a single vehicle. 管理每辆车的传感器与通信

    Responsibilities:
    - Initialize/close VehicleSensors (LiDAR/GPS)
    - Read current state (speed, accel, gyro, GPS, optional LiDAR front distance)
    - Publish minimal state to the neighbor network
    - Read neighbors' shared states
    """

    def __init__(self,
                vehicle_id: int,
                rate_hz: float,
                controller,
                observer,
                comm_endpoint,
                sensors,
                qcar,
                use_lidar: bool = True,
                use_gps: bool = True
        ):

        self.vehicle_id = vehicle_id
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz

        self.use_lidar = bool(use_lidar)
        self.use_gps = bool(use_gps)

        self.state_true = None            # 来自第三方模型的真值
        self.state_hat = None             # 观测器估计
        self.u = None                     # 当前控制输入

        # 组件
        self.qcar = qcar                # QCar 设备对象
        self.controller = controller      # 控制器对象
        self.observer = observer          # 观测器对象
        self.comm = comm_endpoint         # 通信端点（发/收）
        self.sensors = sensors            # 传感器集合

        # 传感器组件：直接用你写好的 VehicleSensors
        self.sensors = VehicleSensors(
            vehicle_id=self.vehicle_id,
            rate_hz=self.rate_hz,
            use_lidar=use_lidar,
            use_gps=use_gps
        )

        # 存储最近一次测量和状态
        self.state_true: Optional[np.ndarray] = None    # 真值 (根据需要定义)
        self.state_hat: Optional[np.ndarray] = None     # 观测器估计
        self.u: Optional[np.ndarray] = None             # 控制输入

    # 开启 VehicleAgent 与rt Modle 的连接
    def open(self) -> None:
        # Open QCar device
        if IS_PHYSICAL_QCAR:
            self.qcar = QCar(readMode=1, frequency=self.rate_hz)
        else:
            robots = readRobots()
            key = f"QC2_{self.vehicle_id}"
            if key not in robots:
                raise KeyError(f"Robot key '{key}' not found in QLabs robots. Available: {list(robots.keys())}")
            info = robots[key]
            self.qcar = QCar(readMode=1, frequency=self.rate_hz, hilPort=info["hilPort"])
        # enter device context
        self.qcar.__enter__()

        # Open sensors
        self.sensors.open()

    # Close the VehicleAgent 
    def close(self) -> None:
       self.sensors.close()


    # === 主循环调用的接口 ===
    def update_and_get_state(self, t: float) -> Tuple[Optional[np.ndarray], Dict[str, Any]]:
        """
        由主循环调用：
        - 统一读取传感器（车辆状态 + LiDAR + GPS
        - 更新内部缓存
        - （可选）从 GPS 构造一个“真值状态” state_true
        返回: (state_true, measurements_dict)
        """
        meas = self.sensors.read_all(self.qcar, timestamp=t)
        self.last_measurements = meas

        # 根据需要，从 meas 构造你的“真值状态向量”，比如 [x, y, yaw, v]
        gps_pos = meas.get("gps_pos", None)
        gps_rpy = meas.get("gps_rpy", None)
        velocity = meas.get("v", None)
        accel = meas.get("accel", None)
        front_m = meas.get("front_m", None)

        # if gps_pos is not None and velocity is not None and accel is not None:
            # 举个例子：pos=[x,y,z], rpy=[roll,pitch,yaw]
        x_accel,_,_ = accel
        self.state_true = np.array([velocity, x_accel], dtype=float)
        # else:
            # self.state_true = None
            # print(f"ERROR!!!!Vehicle {self.vehicle_id}: Incomplete measurements for state_true construction.")

        return self.state_true, meas
    