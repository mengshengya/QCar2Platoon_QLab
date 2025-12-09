from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Literal, Tuple, Optional, Any
import os
import csv
import json
import re


@dataclass
class CommConfig:
    # 模式: 全互联、手动邻接、基于半径
    mode: Literal["all_to_all", "manual", "radius"] = "manual"
    # 手动邻居表（当 mode == "manual" 时使用）
    manual_neighbors: Dict[int, List[int]] = field(default_factory=dict)
    # 半径模式下的邻居半径（米）
    radius_m: float = 20.0


@dataclass
class SimConfig:
    # 车辆数量与类型
    num_cars: int = 4
    car_type: str = "QC2"

    # 仿真时间长度（秒）
    duration_s = 5
    # 控制器参数
    sim_update_rate: int = 50
    # 传感器使用开关
    use_gps: bool = True
    use_lidar: bool = True
    initial_throttle: float = 0.01
    initial_steering: float = 0.0
    # 每车控制策略配置：{vehicle_id: PolicySpec}
    policies: Dict[int, "PolicySpec"] = field(default_factory=dict)
    # 单车每步最大处理时限（毫秒）；用于全队列统一节拍时的软性保护
    max_vehicle_step_ms: float = 10.0
    # 每车观测器配置：{vehicle_id: ObserverSpec}
    observers: Dict[int, "ObserverSpec"] = field(default_factory=dict)

def get_simconfig():
    return SimConfig()

@dataclass
class PolicySpec:
    """控制策略配置项。

    type: 策略类型（constant, pid_speed, lateral_sine）
    params: 传给策略构造的参数字典
    """
    type: str
    params: Dict[str, float] = field(default_factory=dict)


@dataclass
class ObserverSpec:
    """观测器配置项。

    type: 观测器类型（pass_through, ema, headway_gap）
    params: 传给观测器构造的参数字典
    """
    type: str
    params: Dict[str, Any] = field(default_factory=dict)





