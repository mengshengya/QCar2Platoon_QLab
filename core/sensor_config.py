from dataclasses import dataclass, field

# ---------------- Lidar processing configuration -----------------
@dataclass
class LidarConfig:
    # 有效量程（米）
    min_range_m: float = 0.05
    max_range_m: float = 7.5

    # 前后扇区的半角（度），中心轴为前向 0°，后向 180°
    front_half_deg: float = 10.0
    rear_half_deg: float = 10.0

    # 单车道宽度（米）；用于按横向范围筛选：|y| <= lane_half_width_m
    lane_half_width_m: float = 0.5



def get_lidar_config() -> LidarConfig:
    return LidarConfig()
