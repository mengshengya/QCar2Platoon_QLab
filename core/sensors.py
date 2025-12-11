import os
import csv
import time
import json  # 用于序列化雷达日志
import numpy as np

from qvl.multi_agent import readRobots
from typing import Optional, Tuple, Dict, Any
from core.sensor_config import get_lidar_config, LidarConfig

try:
    from pal.products.qcar import QCarLidar
except Exception:
    QCarLidar = None  # type: ignore

try:
    from pal.products.qcar import QCarGPS
except Exception:
    QCarGPS = None  # type: ignore

try:        
    from pal.products.qcar import IS_PHYSICAL_QCAR
except Exception:
    IS_PHYSICAL_QCAR = False  # best-effort default


class VehicleSensors:
    """
    Manage optional Lidar and GPS for a vehicle.

    Responsibilities:
    - Initialize Lidar/GPS for physical or simulated QCar
    - Provide scan and pose reads
    - Compute front distance from Lidar scan
    """

    def __init__(self, vehicle_id: int, rate_hz: float,
                 use_lidar: bool = False, use_gps: bool = True):
        self.vehicle_id = int(vehicle_id)
        self.rate_hz = float(rate_hz)
        self.use_lidar = bool(use_lidar) 
        self.use_gps = bool(use_gps) 
        self.lidar = None
        self.gps = None
        # no EKF state retained here
        # Cache last valid front distance to handle occasional NaNs
        self._last_front_m: Optional[float] = None

    # Lifecycle
    def open(self):
        # Open Lidar
        if self.use_lidar and QCarLidar is not None:
            try:
                if IS_PHYSICAL_QCAR:
                    self.lidar = QCarLidar(enableFiltering=True)
                    print(f"Vehicle {self.vehicle_id}: Physical LiDAR opened")
                else:
                    robots = readRobots()
                    key = f"QC2_{self.vehicle_id}"
                    info = robots[key]
                    lidar_port = info.get("lidarPort", 18966 + self.vehicle_id)
                    self.lidar = QCarLidar(rangingDistanceMode=2, enableFiltering=False,
                                           numMeasurements = 384,
                                            lidarPort=lidar_port)
                    # self.lidar = QCarLidar(rangingDistanceMode=2, enableFiltering=True,
                    #                        numMeasurements = 1500,
                    #                         lidarPort=info["lidarPort"])
                    print(f"Vehicle {self.vehicle_id}: LiDAR opened on port {lidar_port}")
            except Exception:
                self.lidar = None
                print(f"Vehicle {self.vehicle_id}: Failed to open LiDAR")


        # Open GPS
        if self.use_gps and QCarGPS is not None:
            try:
                init_pose = self._load_initial_pose_from_csv(default=[0.0, 0.0, 0.0])
                if IS_PHYSICAL_QCAR:
                    self.gps = QCarGPS(initialPose=init_pose)
                else:
                    robots = readRobots()
                    key = f"QC2_{self.vehicle_id}"
                    info = robots[key]
                    gps_port = info.get("gpsPort", 18967 + self.vehicle_id)
                    lidar_ideal_port = info.get("lidarIdealPort", 18968 + self.vehicle_id)
                    self.gps = QCarGPS(initialPose=init_pose, gpsPort=gps_port,
                                       lidarIdealPort=lidar_ideal_port)
                    print(f"Vehicle {self.vehicle_id}: GPS opened on port {gps_port}")
            except Exception:
                self.gps = None
                print(f"Vehicle {self.vehicle_id}: Failed to open GPS")

    def close(self):
        self.lidar = None
        self.gps = None

    def read_all(self, qcar, timestamp: Optional[float] = None) -> Dict[str, Any]:
        """
        统一读取本车的所有可用测量：
        - LiDAR 前/后距离（以及可选的点云）
        - GPS 位置 + 姿态
        - 车体传感器（速度、加速度、yaw rate 等）
        返回一个 dict，供 VehicleAgent / 观测器 / 控制器使用。
        """
        out: Dict[str, Any] = {}

        # 1) 车体状态（速度、加速度、yaw rate...）
        veh_state = self.read_vehicle_state(qcar)
        # veh_state 可能为空 dict
        out.update(veh_state)

        # 2) GPS pose
        pos, rpy = self.read_gps_pose()
        out["gps_pos"] = pos    # None or np.ndarray shape (3,)
        out["gps_rpy"] = rpy    # None or np.ndarray shape (3,)

        # 3) LiDAR 前/后距离
        if self.use_lidar:
            lidar_info = self.measure_front_rear_from_lidar(
                data_dir="",        # 保存交给 logger 来做，这里可以传空或 None
                cfg=None,
                timestamp=timestamp
            )
        else:
            lidar_info = {
                "front_m": float("nan"),
                "rear_m": float("nan"),
                "n_front": 0,
                "n_rear": 0,
                "xf": np.array([]),
                "yf": np.array([]),
                "rf": np.array([]),
                "xr": np.array([]),
                "yr": np.array([]),
                "rr": np.array([]),
                "front_fallback": False,
            }
        # out["lidar"] = lidar_info  

        out["front_m"] = lidar_info["front_m"]
        out["rear_m"] = lidar_info["rear_m"]
        # 4) 统一加上时间戳
        if timestamp is not None:
            out["t_s"] = float(timestamp)

        return out


    # ----- Raw LiDAR read (ranges [m], angles [rad]) -----
    def _read_lidar_raw(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        # 原始读数方法
        # _read_lidar_raw() 返回 (ranges[m], angles[rad])
        lid = self.lidar
        if lid is None:
            print("ERROR: Lidar does not configured")
            return None, None

        lid.read()
        
        distances = getattr(lid, "distances", None)
        angles = getattr(lid, "angles", None)
        if distances is None or angles is None:
            print("distances is None or angles is None")
            return None, None
        try:
            distances = np.asarray(distances, dtype=float)
            angles = np.asarray(angles, dtype=float)
        except Exception:
            print(f"LiDAR no data (veh {self.vehicle_id})")
            return None, None
        
        # 额外的空/全 NaN 检查
        if distances.size == 0 or angles.size == 0:
            print(f"LiDAR empty scan (veh {self.vehicle_id})")
            return None, None
        if not np.isfinite(distances).any():
            print(f"LiDAR invalid scan (veh {self.vehicle_id})")
            return None, None
        
        return distances, angles

    def _log_lidar_snapshot(self, lidar_info: Any, timestamp: Optional[float]) -> None:
        log_root = os.path.join(os.getcwd(), "log", "lidar")  # 组合雷达日志保存路径
        os.makedirs(log_root, exist_ok=True)  # 确保log/lidar目录存在并可写
        distances, angles = (None, None)  # 初始化距离与角度的容器
        if isinstance(lidar_info, tuple) and len(lidar_info) == 2:  # 支持_read_lidar_raw返回的元组格式
            distances, angles = lidar_info  # 将元组拆解为距离和角度
        elif isinstance(lidar_info, dict):  # 支持预判你所需的dict格式
            distances = lidar_info.get("distance")  # 从dict中提取距离
            angles = lidar_info.get("angles")  # 从dict中提取角度
        ts = round(float(timestamp) if timestamp is not None else time.time(), 4)  # 时间戳四舍五入到小数4位
        dist_arr = np.round(np.asarray(distances, dtype=float), 4).tolist() if distances is not None else []  # 距离数组保留4位
        ang_arr = np.round(np.asarray(angles, dtype=float), 4).tolist() if angles is not None else []  # 角度数组保留4位
        entry = {  # 构建写入日志的单条数据
            "vehicle_id": self.vehicle_id,  # 车辆ID用于区分日志文件
            "timestamp": ts,  # 时间戳秒（4位小数）
            "distance": dist_arr,  # 距离数据列表（4位小数）
            "angles": ang_arr,  # 角度数据列表（4位小数）
        }
        log_path = os.path.join(log_root, f"vehicle_{self.vehicle_id}.jsonl")  # 为每个车辆生成专属jsonl日志文件
        with open(log_path, "a", encoding="utf-8") as fp:  # 追加模式打开日志文件
            fp.write(json.dumps(entry) + "\n")  # 写入一行JSON日志个数

    # ----- Transform and filter helpers -----
    def _to_body_frame(self, distances: np.ndarray, angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        # Convert from LiDAR frame to body frame: flip and shift to align forward = 0 rad
        angles_body = angles * (-1) + np.pi
        # normalize to [-pi, pi]
        # angles_body = (angles_body + np.pi) % (2 * np.pi) - np.pi
        # polar to Cartesian in body frame (x: forward, y: left)
        x = distances * np.cos(angles_body)
        y = distances * np.sin(angles_body)
        return x, y, distances, angles_body

    def _range_and_lane_mask(self, r: np.ndarray, y: np.ndarray, cfg: LidarConfig) -> np.ndarray:
        # 按量程与车道宽度过滤
        mask_r = (r >= float(cfg.min_range_m)) & (r <= float(cfg.max_range_m)) & np.isfinite(r)
        mask_lane = np.abs(y) <= float(cfg.lane_half_width_m)
        return mask_r & mask_lane

    def _sector_masks(self, angles_body: np.ndarray, cfg: LidarConfig) -> Tuple[np.ndarray, np.ndarray]:
        # 前/后扇区掩码
        eps_front = np.deg2rad(float(cfg.front_half_deg))
        eps_rear = np.deg2rad(float(cfg.rear_half_deg))
        front = np.abs(angles_body - np.pi) <= eps_front
        rear = np.abs(angles_body) <= eps_rear
        return front, rear

    # ----- Public API: measure and save -----
    def measure_front_rear_from_lidar(self, data_dir: str, cfg: Optional[LidarConfig] = None,
                                      timestamp: Optional[float] = None) -> Dict[str, Any]:
        """
        Use LiDAR scan to compute front/rear distances with configured sectors and lane width.
        Saves filtered front/rear point sets to data_dir.
        Returns dict with 'front_m', 'rear_m', 'n_front', 'n_rear'.
        """
        out = {}
        if cfg is None:
            cfg = get_lidar_config()
        dists, angs = self._read_lidar_raw()
        if dists is None or angs is None:
            # No fresh data; try fallback for front only
            front_fallback = False
            front_val = float("nan")
            if self._last_front_m is not None and np.isfinite(self._last_front_m):
                front_val = float(self._last_front_m)
                front_fallback = True
            return {
                "front_m": front_val,
                "rear_m": float("nan"),
                "n_front": 0,
                "n_rear": 0,
                "front_fallback": front_fallback,
            }
        # transform
        x, y, r, a_body = self._to_body_frame(dists, angs)
        # masks
        mask_common = self._range_and_lane_mask(r, y, cfg)
        m_front, m_rear = self._sector_masks(a_body, cfg)
        mf = mask_common & m_front
        mr = mask_common & m_rear
        xf, yf, rf = x[mf], y[mf], r[mf]
        xr, yr, rr = x[mr], y[mr], r[mr]
        # distances: use longitudinal x for clearer front/rear gap
        # Front sector (x >= 0 typically): closest ahead is minimal x
        if xf.size:
            try:
                front_m = float(np.max(xf))
            except Exception:
                front_m = float("nan")
        else:
            front_m = float("nan")

        # Rear sector (x <= 0 typically): closest behind is smallest |x|, i.e., -max(x) since x is negative
        if xr.size:
            try:
                rear_m = float(np.min(xr))
            except Exception:
                rear_m = float("nan")
        else:
            rear_m = float("nan")

        # Apply bad-data strategy for front: fallback to last valid if NaN
        front_fallback = False
        if not np.isfinite(front_m):
            if self._last_front_m is not None and np.isfinite(self._last_front_m):
                front_m = float(self._last_front_m)
                front_fallback = True
        else:
            # Update cache on valid new measurement
            self._last_front_m = float(front_m)

        # Prepare output only; saving handled by FleetLogger
        t = float(timestamp) if timestamp is not None else time.time()
        return {
                    "t_s": f"{t:.4f}",
                    "front_m": front_m,
                    "rear_m": rear_m,
                    "n_front": int(rf.size),
                    "n_rear": int(rr.size),
                    "xf": xf,
                    "yf": yf,
                    "rf": rf,
                    "xr": xr,
                    "yr": yr,
                    "rr": rr,
                    "front_fallback": front_fallback,
                } 


    # GPS reads
    def read_gps_pose(self):
        if self.gps is None:
            return None, None
        try:
            ok = self.gps.readGPS()
        except Exception:
            ok = False
        if not ok:
            return None, None
        try:
            pos = np.asarray(self.gps.position, dtype=float)
            rpy = np.asarray(self.gps.orientation, dtype=float)
        except Exception:
            return None, None
        return pos, rpy

    # Vehicle (QCar) reads
    def read_vehicle_state(self, qcar):
        """Read QCar basic sensors and return a dict with speed and accelerometer.
        Returns: { 'v': float, 'accel': np.ndarray shape (3,), 'gyro_z': float } where available.
        """
        out = {}
        try:
            # refresh device
            qcar.read()
        except Exception:
            return out

        # Speed (tachometer or similar)
        try:
            if hasattr(qcar, 'motorTach'):
                out['v'] = float(qcar.motorTach)
        except Exception:
            pass

        # Accelerometer (try common attribute names)
        ax = ay = az = None
        try:
            if hasattr(qcar, 'accelerometer'):
                acc = qcar.accelerometer
                try:
                    acc = np.asarray(acc, dtype=float).reshape(-1)
                    if acc.size >= 3:
                        ax, ay, az = acc[0], acc[1], acc[2]
                except Exception:
                    pass
            elif hasattr(qcar, 'accel'):
                acc = qcar.accel
                try:
                    acc = np.asarray(acc, dtype=float).reshape(-1)
                    if acc.size >= 3:
                        ax, ay, az = acc[0], acc[1], acc[2]
                except Exception:
                    pass
            else:
                # try component-wise attributes
                for name, var in (('accelX','ax'), ('accelY','ay'), ('accelZ','az'),
                                   ('ax','ax'), ('ay','ay'), ('az','az')):
                    if ax is None and var=='ax' and hasattr(qcar, name):
                        try:
                            ax = float(getattr(qcar, name))
                        except Exception:
                            pass
                    if ay is None and var=='ay' and hasattr(qcar, name):
                        try:
                            ay = float(getattr(qcar, name))
                        except Exception:
                            pass
                    if az is None and var=='az' and hasattr(qcar, name):
                        try:
                            az = float(getattr(qcar, name))
                        except Exception:
                            pass
        except Exception:
            pass

        if ax is not None and ay is not None and az is not None:
            out['accel'] = np.array([ax, ay, az], dtype=float)

        # Gyroscope Z (yaw rate)
        try:
            gyro_z = None
            if hasattr(qcar, 'gyroscope'):
                g = qcar.gyroscope
                try:
                    g = np.asarray(g, dtype=float).reshape(-1)
                    if g.size >= 3:
                        gyro_z = float(g[2])
                except Exception:
                    pass
            elif hasattr(qcar, 'gyro'):
                g = qcar.gyro
                try:
                    g = np.asarray(g, dtype=float).reshape(-1)
                    if g.size >= 3:
                        gyro_z = float(g[2])
                except Exception:
                    pass
            else:
                for name in ('gyroZ', 'gyro_z', 'gz', 'yawRate', 'yaw_rate'):
                    if hasattr(qcar, name):
                        try:
                            gyro_z = float(getattr(qcar, name))
                        except Exception:
                            pass
                        break
            if gyro_z is not None:
                out['gyro_z'] = gyro_z
        except Exception:
            pass
        return out

    # No EKF utilities here (removed by request)

    # Helpers
    def _load_initial_pose_from_csv(self, default=None):
        if default is None:
            default = [0.0, 0.0, 0.0]
        try:
            base_dir = os.path.dirname(__file__)
            csv_path = os.path.normpath(os.path.join(base_dir, "..", "data", "QcarInitSettingOpenRoad.csv"))
            with open(csv_path, "r", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                target = str(self.vehicle_id)
                for row in reader:
                    idx = row.get("QcarIndex")
                    if idx is None or str(idx).strip() != target:
                        continue
                    x = float(row.get("PositionX", 0.0))
                    y = float(row.get("PositionY", 0.0))
                    # typical header
                    yaw_key = "RotationZ"
                    if yaw_key not in row:
                        for k in row.keys():
                            if k.lower() == "rotationz":
                                yaw_key = k
                                break
                    yaw = float(row.get(yaw_key, 0.0))
                    return [x, y, yaw]
        except Exception:
            pass
        return list(default)
