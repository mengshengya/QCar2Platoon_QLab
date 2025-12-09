"""Spawn a 5-car fleet and poll each car's LiDAR every 50 ms."""
import logging
import sys, pathlib
import time
from pathlib import Path

sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))

from core.fleet import QCarFleet
from core.sensors import VehicleSensors


def main():
    num_cars = 1
    # Set up logging to file
    log_dir = Path(__file__).resolve().parent.parent / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(
        filename=log_dir / "lidar_fleet.log",
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    fleet = QCarFleet(num_cars=num_cars, car_type="QC2")
    fleet.connect()
    fleet.spawn_fleet()

    sensors = []
    for vid in range(num_cars):
        vs = VehicleSensors(vehicle_id=vid, rate_hz=10, use_lidar=True, use_gps=False)
        vs.open()
        sensors.append(vs)

    try:
        while True:
            for vs in sensors:
                distances, angles = vs._read_lidar_raw()
                print(f"Vehicle {vs.vehicle_id}: LiDAR distance={distances}")
                if distances is not None and angles is not None:
                    # Log a short summary each read
                    logging.info(
                        "veh=%d samples=%d dist_first=%.3f dist_mean=%.3f",
                        vs.vehicle_id,
                        n,
                        float(distances[0]) if n else float("nan"),
                        float(distances.mean()) if n else float("nan"),
                    )
                time.sleep(0.1)
    finally:
        for vs in sensors:
            try:
                vs.close()
            except Exception:
                pass
        fleet.terminate()


if __name__ == "__main__":
    main()
