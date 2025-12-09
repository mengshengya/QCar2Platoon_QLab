import sys
import time
import numpy as np
from qvl.multi_agent import MultiAgent
from qvl.qlabs import QuanserInteractiveLabs
from qvl.real_time import QLabsRealTime
from qvl.free_camera import QLabsFreeCamera


import pandas as pd
import os

class QCarFleet:
    """
    A class to manage a fleet of QCars in Quanser Interactive Labs (QLabs).
    """

    def __init__(self, num_cars=1, qcars_config=None, car_type="QC2"):
        """
        Initialize a QCar fleet.

        Args:
            num_cars (int): Number of cars to spawn (ignored if qcars_config is provided)
            qcars_config (list[dict] or None): Custom configuration for QCars
            car_type (str): Default robot type if using auto-generation
        """
        self.qlabs = QuanserInteractiveLabs()
        self.camera = None
        self.multi_agent = None

        if qcars_config is not None:
            # use user-defined configuration
            self.qcars_config = qcars_config
        else:
            # auto-generate default configuration
            self.qcars_config = self._generate_default_fleet(num_cars, car_type)

    # -------------------------------
    #  Internal helper functions
    # -------------------------------
    def _generate_default_fleet(self, num_cars, car_type):
        """Generate a default linear formation of QCars."""
        qcars = []

        base_dir = os.path.dirname(__file__)
        csv_path = os.path.join(base_dir, "..", "data", "QcarInitSettingOpenRoad.csv")
        InitPositionTable = pd.read_csv(csv_path)

        InitPositionTable = InitPositionTable.to_numpy()

        for i in range(num_cars):
            qcars.append({
                # Use 'ActorNumber' (capital A) to match qvl.MultiAgent API
                "ActorNumber": i,
                "RobotType": car_type,
                "Location": InitPositionTable[i, 1:4],
                "Rotation": InitPositionTable[i, 4:7],  
                "Radians": True,
                "Scale": 1,
            })
        return qcars

    # -------------------------------
    #  QLabs management
    # -------------------------------
    def connect(self, host="localhost"):
        """Connect to QLabs."""
        print("Connecting to QLabs...")
        if not self.qlabs.open(host):
            print("❌ Unable to connect to QLabs.")
            sys.exit()
        print("✅ Connected to QLabs.")

        # Terminate any previous real-time models and clear environment
        QLabsRealTime().terminate_all_real_time_models()
        time.sleep(1)
        self.qlabs.destroy_all_spawned_actors()

    def setup_camera(self, location, rotation):
        """Spawn and possess a free camera in QLabs."""
        print("📷 Setting up camera...")
        self.camera = QLabsFreeCamera(self.qlabs)
        self.camera.spawn_degrees(location=location, rotation=rotation)
        self.camera.possess()
        print("✅ Camera setup complete.")

    def spawn_fleet(self):
        """Spawn all QCars defined in configuration."""
        print(f"🚗 Spawning {len(self.qcars_config)} QCars...")
        self.multi_agent = MultiAgent(self.qcars_config)
        print("✅ All QCars spawned with indexes:")
        for car in self.qcars_config:
            idx = car.get('ActorNumber', car.get('actorNumber', '?'))
            print(f"   - Car ID {idx}: {car['RobotType']} at {car['Location']}")

    def close(self):
        """Close QLabs connection."""
        if self.qlabs:
            self.qlabs.close()
            print("🔌 QLabs connection closed.")

    def reset(self):
        """Reset (clear) the environment."""
        print("♻️ Resetting environment...")
        self.qlabs.destroy_all_spawned_actors()
        print("✅ Environment cleared.")

    def terminate(self):
        """Terminate the fleet and close connection."""
        print("🛑 Terminating fleet...")
        self.close()
        # Terminate any previous real-time models and clear environment
        QLabsRealTime().terminate_all_real_time_models()
        time.sleep(1)
        QLabsRealTime().terminate_all_real_time_models()
        print("✅ Fleet terminated.")

# ===============================
# Example usage
# ===============================
if __name__ == "__main__":
    # Option 1: automatic generation of N cars
    fleet = QCarFleet(num_cars=3, car_type="QC2")

    # Option 2: manually define positions (uncomment below)
    # QCars = [
    #     {"ID": 0, "RobotType": "QCar2", "Location": [-12.8, -4.6, 0], "Rotation": [0, 0, -0.7], "Radians": True, "Scale": 1},
    #     {"ID": 1, "RobotType": "QC2", "Location": [22.5, 0.8, 0], "Rotation": [0, 0, np.pi/2], "Radians": True, "Scale": 1},
    # ]
    # fleet = QCarFleet(qcars_config=QCars)

    fleet.connect()
    # fleet.setup_camera(location=[28.0, -11.7, 33.2],
    #                    rotation=[0, 51.4, 141.5])
    fleet.spawn_fleet()
    fleet.close()
    fleet.terminate()
