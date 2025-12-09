"""Quick LiDAR read loop for a 5-car fleet.

Creates 5 QCarLidar instances (virtual ports 18966 + i) and polls each
every 50 ms, printing a short summary of the raw distances/angles received.
"""
import time

from pal.products.qcar import QCarLidar


def main():
    # Instantiate five lidars on consecutive virtual ports
    lidars = []
    for i in range(5):
        port = 18966 + i
        try:
            lidars.append(QCarLidar(lidarPort=port, enableFiltering=False))
            print(f"Vehicle {i}: LiDAR opened on port {port}")
        except Exception as exc:
            print(f"Vehicle {i}: Failed to open LiDAR on port {port}: {exc}")
            lidars.append(None)

    try:
        while True:
            for i, lidar in enumerate(lidars):
                if lidar is None:
                    continue
                t0 = time.time()
                ok = False
                try:
                    ok = lidar.read()
                except Exception as exc:
                    print(f"Vehicle {i}: LiDAR read error: {exc}")
                distances = getattr(lidar, "distances", None)
                angles = getattr(lidar, "angles", None)
                n = len(distances) if distances is not None else 0
                dt_ms = (time.time() - t0) * 1000.0
                print(f"Vehicle {i}: ok={ok} samples={n} elapsed={dt_ms:.2f} ms")
                time.sleep(0.05)  # 50 ms between reads
    finally:
        for i, lidar in enumerate(lidars):
            if lidar is None:
                continue
            try:
                lidar.terminate()
                print(f"Vehicle {i}: LiDAR terminated")
            except Exception as exc:
                print(f"Vehicle {i}: Termination error: {exc}")


if __name__ == "__main__":
    main()
