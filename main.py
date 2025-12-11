from pal.products.qcar import IS_PHYSICAL_QCAR                   # whether using physical QCar
from core.fleet import QCarFleet                                 # fleet spawner/manager

from core.comm import comm                                       # comms placeholder (unused yet)
from core.vehicle import VehicleAgent                            # vehicle agent abstraction
# from control.controller import ControllerFactory               # future controller factory
# from control.strategies import ConstantPolicy, build_policy_from_spec
# from observer.observer import ObserverFactory, EMAObserver, HeadwayGapObserver, PassThroughObserver, build_observer_from_spec
from config.setting import get_simconfig                         # simulation config loader
# from simulation.logging import FleetLogger                      # legacy logger (unused)

from simulation.snapshot import make_snapshot                    # JSON-safe snapshot builder
from simulation.consumers import LogConsumer                     # background logger thread

import os                                                         # stdlib path utilities
import threading                                                  # thread + event primitives
import queue                                                      # thread-safe queues
import time as _time                                              # monotonic timing helpers
import traceback                                                  # optional debugging
from pathlib import Path
from datetime import datetime

# Shared queues/events for producer-consumer model
log_queue = queue.Queue(maxsize=2000)                             # buffer snapshots for logger
stop_event = threading.Event()                                    # signal to stop consumer threads

log_dir = Path("log")
log_dir.mkdir(parents=True, exist_ok=True)  # safe if it already exists
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
log_file = log_dir / f"run_{timestamp}.log"
log_thread = LogConsumer(log_queue, log_file, stop_event)    # configure log file path
log_thread.start()                                                # launch logger in background

def main():
    sim_cfg = get_simconfig()                                     # load simulation configuration
    update_rate = sim_cfg.sim_update_rate                         # update frequency [Hz]
    num_cars = sim_cfg.num_cars                                   # fleet size

    fleet = QCarFleet(num_cars=num_cars, car_type="QC2")          # 1) spawn cars in QLabs
    fleet.close()                                                 # release QLabs control if not needed
    fleet.connect()                                               # connect to QLabs
    fleet.spawn_fleet()                                           # create car models

    vehicles: list[VehicleAgent] = []                             # 2) create vehicle agents
    for car_id in range(num_cars):
        veh = VehicleAgent(                                       # construct each agent
            vehicle_id=car_id,
            rate_hz=update_rate,
            controller=None,
            observer=None,
            comm_endpoint=None,
            sensors=None,
            qcar=None,
            use_lidar=sim_cfg.use_lidar,
            use_gps=sim_cfg.use_gps,
        )
        veh.open()                                                # open hardware/sim handles
        vehicles.append(veh)                                      # store agent
        print(f"Car {car_id} connected")                          # connection feedback

    duration_s = sim_cfg.duration_s                               # total sim duration
    dt = 1.0 / float(update_rate)                                 # period per cycle
    t0 = _time.perf_counter()                                     # start timestamp
    t_next = t0                                                   # target time for next cycle
    try:
        while (_time.perf_counter() - t0) < duration_s:           # run main loop for duration
            now_perf = _time.perf_counter()                       # current time
            sleep_t = t_next - now_perf                           # remaining time before next cycle
            if sleep_t > 0:
                _time.sleep(sleep_t)                              # wait to keep pace
            cycle_start = _time.perf_counter()                    # cycle start time
            t_rel = cycle_start - t0                              # time since start
            now = t_rel                                           # simulation time (seconds)
            for vehicle in vehicles:                              # 1) read all vehicle states
                state = vehicle.update_and_get_state(now)
            # 2) read neighbor comms (reserved)
            # 3) update control/observer (reserved)

            # 4) apply control commands (reserved)
                applied_control_cmd = vehicle.apply_control_cmd(now, None, None)

            snapshot = make_snapshot(now, vehicles)               # package snapshot
            log_queue.put(snapshot)                               # enqueue for logging
            # GUI not implemented yet: placeholder for gui_queue.put(snapshot)

            t_next += dt                                          # schedule next cycle
    finally:
        stop_event.set()                                          # signal log thread to stop
        log_queue.join()                                          # wait for log queue to drain
        log_thread.join(timeout=2.0)                              # wait for log thread exit
        for v in vehicles:                                        # close all vehicle resources
            try:
                v.close()
            except Exception:
                traceback.print_exc()
        fleet.terminate()                                         # terminate the rt modle in QLabs


if __name__ == "__main__":                                        # script entrypoint
    main()                                                        # run simulation
