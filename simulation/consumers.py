# simulation/consumers.py
import json                                                         # JSON serialization
import threading                                                    # thread base class
import queue                                                        # thread-safe queues
from pathlib import Path                                            # path handling


class LogConsumer(threading.Thread):
    """
    Writes per-vehicle logs to log/vehicles/vehicle_<id>_log_<run_tag>.log.
    Each line is one vehicle's snapshot with timestamp.
    """

    def __init__(self, q: queue.Queue, log_dir: Path, stop_event: threading.Event, run_tag: str):
        super().__init__(daemon=True)                               # daemon thread exits with main thread
        self.q = q                                                  # input queue
        self.log_dir = Path(log_dir)                                # base directory for vehicle logs
        self.stop_event = stop_event                                # stop signal
        self.run_tag = run_tag                                      # unique tag (e.g., timestamp)

    def run(self):
        veh_dir = self.log_dir / "vehicles"
        veh_dir.mkdir(parents=True, exist_ok=True)                  # ensure folder exists
        while not self.stop_event.is_set() or not self.q.empty():   # process until stop + queue empty
            try:
                snapshot = self.q.get(timeout=0.1)                  # fetch one snapshot
            except queue.Empty:
                continue                                            # nothing to do, poll again

            ts = snapshot.get("timestamp")
            vehicles = snapshot.get("vehicles", [])
            for veh in vehicles:
                vid = veh.get("id", "unknown")
                log_path = veh_dir / f"vehicle_{vid}_log_{self.run_tag}.log"
                entry = {"timestamp": ts, **veh}
                with log_path.open("a", encoding="utf-8") as f:
                    f.write(json.dumps(entry) + "\n")

            self.q.task_done()                                      # mark task done


class GuiConsumer(threading.Thread):
    def __init__(self, q: queue.Queue, stop_event: threading.Event, gui_renderer):
        super().__init__(daemon=True)                               # daemon thread
        self.q = q                                                  # input queue
        self.stop_event = stop_event                                # stop signal
        self.gui_renderer = gui_renderer                            # GUI render callback

    def run(self):
        while not self.stop_event.is_set() or not self.q.empty():   # process until stop + queue empty
            try:
                snapshot = self.q.get(timeout=0.1)                  # get one snapshot
            except queue.Empty:
                continue                                            # keep polling
            self.gui_renderer(snapshot)                             # render snapshot
            self.q.task_done()                                      # mark done
