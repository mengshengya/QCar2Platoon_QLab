# simulation/consumers.py
import json                                                         # JSON serialization
import threading                                                    # thread base class
import queue                                                        # thread-safe queues
import time                                                         # timing utilities
from pathlib import Path                                            # path handling


class LogConsumer(threading.Thread):
    def __init__(self, q: queue.Queue, log_path: Path, stop_event: threading.Event, flush_interval=1.0):
        super().__init__(daemon=True)                               # daemon thread exits with main thread
        self.q = q                                                  # input queue
        self.log_path = Path(log_path)                              # log file path
        self.stop_event = stop_event                                # stop signal
        self.flush_interval = flush_interval                        # flush period

    def run(self):
        self.log_path.parent.mkdir(parents=True, exist_ok=True)     # ensure folder exists
        with self.log_path.open("a", encoding="utf-8") as f:        # open log file for append
            last_flush = time.time()                                # last flush timestamp
            while not self.stop_event.is_set() or not self.q.empty():  # process until stop + queue empty
                try:
                    snapshot = self.q.get(timeout=0.1)              # fetch one snapshot
                except queue.Empty:
                    continue                                        # nothing to do, poll again
                f.write(json.dumps(snapshot) + "\n")                # append JSON line
                if time.time() - last_flush >= self.flush_interval: # periodic flush to disk
                    f.flush()
                    last_flush = time.time()
                self.q.task_done()                                  # mark task done
            f.flush()                                               # final flush on exit


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
