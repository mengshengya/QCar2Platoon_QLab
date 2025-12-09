import queue                                                       # thread-safe queues
import threading                                                   # threads and events
import time                                                        # timing utilities
from simulation.consumers import LogConsumer, GuiConsumer          # consumer thread classes
from simulation.snapshot import make_snapshot                      # snapshot helper
# from core.gui import render_snapshot                             # GUI renderer (not implemented yet)


def start_log_consumer(log_path: str = "log/run.log", maxsize: int = 2000):
    """Start log consumer thread and return (queue, stop_event, thread)."""
    log_q = queue.Queue(maxsize=maxsize)                           # create log queue
    stop_event = threading.Event()                                 # stop signal
    log_thread = LogConsumer(log_q, log_path, stop_event)          # configure log thread
    log_thread.start()                                             # launch log thread
    return log_q, stop_event, log_thread                           # hand back handles


def start_gui_consumer(gui_renderer, maxsize: int = 500):
    """Start GUI consumer thread if renderer is provided."""
    if gui_renderer is None:                                       # skip when GUI not ready
        return None, None, None
    gui_q = queue.Queue(maxsize=maxsize)                           # create GUI queue
    stop_event = threading.Event()                                 # stop signal for GUI thread
    gui_thread = GuiConsumer(gui_q, stop_event, gui_renderer)      # configure GUI thread
    gui_thread.start()                                             # launch GUI thread
    return gui_q, stop_event, gui_thread                           # hand back handles


def stop_consumer(q: queue.Queue | None, stop_event: threading.Event | None, thread: threading.Thread | None):
    """Common shutdown logic: signal, drain queue, join thread."""
    if stop_event is not None:
        stop_event.set()                                           # tell thread to exit
    if q is not None:
        q.join()                                                   # wait for remaining tasks
    if thread is not None:
        thread.join(timeout=2.0)                                   # wait for thread to finish


def demo_loop(vehicles, render_fn=None):
    """Example loop: push snapshots into log/GUI queues (not auto-run)."""
    log_q, log_stop, log_thread = start_log_consumer()             # start log consumer
    gui_q, gui_stop, gui_thread = start_gui_consumer(render_fn)    # optionally start GUI consumer
    try:
        while True:                                                # simple demonstration loop
            now = time.time()                                      # current time
            snapshot = make_snapshot(now, vehicles)                # pack snapshot
            log_q.put(snapshot)                                    # send to logger
            if gui_q is not None:
                gui_q.put(snapshot)                                # send to GUI
            time.sleep(0.02)                                       # control loop rate
    except KeyboardInterrupt:
        pass                                                       # allow Ctrl+C
    finally:
        stop_consumer(log_q, log_stop, log_thread)                 # stop log thread
        stop_consumer(gui_q, gui_stop, gui_thread)                 # stop GUI thread
