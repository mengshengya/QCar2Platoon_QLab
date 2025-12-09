from typing import Iterable, Any
import numpy as np

# Snapshot helpers used by the producer loop to feed log/GUI queues.

def make_snapshot(now_ts: float, vehicles: Iterable) -> dict:
    """Package current timestamp and all vehicle states into a JSON-safe dict."""
    return {
        "timestamp": float(now_ts),                # force plain float for JSON
        "vehicles": [serialize_vehicle(v) for v in vehicles],  # per-vehicle payloads
    }


def serialize_vehicle(vehicle) -> dict:
    """Serialize a single vehicle using only JSON-safe values."""
    # state_true = getattr(vehicle, "state_true", None)                  # numpy array or None
    measurements = getattr(vehicle, "last_measurements", {})           # dict of recent sensor reads
    # control = getattr(vehicle, "last_control", None)
    return {
        "id": getattr(vehicle, "vehicle_id", None),                    # vehicle identifier
        # "state_true": _to_plain(state_true),                           # convert numpy -> list
        "measurements": {k: _to_plain(v) for k, v in measurements.items()},  # flatten nested values
        # "control": _to_plain(control),
    }


def _to_plain(val: Any) -> Any:
    """Convert numpy values to plain Python types for logging."""
    if isinstance(val, np.ndarray):
        return val.tolist()                                            # numpy -> list
    if isinstance(val, dict):
        return {k: _to_plain(v) for k, v in val.items()}               # recurse into dict
    if isinstance(val, (list, tuple)):
        return [_to_plain(v) for v in val]                             # recurse into sequences
    return val                                                         # keep scalars/None as-is
