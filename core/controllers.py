from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Dict, Any, Optional, Tuple

import numpy as np


# ===== Data structures =====
@dataclass
class ControlCmd:
    throttle: float
    steering: float


class ControlStrategy:
    """Base class for control strategies."""

    name: str = "base"

    def compute(
        self,
        t: float,
        meas: Dict[str, Any],
        neighbor: Optional[Dict[str, Any]] = None,
    ) -> ControlCmd:
        raise NotImplementedError


@dataclass
class ConstantStrategy(ControlStrategy):
    """Always outputs a fixed throttle/steering."""

    throttle: float
    steering: float = 0.0
    name: str = "constant"

    def compute(
        self,
        t: float,
        meas: Dict[str, Any],
        neighbor: Optional[Dict[str, Any]] = None,
    ) -> ControlCmd:
        return ControlCmd(float(self.throttle), float(self.steering))


@dataclass
class HeadwayStrategy(ControlStrategy):
    """
    Simple longitudinal headway controller.

    Uses LiDAR front distance + ego speed/accel and optional leader speed/accel.
    """

    min_gap: float
    time_headway: float
    kp: float
    kv: float
    ka: float
    thr_min: float
    thr_max: float
    fallback_throttle: float
    steering: float = 0.0
    name: str = "headway"

    def compute(
        self,
        t: float,
        meas: Dict[str, Any],
        neighbor: Optional[Dict[str, Any]] = None,
    ) -> ControlCmd:
        v = float(meas.get("v") or 0.0)
        accel = meas.get("accel", None)
        ax = float(accel[0]) if accel is not None and len(accel) else 0.0
        front_m = meas.get("front_m", np.nan)

        # No front measurement -> fallback throttle
        if not np.isfinite(front_m):
            return ControlCmd(self._clamp(self.fallback_throttle), float(self.steering))

        desired_gap = self.min_gap + self.time_headway * max(v, 0.0)
        gap_err = front_m - desired_gap

        # lead_v = float(neighbor.get("lead_v", 0.0)) if neighbor else 0.0
        # lead_a = float(neighbor.get("lead_a", 0.0)) if neighbor else 0.0
        # rel_v = lead_v - v
        # rel_a = lead_a - ax

        raw_thr = self.kp * gap_err 
        # + self.kv * rel_v + self.ka * rel_a
        thr = self._clamp(raw_thr)
        if not np.isfinite(thr):
            thr = self.fallback_throttle
        return ControlCmd(thr, float(self.steering))

    def _clamp(self, val: float) -> float:
        return float(np.clip(val, self.thr_min, self.thr_max))


# ===== Factory =====
class StrategyFactory:
    """Load per-vehicle strategy from JSON config."""

    def __init__(self, cfg_path: str = "config/config_strategy.json"):
        with open(cfg_path, "r", encoding="utf-8") as f:
            self.cfg = json.load(f)

    def build_for_vehicle(self, vehicle_id: int) -> ControlStrategy:
        spec = self.cfg.get(str(vehicle_id))
        if spec is None:
            raise KeyError(f"No control strategy for vehicle {vehicle_id}")
        return self._build_from_spec(spec)

    def _build_from_spec(self, spec: Dict[str, Any]) -> ControlStrategy:
        stype = spec.get("type")
        params = spec.get("params", {}) or {}
        if stype == "constant":
            return ConstantStrategy(**params)
        if stype == "headway":
            return HeadwayStrategy(**params)
        raise ValueError(f"Unknown strategy type: {stype}")


# ===== Vehicle-level wrapper =====
class VehicleController:
    """Wrapper that binds a strategy to a vehicle."""

    def __init__(self, vehicle_id: int, rate_hz: float, strategy: ControlStrategy):
        self.vehicle_id = int(vehicle_id)
        self.rate_hz = float(rate_hz)
        self.strategy = strategy

    @classmethod
    def from_config(
        cls, vehicle_id: int, rate_hz: float, cfg_path: str = "config/config_strategy.json"
    ) -> "VehicleController":
        strategy = StrategyFactory(cfg_path).build_for_vehicle(vehicle_id)
        return cls(vehicle_id, rate_hz, strategy)

    def compute(
        self,
        t: float,
        meas: Dict[str, Any],
        neighbor: Optional[Dict[str, Any]] = None,
    ) -> Tuple[float, float]:
        cmd = self.strategy.compute(t, meas, neighbor)
        return cmd.throttle, cmd.steering
