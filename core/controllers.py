from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Dict, Any, Optional, Tuple
from pathlib import Path
from datetime import datetime

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
        self.last_raw_thr: Optional[float] = None
        raw_thr = self.throttle
        self.last_raw_thr = float(raw_thr)
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
        self.last_raw_thr: Optional[float] = None
        v = float(meas.get("v") or 0.0)
        accel = meas.get("accel", None)
        ax = float(accel[0]) if accel is not None and len(accel) else 0.0
        front_m = meas.get("front_m", np.nan)

        # No front measurement -> fallback throttle
        if not np.isfinite(front_m):
            return ControlCmd(self._clamp(self.fallback_throttle), float(self.steering))

        desired_gap = self.min_gap + self.time_headway * max(v, 0.0)
        gap_err = front_m + desired_gap

        lead_v = lead_a = 0.0
        if neighbor:
            try:
                lead_v = float(neighbor.get("v", 0.0) or 0.0)
            except Exception:
                lead_v = 0.0
            nb_accel = neighbor.get("accel") if isinstance(neighbor, dict) else None
            if nb_accel is not None:
                try:
                    lead_a = float(np.asarray(nb_accel, dtype=float).reshape(-1)[0])
                except Exception:
                    lead_a = 0.0

        rel_v = lead_v - v
        rel_a = lead_a - ax

        raw_thr = self.kp * gap_err + self.kv * rel_v + self.ka * rel_a
        self.last_raw_thr = float(raw_thr)
        thr = self._clamp(raw_thr)
        if not np.isfinite(thr):
            thr = self.fallback_throttle
        return ControlCmd(thr, float(self.steering))

    def _clamp(self, val: float) -> float:
        return float(np.clip(val, self.thr_min, self.thr_max))

@dataclass
class DistributedStrategy(ControlStrategy):
    """
    Distributed controller.
    ui = sum from j = 0 until j = i-1 K_{ij}(xi - xj)
    """

    min_gap: float
    time_headway: float
    K: np.array
    thr_min: float
    thr_max: float
    fallback_throttle: float
    steering: float = 0.0
    name: str = "distributed"

    def compute(
        self,
        t: float,
        meas: Dict[str, Any],
        neighbor: Optional[Dict[str, Any]] = None,
    ) -> ControlCmd:
        self.last_raw_thr: Optional[float] = None
        v = float(meas.get("v") or 0.0)
        accel = meas.get("accel", None)
        ax = float(accel[0]) if accel is not None and len(accel) else 0.0
        front_m = meas.get("front_m", np.nan)

        # No front measurement -> fallback throttle
        if not np.isfinite(front_m):
            return ControlCmd(self._clamp(self.fallback_throttle), float(self.steering))

        desired_gap = self.min_gap + self.time_headway * max(v, 0.0)
        gap_err = front_m + desired_gap

        lead_v = lead_a = 0.0
        if neighbor:
            try:
                lead_v = float(neighbor.get("v", 0.0) or 0.0)
            except Exception:
                lead_v = 0.0
            nb_accel = neighbor.get("accel") if isinstance(neighbor, dict) else None
            if nb_accel is not None:
                try:
                    lead_a = float(np.asarray(nb_accel, dtype=float).reshape(-1)[0])
                except Exception:
                    lead_a = 0.0

        rel_v = lead_v - v
        rel_a = lead_a - ax

        raw_thr = self.kp * gap_err + self.kv * rel_v + self.ka * rel_a
        self.last_raw_thr = float(raw_thr)
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
        self.logger = self._init_logger()

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
        raw_thr = getattr(self.strategy, "last_raw_thr", None)
        if raw_thr is not None and self.logger is not None:
            self.logger.info(f"t={t:.3f}\traw_thr={raw_thr:.6f}")
        return cmd.throttle, cmd.steering

    def _init_logger(self) -> logging.Logger:
        log_dir = Path("log/controller")
        log_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = log_dir / f"vehicle_{self.vehicle_id}_controller_{ts}.log"

        logger_name = f"vehicle_controller_{self.vehicle_id}"
        logger = logging.getLogger(logger_name)
        if not logger.handlers:
            logger.setLevel(logging.INFO)
            handler = logging.FileHandler(log_path, encoding="utf-8")
            handler.setFormatter(logging.Formatter("%(asctime)s\t%(message)s"))
            logger.addHandler(handler)
            logger.propagate = False
        return logger
