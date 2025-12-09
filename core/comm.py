from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional, Iterable, Any, Tuple


class CommNetwork:
    """
    Lightweight in-memory communication network for multi-vehicle simulations.

    Responsibilities:
    - Maintain neighbor topology (manual, all-to-all, or radius-based via positions)
    - Store latest message per agent with timestamp
    - Provide neighbor reads for the current agent

    Notes:
    - This is process-local and intended for a single Python process controlling
      multiple vehicles (as in main.py). For multi-process or distributed setups,
      replace the backend with sockets/Redis/ZeroMQ as needed.
    """

    def __init__(self):
        # Use RLock to allow nested acquisitions (configure_* calling register_agents)
        self._lock = threading.RLock()
        self._agents: List[int] = []
        self._neighbors: Dict[int, List[int]] = {}
        self._latest: Dict[int, Tuple[float, Dict[str, Any]]] = {}
        self._positions: Dict[int, Tuple[float, Tuple[float, float, float]]] = {}

    # --- configuration ---
    def reset(self):
        with self._lock:
            self._agents.clear()
            self._neighbors.clear()
            self._latest.clear()
            self._positions.clear()

    def register_agents(self, agent_ids: Iterable[int]):
        with self._lock:
            for a in agent_ids:
                a = int(a)
                if a not in self._agents:
                    self._agents.append(a)
                self._neighbors.setdefault(a, [])

    def configure_all_to_all(self, num_agents: Optional[int] = None, agent_ids: Optional[Iterable[int]] = None):
        """Create an all-to-all neighbor topology.

        Either provide `num_agents` (agent IDs 0..num_agents-1) or an explicit list of `agent_ids`.
        """
        if agent_ids is None and num_agents is None:
            raise ValueError("Provide num_agents or agent_ids")
        ids = list(agent_ids) if agent_ids is not None else list(range(int(num_agents)))
        with self._lock:
            self.register_agents(ids)
            for a in ids:
                self._neighbors[a] = [b for b in ids if b != a]
        print(f"[CommNetwork] Configured all-to-all neighbors for {len(ids)} agents.")

    def configure_manual(self, neighbors: Dict[int, Iterable[int]]):
        """Set neighbors per agent explicitly."""
        with self._lock:
            self.register_agents(neighbors.keys())
            for a, nbs in neighbors.items():
                a = int(a)
                nlist = sorted({int(b) for b in nbs if int(b) != a})
                self._neighbors[a] = nlist
        print(f"[CommNetwork] Configured manual neighbors for {len(neighbors)} agents.")

    def recompute_neighbors_by_radius(self, radius_m: float):
        """Compute neighbors based on last known positions and a distance radius."""
        with self._lock:
            ids = list(self._agents)
            # collect latest positions (x,y); z ignored for neighborhood
            pos: Dict[int, Tuple[float, float]] = {}
            for a in ids:
                p = self._positions.get(a)
                if p is None:
                    continue
                _, (x, y, _z) = p
                pos[a] = (float(x), float(y))
            r2 = float(radius_m) ** 2
            for a in ids:
                pa = pos.get(a)
                if pa is None:
                    continue
                nbs: List[int] = []
                for b in ids:
                    if b == a:
                        continue
                    pb = pos.get(b)
                    if pb is None:
                        continue
                    dx = pa[0] - pb[0]
                    dy = pa[1] - pb[1]
                    if (dx*dx + dy*dy) <= r2:
                        nbs.append(b)
                self._neighbors[a] = sorted(nbs)

    # --- messaging ---
    def publish(self, agent_id: int, message: Dict[str, Any]):
        """Publish a message for this agent (overwrites latest). Adds timestamp."""
        t = time.time()
        msg = dict(message) if message is not None else {}
        # capture position if present
        pos = msg.get("pos") or msg.get("position")
        if pos is not None:
            try:
                x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
                with self._lock:
                    self._positions[int(agent_id)] = (t, (x, y, z))
            except Exception:
                pass
        with self._lock:
            self._latest[int(agent_id)] = (t, msg)

    def read_neighbors(self, agent_id: int, keys: Optional[Iterable[str]] = None,
                        max_age_s: Optional[float] = None) -> Dict[int, Dict[str, Any]]:
        """
        Return latest messages from neighbors of `agent_id`.
        - `keys`: if provided, return only these fields
        - `max_age_s`: if provided, filter out messages older than this age
        """
        agent_id = int(agent_id)
        with self._lock:
            nbs = list(self._neighbors.get(agent_id, []))
            out: Dict[int, Dict[str, Any]] = {}
            now = time.time()
            for nb in nbs:
                rec = self._latest.get(nb)
                if rec is None:
                    continue
                t, msg = rec
                if max_age_s is not None and (now - t) > float(max_age_s):
                    continue
                if keys is None:
                    out[nb] = dict(msg)
                else:
                    kset = set(keys)
                    out[nb] = {k: v for k, v in msg.items() if k in kset}
            return out

    # --- helpers ---
    def neighbors_of(self, agent_id: int) -> List[int]:
        with self._lock:
            return list(self._neighbors.get(int(agent_id), []))

    def agents(self) -> List[int]:
        with self._lock:
            return list(self._agents)


# Module-level singleton for convenience
comm = CommNetwork()


__all__ = ["CommNetwork", "comm"]
