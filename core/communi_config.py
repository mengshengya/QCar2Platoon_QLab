from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Literal, Tuple, Optional, Any
import os
import csv
import json
import re

@dataclass
class CommConfig:
    # 模式: 全互联、手动邻接、基于半径
    mode: Literal["all_to_all", "manual", "radius"] = "manual"
    # 手动邻居表（当 mode == "manual" 时使用）
    manual_neighbors: Dict[int, List[int]] = field(default_factory=dict)
    # 半径模式下的邻居半径（米）
    radius_m: float = 20.0


def apply_comm_config(comm_api, sim_cfg: SimConfig, comm_cfg: CommConfig) -> None:
    """根据配置应用车辆通信拓扑。

    - all_to_all: 全互联
    - manual: 使用 `manual_neighbors`
    - radius: 先注册全部，再按半径计算邻居
    """
    if comm_cfg.mode == "all_to_all":
        comm_api.configure_all_to_all(num_agents=sim_cfg.num_cars)
    elif comm_cfg.mode == "manual":
        neighbors = dict(comm_cfg.manual_neighbors)
        if not neighbors:
            loaded_neighbors, adj, src = _load_manual_neighbors_from_data(sim_cfg)
            if loaded_neighbors:
                neighbors = loaded_neighbors
                # store for later access by users
                global MANUAL_COMM_ADJ, MANUAL_COMM_SOURCE
                MANUAL_COMM_ADJ = adj
                MANUAL_COMM_SOURCE = src
            else:
                # fallback: simple chain topology
                neighbors = {i: [j for j in (i - 1, i + 1) if 0 <= j < sim_cfg.num_cars] for i in range(sim_cfg.num_cars)}
        comm_api.configure_manual(neighbors)
    elif comm_cfg.mode == "radius":
        # 初始注册所有车辆，再按半径计算（需要车辆持续发布位置）
        comm_api.configure_all_to_all(num_agents=sim_cfg.num_cars)
        comm_api.recompute_neighbors_by_radius(comm_cfg.radius_m)
    else:
        raise ValueError(f"Unknown comm mode: {comm_cfg.mode}")

# Expose last loaded manual comm adjacency for external use (if any)
MANUAL_COMM_ADJ: Optional[List[List[float]]] = None
MANUAL_COMM_SOURCE: str = ""

def _data_dir() -> str:
    # project root is one level up from this file's directory
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    return os.path.join(root, "data")


def _load_manual_neighbors_from_data(sim_cfg: Optional[SimConfig] = None) -> Tuple[Dict[int, List[int]], Optional[List[List[float]]], str]:
    """
    尝试从 data 目录读取手动通信拓扑，返回 (neighbors, source_path)。
    支持以下文件：
    - data/ManualComm.csv
      1) 列格式 id,neighbors 其中 neighbors 为分号或空格分隔的整数列表
      2) 边表格式 src,dst 表示无向边
    - data/ManualComm.json
      字典格式 {"0": [1,2], "1": [0], ...}
    若未找到或解析失败，返回 ({}, "").
    """
    data_dir = _data_dir()
    candidates = [
        os.path.join(data_dir, "ManualComm.csv"),
        os.path.join(data_dir, "manual_comm.csv"),
        os.path.join(data_dir, "ManualComm.json"),
        os.path.join(data_dir, "manual_comm.json"),
    ]
    for path in candidates:
        if not os.path.isfile(path):
            continue
        try:
            if path.lower().endswith(".json"):
                with open(path, "r", encoding="utf-8") as f:
                    raw = json.load(f)
                neighbors: Dict[int, List[int]] = {}
                for k, vs in raw.items():
                    i = int(k)
                    neighbors[i] = sorted({int(v) for v in (vs or []) if int(v) != i})
                # derive adjacency from neighbors only (unweighted -> 1.0)
                size = _infer_size_from_neighbors(neighbors, sim_cfg)
                adj = [[0.0 for _ in range(size)] for __ in range(size)]
                for i, lst in neighbors.items():
                    for j in lst:
                        adj[i][j] = 1.0
                return neighbors, adj, path
            elif path.lower().endswith(".csv"):
                with open(path, "r", encoding="utf-8-sig", newline="") as f:
                    reader = csv.DictReader(f)
                    headers_raw = reader.fieldnames or []
                    headers = [h.strip().lower() for h in headers_raw]
                    headers_norm = [re.sub(r"[\s_]+", "", h) for h in headers]
                    neighbors: Dict[int, List[int]] = {}
                    # Format A: id,neighbors (list)
                    if set(["id", "neighbors"]).issubset(headers):
                        for row in reader:
                            i = int(row.get("id") or row.get("ID"))
                            field = row.get("neighbors") or row.get("Neighbors") or ""
                            toks = re.split(r"[;\s]+", str(field).strip()) if field is not None else []
                            vals = [int(t) for t in toks if t != "" and int(t) != i]
                            neighbors[i] = sorted(set(vals))
                        size = _infer_size_from_neighbors(neighbors, sim_cfg)
                        adj = [[0.0 for _ in range(size)] for __ in range(size)]
                        for i, lst in neighbors.items():
                            for j in lst:
                                adj[i][j] = 1.0
                        return neighbors, adj, path
                    # Format B: src,dst edge list (unweighted)
                    elif set(["src", "dst"]).issubset(headers):
                        for row in reader:
                            try:
                                a = int(row.get("src") or row.get("SRC"))
                                b = int(row.get("dst") or row.get("DST"))
                            except Exception:
                                continue
                            if a == b:
                                continue
                            neighbors.setdefault(a, set()).add(b)
                            neighbors.setdefault(b, set()).add(a)
                        neighbors = {k: sorted(vs) for k, vs in neighbors.items()}
                        size = _infer_size_from_neighbors(neighbors, sim_cfg)
                        adj = [[0.0 for _ in range(size)] for __ in range(size)]
                        for i, lst in neighbors.items():
                            for j in lst:
                                adj[i][j] = 1.0
                        return neighbors, adj, path
                    # Format C: hostindex, neighborindex, weight (directed, weighted)
                    else:
                        # try flexible header matching
                        def find_col(candidates: List[str]) -> Optional[str]:
                            for cand in candidates:
                                if cand in headers:
                                    return cand
                            for cand in candidates:
                                cn = re.sub(r"[\s_]+", "", cand)
                                for h, hn in zip(headers, headers_norm):
                                    if hn == cn:
                                        return h
                            return None

                        col_host = find_col(["hostindex", "host", "src", "source"]) 
                        col_nb = find_col(["neighborindex", "neighbor", "neighbour", "dst", "target"]) 
                        col_w = find_col(["weight", "w"]) 
                        if col_host and col_nb and col_w:
                            weights: Dict[Tuple[int, int], float] = {}
                            for row in reader:
                                try:
                                    i = int(str(row.get(col_host)).strip())
                                    j = int(str(row.get(col_nb)).strip())
                                    w = float(str(row.get(col_w)).strip())
                                except Exception:
                                    continue
                                if i == j:
                                    continue
                                neighbors.setdefault(i, set()).add(j)
                                # keep the max weight if duplicate entries
                                key = (i, j)
                                if key in weights:
                                    weights[key] = max(weights[key], w)
                                else:
                                    weights[key] = w
                            # finalize neighbors to lists
                            neighbors_list = {k: sorted(vs) for k, vs in neighbors.items()}
                            # build adjacency matrix size
                            max_idx = -1
                            for (i, j) in weights.keys():
                                max_idx = max(max_idx, i, j)
                            size = max_idx + 1
                            if sim_cfg is not None:
                                size = max(size, int(sim_cfg.num_cars))
                            adj = [[0.0 for _ in range(size)] for __ in range(size)]
                            for (i, j), w in weights.items():
                                adj[i][j] = float(w)
                            return neighbors_list, adj, path
        except Exception:
            # try next candidate
            continue
    return {}, None, ""


def _infer_size_from_neighbors(neighbors: Dict[int, List[int]], sim_cfg: Optional[SimConfig]) -> int:
    max_idx = -1
    for i, lst in neighbors.items():
        max_idx = max(max_idx, int(i))
        for j in lst:
            max_idx = max(max_idx, int(j))
    size = max_idx + 1
    if sim_cfg is not None:
        size = max(size, int(sim_cfg.num_cars))
    return size