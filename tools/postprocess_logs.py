#!/usr/bin/env python3  # enable running as script on Unix-like shells
import argparse  # parse command-line options
import json  # read/write JSON lines
import math  # check for NaN/inf
from collections import defaultdict  # group rows by vehicle id
from pathlib import Path  # path handling
from typing import Any, Dict, List  # type hints

# How to run it?
# python tools/postprocess_logs.py log/run_20251205_153135.log --field measurements.front_m
# python tools/postprocess_logs.py --vehicle-log log/vehicles/vehicle_0_20251205_153135.jsonl --field measurements.front_m

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Split run log per vehicle and optionally plot one field.")  # create CLI parser
    parser.add_argument("logfile", nargs="?", type=Path, help="JSONL log file from main run (e.g., log/run_YYYYMMDD_HHMMSS.log)")  # log input (optional if --vehicle-log is used)
    parser.add_argument("--vehicle-log", type=Path, help="Existing per-vehicle JSONL to plot directly (vehicle_<id>_<timestamp>.jsonl)")  # allow plotting from existing vehicle log
    parser.add_argument("--field", default="state_true[0]", help="Field to plot, e.g., state_true[0], measurements.v, measurements.front_m")  # plot field
    parser.add_argument("--no-plot", action="store_true", help="Skip plotting even if matplotlib is available")  # disable plotting
    return parser.parse_args()  # return parsed args


def pick_field(sample: Dict[str, Any], expr: str) -> Any:
    parts = expr.split(".")  # split by dot for nested dicts
    cur: Any = sample  # start at root dict
    for part in parts:  # walk each path segment
        if "[" in part and part.endswith("]"):  # handle index access like name[0]
            name, idx_txt = part[:-1].split("[", 1)  # separate name and index text
            cur = cur.get(name) if isinstance(cur, dict) else None  # dive into dict
            if cur is None or not isinstance(cur, (list, tuple)):  # validate container
                return None  # missing container
            if not idx_txt.isdigit():  # ensure numeric index
                return None  # bad index
            idx = int(idx_txt)  # parse index
            if idx < 0 or idx >= len(cur):  # bounds check
                return None  # out of range
            cur = cur[idx]  # pick indexed element
        else:
            if not isinstance(cur, dict) or part not in cur:  # ensure key exists
                return None  # missing path
            cur = cur[part]  # follow key
    try:
        return float(cur)  # coerce to float if possible
    except Exception:
        return None  # non-numeric values are skipped


def load_log_lines(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []  # container for parsed rows
    with path.open("r", encoding="utf-8") as f:  # open file for reading
        for line in f:  # iterate lines
            line = line.strip()  # trim whitespace
            if not line:  # skip blanks
                continue  # continue loop
            try:
                rows.append(json.loads(line))  # parse JSON line
            except json.JSONDecodeError:
                continue  # skip malformed lines
    return rows  # return parsed records


def split_per_vehicle(records: List[Dict[str, Any]]) -> Dict[Any, List[Dict[str, Any]]]:
    per_vehicle: Dict[Any, List[Dict[str, Any]]] = defaultdict(list)  # map vid -> list of rows
    for rec in records:  # iterate top-level snapshots
        t = rec.get("timestamp")  # pull run timestamp
        for veh in rec.get("vehicles", []):  # iterate vehicle entries
            vid = veh.get("id")  # vehicle id
            if vid is None:  # skip missing id
                continue  # next vehicle
            per_vehicle[vid].append({"t": t, **veh})  # store with time included
    return per_vehicle  # return grouped data


def group_vehicle_log(rows: List[Dict[str, Any]]) -> Dict[Any, List[Dict[str, Any]]]:
    per_vehicle: Dict[Any, List[Dict[str, Any]]] = defaultdict(list)  # map vid -> rows (already per-vehicle format)
    for row in rows:  # iterate entries from vehicle log
        vid = row.get("id")  # vehicle id at top-level
        if vid is None:  # skip if missing
            continue  # next row
        per_vehicle[vid].append(row)  # append row to that vehicle
    return per_vehicle  # return grouped rows


def write_vehicle_logs(per_vehicle: Dict[Any, List[Dict[str, Any]]], timestamp_tag: str) -> None:
    out_dir = Path("log/vehicles")  # output folder
    out_dir.mkdir(parents=True, exist_ok=True)  # ensure folder exists
    for vid, rows in per_vehicle.items():  # iterate each vehicle
        out_path = out_dir / f"vehicle_{vid}_{timestamp_tag}.jsonl"  # build filename
        with out_path.open("w", encoding="utf-8") as f:  # open file for writing
            for row in rows:  # iterate rows
                f.write(json.dumps(row) + "\n")  # write JSON line
        print(f"Wrote {len(rows)} rows -> {out_path}")  # status message


def plot_field(per_vehicle: Dict[Any, List[Dict[str, Any]]], field: str, timestamp_tag: str) -> None:
    try:
        import matplotlib.pyplot as plt  # lazy import matplotlib
    except ImportError:
        print("matplotlib not installed; skipping plots")  # notify missing lib
        return  # exit plotting

    plot_dir = Path("log/plots")  # folder for plots
    plot_dir.mkdir(parents=True, exist_ok=True)  # ensure folder exists
    for vid, rows in per_vehicle.items():  # iterate vehicles
        ts: List[float] = []  # time axis values
        ys: List[float] = []  # data values
        missing_ts: List[float] = []  # time stamps where data is missing
        missing_ys: List[float] = []  # placeholder y values for missing data
        for row in rows:  # iterate rows
            y_raw = pick_field(row, field)  # extract chosen field
            if y_raw is None:  # handle missing value
                missing_ts.append(row.get("t"))  # record time for missing point
                missing_ys.append(0.0)  # place marker at zero
                continue  # proceed to next row
            if isinstance(y_raw, float) and (math.isnan(y_raw) or math.isinf(y_raw)):  # handle NaN/inf
                missing_ts.append(row.get("t"))  # record missing time
                missing_ys.append(0.0)  # mark at zero
                continue  # proceed
            ts.append(row.get("t"))  # append time for valid value
            ys.append(float(y_raw))  # append numeric value
        if not ts and not missing_ts:  # no data at all
            print(f"No data for field '{field}' on vehicle {vid}")  # report empty series
            continue  # next vehicle
        plt.figure()  # new figure
        if ts:  # plot valid data if present
            plt.plot(ts, ys, label=f"veh {vid}")  # line plot
        if missing_ts:  # plot missing markers
            plt.scatter(missing_ts, missing_ys, color="red", marker="x", label="missing/NaN")  # mark missing points
        plt.xlabel("time [s]")  # x-axis label
        plt.ylabel(field)  # y-axis label
        plt.title(f"{field} vs time (veh {vid})")  # title
        plt.grid(True)  # grid for readability
        plt.legend()  # show legend
        safe_field = field.replace(".", "_").replace("[", "").replace("]", "")  # sanitize filename
        plot_path = plot_dir / f"plot_{vid}_{safe_field}_{timestamp_tag}.png"  # build output path
        plt.savefig(plot_path, dpi=150, bbox_inches="tight")  # save plot
        plt.close()  # free figure
        print(f"Saved plot -> {plot_path}")  # status message


def main() -> None:
    args = parse_args()  # parse CLI options
    if args.vehicle_log is None and args.logfile is None:  # ensure some input provided
        raise SystemExit("Provide a run log or --vehicle-log")  # abort with guidance

    if args.vehicle_log is not None:  # branch: plotting from existing vehicle log
        if not args.vehicle_log.exists():  # verify file exists
            raise SystemExit(f"Vehicle log not found: {args.vehicle_log}")  # abort if missing
        tag = args.vehicle_log.stem.replace("vehicle_", "") or args.vehicle_log.stem  # derive tag from vehicle log name
        rows = load_log_lines(args.vehicle_log)  # load vehicle log rows
        per_vehicle = group_vehicle_log(rows)  # group rows (likely single vehicle)
    else:
        if not args.logfile.exists():  # check run log existence
            raise SystemExit(f"Log file not found: {args.logfile}")  # abort if missing
        stem = args.logfile.stem  # e.g., run_YYYYMMDD_HHMMSS
        tag = stem.replace("run_", "") or stem  # extract timestamp tag
        records = load_log_lines(args.logfile)  # read run log lines
        per_vehicle = split_per_vehicle(records)  # group by vehicle
        write_vehicle_logs(per_vehicle, tag)  # emit per-vehicle logs

    if not args.no_plot:  # plotting enabled?
        plot_field(per_vehicle, args.field, tag)  # generate plots


if __name__ == "__main__":  # script entrypoint
    main()  # run main workflow
