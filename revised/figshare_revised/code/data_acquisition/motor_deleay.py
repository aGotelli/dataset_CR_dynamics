"""
Diagnostic script for Reviewer 5, Comment 5.32.
Quantifies the total sequential CAN readout latency across the four
Cybergear motors. Internal verification only -- not released.
"""

import time
import csv
import statistics

# --- plug in your existing CAN setup from read4MotorCircle.py here ---
# import can
# bus = can.interface.Bus(channel=..., bustype=...)

def read_motor_state(motor_id):
    """
    Replace with the exact call read4MotorCircle.py already makes to
    query one motor over CAN -- copy it in verbatim, don't reimplement it.
    """
    raise NotImplementedError("plug in the existing per-motor CAN read call")


def run_latency_check(duration_s=60.0, out_path="motor_can_latency.csv"):
    rows = []
    t_start = time.perf_counter()
    while time.perf_counter() - t_start < duration_s:
        t0 = time.perf_counter()
        read_motor_state(1); t1 = time.perf_counter()
        read_motor_state(2); t2 = time.perf_counter()
        read_motor_state(3); t3 = time.perf_counter()
        read_motor_state(4); t4 = time.perf_counter()
        rows.append({"t0": t0, "dt1": t1-t0, "dt2": t2-t1,
                     "dt3": t3-t2, "dt4": t4-t3, "dt_total": t4-t0})

    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader(); w.writerows(rows)

    totals = [r["dt_total"] for r in rows]
    print(f"n = {len(totals)}")
    print(f"mean: {1e3*statistics.mean(totals):.3f} ms")
    print(f"std:  {1e3*statistics.stdev(totals):.3f} ms")
    print(f"max:  {1e3*max(totals):.3f} ms")


if __name__ == "__main__":
    run_latency_check()