import math
import multiprocessing
import time


def planner_heavy_compute(bursts, burst_seconds):
    """Occupy a separate interpreter in repeated planner-like bursts."""
    checksum = 0.0
    for _ in range(bursts):
        deadline = time.perf_counter() + burst_seconds
        while time.perf_counter() < deadline:
            checksum += math.sin(checksum + 0.1) ** 2
    return checksum


def test_separate_planner_process_does_not_block_20_hz_tracker_schedule():
    process = multiprocessing.Process(
        target=planner_heavy_compute,
        args=(5, 0.2),
    )
    process.start()
    intervals = []
    period = 0.05
    deadline = time.perf_counter() + 1.0
    previous = time.perf_counter()
    while time.perf_counter() < deadline:
        next_tick = previous + period
        time.sleep(max(next_tick - time.perf_counter(), 0.0))
        now = time.perf_counter()
        intervals.append(now - previous)
        previous = now
    process.join(timeout=2.0)
    if process.is_alive():
        process.terminate()
        process.join()

    ordered = sorted(intervals)
    p95 = ordered[max(math.ceil(0.95 * len(ordered)) - 1, 0)]
    assert process.exitcode == 0
    assert len(intervals) >= 19
    assert p95 < 0.065
