#!/usr/bin/env python3
"""Measure verified XYZ write/read round trips through the paxi_15eg snapshot.

The three 16-bit values are written sequentially to parameters 23, 24 and 25.
A group completes only when a full frame contains all three expected values.
Use a stationary bench, isolated motor power, and exclusive AXI access.
"""
import argparse
import ctypes
import fcntl
import json
import math
import mmap
import os
from pathlib import Path
import platform
import sys
import time

from uio_read_test_valid import BASE, WORDS, find_device
from UIO_rw.uio_write_verify import aligned_snapshots, frame_fields, require_bench_state


def reference_xyz(words):
    return (words[2] >> 16, words[3] & 0xffff, words[3] >> 16)


def capture_baseline(read_frame):
    first, second, validation = aligned_snapshots(read_frame)
    fields = [frame_fields(first), frame_fields(second)]
    result = {"validation": validation, "samples": fields, "words_u32": [first, second]}
    if validation["state"] != "LIKELY_ALIGNED":
        raise RuntimeError("Baseline alignment unconfirmed; re-establish the known PL read origin")
    for sample in fields:
        require_bench_state(sample)
    if fields[0]["ref_xyz_u16"] != fields[1]["ref_xyz_u16"]:
        raise RuntimeError("XYZ changed during baseline; check other writers")
    return result


def percentile_ms(sorted_values, percent):
    position = (len(sorted_values) - 1) * percent / 100
    lower = int(position)
    upper = min(lower + 1, len(sorted_values) - 1)
    value = sorted_values[lower] + (sorted_values[upper] - sorted_values[lower]) * (position - lower)
    return value / 1e6


def benchmark(read_frame, write_command, baseline, duration, timeout, max_groups, result):
    original = tuple(baseline["samples"][-1]["ref_xyz_u16"])
    patterns = [tuple(value ^ mask for value, mask in zip(original, masks))
                for masks in ((1, 2, 3), (2, 3, 1))]
    result.update(requested_duration_seconds=duration, group_timeout_seconds=timeout,
                  patterns_xyz_u16=patterns, completed_groups=0, attempted_groups=0,
                  write_commands_completed=0, full_frames_read=0, pending_frames=0,
                  timeout_count=0, last_confirmed_xyz_u16=list(original),
                  interval_samples=[])
    latencies = []
    previous = original
    previous_clock = baseline["samples"][-1]["clk_cnt"]
    clock_ticks = 0
    first_poll_ns = None
    last_poll_ns = None
    started = time.perf_counter_ns()
    deadline = started + int(duration * 1e9)
    interval_started = started
    interval_groups = 0
    result["end_reason"] = "error"

    try:
        while result["completed_groups"] < max_groups:
            expected = patterns[result["completed_groups"] % 2]
            allowed = [set((old, new)) for old, new in zip(previous, expected)]
            result["attempted_groups"] += 1
            cycle_started = time.perf_counter_ns()
            cycle_deadline = cycle_started + int(timeout * 1e9)
            for parameter, value in zip((23, 24, 25), expected):
                write_command((parameter << 16) | value)
                result["write_commands_completed"] += 1
            while True:
                words = read_frame()
                result["full_frames_read"] += 1
                now = time.perf_counter_ns()
                status = words[0] >> 16
                clock = words[0] & 0xffff
                delta = (clock - previous_clock) & 0xffff
                observed = reference_xyz(words)
                # A whole 256-word read always ends before checking state or time.
                if status & ~0x18 or delta > 1000:
                    result["failure_frame_u32"] = words
                    raise RuntimeError("Status/counter anomaly during polling; inspect alignment and PL state")
                if any(value not in options for value, options in zip(observed, allowed)):
                    result["failure_frame_u32"] = words
                    raise RuntimeError("Unexpected XYZ during polling; inspect alignment or concurrent writes")
                if first_poll_ns is None:
                    first_poll_ns = now
                else:
                    clock_ticks += delta
                last_poll_ns = now
                previous_clock = clock
                if now > cycle_deadline:
                    result.update(failure_frame_u32=words, failed_expected_xyz_u16=expected)
                    result["timeout_count"] += 1
                    raise RuntimeError("Timed out waiting for the current XYZ group")
                if observed == expected:
                    break
                result["pending_frames"] += 1

            latencies.append(now - cycle_started)
            result["completed_groups"] += 1
            result["last_confirmed_xyz_u16"] = list(expected)
            previous = expected
            if now - interval_started >= 1_000_000_000:
                elapsed = (now - interval_started) / 1e9
                groups = result["completed_groups"] - interval_groups
                result["interval_samples"].append({
                    "elapsed_seconds": (now - started) / 1e9,
                    "interval_seconds": elapsed, "completed_groups": groups,
                    "round_trip_hz": groups / elapsed})
                interval_started, interval_groups = now, result["completed_groups"]
            if now >= deadline:
                result["end_reason"] = "duration_elapsed"
                break
        else:
            result["end_reason"] = "max_groups_reached"
    except KeyboardInterrupt:
        result["end_reason"] = "interrupted"
        raise
    finally:
        elapsed = (time.perf_counter_ns() - started) / 1e9
        counter_seconds = 0.0
        if first_poll_ns is not None and last_poll_ns is not None:
            counter_seconds = (last_poll_ns - first_poll_ns) / 1e9
        count = result["completed_groups"]
        result.update(measured_duration_seconds=elapsed,
                      round_trip_average_hz=count / elapsed,
                      observed_counter_ticks=clock_ticks,
                      counter_observation_seconds=counter_seconds,
                      observed_counter_hz=clock_ticks / counter_seconds if counter_seconds else None)
        if latencies:
            ordered = sorted(latencies)
            result.update(round_trip_mean_ms=sum(latencies) / len(latencies) / 1e6,
                          round_trip_p50_ms=percentile_ms(ordered, 50),
                          round_trip_p95_ms=percentile_ms(ordered, 95),
                          round_trip_p99_ms=percentile_ms(ordered, 99),
                          round_trip_fastest_ms=ordered[0] / 1e6,
                          round_trip_slowest_ms=ordered[-1] / 1e6,
                          average_frames_per_completed_group=result["full_frames_read"] / count)
        intervals = result["interval_samples"]
        if intervals:
            result["best_interval_hz"] = max(s["round_trip_hz"] for s in intervals)


def restore_xyz(read_frame, write_command, original, report):
    # Attempt all three restores even if one native call raises.
    write_errors = []
    for parameter, value in zip((23, 24, 25), original):
        try:
            write_command((parameter << 16) | value)
        except (OSError, RuntimeError, KeyboardInterrupt) as exc:
            write_errors.append(str(exc) or type(exc).__name__)
    time.sleep(0.02)
    first, second, validation = aligned_snapshots(read_frame)
    fields = [frame_fields(first), frame_fields(second)]
    report["restore"] = {"expected_xyz_u16": original, "samples": fields,
                         "validation": validation, "words_u32": [first, second],
                         "write_errors": write_errors}
    if validation["state"] != "LIKELY_ALIGNED":
        raise RuntimeError("Restoration alignment unconfirmed")
    for sample in fields:
        require_bench_state(sample)
        if sample["ref_xyz_u16"] != list(original):
            raise RuntimeError("Restoration XYZ mismatch")
    if write_errors:
        raise RuntimeError("Restoration write reported an error")
    report["restored"] = True


def run_session(read_frame, write_command, args, report, save):
    baseline = capture_baseline(read_frame)
    original = tuple(baseline["samples"][-1]["ref_xyz_u16"])
    report.update(baseline=baseline, original_xyz_u16=original, restored=None)
    save()
    print("Original XYZ = {}. Starting {:.1f} s continuous round-trip test.".format(
        original, args.seconds), flush=True)
    result = {}
    report["round_trip_benchmark"] = result
    try:
        benchmark(read_frame, write_command, baseline, args.seconds,
                  args.timeout, args.max_groups, result)
        if result["counter_observation_seconds"] >= 0.1 and not 80 <= result["observed_counter_hz"] <= 120:
            raise RuntimeError("Benchmark counter rate differs from the baseline 100 Hz assumption")
    except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as exc:
        report["benchmark_error"] = str(exc) or type(exc).__name__
        raise
    finally:
        try:
            restore_xyz(read_frame, write_command, original, report)
        except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as exc:
            report.update(restored=False, restore_error=str(exc) or type(exc).__name__)
            raise
        finally:
            if report["restored"] is not True:
                print("RESTORE UNCONFIRMED. Keep motor power isolated. Original XYZ = {}".format(
                    original), file=sys.stderr, flush=True)


def run_device(args, report, save):
    if sys.byteorder != "little" or platform.machine().lower() not in ("aarch64", "arm64"):
        raise RuntimeError("Requires little-endian ARM64 Linux")
    native = ctypes.CDLL(str(Path(__file__).resolve().with_name("libuio_mmio.so")))
    native.uio_write32.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    native.uio_write32.restype = None
    native.uio_read_frame.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32)]
    native.uio_read_frame.restype = None
    device, size = find_device(args.device)
    report.update(device=device, base_hex=hex(BASE), access_method="native_volatile_uint32",
                  parameters=[23, 24, 25], readback_indices_u16=[5, 6, 7],
                  group_payload_bytes=6, command_bytes_per_group=12, full_frame_bytes=1024)
    fd = os.open(device, os.O_RDWR | os.O_SYNC)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with mmap.mmap(fd, min(size, mmap.PAGESIZE), flags=mmap.MAP_SHARED,
                       prot=mmap.PROT_READ | mmap.PROT_WRITE) as mem:
            anchor = ctypes.c_char.from_buffer(mem)
            address = ctypes.addressof(anchor)
            buffer = (ctypes.c_uint32 * WORDS)()

            def read_frame():
                native.uio_read_frame(address, buffer)
                return list(buffer)

            def write_command(value):
                native.uio_write32(address, value)

            try:
                run_session(read_frame, write_command, args, report, save)
            finally:
                del anchor
    finally:
        os.close(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=0.1, help="Per-group acknowledgement timeout in seconds")
    parser.add_argument("--max-groups", type=int, default=100000)
    parser.add_argument("--output", required=True, help="New JSON filename")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 60:
        parser.error("seconds must be between 1 and 60")
    if not math.isfinite(args.timeout) or not 0.01 <= args.timeout <= 1:
        parser.error("timeout must be between 0.01 and 1")
    if not 1 <= args.max_groups <= 100000:
        parser.error("max-groups must be between 1 and 100000")
    report = {"result": "INCOMPLETE",
              "method": "write_xyz_then_poll_complete_frames_until_exact_xyz_match",
              "timing_scope": "first XYZ command through verified full-frame readback; no sleep or log I/O in benchmark"}
    with open(args.output, "x", encoding="utf-8") as output:
        def save():
            output.seek(0)
            json.dump(report, output, indent=2, ensure_ascii=False)
            output.truncate()
            output.flush()
            os.fsync(output.fileno())

        try:
            run_device(args, report, save)
            report["result"] = "PASS"
        except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as exc:
            errors = [report[k] for k in ("benchmark_error", "restore_error") if k in report]
            report.update(result="FAIL", error=" | ".join(errors) if errors else str(exc) or type(exc).__name__)
        save()
    result = report.get("round_trip_benchmark")
    if result and "round_trip_average_hz" in result:
        print("Verified XYZ round trips: {:.2f} Hz, {} groups in {:.6f} s".format(
            result["round_trip_average_hz"], result["completed_groups"],
            result["measured_duration_seconds"]))
        if "round_trip_p95_ms" in result:
            print("Latency p50={:.3f} ms, p95={:.3f} ms, p99={:.3f} ms".format(
                result["round_trip_p50_ms"], result["round_trip_p95_ms"], result["round_trip_p99_ms"]))
    print("Result={}, restored={}".format(report["result"], report.get("restored")))
    if report.get("error"):
        print(report["error"])
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except OSError as exc:
        sys.exit(str(exc))
