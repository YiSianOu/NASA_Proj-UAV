#!/usr/bin/env python3
"""Bench test for paxi_15eg: write param[23], verify datas[5], restore it.

Run in the board container with uio_read_test_valid.py in the same directory.
Use only on a stationary bench with motor power isolated and exclusive AXI access.
"""
import argparse
import ctypes
import fcntl
import json
import mmap
import os
import sys
import time

from uio_read_test_valid import BASE, READ_OFFSET, WORDS, find_device, unpack, validate_frame


PARAM = 23
COMMAND_OFFSET = 0x00


def frame_fields(words):
    halves = unpack(words)
    return {"status": halves[1], "clk_cnt": halves[0], "ref_xyz_u16": halves[5:8]}


def require_bench_state(fields):
    # Status bits 0..2 = armed, fly, auto. Bits 3..4 encode mode.
    if fields["status"] & ~0x18:
        raise RuntimeError("PL status has armed/fly/auto or reserved bits set")


def run_test(read_frame, write_command, report, save):
    # Each frame's final AXI read latches the snapshot for the NEXT frame.
    read_frame()
    first_latched = time.monotonic()
    time.sleep(0.1)
    first = read_frame()
    second_latched = time.monotonic()
    time.sleep(0.1)
    second = read_frame()
    validation = validate_frame(second, first, second_latched, first_latched)
    before, baseline = frame_fields(first), frame_fields(second)
    report.update(validation=validation, baseline_frames=[before, baseline])
    if not any(first) or validation["state"] != "LIKELY_ALIGNED":
        raise RuntimeError("No reliable aligned baseline; check PL reset and exclusive access")
    for fields in (before, baseline):
        require_bench_state(fields)
    if before["ref_xyz_u16"] != baseline["ref_xyz_u16"]:
        raise RuntimeError("Reference changed during baseline; check other writers")

    original, y, z = baseline["ref_xyz_u16"]
    report.update(original_u16=original, restored=None, steps=[])
    save()  # Persist the original before the first write.
    print("Baseline param[23] = 0x{:04X}".format(original), flush=True)

    def write_and_check(label, value):
        command = (PARAM << 16) | value
        step = {"phase": label, "command_hex": "0x{:08X}".format(command),
                "expected_u16": value}
        report["steps"].append(step)
        write_command(command)
        time.sleep(0.02)  # pos_r_i_0 is sampled on clk_1k.
        read_frame()     # Discard old snapshot, latch the post-write values.
        fields = frame_fields(read_frame())
        step["readback"] = fields
        step["match"] = fields["ref_xyz_u16"] == [value, y, z]
        if label == "restore":
            report["restored"] = step["match"]
        print("{}: wrote {} | PL datas[5] = 0x{:04X} | {}".format(
            label, step["command_hex"], fields["ref_xyz_u16"][0],
            "MATCH" if step["match"] else "MISMATCH"), flush=True)
        if not step["match"]:
            raise RuntimeError(label + ": PL reference readback mismatch")
        require_bench_state(fields)

    try:
        write_and_check("A", original ^ 1)
        write_and_check("B", original ^ 2)
    finally:
        # Also attempt rollback after a mismatch, exception, or Ctrl+C.
        try:
            write_and_check("restore", original)
        finally:
            if report["restored"] is not True:
                print("RESTORE UNCONFIRMED: keep motor power isolated; original param[23]"
                      " = 0x{:04X}. Re-establish the known bench state.".format(original),
                      file=sys.stderr, flush=True)


def test_device(requested, report, save):
    device, size = find_device(requested)
    report.update(device=device, base_hex=hex(BASE), parameter=PARAM, readback_index_u16=5)
    fd = os.open(device, os.O_RDWR | os.O_SYNC)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with mmap.mmap(fd, min(size, mmap.PAGESIZE), flags=mmap.MAP_SHARED,
                       prot=mmap.PROT_READ | mmap.PROT_WRITE) as mem:
            command = ctypes.c_uint32.from_buffer(mem, COMMAND_OFFSET)
            data = ctypes.c_uint32.from_buffer(mem, READ_OFFSET)

            def read_frame():
                return [data.value for _ in range(WORDS)]

            def write_command(value):
                command.value = value

            try:
                run_test(read_frame, write_command, report, save)
            finally:
                del command, data
    finally:
        os.close(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", help="Optional UIO device; checked against sysfs")
    parser.add_argument("--output", required=True, help="New JSON file; existing files are preserved")
    args = parser.parse_args()
    report = {"result": "INCOMPLETE"}
    with open(args.output, "x", encoding="utf-8") as output:
        def save():
            output.seek(0)
            json.dump(report, output, indent=2, ensure_ascii=False)
            output.truncate()
            output.flush()
            os.fsync(output.fileno())

        try:
            test_device(args.device, report, save)
            report["result"] = "PASS"
        except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as exc:
            report.update(result="FAIL", error=str(exc) or type(exc).__name__)
        save()
    print(report["result"], report.get("error", ""), flush=True)
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except OSError as exc:
        sys.exit(str(exc))
