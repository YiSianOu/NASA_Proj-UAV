#!/usr/bin/env python3
"""讀取 backup paxi_15eg 的 PL 快照，做 Alignment Validation 及選用的 MPC 速度測試。

只從 UIO 的 0x0C 讀取 256 個 32-bit word；不寫入、不重置、不嘗試自動同步。
每次 MMIO 讀取可能推進 PL 的 data_cnt，因此所有驗證都只使用已讀到的資料。
驗證僅能回報「可能已對齊」或可疑情況，無法證明位元流版本及資料起點。
Bench 模式維持每輪 256 次讀取，量測無 sleep 的持續讀取吞吐與 MPC 數值變化率。
只使用 Python 標準函式庫；適用板端 Linux 的 Python 3。
"""
import argparse
import ctypes
import fcntl
import json
import mmap
import os
from pathlib import Path
import sys
import time

BASE = 0x80000000
NAME = 'LED_lite_v1_0'
WORDS = 256
READ_OFFSET = 0x0C
STATUS_USED_MASK = 0x001F  # axi.v: {11'b0, mode[1:0], auto, fly, armed}
# axi.v: datas[262:342], [342:422], [422:502] 為 x/y/z 各 80 筆 16-bit
# 32-bit word index = 16-bit index // 2；共 120 個 word / 480 bytes。
MPC_WORD_START = 131
MPC_WORD_END = 251  # exclusive
MPC_BYTES_PER_FRAME = (MPC_WORD_END - MPC_WORD_START) * 4


def number(path):
    return int(path.read_text().strip(), 0)


def find_device(requested):
    matches = []
    for entry in sorted(Path('/sys/class/uio').glob('uio*')):
        if requested and entry.name != Path(requested).name:
            continue
        try:
            name = (entry / 'name').read_text().strip()
            region = entry / 'maps/map0'
            addr = number(region / 'addr')
            size = number(region / 'size')
            offset = number(region / 'offset')
        except (OSError, ValueError):
            continue
        if name == NAME and addr == BASE:
            matches.append((str(Path('/dev') / entry.name), size, offset))
    if len(matches) != 1:
        raise RuntimeError('無法唯一確認目標 UIO。請檢查 /sys/class/uio、裝置名稱與位址；'
                           '可用 --device /dev/uioN 指定，但仍會核對硬體資訊。')
    device, size, offset = matches[0]
    if offset != 0 or size < READ_OFFSET + 4:
        raise RuntimeError('UIO 映射大小或 offset 不符合這份 backup，停止存取。')
    if not Path(device).exists():
        raise RuntimeError(device + ' 不存在於容器內，請檢查 Docker 裝置設定。')
    return device, size


def unpack(words):
    return [half for word in words for half in (word & 0xffff, word >> 16)]


def validate_frame(words, previous_frame=None, start_time=None, previous_start_time=None,
                   counter_hz=100.0, expected_esc=None, rc_range=None):
    """僅處理已取得的快照資料；絕不觸碰 MMIO、重排資料或推進計數器。

    previous_frame 是上一個非零快照的 words_u32；start_time 等需為相同
    monotonic 時鐘基準下的時間戳。沒有足夠依據時回報 INCONCLUSIVE。
    """
    if len(words) != WORDS:
        raise ValueError('每輪資料必須恰好有 {} 個 32-bit word'.format(WORDS))

    values = unpack(words)
    status = values[1]
    clock = values[0]
    esc = values[30:34]
    rc = values[34:38]
    observed = {
        'clk_cnt_u16': clock,
        'status_u16': status,
        'armed': status & 1,
        'fly': (status >> 1) & 1,
        'auto': (status >> 2) & 1,
        'mode': (status >> 3) & 3,
        'esc_mag_u16': esc,
        'rc_mag_u16': rc,
    }
    checks = {
        'status_reserved_bits_zero': None,
        'counter_increment_matches_assumed_hz': None,
        'esc_matches_expected': None,
        'rc_within_expected_range': None,
    }
    result = {'state': 'INCONCLUSIVE', 'observed_if_aligned': observed,
              'checks': checks, 'counter_comparison': None, 'notes': []}
    notes = result['notes']

    if not any(words):
        result['state'] = 'NO_DATA'
        notes.append('全零資料可能是重置後尚未更新的快照；不能用來確認對齊。')
        return result

    # 11 個保留位元在 axi.v 中被明確接成 0；不依賴 armed/fly 等動態狀態。
    checks['status_reserved_bits_zero'] = (status & ~STATUS_USED_MASK) == 0
    if not checks['status_reserved_bits_zero']:
        notes.append('假設索引 1 為 status 時，其保留位元不為 0。')

    # 只有「操作者知道測試期間 ESC 固定」時才檢查固定值。
    if expected_esc is not None:
        checks['esc_matches_expected'] = all(value == expected_esc for value in esc)
        if not checks['esc_matches_expected']:
            notes.append('ESC 數值與指定的測試期望值不符；若馬達命令會變化，請勿設定 --expected-esc。')

    # RC 範圍取決於實際 RTL/遙控器設定，不擅自假設一定在 1000～2000。
    if rc_range is not None:
        low, high = rc_range
        checks['rc_within_expected_range'] = all(low <= value <= high for value in rc)
        if not checks['rc_within_expected_range']:
            notes.append('RC 數值超出指定範圍；也可能是測試狀態或資料單位不同。')

    # clk_cnt 在 axi.v 由 clk_100 推進。用兩輪「實際擷取起始時間差」
    # 檢查增量（16-bit wrap-around）；第一筆有效快照沒有前一筆可供比較。
    if (previous_frame is not None and start_time is not None
            and previous_start_time is not None and counter_hz > 0):
        previous_clock = unpack(previous_frame)[0]
        seconds = start_time - previous_start_time
        expected_ticks = seconds * counter_hz
        actual_ticks = (clock - previous_clock) & 0xffff
        # 小於 5 ticks 的間隔無足夠辨識力，不下「已對齊」判斷。
        if 5 <= expected_ticks < 65536:
            tolerance = max(2.0, expected_ticks * 0.20)
            passed = abs(actual_ticks - expected_ticks) <= tolerance
            checks['counter_increment_matches_assumed_hz'] = passed
            result['counter_comparison'] = {
                'previous_clk_cnt_u16': previous_clock,
                'current_clk_cnt_u16': clock,
                'increment_mod_65536': actual_ticks,
                'elapsed_wall_seconds': seconds,
                'expected_increment': expected_ticks,
                'tolerance_ticks': tolerance,
                'assumed_counter_hz': counter_hz,
            }
            if not passed:
                notes.append('clk_cnt 增量與指定的 --counter-hz 不符；也可能是時鐘/時序假設有誤。')
        else:
            notes.append('擷取間隔太短或超出可比較範圍；略過 clk_cnt 速度檢查。')
    else:
        notes.append('缺少前一筆非零快照、時間戳，或已關閉 clk_cnt 速度檢查。')

    checked = [value for value in checks.values() if value is not None]
    if any(value is False for value in checked):
        result['state'] = 'CHECK_FAILED'
        notes.append('檢查未通過不等於已證明資料錯位，也可能是運作狀態或假設不符。')
    elif checks['counter_increment_matches_assumed_hz'] is True:
        result['state'] = 'LIKELY_ALIGNED'
        notes.append('多輪 clk_cnt 變化及已啟用檢查符合預期；非絕對的硬體同步保證。')
    else:
        result['state'] = 'INCONCLUSIVE'
        notes.append('通過的靜態欄位檢查仍不足以單獨證明資料起點。')
    return result


def summarize_validation(records):
    states = [record['alignment_validation']['state'] for record in records]
    if 'CHECK_FAILED' in states:
        state = 'CHECK_FAILED'
    elif 'LIKELY_ALIGNED' in states:
        state = 'LIKELY_ALIGNED'
    elif all(value == 'NO_DATA' for value in states):
        state = 'NO_DATA'
    else:
        state = 'INCONCLUSIVE'
    return {'state': state, 'per_round_states': states,
            'meaning': '僅根據已讀原始資料檢查；無法證明 bitstream 版本、Reset 或無其他 AXI 讀取者。'}



def _percentile(sorted_ns, percentile):
    """對連續讀取耗時做線性插值百分位數，單位為 ms。"""
    if not sorted_ns:
        return None
    position = (len(sorted_ns) - 1) * percentile / 100.0
    lower = int(position)
    fraction = position - lower
    higher = min(lower + 1, len(sorted_ns) - 1)
    return (sorted_ns[lower] + (sorted_ns[higher] - sorted_ns[lower]) * fraction) / 1e6


def _mpc_preview(words):
    """不額外讀 MMIO；只從已取得的完整 frame 擷取三軸第一筆 raw u16。"""
    return {'x0_u16': words[131] & 0xffff,
            'y0_u16': words[171] & 0xffff,
            'z0_u16': words[211] & 0xffff}


def benchmark_mpc(reg, seconds, max_frames, alignment_state):
    """在相同映射上連續讀取完整 frame；量測時間內不印出、不寫 JSON、不 sleep。

    每次循環必須完整執行 256 次 32-bit MMIO 讀取，避免結束在半包。
    不要求 MPC 一定發生變化；變化率僅代表「可觀察到的數值變化」，
    不能當作 MPC 演算法真正計算或發布新 setpoint 的更新率。
    """
    read_durations_ns = []
    read_count = 0
    mpc_changes = 0
    mpc_opt_changes = 0
    mpc_nonzero_frames = 0
    status_reserved_invalid = 0
    counter_changes = 0
    counter_ticks_sum = 0
    counter_large_jumps = 0
    previous_mpc = None
    previous_mpc_opt = None
    first_mpc_opt = None
    last_mpc_opt = None
    previous_clk = None
    first_preview = None
    last_preview = None
    samples = []
    sample_step_ns = 500_000_000  # 每 0.5s 記錄輕量統計，非額外硬體讀取
    started_ns = time.perf_counter_ns()
    deadline_ns = started_ns + int(seconds * 1e9)
    next_sample_ns = started_ns + sample_step_ns
    end_ns = started_ns

    while read_count < max_frames:
        read_started_ns = time.perf_counter_ns()
        words = [reg.value for _ in range(WORDS)]  # 此模式唯一的 MMIO 存取，完整 256 次
        read_end_ns = time.perf_counter_ns()
        read_durations_ns.append(read_end_ns - read_started_ns)
        read_count += 1

        # MPC future x/y/z: each 80 x 16-bit; just use the 120 packed words.
        mpc_words = words[MPC_WORD_START:MPC_WORD_END]
        if any(mpc_words):
            mpc_nonzero_frames += 1
        if previous_mpc is not None and mpc_words != previous_mpc:
            mpc_changes += 1
        previous_mpc = mpc_words
        # axi.v: mpc_opt_10/11/12 = datas[23], datas[24], datas[25]
        mpc_opt = ((words[11] >> 16) & 0xffff,
                   words[12] & 0xffff, (words[12] >> 16) & 0xffff)
        if previous_mpc_opt is not None and mpc_opt != previous_mpc_opt:
            mpc_opt_changes += 1
        previous_mpc_opt = mpc_opt
        if first_mpc_opt is None:
            first_mpc_opt = list(mpc_opt)
        last_mpc_opt = list(mpc_opt)

        word0 = words[0]
        clk = word0 & 0xffff
        status = (word0 >> 16) & 0xffff
        if status & ~STATUS_USED_MASK:
            status_reserved_invalid += 1
        if previous_clk is not None:
            delta = (clk - previous_clk) & 0xffff
            if delta:
                counter_changes += 1
                counter_ticks_sum += delta
                # 高速讀取時，相鄰 frame 不應相差上千 ticks；可能重置或對齊錯誤。
                if delta > 1000:
                    counter_large_jumps += 1
        previous_clk = clk
        if first_preview is None:
            first_preview = _mpc_preview(words)
        last_preview = _mpc_preview(words)

        # Benchmark 時不做磁碟 I/O 或 terminal I/O。
        end_ns = time.perf_counter_ns()
        if end_ns >= next_sample_ns and len(samples) < 240:
            elapsed = (end_ns - started_ns) / 1e9
            samples.append({'elapsed_seconds': elapsed,
                            'completed_frames': read_count,
                            'average_frame_hz': read_count / elapsed,
                            'observed_mpc_change_events': mpc_changes})
            next_sample_ns += sample_step_ns
        if end_ns >= deadline_ns:
            break

    elapsed_seconds = (end_ns - started_ns) / 1e9
    if read_count == 0 or elapsed_seconds <= 0:
        raise RuntimeError('Benchmark 未讀到任何完整 frame')
    durations = sorted(read_durations_ns)
    hz = read_count / elapsed_seconds
    read_only_seconds = sum(read_durations_ns) / 1e9
    result = {
        'method': 'continuous_full_frame_256x32bit_mmio_no_sleep',
        'device_read_offset': READ_OFFSET,
        'requested_duration_seconds': seconds,
        'measured_duration_seconds': elapsed_seconds,
        'end_reason': 'max_frames_reached' if read_count >= max_frames else 'duration_elapsed',
        'frame_count': read_count,
        'mmio_reads': read_count * WORDS,
        'full_frame_bytes': WORDS * 4,
        'mpc_future_x_u16_indices': [262, 341],
        'mpc_future_y_u16_indices': [342, 421],
        'mpc_future_z_u16_indices': [422, 501],
        'mpc_payload_bytes_per_frame': MPC_BYTES_PER_FRAME,
        'full_frame_average_hz': hz,
        'full_frame_throughput_kib_per_second': hz * (WORDS * 4) / 1024.0,
        'mpc_effective_payload_kib_per_second': hz * MPC_BYTES_PER_FRAME / 1024.0,
        'read_only_mean_ms_per_frame': read_only_seconds * 1000 / read_count,
        'read_only_p50_ms': _percentile(durations, 50),
        'read_only_p95_ms': _percentile(durations, 95),
        'read_only_p99_ms': _percentile(durations, 99),
        'read_only_slowest_ms': durations[-1] / 1e6,
        'read_only_fastest_ms': durations[0] / 1e6,
        'observed_mpc_value_change_events': mpc_changes,
        'observed_mpc_value_change_hz': mpc_changes / elapsed_seconds,
        'observed_mpc_opt_change_events': mpc_opt_changes,
        'observed_mpc_opt_change_hz': mpc_opt_changes / elapsed_seconds,
        'mpc_opt_10_11_12_first_u16_if_aligned': first_mpc_opt,
        'mpc_opt_10_11_12_last_u16_if_aligned': last_mpc_opt,
        'mpc_nonzero_frames': mpc_nonzero_frames,
        'observed_clk_cnt_change_events': counter_changes,
        'observed_clk_cnt_ticks_sum_mod_65536': counter_ticks_sum,
        'observed_clk_cnt_tick_hz_if_aligned': counter_ticks_sum / elapsed_seconds,
        'clk_cnt_large_jump_count': counter_large_jumps,
        'status_reserved_bits_invalid_frames_if_aligned': status_reserved_invalid,
        'alignment_validation_before_benchmark': alignment_state,
        'mpc_first_values_if_aligned': first_preview,
        'mpc_last_values_if_aligned': last_preview,
        'progress_samples': samples,
        'notes': [
            '這是此容器中 Python 讀取完整 1024-byte frame 的持續吞吐，包含 MPC 欄位的擷取/比較成本；不是 PL MPC 演算法的計算速率。',
            '480-byte MPC effective payload/s 是 240 個 16-bit 欄位的等效讀取率；實際 MMIO 仍讀取整包 1024 bytes。',
            'MPC 數值即使不變仍可能已重新計算；僅觀察數值變化無法量測精確的新資料發布頻率。',
            '另外同時追蹤 mpc_opt_10/11/12（datas[23:25]）的變化率，和 240 個 mpc_future 值分開統計。',
            '所有 MPC 欄位與 clk_cnt 的索引解讀都以已對齊且 bitstream 與 axi.v 一致為前提。',
            '速度測試只在讀滿 256 個 word 後停止，不額外讀/寫 AXI、不嘗試同步。'
        ],
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', help='例如 /dev/uio4；預設依 sysfs 自動尋找')
    parser.add_argument('--frames', type=int, default=3, help='讀取輪數，預設 3')
    parser.add_argument('--interval', type=float, default=0.1, help='輪次間隔秒數，預設 0.1')
    parser.add_argument('--aligned', action='store_true',
                        help='操作者已在外部確認起點；僅記錄宣告，不會重置或跳過檢查')
    parser.add_argument('--counter-hz', type=float, default=100.0,
                        help='依 axi.v 的 clk_100 推定為 100 Hz；設 0 可略過速度檢查')
    parser.add_argument('--expected-esc', type=int,
                        help='選用：已知四個 esc_mag 在測試時皆固定為此數值才設定，例如 1800')
    parser.add_argument('--rc-range', type=int, nargs=2, metavar=('MIN', 'MAX'),
                        help='選用：已知 RC 四個欄位的有效範圍才設定，例如 900 2100')
    parser.add_argument('--benchmark-seconds', type=float, default=0.0,
                        help='選用：原本讀取測試後，額外連續無 sleep 量測幾秒（建議 5；預設 0 = 關閉）')
    parser.add_argument('--benchmark-max-frames', type=int, default=1000000,
                        help='Benchmark 安全上限：最多完整讀取幾個 frame（預設 1000000）')
    parser.add_argument('--output', help='可選：儲存原始資料、驗證結果、Benchmark 統計的 JSON（路徑不得已存在）')
    args = parser.parse_args()
    if not 1 <= args.frames <= 1000 or not 0 <= args.interval <= 60:
        parser.error('frames 必須為 1～1000，interval 必須為 0～60 秒')
    if not 0 <= args.counter_hz <= 100000:
        parser.error('counter-hz 必須為 0～100000')
    if args.expected_esc is not None and not 0 <= args.expected_esc <= 0xffff:
        parser.error('expected-esc 必須為 0～65535')
    if args.rc_range is not None:
        low, high = args.rc_range
        if not 0 <= low <= high <= 0xffff:
            parser.error('rc-range 必須滿足 0 <= MIN <= MAX <= 65535')
    if not 0 <= args.benchmark_seconds <= 120:
        parser.error('benchmark-seconds 必須為 0～120 秒')
    if not 1 <= args.benchmark_max_frames <= 1000000:
        parser.error('benchmark-max-frames 必須為 1～1000000')
    if args.output and Path(args.output).exists():
        parser.error('輸出檔已存在，請更換名稱')

    device, size = find_device(args.device)
    print('裝置：{}；實體位址：0x{:08X}；map0 大小：0x{:X}'.format(device, BASE, size), flush=True)
    print('只讀取偏移 0x0C；每輪 256 次 32-bit 讀取；不寫入、不重置；'
          '如啟用 Benchmark，會額外連續讀取完整 frame。', flush=True)
    print('對齊：{}；自動驗證啟用，但不會自動同步。'.format(
        '操作者已宣告起點對齊' if args.aligned else '起點未知'), flush=True)

    records = []
    previous_nonzero = None
    previous_nonzero_started = None
    benchmark_result = None
    fd = os.open(device, os.O_RDWR | os.O_SYNC)
    try:
        # 協同鎖只阻止同樣使用 flock 的程式；無法阻止 /dev/mem 或其他 AXI Master。
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        length = min(size, mmap.PAGESIZE)
        with mmap.mmap(fd, length, flags=mmap.MAP_SHARED,
                       prot=mmap.PROT_READ | mmap.PROT_WRITE, offset=0) as mem:
            # from_buffer 需要可寫映射，但此程式從未對 .value 賦值。
            # 每次 .value 為單次對齊的 uint32 讀取，不用切片或 memcpy 讀 MMIO。
            reg = ctypes.c_uint32.from_buffer(mem, READ_OFFSET)
            try:
                previous = None
                for index in range(args.frames):
                    started = time.monotonic()
                    words = [reg.value for _ in range(WORDS)]  # 基本測試唯一的 MMIO 讀取位置
                    elapsed = time.monotonic() - started
                    values = unpack(words)
                    validation = validate_frame(
                        words, previous_frame=previous_nonzero,
                        start_time=started, previous_start_time=previous_nonzero_started,
                        counter_hz=args.counter_hz, expected_esc=args.expected_esc,
                        rc_range=args.rc_range)
                    records.append({'round': index + 1, 'elapsed_seconds': elapsed,
                                    'capture_start_monotonic_seconds': started,
                                    'words_u32': words, 'halves_u16': values,
                                    'alignment_validation': validation})
                    changed = None if previous is None else sum(a != b for a, b in zip(previous, words))
                    print('輪次 {}：讀取 {} bytes，耗時 {:.3f} ms，與上一輪不同字數：{}'.format(
                        index + 1, WORDS * 4, elapsed * 1000, changed), flush=True)
                    print('前 8 個原始字：' + ' '.join('{:08X}'.format(w) for w in words[:8]), flush=True)
                    print('Alignment Validation：{}'.format(validation['state']), flush=True)
                    if validation['state'] in ('CHECK_FAILED', 'INCONCLUSIVE'):
                        print('  判斷說明：' + '；'.join(validation['notes']), flush=True)
                    obs = validation['observed_if_aligned']
                    if args.aligned or validation['state'] == 'LIKELY_ALIGNED':
                        print('假設已對齊：clk_cnt={}；armed={} fly={} auto={} mode={}'.format(
                            obs['clk_cnt_u16'], obs['armed'], obs['fly'], obs['auto'], obs['mode']), flush=True)
                    if not any(words):
                        print('本輪全為零：重置後首輪可能如此，不能單憑此判定故障。', flush=True)
                    else:
                        previous_nonzero = words
                        previous_nonzero_started = started
                    previous = words
                    if index + 1 < args.frames:
                        time.sleep(args.interval)
                # Bench 和原本的三輪測試共用同一個 fd/mmap 與互斥鎖。
                # 完整讀取每個 256-word frame，保持 modulo-256 讀取相位。
                if args.benchmark_seconds > 0:
                    validation_summary = summarize_validation(records)
                    print('開始 MPC Benchmark：{:.3f} 秒，無 sleep，每輪完整讀取 {} bytes；'
                          '過程中不印出或儲存每筆 frame。'.format(
                              args.benchmark_seconds, WORDS * 4), flush=True)
                    benchmark_result = benchmark_mpc(
                        reg, args.benchmark_seconds, args.benchmark_max_frames,
                        validation_summary['state'])
                    print('Benchmark：{:.1f} frames/s，{:.2f} KiB/s（完整 AXI frame），'
                          '{:.2f} KiB/s（MPC 有效欄位）。'.format(
                              benchmark_result['full_frame_average_hz'],
                              benchmark_result['full_frame_throughput_kib_per_second'],
                              benchmark_result['mpc_effective_payload_kib_per_second']), flush=True)
                    print('單次完整 frame 耗時：p50={:.3f} ms，p95={:.3f} ms，'
                          'p99={:.3f} ms。'.format(
                              benchmark_result['read_only_p50_ms'],
                              benchmark_result['read_only_p95_ms'],
                              benchmark_result['read_only_p99_ms']), flush=True)
                    print('MPC future 240 筆數值變化：{} 次（觀察值 {:.2f} 次/s，不等於 MPC 真正更新率）。'.format(
                        benchmark_result['observed_mpc_value_change_events'],
                        benchmark_result['observed_mpc_value_change_hz']), flush=True)
                    print('MPC opt 3 筆數值變化：{} 次（觀察值 {:.2f} 次/s）。'.format(
                        benchmark_result['observed_mpc_opt_change_events'],
                        benchmark_result['observed_mpc_opt_change_hz']), flush=True)
                    print('clk_cnt 變化：{} 次；假設已對齊時計數約 {:.1f} ticks/s。'.format(
                        benchmark_result['observed_clk_cnt_change_events'],
                        benchmark_result['observed_clk_cnt_tick_hz_if_aligned']), flush=True)
                    if benchmark_result['status_reserved_bits_invalid_frames_if_aligned'] or \
                            benchmark_result['clk_cnt_large_jump_count']:
                        print('警告：Bench 中發現 status 保留位元異常或 clk_cnt 大幅跳變；'
                              '應檢查對齊/重置/並行讀取。', flush=True)
            finally:
                del reg
    finally:
        os.close(fd)

    summary = summarize_validation(records)
    if args.output:
        with open(args.output, 'x', encoding='utf-8') as output:
            json.dump({'device': device, 'base': BASE,
                       'alignment_asserted_by_user': args.aligned,
                       'alignment_validation_settings': {
                           'counter_hz_assumed': args.counter_hz,
                           'expected_esc_if_configured': args.expected_esc,
                           'rc_range_if_configured': args.rc_range},
                       'alignment_validation_summary': summary,
                       'rounds': records,
                       'mpc_benchmark': benchmark_result}, output, ensure_ascii=False, indent=2)
        print('原始資料、Validation 及 Benchmark 統計已存至 ' + args.output)
    print('整體 Alignment Validation：' + summary['state'])
    print('注意：LIKELY_ALIGNED 不代表已驗證 bitstream、Reset、無其他讀取者或所有欄位。')
    if benchmark_result is not None:
        print('注意：Bench 的 frames/s 是容器讀取能力；MPC 數值變化率不是 MPC 真正的更新頻率。')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('\n已中斷；PL 可能停在一包中途，下次不可假設從索引 0 開始。', file=sys.stderr)
        sys.exit(130)
    except (OSError, ValueError, RuntimeError) as error:
        print('錯誤：{}；若已開始讀取，PL 的讀取位置可能已改變。'.format(error), file=sys.stderr)
        sys.exit(1)
