#!/usr/bin/env python3
"""离线重跑完整 C++ 解码，逐包打印结果；默认使用 2026-11-15 的 1 秒实采。"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / 'testdata/x410_fixed_tone_live_20261115'
STATUS = ['OK', 'INVALID_INPUT', 'TIMING_FAILED', 'CFO_FAILED', 'SFD_FAILED',
          'PHR_FAILED', 'PAYLOAD_FAILED', 'FCS_FAILED', 'QUEUE_FULL',
          'INTERNAL_ERROR', 'CIR_FAILED']


def independent_crc(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value


def decoder() -> Path:
    build = ROOT / 'gr-uwb/build-offline-decoder'
    env = os.environ.copy()
    env.pop('LD_LIBRARY_PATH', None)
    subprocess.run(['cmake', '-S', str(ROOT / 'gr-uwb'), '-B', str(build),
                    '-DCMAKE_BUILD_TYPE=Release', '-DUWB_BUILD_GNURADIO=OFF',
                    '-DUWB_BUILD_EXPERIMENTS=OFF', '-DENABLE_UHD_BACKEND=OFF'],
                   env=env, check=True, stdout=sys.stderr)
    subprocess.run(['cmake', '--build', str(build), '--target',
                    'uwb_dw3000_decode', '-j4'], env=env, check=True, stdout=sys.stderr)
    return build / 'apps/uwb_dw3000_decode'


def run(args: argparse.Namespace) -> int:
    default = DATA / ('rx_ch3_rx1_1s.sc16' if args.raw else 'clean_1s.sc16')
    iq = (args.input or default).resolve()
    starts = (args.starts or DATA / 'packet_starts_1s.txt').resolve()
    known = {DATA / 'clean_1s.sc16', DATA / 'rx_ch3_rx1_1s.sc16'}
    if iq not in known and args.starts is None:
        raise RuntimeError('其他采集文件必须用 --starts 指定对应的包起点文件')
    for path in (iq, starts):
        if not path.is_file():
            raise RuntimeError(f'文件不存在：{path}')
    if iq.stat().st_size % 4:
        raise RuntimeError('SC16 文件长度必须是 4 字节的整数倍')
    seeds = [int(v) for v in starts.read_text().split()]
    if not seeds or any(v < 0 for v in seeds):
        raise RuntimeError('包起点文件必须包含非负整数样点位置，每行一个')
    if args.json_out and args.json_out.resolve() in {iq, starts}:
        raise RuntimeError('--json-out 不能覆盖输入文件')
    command = [str(decoder()), str(iq), str(starts),
               str(ROOT / 'testdata/reference_preamble_code10_998p4.cf32'),
               str(ROOT / 'testdata/resampler_65_48/taps_quality_minorder.txt'),
               str(args.preamble), args.sfd, str(args.pre_guard), str(args.window)]
    print(f'输入：{iq}\n包窗：{starts}（{len(seeds)} 个）', flush=True)
    print(f'配置：737.28 → 998.4 MS/s，code 10，{args.preamble} SYNC，'
          f'{args.sfd}，默认 CFO 配置', flush=True)
    print('离线完整解码；time_ms 是候选种子在文件中的时间，不是精确 ToA。', flush=True)
    print(' ID    time_ms SYNC SFD PHR LEN  CFO_kHz Nfit FCS CRC STATUS', flush=True)
    counts: Counter[str] = Counter()
    rows = []
    passed = 0
    env = os.environ.copy()
    env.pop('LD_LIBRARY_PATH', None)
    process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, text=True, env=env)
    try:
        assert process.stdout is not None
        for line in process.stdout:
            row = json.loads(line)
            if 'input_error' in row:
                status = row['input_error'].upper()
                print(f"{row['id'] + 1:3d} {row['seed']/737280:10.6f} "
                      f'   -   -   -   -        -    -   -   - {status}', flush=True)
            else:
                status = STATUS[row['status']] if 0 <= row['status'] < len(STATUS) else str(row['status'])
                payload = bytes.fromhex(row['hex'])
                crc_ok = (independent_crc(payload[:-2]) == int.from_bytes(payload[-2:], 'little')) if len(payload) >= 2 else None
                row['independent_crc_pass'] = crc_ok
                good = row['status'] == 0 and row['fcs'] and crc_ok is True
                passed += bool(good)
                if row['status'] == 0 and not good:
                    status = 'CRC_CHECK_FAILED'
                check = lambda value: 'OK' if value else '--'
                print(f"{row['id'] + 1:3d} {row['seed']/737280:10.6f} "
                      f"{row['peaks']:4d} {check(row['sfd_ok']):>3} "
                      f"{check(row['phr_ok']):>3} {row['length']:3d} "
                      f"{row['cfo_hz']/1000:8.3f} {row['cfo_points']:4d} "
                      f"{check(row['fcs']):>3} {check(crc_ok) if crc_ok is not None else '-':>3} "
                      f'{status}', flush=True)
                if args.payload and payload:
                    print('    PSDU: ' + payload.hex(), flush=True)
            counts[status] += 1
            rows.append(row)
        if process.wait() != 0:
            raise RuntimeError('C++ 解码程序运行失败')
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait()
    if len(rows) != len(seeds):
        raise RuntimeError(f'返回结果 {len(rows)} 个，与包窗数 {len(seeds)} 不一致')
    print(f'\n汇总：{len(rows)} 包，完整解码且 FCS/独立 CRC 通过 '
          f'{passed}/{len(rows)}（{passed/len(rows):.1%}），失败 {len(rows)-passed}。')
    print('状态：' + ', '.join(f'{k}={v}' for k, v in counts.items()))
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(''.join(json.dumps(r) + '\n' for r in rows))
        print(f'逐包 JSONL：{args.json_out.resolve()}')
    return 0 if passed == len(rows) else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--input', type=Path, help='SC16 文件；其他采集需要同时指定 --starts')
    mode.add_argument('--raw', action='store_true', help='对同一次采集的未消除原始数据进行对照')
    parser.add_argument('--starts', type=Path, help='候选起点样点，每行一个整数；默认本次 200 个包窗')
    parser.add_argument('--preamble', type=int, default=256)
    parser.add_argument('--sfd', choices=['4z1', '4z2', '4z3', '4z4', 'ieee', 'decawave'], default='4z2')
    parser.add_argument('--pre-guard', type=int, default=14746)
    parser.add_argument('--window', type=int, default=390000)
    parser.add_argument('--payload', action='store_true', help='同时打印解出的完整 PSDU 十六进制')
    parser.add_argument('--json-out', type=Path, help='另存逐包结果 JSONL')
    args = parser.parse_args()
    if args.preamble < 1 or args.pre_guard < 0 or not 1 <= args.window <= 10000000:
        parser.error('前导和窗口长度必须为正，pre-guard 不能为负')
    try:
        return run(args)
    except (RuntimeError, OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f'错误：{exc}', file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    sys.exit(main())
