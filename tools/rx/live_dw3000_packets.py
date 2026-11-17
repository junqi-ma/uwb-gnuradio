#!/usr/bin/env python3
"""启动 X410 原生实时收包：逐包打印，每 50 个完成的包汇总一次。"""
from __future__ import annotations
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / 'gr-uwb/build-live-wrapper'


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--args', default='addr=192.168.10.2,recv_frame_size=8000,num_recv_frames=32768', help='UHD device args')
    p.add_argument('--channel', type=int, default=3)
    p.add_argument('--antenna', default='RX1', help='X410 第二接收连接器的 UHD 名称')
    p.add_argument('--subdev', default='')
    p.add_argument('--freq', type=float, default=6489.6e6)
    p.add_argument('--gain', type=float, default=60)
    p.add_argument('--seconds', type=float, default=0, help='0=持续运行，Ctrl+C 停止')
    p.add_argument('--workers', type=int, default=2)
    p.add_argument('--queue', type=int, default=24, help='预分配包窗口池容量')
    p.add_argument('--raw-blocks', type=int, default=128, help='原始接收队列块数，每块 65536 样点')
    p.add_argument('--summary', type=int, default=50, help='每多少个完成的包汇总')
    p.add_argument('--cir', choices=['auto', 'bypass', 'full'], default='auto', help='默认快速解码，失败时完整 CIR 重试；也可固定 bypass/full')
    p.add_argument('--replay', type=Path, help='诊断：按采样率回放原始 SC16，替代 UHD')
    p.add_argument('native_args', nargs=argparse.REMAINDER,
                   help='在 -- 后传入 C++ live 参数，例如 -- --sync locked --taps eq16')
    a = p.parse_args()
    if a.channel < 0 or a.seconds < 0 or not 1 <= a.workers <= 32 or not 1 <= a.queue <= 1024 or a.summary < 1 or not 6 <= a.raw_blocks <= 4096:
        p.error('通道、时长、worker、队列或汇总间隔无效')
    if a.replay:
        a.replay = a.replay.resolve()
        if not a.replay.is_file():
            p.error(f'回放文件不存在：{a.replay}')
    env = os.environ.copy()
    env.pop('LD_LIBRARY_PATH', None)  # Use this build's RPATH, not an older libuwb.
    BUILD.mkdir(parents=True, exist_ok=True)
    log_path = BUILD / 'live-console-build.log'
    print('检查并构建当前源码的实时接收程序...', flush=True)
    commands = [
        ['cmake', '-S', str(ROOT / 'gr-uwb'), '-B', str(BUILD), '-DCMAKE_BUILD_TYPE=Release', '-DENABLE_DOXYGEN=OFF', '-DUWB_BUILD_GNURADIO=OFF', '-DUWB_BUILD_EXPERIMENTS=OFF'],
        ['cmake', '--build', str(BUILD), '--target', 'uwb_dw3000_live', '-j', '4'],
    ]
    try:
        with log_path.open('w') as log:
            for command in commands:
                subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f'构建失败：{exc}\n日志：{log_path}', file=sys.stderr)
        print('\n'.join(log_path.read_text().splitlines()[-30:]), file=sys.stderr)
        return 2
    binary = BUILD / 'apps/uwb_dw3000_live'
    command = [str(binary)]
    for key in ['args', 'channel', 'antenna', 'subdev', 'freq', 'gain', 'seconds',
                'workers', 'queue', 'raw_blocks', 'summary', 'cir']:
        command += ['--' + key.replace('_', '-'), str(getattr(a, key))]
    if a.replay:
        command += ['--replay', str(a.replay)]
    command += a.native_args[1:] if a.native_args[:1] == ['--'] else a.native_args
    print('启动原生接收程序；实际配置以 C++ START 输出为准；Ctrl+C 停止。', flush=True)
    try:
        os.chdir(ROOT)
        os.execve(str(binary), command, env)
    except OSError as exc:
        print(f'启动失败：{exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
