"""
Watch RAM, swap and shared memory while the robot runs, and say which process is growing.

    ros2 run stretch4_rtabmap memory_monitor              # every 10 s, until Ctrl-C
    ros2 run stretch4_rtabmap memory_monitor --period 5

Every period it prints the system totals and the processes
using the most memory, with how much each grew in the last 10 minutes, and warns when memory is
running out. Everything is also written to ~/rtabmap_debug/memory_<start time>.csv.

"""

import argparse
from collections import deque
import glob
import os
import re
import sys
import time

GB = 1024.0 * 1024.0   # kB per GB


def meminfo():
    out = {}
    with open('/proc/meminfo') as f:
        for line in f:
            key, value = line.split(':', 1)
            out[key] = int(value.split()[0])
    return out


def process_name(pid, cmdline):
    """Short readable name: the ROS node name if there is one, else the executable."""
    match = re.search(r'__node:=([\w/]+)', cmdline)
    if match:
        return match.group(1)
    parts = cmdline.split()
    if not parts:
        try:
            with open(f'/proc/{pid}/comm') as f:
                return f.read().strip()
        except OSError:
            return '?'
    exe = os.path.basename(parts[0])
    if exe.startswith('python') and len(parts) > 1:
        exe = os.path.basename(parts[1])
    if exe == 'ros2' and len(parts) > 3:
        exe = f'ros2 {parts[1]} {parts[3]}'
    return exe[:40]


def gpu_kb(pid):
    """GPU memory of a process from its DRM fdinfo (counted once per DRM client)."""
    clients = {}
    for path in glob.glob(f'/proc/{pid}/fdinfo/*'):
        try:
            with open(path) as f:
                text = f.read()
        except OSError:
            continue
        if 'drm-driver' not in text:
            continue
        client = re.search(r'drm-client-id:\s*(\d+)', text)
        total = 0.0
        units = {'KiB': 1, 'MiB': 1024, 'GiB': 1024 * 1024}
        for value, unit in re.findall(r'^drm-resident-\S+:\s*(\d+)\s*(\w+)?', text, re.M):
            total += float(value) * units.get(unit, 1 / 1024)
        clients[client.group(1) if client else path] = total
    return int(sum(clients.values()))


def processes(min_kb):
    """Return {pid: dict} for every readable process using at least min_kb (resident + swap)."""
    out = {}
    for path in glob.glob('/proc/[0-9]*'):
        pid = int(path.rsplit('/', 1)[1])
        try:
            with open(f'{path}/status') as f:
                status = dict(line.split(':', 1) for line in f if ':' in line)
            with open(f'{path}/cmdline') as f:
                cmdline = f.read().replace('\0', ' ').strip()
        except OSError:
            continue

        def kb(key):
            return int(status.get(key, '0 kB').split()[0])
        rss, swap = kb('VmRSS'), kb('VmSwap')
        gpu = gpu_kb(pid)
        if rss + swap + gpu < min_kb:
            continue
        out[pid] = {'name': process_name(pid, cmdline), 'rss': rss, 'anon': kb('RssAnon'),
                    'shmem': kb('RssShmem'), 'swap': swap, 'gpu': gpu}
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('--period', type=float, default=10.0, help='seconds between samples')
    parser.add_argument('--top', type=int, default=10, help='processes shown')
    parser.add_argument('--warn-available-gb', type=float, default=4.0)
    parser.add_argument('--warn-growth-gb', type=float, default=1.0,
                        help='warn when anything grows this much in 10 minutes')
    args = parser.parse_args()
    sys.stdout.reconfigure(line_buffering=True)   # show each line now, also when piped to a file

    os.makedirs(os.path.expanduser('~/rtabmap_debug'), exist_ok=True)
    csv_path = os.path.expanduser(f'~/rtabmap_debug/memory_{time.strftime("%Y%m%d_%H%M%S")}.csv')
    csv = open(csv_path, 'w')
    csv.write('time,pid,name,resident_kb,anon_kb,shmem_kb,swap_kb,gpu_kb\n')
    print(f'memory_monitor: every {args.period:.0f} s, log in {csv_path}  (Ctrl-C to stop)')

    window = int(600 / args.period) + 1             # samples in 10 minutes
    history = deque(maxlen=window)                  # (time, system dict, {pid: total kB})
    start = None
    try:
        while True:
            now = time.time()
            m = meminfo()
            procs = processes(min_kb=20 * 1024)
            used = m['MemTotal'] - m['MemAvailable']
            swap_used = m['SwapTotal'] - m['SwapFree']
            gpu_sum = sum(p['gpu'] for p in procs.values())
            mapped_shmem = sum(p['shmem'] for p in procs.values())
            unattributed = max(0, m['Shmem'] - mapped_shmem - gpu_sum)
            system = {'used': used, 'available': m['MemAvailable'], 'shmem': m['Shmem'],
                      'unattributed': unattributed, 'swap': swap_used}
            totals = {pid: p['rss'] + p['swap'] for pid, p in procs.items()}
            history.append((now, system, totals))
            start = start or (now, system)

            stamp = time.strftime('%H:%M:%S')
            csv.write(f'{stamp},0,SYSTEM used/available/shmem/swap/unattributed,{used},'
                      f'{m["MemAvailable"]},{m["Shmem"]},{swap_used},{unattributed}\n')
            for pid, p in procs.items():
                csv.write(f'{stamp},{pid},{p["name"].replace(",", " ")},{p["rss"]},{p["anon"]},'
                          f'{p["shmem"]},{p["swap"]},{p["gpu"]}\n')
            csv.flush()

            old_t, old_system, old_totals = history[0]
            span = (now - old_t) / 60.0

            def grew(key):
                return (system[key] - old_system[key]) / GB

            trend = (f' | last {span:.0f} min: shared {grew("shmem"):+.1f}, '
                     f'used {grew("used"):+.1f} GB' if span >= 1 else '')
            print(f'\n=== {stamp}  RAM used {used / GB:5.1f} GB, available '
                  f'{m["MemAvailable"] / GB:5.1f} GB | shared {m["Shmem"] / GB:5.1f} GB '
                  f'(unattributed {unattributed / GB:4.1f}) | swap {swap_used / GB:4.1f}/'
                  f'{m["SwapTotal"] / GB:.0f} GB{trend}')
            print(f'  {"process":32s} {"resident":>8s} {"anon":>7s} {"shared":>7s} {"swap":>7s} '
                  f'{"gpu":>7s} {"10min":>7s}')
            ranked = sorted(procs.items(), key=lambda kv: -(kv[1]['rss'] + kv[1]['swap']
                                                            + kv[1]['gpu']))
            for pid, p in ranked[:args.top]:
                delta = (totals[pid] - old_totals[pid]) / GB if pid in old_totals else 0.0
                print(f'  {p["name"][:32]:32s} {p["rss"] / GB:8.2f} {p["anon"] / GB:7.2f} '
                      f'{p["shmem"] / GB:7.2f} {p["swap"] / GB:7.2f} {p["gpu"] / GB:7.2f} '
                      f'{delta:+7.2f}')

            if m['MemAvailable'] / GB < args.warn_available_gb:
                print(f'  !! only {m["MemAvailable"] / GB:.1f} GB available: the out-of-memory '
                      'killer will pick the biggest process (rtabmap) soon')
            if span >= 1 and grew('unattributed') > args.warn_growth_gb:
                print(f'  !! unattributed shared memory grew {grew("unattributed"):+.1f} GB in '
                      f'{span:.0f} min: held through the kernel, not by any process directly')
            for pid, total in totals.items():
                if pid in old_totals and (total - old_totals[pid]) / GB > args.warn_growth_gb:
                    print(f'  !! {procs[pid]["name"]} (pid {pid}) grew '
                          f'{(total - old_totals[pid]) / GB:+.1f} GB in {span:.0f} min')
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        csv.close()
        if start:
            print(f'\nmemory_monitor: log in {csv_path}')


if __name__ == '__main__':
    main()
