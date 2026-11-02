"""
Set up and manage the Jetson half of navigation_jetson.launch.py.

    ros2 run stretch4_rtabmap jetson check              # network, clocks, image, build, disk
    ros2 run stretch4_rtabmap jetson deploy             # copy sources over and build
    ros2 run stretch4_rtabmap jetson push-map ~/stretch_maps/<my_map>.db
    ros2 run stretch4_rtabmap jetson pull-map <my_map>.db
    ros2 run stretch4_rtabmap jetson stop               # stop GLIM + RTAB-Map (saves the db)
    ros2 run stretch4_rtabmap jetson shell              # bash in the running container
    ros2 run stretch4_rtabmap jetson memory             # memory_monitor in the container

STRETCH_JETSON: the Jetson's user@address (default jetson1@192.168.1.101). Files are in the
Jetson's ~/stretch_gpu; its home/ is mounted in the container as this computer's home directory.
"""

import argparse
import os
import shlex
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
import yaml

JETSON = os.environ.get('STRETCH_JETSON', 'jetson1@192.168.1.101')
IMAGE = os.environ.get('STRETCH_GPU_IMAGE', 'stretch4_rtabmap_gpu:jazzy')
CONTAINER = 'stretch_gpu'
HOME = os.path.expanduser('~')
# The workspace this package is installed in: <workspace>/install/stretch4_rtabmap.
WORKSPACE = os.path.dirname(os.path.dirname(get_package_prefix('stretch4_rtabmap')))
WS = os.path.join(WORKSPACE, 'src')
# The same path on the Jetson, under the home directory mounted in the container.
REMOTE_WS = os.path.join('stretch_gpu/home', os.path.relpath(WORKSPACE, HOME))
REMOTE_SRC = f'{REMOTE_WS}/src'
REMOTE_MAPS = 'stretch_gpu/home/stretch_maps'
SCRIPTS = f'{REMOTE_SRC}/stretch4_rtabmap/jetson'

MIN_FREE_GB = 60   # the image (~25 GB), the build and the database
REPOS_FILE = os.path.join(WS, 'stretch4_rtabmap', 'stretch4_rtabmap.repos')
# Repositories from stretch4_rtabmap.repos that the Jetson builds.
JETSON_REPOS = ['gtsam', 'gtsam_points', 'glim', 'glim_ros2', 'rtabmap', 'rtabmap_ros']


# rtabmap_ros packages the Jetson does not run (GUIs, RViz, Nav2 plugins, odometry, demos).
RTABMAP_ROS_SKIP = ['rtabmap_viz', 'rtabmap_rviz_plugins', 'rtabmap_costmap_plugins',
                    'rtabmap_demos', 'rtabmap_examples', 'rtabmap_odom', 'rtabmap_python',
                    'rtabmap_ros', 'docker']


def find_sources():
    """
    Return {source directory here: directory name on the Jetson}.

    JETSON_REPOS and stretch4_rtabmap, looked up in <workspace>/src, then <workspace>/src/src.
    """
    if not os.path.isfile(REPOS_FILE):
        sys.exit(f'jetson: {REPOS_FILE} is missing; clone stretch4_rtabmap into {WS}')
    with open(REPOS_FILE) as f:
        listed = yaml.safe_load(f)['repositories']
    missing = [name for name in JETSON_REPOS if name not in listed]
    if missing:
        sys.exit(f'jetson: {REPOS_FILE} does not list {", ".join(missing)}')
    paths = JETSON_REPOS + ['stretch4_rtabmap']
    sources = {}
    for path in paths:
        found = [d for d in (path, os.path.join('src', path))
                 if os.path.isdir(os.path.join(WS, d))]
        if not found:
            sys.exit(f'jetson: {path} is not in {WS}; get the sources with\n'
                     f'  vcs import {WS} < {REPOS_FILE}')
        sources[found[0]] = os.path.basename(path)
    return sources


def ssh(command, check=True, tty=False, capture=False):
    args = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5'] + (['-t'] if tty else [])
    result = subprocess.run(args + [JETSON, command], text=True,
                            capture_output=capture, check=False)
    if check and result.returncode != 0:
        sys.exit(f'jetson: failed on {JETSON}: {command}\n{result.stderr or ""}')
    return result


def container_running():
    out = ssh(f'docker ps -q --filter name=^/{CONTAINER}$', capture=True).stdout
    return bool(out.strip())


def git_version(path):
    def git(*args):
        return subprocess.run(['git', '-C', path] + list(args), capture_output=True,
                              text=True).stdout.strip()
    rev = git('rev-parse', '--short', 'HEAD') or 'no-git'
    dirty = git('status', '--porcelain', '--untracked-files=no')
    return f'{rev}{" +uncommitted" if dirty else ""}'


def versions_text():
    return ''.join(f'{name} {git_version(os.path.join(WS, src))}\n'
                   for src, name in find_sources().items())


def nuc_address():
    """Return this computer's address as the Jetson sees it (from the ssh connection)."""
    out = ssh('echo $SSH_CONNECTION', capture=True).stdout.split()
    return out[0]


def jetson_problems():
    """Return what the Jetson lacks to run the container, as messages (empty: nothing)."""
    problems = []
    release = ssh('head -1 /etc/nv_tegra_release', check=False, capture=True).stdout
    if '# R36' not in release:
        problems.append(f'JetPack 6 (L4T R36) needed; /etc/nv_tegra_release: '
                        f'{release.strip() or "missing"}')
    info = ssh("docker info --format '{{json .Runtimes}}'", check=False, capture=True)
    if info.returncode != 0:
        problems.append('docker does not run for this user: install docker and '
                        '`sudo usermod -aG docker $USER` on the Jetson, then log in again')
    elif 'nvidia' not in info.stdout:
        problems.append('docker has no nvidia runtime: `sudo apt install nvidia-container` on '
                        'the Jetson')
    return problems


def deploy(args):
    if os.path.relpath(WORKSPACE, HOME).startswith('..'):
        sys.exit(f'jetson: the workspace {WORKSPACE} must be under {HOME}: the container mounts '
                 'only the home directory.')
    problems = jetson_problems()
    if problems:
        sys.exit('jetson: the Jetson is not ready:\n  ' + '\n  '.join(problems))
    nuc = nuc_address()
    ssh(f'mkdir -p {REMOTE_SRC} {REMOTE_MAPS} stretch_gpu/home/.ros')
    for src, name in find_sources().items():
        excludes = ['.git', '__pycache__', '*.pyc', '/build/', '/install/', '/log/']
        if name == 'rtabmap_ros':
            excludes += [f'/{p}/' for p in RTABMAP_ROS_SKIP]
        print(f'jetson: copying {name}')
        subprocess.run(['rsync', '-a', '--delete', '--info=stats0']
                       + [f'--exclude={e}' for e in excludes]
                       + [os.path.join(WS, src) + '/', f'{JETSON}:{REMOTE_SRC}/{name}/'],
                       check=True)
    env = (f'IMAGE={IMAGE}\nCONTAINER_HOME={HOME}\nWORKSPACE={WORKSPACE}\n'
           f'NUC_ROUTER=tcp/{nuc}:7447\n')
    ssh(f'printf %s {shlex.quote(env)} > stretch_gpu/env && '
        f'printf %s {shlex.quote(versions_text())} > stretch_gpu/VERSIONS && '
        f'ln -sfn {os.path.relpath(SCRIPTS, "stretch_gpu")} stretch_gpu/jetson && '
        f'ln -sfn jetson/run.sh stretch_gpu/run.sh')

    has_image = ssh(f'docker image inspect {IMAGE} >/dev/null 2>&1', check=False).returncode == 0
    if args.image or not has_image:
        print(f'jetson: building the {IMAGE} image (base image download + apt packages, '
              '~30 min the first time)')
        ssh(f'docker build -t {IMAGE} {SCRIPTS}')
    if args.no_build:
        return
    print('jetson: building the workspace (first time: 1-2 h; afterwards only what changed)')
    ssh(f'JOBS={args.jobs} {SCRIPTS}/build.sh ' + ' '.join(map(shlex.quote, args.colcon)))
    print('jetson: deployed. Versions:\n' + versions_text())


def push_map(args):
    path = os.path.expanduser(args.db)
    if not os.path.isfile(path):
        sys.exit(f'jetson: no such file {path}')
    if container_running():
        sys.exit('jetson: RTAB-Map is running on the Jetson; stop it first (jetson stop).')
    subprocess.run(['rsync', '-a', '--info=progress2', path, f'{JETSON}:{REMOTE_MAPS}/'],
                   check=True)
    print(f'jetson: use database_path:={os.path.expanduser("~/stretch_maps")}/'
          f'{os.path.basename(path)}')


def pull_map(args):
    name = os.path.basename(args.db)
    dest = os.path.expanduser(args.dest or os.path.join('~/stretch_maps', name))
    if os.path.exists(dest) and not args.overwrite:
        sys.exit(f'jetson: {dest} exists; pass --overwrite or --dest <path>.')
    if container_running():
        sys.exit('jetson: RTAB-Map is running on the Jetson; stop it first (jetson stop).')
    subprocess.run(['rsync', '-a', '--info=progress2', f'{JETSON}:{REMOTE_MAPS}/{name}', dest],
                   check=True)
    print(f'jetson: copied to {dest}')


def clock_offset_ms(samples=40):
    """Jetson clock minus this one, from the round trips with the smallest delay."""
    proc = subprocess.Popen(
        ['ssh', '-o', 'BatchMode=yes', JETSON,
         'python3 -u -c "import sys, time\nfor _ in sys.stdin: print(repr(time.time()))"'],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    results = []
    for _ in range(samples):
        t0 = time.time()
        proc.stdin.write('x\n')
        proc.stdin.flush()
        remote = float(proc.stdout.readline())
        t1 = time.time()
        results.append((t1 - t0, remote - (t0 + t1) / 2))
        time.sleep(0.02)
    proc.stdin.close()
    proc.wait()
    best = sorted(results)[:5]
    return sorted(o for _, o in best)[2] * 1e3, best[0][0] * 1e3


def check(args):
    problems = []

    def report(ok, text):
        print(f'  [{"ok" if ok else "!!"}] {text}')
        if not ok:
            problems.append(text)

    print(f'jetson: checking {JETSON}')
    if ssh('true', check=False, capture=True).returncode != 0:
        sys.exit(f'  [!!] cannot ssh to {JETSON} without a password '
                 '(set STRETCH_JETSON=user@address, then ssh-copy-id it)')
    for problem in jetson_problems():
        report(False, problem)
    offset, rtt = clock_offset_ms()
    report(abs(offset) < 50.0, f'clock offset {offset:+.1f} ms (round trip {rtt:.2f} ms); over '
           '50 ms: check that both computers sync their clocks (timedatectl)')
    speed = ssh(f'cat /sys/class/net/$(ip -o route get {nuc_address()} | '
                "sed 's/.* dev \\([^ ]*\\).*/\\1/')/speed", check=False, capture=True)
    mbit = speed.stdout.strip()
    report(mbit == '1000' or mbit.isdigit() and int(mbit) > 1000,
           f'Jetson wired link {mbit or "?"} Mb/s (needs 1000: images + lidar ~40 MB/s)')
    router = subprocess.run(['pgrep', '-f', 'rmw_zenohd'], capture_output=True).returncode == 0
    report(router, 'Zenoh router running on this computer')
    image = ssh(f'docker image inspect {IMAGE} >/dev/null 2>&1', check=False).returncode == 0
    report(image, f'image {IMAGE} on the Jetson (jetson deploy builds it)')
    # colcon's marker that stretch4_rtabmap is installed.
    built = ssh(f'test -f {REMOTE_WS}/install/stretch4_rtabmap/share/'
                'colcon-core/packages/stretch4_rtabmap', check=False).returncode == 0
    report(built, 'Jetson workspace built (jetson deploy)')
    remote_versions = ssh('cat stretch_gpu/VERSIONS 2>/dev/null', check=False,
                          capture=True).stdout
    same = remote_versions == versions_text()
    report(same, 'Jetson sources match this computer' + ('' if same else
           ' -- differ, run jetson deploy:\n' + remote_versions))
    disk = ssh('df -BG --output=avail $HOME | tail -1', capture=True).stdout.strip()
    need = 15 if built else MIN_FREE_GB   # once built, only the database grows
    report(int(disk.rstrip('G') or 0) > need,
           f'Jetson free disk {disk} (needs {need}G: ' +
           ('database + growth)' if built else 'image, build, database)'))
    mem = ssh("free -g | awk '/Mem:/{print $7}'", capture=True).stdout.strip()
    print(f'  [..] Jetson available memory {mem} GB')
    print(f'  [..] GLIM + RTAB-Map container: {"running" if container_running() else "stopped"}')
    print('jetson: all good' if not problems else f'jetson: {len(problems)} problem(s)')
    return 1 if problems else 0


def stop(args):
    if not container_running():
        print('jetson: not running')
        return
    print('jetson: stopping (RTAB-Map saves the database; up to 150 s)')
    ssh(f'docker stop --signal SIGINT --time 150 {CONTAINER}')


def in_container(command):
    if not container_running():
        sys.exit('jetson: the container is not running (start navigation_jetson.launch.py first).')
    ssh(f'docker exec -it {CONTAINER} bash '
        f'{WS}/stretch4_rtabmap/jetson/entry.sh {command}',
        tty=True, check=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('check', help='network, clocks, image, build, disk')
    p = sub.add_parser('deploy', help='copy sources to the Jetson and build')
    p.add_argument('--image', action='store_true', help='rebuild the Docker image too')
    p.add_argument('--no-build', action='store_true', help='copy only')
    p.add_argument('--jobs', type=int, default=4, help='parallel compile jobs on the Jetson')
    p.add_argument('colcon', nargs='*', help='extra colcon arguments (after --)')
    p = sub.add_parser('push-map', help='copy a database to the Jetson')
    p.add_argument('db')
    p = sub.add_parser('pull-map', help='copy a database back from the Jetson')
    p.add_argument('db', help='file name in the Jetson stretch_maps')
    p.add_argument('--dest', help='default ~/stretch_maps/<name>')
    p.add_argument('--overwrite', action='store_true')
    sub.add_parser('stop', help='stop GLIM and RTAB-Map on the Jetson')
    sub.add_parser('shell', help='bash in the running container')
    sub.add_parser('memory', help='memory_monitor in the running container')
    args = parser.parse_args()

    if args.command == 'check':
        sys.exit(check(args))
    if args.command == 'shell':
        return in_container('bash')
    if args.command == 'memory':
        return in_container('ros2 run stretch4_rtabmap memory_monitor')
    {'deploy': deploy, 'push-map': push_map, 'pull-map': pull_map, 'stop': stop}[args.command](
        args)


if __name__ == '__main__':
    main()
