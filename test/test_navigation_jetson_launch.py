import importlib.util
import os
import shlex

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    'navigation_jetson', os.path.join(HERE, '..', 'launch', 'navigation_jetson.launch.py'))
navigation_jetson = importlib.util.module_from_spec(spec)
spec.loader.exec_module(navigation_jetson)


def test_jetson_command_keeps_values_with_spaces_whole():
    cmd = navigation_jetson.jetson_command(
        'jetson1@192.168.1.101', 'tcp/192.168.1.100:7447',
        {'mode': 'localization', 'initial_pose': '1 2 0 0 0 1.57',
         'database_path': '/home/hello-robot/stretch_maps/a b.db'})
    assert cmd[:2] == ['ssh', '-tt']
    assert cmd[-2] == 'jetson1@192.168.1.101'
    remote = shlex.split(cmd[-1])           # what the Jetson's shell does with it
    assert remote == ['stretch_gpu/run.sh', 'nuc_router:=tcp/192.168.1.100:7447',
                      'mode:=localization', 'initial_pose:=1 2 0 0 0 1.57',
                      'database_path:=/home/hello-robot/stretch_maps/a b.db']


def test_jetson_command_without_arguments_uses_jetson_defaults():
    remote = shlex.split(navigation_jetson.jetson_command('j', 'tcp/x:7447', {})[-1])
    assert remote == ['stretch_gpu/run.sh', 'nuc_router:=tcp/x:7447']


def test_jetson_command_without_router_leaves_it_to_run_sh():
    remote = shlex.split(navigation_jetson.jetson_command('j', '', {'mode': 'slam'})[-1])
    assert remote == ['stretch_gpu/run.sh', 'mode:=slam']
