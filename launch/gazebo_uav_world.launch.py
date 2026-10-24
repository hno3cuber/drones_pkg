"""一键起 Gazebo 世界 + 四架 PX4 x500。

架构（与 launch/gazebo_uav_config.py 配套）：
    launch 自己先起 gz server 跑 obstacles_world.sdf，等 world 就绪后，
    再拉起四架 PX4。四架全部以 PX4_GZ_STANDALONE=1 连到这个世界，
    没有任何一架负责起 world，因此：
      - world 生命周期独立，单架 PX4 崩溃不影响其他架与仿真世界
      - 之后想单独调试某一架，只跑这一架 PX4 即可，世界照旧在

    world 文件由 scripts/gen_world.py 从 params/obstacles.yaml 生成，
    world 名默认 obstacles_world，通过 GZ_SIM_RESOURCE_PATH 挂进来，
    不往 PX4 仓库里塞任何文件。

前置：
    python3 scripts/gen_world.py        # 生成 worlds/obstacles_world.sdf
    colcon build --packages-select drones_pkg   # 让 install/ 里有该 world

用法:
    ros2 launch drones_pkg gazebo_uav_world.launch.py
    ros2 launch drones_pkg gazebo_uav_world.launch.py headless:=true
    ros2 launch drones_pkg gazebo_uav_world.launch.py gui:=false
"""

import os
import subprocess
import sys
import time

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# 同目录的仿真侧身份配置
sys.path.insert(0, os.path.dirname(__file__))
from gazebo_uav_config import (  # noqa: E402
    AIRFRAME, SIM_MODEL, UAV_ORDER, UAV_POSES, WORLD_NAME,
)


def _find_px4_root(px4_root_arg):
    """定位 PX4 源码根目录（编译产物 build/px4_sitl_default 在此之下）。"""
    if px4_root_arg:
        return os.path.expanduser(px4_root_arg)
    candidates = [
        os.environ.get('PX4_ROOT'),
        os.path.join(os.path.expanduser('~'), 'PX4-Autopilot'),
        '/home/hno3/PX4-Autopilot',
    ]
    for path in candidates:
        if path and os.path.isfile(
                os.path.join(path, 'build', 'px4_sitl_default', 'bin', 'px4')):
            return path
    return None


def _wait_world_ready(world_name, timeout=60.0):
    """轮询 gz 话题，直到 /world/<name>/clock 出现（即 world 已就绪）。

    保留给外部脚本/调试使用：launch 内部不用它（见 _launch_setup 中的说明），
    但想在启动脚本里手动做同步时可以直接调用。
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            out = subprocess.run(
                ['gz', 'topic', '-l'],
                capture_output=True, text=True, timeout=5.0)
            if f'/world/{world_name}/clock' in out.stdout:
                return True
        except (subprocess.SubprocessError, OSError):
            pass
        time.sleep(1.0)
    return False


def _launch_setup(context, *args, **kwargs):
    px4_root = _find_px4_root(LaunchConfiguration('px4_root').perform(context))
    if not px4_root:
        raise RuntimeError(
            "找不到 PX4（需要 build/px4_sitl_default/bin/px4）。"
            "请用 px4_root:=/path/to/PX4-Autopilot 指定。")

    world_name = LaunchConfiguration('world_name').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in (
        'true', '1', 'yes')

    # ---- world 文件定位：install/ 优先，回退到源码树 worlds/ ----
    pkg_share = get_package_share_directory('drones_pkg')
    world_file = os.path.join(pkg_share, 'worlds', f'{world_name}.sdf')
    if not os.path.isfile(world_file):
        src_world = os.path.join(
            os.path.dirname(pkg_share), '..', '..', '..', '..',
            'src', 'drones_pkg', 'worlds', f'{world_name}.sdf')
        src_world = os.path.abspath(src_world)
        if os.path.isfile(src_world):
            world_file = src_world

    if not os.path.isfile(world_file):
        raise RuntimeError(
            f"找不到 world 文件 {world_name}.sdf。先跑：\n"
            f"    python3 scripts/gen_world.py\n"
            f"再 colcon build --packages-select drones_pkg")

    px4_bin = os.path.join(px4_root, 'build', 'px4_sitl_default', 'bin', 'px4')
    px4_run_dir = os.path.join(px4_root, 'build', 'px4_sitl_default')

    # gz server 需要的资源路径：PX4 模型/世界 + 我们的 world 目录
    res_paths = [
        os.path.join(px4_root, 'Tools', 'simulation', 'gz', 'models'),
        os.path.join(px4_root, 'Tools', 'simulation', 'gz', 'worlds'),
        os.path.dirname(world_file),
    ]
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = ':'.join(
        [p for p in res_paths if os.path.isdir(p)] +
        ([existing] if existing else []))

    plugin_path = os.path.join(px4_root, 'build', 'px4_sitl_default',
                               'src', 'modules', 'simulation', 'gz_plugins')
    server_config = os.path.join(px4_root, 'src', 'modules', 'simulation',
                                 'gz_bridge', 'server.config')

    base_env = {
        **os.environ,
        'GZ_SIM_RESOURCE_PATH': gz_resource_path,
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join(
            filter(None, [plugin_path,
                          os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')])),
        'GZ_SIM_SERVER_CONFIG_PATH': server_config,
    }

    actions = []

    # ---- 1) 起 gz server（headless 时不起 GUI） ----
    gz_cmd = ['gz', 'sim', '-r', '-s', '-v', '2', world_file]
    actions.append(LogInfo(msg=f"[gz] world={world_file}"))
    actions.append(ExecuteProcess(
        cmd=gz_cmd, name='gz_server', output='screen', env=base_env))

    if not headless:
        actions.append(ExecuteProcess(
            cmd=['gz', 'sim', '-g', '-v', '1'],
            name='gz_gui', output='screen', env=base_env))

    # ---- 2) 等 world 就绪 ----
    # 注意：这里不能阻塞。ExecuteProcess 只是注册进程，真正 spawn 发生在全部
    # action 处理完之后；若在此处轮询，gz 尚未启动，必然等到超时。
    # PX4 自身在 px4-rc.gzsim 里会重试 30 次等待 /world/*/clock，因此把等待
    # 交给 PX4，launch 只管按序注册进程即可。
    actions.append(LogInfo(
        msg=f"[gz] world '{world_name}' 启动中，PX4 将自行等待其就绪"))

    # ---- 3) 依次拉起四架 PX4 ----
    for name in UAV_ORDER:
        instance, (px, py) = UAV_POSES[name]
        env = {
            **base_env,
            'PX4_SYS_AUTOSTART': AIRFRAME,
            'PX4_SIM_MODEL': SIM_MODEL,
            'PX4_GZ_WORLD': world_name,
            'PX4_GZ_MODEL_POSE': f'{px},{py},0,0,0,0',
            # 世界已由本 launch 起好，全部实例都只负责 spawn 自己
            'PX4_GZ_STANDALONE': '1',
            # 关掉 PX4 自带的相机跟随，避免四架互相抢镜头
            'PX4_GZ_NO_FOLLOW': '1',
        }
        actions.append(LogInfo(
            msg=f"[px4] {name}: instance={instance} pose=({px}, {py})"))
        actions.append(ExecuteProcess(
            cmd=[px4_bin, '-i', str(instance)],
            name=f'px4_{name}',
            output='screen',
            env=env,
            cwd=px4_run_dir,
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name', default_value=WORLD_NAME,
            description='world 名（对应 worlds/<name>.sdf）'),
        DeclareLaunchArgument(
            'px4_root', default_value='',
            description='PX4-Autopilot 根目录（留空则自动查找）'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true 时只起 gz server，不起 GUI'),
        OpaqueFunction(function=_launch_setup),
    ])
