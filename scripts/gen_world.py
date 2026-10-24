#!/usr/bin/env python3
"""把 params/obstacles.yaml 编译成一个带真实碰撞体的 Gazebo world。

为什么要有这个脚本：
    PX4 起 world 时固定去 $PX4_GZ_WORLDS/<world>.sdf 找文件。往 PX4 仓库里
    塞自定义 world 会污染 upstream（git 变脏、升级冲突）。这里改为把生成的
    world 放进 drones_pkg 自己的目录，启动时通过 GZ_SIM_RESOURCE_PATH 挂进去。

    world 以 PX4 官方的 default.sdf 为底座（同样的物理步长、重力、光照、
    spherical_coordinates），再按 yaml 追加每根圆柱。底座文件路径可由
    --base 覆盖，找不到底座时内置一份等价的最小 world 兜底。

用法:
    python3 scripts/gen_world.py                     # 默认读 params/obstacles.yaml
    python3 scripts/gen_world.py --check             # 只校验 yaml，不写文件
"""

import argparse
import os
import sys

try:
    import yaml
except ImportError:
    sys.exit("需要 pyyaml：pip3 install pyyaml  （ROS 2 环境通常自带 python3-yaml）")

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)
DEFAULT_YAML = os.path.join(PKG_ROOT, 'params', 'obstacles.yaml')

def find_base_world():
    """按优先级找 PX4 官方 default.sdf 作为底座。

    不直接写死 ~/PX4-Autopilot：home 可能被环境重定向（容器、沙箱、
    不同的机器），而 PX4 也可能装在别处。依次尝试：
      1) 环境变量 PX4_GZ_WORLDS（PX4 编译后 gz_env.sh 会导出）
      2) 常见 PX4 源码位置下的 default.sdf
    全部落空则返回 None，由调用方退回内置兜底 world。
    """
    candidates = []

    worlds_dir = os.environ.get('PX4_GZ_WORLDS')
    if worlds_dir:
        candidates.append(os.path.join(worlds_dir, 'default.sdf'))

    for home in (os.path.expanduser('~'), '/home/hno3', '/root'):
        candidates.append(os.path.join(
            home, 'PX4-Autopilot', 'Tools', 'simulation', 'gz', 'worlds',
            'default.sdf'))

    for path in candidates:
        if os.path.isfile(path):
            return path
    return None

# 底座缺失时用的等价最小 world（字段与 default.sdf 对齐）
FALLBACK_BASE = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="{world_name}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1 1</size>
            </plane>
          </geometry>
          <surface>
            <friction><ode/></friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>500 500</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
    <light name="sunUTC" type="directional">
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range>
        <linear>0</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""


def load_obstacles(path):
    """读 yaml，做基本校验，返回障碍列表。"""
    if not os.path.isfile(path):
        sys.exit(f"找不到障碍物文件：{path}")

    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict) or 'obstacles' not in data:
        sys.exit(f"{path} 缺少顶层 'obstacles' 键")

    items = data['obstacles']
    if not items:
        sys.exit(f"{path} 的 obstacles 列表为空")

    out = []
    for i, item in enumerate(items):
        try:
            out.append({
                'name': f'obstacle_{i}',
                'x': float(item['x']),
                'y': float(item['y']),
                'radius': float(item['radius']),
                'height': float(item['height']),
            })
        except (TypeError, KeyError) as e:
            sys.exit(f"第 {i} 个障碍物字段缺失或非法（需要 x/y/radius/height）：{e}")
        except ValueError as e:
            sys.exit(f"第 {i} 个障碍物数值不是数字：{e}")
    return out


def cylinder_sdf(ob):
    """生成一根圆柱：底面贴地，占据 z ∈ [0, height]，与 RViz 侧约定一致。"""
    z_center = ob['height'] / 2.0
    return f"""    <model name="{ob['name']}">
      <static>true</static>
      <pose>{ob['x']:.4f} {ob['y']:.4f} {z_center:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{ob['radius']:.4f}</radius>
              <length>{ob['height']:.4f}</length>
            </cylinder>
          </geometry>
          <surface>
            <friction><ode/></friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{ob['radius']:.4f}</radius>
              <length>{ob['height']:.4f}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.85 0.20 0.20 1</ambient>
            <diffuse>0.85 0.20 0.20 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def build_world(obstacles, world_name, base_path):
    """在底座 world 的 </world> 前插入障碍物。"""
    if os.path.isfile(base_path):
        with open(base_path, 'r', encoding='utf-8') as f:
            base = f.read()
        # 底座 world 名统一改成目标名
        import re
        base = re.sub(r'<world\s+name="[^"]*"', f'<world name="{world_name}"',
                      base, count=1)
    else:
        base = FALLBACK_BASE.format(world_name=world_name)

    if '</world>' not in base:
        sys.exit("底座 world 里找不到 </world>，无法插入障碍物")

    blocks = ''.join(cylinder_sdf(ob) for ob in obstacles)
    return base.replace('</world>', blocks + '  </world>', 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--yaml', default=DEFAULT_YAML,
                    help=f'障碍物 yaml 路径（默认 {DEFAULT_YAML}）')
    ap.add_argument('--out', default=None,
                    help='输出 world 路径（默认 <pkg>/worlds/<world_name>.sdf）')
    ap.add_argument('--base', default=None,
                    help='底座 world 路径（默认自动查找 PX4 官方 default.sdf，'
                         '找不到则用内置兜底）')
    ap.add_argument('--world-name', default='obstacles_world',
                    help='world 名（默认 obstacles_world）')
    ap.add_argument('--check', action='store_true',
                    help='只校验 yaml 并打印摘要，不写文件')
    args = ap.parse_args()

    obstacles = load_obstacles(args.yaml)

    if args.check:
        print(f"[check] {args.yaml} 合法，共 {len(obstacles)} 个障碍：")
        for ob in obstacles:
            print(f"  - {ob['name']}: ({ob['x']}, {ob['y']}) "
                  f"r={ob['radius']} h={ob['height']}")
        return

    base = args.base or find_base_world()
    world = build_world(obstacles, args.world_name, base)

    out = args.out or os.path.join(PKG_ROOT, 'worlds',
                                   f'{args.world_name}.sdf')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w', encoding='utf-8') as f:
        f.write(world)

    print(f"[gen] 写入 {out}")
    print(f"      底座：{base if base else '内置兜底（未找到 PX4 default.sdf）'}")
    for ob in obstacles:
        print(f"      {ob['name']}: ({ob['x']}, {ob['y']}) "
              f"r={ob['radius']} h={ob['height']}")


if __name__ == '__main__':
    main()
