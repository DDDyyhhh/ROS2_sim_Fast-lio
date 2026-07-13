#!/usr/bin/env python3
"""
random_obstacles.py — 在 grassland_50x50.world 的草坪上随机生成障碍物

用法:
  python3 random_obstacles.py                     # 生成 5 个随机障碍物并打印 SDF
  python3 random_obstacles.py --count 10           # 指定障碍物数量
  python3 random_obstacles.py --seed 42            # 固定随机种子（可复现）
  python3 random_obstacles.py --append             # 追加到世界文件末尾
  python3 random_obstacles.py --count 8 --append   # 生成 8 个并追加

输出: 打印到 stdout 的 SDF <model> 片段，可直接插入 .world 文件
"""

import argparse
import random
import math

# 草坪边界（留出 2m 边距避免障碍物怼墙）
BOUNDARY = 23  # ±25m 内，留 2m 边距

# 障碍物模板
OBSTACLE_TEMPLATES = [
    # 树（圆柱树干 + 球树冠）
    {
        "name": "random_tree",
        "generate": lambda i, x, y, theta: f"""    <model name="random_tree_{i}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 {theta}</pose>
      <link name="trunk">
        <collision name="collision">
          <geometry><cylinder><radius>{random.uniform(0.1, 0.25):.3f}</radius><length>{random.uniform(1.0, 2.0):.2f}</length></cylinder></geometry>
        </collision>
        <visual name="trunk_vis">
          <geometry><cylinder><radius>{random.uniform(0.1, 0.25):.3f}</radius><length>{random.uniform(1.0, 2.0):.2f}</length></cylinder></geometry>
          <material><ambient>0.4 0.2 0.1 1</ambient><diffuse>0.5 0.25 0.1 1</diffuse></material>
        </visual>
        <visual name="canopy">
          <pose>0 0 {random.uniform(1.0, 2.0):.2f} 0 0 0</pose>
          <geometry><sphere><radius>{random.uniform(0.8, 2.0):.2f}</radius></sphere></geometry>
          <material><ambient>0.15 0.5 0.1 1</ambient><diffuse>0.2 0.55 0.12 1</diffuse></material>
        </visual>
      </link>
    </model>""",
    },
    # 花坛（矩形矮盒）
    {
        "name": "random_flowerbed",
        "generate": lambda i, x, y, theta: f"""    <model name="random_flowerbed_{i}">
      <static>true</static>
      <pose>{x} {y} 0.2 0 0 {theta}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{random.uniform(1.0, 3.0):.2f} {random.uniform(0.8, 2.0):.2f} 0.4</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{random.uniform(1.0, 3.0):.2f} {random.uniform(0.8, 2.0):.2f} 0.4</size></box></geometry>
          <material>
            <ambient>{random.uniform(0.3, 0.9):.2f} {random.uniform(0.1, 0.5):.2f} {random.uniform(0.1, 0.6):.2f} 1</ambient>
            <diffuse>{random.uniform(0.4, 1.0):.2f} {random.uniform(0.15, 0.6):.2f} {random.uniform(0.15, 0.7):.2f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>""",
    },
    # 石头（球体）
    {
        "name": "random_rock",
        "generate": lambda i, x, y, theta: f"""    <model name="random_rock_{i}">
      <static>true</static>
      <pose>{x} {y} {random.uniform(0.1, 0.25):.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><sphere><radius>{random.uniform(0.12, 0.35):.3f}</radius></sphere></geometry>
        </collision>
        <visual name="visual">
          <geometry><sphere><radius>{random.uniform(0.12, 0.35):.3f}</radius></sphere></geometry>
          <material>
            <ambient>{random.uniform(0.3, 0.5):.2f} {random.uniform(0.3, 0.5):.2f} {random.uniform(0.3, 0.5):.2f} 1</ambient>
            <diffuse>{random.uniform(0.35, 0.6):.2f} {random.uniform(0.35, 0.6):.2f} {random.uniform(0.35, 0.6):.2f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>""",
    },
    # 喷灌头（小圆柱）
    {
        "name": "random_sprinkler",
        "generate": lambda i, x, y, theta: f"""    <model name="random_sprinkler_{i}">
      <static>true</static>
      <pose>{x} {y} 0.08 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>0.05</radius><length>0.15</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.05</radius><length>0.15</length></cylinder></geometry>
          <material>
            <ambient>0.2 0.6 0.8 1</ambient>
            <diffuse>0.25 0.65 0.9 1</diffuse>
          </material>
        </visual>
      </link>
    </model>""",
    },
]


def generate(count=5, seed=None, append=False, world_path=None):
    if seed is not None:
        random.seed(seed)

    used_positions = []  # 避免障碍物重叠

    def position_available(x, y, min_dist=2.0):
        """检查新位置是否离已有障碍物足够远"""
        for px, py in used_positions:
            dist = math.sqrt((x - px) ** 2 + (y - py) ** 2)
            if dist < min_dist:
                return False
        return True

    models = []
    for i in range(count):
        # 随机选模板
        template = random.choice(OBSTACLE_TEMPLATES)
        # 随机找不重叠的位置
        for attempt in range(100):
            x = random.uniform(-BOUNDARY, BOUNDARY)
            y = random.uniform(-BOUNDARY, BOUNDARY)
            if position_available(x, y, min_dist=2.5):
                used_positions.append((x, y))
                break

        theta = random.uniform(0, 2 * math.pi)
        models.append(template["generate"](i, x, y, theta))

    sdf_output = "\n".join(models)

    if append and world_path:
        # 追加到世界文件
        with open(world_path, "r") as f:
            content = f.read()
        # 在 </world> 前插入
        insert_marker = "  </world>"
        if insert_marker in content:
            content = content.replace(insert_marker, sdf_output + "\n\n  </world>")
            with open(world_path, "w") as f:
                f.write(content)
            print(f"✅ 已将 {count} 个随机障碍物追加到 {world_path}")
        else:
            print("❌ 世界文件中未找到 </world> 标记")
    else:
        print(sdf_output)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="随机生成草坪障碍物 SDF 模型")
    parser.add_argument("--count", type=int, default=5, help="障碍物数量")
    parser.add_argument("--seed", type=int, default=None, help="随机种子")
    parser.add_argument("--append", action="store_true", help="追加到世界文件")
    parser.add_argument(
        "--world",
        default="/home/yh/mower_ws/src/outdoor_sim/worlds/grassland_50x50.world",
        help="世界文件路径",
    )
    args = parser.parse_args()
    generate(
        count=args.count,
        seed=args.seed,
        append=args.append,
        world_path=args.world if args.append else None,
    )
    if not args.append:
        print(f"\n# 生成 {args.count} 个随机障碍物，插入到 grassland_50x50.world 的 </world> 之前")
        print(f"# 或使用 --append 自动追加: python3 random_obstacles.py --count {args.count} --append")
