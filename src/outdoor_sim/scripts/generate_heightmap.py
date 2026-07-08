#!/usr/bin/env python3
"""
generate_heightmap.py — 为 Ignition Fortress 生成斜坡地形高度图

输出：513x513 8-bit 灰度 PNG (2^9+1, Ogre 要求)
      像素值 0-255 <--> 高度 0-4m
      东北-西南走向脊线，西北丘，东南洼地

用法：
  python3 generate_heightmap.py [--output heightmap_30x30.png] [--seed 42]
"""

import argparse
import numpy as np
from PIL import Image


def gaussian(x, y, cx, cy, sigma_x, sigma_y, amp):
    """2D 高斯函数"""
    return amp * np.exp(-((x - cx) ** 2 / (2 * sigma_x ** 2) +
                          (y - cy) ** 2 / (2 * sigma_y ** 2)))


def make_heightmap(size=513, seed=42):
    """
    生成 513x513 高度图 (2^9+1, Ogre 要求)，地形特征：

    地形元素               位置        最高影响
    (1) 西北丘陵             (0.2, 0.8)  +0.4
    (2) 东南洼地             (0.8, 0.2)  -0.3
    (3) 东北-西南脊线        对角线附近   +0.2
    (4) 中部缓坡起伏        全区域        +/-0.08
    (5) 整体南高北低趋势     --            +0.15
    """
    rng = np.random.RandomState(seed)
    H, W = size, size

    # 归一化坐标 [0, 1]
    x = np.linspace(0, 1, W, dtype=np.float32)
    y = np.linspace(0, 1, H, dtype=np.float32)
    xx, yy = np.meshgrid(x, y)

    # --- 地形基面 ---
    terrain = np.zeros((H, W), dtype=np.float32)

    # (1) 西北丘陵 (x~0.2, y~0.8) -- 最高点
    hill = gaussian(xx, yy, 0.2, 0.78, 0.12, 0.10, 0.40)
    hill += gaussian(xx, yy, 0.15, 0.85, 0.06, 0.06, 0.15)
    terrain += hill

    # (2) 东南洼地 (x~0.8, y~0.2) -- 最低点
    pond = gaussian(xx, yy, 0.78, 0.22, 0.10, 0.08, -0.30)
    terrain += pond

    # (3) 东北-西南脊线：一条斜向隆起
    ridge_dist = np.abs((xx + yy) - 0.9) / 0.707
    ridge = np.maximum(0, 1 - ridge_dist / 0.25) * 0.20
    terrain += ridge

    # (4) 中部缓坡起伏 -- 小尺度噪声
    noise = rng.randn(H, W).astype(np.float32) * 0.08
    terrain += noise

    # (5) 南高北低趋势（y 越大越高）
    slope = (yy - 0.3) * 0.15
    terrain += slope

    # --- 整体偏移到 [0, 1] 区间 ---
    t_min, t_max = terrain.min(), terrain.max()
    terrain = (terrain - t_min) / (t_max - t_min)
    terrain = terrain * 0.9 + 0.05
    terrain = np.clip(terrain, 0.0, 1.0)

    # --- 转 8-bit ---
    img_data = (terrain * 255).astype(np.uint8)

    # 边缘渐变下降，让 Ignition 边界更自然
    edge_mask = np.ones((H, W), dtype=np.float32)
    fade = 10
    for i in range(fade):
        t = (fade - i) / fade
        edge_mask[i, :] *= t
        edge_mask[-(i + 1), :] *= t
        edge_mask[:, i] *= t
        edge_mask[:, -(i + 1)] *= t
    img_data = (img_data.astype(np.float32) * edge_mask).astype(np.uint8)

    return img_data


def main():
    parser = argparse.ArgumentParser(description="生成斜坡地形高度图")
    parser.add_argument(
        "--output", default="heightmap_30x30.png",
        help="输出 PNG 路径 (默认: heightmap_30x30.png)",
    )
    parser.add_argument(
        "--size", type=int, default=513,
        help="图像尺寸 (默认: 513, Ogre 要求 2^n+1)",
    )
    parser.add_argument(
        "--seed", type=int, default=42, help="随机种子 (默认: 42)"
    )
    args = parser.parse_args()

    print(f"生成 {args.size}x{args.size} 高度图 (seed={args.seed})...")
    img_data = make_heightmap(size=args.size, seed=args.seed)

    img = Image.fromarray(img_data, mode="L")
    img.save(args.output)
    print(f"已保存: {args.output} ({img.size[0]}x{img.size[1]})")

    vals = img_data.flatten()
    print(f"  高度范围: {vals.min()}--{vals.max()} (8-bit)")
    print(f"  对应实尺: {vals.min()/255*4:.2f}--{vals.max()/255*4:.2f}m")


if __name__ == "__main__":
    main()
