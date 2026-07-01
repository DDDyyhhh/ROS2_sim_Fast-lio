#!/usr/bin/env python3
"""
pcd_to_pgm.py — 将 PCD 点云地图转换为 Nav2 可用的 2D pgm + yaml

用法:
    python3 pcd_to_pgm.py [输入.pcd] [输出前缀]

默认输入: /home/orangepi/ros2_ws/my_3d_map.pcd
默认输出: /home/orangepi/ros2_ws/fastlio_map.pgm + fastlio_map.yaml
"""

import sys
import os
import struct
import numpy as np
import cv2

# ── 配置参数 ─────────────────────────────────────────────────
Z_MIN = 0.1      # 最低高度（滤除地面）
Z_MAX = 1.5      # 最高高度（滤除天花板/上空噪点）
RESOLUTION = 0.05  # 米/像素
OCCUPIED = 0       # 障碍物像素值 (黑色)
FREE = 254         # 空闲像素值 (白色)
UNKNOWN = 205      # 未知区域像素值 (灰色)
# ────────────────────────────────────────────────────────────


def parse_pcd_header(filepath):
    """解析 PCD 文件头，返回 header dict 和 data_offset (文件指针位置)。"""
    header = {}
    with open(filepath, 'rb') as f:
        while True:
            line = f.readline()
            if not line:
                break
            line = line.decode('utf-8', errors='ignore').strip()
            if line.startswith('VERSION'):
                header['version'] = line.split()[-1]
            elif line.startswith('FIELDS'):
                header['fields'] = line.split()[1:]
            elif line.startswith('SIZE'):
                header['size'] = list(map(int, line.split()[1:]))
            elif line.startswith('TYPE'):
                header['type'] = line.split()[1:]
            elif line.startswith('COUNT'):
                header['count'] = list(map(int, line.split()[1:]))
            elif line.startswith('WIDTH'):
                header['width'] = int(line.split()[-1])
            elif line.startswith('HEIGHT'):
                header['height'] = int(line.split()[-1])
            elif line.startswith('POINTS'):
                header['points'] = int(line.split()[-1])
            elif line.startswith('DATA'):
                header['data'] = line.split()[-1]
                break  # 遇到 DATA 行后结束头部解析
        data_offset = f.tell()
    return header, data_offset


def load_points_ascii(filepath, header, data_offset):
    """解析 ASCII 格式点云。"""
    # 只取前 3 列 (x, y, z)
    # 使用 genfromtxt 跳过头部（通过 data_offset 之前已经读过 header）
    # 但更简单: 直接从 DATA 行之后逐行解析
    points = []
    with open(filepath, 'r') as f:
        # 跳过头部
        f.seek(0)
        while True:
            line = f.readline()
            if not line:
                break
            if line.startswith('DATA'):
                break
        # 解析数据行
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 3:
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    points.append((x, y, z))
                except ValueError:
                    continue
    return np.array(points, dtype=np.float64)


def load_points_binary(filepath, header, data_offset):
    """解析二进制格式点云 (DATA binary)。"""
    fields = header.get('fields', ['x', 'y', 'z'])
    sizes = header.get('size', [4, 4, 4])
    types = header.get('type', ['F', 'F', 'F'])
    count = header.get('count', [1, 1, 1])
    n_points = header.get('points', 0)

    if n_points == 0:
        return np.empty((0, 3))

    # 计算每个点的字节数
    point_step = sum(s * c for s, c in zip(sizes, count))

    with open(filepath, 'rb') as f:
        f.seek(data_offset)
        raw = f.read(n_points * point_step)

    # 找到 x, y, z 的偏移
    offset_map = {}
    off = 0
    for fld, sz, tp, cnt in zip(fields, sizes, types, count):
        offset_map[fld] = (off, sz, tp, cnt)
        off += sz * cnt

    dtype_parts = []
    for fld in ['x', 'y', 'z']:
        if fld not in offset_map:
            continue
        off, sz, tp, cnt = offset_map[fld]
        if tp == 'F':
            np_type = 'f4' if sz == 4 else 'f8'
        elif tp == 'U':
            np_type = 'u4' if sz == 4 else 'u2' if sz == 2 else 'u1'
        elif tp == 'I':
            np_type = 'i4' if sz == 4 else 'i2' if sz == 2 else 'i1'
        else:
            continue
        for _ in range(cnt):
            dtype_parts.append((fld, np_type))

    if not dtype_parts:
        return np.empty((0, 3))

    dtype = np.dtype(dtype_parts)
    # 需要处理偏移
    structured = np.frombuffer(raw, dtype=dtype, count=n_points)

    result = np.column_stack([
        structured['x'].astype(np.float64),
        structured['y'].astype(np.float64),
        structured['z'].astype(np.float64),
    ])
    return result


def load_points(filepath):
    """自动检测格式并加载点云。"""
    header, data_offset = parse_pcd_header(filepath)
    print(f"  PCD 版本: {header.get('version', 'N/A')}")
    print(f"  点数: {header.get('points', 0):,}")
    print(f"  字段: {header.get('fields', 'N/A')}")
    print(f"  数据格式: {header.get('data', 'N/A')}")

    if header.get('data') == 'binary':
        pts = load_points_binary(filepath, header, data_offset)
    else:
        pts = load_points_ascii(filepath, header, data_offset)

    print(f"  实际加载点数: {len(pts):,}")
    return pts


def build_occupancy_grid(points, resolution, z_min, z_max):
    """将 3D 点投影到 2D 占用网格。"""
    # 高度滤波
    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    filtered = points[mask]
    print(f"  Z 在 [{z_min}, {z_max}]m 内的点数: {len(filtered):,} ({len(filtered)/len(points)*100:.1f}%)")

    if len(filtered) == 0:
        raise RuntimeError("高度滤波后没有剩余点，请调整 Z_MIN / Z_MAX 范围")

    x = filtered[:, 0]
    y = filtered[:, 1]

    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()
    print(f"  X 范围: [{x_min:.3f}, {x_max:.3f}] m")
    print(f"  Y 范围: [{y_min:.3f}, {y_max:.3f}] m")

    # 计算图像尺寸（像素）
    width = int(np.ceil((x_max - x_min) / resolution)) + 1
    height = int(np.ceil((y_max - y_min) / resolution)) + 1
    print(f"  网格尺寸: {width} x {height} pixels (分辨率 {resolution} m/px)")

    # 创建图像，默认全部未知（灰色）
    img = np.full((height, width), UNKNOWN, dtype=np.uint8)

    # 将点云投影到像素坐标
    # PGM 坐标系: 行从上到下 (Y 轴向下), 列从左到右 (X 轴向右)
    # 世界坐标: X 向右, Y 向前
    # 约定: 图像行对应 -Y (上/北方向), 列对应 X
    # 这样图像的左上角对应 (x_min, y_max)
    col = ((x - x_min) / resolution).astype(int)
    row = ((y_max - y) / resolution).astype(int)  # Y 轴反转：像素行从上到下

    # 边界裁剪
    valid = (col >= 0) & (col < width) & (row >= 0) & (row < height)
    col, row = col[valid], row[valid]

    # 标记障碍物（黑色）
    img[row, col] = OCCUPIED
    print(f"  投影到网格的像素数: {len(col):,}")

    # 在障碍物之间标记空闲区域（白色）
    # 用膨胀处理：对障碍物像素进行一定范围的生长
    # 然后标记非障碍物且非未知的区域为空闲
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    occupied_mask = (img == OCCUPIED).astype(np.uint8)

    # 膨胀障碍物半径 ~= 机器人半径 (约 0.25m / resolution = 5 pixels)
    inflate_radius = int(0.25 / resolution)  # 机器人半径的膨胀
    inflate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                (inflate_radius * 2 + 1,
                                                 inflate_radius * 2 + 1))
    inflated = cv2.dilate(occupied_mask, inflate_kernel, iterations=1)

    # 使用 floodFill 从边缘开始填充空闲区域
    # 先创建一个二值图: 0=障碍/膨胀区, 255=可通行
    binary = np.ones((height + 2, width + 2), dtype=np.uint8) * 255
    binary[1:height+1, 1:width+1][inflated == 1] = 0

    # 从四个边缘进行泛洪填充
    cv2.floodFill(binary, None, (0, 0), 0)

    # binary[1:height+1, 1:width+1] 中仍为 255 的区域是不可达的空闲区域
    # 但我们标记所有非障碍物的可达区域为空闲
    free_mask = (binary[1:height+1, 1:width+1] == 255)

    # 空闲区域设为白色，但不覆盖障碍物
    img[free_mask & ~inflated.astype(bool)] = FREE

    # 统计
    n_occupied = np.sum(img == OCCUPIED)
    n_free = np.sum(img == FREE)
    n_unknown = np.sum(img == UNKNOWN)
    total = img.size
    print(f"  占用率:  {n_occupied/total*100:5.1f}%  ({n_occupied:,} px)")
    print(f"  空闲率:  {n_free/total*100:5.1f}%  ({n_free:,} px)")
    print(f"  未知率:  {n_unknown/total*100:5.1f}%  ({n_unknown:,} px)")

    return img, (x_min, y_max)  # 返回图像和原点坐标


def save_pgm(img, filepath):
    """保存为 PGM 图像 (P5 二进制格式)。"""
    cv2.imwrite(filepath, img)
    print(f"  PGM 已保存: {filepath}")


def save_yaml(filepath, image_name, resolution, origin, width, height):
    """生成 Nav2 格式的 yaml 配置文件。"""
    yaml_content = f"""# Nav2 地图配置文件
# 由 pcd_to_pgm.py 自动生成
image: {image_name}
mode: trinary
resolution: {resolution}
origin: [{origin[0]:.6f}, {origin[1]:.6f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    with open(filepath, 'w') as f:
        f.write(yaml_content)
    print(f"  YAML 已保存: {filepath}")


def main():
    # 解析命令行参数
    if len(sys.argv) >= 2:
        input_pcd = sys.argv[1]
    else:
        input_pcd = "/home/orangepi/ros2_ws/my_3d_map.pcd"

    if len(sys.argv) >= 3:
        output_prefix = sys.argv[2]
    else:
        output_prefix = "/home/orangepi/ros2_ws/fastlio_map"

    pgm_path = output_prefix + ".pgm"
    yaml_path = output_prefix + ".yaml"
    pgm_filename = os.path.basename(pgm_path)

    print("=" * 60)
    print("PCD → PGM + YAML 转换工具")
    print("=" * 60)
    print(f"输入:  {input_pcd}")
    print(f"输出:  {pgm_path}")
    print(f"       {yaml_path}")
    print(f"参数:  Z=[{Z_MIN}, {Z_MAX}]m, 分辨率={RESOLUTION}m/px")
    print("-" * 60)

    # 加载点云
    if not os.path.isfile(input_pcd):
        print(f"❌ 文件不存在: {input_pcd}")
        sys.exit(1)

    print("📡 加载点云...")
    points = load_points(input_pcd)
    if len(points) == 0:
        print("❌ 未加载到任何点")
        sys.exit(1)

    # 构建占用网格
    print("\n🗺️  构建占用网格...")
    img, origin = build_occupancy_grid(points, RESOLUTION, Z_MIN, Z_MAX)

    # 保存
    print("\n💾 保存文件...")
    save_pgm(img, pgm_path)
    h, w = img.shape
    save_yaml(yaml_path, pgm_filename, RESOLUTION, origin, w, h)

    print("\n" + "=" * 60)
    print("✅ 转换完成！Nav2 地图文件已生成:")
    print(f"   {pgm_path}")
    print(f"   {yaml_path}")
    print("=" * 60)


if __name__ == "__main__":
    main()
