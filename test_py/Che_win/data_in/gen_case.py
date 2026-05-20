# 案例 *.input
# 格式为
# 格式如下
# 第一行为矩形可行域 x_min x_max y_min y_max
# 第二行为多边形数量，此后每两个一个多边形
# 第三行为多边形顶点数量 NP
# 第四行为多边形顶点坐标 x1 y1 x2 y2 ... xNP yNP

"""
0.00000 6.68517 0.00000 1.61503
4
8
0.00000 0.00000 0.82367 0.00000 0.89156 0.12755 0.86421 0.30784 0.70798 0.31872 0.63283 0.37970 0.34970 0.34904 0.00000 0.37080
4
0.00000 0.00000 0.30850 0.00000 0.30850 0.21721 0.00000 0.21721
4
0.00000 0.00000 1.00000 0.00000 1.00000 0.08603 0.00000 0.08603
6
0.00000 0.00000 0.58042 -0.05702 0.71951 0.15722 0.71951 0.27587 0.58042 0.49011 0.00000 0.43309
"""

# 要求生成 n (默认100) 个 顶点在 m0~m1 (默认 80~100) 范围的随机多边形 或者 正多边形。
# 随机多边形和正多边形的比例大致 1:1
# 多边形的包围盒长宽控制在 0.02~0.5 之间，使用随机缩放。
# 矩形可行域的大致是一个 6:2 的矩形，面积要是所有多边形包围盒面积之和的 1.5 被
# 将案例生成输出到指定文件夹 test_py/Che_linux/data_v3 

import random
import math
import os
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

from polygenerator import (
    random_polygon,
    random_star_shaped_polygon,
    random_convex_polygon,
)

GRID_SIZE = 1e-4  # 网格对齐精度

def snap_to_grid(value):
    """将值对齐到最近的网格点"""
    return round(value / GRID_SIZE) * GRID_SIZE

def snap_point(x, y):
    """将点坐标对齐到网格"""
    return snap_to_grid(x), snap_to_grid(y)

def snap_vertices(vertices):
    """将所有顶点对齐到网格"""
    return [snap_point(x, y) for x, y in vertices]

def generate_regular_polygon(n_vertices, center_x, center_y, radius):
    vertices = []
    for i in range(n_vertices):
        angle = 2 * math.pi * i / n_vertices
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        x, y = snap_point(x, y)
        vertices.append((x, y))
    return vertices

def generate_random_polygon(n_vertices, center_x, center_y, avg_radius):
    vertices = random_polygon(n_vertices)
    # 将原始顶点对齐到网格
    vertices = snap_vertices(vertices)
    
    xs = [v[0] for v in vertices]
    ys = [v[1] for v in vertices]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    
    width = x_max - x_min
    height = y_max - y_min
    center_x_orig = (x_min + x_max) / 2
    center_y_orig = (y_min + y_max) / 2
    
    scale = avg_radius * 2 / max(width, height)
    
    scaled_vertices = []
    for x, y in vertices:
        new_x = (x - center_x_orig) * scale + center_x
        new_y = (y - center_y_orig) * scale + center_y
        new_x, new_y = snap_point(new_x, new_y)
        scaled_vertices.append((new_x, new_y))
    
    return scaled_vertices

def get_bounding_box(vertices):
    xs = [v[0] for v in vertices]
    ys = [v[1] for v in vertices]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    return x_min, x_max, y_min, y_max

def scale_polygon(vertices, scale_x, scale_y):
    scaled_vertices = []
    for x, y in vertices:
        new_x = x * scale_x
        new_y = y * scale_y
        new_x, new_y = snap_point(new_x, new_y)
        scaled_vertices.append((new_x, new_y))
    return scaled_vertices

def translate_polygon(vertices, dx, dy):
    translated_vertices = []
    for x, y in vertices:
        new_x = x + dx
        new_y = y + dy
        new_x, new_y = snap_point(new_x, new_y)
        translated_vertices.append((new_x, new_y))
    return translated_vertices

def normalize_polygon(vertices):
    x_min, x_max, y_min, y_max = get_bounding_box(vertices)
    dx = -x_min
    dy = -y_min
    return translate_polygon(vertices, dx, dy)

def generate_polygon_with_bbox_size(n_vertices, target_width, target_height, is_regular, 
                                    distribution_mode="bottom_left", domain_width=None, domain_height=None):
    if is_regular:
        vertices = generate_regular_polygon(n_vertices, 0, 0, 1.0)
    else:
        vertices = generate_random_polygon(n_vertices, 0, 0, 1.0)
    
    x_min, x_max, y_min, y_max = get_bounding_box(vertices)
    width = x_max - x_min
    height = y_max - y_min
    
    if width > 0 and height > 0:
        scale_x = target_width / width
        scale_y = target_height / height
        vertices = scale_polygon(vertices, scale_x, scale_y)
    
    # 根据分布模式决定位置
    if distribution_mode == "bottom_left":
        # 当前逻辑：归一化到左下角 + 小随机偏移
        vertices = normalize_polygon(vertices)
        
        # 生成网格对齐的随机位移
        max_offset = 0.01  # 最大位移
        dx = snap_to_grid(random.uniform(0, max_offset))
        dy = snap_to_grid(random.uniform(0, max_offset))
        
    elif distribution_mode == "left_half_random":
        # 随机分布在左半部分
        if domain_width is None or domain_height is None:
            raise ValueError("domain_width and domain_height are required for left_half_random mode")
        
        # 计算多边形包围盒
        x_min, x_max, y_min, y_max = get_bounding_box(vertices)
        poly_width = x_max - x_min
        poly_height = y_max - y_min
        
        # 计算可放置范围（左半部分）
        max_x = domain_width / 2 - poly_width
        max_y = domain_height - poly_height
        
        # 确保有足够的空间
        if max_x > 0 and max_y > 0:
            dx = snap_to_grid(random.uniform(0, max_x))
            dy = snap_to_grid(random.uniform(0, max_y))
        else:
            # 如果没有足够空间，回退到左下角模式
            vertices = normalize_polygon(vertices)
            max_offset = 0.01
            dx = snap_to_grid(random.uniform(0, max_offset))
            dy = snap_to_grid(random.uniform(0, max_offset))
    else:
        raise ValueError(f"Unsupported distribution_mode: {distribution_mode}")
    
    vertices = translate_polygon(vertices, dx, dy)
    
    return vertices

def calculate_bbox_area(vertices):
    x_min, x_max, y_min, y_max = get_bounding_box(vertices)
    return (x_max - x_min) * (y_max - y_min)

def format_vertices(vertices):
    coords = []
    for v in vertices:
        coords.extend([f"{v[0]:.5f}", f"{v[1]:.5f}"])
    return " ".join(coords)

def generate_case(n_polygons, m0, m1, output_path, case_id, regular_ratio=0.0, distribution_mode="bottom_left"):
    polygons = []
    total_area = 0.0
    
    for i in range(n_polygons):
        n_vertices = random.randint(m0, m1)
        is_regular = random.random() < regular_ratio
        
        # 生成网格对齐的包围盒尺寸
        target_width = snap_to_grid(random.uniform(0.025, 0.5))
        # target_height = snap_to_grid(random.uniform(0.02, 0.5))
        target_height = target_width
        
        vertices = generate_polygon_with_bbox_size(n_vertices, target_width, target_height, is_regular)
        polygons.append((n_vertices, vertices))
        
        area = calculate_bbox_area(vertices)
        total_area += area
    
    domain_area = total_area * 1.5
    domain_height = math.sqrt(domain_area / 3.0)
    domain_width = domain_height * 3.0
    
    # 将可行域边界对齐到网格
    x_min = 0.0
    x_max = snap_to_grid(domain_width)
    y_min = 0.0
    y_max = snap_to_grid(domain_height)
    
    # 重新生成多边形，使用正确的分布模式和可行域尺寸
    polygons = []
    total_area = 0.0
    
    for i in range(n_polygons):
        n_vertices = random.randint(m0, m1)
        is_regular = random.random() < regular_ratio
        
        # 生成网格对齐的包围盒尺寸
        target_width = snap_to_grid(random.uniform(0.025, 0.5))
        target_height = target_width
        
        vertices = generate_polygon_with_bbox_size(
            n_vertices, target_width, target_height, is_regular,
            distribution_mode, domain_width, domain_height
        )
        polygons.append((n_vertices, vertices))
        
        area = calculate_bbox_area(vertices)
        total_area += area
    
    lines = []
    lines.append(f"{x_min:.5f} {x_max:.5f} {y_min:.5f} {y_max:.5f}")
    lines.append(str(n_polygons))
    
    for n_vertices, vertices in polygons:
        lines.append(str(n_vertices))
        lines.append(format_vertices(vertices))
    
    with open(output_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    
    return total_area, domain_area

def read_case(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    domain_parts = list(map(float, lines[0].strip().split()))
    x_min, x_max, y_min, y_max = domain_parts
    
    n_polygons = int(lines[1].strip())
    
    polygons = []
    idx = 2
    for _ in range(n_polygons):
        n_vertices = int(lines[idx].strip())
        idx += 1
        coords = list(map(float, lines[idx].strip().split()))
        idx += 1
        
        vertices = []
        for i in range(n_vertices):
            x = coords[i * 2]
            y = coords[i * 2 + 1]
            vertices.append((x, y))
        polygons.append(vertices)
    
    return (x_min, x_max, y_min, y_max), polygons

def visualize_case(input_path, output_path):
    domain, polygons = read_case(input_path)
    x_min, x_max, y_min, y_max = domain
    
    fig, ax = plt.subplots(1, 1, figsize=(12, 4))
    
    domain_rect = MplPolygon(
        [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)],
        fill=False, edgecolor='red', linewidth=2, linestyle='--'
    )
    ax.add_patch(domain_rect)
    
    colors = plt.cm.tab20(range(len(polygons)))
    for i, vertices in enumerate(polygons):
        poly_patch = MplPolygon(vertices, fill=True, facecolor=colors[i], 
                                edgecolor='black', linewidth=0.5, alpha=0.6)
        ax.add_patch(poly_patch)
    
    ax.set_xlim(x_min - 0.1, x_max + 0.1)
    ax.set_ylim(y_min - 0.1, y_max + 0.1)
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(f'{os.path.basename(input_path)} - {len(polygons)} polygons')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

def visualize_polygon_array(input_path, output_path):
    """将多边形按包围盒面积从大到小排序，绘制在10x10网格阵列中，填充淡蓝色"""
    domain, polygons = read_case(input_path)
    
    if len(polygons) != 100:
        print(f"警告: 多边形数量不是100，当前有{len(polygons)}个")
    
    areas = []
    for vertices in polygons:
        x_min, x_max, y_min, y_max = get_bounding_box(vertices)
        area = (x_max - x_min) * (y_max - y_min)
        areas.append(area)
    
    polygon_data = list(zip(polygons, areas))
    polygon_data.sort(key=lambda x: x[1], reverse=True)
    
    max_area = polygon_data[0][1] if polygon_data else 0
    max_width = math.sqrt(max_area) * 1.2
    max_height = math.sqrt(max_area) * 1.2
    
    fig, ax = plt.subplots(figsize=(12, 12))
    
    rows = 10
    cols = 10
    
    # 使用淡蓝色填充
    face_color = 'lightblue'
    edge_color = 'darkblue'
    
    for idx, (vertices, area) in enumerate(polygon_data[:100]):
        row = idx // cols
        col = idx % cols
        
        center_x = (col + 0.5) * max_width
        center_y = (rows - row - 0.5) * max_height
        
        x_min, x_max, y_min, y_max = get_bounding_box(vertices)
        poly_center_x = (x_min + x_max) / 2
        poly_center_y = (y_min + y_max) / 2
        
        translated_vertices = []
        for x, y in vertices:
            dx = center_x - poly_center_x
            dy = center_y - poly_center_y
            translated_vertices.append((x + dx, y + dy))
        
        poly_patch = MplPolygon(
            translated_vertices,
            fill=True,
            facecolor=face_color,
            edgecolor=edge_color,
            linewidth=0.8,
            alpha=0.8
        )
        ax.add_patch(poly_patch)
    
    total_width = cols * max_width
    total_height = rows * max_height
    
    ax.set_xlim(0, total_width)
    ax.set_ylim(0, total_height)
    ax.set_aspect('equal')
    ax.axis('off')
    
    plt.title(f'{os.path.basename(input_path)} - 多边形阵列 (按面积从大到小排序)', fontsize=14)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"  最大包围盒面积: {max_area:.6f}")
    print(f"  网格单元尺寸: {max_width:.6f} × {max_height:.6f}")

def main():
    output_dir = "/home/monsteru/workspace/hw_sec_2026_v3.1_separator/test_py/Che_linux/data_v3"
    os.makedirs(output_dir, exist_ok=True)
    
    n_polygons_list = [40]
    m0, m1 = 80, 100
    
    # 分布模式开关
    # "bottom_left": 左下角集中分布（默认）
    # "left_half_random": 左半部分随机均匀分布
    distribution_mode = "left_half_random"  # 可修改此值切换模式
    
    print(f"使用分布模式: {distribution_mode}")
    
    for i, n_polygons in enumerate(n_polygons_list):
        case_id = i + 1
        
        # 根据分布模式生成不同的文件名
        if distribution_mode == "bottom_left":
            filename_suffix = f"n{n_polygons_list[i]}_m{m1}"
        else:
            filename_suffix = f"n{n_polygons_list[i]}_m{m1}_{distribution_mode}"
        
        input_path = os.path.join(output_dir, f"practice_00_{filename_suffix}.in")
        total_area, domain_area = generate_case(n_polygons, m0, m1, input_path, case_id, 
                                              distribution_mode=distribution_mode)
        print(f"生成案例 {case_id}: {n_polygons} 个多边形, 包围盒总面积={total_area:.5f}, 可行域面积={domain_area:.5f}")
        
        png_path = os.path.join(output_dir, f"practice_00_{filename_suffix}.png")
        visualize_case(input_path, png_path)
        print(f"  可视化已保存至: {png_path}")
        
        array_png_path = os.path.join(output_dir, f"practice_00_{filename_suffix}_array.png")
        visualize_polygon_array(input_path, array_png_path)
        print(f"  多边形阵列图已保存至: {array_png_path}")

if __name__ == "__main__":
    random.seed(1)
    main()
