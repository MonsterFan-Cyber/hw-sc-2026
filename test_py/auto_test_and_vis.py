#!/usr/bin/env python3
"""
自动化测试 + 可视化脚本
功能:
  1. 对所有 data/*.in 运行 run_and_test.sh
    - 如果是 linux 使用  Che_linux/run_and_test.sh + Che_linux/Checker
    - 如果是 win 使用 Che_win/run_and_test.bat + Che_win/Checker.exe
  2. 用 Checker 判分
  3. 解析输入文件和输出文件，绘制排样前/后多边形分布图 （跑完直接绘制）
     - 放样矩形空间用蓝色虚线框表示
     - 排样前多边形用浅灰色填充
     - 排样后多边形用浅绿色填充
     - 红色箭头从旧位置质心指向新位置质心
"""

# 新增要求 1
# 在 Checker 调用后，会输出哪些多边形相交了（多边形编号从 1 开始），输入格式如下
"""
  [Checker stdout]:
  [Checker stderr]:
    [INFO]Checker.cpp:246|IsSatisfiedConstraints|Wrong: polygon 9 and 14 overlap with each other.
    [INFO]Checker.cpp:328|main|Parameter: lambda: 3.00
    [INFO]Checker.cpp:329|main|Result   : score : 0
    wrong answer WA
  Checker score: 0

"""
# 收集所有交叠对，每一对交叠生成一个 bug 案例
# 命名规则：
#   name.in -> name_p小_p大_bug.in （例如 practice_1_p9_p14_bug.in）
#   其中 p小、p大 为原文件中的多边形编号，小的在前，大的在后
# 文件内容：
#   - 边界：保持原样
#   - 多边形：只放置原始案例中的两个冲突多边形
#   - 编号：文件内重新从 1 开始编号，旧编号小的为 1，大的为 2
# 注意保持 *.in 的格式，格式参考 docs/in_format.md 文件
# 所有的 bug 案例需要保存到 test_py/bug_case 当中


#  ====================================================




import subprocess
import sys
import os
import glob
import re
import shutil
import platform

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon, FancyArrowPatch
from matplotlib.collections import PatchCollection
import numpy as np
# SOLUTION_DIR = "src_merge"
# src_name_cpp = "Solution.cpp"
# src_name = "Solution"

SOLUTION_DIR = "src"
src_name_cpp = "main.cpp"
src_name = "main"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
SOLUTION_CPP = os.path.join(ROOT_DIR, SOLUTION_DIR, src_name_cpp)

IS_WINDOWS = platform.system().lower().startswith("win")
CHE_DIR = os.path.join(SCRIPT_DIR, "Che_win" if IS_WINDOWS else "Che_linux")
DATA_DIR = os.path.join(CHE_DIR, "data_in")
OUTPUT_DIR = os.path.join(CHE_DIR, "data_out")
VIS_DIR = os.path.join(SCRIPT_DIR, "vis_output")
CHECKER = os.path.join(CHE_DIR, "Checker.exe" if IS_WINDOWS else "Checker")
BUG_CASE_DIR = os.path.join(SCRIPT_DIR, "bug_case")



def parse_input(input_file):
    """解析输入文件，返回 (feasible_poly, polys)
    feasible_poly: list of (x, y) - 可行域多边形顶点
    polys: list of list of (x, y) - 待排样多边形
    """
    with open(input_file, 'r') as f:
        tokens = f.read().split()
    pos = 0

    # 读取可行域多边形
    n_feasible = int(tokens[pos]); pos += 1
    feasible_poly = []
    for _ in range(n_feasible):
        x = float(tokens[pos]); pos += 1
        y = float(tokens[pos]); pos += 1
        feasible_poly.append((x, y))

    n = int(tokens[pos]); pos += 1
    polys = []
    for _ in range(n):
        ni = int(tokens[pos]); pos += 1
        verts = []
        for __ in range(ni):
            x = float(tokens[pos]); pos += 1
            y = float(tokens[pos]); pos += 1
            verts.append((x, y))
        polys.append(verts)

    return feasible_poly, polys


def parse_output(output_content, n):
    """解析输出文件，返回 [(dx1,dy1), (dx2,dy2), ...]"""
    lines = output_content.strip().split('\n')
    results = []
    
    # 空文件检查
    if not lines or (len(lines) == 1 and lines[0].strip() == ''):
        print(f"  警告: 输出文件为空，返回 {n} 个 (0,0) 位移")
        return [(0.0, 0.0) for _ in range(n)]
    
    start = 0
    # 跳过第一行如果它是数量N
    if lines[0].strip().isdigit():
        start = 1
    
    # 检查是否有足够的行
    if start + n > len(lines):
        print(f"  警告: 输出行数不足 (需要 {n} 行，实际 {len(lines)-start} 行)，填充 (0,0)")
        for i in range(n):
            if start + i < len(lines):
                parts = lines[start + i].split()
                if len(parts) >= 2:
                    dx = float(parts[0])
                    dy = float(parts[1])
                    results.append((dx, dy))
                else:
                    results.append((0.0, 0.0))
            else:
                results.append((0.0, 0.0))
        return results
    
    for i in range(n):
        parts = lines[start + i].split()
        if len(parts) >= 2:
            dx = float(parts[0])
            dy = float(parts[1])
            results.append((dx, dy))
        else:
            print(f"  警告: 第 {start+i+1} 行格式错误，使用 (0,0)")
            results.append((0.0, 0.0))
    return results


def compute_aabb(verts):
    """计算AABB包围盒，返回 (cx, cy, hw, hh)"""
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    xMin, xMax = min(xs), max(xs)
    yMin, yMax = min(ys), max(ys)
    eps = 1e-5
    xMin -= eps; xMax += eps
    yMin -= eps; yMax += eps
    cx = (xMin + xMax) / 2
    cy = (yMin + yMax) / 2
    hw = (xMax - xMin) / 2
    hh = (yMax - yMin) / 2
    return cx, cy, hw, hh


def parse_overlap_pairs(checker_stderr):
    """解析Checker stderr，提取交叠的多边形对
    返回: [(p1, p2), ...] 其中 p1 < p2
    """
    pairs = []
    pattern = r'polygon\s+(\d+)\s+and\s+(\d+)\s+overlap'
    for line in checker_stderr.split('\n'):
        m = re.search(pattern, line)
        if m:
            p1, p2 = int(m.group(1)), int(m.group(2))
            if p1 > p2:
                p1, p2 = p2, p1
            pairs.append((p1, p2))
    return pairs


def generate_bug_case(basename, input_file, overlap_pairs):
    """为每个交叠对生成bug案例文件"""
    if not overlap_pairs:
        return
    
    feasible_poly, polys = parse_input(input_file)
    os.makedirs(BUG_CASE_DIR, exist_ok=True)
    
    for p_small, p_big in overlap_pairs:
        bug_name = f"{basename}_p{p_small}_p{p_big}_bug"
        bug_file = os.path.join(BUG_CASE_DIR, bug_name + ".in")
        
        with open(bug_file, 'w') as f:
            # 写入可行域多边形
            f.write(f"{len(feasible_poly)}\n")
            coords = " ".join(f"{x:.5f} {y:.5f}" for x, y in feasible_poly)
            f.write(f"{coords}\n")
            # 写入两个冲突多边形
            f.write("2\n")
            poly_small = polys[p_small - 1]
            f.write(f"{len(poly_small)}\n")
            coords = " ".join(f"{x:.5f} {y:.5f}" for x, y in poly_small)
            f.write(f"{coords}\n")
            poly_big = polys[p_big - 1]
            f.write(f"{len(poly_big)}\n")
            coords = " ".join(f"{x:.5f} {y:.5f}" for x, y in poly_big)
            f.write(f"{coords}\n")
        
        print(f"  生成bug案例: {bug_file}")


def build_test_command(rel_solution, rel_inp_file, rel_out_file):
    """根据平台构造测试命令"""
    if IS_WINDOWS:
        return ["cmd", "/c", "run_and_test.bat", rel_solution, rel_inp_file, rel_out_file, "60000"]
    return ["bash", "run_and_test.sh", rel_solution, rel_inp_file, rel_out_file, "60000"]


def run_tests():
    print("=" * 60)
    print("[1] 运行自动化测试 ...")
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    os.makedirs(VIS_DIR, exist_ok=True)

    input_files = sorted(glob.glob(os.path.join(DATA_DIR, "*.in")))
    if not input_files:
        print("未找到测试数据!")
        return []

    results = []
    for inp_file in input_files:
        basename = os.path.basename(inp_file).replace('.in', '')
        out_file = os.path.join(OUTPUT_DIR, basename + ".out")
        print(f"\n--- 测试 {basename} ---")

        # 编译+运行 via run_and_test.sh (in Che directory)
        # 需要使用相对于 CHE_DIR 的相对路径
        rel_solution = os.path.relpath(SOLUTION_CPP, CHE_DIR)
        rel_inp_file = os.path.relpath(inp_file, CHE_DIR)
        rel_out_file = os.path.relpath(out_file, CHE_DIR)
        cmd = build_test_command(rel_solution, rel_inp_file, rel_out_file)
        r = subprocess.run(cmd, capture_output=True, text=True, cwd=CHE_DIR, timeout=120)
        print(r.stdout)
        if r.stderr:
            for line in r.stderr.strip().split('\n'):
                if not line.startswith('[INFO]'):
                    print(f"  stderr: {line}")

        if r.returncode != 0:
            print(f"  测试失败! exit code={r.returncode}")
            results.append((basename, inp_file, out_file, False, None))
            continue

        # Checker判分
        score, overlap_pairs = run_checker(inp_file, out_file)
        if score is not None:
            print(f"  Checker score: {score}")
        elif os.path.isfile(CHECKER):
            print(f"  Checker未能获取得分")
        else:
            print(f"  Checker不存在，跳过判分")
        
        # 生成bug案例
        if overlap_pairs:
            generate_bug_case(basename, inp_file, overlap_pairs)

        ok = os.path.isfile(out_file) and os.path.getsize(out_file) > 0
        results.append((basename, inp_file, out_file, ok, score))
        print(f"  结果: {'PASS' if ok else 'FAIL'}")

        # 立即可视化
        if ok:
            print(f"\n[可视化] {basename} ...")
            visualize(basename, inp_file, out_file, score=score)
        else:
            print(f"  跳过可视化（测试未通过）")

    return results


def run_checker(input_file, output_file):
    """运行Checker并返回 (score, overlap_pairs)
    score: 得分，失败返回None
    overlap_pairs: 交叠对列表 [(p1, p2), ...]
    """
    if not os.path.isfile(CHECKER):
        return None, []
    try:
        if not IS_WINDOWS:
            os.chmod(CHECKER, 0o755)
        chk_cmd = [CHECKER, input_file, output_file, output_file]
        cr = subprocess.run(chk_cmd, capture_output=True, text=True, cwd=CHE_DIR, timeout=30)
        
        print("  [Checker stdout]:")
        for line in (cr.stdout or '').split('\n'):
            if line.strip():
                print(f"    {line}")
        
        print("  [Checker stderr]:")
        for line in (cr.stderr or '').split('\n'):
            if line.strip():
                print(f"    {line}")
        
        score = None
        for line in (cr.stderr or '').split('\n'):
            m = re.search(r'score\s*:\s*(\d+)', line)
            if m:
                score = int(m.group(1))
                break
        
        overlap_pairs = parse_overlap_pairs(cr.stderr or '')
        return score, overlap_pairs
    except Exception as e:
        print(f"  Checker执行异常: {e}")
    return None, []


def visualize(basename, input_file, output_file, score=None):
    """绘制排样前/后对比图"""
    feasible_poly, polys = parse_input(input_file)
    n = len(polys)

    if not os.path.isfile(output_file):
        print(f"  输出文件不存在，跳过可视化: {output_file}")
        return

    with open(output_file, 'r') as f:
        out_content = f.read()
    displacements = parse_output(out_content, n)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    for ax, title, apply_disp in [(ax1, "Before Placement", False),
                                    (ax2, "After Placement", True)]:
        ax.set_title(title, fontsize=14)
        ax.set_aspect('equal')

        # 绘制可行域多边形
        feasible_polygon = MplPolygon(feasible_poly, closed=True,
                                      fill=False, edgecolor='blue', linestyle='--', linewidth=2)
        ax.add_patch(feasible_polygon)

        # 从可行域多边形计算坐标范围
        xs = [v[0] for v in feasible_poly]
        ys = [v[1] for v in feasible_poly]
        xMin, xMax = min(xs), max(xs)
        yMin, yMax = min(ys), max(ys)

        for i, verts in enumerate(polys):
            dx, dy = displacements[i] if apply_disp else (0, 0)
            moved_verts = [(v[0] + dx, v[1] + dy) for v in verts]

            # 多边形填充
            color = '#90EE90' if apply_disp else '#D3D3D3'
            alpha = 0.5 if apply_disp else 0.3
            poly = MplPolygon(moved_verts, closed=True, facecolor=color,
                              edgecolor='black', linewidth=0.8, alpha=alpha)
            ax.add_patch(poly)

            # 标注编号
            cx_orig, cy_orig, hw, hh = compute_aabb(verts)
            cx_moved = cx_orig + dx
            cy_moved = cy_orig + dy
            cx_show = cx_moved if apply_disp else cx_orig
            cy_show = cy_moved if apply_disp else cy_orig
            ax.text(cx_show, cy_show, str(i+1), ha='center', va='center', fontsize=6, color='black')

        # 设置坐标范围
        bw = xMax - xMin
        bh = yMax - yMin
        margin_x = bw * 0.1
        margin_y = bh * 0.1
        ax.set_xlim(xMin - margin_x, xMax + margin_x)
        ax.set_ylim(yMin - margin_y, yMax + margin_y)
        ax.grid(True, alpha=0.3)

    # 计算移动距离之和
    total_dist = 0.0
    for i, verts in enumerate(polys):
        dx, dy = displacements[i]
        total_dist += np.sqrt(dx * dx + dy * dy)

    # 在右图上添加红色箭头
    for i, verts in enumerate(polys):
        dx, dy = displacements[i]
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            continue
        cx_orig, cy_orig, hw, hh = compute_aabb(verts)
        cx_new = cx_orig + dx
        cy_new = cy_orig + dy
        ax2.annotate('', xy=(cx_new, cy_new), xytext=(cx_orig, cy_orig),
                     arrowprops=dict(arrowstyle='->', color='red', lw=1.5))

    score_str = f"  Score = {score}" if score is not None else ""
    fig.suptitle(f"Box Placement: {basename} (N={n})  Total Dist = {total_dist:.4f}{score_str}", fontsize=16, fontweight='bold')
    plt.tight_layout()

    vis_file = os.path.join(VIS_DIR, f"{basename}_vis.png")
    fig.savefig(vis_file, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  可视化保存: {vis_file}")


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    os.makedirs(VIS_DIR, exist_ok=True)

    # 1. 运行测试（每个案例完成后立即可视化）
    results = run_tests()

    # 2. 总结
    print("=" * 60)
    print("[2] 测试总结")
    for basename, _, _, ok, score in results:
        score_str = f" (score={score})" if score is not None else ""
        print(f"  {basename}: {'PASS' if ok else 'FAIL'}{score_str}")
    n_pass = sum(1 for _, _, _, ok, _ in results if ok)
    print(f"  总计: {n_pass}/{len(results)} 通过")



if __name__ == '__main__':
    main()
