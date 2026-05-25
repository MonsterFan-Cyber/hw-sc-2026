"""数据文件解析器 - 解析 .snpst 二进制快照文件"""

import struct
from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class FrameData:
    """单帧快照数据"""
    translations: List[Tuple[float, float]]  # (dx, dy) per polygon
    validity: List[bool]                     # True=合法, False=非法


@dataclass
class SnpstData:
    """解析后的 .snpst 快照数据"""
    boundary_poly: List[Tuple[float, float]]      # 可行域多边形顶点
    polygons: List[List[Tuple[float, float]]]     # 初始多边形列表
    frames: List[FrameData] = field(default_factory=list)  # 所有快照帧


def parse_snpst_file(filepath: str) -> SnpstData:
    """解析 .snpst 二进制快照文件

    格式（小端序）：
      多边形模块：
        u32  可行域顶点数
        f32[2*n]  可行域顶点坐标 (x,y 交替)
        u32  多边形数量
        per polygon: u32 顶点数 + f32[2*m] 坐标
      快照模块：
        u32  快照帧数
        per frame: f32[2*N] 平移向量 + u8[N] 合法性标记
    """
    with open(filepath, 'rb') as f:
        data = f.read()

    offset = 0

    # --- 可行域多边形 ---
    n_boundary = struct.unpack_from('<I', data, offset)[0]; offset += 4
    boundary_poly: List[Tuple[float, float]] = []
    for _ in range(n_boundary):
        x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        boundary_poly.append((x, y))

    # --- 初始多边形 ---
    n_polys = struct.unpack_from('<I', data, offset)[0]; offset += 4
    polygons: List[List[Tuple[float, float]]] = []
    for _ in range(n_polys):
        n_verts = struct.unpack_from('<I', data, offset)[0]; offset += 4
        vertices: List[Tuple[float, float]] = []
        for _ in range(n_verts):
            x = struct.unpack_from('<f', data, offset)[0]; offset += 4
            y = struct.unpack_from('<f', data, offset)[0]; offset += 4
            vertices.append((x, y))
        polygons.append(vertices)

    # --- 快照帧 ---
    n_frames = struct.unpack_from('<I', data, offset)[0]; offset += 4
    frames: List[FrameData] = []
    for _ in range(n_frames):
        translations: List[Tuple[float, float]] = []
        for _ in range(n_polys):
            dx = struct.unpack_from('<f', data, offset)[0]; offset += 4
            dy = struct.unpack_from('<f', data, offset)[0]; offset += 4
            translations.append((dx, dy))
        validity: List[bool] = []
        for _ in range(n_polys):
            v = struct.unpack_from('<B', data, offset)[0]; offset += 1
            validity.append(bool(v))
        frames.append(FrameData(translations=translations, validity=validity))

    return SnpstData(
        boundary_poly=boundary_poly,
        polygons=polygons,
        frames=frames,
    )