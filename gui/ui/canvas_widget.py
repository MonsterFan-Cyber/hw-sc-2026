"""图形显示组件 - 使用 PyQtGraph 渲染多边形"""

from enum import Enum, auto
from typing import List, Tuple, Optional

import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QColor, QPolygonF, QBrush, QPen


class DisplayMode(Enum):
    """画布显示模式（三态互斥）"""
    ORIGINAL = auto()   # 显示原始多边形
    RESULT = auto()     # 显示求解结果（平移后多边形）
    FRAME = auto()      # 显示某一帧中间结果


class CanvasWidget(QWidget):
    """基于 PyQtGraph 的多边形显示组件
    
    功能:
        - 绘制边界矩形
        - 绘制原始多边形
        - 绘制求解后的平移多边形
        - 支持缩放/平移
    """
    
    # 多边形颜色列表（循环使用）
    COLORS = [
        (100, 149, 237),   # 矢车菊蓝
        (255, 127, 80),    # 珊瑚色
        (60, 179, 113),    # 海洋绿
        (238, 130, 238),   # 紫罗兰
        (255, 215, 0),     # 金色
        (65, 105, 225),    # 皇家蓝
        (255, 99, 71),     # 番茄红
        (50, 205, 50),     # 石灰绿
        (147, 112, 219),   # 中紫色
        (218, 165, 32),    # 金菊色
    ]
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self._boundary: Optional[Tuple[float, float, float, float]] = None  # (l, r, u, d)
        self._original_polys: List[List[Tuple[float, float]]] = []
        self._translations: List[Tuple[float, float]] = []
        self._frame_polys: List[List[Tuple[float, float]]] = []
        self._frame_validity: List[bool] = []
        
        # 显式显示模式（三态互斥）
        self._mode: DisplayMode = DisplayMode.ORIGINAL
        
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # 创建 PlotWidget
        self._plot = pg.PlotWidget()
        self._plot.setBackground('w')
        self._plot.showGrid(x=True, y=True, alpha=0.3)
        self._plot.setLabel('left', 'Y')
        self._plot.setLabel('bottom', 'X')
        self._plot.setAspectLocked(True)  # 保持等比例
        
        # 启用抗锯齿
        self._plot.setAntialiasing(True)
        
        layout.addWidget(self._plot)
    
    @property
    def display_mode(self) -> DisplayMode:
        """当前显示模式"""
        return self._mode
    
    def set_data(self, boundary: Tuple[float, float, float, float],
                 poly: List[Tuple[float, float]],
                 polygons: List[List[Tuple[float, float]]],
                 keep_view: bool = False):
        """设置原始数据，切换到 ORIGINAL 模式"""
        self._boundary = boundary
        self._poly = poly
        self._original_polys = polygons
        self._translations = []
        self._frame_polys = []
        self._mode = DisplayMode.ORIGINAL
        self.redraw(keep_view=keep_view)
    
    def set_translations(self, translations: List[Tuple[float, float]],
                         keep_view: bool = False):
        """设置平移向量，切换到 RESULT 模式"""
        self._translations = translations.copy() if translations else []
        self._frame_polys = []
        self._mode = DisplayMode.RESULT
        self.redraw(keep_view=keep_view)
    
    def set_frame_polys(self, polygons: List[List[Tuple[float, float]]],
                        boundary: Optional[Tuple[float, float, float, float]] = None,
                        validity: Optional[List[bool]] = None,
                        keep_view: bool = False):
        """设置中间帧多边形，切换到 FRAME 模式

        Args:
            polygons: 多边形列表，每个多边形是顶点列表
            boundary: 边界矩形，若为 None 则使用已有边界
            validity: 每个多边形的合法性标记列表，None 表示全合法
            keep_view: 是否保持当前视口位置和缩放
        """
        if boundary is not None:
            self._boundary = boundary
        self._translations = []
        self._frame_polys = polygons
        self._frame_validity = validity if validity is not None else []
        self._mode = DisplayMode.FRAME
        self.redraw(keep_view=keep_view)
    
    def switch_to_original(self):
        """切换到 ORIGINAL 模式（不清除数据）"""
        if self._original_polys:
            self._mode = DisplayMode.ORIGINAL
            self.redraw()
    
    def switch_to_result(self):
        """切换到 RESULT 模式"""
        if self._translations and len(self._translations) == len(self._original_polys):
            self._mode = DisplayMode.RESULT
            self.redraw()
    
    def switch_to_frame(self):
        """切换到 FRAME 模式"""
        if self._frame_polys:
            self._mode = DisplayMode.FRAME
            self.redraw()
    
    def clear(self):
        """清空画布"""
        self._plot.clear()
    
    def redraw(self, keep_view: bool = True):
        """重绘所有内容（根据当前 _mode 决定显示什么）
        
        Args:
            keep_view: 是否保持当前视口位置和缩放
        """
        vb = self._plot.getViewBox()
        
        if keep_view:
            state = vb.getState()
        
        vb.disableAutoRange()
        self._plot.clear()
        
        if self._poly is not None:
            self._draw_boundary(self._poly)
        
        if self._mode == DisplayMode.FRAME and self._frame_polys:
            self._draw_frame_polys()
        elif self._mode == DisplayMode.RESULT and self._translations and len(self._translations) == len(self._original_polys):
            self._draw_result_polys()
        elif self._mode == DisplayMode.ORIGINAL and self._original_polys:
            self._draw_original_polys()
        
        if keep_view:
            vb.setState(state)
        else:
            vb.enableAutoRange()
    
    def _draw_boundary(self, poly: List[Tuple[float, float]]):
        """绘制边界矩形
        
        boundary: (l, r, u, d)
        注意: u 是上边界，d 是下边界
        """
        # l, r, u, d = boundary
        
        # 矩形四个角
        # x = [l, r, r, l, l]
        # y = [d, d, u, u, d]
        x = [v[0] for v in poly]
        y = [v[1] for v in poly]

        x.append(x[0])
        y.append(y[0])
        
        # 边界矩形：粗黑线
        boundary_curve = self._plot.plot(
            x, y,
            pen=pg.mkPen(color='k', width=2, style=Qt.SolidLine),
            name='Boundary'
        )
        print(x)
        
        # 虚线区域半透明填充
        # fill = pg.FillBetweenItem(
        #     self._plot.plot(x, y, pen=None),
        #     self._plot.plot([l, r], [d, d], pen=None),
        #     brush=pg.mkBrush(200, 200, 200, 50)
        # )
        # # 用 QRectF 绘制半透明背景
        # from PyQt5.QtCore import QRectF
        # from PyQt5.QtGui import QBrush, QColor
        
        # rect_item = pg.QtWidgets.QGraphicsRectItem(
        #     QRectF(l, d, r - l, u - d)
        # )
        # rect_item.setBrush(QBrush(QColor(220, 220, 255, 30)))
        # rect_item.setPen(pg.mkPen(color=(200, 200, 255), width=1, style=Qt.DashLine))
        # self._plot.addItem(rect_item)
    
    def _draw_original_polys(self):
        """绘制原始多边形（初始位置）"""
        for i, poly in enumerate(self._original_polys):
            color = self.COLORS[i % len(self.COLORS)]
            self._draw_polygon(poly, color, filled=True, alpha=80, stroke_alpha=200)
    
    INVALID_COLOR = (220, 50, 50)  # 非法多边形颜色（红色）

    def _draw_frame_polys(self):
        """绘制中间帧多边形，非法多边形以红色显示"""
        for i, poly in enumerate(self._frame_polys):
            is_valid = self._frame_validity[i] if i < len(self._frame_validity) else True
            color = self.COLORS[i % len(self.COLORS)] if is_valid else self.INVALID_COLOR
            self._draw_polygon(poly, color, filled=True, alpha=50, stroke_alpha=180,
                               style=Qt.DashLine, width=2)
    
    def _draw_result_polys(self):
        """绘制平移后的多边形"""
        for i, (poly, trans) in enumerate(zip(self._original_polys, self._translations)):
            tx, ty = trans
            moved_poly = [(x + tx, y + ty) for x, y in poly]
            color = self.COLORS[i % len(self.COLORS)]
            self._draw_polygon(moved_poly, color, filled=True, alpha=60, stroke_alpha=220)
            
            # 绘制平移方向箭头
            if abs(tx) > 1e-6 or abs(ty) > 1e-6:
                # 计算多边形中心
                cx = sum(x for x, y in poly) / len(poly)
                cy = sum(y for x, y in poly) / len(poly)
                
                # 绘制虚线从原中心到新中心
                arrow = self._plot.plot(
                    [cx, cx + tx],
                    [cy, cy + ty],
                    pen=pg.mkPen(color=color, width=1, style=Qt.DashLine)
                )
                
                # 绘制原始多边形（半透明浅灰）
                self._draw_polygon(poly, (150, 150, 150), filled=False, alpha=60,
                                   style=Qt.DashLine, width=1)
    
    def _draw_polygon(self, vertices: List[Tuple[float, float]],
                      color: Tuple[int, int, int],
                      filled: bool = True,
                      alpha: int = 80,
                      stroke_alpha: int = 200,
                      style=Qt.SolidLine,
                      width: int = 2):
        """绘制单个多边形
        
        Args:
            vertices: 顶点列表 [(x, y), ...]
            color: RGB 颜色元组
            filled: 是否填充
            alpha: 填充透明度 0-255
            stroke_alpha: 描边透明度 0-255
            style: 线条样式
            width: 线条宽度
        """
        n = len(vertices)
        if n < 2:
            return
        
        # 闭合多边形
        xs = [v[0] for v in vertices] + [vertices[0][0]]
        ys = [v[1] for v in vertices] + [vertices[0][1]]
        
        if filled:
            # 使用 QGraphicsPolygonItem 填充多边形（避免 fillLevel 在放大时失效）
            polygon = QPolygonF([QPointF(x, y) for x, y in vertices])
            
            # 1. 半透明填充
            fill_item = pg.QtWidgets.QGraphicsPolygonItem(polygon)
            fill_item.setBrush(QBrush(QColor(*color, alpha)))
            fill_item.setPen(QPen(Qt.NoPen))  # 无描边
            self._plot.addItem(fill_item)
            
            # 2. 不透明描边（在填充之上）
            stroke_item = pg.QtWidgets.QGraphicsPolygonItem(polygon)
            stroke_item.setBrush(QBrush(Qt.NoBrush))  # 无填充
            stroke_pen = QPen(QColor(*color, stroke_alpha), width, style)
            stroke_pen.setCosmetic(True)  # 线宽不随缩放变化
            stroke_item.setPen(stroke_pen)
            self._plot.addItem(stroke_item)
        else:
            pen = pg.mkPen(color=color + (stroke_alpha,), width=width, style=style)
            curve = pg.PlotDataItem(xs, ys, pen=pen, antialias=True)
            self._plot.addItem(curve)
    
    def reset_view(self):
        """重置视图到合适范围"""
        # return
        if self._boundary:
            l, r, u, d = self._boundary
            # 加一点边距
            margin = max((r - l), (u - d)) * 0.05
            vb = self._plot.getViewBox()
            # 解锁等比以分别设置 X/Y 范围，避免互相覆盖
            aspect_locked = vb.state.get('aspectLocked', False)
            vb.setAspectLocked(False)
            self._plot.setXRange(l - margin, r + margin)
            self._plot.setYRange(d - margin, u + margin)
            vb.setAspectLocked(aspect_locked)
        else:
            self._plot.autoRange()
