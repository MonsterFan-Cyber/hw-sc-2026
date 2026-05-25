"""主窗口 - QSplitter 布局，组合所有组件"""

from typing import List, Tuple, Optional

from PyQt5.QtWidgets import (
    QMainWindow, QSplitter, QMessageBox,
    QStatusBar
)
from PyQt5.QtCore import Qt, QTimer

from gui.core.data_loader import parse_snpst_file, SnpstData, FrameData
from gui.ui.canvas_widget import CanvasWidget, DisplayMode
from gui.ui.control_panel import ControlPanel
from gui.ui.terminal_widget import TerminalWidget


class MainWindow(QMainWindow):
    """主窗口

    布局:
        +--------------------------+
        |     Menu / Toolbar       |
        +------------+-------------+
        |            |  Control    |
        |  Canvas    |  Panel      |
        |            |             |
        +------------+-------------+
        |     Terminal (日志)       |
        +--------------------------+
        |      Status Bar          |
        +--------------------------+
    """

    def __init__(self):
        super().__init__()

        self._snpst_data: Optional[SnpstData] = None
        self._frames: List[FrameData] = []
        self._translations: List[Tuple[float, float]] = []
        self._current_frame_idx: int = 0  # 1-based，0 表示未选中

        # 自动播放定时器
        self._play_timer = QTimer(self)
        self._play_timer.setInterval(50)
        self._play_timer.timeout.connect(self._on_play_tick)

        self._setup_ui()
        self._setup_connections()

        self.setWindowTitle("Polygon Packing - Snapshot Viewer")
        self.resize(1200, 800)

    def _setup_ui(self):
        self._canvas = CanvasWidget()
        self._control = ControlPanel()
        self._terminal = TerminalWidget()

        # 水平分割器 (Canvas | ControlPanel)
        self._h_splitter = QSplitter(Qt.Horizontal)
        self._h_splitter.addWidget(self._canvas)
        self._h_splitter.addWidget(self._control)
        self._h_splitter.setStretchFactor(0, 4)
        self._h_splitter.setStretchFactor(1, 1)
        self._h_splitter.setSizes([800, 300])
        self._h_splitter.setHandleWidth(5)
        self._h_splitter.setStyleSheet(
            "QSplitter::handle { background-color: #555; }"
            "QSplitter::handle:hover { background-color: #888; }"
        )

        # 垂直分割器 (上部 | Terminal)
        self._v_splitter = QSplitter(Qt.Vertical)
        self._v_splitter.addWidget(self._h_splitter)
        self._v_splitter.addWidget(self._terminal)
        self._v_splitter.setStretchFactor(0, 3)
        self._v_splitter.setStretchFactor(1, 1)
        self._v_splitter.setSizes([600, 200])
        self._v_splitter.setHandleWidth(5)
        self._v_splitter.setCollapsible(0, False)
        self._v_splitter.setCollapsible(1, False)
        self._terminal.setMinimumHeight(200)
        self._v_splitter.setStyleSheet(
            "QSplitter::handle { background-color: #555; }"
            "QSplitter::handle:hover { background-color: #888; }"
        )

        self.setCentralWidget(self._v_splitter)

        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_bar.showMessage("就绪 - 请加载快照文件 (.snpst)")

    def _setup_connections(self):
        self._control.load_file_clicked.connect(self._on_load_file)
        self._control.view_changed.connect(self._on_view_changed)
        self._control.frame_slider_changed.connect(self._on_frame_slider)
        self._control.frame_jump_clicked.connect(self._on_frame_jump)
        self._control.frame_prev_clicked.connect(self._on_frame_prev)
        self._control.frame_next_clicked.connect(self._on_frame_next)
        self._control.frame_play_clicked.connect(self._on_play)
        self._control.frame_stop_clicked.connect(self._on_stop_play)

    # ========== 事件处理 ==========

    def _on_load_file(self, filepath: str):
        """加载 .snpst 快照文件"""
        try:
            snpst_data = parse_snpst_file(filepath)
            self._snpst_data = snpst_data
            self._frames = snpst_data.frames
            self._current_frame_idx = 1 if self._frames else 0

            # 计算可行域包围盒
            bpoly = snpst_data.boundary_poly
            l = min(v[0] for v in bpoly)
            r = max(v[0] for v in bpoly)
            u = max(v[1] for v in bpoly)
            d = min(v[1] for v in bpoly)
            boundary = (l, r, u, d)

            # 更新 Canvas（原始多边形 + 可行域边界）
            self._canvas.set_data(boundary, bpoly, snpst_data.polygons)
            self._canvas.reset_view()

            # 为 RESULT 模式准备最后一帧的平移向量
            if self._frames:
                self._translations = self._frames[-1].translations
                self._canvas.set_translations(self._translations)
                # set_translations 会切换到 RESULT 模式，切回 ORIGINAL
                self._canvas.switch_to_original()

            # 更新 Control
            n_polys = len(snpst_data.polygons)
            n_frames = len(self._frames)
            self._control.set_file_loaded(has_frames=n_frames > 0)
            self._control.set_file_stats(n_polys, n_frames)
            self._control.reset_frames()
            if n_frames > 0:
                self._control.set_frame_info(1, n_frames)

            # 日志
            self._terminal.append_info(f"加载文件: {filepath}")
            self._terminal.append_info(
                f"可行域: {len(bpoly)} 个顶点  "
                f"L={l:.2f} R={r:.2f} U={u:.2f} D={d:.2f}"
            )
            self._terminal.append_info(f"多边形数量: {n_polys}")
            for i, poly in enumerate(snpst_data.polygons):
                self._terminal.append_info(f"  多边形 {i+1}: {len(poly)} 个顶点")
            self._terminal.append_info(f"快照帧数: {n_frames}")

            self._status_bar.showMessage(
                f"已加载: {filepath} | {n_polys} 个多边形 | {n_frames} 帧"
            )

        except Exception as e:
            self._show_error(f"加载文件失败: {e}")
            self._terminal.append_error(f"加载文件失败: {e}")

    def _on_view_changed(self, mode: int):
        """切换视图（0=原始, 1=结果, 2=帧）"""
        if mode == 2:
            if self._frames:
                self._show_frame(self._current_frame_idx, keep_view=True)
            else:
                self._canvas.switch_to_frame()
        elif mode == 1:
            self._canvas.switch_to_result()
        elif mode == 0:
            self._canvas.switch_to_original()

    # ========== 帧管理方法 ==========

    def _on_frame_slider(self, target: int):
        if 1 <= target <= len(self._frames):
            self._current_frame_idx = target
            self._show_frame(self._current_frame_idx, keep_view=True)

    def _on_frame_jump(self, target: int):
        if 1 <= target <= len(self._frames):
            self._current_frame_idx = target
            self._show_frame(self._current_frame_idx, keep_view=True)

    def _on_frame_prev(self):
        if self._frames and self._current_frame_idx > 1:
            self._current_frame_idx -= 1
            self._show_frame(self._current_frame_idx, keep_view=True)

    def _on_frame_next(self):
        if self._frames and self._current_frame_idx < len(self._frames):
            self._current_frame_idx += 1
            self._show_frame(self._current_frame_idx, keep_view=True)

    # ========== 自动播放 ==========

    def _on_play(self):
        if not self._frames:
            return
        self._play_timer.start()
        self._control.set_playing_state(True)

    def _on_stop_play(self):
        self._play_timer.stop()
        self._control.set_playing_state(False)

    def _on_play_tick(self):
        if not self._frames:
            self._play_timer.stop()
            self._control.set_playing_state(False)
            return
        next_idx = self._current_frame_idx + 1
        if next_idx > len(self._frames):
            self._play_timer.stop()
            self._control.set_playing_state(False)
            self._terminal.append_info("⏹ 自动播放结束")
            return
        self._current_frame_idx = next_idx
        self._show_frame(self._current_frame_idx, keep_view=True)

    def _show_frame(self, index: int, keep_view: bool = False):
        """显示指定帧

        Args:
            index: 帧序号（1-based）
            keep_view: 是否保持当前视口位置和缩放
        """
        if not (1 <= index <= len(self._frames)):
            return
        if self._snpst_data is None:
            return

        frame_data = self._frames[index - 1]
        orig_polys = self._snpst_data.polygons

        # 将平移向量叠加到初始顶点
        translated = [
            [(x + dx, y + dy) for x, y in poly]
            for poly, (dx, dy) in zip(orig_polys, frame_data.translations)
        ]

        self._canvas.set_frame_polys(
            translated,
            validity=frame_data.validity,
            keep_view=keep_view
        )

        n_polys = len(orig_polys)
        valid_count = sum(frame_data.validity)
        self._control.set_frame_validity(valid_count, n_polys)
        self._control.set_frame_info(index, len(self._frames))
        self._control.set_view_mode(2)
        self._status_bar.showMessage(
            f"帧 {index} / {len(self._frames)} | 合法: {valid_count} / {n_polys}"
        )

    def _show_error(self, message: str):
        QMessageBox.critical(self, "错误", message)

    def closeEvent(self, event):
        self._play_timer.stop()
        event.accept()
