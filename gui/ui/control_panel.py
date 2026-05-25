"""控制面板 - 功能按钮和状态显示"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QPushButton, QLabel,
    QGroupBox, QFileDialog, QRadioButton, QHBoxLayout,
    QSpinBox, QButtonGroup, QSlider
)
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QFont


class ControlPanel(QWidget):
    """右上控制面板

    包含:
        - 加载快照文件按钮
        - 切换显示视图
        - 帧导航
        - 状态信息显示
    """

    # 信号
    load_file_clicked = pyqtSignal(str)      # 文件路径
    view_changed = pyqtSignal(int)           # 0=原始, 1=结果, 2=帧
    frame_slider_changed = pyqtSignal(int)   # 拖动条松手时的帧值（1-based）
    frame_jump_clicked = pyqtSignal(int)     # 跳转到指定帧（SpinBox跳转）
    frame_prev_clicked = pyqtSignal()        # 上一帧
    frame_next_clicked = pyqtSignal()        # 下一帧
    frame_play_clicked = pyqtSignal()        # 自动播放
    frame_stop_clicked = pyqtSignal()        # 停止播放

    def __init__(self, parent=None):
        super().__init__(parent)

        self._file_loaded = False
        self._playing = False
        self._slider_dragging = False
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        # ---- 文件操作 ----
        file_group = QGroupBox("文件操作")
        file_layout = QVBoxLayout(file_group)

        self._btn_load = QPushButton("加载快照文件")
        self._btn_load.clicked.connect(self._on_load_file)
        file_layout.addWidget(self._btn_load)

        self._lbl_file = QLabel("未加载文件")
        self._lbl_file.setWordWrap(True)
        self._lbl_file.setStyleSheet("color: gray;")
        font = self._lbl_file.font()
        font.setPointSize(9)
        self._lbl_file.setFont(font)
        file_layout.addWidget(self._lbl_file)

        layout.addWidget(file_group)

        # ---- 显示控制（三态互斥） ----
        view_group = QGroupBox("视图")
        view_layout = QVBoxLayout(view_group)

        self._view_group = QButtonGroup(self)
        self._view_group.setExclusive(True)

        self._radio_original = QRadioButton("原始数据")
        self._radio_result = QRadioButton("求解结果")
        self._radio_frame = QRadioButton("当前帧")

        self._radio_original.setEnabled(False)
        self._radio_result.setEnabled(False)
        self._radio_frame.setEnabled(False)

        self._view_group.addButton(self._radio_original, 0)
        self._view_group.addButton(self._radio_result, 1)
        self._view_group.addButton(self._radio_frame, 2)
        self._view_group.buttonClicked[int].connect(self._on_view_changed)

        view_layout.addWidget(self._radio_original)
        view_layout.addWidget(self._radio_result)
        view_layout.addWidget(self._radio_frame)

        layout.addWidget(view_group)

        # ---- 帧导航 (拖动条 + 跳转) ----
        frame_group = QGroupBox("帧导航")
        frame_layout = QVBoxLayout(frame_group)

        # 帧序号显示
        self._lbl_frame_info = QLabel("帧: 0 / 0")
        self._lbl_frame_info.setStyleSheet("font-weight: bold;")
        font = self._lbl_frame_info.font()
        font.setPointSize(11)
        self._lbl_frame_info.setFont(font)
        frame_layout.addWidget(self._lbl_frame_info)

        # 拖动条（水平滑块，1-based）
        self._frame_slider = QSlider(Qt.Horizontal)
        self._frame_slider.setMinimum(1)
        self._frame_slider.setMaximum(1)
        self._frame_slider.setValue(1)
        self._frame_slider.setEnabled(False)
        self._frame_slider.sliderPressed.connect(self._on_slider_pressed)
        self._frame_slider.sliderReleased.connect(self._on_slider_released)
        self._frame_slider.valueChanged.connect(self._on_slider_value_changed)
        frame_layout.addWidget(self._frame_slider)

        # 跳转控件
        jump_layout = QHBoxLayout()
        jump_layout.addWidget(QLabel("跳转到:"))
        self._spin_frame_jump = QSpinBox()
        self._spin_frame_jump.setMinimum(1)
        self._spin_frame_jump.setMaximum(1)
        self._spin_frame_jump.setValue(1)
        self._spin_frame_jump.setEnabled(False)
        jump_layout.addWidget(self._spin_frame_jump)

        self._btn_frame_jump = QPushButton("跳转")
        self._btn_frame_jump.clicked.connect(self._on_frame_jump)
        self._btn_frame_jump.setEnabled(False)
        jump_layout.addWidget(self._btn_frame_jump)
        frame_layout.addLayout(jump_layout)

        # 上一帧 / 下一帧按钮
        nav_layout = QHBoxLayout()
        self._btn_prev = QPushButton("◀ 上一帧")
        self._btn_prev.clicked.connect(self._on_prev)
        self._btn_prev.setEnabled(False)
        nav_layout.addWidget(self._btn_prev)

        self._btn_next = QPushButton("下一帧 ▶")
        self._btn_next.clicked.connect(self._on_next)
        self._btn_next.setEnabled(False)
        nav_layout.addWidget(self._btn_next)
        frame_layout.addLayout(nav_layout)

        # 自动播放按钮
        play_layout = QHBoxLayout()
        self._btn_play = QPushButton("▶ 播放")
        self._btn_play.clicked.connect(self._on_play)
        self._btn_play.setEnabled(False)
        play_layout.addWidget(self._btn_play)

        self._btn_stop_play = QPushButton("⏹ 停止")
        self._btn_stop_play.clicked.connect(self._on_stop_play)
        self._btn_stop_play.setEnabled(False)
        play_layout.addWidget(self._btn_stop_play)
        frame_layout.addLayout(play_layout)

        layout.addWidget(frame_group)

        # ---- 状态信息 ----
        status_group = QGroupBox("状态")
        status_layout = QVBoxLayout(status_group)

        self._lbl_best = QLabel("多边形总数: --")
        self._lbl_plen = QLabel("快照帧数: --")
        self._lbl_mlen = QLabel("当前帧合法: --")

        status_layout.addWidget(self._lbl_best)
        status_layout.addWidget(self._lbl_plen)
        status_layout.addWidget(self._lbl_mlen)

        layout.addWidget(status_group)

        # 弹性空间
        layout.addStretch()

    def _on_load_file(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self, "选择快照文件",
            ".", "快照文件 (*.snpst);;所有文件 (*.*)"
        )
        if filepath:
            self._file_loaded = True
            self._lbl_file.setText(filepath)
            self._lbl_file.setStyleSheet("color: black;")
            font = self._lbl_file.font()
            font.setPointSize(9)
            self._lbl_file.setFont(font)
            self.load_file_clicked.emit(filepath)

    def _on_view_changed(self, mode: int):
        self.view_changed.emit(mode)

    def set_view_mode(self, mode: int):
        """外部设置视图模式（不触发信号）"""
        radio = self._view_group.button(mode)
        if radio:
            radio.blockSignals(True)
            radio.setChecked(True)
            radio.blockSignals(False)

    def set_file_loaded(self, has_frames: bool):
        """文件加载后启用视图控件"""
        self._radio_original.setEnabled(True)
        self._radio_result.setEnabled(has_frames)
        self._radio_frame.setEnabled(has_frames)
        self._radio_original.setChecked(True)

    def set_file_stats(self, poly_count: int, frame_count: int):
        """更新文件统计信息"""
        self._lbl_best.setText(f"多边形总数: {poly_count}")
        self._lbl_plen.setText(f"快照帧数: {frame_count}")
        self._lbl_mlen.setText("当前帧合法: --")

    def set_frame_validity(self, valid_count: int, total: int):
        """更新当前帧合法多边形数"""
        self._lbl_mlen.setText(f"当前帧合法: {valid_count} / {total}")

    # ========== 帧导航方法 (拖动条) ==========

    def _on_slider_pressed(self):
        self._slider_dragging = True

    def _on_slider_released(self):
        self._slider_dragging = False
        target = self._frame_slider.value()
        self.frame_slider_changed.emit(target)

    def _on_slider_value_changed(self, value: int):
        if self._slider_dragging:
            total = self._frame_slider.maximum()
            self._lbl_frame_info.setText(f"帧: {value} / {total}")

    def _on_frame_jump(self):
        target = self._spin_frame_jump.value()
        self.frame_jump_clicked.emit(target)

    # ========== 上一帧 / 下一帧 ==========

    def _on_prev(self):
        self.frame_prev_clicked.emit()

    def _on_next(self):
        self.frame_next_clicked.emit()

    # ========== 自动播放 ==========

    def _on_play(self):
        self.frame_play_clicked.emit()

    def _on_stop_play(self):
        self.frame_stop_clicked.emit()

    def set_playing_state(self, playing: bool):
        """更新自动播放状态"""
        self._playing = playing
        self._btn_play.setEnabled(not playing and self._frame_slider.maximum() > 0)
        self._btn_stop_play.setEnabled(playing)
        self._frame_slider.setEnabled(not playing and self._frame_slider.maximum() > 0)
        self._spin_frame_jump.setEnabled(not playing and self._spin_frame_jump.maximum() > 0)
        self._btn_frame_jump.setEnabled(not playing and self._spin_frame_jump.maximum() > 0)

    def set_frame_info(self, current: int, total: int):
        """更新帧导航信息

        Args:
            current: 当前帧序号（1-based），0 表示无帧
            total: 总帧数
        """
        self._lbl_frame_info.setText(f"帧: {current} / {total}")

        has_frames = total > 0

        self._frame_slider.setEnabled(has_frames and not self._playing)
        self._frame_slider.setMaximum(max(1, total))
        if not self._slider_dragging:
            self._frame_slider.blockSignals(True)
            self._frame_slider.setValue(current if current > 0 else 1)
            self._frame_slider.blockSignals(False)

        self._spin_frame_jump.setEnabled(has_frames and not self._playing)
        self._spin_frame_jump.setMaximum(max(1, total))
        self._spin_frame_jump.setValue(current if current > 0 else 1)
        self._btn_frame_jump.setEnabled(has_frames and not self._playing)

        self._btn_play.setEnabled(has_frames and not self._playing)
        self._btn_prev.setEnabled(has_frames and not self._playing and current > 1)
        self._btn_next.setEnabled(has_frames and not self._playing and current < total)

    def reset_frames(self):
        """重置帧导航"""
        self._slider_dragging = False
        self._playing = False
        self.set_frame_info(0, 0)
