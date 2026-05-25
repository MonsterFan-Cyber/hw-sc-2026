"""终端消息组件 - 底部日志显示区域"""

import re
from datetime import datetime

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QTextEdit, QHBoxLayout,
    QPushButton, QLabel, QCheckBox
)
from PyQt5.QtCore import Qt


class TerminalWidget(QWidget):
    """底部终端/日志显示区域
    
    功能:
        - 显示来自求解器 stderr 的日志
        - 显示求解状态消息
        - 支持清空
        - VSCode C++ 风格着色：数字用浅绿色
        - 自动滚动开关
    """
    
    MAX_LINES = 5000  # 最大显示行数
    
    # VSCode C++ 配色
    COLOR_LOG      = '#d4d4d4'  # 普通文本
    COLOR_INFO     = '#569cd6'  # 关键字蓝
    COLOR_ERROR    = '#f44747'  # 错误红
    COLOR_RESULT   = '#6a9955'  # 注释绿
    COLOR_ITER     = '#ce9178'  # 字符串橙
    COLOR_NUMBER   = '#b5cea8'  # 数字浅绿
    
    # 匹配整数和浮点数（包括科学计数法）
    _RE_NUMBER = re.compile(r'(?<![\w.])-?\d+\.?\d*(?:e[+\-]?\d+)?(?![\w])', re.IGNORECASE)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._auto_scroll = True
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        
        # 标题栏
        header = QHBoxLayout()
        title = QLabel("求解日志")
        title.setStyleSheet("font-weight: bold; font-size: 18px;")
        header.addWidget(title)
        header.addStretch()
        
        # 自动滚动开关
        self._chk_auto_scroll = QCheckBox("自动滚动")
        self._chk_auto_scroll.setChecked(True)
        self._chk_auto_scroll.setStyleSheet("color: #ccc;")
        self._chk_auto_scroll.stateChanged.connect(self._on_auto_scroll_changed)
        header.addWidget(self._chk_auto_scroll)
        
        self._btn_clear = QPushButton("清空")
        self._btn_clear.setFixedSize(60, 24)
        self._btn_clear.clicked.connect(self.clear_log)
        header.addWidget(self._btn_clear)
        
        layout.addLayout(header)
        
        # 日志文本框
        self._text = QTextEdit()
        self._text.setReadOnly(True)
        self._text.setMinimumHeight(100)  # 功能1：终端最小高度
        self._text.setStyleSheet("""
            QTextEdit {
                font-family: "Cascadia Code", "Consolas", "Courier New", monospace;
                font-size: 14px;
                background-color: #1e1e1e;
                color: #d4d4d4;
                border: 1px solid #555;
                padding: 6px 10px;
            }
        """)
        self._text.document().setMaximumBlockCount(self.MAX_LINES)
        layout.addWidget(self._text)
    
    def _on_auto_scroll_changed(self, state):
        """自动滚动开关状态变化"""
        self._auto_scroll = (state == Qt.Checked)
        if self._auto_scroll:
            self._scroll_to_bottom()
    
    def _scroll_to_bottom(self):
        """滚动到底部"""
        sb = self._text.verticalScrollBar()
        sb.setValue(sb.maximum())
    
    def _now(self) -> str:
        """当前时间戳 HH:MM:SS"""
        return datetime.now().strftime('%H:%M:%S')
    
    def _colorize_numbers(self, text: str) -> str:
        """将文本中的数字用 VSCode 数字颜色包装
        
        已转义 HTML 实体的文本可能会干扰正则，这里假定输入不含 HTML。
        """
        def repl(m):
            return f'<span style="color: {self.COLOR_NUMBER};">{m.group(0)}</span>'
        return self._RE_NUMBER.sub(repl, text)
    
    def _build_line(self, color: str, label: str, message: str) -> str:
        """构件一行 HTML，带时间戳和数字着色"""
        ts = self._now()
        colored_msg = self._colorize_numbers(message)
        return (
            f'<span style="color: #888;">[{ts}]</span> '
            f'<span style="color: {color};">{label}{colored_msg}</span>'
        )
    
    def append_log(self, message: str):
        """添加一条普通日志"""
        ts = self._now()
        colored_msg = self._colorize_numbers(message)
        self._text.append(
            f'<span style="color: #888;">[{ts}]</span> '
            f'<span style="color: {self.COLOR_LOG};">{colored_msg}</span>'
        )
        if self._auto_scroll:
            self._scroll_to_bottom()
    
    def append_info(self, message: str):
        """添加一条信息消息（蓝色）"""
        self._text.append(self._build_line(self.COLOR_INFO, '[信息] ', message))
        if self._auto_scroll:
            self._scroll_to_bottom()
    
    def append_error(self, message: str):
        """添加一条错误消息（红色）"""
        self._text.append(self._build_line(self.COLOR_ERROR, '[错误] ', message))
        if self._auto_scroll:
            self._scroll_to_bottom()
    
    def append_result(self, message: str):
        """添加一条结果消息（绿色）"""
        self._text.append(self._build_line(self.COLOR_RESULT, '', message))
        if self._auto_scroll:
            self._scroll_to_bottom()
    
    def append_iteration(self, iteration: int, embed_length: float, move_distance: float):
        """添加一条迭代信息（橙色）"""
        msg = f"迭代 {iteration}: 嵌入长度={embed_length:.6f}, 移动距离={move_distance:.6f}"
        self._text.append(self._build_line(self.COLOR_ITER, '', msg))
        if self._auto_scroll:
            self._scroll_to_bottom()
    
    def clear_log(self):
        """清空日志"""
        self._text.clear()
    
    def get_text(self) -> str:
        return self._text.toPlainText()