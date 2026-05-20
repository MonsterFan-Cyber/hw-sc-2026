"""
多边形布局编辑器  v4
====================
用法：
    python polygon_editor.py [in文件] [out文件]
    不传参数时启动后弹窗选择文件。

依赖：
    pip install matplotlib shapely

操作说明：
    左键单击多边形        — 单选
    Ctrl+左键             — 多选 / 取消单个（Toggle）
    左键点击空白           — 取消所有选中
    右键单击已选中         — 进入拖拽（以点击位置为抓取点）
    右键单击未选中         — 改为单选并进入拖拽
    Ctrl+右键未选中        — 仅加入多选，不进入拖拽
    右键再次单击（拖拽中） — 放置，退出拖拽
    方向键                 — 移动选中多边形（步长=视图宽度×0.5%）
    Shift+方向键           — 大步移动（×10）
    Ctrl+滚轮              — 以鼠标为中心缩放视图
    中键拖拽               — 平移视图
    Ctrl+Z / Ctrl+Y        — 撤销 / 恢复
    尺寸 [Entry][▲][▼]    — 缩放选中多边形（步长 0.05，Enter 确认输入）
    旋转 [Entry][▲][▼]    — 旋转选中多边形（步长 1°，Enter 确认输入）
    导出按钮               — 弹窗输入文件名，导出当前布局为 .in 文件
    还原视图按钮           — 恢复到初始视角（100%）
"""

import sys
import tkinter as tk
from tkinter import filedialog, messagebox

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import Point, Polygon as ShapelyPolygon


# ═══════════════════════════ 工具函数 ════════════════════════════

def _build_colors(n: int):
    c1 = plt.cm.tab20(np.linspace(0, 1, 20))
    c2 = plt.cm.tab20b(np.linspace(0, 1, 20))
    palette = np.vstack([c1, c2])
    return [palette[i % len(palette)] for i in range(n)]


def parse_in_file(filepath: str):
    with open(filepath, 'r') as f:
        tokens = f.read().split()
    idx = 0
    boundary = tuple(float(tokens[idx + j]) for j in range(4)); idx += 4
    n = int(tokens[idx]); idx += 1
    polys = []
    for _ in range(n):
        nv = int(tokens[idx]); idx += 1
        v = np.array([[float(tokens[idx + k * 2]), float(tokens[idx + k * 2 + 1])]
                      for k in range(nv)])
        idx += nv * 2
        polys.append(v)
    return boundary, polys


def parse_out_file(filepath: str):
    with open(filepath, 'r') as f:
        tokens = f.read().split()
    idx = 0; n = int(tokens[idx]); idx += 1
    trans = []
    for _ in range(n):
        trans.append((float(tokens[idx]), float(tokens[idx + 1]))); idx += 2
    return trans


def _rotate_verts(verts: np.ndarray, delta_deg: float, center: np.ndarray) -> np.ndarray:
    """绕 center 旋转 delta_deg 度（逆时针为正）。"""
    theta = np.radians(delta_deg)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return center + (verts - center) @ R.T


def _scale_verts(verts: np.ndarray, factor: float, center: np.ndarray) -> np.ndarray:
    """绕 center 缩放 factor 倍。"""
    return center + (verts - center) * factor


# ═══════════════════════════ 主编辑器 ════════════════════════════

class PolygonEditor:
    # 方向键
    ARROW_STEP_RATIO = 0.005
    ARROW_SHIFT_MULT = 10
    ARROW_COMMIT_MS  = 400
    # 变换步长
    SCALE_STEP  = 0.05
    ROTATE_STEP = 1.0

    def __init__(self, root: tk.Tk, in_file: str, out_file: str):
        self.root = root
        self.root.title("多边形布局编辑器")

        # ── 数据 ──────────────────────────────────────────────────
        boundary, raw = parse_in_file(in_file)
        trans         = parse_out_file(out_file)
        self.boundary = boundary
        self.n        = len(raw)
        self.polys: list[np.ndarray] = [
            raw[i] + np.array(trans[i]) for i in range(self.n)
        ]
        # 累积变换状态（仅用于显示，不参与几何计算）
        self.poly_scale  = [1.0] * self.n   # 相对于初始平移后顶点的缩放倍数
        self.poly_rotate = [0.0] * self.n   # 相对于初始平移后顶点的累积旋转角（°）

        # ── 交互状态 ──────────────────────────────────────────────
        self.selected:  set[int] = set()
        self.ctrl_held: bool     = False
        self.drag_mode: bool     = False
        self.drag_anchor         = None
        self.drag_pre:  list     = []

        # ── 平移状态 ──────────────────────────────────────────────
        self.pan_active        = False
        self.pan_xlim          = None
        self.pan_ylim          = None
        self.pan_inv_transform = None
        self.pan_start_pixel   = None

        # ── 方向键防抖 ─────────────────────────────────────────────
        self._arrow_timer = None
        self._arrow_pre:  list      = None
        self._arrow_sel:  frozenset = None

        # ── 撤销 / 恢复 ───────────────────────────────────────────
        # 每条记录：list of (i, old_verts, new_verts,
        #                     old_scale, new_scale, old_rotate, new_rotate)
        self.undo_stack: list = []
        self.redo_stack: list = []

        # ── 颜色 ──────────────────────────────────────────────────
        self.colors = _build_colors(self.n)

        # ── 初始视图 ───────────────────────────────────────────────
        l, r, b, t = self.boundary
        m = max(r - l, t - b) * 0.04
        self.orig_xlim = (l - m, r + m)
        self.orig_ylim = (b - m, t + m)

        # ── 搭建 UI ───────────────────────────────────────────────
        self._build_ui()
        self.ax.set_xlim(self.orig_xlim)
        self.ax.set_ylim(self.orig_ylim)
        self._update_transform_display()
        self._redraw()
        self._update_overlap_info()

    # ════════════════════════ UI 搭建 ════════════════════════════

    def _build_ui(self):
        # ── 工具栏 ────────────────────────────────────────────────
        tb = tk.Frame(self.root, bd=1, relief=tk.RAISED)
        tb.pack(side=tk.TOP, fill=tk.X)

        # 文件 / 撤销恢复
        tk.Button(tb, text="📂  导出 .in",
                  command=self._export, padx=6
                  ).pack(side=tk.LEFT, padx=4, pady=3)
        tk.Button(tb, text="↩ 撤销",
                  command=self._undo, padx=6
                  ).pack(side=tk.LEFT, padx=2, pady=3)
        tk.Button(tb, text="↪ 恢复",
                  command=self._redo, padx=6
                  ).pack(side=tk.LEFT, padx=2, pady=3)

        self._sep(tb)

        # 视图控制
        tk.Button(tb, text="🔍 还原视图",
                  command=self._reset_view, padx=6
                  ).pack(side=tk.LEFT, padx=2, pady=3)
        self.zoom_var = tk.StringVar(value="缩放：100%")
        tk.Label(tb, textvariable=self.zoom_var,
                 font=("Consolas", 11, "bold"), fg="#0055cc", padx=6
                 ).pack(side=tk.LEFT)

        self._sep(tb)

        # ── 尺寸缩放控件 ───────────────────────────────────────────
        sc = tk.Frame(tb)
        sc.pack(side=tk.LEFT, padx=4, pady=2)

        tk.Label(sc, text="尺寸:", font=("", 9)
                 ).grid(row=0, column=0, rowspan=2, sticky='e', padx=(0, 2))

        self.scale_var = tk.StringVar(value="1.00")
        self.scale_entry = tk.Entry(
            sc, textvariable=self.scale_var,
            width=6, font=("Consolas", 9), state='disabled',
            justify='right'
        )
        self.scale_entry.grid(row=0, column=1, rowspan=2, padx=1)
        for ev in ('<Return>', '<KP_Enter>'):
            self.scale_entry.bind(ev, lambda e: self._on_scale_enter())

        tk.Button(sc, text="▲", command=lambda: self._scale_step(+1),
                  font=("", 7), width=2, pady=0, relief=tk.GROOVE
                  ).grid(row=0, column=2, sticky='s', padx=1)
        tk.Button(sc, text="▼", command=lambda: self._scale_step(-1),
                  font=("", 7), width=2, pady=0, relief=tk.GROOVE
                  ).grid(row=1, column=2, sticky='n', padx=1)

        self._sep(tb)

        # ── 旋转控件 ───────────────────────────────────────────────
        rc = tk.Frame(tb)
        rc.pack(side=tk.LEFT, padx=4, pady=2)

        tk.Label(rc, text="旋转:", font=("", 9)
                 ).grid(row=0, column=0, rowspan=2, sticky='e', padx=(0, 2))

        self.rotate_var = tk.StringVar(value="0.0")
        self.rotate_entry = tk.Entry(
            rc, textvariable=self.rotate_var,
            width=7, font=("Consolas", 9), state='disabled',
            justify='right'
        )
        self.rotate_entry.grid(row=0, column=1, rowspan=2, padx=1)
        for ev in ('<Return>', '<KP_Enter>'):
            self.rotate_entry.bind(ev, lambda e: self._on_rotate_enter())

        tk.Label(rc, text="°", font=("", 9)
                 ).grid(row=0, column=2, rowspan=2)
        tk.Button(rc, text="▲", command=lambda: self._rotate_step(+1),
                  font=("", 7), width=2, pady=0, relief=tk.GROOVE
                  ).grid(row=0, column=3, sticky='s', padx=1)
        tk.Button(rc, text="▼", command=lambda: self._rotate_step(-1),
                  font=("", 7), width=2, pady=0, relief=tk.GROOVE
                  ).grid(row=1, column=3, sticky='n', padx=1)

        # ── 状态栏 ────────────────────────────────────────────────
        self.status_var = tk.StringVar()
        self._set_status_idle()
        tk.Label(self.root, textvariable=self.status_var,
                 bd=1, relief=tk.SUNKEN, anchor=tk.W, fg="#333"
                 ).pack(side=tk.BOTTOM, fill=tk.X)

        # ── 重叠信息面板 ───────────────────────────────────────────
        self._build_info_panel()

        # ── Matplotlib 画布 ───────────────────────────────────────
        self.fig, self.ax = plt.subplots(figsize=(14, 8))
        self.fig.tight_layout(pad=1.5)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # ── 键盘绑定 ──────────────────────────────────────────────
        for k in ('<Control_L>', '<Control_R>'):
            self.root.bind(k, lambda e: self._set_ctrl(True))
        for k in ('<KeyRelease-Control_L>', '<KeyRelease-Control_R>'):
            self.root.bind(k, lambda e: self._set_ctrl(False))
        self.root.bind('<Control-z>', lambda e: self._undo())
        self.root.bind('<Control-Z>', lambda e: self._undo())
        self.root.bind('<Control-y>', lambda e: self._redo())
        self.root.bind('<Control-Y>', lambda e: self._redo())
        for k in ('<Up>', '<Down>', '<Left>', '<Right>',
                  '<Shift-Up>', '<Shift-Down>', '<Shift-Left>', '<Shift-Right>'):
            self.root.bind(k, self._on_arrow_key)

        # ── Matplotlib 事件 ───────────────────────────────────────
        self.canvas.mpl_connect('button_press_event',   self._on_press)
        self.canvas.mpl_connect('button_release_event', self._on_release)
        self.canvas.mpl_connect('motion_notify_event',  self._on_motion)
        self.canvas.mpl_connect('scroll_event',         self._on_scroll)

    @staticmethod
    def _sep(parent):
        tk.Frame(parent, width=2, bd=1, relief=tk.SUNKEN
                 ).pack(side=tk.LEFT, fill=tk.Y, padx=6, pady=2)

    # ════════════════════════ 重叠信息面板 ═══════════════════════

    def _build_info_panel(self):
        self.info_expanded = True
        self.info_outer = tk.Frame(self.root, bd=1, relief=tk.GROOVE)
        self.info_outer.pack(side=tk.BOTTOM, fill=tk.X)

        hdr = tk.Frame(self.info_outer, bg="#eaecf5")
        hdr.pack(side=tk.TOP, fill=tk.X)
        tk.Label(hdr, text="📊  重叠检测",
                 font=("", 9, "bold"), bg="#eaecf5", fg="#333399"
                 ).pack(side=tk.LEFT, padx=8, pady=2)
        self.toggle_btn = tk.Button(
            hdr, text="▼ 折叠", command=self._toggle_info,
            padx=4, pady=0, relief=tk.FLAT, bg="#eaecf5", font=("", 8))
        self.toggle_btn.pack(side=tk.RIGHT, padx=6)

        self.info_content = tk.Frame(self.info_outer, height=80)
        self.info_content.pack(side=tk.TOP, fill=tk.X)
        self.info_content.pack_propagate(False)

        self.info_text = tk.Text(
            self.info_content, font=("Consolas", 9),
            state=tk.DISABLED, wrap=tk.WORD,
            bg="#fafbff", bd=0, padx=8, pady=4)
        sb = tk.Scrollbar(self.info_content, command=self.info_text.yview)
        self.info_text.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.info_text.pack(fill=tk.BOTH, expand=True)

        self.info_text.tag_configure('head', foreground='#0044aa', font=("Consolas", 9, "bold"))
        self.info_text.tag_configure('ok',   foreground='#1a7a3a', font=("Consolas", 9))
        self.info_text.tag_configure('warn', foreground='#c0392b', font=("Consolas", 9, "bold"))
        self.info_text.tag_configure('info', foreground='#444',    font=("Consolas", 9))

    def _toggle_info(self):
        if self.info_expanded:
            self.info_content.pack_forget()
            self.toggle_btn.config(text="▶ 展开")
        else:
            self.info_content.pack(side=tk.TOP, fill=tk.X)
            self.toggle_btn.config(text="▼ 折叠")
        self.info_expanded = not self.info_expanded

    # ════════════════════ 变换控件辅助方法 ═══════════════════════

    def _get_uniform_scale(self):
        """若所有选中多边形缩放相同则返回该值，否则返回 None。"""
        if not self.selected:
            return 1.0
        vals = [self.poly_scale[i] for i in self.selected]
        return vals[0] if all(abs(v - vals[0]) < 1e-6 for v in vals) else None

    def _get_uniform_rotate(self):
        """若所有选中多边形旋转相同则返回该值，否则返回 None。"""
        if not self.selected:
            return 0.0
        vals = [self.poly_rotate[i] for i in self.selected]
        return vals[0] if all(abs(v - vals[0]) < 0.01 for v in vals) else None

    def _update_transform_display(self):
        """根据当前选中状态刷新尺寸/旋转输入框显示。"""
        if not self.selected:
            self.scale_var.set("1.00")
            self.rotate_var.set("0.0")
            self.scale_entry.config(state='disabled')
            self.rotate_entry.config(state='disabled')
            return

        self.scale_entry.config(state='normal')
        self.rotate_entry.config(state='normal')

        us = self._get_uniform_scale()
        ur = self._get_uniform_rotate()
        self.scale_var.set(f"{us:.2f}" if us is not None else "—")
        self.rotate_var.set(f"{ur:.1f}" if ur is not None else "—")

    # ── 尺寸缩放 ──────────────────────────────────────────────────

    def _scale_step(self, direction: int):
        """箭头按钮：对每个选中多边形的 poly_scale 加减 SCALE_STEP（加法）。"""
        if not self.selected:
            return

        undo_entry = []
        for i in sorted(self.selected):
            old_v   = self.polys[i].copy()
            old_sc  = self.poly_scale[i]
            old_rot = self.poly_rotate[i]

            new_sc = max(0.05, old_sc + direction * self.SCALE_STEP)
            factor = new_sc / old_sc if old_sc > 1e-9 else new_sc
            cen    = old_v.mean(axis=0)
            new_v  = _scale_verts(old_v, factor, cen)

            self.polys[i]       = new_v
            self.poly_scale[i]  = new_sc
            undo_entry.append((i, old_v, new_v, old_sc, new_sc, old_rot, old_rot))

        self._push_undo(undo_entry)
        self._update_transform_display()
        self._update_overlap_info()
        self._redraw()

    def _on_scale_enter(self):
        """Enter 键：解析输入值，将所有选中多边形缩放设为该绝对值。"""
        if not self.selected:
            return
        try:
            target = float(self.scale_var.get())
            if target <= 0:
                raise ValueError
        except ValueError:
            self._update_transform_display()
            return

        undo_entry = []
        for i in sorted(self.selected):
            old_v   = self.polys[i].copy()
            old_sc  = self.poly_scale[i]
            old_rot = self.poly_rotate[i]

            factor = target / old_sc if old_sc > 1e-9 else target
            cen    = old_v.mean(axis=0)
            new_v  = _scale_verts(old_v, factor, cen)

            self.polys[i]      = new_v
            self.poly_scale[i] = target
            undo_entry.append((i, old_v, new_v, old_sc, target, old_rot, old_rot))

        self._push_undo(undo_entry)
        self._update_transform_display()
        self._update_overlap_info()
        self._redraw()

    # ── 旋转 ──────────────────────────────────────────────────────

    def _rotate_step(self, direction: int):
        """箭头按钮：对每个选中多边形旋转 ±ROTATE_STEP 度。"""
        if not self.selected:
            return

        delta = direction * self.ROTATE_STEP
        undo_entry = []
        for i in sorted(self.selected):
            old_v   = self.polys[i].copy()
            old_sc  = self.poly_scale[i]
            old_rot = self.poly_rotate[i]

            cen   = old_v.mean(axis=0)
            new_v = _rotate_verts(old_v, delta, cen)
            new_rot = old_rot + delta

            self.polys[i]        = new_v
            self.poly_rotate[i]  = new_rot
            undo_entry.append((i, old_v, new_v, old_sc, old_sc, old_rot, new_rot))

        self._push_undo(undo_entry)
        self._update_transform_display()
        self._update_overlap_info()
        self._redraw()

    def _on_rotate_enter(self):
        """Enter 键：解析输入值，将所有选中多边形旋转设为该绝对角度。"""
        if not self.selected:
            return
        try:
            target = float(self.rotate_var.get())
        except ValueError:
            self._update_transform_display()
            return

        undo_entry = []
        for i in sorted(self.selected):
            old_v   = self.polys[i].copy()
            old_sc  = self.poly_scale[i]
            old_rot = self.poly_rotate[i]
            delta   = target - old_rot

            if abs(delta) < 1e-10:
                continue
            cen   = old_v.mean(axis=0)
            new_v = _rotate_verts(old_v, delta, cen)

            self.polys[i]       = new_v
            self.poly_rotate[i] = target
            undo_entry.append((i, old_v, new_v, old_sc, old_sc, old_rot, target))

        self._push_undo(undo_entry)
        self._update_transform_display()
        self._update_overlap_info()
        self._redraw()

    # ════════════════════════ 视图控制 ═══════════════════════════

    def _set_ctrl(self, val: bool):
        self.ctrl_held = val

    def _reset_view(self):
        self.ax.set_xlim(self.orig_xlim)
        self.ax.set_ylim(self.orig_ylim)
        self._update_zoom_label()
        self.canvas.draw_idle()

    def _update_zoom_label(self):
        ow = self.orig_xlim[1] - self.orig_xlim[0]
        cw = self.ax.get_xlim()[1] - self.ax.get_xlim()[0]
        pct = (ow / cw * 100) if cw > 0 else 100.0
        self.zoom_var.set(f"缩放：{pct:.0f}%")

    def _on_scroll(self, event):
        if not self.ctrl_held or event.inaxes != self.ax or event.xdata is None:
            return
        scale = 0.80 if event.button == 'up' else 1.25
        x, y  = event.xdata, event.ydata
        xl, yl = self.ax.get_xlim(), self.ax.get_ylim()
        self.ax.set_xlim(x + (xl[0] - x) * scale, x + (xl[1] - x) * scale)
        self.ax.set_ylim(y + (yl[0] - y) * scale, y + (yl[1] - y) * scale)
        self._update_zoom_label()
        self.canvas.draw_idle()

    # ════════════════════════ 方向键移动 ═════════════════════════

    def _on_arrow_key(self, event):
        # 焦点在 Entry/Text 时不触发多边形移动
        focused = self.root.focus_get()
        if isinstance(focused, (tk.Entry, tk.Text)):
            return
        if not self.selected or self.drag_mode:
            return

        ks = event.keysym
        direction = next((d for d in ('Up', 'Down', 'Left', 'Right')
                          if d.lower() in ks.lower()), None)
        if direction is None:
            return

        shift = bool(event.state & 0x0001)
        step  = (self.ax.get_xlim()[1] - self.ax.get_xlim()[0]) * self.ARROW_STEP_RATIO
        if shift:
            step *= self.ARROW_SHIFT_MULT

        dx, dy = {'Left': (-step, 0), 'Right': (step, 0),
                  'Up':   (0,  step), 'Down':  (0, -step)}[direction]

        if self._arrow_pre is None:
            self._arrow_pre = [p.copy() for p in self.polys]
            self._arrow_sel  = frozenset(self.selected)
        if self._arrow_timer is not None:
            self.root.after_cancel(self._arrow_timer)

        offset = np.array([dx, dy])
        for i in self.selected:
            self.polys[i] = self.polys[i] + offset

        self._arrow_timer = self.root.after(self.ARROW_COMMIT_MS, self._commit_arrow)
        self._update_overlap_info()
        self._redraw()

    def _commit_arrow(self):
        if self._arrow_pre is None:
            return
        entry = []
        for i in self._arrow_sel:
            old = self._arrow_pre[i]
            new = self.polys[i].copy()
            if not np.allclose(old, new):
                entry.append((i, old, new,
                               self.poly_scale[i], self.poly_scale[i],
                               self.poly_rotate[i], self.poly_rotate[i]))
        if entry:
            self._push_undo(entry)
        self._arrow_timer = self._arrow_pre = self._arrow_sel = None
        self._set_status_idle()

    def _force_commit_arrow(self):
        if self._arrow_timer is not None:
            self.root.after_cancel(self._arrow_timer)
            self._arrow_timer = None
        self._commit_arrow()

    # ════════════════════════ 鼠标事件 ═══════════════════════════

    def _on_press(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        if event.button == 1:
            self._left_click(event.xdata, event.ydata)
        elif event.button == 2:
            self.pan_active        = True
            self.pan_xlim          = self.ax.get_xlim()
            self.pan_ylim          = self.ax.get_ylim()
            self.pan_inv_transform = self.ax.transData.inverted()
            self.pan_start_pixel   = (event.x, event.y)
        elif event.button == 3:
            self._right_click(event.xdata, event.ydata)

    def _on_release(self, event):
        if event.button == 2:
            self.pan_active      = False
            self.pan_start_pixel = None

    def _on_motion(self, event):
        if self.drag_mode and event.inaxes == self.ax and event.xdata is not None:
            dx = event.xdata - self.drag_anchor[0]
            dy = event.ydata - self.drag_anchor[1]
            for i in self.selected:
                self.polys[i] = self.drag_pre[i] + np.array([dx, dy])
            self._redraw()
            return

        if self.pan_active and self.pan_start_pixel is not None:
            sd = self.pan_inv_transform.transform(self.pan_start_pixel)
            cd = self.pan_inv_transform.transform((event.x, event.y))
            dx, dy = cd[0] - sd[0], cd[1] - sd[1]
            self.ax.set_xlim(self.pan_xlim[0] - dx, self.pan_xlim[1] - dx)
            self.ax.set_ylim(self.pan_ylim[0] - dy, self.pan_ylim[1] - dy)
            self._update_zoom_label()
            self.canvas.draw_idle()

    # ════════════════════════ 命中检测 / 选中 ════════════════════

    def _hit_test(self, x: float, y: float):
        pt = Point(x, y)
        for i in range(self.n - 1, -1, -1):
            try:
                if ShapelyPolygon(self.polys[i]).contains(pt):
                    return i
            except Exception:
                pass
        return None

    def _left_click(self, x: float, y: float):
        if self.drag_mode:
            return
        self._force_commit_arrow()
        hit = self._hit_test(x, y)
        if hit is None:
            self.selected.clear()
        elif self.ctrl_held:
            self.selected.discard(hit) if hit in self.selected \
                else self.selected.add(hit)
        else:
            self.selected = {hit}
        self._set_status_idle()
        self._update_transform_display()
        self._update_overlap_info()
        self._redraw()

    def _right_click(self, x: float, y: float):
        if self.drag_mode:
            self._commit_drag(x, y)
            return
        hit = self._hit_test(x, y)
        if hit is None:
            return
        if self.ctrl_held:
            self._force_commit_arrow()
            self.selected.add(hit)
            self._set_status_idle()
            self._update_transform_display()
            self._update_overlap_info()
            self._redraw()
        else:
            if hit not in self.selected:
                self._force_commit_arrow()
                self.selected = {hit}
            self._start_drag(x, y)

    # ════════════════════════ 拖拽 ═══════════════════════════════

    def _start_drag(self, x: float, y: float):
        self.drag_mode   = True
        self.drag_anchor = (x, y)
        self.drag_pre    = [p.copy() for p in self.polys]
        self.status_var.set(
            f"🔵 拖拽模式 | 选中 {len(self.selected)} 个 | 右键放置")

    def _commit_drag(self, x: float, y: float):
        dx, dy = x - self.drag_anchor[0], y - self.drag_anchor[1]
        entry = []
        for i in self.selected:
            old = self.drag_pre[i].copy()
            new = (old + np.array([dx, dy])).copy()
            self.polys[i] = new
            entry.append((i, old, new,
                          self.poly_scale[i], self.poly_scale[i],
                          self.poly_rotate[i], self.poly_rotate[i]))
        self._push_undo(entry)
        self.drag_mode = False
        self.drag_anchor = None
        self.drag_pre = []
        self._set_status_idle()
        self._update_overlap_info()
        self._redraw()

    # ════════════════════════ 撤销 / 恢复 ════════════════════════

    def _push_undo(self, entry: list):
        if entry:
            self.undo_stack.append(entry)
            self.redo_stack.clear()

    def _undo(self):
        self._force_commit_arrow()
        if not self.undo_stack:
            self.status_var.set("⚠️  没有可撤销的操作"); return
        entry = self.undo_stack.pop()
        redo_entry = []
        for item in entry:
            i = item[0]
            self.polys[i] = item[1].copy()         # 恢复 old_verts
            if len(item) >= 7:
                self.poly_scale[i]  = item[3]      # 恢复 old_scale
                self.poly_rotate[i] = item[5]      # 恢复 old_rotate
            redo_entry.append(item)
        self.redo_stack.append(redo_entry)
        self._set_status_idle()
        self._update_transform_display()
        self._update_overlap_info()
        self._redraw()

    def _redo(self):
        self._force_commit_arrow()
        if not self.redo_stack:
            self.status_var.set("⚠️  没有可恢复的操作"); return
        entry = self.redo_stack.pop()
        undo_entry = []
        for item in entry:
            i = item[0]
            self.polys[i] = item[2].copy()         # 恢复 new_verts
            if len(item) >= 7:
                self.poly_scale[i]  = item[4]      # 恢复 new_scale
                self.poly_rotate[i] = item[6]      # 恢复 new_rotate
            undo_entry.append(item)
        self.undo_stack.append(undo_entry)
        self._set_status_idle()
        self._update_transform_display()
        self._update_overlap_info()
        self._redraw()

    # ════════════════════════ 重叠检测 ═══════════════════════════

    def _build_shapely(self) -> list:
        result = []
        for v in self.polys:
            try:
                p = ShapelyPolygon(v)
                if not p.is_valid:
                    p = p.buffer(0)
                result.append(p if p.is_valid else None)
            except Exception:
                result.append(None)
        return result

    def _compute_overlaps(self):
        if not self.selected:
            return set(), {}
        shapes  = self._build_shapely()
        sel_set = self.selected
        internal: set  = set()
        external: dict = {}
        for i in sorted(sel_set):
            pi = shapes[i]
            if pi is None:
                continue
            for j in range(self.n):
                if j == i:
                    continue
                pj = shapes[j]
                if pj is None:
                    continue
                try:
                    if pi.intersects(pj) and not pi.touches(pj):
                        if j in sel_set:
                            if i < j:
                                internal.add(frozenset({i, j}))
                        else:
                            external.setdefault(i, []).append(j)
                except Exception:
                    pass
        return internal, external

    def _update_overlap_info(self):
        self.info_text.config(state=tk.NORMAL)
        self.info_text.delete('1.0', tk.END)
        if not self.selected:
            self.info_text.insert(tk.END, "未选中任何多边形", 'info')
            self.info_text.config(state=tk.DISABLED)
            return

        internal, external = self._compute_overlaps()
        sel_sorted = sorted(self.selected)
        self.info_text.insert(tk.END,
            "已选中：" + "、".join(f"{i+1}号" for i in sel_sorted) + "\n", 'head')

        if not internal and not external:
            self.info_text.insert(tk.END, "✅ 无重叠\n", 'ok')
        else:
            if internal:
                pairs = "  ".join(
                    f"{min(a,b)+1}↔{max(a,b)+1}"
                    for a, b in sorted(sorted(p) for p in internal))
                self.info_text.insert(tk.END,
                    f"⚠️ 内部重叠（选中组内）：{pairs}\n", 'warn')
            if external:
                parts = [f"{i+1}号→[{', '.join(str(j+1) for j in sorted(js))}]"
                         for i, js in sorted(external.items())]
                self.info_text.insert(tk.END,
                    f"⚠️ 外部重叠（与未选中）：{'；'.join(parts)}\n", 'warn')
            if len(self.selected) > 1:
                involved = {idx for pair in internal for idx in pair}
                no_ov = [i for i in sel_sorted if i not in involved and i not in external]
                if no_ov:
                    self.info_text.insert(tk.END,
                        "✅ 无重叠多边形：" + "、".join(f"{i+1}号" for i in no_ov) + "\n", 'ok')

        self.info_text.config(state=tk.DISABLED)

    # ════════════════════════ 导出 ═══════════════════════════════

    def _export(self):
        fp = filedialog.asksaveasfilename(
            defaultextension=".in",
            filetypes=[("Input files", "*.in"), ("All files", "*.*")],
            title="导出 .in 文件")
        if not fp:
            return
        try:
            l, r, b, t = self.boundary
            with open(fp, 'w') as f:
                f.write(f"{l:.5f} {r:.5f} {b:.5f} {t:.5f}\n{self.n}\n")
                for poly in self.polys:
                    f.write(f"{len(poly)}\n")
                    f.write(" ".join(f"{x:.5f} {y:.5f}" for x, y in poly) + "\n")
            messagebox.showinfo("导出成功", f"已保存至：\n{fp}")
        except Exception as e:
            messagebox.showerror("导出失败", str(e))

    # ════════════════════════ 状态栏 ═════════════════════════════

    def _set_status_idle(self):
        n  = len(self.selected)
        un = len(self.undo_stack)
        rn = len(self.redo_stack)
        tip = (f"✅ 已选中 {n} 个 | 右键拖拽 · 方向键 · 尺寸/旋转变换 | "
               if n else
               "就绪 | 左键选中 · Ctrl+左键多选 · 右键拖拽 · 方向键移动 | ")
        self.status_var.set(
            tip + f"Ctrl+Z ({un}) | Ctrl+Y ({rn}) | Ctrl+滚轮缩放 | 中键平移")

    # ════════════════════════ 绘制 ═══════════════════════════════

    def _redraw(self):
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        self.ax.clear()

        l, r, b, t = self.boundary
        self.ax.add_patch(plt.Rectangle(
            (l, b), r - l, t - b,
            fill=False, edgecolor='#222', linestyle='--', linewidth=2, zorder=0))

        draw_order = sorted(range(self.n),
                            key=lambda i: 1 if i in self.selected else 0)
        for i in draw_order:
            v  = self.polys[i]
            c  = self.colors[i]
            sel = i in self.selected
            self.ax.add_patch(MplPolygon(
                v, closed=True,
                facecolor=(*c[:3], 0.40),
                edgecolor='#e63030' if sel else (*c[:3], 0.90),
                linewidth=2.5 if sel else 0.8,
                zorder=2 if sel else 1))
            cx, cy = v[:, 0].mean(), v[:, 1].mean()
            self.ax.text(cx, cy, str(i + 1),
                         fontsize=6, ha='center', va='center',
                         color='#111', zorder=3,
                         fontweight='bold' if sel else 'normal')

        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        self.ax.set_aspect('equal', adjustable='datalim')
        self.ax.grid(True, alpha=0.25, linewidth=0.5)
        self.ax.set_title(f"多边形布局编辑器  |  共 {self.n} 个多边形",
                          fontsize=10, pad=6)
        self._update_zoom_label()
        self.canvas.draw_idle()


# ═══════════════════════════ 入口 ════════════════════════════════

def main():
    root = tk.Tk()
    root.geometry("1280x860")

    if len(sys.argv) >= 3:
        in_file, out_file = sys.argv[1], sys.argv[2]
    else:
        in_file = filedialog.askopenfilename(
            title="选择 .in 文件",
            filetypes=[("Input files", "*.in"), ("All files", "*.*")])
        if not in_file:
            root.destroy(); return
        out_file = filedialog.askopenfilename(
            title="选择 .out 文件",
            filetypes=[("Output files", "*.out"), ("All files", "*.*")])
        if not out_file:
            root.destroy(); return

    try:
        PolygonEditor(root, in_file, out_file)
    except Exception as e:
        messagebox.showerror("加载失败", str(e))
        root.destroy()
        return

    root.mainloop()


if __name__ == "__main__":
    main()
