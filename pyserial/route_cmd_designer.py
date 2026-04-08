"""
路径命令可视化设计器（Tkinter）

功能：
1. 可视化 8x10 栅格地图（ROS 坐标视角：右下角为 (0,0)，x+ 向上，y+ 向左）
2. 鼠标拖拽绘制路径（仅允许四邻接）
3. 自动读取 C 代码中的单向边阻断与特殊路段计数补偿规则
4. 支持地图编辑：障碍开关、单边障碍编辑、地块涂色、十字路口小圆点涂色
5. 生成与车端 translate_route_cmd.c 一致的命令串
6. 支持意图尾巴与自定义动作附加

运行：
python pyserial/route_cmd_designer.py
"""

from __future__ import annotations

import re
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Dict, List, Optional, Tuple


MAP_X_COUNT = 8
MAP_Y_COUNT = 10
CELL_SIZE = 56
CANVAS_PAD = 30

# 画布列数对应 y 轴（左右），行数对应 x 轴（上下）
CANVAS_COLS = MAP_Y_COUNT
CANVAS_ROWS = MAP_X_COUNT

# 方向编码与 C 端保持一致
HEADING_X_PLUS = 0
HEADING_Y_PLUS = 1
HEADING_X_MINUS = 2
HEADING_Y_MINUS = 3

INTENT_TAIL_MAP = {
    0: "",
    1: "K",
    2: "K",
    3: "tfObd",
    4: "O",
}

INTENT_ITEMS = [
    ("0-无意图", 0),
    ("1-拿正方体", 1),
    ("2-拿圆环", 2),
    ("3-放圆环", 3),
    ("4-放正方体", 4),
]

YAW_ITEMS = [
    ("0° (X+)", HEADING_X_PLUS),
    ("90° (Y+)", HEADING_Y_PLUS),
    ("180° (X-)", HEADING_X_MINUS),
    ("-90° (Y-)", HEADING_Y_MINUS),
]

EDIT_MODE_ITEMS = [
    ("路径绘制", "path"),
    ("障碍开关", "obstacle"),
    ("单边障碍编辑", "edge"),
    ("地块涂色", "cell_paint"),
    ("十字路口点涂色", "cross_dot"),
    ("橡皮擦", "eraser"),
]

PALETTE_CLEAR = "清除"
CELL_PALETTE_ITEMS = [
    ("浅黄", "#fde68a"),
    ("浅绿", "#bbf7d0"),
    ("浅蓝", "#bfdbfe"),
    ("淡红", "#fecaca"),
    ("淡紫", "#ddd6fe"),
    (PALETTE_CLEAR, ""),
]

DOT_PALETTE_ITEMS = [
    ("红点", "#dc2626"),
    ("蓝点", "#2563eb"),
    ("绿点", "#16a34a"),
    ("橙点", "#ea580c"),
    ("黑点", "#111827"),
    (PALETTE_CLEAR, ""),
]


@dataclass
class ParseResult:
    blocked_edges: set[Tuple[int, int, int, int]]
    route_units: Dict[Tuple[int, int, int, int], int]
    obstacles: set[Tuple[int, int]]


class RouteCmdDesigner:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("路径命令可视化设计器")
        self.root.geometry("1880x930")

        self.project_root = Path(__file__).resolve().parents[1]
        self.astar_c = self.project_root / "Core" / "Application" / "astar.c"
        self.translate_c = self.project_root / "Core" / "Application" / "translate_route_cmd.c"

        parsed = self._parse_rules_from_c_files()
        self.blocked_edges = parsed.blocked_edges
        self.edge_units = parsed.route_units
        self.obstacles = parsed.obstacles

        self.start_point: Tuple[int, int] = (0, 0)
        self.path_points: List[Tuple[int, int]] = [self.start_point]
        self.dragging = False
        self.edge_edit_first: Optional[Tuple[int, int]] = None

        self.cell_paints: Dict[Tuple[int, int], str] = {}
        self.cross_dots: Dict[Tuple[int, int], str] = {}

        self._build_ui()
        self._refresh_all()

    # ------------------------------- UI -------------------------------

    def _build_ui(self) -> None:
        container = ttk.Frame(self.root, padding=10)
        container.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(container)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right = ttk.Frame(container)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(12, 0))

        canvas_w = CANVAS_COLS * CELL_SIZE + CANVAS_PAD * 2
        canvas_h = CANVAS_ROWS * CELL_SIZE + CANVAS_PAD * 2
        self.canvas = tk.Canvas(left, width=canvas_w, height=canvas_h, bg="#f5f7fa", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.canvas.bind("<ButtonPress-1>", self._on_mouse_down)
        self.canvas.bind("<B1-Motion>", self._on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_mouse_up)

        self.status_var = tk.StringVar(value="就绪")
        ttk.Label(left, textvariable=self.status_var, foreground="#1f6feb").pack(anchor=tk.W, pady=(8, 0))

        ctrl = ttk.LabelFrame(right, text="路径配置", padding=10)
        ctrl.pack(fill=tk.X)

        ttk.Label(ctrl, text="起点 X").grid(row=0, column=0, sticky=tk.W, padx=4, pady=4)
        self.start_x_var = tk.IntVar(value=0)
        ttk.Spinbox(ctrl, from_=0, to=MAP_X_COUNT - 1, textvariable=self.start_x_var, width=8).grid(
            row=0, column=1, sticky=tk.W, padx=4, pady=4
        )

        ttk.Label(ctrl, text="起点 Y").grid(row=1, column=0, sticky=tk.W, padx=4, pady=4)
        self.start_y_var = tk.IntVar(value=0)
        ttk.Spinbox(ctrl, from_=0, to=MAP_Y_COUNT - 1, textvariable=self.start_y_var, width=8).grid(
            row=1, column=1, sticky=tk.W, padx=4, pady=4
        )

        ttk.Label(ctrl, text="初始朝向").grid(row=2, column=0, sticky=tk.W, padx=4, pady=4)
        self.yaw_text_var = tk.StringVar(value=YAW_ITEMS[0][0])
        self.yaw_combo = ttk.Combobox(
            ctrl,
            textvariable=self.yaw_text_var,
            state="readonly",
            values=[label for label, _ in YAW_ITEMS],
            width=15,
        )
        self.yaw_combo.grid(row=2, column=1, sticky=tk.W, padx=4, pady=4)

        ttk.Label(ctrl, text="意图").grid(row=3, column=0, sticky=tk.W, padx=4, pady=4)
        self.intent_text_var = tk.StringVar(value=INTENT_ITEMS[0][0])
        self.intent_combo = ttk.Combobox(
            ctrl,
            textvariable=self.intent_text_var,
            state="readonly",
            values=[label for label, _ in INTENT_ITEMS],
            width=15,
        )
        self.intent_combo.grid(row=3, column=1, sticky=tk.W, padx=4, pady=4)

        ttk.Label(ctrl, text="自定义尾部动作").grid(row=4, column=0, sticky=tk.W, padx=4, pady=4)
        self.custom_tail_var = tk.StringVar(value="")
        ttk.Entry(ctrl, textvariable=self.custom_tail_var, width=18).grid(row=4, column=1, sticky=tk.W, padx=4, pady=4)

        ttk.Button(ctrl, text="应用起点并清空路径", command=self._apply_start).grid(
            row=5, column=0, columnspan=2, sticky=tk.EW, padx=4, pady=(8, 4)
        )
        ttk.Button(ctrl, text="撤销一步", command=self._undo_step).grid(
            row=6, column=0, sticky=tk.EW, padx=4, pady=4
        )
        ttk.Button(ctrl, text="清空路径", command=self._clear_path_keep_start).grid(
            row=6, column=1, sticky=tk.EW, padx=4, pady=4
        )

        action_box = ttk.LabelFrame(right, text="快捷动作按钮", padding=10)
        action_box.pack(fill=tk.X, pady=(10, 0))
        for idx, token in enumerate(["K", "O", "t", "d", "f", "b", "w", "L", "R", "A"]):
            ttk.Button(action_box, text=token, width=4, command=lambda c=token: self._append_tail_token(c)).grid(
                row=idx // 5,
                column=idx % 5,
                padx=3,
                pady=3,
                sticky=tk.EW,
            )

        edit_box = ttk.LabelFrame(right, text="地图编辑", padding=10)
        edit_box.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(edit_box, text="编辑模式").grid(row=0, column=0, sticky=tk.W, padx=4, pady=4)
        self.edit_mode_text_var = tk.StringVar(value=EDIT_MODE_ITEMS[0][0])
        self.edit_mode_combo = ttk.Combobox(
            edit_box,
            textvariable=self.edit_mode_text_var,
            state="readonly",
            values=[label for label, _ in EDIT_MODE_ITEMS],
            width=16,
        )
        self.edit_mode_combo.grid(row=0, column=1, sticky=tk.W, padx=4, pady=4)
        self.edit_mode_combo.bind("<<ComboboxSelected>>", self._on_edit_mode_changed)

        ttk.Label(edit_box, text="地块颜色").grid(row=1, column=0, sticky=tk.W, padx=4, pady=4)
        self.cell_color_text_var = tk.StringVar(value=CELL_PALETTE_ITEMS[0][0])
        self.cell_color_combo = ttk.Combobox(
            edit_box,
            textvariable=self.cell_color_text_var,
            state="readonly",
            values=[label for label, _ in CELL_PALETTE_ITEMS],
            width=16,
        )
        self.cell_color_combo.grid(row=1, column=1, sticky=tk.W, padx=4, pady=4)

        ttk.Label(edit_box, text="路口点颜色").grid(row=2, column=0, sticky=tk.W, padx=4, pady=4)
        self.dot_color_text_var = tk.StringVar(value=DOT_PALETTE_ITEMS[0][0])
        self.dot_color_combo = ttk.Combobox(
            edit_box,
            textvariable=self.dot_color_text_var,
            state="readonly",
            values=[label for label, _ in DOT_PALETTE_ITEMS],
            width=16,
        )
        self.dot_color_combo.grid(row=2, column=1, sticky=tk.W, padx=4, pady=4)

        ttk.Button(edit_box, text="清空单边障碍", command=self._clear_all_blocked_edges).grid(
            row=3, column=0, sticky=tk.EW, padx=4, pady=(8, 4)
        )
        ttk.Button(edit_box, text="清空地块颜色", command=self._clear_all_cell_paints).grid(
            row=3, column=1, sticky=tk.EW, padx=4, pady=(8, 4)
        )
        ttk.Button(edit_box, text="清空路口圆点", command=self._clear_all_cross_dots).grid(
            row=4, column=0, columnspan=2, sticky=tk.EW, padx=4, pady=(2, 2)
        )
        ttk.Label(
            edit_box,
            text="单边障碍编辑：先点起点，再点相邻终点。",
            foreground="#445",
        ).grid(row=5, column=0, columnspan=2, sticky=tk.W, padx=4, pady=(4, 0))

        output_box = ttk.LabelFrame(right, text="生成结果", padding=10)
        output_box.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        ttk.Label(output_box, text="路径命令(不含尾巴)").pack(anchor=tk.W)
        self.route_cmd_var = tk.StringVar(value="")
        ttk.Entry(output_box, textvariable=self.route_cmd_var, width=40).pack(fill=tk.X, pady=(2, 8))

        ttk.Label(output_box, text="最终命令(含意图与自定义尾巴)").pack(anchor=tk.W)
        self.final_cmd_var = tk.StringVar(value="")
        ttk.Entry(output_box, textvariable=self.final_cmd_var, width=40).pack(fill=tk.X, pady=(2, 8))

        btn_row = ttk.Frame(output_box)
        btn_row.pack(fill=tk.X)
        ttk.Button(btn_row, text="生成命令", command=self._generate_command).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(btn_row, text="复制最终命令", command=self._copy_final_cmd).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(btn_row, text="重载规则", command=self._reload_rules).pack(side=tk.LEFT)

        ttk.Label(output_box, text="路径点列表").pack(anchor=tk.W, pady=(10, 2))
        self.path_text = tk.Text(output_box, width=40, height=10, font=("Consolas", 10))
        self.path_text.pack(fill=tk.BOTH, expand=True)

        ttk.Label(
            output_box,
            text="提示：ROS 视角坐标，x+ 向上、y+ 向左；路径模式下拖拽仅允许上下左右一步。",
            foreground="#445",
        ).pack(anchor=tk.W, pady=(8, 0))

    # ---------------------------- Parse Rules ----------------------------

    def _parse_rules_from_c_files(self) -> ParseResult:
        blocked_edges: set[Tuple[int, int, int, int]] = set()
        route_units: Dict[Tuple[int, int, int, int], int] = {}
        obstacles: set[Tuple[int, int]] = set()

        if self.astar_c.exists():
            content = self.astar_c.read_text(encoding="utf-8", errors="ignore")

            blocked_pat = re.compile(
                r"AStar_SetEdgeBlockedTo\s*\(\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(\d+)\s*\)"
            )
            for m in blocked_pat.finditer(content):
                x1, y1, x2, y2, blocked = map(int, m.groups())
                if blocked == 1:
                    blocked_edges.add((x1, y1, x2, y2))

            obs_pat = re.compile(r"AStar_SetObstacle\s*\(\s*(-?\d+)\s*,\s*(-?\d+)\s*\)")
            for m in obs_pat.finditer(content):
                x, y = map(int, m.groups())
                if self._is_valid_grid(x, y):
                    obstacles.add((x, y))

            rect_pat = re.compile(
                r"AStar_SetObstacleRect\s*\(\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*\)"
            )
            for m in rect_pat.finditer(content):
                x1, y1, x2, y2 = map(int, m.groups())
                min_x, max_x = min(x1, x2), max(x1, x2)
                min_y, max_y = min(y1, y2), max(y1, y2)
                for x in range(min_x, max_x + 1):
                    for y in range(min_y, max_y + 1):
                        if self._is_valid_grid(x, y):
                            obstacles.add((x, y))

        if self.translate_c.exists():
            content = self.translate_c.read_text(encoding="utf-8", errors="ignore")
            unit_pat = re.compile(
                r"TranslateRouteCmd_AddEdgeRouteUnits\s*\(\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(\d+)\s*\)"
            )
            for m in unit_pat.finditer(content):
                x1, y1, x2, y2, units = map(int, m.groups())
                route_units[(x1, y1, x2, y2)] = units

        return ParseResult(blocked_edges=blocked_edges, route_units=route_units, obstacles=obstacles)

    # ---------------------------- Draw Helpers ----------------------------

    def _grid_to_canvas(self, x: int, y: int) -> Tuple[int, int, int, int]:
        # ROS 坐标视角：原点在右下角，x+ 向上，y+ 向左
        col = MAP_Y_COUNT - 1 - y
        row = MAP_X_COUNT - 1 - x

        x0 = CANVAS_PAD + col * CELL_SIZE
        y0 = CANVAS_PAD + row * CELL_SIZE
        x1 = x0 + CELL_SIZE
        y1 = y0 + CELL_SIZE
        return x0, y0, x1, y1

    def _canvas_to_grid(self, px: int, py: int) -> Optional[Tuple[int, int]]:
        gx = px - CANVAS_PAD
        gy = py - CANVAS_PAD
        if gx < 0 or gy < 0:
            return None

        col = gx // CELL_SIZE
        row = gy // CELL_SIZE
        if col < 0 or col >= MAP_Y_COUNT or row < 0 or row >= MAP_X_COUNT:
            return None

        x = MAP_X_COUNT - 1 - int(row)
        y = MAP_Y_COUNT - 1 - int(col)
        if not self._is_valid_grid(x, y):
            return None
        return x, y

    def _draw_map(self) -> None:
        self.canvas.delete("all")

        # 背景边框
        x0 = CANVAS_PAD
        y0 = CANVAS_PAD
        x1 = CANVAS_PAD + CANVAS_COLS * CELL_SIZE
        y1 = CANVAS_PAD + CANVAS_ROWS * CELL_SIZE
        self.canvas.create_rectangle(x0, y0, x1, y1, outline="#888", width=2)

        # 网格与坐标
        for x in range(MAP_X_COUNT):
            for y in range(MAP_Y_COUNT):
                c0x, c0y, c1x, c1y = self._grid_to_canvas(x, y)
                fill = "#f0f3f8"
                if (x, y) in self.obstacles:
                    fill = "#4b5563"
                if (x, y) in self.cell_paints:
                    fill = self.cell_paints[(x, y)]
                self.canvas.create_rectangle(c0x, c0y, c1x, c1y, fill=fill, outline="#c6cbd6")
                self.canvas.create_text(
                    (c0x + c1x) // 2,
                    (c0y + c1y) // 2,
                    text=f"{x},{y}",
                    fill=self._best_text_color(fill),
                    font=("Consolas", 9),
                )

        # 十字路口小圆点
        for (x, y), dot_color in self.cross_dots.items():
            if not self._is_valid_grid(x, y):
                continue
            cx, cy = self._cell_center(x, y)
            r = 6
            self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill=dot_color, outline="")

        # 单边障碍编辑起点高亮
        if self.edge_edit_first is not None and self._edit_mode_from_combo() == "edge":
            sx, sy = self.edge_edit_first
            if self._is_valid_grid(sx, sy):
                c0x, c0y, c1x, c1y = self._grid_to_canvas(sx, sy)
                self.canvas.create_rectangle(c0x + 2, c0y + 2, c1x - 2, c1y - 2, outline="#f97316", width=3)

        # 单向阻断边
        for (x1g, y1g, x2g, y2g) in self.blocked_edges:
            if not (self._is_valid_grid(x1g, y1g) and self._is_valid_grid(x2g, y2g)):
                continue
            c1 = self._cell_center(x1g, y1g)
            c2 = self._cell_center(x2g, y2g)
            self.canvas.create_line(c1[0], c1[1], c2[0], c2[1], fill="#d04e4e", width=3, arrow=tk.LAST)

        # 特殊路段计数
        for (x1g, y1g, x2g, y2g), units in self.edge_units.items():
            if not (self._is_valid_grid(x1g, y1g) and self._is_valid_grid(x2g, y2g)):
                continue
            if units == 1:
                continue
            c1 = self._cell_center(x1g, y1g)
            c2 = self._cell_center(x2g, y2g)
            mx = (c1[0] + c2[0]) / 2
            my = (c1[1] + c2[1]) / 2
            self.canvas.create_oval(mx - 10, my - 10, mx + 10, my + 10, fill="#1d4ed8", outline="")
            self.canvas.create_text(mx, my, text=str(units), fill="#fff", font=("Consolas", 9, "bold"))

        # 轴说明：x+ 向上，y+ 向左，原点右下
        self.canvas.create_line(x1 + 18, y1 - 4, x1 + 18, y1 - 76, fill="#111", width=2, arrow=tk.LAST)
        self.canvas.create_text(x1 + 18, y1 - 88, text="X+", fill="#111", anchor=tk.S, font=("Consolas", 10, "bold"))
        self.canvas.create_line(x1 - 4, y1 + 18, x1 - 76, y1 + 18, fill="#111", width=2, arrow=tk.LAST)
        self.canvas.create_text(x1 - 90, y1 + 18, text="Y+", fill="#111", anchor=tk.E, font=("Consolas", 10, "bold"))
        self.canvas.create_text(x1 - 4, y1 + 34, text="原点(0,0)", fill="#111", anchor=tk.E, font=("Consolas", 9))

    def _draw_path(self) -> None:
        if not self.path_points:
            return

        # 画线
        for idx in range(1, len(self.path_points)):
            x0, y0 = self._cell_center(*self.path_points[idx - 1])
            x1, y1 = self._cell_center(*self.path_points[idx])
            self.canvas.create_line(x0, y0, x1, y1, fill="#0891b2", width=5)

        # 画点
        for idx, (x, y) in enumerate(self.path_points):
            cx, cy = self._cell_center(x, y)
            r = 8
            color = "#15803d"
            if idx == 0:
                color = "#f59e0b"
            elif idx == len(self.path_points) - 1:
                color = "#dc2626"
            self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill=color, outline="white", width=1)
            self.canvas.create_text(cx, cy - 14, text=str(idx), fill="#111", font=("Consolas", 9, "bold"))

    def _cell_center(self, x: int, y: int) -> Tuple[int, int]:
        x0, y0, x1, y1 = self._grid_to_canvas(x, y)
        return ((x0 + x1) // 2, (y0 + y1) // 2)

    # ---------------------------- Interaction ----------------------------

    def _on_mouse_down(self, event: tk.Event) -> None:
        cell = self._canvas_to_grid(event.x, event.y)
        if cell is None:
            return

        mode = self._edit_mode_from_combo()
        if mode != "path":
            self.dragging = False
            self._handle_map_edit_click(cell)
            return

        self.dragging = True
        if not self.path_points:
            self.path_points = [self.start_point]

        self._try_append_cell(cell)

    def _on_mouse_drag(self, event: tk.Event) -> None:
        if not self.dragging or self._edit_mode_from_combo() != "path":
            return
        cell = self._canvas_to_grid(event.x, event.y)
        if cell is None:
            return
        self._try_append_cell(cell)

    def _on_mouse_up(self, _event: tk.Event) -> None:
        self.dragging = False

    def _try_append_cell(self, cell: Tuple[int, int]) -> None:
        if (cell in self.obstacles) and (cell != self.start_point):
            self.status_var.set(f"格子 {cell} 是障碍物，不能走")
            return

        if not self.path_points:
            self.path_points = [cell]
            self.start_point = cell
            self._refresh_all()
            return

        last = self.path_points[-1]
        if cell == last:
            return

        # 往回拖一格：撤销
        if len(self.path_points) >= 2 and cell == self.path_points[-2]:
            self.path_points.pop()
            self.status_var.set("回退一步")
            self._refresh_all()
            return

        dx = cell[0] - last[0]
        dy = cell[1] - last[1]
        if abs(dx) + abs(dy) != 1:
            self.status_var.set(f"{last} -> {cell} 不是相邻格，忽略")
            return

        if self._is_blocked_edge(last, cell):
            self.status_var.set(f"{last} -> {cell} 在 A* 中被单向封锁")
            return

        self.path_points.append(cell)
        self.status_var.set(f"追加节点 {cell}")
        self._refresh_all()

    def _handle_map_edit_click(self, cell: Tuple[int, int]) -> None:
        mode = self._edit_mode_from_combo()
        if mode == "obstacle":
            self._toggle_obstacle(cell)
            return
        if mode == "edge":
            self._toggle_blocked_edge(cell)
            return
        if mode == "cell_paint":
            self._paint_cell(cell)
            return
        if mode == "cross_dot":
            self._paint_cross_dot(cell)
            return
        if mode == "eraser":
            self._erase_cell_marks(cell)

    def _toggle_obstacle(self, cell: Tuple[int, int]) -> None:
        if cell in self.obstacles:
            self.obstacles.remove(cell)
            action = f"移除障碍 {cell}"
        else:
            if cell == self.start_point:
                self.status_var.set("起点不能设置为障碍")
                return
            self.obstacles.add(cell)
            action = f"添加障碍 {cell}"

        invalid = self._sync_path_after_map_change()
        self.status_var.set(action if not invalid else f"{action}；{invalid}")
        self._refresh_all()

    def _toggle_blocked_edge(self, cell: Tuple[int, int]) -> None:
        if self.edge_edit_first is None:
            self.edge_edit_first = cell
            self.status_var.set(f"单边障碍起点已选 {cell}，请点相邻终点")
            self._refresh_all()
            return

        first = self.edge_edit_first
        if cell == first:
            self.edge_edit_first = None
            self.status_var.set("已取消单边障碍编辑")
            self._refresh_all()
            return

        dx = cell[0] - first[0]
        dy = cell[1] - first[1]
        if abs(dx) + abs(dy) != 1:
            self.edge_edit_first = cell
            self.status_var.set(f"终点需与起点相邻，已改选新起点 {cell}")
            self._refresh_all()
            return

        edge = (first[0], first[1], cell[0], cell[1])
        if edge in self.blocked_edges:
            self.blocked_edges.remove(edge)
            action = f"已解除单边障碍 {first}->{cell}"
        else:
            self.blocked_edges.add(edge)
            action = f"已添加单边障碍 {first}->{cell}"

        self.edge_edit_first = None
        invalid = self._sync_path_after_map_change()
        self.status_var.set(action if not invalid else f"{action}；{invalid}")
        self._refresh_all()

    def _paint_cell(self, cell: Tuple[int, int]) -> None:
        color = self._palette_value_from_label(self.cell_color_text_var.get(), CELL_PALETTE_ITEMS)
        if color:
            self.cell_paints[cell] = color
            self.status_var.set(f"地块 {cell} 已涂色")
        else:
            self.cell_paints.pop(cell, None)
            self.status_var.set(f"地块 {cell} 颜色已清除")
        self._refresh_all()

    def _paint_cross_dot(self, cell: Tuple[int, int]) -> None:
        color = self._palette_value_from_label(self.dot_color_text_var.get(), DOT_PALETTE_ITEMS)
        if color:
            self.cross_dots[cell] = color
            self.status_var.set(f"十字路口点 {cell} 已涂色")
        else:
            self.cross_dots.pop(cell, None)
            self.status_var.set(f"十字路口点 {cell} 颜色已清除")
        self._refresh_all()

    def _erase_cell_marks(self, cell: Tuple[int, int]) -> None:
        removed_parts: List[str] = []

        if cell in self.cell_paints:
            self.cell_paints.pop(cell, None)
            removed_parts.append("地块颜色")

        if cell in self.cross_dots:
            self.cross_dots.pop(cell, None)
            removed_parts.append("路口圆点")

        if cell in self.obstacles and cell != self.start_point:
            self.obstacles.remove(cell)
            removed_parts.append("障碍")

        removed_edges = [
            edge
            for edge in self.blocked_edges
            if (edge[0], edge[1]) == cell or (edge[2], edge[3]) == cell
        ]
        if removed_edges:
            for edge in removed_edges:
                self.blocked_edges.remove(edge)
            removed_parts.append(f"单边障碍{len(removed_edges)}条")

        invalid = self._sync_path_after_map_change()

        if removed_parts:
            base = f"橡皮擦已清除 {cell} 的" + "、".join(removed_parts)
            self.status_var.set(base if not invalid else f"{base}；{invalid}")
        else:
            self.status_var.set(f"{cell} 无可清除内容")
        self._refresh_all()

    # ---------------------------- Commands ----------------------------

    def _generate_command(self) -> None:
        try:
            route_cmd = self._build_route_cmd_from_path()
        except ValueError as exc:
            messagebox.showerror("生成失败", str(exc))
            return

        intent_code = self._intent_code_from_combo()
        intent_tail = INTENT_TAIL_MAP.get(intent_code, "")
        custom_tail = self.custom_tail_var.get().strip()
        final_cmd = route_cmd + intent_tail + custom_tail

        self.route_cmd_var.set(route_cmd)
        self.final_cmd_var.set(final_cmd)
        self.status_var.set("命令生成成功")

    def _build_route_cmd_from_path(self) -> str:
        if len(self.path_points) < 2:
            return ""

        heading = self._heading_from_combo()
        cmd: List[str] = []
        i = 1

        while i < len(self.path_points):
            from_xy = self.path_points[i - 1]
            to_xy = self.path_points[i]
            seg_heading = self._step_to_heading(from_xy, to_xy)

            turn = self._turn_cmd(heading, seg_heading)
            if turn:
                cmd.append(turn)
            heading = seg_heading

            units = 0
            while i < len(self.path_points):
                a = self.path_points[i - 1]
                b = self.path_points[i]
                h = self._step_to_heading(a, b)
                if h != seg_heading:
                    break
                units += self.edge_units.get((a[0], a[1], b[0], b[1]), 1)
                i += 1

            if units <= 0:
                raise ValueError("存在累计前进计数为 0 的连续段，无法编码为 1~9 命令")

            while units >= 9:
                cmd.append("9")
                units -= 9
            if units > 0:
                cmd.append(str(units))

        return "".join(cmd)

    # ---------------------------- Button Actions ----------------------------

    def _apply_start(self) -> None:
        x = int(self.start_x_var.get())
        y = int(self.start_y_var.get())
        if not self._is_valid_grid(x, y):
            messagebox.showerror("参数错误", "起点坐标超出范围")
            return
        if (x, y) in self.obstacles:
            messagebox.showerror("参数错误", "起点落在障碍物上")
            return

        self.start_point = (x, y)
        self.path_points = [self.start_point]
        self.route_cmd_var.set("")
        self.final_cmd_var.set("")
        self.status_var.set(f"起点已设置为 {self.start_point}")
        self._refresh_all()

    def _undo_step(self) -> None:
        if len(self.path_points) > 1:
            self.path_points.pop()
            self.status_var.set("撤销成功")
            self._refresh_all()

    def _clear_path_keep_start(self) -> None:
        self.path_points = [self.start_point]
        self.route_cmd_var.set("")
        self.final_cmd_var.set("")
        self.status_var.set("路径已清空")
        self._refresh_all()

    def _append_tail_token(self, token: str) -> None:
        current = self.custom_tail_var.get()
        self.custom_tail_var.set(current + token)

    def _copy_final_cmd(self) -> None:
        text = self.final_cmd_var.get().strip()
        if not text:
            messagebox.showinfo("提示", "当前没有可复制的命令")
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.status_var.set("最终命令已复制到剪贴板")

    def _reload_rules(self) -> None:
        parsed = self._parse_rules_from_c_files()
        self.blocked_edges = parsed.blocked_edges
        self.edge_units = parsed.route_units
        self.obstacles = parsed.obstacles
        self.edge_edit_first = None

        invalid = self._sync_path_after_map_change()
        self.status_var.set("规则已从 C 文件重载")
        if invalid:
            self.status_var.set(f"规则已从 C 文件重载；{invalid}")
        self._refresh_all()

    def _clear_all_blocked_edges(self) -> None:
        self.blocked_edges.clear()
        self.edge_edit_first = None
        invalid = self._sync_path_after_map_change()
        self.status_var.set("已清空所有单边障碍" if not invalid else f"已清空所有单边障碍；{invalid}")
        self._refresh_all()

    def _clear_all_cell_paints(self) -> None:
        self.cell_paints.clear()
        self.status_var.set("已清空所有地块颜色")
        self._refresh_all()

    def _clear_all_cross_dots(self) -> None:
        self.cross_dots.clear()
        self.status_var.set("已清空所有十字路口圆点")
        self._refresh_all()

    def _on_edit_mode_changed(self, _event: Optional[tk.Event] = None) -> None:
        self.edge_edit_first = None
        self.status_var.set(f"编辑模式：{self.edit_mode_text_var.get()}")
        self._refresh_all()

    # ---------------------------- Utility ----------------------------

    def _refresh_all(self) -> None:
        self._draw_map()
        self._draw_path()

        self.path_text.delete("1.0", tk.END)
        self.path_text.insert(tk.END, "索引\t坐标\n")
        for i, p in enumerate(self.path_points):
            self.path_text.insert(tk.END, f"{i}\t{p}\n")

    @staticmethod
    def _is_valid_grid(x: int, y: int) -> bool:
        return 0 <= x < MAP_X_COUNT and 0 <= y < MAP_Y_COUNT

    def _edit_mode_from_combo(self) -> str:
        text = self.edit_mode_text_var.get()
        for label, value in EDIT_MODE_ITEMS:
            if label == text:
                return value
        return "path"

    @staticmethod
    def _palette_value_from_label(text: str, palette_items: List[Tuple[str, str]]) -> str:
        for label, value in palette_items:
            if label == text:
                return value
        return ""

    @staticmethod
    def _best_text_color(bg_hex: str) -> str:
        if not isinstance(bg_hex, str) or not bg_hex.startswith("#") or len(bg_hex) != 7:
            return "#111"
        try:
            r = int(bg_hex[1:3], 16)
            g = int(bg_hex[3:5], 16)
            b = int(bg_hex[5:7], 16)
        except ValueError:
            return "#111"

        # 使用亮度估计决定前景色，保证坐标数字可读
        luminance = 0.299 * r + 0.587 * g + 0.114 * b
        return "#111" if luminance >= 145 else "#fff"

    def _sync_path_after_map_change(self) -> str:
        if not self.path_points:
            return ""

        if self.start_point in self.obstacles:
            self.obstacles.discard(self.start_point)

        for i in range(1, len(self.path_points)):
            prev_cell = self.path_points[i - 1]
            curr_cell = self.path_points[i]
            if curr_cell in self.obstacles:
                self.path_points = [self.start_point]
                self.route_cmd_var.set("")
                self.final_cmd_var.set("")
                return f"路径经过障碍 {curr_cell}，已自动清空路径"
            if self._is_blocked_edge(prev_cell, curr_cell):
                self.path_points = [self.start_point]
                self.route_cmd_var.set("")
                self.final_cmd_var.set("")
                return f"路径边 {prev_cell}->{curr_cell} 被封锁，已自动清空路径"

        return ""

    def _is_blocked_edge(self, from_xy: Tuple[int, int], to_xy: Tuple[int, int]) -> bool:
        x1, y1 = from_xy
        x2, y2 = to_xy
        return (x1, y1, x2, y2) in self.blocked_edges

    @staticmethod
    def _step_to_heading(from_xy: Tuple[int, int], to_xy: Tuple[int, int]) -> int:
        dx = to_xy[0] - from_xy[0]
        dy = to_xy[1] - from_xy[1]
        if dx == 1 and dy == 0:
            return HEADING_X_PLUS
        if dx == 0 and dy == 1:
            return HEADING_Y_PLUS
        if dx == -1 and dy == 0:
            return HEADING_X_MINUS
        if dx == 0 and dy == -1:
            return HEADING_Y_MINUS
        raise ValueError(f"非法步进: {from_xy} -> {to_xy}")

    @staticmethod
    def _turn_cmd(current: int, target: int) -> str:
        diff = (target - current + 4) % 4
        if diff == 0:
            return ""
        if diff == 1:
            return "L"
        if diff == 3:
            return "R"
        return "A"

    def _heading_from_combo(self) -> int:
        text = self.yaw_text_var.get()
        for label, value in YAW_ITEMS:
            if label == text:
                return value
        return HEADING_X_PLUS

    def _intent_code_from_combo(self) -> int:
        text = self.intent_text_var.get()
        for label, value in INTENT_ITEMS:
            if label == text:
                return value
        return 0


def main() -> None:
    root = tk.Tk()
    app = RouteCmdDesigner(root)
    _ = app
    root.mainloop()


if __name__ == "__main__":
    main()
