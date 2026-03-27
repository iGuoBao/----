"""
栅格地图编辑器
- 严格遵守ROS坐标系：右下角是(0,0)，x轴向上，y轴向左
- 48x48的黑白栅格地图
- 默认白色，黑色表示障碍物
- 点击格子切换黑白
- 生成AStar_SetObstacle()函数调用
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, simpledialog
import numpy as np


class GridMapEditor:
    def __init__(self, root, grid_size=None, grid_rows=None, grid_cols=None):
        self.root = root
        # 支持两种用法：grid_size（正方形）或 grid_rows/grid_cols（矩形）
        # 固定为40x32格，100mm每格
        self.grid_rows = 32
        self.grid_cols = 40
        self.root.title(f"栅格地图编辑器 (32x40) - ROS坐标系")
        self.cell_size = 15  # 每个格子的像素大小，适配100mm格子
        # 初始化地图数组 (0=白色/可通行, 1=黑色/障碍物)
        self.grid_map = np.zeros((self.grid_rows, self.grid_cols), dtype=int)
        
        # 初始化颜色数组 (0=白色, 1=红色, 2=蓝色) - 涂改颜色，不影响通行性
        self.color_map = np.zeros((self.grid_rows, self.grid_cols), dtype=int)
        
        # 初始化得分块数组 (0=无得分块, 1=黄色, 2=绿色) - 得分块，可通行
        self.score_map = np.zeros((self.grid_rows, self.grid_cols), dtype=int)
        
        # 涂改模式相关
        self.painting_mode = False
        self.paint_color = 1  # 1=红色, 2=蓝色
        
        # 得分块模式相关
        self.scoring_mode = False
        self.score_color = 1  # 1=黄色, 2=绿色
                # ====== 精确障碍物分布（根据图纸推算，单位100mm格） ======
        # # 左侧拱门障碍物（左边缘150mm宽，约1.5格，向右取2格，纵向从y=0到y=4.6格）
        # for r in range(27, 32):  # x=150~600mm, 32-1-27=4, 32-1-31=0
        #     self.grid_map[r][1] = 1
        # # 左上拱门
        # for r in range(0, 5):
        #     self.grid_map[r][1] = 1
        # # 左中拱门
        # for r in range(14, 19):
        #     self.grid_map[r][1] = 1
        # # 右侧拱门障碍物（右边缘150mm宽，约1.5格，向左取2格）
        # for r in range(27, 32):
        #     self.grid_map[r][38] = 1
        # for r in range(0, 5):
        #     self.grid_map[r][38] = 1
        # for r in range(14, 19):
        #     self.grid_map[r][38] = 1
        # # 内部圆柱障碍物（以中心点落格，直径150mm约1.5格，取1格覆盖）
        # self.grid_map[8][7] = 1  # 左上圆柱
        # self.grid_map[16][19] = 1  # 中央圆柱
        # self.grid_map[24][31] = 1  # 右下圆柱S
        # # 内部方块障碍物（50mm边长，约0.5格，取1格覆盖）
        # self.grid_map[4][35] = 1
        # self.grid_map[28][5] = 1
        # # 右上、右下、左下等小障碍物
        # self.grid_map[2][38] = 1
        # self.grid_map[29][38] = 1
        # self.grid_map[29][1] = 1
        # # 其他图中明显障碍物（如有遗漏可补充）
        # # ...可根据需要继续补充
        # ====== END ======

        
        # 路径录制数据
        self.recording = False
        self.paths = []  # 存储多条路径，每条路径是(color, points_list)，points_list是[(x,y),...] ROS坐标
        self.current_recording_path = None  # 当前录制的路径索引
        # 自定义路径存储
        self.saved_paths = {}
        self.current_path_name = None
        
        # 创建界面
        self.create_widgets()
        # 启动时尝试加载上次保存的地图
        self.load_map()
        # 载入自定义路径列表
        self.load_paths()
        
    def create_widgets(self):
        # 主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 左侧：画布和控制
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 说明标签
        info_label = ttk.Label(
            left_frame, 
            text=f"ROS坐标系：右下角(0,0), X轴向上↑, Y轴向左←\n点击格子切换黑白，黑色=障碍物",
            justify=tk.CENTER
        )
        info_label.pack(pady=5)
        
        # 坐标显示标签
        self.coord_label = ttk.Label(
            left_frame,
            text="坐标: --",
            font=('Arial', 10, 'bold'),
            foreground='blue'
        )
        self.coord_label.pack(pady=2)
        
        # 画布框架（带滚动条）
        canvas_frame = ttk.Frame(left_frame)
        canvas_frame.pack(fill=tk.BOTH, expand=True)
        
        # 创建画布和滚动条
        self.canvas = tk.Canvas(
            canvas_frame,
            width=min(800, self.grid_cols * self.cell_size + 80),
            height=min(600, self.grid_rows * self.cell_size + 80),
            bg='white'
        )
        
        v_scrollbar = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        h_scrollbar = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL, command=self.canvas.xview)
        
        self.canvas.configure(yscrollcommand=v_scrollbar.set, xscrollcommand=h_scrollbar.set)
        
        # 布局
        self.canvas.grid(row=0, column=0, sticky='nsew')
        v_scrollbar.grid(row=0, column=1, sticky='ns')
        h_scrollbar.grid(row=1, column=0, sticky='ew')
        
        canvas_frame.grid_rowconfigure(0, weight=1)
        canvas_frame.grid_columnconfigure(0, weight=1)
        
        # 绘制网格
        self.draw_grid()
        # 路径点标记
        self.path_markers = []
        
        # 绑定鼠标事件
        self.canvas.bind('<Button-1>', self.on_cell_click)
        self.canvas.bind('<B1-Motion>', self.on_cell_drag)
        self.canvas.bind('<Motion>', self.on_mouse_move)
        self.canvas.bind('<Leave>', self.on_mouse_leave)
        
        # 控制按钮
        button_frame = ttk.Frame(left_frame)
        button_frame.pack(pady=10)
        
        ttk.Button(button_frame, text="清空地图", command=self.clear_map).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="反转地图", command=self.invert_map).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="保存地图", command=self.save_map).pack(side=tk.LEFT, padx=5)
        self.record_button = ttk.Button(button_frame, text="开始录制路径", command=self.toggle_recording)
        self.record_button.pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="清除路径", command=self.clear_path).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="输入路径坐标", command=self.input_path_coordinates).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="生成代码", command=self.generate_code).pack(side=tk.LEFT, padx=5)
        
        # 涂改模式控件
        paint_frame = ttk.Frame(left_frame)
        paint_frame.pack(pady=5)
        self.paint_button = ttk.Button(paint_frame, text="开启涂改模式", command=self.toggle_painting)
        self.paint_button.pack(side=tk.LEFT, padx=5)
        
        # 颜色选择
        color_frame = ttk.Frame(paint_frame)
        color_frame.pack(side=tk.LEFT, padx=10)
        ttk.Label(color_frame, text="涂改颜色:").pack(side=tk.LEFT)
        self.color_var = tk.IntVar(value=1)
        ttk.Radiobutton(color_frame, text="红色", variable=self.color_var, value=1, command=self.set_paint_color).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(color_frame, text="蓝色", variable=self.color_var, value=2, command=self.set_paint_color).pack(side=tk.LEFT, padx=2)
        
        # 得分块模式控件
        score_frame = ttk.Frame(left_frame)
        score_frame.pack(pady=5)
        self.score_button = ttk.Button(score_frame, text="开启得分块模式", command=self.toggle_scoring)
        self.score_button.pack(side=tk.LEFT, padx=5)
        
        # 得分块颜色选择
        score_color_frame = ttk.Frame(score_frame)
        score_color_frame.pack(side=tk.LEFT, padx=10)
        ttk.Label(score_color_frame, text="得分块颜色:").pack(side=tk.LEFT)
        self.score_color_var = tk.IntVar(value=1)
        ttk.Radiobutton(score_color_frame, text="黄色", variable=self.score_color_var, value=1, command=self.set_score_color).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(score_color_frame, text="绿色", variable=self.score_color_var, value=2, command=self.set_score_color).pack(side=tk.LEFT, padx=2)
        
        # 右侧：代码输出
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))
        
        ttk.Label(right_frame, text="生成的C代码：").pack(anchor=tk.W)
        
        self.code_text = scrolledtext.ScrolledText(
            right_frame,
            width=50,
            height=30,
            font=('Consolas', 9)
        )
        self.code_text.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # 复制按钮
        ttk.Button(right_frame, text="复制到剪贴板", command=self.copy_to_clipboard).pack(pady=5)
        
        # 统计信息
        self.stats_label = ttk.Label(right_frame, text="障碍物数量: 0")
        self.stats_label.pack(pady=5)
        self.path_stats_label = ttk.Label(right_frame, text="路径点数量: 0")
        self.path_stats_label.pack(pady=5)
        # 路径管理区块
        path_frame = ttk.LabelFrame(right_frame, text="路径管理")
        path_frame.pack(fill=tk.X, pady=5)
        self.path_listbox = tk.Listbox(path_frame, height=5)
        self.path_listbox.pack(side=tk.LEFT, fill=tk.X, expand=True)
        path_scroll = ttk.Scrollbar(path_frame, orient=tk.VERTICAL, command=self.path_listbox.yview)
        self.path_listbox.config(yscrollcommand=path_scroll.set)
        path_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        btn_frame2 = ttk.Frame(right_frame)
        btn_frame2.pack(fill=tk.X)
        ttk.Button(btn_frame2, text="新建", command=self.new_path).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame2, text="保存当前", command=self.save_current_path).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame2, text="加载", command=self.load_selected_path).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame2, text="删除", command=self.delete_selected_path).pack(side=tk.LEFT, padx=2)
        
    def draw_grid(self):
        """绘制网格和坐标轴"""
        self.canvas.delete('all')
        # 清除路径标记
        self.path_markers = []
        
        # 计算偏移量（留出空间显示坐标）
        offset_x = 40
        offset_y = 20
        
        # 设置滚动区域
        total_width = self.grid_cols * self.cell_size + offset_x + 20
        total_height = self.grid_rows * self.cell_size + offset_y + 40
        self.canvas.configure(scrollregion=(0, 0, total_width, total_height))
        
        # 存储格子的矩形ID
        self.cell_rects = {}
        
        # 绘制格子
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                x1 = offset_x + col * self.cell_size
                y1 = offset_y + row * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                
                # 根据得分块、涂改颜色、地图值选择颜色
                if self.score_map[row][col] == 1:
                    color = 'yellow'
                elif self.score_map[row][col] == 2:
                    color = 'green'
                elif self.color_map[row][col] == 1:
                    color = 'red'
                elif self.color_map[row][col] == 2:
                    color = 'blue'
                else:
                    color = 'black' if self.grid_map[row][col] == 1 else 'white'
                
                rect = self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=color,
                    outline='gray',
                    tags=f'cell_{row}_{col}'
                )
                self.cell_rects[(row, col)] = rect
        
        # 重新绘制已有路径点标记
        for color, points in self.paths:
            for (x_ros, y_ros) in points:
                row = self.grid_rows - 1 - x_ros
                col = self.grid_cols - 1 - y_ros
                cx = offset_x + col * self.cell_size + self.cell_size/2
                cy = offset_y + row * self.cell_size + self.cell_size/2
                r = self.cell_size/3
                marker = self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill=color, outline='')
                self.path_markers.append(marker)
        
        # 绘制坐标轴标签
        # X轴标签（左侧，从上到下：上方显示大值，下方显示小值）
        for row in range(0, self.grid_rows, 4):  # 每4个格子显示一个
            x_coord = self.grid_rows - 1 - row  # ROS坐标：上方是大值
            y_pos = offset_y + row * self.cell_size + self.cell_size // 2
            self.canvas.create_text(20, y_pos, text=f'x={x_coord}', font=('Arial', 8))
        
        # Y轴标签（底部，从左到右：左侧显示大值，右侧显示小值）
        for col in range(0, self.grid_cols, 4):  # 每4个格子显示一个
            y_coord = self.grid_cols - 1 - col  # ROS坐标：左侧是大值
            x_pos = offset_x + col * self.cell_size + self.cell_size // 2
            self.canvas.create_text(
                x_pos, 
                offset_y + self.grid_rows * self.cell_size + 15,
                text=f'y={y_coord}', 
                font=('Arial', 8)
            )
        
        # 绘制坐标系说明
        self.canvas.create_text(
            offset_x + self.grid_cols * self.cell_size // 2,
            5,
            text="↑ X轴正方向",
            font=('Arial', 10, 'bold'),
            fill='red'
        )
        
        self.canvas.create_text(
            10,
            offset_y + self.grid_rows * self.cell_size // 2,
            text="← Y",
            font=('Arial', 10, 'bold'),
            fill='blue',
            angle=90
        )
        
        # 标注原点（右下角）
        origin_x = offset_x + (self.grid_cols - 1) * self.cell_size
        origin_y = offset_y + (self.grid_rows - 1) * self.cell_size
        self.canvas.create_text(
            origin_x + self.cell_size // 2,
            origin_y + self.cell_size // 2,
            text="(0,0)",
            font=('Arial', 8, 'bold'),
            fill='red'
        )
    
    def get_cell_from_click(self, event):
        """从点击位置获取格子坐标"""
        offset_x = 40
        offset_y = 20
        
        col = (event.x - offset_x) // self.cell_size
        row = (event.y - offset_y) // self.cell_size
        
        if 0 <= row < self.grid_rows and 0 <= col < self.grid_cols:
            return row, col
        return None, None
    
    def toggle_cell(self, row, col):
        """切换格子的状态"""
        if row is not None and col is not None:
            # 切换状态
            self.grid_map[row][col] = 1 - self.grid_map[row][col]
            
            # 更新颜色
            color = 'black' if self.grid_map[row][col] == 1 else 'white'
            self.canvas.itemconfig(self.cell_rects[(row, col)], fill=color)
            
            # 更新统计
            self.update_stats()
    
    def on_cell_click(self, event):
        """鼠标点击事件"""
        row, col = self.get_cell_from_click(event)
        if row is not None and col is not None:
            if self.scoring_mode:
                # 得分块模式：设置得分块颜色，如果已有颜色则变回白色
                if self.score_map[row][col] != 0:
                    self.score_map[row][col] = 0
                else:
                    self.score_map[row][col] = self.score_color
                self.draw_grid()
            elif self.painting_mode:
                # 涂改模式：设置颜色，如果已有颜色则变回白色，不改变通行性
                if self.color_map[row][col] != 0:
                    self.color_map[row][col] = 0
                else:
                    self.color_map[row][col] = self.paint_color
                self.draw_grid()
            elif self.recording:
                # 录制路径
                self.add_path_point(row, col)
            else:
                # 正常模式：切换通行性
                self.toggle_cell(row, col)
            self.last_toggled = (row, col)

    def on_cell_drag(self, event):
        """鼠标拖动事件"""
        row, col = self.get_cell_from_click(event)
        if (row, col) != getattr(self, 'last_toggled', None):
            if self.recording and row is not None and col is not None:
                self.add_path_point(row, col)
            else:
                self.toggle_cell(row, col)
            self.last_toggled = (row, col)
    
    def on_cell_drag(self, event):
        """鼠标拖动事件"""
        row, col = self.get_cell_from_click(event)
        if (row, col) != getattr(self, 'last_toggled', None):
            if self.scoring_mode:
                # 得分块模式：设置得分块颜色，如果已有颜色则变回白色
                if self.score_map[row][col] != 0:
                    self.score_map[row][col] = 0
                else:
                    self.score_map[row][col] = self.score_color
                self.draw_grid()
            elif self.painting_mode:
                # 涂改模式：设置颜色，如果已有颜色则变回白色
                if self.color_map[row][col] != 0:
                    self.color_map[row][col] = 0
                else:
                    self.color_map[row][col] = self.paint_color
                self.draw_grid()
            elif self.recording:
                # 录制路径
                self.add_path_point(row, col)
            else:
                # 正常模式：切换通行性
                self.toggle_cell(row, col)
            self.last_toggled = (row, col)
    
    def on_mouse_move(self, event):
        """鼠标移动事件 - 显示当前坐标"""
        row, col = self.get_cell_from_click(event)
        if row is not None and col is not None:
            # 转换为ROS坐标
            x_ros = self.grid_rows - 1 - row
            y_ros = self.grid_cols - 1 - col
            status = "障碍物" if self.grid_map[row][col] == 1 else "可通行"
            self.coord_label.config(text=f"坐标: ({x_ros}, {y_ros}) - {status}")
        else:
            self.coord_label.config(text="坐标: --")
    
    def on_mouse_leave(self, event):
        """鼠标离开画布"""
        self.coord_label.config(text="坐标: --")
    
    def clear_map(self):
        """清空地图"""
        self.grid_map.fill(0)
        self.color_map.fill(0)
        self.score_map.fill(0)
        self.draw_grid()
        self.update_stats()

    def new_path(self):
        """新建一个空路径，输入名称"""
        name = tk.simpledialog.askstring("新建路径", "输入路径名称：")
        if not name:
            return
        # 设置当前名称
        self.current_path_name = name
        self.saved_paths[name] = []
        self.update_path_listbox()
        self.save_paths()

    def save_current_path(self):
        """将当前录制的路径保存到字典"""
        if self.current_recording_path is None or not self.paths:
            messagebox.showerror("保存失败", "没有正在录制的路径")
            return
            
        if not self.current_path_name:
            name = tk.simpledialog.askstring("保存路径", "输入路径名称：")
            if not name:
                return
            self.current_path_name = name
        else:
            name = self.current_path_name
        
        # 保存当前录制的路径
        color, points = self.paths[self.current_recording_path]
        self.saved_paths[name] = list(points)
        self.update_path_listbox()
        self.save_paths()
        messagebox.showinfo("保存", f"路径 '{name}' 已保存")

    def load_selected_path(self):
        """从列表中加载选中的路径"""
        sel = self.path_listbox.curselection()
        if not sel:
            return
        name = self.path_listbox.get(sel[0])
        pts = self.saved_paths.get(name, [])
        self.current_path_name = name
        # 将加载的路径添加到当前路径列表中（蓝色）
        self.paths.append(('blue', list(pts)))
        # 重绘地图和标记
        self.draw_grid()
        self.update_stats()
        messagebox.showinfo("加载", f"已加载路径 '{name}'，共 {len(pts)} 点")

    def delete_selected_path(self):
        """删除列表中当前选中的路径"""
        sel = self.path_listbox.curselection()
        if not sel:
            return
        name = self.path_listbox.get(sel[0])
        if messagebox.askyesno("删除", f"确认删除路径 '{name}'？"):
            self.saved_paths.pop(name, None)
            if self.current_path_name == name:
                self.current_path_name = None
                self.path_points.clear()
                for m in getattr(self, 'path_markers', []):
                    self.canvas.delete(m)
                self.path_markers = []
            self.update_path_listbox()
            self.save_paths()

    def add_path_point(self, row, col):
        """记录路径点（以ROS坐标存储），并在画布上做标记"""
        if self.current_recording_path is None:
            return
        
        # 转换为ROS坐标系
        x_ros = self.grid_rows - 1 - row
        y_ros = self.grid_cols - 1 - col
        
        # 添加到当前录制的路径
        color, points = self.paths[self.current_recording_path]
        points.append((x_ros, y_ros))
        
        # 在格子中心画一个小圆作为标记
        offset_x = 40
        offset_y = 20
        cx = offset_x + col * self.cell_size + self.cell_size/2
        cy = offset_y + row * self.cell_size + self.cell_size/2
        r = self.cell_size/3
        marker = self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill=color, outline='')
        self.path_markers.append(marker)
        self.update_stats()

    def toggle_recording(self):
        """开启/停止录制"""
        self.recording = not self.recording
        text = "停止录制路径" if self.recording else "开始录制路径"
        self.record_button.config(text=text)
        
        if self.recording:
            # 开始录制，创建一个新路径
            self.paths.append(('blue', []))
            self.current_recording_path = len(self.paths) - 1
            messagebox.showinfo("录制", "已开始录制路径，点击或拖动格子以添加点")
        else:
            # 停止录制
            if self.current_recording_path is not None:
                color, points = self.paths[self.current_recording_path]
                messagebox.showinfo("录制", f"路径录制结束，共 {len(points)} 个点")
            self.current_recording_path = None

    def toggle_painting(self):
        """开启/停止涂改模式"""
        self.painting_mode = not self.painting_mode
        text = "停止涂改模式" if self.painting_mode else "开启涂改模式"
        self.paint_button.config(text=text)
        if self.painting_mode:
            messagebox.showinfo("涂改", f"已开启涂改模式，当前颜色：{'红色' if self.paint_color == 1 else '蓝色'}")
        else:
            messagebox.showinfo("涂改", "已停止涂改模式")

    def set_paint_color(self):
        """设置涂改颜色"""
        self.paint_color = self.color_var.get()
    
    def toggle_scoring(self):
        """开启/停止得分块模式"""
        self.scoring_mode = not self.scoring_mode
        text = "停止得分块模式" if self.scoring_mode else "开启得分块模式"
        self.score_button.config(text=text)
        if self.scoring_mode:
            messagebox.showinfo("得分块", f"已开启得分块模式，当前颜色：{'黄色' if self.score_color == 1 else '绿色'}")
        else:
            messagebox.showinfo("得分块", "已停止得分块模式")
    
    def set_score_color(self):
        """设置得分块颜色"""
        self.score_color = self.score_color_var.get()

    def clear_path(self):
        """清除所有路径"""
        self.paths.clear()
        self.current_recording_path = None
        # 删除所有标记
        for m in getattr(self, 'path_markers', []):
            self.canvas.delete(m)
        self.path_markers = []
        self.update_stats()

    def input_path_coordinates(self):
        """输入路径坐标列表"""
        # 弹出对话框让用户输入坐标，每行一个x,y
        input_text = simpledialog.askstring(
            "输入路径坐标", 
            "请输入路径坐标，每行一个x,y（用逗号或空格分隔）：\n例如：\n100,1900\n100,1868\n...",
            parent=self.root
        )
        if not input_text:
            return
        
        # 解析输入的坐标
        new_points = []
        lines = input_text.strip().split('\n')
        for line in lines:
            line = line.strip()
            if not line:
                continue
            # 支持逗号或空格分隔
            parts = line.replace(',', ' ').split()
            if len(parts) >= 2:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    new_points.append((x, y))
                except ValueError:
                    messagebox.showerror("输入错误", f"无法解析坐标: {line}")
                    return
        
        if not new_points:
            messagebox.showerror("输入错误", "没有有效的坐标点")
            return
        
        # 添加新路径（黄色），不清除现有路径
        # 将毫米坐标转换为格子坐标
        grid_points = []
        for x_mm, y_mm in new_points:
            # 转换为格子坐标（假设每格100mm）
            x_grid = x_mm / 100.0
            y_grid = y_mm / 100.0
            grid_points.append((x_grid, y_grid))
        
        self.paths.append(('yellow', grid_points))
        
        # 重绘地图以显示新路径
        self.draw_grid()
        self.update_stats()
        messagebox.showinfo("成功", f"已添加新路径，共 {len(new_points)} 个点")

    def update_path_listbox(self):
        """刷新列表框显示"""
        self.path_listbox.delete(0, tk.END)
        for name in sorted(self.saved_paths.keys()):
            self.path_listbox.insert(tk.END, name)

    def save_paths(self):
        """将所有自定义路径写入文件"""
        fname = "saved_paths.txt"
        try:
            with open(fname, 'w') as f:
                for name, pts in self.saved_paths.items():
                    f.write(f"#NAME:{name}\n")
                    for x, y in pts:
                        f.write(f"{x} {y}\n")
        except Exception as e:
            messagebox.showerror("保存路径失败", str(e))

    def load_paths(self):
        """从文件加载自定义路径"""
        fname = "saved_paths.txt"
        try:
            with open(fname, 'r') as f:
                lines = [l.rstrip('\n') for l in f]
        except FileNotFoundError:
            return
        except Exception as e:
            messagebox.showerror("加载路径失败", str(e))
            return
        cur_name = None
        for l in lines:
            if l.startswith("#NAME:"):
                cur_name = l.split(":",1)[1]
                self.saved_paths[cur_name] = []
            elif cur_name and l.strip():
                parts = l.split()
                if len(parts) >= 2:
                    try:
                        x = int(parts[0]); y = int(parts[1])
                        self.saved_paths[cur_name].append((x,y))
                    except ValueError:
                        pass
        self.update_path_listbox()

    def save_map(self):
        """将当前地图及路径保存到默认文件"""
        filename = "saved_map.txt"
        try:
            with open(filename, 'w') as f:
                # 写入地图行
                for row in range(self.grid_rows):
                    f.write(''.join(str(int(v)) for v in self.grid_map[row]) + "\n")
                # 写入颜色行
                f.write("# Color map\n")
                for row in range(self.grid_rows):
                    f.write(''.join(str(int(v)) for v in self.color_map[row]) + "\n")
                # 写入得分块行
                f.write("# Score map\n")
                for row in range(self.grid_rows):
                    f.write(''.join(str(int(v)) for v in self.score_map[row]) + "\n")
                # 如果有路径则追加
                if self.paths:
                    f.write("\n# Paths\n")
                    for color, points in self.paths:
                        f.write(f"#COLOR:{color}\n")
                        for x, y in points:
                            f.write(f"{x} {y}\n")
            messagebox.showinfo("保存成功", f"地图已保存到 {filename}")
        except Exception as e:
            messagebox.showerror("保存失败", str(e))

    def load_map(self):
        """读取默认保存文件并加载地图与路径"""
        filename = "saved_map.txt"
        try:
            with open(filename, 'r') as f:
                lines = [l.rstrip('\n') for l in f]
        except FileNotFoundError:
            return
        except Exception as e:
            messagebox.showerror("加载失败", str(e))
            return
        # parse map: consecutive lines of 0/1 until blank or comment
        map_lines = []
        color_lines = []
        score_lines = []
        path_lines = []
        phase = 'map'
        for l in lines:
            if phase == 'map':
                if l.strip() == '' or l.startswith('#'):
                    if l.startswith('# Color map'):
                        phase = 'color'
                    elif l.startswith('# Score map'):
                        phase = 'score'
                    elif l.startswith('# Paths'):
                        phase = 'paths'
                    elif l.startswith('# Path points'):
                        phase = 'path'
                    else:
                        phase = 'path'
                else:
                    map_lines.append(l)
            elif phase == 'color':
                if l.strip() == '' or l.startswith('#'):
                    if l.startswith('# Score map'):
                        phase = 'score'
                    elif l.startswith('# Paths'):
                        phase = 'paths'
                    elif l.startswith('# Path points'):
                        phase = 'path'
                    else:
                        phase = 'path'
                else:
                    color_lines.append(l)
            elif phase == 'score':
                if l.strip() == '' or l.startswith('#'):
                    if l.startswith('# Paths'):
                        phase = 'paths'
                    elif l.startswith('# Path points'):
                        phase = 'path'
                    else:
                        phase = 'path'
                else:
                    score_lines.append(l)
            else:
                path_lines.append(l)
        # populate grid_map
        if map_lines:
            for i, row in enumerate(map_lines[:self.grid_rows]):
                for j, ch in enumerate(row[:self.grid_cols]):
                    self.grid_map[i][j] = 1 if ch == '1' else 0
        # populate color_map
        if color_lines:
            for i, row in enumerate(color_lines[:self.grid_rows]):
                for j, ch in enumerate(row[:self.grid_cols]):
                    self.color_map[i][j] = int(ch) if ch in '012' else 0
        # populate score_map
        if score_lines:
            for i, row in enumerate(score_lines[:self.grid_rows]):
                for j, ch in enumerate(row[:self.grid_cols]):
                    self.score_map[i][j] = int(ch) if ch in '012' else 0
        # parse paths
        self.paths.clear()
        current_color = 'blue'  # 默认颜色
        current_points = []
        has_color_markers = any(pl.startswith('#COLOR:') for pl in path_lines)
        
        if has_color_markers:
            # 新格式：有颜色标记
            for pl in path_lines:
                if pl.startswith('#COLOR:'):
                    # 保存之前的路径
                    if current_points:
                        self.paths.append((current_color, current_points))
                    # 开始新路径
                    current_color = pl.split(':', 1)[1]
                    current_points = []
                elif pl.strip():
                    parts = pl.split()
                    if len(parts) >= 2:
                        try:
                            x = float(parts[0])
                            y = float(parts[1])
                            current_points.append((x, y))
                        except ValueError:
                            pass
            # 保存最后一个路径
            if current_points:
                self.paths.append((current_color, current_points))
        else:
            # 旧格式：没有颜色标记，所有点都是蓝色路径
            for pl in path_lines:
                if pl.strip() and not pl.startswith('#'):
                    parts = pl.split()
                    if len(parts) >= 2:
                        try:
                            x = float(parts[0])
                            y = float(parts[1])
                            current_points.append((x, y))
                        except ValueError:
                            pass
            if current_points:
                self.paths.append(('blue', current_points))
        # redraw map/markers and update stats
        self.draw_grid()
        # draw existing path markers (draw_grid already does)
        self.update_stats()
        # 此处不涉及自定义路径文件

    def update_path_listbox(self):
        """刷新列表框显示"""
        self.path_listbox.delete(0, tk.END)
        for name in sorted(self.saved_paths.keys()):
            self.path_listbox.insert(tk.END, name)

    def save_paths(self):
        """将所有自定义路径写入文件"""
        fname = "saved_paths.txt"
        try:
            with open(fname, 'w') as f:
                for name, pts in self.saved_paths.items():
                    f.write(f"#NAME:{name}\n")
                    for x, y in pts:
                        f.write(f"{x} {y}\n")
        except Exception as e:
            messagebox.showerror("保存路径失败", str(e))

    def load_paths(self):
        """从文件加载自定义路径"""
        fname = "saved_paths.txt"
        try:
            with open(fname, 'r') as f:
                lines = [l.rstrip('\n') for l in f]
        except FileNotFoundError:
            return
        except Exception as e:
            messagebox.showerror("加载路径失败", str(e))
            return
        cur_name = None
        for l in lines:
            if l.startswith("#NAME:"):
                cur_name = l.split(":",1)[1]
                self.saved_paths[cur_name] = []
            elif cur_name and l.strip():
                parts = l.split()
                if len(parts) >= 2:
                    try:
                        x = int(parts[0]); y = int(parts[1])
                        self.saved_paths[cur_name].append((x,y))
                    except ValueError:
                        pass
        self.update_path_listbox()
    
    def invert_map(self):
        """反转地图"""
        self.grid_map = 1 - self.grid_map
        self.draw_grid()
        self.update_stats()
    
    def update_stats(self):
        """更新统计信息"""
        obstacle_count = np.sum(self.grid_map)
        self.stats_label.config(text=f"障碍物数量: {obstacle_count}")
        total_path_points = sum(len(points) for color, points in self.paths)
        self.path_stats_label.config(text=f"路径点数量: {total_path_points}")
    
    def generate_code(self):
        """生成C代码"""
        code_lines = []
        code_lines.append("// 生成的障碍物设置代码")
        code_lines.append(f"// 总共 {np.sum(self.grid_map)} 个障碍物")
        code_lines.append("// 坐标系：右下角(0,0)，X轴向上，Y轴向左\n")
        
        obstacle_count = 0
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                if self.grid_map[row][col] == 1:
                    # 转换为ROS坐标系：右下角(0,0)，x向上，y向左
                    x_ros = self.grid_rows - 1 - row  # 上方row=0对应x=grid_rows-1，下方对应x=0
                    y_ros = self.grid_cols - 1 - col  # 左侧col=0对应y=grid_cols-1，右侧对应y=0
                    code_lines.append(f"AStar_SetObstacle({x_ros}, {y_ros});")
                    obstacle_count += 1
        
        # 如果有路径点，则生成PathTracking_AddPoint代码，每个点之间插入1个线性插值
        if self.paths:
            code_lines.append("")
            code_lines.append("// 手动录制路径（包含线性插值）")
            for path_idx, (color, points) in enumerate(self.paths):
                if not points:
                    continue
                code_lines.append(f"// 路径 {path_idx + 1} ({color})")
                interpolated_points = []
                for i in range(len(points) - 1):
                    x1, y1 = points[i]
                    x2, y2 = points[i + 1]
                    interpolated_points.append((x1, y1))
                    # 插入1个插值点（中间点）
                    interp_x = x1 + (x2 - x1) / 2
                    interp_y = y1 + (y2 - y1) / 2
                    interpolated_points.append((interp_x, interp_y))
                # 添加最后一个点
                interpolated_points.append(points[-1])
                
                for (x, y) in interpolated_points:
                    code_lines.append(f"PathTracking_AddPoint({100*x:.0f}, {100*y:.0f});")
        
        code = '\n'.join(code_lines)
        
        # 显示代码
        self.code_text.delete(1.0, tk.END)
        self.code_text.insert(1.0, code)
        
        message = f"已生成 {obstacle_count} 个障碍物的设置代码"
        if self.paths:
            total_points = sum(len(points) for color, points in self.paths)
            message += f"，路径点 {total_points} 个（{len(self.paths)} 条路径）"
        messagebox.showinfo("代码生成完成", message)
    
    def copy_to_clipboard(self):
        """复制代码到剪贴板"""
        code = self.code_text.get(1.0, tk.END)
        self.root.clipboard_clear()
        self.root.clipboard_append(code)
        messagebox.showinfo("复制成功", "代码已复制到剪贴板")


def main():
    root = tk.Tk()
    app = GridMapEditor(root, grid_rows=64, grid_cols=80)
    
        # 设置窗口大小适配大格子
    root.geometry("800x600")
    
    root.mainloop()


if __name__ == '__main__':
    main()
