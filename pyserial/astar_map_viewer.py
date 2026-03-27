"""
A* 代价地图可视化工具 (Tkinter版本)
功能：
1. 选择串口并连接
2. 接收单片机发送的代价地图数据（48x48 网格）
3. 实时可视化显示，符合ROS坐标系（X+向前，Y+向左）
4. 不同代价显示不同颜色
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import numpy as np
import threading
import time
from datetime import datetime

# 地图参数（与STM32端保持一致）
MAP_WIDTH = 48   # 网格宽度
MAP_HEIGHT = 48  # 网格高度
MAP_WIDTH_MM = 3000   # 实际宽度(mm)
MAP_HEIGHT_MM = 3000  # 实际高度(mm)

class SerialThread(threading.Thread):
    """串口接收线程"""
    
    def __init__(self, callback_map, callback_msg):
        super().__init__(daemon=True)
        self.serial_port = None
        self.running = False
        self.callback_map = callback_map  # 地图数据回调
        self.callback_msg = callback_msg  # 消息回调
        
    def set_serial(self, port, baudrate=115200):
        """设置串口"""
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.callback_msg(f"✅ 串口 {port} 连接成功，波特率 {baudrate}")
            return True
        except Exception as e:
            self.callback_msg(f"❌ 串口连接失败: {str(e)}")
            return False
    
    def run(self):
        """接收数据主循环"""
        self.running = True
        buffer = bytearray()
        
        while self.running:
            if not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.1)
                continue
            
            try:
                # 读取可用数据
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer.extend(data)
                    
                    # 计算一帧完整地图数据的大小
                    # 每行：MAP_WIDTH个字节 + 1个换行符(0x0A)
                    expected_size = MAP_HEIGHT * (MAP_WIDTH + 1)
                    
                    # 查找完整的地图数据帧（至少包含MAP_HEIGHT个换行符）
                    newline_count = buffer.count(0x0A)
                    
                    if len(buffer) >= expected_size and newline_count >= MAP_HEIGHT:
                        # 提取一帧数据
                        map_data = buffer[:expected_size]
                        buffer = buffer[expected_size:]  # 清除已处理的数据
                        
                        # 验证数据帧有效性（应该正好有MAP_HEIGHT个换行符）
                        if map_data.count(0x0A) == MAP_HEIGHT:
                            # 解析地图数据
                            grid_map = self.parse_map_data(map_data)
                            if grid_map is not None:
                                self.callback_map(grid_map)
                                self.callback_msg(f"📡 接收到地图数据 ({MAP_HEIGHT}x{MAP_WIDTH})")
                        else:
                            self.callback_msg(f"⚠️ 数据帧格式错误，换行符数量: {map_data.count(0x0A)}")
                            # 清空缓冲区，等待下一帧
                            buffer.clear()
                
                time.sleep(0.05)  # 降低CPU占用
                
            except Exception as e:
                self.callback_msg(f"⚠️ 数据接收错误: {str(e)}")
                time.sleep(0.1)
    
    def parse_map_data(self, data):
        """
        解析地图数据
        STM32打印格式（grid[row][col]其中row=Y索引，col=X索引）：
        - 外层循环col=X轴（0~47），内层循环row=Y轴（0~47）
        - 每行发送：固定X值时，所有Y值（row=0到47）
        - 行1: grid[0][0], grid[1][0], ..., grid[47][0]  (X=0, Y=0~47)
        - 行2: grid[0][1], grid[1][1], ..., grid[47][1]  (X=1, Y=0~47)
        """
        try:
            grid_map = np.zeros((MAP_HEIGHT, MAP_WIDTH), dtype=np.uint8)
            
            # 移除所有换行符，只保留数据字节
            clean_data = bytearray()
            for byte in data:
                if byte != 0x0A:  # 跳过换行符
                    clean_data.append(byte)
            
            # 检查数据长度
            expected_length = MAP_HEIGHT * MAP_WIDTH
            if len(clean_data) < expected_length:
                self.callback_msg(f"⚠️ 数据不完整: 接收{len(clean_data)}字节，期望{expected_length}字节")
                return None
            
            # 解析数据到地图
            # STM32发送顺序：外循环X(列索引)，内循环Y(行索引)
            # 每行48字节代表：固定X值，Y从0到47
            data_idx = 0
            for col in range(MAP_WIDTH):  # col=X轴索引(0~47)
                for row in range(MAP_HEIGHT):  # row=Y轴索引(0~47)
                    if data_idx < len(clean_data):
                        grid_map[row][col] = clean_data[data_idx]  # grid_map[Y索引][X索引]
                        data_idx += 1
            
            return grid_map
            
        except Exception as e:
            self.callback_msg(f"❌ 地图解析失败: {str(e)}")
            return None
    
    def stop(self):
        """停止线程"""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()


class MapCanvas(tk.Canvas):
    """地图显示画布"""
    
    def __init__(self, parent):
        self.cell_size = 12  # 每个格子的像素大小
        canvas_width = MAP_WIDTH * self.cell_size + 100
        canvas_height = MAP_HEIGHT * self.cell_size + 80
        
        super().__init__(parent, width=canvas_width, height=canvas_height, bg='white')
        
        self.grid_map = np.zeros((MAP_HEIGHT, MAP_WIDTH), dtype=np.uint8)
        self.cell_rects = {}
        
        self.offset_x = 60
        self.offset_y = 20
        
        self.draw_grid()
        
    def get_color_for_value(self, value):
        """根据代价值返回颜色"""
        if value == 0:
            return '#FFFFFF'  # 白色 - 空闲
        elif value < 15:
            return '#E8F5E9'  # 浅绿
        elif value == 15:
            return '#FFEB3B'  # 黄色 - 层3
        elif 15 < value < 30:
            return '#FFD54F'  # 浅黄
        elif value == 30:
            return '#FF9800'  # 橙色 - 层2
        elif 30 < value < 60:
            return '#FF7043'  # 浅橙
        elif value == 60:
            return '#F44336'  # 红色 - 层1
        elif 60 < value < 255:
            return '#E91E63'  # 深红
        else:  # value == 255
            return '#000000'  # 黑色 - 障碍物
    
    def draw_grid(self):
        """绘制网格"""
        self.delete('all')
        self.cell_rects = {}
        
        # 绘制标题
        self.create_text(
            self.offset_x + MAP_WIDTH * self.cell_size // 2,
            10,
            text="A* 代价地图 (ROS坐标系)",
            font=('Arial', 12, 'bold')
        )
        
        # 绘制格子 (ROS坐标系：X向前上，Y向左)
        # 数组布局：grid_map[row][col]，row=Y轴索引(0~47)，col=X轴索引(0~47)
        # 
        # ROS坐标系定义：
        # - X轴：向前（屏幕向上），X=0在下方，X=47在上方
        # - Y轴：向左，Y=0在右侧，Y=47在左侧
        # - 原点(X=0,Y=0)在右下角
        for row in range(MAP_HEIGHT):  # row = Y轴索引 (0~47)
            for col in range(MAP_WIDTH):  # col = X轴索引 (0~47)
                # 屏幕显示转换：
                # X轴(col索引): 0在屏幕下方，47在屏幕上方 → 上下翻转
                # Y轴(row索引): 0在屏幕右侧，47在屏幕左侧 → 左右翻转
                display_row = MAP_HEIGHT - 1 - col  # X轴 → 屏幕行（上下翻转）
                display_col = MAP_WIDTH - 1 - row   # Y轴 → 屏幕列（左右翻转）
                
                x1 = self.offset_x + display_col * self.cell_size
                y1 = self.offset_y + display_row * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                
                color = self.get_color_for_value(self.grid_map[row][col])
                
                rect = self.create_rectangle(
                    x1, y1, x2, y2,
                    fill=color,
                    outline='gray',
                    width=0.5
                )
                self.cell_rects[(row, col)] = rect
        
        # X轴标签（左侧，从下到上）
        # X轴对应grid的col索引(列)
        for i in range(0, MAP_HEIGHT, 8):
            # X=0在下方，X=47在上方
            display_row = MAP_HEIGHT - 1 - i
            y_pos = self.offset_y + display_row * self.cell_size + self.cell_size // 2
            self.create_text(25, y_pos, text=f'x={i}', font=('Arial', 7))
        
        # Y轴标签（底部，从右到左）
        # Y轴对应grid的row索引(行)
        for i in range(0, MAP_WIDTH, 8):
            # Y=0在右侧，Y=47在左侧
            display_col = MAP_WIDTH - 1 - i
            x_pos = self.offset_x + display_col * self.cell_size + self.cell_size // 2
            self.create_text(
                x_pos,
                self.offset_y + MAP_HEIGHT * self.cell_size + 15,
                text=f'y={i}',
                font=('Arial', 7)
            )
        
        # 坐标轴箭头
        self.create_text(10, self.offset_y + MAP_HEIGHT * self.cell_size // 2,
                        text='X↑', font=('Arial', 10, 'bold'), fill='red')
        self.create_text(self.offset_x + MAP_WIDTH * self.cell_size // 2,
                        self.offset_y + MAP_HEIGHT * self.cell_size + 35,
                        text='← Y', font=('Arial', 10, 'bold'), fill='blue')
    
    def update_map(self, grid_map):
        """更新地图显示"""
        self.grid_map = grid_map.copy()
        
        # 更新每个格子的颜色
        # grid_map[row][col]，row=Y轴索引，col=X轴索引
        for row in range(MAP_HEIGHT):
            for col in range(MAP_WIDTH):
                color = self.get_color_for_value(grid_map[row][col])
                rect = self.cell_rects.get((row, col))
                if rect:
                    self.itemconfig(rect, fill=color)


class AStarMapViewer:
    """主窗口"""
    
    def __init__(self, root):
        self.root = root
        self.root.title('A* 代价地图可视化工具')
        self.root.geometry('1400x800')
        
        self.serial_thread = SerialThread(
            callback_map=self.on_map_received,
            callback_msg=self.on_message_received
        )
        
        self.init_ui()
        self.refresh_ports()
        
    def init_ui(self):
        """初始化UI"""
        # 主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 左侧：控制面板
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        # 串口设置组
        port_group = ttk.LabelFrame(left_frame, text="串口设置", padding=10)
        port_group.pack(fill=tk.X, pady=5)
        
        # 串口选择
        ttk.Label(port_group, text="串口:").grid(row=0, column=0, sticky=tk.W, pady=2)
        port_frame = ttk.Frame(port_group)
        port_frame.grid(row=0, column=1, sticky=tk.EW, pady=2)
        
        self.port_combo = ttk.Combobox(port_frame, width=25, state='readonly')
        self.port_combo.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        self.refresh_btn = ttk.Button(port_frame, text="🔄", width=3, command=self.refresh_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=(5, 0))
        
        # 波特率选择
        ttk.Label(port_group, text="波特率:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.baudrate_combo = ttk.Combobox(port_group, width=25, state='readonly')
        self.baudrate_combo['values'] = ['9600', '19200', '38400', '57600', '115200', '230400']
        self.baudrate_combo.set('115200')
        self.baudrate_combo.grid(row=1, column=1, sticky=tk.EW, pady=2)
        
        # 连接按钮
        self.connect_btn = ttk.Button(port_group, text="连接串口", command=self.toggle_connection)
        self.connect_btn.grid(row=2, column=0, columnspan=2, sticky=tk.EW, pady=(10, 0))
        
        port_group.grid_columnconfigure(1, weight=1)
        
        # 地图统计组
        stats_group = ttk.LabelFrame(left_frame, text="地图统计", padding=10)
        stats_group.pack(fill=tk.X, pady=5)
        
        self.stats_text = tk.Text(stats_group, height=12, width=35, font=('Consolas', 9))
        self.stats_text.pack(fill=tk.BOTH)
        self.stats_text.insert(1.0, "等待数据...")
        self.stats_text.config(state=tk.DISABLED)
        
        # 操作按钮组
        action_group = ttk.LabelFrame(left_frame, text="操作", padding=10)
        action_group.pack(fill=tk.X, pady=5)
        
        ttk.Button(action_group, text="清空日志", command=self.clear_log).pack(fill=tk.X, pady=2)
        ttk.Button(action_group, text="保存地图截图", command=self.save_map).pack(fill=tk.X, pady=2)
        
        # 日志显示组
        log_group = ttk.LabelFrame(left_frame, text="日志", padding=5)
        log_group.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_group, height=10, font=('Consolas', 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # 右侧：地图显示（带滚动条）
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # 创建画布容器
        canvas_container = ttk.Frame(right_frame)
        canvas_container.pack(fill=tk.BOTH, expand=True)
        
        # 滚动条
        v_scrollbar = ttk.Scrollbar(canvas_container, orient=tk.VERTICAL)
        h_scrollbar = ttk.Scrollbar(canvas_container, orient=tk.HORIZONTAL)
        
        # 创建地图画布
        self.map_canvas = MapCanvas(canvas_container)
        self.map_canvas.config(
            yscrollcommand=v_scrollbar.set,
            xscrollcommand=h_scrollbar.set
        )
        
        v_scrollbar.config(command=self.map_canvas.yview)
        h_scrollbar.config(command=self.map_canvas.xview)
        
        # 设置滚动区域
        self.map_canvas.config(scrollregion=self.map_canvas.bbox('all'))
        
        # 布局
        self.map_canvas.grid(row=0, column=0, sticky='nsew')
        v_scrollbar.grid(row=0, column=1, sticky='ns')
        h_scrollbar.grid(row=1, column=0, sticky='ew')
        
        canvas_container.grid_rowconfigure(0, weight=1)
        canvas_container.grid_columnconfigure(0, weight=1)
        
        # 颜色图例
        legend_frame = ttk.LabelFrame(right_frame, text="代价颜色图例", padding=5)
        legend_frame.pack(fill=tk.X, pady=5)
        
        legend_data = [
            ("白色", "#FFFFFF", "0 - 空闲"),
            ("黄色", "#FFEB3B", "15 - 层3惩罚"),
            ("橙色", "#FF9800", "30 - 层2惩罚"),
            ("红色", "#F44336", "60 - 层1惩罚"),
            ("黑色", "#000000", "255 - 障碍物")
        ]
        
        for i, (name, color, desc) in enumerate(legend_data):
            frame = ttk.Frame(legend_frame)
            frame.pack(side=tk.LEFT, padx=5)
            
            color_box = tk.Canvas(frame, width=20, height=20, bg=color, 
                                 highlightthickness=1, highlightbackground='gray')
            color_box.pack(side=tk.LEFT)
            
            ttk.Label(frame, text=desc, font=('Arial', 8)).pack(side=tk.LEFT, padx=5)
        
        # 状态栏
        self.status_label = ttk.Label(self.root, text="就绪", relief=tk.SUNKEN)
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X)
        
    def refresh_ports(self):
        """刷新可用串口列表"""
        ports = serial.tools.list_ports.comports()
        port_list = [f"{port.device} - {port.description}" for port in ports]
        
        if not port_list:
            port_list = ["未找到串口"]
            self.connect_btn.config(state=tk.DISABLED)
        else:
            self.connect_btn.config(state=tk.NORMAL)
        
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.current(0)
    
    def toggle_connection(self):
        """切换串口连接状态"""
        if not self.serial_thread.running:
            # 连接
            port_text = self.port_combo.get()
            if "未找到串口" in port_text:
                return
            
            port = port_text.split(' - ')[0]
            baudrate = int(self.baudrate_combo.get())
            
            if self.serial_thread.set_serial(port, baudrate):
                self.serial_thread.start()
                self.connect_btn.config(text="断开连接", style='Danger.TButton')
                self.port_combo.config(state=tk.DISABLED)
                self.baudrate_combo.config(state=tk.DISABLED)
                self.status_label.config(text=f'已连接到 {port}')
        else:
            # 断开
            self.serial_thread.stop()
            time.sleep(0.2)
            self.connect_btn.config(text="连接串口")
            self.port_combo.config(state='readonly')
            self.baudrate_combo.config(state='readonly')
            self.status_label.config(text='已断开连接')
    
    def on_map_received(self, grid_map):
        """处理接收到的地图数据（线程安全）"""
        self.root.after(0, self._update_map_ui, grid_map)
    
    def _update_map_ui(self, grid_map):
        """在主线程中更新UI"""
        self.map_canvas.update_map(grid_map)
        
        # 更新统计信息
        obstacle_count = np.sum(grid_map == 255)
        penalty_count = np.sum((grid_map > 0) & (grid_map < 255))
        free_count = np.sum(grid_map == 0)
        
        stats_text = f"""地图尺寸: {MAP_HEIGHT} × {MAP_WIDTH}
总格子数: {MAP_HEIGHT * MAP_WIDTH}

障碍物: {obstacle_count} 格
惩罚区: {penalty_count} 格
空闲区: {free_count} 格

代价分布:
  0 (空闲): {free_count}
  1-14:     {np.sum((grid_map >= 1) & (grid_map <= 14))}
  15 (层3): {np.sum(grid_map == 15)}
  30 (层2): {np.sum(grid_map == 30)}
  60 (层1): {np.sum(grid_map == 60)}
  255 (障碍): {obstacle_count}
"""
        
        self.stats_text.config(state=tk.NORMAL)
        self.stats_text.delete(1.0, tk.END)
        self.stats_text.insert(1.0, stats_text)
        self.stats_text.config(state=tk.DISABLED)
    
    def on_message_received(self, message):
        """显示日志消息（线程安全）"""
        self.root.after(0, self._append_log, message)
    
    def _append_log(self, message):
        """在主线程中追加日志"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
    
    def clear_log(self):
        """清空日志"""
        self.log_text.delete(1.0, tk.END)
    
    def save_map(self):
        """保存地图截图"""
        try:
            # 使用Tkinter的postscript生成PS文件，然后转换
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"astar_map_{timestamp}.ps"
            
            # 获取画布的边界
            x = self.map_canvas.winfo_rootx()
            y = self.map_canvas.winfo_rooty()
            
            # 生成postscript文件
            self.map_canvas.postscript(file=filename, colormode='color')
            
            self.on_message_received(f"✅ 地图已保存: {filename}")
            self.on_message_received("💡 提示: 可使用截图工具或PS转换器转为PNG")
            
        except Exception as e:
            messagebox.showerror("保存失败", f"保存地图失败: {str(e)}")
            self.on_message_received(f"❌ 保存失败: {str(e)}")
    
    def on_closing(self):
        """关闭窗口时停止串口线程"""
        if self.serial_thread.running:
            self.serial_thread.stop()
            time.sleep(0.2)
        self.root.destroy()


def main():
    root = tk.Tk()
    app = AStarMapViewer(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == '__main__':
    main()
