"""
PID调试工具
- 接收单片机速度数据并实时绘图
- 发送PID参数调整命令
- 自动保存日志文件
"""

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import threading
import time
from datetime import datetime
from collections import deque
import os


class PIDTuner:
    def __init__(self, root):
        self.root = root
        self.root.title("PID调试工具 - 速度监控")
        self.root.geometry("1400x800")
        
        # 串口相关
        self.serial_port = None
        self.is_connected = False
        self.read_thread = None
        self.stop_thread = False
        
        # 数据存储
        self.max_points = 500  # 显示最近500个点
        self.time_data = deque(maxlen=self.max_points)
        self.target_speed = deque(maxlen=self.max_points)
        self.left_speed = deque(maxlen=self.max_points)
        self.right_speed = deque(maxlen=self.max_points)
        self.start_time = time.time()
        
        # 日志文件
        self.log_file = None
        self.log_folder = "log"
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)
        
        # PID参数（用户界面显示的实际值）
        self.pid_params = {
            'pl': 0.0,  # 左轮P
            'il': 0.0,  # 左轮I
            'dl': 0.0,  # 左轮D
            'pr': 0.0,  # 右轮P
            'ir': 0.0,  # 右轮I
            'dr': 0.0,  # 右轮D
            'target_speed': 0.0  # 期望速度
        }
        
        self.create_widgets()
        self.update_plot()
        
    def create_widgets(self):
        # 主容器
        main_container = ttk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 左侧：图表
        left_frame = ttk.Frame(main_container)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 创建图表
        self.fig = Figure(figsize=(10, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel('时间 (s)')
        self.ax.set_ylabel('速度 (mm/s)')
        self.ax.set_title('速度实时监控')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(['目标速度', '左轮速度', '右轮速度'])
        
        self.canvas = FigureCanvasTkAgg(self.fig, left_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 右侧：控制面板
        right_frame = ttk.Frame(main_container, width=350)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(5, 0))
        right_frame.pack_propagate(False)
        
        # 串口连接
        conn_frame = ttk.LabelFrame(right_frame, text="串口连接", padding=10)
        conn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(conn_frame, text="端口:").grid(row=0, column=0, sticky=tk.W)
        self.port_combo = ttk.Combobox(conn_frame, width=15, state='readonly')
        self.port_combo.grid(row=0, column=1, padx=5)
        self.refresh_ports()
        
        ttk.Button(conn_frame, text="刷新", command=self.refresh_ports, width=8).grid(row=0, column=2)
        
        ttk.Label(conn_frame, text="波特率:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.baud_combo = ttk.Combobox(conn_frame, width=15, state='readonly',
                                       values=['9600', '115200', '230400', '460800', '921600'])
        self.baud_combo.set('115200')
        self.baud_combo.grid(row=1, column=1, columnspan=2, padx=5, sticky=tk.W)
        
        self.connect_btn = ttk.Button(conn_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=2, column=0, columnspan=3, pady=5, sticky=tk.EW)
        
        self.status_label = ttk.Label(conn_frame, text="状态: 未连接", foreground="red")
        self.status_label.grid(row=3, column=0, columnspan=3)
        
        # PID参数设置
        pid_frame = ttk.LabelFrame(right_frame, text="PID参数设置", padding=10)
        pid_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # 创建滚动区域
        canvas_scroll = tk.Canvas(pid_frame, height=300)
        scrollbar = ttk.Scrollbar(pid_frame, orient="vertical", command=canvas_scroll.yview)
        scrollable_frame = ttk.Frame(canvas_scroll)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas_scroll.configure(scrollregion=canvas_scroll.bbox("all"))
        )
        
        canvas_scroll.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas_scroll.configure(yscrollcommand=scrollbar.set)
        
        canvas_scroll.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # PID参数输入框
        self.pid_entries = {}
        pid_labels = [
            ('pl', '左轮P', 0.0, 2.5, 0.01),
            ('il', '左轮I', 0.0, 2.5, 0.01),
            ('dl', '左轮D', 0.0, 2.5, 0.01),
            ('pr', '右轮P', 0.0, 2.5, 0.01),
            ('ir', '右轮I', 0.0, 2.5, 0.01),
            ('dr', '右轮D', 0.0, 2.5, 0.01),
            ('target_speed', '目标速度', 0.0, 2550.0, 10.0)
        ]
        
        for i, (key, label, min_val, max_val, step) in enumerate(pid_labels):
            ttk.Label(scrollable_frame, text=f"{label}:").grid(row=i, column=0, sticky=tk.W, pady=3)
            
            entry = ttk.Entry(scrollable_frame, width=10)
            entry.insert(0, "0.0")
            entry.grid(row=i, column=1, padx=5)
            self.pid_entries[key] = entry
            
            # 显示范围
            range_text = f"[{min_val}~{max_val}]"
            ttk.Label(scrollable_frame, text=range_text, font=('Arial', 8)).grid(row=i, column=2, sticky=tk.W)
        
        # 快捷按钮
        btn_frame = ttk.Frame(scrollable_frame)
        btn_frame.grid(row=len(pid_labels), column=0, columnspan=3, pady=10, sticky=tk.EW)
        
        ttk.Button(btn_frame, text="发送PID参数 (0xAA)", 
                  command=self.send_pid_params).pack(fill=tk.X, pady=2)
        ttk.Button(btn_frame, text="开始测试 (0x11)", 
                  command=self.send_start_test).pack(fill=tk.X, pady=2)
        ttk.Button(btn_frame, text="停止测试 (0x22)", 
                  command=self.send_stop_test).pack(fill=tk.X, pady=2)
        ttk.Button(btn_frame, text="清空图表", 
                  command=self.clear_plot).pack(fill=tk.X, pady=2)
        
        # 日志显示
        log_frame = ttk.LabelFrame(right_frame, text="接收日志", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=10, width=40, font=('Consolas', 8))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
    def refresh_ports(self):
        """刷新可用串口"""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.current(0)
    
    def toggle_connection(self):
        """连接/断开串口"""
        if not self.is_connected:
            self.connect_serial()
        else:
            self.disconnect_serial()
    
    def connect_serial(self):
        """连接串口"""
        port = self.port_combo.get()
        baud = int(self.baud_combo.get())
        
        if not port:
            messagebox.showerror("错误", "请选择串口")
            return
        
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            self.is_connected = True
            self.connect_btn.config(text="断开")
            self.status_label.config(text=f"状态: 已连接 {port}", foreground="green")
            
            # 创建日志文件
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_filename = os.path.join(self.log_folder, f"pid_log_{timestamp}.csv")
            self.log_file = open(log_filename, 'w')
            self.log_file.write("时间,目标速度,左轮速度,右轮速度\n")
            
            # 启动读取线程
            self.stop_thread = False
            self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.read_thread.start()
            
            self.log_message(f"已连接到 {port}, 波特率 {baud}")
            self.log_message(f"日志保存至: {log_filename}")
            
        except Exception as e:
            messagebox.showerror("连接失败", f"无法打开串口: {str(e)}")
    
    def disconnect_serial(self):
        """断开串口"""
        self.stop_thread = True
        if self.read_thread:
            self.read_thread.join(timeout=1)
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        
        self.is_connected = False
        self.connect_btn.config(text="连接")
        self.status_label.config(text="状态: 未连接", foreground="red")
        self.log_message("已断开连接")
    
    def read_serial_data(self):
        """读取串口数据线程"""
        buffer = ""
        while not self.stop_thread:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # 处理完整的行
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self.parse_data(line)
                
                time.sleep(0.01)
            except Exception as e:
                self.log_message(f"读取错误: {str(e)}")
                break
    
    def parse_data(self, line):
        """解析接收到的数据"""
        try:
            # 格式: D,target*10,left_speed,right_speed
            if line.startswith('D,'):
                parts = line.split(',')
                if len(parts) == 4:
                    target = int(parts[1]) / 10.0  # 恢复实际速度
                    left = int(parts[2])
                    right = int(parts[3])
                    
                    # 记录时间
                    current_time = time.time() - self.start_time
                    
                    # 添加到数据队列
                    self.time_data.append(current_time)
                    self.target_speed.append(target)
                    self.left_speed.append(left)
                    self.right_speed.append(right)
                    
                    # 写入日志文件
                    if self.log_file:
                        self.log_file.write(f"{current_time:.3f},{target:.1f},{left},{right}\n")
                        self.log_file.flush()
                    
                    # 更新日志显示
                    log_msg = f"[{current_time:.2f}s] 目标:{target:.1f} 左:{left} 右:{right}"
                    self.log_message(log_msg)
            else:
                # 其他消息直接显示
                self.log_message(line)
                
        except Exception as e:
            self.log_message(f"解析错误: {line} - {str(e)}")
    
    def log_message(self, message):
        """在日志窗口显示消息"""
        def append():
            self.log_text.insert(tk.END, message + '\n')
            self.log_text.see(tk.END)
            # 限制日志行数
            lines = int(self.log_text.index('end-1c').split('.')[0])
            if lines > 1000:
                self.log_text.delete('1.0', '500.0')
        
        if threading.current_thread() != threading.main_thread():
            self.root.after(0, append)
        else:
            append()
    
    def update_plot(self):
        """更新图表"""
        try:
            if len(self.time_data) > 0:
                self.ax.clear()
                self.ax.plot(list(self.time_data), list(self.target_speed), 'r-', label='目标速度', linewidth=2)
                self.ax.plot(list(self.time_data), list(self.left_speed), 'b-', label='左轮速度', linewidth=1.5)
                self.ax.plot(list(self.time_data), list(self.right_speed), 'g-', label='右轮速度', linewidth=1.5)
                self.ax.set_xlabel('时间 (s)')
                self.ax.set_ylabel('速度 (mm/s)')
                self.ax.set_title('速度实时监控')
                self.ax.grid(True, alpha=0.3)
                self.ax.legend(loc='upper right')
                
                self.canvas.draw()
        except Exception as e:
            print(f"绘图错误: {e}")
        
        # 100ms更新一次
        self.root.after(100, self.update_plot)
    
    def clear_plot(self):
        """清空图表"""
        self.time_data.clear()
        self.target_speed.clear()
        self.left_speed.clear()
        self.right_speed.clear()
        self.start_time = time.time()
        self.log_message("图表已清空")
    
    def send_pid_params(self):
        """发送PID参数 (0xAA)"""
        if not self.is_connected:
            messagebox.showwarning("警告", "请先连接串口")
            return
        
        try:
            # 读取用户输入的实际值
            pl = float(self.pid_entries['pl'].get())
            il = float(self.pid_entries['il'].get())
            dl = float(self.pid_entries['dl'].get())
            pr = float(self.pid_entries['pr'].get())
            ir = float(self.pid_entries['ir'].get())
            dr = float(self.pid_entries['dr'].get())
            target = float(self.pid_entries['target_speed'].get())
            
            # 转换为单片机接收格式（严格按照协议）
            # 所有PID参数都是 /100.0，所以发送时 *100
            byte1 = int(pl * 100) & 0xFF
            byte2 = int(il * 100) & 0xFF
            byte3 = int(dl * 100) & 0xFF
            byte4 = int(pr * 100) & 0xFF
            byte5 = int(ir * 100) & 0xFF
            byte6 = int(dr * 100) & 0xFF
            # 期望速度是 *10.0，所以发送时 /10
            byte7 = int(target / 10.0) & 0xFF
            
            # 构造命令
            command = bytes([0xAA, byte1, byte2, byte3, byte4, byte5, byte6, byte7])
            
            self.serial_port.write(command)
            self.log_message(f"发送PID参数: PL={pl:.2f} IL={il:.2f} DL={dl:.2f} PR={pr:.2f} IR={ir:.2f} DR={dr:.2f} Speed={target:.1f}")
            self.log_message(f"  -> 字节: {' '.join([f'{b:02X}' for b in command])}")
            
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")
        except Exception as e:
            messagebox.showerror("错误", f"发送失败: {str(e)}")
    
    def send_start_test(self):
        """发送开始测试命令 (0x11)"""
        if not self.is_connected:
            messagebox.showwarning("警告", "请先连接串口")
            return
        
        try:
            target = float(self.pid_entries['target_speed'].get())
            # 速度格式与0xAA统一：发送 target/10，单片机接收后*10
            byte1 = int(target / 10.0) & 0xFF
            
            command = bytes([0x11, byte1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            self.serial_port.write(command)
            self.log_message(f"发送开始测试命令，速度={target:.1f}")
            self.log_message(f"  -> 字节: {' '.join([f'{b:02X}' for b in command])}")
            if target > 2550:
                self.log_message(f"  警告: 速度{target}超过2550，已截断")
            
        except ValueError:
            messagebox.showerror("错误", "请输入有效的目标速度")
        except Exception as e:
            messagebox.showerror("错误", f"发送失败: {str(e)}")
    
    def send_stop_test(self):
        """发送停止测试命令 (0x22)"""
        if not self.is_connected:
            messagebox.showwarning("警告", "请先连接串口")
            return
        
        try:
            command = bytes([0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            self.serial_port.write(command)
            self.log_message("发送停止测试命令")
            
        except Exception as e:
            messagebox.showerror("错误", f"发送失败: {str(e)}")
    
    def on_closing(self):
        """关闭窗口"""
        if self.is_connected:
            self.disconnect_serial()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = PIDTuner(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == '__main__':
    main()
