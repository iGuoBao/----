"""
总线舵机控制上位机
- 支持单个/多个舵机控制
- 可视化滑块控制舵机角度
- 完整的配置指令支持
- 串口通信 115200
"""

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
from datetime import datetime


class ServoController:
    def __init__(self, root):
        self.root = root
        self.root.title("总线舵机控制上位机")
        self.root.geometry("1200x800")
        
        # 串口相关
        self.serial_port = None
        self.is_connected = False
        self.read_thread = None
        self.stop_thread = False
        
        # 舵机列表 (最多支持8个舵机的快捷控制)
        self.servo_count = 6
        self.servo_widgets = []
        
        self.create_widgets()
        
    def create_widgets(self):
        # 主容器
        main_container = ttk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 顶部：串口连接
        top_frame = ttk.LabelFrame(main_container, text="串口连接", padding=10)
        top_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(top_frame, text="端口:").grid(row=0, column=0, padx=5)
        self.port_combo = ttk.Combobox(top_frame, width=15, state='readonly')
        self.port_combo.grid(row=0, column=1, padx=5)
        self.refresh_ports()
        
        ttk.Button(top_frame, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        
        ttk.Label(top_frame, text="波特率:").grid(row=0, column=3, padx=5)
        self.baud_combo = ttk.Combobox(top_frame, width=10, state='readonly')
        self.baud_combo['values'] = ['9600', '19200', '38400', '57600', '115200', '128000', '256000', '1000000']
        self.baud_combo.set('115200')  # 默认115200
        self.baud_combo.grid(row=0, column=4, padx=5)
        
        self.connect_btn = ttk.Button(top_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=10)
        
        self.status_label = ttk.Label(top_frame, text="● 未连接", foreground="red")
        self.status_label.grid(row=0, column=6, padx=10)
        
        # 中间：左右分栏
        middle_frame = ttk.Frame(main_container)
        middle_frame.pack(fill=tk.BOTH, expand=True)
        
        # 左侧：舵机控制区
        left_frame = ttk.Frame(middle_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 舵机控制标签页
        notebook = ttk.Notebook(left_frame)
        notebook.pack(fill=tk.BOTH, expand=True)
        
        # 标签页1：快捷控制
        quick_frame = ttk.Frame(notebook)
        notebook.add(quick_frame, text="快捷控制")
        self.create_quick_control(quick_frame)
        
        # 标签页2：高级配置
        config_frame = ttk.Frame(notebook)
        notebook.add(config_frame, text="高级配置")
        self.create_config_panel(config_frame)
        
        # 标签页3：自定义指令
        custom_frame = ttk.Frame(notebook)
        notebook.add(custom_frame, text="自定义指令")
        self.create_custom_panel(custom_frame)
        
        # 右侧：日志和监控
        right_frame = ttk.Frame(middle_frame, width=400)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(10, 0))
        right_frame.pack_propagate(False)
        
        # 日志显示
        log_frame = ttk.LabelFrame(right_frame, text="通信日志", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_text = scrolledtext.ScrolledText(
            log_frame, 
            height=20, 
            width=50, 
            font=('Consolas', 9)
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # 日志控制
        log_btn_frame = ttk.Frame(log_frame)
        log_btn_frame.pack(fill=tk.X, pady=(5, 0))
        ttk.Button(log_btn_frame, text="清空日志", command=self.clear_log).pack(side=tk.LEFT, padx=2)
        ttk.Button(log_btn_frame, text="保存日志", command=self.save_log).pack(side=tk.LEFT, padx=2)
        
    def create_quick_control(self, parent):
        """创建快捷控制面板"""
        # 滚动区域
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 顶部全局控制
        global_frame = ttk.LabelFrame(scrollable_frame, text="全局控制", padding=10)
        global_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Button(global_frame, text="全部归中位 (1500)", 
                  command=lambda: self.send_all_servos(1500, 1000)).pack(side=tk.LEFT, padx=5)
        ttk.Button(global_frame, text="释放所有扭力", 
                  command=self.release_all_servos).pack(side=tk.LEFT, padx=5)
        ttk.Button(global_frame, text="恢复所有扭力", 
                  command=self.recover_all_servos).pack(side=tk.LEFT, padx=5)
        
        # 创建多个舵机控制组
        for i in range(self.servo_count):
            servo_frame = self.create_servo_control_group(scrollable_frame, i)
            servo_frame.pack(fill=tk.X, padx=10, pady=5)
    
    def create_servo_control_group(self, parent, servo_id):
        """创建单个舵机控制组"""
        frame = ttk.LabelFrame(parent, text=f"舵机 #{servo_id:03d}", padding=10)
        
        # 存储控件引用
        widgets = {'id': servo_id}
        
        # 第一行：使能和当前位置
        row1 = ttk.Frame(frame)
        row1.pack(fill=tk.X, pady=2)
        
        widgets['enabled'] = tk.BooleanVar(value=True)
        ttk.Checkbutton(row1, text="启用", variable=widgets['enabled']).pack(side=tk.LEFT, padx=5)
        
        ttk.Label(row1, text="当前位置:").pack(side=tk.LEFT, padx=5)
        widgets['pos_label'] = ttk.Label(row1, text="1500", font=('Arial', 10, 'bold'))
        widgets['pos_label'].pack(side=tk.LEFT)
        
        ttk.Label(row1, text="PWM").pack(side=tk.LEFT)
        
        # 第二行：位置滑块
        row2 = ttk.Frame(frame)
        row2.pack(fill=tk.X, pady=5)
        
        ttk.Label(row2, text="500").pack(side=tk.LEFT)
        
        widgets['position'] = tk.IntVar(value=1500)
        widgets['slider'] = ttk.Scale(
            row2, 
            from_=0, 
            to=2500, 
            orient=tk.HORIZONTAL,
            variable=widgets['position'],
            command=lambda v, id=servo_id: self.on_slider_change(id, v)
        )
        widgets['slider'].pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        ttk.Label(row2, text="2500").pack(side=tk.LEFT)
        
        # 第三行：时间和快捷按钮
        row3 = ttk.Frame(frame)
        row3.pack(fill=tk.X, pady=2)
        
        ttk.Label(row3, text="时间(ms):").pack(side=tk.LEFT, padx=5)
        widgets['time'] = ttk.Entry(row3, width=6)
        widgets['time'].insert(0, "1000")
        widgets['time'].pack(side=tk.LEFT, padx=5)
        
        ttk.Button(row3, text="执行", 
                  command=lambda id=servo_id: self.execute_servo(id)).pack(side=tk.LEFT, padx=2)
        ttk.Button(row3, text="读位置", 
                  command=lambda id=servo_id: self.read_position(id)).pack(side=tk.LEFT, padx=2)
        ttk.Button(row3, text="释放", 
                  command=lambda id=servo_id: self.release_servo(id)).pack(side=tk.LEFT, padx=2)
        ttk.Button(row3, text="恢复", 
                  command=lambda id=servo_id: self.recover_servo(id)).pack(side=tk.LEFT, padx=2)
        
        self.servo_widgets.append(widgets)
        return frame
    
    def create_config_panel(self, parent):
        """创建配置面板"""
        # 滚动区域
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # ID设置
        id_frame = ttk.LabelFrame(scrollable_frame, text="ID管理", padding=10)
        id_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(id_frame, text="旧ID:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.old_id_entry = ttk.Entry(id_frame, width=10)
        self.old_id_entry.insert(0, "255")
        self.old_id_entry.grid(row=0, column=1, padx=5)
        
        ttk.Label(id_frame, text="新ID:").grid(row=0, column=2, sticky=tk.W, padx=5)
        self.new_id_entry = ttk.Entry(id_frame, width=10)
        self.new_id_entry.insert(0, "0")
        self.new_id_entry.grid(row=0, column=3, padx=5)
        
        ttk.Button(id_frame, text="修改ID", command=self.change_id).grid(row=0, column=4, padx=10)
        
        ttk.Label(id_frame, text="提示: 广播ID=255可修改未知ID的舵机", 
                 foreground="blue", font=('Arial', 9)).grid(row=1, column=0, columnspan=6, sticky=tk.W, padx=5, pady=5)
        
        # 工作模式
        mode_frame = ttk.LabelFrame(scrollable_frame, text="工作模式设置", padding=10)
        mode_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 第一行：ID和下拉选择
        ttk.Label(mode_frame, text="舵机ID:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.mode_id_entry = ttk.Entry(mode_frame, width=10)
        self.mode_id_entry.insert(0, "0")
        self.mode_id_entry.grid(row=0, column=1, padx=5)
        
        ttk.Label(mode_frame, text="模式:").grid(row=0, column=2, sticky=tk.W, padx=5)
        self.mode_combo = ttk.Combobox(mode_frame, width=25, state='readonly')
        self.mode_combo['values'] = [
            "1-舵机270°顺时针", "2-舵机270°逆时针",
            "3-舵机180°顺时针", "4-舵机180°逆时针",
            "5-马达360°定圈顺时针", "6-马达360°定圈逆时针",
            "7-马达360°定时顺时针", "8-马达360°定时逆时针"
        ]
        self.mode_combo.current(0)
        self.mode_combo.grid(row=0, column=3, padx=5)
        
        ttk.Button(mode_frame, text="设置选中模式", command=self.set_mode).grid(row=0, column=4, padx=10)
        
        # 第二行：快捷模式按钮
        ttk.Label(mode_frame, text="快捷设置:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        
        quick_modes = [
            ("舵机270°CW", 1),
            ("舵机270°CCW", 2),
            ("舵机180°CW", 3),
            ("舵机180°CCW", 4),
        ]
        
        for i, (name, mode) in enumerate(quick_modes):
            btn = ttk.Button(
                mode_frame, 
                text=name,
                command=lambda m=mode: self.set_mode_direct(m),
                width=12
            )
            btn.grid(row=1, column=i+1, padx=2, pady=5)
        
        # 第三行：广播模式设置
        ttk.Label(mode_frame, text="批量设置:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=5)
        ttk.Button(mode_frame, text="广播设置模式1 (所有舵机)",
                  command=lambda: self.set_mode_broadcast(1),
                  width=25).grid(row=2, column=1, columnspan=2, padx=5, pady=5, sticky=tk.W)
        
        # 校准和限位
        calib_frame = ttk.LabelFrame(scrollable_frame, text="校准和限位", padding=10)
        calib_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(calib_frame, text="舵机ID:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.calib_id_entry = ttk.Entry(calib_frame, width=10)
        self.calib_id_entry.insert(0, "0")
        self.calib_id_entry.grid(row=0, column=1, padx=5)
        
        ttk.Button(calib_frame, text="校正中值(1500)", 
                  command=self.calibrate_center).grid(row=0, column=2, padx=5)
        ttk.Button(calib_frame, text="设置最小值", 
                  command=self.set_min_pos).grid(row=0, column=3, padx=5)
        ttk.Button(calib_frame, text="设置最大值", 
                  command=self.set_max_pos).grid(row=0, column=4, padx=5)
        
        # 系统指令
        sys_frame = ttk.LabelFrame(scrollable_frame, text="系统指令", padding=10)
        sys_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(sys_frame, text="舵机ID:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.sys_id_entry = ttk.Entry(sys_frame, width=10)
        self.sys_id_entry.insert(0, "0")
        self.sys_id_entry.grid(row=0, column=1, padx=5)
        
        ttk.Button(sys_frame, text="读取版本", 
                  command=self.read_version).grid(row=0, column=2, padx=5)
        ttk.Button(sys_frame, text="读取温度电压", 
                  command=self.read_temp_voltage).grid(row=0, column=3, padx=5)
        
        # 波特率设置
        ttk.Label(sys_frame, text="波特率:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.baud_set_combo = ttk.Combobox(sys_frame, width=15, state='readonly')
        self.baud_set_combo['values'] = [
            "1-9600", "2-19200", "3-38400", "4-57600",
            "5-115200", "6-128000", "7-256000", "8-1000000"
        ]
        self.baud_set_combo.current(4)  # 默认115200
        self.baud_set_combo.grid(row=1, column=1, padx=5)
        
        ttk.Button(sys_frame, text="设置波特率", 
                  command=self.set_baudrate).grid(row=1, column=2, padx=5)
        ttk.Label(sys_frame, text="注意:设置后需重启舵机", 
                 foreground="red", font=('Arial', 8)).grid(row=1, column=3, sticky=tk.W, padx=5)
        
        ttk.Button(sys_frame, text="部分恢复出厂", 
                  command=self.partial_reset).grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky=tk.EW)
        ttk.Button(sys_frame, text="完全恢复出厂", 
                  command=self.full_reset).grid(row=2, column=2, columnspan=2, padx=5, pady=5, sticky=tk.EW)
    
    def create_custom_panel(self, parent):
        """创建自定义指令面板"""
        frame = ttk.Frame(parent, padding=10)
        frame.pack(fill=tk.BOTH, expand=True)
        
        # 说明
        info_text = """
指令格式说明：
• 单个舵机: #000P1500T1000!
• 多个舵机: {G0000#000P1500T1000!#001P2000T1000!}
• ID必须3位，PWM必须4位，时间必须4位
        """
        ttk.Label(frame, text=info_text, justify=tk.LEFT).pack(anchor=tk.W, pady=5)
        
        # 输入框
        ttk.Label(frame, text="自定义指令:").pack(anchor=tk.W, pady=5)
        self.custom_cmd_entry = ttk.Entry(frame, width=60, font=('Consolas', 10))
        self.custom_cmd_entry.pack(fill=tk.X, pady=5)
        
        # 按钮
        btn_frame = ttk.Frame(frame)
        btn_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(btn_frame, text="发送指令", command=self.send_custom_command).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="清空", 
                  command=lambda: self.custom_cmd_entry.delete(0, tk.END)).pack(side=tk.LEFT, padx=5)
        
        # 常用指令模板
        template_frame = ttk.LabelFrame(frame, text="常用模板", padding=10)
        template_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        templates = [
            ("单舵机控制", "#000P1500T1000!"),
            ("双舵机控制", "{G0000#000P1500T1000!#001P2000T1000!}"),
            ("读取位置", "#000PRAD!"),
            ("释放扭力", "#000PULK!"),
            ("恢复扭力", "#000PULR!"),
            ("暂停", "#000PDPT!"),
            ("继续", "#000PDCT!"),
            ("停止", "#000PDST!"),
        ]
        
        for i, (name, cmd) in enumerate(templates):
            row = i // 2
            col = i % 2
            btn = ttk.Button(
                template_frame, 
                text=name,
                command=lambda c=cmd: self.custom_cmd_entry.insert(0, c)
            )
            btn.grid(row=row, column=col, padx=5, pady=5, sticky=tk.EW)
        
        template_frame.columnconfigure(0, weight=1)
        template_frame.columnconfigure(1, weight=1)
    
    def refresh_ports(self):
        """刷新串口列表"""
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
        if not port:
            messagebox.showerror("错误", "请选择串口")
            return
        
        try:
            baud = int(self.baud_combo.get())
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            self.is_connected = True
            self.connect_btn.config(text="断开")
            self.status_label.config(text=f"● 已连接 ({baud})", foreground="green")
            
            # 禁用波特率选择
            self.baud_combo.config(state='disabled')
            
            # 启动读取线程
            self.stop_thread = False
            self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.read_thread.start()
            
            self.log_message(f"已连接到 {port}, 波特率 {baud}")
            
        except Exception as e:
            messagebox.showerror("连接失败", f"无法打开串口: {str(e)}")
    
    def disconnect_serial(self):
        """断开串口"""
        self.stop_thread = True
        if self.read_thread:
            self.read_thread.join(timeout=1)
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        self.is_connected = False
        self.connect_btn.config(text="连接")
        self.status_label.config(text="● 未连接", foreground="red")
        
        # 恢复波特率选择
        self.baud_combo.config(state='readonly')
        
        self.log_message("已断开连接")
    
    def read_serial_data(self):
        """读取串口数据"""
        buffer = ""
        while not self.stop_thread:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # 处理完整的响应（以!结尾）
                    while '!' in buffer:
                        response, buffer = buffer.split('!', 1)
                        response = response.strip() + '!'
                        if response.startswith('#'):
                            self.log_message(f"← {response}")
                
                time.sleep(0.01)
            except Exception as e:
                self.log_message(f"读取错误: {str(e)}")
                break
    
    def send_command(self, command):
        """发送指令"""
        if not self.is_connected:
            messagebox.showwarning("警告", "请先连接串口")
            return False
        
        try:
            self.serial_port.write(command.encode('utf-8'))
            self.log_message(f"→ {command}")
            return True
        except Exception as e:
            self.log_message(f"发送失败: {str(e)}")
            return False
    
    def format_id(self, servo_id):
        """格式化ID为3位"""
        return f"{int(servo_id):03d}"
    
    def format_pwm(self, pwm):
        """格式化PWM为4位"""
        return f"{int(pwm):04d}"
    
    def format_time(self, time_ms):
        """格式化时间为4位"""
        return f"{int(time_ms):04d}"
    
    def on_slider_change(self, servo_id, value):
        """滑块变化"""
        value = int(float(value))
        self.servo_widgets[servo_id]['pos_label'].config(text=str(value))
    
    def execute_servo(self, servo_id):
        """执行舵机动作"""
        widgets = self.servo_widgets[servo_id]
        if not widgets['enabled'].get():
            return
        
        position = widgets['position'].get()
        time_ms = widgets['time'].get()
        
        try:
            time_ms = int(time_ms)
            cmd = f"#{self.format_id(servo_id)}P{self.format_pwm(position)}T{self.format_time(time_ms)}!"
            self.send_command(cmd)
        except ValueError:
            messagebox.showerror("错误", "时间必须是数字")
    
    def read_position(self, servo_id):
        """读取舵机位置"""
        cmd = f"#{self.format_id(servo_id)}PRAD!"
        self.send_command(cmd)
    
    def release_servo(self, servo_id):
        """释放舵机"""
        cmd = f"#{self.format_id(servo_id)}PULK!"
        self.send_command(cmd)
    
    def recover_servo(self, servo_id):
        """恢复舵机"""
        cmd = f"#{self.format_id(servo_id)}PULR!"
        self.send_command(cmd)
    
    def send_all_servos(self, position, time_ms):
        """控制所有舵机"""
        commands = []
        for i in range(self.servo_count):
            if self.servo_widgets[i]['enabled'].get():
                commands.append(f"#{self.format_id(i)}P{self.format_pwm(position)}T{self.format_time(time_ms)}!")
        
        if commands:
            if len(commands) == 1:
                self.send_command(commands[0])
            else:
                multi_cmd = "{G0000" + "".join(commands) + "}"
                self.send_command(multi_cmd)
    
    def release_all_servos(self):
        """释放所有舵机"""
        for i in range(self.servo_count):
            if self.servo_widgets[i]['enabled'].get():
                self.release_servo(i)
                time.sleep(0.05)
    
    def recover_all_servos(self):
        """恢复所有舵机"""
        for i in range(self.servo_count):
            if self.servo_widgets[i]['enabled'].get():
                self.recover_servo(i)
                time.sleep(0.05)
    
    def change_id(self):
        """修改ID"""
        try:
            old_id = int(self.old_id_entry.get())
            new_id = int(self.new_id_entry.get())
            
            if old_id == 255:
                if not messagebox.askyesno("确认", 
                    f"使用广播ID(255)将修改总线上所有舵机的ID为{new_id}！\n\n"
                    f"警告: 如果有多个舵机，它们将变成相同ID，导致冲突！\n\n"
                    f"建议: 每次只连接一个舵机进行ID设置。\n\n是否继续？"):
                    return
            
            cmd = f"#{self.format_id(old_id)}PID{self.format_id(new_id)}!"
            self.send_command(cmd)
            self.log_message(f"修改ID: #{old_id:03d} -> #{new_id:03d}")
        except ValueError:
            messagebox.showerror("错误", "ID必须是0-254之间的数字")
    
    def set_mode_direct(self, mode):
        """直接设置模式（快捷按钮用）"""
        try:
            servo_id = int(self.mode_id_entry.get())
            cmd = f"#{self.format_id(servo_id)}PMOD{mode}!"
            self.send_command(cmd)
            self.log_message(f"快捷设置: 舵机#{servo_id} -> 模式{mode}")
        except ValueError:
            messagebox.showerror("错误", "ID必须是数字")
    
    def set_mode_broadcast(self, mode):
        """广播设置模式（所有舵机）"""
        if messagebox.askyesno("确认", f"将所有舵机设置为模式{mode}？\n\n这将影响总线上的所有舵机！"):
            cmd = f"#{self.format_id(255)}PMOD{mode}!"
            self.send_command(cmd)
            self.log_message(f"广播命令: 设置所有舵机为模式{mode}")
    
    def set_mode(self):
        """设置工作模式"""
        try:
            servo_id = int(self.mode_id_entry.get())
            mode = int(self.mode_combo.get().split('-')[0])
            cmd = f"#{self.format_id(servo_id)}PMOD{mode}!"
            self.send_command(cmd)
        except (ValueError, IndexError):
            messagebox.showerror("错误", "请选择有效的模式")
    
    def read_mode(self):
        """读取工作模式"""
        try:
            servo_id = int(self.mode_id_entry.get())
            cmd = f"#{self.format_id(servo_id)}PMOD!"
            self.send_command(cmd)
        except ValueError:
            messagebox.showerror("错误", "ID必须是数字")
    
    def calibrate_center(self):
        """校正中值"""
        try:
            servo_id = int(self.calib_id_entry.get())
            cmd = f"#{self.format_id(servo_id)}PSCK!"
            self.send_command(cmd)
        except ValueError:
            messagebox.showerror("错误", "ID必须是数字")
    
    def set_min_pos(self):
        """设置最小值"""
        try:
            servo_id = int(self.calib_id_entry.get())
            cmd = f"#{self.format_id(servo_id)}PSMI!"
            self.send_command(cmd)
        except ValueError:
            messagebox.showerror("错误", "ID必须是数字")
    
    def set_max_pos(self):
        """设置最大值"""
        try:
            servo_id = int(self.calib_id_entry.get())
            cmd = f"#{self.format_id(servo_id)}PSMX!"
            self.send_command(cmd)
        except ValueError:
            messagebox.showerror("错误", "ID必须是数字")
    
    def read_version(self):
        """读取版本"""
        try:
            servo_id = int(self.sys_id_entry.get())
            cmd = f"#{self.format_id(servo_id)}PVER!"
            self.send_command(cmd)
        except ValueError:
            messagebox.showerror("错误", "ID必须是数字")
    
    def read_temp_voltage(self):
        """读取温度和电压"""
        try:
            servo_id = int(self.sys_id_entry.get())
            cmd = f"#{self.format_id(servo_id)}PRTV!"
            self.send_command(cmd)
        except ValueError:
            messagebox.showerror("错误", "ID必须是数字")
    
    def set_baudrate(self):
        """设置舵机波特率"""
        try:
            servo_id = int(self.sys_id_entry.get())
            baud_str = self.baud_set_combo.get()
            baud_code = int(baud_str.split('-')[0])
            baud_value = baud_str.split('-')[1]
            
            if messagebox.askyesno("确认", 
                f"将舵机#{servo_id}的波特率设置为{baud_value}？\n\n"
                f"警告: 设置后舵机将使用新波特率通信！\n"
                f"需要重启舵机并重新连接上位机。\n\n"
                f"是否继续？"):
                cmd = f"#{self.format_id(servo_id)}PBD{baud_code}!"
                self.send_command(cmd)
                self.log_message(f"设置波特率: 舵机#{servo_id} -> {baud_value}")
                messagebox.showinfo("提示", 
                    f"已发送波特率设置命令！\n\n"
                    f"请重启舵机电源，并使用新波特率{baud_value}重新连接。")
        except (ValueError, IndexError):
            messagebox.showerror("错误", "请选择有效的波特率")
    
    def partial_reset(self):
        """部分恢复出厂"""
        try:
            servo_id = int(self.sys_id_entry.get())
            if messagebox.askyesno("确认", f"确定要恢复舵机#{servo_id}的出厂设置吗？(保留ID)"):
                cmd = f"#{self.format_id(servo_id)}PCLE0!"
                self.send_command(cmd)
        except ValueError:
            messagebox.showerror("错误", "ID必须是数字")
    
    def full_reset(self):
        """完全恢复出厂"""
        try:
            servo_id = int(self.sys_id_entry.get())
            if messagebox.askyesno("确认", f"确定要完全恢复舵机#{servo_id}的出厂设置吗？(ID将变为000)"):
                cmd = f"#{self.format_id(servo_id)}PCLE!"
                self.send_command(cmd)
        except ValueError:
            messagebox.showerror("错误", "ID必须是数字")
    
    def send_custom_command(self):
        """发送自定义指令"""
        cmd = self.custom_cmd_entry.get().strip()
        if cmd:
            self.send_command(cmd)
        else:
            messagebox.showwarning("警告", "请输入指令")
    
    def log_message(self, message):
        """记录日志"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        log_entry = f"[{timestamp}] {message}\n"
        
        def append():
            self.log_text.insert(tk.END, log_entry)
            self.log_text.see(tk.END)
            # 限制日志行数
            lines = int(self.log_text.index('end-1c').split('.')[0])
            if lines > 1000:
                self.log_text.delete('1.0', '500.0')
        
        if threading.current_thread() != threading.main_thread():
            self.root.after(0, append)
        else:
            append()
    
    def clear_log(self):
        """清空日志"""
        self.log_text.delete(1.0, tk.END)
    
    def save_log(self):
        """保存日志"""
        from tkinter import filedialog
        filename = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("文本文件", "*.txt"), ("所有文件", "*.*")]
        )
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(self.log_text.get(1.0, tk.END))
                messagebox.showinfo("成功", "日志已保存")
            except Exception as e:
                messagebox.showerror("错误", f"保存失败: {str(e)}")
    
    def on_closing(self):
        """关闭窗口"""
        if self.is_connected:
            self.disconnect_serial()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = ServoController(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == '__main__':
    main()
