#!/usr/bin/env python3

# 这是用来打印单片机mpu6050传感器数据的脚本。
# 它每100ms从指定串口读取数据，并将每条数据写入日志文件，前面加上本地时间戳。

import argparse
import threading
import re
import time
import datetime
import sys
import os
import serial.tools.list_ports

try:
	import serial
	from serial.serialutil import SerialException
except Exception as e:
	print("缺少依赖: pyserial。请通过 `pip install pyserial` 安装。", file=sys.stderr)
	raise


def open_serial(port, baud, timeout=0.02):
	return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def reader_loop(port, baud, outfile, interval_ms, encoding, parse_floats, stop_event):
	"""循环读取串口并将每条原始数据写入 outfile（追加），每行前加本地时间戳。

	outfile: 完整路径，文件以追加模式打开。
	interval_ms: 轮询间隔（毫秒）用于空读等待。
	parse_floats: 若为 True，则尝试从行中提取浮点数并单独记录。
	"""
	interval = max(1, int(interval_ms)) / 1000.0

	# 确保日志目录存在
	try:
		outdir = os.path.dirname(outfile) or os.path.dirname(__file__)
		if outdir:
			os.makedirs(outdir, exist_ok=True)
	except Exception:
		pass

	ser = None
	try:
		ser = open_serial(port, baud, timeout=0.1)
	except Exception as e:
		print(f"无法打开串口 {port} @ {baud}: {e}", file=sys.stderr)

	with open(outfile, 'a', encoding=encoding, errors='replace') as f:
		f.write(f"# Log started: {datetime.datetime.now().isoformat()}\n")
		f.flush()

		# 主循环
		while not stop_event.is_set():
			line = None
			try:
				if ser is not None and getattr(ser, 'is_open', True):
					# 读取一行（包含换行），若超时返回空
					raw = ser.readline()
					if not raw:
						# 空读，睡眠后继续
						time.sleep(interval)
						continue
					try:
						line = raw.decode(encoding, errors='replace')
					except Exception:
						line = str(raw)
				else:
					# 无串口：尝试从标准输入读取一行作为模拟（非阻塞检测）
					r = select_stdin_if_available()
					if r:
						try:
							line = sys.stdin.readline()
						except Exception:
							line = None
					else:
						time.sleep(interval)
						continue

				if line is None:
					continue

				line = line.rstrip('\r\n')
				ts = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
				out_line = f"{ts} - {line}\n"

				# 如果需要尝试解析浮点数，提取并写入一行（保持原样也一并写入）
				if parse_floats:
					floats = re.findall(r"[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?", line)
					if floats:
						f.write(out_line)
						f.write(f"{ts} - parsed_floats: {', '.join(floats)}\n")
					else:
						f.write(out_line)
				else:
					f.write(out_line)

				f.flush()
				# 同步打印到控制台，便于实时观察
				print(out_line, end='')

			except SerialException as e:
				print(f"串口错误: {e}", file=sys.stderr)
				time.sleep(0.5)
			except Exception as e:
				# 捕获并记录所有异常，防止线程退出
				print(f"读取循环异常: {e}", file=sys.stderr)
				time.sleep(0.1)


def select_stdin_if_available():
	"""返回 sys.stdin（可读）或空列表。避免在非交互式环境阻塞。"""
	try:
		import select
		if hasattr(sys.stdin, 'fileno'):
			r, _, _ = select.select([sys.stdin], [], [], 0)
			return r
	except Exception:
		pass
	return []




def main():
	parser = argparse.ArgumentParser(description='每100ms从串口读取数据并记录')
	# 打印所有可用的串口
	ports = [port.device for port in serial.tools.list_ports.comports()]
	print("可用的串口:")
	for p in ports:
		print(f" - {p}")
	parser.add_argument('--port', default=None, help='串口名，例如 COM3 或 /dev/ttyUSB0（可由环境变量 SERIAL_PORT 提供）')
	parser.add_argument('--baud', type=int, default=115200, help='波特率，默认115200')
	parser.add_argument('--outfile', default=None, help='输出日志文件（默认打印到控制台）')
	parser.add_argument('--interval', type=int, default=100, help='轮询间隔ms，默认100')
	parser.add_argument('--encoding', default='utf-8', help='尝试解码串口数据的编码，默认utf-8')
	parser.add_argument('--parse-floats', action='store_true', help='尝试从ASCII连贯流中解析浮点数并单独记录（适用于没有换行的裸数字输出）')
	args = parser.parse_args()

	# 如果没有通过命令行提供 --port，尝试从环境变量 SERIAL_PORT 读取，
	# 否则交互提示用户输入；若用户未输入则退出。
	if args.port is None:
		env_port = os.environ.get('SERIAL_PORT')
		if env_port:
			args.port = env_port
		else:
			try:
				user_port = input('请输入串口端口 (例如 COM3 或 /dev/ttyUSB0)，或直接回车退出: ').strip()
			except Exception:
				user_port = ''
			if not user_port:
				print("未指定串口 (--port) 且未设置 SERIAL_PORT 环境变量，退出。", file=sys.stderr)
				sys.exit(2)
			args.port = user_port
			args.port = user_port

	# 如果没有指定输出文件，则在项目内创建 log 目录并使用基于端口与本地时间的默认文件名
	if args.outfile is None:
		logs_dir = os.path.join(os.path.dirname(__file__), 'log')
		try:
			os.makedirs(logs_dir, exist_ok=True)
		except Exception:
			pass
		ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
		safe_port = (args.port or 'unknown').replace('/', '_').replace('\\', '_')
		args.outfile = os.path.join(logs_dir, f"serial_{safe_port}_{ts}.log")
		print(f"未指定 --outfile，默认将日志写入: {args.outfile}")

	stop_event = threading.Event()
	t = threading.Thread(target=reader_loop, args=(args.port, args.baud, args.outfile, args.interval, args.encoding, args.parse_floats, stop_event), daemon=True)
	t.start()

	try:
		while t.is_alive():
			t.join(timeout=0.5)
	except KeyboardInterrupt:
		print('\n收到中断，正在停止...')
		stop_event.set()
		t.join(timeout=2.0)


if __name__ == '__main__':
	main()
