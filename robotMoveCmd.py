from pyb import UART
import time


class RobotMoveCmd():
	uart = UART(3, 115200)  # 设置串口波特率，与stm32一致
	uart.init(115200, bits=8, parity=None, stop=1)

	def send_cmd(self, cmd: str):
		self.uart.read()  # 读取（清空）缓冲区数据

		for i in range(10):  # 发送指令告诉下位机需要回传动作组是否执行完成
			self.uart.write(cmd)
			for j in range(10):
				time.sleep_ms(1)
				if self.uart.any():  # 接收指令
					try:  # 用来判断串口数据异常
						string = self.uart.read()
						if string:
							string = string.decode()
							if string.find("cmdOk") >= 0:  # 接收到下位机返回成功指令
								return 1
					except Exception as e:  # 串口数据异常进入
						print('Error:', e)
		return -1

		#   功能：设置小车运动状态

	def car_move(self, speed1, speed2, speed3, speed4):
		self.send_cmd("$Car:{},{},{},{}!".format(float(speed1), float(speed2), float(speed3), float(speed4)))
