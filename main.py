from pyb import UART, Pin, Timer
import sensor, time, pyb
import colorTrace, mapLineWalk, apriltag, faceTrack

color_trace = colorTrace.ColorTrace()
map_line_walk = mapLineWalk.MapLineWalk()
april_tag = apriltag.Apriltag()
face_track = faceTrack.FaceTrack()

led = pyb.LED(3)

uart = UART(3, 115200)  # 设置串口波特率，与stm32一致
uart.init(115200, bits=8, parity=None, stop=1)

tim = Timer(4, freq=1000)  # Frequency in Hz
led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
led_dac.pulse_width_percent(0)

run_app_status = 0

uart.write("#openmv reset!")

# 调整中点，用于调整机械臂抓取物块中点
# 如果机械臂抓取偏左，mid_block_cx减小，反之增加
# 如果机械臂抓取偏前，mid_block_cy减小，反之增加
# 如果机械臂抓取偏下或偏上，就调节2号舵机偏差
mid_block_cx = 80.5
mid_block_cy = 60.5

while (True):
	if uart.any():  # 接收指令
		try:  # 用来判断串口数据异常
			string = uart.read()
			if string:
				string = string.decode()
				# print(string, run_app_status,string.find("#start_led!"))
				if string.find("#StartLed!") >= 0:  # 开灯指令
					led_dac.pulse_width_percent(100)
				elif string.find("#StopLed!") >= 0:  # 关灯指令
					led_dac.pulse_width_percent(0)
				elif string.find("#RunStop!") >= 0:  # 停止所有运行并复位
					run_app_status = 0
					for j in range(3):
						uart.write("$MV0!")
						time.sleep_ms(100)
					uart.write("$Car:0,0,0,0!\n")  # 发送控制小车行动的指令
					uart.write("$KMS:0,160,50,1000!\n")
					led_dac.pulse_width_percent(0)
				elif string.find("#CarColorTrace!") >= 0:  # 小车颜色追踪
					run_app_status = 1
					for j in range(3):
						uart.write("$MV1!")
						time.sleep_ms(100)
					color_trace.init(mid_block_cx, mid_block_cy)
				elif string.find("#Map3LineWalk!") >= 0:  # 巡线地图3
					run_app_status = 2
					for j in range(3):
						uart.write("$MV2!")
						time.sleep_ms(100)
					map_line_walk.init()
					self.kinematic.kinematics_move(0, 160, 50, 1000)
					time.sleep_ms(1000)
				elif string.find("#ArmColorTrace!") >= 0:  # 机械臂颜色追踪
					run_app_status = 3
					for j in range(3):
						uart.write("$MV3!")
						time.sleep_ms(100)
					color_trace.init(mid_block_cx, mid_block_cy)
				elif string.find("#AprilTagSort!") >= 0:  # 二维码标签分拣
					run_app_status = 4
					for j in range(3):
						uart.write("$MV4!")
						time.sleep_ms(100)
					april_tag.init(mid_block_cx, mid_block_cy)
				elif string.find("#AprilTagStack!") >= 0:  # 二维码标签码垛
					run_app_status = 5
					for j in range(3):
						uart.write("$MV5!")
						time.sleep_ms(100)
					april_tag.init(mid_block_cx, mid_block_cy)
				elif string.find("#TraceGrasp!") >= 0:  # 颜色追踪抓取
					run_app_status = 6
					for j in range(3):
						uart.write("$MV6!")
						time.sleep_ms(100)
					color_trace.init(mid_block_cx, mid_block_cy)
				elif string.find("#Map1LineWalk!") >= 0:  # 地图1巡线
					run_app_status = 7
					for j in range(3):
						uart.write("$MV7!")
						time.sleep_ms(100)
					map_line_walk.init(mid_block_cx, mid_block_cy)
				elif string.find("#Map2LineWalk!") >= 0:  # 地图2巡线
					run_app_status = 8
					for j in range(3):
						uart.write("$MV8!")
						time.sleep_ms(100)
					map_line_walk.init(mid_block_cx, mid_block_cy)
				elif string.find("#FaceTrack!") >= 0:  # 人脸识别
					run_app_status = 9
					for j in range(3):
						uart.write("$MV9!")
						time.sleep_ms(100)
					face_track.init()
		except Exception as e:  # 串口数据异常进入
			print('Error:', e)

	if run_app_status == 1:
		color_trace.run_dynamic_trace()  # 运行小车颜色追踪
	elif run_app_status == 2:
		map_line_walk.run_map3()  # 运行地图3巡线功能
	elif run_app_status == 3:
		color_trace.run_state_trace()  # 运行平台颜色追踪
	elif run_app_status == 4:
		april_tag.run_sort()  # 运行二维码标签分拣
	elif run_app_status == 5:
		april_tag.run_stack()  # 运行二维码标签码垛
	elif run_app_status == 6:
		color_trace.run_trace_grasp()  # 运行颜色追踪抓取功能，需要换上黑金大爪子
	elif run_app_status == 7:
		map_line_walk.run_map1()  # 运行地图1巡线功能
	elif run_app_status == 8:
		map_line_walk.run_map2()  # 运行地图2巡线功能
	elif run_app_status == 9:
		face_track.run_track()  # 运行人脸追踪功能
