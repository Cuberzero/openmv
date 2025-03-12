import sensor, image, time, math
from pyb import Pin, Timer, UART
import kinematics, robotMoveCmd


class Apriltag():
    sensor.reset()  # 初始化摄像头
    sensor.set_pixformat(sensor.RGB565)  # 图像格式为 RGB565 灰度 GRAYSCALE
    sensor.set_framesize(sensor.QQVGA)  # QQVGA: 160x120
    sensor.skip_frames(n=2000)  # 在更改设置后，跳过n张照片，等待感光元件变稳定
    sensor.set_auto_gain(True)  # 使用颜色识别时需要关闭自动自动增益
    sensor.set_auto_whitebal(True)  # 使用颜色识别时需要关闭自动自动白平衡

    red_threshold = (0, 100, 20, 127, 0, 127)
    blue_threshold = (0, 100, -128, 127, -128, -15)
    green_threshold = (0, 100, -128, -28, 0, 70)
    yellow_threshold = (57, 100, -33, 70, 48, 127)

    uart = UART(3, 115200)  # 设置串口波特率，与stm32一致
    uart.init(115200, bits=8, parity=None, stop=1)

    tim = Timer(4, freq=1000)  # Frequency in Hz
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
    led_dac.pulse_width_percent(0)

    cap_color_status = 0  # 抓取物块颜色标志，用来判断物块抓取
    # 机械臂移动位置
    move_x = 0
    move_y = 160

    mid_block_cx = 80.5
    mid_block_cy = 60.5

    mid_block_cnt = 0  # 用来记录机械臂已对准物块计数，防止误差

    move_status = 0  # 机械臂移动的方式

    # 基准点
    datum_mark_cx = 200
    datum_mark_cy = 120

    block_cnt = 0  # 记录码垛物块数量

    # 用来记录已经抓取到标签
    apriltag_succeed_flag = 0

    block_degress = 0  # 机械爪旋转角度

    cap_color_status = 0  # 抓取物块颜色标志，用来判断物块抓取的顺序

    car_spin_status = 0  # 用来判断小车旋转状态

    # 机器人运动
    robot_move_cmd = robotMoveCmd.RobotMoveCmd()
    kinematic = kinematics.Kinematics()

    def init(self, cx=80.5, cy=60.5):  # 初始化巡线配置，传入两个参数调整中位值
        sensor.reset()  # 初始化摄像头
        sensor.set_pixformat(sensor.RGB565)  # 图像格式为 RGB565 灰度 GRAYSCALE
        sensor.set_framesize(sensor.QQVGA)  # QQVGA: 160x120
        sensor.skip_frames(n=2000)  # 在更改设置后，跳过n张照片，等待感光元件变稳定
        sensor.set_auto_gain(True)  # 使用颜色识别时需要关闭自动自动增益
        sensor.set_auto_whitebal(True)  # 使用颜色识别时需要关闭自动自动白平衡

        self.uart.init(115200, bits=8, parity=None, stop=1)
        self.led_dac.pulse_width_percent(0)

        self.cap_color_status = 0  # 抓取物块颜色标志，用来判断物块抓取
        # 机械臂移动位置
        self.move_x = 0
        self.move_y = 160

        self.mid_block_cx = cx
        self.mid_block_cy = cy

        self.mid_block_cnt = 0  # 用来记录机械臂已对准物块计数，防止误差

        self.move_status = 0  # 机械臂移动的方式

        # 寻找码垛得基准点
        self.datum_mark_cx = 200
        self.datum_mark_cy = 120

        self.block_cnt = 0  # 记录码垛物块数量

        # 用来记录已经抓取到标签
        self.apriltag_succeed_flag = 0

        self.cap_color_status = 0  # 抓取物块颜色标志，用来判断物块抓取的顺序

        self.block_degress = 0  # 机械爪旋转角度

        self.car_spin_status = 0  # 用来判断小车旋转状态

        self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
        time.sleep_ms(1000)

    def run_sort(self):  # 分拣
        # 物块中心点
        block_cx = self.mid_block_cx
        block_cy = self.mid_block_cy
        color_read_succed = 0  # 是否识别到颜色
        # 获取图像
        img = sensor.snapshot()

        if self.apriltag_succeed_flag == 0:  # 识别抓取标签
            for tag in img.find_apriltags():  # defaults to TAG36H11 without "families".
                img.draw_rectangle(tag.rect(), color=(255, 0, 0))  # 画框
                img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))  # 画十字
                img.draw_string(tag[0], (tag[1] - 10), "{}".format(tag.id()), color=(255, 0, 0))
                block_cx = tag.cx()
                block_cy = tag.cy()
                self.block_degress = 180 * tag.rotation() / math.pi  # 求April Tags旋转的角度
                self.cap_color_status = tag.id()
                color_read_succed = 1

        elif self.apriltag_succeed_flag == 1:  # 抓取到标签后颜色分拣
            red_blobs = img.find_blobs([self.red_threshold], x_stride=15, y_stride=15, pixels_threshold=25)
            blue_blobs = img.find_blobs([self.blue_threshold], x_stride=15, y_stride=15, pixels_threshold=25)
            green_blobs = img.find_blobs([self.green_threshold], x_stride=15, y_stride=15, pixels_threshold=25)
            # ***************首先进行色块检测********************
            if red_blobs and self.cap_color_status == 1:  # 红色
                color_read_succed = 1
                for y in red_blobs:
                    img.draw_rectangle((y[0], y[1], y[2], y[3]), color=(255, 255, 255))
                    img.draw_cross(y[5], y[6], size=2, color=(255, 0, 0))
                    img.draw_string(y[0], (y[1] - 10), "red", color=(255, 0, 0))
                    # print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","红色")
                    block_cx = y[5]
                    block_cy = y[6]

            elif blue_blobs and self.cap_color_status == 2:  # 蓝色
                color_read_succed = 1
                for y in blue_blobs:
                    img.draw_rectangle((y[0], y[1], y[2], y[3]), color=(255, 255, 255))
                    img.draw_cross(y[5], y[6], size=2, color=(0, 0, 255))
                    img.draw_string(y[0], (y[1] - 10), "blue", color=(0, 0, 255))
                    # print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","蓝色")
                    block_cx = y[5]
                    block_cy = y[6]

            elif green_blobs and self.cap_color_status == 3:  # 绿色
                color_read_succed = 1
                for y in green_blobs:
                    img.draw_rectangle((y[0], y[1], y[2], y[3]), color=(255, 255, 255))
                    img.draw_cross(y[5], y[6], size=2, color=(0, 255, 0))
                    img.draw_string(y[0], (y[1] - 10), "green", color=(0, 255, 0))
                    # print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","绿色")
                    block_cx = y[5]
                    block_cy = y[6]

        # 识别不到物块，旋转小车
        if color_read_succed == 0 and (self.move_status == 0 or self.move_status == 3):
            if self.car_spin_status != 1:
                self.car_spin_status = 1
                self.robot_move_cmd.car_move(-0.1, 0.1, -0.1, 0.1)

        # ************************************************ 运动机械臂*************************************************************************************
        if color_read_succed == 1:  # 识别到颜色
            if self.move_status == 0:  # 第0阶段：小车调整位置
                if block_cx - self.mid_block_cx > 10:  # 右转
                    if self.car_spin_status != 1:
                        self.car_spin_status = 1
                        self.robot_move_cmd.car_move(0.05, -0.05, 0.05, -0.05)
                elif block_cx - self.mid_block_cx < -10:  # 左转
                    if self.car_spin_status != 2:
                        self.car_spin_status = 2
                        self.robot_move_cmd.car_move(-0.05, 0.05, -0.05, 0.05)
                else:
                    if self.car_spin_status != 3:
                        self.car_spin_status = 3
                        self.robot_move_cmd.car_move(0, 0, 0, 0)
                    self.move_status = 1

            if self.move_status == 1:  # 第1阶段：机械臂寻找物块位置
                if (abs(block_cx - self.mid_block_cx) > 2):
                    if block_cx > self.mid_block_cx:
                        self.move_x += 0.2
                    else:
                        self.move_x -= 0.2
                if (abs(block_cy - self.mid_block_cy) > 2):
                    if block_cy > self.mid_block_cy and self.move_y > 80:
                        self.move_y -= 0.3
                    else:
                        self.move_y += 0.3
                if abs(block_cy - self.mid_block_cy) <= 2 and abs(block_cx - self.mid_block_cx) <= 2:  # 寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 50:  # 计数100次对准物块，防止误差
                        self.mid_block_cnt = 0
                        self.move_status = 2
                else:
                    self.mid_block_cnt = 0
                    self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 0)
                time.sleep_ms(10)

            elif self.move_status == 2:  # 第2阶段：机械臂抓取物块
                self.move_status = 3
                # 判断矩形倾角，改变机械爪
                spin_calw = 1500
                if self.block_degress % 90 < 45:
                    spin_calw = int(1500 - self.block_degress % 90 * 500 / 90)
                else:
                    spin_calw = int((90 - self.block_degress % 90) * 500 / 90 + 1500)

                if spin_calw >= 2500 and spin_calw <= 500:
                    spin_calw = 1500
                self.kinematic.send_str("{{#004P{:0^4}T1000!}}".format(spin_calw))  # 旋转和张开机械爪
                l = math.sqrt(self.move_x * self.move_x + self.move_y * self.move_y)
                sin = self.move_y / l
                cos = self.move_x / l
                self.move_x = (l + 76) * cos
                self.move_y = (l + 76) * sin
                time.sleep_ms(100)
                self.kinematic.send_str("{#005P1000T1000!}")
                time.sleep_ms(100)
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)  # 移动机械臂到物块上方
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x, self.move_y, -48, 1000)  # 移动机械臂下移到物块
                time.sleep_ms(1200)
                self.kinematic.send_str("{#005P1700T1000!}")  # 机械爪抓取物块
                time.sleep_ms(1200)
                self.kinematic.send_str("{#004P1500T1000!}")  # 旋转和张开机械爪
                # 机械臂复位
                self.move_x = 0
                self.move_y = 160
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
                time.sleep_ms(1200)
                self.robot_move_cmd.car_move(-0.1, 0.1, -0.1, 0.1)# 小车右转
                self.mid_block_cnt = 0
                self.apriltag_succeed_flag = 1  # 抓取到标签后颜色分拣

            elif self.move_status == 3:  # 第3阶段：小车旋转寻找放下物块的框框
                if block_cx - self.mid_block_cx > 10:  # 右转
                    if self.car_spin_status != 1:
                        self.car_spin_status = 1
                        self.robot_move_cmd.car_move(0.05, -0.05, 0.05, -0.05)
                elif block_cx - self.mid_block_cx < -10:  # 左转
                    if self.car_spin_status != 2:
                        self.car_spin_status = 2
                        self.robot_move_cmd.car_move(-0.05, 0.05, -0.05, 0.05)
                else:
                    if self.car_spin_status != 3:
                        self.car_spin_status = 3
                        self.robot_move_cmd.car_move(0, 0, 0, 0)
                    self.move_status = 4

            elif self.move_status == 4:  # 第4阶段：小车调整位置
                if (abs(block_cx - self.mid_block_cx) > 5):
                    if block_cx > self.mid_block_cx:
                        self.move_x += 0.3
                    else:
                        self.move_x -= 0.3
                if (abs(block_cy - self.mid_block_cy) > 5):
                    if block_cy > self.mid_block_cy and self.move_y > 1:
                        self.move_y -= 0.2
                    else:
                        self.move_y += 0.2
                if abs(block_cy - self.mid_block_cy) <= 5 and abs(block_cx - self.mid_block_cx) <= 5:  # 寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 5:  # 计数5次对准物块，防止误差
                        self.mid_block_cnt = 0
                        self.move_status = 5
                else:
                    self.mid_block_cnt = 0
                    self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 0)
                time.sleep_ms(10)

            elif self.move_status == 5:  # 第5阶段：机械臂放下物块并归位
                self.move_status = 0
                l = math.sqrt(self.move_x * self.move_x + self.move_y * self.move_y)
                sin = self.move_y / l
                cos = self.move_x / l
                self.move_x = (l + 79) * cos
                self.move_y = (l + 79) * sin
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)  # 移动机械臂到物块上方
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x, self.move_y, -43, 1000)  # 移动机械臂下移到物块
                time.sleep_ms(1200)
                self.kinematic.send_str("{#005P1000T1000!}")  # 机械爪放下物块
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)  # 移动机械臂抬起
                time.sleep_ms(1200)
                self.move_x = 0  # 机械臂归位
                self.move_y = 100
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
                time.sleep_ms(1200)
                self.mid_block_cnt = 0
                self.cap_color_status = 0
                self.apriltag_succeed_flag = 0  # 识别抓取标签

    def run_stack(self):  # 码垛
        # 物块中心点
        block_cx = self.mid_block_cx
        block_cy = self.mid_block_cy
        block_read_succed = 0  # 是否识别到物块
        # 获取图像
        img = sensor.snapshot()
        # 识别抓取标签
        for tag in img.find_apriltags():  # defaults to TAG36H11 without "families".
            img.draw_rectangle(tag.rect(), color=(255, 0, 0))  # 画框
            img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))  # 画十字
            img.draw_string(tag[0], (tag[1] - 10), "{}".format(tag.id()), color=(255, 0, 0))
            block_cx = tag.cx()
            block_cy = tag.cy()
            self.block_degress = 180 * tag.rotation() / math.pi  # 求April Tags旋转的角度
            block_read_succed = 1

        # ************************************************ 运动机械臂*************************************************************************************
        if block_read_succed == 1:  # 识别到颜色或者到路口

            if self.move_status == 0:  # 第0阶段：机械臂寻找物块位置
                if (abs(block_cx - self.mid_block_cx) > 2):
                    if block_cx > self.mid_block_cx:
                        self.move_x += 0.2
                    else:
                        self.move_x -= 0.2
                if (abs(block_cy - self.mid_block_cy) > 2):
                    if block_cy > self.mid_block_cy and self.move_y > 80:
                        self.move_y -= 0.3
                    else:
                        self.move_y += 0.3
                if abs(block_cy - self.mid_block_cy) <= 2 and abs(block_cx - self.mid_block_cx) <= 2:  # 寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 50:  # 计数100次对准物块，防止误差
                        self.mid_block_cnt = 0
                        self.move_status = 1
                else:
                    self.mid_block_cnt = 0
                    self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 0)
                time.sleep_ms(10)

            elif self.move_status == 1:  # 第1阶段：机械臂抓取物块，并码垛
                self.move_status = 2
                # 判断矩形倾角，改变机械爪
                spin_calw = 1500
                if self.block_degress % 90 < 45:
                    spin_calw = int(1500 - self.block_degress % 90 * 500 / 90)
                else:
                    spin_calw = int((90 - self.block_degress % 90) * 500 / 90 + 1500)

                if spin_calw >= 2500 and spin_calw <= 500:
                    spin_calw = 1500
                self.kinematic.send_str("{{#004P{:0^4}T1000!}}".format(spin_calw))  # 旋转和张开机械爪
                l = math.sqrt(self.move_x * self.move_x + self.move_y * self.move_y)
                sin = self.move_y / l
                cos = self.move_x / l
                self.move_x = (l + 76) * cos
                self.move_y = (l + 76) * sin
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)  # 移动机械臂到物块上方
                self.kinematic.send_str("{#005P1000T1000!}")
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x, self.move_y, -48, 1000)  # 移动机械臂下移到物块
                time.sleep_ms(1200)
                self.kinematic.send_str("{#005P1700T1000!}")  # 机械爪抓取物块
                time.sleep_ms(1200)
                self.kinematic.send_str("{#004P1500T1000!}")  # 旋转和张开机械爪
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)  # 移动机械臂抬起
                time.sleep_ms(1200)
                # 机械臂旋转到要方向物块的指定位置，码垛基点
                self.move_x = self.datum_mark_cx
                self.move_y = self.datum_mark_cy
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
                time.sleep_ms(1200)
                self.move_status = 0
                if self.block_cnt == 0:
                    self.block_cnt = 1
                    self.kinematic.kinematics_move(self.move_x, self.move_y, -48, 1000)  # 移动机械臂到物块上方
                elif self.block_cnt == 1:
                    self.block_cnt = 2
                    # 标记基准点
                    l = math.sqrt(self.datum_mark_cx * self.datum_mark_cx + self.datum_mark_cy * self.datum_mark_cy)
                    sin = self.datum_mark_cy / l
                    cos = self.datum_mark_cx / l
                    self.move_x = (l - 1) * cos
                    self.move_y = (l - 1) * sin
                    self.kinematic.kinematics_move(self.move_x, self.move_y, -20, 1000)  # 移动机械臂到物块上方
                elif self.block_cnt == 2:
                    self.block_cnt = 0
                    # 标记基准点
                    l = math.sqrt(self.datum_mark_cx * self.datum_mark_cx + self.datum_mark_cy * self.datum_mark_cy)
                    sin = self.datum_mark_cy / l
                    cos = self.datum_mark_cx / l
                    self.move_x = (l - 11) * cos
                    self.move_y = (l - 11) * sin
                    self.kinematic.kinematics_move(self.move_x, self.move_y, 5, 1000)  # 移动机械臂到物块上方
                time.sleep_ms(1200)
                self.kinematic.send_str("{#005P1000T1000!}")  # 机械爪放下物块
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)  # 移动机械臂抬起
                time.sleep_ms(1200)
                self.move_x = 0  # 机械臂归位
                self.move_y = 160
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
                time.sleep_ms(1200)
                self.mid_block_cnt = 0
                self.cap_color_status = 0
