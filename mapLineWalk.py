import sensor, image, time, math
from pyb import Pin, Timer, UART
import kinematics, robotMoveCmd


class MapLineWalk():
    sensor.reset()  # 初始化摄像头
    sensor.set_pixformat(sensor.RGB565)  # 图像格式为 RGB565 灰度 GRAYSCALE
    sensor.set_framesize(sensor.QQVGA)  # QQVGA: 160x120
    sensor.skip_frames(n=2000)  # 在更改设置后，跳过n张照片，等待感光元件变稳定
    sensor.set_auto_gain(True)  # 使用颜色识别时需要关闭自动自动增益
    sensor.set_auto_whitebal(True)  # 使用颜色识别时需要关闭自动自动白平衡

    # 设置阈值，如果是黑线，GRAYSCALE_THRESHOLD = [(0, 64)]；
    # 如果是白线，GRAYSCALE_THRESHOLD = [(128，255)]
    GRAYSCALE_THRESHOLD = [(0, 64)]

    # 每个roi为(x, y, w, h)，线检测算法将尝试找到每个roi中最大的blob的质心。
    # 然后用不同的权重对质心的x位置求平均值，其中最大的权重分配给靠近图像底部的roi，
    # 较小的权重分配给下一个roi，以此类推。
    ROIS = [  # [ROI, weight]
        (0, 100, 200, 20, 0.5, 1),  # 你需要为你的应用程序调整权重
        (0, 75, 200, 20, 0.2, 2),
        (0, 50, 200, 20, 0.2, 3),  # 取决于你的机器人是如何设置的。
        (0, 25, 200, 20, 0.1, 4),
        (0, 000, 200, 20, 0.1, 5)
    ]
    # roi代表三个取样区域，（x,y,w,h,weight）,代表左上顶点（x,y）宽高分别为w和h的矩形，
    # weight为当前矩形的权值。注意本例程采用的QQVGA图像大小为160x120，roi即把图像横分成三个矩形。
    # 三个矩形的阈值要根据实际情况进行调整，离机器人视野最近的矩形权值要最大，
    # 如上图的最下方的矩形，即(0, 100, 200, 20, 0.7,1)（最后一个值1是用来记录的）

    red_threshold = (0, 100, 20, 127, 0, 127)
    blue_threshold = (0, 100, -128, 127, -128, -15)
    green_threshold = (0, 100, -128, -28, 0, 70)

    uart = UART(3, 115200)  # 设置串口波特率，与stm32一致
    uart.init(115200, bits=8, parity=None, stop=1)

    tim = Timer(4, freq=1000)  # Frequency in Hz
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
    led_dac.pulse_width_percent(100)

    cap_color_status = 0  # 抓取物块颜色标志，用来判断物块抓取
    # 机械臂移动位置
    move_x = 0
    move_y = 160
    mid_block_cx = 80.5
    mid_block_cy = 60.5
    mid_block_cnt = 0  # 用来记录机械臂已对准物块计数，防止误差
    move_status = 0  # 机械臂移动的方式

    adjust_position = 0  # 用调整身位

    cap_block_cnt = 0  # 用来计数抓取物块数量

    crossing_flag = 0  # 标记路口情况计数，判断是否经过一个路口

    is_line_flag = 1  # 是否可以巡线标志

    over_flag = 0  # 用来标记小车180度翻转
    mid_over_flag = 0  # 记录小车翻转到一半
    mid_over_cnt = 0  # 记录小车翻转到一半计数
    mid_adjust_position = 0  # 小车到中间横线时需要调整身位后在寻找分拣区，变量为标志位
    adjust_position_cnt = 0  # 调整身位计数
    car_back_flag = 0  # 小车后退还是前进标志
    crossing_record_cnt = 0  # 用来记录经过的路口数量

    crossing_cnt = 0  # MAP3路口计数

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
        self.led_dac.pulse_width_percent(100)

        self.cap_color_status = 0  # 抓取物块颜色标志，用来判断物块抓取
        # 机械臂移动位置
        self.move_x = 0
        self.move_y = 160
        self.mid_block_cx = cx
        self.mid_block_cy = cy
        self.mid_block_cnt = 0  # 用来记录机械臂已对准物块计数，防止误差
        self.move_status = 0  # 机械臂移动的方式

        self.adjust_position = 0  # 用调整身位

        self.cap_block_cnt = 0  # 用来计数抓取物块数量

        self.crossing_flag = 0  # 标记路口情况计数，判断是否经过一个路口

        self.is_line_flag = 1  # 是否可以巡线标志

        self.over_flag = 0  # 用来标记小车180度翻转
        self.mid_over_flag = 0  # 记录小车翻转到一半
        self.mid_over_cnt = 0  # 记录小车翻转到一半计数
        self.mid_adjust_position = 0  # 小车到中间横线时需要调整身位后在寻找分拣区，变量为标志位
        self.adjust_position_cnt = 0  # 调整身位计数
        self.car_back_flag = 0  # 小车后退还是前进标志
        self.crossing_record_cnt = 0  # 用来记录经过的路口数量

        self.crossing_cnt = 0  # MAP3路口计数
        self.speed_motor1 = 0
        self.speed_motor2 = 0
        self.speed_motor3 = 0
        self.speed_motor4 = 0

        self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
        time.sleep_ms(1000)

    def line_walk(self, img):  # 巡线功能,传入img是二值化的图像
        weight_sum = 0  # 权值和初始化
        centroid_sum = 0

        # 计算基础速度
        base_speed = 0.3  # 基础速度，可以根据需要调整

        # 记录寻找到的黑线块
        blob_roi1 = 0  # 记录最大的块
        blob_roi2 = 0
        blob_roi3 = 0
        blob_roi4 = 0
        blob_roi5 = 0

        # 利用颜色识别分别寻找三个矩形区域内的线段
        for r in self.ROIS:
            blobs = img.find_blobs(self.GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True)
            # 目标区域找到直线
            if blobs:
                # 查找像素最多的blob的索引。
                largest_blob = 0
                most_pixels = 0
                for i in range(len(blobs)):
                    # 目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
                    if blobs[i].pixels() > most_pixels:
                        most_pixels = blobs[i].pixels()
                        largest_blob = i
                if blobs[largest_blob].w() > 10 and blobs[largest_blob].h() > 5:  # 太小说明检测到虚线之类的,略过
                    # 在色块周围画一个矩形。
                    img.draw_rectangle(blobs[largest_blob].rect())
                    img.draw_cross(blobs[largest_blob].cx(),
                                   blobs[largest_blob].cy())  # 将此区域的像素数最大的颜色块画矩形和十字形标记出来
                    img.draw_string(blobs[largest_blob].x(), (blobs[largest_blob].y()), "{}".format(r[5]),
                                    color=(255, 255, 255))

                    if r[5] == 1:
                        blob_roi1 = blobs[largest_blob].rect()
                    elif r[5] == 2:
                        blob_roi2 = blobs[largest_blob].rect()
                    elif r[5] == 3:
                        blob_roi3 = blobs[largest_blob].rect()
                    elif r[5] == 4:
                        blob_roi4 = blobs[largest_blob].rect()
                    elif r[5] == 5:
                        blob_roi5 = blobs[largest_blob].rect()

                    centroid_sum += blobs[largest_blob].cx() * r[4]  # r[4] is the roi weight.
                    weight_sum += r[4]

        if self.over_flag == 1:  # 原地翻转
            if blob_roi3 == 0 and blob_roi5 == 0 and self.mid_over_flag == 0:  # 摄像头已经识别不到线，说明翻转到一半
                self.mid_over_cnt += 1
                if self.mid_over_cnt > 5:
                    self.mid_over_flag = 1
                    self.mid_over_cnt = 0
            elif blob_roi1 != 0 and blob_roi3 != 0 and blob_roi5 != 0 and self.mid_over_flag == 1:  # 重新识别到三条范围内的线，取消翻转，重新开始巡线
                self.over_flag = 0
                self.mid_over_flag = 0
                time.sleep_ms(200)
            self.robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)
            return

        # **********************************判断路口情况，检测两个自定义的范围都检测到路口就说明经过一个路口***********************************************
        if self.crossing_flag == 0 and (
                (blob_roi3 != 0 and blob_roi3[2] > 90) or (blob_roi4 != 0 and blob_roi4[2] > 90)):
            self.crossing_flag = 1
        elif self.crossing_flag == 1 and blob_roi1 != 0 and blob_roi1[2] > 90:  # 1号ROIS检测到路口
            self.crossing_flag = 2

        if weight_sum > 0 and self.is_line_flag == 1:  # 如果识别到线条或者没有特殊情况，开始巡线

            center_pos = (centroid_sum / weight_sum)  # Determine center of line.

            # 将center_pos转换为一个偏角。我们用的是非线性运算，所以越偏离直线，响应越强。
            # 非线性操作很适合用于这样的算法的输出，以引起响应“触发器”。
            deflection_angle = 0
            # 机器人应该转的角度

            # 80是X的一半，60是Y的一半。
            # 下面的等式只是计算三角形的角度，其中三角形的另一边是中心位置与中心的偏差，相邻边是Y的一半。
            # 这样会将角度输出限制在-45至45度左右。（不完全是-45至45度）。

            deflection_angle = -math.atan((center_pos - 80) / 60)
            # 角度计算.80 60 分别为图像宽和高的一半，图像大小为QQVGA 160x120.
            # 注意计算得到的是弧度值

            deflection_angle = math.degrees(deflection_angle)
            # 将计算结果的弧度值转化为角度值

            # 现在你有一个角度来告诉你该如何转动机器人。
            # 通过该角度可以合并最靠近机器人的部分直线和远离机器人的部分直线，以实现更好的预测。

            # print("Turn Angle: %f" % deflection_angle)\

            if self.mid_adjust_position == 1:  # 调整身位
                if blob_roi5 != 0 and blob_roi5[2] > 100 and abs(deflection_angle) < 5:  # 身位调整完毕
                    self.adjust_position_cnt += 1
                    self.robot_move_cmd.car_move(0, 0, 0, 0)
                    if self.adjust_position_cnt > 50:  # 调整身位成功
                        self.adjust_position_cnt = 0
                        self.is_line_flag = 0
                        self.car_back_flag = 0
                        self.mid_adjust_position = 0
                        self.move_x = 120  # 旋转机械臂寻找颜色框
                        if self.crossing_record_cnt == 3:  # 第三个路口的机械臂旋转的方向不同
                            self.move_x = -120
                        self.move_y = 60
                        self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)
                        time.sleep_ms(1200)
                    return
                # 小车身位太靠前，需后退
                elif blob_roi5 == 0 or (blob_roi4 != 0 and blob_roi4[2] > 100) or (
                        blob_roi3 != 0 and blob_roi3[2] > 100) or (blob_roi2 != 0 and blob_roi2[2] > 100) or (
                        blob_roi1 != 0 and blob_roi1[2] > 100):
                    self.car_back_flag = 1
                elif blob_roi5 != 0 and blob_roi5[2] < 50:  # 小车倒退出线
                    self.car_back_flag = 0
                self.adjust_position_cnt = 0

            if self.mid_adjust_position == 1 or self.car_back_flag == 1:  # map2的地图调整
                if self.car_back_flag == 1 and self.crossing_record_cnt != 12:  # 小车需倒退
                    if deflection_angle < -6:  # 右转
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = -base_speed / 2
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = -base_speed / 2
                    elif deflection_angle > 6:  # 左
                        self.speed_motor1 = -base_speed / 2
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = -base_speed / 2
                        self.speed_motor4 = -base_speed / 3
                    else:
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = -base_speed / 3
                elif self.car_back_flag == 0 and self.crossing_record_cnt != 12:
                    # 计算每个电机的速度，根据偏转角度进行调整
                    if deflection_angle < -10:  # 右转
                        self.speed_motor1 = base_speed / 3
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = base_speed / 3
                        self.speed_motor4 = -base_speed / 3
                    elif deflection_angle > 10:  # 左转
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = base_speed / 3
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = base_speed / 3
                    else:
                        self.speed_motor1 = base_speed / 3
                        self.speed_motor2 = base_speed / 3
                        self.speed_motor3 = base_speed / 3
                        self.speed_motor4 = base_speed / 3
                elif self.car_back_flag == 1 and self.crossing_record_cnt == 12:
                    if deflection_angle < -20:  # 右转
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = -base_speed / 2
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = -base_speed / 2
                    elif deflection_angle > 20:  # 左
                        self.speed_motor1 = -base_speed / 2
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = -base_speed / 2
                        self.speed_motor4 = -base_speed / 3
                    else:
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = -base_speed / 3

            else:
                if deflection_angle < 0:  # 右转
                    # 右侧电机速度增加，左侧电机速度减少
                    self.speed_motor1 = base_speed * (1 + abs(deflection_angle) / 45)
                    self.speed_motor2 = base_speed * (1 - abs(deflection_angle) / 45)
                    self.speed_motor3 = base_speed * (1 + abs(deflection_angle) / 45)
                    self.speed_motor4 = base_speed * (1 - abs(deflection_angle) / 45)
                else:  # 左转
                    # 左侧电机速度增加，右侧电机速度减少
                    self.speed_motor1 = base_speed * (1 - abs(deflection_angle) / 45)
                    self.speed_motor2 = base_speed * (1 + abs(deflection_angle) / 45)
                    self.speed_motor3 = base_speed * (1 - abs(deflection_angle) / 45)
                    self.speed_motor4 = base_speed * (1 + abs(deflection_angle) / 45)

            # 打印四个电机的速度.
            # print("Motor deflection_angle:", deflection_angle)
            # print("Motor Speeds (m/s):")
            # print("Motor 1:", speed_motor1)
            # print("Motor 2:", speed_motor2)
            # print("Motor 3:", speed_motor3)
            # print("Motor 4:", speed_motor4)

            self.robot_move_cmd.car_move(self.speed_motor1, self.speed_motor2, self.speed_motor3, self.speed_motor4)

        # ************************************************ 出线暂停*************************************************************************************
        else:
            self.robot_move_cmd.car_move(0, 0, 0, 0)
            time.sleep_ms(500)

    def run_map1(self):  # 地图1巡线分拣
        # 物块中心点
        block_cx = self.mid_block_cx
        block_cy = self.mid_block_cy

        color_read_succeed = 0  # 是否识别到颜色
        color_status = 0

        # 获取图像
        img = sensor.snapshot()
        red_blobs = img.find_blobs([self.red_threshold], x_stride=15, y_stride=15, pixels_threshold=25)
        blue_blobs = img.find_blobs([self.blue_threshold], x_stride=15, y_stride=15, pixels_threshold=25)
        green_blobs = img.find_blobs([self.green_threshold], x_stride=15, y_stride=15, pixels_threshold=25)

        if self.cap_block_cnt >= 3:  # 抓取3个物块后停止机器人
            return

        # ***************首先进行色块检测，如果没有检测到色块，那就寻线********************
        if red_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'R'):  # 红色
            color_status = 'R'
            color_read_succeed = 1
            for y in red_blobs:
                img.draw_rectangle((y[0], y[1], y[2], y[3]), color=(255, 255, 255))
                img.draw_cross(y[5], y[6], size=2, color=(255, 0, 0))
                img.draw_string(y[0], (y[1] - 10), "red", color=(255, 0, 0))
                # print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","红色")
                block_cx = y[5]
                block_cy = y[6]

        elif blue_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'B'):  # 蓝色
            color_status = 'B'
            color_read_succeed = 1
            for y in blue_blobs:
                img.draw_rectangle((y[0], y[1], y[2], y[3]), color=(255, 255, 255))
                img.draw_cross(y[5], y[6], size=2, color=(0, 0, 255))
                img.draw_string(y[0], (y[1] - 10), "blue", color=(0, 0, 255))
                # print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","蓝色")
                block_cx = y[5]
                block_cy = y[6]

        elif green_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'G'):  # 绿色
            color_status = 'G'
            color_read_succeed = 1
            for y in green_blobs:
                img.draw_rectangle((y[0], y[1], y[2], y[3]), color=(255, 255, 255))
                img.draw_cross(y[5], y[6], size=2, color=(0, 255, 0))
                img.draw_string(y[0], (y[1] - 10), "green", color=(0, 255, 0))
                # print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","绿色")
                block_cx = y[5]
                block_cy = y[6]

        else:  # 没有检测到色块，巡线检测
            img = img.to_grayscale()
            self.line_walk(img)

        # ************************************************ 运动机械臂*************************************************************************************
        if color_read_succeed == 1 or (self.move_status == 3 and self.crossing_flag == 2):  # 识别到颜色或者到路口
            if self.move_status == 0:  # 第0阶段：首先调整机器人身位
                if block_cy - self.mid_block_cy < -10:  # 前进
                    self.mid_block_cnt = 0
                    if self.adjust_position != 1:
                        self.adjust_position = 1
                        self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)
                elif block_cy - self.mid_block_cy > 10:  # 后退
                    self.mid_block_cnt = 0
                    if self.adjust_position != 4:
                        self.adjust_position = 4
                        self.robot_move_cmd.car_move(-0.1, -0.1, -0.1, -0.1)
                elif block_cx - self.mid_block_cx > 10:  # 右转
                    self.mid_block_cnt = 0
                    if self.adjust_position != 3:
                        self.adjust_position = 3
                        self.robot_move_cmd.car_move(0.1, -0.05, 0.1, -0.05)
                elif block_cx - self.mid_block_cx < -10:  # 左转
                    self.mid_block_cnt = 0
                    if self.adjust_position != 2:
                        self.adjust_position = 2
                        self.robot_move_cmd.car_move(-0.05, 0.1, -0.05, 0.1)
                else:  # 调整完毕，停止
                    self.robot_move_cmd.car_move(0, 0, 0, 0)
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 10:
                        self.adjust_position = 0
                        self.move_status = 1
                        self.is_line_flag = 0
                        self.mid_block_cnt = 0
                        time.sleep_ms(2000)

            elif self.move_status == 1:  # 第1阶段：机械臂寻找物块位置
                if (abs(block_cx - self.mid_block_cx) > 2):
                    if block_cx > self.mid_block_cx:
                        self.move_x += 0.5
                    else:
                        self.move_x -= 0.5
                if (abs(block_cy - self.mid_block_cy) > 2):
                    if block_cy > self.mid_block_cy and self.move_y > 1:
                        self.move_y -= 0.3
                    else:
                        self.move_y += 0.3
                if abs(block_cy - self.mid_block_cy) <= 2 and abs(block_cx - self.mid_block_cx) <= 2:  # 寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 50:  # 计数100次对准物块，防止误差
                        self.mid_block_cnt = 0
                        self.move_status = 2
                        self.cap_color_status = color_status
                else:
                    self.mid_block_cnt = 0
                    self.kinematic.kinematics_move(self.move_x, self.move_y, 40, 0)
                time.sleep_ms(10)

            elif self.move_status == 2:  # 第2阶段：机械臂抓取物块
                self.move_status = 3
                # 判断矩形倾角，改变机械爪
                for r in img.find_rects():
                    if r.corners()[0][0] < block_cx and r.corners()[0][1] > block_cy and r.corners()[2][
                        0] > block_cx and r.corners()[2][1] < block_cy:  # 判断矩形是否为物块
                        p0 = r.corners()[0]
                        p1 = r.corners()[1]
                        p2 = r.corners()[2]
                        p3 = r.corners()[3]
                        spin_calw = 1500
                        if abs(p2[1] - p3[1]) > 2:  # 计算物块对齐角度
                            spin_calw += (p2[1] - p3[1]) * 15
                        elif abs(p3[0] - p0[0]) > 2:
                            spin_calw += (p3[0] - p0[0]) * 15
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
                self.kinematic.kinematics_move(self.move_x, self.move_y, -45, 1000)  # 移动机械臂下移到物块
                time.sleep_ms(1200)
                self.kinematic.send_str("{#005P1700T1000!}")  # 机械爪抓取物块
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)  # 移动机械臂抬起
                time.sleep_ms(1200)
                self.move_x = 0  # 机械臂归位
                self.move_y = 160
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
                time.sleep_ms(1200)
                self.crossing_flag = 0
                self.mid_block_cnt = 0
                self.is_line_flag = 1  # 机器人开始巡线

            elif self.move_status == 3:  # 第3阶段：机器人寻找路口找出色框
                if self.crossing_flag == 2:
                    self.is_line_flag = 0  # 机器人停止巡线
                    self.robot_move_cmd.car_move(0, 0, 0, 0)
                    # 检测色框是否和色块一样
                    self.move_x = 100  # 机械臂向右旋转
                    self.move_y = 150
                    self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)
                    time.sleep_ms(1200)
                    if self.cap_color_status == color_status:  # 色框和色块一样
                        self.mid_block_cnt += 1
                    else:
                        self.mid_block_cnt -= 1
                    if self.mid_block_cnt > 3:  # 色框和色块一样
                        self.move_status = 4
                        self.mid_block_cnt = 0
                    elif self.mid_block_cnt < -3:  # 颜色框和物块不同
                        self.move_x = 0  # 归位
                        self.move_y = 160
                        self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
                        time.sleep_ms(1200)
                        self.is_line_flag = 1  # 机器人开始巡线
                        self.crossing_flag = 0
                        self.mid_block_cnt = 0

            elif self.move_status == 4:  # 第4阶段：机械臂寻找放下物块的框框
                if (abs(block_cx - self.mid_block_cx) > 2):
                    if block_cx > self.mid_block_cx and self.move_y > 1:
                        self.move_y -= 0.3
                    else:
                        self.move_y += 0.3
                if (abs(block_cy - self.mid_block_cy) > 2):
                    if block_cy > self.mid_block_cy and self.move_y > 1:
                        self.move_x -= 0.2
                    else:
                        self.move_x += 0.2
                if abs(block_cy - self.mid_block_cy) <= 3 and abs(block_cx - self.mid_block_cx) <= 3:  # 寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 5:  # 计数5次对准物块，防止误差
                        self.mid_block_cnt = 0
                        self.move_status = 5
                        self.cap_color_status = color_status
                else:
                    self.mid_block_cnt = 0
                    self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 0)
                time.sleep_ms(10)

            elif self.move_status == 5:  # 第5阶段：机械臂放下物块并归位
                self.move_status = 0
                l = math.sqrt(self.move_x * self.move_x + self.move_y * self.move_y)
                sin = self.move_y / l
                cos = self.move_x / l
                self.move_x = (l + 86) * cos
                self.move_y = (l + 86) * sin
                self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)  # 移动机械臂到物块上方
                time.sleep_ms(1500)
                self.kinematic.kinematics_move(self.move_x, self.move_y, -20, 1000)  # 移动机械臂下移到物块
                time.sleep_ms(1500)
                self.kinematic.send_str("{#005P1000T1000!}")  # 机械爪放下物块
                time.sleep_ms(1500)
                self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)  # 移动机械臂抬起
                time.sleep_ms(1500)
                self.move_x = 0  # 机械臂归位
                self.move_y = 160
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
                time.sleep_ms(1500)
                self.mid_block_cnt = 0
                self.is_line_flag = 1  # 机器人开始巡线
                self.adjust_position = 0  # 第一次开始启动
                self.cap_block_cnt += 1  # 记录抓取色块的数量
                self.cap_color_status = 0

    def run_map2(self):  # 地图2巡线分拣
        # 物块中心点
        block_cx = self.mid_block_cx
        block_cy = self.mid_block_cy

        color_read_succeed = 0  # 是否识别到颜色
        color_status = 0

        # 获取图像
        img = sensor.snapshot()

        # ***************首先巡线找路口，计算路口位置后在识别夹取色块********************
        if self.is_line_flag == 1:
            self.line_walk(img.to_grayscale())

        else:
            red_blobs = img.find_blobs([self.red_threshold], x_stride=15, y_stride=15, pixels_threshold=25)
            blue_blobs = img.find_blobs([self.blue_threshold], x_stride=15, y_stride=15, pixels_threshold=25)
            green_blobs = img.find_blobs([self.green_threshold], x_stride=15, y_stride=15, pixels_threshold=25)

            if red_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'R'):  # 红色
                color_status = 'R'
                color_read_succeed = 1
                for y in red_blobs:
                    img.draw_rectangle((y[0], y[1], y[2], y[3]), color=(255, 255, 255))
                    img.draw_cross(y[5], y[6], size=2, color=(255, 0, 0))
                    img.draw_string(y[0], (y[1] - 10), "red", color=(255, 0, 0))
                    # print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","红色")
                    block_cx = y[5]
                    block_cy = y[6]

            elif blue_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'B'):  # 蓝色
                color_status = 'B'
                color_read_succeed = 1
                for y in blue_blobs:
                    img.draw_rectangle((y[0], y[1], y[2], y[3]), color=(255, 255, 255))
                    img.draw_cross(y[5], y[6], size=2, color=(0, 0, 255))
                    img.draw_string(y[0], (y[1] - 10), "blue", color=(0, 0, 255))
                    # print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","蓝色")
                    block_cx = y[5]
                    block_cy = y[6]

            elif green_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'G'):  # 绿色
                color_status = 'G'
                color_read_succeed = 1
                for y in green_blobs:
                    img.draw_rectangle((y[0], y[1], y[2], y[3]), color=(255, 255, 255))
                    img.draw_cross(y[5], y[6], size=2, color=(0, 255, 0))
                    img.draw_string(y[0], (y[1] - 10), "green", color=(0, 255, 0))
                    # print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","绿色")
                    block_cx = y[5]
                    block_cy = y[6]

        # **********************************判断路口情况***********************************************
        if self.crossing_flag == 2:  # 找到路口
            self.crossing_flag = 0
            self.crossing_record_cnt += 1
            if self.crossing_record_cnt == 2 or self.crossing_record_cnt == 5 or self.crossing_record_cnt == 9:  # 经过的路口数量在物品区，小车停止，开始颜色识别
                self.is_line_flag = 0
                self.robot_move_cmd.car_move(0, 0, 0, 0)
            elif self.crossing_record_cnt == 3:  # 第3个路口小车需要右转,改为颜色识别，旋转机械臂到左边
                self.mid_adjust_position = 1
                self.robot_move_cmd.car_move(0.3, 0.3, 0.3, 0.3)
                time.sleep_ms(500)
                self.robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)
                time.sleep_ms(1400)
            elif self.crossing_record_cnt == 4 or self.crossing_record_cnt == 11:  # 第4,11个路口小车需要右转
                self.robot_move_cmd.car_move(0.3, 0.3, 0.3, 0.3)
                time.sleep_ms(600)
                self.robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)
                time.sleep_ms(1400)
            elif self.crossing_record_cnt == 6:  # 第6个路口小车需要左转,改为颜色识别，旋转机械臂到右边
                self.mid_adjust_position = 1
                self.robot_move_cmd.car_move(0.3, 0.3, 0.3, 0.3)
                time.sleep_ms(600)
                self.robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)
                time.sleep_ms(1400)
            elif self.crossing_record_cnt == 7:  # 第7个路口小车需要旋转180度
                self.over_flag = 1
                self.robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)
                time.sleep_ms(1000)
                return
            elif self.crossing_record_cnt == 8:  # 第8个路口小车需要左转
                self.robot_move_cmd.car_move(0.3, 0.3, 0.3, 0.3)
                time.sleep_ms(600)
                self.robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)
                time.sleep_ms(1400)
            elif self.crossing_record_cnt == 10:  # 第10个路口小车需要右转,改为颜色识别，旋转机械臂到右边
                self.mid_adjust_position = 1
                self.robot_move_cmd.car_move(0.3, 0.3, 0.3, 0.3)
                time.sleep_ms(600)
                self.robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)
                time.sleep_ms(1400)
            elif self.crossing_record_cnt == 12:  # 第12个路口小车回到原点，需要原地旋转180度后倒车
                self.robot_move_cmd.car_move(0, 0, 0, 0)
                self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 500)
                time.sleep_ms(600)
                self.crossing_flag = 1
                self.car_back_flag = 1
                self.over_flag = 1
                return
            elif self.crossing_record_cnt == 13:  # 第13个路口小车回到原点:
                self.robot_move_cmd.car_move(0, 0, 0, 0)
                self.is_line_flag = 0
                return

        # ************************************************ 运动机械臂*************************************************************************************
        if color_read_succeed == 1:  # 识别到颜色块或者在特殊阶段
            if self.move_status == 0:  # 第0阶段：机械臂寻找物块位置
                if (abs(block_cx - self.mid_block_cx) > 2):
                    if block_cx > self.mid_block_cx:
                        self.move_x += 0.3
                    else:
                        self.move_x -= 0.3
                if (abs(block_cy - self.mid_block_cy) > 2):
                    if block_cy > self.mid_block_cy and self.move_y > 1:
                        self.move_y -= 0.2
                    else:
                        self.move_y += 0.2
                if abs(block_cy - self.mid_block_cy) <= 2 and abs(block_cx - self.mid_block_cx) <= 2:  # 寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 50:  # 计数100次对准物块，防止误差
                        self.mid_block_cnt = 0
                        self.move_status = 1
                        self.cap_color_status = color_status
                else:
                    self.mid_block_cnt = 0
                    self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 0)
                time.sleep_ms(10)

            elif self.move_status == 1:  # 第1阶段：机械臂抓取物块
                self.move_status = 2
                # 判断矩形倾角，改变机械爪
                for r in img.find_rects():
                    if r.corners()[0][0] < block_cx and r.corners()[0][1] > block_cy and r.corners()[2][
                        0] > block_cx and r.corners()[2][1] < block_cy:  # 判断矩形是否为物块
                        p0 = r.corners()[0]
                        p1 = r.corners()[1]
                        p2 = r.corners()[2]
                        p3 = r.corners()[3]
                        spin_calw = 1500
                        if abs(p2[1] - p3[1]) > 2:  # 计算物块对齐角度
                            spin_calw += (p2[1] - p3[1]) * 15
                        elif abs(p3[0] - p0[0]) > 2:
                            spin_calw += (p3[0] - p0[0]) * 15
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
                self.kinematic.kinematics_move(self.move_x, self.move_y, -45, 1000)  # 移动机械臂下移到物块
                time.sleep_ms(1200)
                self.kinematic.send_str("{#005P1700T1000!}")  # 机械爪抓取物块
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)  # 移动机械臂抬起
                time.sleep_ms(1200)
                self.kinematic.send_str("{#004P1500T0300!}")  # 机械爪抓取物块
                time.sleep_ms(500)
                self.move_x = 0  # 机械臂归位
                self.move_y = 160
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
                time.sleep_ms(1200)
                self.mid_block_cnt = 0
                self.is_line_flag = 1  # 机器人开始巡线
                self.over_flag = 1  # 原地翻转标志
            elif self.move_status == 2:  # 第2阶段：调整身位
                if block_cx - self.mid_block_cx > 20:
                    if self.crossing_record_cnt == 3:  # 路口处于3的时候颜色区在机械臂左侧，其他时候在右侧
                        if self.adjust_position != 1:
                            self.adjust_position = 1
                            self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)  # 前进
                    else:
                        if self.adjust_position != 4:
                            self.adjust_position = 4
                            self.robot_move_cmd.car_move(-0.1, -0.1, -0.1, -0.1)  # 后退
                    return

                elif block_cx - self.mid_block_cx < -20:
                    if self.crossing_record_cnt == 3:  # 路口处于3的时候颜色区在机械臂左侧，其他时候在右侧
                        if self.adjust_position != 4:
                            self.adjust_position = 4
                            self.robot_move_cmd.car_move(-0.1, -0.1, -0.1, -0.1)  # 后退
                    else:
                        if self.adjust_position != 1:
                            self.adjust_position = 1
                            self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)  # 前进
                    return
                else:  # 调整完毕，停止
                    if self.adjust_position != 5:
                        self.adjust_position = 5
                        self.robot_move_cmd.car_move(0, 0, 0, 0)
                        self.move_status = 3
            elif self.move_status == 3:  # 第3阶段：机械臂寻找放下物块的框框
                if abs(block_cx - self.mid_block_cx) > 5:
                    if block_cx > self.mid_block_cx and self.move_y > 1:
                        if self.crossing_record_cnt == 3:  # 路口处于3的时候颜色区在机械臂左侧，其他时候在右侧
                            self.move_y += 0.5
                        else:
                            self.move_y -= 0.5
                    else:
                        if self.crossing_record_cnt == 3:
                            self.move_y -= 0.5
                        else:
                            self.move_y += 0.5
                if abs(block_cy - self.mid_block_cy) > 5:
                    if block_cy > self.mid_block_cy:
                        if self.crossing_record_cnt == 3:
                            self.move_x += 0.3
                        else:
                            self.move_x -= 0.3
                    else:
                        if self.crossing_record_cnt == 3:
                            self.move_x -= 0.3
                        else:
                            self.move_x += 0.3
                if abs(block_cy - self.mid_block_cy) <= 5 and abs(block_cx - self.mid_block_cx) <= 5:  # 寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 10:  # 计数10次对准物块，防止误差
                        self.mid_block_cnt = 0
                        self.move_status = 4
                else:
                    self.mid_block_cnt = 0
                    self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 0)
                time.sleep_ms(10)

            elif self.move_status == 4:  # 第4阶段：机械臂下放物块
                self.move_status = 0
                l = math.sqrt(self.move_x * self.move_x + self.move_y * self.move_y)
                sin = self.move_y / l
                cos = self.move_x / l
                self.move_x = (l + 86) * cos
                self.move_y = (l + 86) * sin
                time.sleep_ms(100)
                self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)  # 移动机械臂到物块上方
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x, self.move_y, -45, 1000)  # 移动机械臂下移到物块
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
                self.is_line_flag = 1  # 机器人开始巡线
                self.cap_color_status = 0
                self.adjust_position = 0

    def run_map3(self):  # 地图3巡线
        # 获取图像并进行二值化处理
        img = sensor.snapshot().to_grayscale()
        base_speed = 0.3
        # 记录寻找到的黑线块
        blob_roi1 = 0  # 记录最大的块
        blob_roi2 = 0
        blob_roi3 = 0
        blob_roi4 = 0
        blob_roi5 = 0
        acute_crossing_flag = 0  # 路口判断
        weight_sum = 0  # 权值和初始化
        centroid_sum = 0

        # 利用颜色识别分别寻找三个矩形区域内的线段
        for r in self.ROIS:
            blobs = img.find_blobs(self.GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True)
            # 目标区域找到直线
            if blobs:
                # 查找像素最多的blob的索引。
                largest_blob = -1
                second_largest_blob = -1
                most_pixels = 0
                second_most_pixels = 0
                for i in range(len(blobs)):
                    # 目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
                    if blobs[i].pixels() > most_pixels:
                        most_pixels = blobs[i].pixels()
                        largest_blob = i
                for i in range(len(blobs)):
                    # 目标区域找到的颜色块（线段块）可能不止一个，找到第二大的一个
                    if abs(blobs[i].x() - blobs[largest_blob].x()) > 20:  # 先排除一定范围的点
                        if blobs[i].pixels() > second_most_pixels and blobs[i].pixels() < most_pixels:
                            second_most_pixels = blobs[i].pixels()
                            second_largest_blob = i

                if blobs[largest_blob].w() > 10 and blobs[largest_blob].h() > 5:  # 太小说明检测到虚线之类的,略过
                    # 在色块周围画一个矩形。
                    img.draw_rectangle(blobs[largest_blob].rect())
                    img.draw_cross(blobs[largest_blob].cx(),
                                   blobs[largest_blob].cy())  # 将此区域的像素数最大的颜色块画矩形和十字形标记出来
                    img.draw_string(blobs[largest_blob].x(), blobs[largest_blob].y() - 10,
                                    "W:{} H:{}".format(blobs[largest_blob].w(), blobs[largest_blob].h()),
                                    color=(255, 255, 255))

                    if r[5] == 1:
                        blob_roi1 = blobs[largest_blob].rect()
                    elif r[5] == 2:
                        blob_roi2 = blobs[largest_blob].rect()
                    elif r[5] == 3:
                        blob_roi3 = blobs[largest_blob].rect()
                    elif r[5] == 4:
                        blob_roi4 = blobs[largest_blob].rect()
                    elif r[5] == 5:
                        blob_roi5 = blobs[largest_blob].rect()

                if second_largest_blob != -1 and blobs[second_largest_blob].w() > 10 and blobs[
                    second_largest_blob].h() > 5:
                    # 在色块周围画一个矩形。
                    img.draw_rectangle(blobs[second_largest_blob].rect())
                    img.draw_cross(blobs[second_largest_blob].cx(),
                                   blobs[second_largest_blob].cy())  # 将此区域的像素数第二大的颜色块画矩形和十字形标记出来
                    img.draw_string(blobs[second_largest_blob].x(), blobs[second_largest_blob].y() - 10,
                                    "W:{} H:{}".format(blobs[second_largest_blob].w(), blobs[second_largest_blob].h()),
                                    color=(255, 0, 0))
                    acute_crossing_flag += 1
                # print("acute_crossing_flag:", acute_crossing_flag)

                ##print(blobs[largest_blob].rect())
                if blobs[largest_blob].w() > 50:  # 遇到地图上的路口
                    centroid_sum += blobs[largest_blob].cx() * (r[4] * 2)  # r[4] is the roi weight.
                    weight_sum += (r[4] * 2)
                else:
                    centroid_sum += blobs[largest_blob].cx() * r[4]  # r[4] is the roi weight.
                    weight_sum += r[4]

        if acute_crossing_flag >= 2 and self.crossing_cnt >= 2:  # 发现锐角，右转3S后重新检测
            self.robot_move_cmd.car_move(0.2, 0.2, 0.2, 0.2)  # 前进
            time.sleep_ms(1300)
            self.robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)  # 右转
            time.sleep_ms(1450)
        # print((blob_roi1[0]+blob_roi1[2])/2,(blob_roi2[0]+blob_roi2[2])/2,(blob_roi3[0]+blob_roi3[2])/2,weight_sum)

        if weight_sum > 0 and self.crossing_cnt <= 4:
            center_pos = (centroid_sum / weight_sum)  # Determine center of line.

            # 将center_pos转换为一个偏角。我们用的是非线性运算，所以越偏离直线，响应越强。
            # 非线性操作很适合用于这样的算法的输出，以引起响应“触发器”。
            deflection_angle = 0
            # 机器人应该转的角度

            # 80是X的一半，60是Y的一半。
            # 下面的等式只是计算三角形的角度，其中三角形的另一边是中心位置与中心的偏差，相邻边是Y的一半。
            # 这样会将角度输出限制在-45至45度左右。（不完全是-45至45度）。

            deflection_angle = -math.atan((center_pos - 80) / 60)
            # 角度计算.80 60 分别为图像宽和高的一半，图像大小为QQVGA 160x120.
            # 注意计算得到的是弧度值

            deflection_angle = math.degrees(deflection_angle)
            # 将计算结果的弧度值转化为角度值

            # 现在你有一个角度来告诉你该如何转动机器人。
            # 通过该角度可以合并最靠近机器人的部分直线和远离机器人的部分直线，以实现更好的预测。

            #print("Turn Angle:", deflection_angle)

            if acute_crossing_flag == 1:  # 遇到路口，减慢速度
                if deflection_angle < 0:  # 右转
                    # 右侧电机速度增加，左侧电机速度减少
                    self.speed_motor1 = base_speed * (1 + abs(deflection_angle) / 45) / 2
                    self.speed_motor2 = base_speed * (1 - abs(deflection_angle) / 45) / 2
                    self.speed_motor3 = base_speed * (1 + abs(deflection_angle) / 45) / 2
                    self.speed_motor4 = base_speed * (1 - abs(deflection_angle) / 45) / 2
                else:  # 左转
                    # 左侧电机速度增加，右侧电机速度减少
                    self.speed_motor1 = base_speed * (1 - abs(deflection_angle) / 45) / 2
                    self.speed_motor2 = base_speed * (1 + abs(deflection_angle) / 45) / 2
                    self.speed_motor3 = base_speed * (1 - abs(deflection_angle) / 45) / 2
                    self.speed_motor4 = base_speed * (1 + abs(deflection_angle) / 45) / 2
            else:
                if deflection_angle < 0:  # 右转
                    # 右侧电机速度增加，左侧电机速度减少
                    self.speed_motor1 = base_speed * (1 + abs(deflection_angle) / 45)
                    self.speed_motor2 = base_speed * (1 - abs(deflection_angle) / 45)
                    self.speed_motor3 = base_speed * (1 + abs(deflection_angle) / 45)
                    self.speed_motor4 = base_speed * (1 - abs(deflection_angle) / 45)
                else:  # 左转
                    # 左侧电机速度增加，右侧电机速度减少
                    self.speed_motor1 = base_speed * (1 - abs(deflection_angle) / 45)
                    self.speed_motor2 = base_speed * (1 + abs(deflection_angle) / 45)
                    self.speed_motor3 = base_speed * (1 - abs(deflection_angle) / 45)
                    self.speed_motor4 = base_speed * (1 + abs(deflection_angle) / 45)

            self.robot_move_cmd.car_move(self.speed_motor1, self.speed_motor2, self.speed_motor3, self.speed_motor4)

        elif self.crossing_cnt == 5:
            self.robot_move_cmd.car_move(0, 0, 0, 0)  # 发送控制小车行动的指令
        else:
            self.robot_move_cmd.car_move(self.speed_motor1, self.speed_motor2, self.speed_motor3, self.speed_motor4)  # 发送控制小车行动的指令
            #time.sleep_ms(200)

        if blob_roi1 != 0 and blob_roi2 != 0 and blob_roi3 != 0 and acute_crossing_flag == 0 and self.crossing_cnt <= 4:
            # print("blob_roi2[0] - blob_roi3[0]:",blob_roi2[0] - blob_roi3[0])
            #print("blob_roi3[2] - blob_roi2[2]:", blob_roi3[2] - blob_roi2[2])
            if blob_roi3[0] - blob_roi2[0] > 50 and blob_roi2[2] - blob_roi3[2] > 50 and abs(
                    blob_roi3[0] - blob_roi1[0]) < 30:  # 遇到倒T字转角路口，左转2S后重新检测
                self.robot_move_cmd.car_move(0.2, 0.2, 0.2, 0.2)  # 前进
                time.sleep_ms(1000)
                self.robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)  # 左转
                time.sleep_ms(1200)
                return
            elif ((blob_roi2[0] + blob_roi2[2]) - (blob_roi3[0] + blob_roi3[2]) > 50 and
                  blob_roi2[2] - blob_roi3[2] > 50 and abs(blob_roi3[0] - blob_roi1[0]) < 30):  # 遇到倒T字转角路口，左转2S后重新检测
                self.robot_move_cmd.car_move(0.2, 0.2, 0.2, 0.2)  # 前进
                time.sleep_ms(1000)
                self.robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)  # 右转
                time.sleep_ms(1200)
                return
            elif blob_roi2[0] - blob_roi3[0] > 50 and blob_roi3[2] - blob_roi2[
                2] > 100:  # 遇到T字转角路口，要计算经过第几次来决定小车旋转方向
                print("self.crossing_cnt:", self.crossing_cnt)
                if self.crossing_cnt == 0:  # 遇到第1个T字路口，左转2S后重新检测
                    self.robot_move_cmd.car_move(0.2, 0.2, 0.2, 0.2)  # 前进
                    time.sleep_ms(1000)
                    self.robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)  # 左转
                    time.sleep_ms(1200)
                elif self.crossing_cnt == 1:  # 遇到第2个T字路口，左转2S后重新检测
                    self.robot_move_cmd.car_move(0.2, 0.2, 0.2, 0.2)  # 前进
                    time.sleep_ms(1200)
                    self.robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)  # 右转
                    time.sleep_ms(1200)
                elif self.crossing_cnt == 2:  # 遇到第3个T字路口，左转2S后重新检测
                    self.robot_move_cmd.car_move(0.2, 0.2, 0.2, 0.2)  # 前进
                    time.sleep_ms(1400)
                    self.robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)  # 右转
                    time.sleep_ms(1400)
                    self.crossing_cnt += 1
                    return
                self.crossing_cnt += 1

            while self.crossing_cnt == 3 and (
                (blob_roi3 != 0 and blob_roi3[2] > 110) or (blob_roi2 != 0 and blob_roi2[2] > 110)):
                self.robot_move_cmd.car_move(0, 0, 0, 0)
                time.sleep_ms(1200)



