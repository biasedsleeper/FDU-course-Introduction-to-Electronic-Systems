"""2026 06"""
import RPi.GPIO as GPIO
import time
import threading
import functions
import cv2

# ==================== 全局引脚 & 全局变量 ====================
EA, I2, I1, EB, I4, I3, RS, LS = (13, 19, 26, 16, 20, 21, 6, 5)
FREQUENCY = 50
overprint = 0

# 测速全局变量
lspeed = 0
rspeed = 0
lcounter = 0
rcounter = 0
stop_getspeed = False
# 识别全局变量
found = False
RED_finished = False
BLUE_finished = False
YELLOW_finished = False
AREA_amount = 0


# 测距和颜色全局变量
distance = 999.0      # 当前距离，初始设为很远
current_color = "None" # 当前识别颜色
cx, cy = 160, 120
stop_sensor = False    # 传感器线程停止标志

# ==================== PID 类 ====================
class PID:
    def __init__(self, P=80, I=0, D=0, speed=0.4, duty=26):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.err_pre = 0
        self.err_last = 0
        self.u = 0
        self.integral = 0
        self.ideal_speed = speed

    def update(self, feedback_value):
        self.err_pre = self.ideal_speed - feedback_value
        self.integral += self.err_pre
        self.u = self.Kp * self.err_pre + self.Ki * self.integral + self.Kd * (self.err_pre - self.err_last)
        self.err_last = self.err_pre
        if self.u > 100:
            self.u = 100
        elif self.u < 0:
            self.u = 0
        return self.u


# ==================== 编码器回调 ====================
def my_callback(channel):
    global lcounter, rcounter
    if channel == LS:
        lcounter += 1
    elif channel == RS:
        rcounter += 1


# ==================== 测速线程 ====================
def getspeed():
    global rspeed, lspeed, lcounter, rcounter
    try:
        GPIO.remove_event_detect(LS)
        GPIO.remove_event_detect(RS)
    except:
        pass
    GPIO.add_event_detect(LS, GPIO.RISING, callback=my_callback)
    GPIO.add_event_detect(RS, GPIO.RISING, callback=my_callback)
    while not stop_getspeed:
        rspeed = rcounter / 585.0
        lspeed = lcounter / 585.0
        rcounter = 0
        lcounter = 0
        time.sleep(0.1)


# ==================== 测距线程 ====================
def distance_thread():
    """
    持续读取超声波距离，更新全局变量 distance
    """
    global distance, stop_sensor
    while not stop_sensor:
        dist = functions.getDistance()
        if dist > 0:           # 过滤异常值（-1）
            distance = dist
            print(f'距离: {distance}')
        time.sleep(0.05)       # 20Hz 更新


# ==================== 颜色识别线程 ====================
def color_thread(cap,RED_f, BLUE_f, YELLOW_f):
    """
    持续识别颜色，更新全局变量 current_color
    同时调用 show_camera 显示画面
    """
    global current_color, stop_sensor, cx,cy, AREA_amount, found
    while not stop_sensor:
        # show_camera 内部会更新画面，并返回颜色
        color, x, y, area = functions.show_camera(cap, RED_finished, BLUE_finished, YELLOW_finished, window_name="Camera")
        cx, cy = x, y
        current_color = color
        AREA_amount = area
        
        if distance < 550 and AREA_amount > 250 and current_color != "None":
            found = True
        else:
            found = False
        print(f'面积: {AREA_amount}')
        print(f'找到: {found}')
        time.sleep(0.05)       # 20Hz 更新


# ==================== 运动函数（不变）====================

def go_straight(pwma, pwmb, t=3):
    GPIO.output(I1, GPIO.HIGH)
    GPIO.output(I2, GPIO.LOW)
    GPIO.output(I3, GPIO.LOW)
    GPIO.output(I4, GPIO.HIGH)
    i = 0
    speed = 1.5
    l_origin_duty = 5
    r_origin_duty = 7
    pwma.start(r_origin_duty)
    pwmb.start(l_origin_duty)
    R_control = PID(24, 0.027, 19, speed)
    L_control = PID(19, 0.02, 25, speed-0.04)
    while i <= t:
        pwma.ChangeDutyCycle(R_control.update(rspeed))
        pwmb.ChangeDutyCycle(L_control.update(lspeed))
        time.sleep(0.05)
        i += 0.05
    pwma.ChangeDutyCycle(0)
    pwmb.ChangeDutyCycle(0)

def turnleft(pwma, pwmb, duty=30, t=1):
    GPIO.output(I1, GPIO.HIGH)
    GPIO.output(I2, GPIO.LOW)
    GPIO.output(I4, GPIO.LOW)
    GPIO.output(I3, GPIO.HIGH)
    pwma.ChangeDutyCycle(duty+5)
    pwmb.ChangeDutyCycle(duty)
    time.sleep(t)
    pwma.ChangeDutyCycle(0)
    pwmb.ChangeDutyCycle(0)

def turnright(pwma, pwmb, duty=30, t=1):
    GPIO.output(I1, GPIO.LOW)
    GPIO.output(I2, GPIO.HIGH)
    GPIO.output(I4, GPIO.HIGH)
    GPIO.output(I3, GPIO.LOW)
    pwma.ChangeDutyCycle(duty+5)
    pwmb.ChangeDutyCycle(duty)
    time.sleep(t)
    pwma.ChangeDutyCycle(0)
    pwmb.ChangeDutyCycle(0)


def arc_turn_pid(pwma, pwmb, angle_deg, radius_cm, clockwise, total_time):
    """
        PID控制弧线转弯
    """
    
    L_half = 7.0        # 轮距一半
    d = 6.4             # 轮直径
    constant = 180.0 * d  # = 1152
    
    # 计算内外轮理想转速（转/秒）
    v_outer_ideal = 0.7 * angle_deg * (radius_cm + L_half) / (constant * total_time)
    v_inner_ideal = 0.7 * angle_deg * (radius_cm - L_half) / (constant * total_time)
    
    print(f"弧线转弯: 角度={angle_deg}°, 半径={radius_cm}cm, 时间={total_time}s")
    print(f"理想外轮速={v_outer_ideal:.3f}r/s, 理想内轮速={v_inner_ideal:.3f}r/s")
    
    # 设置方向（两轮同向）
    GPIO.output(I1, GPIO.HIGH)   # 左轮正转
    GPIO.output(I2, GPIO.LOW)
    GPIO.output(I3, GPIO.LOW)    # 右轮正转
    GPIO.output(I4, GPIO.HIGH)
    
    #顺时针为1，逆时针为0
    if clockwise == 0:
        # 右轮外圈，左轮内圈
        outer_control = PID(22.5, 0.027, 25, speed=v_outer_ideal+0.5)
        inner_control = PID(19, 0.02, 25, speed=v_inner_ideal)
        outer_pwm = pwma  # 右轮
        inner_pwm = pwmb  # 左轮
    elif clockwise == 1:
        # 左轮外圈，右轮内圈
        outer_control = PID(19.2, 0.02, 25, speed=v_outer_ideal)  # 左轮PID参数
        inner_control = PID(19, 0.027, 25, speed=v_inner_ideal)  # 右轮PID参数
        outer_pwm = pwmb  # 左轮
        inner_pwm = pwma  # 右轮
    
    outer_pwm.start(10)
    inner_pwm.start(10)
    
    # 控制循环
    t = 0
    while t < total_time * 1.07:
        # PID更新，读取全局轮速
        if clockwise == 0:
            outer_speed = rspeed   # 右轮
            inner_speed = lspeed   # 左轮
        elif clockwise == 1:
            outer_speed = lspeed   # 左轮
            inner_speed = rspeed   # 右轮
        
        outer_duty = outer_control.update(outer_speed)
        inner_duty = inner_control.update(inner_speed)
        
        outer_pwm.ChangeDutyCycle(outer_duty)
        inner_pwm.ChangeDutyCycle(inner_duty)
        
        print(f"t={t:.2f} 外轮:{outer_speed:.3f}/{v_outer_ideal:.3f} 内轮:{inner_speed:.3f}/{v_inner_ideal:.3f}")
        
        time.sleep(0.05)
        t += 0.05
    
    # 停车
    outer_pwm.ChangeDutyCycle(0)
    inner_pwm.ChangeDutyCycle(0)
    print("弧线转弯完成")

def go_straight_until_dist(pwma, pwmb, target_dist=15):
    """
    直行直到距离小于 target_dist
    """
    GPIO.output(I1, GPIO.HIGH)
    GPIO.output(I2, GPIO.LOW)
    GPIO.output(I3, GPIO.LOW)
    GPIO.output(I4, GPIO.HIGH)
    speed = 1.7
    l_origin_duty = 7
    r_origin_duty = 7
    pwma.start(r_origin_duty)
    pwmb.start(l_origin_duty)
    R_control = PID(23.5, 0.027, 19, speed)
    L_control = PID(18.7, 0.02, 25, speed-0.07)

    while True:
        pwma.ChangeDutyCycle(R_control.update(rspeed))
        pwmb.ChangeDutyCycle(L_control.update(lspeed))
        

        print("distance: %.1f" % distance, end="\r")
        
        if (distance > 0 and distance < target_dist):
            break
        if (distance < 20):
            break
        time.sleep(0.1)
    
    pwma.ChangeDutyCycle(0)
    pwmb.ChangeDutyCycle(0)


def face_target(pwma, pwmb, target_cx=160, dead_zone=10, timeout=3):
    """
    面朝目标
    """
    global cx  # 颜色中心x坐标，由 color_thread 实时更新
    
    # PID参数（需要调参）
    Kp = 0.155
    Ki = 0.007     
    Kd = 1.0      
    
    error = 0
    integral = 0
    last_error = 0
    
    pwma.start(0)
    pwmb.start(0)
    t = 0
    while t < timeout:
        # 计算偏差：目标在左边(cx < 160)则 error 为负，需要左转
        if cx < (target_cx - dead_zone):
            error = cx - (target_cx - dead_zone)
        elif cx > (target_cx + dead_zone):
            error = cx - (target_cx + dead_zone)
        #elif cx >= (target_cx - 5) and cx <= (target_cx + 5):
            #break
        else:
            error = 0
        
        
        
        # PID计算
        integral += error
        derivative = error - last_error
        if error < 20 and error > 0:
            output = Kp * error + Ki * integral + Kd * derivative
        if error >= 20:
            output = 0.142 * (error-30) + Kp * 25 + Ki * integral + Kd * derivative
            
        if error > -20 and error < 0:
            output = Kp * error + Ki * integral + Kd * derivative
        if error <= 20:
            output = 0.142 * (error+30) + Kp * 25 + Ki * integral + Kd * derivative
        last_error = error
        
        # 输出限幅
        if output > 45:
            output = 45
        elif output < -45:
            output = -45
        
        # 根据 output 符号判断转向方向
        # output > 0: 目标在右边，需要右转（原地右转：左轮前进，右轮后退）
        # output < 0: 目标在左边，需要左转（原地左转：左轮后退，右轮前进）
        duty = abs(output) 
        
        
        if output > 0:
            # 右转
            GPIO.output(I1, GPIO.LOW)
            GPIO.output(I2, GPIO.HIGH)
            GPIO.output(I4, GPIO.HIGH)
            GPIO.output(I3, GPIO.LOW)
        else:
            # 左转
            GPIO.output(I1, GPIO.HIGH)
            GPIO.output(I2, GPIO.LOW)
            GPIO.output(I4, GPIO.LOW)
            GPIO.output(I3, GPIO.HIGH)
        
        pwma.ChangeDutyCycle(duty+5)
        pwmb.ChangeDutyCycle(duty)
        
        
        time.sleep(0.05)
        t += 0.05
    
    # 停车
    pwma.ChangeDutyCycle(0)
    pwmb.ChangeDutyCycle(0)

def search(pwma, pwmb, total_time = 4):
    """
    搜索(其实是就是旋转扫描)
    """
    timeuse =0
    while found == False and timeuse < total_time:
        turnleft(pwma, pwmb, duty=20.5, t=0.05)
        timeuse += 0.05


# ==================== 主程序 ====================
def main():
    global stop_getspeed, stop_sensor
    global RED_finished, BLUE_finished, YELLOW_finished
    # GPIO初始化
    GPIO.setwarnings(False)
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([EA, I2, I1, EB, I4, I3], GPIO.OUT)
    GPIO.setup([LS, RS], GPIO.IN)
    GPIO.output([EA, I2, EB, I3], GPIO.LOW)
    GPIO.output([I1, I4], GPIO.HIGH)

    pwma = GPIO.PWM(EA, FREQUENCY)
    pwmb = GPIO.PWM(EB, FREQUENCY)
    pwma.start(0)
    pwmb.start(0)

    # 1. 启动测速线程
    thread1 = threading.Thread(target=getspeed, daemon=True)
    thread1.start()

    # 2. 初始化摄像头
    print("正在初始化摄像头...")
    cap = functions.init_camera()
    print("摄像头就绪")

    # 3. 启动测距线程
    thread_dist = threading.Thread(target=distance_thread, daemon=True)
    thread_dist.start()
    print("测距线程已启动")

    # 4. 启动颜色识别+显示线程
    thread_color = threading.Thread(target=color_thread, args=(cap,RED_finished, BLUE_finished, YELLOW_finished), daemon=True)
    thread_color.start()
    print("颜色识别线程已启动")

    # 预热，等传感器稳定
    time.sleep(2)
    count=0
    stage=0
    # ================== 动作程序 ==================
    try:
            while count == 0:
                count=count+1
                if stage==0:
                    RED_finished = True
                    BLUE_finished = False
                    YELLOW_finished = True
                    face_target(pwma, pwmb, target_cx=160, dead_zone=10, timeout=4) 
                    go_straight(pwma,pwmb,1)
                    time.sleep(0.2)
                    face_target(pwma, pwmb, target_cx=160, dead_zone=10, timeout=2)
                    go_straight(pwma,pwmb,1)
                    face_target(pwma, pwmb, target_cx=160, dead_zone=10, timeout=1.5)
                    go_straight_until_dist(pwma,pwmb,50)
                    time.sleep(0.2)
                    face_target(pwma, pwmb, target_cx=160, dead_zone=10, timeout=3)
                    
                    stage = 1
                    print("第0阶段完成")
                    
                    time.sleep(0.2)
                         
                if stage==1:
                    arc_turn_pid(pwma, pwmb, 80, 25, 1, 0.7)
                    arc_turn_pid(pwma, pwmb, 80, 25, 0, 0.7)
                    
                    stage = 2
                    print("第1阶段完成")
                    time.sleep(0.2)
                
                if stage==2:
                    RED_finished = True
                    BLUE_finished = True
                    YELLOW_finished = False
                    search(pwma, pwmb, 15)
                    face_target(pwma, pwmb, target_cx=160, dead_zone=10, timeout=3)
                    if current_color == "None":
                        search(pwma, pwmb, 15)
                    time.sleep(0.2)
                    
                    go_straight(pwma,pwmb,1.7)
                    time.sleep(0.1)
                    face_target(pwma, pwmb, target_cx=160, dead_zone=10, timeout=2)
                    go_straight(pwma,pwmb,1.7)
                    time.sleep(0.1)
                    face_target(pwma, pwmb, target_cx=160, dead_zone=10, timeout=3)
                    
                    go_straight_until_dist(pwma,pwmb,45)
                    face_target(pwma, pwmb, target_cx=160, dead_zone=10, timeout=2)

                    stage = 3
                    print("第2阶段完成")
                    time.sleep(0.2)
                
                if stage==3:
                   
                    turnleft(pwma, pwmb, duty=30, t=0.64)
                    time.sleep(0.2)
                    arc_turn_pid(pwma, pwmb, 195, 50, 1, 3.5)
                    time.sleep(0.7)
                    arc_turn_pid(pwma, pwmb, 195, 50, 1, 3)
                    time.sleep(0.7)
                    arc_turn_pid(pwma, pwmb, 190, 50, 1, 3)
                    go_straight(pwma,pwmb,0.3)
                    
                    
                    RED_finished = False
                    BLUE_finished = True
                    YELLOW_finished = True

                    stage = 4
                    print("第3阶段完成")
                
                if stage==4:
                    search(pwma, pwmb, 10)
                    face_target(pwma, pwmb, target_cx=160, dead_zone=10, timeout=3)
                    search(pwma, pwmb, 10)
                    
                    go_straight(pwma,pwmb,1.7)
                    time.sleep(0.1)
                    face_target(pwma, pwmb, target_cx=160, dead_zone=10, timeout=2)
                    go_straight(pwma,pwmb,1.7)
                    time.sleep(0.1)
                    face_target(pwma, pwmb, target_cx=160, dead_zone=10, timeout=2)
                        
                    
                    go_straight_until_dist(pwma,pwmb,48)
                    face_target(pwma, pwmb, target_cx=160, dead_zone=10, timeout=2)
                        
                    arc_turn_pid(pwma, pwmb, 80, 25, 0, 0.7)
                    time.sleep(0.1)
                    arc_turn_pid(pwma, pwmb, 120, 25, 1, 0.7)
                    go_straight(pwma,pwmb,5)
                    print("第4阶段完成")
                


    except KeyboardInterrupt:
        print("\n手动中断")
    finally:
        # 停止所有线程，要清理，不然下次有好果汁吃
        stop_getspeed = True
        stop_sensor = True
        time.sleep(0.3)
        
        pwma.stop()
        pwmb.stop()
        cap.release()
        cv2.destroyAllWindows()
        
        try:
            GPIO.remove_event_detect(LS)
            GPIO.remove_event_detect(RS)
        except:
            pass
        GPIO.cleanup()
        print("GPIO已清理退出")

if __name__ == "__main__":
    main()