"""2026 06"""
import smbus
import time
import numpy as np
import RPi.GPIO as GPIO
import cv2

# 全局初始化 I2C 和 地址
bus = smbus.SMBus(1)
ADDR_KS103 = 0x74
CMD_DIST = 0xb0
current_color = "None"

# 识别颜色的HSV空间区间
color_range = {
    "red1":   np.array([0, 18, 121]),
    "red2":   np.array([19, 255, 255]),
    "red3":   np.array([158, 18, 121]),
    "red4":   np.array([179, 255, 255]),
    "green1": np.array([44, 42, 55]),
    "green2": np.array([86, 255, 255]),
    "blue1":  np.array([100, 104, 72]),
    "blue2":  np.array([116, 255, 255]),
    "yellow1": np.array([21, 61, 100]),
    "yellow2": np.array([45, 175, 255])
}

# 颜色对应的BGR显示色
color_bgr = {
    "Red":    (0, 0, 255),
    "Green":  (0, 255, 0),
    "Blue":   (255, 0, 0),
    "Yellow": (0, 255, 255),
    "None":   (128, 128, 128)
}


def getDistance():
    """
    超声波KS103测距,单位:厘米
    """
    try:
        bus.write_byte_data(ADDR_KS103, 0x02, CMD_DIST)
        time.sleep(0.04)
        high = bus.read_byte_data(ADDR_KS103, 0x02)
        low  = bus.read_byte_data(ADDR_KS103, 0x03)
        dist_mm = (high << 8) + low
        dist_cm = dist_mm / 10.0
        return dist_cm
    except Exception as e:
        return -1.0


def init_camera():
    """
    初始化摄像头
    """
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 7)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(5):
        cap.read()
        time.sleep(0.05)
    return cap


# ==================== 新增：图像处理总函数 ====================
def process_frame(frame,RED_f, BLUE_f, YELLOW_f):
    """
    图像处理基础层：对输入帧进行颜色识别，返回完整信息
    
    返回: (color, cx, cy, rect, target_mask)
        color: 识别到的颜色名 ("Red"/"Green"/"Blue"/"Yellow"/"None")
        cx, cy: 几何中心坐标 (int, int)，未识别到时为 (None, None)
        rect: 外接矩形 (x, y, w, h)，未识别到时为 None
        target_mask: 对应颜色的掩膜，未识别到时为 None
    """
    if frame is None:
        return "None", None, None, None, None
    
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    
    h = frame.shape[0]
    
    # 各颜色掩膜（屏蔽下方 1/3） 这里屏蔽下方是一开始调试的时候为了防止地面反光 试验场地地上是灰布，不会反光
    mask_red1 = cv2.inRange(hsv, color_range["red1"], color_range["red2"])
    mask_red2 = cv2.inRange(hsv, color_range["red3"], color_range["red4"])
    mask_red = mask_red1 + mask_red2
    mask_red[int(h * 2/3):, :] = 0

    mask_green = cv2.inRange(hsv, color_range["green1"], color_range["green2"])
    mask_green[int(h * 2/3):, :] = 0

    mask_blue = cv2.inRange(hsv, color_range["blue1"], color_range["blue2"])
    mask_blue[int(h * 2/3):, :] = 0

    mask_yellow = cv2.inRange(hsv, color_range["yellow1"], color_range["yellow2"])
    mask_yellow[int(h * 2/3):, :] = 0

    # 统计像素
    cnt_red = cv2.countNonZero(mask_red)
    #cnt_green = cv2.countNonZero(mask_green) 绿色设为0不识别,按照班级的具体要求,有些组有绿色块
    cnt_green = 0 
    cnt_blue = cv2.countNonZero(mask_blue)
    cnt_yellow = cv2.countNonZero(mask_yellow)

    #这几个_f是屏蔽不需要识别的颜色的变量

    if RED_f == True:
        cnt_red = 0
    if BLUE_f == True:
        cnt_blue = 0
    if YELLOW_f == True:
        cnt_yellow = 0
    

    # 找主导颜色
    max_cnt = max(cnt_red, cnt_green, cnt_blue, cnt_yellow)
    now_color = "None"
    target_mask = None

    if max_cnt < 200:
        now_color = "None"
    elif max_cnt == cnt_red:
        now_color = "Red"
        target_mask = mask_red
    elif max_cnt == cnt_green:
        now_color = "Green"
        target_mask = mask_green
    elif max_cnt == cnt_blue:
        now_color = "Blue"
        target_mask = mask_blue
    elif max_cnt == cnt_yellow:
        now_color = "Yellow"
        target_mask = mask_yellow
    #=============================================================
    # 计算几何中心和矩形
    cx, cy = None, None
    rect = None
    area = 0
    
    if target_mask is not None:
        contours = cv2.findContours(target_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        if len(contours) > 0:
            # 最大轮廓
            max_contour = max(contours, key=cv2.contourArea)
            area = int(cv2.contourArea(max_contour))
            
            if area >= 500:
                # 外接矩形
                x, y, w, h_rect = cv2.boundingRect(max_contour)
                rect = (x, y, w, h_rect)
                
                # 质心（Moments）
                M = cv2.moments(max_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = 0, 0

    return now_color, cx, cy, rect, target_mask, area


# ==================== get_color：简化用 ====================
def get_color(cap):
    """
    从摄像头读取一帧，返回识别到的颜色名
    保持原有接口不变
    由于主程序用线程更新颜色，这个没啥用
    """
    ret, frame = cap.read()
    if not ret or frame is None:
        return "None"
    
    color, meiyou1, meiyou2, meiyou3, meiyou4, meiyou5 = process_frame(frame)
    return color


# ==================== show_camera：显示画面 + 标注 ====================
def show_camera(cap, RED_f, BLUE_f, YELLOW_f, window_name="Camera"):
    """
    打包调用process_frame()，显示画面并标注颜色
    """
    for i in range(5): # 至多尝试五次
        ret, frame = cap.read()
        if ret and frame is not None:
            break
        time.sleep(0.05)
    
    if not ret or frame is None:
        return "None", 160, 120
    
    color, cx, cy, rect, target_mask, area = process_frame(frame,RED_f, BLUE_f, YELLOW_f)
    
    # 画矩形和中心点
    if rect is not None:
        x, y, w, h_rect = rect
        bgr = color_bgr[color]
        cv2.rectangle(frame, (x, y), (x + w, y + h_rect), bgr, 2)
        cv2.circle(frame, (cx, cy), 4, bgr, -1)
        cv2.putText(frame, f"({cx},{cy})", (cx + 5, cy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 1)
    else:
        # 未识别到时，cx, cy 设为画面中心
        cx, cy = 160, 120
        # 在画面中心画一个灰色十字，表示默认参考点
        cv2.drawMarker(frame, (160, 120), (128, 128, 128), 
                       cv2.MARKER_CROSS, 20, 2)
    
    # 显示颜色名
    cv2.putText(frame, color, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    cv2.imshow(window_name, frame)
    cv2.waitKey(1)
    
    return color, cx, cy, area

