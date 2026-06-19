import cv2
import numpy as np

"""这是PJ前期测试代码,用于测试颜色识别和颜色定位,用手调滑块可以标定要识别颜色区间"""

# 创建窗口滑块面板
cv2.namedWindow("HSV_Tune")
cv2.namedWindow("Origin")
cv2.namedWindow("Mask")

def nothing(x):
    pass

# 六个HSV滑块
cv2.createTrackbar("H_min", "HSV_Tune", 0, 179, nothing)
cv2.createTrackbar("H_max", "HSV_Tune", 179, 179, nothing)
cv2.createTrackbar("S_min", "HSV_Tune", 30, 255, nothing)
cv2.createTrackbar("S_max", "HSV_Tune", 255, 255, nothing)
cv2.createTrackbar("V_min", "HSV_Tune", 30, 255, nothing)
cv2.createTrackbar("V_max", "HSV_Tune", 255, 255, nothing)

# 形态学核
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))

# 预定义颜色HSV范围
color_range = {
    "red1":   np.array([0, 18, 121]),
    "red2":   np.array([19, 255, 255]),
    "red3":   np.array([158, 18, 121]),
    "red4":   np.array([179, 255, 255]),
    "green1": np.array([44, 42, 55]),
    "green2": np.array([86, 255, 255]),
    "blue1":  np.array([100, 104, 72]),
    "blue2":  np.array([116, 255, 255]),
    "yellow1": np.array([21, 44, 100]),
    "yellow2": np.array([45, 170, 170])
}

# 颜色对应的BGR显示色
color_bgr = {
    "Red":    (0, 0, 255),
    "Green":  (0, 255, 0),
    "Blue":   (255, 0, 0),
    "Yellow": (0, 255, 255),
    "None":   (128, 128, 128)
}

# 打开摄像头
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 7)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        continue

    # 原图高斯模糊降噪
    blur = cv2.GaussianBlur(frame, (5,5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # 读取滑块阈值
    h_min = cv2.getTrackbarPos("H_min", "HSV_Tune")
    h_max = cv2.getTrackbarPos("H_max", "HSV_Tune")
    s_min = cv2.getTrackbarPos("S_min", "HSV_Tune")
    s_max = cv2.getTrackbarPos("S_max", "HSV_Tune")
    v_min = cv2.getTrackbarPos("V_min", "HSV_Tune")
    v_max = cv2.getTrackbarPos("V_max", "HSV_Tune")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    # 生成当前滑块掩膜
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, kernel)
    mask = cv2.dilate(mask, kernel)

    # ========== 统计画面上方 3/4 的颜色像素 ==========
    h = frame.shape[0]
    
    mask_red1 = cv2.inRange(hsv, color_range["red1"], color_range["red2"])
    mask_red2 = cv2.inRange(hsv, color_range["red3"], color_range["red4"])
    mask_red = mask_red1 + mask_red2
    mask_red[int(h * 3/4):, :] = 0

    mask_green = cv2.inRange(hsv, color_range["green1"], color_range["green2"])
    mask_green[int(h * 3/4):, :] = 0

    mask_blue = cv2.inRange(hsv, color_range["blue1"], color_range["blue2"])
    mask_blue[int(h * 3/4):, :] = 0

    mask_yellow = cv2.inRange(hsv, color_range["yellow1"], color_range["yellow2"])
    mask_yellow[int(h * 3/4):, :] = 0

    cnt_red   = cv2.countNonZero(mask_red)
    cnt_green = cv2.countNonZero(mask_green)
    cnt_blue  = cv2.countNonZero(mask_blue)
    cnt_yellow = cv2.countNonZero(mask_yellow)

    # 找主导颜色
    max_cnt = max(cnt_red, cnt_green, cnt_blue, cnt_yellow)
    now_color = "None"
    target_mask = None

    if max_cnt < 500:
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

    # ========== 新增：画矩形和中心点 ==========
    if target_mask is not None:
       
        contours = cv2.findContours(target_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        if len(contours) > 0:
            # 取最大轮廓
            max_contour = max(contours, key=cv2.contourArea)
            
            # 最小外接矩形
            x, y, w, h_rect = cv2.boundingRect(max_contour)
            
            # 几何中心
            cx = x + w // 2
            cy = y + h_rect // 2
            
            # 画矩形（用对应颜色）
            bgr = color_bgr[now_color]
            cv2.rectangle(frame, (x, y), (x + w, y + h_rect), bgr, 2)
            
            # 画中心点
            cv2.circle(frame, (cx, cy), 4, bgr, -1)
            
            # 显示中心坐标
            cv2.putText(frame, f"({cx},{cy})", (cx + 5, cy - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 1)

    # 显示当前颜色
    cv2.putText(frame, now_color, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # 窗口显示
    cv2.imshow("Origin", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()