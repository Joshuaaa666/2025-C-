#主改1 改焦距
"""改进的旋转正方形检测方法，使用角点计算边长"""
import sensor
import image,math
import time
import pyb
# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # 设置像素格式为RGB565
sensor.set_framesize(sensor.VGA)    # 设置帧大小为QVGA
sensor.set_windowing((200, 240))
sensor.skip_frames(time=2000)        # 等待摄像头稳定
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)



roi = (0, 0, 200, 240)  # 示例：左上角(80,60)，宽160，高120（图像中间的1/4区域）
black_threshold = (2, 36, -58, 57, -40, 37)
clock = time.clock()

#黑框宽度
boader_width_cm=2.0
# 物体的实际宽度（单位：厘米）
object_width_cm = 21.0
object_height_cm = 29.0
# 焦距（单位：像素）
focal_length_pixels = 560
#
MERGE_DISTANCE = 10  # 像素，合并距离
MAX_THETA_DIFF = 15  # 度，最大角度差
MIN_LINE_LENGTH = 12 # 像素，最小线段长度
CLUSTER_DISTANCE = 30  # 像素，聚类距离
side_length_buffer = []
# 全局列表用于存储5次测量结果（放在循环外部）
# 全局列表用于存储5次测量结果
measurement_history = []
side_length_history = []

#设置长度和距离为全局变量
distance_cm = None
side_length_cm = None


def find_frame1_blob(blobs):
    # 优先选择周长大的blob
    frame_blob = None
    max_ratio = 0
    for blob in blobs:
        ratio = blob.perimeter()
        if ratio > max_ratio:
            frame_blob = blob
            max_ratio = ratio
    return frame_blob
def find_frame2_blob(blobs):
    frame_blob = None
    max_score = 0
    for blob in blobs:
        # 综合周长和面积
        score = blob.perimeter() * math.sqrt(blob.area())
        if score > max_score:
            frame_blob = blob
            max_score = score
    return frame_blob
# 找到最大的色块
def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob = blob
            max_size = blob.pixels()
    return max_blob

# 测距函数
def calculate_distance(object_height_pixels, object_height_cm, focal_length_pixels):
    return object_height_cm * focal_length_pixels /object_height_pixels  # 实际宽度 × 焦距 / 像素宽度 = 距离
#求a4中心点
def calculate_center(blob):
    cx=blob.cx()
    cy=blob.cy()
    return cx,cy
#从上至下扫描黑框
def measure_border_width(img, blob):
    x = blob.cx()
    y = blob.y()
    border_width = 0

    for i in range(1, 30):
        if y + i >= img.height():
            break

        # RGB模式下get_pixel返回(r,g,b)元组
        r, g, b = img.get_pixel(x, y + i)

        # 转换为LAB色彩空间（近似计算）
        # 注意：这是简化版LAB转换，OpenMV内置find_blobs使用更精确的转换
        l = (r + g + b) / 3  # 近似亮度
        a = r - g  # 简化A通道
        b_val = b - (r + g)/2  # 简化B通道

        # LAB阈值判断
        if not (black_threshold[0] <= l <= black_threshold[1] and
                black_threshold[2] <= a <= black_threshold[3] and
                black_threshold[4] <= b_val <= black_threshold[5]):
            border_width = i
            break

    return border_width
# 新增图像预处理函数
def preprocess_image(img):
    # 高斯模糊降噪
    img.gaussian(1)
    # 边缘增强
    img.laplacian(1, sharpen=True)
    return img



# 改进的角点排序函数
def sort_corners(corners):
    """确保返回 [左下, 左上, 右上, 右下] 的顺序"""
    if len(corners) != 4:
        return corners

    # 计算中心点
    center_x = sum(p[0] for p in corners) / 4
    center_y = sum(p[1] for p in corners) / 4

    # 定义角度计算（从正x轴逆时针，范围[-π, π]）
    def angle_from_center(p):
        return math.atan2(p[1] - center_y, p[0] - center_x)

    # 按角度逆时针排序（此时顺序可能是 [左下, 左上, 右上, 右下] 或类似）
    sorted_ccw = sorted(corners, key=angle_from_center)

    # 强制调整顺序到 [左下, 左上, 右上, 右下]
    # 1. 找到 y 值最大的点（最下方的点）
    bottom_points = sorted(sorted_ccw, key=lambda p: -p[1])[:2]  # y最大的两个点
    # 2. 在这两个点中选 x 较小的作为左下
    bottom_left = min(bottom_points, key=lambda p: p[0])
    # 3. 确定左下角在列表中的位置
    idx = sorted_ccw.index(bottom_left)
    # 4. 重新排列顺序
    return sorted_ccw[idx:] + sorted_ccw[:idx]
def calculate_side_length(obj_blob, frame_blob, shape_type, rect_corners=None):
    """
    计算物体边长
    参数:
        obj_blob: 内部物体的blob对象
        frame_blob: 外框的blob对象
        shape_type: 物体形状类型 ("矩形", "三角形", "圆形")
        rect_corners: 矩形的角点列表 (仅当形状为矩形时使用)
    返回:
        物体的边长 (单位:厘米)
    """
    if shape_type == "矩形":
        # 使用矩形检测的角点
        if rect_corners and len(rect_corners) >= 2:
            top_left = rect_corners[0]
            bottom_left = rect_corners[1]
            dx = bottom_left[0] - top_left[0]
            dy = bottom_left[1] - top_left[1]
            side_px = math.sqrt(dx**2 + dy**2)
            return (side_px / frame_blob.h()) * object_height_cm
        else:
            # 后备方案：使用blob宽度
            return (obj_blob.h() / frame_blob.h()) * object_height_cm

    elif shape_type == "三角形":
        # 使用blob角点计算
        corners = obj_blob.corners()[:3]  # 只取前三个点
        sorted_corners = sort_corners(corners)

        if len(sorted_corners) >= 2:
            top_left = sorted_corners[0]
            bottom_left = sorted_corners[1] if len(sorted_corners) > 1 else sorted_corners[0]
            dx = bottom_left[0] - top_left[0]
            dy = bottom_left[1] - top_left[1]
            side_px = math.sqrt(dx**2 + dy**2)
            return (side_px / frame_blob.h()) * object_height_cm + 1  # 三角形补偿值
        else:
            return (obj_blob.h() / frame_blob.h()) * object_height_cm + 1

    elif shape_type == "圆形":
        # 使用高度比例计算
        return (obj_blob.h() / frame_blob.h()) * object_height_cm + 0.3  # 圆形补偿值

    else:
        # 未知形状使用默认方法
        return (obj_blob.h() / frame_blob.h()) * object_height_cm


def detect_min_square_by_edges(img, inner_roi, frame_blob, object_height_cm):
    """改进版：检测垂直相交的线段，补全正方形并计算边长"""
    # 计算每厘米对应的像素数
    pixels_per_cm = frame_blob.h() / object_height_cm

    # 查找线段
    lines = img.find_line_segments(
        roi=inner_roi,
        merge_distance=MERGE_DISTANCE,
        max_theta_diff=MAX_THETA_DIFF
    )

    # 筛选有效线段（长度足够）
    valid_lines = [line for line in lines if line.length() >= MIN_LINE_LENGTH]

    if not valid_lines:
        print("未检测到有效线段")
        return 0

    # 绘制所有有效线段（蓝色）
    for line in valid_lines:
        img.draw_line(line.line(), color=(0, 0, 255), thickness=2)

    # 寻找垂直相交的线段对
    square_sides = []
    for i in range(len(valid_lines)):
        for j in range(i+1, len(valid_lines)):
            line1 = valid_lines[i]
            line2 = valid_lines[j]

            # 计算两条线段的角度差（0-180度）
            theta_diff = abs(line1.theta() - line2.theta())
            theta_diff = min(theta_diff, 180 - theta_diff)

            # 检查是否近似垂直（85-95度）
            if 85 <= theta_diff <= 95:
                # 检查是否相交（简化版：检查端点距离）
                dist = min(
                    (line1.x1() - line2.x1())**2 + (line1.y1() - line2.y1())**2,
                    (line1.x1() - line2.x2())**2 + (line1.y1() - line2.y2())**2,
                    (line1.x2() - line2.x1())**2 + (line1.y2() - line2.y1())**2,
                    (line1.x2() - line2.x2())**2 + (line1.y2() - line2.y2())**2
                )

                # 如果端点距离足够近（视为相交）
                if dist < (max(line1.length(), line2.length()))**2 / 7:
                    # 取两条线中较长的一条作为正方形边长候选
                    side_length = max(line1.length(), line2.length())
                    square_sides.append(side_length)

                    # 绘制这对垂直线段（绿色）
                    img.draw_line(line1.line(), color=(0, 255, 0), thickness=2)
                    img.draw_line(line2.line(), color=(0, 255, 0), thickness=2)

    # 如果有找到垂直相交的线段对
    if square_sides:
        # 取所有候选边长中的最小值（因为要找最小正方形）
        min_side = min(square_sides)
        side_cm = min_side*1.466 / pixels_per_cm

        # 绘制最短的正方形边（红色）
        for line in valid_lines:
            if abs(line.length() - min_side) < 5:  # 近似等于最短边长
                img.draw_line(line.line(), color=(255, 0, 0), thickness=3)
                break

        print(f"可忽略找到垂直相交线段，边长: {side_cm:.1f}cm")
        return side_cm
    else:
        # 后备方案：如果没有垂直相交的线段，使用最短线段
        shortest_line = min(valid_lines, key=lambda l: l.length())
        side_cm = shortest_line.length() / pixels_per_cm

        # 绘制最短线段（红色）
        img.draw_line(shortest_line.line(), color=(255, 0, 0), thickness=3)

        print(f"未找到垂直相交线段，使用最短线段: {side_cm:.1f}cm")
        return side_cm
def update_buffer(value):
    """直接处理value值并返回"""
    # 边界处理
    if value <= 6.4:
        value = 6.5
    elif value > 11.5:
        value = 11.5
    # 其他情况保持原值
    return value



#处理接收字符从而开始不同的测量任务

LED1 = pyb.LED(1)
LED2 = pyb.LED(2)
LED3 = pyb.LED(3)
LED3.on()

uart =pyb.UART(3,9600)
#主任务
task = None




while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([black_threshold],pixels_threshold=200,area_threshold=200,roi=roi)  # 查找色块

    if blobs:
        max_blob = find_frame2_blob(blobs)  # 找到最大的黑框（目标黑框）
        img.draw_rectangle(max_blob.rect(), color=255)  # 绘制黑框的矩形框



        # 核心：用黑框整体的像素宽度计算距离
        object_pixel_height = max_blob.h()  # 黑框整体的像素宽度
        distance_cm = calculate_distance(max_blob.h(), object_height_cm, focal_length_pixels)  # 计算距离
        print(max_blob.h())

        # 计算黑框中心点
        center_x, center_y = calculate_center(max_blob)
        # 3. 设置内部ROI（缩小范围避免边缘干扰） 避免黑框的黑边
        inner_roi = (

            max_blob.x() + 12,
            max_blob.y() + 11,
            max(1, max_blob.w() - 25),
            max(1, max_blob.h() - 25)
            )
        #画roi
        img.draw_rectangle(inner_roi, color=(0, 255, 255))

        # 查找内部 ROI 的黑色物体
        objs = img.find_blobs([black_threshold], roi=inner_roi)  # 新增这一行
        obj_blob = find_max(objs)
        if not obj_blob:
            print("NO OBJS")
            continue
        wait = 1 #确保每次只执行一次
        receive  = uart.read()
        if receive:
           command = receive.decode('utf-8')
           if(command == 'A'):
                task = 1
                wait = 0
                LED1.on()
                LED2.off()
                LED3.off()


                print("任务1开启，并且只执行一次")
           if(command == 'B'):
                task = 2
                wait = 0
                LED2.on()
                LED1.off()
                LED3.off()
                print("任务2开启，并且只执行一次")
           if(command == '3'):
                task = 3
                wait = 0
                LED1.on()
                LED2.on()
                LED3.on()
        if wait == 0:
           wait = 1
           if task==1:
                    density =obj_blob.density()
                    side_length_cm=0
                    if density > 0.9:
                        print( "矩形")
                        shape_type = "矩形"
                        #side_length_cm=side_length_cm+1
                        # 取面积最大的矩形
                        # 使用find_rects检测矩形
                        rects = img.find_rects(threshold=10000, roi=inner_roi)
                        if rects:
                            max_rect = max(rects, key=lambda r: r.w() * r.h())
                            #img.draw_rectangle(max_rect.rect(), color=(0, 255, 0))

                            # 获取并排序角点
                            corners = max_rect.corners()
                            sorted_corners = sort_corners(corners)


                            # 绘制角点
                            for i, (x, y) in enumerate(sorted_corners):
                                img.draw_circle(x, y, 5, color=(255, 0, 0))
                                img.draw_string(x+5, y+5, str(i), color=(255, 0, 0))

                    elif density > 0.6:
                        shape_type = "圆形"
                        corners=0
                        print("圆形")
                    elif density > 0.4:
                         print("三角形")
                         shape_type = "三角形"
                         #side_length_cm=side_length_cm+1
                         # 获取角点
                         corners = obj_blob.corners()[:3]  # 强制只取前3个点

                         # 排序角点（使用sort_corners函数）
                         sorted_corners = sort_corners(corners)


                         # 绘制角点和边（可视化）
                         for i in range(len(sorted_corners)):
                             start = sorted_corners[i]
                             end = sorted_corners[(i+1) % len(sorted_corners)]
                             #img.draw_line(start[0], start[1], end[0], end[1], color=(0, 255, 0))
                             img.draw_circle(start[0], start[1], 5, color=(255, 0, 0))

                    else:
                         corners=0
                         print("未知形状")
                         shape_type = "未知"
                    # 统一调用边长计算函数
                    side_length_cm = calculate_side_length(
                         obj_blob,
                         max_blob,
                         shape_type,
                         rect_corners=corners
                     )
                    print(f"边长: {side_length_cm:.1f}cm")
                    mes = f"{distance_cm},{side_length_cm},\n"
                    uart.write(mes.encode('utf-8'))

           elif task==2:
            # 检测最小正方形边长
                    side_length_cm = detect_min_square_by_edges(img, inner_roi, max_blob, object_width_cm)

                    # 处理当前测量值并保存
                    processed_value = update_buffer(side_length_cm)
                    measurement_history.append(processed_value)

                    # 保持最多5次测量记录
                    if len(measurement_history) > 5:
                        measurement_history.pop(0)

                    # 当有5次测量时，取最小值作为最终结果
                    if len(measurement_history) == 5:
                        final_result = min(measurement_history)
                        print(f"5次测量值: {measurement_history} | 最终结果: {final_result:.1f}cm")

                    # 显示信息
                    img.draw_string(10, 10, f"Distance: {distance_cm:.1f} cm", color=(255, 0, 0))
                    img.draw_string(10, 30, f"Center: ({center_x},{center_y})", color=(0, 255, 255))
                    print(f"Distance: {distance_cm:.1f} cm | Center: ({center_x}, {center_y})")
                    mes = f"{distance_cm},{side_length_cm},\n"
                    uart.write(mes.encode('utf-8'))


                    # 延时100ms
                    time.sleep_ms(100)
           elif task==3:#发挥第四问
                corners = obj_blob.corners()[:4]  # 确保取前4个角点

                # 增加角点有效性检查
                if len(corners) < 4:
                    print("警告：未检测到4个角点！使用备用方案")
                    side_length_cm = obj_blob.h() / max_blob.h() * object_height_cm + 1
                else:
                    sorted_corners = sort_corners(corners)  # 假设该函数返回[左上, 右上, 右下, 左下]

                    # 正确选择对角点：左上(0)和右下(2)
                    top_left = sorted_corners[1]
                    bottom_right = sorted_corners[3]

                    # 计算实际像素距离
                    dx = bottom_right[0] - top_left[0]
                    dy = bottom_right[1] - top_left[1]
                    diagonal_px = math.sqrt(dx**2 + dy**2)  # 对角线长度

                    # 修正计算公式：对角线→边长转换
                    side_length_cm = (diagonal_px / math.sqrt(2)) * (object_height_cm / max_blob.h())+1.40
                    # if 75<max_blob.w()<79 and max_blob.h() > 100:
                    #     side_length_cm+=0.30
                    # if 60<max_blob.w()<75 and max_blob.h() > 100:
                    #     side_length_cm+=0.55
                    # if 0<max_blob.w()<60 and max_blob.h() > 100:
                    #     side_length_cm+=5.65
                    #     # 记录当前测量结果（最多保留5次）
                    # if max_blob.h()<100 and 0<max_blob.w()<58 :
                    #     side_length_cm+=5.3

                    side_length_history.append(side_length_cm)
                    if len(side_length_history) > 5:
                        side_length_history.pop(0)  # 移除最旧的记录

                    # 计算中位数（当有5次数据时）
                    final_length_cm = side_length_cm  # 默认使用当前值
                    if len(side_length_history) == 5:
                        sorted_history = sorted(side_length_history)
                        final_length_cm = sorted_history[2]  # 取中位数
                    # 调试打印
                    side_length_cm=final_length_cm
                   # print(f"对角线像素: {diagonal_px:.1f}px, 参照物高度: {max_blob.h()}px")

                #print(f"当前边长: {side_length_cm:.1f}cm, 最终中位数: {final_length_cm:.1f}cm")
                print(f"当前边长: {final_length_cm:.1f}cm")
                mes = f"{distance_cm},{final_length_cm},\n"
                uart.write(mes.encode('utf-8'))








        # 方法1：使用原始方法计算尺寸
        # obj_width_cm, obj_height_cm = calculate_object_size(obj_blob, max_blob, boader_width_cm)

        # 方法2：使用角点坐标计算尺寸（更精确）


        # 绘制角点（可视化）
        corners = obj_blob.corners()
        for i, corner in enumerate(corners):
            img.draw_circle(corner[0], corner[1], 3, color=(255, 0, 0))
            img.draw_string(corner[0]+5, corner[1]+5, str(i), color=(255, 0, 0))

        #print(f"物体尺寸: {obj_width_cm:.1f}cm x {obj_height_cm:.1f}cm")

        # 在图像上显示距离和中心点信息
        img.draw_string(10, 10, f"Distance: {distance_cm:.1f} cm", color=(255, 0, 0))  # 显示距离
        img.draw_string(10, 30, f"Center: ({center_x},{center_y})", color=(0, 255, 255))  # 显示中心点
        #img.draw_cross(center_x, center_y, color=(0, 255, 0), size=15, thickness=1)  # 绘制中心十字

        # 分析内部几何图形
        #analyze_inner_shape(img, max_blob, border_pixel_width)
        # print(f"Distance: {distance_cm:.1f} cm | Center: ({center_x}, {center_y})")
        # print(f"Blob宽宽度: {max_blob.w()}px")
        # print(f"Blob高宽度: {max_blob.h()}px")
   # print(clock.fps())  # 打印帧率

