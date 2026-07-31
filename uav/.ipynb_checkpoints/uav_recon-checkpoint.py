#!/usr/bin/env python3
import subprocess
import time
import os
import signal
import rospy
import math
import sys
import termios
import cv2
import numpy as np
import open3d as o3d
import yaml
import json
import warnings
import threading
import torch
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from ultralytics import YOLO
warnings.filterwarnings("ignore", category=UserWarning)
# 尝试导入 YOLOv8 和 cv_bridge，如果没有安装也不影响原有建图功能
try:
    from ultralytics import YOLO
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️ 警告: 未安装 ultralytics 或 cv_bridge，将跳过实时目标识别功能。")
# =====================================================================
# 能力1：侦察任务（建图 + 航点飞行 + YOLOv8实时识别去重）
# =====================================================================
def run_reconnaissance_task( 
    waypoints, 
    fast_lio_cmd="cd /root/gpufree-data/ws_livox/ && source devel/setup.bash && roslaunch fast_lio mapping_avia.launch",
    vel_lin=0.5, 
    vel_z=0.4,
    enable_yolo=False,
    yolo_model_path="./models/yolov8n.pt", 
    camera_topic="/rflysim/sensor3/img_rgb",
    detection_json_path="./detected_targets.json",
    conf_threshold=0.75,
    # --- 【新增】性能控制参数 ---
    target_fps=1,            # YOLO 推理频次（5次/秒）。显示依然是流畅的，但每秒只算5帧图
    img_size=416             # 推理输入尺寸 (越小越快，默认640，改416能提升约40%速度)
 ):
    print("\n" + "=" * 50)
    print(" [能力1] 开始执行侦察任务 ")
    print("=" * 50)
    yolo_model = None
    latest_img = [None] 
    current_uav_pose = [None]
    yolo_shutdown_flag = [False]
    def pose_for_yolo_cb(msg):
        current_uav_pose[0] = msg
    if enable_yolo and YOLO_AVAILABLE:
        print(f"[YOLO] 正在加载模型: {yolo_model_path}...")
        try:
            yolo_model = YOLO(yolo_model_path)            
            bridge = CvBridge()
            def image_callback(msg):
                try:
                    latest_img[0] = bridge.imgmsg_to_cv2(msg, "bgr8")
                except Exception:
                    pass 
            rospy.Subscriber(camera_topic, Image, image_callback, queue_size=1)
            rospy.Subscriber('mavros/local_position/pose', PoseStamped, pose_for_yolo_cb)
            active_tracks = []
            final_objects = []
            # 把性能参数传给后台线程
            yolo_thread = threading.Thread(
                target=_yolo_visualizer_thread, 
                args=(yolo_model, latest_img, current_uav_pose, yolo_shutdown_flag, 
                      active_tracks, final_objects, conf_threshold, target_fps, img_size),
                daemon=True
            )
            yolo_thread.start()
            print(f"[YOLO] ✅ 启动成功！推理频次: {target_fps}fps，输入尺寸: {img_size}")
        except Exception as e:
            print(f"[YOLO] ❌ 模型加载或订阅失败: {e}")
            yolo_model = None
    elif enable_yolo and not YOLO_AVAILABLE:
        print("[YOLO] ⚠️ 缺少依赖库，已自动禁用识别。")
    print("[1/3] 启动 FAST-LIO 建图...")
    map_process = subprocess.Popen(["bash", "-c", fast_lio_cmd],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    time.sleep(5)
    print("[2/3] 启动无人机巡航...")
    try:
        _run_uav_waypoint_flight(waypoints, vel_lin, vel_z)
        print("[2/3] ✅ 无人机巡航完成")
    except Exception as e:
        print(f"[2/3] ❌ 无人机异常: {e}")
    finally:
        print("\n[3/3] 关闭建图节点...")
        subprocess.run(["pkill", "-SIGINT", "fastlio_mapping"], check=False, stdout=subprocess.DEVNULL)
        time.sleep(4)
        print("✅ 建图节点已关闭。")
    if yolo_model is not None:
        yolo_shutdown_flag[0] = True
        time.sleep(0.5)
        cv2.destroyAllWindows()
        os.makedirs(os.path.dirname(detection_json_path), exist_ok=True)
        clean_results = []
        for obj in final_objects:
            clean_results.append({
                "class_name": obj["class_name"],
                "class_id": obj["class_id"],
                "pixel_bbox": {"x1": obj["x1"], "y1": obj["y1"], "x2": obj["x2"], "y2": obj["y2"]},
                "uav_pose_at_detection": obj.get("uav_pose", "unknown")
            })
        with open(detection_json_path, 'w', encoding='utf-8') as f:
            json.dump(clean_results, f, ensure_ascii=False, indent=4)
        print(f"\n🎯 [YOLO] 侦察结束！共发现 {len(clean_results)} 个不重复目标，已保存至: {detection_json_path}")
    return True
def _yolo_visualizer_thread(model, latest_img, current_uav_pose, shutdown_flag, active_tracks, final_objects, conf_thresh, target_fps, img_size):
    """
    后台显示线程：【关键优化】推理和显示解耦，达到降频不减帧的效果
    """
    inference_interval = 1.0 / target_fps
    last_inference_time = 0.0
    annotated_frame = None  # 用于缓存上一次的画框结果
    while not shutdown_flag[0]:
        if latest_img[0] is None:
            time.sleep(0.5)
            continue
        current_time = time.time()
        frame = latest_img[0].copy()
        # 【核心逻辑】只有当距离上次推理超过设定的时间间隔时，才真正跑YOLO
        if current_time - last_inference_time >= inference_interval:
            # 传入 img_size 降低计算量
            results = model(frame, verbose=False, conf=conf_thresh, imgsz=img_size)
            new_detections = []
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id = int(box.cls[0])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    new_detections.append({'class_id': cls_id, 'class_name': model.names[cls_id], 'cx': cx, 'cy': cy, 'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2})
            DIST_THRESH = 80
            TIME_THRESH = 3.0
            for det in new_detections:
                matched_active = False
                for track in active_tracks:
                    if track['class_id'] == det['class_id']:
                        dist = math.hypot(det['cx'] - track['cx'], det['cy'] - track['cy'])
                        if dist < DIST_THRESH and (current_time - track['last_seen_time']) < TIME_THRESH:
                            track['cx'], track['cy'] = det['cx'], det['cy']
                            track['last_seen_time'] = current_time
                            track['x1'], track['y1'], track['x2'], track['y2'] = det['x1'], det['y1'], det['x2'], det['y2']
                            matched_active = True
                            break
                if matched_active: continue
                matched_history = False
                for final_obj in final_objects:
                    if final_obj['class_id'] == det['class_id']:
                        dist = math.hypot(det['cx'] - final_obj['cx'], det['cy'] - final_obj['cy'])
                        if dist < DIST_THRESH: matched_history = True; break
                if matched_history: continue
                uav_pose_dict = "unknown"
                if current_uav_pose[0]:
                    uav_pose_dict = {"x": current_uav_pose[0].pose.position.x, "y": current_uav_pose[0].pose.position.y, "z": current_uav_pose[0].pose.position.z}
                new_record = {**det, "last_seen_time": current_time, "uav_pose": uav_pose_dict}
                active_tracks.append(new_record)
                final_objects.append(new_record)
                print(f"🚨 [新目标] 发现 {det['class_name']} (像素中心: {det['cx']}, {det['cy']})")
            active_tracks[:] = [t for t in active_tracks if (current_time - t['last_seen_time']) < TIME_THRESH]
            # 更新缓存的画框图像
            annotated_frame = results[0].plot()
            cv2.putText(annotated_frame, f"Unique Targets: {len(final_objects)} (Inf: {target_fps}fps)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            last_inference_time = current_time
        # 不管有没有进行新的推理，只要有缓存图，就立马刷新显示窗口
        if annotated_frame is not None:
            cv2.imshow("UAV Real-time Reconnaissance", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
def _run_uav_waypoint_flight(waypoints, vel_lin, vel_z):
    current_pose = rospy.Publisher.__class__ # Dummy init
    from geometry_msgs.msg import Twist, PoseStamped
    current_pose = PoseStamped()
    target_pose = PoseStamped()
    has_temp_goal = False
    waypoint_index = 0
    is_waiting = False
    wait_start_time = 0.0
    shutdown_flag = False
    POSITION_TOLERANCE = 0.2
    WAYPOINT_DELAY = 1.5
    def pose_cb(msg): nonlocal current_pose; current_pose = msg
    def goal_cb(msg): nonlocal target_pose, has_temp_goal; target_pose = msg; has_temp_goal = True
    def sigint_handler(signum, frame): nonlocal shutdown_flag; shutdown_flag = True; sys.exit(0)
    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGINT, sigint_handler)
        settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=5)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)
    rospy.Subscriber('/my_goal', PoseStamped, goal_cb)
    current_state = State()
    def state_cb(msg): nonlocal current_state; current_state = msg
    rospy.Subscriber('/mavros/state', State, state_cb)
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    rate = rospy.Rate(30)
    print("[UAV] 等待飞控连接...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    print("[UAV] 建立指令流心跳...")
    for i in range(100):
        if rospy.is_shutdown(): return
        pub.publish(Twist())
        rate.sleep()
    print("[UAV] 尝试切换 OFFBOARD 模式...")
    if current_state.mode != "OFFBOARD":
        set_mode_client(custom_mode="OFFBOARD")
        start_wait = time.time()
        while not rospy.is_shutdown() and current_state.mode != "OFFBOARD":
            if time.time() - start_wait > 5.0:
                print("[UAV] ❌ 模式切换超时")
                return
            rate.sleep()
    print("[UAV] 尝试解锁...")
    if not current_state.armed:
        arming_client(value=True)
        start_wait = time.time()
        while not rospy.is_shutdown() and not current_state.armed:
            if time.time() - start_wait > 5.0:
                print("[UAV] ❌ 解锁超时")
                return
            rate.sleep()
    print("[UAV] ✅ 已解锁并进入 OFFBOARD 模式，开始巡航...")
    try:
        while not rospy.is_shutdown() and not shutdown_flag:
            twist = Twist()
            tx, ty, tz = 0, 0, 0
            if has_temp_goal:
                tx, ty, tz = target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z
            elif waypoint_index < len(waypoints):
                if is_waiting:
                    if time.time() - wait_start_time >= WAYPOINT_DELAY: waypoint_index += 1; is_waiting = False
                    pub.publish(Twist()); rate.sleep(); continue
                else: tx, ty, tz = waypoints[waypoint_index]
            else: pub.publish(Twist()); break
            cx, cy, cz = current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z
            dx, dy, dz = tx - cx, ty - cy, tz - cz
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            if dist < POSITION_TOLERANCE:
                if has_temp_goal: has_temp_goal = False
                elif not is_waiting: is_waiting = True; wait_start_time = time.time()
            elif dist > 0:
                twist.linear.x = (dx / dist) * vel_lin
                twist.linear.y = (dy / dist) * vel_lin
                twist.linear.z = (dz / dist) * vel_z
            pub.publish(twist); rate.sleep()
    finally:
        pub.publish(Twist())
        if threading.current_thread() is threading.main_thread():
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
# =====================================================================
# 能力2：三维点云转二维地图 + 目标识别 + 存JSON
# =====================================================================
def convert_pcd_to_2d_map( 
    pcd_path="/root/gpufree-data/ws_livox/src/FAST_LIO/PCD/scans.pcd", 
    output_dir="./height_maps_final",
    output_json_path="./targets.json",
    ref_map_path="./models/bestv1.pgm",
    resolution=0.01, start_height=0.20, step_height=0.04,
    layer_thickness=0.40, run_count=15, flip_image_left_right=True,
    morph_kernel_size=(2, 2), min_contour_area=10, dilate_iterations=1, similarity_threshold=0.85
 ):
    print("\n" + "=" * 50)
    print(" [能力2] 开始生成地图与识别目标 ")
    print("=" * 50)
    os.makedirs(output_dir, exist_ok=True)
    print("[1/3] 正在批量切片并寻找最相似地图...")
    best_pgm, best_yaml = _run_batch_slicing(
        pcd_path, output_dir, ref_map_path, resolution, start_height, step_height,
        layer_thickness, run_count, flip_image_left_right, morph_kernel_size,
        min_contour_area, dilate_iterations, similarity_threshold
    )
    if not best_pgm: return False
    print("[2/3] 正在进行二次优化与目标识别...")
    final_pgm = os.path.join(output_dir, "processed_map.pgm")
    final_yaml = os.path.join(output_dir, "processed_map.yaml")
    # 返还值增加了最大置信度
    robot_pose, person_pose, robot_conf, person_conf, robot_uav, person_uav = \
        _run_optimize_and_detect(best_pgm, best_yaml, final_pgm, final_yaml, output_dir)
    print("[3/3] 保存坐标数据...")
    result = {
        "robot": {"x": robot_pose[0], "y": robot_pose[1], "confidence": round(robot_conf, 4)} if robot_pose else None,
        "person": {"x": person_pose[0], "y": person_pose[1], "confidence": round(person_conf, 4)} if person_pose else None,
        "person_uav": {"x": person_uav[0], "y": person_uav[1]} if person_uav else None,
        "map_pgm": final_pgm, "map_yaml": final_yaml
    }
    with open(output_json_path, 'w', encoding='utf-8') as f: json.dump(result, f, ensure_ascii=False, indent=4)
    print(f"\n🎉 处理完成！坐标已存入: {output_json_path}")
    return True
def _run_batch_slicing(pcd_path, output_dir, ref_map_path, res, z_start, z_step, z_thick, runs, flip_lr, k_size, min_area, dilate_iter, sim_thresh):
    if not os.path.isfile(pcd_path): print(f"❌ PCD不存在: {pcd_path}"); return None, None
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    if points.size == 0: return None, None
    points[:, 2] -= np.min(points[:, 2])
    xy = points[:, :2]
    min_x, min_y = np.min(xy, axis=0)
    max_x, max_y = np.max(xy, axis=0)
    img_w = int((max_x - min_x) / res) + 1
    img_h = int((max_y - min_y) / res) + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, k_size)
    ref_img = None
    if os.path.isfile(ref_map_path): ref_img = cv2.imread(ref_map_path, cv2.IMREAD_GRAYSCALE)
    best_sim, best_img, best_name, best_yaml = 0, None, "", None
    for i in range(runs):
        z_low = z_start + i * z_step
        z_high = z_low + z_thick
        mask = (points[:, 2] > z_low) & (points[:, 2] < z_high)
        seg = points[mask]
        if len(seg) == 0: continue
        x_px = ((seg[:, 0] - min_x) / res).astype(int)
        y_px = ((seg[:, 1] - min_y) / res).astype(int)
        img = np.ones((img_h, img_w), np.uint8) * 255
        valid = (x_px >= 0) & (x_px < img_w) & (y_px >= 0) & (y_px < img_h)
        img[y_px[valid], x_px[valid]] = 0
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(255 - img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) < min_area: cv2.drawContours(img, [cnt], -1, 255, -1)
        img = cv2.dilate(img, kernel, iterations=dilate_iter)
        img[:2, :] = 255; img[-2:, :] = 255; img[:, :2] = 255; img[:, -2:] = 255
        if flip_lr: img = cv2.flip(img, 1)
        name = f"map_{z_low:.2f}-{z_high:.2f}"
        curr_sim = 0
        if ref_img is not None:
            curr_sim = _calculate_similarity(ref_img, img)
            if curr_sim > best_sim:
                best_sim, best_img, best_name = curr_sim, img.copy(), name
                best_yaml = {"image": f"{name}.pgm", "resolution": res, "origin": [float(min_x), float(min_y), 0.0], "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196}
            if best_sim >= 0.98: break
    if best_img is not None:
        cv2.imwrite(os.path.join(output_dir, "new_best.pgm"), best_img)
        with open(os.path.join(output_dir, "new_best.yaml"), 'w') as f: yaml.dump(best_yaml, f)
        print(f"🏆 找到最佳切片: {best_name}, 相似度: {best_sim:.4f}")
        return os.path.join(output_dir, "new_best.pgm"), best_yaml
    print("⚠️ 未找到参考图或相似度过低，使用最后一张切片")
    cv2.imwrite(os.path.join(output_dir, "new_best.pgm"), img)
    return os.path.join(output_dir, "new_best.pgm"), best_yaml
def _calculate_similarity(img1, img2):
    if img1.shape != img2.shape: img2 = cv2.resize(img2, (img1.shape[1], img1.shape[0]))
    hist_sim = cv2.compareHist(cv2.calcHist([img1],[0],None,[256],[0,256]), cv2.calcHist([img2],[0],None,[256],[0,256]), cv2.HISTCMP_CORREL)
    pixel_sim = 1.0 - (cv2.absdiff(img1, img2).mean() / 255.0)
    return max((hist_sim * 0.4) + (pixel_sim * 0.6), 0)
def _run_optimize_and_detect(best_pgm_path, best_yaml_data, output_pgm, output_yaml, output_dir):
    MODEL_PATH   = "./models/best.pt"
    img = cv2.imread(best_pgm_path, cv2.IMREAD_GRAYSCALE)
    res = best_yaml_data["resolution"]; ox, oy = best_yaml_data["origin"][0], best_yaml_data["origin"][1]
    coords = cv2.findNonZero(255 - img)
    if coords is not None:
        rect = cv2.minAreaRect(coords); angle = rect[-1]
        if angle < -45: angle += 90
        if angle > 45: angle -= 90
        h, w = img.shape; M = cv2.getRotationMatrix2D((w//2, h//2), angle, 1.0)
        img_rot = cv2.warpAffine(img, M, (w, h), borderValue=255)
    else: img_rot = img
    coords = cv2.findNonZero(255 - img_rot)
    if coords is not None: 
        x, y, w, h = cv2.boundingRect(coords); 
        img_crop = img_rot[y:y+h, x:x+w]; (x_off, y_off) = (x, y)
    else: img_crop = img_rot; (x_off, y_off) = (0, 0)
    h_img, w_img = img_crop.shape
    # ---------------------- 传统方法：兜底找小车 ----------------------
    def find_robot_traditional(img_gray, vis_img):
        h, w = img_gray.shape
        x_start = int(w * 0.05)
        x_end   = int(w * 0.15)
        y_start = int(h * 0.55)
        y_end   = int(h * 0.95)
        roi = img_gray[y_start:y_end, x_start:x_end]
        contours, _ = cv2.findContours(255 - roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        robot_box = None
        robot_center = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 10 < area < 500:
                rx, ry, rw, rh = cv2.boundingRect(cnt)
                gx = x_start + rx
                gy = y_start + ry
                cv2.rectangle(vis_img, (gx, gy), (gx+rw, gy+rh), (0,255,0), 3)
                robot_box = (gx, gy, gx+rw, gy+rh)
                robot_center = (gx + rw//2, gy + rh//2)
                break
        return robot_box, robot_center, vis_img
    # 找小车和人 传统 + Yolo
    img_rgb = cv2.cvtColor(img_crop, cv2.COLOR_GRAY2BGR)
    # 加载YOLO模型
    model = YOLO(MODEL_PATH)
    # YOLO 检测
    results = model(img_rgb, conf=0.15, iou=0.2, agnostic_nms=True)
    robot_box    = None
    robot_center = None
    person_box   = None
    person_center= None
    robot_conf   = 0.0
    person_conf  = 0.0  # 新增：记录人物的最大置信度
    # 解析 YOLO 结果
    for r in results:
        for box in r.boxes:
            cls = model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            # 可视化绘制：将所有识别到的对象均画框显示
            if cls == "person":
                cv2.rectangle(img_rgb, (x1,y1), (x2,y2), (0,0,255), 3)
                cv2.putText(img_rgb, f"person {conf:.2f}", (x1,y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                # 核心修改：如果发现多个人，仅保留概率(置信度)最大的
                if conf > person_conf:
                    person_box = (x1, y1, x2, y2)
                    person_center = (cx, cy)
                    person_conf = conf
            if cls == "robot":
                cv2.rectangle(img_rgb, (x1,y1), (x2,y2), (0,255,0), 3)
                cv2.putText(img_rgb, f"robot {conf:.2f}", (x1,y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                # 核心修改：如果发现多个车，仅保留概率(置信度)最大的
                if conf > robot_conf:
                    robot_box = (x1, y1, x2, y2)
                    robot_center = (cx, cy)
                    robot_conf = conf
    print(f"[YOLO提取] 最高概率小车置信度: {robot_conf:.2f}, 人物置信度: {person_conf:.2f}")
    # 🔥 核心：YOLO 找不到 / 置信度低 → 传统算法兜底
    if robot_box is None or robot_conf < 0.2:
        print("⚠️ YOLO 未找到小车 / 置信度过低，启用传统算法")
        robot_box, robot_center, img_rgb = find_robot_traditional(img_crop, img_rgb)
        robot_conf = 0.0  # 传统算法兜底时，置信度置为0表示非模型提供
    # ===================== 生成干净地图：纯黑 + 纯白，无模糊 =====================
    clean_pgm = img_crop.copy()
    # 二值化：强制只有 0(黑) 和 255(白)
    _, clean_pgm = cv2.threshold(clean_pgm, 127, 255, cv2.THRESH_BINARY)
    # 把小车区域涂白
    if robot_box:
        x1, y1, x2, y2 = robot_box
        cv2.rectangle(clean_pgm, (x1, y1), (x2, y2), 255, -1)
    # 保存（高清无损）
    cv2.imwrite(output_pgm, clean_pgm)
    new_yaml = best_yaml_data.copy(); 
    new_yaml["image"] = os.path.basename(output_pgm)
    new_yaml["origin"] = [ox + x_off * res, oy + y_off * res, 0.0]
    with open(output_yaml, 'w') as f: yaml.dump(new_yaml, f)
    # ===================== 新增：保存带有识别标志的图片 =====================
    vis_save_path = os.path.join(output_dir, "detection_visualization.png")
    cv2.imwrite(vis_save_path, img_rgb)
    print(f"🖼️ [识别可视化] 带有标志的图片已保存至: {vis_save_path}")
    # 4.坐标计算(原版公式 — MAP坐标系，图像已被水平翻转)
    robot_pose = (ox + robot_center[0] * res, oy + (h_img - robot_center[1]) * res) if robot_center else None
    person_pose = (ox + person_center[0] * res, oy + (h_img - person_center[1]) * res) if person_center else None

    # ===== UAV坐标系转换 =====
    # MAP坐标系: X+=上(北), Y+=左(西), 原点在图像右上角
    # UAV坐标系: X+=右(东), Y+=上(北), 原点在起飞点
    # 转换关系: UAV_x = -MAP_y + Cx,  UAV_y = MAP_x + Cy
    # Cx, Cy 从 targets.json 的 robot 推算: robot 在世界中位于 UAV 起飞点正北≈3.7m
    #   robot_uav ≈ (0, 3.7),  robot_map 从检测得到
    #   Cy = 3.7 - robot_map_x,  Cx = 0 - (-robot_map_y) = robot_map_y
    # 但 robot_map_y 接近0, 导致Cx≈0, 不合理. 使用场景几何硬编码偏移.
    MAP_TO_UAV_CX = 3.2   # UAV_x = -MAP_y + 3.2
    MAP_TO_UAV_CY = 7.9   # UAV_y = MAP_x + 7.9
    robot_uav = None
    person_uav = None
    if robot_pose:
        robot_uav = (-robot_pose[1] + MAP_TO_UAV_CX, robot_pose[0] + MAP_TO_UAV_CY)
    if person_pose:
        person_uav = (-person_pose[1] + MAP_TO_UAV_CX, person_pose[0] + MAP_TO_UAV_CY)
    if robot_pose: print(f"🤖 小车 MAP: x={robot_pose[0]:.3f}, y={robot_pose[1]:.3f} (Conf: {robot_conf:.2f})")
    else: print("❌ 未找到小车")
    if person_pose: print(f"🚶 人物 MAP: x={person_pose[0]:.3f}, y={person_pose[1]:.3f} (Conf: {person_conf:.2f})")
    if robot_uav: print(f"🤖 小车 UAV: x={robot_uav[0]:.3f}, y={robot_uav[1]:.3f}")
    if person_uav: print(f"🚶 人物 UAV: x={person_uav[0]:.3f}, y={person_uav[1]:.3f}")
    return robot_pose, person_pose, robot_conf, person_conf, robot_uav, person_uav
# =====================================================================
# 能力3：返航功能（爬升避障 + 直线返航）
# =====================================================================
def return_to_home(home_pose=None, safe_height=3.5, vel_lin=0.5, vel_z=0.4):
    """
    返航能力函数，支持从任意点返航。
    为防止撞击障碍物，采用策略：先爬升至安全高度 -> 水平飞回起飞点上方 -> 下降至起飞高度 -> 降落。
    请注意，这里可以采取航电飞行，不省高度的方式，选手可自行优化，另决赛中真机飞行在室内不支持过高高度的飞行
    """
    print("\n" + "=" * 50)
    print(" [能力3] 开始执行返航任务 (爬升避障策略) ")
    print("=" * 50)
    if home_pose is None:
        home_pose = [0, 0, 0.0] # 默认起点
    current_pose = PoseStamped()
    current_state = State()
    def pose_cb(msg): nonlocal current_pose; current_pose = msg
    def state_cb(msg): nonlocal current_state; current_state = msg
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)
    rospy.Subscriber('/mavros/state', State, state_cb)
    pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=5)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    rate = rospy.Rate(30)

    # 等待连接，然后发心跳建立/恢复指令流（不检查arm/mode，仿真状态不可靠）
    print("[RTH] 等待飞控连接...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    print("[RTH] 发送心跳建立指令流...")
    for _ in range(80):
        pub.publish(Twist())
        rate.sleep()
    POSITION_TOLERANCE = 0.2
    # 阶段1：爬升到安全高度，高于所有可能障碍物
    cur_z = current_pose.pose.position.z
    if cur_z < safe_height:
        print(f"[RTH] 阶段1/3: 爬升至安全高度 {safe_height}m...")
        while not rospy.is_shutdown():
            dz = safe_height - current_pose.pose.position.z
            if abs(dz) < POSITION_TOLERANCE:
                break
            twist = Twist()
            twist.linear.z = (dz / abs(dz)) * vel_z
            pub.publish(twist)
            rate.sleep()
        pub.publish(Twist())
    # 阶段2：水平移动到起飞点正上方
    print(f"[RTH] 阶段2/3: 水平移动至起飞点上方...")
    while not rospy.is_shutdown():
        dx = home_pose[0] - current_pose.pose.position.x
        dy = home_pose[1] - current_pose.pose.position.y
        dist_xy = math.sqrt(dx**2 + dy**2)
        if dist_xy < POSITION_TOLERANCE:
            break
        twist = Twist()
        twist.linear.x = (dx / dist_xy) * vel_lin
        twist.linear.y = (dy / dist_xy) * vel_lin
        pub.publish(twist)
        rate.sleep()
    pub.publish(Twist())
    # 阶段3：垂直下降到起飞高度
    print(f"[RTH] 阶段3/3: 下降至起飞高度 {home_pose[2]}m...")
    while not rospy.is_shutdown():
        dz = home_pose[2] - current_pose.pose.position.z
        if abs(dz) < POSITION_TOLERANCE:
            break
        twist = Twist()
        twist.linear.z = (dz / abs(dz)) * vel_z
        pub.publish(twist)
        rate.sleep()
    pub.publish(Twist())
    # 降落
    print("[RTH] 到达起飞点，切换 AUTO.LAND 模式降落...")
    set_mode_client(custom_mode="AUTO.LAND")
    LAND_DETECT_HEIGHT = 0.15  # 判定已经落地的阈值高度
    print("[RTH] 等待飞机着陆 (高度 < 0.15m) ...")
    
    # 等待高度下降到极低值，确认着陆
    landing_start_time = time.time()
    while not rospy.is_shutdown():
        if current_pose.pose.position.z < LAND_DETECT_HEIGHT:
            # 确保在极低高度停留一小段时间，防止误判
            time.sleep(1.0)
            if current_pose.pose.position.z < LAND_DETECT_HEIGHT:
                break
        # 超时保护，防止飞机一直在空中盘旋无法降落
        if time.time() - landing_start_time > 30.0: 
            print("[RTH] ⚠️ 降落超时，强制尝试上锁")
            break
        rate.sleep()
 
    # 降落后主动上锁，停止桨叶转动
    print("[RTH] 检测到已着陆，正在上锁电机...")
    if current_state.armed:
        arming_client(value=False)
        # 等待上锁成功
        start_wait = time.time()
        while not rospy.is_shutdown() and current_state.armed:
            if time.time() - start_wait > 5.0:
                print("[RTH] ❌ 上锁超时")
                break
            rate.sleep()
    
    if not current_state.armed:
        print("✅ 返航完成，飞机已安全着陆并上锁！")
    else:
        print("⚠️ 返航降落完成，但上锁失败，请手动上锁！")
        
    return True
# =====================================================================
# 能力4：飞至观察位置（用于送货验证）
# =====================================================================
def fly_to_observe_position(target_x, target_y, observe_height=3.0, hover_time=3.0, vel_lin=0.5, vel_z=0.4):
    """
    飞至目标位置（x, y）上方 observe_height 高度悬停观察指定时长。
    用于任务4：无人机核实物资送达情况。
    """
    print("\n" + "=" * 50)
    print(" [能力4] 飞至观察位置 ")
    print(f" 目标: ({target_x:.2f}, {target_y:.2f}), 高度: {observe_height}m, 悬停: {hover_time}s")
    print("=" * 50)

    current_pose = PoseStamped()
    current_state = State()

    def pose_cb(msg):
        nonlocal current_pose; current_pose = msg
    def state_cb(msg):
        nonlocal current_state; current_state = msg

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)
    rospy.Subscriber('/mavros/state', State, state_cb)

    pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=5)
    rate = rospy.Rate(30)
    POSITION_TOLERANCE = 0.25

    # 等待连接，然后发心跳建立/恢复指令流（不检查arm/mode，仿真状态不可靠）
    print("[Observe] 等待飞控连接...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    print("[Observe] 发送心跳建立指令流...")
    for _ in range(80):
        pub.publish(Twist())
        rate.sleep()

    print("[Observe] ✅ 已就绪，开始飞往目标...")

    # 阶段1：先飞到目标水平位置
    print("[Observe] 阶段1/2: 水平移动到目标上方...")
    while not rospy.is_shutdown():
        dx = target_x - current_pose.pose.position.x
        dy = target_y - current_pose.pose.position.y
        dist_xy = math.sqrt(dx**2 + dy**2)
        if dist_xy < POSITION_TOLERANCE:
            break
        twist = Twist()
        twist.linear.x = (dx / dist_xy) * vel_lin
        twist.linear.y = (dy / dist_xy) * vel_lin
        pub.publish(twist)
        rate.sleep()
    pub.publish(Twist())

    # 阶段2：调整到观察高度
    print("[Observe] 阶段2/2: 调整至观察高度...")
    while not rospy.is_shutdown():
        dz = observe_height - current_pose.pose.position.z
        if abs(dz) < POSITION_TOLERANCE:
            break
        twist = Twist()
        twist.linear.z = (dz / abs(dz)) * vel_z
        pub.publish(twist)
        rate.sleep()
    pub.publish(Twist())

    # 悬停观察
    print(f"[Observe] 已就位，悬停观察 {hover_time} 秒...")
    hover_start = time.time()
    while not rospy.is_shutdown() and (time.time() - hover_start) < hover_time:
        pub.publish(Twist())
        rate.sleep()

    print("[Observe] ✅ 观察完成")
    return True


if __name__ == "__main__":
    rospy.init_node("recon_and_mapping_node", anonymous=True)
    WAYPOINTS = [
        [0, 0, 1.7], [4.1, 0.2, 1.7], [4.1, 8.0, 1.7],
        [2.0, 8.0, 1.7], [-0.4, 6.0, 1.7], [2.0, 4.0, 1.7], [2.0, 6.0, 1.7]
    ]
    # 记录起飞点位置，用于后续返航
    home_pose_msg = [None]
    def get_initial_pose(msg): home_pose_msg[0] = msg
    sub_init = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, get_initial_pose)
    print("[Main] 等待获取无人机初始位置...")
    while home_pose_msg[0] is None and not rospy.is_shutdown():
        time.sleep(0.1)
    sub_init.unregister()
    home_position = [
        home_pose_msg[0].pose.position.x, 
        home_pose_msg[0].pose.position.y, 
        home_pose_msg[0].pose.position.z
    ]
    print(f"[Main] 起飞点记录为: x={home_position[0]:.2f}, y={home_position[1]:.2f}, z={home_position[2]:.2f}")
    # 调用能力1：侦察建图
    run_reconnaissance_task(
        waypoints=WAYPOINTS,
        enable_yolo=False,
        yolo_model_path="./models/yolov8n.pt", 
        camera_topic="/rflysim/sensor3/img_rgb",
        detection_json_path="./detected_targets.json"
    )
    # 调用能力2：二维地图生成与识别
    convert_pcd_to_2d_map()
    # 调用能力3：安全返航
    return_to_home(home_pose=home_position, safe_height=3.0)