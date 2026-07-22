#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import threading
import subprocess
import sys
import os
import json
import time
import cv2
import base64
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
from uav import uav_recon
from ugv import ugv_navigation
import math

class MissionExecutor:
    def __init__(self):
        rospy.init_node('mission_executor_node', anonymous=True)
        self.task_lock = threading.Lock()
        self.is_busy = False
        rospy.Subscriber('/mission_command', String, self.command_callback)
        self.feedback_pub = rospy.Publisher('/mission_feedback', String, queue_size=10)
        self.vision_query_pub = rospy.Publisher('/vision_query', String, queue_size=5)
        self.targets_json_path = "./targets.json"
        self.detected_json_path = "./detected_targets.json"
        self.uav_home_position = None
        self.bridge = CvBridge()
        self.latest_img = None
        self.vision_result = None
        rospy.Subscriber('/rflysim/sensor3/img_rgb', RosImage, self._img_callback)
        rospy.Subscriber('/vision_result', String, self._vision_result_callback)
        self._init_uav_home_position()
        rospy.loginfo("Mission Executor Node Started.")

    def _img_callback(self, msg):
        try:
            self.latest_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _vision_result_callback(self, msg):
        try:
            self.vision_result = json.loads(msg.data).get('result', '')
        except Exception:
            self.vision_result = msg.data

    def _init_uav_home_position(self):
        """获取无人机初始位置作为返航点"""
        try:
            msg = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=5.0)
            self.uav_home_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            rospy.loginfo(f"UAV home recorded: {self.uav_home_position}")
        except rospy.ROSException:
            self.uav_home_position = [0.0, 0.0, 0.0]
            rospy.logwarn("UAV home not available, using default [0,0,0]")

    def _match_command(self, raw_cmd):
        """精确匹配大模型输出的指令名"""
        cmd = raw_cmd.strip().lower()
        valid = ["uav_reconnaissance_mission", "result_sync", "car_navigation",
                 "thing_detect", "back_home", "sequence_all"]
        if cmd in valid:
            return cmd
        rospy.logwarn(f"Unknown command: '{cmd}' (expected exact LLM output)")
        return None

    def command_callback(self, msg):
        command = msg.data.strip()
        if self.is_busy:
            rospy.logwarn(f"System busy, rejecting: {command}")
            return
        thread = threading.Thread(target=self.dispatch_task, args=(command,))
        thread.start()

    def dispatch_task(self, command):
        with self.task_lock:
            self.is_busy = True
            try:
                matched = self._match_command(command)
                if matched is None:
                    rospy.logwarn(f"Unknown command: {command}")
                    self.send_feedback("error", f"无法识别指令: {command}")
                    return

                rospy.loginfo(f"Matched '{command}' -> task: {matched}")

                if matched == "sequence_all":
                    self.execute_sequence()
                elif matched == "uav_reconnaissance_mission":
                    self.handle_uav_recon()
                elif matched == "result_sync":
                    self.handle_result_sync()
                elif matched == "car_navigation":
                    self.handle_car_navigation()
                elif matched == "thing_detect":
                    self.handle_thing_detect()
                elif matched == "back_home":
                    self.handle_back_home()
            except Exception as e:
                rospy.logerr(f"Task execution error: {e}")
                self.send_feedback("error", str(e))
            finally:
                self.is_busy = False

    def _query_vision(self, query, timeout=30.0):
        """拍摄当前画面，base64编码发送给多模态大模型分析"""
        waited = 0.0
        while self.latest_img is None and waited < 5.0:
            rospy.sleep(0.2)
            waited += 0.2
        if self.latest_img is None:
            rospy.logwarn("No camera image available for vision query")
            return "no_image"

        # 保存一张到磁盘方便人工检查
        debug_path = "/tmp/mission_vision_debug.jpg"
        os.makedirs(os.path.dirname(debug_path), exist_ok=True)
        cv2.imwrite(debug_path, self.latest_img)

        h, w = self.latest_img.shape[:2]
        rospy.loginfo(f"Camera image: {w}x{h}, mean brightness={self.latest_img.mean():.1f}, saved to {debug_path}")

        # JPEG编码 + base64
        _, jpg = cv2.imencode('.jpg', self.latest_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        img_b64 = base64.b64encode(jpg).decode('ascii')
        rospy.loginfo(f"JPEG size: {len(jpg)} bytes, base64: {len(img_b64)} chars")

        self.vision_result = None
        payload = json.dumps({"image_b64": img_b64, "query": query}, ensure_ascii=False)
        self.vision_query_pub.publish(payload)
        rospy.loginfo(f"Vision query published, msg length={len(payload)}")

        start = time.time()
        while self.vision_result is None and (time.time() - start) < timeout:
            rospy.sleep(0.3)
        result = self.vision_result or "timeout"
        rospy.loginfo(f"Vision result arrived after {time.time()-start:.1f}s: {result[:200]}")
        self.vision_result = None
        return result

    def send_feedback(self, step, content):
        msg = String()
        payload = {"step": step, "content": content}
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.feedback_pub.publish(msg)
        rospy.loginfo(f"Feedback sent: {step}")

    # ================== 任务序列执行 ==================
    def execute_sequence(self):
        """依序执行全部 5 个任务"""
        rospy.loginfo("===== Starting full mission sequence =====")
        self.send_feedback("sequence_start", "开始依序执行全部5项任务。")

        rospy.loginfo("[1/5] UAV Reconnaissance")
        self.handle_uav_recon()
        rospy.sleep(1.0)

        rospy.loginfo("[2/5] Result Sync")
        self.handle_result_sync()
        rospy.sleep(1.0)

        rospy.loginfo("[3/5] Car Navigation")
        self.handle_car_navigation()
        rospy.sleep(1.0)

        rospy.loginfo("[4/5] Delivery Verification")
        self.handle_thing_detect()
        rospy.sleep(1.0)

        rospy.loginfo("[5/5] Return Home")
        self.handle_back_home()

        self.send_feedback("sequence_done", "全部5项任务已依序执行完成。")

    # ================== 任务1: 无人机侦察 ==================
    def handle_uav_recon(self):
        rospy.loginfo("[Task 1] UAV reconnaissance...")
        WAYPOINTS = [
            [0, 0, 1.7], [4.1, 0.2, 1.7], [4.1, 8.0, 1.7],
            [2.0, 8.0, 1.7], [-0.4, 6.0, 1.7], [2.0, 4.0, 1.7], [2.0, 6.0, 1.7]
        ]
        success = uav_recon.run_reconnaissance_task(
            waypoints=WAYPOINTS,
            enable_yolo=False,
            yolo_model_path="yolov8n.pt",
            camera_topic="/rflysim/sensor2/img_rgb",
            detection_json_path="./detected_targets.json"
        )
        if success:
            self.send_feedback("recon_done", "侦察完成，已生成点云地图并完成目标识别。")
        else:
            self.send_feedback("recon_fail", "侦察任务失败。")

        # 侦察结束后释放可视化资源，不影响后续任务
        self._cleanup_viz()

    def _cleanup_viz(self):
        """关闭 rviz 等可视化进程，释放 CPU 资源"""
        try:
            subprocess.run(["pkill", "-f", "rviz"], check=False, timeout=3)
            rospy.loginfo("rviz cleaned up")
        except Exception:
            pass

    # ================== 任务2: 同步侦察结果 ==================
    def handle_result_sync(self):
        rospy.loginfo("[Task 2] Syncing results and generating map...")
        success = uav_recon.convert_pcd_to_2d_map()
        if not success:
            self.send_feedback("sync_fail", "地图生成失败。")
            return

        summary = "已生成二维导航地图。"
        if os.path.exists(self.detected_json_path):
            with open(self.detected_json_path, 'r') as f:
                data = json.load(f)
            if data:
                class_names = set(obj.get('class_name', 'unknown') for obj in data)
                summary += f"侦察阶段识别到 {len(data)} 个目标：{'、'.join(class_names)}。"
            else:
                summary += "未识别到特定目标。"

        if os.path.exists(self.targets_json_path):
            with open(self.targets_json_path, 'r') as f:
                tdata = json.load(f)
            if tdata.get('person'):
                p = tdata['person']
                summary += f"救援目标位置: x={p['x']:.2f}, y={p['y']:.2f}。"
            if tdata.get('robot'):
                r = tdata['robot']
                summary += f"无人车初始位置: x={r['x']:.2f}, y={r['y']:.2f}。"

        self.send_feedback("sync_done", summary)

    # ================== 任务3: 无人车运输 ==================
    def handle_car_navigation(self):
        rospy.loginfo("[Task 3] UGV medical transport...")
        if not os.path.exists(self.targets_json_path):
            self.send_feedback("nav_fail", "未找到坐标文件 targets.json，请先执行侦察与同步任务。")
            return
        with open(self.targets_json_path, 'r') as f:
            data = json.load(f)
        robot_pose = data.get('robot')
        person_pose = data.get('person')
        if not robot_pose or not person_pose:
            self.send_feedback("nav_fail", "坐标文件中缺少车或人的位置信息")
            return

        offset_x = -0.5
        target_x = person_pose['x'] + offset_x
        target_y = person_pose['y'] - offset_x
        rospy.loginfo(f"Target person: {person_pose}, nav goal: ({target_x:.2f}, {target_y:.2f})")

        success = ugv_navigation.run_ugv_mission(
            target_x=target_x,
            target_y=target_y,
            start_x=robot_pose['x'],
            start_y=robot_pose['y']
        )
        if success:
            self.unload_goods()
            self.send_feedback("nav_done", "物资已成功送达目标点。")
        else:
            self.send_feedback("nav_fail", "无人车导航失败。")

    def unload_goods(self):
        rospy.loginfo("Unloading medical supplies...")
        rospy.sleep(2.0)
        rospy.loginfo("Medical supplies unloaded.")

    # ================== 任务4: 检测物资送达 ==================
    def handle_thing_detect(self):
        rospy.loginfo("[Task 4] Verifying delivery status...")

        if not os.path.exists(self.targets_json_path):
            self.send_feedback("detect_fail", "未找到targets.json，无法核实送达情况。")
            return

        with open(self.targets_json_path, 'r') as f:
            data = json.load(f)

        person_pose = data.get('person_uav') or data.get('person')
        if not person_pose:
            self.send_feedback("detect_fail", "无法获取救援人员位置。")
            return

        target_x = person_pose['x']
        target_y = person_pose['y']
        rospy.loginfo(f"Flying UAV to person: ({target_x:.2f}, {target_y:.2f}) [UAV frame]")
        observe_height = 3.0

        self.send_feedback("detect_progress", f"无人机正飞往人员位置上方核实...")

        success = uav_recon.fly_to_observe_position(
            target_x=target_x,
            target_y=target_y,
            observe_height=observe_height,
            hover_time=3.0
        )

        if not success:
            self.send_feedback("detect_fail", "无人机无法飞抵目标位置进行核实。")
            return

        # 用多模态大模型分析当前画面
        query = (
            "这是一张无人机下视摄像头拍摄的救援场景画面。"
            "请仔细观察并描述图片中的内容（地形、物体、人员等），"
            "然后判断救援物资（医疗箱）是否在受伤人员附近。"
            "如确认物资已到位请回答'已送达'，否则回答'未送达'。"
        )
        result = self._query_vision(query)
        rospy.loginfo(f"Vision LLM raw result: {result}")

        # 保存到日志方便人工核对
        try:
            with open("/tmp/vision_debug.log", "w") as f:
                f.write(f"Query: {query}\n\nResult: {result}\n")
        except Exception:
            pass

        if "已送达" in result and "未送达" not in result:
            self.send_feedback("detect_done",
                f"任务完成：救援物资已准确送达人员附近。多模态识别结论：{result}")
        elif "未送达" in result:
            self.send_feedback("detect_done",
                f"任务未完成：救援物资未送达指定位置。多模态识别结论：{result}")
        else:
            self.send_feedback("detect_done",
                f"视觉检测结果：{result}")

    # ================== 任务5: 全员返航 ==================
    def handle_back_home(self):
        rospy.loginfo("[Task 5] All units return to base...")
        self.send_feedback("home_progress", "无人机与无人车正在执行返航...")

        ugv_result = [None]
        uav_result = [None]

        def ugv_return():
            try:
                if os.path.exists(self.targets_json_path):
                    with open(self.targets_json_path, 'r') as f:
                        data = json.load(f)
                    robot_pose = data.get('robot')
                    if robot_pose:
                        rospy.loginfo(f"UGV returning to start: ({robot_pose['x']:.2f}, {robot_pose['y']:.2f})")
                        # 轻量返航：复用现有导航栈，仅发送新目标
                        ok = ugv_navigation.navigate_back_to_start(
                            target_x=robot_pose['x'],
                            target_y=robot_pose['y']
                        )
                        ugv_result[0] = ok
                        return
                ugv_result[0] = False
            except Exception as e:
                rospy.logerr(f"UGV return error: {e}")
                ugv_result[0] = False

        def uav_return():
            try:
                rospy.loginfo(f"UAV returning to home: {self.uav_home_position}")
                ok = uav_recon.return_to_home(
                    home_pose=self.uav_home_position,
                    safe_height=3.0
                )
                uav_result[0] = ok
            except Exception as e:
                rospy.logerr(f"UAV return error: {e}")
                uav_result[0] = False

        t_ugv = threading.Thread(target=ugv_return)
        t_uav = threading.Thread(target=uav_return)
        t_ugv.start()
        t_uav.start()
        t_ugv.join(timeout=300)
        t_uav.join(timeout=300)

        ugv_ok = ugv_result[0] if ugv_result[0] is not None else False
        uav_ok = uav_result[0] if uav_result[0] is not None else False

        parts = []
        parts.append("无人车已返回营区出发点" if ugv_ok else "无人车返航失败")
        parts.append("无人机已返回并降落" if uav_ok else "无人机返航失败")
        summary = "返航完成：" + "；".join(parts) + "。"
        self.send_feedback("home_done", summary)


if __name__ == "__main__":
    try:
        executor = MissionExecutor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
