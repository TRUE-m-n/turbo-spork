#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import subprocess
import sys
import time
import threading
import signal
import math
import json
import os
import rospy
import tf2_ros
import tf.transformations
import sensor_msgs.point_cloud2 as pc2
import actionlib
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import UE4CtrlAPI as UE4CtrlAPI

# ====================== 配置路径 ======================
MOVE_BASE_LAUNCH = "./ugv/movebase.launch"
# =====================================================
shutdown_flag = False
launch_process = None
def signal_handler(sig, frame):
    global shutdown_flag, launch_process
    rospy.loginfo("收到中断信号，正在关闭...")
    shutdown_flag = True
    if launch_process and launch_process.poll() is None:
        launch_process.terminate()
        launch_process.wait()
    rospy.signal_shutdown("User requested shutdown")
    sys.exit(0)
class UGVController:
    def __init__(self, target_x, target_y):
        # 兼容模块调用
        if not rospy.core.is_initialized():
            rospy.init_node("ugv_controller_node", anonymous=True, disable_signals=True)
        self.target_x = target_x
        self.target_y = target_y
        # 1. 状态订阅与服务
        self.current_state = State()
        rospy.Subscriber('/ugv/mavros/state', State, self.state_cb)
        self.arming_client = rospy.ServiceProxy('/ugv/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/ugv/mavros/set_mode', SetMode)
        # 2. 指令发布 (用于心跳和解锁)
        self.vel_pub = rospy.Publisher('/ugv/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        # 3. 导航辅助
        self.scan_pub = rospy.Publisher('/rflysim/sensor0/vehicle_lidar_points', LaserScan, queue_size=10)
        rospy.Subscriber('/rflysim/sensor0/vehicle_lidar', PointCloud2, self.cloud_callback)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/rflysim/uav2/local/odom', Odometry, self.odom_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # 4. move_base 客户端
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # 5. 【新增】速度指令缓存与转发逻辑
        self.current_cmd_vel = Twist()
        self.yaw = 0.0
        self.last_cmd_vel_time = rospy.Time.now()
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.loginfo("UGV Controller 初始化完成，等待飞控连接...")
    def state_cb(self, msg):
        self.current_state = msg
    def cloud_callback(self, cloud_msg):
        # (保持原逻辑不变)
        scan = LaserScan()
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 0.01
        scan.range_min = 0.5
        scan.range_max = 20.0
        scan.time_increment = 0.00001
        scan.scan_time = 0.1
        beam_count = int(math.ceil((scan.angle_max - scan.angle_min) / scan.angle_increment))
        scan.ranges = [float('nan')] * beam_count
        # scan.header.stamp = cloud_msg.header.stamp
        scan.header.stamp = rospy.Time.now() -   rospy.Duration(0.05)
        scan.header.frame_id = "livox_frame"
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            x, y = point[0], point[1]
            r = math.hypot(x, y)
            angle = math.atan2(y, x)
            idx = int(round((angle - scan.angle_min) / scan.angle_increment))
            if 0 <= idx < beam_count and scan.range_min <= r <= scan.range_max:
                if math.isnan(scan.ranges[idx]) or r < scan.ranges[idx]:
                    scan.ranges[idx] = r
        self.scan_pub.publish(scan)
    def odom_callback(self, odom_msg):
        # (保持原逻辑不变)
        if rospy.is_shutdown(): return
        t = TransformStamped()
        # t.header.stamp = odom_msg.header.stamp
        t.header.stamp = rospy.Time.now() -  rospy.Duration(0.05)

        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
        
    def cmd_vel_callback(self, msg):
        # 只保存原始指令，不在此处发布
        self.current_cmd_vel = msg        # ← 取消注释这一行
        self.last_cmd_vel_time = rospy.Time.now()
        
    def cmd_vel_loop(self):
        rate = rospy.Rate(20)
        w_thresh = 0.2      # 超过此值开始降线速度
        w_max = 0.3         # 超过此值线速度降为0
        
        while not rospy.is_shutdown() and not shutdown_flag:
            twist_out = Twist()
            # 如果0.5秒内收到过新指令，则使用当前指令
            if (rospy.Time.now() - self.last_cmd_vel_time).to_sec() < 0.5:
                raw = self.current_cmd_vel
                final_v = raw.linear.x
                final_w = raw.angular.z * 1.8
                
                # 转弯时降低线速度（保留原逻辑）
                if abs(final_w) > w_thresh:
                    factor = max(0.0, 1.0 - (abs(final_w) - w_thresh) / (w_max - w_thresh))
                    final_v = final_v * factor
                
                twist_out.linear.x = final_v
                twist_out.angular.z = final_w
            else:
                # 超时则发零指令保证安全
                twist_out.linear.x = 0.0
                twist_out.angular.z = 0.0
            
            self.vel_pub.publish(twist_out)
            rate.sleep()
            
    def setup_offboard_and_arm(self):
        """心跳发送、模式切换与解锁"""
        rate = rospy.Rate(20)
        # 1. 等待连接
        last_request = rospy.Time.now()
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo_throttle(1, "等待 UGV 飞控连接...")
            rate.sleep()
        # 2. 预发送指令流 (心跳)
        rospy.loginfo("正在发送心跳指令流...")
        for i in range(100):
            if rospy.is_shutdown(): return False
            self.vel_pub.publish(Twist())
            rate.sleep()
        # 3. 切换 Offboard 模式
        rospy.loginfo("尝试切换 OFFBOARD 模式...")
        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD":
                if (rospy.Time.now() - last_request) > rospy.Duration(1.0):
                    self.set_mode_client(custom_mode="OFFBOARD")
                    last_request = rospy.Time.now()
            else:
                rospy.loginfo("✅ OFFBOARD 模式已设置")
                break
            rate.sleep()
        # 4. 解锁
        rospy.loginfo("尝试解锁...")
        while not rospy.is_shutdown():
            if not self.current_state.armed:
                if (rospy.Time.now() - last_request) > rospy.Duration(1.0):
                    self.arming_client(value=True)
                    last_request = rospy.Time.now()
            else:
                rospy.loginfo("✅ 已解锁，UGV 准备就绪")
                break
            rate.sleep()
        return True
    def move_to_goal(self, x, y, yaw=0.0, timeout=60.0):
        """导航到指定点（带costmap清理和超时检测）"""
        rospy.loginfo(f"🚀 导航至目标: ({x:.2f}, {y:.2f})")
        # 清除旧costmap，强制重新规划
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', rospy.Duration(2.0))
            from std_srvs.srv import Empty
            clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_srv()
            rospy.loginfo("   costmaps cleared")
        except Exception:
            pass

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base_client.send_goal(goal)
        # 轮询等待，带进度监控
        start_time = rospy.Time.now()
        rate = rospy.Rate(2)
        last_state = -1
        while not rospy.is_shutdown():
            state = self.move_base_client.get_state()
            if state != last_state:
                state_names = {0:"PENDING", 1:"ACTIVE", 2:"PREEMPTED", 3:"SUCCEEDED",
                               4:"ABORTED", 5:"REJECTED", 6:"PREEMPTING", 7:"RECALLING", 8:"RECALLED", 9:"LOST"}
                rospy.loginfo(f"   move_base state: {state_names.get(state, state)}")
                last_state = state
            if state in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.PREEMPTED,
                         actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED,
                         actionlib.GoalStatus.LOST]:
                break
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("   navigation timed out")
                break
            rate.sleep()

        final_state = self.move_base_client.get_state()
        self.move_base_client.cancel_all_goals()
        if final_state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("✅ 到达目标点")
            return True
        else:
            rospy.logwarn(f"❌ 导航失败 (state={final_state})")
            return False
# ================== 模块化调用接口 ==================
def run_ugv_mission(target_x, target_y, start_x=None, start_y=None):
    """
    外部调用接口：执行无人车任务
    """
    global launch_process
    rospy.loginfo("====== 开始 UGV 任务流程 ======")
    # 1. 先实例化控制器，启动数据转换与TF发布
    controller = UGVController(target_x, target_y)
    # 2. 启动后台指令发布线程
    vel_thread = threading.Thread(target=controller.cmd_vel_loop, daemon=True)
    vel_thread.start()
    # 3. 等待关键数据就绪（非常重要！）
    rospy.loginfo("等待 /rflysim/sensor0/vehicle_lidar_points 话题就绪...")
    try:
        rospy.wait_for_message('/rflysim/sensor0/vehicle_lidar_points', LaserScan, timeout=10.0)
        rospy.loginfo("✅ LaserScan 数据已就绪")
    except rospy.ROSException:
        rospy.logerr("❌ 等待 LaserScan 超时，请检查点云话题是否有数据！")
        return False
    rospy.loginfo("等待 odom -> base_link TF 就绪...")
    rate = rospy.Rate(10)
    timeout_time = rospy.Time.now() + rospy.Duration(10.0)
    while not rospy.is_shutdown():
        try:
            controller.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0))
            rospy.loginfo("✅ TF odom -> base_link 已就绪")
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            if rospy.Time.now() > timeout_time:
                rospy.logerr("❌ 等待 TF 超时，请检查 /rflysim/uav2/local/odom 话题是否有数据！")
                return False
            rate.sleep()
	    # 4. 确保以上就绪后，再启动 move_base
    if os.path.exists(MOVE_BASE_LAUNCH):
        print("启动 movebase...")
        launch_process = subprocess.Popen(["roslaunch", MOVE_BASE_LAUNCH])
        # 等待 move_base action server 就绪（比固定sleep更可靠）
        rospy.loginfo("等待 move_base action server...")
        move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        if not move_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr("❌ move_base 启动超时")
            return False
        rospy.loginfo("✅ move_base 已就绪")
    else:
        rospy.logwarn(f"未找到 Launch 文件: {MOVE_BASE_LAUNCH}")
        return False

    # 5. 执行解锁流程
    if not controller.setup_offboard_and_arm():
        rospy.logerr("解锁失败，任务终止")
        return False

    # 6. 设置初始位姿并等待 AMCL 收敛
    if start_x is not None and start_y is not None:
        initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        rospy.sleep(0.5)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = start_x
        pose_msg.pose.pose.position.y = start_y
        pose_msg.pose.pose.orientation.w = 1.0
        initial_pose_pub.publish(pose_msg)
        rospy.loginfo(f"已发布初始位姿: ({start_x}, {start_y})")

        # 等待 TF map->base_link 可用（AMCL收敛的标志）
        rospy.loginfo("等待 AMCL 收敛 (map->base_link TF)...")
        tf_timeout = rospy.Time.now() + rospy.Duration(15.0)
        map_ready = False
        while not rospy.is_shutdown() and rospy.Time.now() < tf_timeout:
            try:
                controller.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
                rospy.loginfo("✅ AMCL 已收敛，map->base_link TF 就绪")
                map_ready = True
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(0.5)
        if not map_ready:
            rospy.logwarn("⚠️ AMCL 收敛超时，继续尝试导航...")
    else:
        rospy.sleep(2.0)  # 没有初始位姿时短暂等待

    # 7. 执行导航（带重试）
    success = False
    for attempt in range(3):
        rospy.loginfo(f"导航尝试 {attempt+1}/3 ...")
        success = controller.move_to_goal(target_x, target_y, timeout=120.0)
        if success:
            break
        rospy.logwarn(f"导航尝试 {attempt+1} 失败，{'重试中...' if attempt < 2 else '放弃'}")
        rospy.sleep(2.0)
    # ---- 原导航行替换结束 ----
    if success:
        rospy.loginfo("🎉 UGV 导航任务完成")
        # 此处调用ue接口直接释放药箱，因机械臂控制不在赛题重点考核范围，为降低难度聚焦重点直接给出释放函数
        ue = UE4CtrlAPI.UE4CtrlAPI()
        ue.sendUE4ExtAct(2,[0,0,0,0,0,0,20,1,0,0,0,0,0,0,0,0])
        rospy.loginfo("🎉 UGV 释放药箱任务完成")
    else:
        rospy.logwarn("⚠️ UGV 任务未成功完成")
    return success
# ================== 轻量返航接口（复用现有导航栈）==================
def navigate_back_to_start(target_x, target_y, timeout=90.0):
    """
    轻量返航：复用现有导航栈，带重试。
    """
    rospy.loginfo(f"Returning to start: ({target_x:.2f}, {target_y:.2f})")
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("move_base server not available for return")
        return False

    for attempt in range(3):
        # 清除旧costmap，强制重新规划
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', rospy.Duration(1.0))
            from std_srvs.srv import Empty
            clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_srv()
        except Exception:
            pass

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        client.send_goal(goal)
        finished = client.wait_for_result(rospy.Duration(timeout))
        if finished and client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Successfully returned to start")
            return True
        rospy.logwarn(f"Return attempt {attempt+1} failed, {'retrying...' if attempt < 2 else 'giving up'}")
        client.cancel_all_goals()
        rospy.sleep(2.0)
    return False


# ================== 主函数入口 ==================
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    targets_json_path = "../script/targets.json"
    if not os.path.exists(targets_json_path):
        print("nav_fail", "未找到坐标文件 targets.json")
        exit()
    with open(targets_json_path, 'r') as f:
        data = json.load(f)
    robot_pose = data.get('robot')
    person_pose = data.get('person')
    if not robot_pose or not person_pose:
        print("nav_fail", "坐标文件中缺少车或人的位置信息")
        exit()
    offset_x = -0.5
    target_x = person_pose['x'] + offset_x
    target_y = person_pose['y'] - offset_x 
    rospy.loginfo(f"目标点(人): {person_pose}, 导航终点(偏移后): ({target_x:.2f}, {target_y:.2f})")
    try:
        # 示例：直接运行时的默认参数
        success = run_ugv_mission(
            target_x=target_x, 
            target_y=target_y, 
            start_x=robot_pose['x'], 
            start_y=robot_pose['y']
        )
    except Exception as e:
        rospy.logerr(f"主程序异常: {e}")
    finally:
        if launch_process and launch_process.poll() is None:
            launch_process.terminate()
            launch_process.wait()