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

from nav_msgs.msg import Odometry, OccupancyGrid

from nav_msgs.srv import GetPlan, GetPlanRequest

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from mavros_msgs.msg import State

from mavros_msgs.srv import CommandBool, SetMode

import UE4CtrlAPI as UE4CtrlAPI



# ====================== 新增：导入子目标模块 ======================

from .subgoal_dwa import SubGoalManager, generate_straight_path

# ================================================================



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





# ====================== 新增：调用 move_base 全局规划器（C++ Dijkstra/A*）======================

def get_global_path_from_movebase(start_x, start_y, goal_x, goal_y, tolerance=0.1):

    """

    调用 /move_base/make_plan 服务获取全局路径。

    tolerance 改小为 0.1，避免终点漂移导致绕远路。

    """

    try:

        rospy.wait_for_service('/move_base/make_plan', timeout=5.0)

        make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)



        req = GetPlanRequest()

        req.start.header.frame_id = "map"

        req.start.header.stamp = rospy.Time.now()

        req.start.pose.position.x = start_x

        req.start.pose.position.y = start_y

        req.start.pose.orientation.w = 1.0



        req.goal.header.frame_id = "map"

        req.goal.header.stamp = rospy.Time.now()

        req.goal.pose.position.x = goal_x

        req.goal.pose.position.y = goal_y

        req.goal.pose.orientation.w = 1.0



        req.tolerance = tolerance



        resp = make_plan(req)

        if not resp.plan.poses:

            rospy.logwarn("make_plan 返回空路径")

            return []



        # 采样：约每 0.4m 保留一个路径点，避免太密集

        path = []

        last_pt = None

        for pose in resp.plan.poses:

            pt = (pose.pose.position.x, pose.pose.position.y)

            if last_pt is None or math.hypot(pt[0] - last_pt[0], pt[1] - last_pt[1]) >= 0.4:

                path.append(pt)

                last_pt = pt



        # 确保终点在路径中

        if not path or math.hypot(path[-1][0] - goal_x, path[-1][1] - goal_y) > 0.1:

            path.append((goal_x, goal_y))



        rospy.loginfo(f"✅ 全局规划完成，路径点数: {len(path)}")

        return path



    except Exception as e:

        rospy.logwarn(f"调用 make_plan 失败: {e}")

        return []

# ===================================================================================





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

        scan.header.stamp = rospy.Time.now() - rospy.Duration(0.05)

        scan.header.frame_id = "livox_frame"

        for point in pc2.read_points(cloud_msg, skip_nans=True):

            x, y = point[0], point[1]

            r = math.hypot(x, y)

            angle = math.atan2(y, x)

            idx = int(round((angle - scan.angle_min) / scan.angle_increment))

            if 0 <= idx < beam_count and scan.range_min <= r <= scan.range_max:

                if math.isnan(scan.ranges[idx]) or r < scan.ranges[idx]:

                    scan.ranges[idx] = r



        # 检查是否有有效数据，避免发布全 nan 的 scan

        valid_count = sum(1 for r in scan.ranges if not math.isnan(r))

        if valid_count == 0:

            rospy.logwarn_throttle(5.0, "cloud_callback: 无有效点云数据，跳过本次 LaserScan 发布")

            return



        self.scan_pub.publish(scan)



    def odom_callback(self, odom_msg):

        # (保持原逻辑不变)

        if rospy.is_shutdown():

            return

        t = TransformStamped()

        # t.header.stamp = odom_msg.header.stamp

        t.header.stamp = rospy.Time.now() - rospy.Duration(0.05)

        t.header.frame_id = "odom"

        t.child_frame_id = "base_link"

        t.transform.translation.x = odom_msg.pose.pose.position.x

        t.transform.translation.y = odom_msg.pose.pose.position.y

        t.transform.translation.z = 0.0

        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)



    def cmd_vel_callback(self, msg):

        # 只保存原始指令，不在此处发布

        self.current_cmd_vel = msg

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

                # 去掉 1.8 倍放大，避免原地打转

                final_w = raw.angular.z



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

            if rospy.is_shutdown():

                return False

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



    def is_goal_valid(self, x, y):

        """检查目标点是否在自由区域"""

        try:

            # 获取代价地图

            costmap = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout=2.0)



            # 转换坐标到地图索引

            map_x = int((x - costmap.info.origin.position.x) / costmap.info.resolution)

            map_y = int((y - costmap.info.origin.position.y) / costmap.info.resolution)



            # 检查边界

            if map_x < 0 or map_x >= costmap.info.width or map_y < 0 or map_y >= costmap.info.height:

                rospy.logwarn(f"目标点 ({x}, {y}) 超出地图边界")

                return False



            # 检查代价

            index = map_y * costmap.info.width + map_x

            if 0 <= index < len(costmap.data):

                cost = costmap.data[index]

                if cost >= 100:  # 致命障碍或膨胀区（100以上是障碍，-1是未知）

                    rospy.logwarn(f"目标点 ({x}, {y}) 在障碍物内，代价值: {cost}")

                    return False

                rospy.loginfo(f"目标点 ({x}, {y}) 代价值: {cost}，可用")

            return True

        except Exception as e:

            rospy.logwarn(f"检查目标点有效性失败: {e}")

            return True  # 默认允许，避免阻塞



    def move_to_goal(self, x, y, yaw=0.0, timeout=120.0):

        """使用子目标的导航方法（替代原有直接发终点）"""

        rospy.loginfo(f"🚀 子目标导航至终点: ({x:.2f}, {y:.2f})")



        # ===== 新增：终点预检查，不可达则搜索附近替代点 =====

        if not self.is_goal_valid(x, y):

            rospy.logerr(f"目标点 ({x}, {y}) 不可达，尝试寻找附近可达点...")

            found = False

            for r in [0.3, 0.5, 0.8, 1.0, 1.5]:

                for angle in [0, math.pi/4, math.pi/2, 3*math.pi/4, math.pi,

                              -math.pi/4, -math.pi/2, -3*math.pi/4]:

                    tx = x + r * math.cos(angle)

                    ty = y + r * math.sin(angle)

                    if self.is_goal_valid(tx, ty):

                        rospy.loginfo(f"找到替代目标点: ({tx:.2f}, {ty:.2f})")

                        x, y = tx, ty

                        found = True

                        break

                if found:

                    break

            if not found:

                rospy.logerr("附近无可达目标点，导航失败")

                return False

        # ===== 终点预检查完成 =====



        # 清除旧costmap，强制重新规划（只在开始时清一次）

        try:

            rospy.wait_for_service('/move_base/clear_costmaps', rospy.Duration(2.0))

            from std_srvs.srv import Empty

            clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

            clear_srv()

            rospy.loginfo("   costmaps cleared")

        except Exception:

            pass



        # 等待 costmap 重建

        rospy.loginfo("等待 costmap 重建...")

        rospy.sleep(1.0)



        # ===== 新增：子目标管理器 =====

        subgoal_mgr = SubGoalManager(lookahead_distance=2.0, subgoal_tolerance=0.5)



        # 获取当前位置，生成全局路径

        try:

            trans = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))

            start_x = trans.transform.translation.x

            start_y = trans.transform.translation.y

        except:

            rospy.logwarn("无法获取当前位置，使用(0,0)")

            start_x, start_y = 0, 0



        # 用 move_base make_plan 获取全局路径

        path = get_global_path_from_movebase(start_x, start_y, x, y, tolerance=0.1)

        if not path:

            rospy.logwarn("make_plan 返回空，降级为直线路径")

            path = generate_straight_path(start_x, start_y, x, y, num_points=30)



        subgoal_mgr.set_global_path(path)

        subgoal_mgr.set_final_goal(x, y)

        rospy.loginfo(f"全局路径生成完成，起点({start_x:.2f},{start_y:.2f})，终点({x:.2f},{y:.2f})")

        # ===== 子目标管理器初始化完成 =====



        # 循环追踪子目标

        start_time = rospy.Time.now()

        rate = rospy.Rate(5)

        last_subgoal = None

        nav_success = False



        while not rospy.is_shutdown():

            # 获取当前位置

            try:

                trans = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))

                rx, ry = trans.transform.translation.x, trans.transform.translation.y

            except:

                rate.sleep()

                continue



            # 获取当前子目标

            subgoal = subgoal_mgr.get_current_subgoal(rx, ry)



            # 检查是否到达终点

            if subgoal_mgr.is_final_goal_reached(rx, ry):

                rospy.loginfo("✅ 到达终点")

                self.move_base_client.cancel_all_goals()

                nav_success = True

                break



            # 检查超时

            if (rospy.Time.now() - start_time).to_sec() > timeout:

                rospy.logwarn("   navigation timed out")

                self.move_base_client.cancel_all_goals()

                break



            # ================== 新增：ABORTED 检测与路径跳过 ==================

            if last_subgoal is not None:

                state = self.move_base_client.get_state()

                if state == actionlib.GoalStatus.ABORTED:

                    rospy.logwarn(f"move_base 拒绝子目标 ({last_subgoal[0]:.2f}, {last_subgoal[1]:.2f})，尝试跳过...")

                    self.move_base_client.cancel_all_goals()

                    rospy.sleep(0.5)



                    # 从路径中移除被 ABORT 的子目标及之前的所有点

                    new_path = []

                    skipped = False

                    for p in path:

                        if not skipped and math.hypot(p[0] - last_subgoal[0], p[1] - last_subgoal[1]) < 0.2:

                            skipped = True

                            continue

                        if skipped:

                            new_path.append(p)

                    path = new_path

                    if not path:

                        rospy.logerr("所有子目标均被拒绝，导航失败")

                        return False

                    subgoal_mgr.set_global_path(path)

                    rospy.loginfo(f"跳过失败子目标，剩余路径点数: {len(path)}")



                    last_subgoal = None

                    continue

            # =================================================================



            # 如果子目标变化，发送新目标给move_base

            if last_subgoal != subgoal and subgoal is not None:

                rospy.loginfo(f"追踪子目标: ({subgoal[0]:.2f}, {subgoal[1]:.2f})")



                # 取消旧目标

                self.move_base_client.cancel_all_goals()

                rospy.sleep(0.2)  # 给cancel一点时间



                # 发送子目标

                goal = MoveBaseGoal()

                goal.target_pose.header.frame_id = "map"

                goal.target_pose.header.stamp = rospy.Time.now()

                goal.target_pose.pose.position.x = subgoal[0]

                goal.target_pose.pose.position.y = subgoal[1]

                quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

                goal.target_pose.pose.orientation.z = quat[2]

                goal.target_pose.pose.orientation.w = quat[3]



                self.move_base_client.send_goal(goal)

                last_subgoal = subgoal



            rate.sleep()



        return nav_success





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

        # 验证 LaserScan 数据有效性

        scan_msg = rospy.wait_for_message('/rflysim/sensor0/vehicle_lidar_points', LaserScan, timeout=10.0)

        valid_ranges = [r for r in scan_msg.ranges if not math.isnan(r) and scan_msg.range_min < r < scan_msg.range_max]

        if len(valid_ranges) == 0:

            rospy.logerr("❌ LaserScan 数据无效（ranges 全为 nan），请检查原始点云 /rflysim/sensor0/vehicle_lidar 是否有数据！")

            return False

        rospy.loginfo(f"✅ LaserScan 数据已就绪，有效点数: {len(valid_ranges)}")

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



        # 先等待 AMCL 发布 map->odom TF

        rospy.loginfo("等待 AMCL 发布 map->odom TF...")

        tf_timeout = rospy.Time.now() + rospy.Duration(25.0)

        map_odom_ready = False

        spin_rate = rospy.Rate(5)

        while not rospy.is_shutdown() and rospy.Time.now() < tf_timeout:

            try:

                controller.tf_buffer.lookup_transform("map", "odom", rospy.Time(0))

                rospy.loginfo("✅ map->odom TF 已就绪")

                map_odom_ready = True

                break

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):

                spin_rate.sleep()

        if not map_odom_ready:

            rospy.logwarn("⚠️ map->odom TF 等待超时，继续尝试启动 move_base...")



        # 等待 move_base action server 就绪（超时增加到 60 秒）

        rospy.loginfo("等待 move_base action server...")

        move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        if not move_client.wait_for_server(rospy.Duration(60.0)):

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



    if success:

        rospy.loginfo("🎉 UGV 导航任务完成")

        # 此处调用ue接口直接释放药箱，因机械臂控制不在赛题重点考核范围，为降低难度聚焦重点直接给出释放函数

        ue = UE4CtrlAPI.UE4CtrlAPI()

        ue.sendUE4ExtAct(2, [0, 0, 0, 0, 0, 0, 20, 1, 0, 0, 0, 0, 0, 0, 0, 0])

        rospy.loginfo("🎉 UGV 释放药箱任务完成")

    else:

        rospy.logwarn("⚠️ UGV 任务未成功完成")

    return success





# ================== 轻量返航接口（仅修正返航部分）==================

def navigate_back_to_start(target_x, target_y, timeout=90.0):

    """

    轻量返航：复用现有导航栈，带子目标切换。

    关键修正：

    1. 返航终点增加可达性预检查，解决终点在膨胀区/障碍物旁导致永久ABORTED

    2. ABORTED跳过逻辑修正：0.5m模糊匹配，失败则保守跳过前25%点，避免清空路径

    3. 增加SUCCEEDED状态监听，move_base到达即认为成功

    4. server等待30s，ABORTED先清图重试再跳过，3次上限

    """



    rospy.loginfo(f"Returning to start: ({target_x:.2f}, {target_y:.2f})")



    # ===== 新增：返航终点可达性预检查（解决终点在膨胀区问题）=====

    try:

        costmap = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout=3.0)

        map_x = int((target_x - costmap.info.origin.position.x) / costmap.info.resolution)

        map_y = int((target_y - costmap.info.origin.position.y) / costmap.info.resolution)

        if 0 <= map_x < costmap.info.width and 0 <= map_y < costmap.info.height:

            cost = costmap.data[map_y * costmap.info.width + map_x]

            if cost >= 100:

                rospy.logwarn(f"返航终点 ({target_x:.2f}, {target_y:.2f}) 在障碍物/膨胀区内(cost={cost})，搜索附近可达点...")

                found = False

                for r in [0.3, 0.5, 0.8, 1.0, 1.5, 2.0]:

                    for angle in [0, math.pi/4, math.pi/2, 3*math.pi/4, math.pi,

                                  -math.pi/4, -math.pi/2, -3*math.pi/4]:

                        tx = target_x + r * math.cos(angle)

                        ty = target_y + r * math.sin(angle)

                        mx = int((tx - costmap.info.origin.position.x) / costmap.info.resolution)

                        my = int((ty - costmap.info.origin.position.y) / costmap.info.resolution)

                        if 0 <= mx < costmap.info.width and 0 <= my < costmap.info.height:

                            c = costmap.data[my * costmap.info.width + mx]

                            if c < 100:

                                rospy.loginfo(f"找到返航替代终点: ({tx:.2f}, {ty:.2f})")

                                target_x, target_y = tx, ty

                                found = True

                                break

                    if found:

                        break

                if not found:

                    rospy.logerr("返航终点附近无可达点，返航失败")

                    return False

    except Exception as e:

        rospy.logwarn(f"返航终点预检查失败: {e}，继续尝试...")

    # ==============================================================



    # 创建move_base客户端

    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    rospy.loginfo("等待 move_base action server (返航)...")

    if not client.wait_for_server(rospy.Duration(30.0)):

        rospy.logerr("move_base server not available for return (超时30s)")

        return False



    # 只在返航开始时清一次 costmap

    try:

        rospy.wait_for_service('/move_base/clear_costmaps', rospy.Duration(2.0))

        from std_srvs.srv import Empty

        clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        clear_srv()

        rospy.loginfo("返航 costmaps cleared")

    except Exception:

        pass



    rospy.sleep(1.0)



    # 子目标管理器

    subgoal_mgr = SubGoalManager(lookahead_distance=2.0, subgoal_tolerance=0.5)

    subgoal_mgr.set_final_goal(target_x, target_y)



    # 获取当前位置

    try:

        tf_buffer = tf2_ros.Buffer()

        tf_listener = tf2_ros.TransformListener(tf_buffer)

        rospy.sleep(0.5)

        trans = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(2.0))

        start_x = trans.transform.translation.x

        start_y = trans.transform.translation.y

    except Exception as e:

        rospy.logwarn(f"无法获取当前位置: {e}，使用(0,0)")

        start_x, start_y = 0, 0



    # 获取全局路径

    path = get_global_path_from_movebase(start_x, start_y, target_x, target_y, tolerance=0.1)

    if not path:

        rospy.logwarn("返航 make_plan 返回空，降级为直线路径")

        path = generate_straight_path(start_x, start_y, target_x, target_y, num_points=30)



    subgoal_mgr.set_global_path(path)

    rospy.loginfo(f"返航路径生成完成，起点({start_x:.2f},{start_y:.2f})，终点({target_x:.2f},{target_y:.2f})，点数{len(path)}")



    # 循环追踪子目标

    start_time = rospy.Time.now()

    rate = rospy.Rate(5)

    last_subgoal = None

    nav_success = False

    abort_retry_count = 0

    max_abort_retries = 3



    while not rospy.is_shutdown():

        # 获取当前位置

        try:

            trans = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(0.5))

            rx, ry = trans.transform.translation.x, trans.transform.translation.y

        except:

            rate.sleep()

            continue



        # 获取当前子目标

        subgoal = subgoal_mgr.get_current_subgoal(rx, ry)



        # 检查是否到达终点（TF判断）

        if subgoal_mgr.is_final_goal_reached(rx, ry):

            rospy.loginfo("✅ 返航到达起点（TF判断）")

            client.cancel_all_goals()

            nav_success = True

            break



        # 【关键修正】检查move_base是否报告成功（有时TF有延迟，但move_base已到tolerance内）

        if last_subgoal is not None:

            state = client.get_state()

            if state == actionlib.GoalStatus.SUCCEEDED:

                rospy.loginfo("✅ 返航到达起点（move_base报告SUCCEEDED）")

                client.cancel_all_goals()

                nav_success = True

                break



            elif state == actionlib.GoalStatus.ABORTED:

                abort_retry_count += 1

                if abort_retry_count > max_abort_retries:

                    rospy.logerr(f"返航 ABORTED 超过最大重试次数({max_abort_retries})，返航失败")

                    client.cancel_all_goals()

                    return False

                

                # 第一次ABORTED：清除costmap后重试同一子目标

                if abort_retry_count == 1:

                    rospy.logwarn("返航 ABORTED，先清除 costmap 重试同一子目标...")

                    client.cancel_all_goals()

                    try:

                        rospy.wait_for_service('/move_base/clear_costmaps', rospy.Duration(1.0))

                        from std_srvs.srv import Empty

                        clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

                        clear_srv()

                    except Exception:

                        pass

                    rospy.sleep(0.5)

                    last_subgoal = None

                    continue

                

                # 第二次及以上：跳过失败子目标

                rospy.logwarn(f"返航子目标 ({last_subgoal[0]:.2f}, {last_subgoal[1]:.2f}) 再次被拒，尝试跳过...")

                client.cancel_all_goals()

                rospy.sleep(0.5)



                # 【关键修正】更鲁棒的跳过逻辑：0.5m模糊匹配，失败则保守跳过前25%点

                skip_idx = None

                for i, p in enumerate(path):

                    if math.hypot(p[0] - last_subgoal[0], p[1] - last_subgoal[1]) < 0.5:

                        skip_idx = i

                        break

                

                if skip_idx is not None:

                    path = path[skip_idx+1:]

                    rospy.loginfo(f"跳过前{skip_idx+1}个点，剩余{len(path)}个点")

                else:

                    # 找不到匹配点，保守跳过前25%或至少前2个点，绝不清空

                    skip_n = max(2, len(path) // 4)

                    path = path[skip_n:]

                    rospy.loginfo(f"未找到匹配点，保守跳过前{skip_n}个点，剩余{len(path)}个点")

                

                if not path:

                    rospy.logerr("跳过点后路径为空，返航失败")

                    return False

                

                subgoal_mgr.set_global_path(path)

                last_subgoal = None

                continue



        # 检查超时

        if (rospy.Time.now() - start_time).to_sec() > timeout:

            rospy.logwarn("返航超时")

            client.cancel_all_goals()

            break



        # 子目标变化阈值 0.3m，避免频繁cancel/replan

        need_update = False

        if last_subgoal is None:

            need_update = True

        elif subgoal is not None:

            sg_dist = math.hypot(subgoal[0] - last_subgoal[0], subgoal[1] - last_subgoal[1])

            if sg_dist > 0.3:

                need_update = True



        if need_update and subgoal is not None:

            rospy.loginfo(f"返航子目标: ({subgoal[0]:.2f}, {subgoal[1]:.2f})")

            client.cancel_all_goals()

            rospy.sleep(0.2)

            goal = MoveBaseGoal()

            goal.target_pose.header.frame_id = "map"

            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position.x = subgoal[0]

            goal.target_pose.pose.position.y = subgoal[1]

            quat = tf.transformations.quaternion_from_euler(0, 0, 0)

            goal.target_pose.pose.orientation.z = quat[2]

            goal.target_pose.pose.orientation.w = quat[3]

            client.send_goal(goal)

            last_subgoal = subgoal



        rate.sleep()



    return nav_success



# ================== 主函数入口 ==================

if __name__ == "__main__":

    signal.signal(signal.SIGINT, signal_handler)

    signal.signal(signal.SIGTERM, signal_handler)

    targets_json_path = "../script/targets.json"

    #targets_json_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "targets.json")

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
