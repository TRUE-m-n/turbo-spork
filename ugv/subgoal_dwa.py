#!/usr/bin/env python3

# -*- coding: utf-8 -*-

"""

局部子目标切换模块 - 借鉴MPPO的局部子目标思想

解决DWA直接追远端目标导致的NO PATH问题

"""



import math

import rospy





class SubGoalManager:

    """

    管理全局路径上的子目标点

    DWA每次只追最近的子目标，而不是终点

    """

    

    def __init__(self, lookahead_distance=2.0, subgoal_tolerance=0.5):

        self.lookahead_distance = lookahead_distance  # 前视距离

        self.subgoal_tolerance = subgoal_tolerance    # 子目标到达容忍度

        self.global_path = []  # [(x1,y1), (x2,y2), ...]

        self.current_subgoal_idx = 0

        self.final_goal = None

    

    def set_global_path(self, path_points):

        """设置全局路径"""

        self.global_path = path_points

        self.current_subgoal_idx = 0

        rospy.loginfo(f"全局路径设置完成，共{len(path_points)}个点")

    

    def set_final_goal(self, x, y):

        """设置最终目标"""

        self.final_goal = (x, y)

    

    def get_current_subgoal(self, robot_x, robot_y):

        """

        获取当前应该追踪的子目标

        策略：找路径上距离机器人lookahead_distance远的点

        """

        if not self.global_path:

            return self.final_goal

        

        # 找到路径上距离机器人最近的点

        min_dist = float('inf')

        nearest_idx = 0

        for i, (px, py) in enumerate(self.global_path):

            d = math.hypot(px - robot_x, py - robot_y)

            if d < min_dist:

                min_dist = d

                nearest_idx = i

        

        # 向前看 lookahead_distance，找子目标

        target_idx = nearest_idx

        accumulated_dist = 0

        for i in range(nearest_idx, len(self.global_path) - 1):

            dx = self.global_path[i+1][0] - self.global_path[i][0]

            dy = self.global_path[i+1][1] - self.global_path[i][1]

            accumulated_dist += math.hypot(dx, dy)

            target_idx = i + 1

            if accumulated_dist >= self.lookahead_distance:

                break

        

        # 如果已经接近终点，直接追终点

        if target_idx >= len(self.global_path) - 1:

            return self.final_goal

        

        subgoal = self.global_path[target_idx]

        

        # 调试信息

        rospy.loginfo_throttle(2, f"子目标[{target_idx}]: ({subgoal[0]:.2f}, {subgoal[1]:.2f}), "

                                  f"终点: ({self.final_goal[0]:.2f}, {self.final_goal[1]:.2f})")

        

        return subgoal

    

    def is_final_goal_reached(self, robot_x, robot_y):

        """检查是否到达最终目标"""

        if self.final_goal is None:

            return True

        dist = math.hypot(self.final_goal[0] - robot_x, self.final_goal[1] - robot_y)

        return dist < self.subgoal_tolerance





def generate_straight_path(start_x, start_y, goal_x, goal_y, num_points=30):

    """

    生成直线路径（当没有全局规划器时）

    """

    path = []

    for i in range(num_points + 1):

        t = i / num_points

        x = start_x + t * (goal_x - start_x)

        y = start_y + t * (goal_y - start_y)

        path.append((x, y))

    return path
