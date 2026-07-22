#!/usr/bin/env python3
from device_protocol_sdk.abstract_device import AbstractDevice
from device_protocol_sdk.abstract_device import ActionItem
from device_protocol_sdk.model.device_status import DeviceStatus, MessageLevel
from device_protocol_sdk.pusher import DevicePusher

import asyncio
import logging
import json
import threading
import time
from typing import Dict, Any, Optional
import os

# 导入原有的功能模块
from ugv import ugv_navigation
import rospy

# 添加日志配置
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class TeclCarDevice(AbstractDevice):
    def __init__(self):
        super().__init__()
        # 任务执行状态
        self.is_busy = False
        rospy.init_node('tecl_car_node', anonymous=True)
        self.task_lock = threading.Lock()

        # 配置文件路径
        self.targets_json_path = "./targets.json"
        self.detected_json_path = "./detected_targets.json"

        logger.info("✅ TeclCarDevice 初始化完成")

    @property
    def protocol_name(self) -> str:
        return "tecl-car"

    def _get_device_key(self, device_id: int, connection_str: str) -> str:
        return f"{device_id}:{connection_str}"

    def _create_client(self, device_id: int, connection_str: str) -> tuple[bool, Any]:
        try:
            return True, {'su': 1}
        except Exception as e:
            logger.error(f"设备 {device_id} 创建客户端失败: {str(e)}")
            return False, ""

    def _close_client(self, client, device_id: int, connection_str: str) -> bool:
        try:
            logger.info(f"设备 {device_id} 已关闭连接")
            return True
        except Exception as e:
            logger.error(f"设备 {device_id} 关闭连接时出错: {str(e)}")
            return False

    def get_device_status(self, client, device_id: str, connection_str: str) -> DeviceStatus:
        return DeviceStatus(
            is_lock=True,
            heartbeat=False,
            alt=0.0,
            battery=100,
            lat=22.7740215,
            lon=113.9632934,
            airspeed=0.0,
            groundspeed=0.0,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            height=0.0,
            x=1,
            y=1,
            z=1
        )

    def get_action_list(self):
        """定义设备支持的操作能力 - 对应 MissionExecutor 中的任务"""
        return [
            ActionItem(
                name="无人车运输药品至救援目标处",
                command_type="car_navigation",
                description="无人车运送药品至指定地点",
                params={
                    "type": "object",
                    "properties": {},
                    "required": []
                }
            ),
            ActionItem(
                name="全员返航",
                command_type="back_home",
                description="所有无人设备返回营区",
                params={
                    "type": "object",
                    "properties": {},
                    "required": []
                }
            ),
        ]

    def execute(self, client, device_id, connection_str, command_type: str, params: dict) -> dict:
        """执行设备命令 - 直接执行，不创建新线程"""
        logger.info(f"执行命令: {command_type}, 参数: {params}")

        # 检查是否忙碌
        if self.is_busy:
            self.send_text_message(device_id, f"系统忙碌中，无法执行 {command_type}", MessageLevel.WARNING)
            return {'status': 'busy', 'message': '系统忙碌中'}

        # 直接执行（不创建线程）
        self.is_busy = True
        try:
            if command_type == "car_navigation":
                self._handle_car_navigation(device_id)
            elif command_type == "back_home":
                self._handle_back_home(device_id)
            else:
                logger.warning(f"未知指令: {command_type}")
                self.send_text_message(device_id, f"未知指令: {command_type}", MessageLevel.WARNING)
                return {'status': 'error', 'message': f'未知指令: {command_type}'}
        except Exception as e:
            logger.error(f"任务执行异常: {e}")
            self.send_text_message(device_id, f"任务执行失败: {str(e)}", MessageLevel.WARNING)
            return {'status': 'error', 'message': str(e)}
        finally:
            self.is_busy = False

        return {'status': 'success', 'message': f'任务 {command_type} 执行完成'}
    # ================== 任务实现 ==================

    def _handle_car_navigation(self, device_id: int):
        """无人车物资运输"""
        try:
            # 读取坐标文件
            if not os.path.exists(self.targets_json_path):
                self.send_text_message(device_id, "未找到坐标文件 targets.json", MessageLevel.WARNING,role="assistant")
                return

            with open(self.targets_json_path, 'r') as f:
                data = json.load(f)

            robot_pose = data.get('robot')
            person_pose = data.get('person')

            if not robot_pose or not person_pose:
                self.send_text_message(device_id, "坐标文件中缺少车或人的位置信息", MessageLevel.WARNING,role="assistant")
                return

            # 计算目标点（在人的 X 轴负方向偏移 1.5 米）
            offset_x = -0.5
            target_x = person_pose['x'] + offset_x
            target_y = person_pose['y'] - offset_x

            self.send_text_message(device_id,
                                   f"目标点(人): ({person_pose['x']}, {person_pose['y']}), 导航终点(偏移后): ({target_x:.2f}, {target_y:.2f})",
                                   MessageLevel.INFO)

            # 调用无人车任务模块
            success = ugv_navigation.run_ugv_mission(
                target_x=target_x,
                target_y=target_y,
                start_x=robot_pose['x'],
                start_y=robot_pose['y']
            )

            if success:
                # 执行虚拟卸载
                self._unload_goods(device_id)
                self.send_text_message(device_id, "物资已成功送达目标点。", MessageLevel.INFO,role="assistant")
            else:
                self.send_text_message(device_id, "无人车导航失败。", MessageLevel.WARNING,role="assistant")

        except Exception as e:
            self.send_text_message(device_id, f"导航任务异常: {str(e)}", MessageLevel.WARNING)

    def _unload_goods(self, device_id: int):
        """虚拟卸载物资"""
        self.send_text_message(device_id, "📦 正在卸载物资...", MessageLevel.INFO)
        time.sleep(2.0)
        self.send_text_message(device_id, "✅ 物资卸载完成", MessageLevel.INFO)


    def _handle_back_home(self, device_id: int):
        """全员返航"""
        self.send_text_message(device_id, f"无人车正在执行返航...", MessageLevel.INFO)
        ugv_result = [None]
        try:
            # 无人车返航
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

            t_ugv = threading.Thread(target=ugv_return)
            t_ugv.start()
            t_ugv.join(timeout=300)
            ugv_ok = ugv_result[0] if ugv_result[0] is not None else False
            summary = "无人车已返回营区出发点" if ugv_ok else "无人车返航失败"
            self.send_text_message(device_id, summary, MessageLevel.INFO, role="assistant")
        except Exception as e:
            self.send_text_message(device_id, f"返航任务异常: {str(e)}", MessageLevel.WARNING)


async def main():
    async with DevicePusher(lambda: TeclCarDevice()) as pusher:
        await pusher.connect_server(
            "127.0.0.1:50058",
            "无人车类型"
        )


if __name__ == "__main__":
    asyncio.run(main())