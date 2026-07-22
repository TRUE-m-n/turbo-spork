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
import rospy
import cv2
import base64
from std_msgs.msg import String
from uav import uav_recon
from geometry_msgs.msg import PoseStamped
from PIL import Image
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge

# 添加多模态大模型相关导入
from transformers import AutoModel, AutoTokenizer
import torch
import re
import atexit

# 添加日志配置
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


# ================== 全局单例模型管理器 ==================
class MultimodalModelManager:
    """多模态大模型单例管理器"""
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self._initialized = True
        self.model = None
        self.tokenizer = None
        self.model_path = './FM9G4B-V'
        self._load_model()

    def _load_model(self):
        """加载模型（只执行一次）"""
        if not os.path.exists(self.model_path):
            logger.warning(f"模型文件不存在: {self.model_path}，将使用模拟模式")
            self.model = None
            self.tokenizer = None
            return

        try:
            logger.info("正在加载多模态大模型（全局单例）...")
            self.model = AutoModel.from_pretrained(
                self.model_path,
                trust_remote_code=True,
                attn_implementation='sdpa',
                torch_dtype=torch.bfloat16
            )
            self.model = self.model.eval().cuda()
            self.tokenizer = AutoTokenizer.from_pretrained(self.model_path, trust_remote_code=True)
            logger.info("✅ 多模态大模型加载完成（全局唯一实例）")
        except Exception as e:
            logger.error(f"多模态大模型加载失败: {e}")
            self.model = None
            self.tokenizer = None

    def is_available(self) -> bool:
        """检查模型是否可用"""
        return self.model is not None and self.tokenizer is not None

    def get_model(self):
        """获取模型实例"""
        return self.model

    def get_tokenizer(self):
        """获取tokenizer实例"""
        return self.tokenizer

    def cleanup(self):
        """清理模型资源"""
        if self.model is not None:
            logger.info("清理模型资源...")
            del self.model
            self.model = None
        if self.tokenizer is not None:
            del self.tokenizer
            self.tokenizer = None
        if torch.cuda.is_available():
            torch.cuda.empty_cache()


# 创建全局模型管理器实例
model_manager = MultimodalModelManager()


# 注册全局清理函数
def global_cleanup():
    """程序退出时的全局清理"""
    logger.info("程序退出，清理全局资源...")
    model_manager.cleanup()


atexit.register(global_cleanup)


class TeclUavDevice(AbstractDevice):
    def __init__(self):
        super().__init__()
        rospy.init_node('tecl_uav_node', anonymous=True)
        self.task_lock = threading.Lock()
        self.is_busy = False
        self.targets_json_path = "./targets.json"
        self.detected_json_path = "./detected_targets.json"
        self.uav_home_position = None
        self.bridge = CvBridge()
        self.latest_img = None
        self.vision_result = None

        # 使用全局模型管理器（不再重复加载）
        self.model_manager = model_manager
        logger.info(f"✅ TeclUavDevice 初始化完成 (模型可用: {self.model_manager.is_available()})")

        rospy.Subscriber('/rflysim/sensor3/img_rgb', RosImage, self._img_callback)
        rospy.Subscriber('/vision_result', String, self._vision_result_callback)
        self._init_uav_home_position()
        rospy.loginfo("Mission Executor Node Started.")

    def _cleanup(self):
        """清理实例资源（不清理模型，因为模型是全局共享的）"""
        logger.info("清理 TeclUavDevice 实例资源...")
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Program exit")
        # 注意：不清理模型，因为其他实例可能还在使用

    @property
    def protocol_name(self) -> str:
        return "tecl-uav"

    def _get_device_key(self, device_id: int, connection_str: str) -> str:
        return f"{device_id}:{connection_str}"

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
                name="无人机同步结果给无人车",
                command_type="report_results",
                description="无人机报告结果并同步地形给无人车",
                params={}
            ),
            ActionItem(
                name="无人机确认物品是否送达",
                command_type="thing_detect",
                description="无人机检测药品是否正确送达",
                params={}
            ),
            ActionItem(
                name="无人机前往冲突区域侦察",
                command_type="uav_reconnaissance_mission",
                description="无人机前往冲突区域侦察",
                params={
                    "type": "object",
                    "properties": {
                        "waypoints": {
                            "type": "array",
                            "description": "侦察航点列表（可选，默认使用预设航点）",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "x": {"type": "number"},
                                    "y": {"type": "number"},
                                    "z": {"type": "number"}
                                }
                            }
                        },
                        "enable_yolo": {
                            "type": "boolean",
                            "description": "是否启用YOLO目标检测",
                            "default": False
                        }
                    },
                    "required": []
                }
            ),
            ActionItem(
                name="全员返航",
                command_type="back_home",
                description="所有无人设备返回营区",
                params={}
            ),
        ]

    def execute(self, client, device_id, connection_str, command_type: str, params: dict) -> dict:
        """执行设备命令 - 分发到对应的任务方法"""
        logger.info(f"执行命令: {command_type}, 参数: {params}")

        # 检查是否忙碌
        if self.is_busy:
            self.send_text_message(device_id, f"系统忙碌中，无法执行 {command_type}", MessageLevel.WARNING)
            return {'status': 'busy', 'message': '系统忙碌中'}

        # 直接执行，不创建新线程
        self.is_busy = True
        try:
            if command_type == "uav_reconnaissance_mission":
                self._handle_uav_recon(device_id, params)
            elif command_type == "report_results":
                self._handle_result_sync(device_id)
            elif command_type == "thing_detect":
                self._handle_thing_detect(device_id)
            elif command_type == "back_home":
                self._handle_back_home(device_id)
            else:
                logger.warning(f"未知指令: {command_type}")
                self.send_text_message(device_id, f"未知指令: {command_type}", MessageLevel.WARNING)
        except Exception as e:
            logger.error(f"任务执行异常: {e}")
            self.send_text_message(device_id, f"任务执行失败: {str(e)}", MessageLevel.WARNING)
        finally:
            self.is_busy = False

        return {'status': 'executed', 'message': f'任务 {command_type} 已执行完成'}

    # ================== 任务实现 ==================

    def _handle_uav_recon(self, device_id: int, params: dict):
        """执行无人机侦察任务"""
        self.send_text_message(device_id, "🚀 [Step 1] 执行侦察任务...", MessageLevel.INFO)
        waypoints = params.get('waypoints', [])
        # 获取航点，支持自定义或使用默认值
        WAYPOINTS = [
            [0, 0, 1.7], [4.1, 0.2, 1.7], [4.1, 8.0, 1.7],
            [2.0, 8.0, 1.7], [-0.4, 6.0, 1.7], [2.0, 4.0, 1.7], [2.0, 6.0, 1.7]
        ]
        if waypoints:
            WAYPOINTS = waypoints
        try:
            success = uav_recon.run_reconnaissance_task(
                waypoints=WAYPOINTS,
                enable_yolo=False,
                yolo_model_path="yolov8n.pt",
                camera_topic="/rflysim/sensor2/img_rgb",
                detection_json_path="./detected_targets.json"
            )
            logger.info(f"success:{success}")
            if success:
                logger.info("success")
                self.send_text_message(device_id, "侦察完成，已生成点云地图。", MessageLevel.INFO, role="assistant")
            else:
                logger.info("error")
                self.send_text_message(device_id, "侦察任务失败。", MessageLevel.WARNING, role="assistant")
        except Exception as e:
            logger.error(f"_handle_uav_recon执行异常: {e}", exc_info=True)
            self.send_text_message(device_id, f"侦察任务异常: {str(e)}", MessageLevel.WARNING)

    def _handle_result_sync(self, device_id: int):
        """同步结果与生成地图"""
        self.send_text_message(device_id, "🔄 [Step 2] 同步结果与生成地图...", MessageLevel.INFO)

        try:
            # 执行地图生成
            success = uav_recon.convert_pcd_to_2d_map()

            if not success:
                self.send_text_message(device_id, "地图生成失败。", MessageLevel.WARNING, role="assistant")
                return

            # 解读 YOLO 检测结果
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

            self.send_text_message(device_id, summary, MessageLevel.INFO, role="assistant")

        except Exception as e:
            self.send_text_message(device_id, f"同步失败: {str(e)}", MessageLevel.WARNING)

    def _handle_thing_detect(self, device_id: int):
        """检测物资送达情况"""
        self.send_text_message(device_id, "🔍 [Step 4] 检测物资送达情况...", MessageLevel.INFO)
        try:
            if not os.path.exists(self.targets_json_path):
                self.send_text_message(device_id, "未找到targets.json，无法核实送达情况。", MessageLevel.WARNING,
                                       role="assistant")
                return
            with open(self.targets_json_path, 'r') as f:
                data = json.load(f)

            person_pose = data.get('person_uav') or data.get('person')
            if not person_pose:
                self.send_text_message(device_id, f"无法获取救援人员位置",
                                       MessageLevel.INFO)
                return

            target_x = person_pose['x']
            target_y = person_pose['y']
            observe_height = 3.0
            self.send_text_message(device_id, f"无人机正飞往人员位置上方核实...",
                                   MessageLevel.INFO)
            success = uav_recon.fly_to_observe_position(
                target_x=target_x,
                target_y=target_y,
                observe_height=observe_height,
                hover_time=3.0
            )

            if not success:
                self.send_text_message(device_id, "无人机无法飞抵目标位置进行核实。",
                                       MessageLevel.WARNING, role="assistant")
                return

            # 用多模态大模型分析当前画面（使用全局模型管理器）
            query = (
                "这是一张无人机下视摄像头拍摄的救援场景画面。"
                "请仔细观察并描述图片中的内容（地形、物体、人员等），"
                "然后判断救援物资（医疗箱）是否在受伤人员附近。"
                "如确认物资已到位请回答'已送达'，否则回答'未送达'。"
            )
            result = self._query_vision(query)

            # 保存到日志方便人工核对
            try:
                with open("/tmp/vision_debug.log", "w") as f:
                    f.write(f"Query: {query}\n\nResult: {result}\n")
            except Exception:
                pass

            if "已送达" in result and "未送达" not in result:
                self.send_text_message(device_id, f"任务完成：救援物资已准确送达人员附近。多模态识别结论：{result}",
                                       MessageLevel.INFO, role="assistant")
            elif "未送达" in result:
                self.send_text_message(device_id, f"任务未完成：救援物资未送达指定位置。多模态识别结论：{result}",
                                       MessageLevel.INFO, role="assistant")
            else:
                self.send_text_message(device_id, f"视觉检测结果：{result}",
                                       MessageLevel.INFO, role="assistant")

        except Exception as e:
            self.send_text_message(device_id, f"检测任务异常: {str(e)}", MessageLevel.WARNING)

    def _query_vision(self, query: str, timeout: float = 30.0) -> str:
        """
        使用全局多模态大模型分析当前画面
        """
        # 等待获取最新图像
        waited = 0.0
        while self.latest_img is None and waited < 5.0:
            rospy.sleep(0.2)
            waited += 0.2

        if self.latest_img is None:
            logger.warning("没有可用的相机图像")
            return "no_image"

        # 保存调试图像
        debug_path = "/tmp/mission_vision_debug.jpg"
        os.makedirs(os.path.dirname(debug_path), exist_ok=True)
        cv2.imwrite(debug_path, self.latest_img)

        h, w = self.latest_img.shape[:2]
        logger.info(f"相机图像: {w}x{h}, 平均亮度={self.latest_img.mean():.1f}, 保存至 {debug_path}")

        # 检查模型是否可用（使用全局管理器）
        if not self.model_manager.is_available():
            logger.warning("多模态大模型未加载，使用模拟结果")
            return "模型未加载，无法判断"

        try:
            # OpenCV BGR 转 RGB，然后转 PIL Image
            img_rgb = cv2.cvtColor(self.latest_img, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(img_rgb)

            # 构建提示词
            full_prompt = f"{query}\n请用简短的中文回答，不超过50字。"

            # 调用多模态模型
            msgs = [{'role': 'user', 'content': [pil_image, full_prompt]}]

            logger.info("正在调用多模态大模型分析图像...")
            t0 = time.time()

            result = self.model_manager.get_model().chat(
                image=None,  # 图像已在 msgs 中
                msgs=msgs,
                tokenizer=self.model_manager.get_tokenizer(),
                sampling=True
            )

            elapsed = time.time() - t0
            logger.info(f"多模态分析完成 ({elapsed:.1f}s): {result[:200]}")

            return result

        except Exception as e:
            logger.error(f"多模态分析失败: {e}")
            return f"分析失败: {str(e)}"

    def _handle_back_home(self, device_id: int):
        """全员返航"""
        self.send_text_message(device_id, "无人机正在返航...", MessageLevel.INFO)
        uav_result = [None]
        try:
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

            t_uav = threading.Thread(target=uav_return)
            t_uav.start()
            t_uav.join(timeout=300)
            uav_ok = uav_result[0] if uav_result[0] is not None else False
            summary = "无人机已返回并降落" if uav_ok else "无人机返航失败"
            self.send_text_message(device_id, summary, MessageLevel.INFO, role="assistant")
        except Exception as e:
            self.send_text_message(device_id, f"返航任务异常: {str(e)}", MessageLevel.WARNING)


async def main():
    # 注意：模型已经在程序启动时自动加载（通过模块级单例）
    logger.info("程序启动，检查模型状态...")
    if model_manager.is_available():
        logger.info("✅ 模型已就绪")
    else:
        logger.warning("⚠️ 模型未就绪，将使用模拟模式")

    try:
        async with DevicePusher(lambda: TeclUavDevice()) as pusher:
            await pusher.connect_server(
                "127.0.0.1:50058",
                "无人机类型"
            )
            # 保持运行直到收到中断
            while True:
                await asyncio.sleep(1)
                if rospy.is_shutdown():
                    break
    except (KeyboardInterrupt, asyncio.CancelledError):
        logger.info("程序被中断")
    finally:
        logger.info("程序退出")
        # 全局清理会在 atexit 中自动执行


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("通过 Ctrl+C 退出")
    except Exception as e:
        logger.error(f"程序异常: {e}")
    finally:
        # 确保资源清理
        global_cleanup()