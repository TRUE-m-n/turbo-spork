import math
import UE4CtrlAPI as UE4CtrlAPI
import VisionCaptureApi
import time
import math
import ReqCopterSim
import sys
import random
import json
import os

# ============================================================
# 场景物体随机布局，防止选手硬编码轨迹
# ============================================================

def check_overlap(p1, p2, min_dist):
    """检查两个2D点是否距离过近"""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1]) < min_dist

def generate_layout():
    """
    在基准位置附近随机生成物体布局（人员位置固定不变）。
    约束：
      1. 物体不重叠 (最小间距1.0m)
      2. 人员在无人车可通行区域 (不靠近墙壁/障碍物)
      3. 变动范围±0.8m以内，保证公平
    """
    rng = random.Random()

    # ---- 基准位置 ----
    base_tank    = (0.4, -0.3)
    base_bunker1 = (0.4, -1.5)
    base_bunker2 = (1.0, 1.6)
    base_ammo    = (3.0, 1.5)
    # 人员位置固定不变
    person = (3.0, -0.5)

    max_attempts = 100

    for _ in range(max_attempts):
        tank = (base_tank[0] + rng.uniform(-0.3, 0.3),
                base_tank[1] + rng.uniform(-0.2, 0.2))
        bunker1 = (base_bunker1[0] + rng.uniform(-0.3, 0.3),
                   base_bunker1[1] + rng.uniform(-0.3, 0.3))
        bunker2 = (base_bunker2[0] + rng.uniform(-0.6, 0.6),
                   base_bunker2[1] + rng.uniform(-0.6, 0.6))
        ammo = (base_ammo[0] + rng.uniform(-0.6, 0.6),
                base_ammo[1] + rng.uniform(-0.6, 0.6))

        # 碰撞检测 (最小间距1.0m)
        objects = [tank, bunker1, bunker2, ammo, person]
        ok = True
        for i in range(len(objects)):
            for j in range(i+1, len(objects)):
                if check_overlap(objects[i], objects[j], 1.1):
                    ok = False
                    break
            if not ok:
                break

        if ok:
            return {
                "tank": tank, "bunker1": bunker1, "bunker2": bunker2,
                "ammo": ammo, "person": person,
            }

    # 回退
    print("⚠️ 随机布局生成失败，使用基准位置")
    return {
        "tank": base_tank, "bunker1": base_bunker1, "bunker2": base_bunker2,
        "ammo": base_ammo, "person": person,
    }


if __name__ == "__main__":
    layout = generate_layout()

    # 保存布局供后续校验
    layout_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scene_layout.json")
    try:
        with open(layout_path, 'w') as f:
            json.dump(layout, f, indent=2)
        print(f"Scene layout saved: {layout_path}")
    except Exception as e:
        print(f"Layout save warning: {e}")

    print(f"  Tank:    ({layout['tank'][0]:.2f}, {layout['tank'][1]:.2f})")
    print(f"  Bunker1: ({layout['bunker1'][0]:.2f}, {layout['bunker1'][1]:.2f})")
    print(f"  Bunker2: ({layout['bunker2'][0]:.2f}, {layout['bunker2'][1]:.2f})")
    print(f"  Ammo:    ({layout['ammo'][0]:.2f}, {layout['ammo'][1]:.2f})")
    print(f"  Person:  ({layout['person'][0]:.2f}, {layout['person'][1]:.2f}) (fixed)")

    ue = UE4CtrlAPI.UE4CtrlAPI()

    # ---- 固定物体 (不随机) ----
    # 无人机起飞区
    ue.sendUE4PosScale(11, 100000821, 0, [-4.5, -1.75, 0], [0, 0, 0], [0.75, 0.75, 0.75])
    # 无人车起步区
    ue.sendUE4PosScale(12, 100000822, 0, [-4.5, 1.9, 0],
                       [0, 0, math.radians(180)], [0.75, 0.75, 0.75])

    # ---- 随机物体 ----
    # 弹药箱
    ue.sendUE4Pos(13, 883, 0, [layout['ammo'][0], layout['ammo'][1], 0], [0, 0, 0])

    # 军事掩体 ×2
    ue.sendUE4Pos(14, 882, 0, [layout['bunker1'][0], layout['bunker1'][1], 0], [0, 0, 0])
    ue.sendUE4Pos(15, 882, 0, [layout['bunker2'][0], layout['bunker2'][1], 0], [0, 0, 0])

    # 坦克
    ue.sendUE4PosScale(16, 421, 0,
                       [layout['tank'][0], layout['tank'][1], 0.88],
                       [0, 0, 160], [0.2, 0.2, 0.2])

    # 受伤人员
    roll_p = math.radians(220)
    pitch_p = math.radians(90)
    yaw_p = math.radians(0)
    ue.sendUE4PosScale(17, 116000030, 0,
                       [layout['person'][0], layout['person'][1], -0.1],
                       [roll_p, pitch_p, yaw_p], [0.8, 0.8, 0.8])

    # 医疗箱 (附加到小车上，位置相对固定)
    ue.sendUE4Pos(20, 969, 0, [0.356, 0.012, -0.25], [0, 0, 0])
    time.sleep(1)
    ue.sendUE4Attatch(20, 2, 3)
    time.sleep(1)

    # ---- 视觉传感器配置 (保持不变) ----
    req = ReqCopterSim.ReqCopterSim()
    StartCopterID = 1
    TargetIP = req.getSimIpID(StartCopterID)

    vis = VisionCaptureApi.VisionCaptureApi(TargetIP)
    vis.jsonLoad()
    vis.jsonLoad(1)
