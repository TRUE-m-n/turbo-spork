import math
import UE4CtrlAPI as UE4CtrlAPI
import VisionCaptureApi

import time
import math
import ReqCopterSim
import sys


ue = UE4CtrlAPI.UE4CtrlAPI()


if __name__ == "__main__":

    # 创建无人机起飞区
    ue.sendUE4PosScale(11, 100000821, 0, [-4.3, -1.8, 0], [0, 0, 0], [0.75, 0.75, 0.75])
    # 创建无人车起步区
    ue.sendUE4PosScale(
        12, 100000822, 0, [-4.5, 1.8, 0], [0, 0, math.radians(180)], [0.75, 0.75, 0.75]
    )
    # 创建弹药箱
    ue.sendUE4Pos(13, 883, 0, [3, 1.5, 0], [0, 0, 0])
    # 创建军事掩体
    ue.sendUE4Pos(14, 882, 0, [0.4, -1.5, 0], [0, 0, 0])
    ue.sendUE4Pos(15, 882, 0, [1, 1.6, 0], [0, 0, 0])
    # 创建坦克
    ue.sendUE4PosScale(16, 421, 0, [0.4, 0, 0.88], [0, 0, 160], [0.2, 0.2, 0.2])

    # 创建受伤人员（xml中ModelType==1时，传入旋转的角度需要转换成弧度）
    roll_p = math.radians(220)
    pitch_p = math.radians(90)
    yaw_p = math.radians(0)
    ue.sendUE4PosScale(
        17, 116000030, 0, [3, -0.5, -0.1], [roll_p, pitch_p, yaw_p], [0.8, 0.8, 0.8]
    )

    #生成医疗箱
    ue.sendUE4Pos(20,969,0,[0.356,0.012,-0.25],[0,0,0])
    time.sleep(1)

    #医疗箱附加到小车上
    ue.sendUE4Attatch(20,2,3) 
    
    time.sleep(1)

    #小车放下医疗箱#参数第7位为放下的物品的copterID，第8位为1时放下
    #ue.sendUE4ExtAct(2,[0,0,0,0,0,0,20,1,0,0,0,0,0,0,0,0])

# import required libraries
# pip3 install pymavlink pyserial

# 启用ROS发布模式
# VisionCaptureApi.isEnableRosTrans = True

req = ReqCopterSim.ReqCopterSim()  # 获取局域网内所有CopterSim程序的电脑IP列表
StartCopterID = 1  # 初始飞机的ID号
TargetIP = req.getSimIpID(
    StartCopterID
)  # 获取CopterSim的1号程序所在电脑的IP，作为目标IP
# 注意：如果是本电脑运行的话，那TargetIP是127.0.0.1的本机地址；如果是远程访问，则是192打头的局域网地址。
# 因此本程序能同时在本机运行，也能在其他电脑运行。

# print("Request CopterSim Send data.")
# req.sendReSimIP(StartCopterID)  # 请求回传数据到本电脑


vis = VisionCaptureApi.VisionCaptureApi(TargetIP)

# VisionCaptureApi 中的配置函数
vis.jsonLoad() # 加载Config.json中的传感器配置文件
vis.jsonLoad(1)  # 加载Config.json中的传感器配置文件

# isSuss = vis.sendReqToUE4(0, TargetIP)
# vis.startImgCap()  # 开启取图循环，执行本语句之后，已经可以通过vis.Img[i]读取到图片了
# print("Start Image Reciver")
# vis.sendImuReqCopterSim(
#     StartCopterID, TargetIP
# )  # 发送请求，从目标飞机CopterSim读取IMU数据,回传地址为127.0.0.1，默认频率为200Hz
# # 执行本语句之后，会自动开启数据监听，已经可以通过vis.imu读取到IMU数据了。


# ros.EndRosLoop()
