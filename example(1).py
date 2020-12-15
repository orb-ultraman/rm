#####################################################
# 说明：在“可以修改此处代码”位置书写你的代码
#
# 尽量不要修改其他地方的代码
#####################################################

from PySide2.QtWidgets import QApplication

from renderer import startRenderer
from simulator import Simulator
from teleop import TeleOp
from models import parameters
from trajplayer import TrajectoryPlayer

import math
import json

################可以修改此处代码########################
# 模式，遥控或者轨迹
MODE = "trajectory"
#MODE = "teleop"

# 在这里输入轨迹文件路径，其中
# trajectories/line.json 为向前走直线
# trajectories/two_lines.json 直走，旋转，向左平移
# trajectories/rm_letter.json 为画 RM 字母
# trajectories/rectangle.json 为画矩形
# trajectories/random_walk.json 为四处走动

TRAJECTORY_FILE = "trajectories/random_walk.json"
##################结束#################################

class Controller:
    def __init__(self, teleop):
        super().__init__()

        self.teleop = teleop
        ################可以修改此处代码########################
        # 在这里初始化程序变量

        self.odom_x = 0
        self.odom_y = 0
        self.odom_yaw = 0

        self.last_enc = [0 for _ in range(4)]
        ##################结束#################################
        pass

    def step(self, dt, sensor_data, set_control_input, submit_prediction):
        #Get encoder data
        encoder = [sensor_data["wheels"][i]["encoder"] for i in range(4)]
        gyro = sensor_data["gyro"]
        acc = sensor_data["acc"]

        ################可以修改此处代码########################
        # 这个函数是控制周期函数，每0.01s会执行一次，你需要在这里书写姿态估计程序
        # 其中 encoder[0..3] 数组为4个编码器读数，单位为rad（注意存在一定误差！）
        # 调用 submit_prediction([x, y]) 提交你估计的机器人中心位置坐标(x, y) 单位为m

        # 这里是简单的累计编码器程序示例，只能计算小车的前进距离，不能转弯平移

        #Get encoder deltas
        w=[0, 0, 0, 0]
        for i in range(4):
            if(encoder[i] - self.last_enc[i] > 3.14):
                w[i] = encoder[i] - 2* 3.1415926 - self.last_enc[i]
            elif(encoder[i] - self.last_enc[i] < -3.14):
                w[i] = encoder[i] + 2* 3.1415926 - self.last_enc[i]
            else:
                w[i] = encoder[i] - self.last_enc[i]

        # w = [encoder[i] - self.last_enc[i] for i in range(4)]
        self.last_enc = encoder
        a = 0.982720177856 * w[0] - 0.004721699557
        b = 0.987097472379 * w[1] + 0.001214358195
        c = 0.986161404158 * w[2] - 0.000162229578
        d = 0.987112437501 * w[3] - 0.003998837108
        w[0]=a
        w[1]=b
        w[2]=c
        w[3]=d
        acx = 0.989898786 * acc[0] - 0.031905093
        acy = 0.989898786 * acc[1] - 0.027840381
        acw = 0.985481385 * gyro[2] + 0.29774680

        #Accumulate
        #self.odom_x += (w[0] + w[1] + w[2] + w[3]) * parameters.WHEEL_RADIUS / 4
        dx = (w[0] + w[1] + w[2] + w[3]) * parameters.WHEEL_RADIUS / 4
        dy = (-w[0] + w[1] + w[2] - w[3]) * parameters.WHEEL_RADIUS / 4
        self.odom_yaw += (-w[0] + w[1] - w[2] + w[3]) / (parameters.CAR_A + parameters.CAR_B) * parameters.WHEEL_RADIUS / 4
        self.odom_x += dx * math.cos(self.odom_yaw) - dy * math.sin(self.odom_yaw)
        self.odom_y += dx * math.sin(self.odom_yaw) + dy * math.cos(self.odom_yaw)
        
        #Submit
        submit_prediction([self.odom_x, self.odom_y])

        ##################结束#################################
        # 第一项任务和第二项任务无需修改以下代码
        # 仅在第三项任务中需要修改下面的代码

        #get teleop data
        wheel_torque = self.teleop.getControlInput()

        #Output control signals
        set_control_input({
            "wheels": [{"torque": wheel_torque[i]} for i in range(4)]
        })

        ##################结束#################################

app = QApplication([])
player = TrajectoryPlayer(TRAJECTORY_FILE) if MODE == "trajectory" else TeleOp()
controller = Controller(player)
simulator = Simulator(controller)
startRenderer(app, simulator, size = (1280, 720)) #可传入size参数调整GUI的大小
