#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import time
import mujoco
import mujoco.viewer
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose2D
from diagnostic_msgs.msg import KeyValue

pos = {
    "l_sh_p": 0,
    "l_sh_r": 0,
    "l_sh_y": 0,
    "l_el_p": 0,
    "l_wr_y": 0,
    "l_wr_p": 0,
    "l_wr_r": 0,
}

joint = [
    "l_sh_p",
    "l_sh_r",
    "l_sh_y",
    "l_el_p",
    "l_wr_y",
    "l_wr_p",
    "l_wr_r",
]

filtered_data = [[],[],[],[],[],[],[],[],[],[],[],[]]
data_array = [[0]*200 for _ in range(12)]

def KalmanAverageFilter(input_value, data_array):
    length = len(data_array)
    result = input_value / length
    for i in range(length - 1):
        result += data_array[i] / length
    for i in range(length - 1, 0, -1):
        data_array[i] = data_array[i - 1]
    data_array[0] = input_value
    return result

class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')

        self.paused = False
        self.visual_foot_flag = False
        self.joint_state_flag = False
        self.ft_flag = False
        self.imu_flag = False
        self.odom_flag = False
        self.foot_print_state_flag = False
        self.hard_balance_flag = False

        self.filtered_data = [[],[],[],[],[],[],[],[],[],[],[],[]]
        self.data_array = [[0]*200 for _ in range(12)]

        self.SubscribeToJoints()
        self.pub_jointstate = self.create_publisher(JointState,"/robot/joint_states", 10)
        self.clock_pub = self.create_publisher(Clock,"/clock", 10)
        self.time_factor_pub = self.create_publisher(Float32,"/time_factor", 10)


    def SubscribeToJoints(self):
        self.create_subscription(JointState,"/joint_states", self.JointStatesCallback, 10)



    def mujoco(self):
        current_script_path = os.path.abspath(__file__)
        root = os.path.dirname(os.path.dirname(os.path.dirname(current_script_path))) + "/share/robot_sim/mjcf/arm/scene.xml"
        m = mujoco.MjModel.from_xml_path(root)
        d = mujoco.MjData(m)
        with mujoco.viewer.launch_passive(m, d, key_callback=self.key_callback) as viewer:
            start_time = time.time()
            zero_time = d.time
            while 1:
                step_start = time.time()

                d.ctrl[joint.index("l_sh_p")] = pos["l_sh_p"]
                d.ctrl[joint.index("l_sh_r")] = pos["l_sh_r"]
                d.ctrl[joint.index("l_sh_y")] = pos["l_sh_y"]
                d.ctrl[joint.index("l_el_p")] = pos["l_el_p"]
                d.ctrl[joint.index("l_wr_y")] = pos["l_wr_y"]
                d.ctrl[joint.index("l_wr_p")] = pos["l_wr_p"]
                d.ctrl[joint.index("l_wr_r")] = pos["l_wr_r"]

                if not self.paused:
                    if self.visual_foot_flag and self.foot_print_state_flag:
                        self.VisualFoot(viewer,m,d)
                    if self.foot_print_state_flag == False:
                        viewer.user_scn.ngeom = 0
                    mujoco.mj_step(m, d)
                    viewer.sync()


                self.JointPublishSensors(m,d)
                time_until_next_step = m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

                clock_msg = Clock()
                time_sec = d.time - zero_time
                clock_msg.clock.sec = int(time_sec)
                clock_msg.clock.nanosec = int((time_sec - int(time_sec)) * 1e9)
                self.clock_pub.publish(clock_msg)

                elapsed_real_time = time.time() - start_time
                time_factor = d.time / elapsed_real_time if elapsed_real_time != 0 else 0
                time_factor_msg = Float32()
                time_factor_msg.data = time_factor
                self.time_factor_pub.publish(time_factor_msg)
                rclpy.spin_once(self, timeout_sec=0.0)

    def JointPublishSensors(self, m, d):
        joint_msg = JointState()
        joint_msg.name = joint
        joint_msg.position = [d.joint(i).qpos[0] for i in joint]
        joint_msg.velocity = [d.joint(i).qvel[0] for i in joint]
        joint_msg.effort = [d.joint(i).qfrc_smooth[0] for i in joint]
        self.pub_jointstate.publish(joint_msg)

    def JointStatesCallback(self,msg):
        for i, name in enumerate(msg.name):
            if name in pos:
                pos[name] = msg.position[i]



    def key_callback(self,keycode):

        if chr(keycode) == ' ':
          if self.paused == True:
              self.paused = False
          else :
              self.paused = True



def main(args=None):
    rclpy.init(args=args)
    mujoco_node = MujocoNode()

    try:
        mujoco_node.mujoco()  # Mujoco 시뮬레이션 시작
    except KeyboardInterrupt:
        pass
    finally:
        mujoco_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
