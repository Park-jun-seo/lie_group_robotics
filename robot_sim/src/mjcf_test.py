#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import sys
import time
import mujoco
import mujoco.viewer
import numpy as np
import math

class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')

        self.paused = False
        # self.paused = True

            

    def mujoco(self):
        current_script_path = os.path.abspath(__file__)
        root = "src/lie_group_robotics/alice_mujoco/mjcf/arm/scene.xml"
        m = mujoco.MjModel.from_xml_path(root)
        d = mujoco.MjData(m)
        with mujoco.viewer.launch_passive(m, d, key_callback=self.key_callback) as viewer:
            start_time = time.time()
            zero_time = d.time
            while 1:
                step_start = time.time()

                if not self.paused:
                    mujoco.mj_step(m, d)
                    viewer.sync()


                time_until_next_step = m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                
                # elapsed_real_time = time.time() - start_time
                # time_factor = d.time / elapsed_real_time if elapsed_real_time != 0 else 0
                # print("time_factor : ",time_factor)


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