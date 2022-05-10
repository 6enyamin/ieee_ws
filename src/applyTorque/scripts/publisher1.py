

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState
import random
import pandas as pd 
from os.path import expanduser
from pathlib import Path
from math import sin,pi
import sys

torq_type = ['constant','ramp','harmonic'][int(sys.argv[1])]
name = f'model1-{torq_type}-{str(random.random())}'
home = expanduser('~')
DIR = f"{home}/pprs_ws/csv/"
Path(DIR).mkdir(parents=True, exist_ok=True)
l1 = []
class Torque_publisher(Node):
    
    def __init__(self):
    	
        super().__init__('Torque_publsiher_node')        
        publish_topic = "/demo/link/force_demo"
        timer_period = 0.001 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.trajectory_publihser = self.create_publisher(Wrench,publish_topic, 10)
        self.joint_state_subscriber = self.create_subscription(JointState , "/myrobot/joint_states" , self.joint_state_pub , 10)
        self.torque = 0.0
        self.goal_torque = float(sys.argv[2])
        self.c = -1

        self.t = 0

    #Define Subscriber Callback function , in this case we save positions data to a CSV file
    def joint_state_pub(self , msg ) : 

        l1.append(msg.position[0])
        if len(l1)>50000:
            list_dict = {'R1_x':l1} 
            df = pd.DataFrame(list_dict) 
            df.to_csv(f'{DIR}/{name}.csv', index=False) 
            rclpy.shutdown()
                    


    #Define timer callback function
    def timer_callback(self):
    
        force_message = Wrench()
        if torq_type == 'constant':
            self.torque=-self.goal_torque

        elif torq_type == 'ramp':
            if self.torque > self.goal_torque:
                self.c = -1
            elif self.torque < self.goal_torque:
                self.c = 1
            self.torque+=(self.c*0.001)

        elif torq_type == 'harmonic':
            self.torque = -50.*sin(pi*self.t/10)
            self.t+=0.001

        force_message.torque.x = self.torque

        print(force_message.torque.x)

        self.trajectory_publihser.publish(force_message)


def main(args=None):
    rclpy.init(args=args)
    Torque_publisher_object = Torque_publisher()

    rclpy.spin(Torque_publisher_object)
    Torque_publisher_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
