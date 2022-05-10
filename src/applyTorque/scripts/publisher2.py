import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState
import random
from os.path import expanduser
from pathlib import Path
from math import sin,pi
import pandas as pd

torq_type = ['constant','ramp','harmonic'][1]
name = f'model2-{torq_type}-{str(random.random())}'
home = expanduser('~')
DIR = f"{home}/pprs_ws/csv/"
Path(DIR).mkdir(parents=True, exist_ok=True)
l1 = []
l2 = []
class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/demo/link2/force_demo2"
        publish_topic2 = "/demo/link3/force_demo3"

        self.counter = 0      
        timer_period = 0.001 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.trajectory_publihser = self.create_publisher(Wrench,publish_topic, 10)
        self.trajectory_publihser2 = self.create_publisher(Wrench,publish_topic2, 10)

        self.joint_state_subscriber = self.create_subscription(JointState , "/myrobot/R1/joint_states" , self.joint_state_pub , 10)
        self.joint_state_subscriber = self.create_subscription(JointState , "/myrobot/R2/joint_states" , self.joint_state_pub2 , 10)
        self.torque = 0.0
        self.goal_torque =1.8
        self.t = 0
       
   

        
    def joint_state_pub(self , msg ): 
        l1.append(msg.position[0])
        if len(l1)>50000 and len(l2)>50000:
            list_dict = {'R1_x':l1[:50001],'R2_x':l2[:50001]} 
            df = pd.DataFrame(list_dict) 
            df.to_csv(f'{DIR}/{name}.csv', index=False) 
            rclpy.shutdown()

        

    
        
    def joint_state_pub2(self , msg ) :
        l2.append(msg.position[0])



    def timer_callback(self):

        force_message = Wrench()
        force_message2 = Wrench()

        if torq_type == 'constant':
            self.torque=-50.

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
        force_message2.torque.x = self.torque/2

        print(force_message.torque.x)

        self.trajectory_publihser.publish(force_message)
        self.trajectory_publihser2.publish(force_message2)

        
                

    

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()

    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
