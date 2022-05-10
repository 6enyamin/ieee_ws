import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
import random
tag = str(random.random())


class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/demo/Hip1/force_demo"
        publish_topic2 = "/demo/Hip2/force_demo2"
        publish_topic3 = "/demo/Hip3/force_demo3"
        publish_topic4 = "/demo/R41/force_demo4"
        timer_period = 0.001 
        self.counter = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.trajectory_publihser = self.create_publisher(Wrench,publish_topic, 10)
        self.trajectory_publihser2 = self.create_publisher(Wrench,publish_topic2, 10)
        self.trajectory_publihser3 = self.create_publisher(Wrench,publish_topic3, 10)
        self.trajectory_publihser4 = self.create_publisher(Wrench,publish_topic4, 10) 
        
        self.torque = 0.0
        self.c=-1



    def timer_callback(self):

        force_message = Wrench()
        force_message2 = Wrench()
        force_message3 = Wrench()
        force_message4 = Wrench()
        
        
        if self.torque<-150:
            self.c=1
        if self.torque>90:
            self.c=-1
        self.torque = self.torque + (0.1*self.c)
        
        force_message.torque.x = self.torque
        force_message2.torque.x = self.torque
        force_message3.torque.x = self.torque
        force_message4.torque.x = -100.
        print(self.torque)

        self.trajectory_publihser.publish(force_message)
        self.trajectory_publihser2.publish(force_message2)
        self.trajectory_publihser3.publish(force_message3)
        self.trajectory_publihser4.publish(force_message4)
    
    

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()

    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
