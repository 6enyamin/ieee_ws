

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState
import csv 
import sys
import math

#Define a Class that Inherits from Ros Node Class 
class Torque_publisher(Node):
    #Define initializer of class 
    def __init__(self):
    	#Call Parent class(Node) initializer  , and pass the name of the node we want to create
        super().__init__('Torque_publsiher_node')
        #Define a variable for save Topic name , in this case we want to be same with ros_force_plugin 
        publish_topic = "/demo/link/force_demo"
        #Define Timer period of Timer function 
        timer_period = 0.001
        #Create Timer 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #Create a Publisher with the same Topic of ros_force Plugin 
        self.trajectory_publihser = self.create_publisher(Wrench,publish_topic, 10)
        #Create a Subscriber for Subscribe to Joint_State_Publisher
        self.joint_state_subscriber = self.create_subscription(JointState , "/myrobot/joint_states" , self.joint_state_pub , 10)
        #Torque Variables
        self.torque = 0.0
        self.goal_torque =30.0
        self.current_pos = 0.0
        self.last_pos = float(sys.argv[1])/180.0*math.pi
        self.errore_ = 0.0 
        self.errore_last= 0.0 
        #self.k_p = 48
        #self.k_d = 10.0
        #self.k_i = 77
        
        self.k_p = 300.0
        self.k_d = 300.0
        self.k_i = 40.0
        self.e_integ = 0.0
        self.counter = 0 

    #Define Subscriber Callback function , in this case we save positions data to a CSV file
    def joint_state_pub(self , msg ) : 
        #print("joint State is : ")
        #print(msg.position[0])
        
        
        self.current_pos = msg.position[0]
        #self.last_pos = self.current_pos 
        self.errore_last = self.errore_ 
        self.errore_ = self.last_pos - self.current_pos

        print("end of position")



    #Define timer callback function
    def timer_callback(self):
        force_message = Wrench()
        self.counter = self.counter+1 
        e_dot = (self.errore_ - self.errore_last  ) / 0.001
        self.e_integ = self.e_integ + (((self.errore_last + self.errore_ )/2)*0.001)
        
        if self.counter >= 1000 :
        	f = (self.k_p * self.errore_last) + (self.k_d * e_dot) + (self.k_i * self.e_integ)
        else:
            f=0.0 
        force_message.torque.x = f
        self.trajectory_publihser.publish(force_message)
        print(force_message.torque.x)


def main(args=None):
    rclpy.init(args=args)
    Torque_publisher_object = Torque_publisher()

    rclpy.spin(Torque_publisher_object)
    Torque_publisher_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
