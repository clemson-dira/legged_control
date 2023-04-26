#! /usr/bin/env python3

import rospy
import sym_density
from legged_body_msgs.msg import Plan
from legged_body_msgs.msg import State
from legged_body_msgs.msg import Control
from ocs2_msgs.msg import mpc_observation
import std_msgs.msg

import math
import sys

"""
    pub_body_plan_demo is a demo file for publishing generic plan to the body planner node
    Remark: This publishes to plan which legged_body_planner should subcribe to
"""


# Define global parameters
dt = 0.01
#update_rate = 1.0  # Hz
N = 5000
class PubBodyPlanDemo:
    #def __init__(self, dt, update_rate):
    def __init__(self, dt, update_rate):
        rospy.init_node("pub_body_plan_demo", anonymous=True)
        #self.rate = rospy.Rate(update_rate)
        self.observer_sub = rospy.Subscriber(
            '/legged_robot_mpc_observation', mpc_observation, self.observation_callback)
        self.body_plan_pub = rospy.Publisher('plan', Plan, queue_size=1)

    def observation_callback(self, observer_msg):
        # print("Getting observer")
        self.init_time = observer_msg.time
        self.state = observer_msg.state
        self.control = observer_msg.input

    def pub_body_plan(self):
        print("Publishing test plan")
        # Get parametesr
        curr_time = rospy.Time()
        x0 = self.state[6]
        y0 = self.state[7]
        density_plan = sym_density.density()
        t, X, u = density_plan.get_plan(curr_time,x0,y0,N,dt)
        states = []
        time = []
        for i in range(N):
            x_dot = u[0,i]
            y_dot = u[1,i]
            z_dot = 0
            roll_dot = 0
            pitch_dot = 0
            yaw_dot = 0
            x = X[0,i]
            y = X[1,i]
            z = 0
            roll = 0
            pitch = 0
            yaw = 0
            states = states +  State(value=[x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot, 
                                            x, y, z, roll, pitch, yaw])
            time = time + t[i]
        header = std_msgs.msg.Header()
        header.stamp = curr_time

        plan_msg = Plan()
        plan_msg.header.stamp = header.stamp
        plan_msg.plan_timestamp = curr_time
        plan_msg.times = time
        plan_msg.states = states
        plan_msg.controls = [Control(), Control(), Control()]
        self.body_plan_pub.publish(plan_msg)

    def spin(self):
        print("Spinning")
        while (not rospy.is_shutdown()):
            self.pub_body_plan()
            #self.rate.sleep()

def main():
    #pub_body_plan_demo = PubBodyPlanDemo(dt, update_rate)
    pub_body_plan_demo = PubBodyPlanDemo(dt, N)
    pub_body_plan_demo.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
