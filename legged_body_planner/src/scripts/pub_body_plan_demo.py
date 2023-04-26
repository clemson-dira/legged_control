#! /usr/bin/env python3

import rospy
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
dt = 1.0
update_rate = 20  # Hz


class PubBodyPlanDemo:
    def __init__(self, dt, update_rate):
        rospy.init_node("pub_body_plan_demo", anonymous=True)
        self.rate = rospy.Rate(update_rate)
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
        # states_0 = self.state
        # states_0 = state_0[0:12]
        # print(len(states_0))
        states_0 = State(value=[0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0])
        states_1 = State(value=[0, 0, 0, 0, 0, 0,
                                0.5, 0, 0, 0, 0, 0])
        states_2 = State(value=[0, 0, 0, 0, 0, 0,
                                1.0, 0, 0, 0, 0, 0])
        header = std_msgs.msg.Header()
        header.stamp = curr_time

        plan_msg = Plan()
        plan_msg.header.stamp = header.stamp
        plan_msg.plan_timestamp = curr_time
        plan_msg.times = [dt+0.1, 2*dt+0.1, 3*dt+0.1]
        plan_msg.states = [states_0, states_1, states_2]
        plan_msg.controls = [Control(), Control(), Control()]
        self.body_plan_pub.publish(plan_msg)

    def spin(self):
        print("Spinning")
        while (not rospy.is_shutdown()):
            self.pub_body_plan()
            self.rate.sleep()


def main():
    pub_body_plan_demo = PubBodyPlanDemo(dt, update_rate)
    pub_body_plan_demo.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
