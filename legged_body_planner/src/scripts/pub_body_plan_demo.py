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
    Publishes generic plan to the plan topic
    Remark: This publishes to plan which legged_body_planner should subcribe to
"""


# Define global parameters

dt = 0.1  # Original: 0.01
update_rate = 1.0  # Hz
N = 50  # Original : 100

# Density Parameters
obs_center = [10, 10]
goal = [5, 5]
alpha = 0.2
gain = 100
saturation = 1

# TODO : Graph the state & control plots & compare matlab... weird controls


class PubBodyPlanDemo:
    def __init__(self, dt, N, update_rate):
        rospy.init_node("pub_body_plan_demo", anonymous=True)
        self.rate = rospy.Rate(update_rate)
        self.observer_sub = rospy.Subscriber(
            '/legged_robot_mpc_observation', mpc_observation, self.observation_callback)
        self.body_plan_pub = rospy.Publisher('plan', Plan, queue_size=1)
        self.dt = dt
        self.N = N

    def observation_callback(self, observer_msg):
        # print("Getting observer")
        self.curr_time = observer_msg.time
        self.state = observer_msg.state
        self.control = observer_msg.input
        # print('state', self.state.value)

    def pub_body_plan(self):
        # print("Publishing test plan")
        # Get parametesr
        curr_time = self.curr_time
        x0 = self.state.value[6]
        y0 = self.state.value[7]
        density_plan = sym_density.Density(
            r1=1, r2=2, obs_center=obs_center, goal=goal, alpha=alpha, gain=gain, saturation=saturation)
        t, X, u = density_plan.get_plan(curr_time, x0, y0, self.N, self.dt)
        states = []
        time = []
        controls = []
        for i in range(N):
            x_dot = u[0, i]
            y_dot = u[1, i]
            z_dot = 0
            roll_dot = 0
            pitch_dot = 0
            yaw_dot = 0
            x = X[0, i]
            y = X[1, i]
            z = 0
            roll = 0
            pitch = 0
            yaw = 0
            # print('x: ', x, 'y: ', y)
            print('x_dot: ', x_dot, 'y_dot: ', y_dot)
            states.append(State(value=[x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot,
                                       x, y, z, roll, pitch, yaw]))
            time.append(t[0, i])
            controls.append(Control())

        # print('states',states)
        # print('time',time)
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time(curr_time)
        plan_msg = Plan()
        plan_msg.header.stamp = header.stamp
        plan_msg.plan_timestamp = rospy.Time(curr_time)
        plan_msg.times = time
        plan_msg.states = states

        # ## test code
        # states_0 = State(value=[0, 0, 0, 0, 0, 0,
        #                     0, 0, 0, 0, 0, 0])
        # states_1 = State(value=[0, 0, 0, 0, 0, 0,
        #                     0.5, 0, 0, 0, 0, 0])
        # states_2 = State(value=[0, 0, 0, 0, 0, 0,
        #                     1.0, 0, 0, 0, 0, 0])
        # plan_msg.states = [states_0, states_1, states_2]
        # plan_msg.times = [dt+0.1, 2*dt+0.1, 3*dt+0.1]
        # print('test states',plan_msg.states)
        # print('times',plan_msg.times)
        #  ## end test code

        plan_msg.controls = controls
        self.body_plan_pub.publish(plan_msg)

    def spin(self):
        print("Spinning")
        rospy.wait_for_message(
            '/legged_robot_mpc_observation', mpc_observation)
        while (not rospy.is_shutdown()):
            self.pub_body_plan()
            self.rate.sleep()


def main():
    pub_body_plan_demo = PubBodyPlanDemo(dt, N, update_rate)
    pub_body_plan_demo.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# class PubBodyPlanDemo:
#     def __init__(self, dt, update_rate):
#         rospy.init_node("pub_body_plan_demo", anonymous=True)
#         self.rate = rospy.Rate(update_rate)
#         self.observer_sub = rospy.Subscriber(
#             '/legged_robot_mpc_observation', mpc_observation, self.observation_callback)
#         self.body_plan_pub = rospy.Publisher('plan', Plan, queue_size=1)

#     def observation_callback(self, observer_msg):
#         # print("Getting observer")
#         self.curr_time = observer_msg.time
#         self.state = observer_msg.state
#         self.control = observer_msg.input

#     def pub_body_plan(self):
#         print("Publishing test plan")
#         # print(len(states_0))
#         states_0 = State(value=[0, 0, 0, 0, 0, 0,
#                                 0, 0, 0, 0, 0, 0])
#         states_1 = State(value=[0, 0, 0, 0, 0, 0,
#                                 0.5, 0, 0, 0, 0, 0])
#         states_2 = State(value=[0, 0, 0, 0, 0, 0,
#                                 1.0, 0, 0, 0, 0, 0])
#         header = std_msgs.msg.Header()
#         header.stamp = rospy.Time(self.curr_time)

#         plan_msg = Plan()
#         plan_msg.header.stamp = header.stamp
#         plan_msg.plan_timestamp = header.stamp
#         plan_msg.times = [self.curr_time,
#                           self.curr_time + 1.0, self.curr_time + 2.0]
#         plan_msg.states = [states_0, states_1, states_2]
#         plan_msg.controls = [Control(), Control(), Control()]
#         self.body_plan_pub.publish(plan_msg)

#     def spin(self):
#         rospy.wait_for_message(
#             '/legged_robot_mpc_observation', mpc_observation)
#         while (not rospy.is_shutdown()):
#             self.pub_body_plan()
#             self.rate.sleep()


# def main():
#     pub_body_plan_demo = PubBodyPlanDemo(dt, update_rate)
#     pub_body_plan_demo.spin()


# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass
