#! /usr/bin/env python3

import rospy
import sym_density
from legged_body_msgs.msg import Plan
from legged_body_msgs.msg import State
from legged_body_msgs.msg import Control
from ocs2_msgs.msg import mpc_observation
import std_msgs.msg

import numpy as np

import math
import sys

"""
    pub_body_plan_demo is a demo file for publishing generic plan to the body planner node
    Publishes generic plan to the plan topic
    Remark: This publishes to plan which legged_body_planner should subcribe to
"""

"""
TODO list
1) Enable multiple obstacles
2) Filtering
3) Finite diff for yaw rate
"""


# # Define global parameters

dt = 0.01  # Original: 0.01
update_rate = 10  # Hz
horizon = 200  # Original : 100

# # Density Parameters
obs_center = [5, 0.1]
goal = [7, 0]
alpha = 0.2
gain = 25
saturation = 0.5
rad_from_goal = 0.25


class PubBodyPlanDemo:
    def __init__(self):
        rospy.init_node("pub_body_plan_demo", anonymous=True)
        # Get parameter | TODO Add helper function to warn when param not received
        self.goal = rospy.get_param("/density_plan/goal")
        self.obs_center = rospy.get_param("/density_plan/obs_center")
        self.alpha = rospy.get_param("/density_plan/alpha")
        self.gain = rospy.get_param("/density_plan/gain")
        self.saturation = rospy.get_param("/density_plan/saturation")
        self.rad_from_goal = rospy.get_param("/density_plan/rad_from_goal")
        self.update_rate = rospy.get_param("/density_plan/update_rate")
        self.horizon = rospy.get_param("/density_plan/horizon")
        self.dt = rospy.get_param("/density_plan/dt")

        self.rate = rospy.Rate(self.update_rate)
        self.observer_sub = rospy.Subscriber(
            '/legged_robot_mpc_observation', mpc_observation, self.observation_callback)
        self.body_plan_pub = rospy.Publisher('plan', Plan, queue_size=1)

    def observation_callback(self, observer_msg):
        # print("Getting observer")
        self.curr_time = observer_msg.time
        self.state = observer_msg.state
        self.control = observer_msg.input
        # print('state', self.state.value)

    def pub_body_plan(self):
        # Declare variables
        x_dot_list, y_dot_list, z_dot_list, yaw_dot_list, pitch_dot_list, roll_dot_list, x_list, y_list, z_list, yaw_list, pitch_list, roll_list = ([
        ] for i in range(12))

        # print("Publishing test plan")
        # Get parametesr
        x0 = self.state.value[6]
        y0 = self.state.value[7]
        density_plan = sym_density.Density(
            r1=1, r2=2, obs_center=self.obs_center, goal=self.goal, alpha=self.alpha,
            gain=self.gain, saturation=self.saturation, rad_from_goal=self.rad_from_goal)
        t, X, u = density_plan.get_plan(
            self.curr_time, x0, y0, self.horizon, self.dt)
        states = []
        time = []
        controls = []

        # Forward integrate states
        for i in range(self.horizon):
            x_dot_list.append(u[0, i])
            y_dot_list.append(u[1, i])
            z_dot_list.append(0)
            roll_dot_list.append(0)
            pitch_dot_list.append(0)
            # yaw_dot_list.append(0)
            x_list.append(X[0, i])
            y_list.append(X[1, i])
            z_list.append(0)
            roll_list.append(0)
            pitch_list.append(0)

            # Lift to full states
            if i == 0:
                yaw_list.append(self.getYaw(
                    [x_dot_list[-1], y_dot_list[-1]]))
            else:
                yaw_list.append(self.getYaw(
                    [x_dot_list[-1], y_dot_list[-1]], yaw_list[-1]))
            # TODO : Get yaw rate through RK4 or central diff method, or naiive finite diff

        # Apply filter
        yaw_filter_list = self.applyFirstOrderFilter(yaw_list, 0.98)

        # Get yaw rate from filter
        yaw_dot_list = self.centralDiffMethod(yaw_filter_list, self.dt)
        yaw_dot_filter_list = self.applyFirstOrderFilter(yaw_dot_list, 0.1)

        # Add time, state, and control into legged boddy msg format
        for i in range(self.horizon):
            states.append(State(value=[x_dot_list[i], y_dot_list[i], z_dot_list[i], roll_dot_list[i], pitch_dot_list[i], yaw_dot_filter_list[i],
                                       x_list[i], y_list[i], z_list[i], yaw_filter_list[i], pitch_list[i], roll_list[i]]))
            time.append(t[0, i])
            controls.append(Control())

        # print('states',states)
        # print('time',time)
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time(self.curr_time)
        plan_msg = Plan()
        plan_msg.header.stamp = header.stamp
        plan_msg.plan_timestamp = rospy.Time(self.curr_time)
        plan_msg.times = time
        plan_msg.states = states

        """
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
        """

        plan_msg.controls = controls
        self.body_plan_pub.publish(plan_msg)

    def spin(self):
        print("Spinning pub body plan demo")
        rospy.wait_for_message(
            '/legged_robot_mpc_observation', mpc_observation)
        while (not rospy.is_shutdown()):
            rospy.loginfo_throttle(1, "Republishing feedback planner")
            self.pub_body_plan()
            self.rate.sleep()

    # Utility function
    # Utility functions w/in class

    def getYaw(self, curr_vel, prev_yaw=None):
        """
        Calculates heading angle given tangent (vel) vectors

        Inputs:
        -------
        curr_vel : Current velocity (x_dot, y_dot)
        prev_yaw : Previous yaw angle

        Output:
        --------
        yaw : float
            Current yaw angle
        """
        curr_vel_x = curr_vel[0]
        curr_vel_y = curr_vel[1]
        if prev_yaw == None:
            # print("yaw ref: ", math.atan2(curr_vel_y, curr_vel_x))
            return math.atan2(curr_vel_y, curr_vel_x)  # Assumes 2d
        else:
            wrapped_yaw = math.atan2(curr_vel_y, curr_vel_x)
            # Assumes yaw rate is 'slow'
            # print("Diff: ", wrapped_yaw - prev_yaw)

            if (wrapped_yaw - prev_yaw) > math.pi:
                quotient = int((wrapped_yaw - prev_yaw)/(2*math.pi))
                wrapped_yaw = wrapped_yaw - math.pi - quotient*(2*math.pi)
            elif (wrapped_yaw - prev_yaw) < -math.pi:
                quotient = int((wrapped_yaw - prev_yaw)/(2*math.pi))
                wrapped_yaw = wrapped_yaw + math.pi - quotient*(2*math.pi)
        # print("yaw ref: ", math.atan2(curr_vel_y, curr_vel_x))
        return wrapped_yaw

    def applyFirstOrderFilter(self, states, alpha):
        """
        Apply discrete first order filter T(s) = w_c/(s+w_c)

        Inputs:
        -------
        states : list
            States to be filtered
        alpha : float
            Filter constant
        Outputs:
        --------
        filtered_states : list
            Filtered state
        """

        N = len(states)
        for i in range(1, N):
            weight = math.pow(alpha, i)
            states[i] = weight*states[0] + (1-weight)*states[i]

        return states

    def centralDiffMethod(self, state, dt):
        """
        Apply central difference to get time derivate of state

        Inputs:
        -------
        state : list
            A trajectory of this state
        Outputs:
        --------
        state_dot : list
            Trajectory of time derivative of the state
        """
        N = len(state)
        state_dot = []

        for i in range(N):
            lower_idx_bound = max(i-1, 0)
            upper_idx_bound = min(i+1, N-1)
            derivative = (state[upper_idx_bound] -
                          state[lower_idx_bound])/(2*dt)
            state_dot.append(derivative)

        return state_dot


def main():
    pub_body_plan_demo = PubBodyPlanDemo()
    pub_body_plan_demo.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
