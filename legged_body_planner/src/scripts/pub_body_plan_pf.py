#! /usr/bin/env python

import rospy
from legged_body_msgs.msg import Plan
from legged_body_msgs.msg import State
from legged_body_msgs.msg import Control
from ocs2_msgs.msg import mpc_observation
import std_msgs.msg

import numpy as np

import math
import sys

"""
    pub_body_plan_pf loads an offline pf plan as a csv file for publishing generic plan to the body planner node
    Publishes generic plan to the plan topic
    Remark: This publishes to plan which legged_body_planner should subcribe to
"""

class PubBodyPlanDemo:
    def __init__(self):
        rospy.init_node("pub_body_plan_demo", anonymous=True)
        # Get parameter | TODO Add helper function to warn when param not received
        self.dt = rospy.get_param("/density_plan/dt")
        self.window_size = rospy.get_param("/density_plan/window_size")
        self.alpha1 = rospy.get_param("/density_plan/alpha1")
        self.alpha2 = rospy.get_param("/density_plan/alpha2")
        self.update_rate = rospy.get_param("/density_plan/update_rate")
        self.horizon = rospy.get_param("/density_plan/horizon")

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
        ] for i in range(12)) # Nice way to initialize list

        # print("Publishing test plan")
        # Get parametesr
        x0 = self.state.value[6]
        y0 = self.state.value[7]
        
        #### Read pf plan from text file #########
        X = np.zeros((2, self.horizon))
        u = np.zeros((2, self.horizon))
        t = np.zeros((1, self.horizon))
        X[0,0] = self.state.value[6]
        X[1,0] = self.state.value[7]
        u[0,0] = self.state.value[0]
        u[1,0] = self.state.value[1]
        t[0,0] = self.curr_time
        line_idx=1
        with open("/home/dira/legged_robot_ws/src/legged_control/legged_body_planner/src/scripts/pf_section1.txt", "r") as filestream:
                for line in filestream:
                    if(line_idx>1): #skip the first state to avoid jerks
                        current_line=line.split(",")
                        t[0,line_idx] = np.add(float(current_line[0]),self.curr_time)
                        X[0,line_idx] = np.subtract(float(current_line[1]), 0) # TODO : Add as a param into yaml
                        X[1,line_idx] = np.subtract(float(current_line[2]), 0) # TODO : Same above
                        u[0,line_idx] = float(current_line[3])
                        u[1,line_idx] = float(current_line[4])
                    line_idx=line_idx+1
                    if(line_idx==self.horizon):
                        break  
        filestream.close()
        #### End #########   
                                 
        states = []
        time = []
        controls = []
        # Appending states
        for i in range(0,self.horizon,10):
            x_dot_list.append(u[0, i])
            y_dot_list.append(u[1, i])
            z_dot_list.append(0)
            roll_dot_list.append(0)
            pitch_dot_list.append(0)
            x_list.append(X[0, i])
            y_list.append(X[1, i])
            z_list.append(0)
            roll_list.append(0)
            pitch_list.append(0)
            yaw_list.append(0)
            yaw_dot_list.append(0)

            ## add if else condition to test in simulation vs hardware
            # Lift to full states
            # if i == 0:
            #     yaw_list.append(self.getYaw(
            #         [x_dot_list[-1], y_dot_list[-1]]))
            # else:
            #     yaw_list.append(self.getYaw(
            #         [x_dot_list[-1], y_dot_list[-1]], yaw_list[-1]))
                       
        #Add time, state, and control into legged boddy msg format
        for i in range(len(x_dot_list)):
            states.append(State(value=[x_dot_list[i], y_dot_list[i], z_dot_list[i], roll_dot_list[i],
                                       pitch_dot_list[i], yaw_dot_list[i],
                                       x_list[i], y_list[i], z_list[i],
                                       yaw_list[i], pitch_list[i], roll_list[i]]))
            time.append(t[0, i])
            controls.append(Control())
        
        # Apply filter (not necessary for pf planner)
        # filtered_x_list = self.movingAverageFilter(x_list, self.window_size)
        # filtered_y_list = self.movingAverageFilter(y_list, self.window_size)
        # filtered_x_dot_list = self.movingAverageFilter(
        #     x_dot_list, self.window_size)
        # filtered_y_dot_list = self.movingAverageFilter(
        #     y_dot_list, self.window_size)
        # filtered_yaw_list = self.applyFirstOrderFilter(yaw_list, self.alpha1)

        # # Get yaw rate from filter
        # yaw_dot_list = self.centralDiffMethod(filtered_yaw_list, self.dt)
        # filtered_yaw_dot_list = self.applyFirstOrderFilter(
        #     yaw_dot_list, self.alpha1)

        # Add time, state, and control into legged boddy msg format
        # for i in range(self.horizon):
        #     states.append(State(value=[filtered_x_dot_list[i], filtered_y_dot_list[i], z_dot_list[i], roll_dot_list[i],
        #                                pitch_dot_list[i], filtered_yaw_dot_list[i],
        #                                filtered_x_list[i], filtered_y_list[i], z_list[i],
        #                                filtered_yaw_list[i], pitch_list[i], roll_list[i]]))
        #     time.append(t[0, i])
        #     controls.append(Control())

        # print('x:',x_list)
        # print('x_dot:',x_dot_list)
            
        #print('states',states)
        #print('time',time)
        
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time(self.curr_time)
        plan_msg = Plan()
        plan_msg.header.stamp = header.stamp
        plan_msg.plan_timestamp = rospy.Time(self.curr_time)
        plan_msg.controls = controls
        plan_msg.times = time
        plan_msg.states = states
        
        ''''
        ## test code
        states_0 = State(value=[0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0])
        states_1 = State(value=[0, 0, 0, 0, 0, 0,
                            0.2, 0, 0, 0, 0, 0])
        states_2 = State(value=[0, 0, 0, 0, 0, 0,
                            0.4, 0, 0, 0, 0, 0])
        dt=0.01
        plan_msg.states = [states_0, states_1, states_2]
        plan_msg.times = [self.curr_time, 1*dt+0.1+self.curr_time, 2*dt+0.1+self.curr_time]
        controls = [Control(),Control(),Control()] 
        plan_msg.controls = controls
        #print('test states',plan_msg.states)
        #print('times',plan_msg.times)
         ## end test code
        '''      
        
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

    def movingAverageFilter(self, data, window_size):
        """
        Applies a moving average filter to the list 'state'

        Inputs:
        --------
        state : list
            Data to be filtered
        window_size : int
            Window size of moving average filter
        Outputs:
        --------
        filtered_data : list
            Filtered data
        """
        filtered_data = [0.0]*len(data)  # Initialize size

        # Ensure window size is odd
        if (window_size % 2 == 0):
            window_size = window_size + 1  # Add 1 to window size

        # Symmetric moving average window
        for i in range(len(data)):  # i: current index

            # Check if window out of range
            lower_idx_bound = max(0, i - (int)(window_size/2))
            upper_idx_bound = min(len(data) - 1, i + (int)(window_size/2))
            # Shrink window if the moving window becomes out of bound
            new_window_size = min(window_size, abs(lower_idx_bound - i)*2 + 1,
                                  abs(upper_idx_bound - i)*2 + 1)

            # Average data in window
            avg_i = 0  # Average at index i
            lower_window_idx = i - (int)(new_window_size/2)
            upper_window_idx = i + (int)(new_window_size/2) + 1
            for j in range(lower_window_idx, upper_window_idx):
                avg_i += data[j]
            avg_i = avg_i/new_window_size
            # print("Filtered data i: ", avg_i)
            filtered_data[i] = avg_i
        return filtered_data


def main():
    pub_body_plan_demo = PubBodyPlanDemo()
    pub_body_plan_demo.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
