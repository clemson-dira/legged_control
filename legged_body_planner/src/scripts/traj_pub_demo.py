#! /usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from legged_body_msgs.msg import State
from legged_body_msgs.msg import Control
from legged_body_msgs.msg import Plan
from ocs2_msgs.msg import mpc_observation
import std_msgs.msg

import math
import sys
import numpy as np


"""
    cmd_vel_pub is a demo file to publish velocity commands to the legged_control repository. 
    Note, that cmd_vel atm uses a fix TIME_TO_TARGET, therefore, it is more of a position command rather than velocity command
"""

# Define global parameters
rospy.init_node('traj_pub_demo', anonymous=True)
# Parameters
period = 20  # Desired period [s]
A = 0.5  # Amplitude
B = 2*math.pi/period
rate_mult = 0.01  # Publish rate a factor of the period

# rate = rospy.Rate(1/(rate_mult*period))  # Hz
rate = rospy.Rate(0.1)
dt = rate_mult*period  # Discrete time step


def cmd_vel_talker():
    print("Running cmd_vel mode (or position mode w/ varying vel)")
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    msg = Twist()
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        curr_time = rospy.Time.now()
        msg.linear.x = 0.0
        msg.linear.y = A*math.sin(B*(curr_time.to_sec() - start_time.to_sec()))
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        pub.publish(msg)
        rate.sleep()


def target_pose_talker():

    print("Running target position mode w/ fixed vel")
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    msg = PoseStamped()

    # Parameters
    target_displacement_vel = 0.5  # TODO (AZ): Read from .info files
    global rate, rate_mult, period
    rate_mult = 0.01
    rate = rospy.Rate(1/(rate_mult*period))
    # rate = rospy.Rate(1)
    # TODO (AZ) : ATM Target Reaching Time is NaN

    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        msg.header.frame_id = "odom"
        curr_time = rospy.Time.now()
        msg.header.stamp = curr_time
        # msg.pose.position.x = A * \
        #     math.cos(B*(curr_time.to_sec()-start_time.to_sec()))
        msg.pose.position.x = 0
        # Move like sine, derivative of sine
        msg.pose.position.y = A * \
            math.sin(B*(curr_time.to_sec()-start_time.to_sec()))
        # msg.pose.position.y = 0.0
        msg.pose.position.z = 0.325  # Go1 spec | Doesn't matter
        # RPY 0 0 0 -> Quaternion 1 0 0 0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        pub.publish(msg)
        rate.sleep()

class OfflinePlan:
    def __init__(self, nom_vel, nom_ang_vel, goal, rate):
        self.nom_vel = nom_vel
        self.nom_ang_vel = nom_ang_vel
        self.goal = goal
        self.rate = rospy.Rate(rate)
        self.observer_sub = rospy.Subscriber('/legged_robot_mpc_observation', mpc_observation, self.observationCallback)
        self.body_plan_pub = rospy.Publisher('plan', Plan, queue_size=1)
    
    def observationCallback(self, observer_msg):
        """
        Calls to the observer to get observed time, states, and control
        Note - time : double, state : list (curr state @ least xyz_dot, ypr_dot, xyz, ypr, default_joint_states)
                control : list (GRFs I believe)
        Inputs:
        -------
        observer_msg : mpc_observation that gives current observed time, state, and input
        """
        self.curr_time = observer_msg.time
        self.state = observer_msg.state
        self.control = observer_msg.input
    
    def estimateTimeToTarget(self, pos1, pos2):
        """
        Given position 1 and position 2, and nominal velocity, find the time taken to go towards
        target
        Inputs:
        -------
        pos1 : list
            Initial position (xyz, ypr)
        pos2 : list
            Target position (xyz, ypr)
        nom_vel : float
            Nominal linear velocity
        nom_ang_vel : float
            Nominal angular velocity
        
        Return:
        --------
        time : double/float
            Estimated time to reach target
        """
        assert np.size(pos1) >= 4, "Position 1 states must at least be size 4"
        assert np.size(pos2) >= 4, "Position 2 state must at least be size 4"
        difference = np.array(pos2[0:4]) - np.array(pos1[0:4])
        dx = difference[0]
        dy = difference[1]
        dyaw = difference[3]
        distance = np.sqrt(dx*dx + dy * dy)
        rotation_time = np.abs(dyaw)/self.nom_ang_vel
        displacement_time = distance/self.nom_vel
        return np.max([rotation_time, displacement_time])

    def pubBodyPlan(self):
        # Get into Plan Msg format
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time(self.curr_time)
        plan_msg = Plan()
        plan_msg.header.stamp = header.stamp
        plan_msg.plan_timestamp = header.stamp
        time_to_target = self.estimateTimeToTarget(self.state.value, self.goal)
        
        plan_msg.times = [self.curr_time, self.curr_time + time_to_target]
        plan_msg.states = [State(value=self.state.value), State(value=self.goal)]
        plan_msg.controls = [Control(), Control()]
        self.body_plan_pub.publish(plan_msg)
    
    def spin(self):
        print("Spinning pub body plan demo")
        rospy.wait_for_message('/legged_robot_mpc_observation', mpc_observation)
        while (not rospy.is_shutdown()):
            rospy.loginfo_throttle(5, "Publishing fixed offline plan")
            self.pubBodyPlan()
            self.rate.sleep()

def bodyPlan():
    print("Running offline body plan")
    goal = [-5, 0, 0.31, -3.1415] # x, y, z, yaw
    nom_vel = 0.1
    nom_ang_vel = 0.314
    rate = 50
    offline_plan = OfflinePlan(nom_vel=nom_vel, nom_ang_vel=nom_ang_vel, goal=goal, rate=rate)
    offline_plan.spin()




def main():
    if len(sys.argv) <= 1:
        print(
            "Please rerun and enter w/ type of publishing mode: 'cmd_vel' OR 'target_pose' OR 'body_plan'")
        return
    type_pub = sys.argv[1]
    if type_pub == "cmd_vel":
        cmd_vel_talker()
    elif type_pub == "target_pose":
        target_pose_talker()
    elif type_pub =="body_plan":
        bodyPlan()
    else:
        print("Non-valid publishing mode. Please choose either cmd_vel or target_pose")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
