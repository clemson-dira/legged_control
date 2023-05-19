#!/usr/bin/env python

import roslaunch
import time

print("Executing waypoint plan...")
time.sleep(1)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

print("Starting Section 1")
launch_args = ['legged_body_planner', 'plan.launch', 'section:=1']
launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
launch.start()
time.sleep(25)
launch.shutdown()

print("Start 2nd waypoint...")
launch_args = ['legged_body_planner', 'plan.launch', 'section:=2']
launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
launch2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
launch2.start()
time.sleep(110)
launch2.shutdown()