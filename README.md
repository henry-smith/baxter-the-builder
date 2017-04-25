# baxter_the_builder
## Startup
Startup requires these commands in baxter shell (possibly in separate terminals):
rosrun baxter_tools enable_robot.py -e

roslaunch baxter_the_builder baxter_left_hand_track.launch 
(file contains launching ar_marker tracker + block pose publisher)

rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800

rosrun baxter_interface joint_trajectory_action_server.py
roslaunch baxter_moveit_config baxter_grippers.launch

rosrun baxter_the_builder baxter_builds.py

## Todo:
Misc. and of varying degrees of importance:
-Figure out better/prettier way of attaching Ar-Tags/attaching blocks
-Figure out what the hell to do with left arm
-Write report/presentation/website

Grasping/Regrasping:
-converting the pose of the block to a gripper pose that move it will always succeed to find
-finding a moveit command/velocity input/control to move a block into any orientation/most orientations on the table

Assembly:
-Brute force movement of blocks onto one another
-Recording and replaying trajectory to start on LfD portion
-Research into methods of implementing recurrent neural nets