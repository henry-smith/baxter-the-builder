# baxter_the_builder
## Startup
Startup requires these commands in baxter shell (./baxter.sh) (possibly in separate terminals):

rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools tuck_arms.py -u
rosrun baxter_tools camera_control.py -c right_hand_camera
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
roslaunch baxter_the_builder left_btb_begin.launch 
(file contains launching ar_marker tracker + block pose publisher + endpoint recorder)

rosrun baxter_the_builder regrasper.py
(just runs whatever test stuff is in main)

## Current abilities:
-Can turn, and flip (somewhat) robustly if order is convinient
-Can easily publish pose of bricks with any desired amount of ar tags
-Can stack bricks pretty well using just position control
-Easy to use Grasping API

## Todo:
Grasping/Regrasping:
-Figure out how to avoid grabbing at top and bottom (where knubs get in way)
-Sometimes flip goes wrong direction, possibly need to find good middle pose to change angle (avoiding IK as much as possible)
-Given an end goal, plan a path of movements that will reach it
-Add more moves, possibly using recorded trajectories

Misc. and of varying degrees of importance and difficulty:
-Spread out functions in regrasping (i.e. stacking bricks seems like it should be in another file)
-Figure out better/prettier way of attaching Ar-Tags/attaching blocks
-Standardize beginning position of arms
-Write report/presentation/website

Assembly:
-Experimentation with controls, recorded trajectories, grasping afterwards, etc.
-Beginning with flat area?
