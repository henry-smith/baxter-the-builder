import sys
sys.path.append('/home/cc/ee106b/sp17/class/ee106b-aam/baxter_labs/src/baxter_the_builder/src')
import builder

rospy.init_node('test_script')

# Set up gripper
gripper = baxter_gripper.Gripper('right')
gripper.calibrate()

# Set up limb
limb = baxter_interface.Limb('right')
build = Builder(limb, gripper)
build.stack_bricks('block_0', 'block_1')