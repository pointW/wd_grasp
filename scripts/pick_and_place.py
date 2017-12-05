import rospy, numpy, tf
from grasp_candidates_classifier.msg import GraspConfigList
from wd_grasp.msg import GoalConfig
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker


grasps = []
goal_pos = Point()
goal_config = GoalConfig()

def grasp_callback(msg):
    global grasps
    grasps = msg.grasps
    
def goal_pos_callback(msg):
  global goal_pos
  goal_pos = msg
  


# Create a ROS node.
rospy.init_node('get_goal')

# Subscribe to the ROS topic that contains the grasps.
sub_grasps = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, grasp_callback)
sub_goal_pos = rospy.Subscriber('goal_position', Point, goal_pos_callback)

# Wait for grasps to arrive.
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    rospy.loginfo('waiting')
    if len(grasps) > 0 and goal_pos.x != 0 :
        rospy.loginfo('Received %d grasps.', len(grasps))
        rospy.loginfo('Received goal pos: %f, %f, %f', goal_pos.x, goal_pos.y, goal_pos.z)
        break
    rate.sleep()

pub_goal = rospy.Publisher('goal_config', GoalConfig, queue_size=1)
pub_pick = rospy.Publisher('pick_config', Quaternion, queue_size=1)
pub_pick_mark = rospy.Publisher('pick_mark', Marker, queue_size=1)
pub_place_mark = rospy.Publisher('place_mark', Marker, queue_size=1)

grasp = grasps[0]
for g in grasps:
  if g.approach.x > 0.85 and g.axis.z < -0.85:
    grasp = g
    break
if grasp.approach.x < 0.85:
  print 'bad grasp'
  exit
print grasp
pick_rotation = numpy.array([[grasp.approach.x, grasp.binormal.x, grasp.axis.x, grasp.bottom.x],
                             [grasp.approach.y, grasp.binormal.y, grasp.axis.y, grasp.bottom.y],
                             [grasp.approach.z, grasp.binormal.z, grasp.axis.z, grasp.bottom.z],
                             [0, 0, 0, 1]])
pick_quat = tf.transformations.quaternion_from_matrix(pick_rotation)
pick_pos = grasp.bottom
pick_marker = Marker()
pick_marker.header.frame_id = "base"
pick_marker.header.stamp = rospy.Time()
pick_marker.ns = "my_namespace"
pick_marker.id = 0
pick_marker.type = 0
pick_marker.action = 0
pick_marker.pose.position.x = pick_pos.x
pick_marker.pose.position.y = pick_pos.y
pick_marker.pose.position.z = pick_pos.z
pick_marker.pose.orientation.x = pick_quat[0]
pick_marker.pose.orientation.y = pick_quat[1]
pick_marker.pose.orientation.z = pick_quat[2]
pick_marker.pose.orientation.w = pick_quat[3]
pick_marker.scale.x = 0.1
pick_marker.scale.y = 0.01
pick_marker.scale.z = 0.01
pick_marker.color.a = 1.0
pick_marker.color.r = 1.0
pick_marker.color.g = 0.0
pick_marker.color.b = 0.0

place_rotation = numpy.array([[grasp.approach.x, grasp.binormal.x, grasp.axis.x, goal_pos.x],
                              [grasp.approach.y, grasp.binormal.y, grasp.axis.y, goal_pos.y],
                              [grasp.approach.z, grasp.binormal.z, grasp.axis.z, goal_pos.z],
                              [0, 0, 0, 1]])
place_quat = tf.transformations.quaternion_from_matrix(place_rotation)
place_pos = goal_pos
place_pos.z = pick_pos.z + 0.05
place_marker = Marker()
place_marker.header.frame_id = "base"
place_marker.header.stamp = rospy.Time()
place_marker.ns = "my_namespace"
place_marker.id = 1
place_marker.type = 0
place_marker.action = 0
place_marker.pose.position.x = place_pos.x
place_marker.pose.position.y = place_pos.y
place_marker.pose.position.z = place_pos.z
place_marker.pose.orientation.x = place_quat[0]
place_marker.pose.orientation.y = place_quat[1]
place_marker.pose.orientation.z = place_quat[2]
place_marker.pose.orientation.w = place_quat[3]
place_marker.scale.x = 0.1
place_marker.scale.y = 0.01
place_marker.scale.z = 0.01
place_marker.color.a = 1.0
place_marker.color.r = 1.0
place_marker.color.g = 0.0
place_marker.color.b = 0.0

for i in range(1, 10):
    pub_pick_mark.publish(pick_marker)
    pub_place_mark.publish(place_marker)
    rospy.sleep(0.1)

s = raw_input('Hit [ENTER] to continue')

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface

gripper = baxter_interface.Gripper('right')

#end_rotation = numpy.array([[grasp.axis.x, grasp.approach.x, grasp.binormal.x, grasp.bottom.x],
                            #[grasp.axis.y, grasp.approach.y, grasp.binormal.y, grasp.bottom.y],
                            #[grasp.axis.z, grasp.approach.z, grasp.binormal.z, grasp.bottom.z],
                            #[0, 0, 0, 1]])
end_rotation = numpy.array([[-grasp.axis.x, grasp.binormal.x, grasp.approach.x, grasp.bottom.x],
                            [-grasp.axis.y, grasp.binormal.y, grasp.approach.y, grasp.bottom.y],
                            [-grasp.axis.z, grasp.binormal.z, grasp.approach.z, grasp.bottom.z],
                            [0, 0, 0, 1]])
end_quat = tf.transformations.quaternion_from_matrix(end_rotation)
orientation = Quaternion(x = end_quat[0],
                         y = end_quat[1],
                         z = end_quat[2],
                         w = end_quat[3])

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander('right_arm')
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                       moveit_msgs.msg.DisplayTrajectory,
                                       queue_size=20)
print "============ Waiting for RVIZ..."
rospy.sleep(10)
print "============ Reference frame: %s" % group.get_planning_frame()
print "============ Reference frame: %s" % group.get_end_effector_link()
print "============ Robot Groups:"
print robot.get_group_names()
print "============ Printing robot state"
print robot.get_current_state()
print "============"

upright_constraints = moveit_msgs.msg.Constraints()
upright_constraints.name = "upright"
constraints = moveit_msgs.msg.OrientationConstraint()
# constraints.header = "upright"
constraints.link_name = group.get_end_effector_link()
constraints.orientation = orientation
constraints.absolute_x_axis_tolerance = 0.4
constraints.absolute_y_axis_tolerance = 0.4
constraints.absolute_z_axis_tolerance = 0.1
constraints.weight = 1
upright_constraints.orientation_constraints.append(constraints)

def enable_upright_constraints():
    group.set_path_constraints(upright_constraints)

def move_to(x, y, z):
    print "============ Generating plan 1"
    pose_target = geometry_msgs.msg.Pose()
    #pose_target.orientation.w = 1.0
    #pose_target.position.x = 0.7
    #pose_target.position.y = -0.05
    #pose_target.position.z = 1.1
    
    pose_target.orientation = orientation
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    
    print pose_target
    group.set_pose_target(pose_target)
    plan1 = group.plan()
    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(5)
    print "============ Visualizing plan1"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory)
    print "============ Waiting while plan1 is visualized..."
    s = raw_input('Hit [ENTER] to continue')
    
    #group.execute(plan1)
   
    rospy.sleep(5)

#current_pos = group.get_current_pose().pose.position
#move_to(current_pos.x, current_pos.y, current_pos.z + 0.1)
#s = raw_input('Hit [ENTER] to continue')

move_to(pick_pos.x - 0.1 * grasp.approach.x, 
        pick_pos.y - 0.1 * grasp.approach.y, 
        pick_pos.z - 0.1 * grasp.approach.z)

#gripper.open()
#rospy.sleep(1.0)
move_to(pick_pos.x, pick_pos.y, pick_pos.z)
#gripper.close()
#rospy.sleep(1.0)

# move_to(pick_pos.x, pick_pos.y, pick_pos.z + 0.1)
# move_to(place_pos.x, place_pos.y, place_pos.z)
# gripper.open()
# rospy.sleep(1.0)





#msg.bottom = goal_pos
#msg.approach = grasp.approach
#msg.binormal = grasp.binormal
#msg.axis = grasp.axis
#rospy.loginfo(msg)
#s = raw_input('Hit [ENTER] to publish')
#pub.publish(msg)
#rospy.sleep(2)
#print 'Published goal config'

