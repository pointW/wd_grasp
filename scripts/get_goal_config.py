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
pub_pick_mark = rospy.Publisher('pick_mark', Marker, queue_size=0)
pub_place_mark = rospy.Publisher('place_mark', Marker, queue_size=0)

grasp = grasps[0]
for g in grasps:
  if g.approach.x > 0.9 and g.approach.y > 0:
    grasp = g
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
pick_marker.color.r = 0.0
pick_marker.color.g = 1.0
pick_marker.color.b = 0.0

place_rotation = numpy.array([[grasp.approach.x, grasp.binormal.x, grasp.axis.x, goal_pos.x],
                              [grasp.approach.y, grasp.binormal.y, grasp.axis.y, goal_pos.y],
                              [grasp.approach.z, grasp.binormal.z, grasp.axis.z, goal_pos.z],
                              [0, 0, 0, 1]])
place_quat = tf.transformations.quaternion_from_matrix(place_rotation)
place_pos = goal_pos
place_pos.z = pick_pos.z + 0.02
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
place_marker.color.r = 0.0
place_marker.color.g = 1.0
place_marker.color.b = 0.0

pub_pick_mark.publish(pick_marker)
pub_place_mark.publish(place_marker)

s = raw_input('Hit [ENTER] to continue')

#pick_msg = Quaternion()
#pick_msg.x = quat[0]
#pick_msg.y = quat[1]
#pick_msg.z = quat[2]
#pick_msg.w = quat[3]





#msg = GoalConfig()
#msg.bottom = goal_pos
#msg.approach = grasp.approach
#msg.binormal = grasp.binormal
#msg.axis = grasp.axis
#rospy.loginfo(msg)
#s = raw_input('Hit [ENTER] to publish')
#pub.publish(msg)
#rospy.sleep(2)
#print 'Published goal config'

