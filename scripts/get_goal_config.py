import rospy
from grasp_candidates_classifier.msg import GraspConfigList
from wd_grasp_generate_messages.msg import GoalConfig
from geometry_msgs import Point 


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
    if len(grasps) > 0 and goal_pos.x != 0 :
        rospy.loginfo('Received %d grasps.', len(grasps))
        rospy.loginfo('Received goal pos: %d, %d, %d', goal_pos.x, goal_pos.y, goal_pos.z)
        break
    rate.sleep()

pub = rospy.Publisher('goal_config', GoalConfig, queue_size=1)
grasp = grasps[0]
msg = GoalConfig()
msg.bottom = goal_pos
msg.approach = grasp.approach
msg.binormal = grasp.binormal
msg.axis = grasp.axis
s = raw_input('Hit [ENTER] to publish')
pub.publish(msg)
rospy.sleep(2)
print 'Published goal config'

