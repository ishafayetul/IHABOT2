import rospy
from std_msgs.msg import Float32MultiArray as FLOAT
from std_msgs.msg import String as str
import time
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib 
from geometry_msgs.msg import PoseStamped

def printMsg(goal_msg):
    rospy.loginfo_once(goal_msg.header.seq)
    rospy.loginfo_once(goal_msg.header.stamp)
    rospy.loginfo_once(goal_msg.pose.position.x)
    rospy.loginfo_once(goal_msg.pose.position.y)
    rospy.loginfo_once(goal_msg.pose.position.z)
    rospy.loginfo_once(goal_msg.pose.orientation.x )
    rospy.loginfo_once(goal_msg.pose.orientation.y)
    rospy.loginfo_once(goal_msg.pose.orientation.z)
    rospy.loginfo_once(goal_msg.pose.orientation.w )

count=0
def callback(GoalPoint):
    rospy.loginfo_once("Command Received: x=%s,y=%s", GoalPoint.data[0],GoalPoint.data[1])
    if (GoalPoint.data[0] == -100.0 and GoalPoint.data[1] == -100.0):
        cancel_pub.publish(GoalID())
        rospy.loginfo_once("Goal aborted by user")
        goal_feed_publisher.publish("Goal Aborted Successully")
        return 0
    
    goal_feed_publisher.publish("Goal+1")
    goal_pose = PoseStamped()
    goal_pose.header.seq=count+1
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = rospy.Time.now()

    goal_pose.pose.position.x = GoalPoint.data[0]
    goal_pose.pose.position.y = GoalPoint.data[1]
    goal_pose.pose.position.z = 0

    goal_pose.pose.orientation.x = 0
    goal_pose.pose.orientation.y = 0
    goal_pose.pose.orientation.z = 0
    goal_pose.pose.orientation.w = 1
    
    goal_publisher.publish(goal_pose)
    rospy.loginfo_once("Goal Published")
    printMsg(goal_pose)  
    return 1
        
if __name__ == '__main__':
    rospy.init_node("Receive_and_Publish")
    rospy.loginfo("Receive_and_Publish node started")
    
    # Initialize the publisher once
    cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=5)
    goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)
    goal_feed_publisher=rospy.Publisher("feedback_goal", str,queue_size=5)
    receive_goal = rospy.Subscriber("Goal_from_Interface", FLOAT, callback)
    rospy.spin()
