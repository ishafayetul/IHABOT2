import rospy
from std_msgs.msg import String as str
from geometry_msgs.msg import PoseStamped
import time
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionResult
def printMsg(goal_msg):
    rospy.loginfo(goal_msg.header.seq)
    rospy.loginfo(goal_msg.header.stamp)
    rospy.loginfo(goal_msg.pose.position.x)
    rospy.loginfo(goal_msg.pose.position.y)
    rospy.loginfo(goal_msg.pose.position.z)
    rospy.loginfo(goal_msg.pose.orientation.x )
    rospy.loginfo(goal_msg.pose.orientation.y)
    rospy.loginfo(goal_msg.pose.orientation.z)
    rospy.loginfo(goal_msg.pose.orientation.w )

GoalPoints = [[-2.45739221572876,1.5868940353393555, 0.0, 0.0, 0.0, 0.0, 1],[1.45739221572876,5.5868940353393555, 0.0, 0.0, 0.0, 0.0, 1]]
loc=""  
goal_pose = PoseStamped()
count=0

cancel_goal=GoalID()
def callback(data):
    global loc
    loc=data.data
    rospy.loginfo("Selecated option "+loc)
    if(loc=="A"):
        rospy.loginfo("Setting point A")

        goal_pose.header.seq=count+1
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = rospy.Time.now()

        goal_pose.pose.position.x = GoalPoints[0][0]
        goal_pose.pose.position.y = GoalPoints[0][1]
        goal_pose.pose.position.z = GoalPoints[0][2]

        goal_pose.pose.orientation.x = GoalPoints[0][3]
        goal_pose.pose.orientation.y = GoalPoints[0][4]
        goal_pose.pose.orientation.z = GoalPoints[0][5]
        goal_pose.pose.orientation.w = GoalPoints[0][6]

    if(loc=="B"):
        rospy.loginfo("Setting point B")

        goal_pose.header.seq=count+1
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = rospy.Time.now()

        goal_pose.pose.position.x = GoalPoints[1][0]
        goal_pose.pose.position.y = GoalPoints[1][1]
        goal_pose.pose.position.z = GoalPoints[1][2]

        goal_pose.pose.orientation.x = GoalPoints[1][3]
        goal_pose.pose.orientation.y = GoalPoints[1][4]
        goal_pose.pose.orientation.z = GoalPoints[1][5]    
        goal_pose.pose.orientation.w = GoalPoints[1][6]
    printMsg(goal_pose)


def result(data):
    flag=data.result
    rospy.loginfo(flag)
    if(flag=="Goal reached."):
        global loc
        loc=""
prev_goal=""  
if __name__ == '__main__':
    
    while(1):
        rospy.init_node('move_base_goal_publisher')
        cancel_goal_publisher = rospy.Publisher("move_base/cancel", GoalID, queue_size=5)
        goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)
        rospy.Subscriber("sending_goal", str, callback)
        rospy.Subscriber("move_base/result", MoveBaseActionResult, result)
        if(loc=="cancel"):
            cancel_goal_publisher.publish(cancel_goal)
            rospy.loginfo("Goal is canceled")
            loc=""
            break
        goal_publisher.publish(goal_pose)
        time.sleep(1)
        prev_goal=loc
        
        
            
        