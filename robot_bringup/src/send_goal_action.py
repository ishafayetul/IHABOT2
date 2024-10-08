import rospy
import actionlib       # Use the actionlib package for client and server
from std_msgs.msg import String as str
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
def printMsg(goal_msg):
    rospy.loginfo(goal_msg.target_pose.header.frame_id)
    rospy.loginfo(goal_pose.target_pose.header.stamp)
    rospy.loginfo(goal_msg.target_pose.pose.position.x)
    rospy.loginfo(goal_msg.target_pose.pose.position.y)
    rospy.loginfo(goal_msg.target_pose.pose.position.z)
    rospy.loginfo(goal_msg.target_pose.pose.orientation.x)
    rospy.loginfo(goal_msg.target_pose.pose.orientation.y)
    rospy.loginfo(goal_msg.target_pose.pose.orientation.z)
    rospy.loginfo(goal_msg.target_pose.pose.orientation.w)

GoalPoints = [[-4.45739221572876,1.5868940353393555, 0.0, 0.0, 0.0, 0.0, 1],[1.45739221572876,5.5868940353393555, 0.0, 0.0, 0.0, 0.0, 1]]
loc=""  
goal_pose = MoveBaseGoal()             
def callback(data):
    global loc
    loc=data.data
    rospy.loginfo("Selecated option "+loc)
    if(loc=="A"):
        rospy.loginfo("Setting point A")
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.header.stamp = rospy.Time.now()
        goal_pose.target_pose.pose.position.x = GoalPoints[0][0]
        goal_pose.target_pose.pose.position.y = GoalPoints[0][1]
        goal_pose.target_pose.pose.position.z = GoalPoints[0][2]
        goal_pose.target_pose.pose.orientation.x = GoalPoints[0][3]
        goal_pose.target_pose.pose.orientation.y = GoalPoints[0][4]
        goal_pose.target_pose.pose.orientation.z = GoalPoints[0][5]
        goal_pose.target_pose.pose.orientation.w = GoalPoints[0][6]

    if(loc=="B"):
        rospy.loginfo("Setting point B")
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.header.stamp = rospy.Time.now()
        goal_pose.target_pose.pose.position.x = GoalPoints[1][0]
        goal_pose.target_pose.pose.position.y = GoalPoints[1][1]
        goal_pose.target_pose.pose.position.z = GoalPoints[1][2]
        goal_pose.target_pose.pose.orientation.x = GoalPoints[1][3]
        goal_pose.target_pose.pose.orientation.y = GoalPoints[1][4]
        goal_pose.target_pose.pose.orientation.z = GoalPoints[1][5]    
        goal_pose.target_pose.pose.orientation.w = GoalPoints[1][6]
    printMsg(goal_pose)
    

#def goalReceiver():
    #print("Goal Receiver Function")
#    rospy.init_node('Goal_Receiver', anonymous=True)
#    rospy.Subscriber("sending_goal", str, callback)
    
def goalSender():   
    rospy.loginfo("GS")    
    client = actionlib.SimpleActionClient('move_base_simple/goal', MoveBaseAction)
    client.wait_for_server()
    client.send_goal(goal_pose)
    printMsg(goal_pose)
    wait = client.wait_for_result(rospy.Duration(900))
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        global loc
        loc=""
        return client.get_result()
    
if __name__ == '__main__':
    while(1):
        try:
            #goalReceiver()
            rospy.Subscriber("sending_goal", str, callback)
            rospy.init_node('move_base_goal_publisher')
            result=goalSender()
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")
        time.sleep(1)
