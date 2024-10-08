#include <ros/ros.h>

#include <std_msgs/Float32.h>

#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"

#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32MultiArray.h"

class velHandler {
public:
  velHandler() {
    cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &velHandler::cmd_velCB, this);
    set_vel_pub = nh.advertise<std_msgs::Float32MultiArray>("set_vel", 10);
  }

  void cmd_velCB(const geometry_msgs::Twist::ConstPtr &cmd) {
    std_msgs::Float32MultiArray array;
    array.data.clear();
    float x = cmd->linear.x;
    float y = cmd->linear.y;
    float rot = cmd->angular.z;

    float front_left = (x - y - rot * wheel_geometry) / wheel_radius;
    float front_right = (x + y + rot * wheel_geometry) / wheel_radius;
    float back_left = (x + y - rot * wheel_geometry) / wheel_radius;
    float back_right = (x - y + rot * wheel_geometry) / wheel_radius;

    //float front_left = 26709 * (x - y - (wheel_separation_width + wheel_separation_lenght)*rot);
    //float front_right = 26709 * (x + y + (wheel_separation_width + wheel_separation_lenght)*rot);
    //float back_left = 26709 * (x + y - (wheel_separation_width + wheel_separation_lenght)*rot);
    //float back_right = 26709 * (x - y + (wheel_separation_width + wheel_separation_lenght)*rot);

    array.data.push_back(front_left);
    array.data.push_back(front_right);
    array.data.push_back(back_left);
    array.data.push_back(back_right);

    set_vel_pub.publish(array);
  }

protected:
  ros::NodeHandle nh;
  ros::Subscriber cmd_vel_sub;
  ros::Publisher set_vel_pub;
  float w = 0.44;
  float wheel_radius = 0.0425;
  float wheel_separation_width = 0.18125;
  float wheel_separation_lenght = 0.125;
  float wheel_geometry= wheel_separation_lenght+wheel_separation_width;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "vel_to_arduino");
  velHandler handler;
  ros::spin();
  return 0;
}