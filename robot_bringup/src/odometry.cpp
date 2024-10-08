#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int64MultiArray.h"

class Odometry_calc{

    public:
        Odometry_calc();
        void spin();

    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Subscriber wheel_sub; //Encoder ticks subscriber for odometry calculation

        ros::Publisher odom_pub; // odom_pub will publish the odometry value
        tf2_ros::TransformBroadcaster odom_broadcaster; //odom_broadcaster for generating odom tf
        double encoder_min; // minimum encoder ticks arduino due can measure -2^31
        double encoder_max; // maximum encoder ticks arduino due can measure 2^31

        double encoder_low_wrap; // variable indicate the direction of odometry
        double encoder_high_wrap; // variable indicate the direction of odometry

	    double prev_encoder_fl; // keeps track of previous encoder value
	    double prev_encoder_fr; // keeps track of previous encoder value 

	    double prev_encoder_bl; // keeps track of previous encoder value
	    double prev_encoder_br; // keeps track of previous encoder value

	    double fl_mult; // variable indicate the direction of odometry
	    double fr_mult; // variable indicate the direction of odometry
        double bl_mult; // variable indicate the direction of odometry
	    double br_mult; // variable indicate the direction of odometry

	    double f_left;
	    double f_right;
		double b_left;
	    double b_right;

	    double rate;
        ros::Duration t_delta;

        ros::Time t_next;

        ros::Time then;

        double enc_fleft ;

        double enc_fright;

		double enc_bleft ;

        double enc_bright;

        double ticks_meter;

        double base_width;
		double WHEEL_RADIUS;
		double WHEEL_SEPARATION_WIDTH;
		double WHEEL_SEPARATION_LENGTH;

		double linear_velocity_x_;
		double linear_velocity_y_;
		double angular_velocity_z_;

        double x_pos_,y_pos_, heading_;

        ros::Time current_time, last_time;


        void encoderCb(const std_msgs::Int64MultiArray::ConstPtr& ticks);


        void init_variables();

        void get_node_params();


        void update();
};

Odometry_calc::Odometry_calc(){


	init_variables();

	ROS_INFO("Started odometry computing node");

	wheel_sub = n.subscribe("/encoder_ticks",30, &Odometry_calc::encoderCb, this);



  	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 30);   
  	


	//Retrieving parameters of this node
	get_node_params();
}


// initializing all the variable
void Odometry_calc::init_variables()
{


	prev_encoder_fl = 0;
	prev_encoder_fr = 0;
	prev_encoder_bl = 0;
	prev_encoder_br = 0;

	fl_mult = 0;
	fr_mult = 0;
	bl_mult = 0;
	br_mult = 0;


	f_left = 0;
	f_right = 0;
	b_left = 0;
	b_right = 0;

	encoder_min =  -2147483648;
	encoder_max =  2147483648;

	rate = 40;

	ticks_meter = 34000;

	base_width = 0.44;
	WHEEL_RADIUS = 0.0425;
	WHEEL_SEPARATION_WIDTH = 0.18125; //0.075;
	WHEEL_SEPARATION_LENGTH = 0.125; //0.1575;
	

	encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min ;
	encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min ;

	t_delta = ros::Duration(1.0 / rate);
	t_next = ros::Time::now() + t_delta;
	
	then = ros::Time::now();


	enc_fleft = 0;
	enc_fright = 0;
	enc_bleft = 0;
	enc_bright = 0;

	linear_velocity_x_ = 0;
	linear_velocity_y_ = 0;
	angular_velocity_z_ = 0;

	x_pos_ = 0;
	y_pos_ = 0;
	heading_ = 0;
	
	current_time = ros::Time::now();
  	last_time = ros::Time::now();

}


void Odometry_calc::get_node_params(){

	
        if(n.getParam("rate", rate)){
	 
		ROS_INFO_STREAM("Rate from param" << rate);	       
	}

        if(n.getParam("encoder_min", encoder_min)){
	 
		ROS_INFO_STREAM("Encoder min from param" << encoder_min);	       
	}


        if(n.getParam("encoder_max", encoder_max)){
	 
		ROS_INFO_STREAM("Encoder max from param" << encoder_max);	       
	}


        if(n.getParam("wheel_low_wrap", encoder_low_wrap)){
	 
		ROS_INFO_STREAM("wheel_low_wrap from param" << encoder_low_wrap);	       
	}


        if(n.getParam("wheel_high_wrap", encoder_high_wrap)){
	 
		ROS_INFO_STREAM("wheel_high_wrap from param" << encoder_high_wrap);	       
	}




        if(n.getParam("ticks_meter", ticks_meter)){
	 
		ROS_INFO_STREAM("Ticks meter" << ticks_meter);	       
	}


        if(n.getParam("base_width", base_width )){
	 
		ROS_INFO_STREAM("Base Width" << base_width );	       
	}

	if(n.getParam("wheel_radius", WHEEL_RADIUS )){
	 
		ROS_INFO_STREAM("Wheel Radius" << WHEEL_RADIUS );	       
	}

	if(n.getParam("wheel_separation_width", WHEEL_SEPARATION_WIDTH )){
	 
		ROS_INFO_STREAM("Wheel Separation Width" << WHEEL_SEPARATION_WIDTH );	       
	}

	if(n.getParam("wheel_separation_lenght", WHEEL_SEPARATION_LENGTH )){
	 
		ROS_INFO_STREAM("Wheel Separation Lenght" << WHEEL_SEPARATION_LENGTH );	       
	}


/*
	ROS_INFO_STREAM("Encoder min" << encoder_min);
	ROS_INFO_STREAM("Encoder max" << encoder_max);
	ROS_INFO_STREAM("Encoder Low Wrap"<< encoder_low_wrap);
	ROS_INFO_STREAM("Encoder High Wrap" << encoder_high_wrap);
*/
	ROS_INFO_STREAM("ticks meter: " << ticks_meter);
	ROS_INFO_STREAM("base width: " << base_width);
	ROS_INFO_STREAM("wheel_separation_width: " << WHEEL_SEPARATION_WIDTH);
	ROS_INFO_STREAM("Wheel Separation Lenght: " << WHEEL_SEPARATION_LENGTH);



}


//Spin function
void Odometry_calc::spin(){

     ros::Rate loop_rate(rate);

     while (ros::ok())
	{
		update();
		loop_rate.sleep();
	
	}


}



//Update function
void Odometry_calc::update(){

	ros::Time now = ros::Time::now();
	
//	ros::Time elapsed;

	double elapsed;

	double d_fleft, d_fright, d_bleft, d_bright, d, th,x,y;
	double vel_wheel_left_front_, vel_wheel_left_rear_, vel_wheel_right_rear_, vel_wheel_right_front_;


	if ( now > t_next) {

		elapsed = now.toSec() - then.toSec(); 

 // 	        ROS_INFO_STREAM("elapsed =" << elapsed);

		

        
		if(enc_fleft == 0 && enc_fright == 0 && enc_bleft == 0 && enc_bright == 0){
			d_fleft = 0;
			d_fright = 0;
			d_bleft = 0;
			d_bright = 0;
		}
		else{
			d_fleft = (f_left - enc_fleft) / ( ticks_meter);
			d_fright = (f_right - enc_fright) / ( ticks_meter);
			d_bleft = (b_left - enc_bleft) / (ticks_meter);
			d_bright = (b_right - enc_bright) / (ticks_meter);
		}
		//ROS_INFO("\n f_left=%.2f, f_right=%.2f, b_left=%.2f, b_right=%.2f\n", f_left, f_right, b_left, b_right);

		//ROS_INFO("\n d_fleft=%.2f, d_fright=%.2f, d_bleft=%.2f, d_bright=%.2f\n", d_fleft, d_fright, d_bleft, d_bright);

		enc_fleft = f_left;
		enc_fright = f_right;
		enc_bleft = b_left;
		enc_bright = b_right;
		
		vel_wheel_left_front_ = d_fleft/elapsed;
		vel_wheel_right_front_ = d_fright/elapsed;
		vel_wheel_left_rear_ = d_bleft/elapsed;
		vel_wheel_right_rear_ = d_bright/elapsed;

		//ROS_INFO("\n elapsed=%.2f, nowtosec=%.2f, thentosec=%.2f, different=%.5f\n",elapsed, now.toSec(), then.toSec(), now.toSec()-then.toSec());
		//ROS_INFO("\n vel_wheel_left_front_=%.2f, vel_wheel_right_front_=%.2f, vel_wheel_left_rear_=%.2f, vel_wheel_right_rear_=%.2f\n", vel_wheel_left_front_, vel_wheel_right_front_, vel_wheel_left_rear_, vel_wheel_right_rear_);

		linear_velocity_x_ = (vel_wheel_left_front_ + vel_wheel_right_front_ + vel_wheel_left_rear_ + vel_wheel_right_rear_) * (WHEEL_RADIUS/4);
	    linear_velocity_y_ = (-vel_wheel_left_front_ + vel_wheel_right_front_ + vel_wheel_left_rear_ - vel_wheel_right_rear_) * (WHEEL_RADIUS/4);
	    angular_velocity_z_ = (-vel_wheel_left_front_ + vel_wheel_right_front_ - vel_wheel_left_rear_ + vel_wheel_right_rear_) * (WHEEL_RADIUS/(4 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)));

		double delta_heading = angular_velocity_z_*1.2; // [radians]
    	double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)); // [m]
    	double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)); // [m]
		//calculate current position of the robot
    	x_pos_ += delta_x;
    	y_pos_ += delta_y;
    	heading_ += delta_heading;

		

		tf2::Quaternion odom_quat ;

		odom_quat.setRPY(0,0,heading_);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = now;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "robot_footprint";

		odom_trans.transform.translation.x = x_pos_;
		odom_trans.transform.translation.y = y_pos_;
		odom_trans.transform.translation.z = 0.0;
		
		odom_trans.transform.rotation.x = odom_quat.x();
		odom_trans.transform.rotation.y = odom_quat.y();
		odom_trans.transform.rotation.z = odom_quat.z();
		odom_trans.transform.rotation.w = odom_quat.w();

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		    
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = now;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = x_pos_;
		odom.pose.pose.position.y = y_pos_;
		odom.pose.pose.position.z = 0.0;
		//set robot's heading direction
		odom.pose.pose.orientation.x = odom_quat.x();
		odom.pose.pose.orientation.y = odom_quat.y();
		odom.pose.pose.orientation.z = odom_quat.z();
		odom.pose.pose.orientation.w = odom_quat.w();
		odom.pose.covariance[0] = 0.001;
		odom.pose.covariance[7] = 0.001;
		odom.pose.covariance[35] = 0.001;

		//set the velocity
		odom.child_frame_id = "robot_footprint";
		odom.twist.twist.linear.x = linear_velocity_x_;
		odom.twist.twist.linear.y = linear_velocity_y_;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		// angular speed from encoders
		odom.twist.twist.angular.z = angular_velocity_z_;
		odom.twist.covariance[0] = 0.0001;
		odom.twist.covariance[7] = 0.0001;
		odom.twist.covariance[35] = 0.0001;

		//publish the message
		odom_pub.publish(odom);

	    then = now;

//		    ROS_INFO_STREAM("dx =" << x_final);

//		    ROS_INFO_STREAM("dy =" << y_final);

	    ros::spinOnce();


		}
	 else { ; }
//		ROS_INFO_STREAM("Not in loop");
		

}


//encoder callback

void Odometry_calc::encoderCb(const std_msgs::Int64MultiArray::ConstPtr& ticks)

{


// ROS_INFO_STREAM("Right tick" << right_ticks->data);


	double fl_enc = ticks->data[0];
	double fr_enc = ticks->data[1];
    double bl_enc = ticks->data[2];
    double br_enc = ticks->data[3];

	//for forward right encoder wrapping
	if((fr_enc < encoder_low_wrap) && (prev_encoder_fr > encoder_high_wrap))
	{
		
		fr_mult = fr_mult + 1;
	}
	

	if((fr_enc > encoder_high_wrap) && (prev_encoder_fr < encoder_low_wrap))

	{
		
		fr_mult = fr_mult - 1;
	}

	//for forward left encoder wrapping 

	if((fl_enc < encoder_low_wrap) && (prev_encoder_fl > encoder_high_wrap))
	{
		
		fl_mult = fl_mult + 1;
	}
	

	if((fl_enc > encoder_high_wrap) && (prev_encoder_fl < encoder_low_wrap))

	{
		
		fl_mult = fl_mult - 1;
	}

	//for back right encoder wrapping
	if((br_enc < encoder_low_wrap) && (prev_encoder_br > encoder_high_wrap))
	{
		
		br_mult = br_mult + 1;
	}
	

	if((br_enc > encoder_high_wrap) && (prev_encoder_br < encoder_low_wrap))

	{
		
		br_mult = br_mult - 1;
	}

	//for back left encoder wrapping 

	if((bl_enc < encoder_low_wrap) && (prev_encoder_bl > encoder_high_wrap))
	{
		
		bl_mult = bl_mult + 1;
	}
	

	if((bl_enc > encoder_high_wrap) && (prev_encoder_bl < encoder_low_wrap))

	{
		
		bl_mult = bl_mult - 1;
	}

	f_right = 1.0 * (fr_enc + fr_mult * (encoder_max - encoder_min ));
	f_left = 1.0 * (fl_enc + fl_mult * (encoder_max - encoder_min ));
	b_right = 1.0 * (br_enc + br_mult * (encoder_max - encoder_min));
	b_left = 1.0 * (bl_enc + bl_mult * (encoder_max - encoder_min));
	prev_encoder_fr = fr_enc;
	prev_encoder_fl = fl_enc;
	prev_encoder_br = br_enc;
	prev_encoder_bl = bl_enc;

//	ROS_INFO_STREAM("Right " << right);



}

int main(int argc, char **argv)

{
	ros::init(argc, argv,"mecanum_tf");
	Odometry_calc obj;
	obj.spin();


	return 0;

}