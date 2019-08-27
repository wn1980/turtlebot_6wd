#include <math.h>

#include <ros/ros.h>
//#include <lino_msgs/Velocities.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

double g_vel_x = 0.0;
double g_vel_y = 0.0;

//double g_vel_dt = 0.0;
double g_imu_dt = 0.0;
double g_imu_z = 0.0;

//ros::Time g_last_loop_time(0.0);
//ros::Time g_last_vel_time(0.0);
ros::Time g_last_imu_time(0.0);

void IMUCallback( const sensor_msgs::Imu& imu){
    //callback every time the robot's angular velocity is received
    ros::Time current_time = ros::Time::now();
    //this block is to filter out imu noise
    if(imu.angular_velocity.z > -0.03 && imu.angular_velocity.z < 0.03)
        g_imu_z = 0.;
    else
        g_imu_z = imu.angular_velocity.z;
/*        
    g_imu_dt = (current_time - g_last_imu_time).toSec();
    
    if(imu.linear_acceleration.x > -0.5 && imu.linear_acceleration.x < 0.5)
		g_vel_x += 0.;
	else
		g_vel_x += imu.linear_acceleration.x * g_imu_dt;
*/    
    g_last_imu_time = current_time;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	ROS_INFO("Seq: [%d]", msg->header.seq);
	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	//ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
	
	g_vel_x = msg->twist.twist.linear.x;
 }

int main(int argc, char** argv){
    ros::init(argc, argv, "raw_velocity");
    ros::NodeHandle n;
    ros::NodeHandle nh_private_("~");
    ros::Subscriber imu_sub = n.subscribe("imu/data", 50, IMUCallback);
    ros::Subscriber odom_sub = n.subscribe("odom_rf2o", 50, odomCallback);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("raw_vel", 50);

    double rate = 10.0;

    ros::Rate r(rate);
    
    while(n.ok()){
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();
        
        geometry_msgs::Twist msg;
        msg.linear.x = g_vel_x;
        msg.angular.z = g_imu_z;
        vel_pub.publish(msg);
        
        r.sleep();
    }
}
