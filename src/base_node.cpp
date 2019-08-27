#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "lino_msgs/PID.h"

#include "PID.h"

double g_vel_x = 0.0;
double g_imu_z = 0.0;

double vel_x = 0.0;
double imu_z = 0.0;

double kp = 0.4, ki = 0.04, kd = 0.04;
	
PID pid_vel_x(-255, 255, kp, ki, kd);
PID pid_imu_z(-255, 255, kp, ki, kd);

ros::Time g_last_loop_time(0.0);
ros::Time g_prev_command_time(0.0);

void cmd_velCallback(const geometry_msgs::Twist& vel){
	g_vel_x = vel.linear.x;
	g_imu_z = vel.angular.z;
	
	g_prev_command_time = ros::Time::now();
}

void imu_velCallback(const geometry_msgs::Twist& vel){
	vel_x = vel.linear.x;
	imu_z = vel.angular.z;
}

void PIDCallback(const lino_msgs::PID& pid){
    pid_vel_x.updateConstants(pid.p, pid.i, pid.d);
    pid_imu_z.updateConstants(pid.p, pid.i, pid.d);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle n;
    ros::NodeHandle nh_private_("~");
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 50, cmd_velCallback);
    ros::Subscriber imu_vel_sub = n.subscribe("raw_vel", 50, imu_velCallback);
    ros::Subscriber pid_sub = n.subscribe("pid", 50, PIDCallback);
    ros::Publisher pid_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_pid", 50);

    double rate = 20.0;

    ros::Rate r(rate);
    
    while(n.ok()){
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();
        
        if((current_time - g_prev_command_time).toSec() >= 0.25){
			g_vel_x = 0.;
			g_imu_z = 0.;
		}
        
        geometry_msgs::Twist msg;
        //msg.linear.x 	= pid_vel_x.compute(g_vel_x, vel_x);
        msg.linear.x 	= g_vel_x;
        msg.angular.z 	= pid_imu_z.compute(g_imu_z, imu_z);
        pid_pub.publish(msg);

        g_last_loop_time = current_time;
        
        r.sleep();
    }
}
