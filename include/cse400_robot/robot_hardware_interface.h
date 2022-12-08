#ifndef ROBOT_HARDWARE_INTERFACE_H
#define ROBOT_HARDWARE_INTERFACE_H

#include <ros/console.h> 
#include <cse400_robot/robot_hardware_interface.h> 
#include <hardware_interface/joint_state_interface.h> 
#include <hardware_interface/joint_command_interface.h> 
#include <controller_manager/controller_manager.h> 
#include <cse400_msgs/Motor.h> 
#include <cse400_msgs/Command.h> 

class Robot : public hardware_interface::RobotHW 
{
        
    private:
        ros::NodeHandle nh;
        
        hardware_interface::JointStateInterface joint_state;
        hardware_interface::VelocityJointInterface joint_command;
        hardware_interface::PositionJointInterface joint_position;
        
        double pos[2];
        double vel[2];
        double eff[2];
        double cmd[2];
        
        
        ros::Subscriber teensy_sub;
        ros::Publisher teensy_pub;
        
        
        cse400_msgs::Motor motor_msg;
        cse400_msgs::Command command_msg;
        
        void teensyCallback(const cse400_msgs::Motor& msg);
        
    public:
 
         Robot(ros::NodeHandle& nodehandle);
       
        
         void read();
         void write();
        
};

#endif // ROBOT_HARDWARE_INTERFACE_H



