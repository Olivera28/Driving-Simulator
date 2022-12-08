#include <cse400_robot/robot_hardware_interface.h> 


Robot::Robot(ros::NodeHandle& nodehandle){
    // Create joint_state_interface for JointA
    hardware_interface::JointStateHandle jointStateHandleFront("stepper_stepper_shaft", &pos[0], &vel[0], &eff[0]);
    hardware_interface::JointStateHandle jointStateHandleRWheel("rear_shaft", &pos[1], &vel[1], &eff[1]);
   
    joint_state.registerHandle(jointStateHandleFront);
    joint_state.registerHandle(jointStateHandleRWheel);
    
    // Register all joints interfaces    
    registerInterface(&joint_state);
    
    
    // Create position joint interface
    hardware_interface::JointHandle jointVelocityHandleFront(jointStateHandleFront, &cmd[0]);
    hardware_interface::JointHandle jointVelocityHandleRWheel(jointStateHandleRWheel, &cmd[1]);
    
    joint_command.registerHandle(jointVelocityHandleFront);
    joint_command.registerHandle(jointVelocityHandleRWheel);
    
    // Register all joints interfaces
    registerInterface(&joint_command);
    
    
    // Create position joint interface
    hardware_interface::JointHandle PositionjointHandleFront(jointStateHandleFront, &pos[0]);
    hardware_interface::JointHandle PositionjointHandleRWheel(jointStateHandleRWheel, &pos[1]);
    
    joint_position.registerHandle(PositionjointHandleFront);
    joint_position.registerHandle(PositionjointHandleRWheel);
    
    // Register all joints interfaces
    registerInterface(&joint_position);
    
    
    teensy_pub = nh.advertise<cse400_msgs::Command>("teensy/Commands", 1000);
    teensy_sub = nh.subscribe("teensy/Motors", 1000, &Robot::teensyCallback, this);
    
}




void Robot::read(){
    pos[0] = motor_msg.front_pos;
    pos[1] = motor_msg.rear_pos;
    
    vel[0] = motor_msg.front_vel;
    vel[1] = motor_msg.rear_vel;
    
}

void Robot::write(){
    command_msg.front_vel = cmd[0];
    command_msg.rear_vel = cmd[1];
    
    teensy_pub.publish(command_msg);
}



void Robot::teensyCallback(const cse400_msgs::Motor& msg){

        motor_msg = msg;
    
}




int main(int argc, char** argv){
    ros::init(argc, argv, "hw_interface");
    
    ros::NodeHandle nh;
    
    Robot MyRobot(nh);
    
    controller_manager::ControllerManager robot_controller_manager(&MyRobot, nh);
    
    ros::Duration period(0.1);
    ros::Rate rate(1.0/period.toSec());
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    while(ros::ok()){
        MyRobot.read();
        robot_controller_manager.update(ros::Time::now(), period);
        MyRobot.write();
        rate.sleep();
    }


}



