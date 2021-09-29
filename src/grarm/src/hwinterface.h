#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/posvel_command_interface.h>
#include <iostream>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>


class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot() 
 { 
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_1("joint1", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_1);

   hardware_interface::JointStateHandle state_handle_2("joint2", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_2);

   hardware_interface::JointStateHandle state_handle_3("joint3", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(state_handle_3);

   hardware_interface::JointStateHandle state_handle_4("joint4", &pos[3], &vel[3], &eff[3]);
   jnt_state_interface.registerHandle(state_handle_4);

   hardware_interface::JointStateHandle state_handle_5("joint5", &pos[4], &vel[4], &eff[4]);
   jnt_state_interface.registerHandle(state_handle_5);

   hardware_interface::JointStateHandle state_handle_6("joint6", &pos[5], &vel[5], &eff[5]);
   jnt_state_interface.registerHandle(state_handle_6);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::PosVelJointHandle pos_vel_handle_1(jnt_state_interface.getHandle("joint1"), &cmd_pos[0], &cmd_vel[0]);
   jnt_pos_vel_interface.registerHandle(pos_vel_handle_1);

   hardware_interface::PosVelJointHandle pos_vel_handle_2(jnt_state_interface.getHandle("joint2"), &cmd_pos[1], &cmd_vel[1]);
   jnt_pos_vel_interface.registerHandle(pos_vel_handle_2);

   hardware_interface::PosVelJointHandle pos_vel_handle_3(jnt_state_interface.getHandle("joint3"), &cmd_pos[2], &cmd_vel[2]);
   jnt_pos_vel_interface.registerHandle(pos_vel_handle_3);

   hardware_interface::PosVelJointHandle pos_vel_handle_4(jnt_state_interface.getHandle("joint4"), &cmd_pos[3], &cmd_vel[3]);
   jnt_pos_vel_interface.registerHandle(pos_vel_handle_4);

   hardware_interface::PosVelJointHandle pos_vel_handle_5(jnt_state_interface.getHandle("joint5"), &cmd_pos[4], &cmd_vel[4]);
   jnt_pos_vel_interface.registerHandle(pos_vel_handle_5);

   hardware_interface::PosVelJointHandle pos_vel_handle_6(jnt_state_interface.getHandle("joint6"), &cmd_pos[5], &cmd_vel[5]);
   jnt_pos_vel_interface.registerHandle(pos_vel_handle_6);

   registerInterface(&jnt_pos_vel_interface);

   
 }

  void write(){
    std::cout<<"Writing Positions: " << cmd_pos[0] << ", " << cmd_pos[1] << ", " << cmd_pos[2] << "\n";
    std::cout<<"Writing Velocities: " << cmd_vel[0] << ", " << cmd_vel[1] << ", " << cmd_vel[2] << "\n";
  }
   
  void read(){
    std::cout<<"Reading.\n";
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PosVelJointInterface jnt_pos_vel_interface;
  double cmd_pos[2];
  double cmd_vel[2];
  double pos[2];
  double vel[2];
  double eff[2];
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_grarm");

  MyRobot robot;
  controller_manager::ControllerManager cm(&robot);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(10.0); // 10 Hz rate
  
  while (ros::ok())
  {
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    
    robot.read();
    cm.update(time,period);
    robot.write();
    rate.sleep();
  }
  return 0;
}
