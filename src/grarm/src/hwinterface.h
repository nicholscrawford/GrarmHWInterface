#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/posvel_command_interface.h>
#include <iostream>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <CppLinuxSerial/SerialPort.hpp>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot() 
 { 
   // connect and register the joint state interface
   //hardware_interface::JointStateHandle state_handle_1("joint1", &pos[0], &vel[0], &eff[0]);
   //jnt_state_interface.registerHandle(state_handle_1);

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
   //hardware_interface::PosVelJointHandle pos_vel_handle_1(jnt_state_interface.getHandle("joint1"), &cmd_pos[0], &cmd_vel[0]);
   //jnt_pos_vel_interface.registerHandle(pos_vel_handle_1);

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

   mn::CppLinuxSerial::SerialPort serialPortA("/dev/ttyUSB0", mn::CppLinuxSerial::BaudRate::B_9600);
   serialPort.SetTimeout(-1);
   serialPort.Open();

   // mn::CppLinuxSerial::SerialPort serialPortB("/dev/ttyUSB0", mn::CppLinuxSerial::BaudRate::B_9600);
   // serialPort.SetTimeout(-1);
   // serialPort.Open();
 }

  void write(){
    // std::cout<<"Writing Positions: " << cmd_pos[0] << ", " << cmd_pos[1] << ", " << cmd_pos[2] << "\n";
    // std::cout<<"Writing Velocities: " << cmd_vel[0] << ", " << cmd_vel[1] << ", " << cmd_vel[2] << "\n";

    //For each joint in serial unit,
    
    //Get commanded position,
    //Adjust units, compensate for gearbox, and LSB Val
    //Convert to hex, LSB first
    //Get commanded velocity,
    //Adjust units, compensate for gearbox and LSB val
    //Convert to hex, LSB first
    //Build then send command
    serialPortA.Write("can send 8001 ...");

    // Read some data back (will block until at least 1 byte is received due to the SetTimeout(-1) call above)
    std::string readData;
    serialPortA.Read(readData);

    // For given set of pos/vel/torque
    // Convert from lsb first to regular hex
    // convert to dec,
    // adjust for gearbox and lsb val
    // store
  }
   
  // void read(){
  //   std::cout<<"Reading.\n";
  // }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PosVelJointInterface jnt_pos_vel_interface;
  double cmd_pos[6];
  double cmd_vel[6];
  double pos[6];
  double vel[6];
  double eff[6];
  mn::CppLinuxSerial::SerialPort serialPortA;
  // mn::CppLinuxSerial::SerialPort serialPortB
  ;
};

std::string DecToHex(long long dec)
{
	long long n = dec;
	// algorithm from geeks for geeks: https://www.geeksforgeeks.org/program-decimal-hexadecimal-conversion/
	
	// convert dec 

	// convert to 2's complement form
	if (n < 0L)
	{
		n = n + 4294967296L;
	}

	// char array to store hexadecimal number 
	char hexaDeciNum[100];

	// counter for hexadecimal number array 
	int i = 0;
	while (n != 0)
	{
		// temporary variable to store remainder 
		int temp = 0;

		// storing remainder in temp variable. 
		temp = n % 16;

		// check if temp < 10 
		if (temp < 10)
		{
			hexaDeciNum[i] = temp + 48;
			i++;
		}
		else
		{
			hexaDeciNum[i] = temp + 55;
			i++;
		}

		n = n / 16;
	}

	std::string finalString = "";

	// printing hexadecimal number array in reverse order 
	for (int j = i - 1; j >= 0; j--)
	{
		finalString += hexaDeciNum[j];
	}

	if (finalString.size() < 8)
	{
		for (; finalString.size() - 8 > 0; )
		{
			finalString = finalString + '0';
		}
	}

	if (n < 0L)
	{
		std::reverse(finalString.begin(), finalString.end());
	}

	// swaps the chars

	for (int l = 0; l < 8; l++)
	{
		if (l % 2 == 0)
		{
			char temp = finalString[l];
			finalString[l] = finalString[l + 1];
			finalString[l + 1] = temp;
		}
	}

	return finalString;
}

int HexCharToDecVal(char hexChar)
{
	switch (hexChar)
	{
	case ('0'):
		return 0;
	case ('1'):
		return 1;
	case ('2'):
		return 2;
	case ('3'):
		return 3;
	case ('4'):
		return 4;
	case ('5'):
		return 5;
	case ('6'):
		return 6;
	case ('7'):
		return 7;
	case ('8'):
		return 8;
	case ('9'):
		return 9;
	case ('A'):
		return 10;
	case ('B'):
		return 11;
	case ('C'):
		return 12;
	case ('D'):
		return 13;
	case ('E'):
		return 14;
	case ('F'):
		return 15;
	default:
		return 0;
	}
}

long long HexToDec(std::string hexString)
{
	int stringSize = hexString.length();
	long long total = 0;
	int j = 0;
	for (int i = stringSize - 1; i >= 0; i--)
	{
	  char currentChar = hexString[j];
	  total += pow(16, i) * HexCharToDecVal(currentChar);
	  j++;
	}

	// for 8 chars
	// convert from 2s complement -1
	if (total > 2147483648L)
	{
		total -= 4294967296L;
	}

	return total;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grarm_hw_interface");

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
    
    // robot.read();
    cm.update(time,period);
    robot.write();
    rate.sleep();
  }
  return 0;
}
