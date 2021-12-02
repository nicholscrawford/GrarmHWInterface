#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/posvel_command_interface.h>
#include <iostream>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <CppLinuxSerial/SerialPort.hpp>
#include <math.h>
#include <algorithm>
#include <cassert>
#include <cstring>
#include <stdlib.h>     /* abs */


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

   mn::CppLinuxSerial::SerialPort serialPortA("/dev/ttyACM0", mn::CppLinuxSerial::BaudRate::B_9600);
   serialPortA.SetTimeout(10);
   serialPortA.Open();

   // mn::CppLinuxSerial::SerialPort serialPortB("/dev/ttyUSB0", mn::CppLinuxSerial::BaudRate::B_9600);
   // serialPort.SetTimeout(-1);
   // serialPort.Open();

   
   double cmd_pos[6];
   double cmd_vel[6];

   for(int i = 0; i < 6; i++){
     cmd_pos[i] = 0;
     cmd_vel[i] = 0;
   }			    
   
 }

  void write(mn::CppLinuxSerial::SerialPort &serialPortA){
    int gearboxRatio = 25;
    double inverseGR = 0.04;
    double radToRotation = 0.159154943091;
    double rotationToRad = 6.2831853071796;
    int pos_pos = 12;
    int vel_pos = 20;
    int torque_pos = 28;
    //Print commanded positions
    std::cout << "POSITIONS: " << cmd_pos[1] << ", " << cmd_pos[2] << ", " << cmd_pos[3] << ", "  << cmd_pos[4] << ", "  << cmd_pos[5] <<"\n";
    std::cout << "VELOCITY: " << cmd_vel[1] << ", " << cmd_vel[2] << ", " << cmd_vel[3] << ", "  << cmd_vel[4] << ", "  << cmd_vel[5] << "\n";
    
    //For each joint in serial unit, temporarily incremented to the second joint
    for(int i = 1; i<=1; i++){
      //Get commanded position,
      //Adjust units, compensate for gearbox, and LSB Val
      long long current_cmd_pos = cmd_pos[i] * gearboxRatio * radToRotation * 100000;

      //Convert to hex, LSB first
      std::string hexPos = LSBSwitch(DecToHex(current_cmd_pos));

      //Get commanded velocity,
      //Adjust units, compensate for gearbox and LSB val
      //long long current_cmd_vel = cmd_vel[i] * gearboxRatio * radToRotation * 100000;
      long long current_cmd_vel = cmd_vel[i] * gearboxRatio * radToRotation * 100000;

      //Convert to hex, LSB first
      std::string hexVel = LSBSwitch(DecToHex(current_cmd_vel));
      //Build then send command
      //Position Sent is stop position command
      //std::string cmd = "can send 800" + std::to_string(i) + " 01000a0a20" + "00000080" + hexVel + "0926" + hexPos +"1b011100" + "\n";
      //Position sent is current pos, no stop pos.
      std::string cmd = "can send 800" + std::to_string(i) + " 01000a0a20" + hexPos + hexVel +"1b011100" + "\n";
      std::cout<<"Writing cmd: " << cmd << "\n";
      if(i <= 3){
	serialPortA.Write(cmd);
      }
      else{
	//serialPortB.Write(cmd);
      }
      ///FOR DEBUGGING/////
      if(cmd_pos[i] > 6.3 || cmd_vel[i] > 6.3){
	std::cout << "CMD POS OR VEL TOO LARGE!!! POS: " << cmd_pos[i] << " VEL: " << cmd_vel[i] << "\n";
	pos[i] = 0;
	vel[i] = 0;
	cmd_pos[i] = 0;
	cmd_vel[i] = 0;
      }
      // else{
      // 	serialPortB.Write(cmd);
      // }
      cmd_prevpos[i] = hexPos;
      // Read some data back (will block until at least 1 byte is received due to the SetTimeout(-1) call above)
      std::string readData;
      if(i <= 3){
	while(true){
	  serialPortA.Read(readData);
	  // std::cout<<"READ: " <<readData<< "\n";
	  if(readData.find("rcv") != std::string::npos){
	    readData = readData.substr(readData.find("rcv"), readData.length());
	    break;
	  }
	}
      }
      else{
	while(true){
	  //serialPortB.Read(readData);
	  //std::cout<<"READ: " <<readData<< "\n";
	  if(readData.find("rcv") != std::string::npos){
	    readData = readData.substr(readData.find("rcv"), readData.length());
	    break;
	  }
	}
      }
      // std::cout << "####Data Out####\n";
      // std::cout << readData << "\n";
      // else{
      // 	serialPortB.Write(cmd);
      // }
      // cout <<"Reading: "<< readData << "\n";
      // For given set of pos/vel/torque
      // Convert from lsb first to regular hex
      // convert to dec,
      // adjust for gearbox and lsb val
      // store
      std::string read_hexPos = readData.substr(pos_pos, 8);
      // std::cout << "Motor's Reported Position: " << HexToDec(LSBSwitch(read_hexPos)) << "\n";
      // std::cout <<"Output Position: " << HexToDec(LSBSwitch(read_hexPos)) * 0.00001 * inverseGR * rotationToRad << "\n";
      pos[i] = HexToDec(LSBSwitch(read_hexPos)) * 0.00001 * inverseGR * rotationToRad;
      std::string read_hexVel = readData.substr(vel_pos, 8);
      // std::cout << "Motor's Reported Velocity: " << HexToDec(LSBSwitch(read_hexVel)) << "\n";
      // std::cout <<"Output Velocity: " << HexToDec(LSBSwitch(read_hexPos)) * 0.00001 * inverseGR * rotationToRad << "\n";
      vel[i] = HexToDec(LSBSwitch(read_hexVel)) * 0.00001 * inverseGR * rotationToRad;
      std::string read_hexTorque = readData.substr(torque_pos, 8);
      // std::cout << "Motor's Reported Torque: " << HexToDec(LSBSwitch(read_hexTorque)) << "\n";
      // std::cout <<"Output Torque: " << HexToDec(LSBSwitch(read_hexTorque)) * 0.001 * 25 << "\n";
      eff[i] = HexToDec(LSBSwitch(read_hexTorque)) * 0.001 * 25;
 
    }
  }
   
  std::string DecToHex(long long dec)
  {
    long long n = dec;
    // algorithm from geeks for geeks: https://www.geeksforgeeks.org/program-decimal-hexadecimal-conversion/
	
    // convert dec 

    // 8 hex digits
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

    // make sure there is enough zeros at the end
    switch(finalString.length()){
    case(0):
      finalString = "00000000";
      break;
    case(1):
      finalString = "0000000" + finalString;
      break;
    case(2):
      finalString = "000000" + finalString;
      break;
    case(3):
      finalString = "00000" + finalString;
      break;
    case(4):
      finalString = "0000" + finalString;
      break;
    case(5):
      finalString = "000" + finalString;
      break;
    case(6):
      finalString = "00" + finalString;
      break;
    case(7):
      finalString = "0" + finalString;
      break;
    case(8):
      break;
	  
    default:
      //std::cout << "Length: " << finalString.length() << "\n";
      //std::cout << "String: " << finalString << "\n";
      return "00000000";
      assert(finalString.length() <= 8);
      break;
    }

    return finalString; 
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

  std::string LSBSwitch(std::string inputString){
    assert(inputString.length() == 8);

    char temp1;
    int j = 8;
    for(int i = 0; i < 4; i+=2){
      char temp1 = inputString[i];
      char temp2 = inputString[i+1];
      inputString[i] = inputString[6-i];
      inputString[i+1] = inputString[7-i];
      inputString[6-i] = temp1;
      inputString[7-i] = temp2;
    }
    return inputString;
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PosVelJointInterface jnt_pos_vel_interface;
  double cmd_pos[6];
  double cmd_vel[6];
  double pos[6];
  double vel[6];
  double eff[6];
  std::string cmd_prevpos[6];
  mn::CppLinuxSerial::SerialPort serialPortA;
  // mn::CppLinuxSerial::SerialPort serialPortB
  ;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grarm_hw_interface");

  MyRobot robot;
  controller_manager::ControllerManager cm(&robot);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(400.0); // 30 Hz rate

  mn::CppLinuxSerial::SerialPort serialPortA("/dev/ttyACM0", mn::CppLinuxSerial::BaudRate::B_9600);
  //mn::CppLinuxSerial::SerialPort serialPortB("/dev/ttyACM1", mn::CppLinuxSerial::BaudRate::B_9600);
  serialPortA.SetTimeout(1000);
  //serialPortB.SetTimeout(1000);
  serialPortA.Open();
  //serialPortB.Open();

  while (ros::ok())
  {
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    
    cm.update(time,period);
    robot.write(serialPortA);
    rate.sleep();
  }
  return 0;
}
