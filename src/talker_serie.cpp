/**
 * @file talker_serie.cpp
 * @author Antoine Leroux (antoineLeroux31@gmail.com)
 * @date 12-nov 2020
 * 
 * @brief node pour envoyé les positions de servo en série au SSC32.
 * On recoit un tableau qui contient les positions, vitesses et temps de
 * chaque patte, on construit les commandes et on les envois.
 * 
 * @version 1.0: Premiere version
 * 
 * environement : ROS Melodic
 * Compilateur: Catkin_make
 * Matériel: Phoenix SS32U Servo Controller
 * */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16MultiArray.h"
#include "serial/serial.h" //bibliotheque pour la communication serie.
#include <string.h>
#include <iostream>
#include <sstream>

#define NB_SERVO 18
#define NB_PARAM 4

//PROTOTYPES-------------------------------------
void Callback(const std_msgs::UInt16MultiArray::ConstPtr& msg);
//GLOBALE-------------------------------------------
std_msgs::UInt16MultiArray tab;
int tab2d[4][18];
std::string result = "";



int main(int argc, char **argv)
{
  //INITIALISATION DE ROS---------------------------
  ros::init(argc, argv, "serial");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("serial_topic", 1000, Callback);
  ros::Rate loop_rate(10);
  
  //serial::Serial my_serial("/dev/ttyUSB0", 9600, serial::Timeout::simpleTimeout(1000)); //ouverture du port série.

  while (ros::ok()) 
  {
    ros::spinOnce();

    loop_rate.sleep();
   
  }
  return 0;
}

void Callback(const std_msgs::UInt16MultiArray::ConstPtr& msg)
{
  int i = 0;
  /*for(std::vector<int>::const_iterator j = msg->data[0].begin();j != msg->data[0].end(); ++j)
  {
    tab2d[i] = *j;
    i++;
  }*/
  for(int i = 0; i < NB_SERVO;i++)
  {
    result = result + "#" + std::to_string(msg->data[0,i]) + "P" + std::to_string(msg->data[1,i]);
  }
  ROS_INFO("%s\n",result.c_str());
}
