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
#include "serial/serial.h" //bibliotheque pour la communication serie.
#include <sstream>



//PROTOTYPES-------------------------------------
void Callback(const std_msgs::String::ConstPtr& msg);

//GLOBALE-------------------------------------------
int[] tableau1D = new int[72];

int main(int argc, char **argv)
{
  //INITIALISATION DE ROS---------------------------
  ros::init(argc, argv, "serial");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("serial_topic", 1000, Callback);
  ros::Rate loop_rate(10);
  
  serial::Serial my_serial("/dev/ttyUSB0", 9600, serial::Timeout::simpleTimeout(1000)); //ouverture du port série.
  
  while (ros::ok()) 
  {
    ros::spinOnce();

    loop_rate.sleep();
   
  }
  return 0;
}

void Callback(const std_msgs::String::ConstPtr& msg)
{
   msg->data
}