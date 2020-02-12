/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h" // for yaw computation
#include <math.h> // for M_PI
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <string>

using namespace std;

// %EndTag(MSG_HEADER)%

#include <sstream>

#undef Manette
#define Manette

/** max value for this robot is 500 mm/sec */
#define ROBOT_LINEAR_MAX_SPEED 0.5f

/** max value for this robot is 0.25 rad/sec => 360 degrés = 2*PI (radian)*/
#define ROBOT_ROTATION_MAX_SPEED 0.25

int testVal = 0;
int enMarche = 1;
int prevButton = 0;
int testLaser = 0;
int laserOn = 1;
int changeVitesse = 0;
int vitPrec = 0;
int modeAuto = 0;
int etape = 0;
int animation = 0;
float vitesse = ROBOT_LINEAR_MAX_SPEED;
float vingPourcent = (ROBOT_LINEAR_MAX_SPEED *20/100);
float testAnalog = 0.0f;
float testAnalog2 = 0.0f;
float distanceTest_ = 0.0f;
std_msgs::String message;
bool test = true;
int valPrec = 0;
int compteur;

string freeServos()
{
  string serie("");
  string patte("");


  for(int i = 0; i < 32; i++)
  {
    patte = to_string(i);
    serie += "#" + patte + "P0";
    //ROS_INFO("%s\n", serie.c_str());
  }

  serie += "\r";
  return serie;
}

string startServos()
{
  std_msgs::String msg;

  msg.data = "#0P1500#1P1500#2P1500#4P1500#5P1500#6P1500#8P1500#9P1500#10P1500#16P1500#17P1500#18P1500#20P1500#21P1500#22P1500#24P1500#25P1500#26P1500\r";

  return msg.data;
}

/*void modeAutomatique()
{
  switch(animation)
  {
    case 0:
      swith(etape)
      {
        case 0:
        etape++;
        break;

        case 1:
        etape++;
        break;

        case 2:
        etape++:
        break;
      }
      break;

    case 1:
      swith(etape)
      {
        case 0:
        etape++;
        break;

        case 1:
        etape++;
        break;

        case 2:
        etape++:
        break;
      }
      break;
        
  }

}*/

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{  
  #ifdef Manette

  testVal = msg->buttons[7];
  testLaser = msg->buttons[6];
  modeAuto = msg->buttons[8];

  if(testVal)
  {
    enMarche = !enMarche;
    test = true;
  }

  if(testLaser)
  {
    laserOn = !laserOn;
  }

  /*if(modeAuto)
  {
    modeAuto = !modeAuto;
    if(modeAuto)
    {
      modeAutomatique();
    }
    else
    {
      startServos();
    }
  }*/

  #else
  //ROS_INFO("Joy callback");//: [%s]", msg->data.c_str());
  testVal = msg->buttons[0];
    testLaser = msg->buttons[1];  //Recoit si le laser est actif ou non
    changeVitesse = msg->buttons[2];  //Recoit s'il y a changement de vitesse 

  #endif

  testAnalog = msg->axes[0];
  testAnalog2 = msg->axes[1];
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{  
  //ROS_INFO("Scan callback %f %f %f",msg->angle_min,msg->angle_max,msg->angle_increment);//: [%s]", msg->data.c_str());
  distanceTest_ = msg->ranges[90]; // valeur devant le robot // 1080 valeur max avec le capteur Stage
  //ROS_INFO("Obstacle distance = %f", distanceTest_);
}

/*
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities

*/

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  float yaw = 0.0f;
  //ROS_INFO("Test position x=%f" , msg->pose.pose.position.x);
  yaw = tf::getYaw(msg->pose.pose.orientation);
  
  //ROS_INFO("Yaw position x=%f" ,(yaw*180.0f)/M_PI);
}
/*
geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
*/
/* variables used to define the command values to be sent out */
float linearSpeedCmd_ = 0;
float rotationSpeedCmd_ = 0;



/**
 * Main loop of the node.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "logic_block"); // it's the name that will be used for the node!
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%

// SECTION THAT DEFINES THE INFORMATIONS THAT OUR NODE WILL PUBLISH
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter_logic", 1000);
  ros::Publisher myJoy_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Publisher serial = n.advertise<std_msgs::String>("serial_topic", 1000);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(20); // value in Hz for the loop
// %EndTag(LOOP_RATE)%

// SECTION THAT DEFINES THE INFORMATIONS THAT WE WANT TO LISTEN TO
ros::Subscriber subJoy   = n.subscribe("joy", 10, joyCallback);
ros::Subscriber subLaser = n.subscribe("base_scan", 10, scanCallback);
ros::Subscriber subPose = n.subscribe("base_pose_ground_truth", 10, positionCallback);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  
// information structures that will be used to send for the different messages
  std_msgs::String msg;
  geometry_msgs::Twist base_cmd;
  int test123 = 123;

  

  while (ros::ok())
  {  
    /*if(modeAuto)
    {
      modeAutomatique();
    }*/

    if(changeVitesse != vitPrec)
    {
      if(changeVitesse == -1)
      {
        vitesse = vitesse - vingPourcent;
        if(vitesse <= 0)
          vitesse = 0;
      }
      else if(changeVitesse == 1)
      {
        vitesse = vitesse + vingPourcent;
        if(vitesse >= ROBOT_LINEAR_MAX_SPEED)
        {
          vitesse = ROBOT_LINEAR_MAX_SPEED;
        }
      }
    }
    vitPrec = changeVitesse;

    //ROS_INFO("vitesse =%f", vitesse);
    if(enMarche > valPrec)// test for dead man switch
    {
      message.data = startServos();
      if((distanceTest_ < 0.5f) && (testAnalog2 > 0.0f) && laserOn)    //Partie qui regarde si le robot approche un mur si le laser est active
      linearSpeedCmd_ = 0;
      else
      linearSpeedCmd_ = (testAnalog2 * ROBOT_LINEAR_MAX_SPEED);
      
      base_cmd.linear.x = linearSpeedCmd_;
      
      rotationSpeedCmd_ = (testAnalog * ROBOT_ROTATION_MAX_SPEED); 
      base_cmd.angular.z = rotationSpeedCmd_;
    }

    else
    {
      base_cmd.linear.x = 0.0f;
      base_cmd.angular.z = 0.0f;
      message.data = freeServos();
    }
    valPrec = enMarche;
    //ROS_INFO("enMArche =%d", valPrec);
  
  
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
      chatter_pub.publish(msg);
      myJoy_pub.publish(base_cmd);

      if(test)
      {
      	ROS_INFO("message = %s\r", message.data.c_str());
      	serial.publish(message);
      	test = false;  
      }
      

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    
  }

  return 0;
}


