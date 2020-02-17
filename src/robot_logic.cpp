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

#define nbServos 18
#define nbPattes 6

#define allume 0
#define eteint 1
#define singleLeg 2

void freeServos();
void startServos();


int testVal = 0;
int enMarche = 1;
int prevButton = 0;
int modeAuto = 0;
int etape = 0;
int animation = 0;
float testAnalog = 0.0f;
float testAnalog2 = 0.0f;
float distanceTest_ = 0.0f;
std_msgs::String message;
int etat = 0;
bool aEnvoye = true;
int valPrec = 0;
int compteur;
int noPatte = 1;
int tabAEnvoye[72];
int tabServos [4][18] = {{0,1,2,4,5,6,8,9,10,16,17,18,20,21,22,24,25,26},
						 {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
						 {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
						 {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};



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
  if(msg->buttons[7])
  	enMarche= !enMarche;

  modeAuto = msg->buttons[8];

  switch(etat)
  {
  	case allume:
  		if(msg->buttons[7])
  		{
  			etat = eteint;
  			aEnvoye = true;
  			//enMarche = false;
  		}
  		if(msg->buttons[4])
  		{
  			etat = singleLeg;
  			startServos();
  		}
  		break;

  	case eteint:
  		if(msg->buttons[7])
  		{
  			etat = allume;
  			aEnvoye = true;
  			//enMarche = true;
  		}
  		break;
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

  testAnalog = msg->axes[0];
  testAnalog2 = msg->axes[1];
}


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
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter_logic", 1000);
  //ros::Publisher myJoy_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Publisher serial = n.advertise<std_msgs::String>("serial_topic", 1000);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(20); // value in Hz for the loop
// %EndTag(LOOP_RATE)%

// SECTION THAT DEFINES THE INFORMATIONS THAT WE WANT TO LISTEN TO
ros::Subscriber subJoy   = n.subscribe("joy", 10, joyCallback);
//ros::Subscriber subLaser = n.subscribe("base_scan", 10, scanCallback);
//ros::Subscriber subPose = n.subscribe("base_pose_ground_truth", 10, positionCallback);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  
// information structures that will be used to send for the different messages
  std_msgs::String msg;
  geometry_msgs::Twist base_cmd;

  

  while (ros::ok())
  {  
  	int k = 0;

    if(enMarche)
    {
    	switch(etat)
    	{
    		case allume:
	    		{
	    			for(int i = 0; i  < nbServos; i++)
	    			{
	    				tabServos[1][i] = 1500;
	    			}
	    		}
	    		break;

	    	case eteint:
	    		{
	    			for(int i = 0; i  < nbServos; i++)
	    			{
	    				tabServos[1][i] = 0;
	    			}
	    		}
	    		break;

	    	case singleLeg:
	    		{
	    			/*if(msg->buttons[8])
	    			{
	    				noPatte++;
	    				if(noPatte > nbPattes)
	    				{
	    					noPatte = 1;
	    				}
	    			}

	    			switch(noPatte)
	    			{
	    				case 1:
	    					if(msg->axes[0] != 0)
	    					{

	    					}
	    				case 2:

	    				case 3:

	    				case 4:

	    				case 5:

	    				case 6:

	    			}*/
	    		}

    	}

    	for(int i = 0; i < nbServos; i++)
    	{
    		for(int j = 0; j < 4; j++)
    		{    			
    			tabAEnvoye[k] = tabServos[j][i];
    			k++;
    		}
    	}
    }
    
    //ROS_INFO("enMArche =%d", valPrec);
  
  
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
      //chatter_pub.publish(msg);
      //myJoy_pub.publish(base_cmd);

      /*if(aEnvoye)
      {
      	ROS_INFO("message = %s\r", message.data.c_str());
      	serial.publish(tabAEnvoye);
      	aEnvoye = false;  
      }*/
      

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    
  }

  return 0;
}

/*void freeServos()
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
  //return serie;
}*/

void startServos()
{
  for (int i = 0; i < nbServos; i++)
  {
  	tabServos[1][i] = 1500;
  }
}




