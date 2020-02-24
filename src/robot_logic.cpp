l/*
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
#define nbBoutons 11
#define nbAxes 8

#define boutonA 0
#define boutonB 1
#define boutonX 2
#define boutonY 3
#define boutonLB 4
#define boutonRB 5
#define boutonBack 6
#define boutonStart 7
#define boutonLogitech 8
#define boutonJoyGauche 9
#define boutonJoyDroit 10

#define axeHorizJoyGauche 0
#define axeVertJoyGauche 1
#define boutonLT 2
#define axeHorizJoyDroit 3
#define axeVertJoyDroit 4
#define boutonRT 5
#define croixHoriz 6
#define croixVert 7

#define allume 0
#define eteint 1
#define singleLeg 2
#define marche 3

void freeServos();
void startServos();


int testVal = 0;
int prevButton = 0;
int modeAuto = 0;
int etape = 0;
int animation = 0;
int singleLegI = 0;
float testAnalog = 0.0f;
float testAnalog2 = 0.0f;
float distanceTest_ = 0.0f;
std_msgs::String message;
int etat = 0;
bool aEnvoye = true;
int valPrec = 0;
int compteur;
int noPatte = 0;
int positionHanche = 0;
int positionGenou = 0;
int positionPatte = 0;
float tabAxes[8];
float tabBoutons[11];
int tabAEnvoye[72];
int tabServos [4][18] = {{0,1,2,4,5,6,8,9,10,16,17,18,20,21,22,24,25,26},
						 {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
						 {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
						 {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};


int iMarche = 0s;
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

  for(int i = 0; i < nbBoutons; i++)
  {
  	tabBoutons[i] = msg->buttons[i];	//Pour pouvoir acceder a la valeur des boutons dans les mouvements
  } 

  for(int i = 0; i < nbAxes; i++)
  {
  	tabAxes[i] = msg->axes[i];	//Pour pouvoir acceder a la valeur des axes dans les mouvements
  } 

  //modeAuto = tabBoutons[boutonBack];

  switch(etat)
  {
  	case allume:
  		if(tabBoutons[boutonStart])	
  		{
  			etat = eteint;	//Envoie l'innstruction d'eteindre l'araignee
  			aEnvoye = true;
  		}
  		if(tabBoutons[boutonY])	
  		{
  			noPatte = 1;
  			etat = singleLeg;
  			positionHanche = 1500;
  			positionGenou = 1500;
  			positionPatte = 1500;
  			aEnvoye = true;
  		}
		if(tabBoutons[boutonA])
		{
			etat = marche;
			iMarche = 0;
		}
  		break;

  	case singleLeg:
		if(tabBoutons[boutonBack])
		{
			startServos();
			singleLegI = 0;			
  			aEnvoye = true;
			noPatte++;
			if(noPatte > nbPattes)
			{
				noPatte = 1;	//Revient a 1 apres 6
			}

			if((noPatte <= 3) && (noPatte >= 1))
			{
				positionHanche = 1500;
	  			positionGenou = 1850;
	  			positionPatte = 1500;
			}
			else if((noPatte <= 6) && (noPatte >= 4))
			{
				positionHanche = 1500;
	  			positionGenou = 1150;
	  			positionPatte = 1500;
			}
		}
		if(tabBoutons[boutonStart])	
  		{
  			etat = eteint;	//Envoie l'innstruction d'eteindre l'araignee
  			aEnvoye = true;
  		}
		break;

  	case eteint:
  		if(tabBoutons[boutonStart])
  		{
  			etat = allume;
  			aEnvoye = true;
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
}

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
  ros::Rate loop_rate(80); // value in Hz for the loop
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

    
	switch(etat)	//Mouvements a effectuer selon l'etat
	{
		case allume:	//L'araignee est en fonction
    		{
    			for(int i = 0; i  < nbServos; i++)
    			{
    				tabServos[1][i] = 1500;	//Met l'araignee en position de base
    			}
    		}
    		break;

    	case eteint:
    		{
    			for(int i = 0; i  < nbServos; i++)
    			{
    				tabServos[1][i] = 0;	//Eteint tous les servos
    			}
    		}
    		break;

    	case singleLeg:
    		{
    			singleLegI++;

    			if(singleLegI == 15)
    			{
    				positionHanche = 1500;
    				positionGenou = 1500;
    				positionPatte = 1500;
    				aEnvoye = true;
    			}

    			if(tabAxes[axeHorizJoyGauche] != 0)
				{
					positionHanche = positionHanche - (tabAxes[axeHorizJoyGauche]*10);

					if(positionHanche <= 750)
					{
						positionHanche = 750;
					}
					if(positionHanche >= 2250)
					{
						positionHanche = 2250;
					}
					
					aEnvoye = true;
				}

				if((tabAxes[axeVertJoyGauche] != 0) && (noPatte <= 3) && (noPatte >= 1))
				{
					positionGenou = positionGenou + (tabAxes[axeVertJoyGauche]*10);

					if(positionGenou <= 750)
					{
						positionGenou = 750;
					}
					if(positionGenou >= 2250)
					{
						positionGenou = 2250;
					}
					
					aEnvoye = true;
				}

				else if ((tabAxes[axeVertJoyGauche] != 0) && (noPatte <= 6) && (noPatte >= 4))
				{
					positionGenou = positionGenou - (tabAxes[axeVertJoyGauche]*10);

					if(positionGenou <= 750)
					{
						positionGenou = 750;
					}
					if(positionGenou >= 2250)
					{
						positionGenou = 2250;
					}
					
					aEnvoye = true;
				}

				if((tabAxes[axeVertJoyDroit] != 0) && (noPatte <= 3) && (noPatte >= 1))
				{
					positionPatte = positionPatte - (tabAxes[axeVertJoyDroit]*10);

					if(positionPatte <= 750)
					{
						positionPatte = 750;
					}
					if(positionPatte >= 2250)
					{
						positionPatte = 2250;
					}
					
					aEnvoye = true;
				}

				else if((tabAxes[axeVertJoyDroit] != 0) && (noPatte <= 6) && (noPatte >= 4))
				{
					positionPatte = positionPatte + (tabAxes[axeVertJoyDroit]*10);

					if(positionPatte <= 750)
					{
						positionPatte = 750;
					}
					if(positionPatte >= 2250)
					{
						positionPatte = 2250;
					}
					
					aEnvoye = true;
				}

    			switch(noPatte)
    			{
    				case 1:
						tabServos[1][0] = positionHanche;
						tabServos[1][1] = positionGenou;
						tabServos[1][2] = positionPatte;
						break;
    					
    				case 2:
    					tabServos[1][3] = positionHanche;
						tabServos[1][4] = positionGenou;
						tabServos[1][5] = positionPatte;
						break;

    				case 3:
    					tabServos[1][6] = positionHanche;
						tabServos[1][7] = positionGenou;
						tabServos[1][8] = positionPatte;
						break;

    				case 4:
    					tabServos[1][9] = positionHanche;
						tabServos[1][10] = positionGenou;
						tabServos[1][11] = positionPatte;
						break;

    				case 5:
    					tabServos[1][12] = positionHanche;
						tabServos[1][13] = positionGenou;
						tabServos[1][14] = positionPatte;
						break;

    				case 6:
    					tabServos[1][15] = positionHanche;
						tabServos[1][16] = positionGenou;
						tabServos[1][17] = positionPatte;
						break;
    			}
    		}

	}

	if(aEnvoye)
	{
		string serie = "";

		for(int i = 0; i < nbServos; i++)
		{
			serie = serie + "#" + to_string(tabServos[0][i]) + "P" + to_string(tabServos[1][i]);
		}
		serie = serie + "\r";
		message.data = serie;
		ROS_INFO("%s\n", message.data.c_str());
	  	serial.publish(message);
	  	aEnvoye = false;  
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
  	aEnvoye = true;
  }
}




