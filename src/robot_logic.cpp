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
#define nbBoutons 11
#define nbAxes 8
#define nbAnimations 3

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

#define marche 0
#define eteint 1
#define singleLeg 2
#define automatique 3

#define nbAutomatismes 3

#define toctoc 0
#define cancan 1
//#define twist 2
#define pause 2

#define delai1 20
#define delai2 40
#define delai3 60
#define delai4 80
//#define delai5 100
//#define delai6 120

void freeServos();
void startServos();


int testVal = 0;
int prevButton = 0;
int etape = 0;
int animation = 0;
int delaiI = 0;
int repetition = 0;
int delaiFermer = 0;
std_msgs::String message;
int etat = eteint;
int quelAuto = toctoc;
int envoiUneFois = 0;
bool aEnvoye = true;
int compteur;
int noPatte = 0;
int positionHanche = 0;
int positionGenou = 0;
int positionPatte = 0;
int servoT = 0;
float tabAxes[8];
float tabBoutons[11];
int tabAEnvoye[72];
int tabServos [4][18] = {{0,1,2,4,5,6,8,9,10,16,17,18,20,21,22,24,25,26},
						 {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
						 {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
						 {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};




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
 
  delaiFermer = 0;	//Repart le timer d'inactivite

  switch(etat)
  {
  	case marche:
  		if(tabBoutons[boutonStart])	
  		{
  			etat = eteint;	//Envoie l'instruction d'eteindre l'araignee
  			aEnvoye = true;
  		}

  		if(tabBoutons[boutonY])	
  		{
  			noPatte = 1;
  			etat = singleLeg;
  			delaiFermer = 0;	//Repart le timer d'inactivite
  		}

  		if((tabAxes[boutonLT] == -1) && (tabAxes[boutonRT] == -1))
  		{
  			repetition = 0;
  			etat = automatique;
  			etape = 0;
  			quelAuto = toctoc;
  			delaiFermer = 0;	//Repart le timer d'inactivite
  		}
  		break;

  	case singleLeg:
		if(tabBoutons[boutonBack])	//Change de patte a controle
		{
			startServos();
			delaiFermer = 0;	//Repart le timer d'inactivite
			delaiI = 0;	//Repart le compteur entre les etapes
			noPatte++;
			if(noPatte > nbPattes)
			{
				noPatte = 1;	//Revient a 1 apres 6
			}

			//Fait monter la patte selectionnee, elle redescend au debut du case singleLeg de l'autre machine a etat
			if((noPatte <= 3) && (noPatte >= 1))
			{
				positionHanche = 1500;
	  			positionGenou = 1850;
	  			positionPatte = 1500;
			}

			//Fait monter la patte selectionnee, elle redescend au debut du case singleLeg de l'autre machine a etat
			else if((noPatte <= 6) && (noPatte >= 4))
			{
				positionHanche = 1500;
	  			positionGenou = 1150;
	  			positionPatte = 1500;
			}
		}

		if((tabAxes[boutonLT] == -1) && (tabAxes[boutonRT] == -1))	//Met l'araigne en mode automatique
  		{
  			repetition = 0;
  			etat = automatique;
  			etape = 0;	//Commence au debut de l'automatisme
  			quelAuto = toctoc;	//Commence au premier automatisme
  			delaiFermer = 0;	//Repart le timer d'inactivite
  		}

		if(tabBoutons[boutonStart])		//Eteint l'araignee
  		{
  			etat = eteint;	//Envoie l'innstruction d'eteindre l'araignee
  			aEnvoye = true;
  		}

		if(tabBoutons[boutonA])	//Met l'araignee en mode marche
		{
			etat = marche;
			etape = 0;
		}

	case automatique:
		{
			if(tabBoutons[boutonBack])
			{
				startServos();	//Remet les servos a 1500
				delaiFermer = 0;	//Repart le timer d'inactivite
				repetition = 0;
				quelAuto++;
				etape = 0;
				if (quelAuto >= nbAutomatismes)
				{
					quelAuto = toctoc;
				}
			}

			if(tabBoutons[boutonY])	
	  		{
	  			noPatte = 1;
	  			etat = singleLeg;
	  			aEnvoye = true;
	  			delaiFermer = 0;	//Repart le timer d'inactivite
	  		}

			if(tabBoutons[boutonStart])	
	  		{
	  			etat = eteint;	//Envoie l'innstruction d'eteindre l'araignee
	  			aEnvoye = true;
	  		}

			if(tabBoutons[boutonA])	//Met l'araignee en mode marche
			{
				etat = marche;
				etape = 0;
			}
		}
		break;


  	case eteint:
  		if(tabBoutons[boutonStart])
  		{
  			etat = marche;
  			startServos();
  			delaiFermer = 0;	//Repart le timer d'inactivite
  			aEnvoye = true;
  		}
  		break;
  }
}

/**
 * Main loop of the node.
 */
int main(int argc, char **argv)
{
	int k = 0;
  	int nbPause = 0;
	float calcul = 0;
	float valPrecHoriz = 0.0;
	float valPrecVert = 0.0;
	float delaiBaseVert = 0.0;
	float delaiBaseHoriz = 0.0;
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
  ros::Publisher serial = n.advertise<std_msgs::String>("serial_topic", 1000, true); //le true pour latcher le message.
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
  	delaiI++;	//Pour mettre un delai entre les etapes
  	delaiFermer++;	//Pour le mode de mise en veille

  	if(delaiFermer >= 4800)
  	{
  		etat = eteint;
  	}
    
	switch(etat)	//Mouvements a effectuer selon l'etat
	{
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
				servoT = 0;
    			if(delaiI == 15)	//Fait resdescendre la patte selectionne
    			{
    				positionHanche = 1500;
    				positionGenou = 1500;
    				positionPatte = 1500;
    				aEnvoye = true;
    			}

    			if((tabAxes[axeHorizJoyGauche] != 0) && (noPatte != 3) && (noPatte !=6))
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

				else if(tabAxes[axeHorizJoyGauche] != 0)
				{
					positionHanche = positionHanche + (tabAxes[axeHorizJoyGauche]*10);

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

				//Partie qui gere la modification du tableau a envoye selon la patte selectionnee
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
    		break;

    	case automatique:
    		{
				servoT = 0;
    			switch(quelAuto)
    			{
    				case toctoc:
    					switch(etape)
    					{
    						case 0:	//Permet a l'araignee de prendre sa position de base
    							delaiFermer = 0;	//Pour ne pas que l'araignne s'eteigne
    							startServos();
    							nbPause = 0;
    							etape++;
    							break;
    						case 1:
    							delaiI = 0;
    							tabServos[1][1] = 1850;
    							etape++;
    							aEnvoye = true;
    							break;
    						case 2:
    							if(delaiI == 10)
    							{
    								tabServos[1][1] = 1500;
    								etape++;
    								aEnvoye = true;
    							} 
    							break;   							
    						case 3:
    							if(delaiI == 20)
    							{
	    							tabServos[1][4] = 1850;
	    							etape++;
	    							aEnvoye = true;
	    						}
	    						break;
    						case 4:
    							if(delaiI == 30)
    							{
	    							tabServos[1][4] = 1500;
	    							etape++;
	    							aEnvoye = true;
	    						}
	    						break;
    						case 5:
    							if(delaiI == 40)
    							{
	    							tabServos[1][7] = 1850;
	    							etape++;
	    							aEnvoye = true;
	    						}
	    						break;
    						case 6:
    							if(delaiI == 50)
    							{
	    							tabServos[1][7] = 1500;
	    							etape++;
	    							aEnvoye = true;
	    						}
	    						break;
    						case 7:
    							if(delaiI == 60)
    							{
	    							tabServos[1][16] = 1150;
	    							etape++;
	    							aEnvoye = true;
	    						}
	    						break;
    						case 8:
    							if(delaiI == 70)
    							{
	    							tabServos[1][16] = 1500;
	    							etape++;
	    							aEnvoye = true;
	    						}
	    						break;
    						case 9:
    							if(delaiI == 80)
    							{
	    							tabServos[1][13] = 1150;
	    							etape++;
	    							aEnvoye = true;
	    						}
	    						break;
    						case 10:
    							if(delaiI == 90)
    							{
	    							tabServos[1][13] = 1500;
	    							etape++;
	    							aEnvoye = true;
	    						}
	    						break;
    						case 11:
    							if(delaiI == 100)
    							{
	    							tabServos[1][10] = 1150;
	    							etape++;
	    							aEnvoye = true;
	    						}
	    						break;
    						case 12:
    							if(delaiI == 110)
    							{
	    							tabServos[1][10] = 1500;
	    							repetition++;
	    							if(repetition == 10)
					    			{
					    				quelAuto++;
					    				etat = automatique;
					    				repetition = 0;
					    				etape = 0;
					    			}
	    						}
	    						if(delaiI == 120)
	    						{
	    							etape = 0;
	    						}
	    						aEnvoye = true;
	    						break;
    					}
    					break;

    				case cancan:
    					switch(etape)
    					{
    						case 0:	//Position de base de l'araignee
    							delaiFermer = 0;	//Pour ne pas que l'araignne s'eteigne
    							delaiI = 0;
    							startServos();
    							tabServos[1][0] = 2000;
    							tabServos[1][6] = 1000;
    							tabServos[1][9] = 1000;
    							tabServos[1][15] = 2000;
    							etape++;
    						break;

    						case 1:
    							if(delaiI == 10)
    							{
    								tabServos[1][1] = 2500;
	    							tabServos[1][2] = 2400;
	    							tabServos[1][7] = 2500;
	    							tabServos[1][8] = 2400;
	    							tabServos[1][13] = 500;
	    							tabServos[1][14] = 695;
	    							etape++;
	    							aEnvoye = true;
    							}
    						break;

    						case 2:
    							if(delaiI == 60)
    							{
	    							tabServos[1][1] = 2200;
	    							tabServos[1][2] = 500;
	    							tabServos[1][7] = 2200;
	    							tabServos[1][8] = 500;
	    							tabServos[1][13] = 700;
	    							tabServos[1][14] = 2500;
	    							etape++;
	    							aEnvoye = true;
    							}
    						break;

    						case 3:
    							if(delaiI == 110)
    							{
    								tabServos[1][1] = 2500;
	    							tabServos[1][2] = 2400;
	    							tabServos[1][7] = 2500;
	    							tabServos[1][8] = 2400;
	    							tabServos[1][13] = 500;
	    							tabServos[1][14] = 695;
	    							etape++;
	    							aEnvoye = true;
    							}
    						break;

    						case 4:
    							if(delaiI == 160)
    							{
    								tabServos[1][1] = 1500;
	    							tabServos[1][2] = 1500;
	    							tabServos[1][7] = 1400;
	    							tabServos[1][8] = 1500;
	    							tabServos[1][13] = 1500;
	    							tabServos[1][14] = 1500;
	    							etape++;
	    							aEnvoye = true;
    							}
    						break;

    						case 5:
    							if(delaiI == 210)
    							{
    								tabServos[1][4] = 2500;
	    							tabServos[1][5] = 2400;
	    							tabServos[1][10] = 500;
	    							tabServos[1][11] = 695;
	    							tabServos[1][16] = 500;
	    							tabServos[1][17] = 695;
	    							etape++;
	    							aEnvoye = true;
    							}
    						break;

    						case 6:
    							if(delaiI == 260)
    							{
    								tabServos[1][4] = 2200;
	    							tabServos[1][5] = 500;
	    							tabServos[1][10] = 700;
	    							tabServos[1][11] = 2500;
	    							tabServos[1][16] = 700;
	    							tabServos[1][17] = 2500;
	    							etape++;
	    							aEnvoye = true;
    							}
    						break;

    						case 7:
    							if(delaiI == 310)
    							{
    								tabServos[1][4] = 2500;
	    							tabServos[1][5] = 2400;
	    							tabServos[1][10] = 500;
	    							tabServos[1][11] = 695;
	    							tabServos[1][16] = 500;
	    							tabServos[1][17] = 695;
	    							etape++;
	    							aEnvoye = true;
    							}
    						break;

    						case 8:
    							if(delaiI == 360)
    							{
    								tabServos[1][4] = 1500;
	    							tabServos[1][5] = 1500;
	    							tabServos[1][10] = 1500;
	    							tabServos[1][11] = 1500;
	    							tabServos[1][16] = 1500;
	    							tabServos[1][17] = 1500;
	    							aEnvoye = true;
    							}
    							if (delaiI == 410)
    							{
    								etape = 0;
    								repetition++;
    								if(repetition == 10)
					    			{
					    				startServos();
					    				quelAuto++;
					    				etat = automatique;
					    				repetition = 0;
					    			}
    							}
    						break;
    					}
    					break;

    				case pause:
    					switch(etape)
    					{
    						case 0:
    							freeServos();
    							nbPause++;
    							if(nbPause == 1)
    							{
    								aEnvoye = true;
    							}
    							repetition++;
    							if(repetition == 1600)
    							{
    								repetition = 0;
    								quelAuto = toctoc;
    							}
    						break;
    					}
						break;
    			}

    		}
			break;
			
		case marche:
			{
				if(abs(valPrecVert - tabAxes[axeVertJoyGauche]) > 0.1)
				{
					delaiBaseVert = (-66.67*(abs(tabAxes[axeVertJoyGauche]))) + 86.67;
					delaiI = delaiBaseVert * (etape - 1);
					if(etape == 0)
					{
						delaiI = 0;
					}
					valPrecVert = tabAxes[axeVertJoyGauche];
				}
				
				else if(abs(valPrecHoriz - tabAxes[axeHorizJoyGauche]) > 0.1)
				{
					delaiBaseHoriz = (-66.67*(abs(tabAxes[axeHorizJoyGauche]))) + 86.67;
					delaiI = delaiBaseHoriz * (etape - 1);
					if(etape == 0)
					{
						delaiI = 0;
					}
					valPrecHoriz = tabAxes[axeHorizJoyGauche];
				}

				//Section qui fait avancer l'araignee
				if ((tabAxes[axeVertJoyGauche] > 0.2) && (tabAxes[axeHorizJoyGauche] < 0.5) && (tabAxes[axeHorizJoyGauche] > -0.5))
				{
					servoT = ((int)delaiBaseVert*1000)/80;
					switch(etape)
					{
						//Rempli le tableau au complet a la premiere etape pour etre certain d'avoir toutes les valeurs voulues
						case 0:
							tabServos[1][0] = 1700;
							tabServos[1][1] = 1800;
							tabServos[1][2] = 1700;
							tabServos[1][3] = 1350;
							tabServos[1][4] = 1500;
							tabServos[1][5] = 1500;
							tabServos[1][6] = 1695;
							tabServos[1][7] = 1700;
							tabServos[1][8] = 1200;
							tabServos[1][9] = 1500;
							tabServos[1][10] = 1500;
							tabServos[1][11] = 1700;
							tabServos[1][12] = 1350;
							tabServos[1][13] = 1300;
							tabServos[1][14] = 1500;
							tabServos[1][15] = 1500;
							tabServos[1][16] = 1500;
							tabServos[1][17] = 1300;
							aEnvoye = true;
							envoiUneFois = 0;	//Remet la variable a 0 pour envoyer la consigne de remettre l'etape a 0 une seule fois quand le bouton est relache
							delaiI = 0;
							etape++;
							break;
							
						
						//A partir de cette etape on ecrit que les index qui changent
						case 1:
							if(delaiI == (int)delaiBaseVert)
							{
								tabServos[1][1] = 1500;
								tabServos[1][3] = 1500;
								tabServos[1][7] = 1340;
								tabServos[1][11] = 1500;
								tabServos[1][13] = 1630;
								tabServos[1][17] = 1500;
								aEnvoye = true;
								etape++;
							}
							break;
							
						case 2:
							if(delaiI == (int)(delaiBaseVert*etape))
							{
								tabServos[1][2] = 1800;
								tabServos[1][4] = 1700;
								tabServos[1][7] = 1260;
								tabServos[1][10] = 1300;
								tabServos[1][16] = 1300;
								aEnvoye = true;
								etape++;
							}
							break;
							
						case 3:
							if(delaiI == (int)(delaiBaseVert*etape))
							{
								tabServos[1][0] = 1500;
								tabServos[1][1] = 1430;
								tabServos[1][2] = 1300;
								tabServos[1][3] = 1775;
								tabServos[1][6] = 1500;
								tabServos[1][7] = 1500;
								tabServos[1][8] = 1700;
								tabServos[1][9] = 1300;
								tabServos[1][11] = 1200;
								tabServos[1][12] = 1650;
								tabServos[1][13] = 1650;
								tabServos[1][15] = 1300;
								tabServos[1][17] = 1800;
								aEnvoye = true;
								etape++;
							}
							break;
						
						case 4:
							if(delaiI == (int)(delaiBaseVert*etape))
							{
								tabServos[1][1] = 1500;
								tabServos[1][3] = 1750;
								tabServos[1][4] = 1500;
								tabServos[1][10] = 1475;
								tabServos[1][11] = 1300;
								tabServos[1][16] = 1730;
								aEnvoye = true;
								etape++;
							}
							break;
						
						case 5:
							if(delaiI == (int)(delaiBaseVert*etape))
							{
								tabServos[1][1] = 1700;
								tabServos[1][3] = 1775;
								tabServos[1][7] = 1700;
								tabServos[1][10] = 1500;
								tabServos[1][11] = 1200;
								tabServos[1][13] = 1350;
								aEnvoye = true;
							}
							else if(delaiI == (int)(delaiBaseVert*6))	//Delai entre etape 5 et etape 0
							{
								etape = 0;
							}
							break;
					}
					break;
				}

				//Section qui fait reculer l'araignee
				else if ((tabAxes[axeVertJoyGauche] < -0.1) && (tabAxes[axeHorizJoyGauche] < 0.5) && (tabAxes[axeHorizJoyGauche] > -0.5))
				{
					servoT = ((int)delaiBaseVert*1000)/80;
					switch(etape)
					{
						//Rempli le tableau au complet a la premiere etape pour etre certain d'avoir toutes les valeurs voulues
						case 0:
							tabServos[1][0] = 1500;
							tabServos[1][1] = 1700;
							tabServos[1][2] = 1300;
							tabServos[1][3] = 1775;
							tabServos[1][4] = 1500;
							tabServos[1][5] = 1500;
							tabServos[1][6] = 1500;
							tabServos[1][7] = 1700;
							tabServos[1][8] = 1700;
							tabServos[1][9] = 1300;
							tabServos[1][10] = 1500;
							tabServos[1][11] = 1200;
							tabServos[1][12] = 1650;
							tabServos[1][13] = 1350;
							tabServos[1][14] = 1500;
							tabServos[1][15] = 1300;
							tabServos[1][16] = 1730;
							tabServos[1][17] = 1800;
							aEnvoye = true;
							envoiUneFois = 0;	//Remet la variable a 0 pour envoyer la consigne de remettre l'etape a 0 une seule fois quand le bouton est relache
							delaiI = 0;
							etape++;
							break;

						//A partir de cette etape on ecrit que les index qui changent
						case 1:
							if(delaiI == (int)(delaiBaseVert*etape))
							{
								tabServos[1][2] = 1500;
								tabServos[1][3] = 1775;
								tabServos[1][7] = 1500;
								tabServos[1][10] = 1475;
								tabServos[1][11] = 1300;
								tabServos[1][13] = 1650;								
								aEnvoye = true;
								etape++;
							}
							break;
						
						case 2:
							if(delaiI == (int)(delaiBaseVert*etape))
							{
								tabServos[1][1] = 1430;
								tabServos[1][3] = 1775;
								tabServos[1][4] = 1700;
								tabServos[1][10] = 1300;
								tabServos[1][11] = 1200;
								tabServos[1][16] = 1300;
								aEnvoye = true;
								etape++;
							}
							break;
						
						case 3:
							if(delaiI == (int)(delaiBaseVert*etape))
							{
								tabServos[1][0] = 1700;
								tabServos[1][1] = 1700;
								tabServos[1][2] = 1500;
								tabServos[1][3] = 1500;
								tabServos[1][6] = 1695;
								tabServos[1][7] = 1260;
								tabServos[1][8] = 1200;
								tabServos[1][9] = 1500;
								tabServos[1][11] = 1500;
								tabServos[1][12] = 1350;
								tabServos[1][13] = 1630;
								tabServos[1][15] = 1500;
								tabServos[1][17] = 1500;
								aEnvoye = true;
								etape++;
							}
							break;
						
						case 4:
							if(delaiI == (int)(delaiBaseVert*etape))
							{
								tabServos[1][2] = 1700;
								tabServos[1][4] = 1500;
								tabServos[1][7] = 1340;
								tabServos[1][10] = 1500;
								tabServos[1][16] = 1500;
								aEnvoye = true;
								etape++;
							}
							break;
						
						case 5:
							if(delaiI == (int)(delaiBaseVert*etape))
							{
								tabServos[1][1] = 1800;
								tabServos[1][3] = 1350;
								tabServos[1][7] = 1700;
								tabServos[1][11] = 1700;
								tabServos[1][13] = 1300;
								tabServos[1][17] = 1300;
								aEnvoye = true;
							}
							else if(delaiI == (int)(delaiBaseVert*6))
							{
								etape = 0;
							}
							break;
					}
					break;
				}

				//Section qui fait bouger de cote vers la droite
				else if ((tabAxes[axeHorizJoyGauche] < -0.1) && (tabAxes[axeVertJoyGauche] < 0.5) && (tabAxes[axeVertJoyGauche] > -0.5))
				{
					servoT = ((int)delaiBaseHoriz*1000)/80;
					switch(etape)
					{
						//Rempli le tableau au complet a la premiere etape pour etre certain d'avoir toutes les valeurs voulues
						case 0:
							tabServos[1][0] = 1500;
							tabServos[1][1] = 1700;
							tabServos[1][2] = 1500;
							tabServos[1][3] = 1500;
							tabServos[1][4] = 1400;
							tabServos[1][5] = 1700;
							tabServos[1][6] = 1500;
							tabServos[1][7] = 1700;
							tabServos[1][8] = 1500;
							tabServos[1][9] = 1300;
							tabServos[1][10] = 1700;
							tabServos[1][11] = 1700;
							tabServos[1][12] = 1500;
							tabServos[1][13] = 1300;
							tabServos[1][14] = 1500;
							tabServos[1][15] = 1700;
							tabServos[1][16] = 1600;
							tabServos[1][17] = 1500;
							aEnvoye = true;
							envoiUneFois = 0;	//Remet la variable a 0 pour envoyer la consigne de remettre l'etape a 0 une seule fois quand le bouton est relache
							delaiI = 0;
							etape++;
							break;
						
						//A partir de cette etape on ecrit que les index qui changent
						case 1:
							if(delaiI == (int)(delaiBaseHoriz*etape))
							{
								tabServos[1][1] = 1500;
								tabServos[1][4] = 1500;
								tabServos[1][7] = 1500;
								tabServos[1][10] = 1600;
								tabServos[1][13] = 1600;
								tabServos[1][14] = 1400;
								tabServos[1][17] = 1700;
								aEnvoye = true;
								etape++;
							}
							break;
						
						case 2:
							if(delaiI == (int)(delaiBaseHoriz*etape))
							{
								tabServos[1][0] = 1300;
								tabServos[1][2] = 1700;
								tabServos[1][4] = 1800;
								tabServos[1][6] = 1700;
								tabServos[1][8] = 1700;
								tabServos[1][9] = 1500;
								tabServos[1][10] = 1200;
								tabServos[1][11] = 1300;
								tabServos[1][14] = 1700;
								tabServos[1][15] = 1500;
								tabServos[1][16] = 1200;
								tabServos[1][17] = 1200;
								aEnvoye = true;
								etape++;
							}
							break;
						
						case 3:
							if(delaiI == (int)(delaiBaseHoriz*etape))
							{
								tabServos[1][4] = 1500;
								tabServos[1][5] = 1500;
								tabServos[1][10] = 1500;
								tabServos[1][11] = 1400;
								tabServos[1][16] = 1500;
								aEnvoye = true;
							}
							else if(delaiI == (int)(delaiBaseHoriz*4))
							{
								etape = 0;
							}
							break;
					}
					break;
				}

				//Section qui fait bouger de cote vers la gauche
				else if ((tabAxes[axeHorizJoyGauche] > 0.1) && (tabAxes[axeVertJoyGauche] < 0.5) && (tabAxes[axeVertJoyGauche] > -0.5))
				{
					servoT = ((int)delaiBaseHoriz*1000)/80;
					switch(etape)
					{
						//Rempli le tableau au complet a la premiere etape pour etre certain d'avoir toutes les valeurs voulues
						case 0:
							tabServos[1][0] = 1300;
							tabServos[1][1] = 1300;
							tabServos[1][2] = 1700;
							tabServos[1][3] = 1500;
							tabServos[1][4] = 1500;
							tabServos[1][5] = 1500;
							tabServos[1][6] = 1700;
							tabServos[1][7] = 1500;
							tabServos[1][8] = 1700;
							tabServos[1][9] = 1500;
							tabServos[1][10] = 1500;
							tabServos[1][11] = 1400;
							tabServos[1][12] = 1500;
							tabServos[1][13] = 1600;
							tabServos[1][14] = 1700;
							tabServos[1][15] = 1500;
							tabServos[1][16] = 1500;
							tabServos[1][17] = 1200;
							aEnvoye = true;
							envoiUneFois = 0;	//Remet la variable a 0 pour envoyer la consigne de remettre l'etape a 0 une seule fois quand le bouton est relache
							delaiI = 0;
							etape++;
							break;
						
						//A partir de cette etape on ecrit que les index qui changent
						case 1:
							if(delaiI == (int)(delaiBaseHoriz*etape))
							{
								tabServos[1][4] = 1800;
								tabServos[1][5] = 1700;
								tabServos[1][10] = 1200;
								tabServos[1][11] = 1300;
								tabServos[1][16] = 1200;
								aEnvoye = true;
								etape++;
							}
							break;
						
						case 2:
							if(delaiI == (int)(delaiBaseHoriz*etape))
							{
								tabServos[1][0] = 1500;
								tabServos[1][2] = 1500;
								tabServos[1][4] = 1500;
								tabServos[1][6] = 1500;
								tabServos[1][8] = 1500;
								tabServos[1][9] = 1300;
								tabServos[1][10] = 1600;
								tabServos[1][11] = 1700;
								tabServos[1][14] = 1400;
								tabServos[1][15] = 1700;
								tabServos[1][16] = 1600;
								tabServos[1][17] = 1700;
								aEnvoye = true;
								etape++;
							}
							break;
						
						case 3:
							if(delaiI == (int)(delaiBaseHoriz*etape))
							{
								tabServos[1][1] = 1700;
								tabServos[1][4] = 1400;
								tabServos[1][7] = 1700;
								tabServos[1][10] = 1700;
								tabServos[1][13] = 1300;
								tabServos[1][14] = 1500;
								tabServos[1][17] = 1500;
								aEnvoye = true;
							}
							else if(delaiI == (int)(delaiBaseHoriz*4))
							{
								etape = 0;
							}
							break;
					}
					break;
				}

				//Section qui fait tourner l'araignee vers la droite
				else if(tabBoutons[boutonRB])
				{
					switch(etape)
					{
						//Rempli le tableau au complet a la premiere etape pour etre certain d'avoir toutes les valeurs voulues
						case 0:
							tabServos[1][0] = 1800;
							tabServos[1][1] = 1500;
							tabServos[1][2] = 1500;
							tabServos[1][3] = 1200;
							tabServos[1][4] = 1800;
							tabServos[1][5] = 1500;
							tabServos[1][6] = 1700;
							tabServos[1][7] = 1500;
							tabServos[1][8] = 1500;
							tabServos[1][9] = 1300;
							tabServos[1][10] = 1200;
							tabServos[1][11] = 1500;
							tabServos[1][12] = 1800;
							tabServos[1][13] = 1500;
							tabServos[1][14] = 1500;
							tabServos[1][15] = 1300;
							tabServos[1][16] = 1200;
							tabServos[1][17] = 1350;
							aEnvoye = true;
							envoiUneFois = 0;	//Remet la variable a 0 pour envoyer la consigne de remettre l'etape a 0 une seule fois quand le bouton est relache
							delaiI = 0;
							etape++;
							break;
						
						//A partir de cette etape on ecrit que les index qui changent
						case 1:
							if(delaiI == delai1)
							{
								tabServos[1][4] = 1400;
								tabServos[1][10] = 1500;
								tabServos[1][16] = 1500;
								aEnvoye = true;
								etape++;
							}
							break;							

						case 2:
							if(delaiI == delai2)
							{
								tabServos[1][0] = 1500;
								tabServos[1][1] = 1800;
								tabServos[1][3] = 1800;
								tabServos[1][6] = 1300;
								tabServos[1][7] = 1800;
								tabServos[1][9] = 1700;
								tabServos[1][12] = 1300;
								tabServos[1][13] = 1200;
								tabServos[1][15] = 1700;
								aEnvoye = true;
								etape++;
							}
							break;
						
						case 3:
							if(delaiI == delai3)
							{
								tabServos[1][1] = 1500;
								tabServos[1][7] = 1500;
								tabServos[1][12] = 1400;
								tabServos[1][13] = 1500;
								aEnvoye = true;
							}
							else if(delaiI == delai4)
							{
								etape = 0;
							}
							
					}
				}

				//Section qui fait tourner l'araignee vers la gauche
				else if(tabBoutons[boutonLB])
				{
					switch(etape)
					{
						//Rempli le tableau au complet a la premiere etape pour etre certain d'avoir toutes les valeurs voulues
						case 0:
							tabServos[1][0] = 1500;
							tabServos[1][1] = 1500;
							tabServos[1][2] = 1500;
							tabServos[1][3] = 1800;
							tabServos[1][4] = 1400;
							tabServos[1][5] = 1500;
							tabServos[1][6] = 1300;
							tabServos[1][7] = 1500;
							tabServos[1][8] = 1500;
							tabServos[1][9] = 1700;
							tabServos[1][10] = 1500;
							tabServos[1][11] = 1500;
							tabServos[1][12] = 1400;
							tabServos[1][13] = 1500;
							tabServos[1][14] = 1500;
							tabServos[1][15] = 1700;
							tabServos[1][16] = 1500;
							tabServos[1][17] = 1350;
							aEnvoye = true;
							envoiUneFois = 0;	//Remet la variable a 0 pour envoyer la consigne de remettre l'etape a 0 une seule fois quand le bouton est relache
							delaiI = 0;
							etape++;
							break;

						//A partir de cette etape on ecrit que les index qui changent
						case 1:
							if(delaiI == delai1)
							{
								tabServos[1][1] = 1800;
								tabServos[1][7] = 1800;
								tabServos[1][12] = 1300;
								tabServos[1][13] = 1200;
								aEnvoye = true;
								etape++;
							}
							break;

						case 2:
							if(delaiI == delai2)
							{
								tabServos[1][0] = 1800;
								tabServos[1][1] = 1500;
								tabServos[1][3] = 1200;
								tabServos[1][6] = 1700;
								tabServos[1][7] = 1500;
								tabServos[1][9] = 1300;
								tabServos[1][12] = 1800;
								tabServos[1][13] = 1500;
								tabServos[1][15] = 1300;
								aEnvoye = true;
								etape++;
							}
							break;
						
						case 3:
							if(delaiI == delai3)
							{
								tabServos[1][4] = 1800;
								tabServos[1][10] = 1200;
								tabServos[1][16] = 1200;
								aEnvoye = true;
							}
							else if(delaiI == delai4)
							{
								etape = 0;
							}
					}
				}
							
				else
				{
					//Des que le bouton est relache, remet l'araigne en position de base
					envoiUneFois++;
					if(envoiUneFois == 1)	//Assure d'envoyer la position de base une seule fois
					{
						etape = 0;
						startServos();
					}
				}			
			}
			break;
	}

	if(aEnvoye)
	{
		string serie = "";

		for(int i = 0; i < nbServos; i++)
		{
			serie = serie + "#" + to_string(tabServos[0][i]) + "P" + to_string(tabServos[1][i]);
		}
		if(servoT != 0)
		{
			serie = serie + "T" + to_string(servoT);
		}
		serie = serie + "\r";
		
		message.data = serie;
		ROS_INFO("%s\n", message.data.c_str());
	  	serial.publish(message);
	  	aEnvoye = false;  //Chaque fois qu'une string est envoyee on le remet a faux pour ne pas l'envoyer plusieurs fois
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

//Fonction qui eteint l'araignee
void freeServos()
{
  for (int i = 0; i < nbServos; i++)
  {
  	tabServos[1][i] = 0;
  }
}

// Fonction qui remet l'araignee a sa position de base
void startServos()
{
  for (int i = 0; i < nbServos; i++)
  {
  	tabServos[1][i] = 1500;
  	aEnvoye = true;
  }
}




