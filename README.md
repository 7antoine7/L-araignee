# L-araignee
## Nodes
Les nodes ROS occupent plusieurs fonctions pour le robot.

1. Joystick

	A. Pour installer la node du joystick, dans un terminal, tapez `sudo apt-get install ros-melodic-joy`
	
	B. Pour verifier si la manette est connectee, tapez `ls -l /dev/input` et trouver js0
	
	C. Pour changer les droits sur la manette, tapez `sudo chmod a+rw /dev/input/js0`
	
	D. Pour lancer la node, tapez `rosrun joy joy_node`
	   
2. Communication série

	A. Pour installer la bibliothèque pour la communication série, dans un terminal, 
	tapez `sudo apt-get install ros-melodic-serial`
	
	B. pour savoir quel port est utilisé par le convertisseur, tapez `sudo dmesg | grep tty` l'entrée la plus récente 	  devrait être votre conertisseur (par exemple `ttyUSB0`)
	
	C. Pour donner les droits d'écriture et de lecture a votre node, tapez `sudo chmod 777 <chemin d'accès du 		convertisseur>`
	
	D. Pour lancer la node, tapez `rosrun Araignnee_pkg talker_serie`
