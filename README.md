# L-araignee
## Nodes
Les nodes ROS occupent plusieurs fonctions pour le robot.
1. Joystick
	Pour installer la node du joystick:
	   ⋅⋅1. Dans un terminal, taper `sudo apt-get install ros-melodic-joy`
	   ⋅⋅2. Pour verifier si la manette est connectee, taper `ls -l /dev/input` et trouver js0
	   ⋅⋅3. Pour changer les droits sur la manette, taper `sudo chmod a+rw /dev/input/js0`
	   ⋅⋅4. Pour lancer la node, taper `rosrun joy joy_node`
	   
2. talker serie
	La node talker serie sert a parler au port serie. 