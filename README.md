1. Turn on turtlebot.
2. "ssh aa274@chicken.local" -> password: aa274
3. On TB3: "roslaunch asl_turtlebot turtlebot3_bringup.launch"
4. Run "rostb3" in each terminal window you open.
5. rosrun chicken_turtlebot detector.py
	- publishes /detector/<object_name> with info for each object
6. roslaunch chicken_turtlebot turtlebot3_nav.launch
	- supervisor subscribes to rviz goal point
	- supervisor publishes /cmd_nav
	- navigator subscribes to /cmd_nav
	- navigator publishes /cmd_vel

