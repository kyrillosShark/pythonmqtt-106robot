# pythonmqtt-106robot



This is a repository for connecting and operating the 106 robot. The robot connects using pahomqtt, uses a slightly modified version of the BerryIMU kalmans filter and gps to achieve waypoint navigation for the robot using Pure Pursuit Control algorithim. 


#####################TO RUN THE SIMULATION############################################################################################################################
To run the simulation, use the drawing tools to create a shape on the map. Make the first waypoint the current location of the robot.
*Important Note*
   The GPS, IMU data contains noise to simulate the measurements of a real robot. The simulatenoise+kalman.py contains a even noisier data but adds an aditional kalmans filter
######################################################################################################################################################################
