# SelfDrive
The Elcano Project is an autonomous vehicle control system that runs on a stack of microprocessors, with no operating system and no machine learning.

Elcano/SelfDrive was a working repository for upgrading the HighLevel part of Elcano/elcano.
There have been no changes to Elcano/SelfDrive sine June 2018.

The Elcano/Elcano repository used to hold everything, but it is deprecated as of June 2, 2019. All future files should go into the repository for the specific microprocessor. Files that span more than one processor are kept in Elcano/General. The other repositories are:
Elcano/LowLevel – Drive by wire
Elcano/HighLevel – Localization, Route Finding and Pilot
Elcano/Sonar – Detects obstacles from ultrasonic sensors
Elcano/Sweep – Scanse Sweep Lidar for Obstacle Detection
Elcano/QDED – Quadrature Edge Detectors for machine vision
Elcano/Transceiver – Transmit and receive boards for remote control option


____________________________________________
Self-driving vehicle core code
To compile the code, make sure you remove the SD and SD_master from the existing elcano libraries and use the built-in SD library. 
