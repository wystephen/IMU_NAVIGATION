# IMU_NAVIGATION
This project is according to the paper write by Skog ,the source code which is achived by Matlab you can find in there website[http://www.openshoe.org].I have just use cpp to achive it ,and add the ROS[http://www.ros.org] interface.

This Project contact server main module.

## DataPreprocess
At this time,this part of code is only use to read data from csv format files,and publish it to the special topical named "imu1/data" and "imu2/data".

## ZeroDetector
This module is use to recongnize the status of the imu(whether the velocity is zeor or not).
I have achieved the GLRT method(but without any extract test).





 @article{Skog2012,
 author = {Skog, Isaac and Nilsson, John Olof and Zachariah, Dave and Handel, Peter},
 doi = {10.1109/IPIN.2012.6418862},
 isbn = {9781467319546},
 journal = {2012 International Conference on Indoor Positioning and Indoor Navigation, IPIN 2012 - Conference Proceedings},
 keywords = {Constraints,Inertial navigation,Pedestrian navigation,Zero-velocity detection},
 mendeley-groups = {IMU/Two-foot,IMU},
 title = {{Fusing the information from two navigation systems using an upper bound on their maximum spatial separation}},
 year = {2012}
 }
