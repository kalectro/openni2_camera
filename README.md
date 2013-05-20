openni2_camera
==============

Access Kinect ASUS Xtion and other OpenNI cameras using OpenNI2 and publish rgb and depth streams via ROS image_transport

This package will not work out of the box and depends on a successful install of OpenNI2
Get it here:
https://github.com/kalectro/OpenNI2

If you want to install OpenNI2 on an ARM processor, make sure to check the compile flags in the file
OpenNI2/ThirdParty/PSCommon/BuildSystem/Platform.Arm

after a successful compilation using make comes the ugly part:
For ARM processors, copy the file libOpenNI2.so from OpenNI2/Bin/Armv6l-Release to one of your cmake lib folder (most likely /usr/lib)
Also copy the entire OpenNI2 folder in OpenNI2/Bin/Armv6l-Release into /usr/lib

Now you are all set to compile the package with catkin_make in your catkin workspace

Now run the command source devel/setup.bash in your catkin workspace (not the install/setup.bash) and cd to the directory OpenNI2/Bin/Armv6l-Release (for ARM processors).
start the program using
rosrun openni2_camera openni2_camera_node

I know this is ugly and if anyone finds a better way to do it, do not hesitate to do a pull request
