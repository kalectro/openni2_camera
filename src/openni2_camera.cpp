#include "OpenNI.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace openni;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "openni2_camera_node");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

	// Initialize Publisher for depth image and advertise
	image_transport::Publisher image_pub = it.advertise("depth_image", 1);
	
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

	try 
	{
		Status initStatus = OpenNI::initialize();
		if (initStatus != STATUS_OK)
		{
			ROS_ERROR("Device could not be initialized because %s", OpenNI::getExtendedError());
			return -1;
		}

		openni::Device device;
		Status openStatus = device.open(ANY_DEVICE);
		if ( openStatus != STATUS_OK ) {
			ROS_ERROR("Device could not be opened because %s", OpenNI::getExtendedError());
			return -1;
		}

		openni::VideoStream depthStream;
		depthStream.create(device, SENSOR_DEPTH);
		depthStream.start();

		std::vector<openni::VideoStream*> streams;
		streams.push_back( &depthStream );

		cv::Mat depthImage;

		while (ros::ok()) 
		{
	    int changedIndex;
	    OpenNI::waitForAnyStream( &streams[0], streams.size(), &changedIndex );
	    if ( changedIndex == 0 ) 
			{
        openni::VideoFrameRef colorFrame;
        depthStream.readFrame( &colorFrame );
        if ( colorFrame.isValid() ) 
				{
          depthImage = cv::Mat(depthStream.getVideoMode().getResolutionY(),
            depthStream.getVideoMode().getResolutionX(),
            CV_16U, (char*)colorFrame.getData() );
            //depthImage.convertTo( depthImage, CV_8U );

						// convert cv::Mat into cv_bridge image
						cv_ptr->image = depthImage;
						cv_ptr->encoding = "mono16";
						image_pub.publish(cv_ptr->toImageMsg());
					
        }
	    }
		}
	}
	catch ( std::exception& ) {
		ROS_ERROR("catching error.... got it!");
	}

	return 0;
}
