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

	ROS_INFO("creating image_transport... this might take a while...");
	image_transport::ImageTransport it(nh);
	// Initialize Publisher for depth and rgb image and advertise
	image_transport::Publisher image_pub_depth = it.advertise("depth_image", 1);
	image_transport::Publisher image_pub_rgb = it.advertise("rgb_image", 1);

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
		while ( openStatus != STATUS_OK ) {
			ROS_ERROR("Device could not be opened because %s", OpenNI::getExtendedError());
			return -1;
		}

		openni::VideoStream depthStream;
		openni::VideoStream rgbStream;
		depthStream.create(device, SENSOR_DEPTH);
		rgbStream.create(device, SENSOR_COLOR);
		depthStream.start();
		rgbStream.start();

		openni::VideoStream** streams = new openni::VideoStream*[2];
		streams[0] = &depthStream;
		streams[1] = &rgbStream;

		cv::Mat depthImage;
		cv::Mat rgbImage;

		cv_bridge::CvImagePtr cv_ptr_depth(new cv_bridge::CvImage);
		cv_bridge::CvImagePtr cv_ptr_rgb(new cv_bridge::CvImage);
		while (ros::ok()) 
		{
	    int changedIndex;
	    OpenNI::waitForAnyStream( streams, 2, &changedIndex );

			switch (changedIndex)
			{
				case 0:
				{
					openni::VideoFrameRef depthFrame;
					depthStream.readFrame( &depthFrame);
					if ( depthFrame.isValid() ) 
					{
				    depthImage = cv::Mat(depthStream.getVideoMode().getResolutionY(),
				      depthStream.getVideoMode().getResolutionX(),
				      CV_16U, (char*)depthFrame.getData() );
				    //depthImage.convertTo( depthImage, CV_8U );

						// convert cv::Mat into cv_bridge image
						cv_ptr_depth->image = depthImage;
						cv_ptr_depth->encoding = "mono16";
						cv_ptr_depth->header.frame_id = "/openni2_depth_frame";
						image_pub_depth.publish(cv_ptr_depth->toImageMsg());
				  }
				} break;
				case 1:	
				{
					openni::VideoFrameRef rgbFrame;
					rgbStream.readFrame( &rgbFrame);
					if ( rgbFrame.isValid() ) 
					{
						rgbImage = cv::Mat(rgbStream.getVideoMode().getResolutionY(),
			      rgbStream.getVideoMode().getResolutionX(),
			      CV_8UC3, (char*)rgbFrame.getData() );

						// convert cv::Mat into cv_bridge image
						cv_ptr_rgb->image = rgbImage;
						cv_ptr_rgb->encoding = "rgb8";
						cv_ptr_rgb->header.frame_id = "/openni2_rgb_frame";
						image_pub_rgb.publish(cv_ptr_rgb->toImageMsg());	
					}
				} break;
				default:
					ROS_WARN("Index %i is neither a depth nor a rgb image stream", changedIndex);
			}	
		}
		ros::spinOnce();
	}

	catch ( std::exception& ) {
		ROS_ERROR("catching error.... got it!");
	}

	return 0;
}
