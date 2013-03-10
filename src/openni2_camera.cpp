/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
 *    Radu Bogdan Rusu <rusu@willowgarage.com>
 *    Suat Gedikli <gedikli@willowgarage.com>
 *    Patrick Mihelich <mihelich@willowgarage.com>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "OpenNI.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

using namespace std;
using namespace openni;

// the following function was copied from openni_camera
sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height, double f) 
{
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

  info->width  = width;
  info->height = height;

  // No distortion
  info->D.resize(5, 0.0);
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // Simple camera matrix: square pixels (fx = fy), principal point at center
  info->K.assign(0.0);
  info->K[0] = info->K[4] = f;
  info->K[2] = (width / 2) - 0.5;
  // Aspect ratio for the camera center on Kinect (and other devices?) is 4/3
  // This formula keeps the principal point the same in VGA and SXGA modes
  info->K[5] = (width * (3./8.)) - 0.5;
  info->K[8] = 1.0;

  // No separate rectified image plane, so R = I
  info->R.assign(0.0);
  info->R[0] = info->R[4] = info->R[8] = 1.0;

  // Then P=K(I|0) = (K|0)
  info->P.assign(0.0);
  info->P[0]  = info->P[5] = f; // fx, fy
  info->P[2]  = info->K[2];     // cx
  info->P[6]  = info->K[5];     // cy
  info->P[10] = 1.0;

  return info;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "openni2_camera");
	ros::NodeHandle nh("~");

	ROS_INFO("creating image_transport... this might take a while...");
	image_transport::ImageTransport it(nh);
	// Initialize Publisher for depth and rgb image and advertise
	image_transport::Publisher image_pub_depth = it.advertise("depth/image_raw", 1);
	image_transport::Publisher image_pub_rgb = it.advertise("rgb/image_raw", 1);
	ros::Publisher pub_depth_camera_info = nh.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1);
	ros::Publisher pub_rgb_camera_info = nh.advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 1);

	string rgb_info_url, depth_info_url;
  nh.param("rgb_camera_info_url", rgb_info_url, string());
  nh.param("depth_camera_info_url", depth_info_url, string());

	// Load the saved calibrations, if they exist
  boost::shared_ptr<camera_info_manager::CameraInfoManager> rgb_info_manager_ = \
			boost::make_shared<camera_info_manager::CameraInfoManager>(nh, "rgb", rgb_info_url);
  boost::shared_ptr<camera_info_manager::CameraInfoManager> depth_info_manager_  = \
			boost::make_shared<camera_info_manager::CameraInfoManager>(nh,  "depth",  depth_info_url);

	if (!rgb_info_manager_->isCalibrated())
    ROS_WARN("Using default parameters for RGB camera calibration.");
  if (!depth_info_manager_->isCalibrated())
    ROS_WARN("Using default parameters for IR camera calibration.");

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
		ROS_INFO("Everything set up... lets stream some images");
		while (ros::ok()) 
		{
	    int changedIndex;
	    OpenNI::waitForAnyStream( streams, 2, &changedIndex );
			// capture time as close to recording as possible
			ros::Time time = ros::Time::now();
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
						cv_ptr_depth->encoding = "16UC1";
						cv_ptr_depth->header.frame_id = "/openni2_depth_frame";
						cv_ptr_depth->header.stamp = time;
						image_pub_depth.publish(cv_ptr_depth->toImageMsg());

						sensor_msgs::CameraInfoPtr info = getDefaultCameraInfo(640, 480, 570.3422241210938);
						info->K[2] -= 5; // cx
						info->K[5] -= 4; // cy
						info->P[2] -= 5; // cx
						info->P[6] -= 4; // cy
						// Fill in header
						info->header.stamp    = time;
						info->header.frame_id = "/openni2_depth_frame";
						pub_depth_camera_info.publish(info);
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
						cv_ptr_rgb->header.stamp = time;
						image_pub_rgb.publish(cv_ptr_rgb->toImageMsg());	

						sensor_msgs::CameraInfoPtr info = getDefaultCameraInfo(640, 480, 525);
						// Fill in header
						info->header.stamp    = time;
						info->header.frame_id = "/openni2_rgb_frame";
						pub_rgb_camera_info.publish(info);
					}
				} break;
				default:
					ROS_WARN("Index %i is neither a depth nor a rgb image stream", changedIndex);
			}
		ros::spinOnce();
		}
	}

	catch ( std::exception& ) {
		ROS_ERROR("catching error.... got it!");
	}

	return 0;
}
