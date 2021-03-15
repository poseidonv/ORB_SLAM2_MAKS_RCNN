/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "std_msgs/Float32MultiArray.h"
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
    cv::Mat mTcw;
	cv::Mat mrgb;
	cv::Mat mdepth;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
	SLAM.LoadMap("/home/poseidon/Documents/github/ORB_SLAM2-master/Examples/ROS/ros_map.bin");
    cout<<"Not set"<<endl;
    SLAM.mpTracker->mState = ORB_SLAM2::Tracking::eTrackingState::LOST;
    cout<<"Set done"<<endl;

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
	//ros::Time time = ros::Time::now();
	image_transport::ImageTransport it(nh);
	image_transport::Publisher image_pub = it.advertise("orb_image",1);
	sensor_msgs::ImagePtr image_msg;
	image_transport::Publisher depth_pub = it.advertise("orb_depth",1);
	sensor_msgs::ImagePtr depth_msg;  

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

	ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("chatter", 1);


	ros::Rate loop_rate(10);
	cv::Mat image_frame;
	cv::Mat depth_frame;
    while(ros::ok())
	{
		image_frame = igb.mrgb;
		image_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",image_frame).toImageMsg();
		image_pub.publish(image_msg);
		depth_frame = igb.mdepth;
		depth_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",depth_frame).toImageMsg();
		depth_pub.publish(depth_msg);

		std_msgs::Float32MultiArray val;
		//cout<<"igb.mTcw type: "<<igb.mTcw.type()<<endl;
		//cout<<"igb.mTcw size: "<<igb.mTcw.size()<<endl;
		//cout<<"igb isContinuous()"<<igb.mTcw.isContinuous()<<endl;
		for(int i = 0; i<igb.mTcw.rows;i++)
		{
			float* ptr = igb.mTcw.ptr<float>(i);
			for(int j = 0;j<igb.mTcw.cols;j++)
			{
				val.data.push_back(float(ptr[j]));
				//cout<<"cout:"<<float(ptr[j])<<endl;
				//ROS_INFO("%lf", double(ptr[j]));
			}
		}
		cout<<"Tcw:"<<igb.mTcw<<endl;
		chatter_pub.publish(val);
		ros::spinOnce();
		loop_rate.sleep();
		
	}
    

    //ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
	// SLAM.SaveMap("/home/poseidon/Documents/github/ORB_SLAM2-master/Examples/ROS/ros_map.bin");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
	mrgb = cv_ptrRGB->image;
	mdepth = cv_ptrD->image;
    cout<<"link start!"<<endl;
    mTcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    //cout<<mTcw<<endl;
	
}


