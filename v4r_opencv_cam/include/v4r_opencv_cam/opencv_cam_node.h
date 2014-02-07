/***************************************************************************
 *   Copyright (C) 2013 by Markus Bader                                    *
 *   markus.bader@tuwien.ac.at                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef OPENCV_CAM_NODE_H
#define OPENCV_CAM_NODE_H


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

class OpenCvCam
{
public:
    class CvCaptureInfo{
    public:
      CvCaptureInfo() 
      : cap_prop_pos_msec_support_( true)
      , cap_prop_pos_frames_support_(true)
      , cap_prop_frame_count_support_(true)
      , cap_prop_frame_height_support_( true)
      , cap_prop_frame_width_support_(true)
      , cap_prop_fps_support_(true) {}
      void update(cv::VideoCapture &videoCapture, cv::Mat &img);
      void getCameraParam(cv::VideoCapture &videoCapture);
      unsigned int getTimeStamp(int min, int sec, int ms);
      
    public:
      double cap_prop_pos_msec_;
      bool cap_prop_pos_msec_support_;
      double cap_prop_pos_frames_; 
      bool cap_prop_pos_frames_support_;
      double cap_prop_frame_count_;
      bool cap_prop_frame_count_support_;
      
      double cap_prop_frame_height_;
      bool cap_prop_frame_height_support_;
      double cap_prop_frame_width_;
      bool cap_prop_frame_width_support_;
      double cap_prop_fps_;
      bool cap_prop_fps_support_;
      
    };
    OpenCvCam (ros::NodeHandle & n);
    ~OpenCvCam ();
    double frequency() {
        return frequency_;
    }
    void init();
    void capture();
    void show();
    void publishCamera();
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    double frequency_;
    int video_device_;
    bool show_camera_image_;
    image_transport::ImageTransport  imageTransport_;
    image_transport::CameraPublisher cameraPublisher_;
    sensor_msgs::CameraInfo cameraInfo_;
    sensor_msgs::Image cameraImage_;

    cv::VideoCapture cap_;
    cv::Mat img_;
    CvCaptureInfo captureInfo_;
    
    void readParam();
};


#endif // OPENCV_CAM_NODE_H
