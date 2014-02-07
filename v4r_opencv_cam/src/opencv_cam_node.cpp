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


#include <v4r_opencv_cam/opencv_cam_defaults.h>
#include <v4r_opencv_cam/opencv_cam_node.h>
#include <camera_info_manager/camera_info_manager.h>



int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "opencv_cam" );
    ros::NodeHandle n;
    OpenCvCam cam ( n );
    ros::Rate rate ( cam.frequency() );
    cam.init();
    while ( ros::ok() ) {
        cam.capture();
        cam.publishCamera();
        cam.show();
        rate.sleep();
    }
    return 0;
}

void OpenCvCam::CvCaptureInfo::getCameraParam(cv::VideoCapture &videoCapture) {
  
        if(cap_prop_frame_width_support_) {
            cap_prop_frame_width_   = videoCapture.get ( CV_CAP_PROP_FRAME_WIDTH);
            if(cap_prop_frame_width_ == -1) {
	      cap_prop_frame_width_support_ = false;
	      ROS_WARN ( "OpenCvCam: CV_CAP_PROP_FRAME_WIDTH not supported");
	    }
        }
        if(cap_prop_frame_height_support_) {
            cap_prop_frame_height_   = videoCapture.get ( CV_CAP_PROP_FRAME_HEIGHT);
            if(cap_prop_frame_height_ == -1) {
	      cap_prop_frame_height_support_ = false;
	      ROS_WARN ( "OpenCvCam: CV_CAP_PROP_FRAME_WIDTH not supported");
	    }
        }
        if(cap_prop_fps_support_) {
            cap_prop_fps_   = videoCapture.get ( CV_CAP_PROP_FPS);
            if(cap_prop_fps_ == -1) {
	      cap_prop_fps_support_ = false;
	      ROS_WARN ( "OpenCvCam: CV_CAP_PROP_FPS not supported");
	    }
        }
}

void OpenCvCam::CvCaptureInfo::update(cv::VideoCapture &videoCapture, cv::Mat &img) {
    if ( videoCapture.isOpened() ) {
        videoCapture >> img;
        if(cap_prop_pos_msec_support_) {
            cap_prop_pos_msec_   = videoCapture.get ( CV_CAP_PROP_POS_MSEC);
            if(cap_prop_pos_msec_ == -1) {
	      cap_prop_pos_msec_support_ = false;
	      ROS_WARN ( "OpenCvCam: CV_CAP_PROP_POS_MSEC not supported");
	    }
        }
        if(cap_prop_pos_frames_support_) {
            cap_prop_pos_frames_  = videoCapture.get (CV_CAP_PROP_POS_FRAMES);
            if(cap_prop_pos_frames_ == -1) cap_prop_pos_frames_support_ = false;
	      ROS_WARN ( "OpenCvCam: CV_CAP_PROP_POS_FRAMES not supported");
        }
        if(cap_prop_frame_count_support_) {
            cap_prop_frame_count_    = videoCapture.get ( CV_CAP_PROP_FRAME_COUNT);
            if(cap_prop_frame_count_ == -1) cap_prop_frame_count_support_ = false;
	      ROS_WARN ( "OpenCvCam: CV_CAP_PROP_FRAME_COUNT not supported");
        }
    }
}

unsigned int OpenCvCam::CvCaptureInfo::getTimeStamp(int min, int sec, int ms) {
    if(cap_prop_pos_msec_support_){
      double millis = cap_prop_pos_msec_;
      min = (int)(millis/1000/60);
      millis -= min*60000;
      sec = (int)(millis/1000);
      ms -= sec*1000;
    }
    return (unsigned int ) cap_prop_frame_count_;
}

OpenCvCam::OpenCvCam ( ros::NodeHandle & n ) :
    n_ ( n ), n_param_ ( "~" ), imageTransport_(n_param_) {

    readParam();

    cameraPublisher_ = imageTransport_.advertiseCamera("image_raw", 1);

}

void OpenCvCam::readParam()
{
    n_param_.param<double> ( "frequency", frequency_, DEFAULT_FREQUENCY);
    ROS_INFO ( "OpenCvCam: frequency: %5.2f", frequency_ );

    n_param_.param<bool> ( "show_camera_image", show_camera_image_, DEFAULT_SHOW_CAMERA_IMAGE );
    ROS_INFO ( "OpenCvCam: debug:  %s", ((show_camera_image_) ? "true" : "false"));

    n_param_.param<int>("video_device", video_device_, DEFAULT_VIDEODEVICE);
    ROS_INFO("OpenCvCam: video_device: %i", video_device_);

    std::string camera_info_url;
    cameraInfo_.width = DEFAULT_FRAME_WIDTH;
    cameraInfo_.height = DEFAULT_FRAME_HEIGHT;
    if(n_param_.getParam("camera_info_url", camera_info_url)) {
        camera_info_manager::CameraInfoManager cinfo(n_param_);
        if(cinfo.validateURL(camera_info_url)) {
            cinfo.loadCameraInfo(camera_info_url);
            cameraInfo_ = cinfo.getCameraInfo();
        } else {
            ROS_FATAL("camera_info_url not valid.");
            n_param_.shutdown();
            return;
        }
    } else {
        XmlRpc::XmlRpcValue double_list;
        n_param_.getParam("K", double_list);
        if((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 9)) {
            for(int i = 0; i < 9; i++) {
                ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                cameraInfo_.K[i] = double_list[i];
            }
        }

        n_param_.getParam("D", double_list);
        if((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 5)) {
            for(int i = 0; i < 5; i++) {
                ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                cameraInfo_.D[i] = double_list[i];
            }
        }

        ROS_INFO("OpenCvCam: tf_camera_id: %s", cameraInfo_.header.frame_id.c_str());
        n_param_.getParam("R", double_list);
        if((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 9)) {
            for(int i = 0; i < 9; i++) {
                ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                cameraInfo_.R[i] = double_list[i];
            }
        }

        n_param_.getParam("P", double_list);
        if((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 12)) {
            for(int i = 0; i < 12; i++) {
                ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                cameraInfo_.P[i] = double_list[i];
            }
        }
    }
    n_param_.param<std::string>("frame_id", cameraInfo_.header.frame_id, DEFAULT_FRAME_ID);
    ROS_INFO("OpenCvCam: frame_id: %s", cameraInfo_.header.frame_id.c_str());
}

void OpenCvCam::init() {
    cap_.open ( video_device_ );
    if ( !cap_.isOpened() ) {
        ROS_ERROR ( "OpenCvCam: Could not initialize capturing device %i", video_device_);
    }

    cap_.set ( CV_CAP_PROP_FRAME_WIDTH, cameraInfo_.width );
    cap_.set ( CV_CAP_PROP_FRAME_HEIGHT, cameraInfo_.height );
    captureInfo_.getCameraParam(cap_);
    ROS_INFO ( "OpenCvCam: frame size %i x %i @ %3.1f Hz",
               (int) captureInfo_.cap_prop_frame_width_ ,
               (int) captureInfo_.cap_prop_frame_height_ ,
               captureInfo_.cap_prop_fps_ );

    if ( cap_.isOpened() ) {
        cap_ >> img_;
    }
    cameraInfo_.width = img_.cols;
    cameraInfo_.height = img_.rows;
    if(show_camera_image_) {
        cv::namedWindow ( DEBUG_WINDOWS_NAME,1 );
    }
}

void OpenCvCam::capture() {
    captureInfo_.update(cap_, img_);
    int min, sec, ms;
    unsigned int  counter = captureInfo_.getTimeStamp(min, sec, ms);
    // ROS_INFO("OpenCvCam: %d %02d:%02d:%03d", counter, min, sec, ms);
}

void OpenCvCam::show() {
    if(!show_camera_image_) return;
    cv::imshow ( DEBUG_WINDOWS_NAME, img_ );
    cv::waitKey(5);
}

void OpenCvCam::publishCamera()
{
    cameraInfo_.header.stamp = ros::Time::now();
    cameraImage_.header = cameraInfo_.header;
    cameraImage_.height = cameraInfo_.height = img_.rows;
    cameraImage_.width = cameraInfo_.width = img_.cols;
    cameraImage_.is_bigendian = true;
    cameraImage_.step = cameraInfo_.width * 3;
    cameraImage_.encoding = "bgr8";
    cameraImage_.data.resize(cameraImage_.height * cameraImage_.width * 3);
    cameraImage_.step = cameraInfo_.width * 3;
    memcpy(&cameraImage_.data[0], img_.data, cameraImage_.data.size());
    cameraPublisher_.publish(cameraImage_, cameraInfo_);
}
OpenCvCam::~OpenCvCam() {
}
