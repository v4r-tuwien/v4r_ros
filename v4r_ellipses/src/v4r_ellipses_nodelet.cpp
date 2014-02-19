/***************************************************************************
 *   Copyright (C) 2014 by Markus Bader                                    *
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

#include "v4r_ellipses_nodelet.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

EllipsesDetectionNode::~EllipsesDetectionNode() {
}

EllipsesDetectionNode::EllipsesDetectionNode(ros::NodeHandle &n) :
    EllipsesDetection(new EllipsesDetectionNode::ParametersNode()), n_(n), callback_counter_(0), imageTransport_(n_) {

}

const EllipsesDetectionNode::ParametersNode *EllipsesDetectionNode::param() {
    return (EllipsesDetectionNode::ParametersNode*) param_;
}

void EllipsesDetectionNode::init() {
    sub_camera_ = imageTransport_.subscribeCamera( "image", 1, &EllipsesDetectionNode::imageCallback, this );
}

void EllipsesDetectionNode::imageCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& camer_info_) {
    callback_counter_++;
    
    cv_bridge::CvImagePtr img;
    try {
        img = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    next();
    std::vector<cv::RotatedRect> ellipses;
    fit_ellipses_opencv ( img->image); 
    
    if (param()->show_camera_image) {
        cv::Mat img_debug;
        cvtColor(img->image, img_debug, CV_GRAY2BGR);
	draw_ellipses(img_debug);	
        cv::imshow( param()->node_name, img_debug);
	if(param()->debug){
	  for(unsigned int i = 0; i < serach_windows_.size(); i++){
	    cv::imshow( param()->node_name + boost::lexical_cast<std::string>(i), serach_windows_[i]);
	  }
	}
        cv::waitKey(param()->show_camera_image_waitkey);
    }
}
