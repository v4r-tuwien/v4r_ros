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
#include <iomanip>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/image_encodings.h>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(V4R, EllipsesDetectionNode, V4R::EllipsesDetectionNode, nodelet::Nodelet)
using namespace V4R;

EllipsesDetectionNode::~EllipsesDetectionNode() {
}

EllipsesDetectionNode::EllipsesDetectionNode() :
    EllipsesDetection(new EllipsesDetectionNode::ParametersNode()), n_(), callback_counter_(0), imageTransport_(n_) {

}

const EllipsesDetectionNode::ParametersNode *EllipsesDetectionNode::param() {
    return (EllipsesDetectionNode::ParametersNode*) param_;
}

void EllipsesDetectionNode::init() {
    sub_camera_ = imageTransport_.subscribeCamera( "image", 1, &EllipsesDetectionNode::imageCallback, this );
}

void EllipsesDetectionNode::imageCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& camer_info) {


    if(callback_counter_ == 0) timeCallbackReceived_ = boost::posix_time::microsec_clock::local_time();
    callback_counter_++;
    if((param()->image_skip >= 0) && (callback_counter_ % (param()->image_skip+1) != 0)) return;
    timeCallbackReceivedLast_ = timeCallbackReceived_;
    timeCallbackReceived_ = boost::posix_time::microsec_clock::local_time();

    try {
        if((image_mono_ == NULL) || !param()->debug_freeze) {
            image_mono_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
            camera_info_ = camer_info;
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    timeDetectionStart_ = boost::posix_time::microsec_clock::local_time();
    next();
    std::vector<cv::RotatedRect> ellipses;
    fit_ellipses_opencv ( image_mono_->image);
    createRings();
    timeDetectionEnd_ = boost::posix_time::microsec_clock::local_time();

    if (param()->show_camera_image) {
        cv::Mat img_debug;
        std::stringstream ss;
        ss << "capture: " << std::setw(3) << (timeDetectionStart_ - timeCallbackReceived_).total_milliseconds() << "ms, ";
        ss << "detection: " << std::setw(3) << (timeDetectionEnd_ - timeDetectionStart_).total_milliseconds() << "ms, ";
        ss << "total: " << std::setw(3) << (timeDetectionEnd_ - timeCallbackReceived_).total_milliseconds() << "ms, ";
        ss << "interval: " << std::setw(3) << (timeCallbackReceived_ - timeCallbackReceivedLast_).total_milliseconds() << "ms";
        if((timeCallbackReceived_ - timeCallbackReceivedLast_).total_milliseconds() > 0) {
            ss << " = " << std::setw(3) << 1000 /(timeCallbackReceived_ - timeCallbackReceivedLast_).total_milliseconds() << "Hz";
        }
        cvtColor(image_mono_->image, img_debug, CV_GRAY2BGR);
        draw_ellipses(img_debug);
        cv::putText(img_debug, ss.str(), cv::Point(10, 15), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar::all(0), 1, CV_AA);
        cv::imshow( param()->node_name, img_debug);
        if(param()->debug) {
            for(unsigned int i = 0; i < serach_windows_.size(); i++) {
                cv::imshow( param()->node_name + boost::lexical_cast<std::string>(i), serach_windows_[i]);
            }
        }
        cv::waitKey(param()->show_camera_image_waitkey);
    }
}

void EllipsesDetectionNode::onInit()
{
    init();
}

void EllipsesDetectionNode::publishTf(const std_msgs::Header &header) {
}
