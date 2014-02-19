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

#ifndef ARTOOLKITPLUS_NODE_H
#define ARTOOLKITPLUS_NODE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <v4r_artoolkitplus/ARParamConfig.h>

#include <opencv/cv.h>
#include "ARToolKitPlus/ar.h"

namespace ARToolKitPlus {
class TrackerSingleMarker;
class TrackerMultiMarker;
class Tracker;
};


class MyLogger;

class ARToolKitPlusNode {
    class Parameter {
    public:
        int skip_frames;
        std::string tf_prefix;
        std::string pattern_frame;
        std::string pattern_file;
        bool useBCH; // simple-id versus BCH-id markers
        double patternWidth;  // define size of the marker
        double borderWidth;
        int threshold;
        int nPattern;
        bool nUpdateMatrix;
        bool tracker_single_marker;
        bool tracker_multi_marker;
        int undist_iterations;
        bool input_distorted;
        int undist_mode;
        int pose_estimation_mode;
    };
public:
    ARToolKitPlusNode(ros::NodeHandle & n);
    ~ARToolKitPlusNode();
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    int callback_counter_;
    image_transport::ImageTransport imageTransport_;
    bool show_camera_image_;
    image_transport::CameraSubscriber cameraSubscriber_;
    tf::TransformBroadcaster transformBroadcaster_;
    dynamic_reconfigure::Server<v4r_artoolkitplus::ARParamConfig> reconfigureServer_;
    dynamic_reconfigure::Server<v4r_artoolkitplus::ARParamConfig>::CallbackType reconfigureFnc_;
    boost::shared_ptr<ARToolKitPlus::TrackerSingleMarker> trackerSingleMarker_;
    boost::shared_ptr<ARToolKitPlus::TrackerMultiMarker> trackerMultiMarker_;
    std::vector<ARToolKitPlus::ARMarkerInfo> arSingleMarkerInfo_;
    std::vector<ARToolKitPlus::ARMarkerInfo> arMultiMarkerInfo_;
    MyLogger *logger_;
    std::string debugWndName_;
    char namespace_[0xFF];
    Parameter param_;

    void initTrackerSingleMarker(const sensor_msgs::CameraInfoConstPtr& camer_info_);
    void initTrackerMultiMarker(const sensor_msgs::CameraInfoConstPtr& camer_info_);
    void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                       const sensor_msgs::CameraInfoConstPtr& info_msg);

    void callbackParameters ( v4r_artoolkitplus::ARParamConfig &config, uint32_t level );
    void publishTf(const std_msgs::Header &header);
    void generateDebugImage(cv::Mat &img);
    int matrix2Tf(const ARFloat M[3][4], tf::Transform &transform);
    void readParam();
    void init();

};

#endif // ARTOOLKITPLUS_NODE_H
