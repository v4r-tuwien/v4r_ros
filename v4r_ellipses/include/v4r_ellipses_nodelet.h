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


#ifndef V4R_ELLIPSES_NODE_H
#define V4R_ELLIPSES_NODE_H

#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include "v4r_ellipses/v4r_ellipses.h"
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <v4r_ellipses/EllipsesDetectionConfig.h>

/// ROS Node
class EllipsesDetectionNode : public EllipsesDetection {
public:
    struct ParametersNode : public Parameters {
        ParametersNode();
        void update(const unsigned long &counter);
        void callbackParameters (v4r_ellipses::EllipsesDetectionConfig &config, uint32_t level );
        ros::NodeHandle node;
        dynamic_reconfigure::Server<v4r_ellipses::EllipsesDetectionConfig> reconfigureServer_;
        dynamic_reconfigure::Server<v4r_ellipses::EllipsesDetectionConfig>::CallbackType reconfigureFnc_;
        std::string node_name;
        bool show_camera_image;
        int show_camera_image_waitkey;
    };
    EllipsesDetectionNode ( ros::NodeHandle &n );
    ~EllipsesDetectionNode();
    void init ();
    void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                       const sensor_msgs::CameraInfoConstPtr& info_msg);
private: //functions
    const ParametersNode *param();
    void update ();
private: // variables
    ros::NodeHandle n_;
    unsigned long  callback_counter_;
    image_transport::ImageTransport imageTransport_;
    image_transport::CameraSubscriber  sub_camera_;

};

#endif //V4R_ELLIPSES_NODE_H
