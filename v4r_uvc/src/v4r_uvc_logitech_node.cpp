/***************************************************************************
 *   Copyright (C) 2012 by Markus Bader                                    *
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


#include <v4r_uvc/v4r_uvc_ros.h>
#include <dynamic_reconfigure/server.h>
#include <v4r_uvc/CameraLogitechConfig.h>

class V4RLogitechNode : public V4RCamNode {
public:
    V4RLogitechNode ( ros::NodeHandle &n ):V4RCamNode(n) {
        reconfigureFnc_ = boost::bind(&V4RLogitechNode::callbackParameters, this,  _1, _2);
        reconfigureServer_.setCallback(reconfigureFnc_);
    }
    void callbackParameters ( v4r_uvc::CameraLogitechConfig &config, uint32_t level ) { 
        show_camera_image_ = config.show_camera_image; 
        camera_freeze_ = config.camera_freeze;
	queueRosParamToV4LCommit_ = true;
    }
protected:
    dynamic_reconfigure::Server<v4r_uvc::CameraLogitechConfig> reconfigureServer_;
    dynamic_reconfigure::Server<v4r_uvc::CameraLogitechConfig>::CallbackType reconfigureFnc_;
protected:
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "v4r_uvc_logitech");
    ros::NodeHandle n;
    V4RLogitechNode node(n);
    ros::Rate rate(100);
    while(ros::ok() && node.grab()) {
        node.publishCamera();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

