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


#include <camera_info_manager/camera_info_manager.h>
#include <v4r_uvc/v4r_uvc_defaults.h>
#include <v4r_uvc/v4r_uvc_node.h>

#include <boost/interprocess/sync/scoped_lock.hpp>
#include "luvcview/v4l2uvc.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "v4r_uvc");
    ros::NodeHandle n;
    V4RCamNode v4r_cam(n);
    ros::Rate rate(100);
    while(ros::ok() && v4r_cam.grab()) {
        v4r_cam.publishCamera();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
