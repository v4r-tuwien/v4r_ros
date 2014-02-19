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
#include "v4r_ellipses_nodelet_defaults.h"

EllipsesDetectionNode::ParametersNode::ParametersNode()
    : Parameters(), node("~"), node_name(node.getNamespace()) {
    node.param<bool>("show_camera_image", show_camera_image, V4R_ELLIPSES_NODE_DEFAULT_SHOW_CAMERA_IMAGE);
    ROS_INFO("%s - show_camera_image:  %s", node_name.c_str(), (show_camera_image ? "true" : "false"));
    node.param<int>("show_camera_image_waitkey", show_camera_image_waitkey, V4R_ELLIPSES_NODE_DEFAULT_SHOW_CAMERA_IMAGE_WAITKEY);
    ROS_INFO("%s - show_camera_image_waitkey: %i", node_name.c_str(), show_camera_image_waitkey);

    reconfigureFnc_ = boost::bind(&EllipsesDetectionNode::ParametersNode::callbackParameters, this ,  _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);


}


void EllipsesDetectionNode::ParametersNode::callbackParameters (v4r_ellipses::EllipsesDetectionConfig &config, uint32_t level ) {
  show_camera_image = config.show_camera_image;
  debug = config.debug;
  threshold_edge_detection = config.threshold_edge_detection;
  threshold_contour_min_points = config.threshold_contour_min_points;
  threshold_polygon = config.threshold_polygon;
  filter_convex = config.filter_convex;
  threshold_rotated_rect_ratio = config.threshold_rotated_rect_ratio;
  filter_rings = config.filter_rings;
  threshold_ring_center = config.threshold_ring_center;
  threshold_ring_ratio = config.threshold_ring_ratio;
  threshold_ellipse_mean = config.threshold_ellipse_mean;
  filter_ellipse_mean_sample_steps = filter_ellipse_mean_sample_steps;
  
}
