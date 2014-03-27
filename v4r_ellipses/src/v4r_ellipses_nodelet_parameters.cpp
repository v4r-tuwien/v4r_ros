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

using namespace V4R;

EllipsesDetectionNode::ParametersNode::ParametersNode()
    : Parameters() 
    , node("~")
    , node_name(node.getNamespace())
    , debug_freeze(V4R_ELLIPSES_NODE_DEFAULT_DEBUG_FREEZE)
    , show_camera_image(V4R_ELLIPSES_NODE_DEFAULT_SHOW_CAMERA_IMAGE)
    , show_camera_image_waitkey(V4R_ELLIPSES_NODE_DEFAULT_SHOW_CAMERA_IMAGE_WAITKEY)
    , image_skip(V4R_ELLIPSES_NODE_DEFAULT_IMAGE_SKIP)
    , skip_second_tf(V4R_ELLIPSES_NODE_DEFAULT_SKIP_SECOND_TF)
    , tf_prefix(node_name)
    {
    node.getParam("debug_freeze", debug_freeze);
    ROS_INFO("%s - debug_freeze:  %s", node_name.c_str(), (debug_freeze ? "true" : "false"));
    node.getParam("show_camera_image", show_camera_image);
    ROS_INFO("%s - show_camera_image:  %s", node_name.c_str(), (show_camera_image ? "true" : "false"));
    node.getParam("show_camera_image_waitkey", show_camera_image_waitkey);
    ROS_INFO("%s - show_camera_image_waitkey: %i", node_name.c_str(), show_camera_image_waitkey);
    node.getParam("image_skip", image_skip);
    ROS_INFO("%s - image_skip: %i", node_name.c_str(), image_skip);
    node.param<std::string>("tf_prefix", tf_prefix, node_name);
    ROS_INFO("%s: tf_prefix: %s", node_name.c_str(), tf_prefix.c_str());
    node.getParam("skip_second_tf", skip_second_tf);
    ROS_INFO("%s - skip_second_tf:  %s", node_name.c_str(), (skip_second_tf ? "true" : "false"));

    reconfigureFnc_ = boost::bind(&EllipsesDetectionNode::ParametersNode::callbackParameters, this ,  _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);


}


void EllipsesDetectionNode::ParametersNode::callbackParameters (v4r_ellipses::EllipsesDetectionConfig &config, uint32_t level ) {
  int kernal_sizes[] = {1, 3, 5, 7};
  show_camera_image = config.show_camera_image;
  show_camera_image_waitkey = config.show_camera_image_waitkey;
  debug = config.debug;
  distorted_input = config.distorted_input;
  debug_freeze = config.debug_freeze;
  image_skip = config.image_skip;
  edge_detection = (EdgeDetection) config.edge_detection;
  threshold_edge_detection1 = config.threshold_edge_detection1;
  threshold_edge_detection2 = config.threshold_edge_detection2;
  if(config.kernel_size_edge_detection > 7) kernel_size_edge_detection = 7;
  else if(config.kernel_size_edge_detection < 3) kernel_size_edge_detection = 3;
  else if(config.kernel_size_edge_detection % 2) kernel_size_edge_detection = config.kernel_size_edge_detection;
  else kernel_size_edge_detection = config.kernel_size_edge_detection+1;
  edge_linking = (EdgeLinking) config.edge_linking;
  threshold_contour_min_points = config.threshold_contour_min_points;
  threshold_polygon = config.threshold_polygon;
  filter_convex = config.filter_convex;
  ellipse_redefinement = config.ellipse_redefinement;
  threshold_rotated_rect_ratio = config.threshold_rotated_rect_ratio;
  filter_contour_mean = config.filter_contour_mean;
  threshold_min_radius = config.threshold_min_radius;
  threshold_max_radius = config.threshold_max_radius;
  filter_rings = config.filter_rings;
  threshold_ring_center = config.threshold_ring_center;
  threshold_ring_ratio = config.threshold_ring_ratio;
  pose_estimation = (PoseEstimation) config.pose_estimation;
  circle_diameter = config.circle_diameter;
  skip_second_tf = config.skip_second_tf;
  
}
