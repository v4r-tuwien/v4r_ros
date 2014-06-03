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

#include "v4r_laser_filter/v4r_laser_corner_filter_node.h"
#include <fstream>
#include <cfloat>
using namespace LaserFilter;

int main(int argc, char **argv) {

    ros::init(argc, argv, "LaserCornerFilter");
    ros::NodeHandle n;
    LaserCornerFilterNode my_node(n);
    ros::spin();
    return 0;
}

LaserCornerFilterNode::LaserCornerFilterNode ( ros::NodeHandle &n )
    :n_ ( n ), n_param_ ( "~" ), callbackCount(0) {
    sub_ = n_.subscribe("scan", 1000, &LaserCornerFilterNode::callback, this);
    init_marker();
    pub_marker_ =  n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);

    reconfigureFnc_ = boost::bind(&LaserCornerFilterNode::callbackParameters, this,  _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);
}

void LaserCornerFilterNode::callbackParameters ( v4r_laser_filter::CornerFilterConfig &config, uint32_t level ) {
    param_.min_line_length = config.min_line_length;
    param_.min_points_on_line = config.min_points_on_line;
    param_.normal_angle_threshold = config.normal_angle_threshold;
    param_.max_corner_line_differnce = config.max_corner_line_differnce;
    param_.max_line_error = config.max_line_error;
    param_.max_corner_error = config.max_corner_error;
    param_.redefine_corners = config.redefine_corners;
    param_.remove_double_detections = config.remove_double_detections;
    param_.remove_double_detections_threshold = config.remove_double_detections_threshold;
    param_.write_scan = config.write_scan;
    param_.read_scan = config.read_scan;

}

void LaserCornerFilterNode::callback (const sensor_msgs::LaserScan::ConstPtr& _msg) {

    if(param_.write_scan) LaserFilter::writeScan(param_.scan_filename, *_msg);
    if(param_.read_scan)  LaserFilter::readScan(param_.scan_filename, msg_scan_);
    else msg_scan_ = *_msg;
    msg_scan_.header = _msg->header;

    LaserFilter::copy(msg_scan_, measurments_, beams_);

    lines_.clear();
    lines_.reserve(measurments_.size());


    for( int i = 0; i < measurments_.size() - param_.min_points_on_line; i++) {
        for( int j = i+param_.min_points_on_line; j < measurments_.size(); j++) {
            float l = cv::norm(measurments_[i]-measurments_[j]);
            if(l > param_.min_line_length) {
                lines_.push_back(LineSegment(i, j, measurments_));
                break;
            }
        }
    }
    std::vector<Corner> corners_canditates;
    Corner corner;
    for( int i = 0; i < lines_.size(); i++) {
        for( int j = i+1; j < lines_.size(); j++) {
            corner.set(lines_[i], lines_[j]);
            if( corner.l0.idx.second == corner.l1.idx.first) {
                if(corner.isCorner(param_.normal_angle_threshold, param_.max_corner_line_differnce)) {
                    corner.updateError(measurments_);
                    if(corner.l0.distances_error > param_.max_line_error) continue;
                    if(corner.l1.distances_error > param_.max_line_error) continue;
                    if(corner.error > param_.max_corner_error) continue;
                    if(param_.redefine_corners) {
                      corner.redefine(measurments_);
                      if(corner.updateError(measurments_) > param_.max_corner_error) continue;
                    }
                    corners_canditates.push_back(corner);
                }
                break;
            }
        }
    }
    
    corners_.clear();
    if(param_.remove_double_detections){
      int nrOfClusters = 0;
      std::vector< int > corner_clusters(corners_canditates.size(), -1);
      int corner_clusters_representative = -1;
      for(size_t i = 0; i < corners_canditates.size(); i++){
        if(corner_clusters[i] >= 0) continue;
        corner_clusters[i] = nrOfClusters++;
        corner_clusters_representative = i;
        for(size_t j = i+1; j < corners_canditates.size(); j++){
          if(corner_clusters[j] >= 0) continue;
          cv::Vec2f v(corners_canditates[i].point() - corners_canditates[j].point());
          float d = norm(v);
          if(d < param_.remove_double_detections_threshold){
            corner_clusters[j] = corner_clusters[i];
            if(corners_canditates[j].error < corners_canditates[i].error){
              corner_clusters_representative = j;
            }
          }
        }
        corners_.push_back(corners_canditates[corner_clusters_representative]);
      }      
    } else {
      corners_ = corners_canditates;
    }
    
    publish_marker ();
}

void LaserCornerFilterNode::init_marker () {
    msg_corner_pose_.header = msg_scan_.header;
    msg_corner_pose_.lifetime = ros::Duration(10,0);
    msg_corner_pose_.ns = "corners";
    msg_corner_pose_.action = visualization_msgs::Marker::ADD;
    msg_corner_pose_.pose.orientation.w = 1.0;
    msg_corner_pose_.type = visualization_msgs::Marker::LINE_LIST;
    msg_corner_pose_.id = 0;
    msg_corner_pose_.scale.x = 0.01;
    msg_corner_pose_.color.r = 1.0;
    msg_corner_pose_.color.g = 1.0;
    msg_corner_pose_.color.b = 0.0;
    msg_corner_pose_.color.a = 1.0;


    msg_corner_l0_ = msg_corner_pose_;
    msg_corner_l0_.ns = "corners_l0";
    msg_corner_l0_.lifetime = ros::Duration(10,0);
    msg_corner_l0_.id = 0;
    msg_corner_l0_.color.r = 1.0;
    msg_corner_l0_.color.g = 0.0;
    msg_corner_l0_.color.b = 0.2;

    msg_corner_l1_ = msg_corner_pose_;
    msg_corner_l1_.ns = "corners_l1";
    msg_corner_l1_.lifetime = ros::Duration(10,0);
    msg_corner_l1_.id = 0;
    msg_corner_l1_.color.r = 0.0;
    msg_corner_l1_.color.g = 1.0;
    msg_corner_l1_.color.b = 0.2;

}

void LaserCornerFilterNode::publish_marker () {
    geometry_msgs::Point p0, p1;

    msg_corner_pose_.header = msg_scan_.header;
    msg_corner_pose_.points.clear();
    msg_corner_l0_.header = msg_scan_.header;
    msg_corner_l0_.points.clear();
    msg_corner_l1_.header = msg_scan_.header;
    msg_corner_l1_.points.clear();


    for(unsigned int i = 0; i < corners_.size(); i++) {
        p0.x = corners_[i].x(), p0.y = corners_[i].y();
        p1.x = p0.x + cos(corners_[i].angle());
        p1.y = p0.y + sin(corners_[i].angle());
        msg_corner_pose_.points.push_back(p0);
        msg_corner_pose_.points.push_back(p1);

        Line &l0 = corners_[i].l0;
        Line &l1 = corners_[i].l1;
        p0.x = l0.p0.x, p0.y = l0.p0.y;
        p1.x = l0.p1.x, p1.y = l0.p1.y;
        msg_corner_l0_.points.push_back(p0);
        msg_corner_l0_.points.push_back(p1);
        p0.x = l1.p0.x, p0.y = l1.p0.y;
        p1.x = l1.p1.x, p1.y = l1.p1.y;
        msg_corner_l1_.points.push_back(p0);
        msg_corner_l1_.points.push_back(p1);


    }
    pub_marker_.publish(msg_corner_pose_);
    pub_marker_.publish(msg_corner_l0_);
    pub_marker_.publish(msg_corner_l1_);

}
