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

#include "v4r_laser_filter/v4r_laser_line_filter_node.h"

#include <fstream>

using namespace LaserFilter;

int main(int argc, char **argv) {

    ros::init(argc, argv, "LineFilter");
    ros::NodeHandle n;
    LaserLineFilterNode my_node(n);
    ros::spin();
    return 0;
}

LaserLineFilterNode::LaserLineFilterNode ( ros::NodeHandle &n )
    :n_ ( n ), n_param_ ( "~" ) {
    sub_ = n_.subscribe("scan", 1000, &LaserLineFilterNode::callback, this);
    pub_laser_line_split_ = n_.advertise<sensor_msgs::LaserScan>("scan_lines_split", 1000);
    pub_laser_line_fit_ = n_.advertise<sensor_msgs::LaserScan>("scan_lines_fit", 1000);
    pub_laser_input_ = n_.advertise<sensor_msgs::LaserScan>("scan_input", 1000);
    pub_marker_ =  n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
    pub_laser_lines_ = n_.advertise<geometry_msgs::PolygonStamped>("line_segments", 1000);

    double tmp;

    n_param_.param<bool>("publish_marker", param_.publish_marker, V4R_LASER_LINE_FILTER_PUBLISH_MARKER);
    ROS_INFO("%s: publish_marker: %s", n_param_.getNamespace().c_str(), ((param_.publish_marker) ? "true" : "false"));

    n_param_.param<bool>("publish_lines", param_.publish_lines, V4R_LASER_LINE_FILTER_PUBLISH_LINES);
    ROS_INFO("%s: publish_lines: %s", n_param_.getNamespace().c_str(), ((param_.publish_lines) ? "true" : "false"));

    n_param_.param<bool>("fit_lines", param_.fit_lines, V4R_LASER_LINE_FILTER_FIT_LINES);
    ROS_INFO("%s: fit_lines: %s", n_param_.getNamespace().c_str(), ((param_.fit_lines) ? "true" : "false"));

    n_param_.param<bool>("split_scan", param_.split_scan, V4R_LASER_LINE_FILTER_SPLIT_SCAN);
    ROS_INFO("%s: split_scan: %s", n_param_.getNamespace().c_str(), ((param_.split_scan) ? "true" : "false"));


    n_param_.getParam("read_scan", param_.read_scan);
    ROS_INFO("%s: read_scan: %s", n_param_.getNamespace().c_str(), ((param_.read_scan) ? "true" : "false"));
    n_param_.getParam("write_scan", param_.write_scan);
    ROS_INFO("%s: write_scan: %s", n_param_.getNamespace().c_str(), ((param_.write_scan) ? "true" : "false"));
    n_param_.getParam("scan_filename", param_.scan_filename);
    ROS_INFO("%s: scan_filename: %s", n_param_.getNamespace().c_str(), param_.scan_filename.c_str());

    n_param_.param<double>("threshold_split", tmp, V4R_LASER_LINE_FILTER_THRESHOLD_SPLIT);
    param_.threshold_split = tmp;
    ROS_INFO("%s: threshold_split: %4.3f", n_param_.getNamespace().c_str(), param_.threshold_split);
    n_param_.getParam("threshold_split_neighbor", param_.threshold_split_neighbor);
    ROS_INFO("%s: threshold_split_neighbor: %s", n_param_.getNamespace().c_str(), ((param_.threshold_split_neighbor) ? "true" : "false"));

    n_param_.param<double>("min_length", tmp, V4R_LASER_LINE_FILTER_MIN_LENGTH);
    param_.min_length = tmp;
    ROS_INFO("%s: min_length: %4.3f", n_param_.getNamespace().c_str(), param_.min_length);

    n_param_.param<int>("min_points_per_line",  param_.min_points_per_line, V4R_LASER_LINE_FILTER_MIN_POINTS_PER_LINE);
    ROS_INFO("%s: min_points_per_line: %i", n_param_.getNamespace().c_str(), param_.min_points_per_line);

    n_param_.param<double>("min_points_per_meter",  tmp, V4R_LASER_LINE_FILTER_MIN_POINTS_PER_METER);
    param_.min_points_per_meter = tmp;
    ROS_INFO("%s: min_points_per_meter: %4.3f", n_param_.getNamespace().c_str(), param_.min_points_per_meter);



    reconfigureFnc_ = boost::bind(&LaserLineFilterNode::callbackParameters, this,  _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);
}

void LaserLineFilterNode::callbackParameters ( v4r_laser_filter::LineFilterConfig &config, uint32_t level ) {
    param_.publish_marker = config.publish_marker;
    param_.threshold_split = config.threshold_split;
    param_.threshold_split_neighbor = config.threshold_split_neighbor;
    param_.min_length = config.min_length;
    param_.min_points_per_line = config.min_points;
    param_.min_points_per_meter = config.min_points_per_meter;
    param_.fit_lines = config.fit_lines;
    param_.split_scan = config.split_scan;
    param_.write_scan = config.write_scan;
    param_.read_scan = config.read_scan;
}

void LaserLineFilterNode::callback (const sensor_msgs::LaserScan::ConstPtr& _msg) {

    if(param_.write_scan) LaserFilter::writeScan(param_.scan_filename, *_msg);
    if(param_.read_scan)  LaserFilter::readScan(param_.scan_filename, msg_scan_);
    else msg_scan_ = *_msg;

    if(pub_laser_input_.getNumSubscribers() > 0) {
        pub_laser_input_.publish(msg_scan_);
    }

    LaserFilter::copy(msg_scan_, measurments_, beams_);

    splitStart();
    sensor_msgs::LaserScan msg = msg_scan_;
    if(param_.split_scan) {
        for (int i = 0; i < msg_scan_.ranges.size(); i++) msg.ranges[i] = nanf("");

        for(unsigned int i = 0; i < lineSegments_.size(); i++) {
            for(unsigned int idx = lineSegments_[i].idx.first; idx < lineSegments_[i].idx.second; idx++) {
                msg.ranges[idx] = msg_scan_.ranges[idx];
            }
        }
    }
    pub_laser_line_split_.publish(msg);


    if(param_.fit_lines) {
        lineFitStart();
    }

    if(param_.publish_marker) {
        publish_marker();
    }
    
    if(param_.publish_lines){
      msg_lines.header = msg.header;
      msg_lines.polygon.points.clear();
      msg_lines.polygon.points.reserve(lineSegments_.size()*2);
      geometry_msgs::Point32 p0, p1;
      for(unsigned int i = 0; i < lineSegments_.size(); i++) {
          p0.x = lineSegments_[i].p0.x, p0.y = lineSegments_[i].p0.y, p0.z = 0;
          p1.x = lineSegments_[i].p1.x, p1.y = lineSegments_[i].p1.y, p0.z = 0;
          msg_lines.polygon.points.push_back(p0);
          msg_lines.polygon.points.push_back(p1);
      }    
      pub_laser_lines_.publish(msg_lines);
    }
}

void LaserLineFilterNode::publish_marker () {
    msg_line_list_.header = msg_scan_.header;
    msg_line_list_.ns = "lines";
    msg_line_list_.action = visualization_msgs::Marker::ADD;
    msg_line_list_.pose.orientation.w = 1.0;
    msg_line_list_.id = 0;
    msg_line_list_.type = visualization_msgs::Marker::LINE_LIST;
    msg_line_list_.scale.x = 0.01;
    msg_line_list_.color.r = 1.0;
    msg_line_list_.color.g = 0.0;
    msg_line_list_.color.b = 0.0;
    msg_line_list_.color.a = 1.0;
    geometry_msgs::Point p0, p1;
    msg_line_list_.points.clear();
    for(unsigned int i = 0; i < lineSegments_.size(); i++) {
        p0.x = lineSegments_[i].p0.x, p0.y = lineSegments_[i].p0.y;
        p1.x = lineSegments_[i].p1.x, p1.y = lineSegments_[i].p1.y;
        msg_line_list_.points.push_back(p0);
        msg_line_list_.points.push_back(p1);
    }
    pub_marker_.publish(msg_line_list_);
}

void LaserLineFilterNode::lineFitStart() {

    std::vector<cv::Point2f> points;
    lines_.resize(lineSegments_.size());
    for(unsigned int i = 0; i < lineSegments_.size(); i++) {
        lineSegments_[i].estimator_theilsen(measurments_, lineSegments_[i]);
        lines_[i].set(lineSegments_[i].p0,lineSegments_[i].p1);
    }
}

void LaserLineFilterNode::splitStart() {
    connectedMeasurments_.clear();
    lineSegments_.clear();

    if(measurments_.size() > 0) {

        std::pair< unsigned int, unsigned int> idx;
        idx.first = 0;

        while(idx.first < measurments_.size()) {
            idx.second = idx.first + 1;
            float threshold = 4 * cv::norm(measurments_[idx.second] - measurments_[idx.second+1]);
            while((idx.second < measurments_.size()) && (measurments_[idx.second].valid)) {
                if(param_.threshold_split_neighbor) {	    
                    float d = cv::norm(measurments_[idx.second] - measurments_[idx.second+1]);
                    if(d > threshold) {
                        break;
                    }
                    threshold = 4 * d;
                }
                idx.second++;
            }
            if((idx.second - idx.first) > 2) {
                connectedMeasurments_.push_back(idx);
            }
            idx.first = idx.second+1;
        }

        for(unsigned int i = 0; i < connectedMeasurments_.size(); i++) {
            unsigned int idx0 = connectedMeasurments_[i].first;
            unsigned int idx1 = connectedMeasurments_[i].second;
            while((idx0 < connectedMeasurments_[i].second) && (measurments_[idx0].valid == false)) {
                idx0++;
            }
            while((idx1 > idx0) && (measurments_[idx1].valid == false)) {
                idx1--;
            }
            if(idx1 > idx0) {
                LineSegment line;
                line.set(idx0, idx1, measurments_);
                split(line);
            }
        }
    }
}

void LaserLineFilterNode::split(LineSegment &line) {
    unsigned int idxMax=line.idx.first;
    float d;
    float dMax = 0;
    for(unsigned int i = line.idx.first; i < line.idx.second; i++) {
        d = fabs(line.distance(measurments_[i]));
        if(d > dMax) {
            dMax = d,  idxMax = i;
        }
    }
    if(dMax > param_.threshold_split) {
        LineSegment l0,l1;
        if(line.idx.first+param_.min_points_per_line < idxMax) {
            l0.set(line.idx.first, idxMax, measurments_);
            split(l0);
        }
        if(idxMax+param_.min_points_per_line < line.idx.second) {
            l1.set(idxMax, line.idx.second, measurments_);
            split(l1);
        }
    } else {
        if(line.length() < param_.min_length) {
            return;
        }
        if(((float) line.nrSupportPoint()) / line.length() < param_.min_points_per_meter) {
            return;
        }
        if(fabs(line.distance(cv::Point2f(0,0))) < (param_.threshold_split*param_.threshold_split)) {
            // line passed the scan center
            return;
        }
        line.id = lineSegments_.size();
        lineSegments_.push_back(line);
    }
}

