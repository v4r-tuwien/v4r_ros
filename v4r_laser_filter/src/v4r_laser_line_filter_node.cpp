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

int main(int argc, char **argv) {

    ros::init(argc, argv, "LaserFilter");
    ros::NodeHandle n;
    LaserLineFilterNode my_node(n);
    ros::spin();
    return 0;
}

LaserLineFilterNode::LaserLineFilterNode ( ros::NodeHandle &n )
    :n_ ( n ), n_param_ ( "~" ) {
    sub_ = n_.subscribe("scan", 1000, &LaserLineFilterNode::callback, this);
    pub_laser_line_split_ = n_.advertise<sensor_msgs::LaserScan>("scan_filtered_lines", 10);
    pub_laser_line_fit_ = n_.advertise<sensor_msgs::LaserScan>("scan_fitted_lines", 10);
    pub_marker_ =  n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    double tmp;

    n_param_.param<bool>("publish_marker", param_.publish_marker, MX_LASER_LINE_FILTER_PUBLISH_MARKER);
    ROS_INFO("%s: min_range: %s", n_param_.getNamespace().c_str(), ((param_.publish_marker) ? "true" : "false"));

    n_param_.param<double>("threshold_split", tmp, MX_LASER_LINE_FILTER_THRESHOLD_SPLIT);
    param_.threshold_split = tmp;
    ROS_INFO("%s: threshold_split: %4.3f", n_param_.getNamespace().c_str(), param_.threshold_split);

    n_param_.param<double>("min_length", tmp, MX_LASER_LINE_FILTER_MIN_LENGTH);
    param_.min_length = tmp;
    ROS_INFO("%s: min_length: %4.3f", n_param_.getNamespace().c_str(), param_.min_length);

    n_param_.param<int>("min_points_per_line",  param_.min_points_per_line, MX_LASER_LINE_FILTER_MIN_POINTS_PER_LINE);
    ROS_INFO("%s: min_points_per_line: %i", n_param_.getNamespace().c_str(), param_.min_points_per_line);

    n_param_.param<double>("min_points_per_meter",  tmp, MX_LASER_LINE_FILTER_MIN_POINTS_PER_METER);
    param_.min_points_per_meter = tmp;
    ROS_INFO("%s: min_points_per_meter: %4.3f", n_param_.getNamespace().c_str(), param_.min_points_per_meter);

    reconfigureFnc_ = boost::bind(&LaserLineFilterNode::callbackParameters, this,  _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);
}

void LaserLineFilterNode::callbackParameters ( v4r_laser_filter::LineFilterConfig &config, uint32_t level ) {
    param_.publish_marker = config.publish_marker;
    param_.threshold_split = config.threshold_split;
    param_.min_length = config.min_length;
    param_.min_points_per_line = config.min_points;
    param_.min_points_per_meter = config.min_points_per_meter;
}

void LaserLineFilterNode::callback (const sensor_msgs::LaserScan::ConstPtr& _msg) {
    if((pub_laser_line_split_.getNumSubscribers() == 0) && (pub_laser_line_split_.getNumSubscribers() == 0)) {
        //    return;
    }
    msg_scan_ = *_msg;
    unsigned int nrOfRanges = msg_scan_.ranges.size();
    measurments_.resize(nrOfRanges);
    for (int i = 0; i < nrOfRanges; i++) {
        float &range = msg_scan_.ranges[i];
        float alpha = msg_scan_.angle_min + (msg_scan_.angle_increment * i);
        measurments_[i].set(alpha, range);
    }

		splitStart();

		sensor_msgs::LaserScan msg = *_msg;
		for (int i = 0; i < nrOfRanges; i++) msg.ranges[i] = nanf("");

		for(unsigned int i = 0; i < lineSegments_.size(); i++) {
			for(unsigned int idx = lineSegments_[i].idx0; idx < lineSegments_[i].idx1; idx++) {
				msg.ranges[idx] = msg_scan_.ranges[idx];
			}
		}
		pub_laser_line_split_.publish(msg);

		if(pub_laser_line_split_.getNumSubscribers() > 0) {
			lineFitStart();
		}
	

    if(param_.publish_marker) {
        publish_marker();
    }
}

void LaserLineFilterNode::publish_marker () {
	if(pub_marker_.getNumSubscribers() == 0) return;
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
    //msg_line_list_.lifetime;
    pub_marker_.publish(msg_line_list_);


    msg_line_list_.id = 1;
    msg_line_list_.color.r = 0.0;
    msg_line_list_.color.g = 1.0;
    msg_line_list_.color.b = 0.0;
    msg_line_list_.color.a = 1.0;
    msg_line_list_.points.clear();
    for(unsigned int i = 0; i < lines_.size(); i++) {
        p0.x = lines_[i].p0.x, p0.y = lines_[i].p0.y;
        p1.x = lines_[i].p1.x, p1.y = lines_[i].p1.y;
        msg_line_list_.points.push_back(p0);
        msg_line_list_.points.push_back(p1);
    }
    //msg_line_list_.lifetime;
    pub_marker_.publish(msg_line_list_);
}

void LaserLineFilterNode::lineFitStart() {

    std::vector<Point> points;
    lines_.resize(lineSegments_.size());
    for(unsigned int i = 0; i < lineSegments_.size(); i++) {
        points.resize(lineSegments_[i].points.size());

        float ox = lineSegments_[i].p0.x;
        float oy = lineSegments_[i].p0.y;
        float dx = lineSegments_[i].p1.x - lineSegments_[i].p0.x;
        float dy = lineSegments_[i].p1.y - lineSegments_[i].p0.y;
        float alpha = atan2(dy,dx)+M_PI/4.;
        float s = sin(alpha), c = cos(alpha);
        for(unsigned int j = 0; j < points.size(); j++) {
            const Point &src = lineSegments_[i].points[j];
            Point &des = points[j];
            des.x =  c * (src.x - ox) + s * (src.y - oy);
            des.y = -s * (src.x - ox) + c * (src.y - oy);
        }
        Point start = points.front(), end = points.back();
        theilsen(points, start, end);
        Point p0(c * start.x - s * start.y + ox,  s * start.x + c * start.y + oy );
        Point p1(c * end.x - s * end.y     + ox,  s * end.x   + c * end.y   + oy );
        lines_[i].set(p0,p1);
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
            while((idx.second < measurments_.size()) && (measurments_[idx.second].valid)) {
                idx.second++;
            }
            if((idx.second - idx.first) > 1) {
                connectedMeasurments_.push_back(idx);
            }
            idx.first = idx.second;
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
    unsigned int idxMax=line.idx0;
    float d;
    float dMax = 0;
    for(unsigned int i = line.idx0; i < line.idx1; i++) {
        d = fabs(line.eq.distance(measurments_[i]));
        if(d > dMax) {
            dMax = d,  idxMax = i;
        }
    }
    if(dMax > param_.threshold_split) {
        LineSegment l0,l1;
        if(line.idx0+param_.min_points_per_line < idxMax) {
            l0.set(line.idx0, idxMax, measurments_);
            split(l0);
        }
        if(idxMax+param_.min_points_per_line < line.idx1) {
            l1.set(idxMax, line.idx1, measurments_);
            split(l1);
        }
    } else {
        if(line.length() < param_.min_length) {
            return;
        }
        if(((float) line.nrSupportPoint()) / line.length() < param_.min_points_per_meter) {
            return;
        }
        if(fabs(line.eq.distance(Point(0,0))) < (param_.threshold_split*param_.threshold_split)) {
            // line passed the scan center
            return;
        }
        line.id = lineSegments_.size();
        line.updatePoints(measurments_);
        lineSegments_.push_back(line);
    }
}

LaserLineFilterNode::Line::Line(const Point &_p0, const Point &_p1) {
    set(_p0,_p1);
};
void LaserLineFilterNode::Line::set(const Point &_p0, const Point &_p1) {
    p0 = _p0, p1 = _p1;
    eq.set(p0, p1);
}
void LaserLineFilterNode::LineSegment::set(unsigned int _idx0, unsigned int _idx1, const std::vector<Measurment> &measurments) {
    idx0 = _idx0, idx1 = _idx1;
    Line::set(measurments[idx0], measurments[idx1]);
}

float LaserLineFilterNode::Line::length() {
    float dx = p1.x-p0.x, dy = p1.y-p0.y;
    return sqrt(dx*dx+dy*dy);
}
void LaserLineFilterNode::LineSegment::updatePoints(const std::vector<Measurment> &measurments) {
    if((idx0 < idx1) && (idx0 >= 0) && (idx1 <  measurments.size())) {
        points.clear();
        points.reserve(idx1-idx0+1);
        for(unsigned int i = idx0; i <= idx1; i++) {
            points.push_back(measurments[i]);
        }
    }
}
bool LaserLineFilterNode::LineSegment::isSupportPoint(int idx) {
    if( (idx < idx0) || (idx > idx1)) {
        return false;
    } else {
        return true;
    }
}
unsigned int LaserLineFilterNode::LineSegment::nrSupportPoint() {
    return idx1 - idx0 + 2;
}

void LaserLineFilterNode::LineEq::set(const Point &p0, const Point &p1) {
    a = +(p0.y *   1. - 1.   * p1.y);
    b = -(p0.x *   1. - 1.   * p1.x);
    c = +(p0.x * p1.y - p0.y * p1.x);
    float r = sqrt(a*a+b*b);
    a = a / r, b = b / r, c = c / r;
}
float LaserLineFilterNode::LineEq::distance(const Point &p) {
    return a*p.x+b*p.y+c;
}



void LaserLineFilterNode::theilsen(const std::vector<Point> &points, Point &start, Point &end) {
    if(points.size() <= 0) return;
    float slope, offset;
    std::vector<float> slops;
    slops.reserve(points.size()*points.size());
    for(unsigned int i = 0; i < points.size(); i++) {
        for(unsigned int j = 0; j < points.size(); j++) {
            if(i == j) continue;
            const Point &pi = points[i];
            const Point &pj = points[j];
            slops.push_back((pi.y-pj.y)/(pi.x-pj.x));
        }
    }
    std::sort(slops.begin(), slops.end());
    slope = slops[slops.size()/2];
    if ((slops.size() % 2) == 0) {
        slope = (slope + slops[slops.size()/2 - 1]) / 2.;
    }

    std::vector<float> offsets;
    offsets.reserve(points.size());

    for(unsigned int i = 0; i < points.size(); i++) {
        const Point &p = points[i];
        offsets.push_back(p.y - slope*p.x);
    }

    std::sort(offsets.begin(), offsets.end());
    offset = offsets[offsets.size()/2];
    if ((offsets.size() % 2) == 0) {
        offset = (offset + offsets[offsets.size()/2 - 1]) / 2.;
    } else {

    }
    start.y = slope * start.x + offset;
    end.y = slope * end.x + offset;
}
