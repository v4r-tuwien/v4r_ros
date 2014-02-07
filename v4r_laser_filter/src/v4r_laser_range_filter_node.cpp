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

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <sstream>

#define MX_LASER_RANGE_FILTER_INF_TO_MAX true
#define MX_LASER_RANGE_FILTER_NAN_TO_MIN false
#define MX_LASER_RANGE_FILTER_MAX_OFFSET -0.01
#define MX_LASER_RANGE_FILTER_MIN_OFFSET 0.01
#define MX_LASER_RANGE_FILTER_MAX  -1
#define MX_LASER_RANGE_FILTER_MIN  -1

/// ROS Node
class LaserRangeFilterNode {
public:
    LaserRangeFilterNode ( ros::NodeHandle &n )
        :n_ ( n ), n_param_ ( "~" ) {
        sub_ = n_.subscribe("scan", 1000, &LaserRangeFilterNode::callback, this);
        pub_ = n_.advertise<sensor_msgs::LaserScan>("scan_filtered", 1000);
	
	n_param_.param<bool>("inf2max", inf2max_, MX_LASER_RANGE_FILTER_INF_TO_MAX);
	ROS_INFO("%s: max_range: %s", n_param_.getNamespace().c_str(), ((inf2max_) ? "true" : "false"));
	n_param_.param<bool>("nan2min", nan2min_, MX_LASER_RANGE_FILTER_NAN_TO_MIN);
	ROS_INFO("%s: min_range: %s", n_param_.getNamespace().c_str(), ((nan2min_) ? "true" : "false"));
	n_param_.param<double>("max_offset", max_offset_, MX_LASER_RANGE_FILTER_MAX_OFFSET);
	ROS_INFO("%s: max_offset: %4.3f", n_param_.getNamespace().c_str(), max_offset_);
	n_param_.param<double>("min_offset", min_offset_, MX_LASER_RANGE_FILTER_MIN_OFFSET);
	ROS_INFO("%s: min_offset: %4.3f", n_param_.getNamespace().c_str(), min_offset_);
	n_param_.param<double>("max", max_, MX_LASER_RANGE_FILTER_MAX);
	ROS_INFO("%s: max: %4.3f", n_param_.getNamespace().c_str(), max_);
	n_param_.param<double>("min", min_, MX_LASER_RANGE_FILTER_MIN);
	ROS_INFO("%s: min: %4.3f", n_param_.getNamespace().c_str(), min_);
	
    }
    void callback (const sensor_msgs::LaserScan::ConstPtr& _msg) {
      if(pub_.getNumSubscribers() == 0) return;
        sensor_msgs::LaserScan msg = *_msg;
	unsigned int nrOfRanges = msg.ranges.size();
	unsigned int nrOfIntensities = msg.intensities.size();
        for (int i = 0; i < nrOfRanges; i++) {
            float &range = msg.ranges[i];
            if (nan2min_ && isnan(range)) {
	      range = msg.range_min + min_offset_;
            }
            if (inf2max_ && isinf(range)) {
                range = msg.range_max + max_offset_;
            }
            if ((max_ > 0.) && range > max_) {
	      range = max_;
            }
            if ((min_ > 0.) && range < min_) {
	      range = min_;
            }
        }        
        pub_.publish(msg);
    }
private:
    ros::Publisher pub_;
    ros::Subscriber sub_;
private: // variables
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    bool inf2max_;
    bool nan2min_;
    double max_offset_;
    double min_offset_;
    double max_;
    double min_;
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "LaserFilter");
    ros::NodeHandle n;
    LaserRangeFilterNode my_node(n);
    ros::spin();
    return 0;
}
