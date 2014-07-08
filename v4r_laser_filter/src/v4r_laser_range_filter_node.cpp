/***************************************************************************
 * Copyright (c) 2014 Markus Bader <markus.bader@tuwien.ac.at>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the TU-Wien.
 * 4. Neither the name of the TU-Wien nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY Markus Bader ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
