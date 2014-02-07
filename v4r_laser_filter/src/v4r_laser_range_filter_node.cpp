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

/// ROS Node
class LaserRangeFilterNode {
public:
    LaserRangeFilterNode ( ros::NodeHandle &n )
        :n_ ( n ), n_param_ ( "~" ) {
        sub_ = n_.subscribe("scan", 1000, &LaserRangeFilterNode::callback, this);
        pub_ = n_.advertise<sensor_msgs::LaserScan>("scan_filtered", 1000);
    }
    void callback (const sensor_msgs::LaserScan::ConstPtr& _msg) {
        sensor_msgs::LaserScan msg = *_msg;
	unsigned int nrOfRanges = msg.ranges.size();
	unsigned int nrOfIntensities = msg.intensities.size();
        for (int i = 0; i < nrOfRanges; i++) {
            float &range = msg.ranges[i];
            if (isnan(range)) {
	      // range = msg.range_max-1;
            }
            if (isinf(range)) {
                range = msg.range_max-0.11;
            }
            if( i < nrOfIntensities){
	      ROS_INFO("intensities: %i", msg.header.seq);
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
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "LaserFilter");
    ros::NodeHandle n;
    LaserRangeFilterNode my_node(n);
    ros::spin();
    return 0;
}
