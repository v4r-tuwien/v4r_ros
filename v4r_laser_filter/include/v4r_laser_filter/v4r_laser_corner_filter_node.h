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
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <sstream>

#include <dynamic_reconfigure/server.h>
#include <v4r_laser_filter/CornerFilterConfig.h>
#include <v4r_laser_filter/v4r_laser_filter_utils.h>

#ifndef V4R_LASER_CORNER_FILTER_NODE
#define V4R_LASER_CORNER_FILTER_NODE


/// ROS Node
class LaserCornerFilterNode {
public:
    struct Parameters {
        Parameters()
            : publish_marker(true)
            , min_line_length(0.1)
            , min_points_on_line(10)
            , normal_angle_threshold(0.1)
            , max_corner_line_differnce(1.2)
            , max_line_error(0.5)
            , max_corner_error(0.5)
            , redefine_corners(true)
            , remove_double_detections(true)
            , remove_double_detections_threshold(0.1)
            , write_scan(false)
            , read_scan(false)
            , scan_filename("/tmp/scan.bin")
            {};
        bool publish_marker;
        float min_line_length;
        int min_points_on_line;
        float normal_angle_threshold;
        float max_corner_line_differnce;
        float max_line_error;
        float max_corner_error;
        bool redefine_corners;
        bool remove_double_detections;
        float remove_double_detections_threshold;
        bool write_scan;
        bool read_scan;
        std::string scan_filename;
    };
    LaserCornerFilterNode ( ros::NodeHandle &n );
    void callback (const sensor_msgs::LaserScan::ConstPtr& msg);
    void callbackParameters ( v4r_laser_filter::CornerFilterConfig &config, uint32_t level );
private:
    void init_marker();
    void publish_marker();
private: // variables
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    Parameters param_;
    ros::Subscriber sub_;
    ros::Publisher pub_marker_;
    unsigned callbackCount;
    visualization_msgs::Marker msg_corner_pose_;
    visualization_msgs::Marker msg_corner_pose_estimation_;
    visualization_msgs::Marker msg_corner_l0_;
    visualization_msgs::Marker msg_corner_l0_estimation_;
    visualization_msgs::Marker msg_corner_l1_;
    visualization_msgs::Marker msg_corner_l1_estimation_;
    visualization_msgs::Marker msg_corner_text_;
    dynamic_reconfigure::Server<v4r_laser_filter::CornerFilterConfig> reconfigureServer_;
    dynamic_reconfigure::Server<v4r_laser_filter::CornerFilterConfig>::CallbackType reconfigureFnc_;
    sensor_msgs::LaserScan msg_scan_;
    std::vector<LaserFilter::Measurment> measurments_;
    std::vector<LaserFilter::Beam> beams_;
    std::vector<LaserFilter::LineSegment> lines_;
    std::vector<LaserFilter::Corner> corners_;

};

#endif // V4R_LASER_CORNER_FILTER_NODE
