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
#include <v4r_laser_filter/LineFilterConfig.h>
#include <v4r_laser_filter/v4r_laser_filter_utils.h>

#ifndef V4R_LASER_LINE_FILTER_NODE
#define V4R_LASER_LINE_FILTER_NODE


#define V4R_LASER_LINE_FILTER_PUBLISH_MARKER false
#define V4R_LASER_LINE_FILTER_PUBLISH_LINES true
#define V4R_LASER_LINE_FILTER_THRESHOLD_SPLIT 0.02
#define V4R_LASER_LINE_FILTER_MIN_LENGTH 0.2
#define V4R_LASER_LINE_FILTER_MIN_POINTS_PER_LINE 20
#define V4R_LASER_LINE_FILTER_MIN_POINTS_PER_METER 10
#define V4R_LASER_LINE_FILTER_FIT_LINES false
#define V4R_LASER_LINE_FILTER_SPLIT_SCAN true

/// ROS Node
class LaserLineFilterNode {
public:
    struct Parameters {
        Parameters()
            : publish_marker(V4R_LASER_LINE_FILTER_PUBLISH_MARKER)
            , publish_lines(V4R_LASER_LINE_FILTER_PUBLISH_LINES)
            , threshold_split(V4R_LASER_LINE_FILTER_THRESHOLD_SPLIT)
            , threshold_split_neighbor(true)
            , min_length(V4R_LASER_LINE_FILTER_MIN_LENGTH)
            , min_points_per_line(V4R_LASER_LINE_FILTER_MIN_POINTS_PER_LINE)
            , min_points_per_meter(V4R_LASER_LINE_FILTER_MIN_POINTS_PER_METER)
            , split_scan(V4R_LASER_LINE_FILTER_SPLIT_SCAN)
            , fit_lines(V4R_LASER_LINE_FILTER_FIT_LINES)
            , write_scan(false)
            , read_scan(false)
            , scan_filename("/tmp/scan.bin") {};
        bool publish_marker;
        bool publish_lines;
        float threshold_split;
        bool threshold_split_neighbor;
        float min_length;
        int min_points_per_line;
        float min_points_per_meter;
        bool split_scan;
        bool fit_lines;
        bool write_scan;
        bool read_scan;
        std::string scan_filename;
    };
    LaserLineFilterNode ( ros::NodeHandle &n );
    void callback (const sensor_msgs::LaserScan::ConstPtr& msg);
    void callbackParameters ( v4r_laser_filter::LineFilterConfig &config, uint32_t level );
private:
    void split(LaserFilter::LineSegment &line);
    void splitStart();
    void readScan(const std::string &filename, sensor_msgs::LaserScan &msg );
    void writeScan(const std::string &filename, const sensor_msgs::LaserScan &msg );
    void lineFitStart();
    void publish_marker();
private: // variables
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    Parameters param_;
    ros::Publisher pub_laser_lines_;
    ros::Publisher pub_laser_line_split_;
    ros::Publisher pub_laser_line_fit_;
    ros::Publisher pub_laser_input_;
    ros::Publisher pub_marker_;
    ros::Subscriber sub_;
    dynamic_reconfigure::Server<v4r_laser_filter::LineFilterConfig> reconfigureServer_;
    dynamic_reconfigure::Server<v4r_laser_filter::LineFilterConfig>::CallbackType reconfigureFnc_;
    sensor_msgs::LaserScan msg_scan_;
    geometry_msgs::PolygonStamped msg_lines;
    visualization_msgs::Marker msg_line_list_;
    std::vector<LaserFilter::Beam> beams_;
    std::vector<LaserFilter::Measurment> measurments_;
    std::vector<std::pair<unsigned int, unsigned int> > connectedMeasurments_;
    std::vector<LaserFilter::LineSegment> lineSegments_;
    std::vector<LaserFilter::Line> lines_;
    float  angle_increment_;
    float  angle_increment_sin_;

};

#endif // V4R_LASER_LINE_FILTER_NODE
