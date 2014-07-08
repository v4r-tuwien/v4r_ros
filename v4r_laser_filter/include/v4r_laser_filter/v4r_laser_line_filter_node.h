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
