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
#include <geometry_msgs/PoseArray.h>
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
    void init_marker_visualization();
    void publish_marker_visualization();
    void publish_marker_posearray();
private: // variables
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    Parameters param_;
    ros::Subscriber sub_;
    ros::Publisher pub_marker_visualization_;
    ros::Publisher pub_marker_posearray_;
    unsigned callbackCount;
    visualization_msgs::Marker msg_corner_pose_;
    visualization_msgs::Marker msg_corner_pose_estimation_;
    visualization_msgs::Marker msg_corner_l0_;
    visualization_msgs::Marker msg_corner_l0_estimation_;
    visualization_msgs::Marker msg_corner_l1_;
    visualization_msgs::Marker msg_corner_l1_estimation_;
    visualization_msgs::Marker msg_corner_text_;
    geometry_msgs::PoseArray msg_marker_posearray_;
    dynamic_reconfigure::Server<v4r_laser_filter::CornerFilterConfig> reconfigureServer_;
    dynamic_reconfigure::Server<v4r_laser_filter::CornerFilterConfig>::CallbackType reconfigureFnc_;
    sensor_msgs::LaserScan msg_scan_;
    std::vector<LaserFilter::Measurment> measurments_;
    std::vector<LaserFilter::Beam> beams_;
    std::vector<LaserFilter::LineSegment> lines_;
    std::vector<LaserFilter::Corner> corners_;

};

#endif // V4R_LASER_CORNER_FILTER_NODE
