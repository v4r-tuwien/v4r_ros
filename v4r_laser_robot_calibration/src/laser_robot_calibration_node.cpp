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

#include "v4r_laser_robot_calibration/laser_robot_calibration_node.h"

using namespace cv;

Point2f operator*( const Matx<float, 3, 3 > &M, const Point2f& p )
{
    return Point2f(M(0,0)*p.x+M(0,1)*p.y+M(0,2), M(1,0)*p.x+M(1,1)*p.y+M(1,2));
}

Scalar green         (   0, 255,   0);
Scalar green_bright  (  51, 255,  51);
Scalar green_dark    (   0, 102,   0);
Scalar red           (   0,   0, 255);
Scalar blue          ( 255,   0,   0);
Scalar blue_bright   ( 255,  51,  51);
Scalar blue_dark     ( 139,   0,   0);
Scalar orange        (   0, 128, 255);
Scalar yellow        (   0, 255, 255);
Scalar cyan          ( 255, 255,   0);
Scalar magenta       ( 255,   0, 255);
Scalar gray          ( 128, 128, 128);
Scalar black         (   0,   0,   0);
Scalar white         ( 255, 255, 255);

int main(int argc, char **argv) {

    ros::init(argc, argv, "LaserRobotCalibration");
    ros::NodeHandle n;
    LaserRobotCalibrationNode my_node(n);
    ros::spin();
    return 0;
}
LaserRobotCalibrationNode::LaserRobotCalibrationNode ( ros::NodeHandle &n )
    :n_ ( n ), n_param_ ( "~" ), callbackCount(0) {
    sub_laser_ = n.subscribe("scan", 1, &LaserRobotCalibrationNode::callbackLaser, this);
    sub_marker_ = n.subscribe("marker", 1, &LaserRobotCalibrationNode::callbackMarker, this);
}

void LaserRobotCalibrationNode::callbackLaser (const sensor_msgs::LaserScan::ConstPtr& msg) {
    msg_scan_ = *msg;

}
void LaserRobotCalibrationNode::callbackMarker (const geometry_msgs::PoseArray::ConstPtr& msg) {
    msg_marker_ = *msg;
    draw_debug_view();
}

void LaserRobotCalibrationNode::draw_debug_view() {
    cv::Rect rectView(0,0, 500, 500);
    debug_view_.create(rectView.size(), CV_8UC3);
    debug_view_.setTo(Scalar(0xFF,0xFF,0xFF));
    float sx = debug_view_.cols/ msg_scan_.range_max*2;
    float sy = debug_view_.rows/ msg_scan_.range_max*2;
    float robot_view_angle_ = 0;
    float ca = cos(robot_view_angle_+M_PI/2), sa = sin(robot_view_angle_+M_PI/2);

    Matx<float, 3, 3 > Sc (sx, 0, 0, 0, sy, 0, 0, 0, 1);   // scaling
    Matx<float, 3, 3 > Sp (-1, 0, 0, 0, 1, 0, 0, 0, 1);    // mirroring
    Matx<float, 3, 3 > R ( ca, -sa, 0, sa, ca, 0, 0, 0, 1); // rotation
    Matx<float, 3, 3 > T ( 1, 0, debug_view_.cols/2., 0, 1, debug_view_.rows/2., 0, 0, 1);  // translation
    Matx<float, 3, 3 > Mr2v = T * R * Sp * Sc;

    std::vector<Point2f> laser_points(msg_scan_.ranges.size());
    for (int i = 0; i < msg_scan_.ranges.size(); i++) {
        float d = msg_scan_.ranges[i];
        float angle = msg_scan_.angle_min + ( msg_scan_.angle_increment * i );
        laser_points[i].x = cos(angle) * d;
        laser_points[i].y = sin(angle) * d;
        Point2f p = Mr2v * laser_points[i];
        if(p.inside(rectView)) {
            debug_view_.at<cv::Vec3b>(p) =  cv::Vec3b(50,10,10);
        }
    }
    std::vector<Point2f> marker_points(msg_marker_.poses.size());
    std::vector<float>   marker_angles(msg_marker_.poses.size());
    for (int i = 0; i < msg_marker_.poses.size(); i++) {
        marker_points[i].x = msg_marker_.poses[i].position.x;
        marker_points[i].y = msg_marker_.poses[i].position.y;
        Point2f p = Mr2v * marker_points[i];
        if(p.inside(rectView)) {
            circle(debug_view_,p, 2, green,1, CV_AA);
        }
    }
    imshow("LaserRobotCalibration",debug_view_);
    waitKey(100);

}
