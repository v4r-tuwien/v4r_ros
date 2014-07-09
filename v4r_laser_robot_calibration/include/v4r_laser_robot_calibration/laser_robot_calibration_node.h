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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/opencv.hpp>

#ifndef V4R_LASER_ROBOT_CALIBRATION_NODE
#define V4R_LASER_ROBOT_CALIBRATION_NODE


/// ROS Node
class LaserRobotCalibrationNode {
    cv::Matx<float, 3, 3 > Mr2v_;
    enum States{
      INIT,
      ROTATE_TO_MARKER,
      ROTATE_NORMAL_TO_MARKER,
    };
public:
    LaserRobotCalibrationNode ( ros::NodeHandle &n );
    void callbackLaser (const sensor_msgs::LaserScan::ConstPtr& msg);
    void callbackMarker (const geometry_msgs::PoseArray::ConstPtr& msg);
    void stateMashine();
private: // variables
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    unsigned callbackCount;
    ros::Subscriber sub_laser_;
    ros::Subscriber sub_marker_;
    ros::Publisher pub_cmd_;
    cv::Mat debug_view_;
    sensor_msgs::LaserScan msg_scan_;
    geometry_msgs::PoseArray msg_marker_;
    void draw_debug_view();
    void rotateToMarkerNearPoint(cv::Point2f anchor);
    cv::Point2f anchor_;
    cv::Vec3f marker_;
    States state_;

    static void mouseCallBack ( int evt, int c, int r, int flags, void *param ) {
        if ( evt == CV_EVENT_LBUTTONDOWN ) {
            LaserRobotCalibrationNode *pLaserRobotCalibrationNode = (LaserRobotCalibrationNode *) param;
            cv::Matx<float, 3, 3 > M = pLaserRobotCalibrationNode->Mr2v_.inv();
            cv::Point2f p(c,r);
            cv::Point2f anchor(M(0,0)*p.x+M(0,1)*p.y+M(0,2), M(1,0)*p.x+M(1,1)*p.y+M(1,2));
            pLaserRobotCalibrationNode->rotateToMarkerNearPoint(anchor);
        }
        if ( evt == CV_EVENT_MBUTTONDOWN ) {
        }
    }
};

#endif //V4R_LASER_ROBOT_CALIBRATION_NODE

