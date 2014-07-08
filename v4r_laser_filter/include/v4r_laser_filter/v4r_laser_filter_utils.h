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
#include <sstream>
#include <boost/iterator/iterator_concepts.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

#ifndef V4R_LASER_FILTER
#define V4R_LASER_FILTER

namespace LaserFilter {
class Pose : public cv::Vec3f {
public:
    Pose();
    Pose(float _x, float _y, float _angle);
    void set( float _x, float _y, float _angle);
    const cv::Point2f& point() const;
    cv::Point2f& point();
    const float& x() const;
    float& x();
    const float& y() const;
    float& y() ;
    const float& angle() const;
    float& angle();
    void normalizeAngle();
};
class Beam {
public:
    Beam();
    void set(float _alpha, float _range);
    float alpha;
    float range;
};
class Measurment : public cv::Point2f {
public:
    Measurment(): cv::Point2f() {};
    Measurment(float _x, float _y) : cv::Point2f(_x, _y) {};
    void set(float _alpha, float _range);
    bool valid;
    bool inf;
    bool nan;
};
class LineEq : public cv::Vec3f {
public:
    LineEq();
    LineEq(const cv::Vec3f& l);
    LineEq(float _a, float _b, float _c);
    void set(const cv::Point2f &p0, const cv::Point2f &p1);
    float distance(const cv::Point2f &p) const;
};
class Line : public LineEq {
public:
    Line();
    Line(const cv::Vec3f& l) ;
    Line(const Line& l) ;
    Line(const cv::Point2f &_p0, const cv::Point2f &_p1);
    void set(const cv::Point2f &_p0, const cv::Point2f &_p1);
    void computeEquation();
    float length() const;
    float angle() const;
    float angleDiff(const Line &line) const;
    cv::Vec2f vector() const;
    cv::Point2f p0, p1;
};
class LineSegment : public Line {
public:
    LineSegment();
    LineSegment(const cv::Vec3f& l);
    LineSegment(const Line& l);
    LineSegment(const LineSegment& l);
    LineSegment(unsigned int idx0, unsigned int idx1,  const std::vector<Measurment> &measurments);
    void set(unsigned int _idx0, unsigned int _idx1, const std::vector<Measurment> &measurments);
    void updateLineStatistic(const std::vector<Measurment> &measurments);
    bool isSupportPoint(int idx);
    unsigned int nrSupportPoint();
    unsigned int id;
    std::pair<unsigned int, unsigned int> idx;
    float distances_mean;
    float distances_variance;
    float distances_error;
    void estimator_theilsen(const std::vector<Measurment> &measurments, Line &line);
private:
  
};

class Corner : public Pose {
public:
    Corner();
    Corner(const Corner &c);
    void set(const LineSegment &_l0, const LineSegment &_l1);
    bool isCorner(float threshold_angle, float max_line_diff); 
    void redefine(const std::vector<Measurment> &measurments);
    float updateError(const std::vector<Measurment> &measurments);
    float error;
    LineSegment l0, l1;
};

void copy(const sensor_msgs::LaserScan& _msg, std::vector<Measurment> &_desMeasurments, std::vector<Beam> &_desBeams);
void intersection(const cv::Vec3f &l1, const cv::Vec3f &l2, cv::Point2f &p);
float dot(const cv::Vec3f &l1, const cv::Point2f &p);
void readScan(const std::string &filename, sensor_msgs::LaserScan &msg );
void writeScan(const std::string &filename, const sensor_msgs::LaserScan &msg );
};
#endif // V4R_LASER_LINE_FILTER_NODE
