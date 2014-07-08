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

#include "v4r_laser_filter/v4r_laser_filter_utils.h"

#include <fstream>
#include <boost/foreach.hpp>
using namespace LaserFilter;

// Start Pose ---------------------------------------------

Pose::Pose() {};
Pose::Pose(float _x, float _y, float _angle): cv::Vec3f(_x, _y, _angle) {};
void Pose::set( float _x, float _y, float _angle) {
    val[0] = _x, val[1] = _y, val[2] = _angle;
}
const cv::Point2f& Pose::point() const {
    return *((cv::Point2f*) val);
}
cv::Point2f& Pose::point() {
    return *((cv::Point2f*) val);
}
const float& Pose::x() const {
    return val[0];
};
float& Pose::x() {
    return val[0];;
};
const float& Pose::y() const {
    return val[1];
};
float& Pose::y() {
    return val[1];;
};
const float& Pose::angle() const {
    return val[2];
};
float& Pose::angle() {
    return val[2];
};
void Pose::normalizeAngle() {
    while(val[2] >  M_PI) val[2] -= 2*M_PI;
    while(val[2] < -M_PI) val[2] += 2*M_PI;
}

//  End Pose ---------------------------------------------

//  Start Beam ---------------------------------------------
Beam::Beam() {};
void Beam::set(float _alpha, float _range) {
    alpha = _alpha, range = _range;
}
//  End Beam ---------------------------------------------


//  Start LineEq ---------------------------------------------
LineEq::LineEq() {};

LineEq::LineEq(const cv::Vec3f& l)
    :cv::Vec3f(l) {};

LineEq::LineEq(float _a, float _b, float _c)
    : cv::Vec3f(_a, _b, _c) {};

void LineEq::set(const cv::Point2f &p0, const cv::Point2f &p1) {
    val[0] = +(p0.y *   1. - 1.   * p1.y);
    val[1] = -(p0.x *   1. - 1.   * p1.x);
    val[2] = +(p0.x * p1.y - p0.y * p1.x);
    float r = sqrt( val[0]* val[0]+ val[1]* val[1]);
    val[0] =  val[0] / r,  val[1] =  val[1] / r,  val[2] =  val[2] / r;
}
float LineEq::distance(const cv::Point2f &p) const {
    return LaserFilter::dot(*this, p);
}
//  End LineEq ---------------------------------------------


//  Start Line ---------------------------------------------

Line::Line()
    :LineEq() {};
Line::Line(const cv::Vec3f& l)
    :LineEq(l) {};
Line::Line(const Line& l)
    :LineEq(l), p0(l.p0), p1(l.p1) {};
Line::Line(const cv::Point2f &_p0, const cv::Point2f &_p1) {
    set(_p0,_p1);
};
void Line::set(const cv::Point2f &_p0, const cv::Point2f &_p1) {
    p0 = _p0, p1 = _p1;
    computeEquation();
}
void Line::computeEquation() {
    LineEq::set(p0, p1);
}
float Line::angle() const {
    cv::Vec2f v = vector();
    return atan2(v[1], v[0]);
}
float Line::angleDiff(const Line &l) const {
    float a0 = this->angle();
    float a1 = l.angle();
    float angle_between_lines = atan2(sin(a1 - a0), cos(a1 - a0));
    return angle_between_lines;
}
float Line::length() const {
    return norm(vector());
}
cv::Vec2f Line::vector() const {
    return cv::Vec2f(p1.x-p0.x, p1.y-p0.y);
}
//  End Line ---------------------------------------------

//  Start LineSegment ---------------------------------------------

LineSegment::LineSegment()
    :Line(), id(0), idx(0,0) {}
LineSegment::LineSegment(const cv::Vec3f& l)
    : Line(l), id(0), idx(0,0) {}
LineSegment::LineSegment(const Line& l)
    : Line(l), id(0), idx(0,0) {}
LineSegment::LineSegment(const LineSegment& l)
    : Line(l), id(l.id), idx(l.idx) {}
LineSegment::LineSegment(unsigned int idx0, unsigned int idx1,  const std::vector<Measurment> &measurments)
    : Line(measurments[idx0], measurments[idx1]), id(0), idx(idx0,idx1) {}

bool LineSegment::isSupportPoint(int i) {
    if( (i < idx.first) || (i > idx.second)) {
        return false;
    } else {
        return true;
    }
}
unsigned int LineSegment::nrSupportPoint() {
    return idx.second - idx.first + 1;
}
void LineSegment::set(unsigned int _idx0, unsigned int _idx1, const std::vector<Measurment> &measurments) {
    idx.first = _idx0, idx.second = _idx1;
    Line::set(measurments[idx.first], measurments[idx.second]);
}

void LineSegment::updateLineStatistic(const std::vector<Measurment> &measurments) {
    float sum_mean = 0, sum_var = 0;
    for(unsigned int i = idx.first; i <= idx.second; i++) {
        float d = this->distance(measurments[i]);
        sum_mean += d;
        sum_var += d*d;
    }
    distances_mean = sum_mean / nrSupportPoint();
    distances_variance = sum_var / nrSupportPoint();
    distances_error = sqrt(distances_variance) / length();
}


void LineSegment::estimator_theilsen(const std::vector<Measurment> &measurments, Line &line) {
    // http://en.wikipedia.org/wiki/Theil%E2%80%93Sen_estimator
    unsigned nrOfPoints = nrSupportPoint();
    if(nrSupportPoint() <= 3) return;
    float slopeYX, slopeXY, offset;
    std::vector<float> slopsXY, slopsYX, offsets;
    slopsYX.reserve(nrOfPoints);
    slopsXY.reserve(nrOfPoints);
    for(unsigned int i = idx.first; i <= idx.second; i++) {
        for(unsigned int j = i; j <= idx.second; j++) {
            if(i == j) continue;
            const cv::Point2f &pi = measurments[i];
            const cv::Point2f &pj = measurments[j];
            float dx = pi.x-pj.x;
            float dy = pi.y-pj.y;
            if(fabs(dx) > FLT_MIN) {
                float s = dy/dx;
                slopsYX.push_back(s);
            }
            if(fabs(dy) > FLT_MIN) {
                float s = dx/dy;
                slopsXY.push_back(s);
            }
        }
    }
    std::sort(slopsYX.begin(), slopsYX.end());
    slopeYX = slopsYX[ (slopsYX.size() % 2?slopsYX.size() / 2:slopsYX.size()/2 - 1) ];
    std::sort(slopsXY.begin(), slopsXY.end());
    slopeXY = slopsXY[ (slopsXY.size() % 2?slopsXY.size() / 2:slopsXY.size()/2 - 1) ];

    offsets.reserve(nrOfPoints);
    if(fabs(slopeYX) < 1.0) {

        for(unsigned int i = idx.first; i <= idx.second; i++) {
            const cv::Point2f &p = measurments[i];
            offsets.push_back(p.y - slopeYX*p.x);
        }

        std::sort(offsets.begin(), offsets.end());
        offset = offsets[offsets.size()/2];
        if ((offsets.size() % 2) == 0) {
            offset = (offset + offsets[offsets.size()/2 - 1]) / 2.;
        } else {

        }
        line.p0.x = measurments[idx.first].x;
        line.p1.x = measurments[idx.second].x;
        line.p0.y = slopeYX * line.p0.x + offset;
        line.p1.y = slopeYX * line.p1.x + offset;
        line.computeEquation();
    } else {
        for(unsigned int i = idx.first; i <= idx.second; i++) {
            const cv::Point2f &p = measurments[i];
            offsets.push_back(p.x - slopeXY*p.y);
        }

        std::sort(offsets.begin(), offsets.end());
        offset = offsets[offsets.size()/2];
        if ((offsets.size() % 2) == 0) {
            offset = (offset + offsets[offsets.size()/2 - 1]) / 2.;
        } else {

        }
        line.p0.y = measurments[idx.first].y;
        line.p1.y = measurments[idx.second].y;
        line.p0.x = slopeXY * line.p0.y + offset;
        line.p1.x = slopeXY * line.p1.y + offset;
        line.computeEquation();
    }
}
//  End LineSegment ---------------------------------------------


//  Start Measurment ---------------------------------------------

void Measurment::set(float _alpha, float _range) {
    nan = isnan(_range), inf = isinf(_range);
    if(nan || inf) {
        valid = false;
    } else {
        x = cos(_alpha) * _range;
        y = sin(_alpha) * _range;
        valid = true;
    }
}
//  End Measurment ---------------------------------------------

//  Start Corner ---------------------------------------------

Corner::Corner() 
: error(FLT_MAX) {};

Corner::Corner(const Corner &c)
: Pose(c), error(c.error), l0(c.l0), l1(c.l1)  {};

void Corner::set(const LineSegment &_l0, const LineSegment &_l1) {
    l0 = _l0, l1 = _l1;
    intersection(l0, l1, point());
    float angle_between_lines = l0.angleDiff(l1);
    this->angle() = l0.angle() - angle_between_lines / 2;
    normalizeAngle();
}

bool Corner::isCorner(float threshold_angle, float max_line_diff) {
    bool result = false;
    float line_diff;
    float angle_between_lines = l0.angleDiff(l1);    
    if(fabs(fabs(angle_between_lines) - M_PI/2) < threshold_angle) {
        intersection(l0, l1, point());
        float n1 = l0.length();
        float n2 = l1.length();
        line_diff = ( n1>n2 ? n1/n2 : n2/n1);
        if(line_diff < max_line_diff) {
            result = true;
        }
    }
    return result;
}

void Corner::redefine(const std::vector<Measurment> &measurments) {
    l0.estimator_theilsen(measurments, l0);
    l1.estimator_theilsen(measurments, l1);
    set(l0, l1);
}
float Corner::updateError(const std::vector<Measurment> &measurments){
  l0.updateLineStatistic(measurments);
  l1.updateLineStatistic(measurments);
  error = sqrt(l0.distances_error*l0.distances_error + l1.distances_error*l1.distances_error);
  return error;
}

//  End Corner ---------------------------------------------


//  Start General Functions ---------------------------------------------

void LaserFilter::copy(const sensor_msgs::LaserScan& _msg, std::vector<Measurment> &_desMeasurments, std::vector<Beam> &_desBeams) {
    unsigned int nrOfRanges = _msg.ranges.size();
    _desMeasurments.resize(nrOfRanges);
    _desBeams.resize(nrOfRanges);
    float angle_increment_ = _msg.angle_increment;
    float angle_increment_sin_ = sin(angle_increment_);
    for (int i = 0; i < nrOfRanges; i++) {
        const float &range = _msg.ranges[i];
        float alpha = _msg.angle_min + (_msg.angle_increment * i);
        _desBeams[i].set(alpha, range);
        _desMeasurments[i].set(alpha, range);
    }
}

void LaserFilter::intersection(const cv::Vec3f &l1, const cv::Vec3f &l2, cv::Point2f &p) {
    cv::Vec3f c = l1.cross(l2);
    p.x = c[0] / c[2], p.y = c[1] / c[2];
}
float LaserFilter::dot(const cv::Vec3f &l1, const cv::Point2f &p) {
    return l1[0]*p.x+ l1[1]*p.y+ l1[2];
}

void LaserFilter::readScan(const std::string &filename, sensor_msgs::LaserScan &msg ) {

    std::ifstream ifs(filename.c_str(), std::ios::in|std::ios::binary);
    ifs.seekg (0, std::ios::end);
    std::streampos end = ifs.tellg();
    ifs.seekg (0, std::ios::beg);
    std::streampos begin = ifs.tellg();

    uint32_t file_size = end-begin;
    boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
    ifs.read((char*) ibuffer.get(), file_size);
    ros::serialization::IStream istream(ibuffer.get(), file_size);
    ros::serialization::deserialize(istream, msg);
    ifs.close();
}
void LaserFilter::writeScan(const std::string &filename, const sensor_msgs::LaserScan &msg ) {
    std::ofstream ofs(filename.c_str(), std::ios::out|std::ios::binary);

    uint32_t serial_size = ros::serialization::serializationLength(msg);
    boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);

    ros::serialization::OStream ostream(obuffer.get(), serial_size);
    ros::serialization::serialize(ostream, msg);
    ofs.write((char*) obuffer.get(), serial_size);
    ofs.close();
}
