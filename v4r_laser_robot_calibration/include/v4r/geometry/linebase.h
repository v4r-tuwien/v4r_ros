/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) <year>  <name of author>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef V4R_LINEBASE_H
#define V4R_LINEBASE_H

#include <opencv/cv.h>
#include <cstdio>

namespace V4R {

#ifndef V4R_LINE2D
#define V4R_LINE2D
template <typename T>
class Line2D {
    friend class Line2DHdl;
protected:
    cv::Vec<T,3> eq_;
public:
    Line2D() {};
    Line2D(const Line2D &l): eq_(l.eq_) {};
    Line2D(cv::Vec<T,3> &r, bool normalize = true)
            : eq_(r) {
        if (normalize) this->normalize();
    };
    Line2D(cv::Mat &r, bool normalize = true)
            : eq_(*r.ptr<T>(0), *r.ptr<T>(1), *r.ptr<T>(2))  {
        if (normalize) this->normalize();

    };
    template <typename T2>
    Line2D(const T2 &x1, const T2 &y1, const T2 &x2, const T2 &y2, bool normalize = true) {
        set(cv::Point_<T2>(x1,y1), cv::Point_<T2>(x2,y2), normalize);
    }
    template <typename T2>
    Line2D(const cv::Point_<T2> &pt1, const cv::Point_<T2> &pt2, bool normalize = true) {
        set(pt1, pt2, normalize);
    }
    template <typename T2>
    Line2D(const cv::Point3_<T2> &pt1, const cv::Point3_<T2> &pt2, bool normalize = true) {
        set(pt1, pt2, normalize);
    }
    template <typename T2, int cn>
    Line2D(const cv::Vec<T2, cn> &v1, const cv::Vec<T2, cn> &v2, bool normalize = true) {
        set(v1, v2, normalize);
    }
    T &A() {
        return eq_.val[0];
    }
    const T &A() const {
        return eq_.val[0];
    }
    T &B() {
        return eq_.val[1];
    }
    const T &B() const {
        return eq_.val[1];
    }
    T &C() {
        return eq_.val[2];
    }
    const T &C() const {
        return eq_.val[2];
    }
    void normalize() {
        double r = sqrt(eq_[0]*eq_[0] + eq_[1]*eq_[1]);
        eq_[0] /= r, eq_[1] /= r, eq_[2] /= r;
    }
    /** @pre normalize */
    template <typename T2>
    T distanceToLine(const cv::Point_<T2> &p) {
        return eq_[0]*((T)p.x) + eq_[1]*((T)p.y) + eq_[2];
    }
    /** @pre normalize */
    template <typename T2>
    std::vector<T> distanceToLine(const std::vector <cv::Point_<T2> > &points) {
        std::vector<T> d(points.size());
        for (int i = 0; i < d.size(); i++)  d[i] = distanceToLine(points[i]);
        return d;
    }
    /** @pre normalize */
    template <typename T2>
    cv::Point_<T> nearestPointOnLine(const cv::Point_<T2> &p) {
        T d = distanceToLine(p);
        return cv::Point_<T>(p.x - d * A(), p.y - d * B());
    }
    cv::Point_<T> intersection(const Line2D<T> &l) const{
        cv::Vec<T,3> h = eq_.cross(l.eq_);
        return cv::Point_<T>(h[0]/h[2],h[1]/h[2]);
    }
    cv::Vec<T,3> &eq() {
        return eq_;
    }
    cv::Vec<T,2> normal() {
        return cv::Vec<T,2>(eq_[0], eq_[1]);
    }
    const cv::Vec<T,3> &vec() const {
        return *this;
    }
    template <typename T2>
    void set(const cv::Point_<T2> &pt1, const cv::Point_<T2> &pt2, bool normalize = true) {
        cv::Vec<T, 3> p1(pt1.x, pt1.y, 1);
        cv::Vec<T, 3> p2(pt2.x, pt2.y, 1);
        eq_ = p1.cross(p2);
        if (normalize) this->normalize();
    }
    template <typename T2>
    void set(const cv::Point3_<T2> &pt1, const cv::Point3_<T2> &pt2, bool normalize = true) {
        cv::Vec<T, 3> p1(pt1.x, pt1.y, 1);
        cv::Vec<T, 3> p2(pt2.x, pt2.y, 1);
        eq_ = p1.cross(p2);
        if (normalize) this->normalize();
    }
    template <typename T2, int cn>
    void set(const cv::Vec<T2, cn> &v1, const cv::Vec<T2, cn> &v2, bool normalize = true) {
        cv::Vec<T, 3> p1(v1[0], v1[1], 1);
        cv::Vec<T, 3> p2(v2[0], v2[1], 1);
        eq_ = p1.cross(p2);
        if (normalize) this->normalize();
    }
    /**
     * produces a human readable string
     * @return string l = [ %-8.3f, %-8.3f, %-8.3f]"
     **/
    std::string human_readable() const
    {
        char pText[0xFF];
        sprintf ( pText, "l = [ %-8.3f, %-8.3f, %-8.3f]", (double) A(), (double) B(), (double) C());
        return pText;
    }
};
typedef Line2D<float> Line2Df;
typedef Line2D<double> Line2Dd;
#endif //V4R_LINE2D

#ifndef V4R_LINESEGMENT2D
#define V4R_LINESEGMENT2D
template <typename T>
class LineSegment2D {
    friend class Line2DHdl;
protected:
    cv::Point_<T> p1_, p2_;
public:
    LineSegment2D() {};
    LineSegment2D(const LineSegment2D &l) : p1_(l.p1_), p2_(l.p2_) {};
    LineSegment2D(cv::Vec<T,4> &v) :p1_(v[0], v[1]), p2_(v[2], v[3]) {};
    template <typename T2>
    LineSegment2D(const cv::Point_<T2> &pt1, const cv::Point_<T2> &pt2):p1_(pt1), p2_(pt2) {};
    template <typename T2, typename T3>
    LineSegment2D(const cv::Vec<T2,3> &eq, const cv::Rect_<T3> &rect) {
        p1_.x = rect.x;
        p2_.x = rect.x + rect.width-1;
        p1_.y = -(eq[0] * p1_.x() + eq[2]) / eq[1];
        p2_.y = -(eq[0] * p2_.x() + eq[2]) / eq[1];
    };
    template <typename T2, typename T3>
    LineSegment2D(const cv::Vec<T2,3> &eq, const cv::Size_<T3> &size) {
        p1_.x = 0;
        p2_.x = size.width-1;
        p1_.y = -(eq[2]) / eq[1];
        p2_.y = -(eq[0] * p2_.x + eq[2]) / eq[1];
    };
    template <typename T2, typename T3>
    LineSegment2D(const Line2D<T2> &line, const cv::Size_<T3> &size) {
        p1_.x = 0;
        p2_.x = size.width-1;
        p1_.y = -(line.C()) / line.B();
        p2_.y = -(line.A() * p2_.x + line.C()) / line.B();
    };
    T &x1() {
        return p1_.x;
    }
    const T &x1() const {
        return  p1_.x;
    }
    T &y1() {
        return  p1_.y;
    }
    const T &y1() const {
        return  p1_.y;
    }
    T &x2() {
        return p2_.x;
    }
    const T &x2() const {
        return p2_.x;
    }
    T &y2() {
        return p2_.y;;
    }
    const T &y2() const {
        return p2_.y;
    }
    cv::Point_<T> &p1() {
        return p1_;
    }
    const cv::Point_<T> &p1() const {
        return p1_;
    }
    cv::Point_<T> &p2() {
        return p2_;
    }
    const cv::Point_<T> &p2() const {
        return p2_;
    }
};
typedef LineSegment2D<float> LineSegment2Df;
typedef LineSegment2D<double> LineSegment2Dd;
#endif //V4R_LINESEGMENT2D

};
#endif // V4R_LINE_H
