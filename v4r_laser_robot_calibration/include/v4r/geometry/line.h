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

#ifndef V4R_LINE_H
#define V4R_LINE_H

#include "v4r/geometry/linebase.h"

namespace V4R {
class Line2DHdl
{
public:
    Line2DHdl();
    Line2DHdl(Line2D<double> &r);
    Line2DHdl(double *p);
    void set(Line2D<double> &r);
    void set(double *p);
    static void normalize(cv::Mat &rLines);
    Line2D<double> &operator ()();
    Line2D<double> &operator ()(unsigned int idx);
    cv::Vec<double,2> &normal();
    cv::Vec<double,2> &computeNormal();
    cv::Vec<double,2> &unit();
    cv::Vec<double,2> &computeUnit();
private:
    cv::Ptr<Line2D<double> > mpLine;
    cv::Vec<double,2> mNormal;
    cv::Vec<double,2> mUnit;
};
};
#endif // V4R_LINE_H
