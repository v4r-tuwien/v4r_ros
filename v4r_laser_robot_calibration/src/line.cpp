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

#include "line.h"

using namespace V4R;
Line2DHdl::Line2DHdl()
        : mpLine(NULL) {
};
Line2DHdl::Line2DHdl(Line2D<double> &r)
        : mpLine(NULL)
{
    set(r);
};
Line2DHdl::Line2DHdl(double *p)
        : mpLine(NULL)
{
    set(p);
};

void Line2DHdl::set(Line2D<double> &r) {
    mpLine = &r;
};

void Line2DHdl::set(double *p) {
    set(*((Line2D<double> *) p));
};

void Line2DHdl::normalize(cv::Mat &rLines) {
    if (rLines.cols != 3) {
        CV_Error( CV_StsUnsupportedFormat, "Line2DHdl::normalize columns size must be 3" ); 
	return;
    }
    if (rLines.type() == CV_64F) {
        Line2D<double> *pLine = (Line2D<double> *) rLines.data;
        for (int i = 0; i < rLines.rows; i++) {
            pLine[i].normalize();
        }
    } else {
        CV_Error( CV_StsUnsupportedFormat, "Line2DHdl::normalize type must be CV_64F" ); 
        return;
    }
}

Line2D<double> &Line2DHdl::operator ()() {
    return *mpLine;
}
Line2D<double> &Line2DHdl::operator ()(unsigned int idx) {
    return mpLine[idx];
}

cv::Vec<double,2> &Line2DHdl::normal() {
    return mNormal;
}
cv::Vec<double,2> &Line2DHdl::computeNormal() {
    mNormal[0] = -mpLine->eq_[0], mNormal[1] = -mpLine->eq_[1];
    return mNormal;
}
cv::Vec<double,2> &Line2DHdl::unit() {
    return mUnit;
}
cv::Vec<double,2> &Line2DHdl::computeUnit() {
    mUnit[0] = mpLine->eq_[1], mUnit[1] = -mpLine->eq_[0];
    return mUnit;
}
