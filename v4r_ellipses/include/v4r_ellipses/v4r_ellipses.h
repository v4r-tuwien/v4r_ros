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


#include <iostream>

#ifndef V4R_ELLIPSES_H
#define V4R_ELLIPSES_H

#include "opencv/cxcore.h"
#include "boost/shared_ptr.hpp"


class EllipsesDetection {
public:
    enum DetectionState {
        VALID = 0,
        NA = 1,
        INVALID_CONTOUR_POINTS,
        INVALID_CONTOUR_CONVEX,
        INVALID_ROTATED_RECT_RATIO,
        INVALID_ELLIPSE
    };
    struct Parameters {
        Parameters();
        bool debug;
        int threshold_edge_detection;
        int threshold_contour_min_points;
        int threshold_polygon;
        bool filter_convex;
	float threshold_rotated_rect_ratio;
        bool filter_rings;
        float threshold_ring_center;
        float threshold_ellipse_mean;
        int filter_ellipse_mean_sample_steps;
        float threshold_ring_ratio;
    };
    class Ellipse {
    public:
        Ellipse(): id(-1), boxContour(), boxEllipse(), detection(VALID) {
        };
        void init() {
            contour = boost::shared_ptr<std::vector<cv::Point2f> > (new std::vector<cv::Point2f>);
            polygon = boost::shared_ptr<std::vector<cv::Point> > (new std::vector<cv::Point>);
            distances = boost::shared_ptr<std::vector<float> > (new std::vector<float>);
        }
        int id;
        DetectionState detection;
        cv::RotatedRect boxEllipse;
        cv::Rect boxContour;
        cv::Point2f centerContour;
        float radiusContour;
        float boxEllipseRatio;
        boost::shared_ptr<std::vector<cv::Point2f> > contour;
        boost::shared_ptr<std::vector<cv::Point> > polygon;
        boost::shared_ptr<std::vector<float> > distances;
        float distance_mean;
        float distance_stdev;
    };
    EllipsesDetection (Parameters *parm);
    ~EllipsesDetection();
protected:
    Parameters *param_;
    void fit_ellipses_opencv (cv::Mat &m);
    void draw_ellipses(cv::Mat &m);
    void next();
    void createEllipseCanditates ( ) ;
    DetectionState filterContour (Ellipse &ellipse);
    DetectionState filterEllipse (Ellipse &ellipse);
    bool filterCheckIsRing(const Ellipse &ellipse);
    void computeDistancesContourToEllipse(Ellipse &ellipse);
    std::vector< std::vector<cv::Point> > contours_;
    std::vector<Ellipse> ellipses_;
    std::vector<cv::Mat> serach_windows_;
};

#endif // V4R_ELLIPSES_H
