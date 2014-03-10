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
#include <list>

#ifndef V4R_ELLIPSES_H
#define V4R_ELLIPSES_H

#include "opencv/cxcore.h"
#include "v4r_utils/contour.h"
#include "v4r_utils/camera.h"
#include "boost/shared_ptr.hpp"
#include <boost/date_time/posix_time/posix_time.hpp>

namespace V4R {

class EllipsesDetection {
public:
    enum DetectionState {
        VALID = 0,
        NA = 1,
        INVALID_CONTOUR_POINTS,
        INVALID_CONTOUR_CONVEX,
        INVALID_ROTATED_RECT_RATIO,
        INVALID_CONTOUR_MEAN,
        INVALID_NO_RING,
        INVALID_IS_INNER_RING,
        INVALID_ELLIPSE
    };
    enum PoseEstimation {
        POSE_ESTIMATION_OFF = 0,
        POSE_ESTIMATION_SOLVEPNP = 1,
        POSE_ESTIMATION_FROM_ELLIPSE = 2,
    };
    enum EdgeDetection {
        EDGE_DETECTION_THRESHOLD = 0,
        EDGE_DETECTION_CANNY = 1
    };
    enum EdgeLinking {
        EDGE_LINKING_OPENCV_APPROX_NONE = 0,
        EDGE_LINKING_OPENCV_APPROX_SIMPLE = 1,
        EDGE_LINKING_V4R_SIMPLE = 2,
        EDGE_LINKING_V4R_COMPLEX = 3,
        EDGE_LINKING_V4R_CONTOUR = 4
    };
    struct Parameters {
        Parameters();
        bool debug;
        bool distorted_input;
        EdgeDetection edge_detection;
        int threshold_edge_detection1;
        int threshold_edge_detection2;
        int kernel_size_edge_detection;
        EdgeLinking edge_linking;
        int threshold_contour_min_points;
        int threshold_polygon;
        bool filter_convex;
        double threshold_rotated_rect_ratio;
        bool filter_contour_mean;
        double threshold_contour_mean;
        bool filter_rings;
        double threshold_ring_center;
        double threshold_ring_ratio;
        PoseEstimation pose_estimation;
        double circle_diameter;
    };
    class ObliqueCone {
    public:
        double fx,fy,cx,cy;
        double radius;
        cv::Mat_<double> C;
        cv::Mat_<double> Q;
        cv::Mat_<double> E;
        cv::Mat_<double> V;
        cv::Mat_<double> N;
        cv::Mat_<double> T;
        cv::Mat_<double> Pi; 
        std::vector<cv::Point3d> translations;
        std::vector<cv::Vec3d> normals;
        void pose(cv::Mat_<double> intrinsic, cv::Mat_<double> distCoeffs, cv::Mat &nvec, cv::Mat &tvec);
        void set(cv::RotatedRect box, cv::Mat_<double> intrinsic, double radius);
        double distance(const cv::Point2d p);
    };
    class Ellipse {
    public:
        Ellipse(): id(-1), outerRing(-1), innerRing(-1), boxContour(), boxEllipse(), detection(VALID) {
        };
        void init() {
            contourUndistort = boost::shared_ptr<std::vector<cv::Point2f> > (new std::vector<cv::Point2f>);
            contourDistort = boost::shared_ptr<std::vector<cv::Point2f> > (new std::vector<cv::Point2f>);
            polygon = boost::shared_ptr<std::vector<cv::Point> > (new std::vector<cv::Point>);
            distances = boost::shared_ptr<std::vector<double> > (new std::vector<double>);
        }
        int id;
        int outerRing;
        int innerRing;
        DetectionState detection;
        cv::RotatedRect boxEllipse;
        cv::Rect boxContour;
        cv::Point2d centerContour;
        double radiusContour;
        double boxEllipseRatio;
        double radiusEllipseMax;
        double radiusEllipseMin;
        ObliqueCone cone;
        boost::shared_ptr<std::vector<cv::Point2f> > contourUndistort;
        boost::shared_ptr<std::vector<cv::Point2f> > contourDistort;
        boost::shared_ptr<std::vector<cv::Point> > polygon;
        boost::shared_ptr<std::vector<double> > distances;
    };
    class Pose {
    public:
        Pose(){};
        Pose(const Pose &p)
	: id(p.id), 
	t0(p.t0), 
	projection(p.projection), 
	R(p.R.clone()), 
	rvec(p.rvec.clone()), 
	tvec(p.tvec.clone()), 
	nvec(p.nvec.clone()) {};
        unsigned long id;
        boost::posix_time::ptime t0;
        cv::Point2f projection;
        cv::Mat_<double> R;
        cv::Mat_<double> rvec;
        cv::Mat_<double> tvec;
        cv::Mat_<double> nvec;
	void roation2Normal();
	void normal2Roation();
    };
    class Marker : public Pose{
    public:
        Marker(){};
	Marker(const Pose &m)
	: Pose(m), A(cv::Mat_<double>::eye(4,4)) {
	}
	Marker(const Marker &m)
	: Pose(m), A(m.A.clone()) {
	}
        cv::Mat_<double> A;
	void update(const boost::posix_time::ptime &t);
    };
    EllipsesDetection (Parameters *parm);
    ~EllipsesDetection();
protected:
    Parameters *param_;
    void fit_ellipses_opencv (const cv::Mat &m, const cv::Mat cameraMatrix, const cv::Mat distCoeffs, const cv::Mat projectionMatrix);
    void edge_detection(const cv::Mat &m);
    void contour_detection();
    void draw_ellipses(cv::Mat &m);
    void next();
    void createEllipseCanditates () ;
    DetectionState filterContour (Ellipse &ellipse);
    DetectionState filterEllipse (Ellipse &ellipse);
    DetectionState filterContourMean(Ellipse &ellipse);
    void estimatePoses();
    void createRings();
    void filterShapes();
    std::vector< std::vector<cv::Point> > contours_;
    std::vector<Ellipse> ellipses_;
    cv::Mat imgGray_;
    cv::Mat imgBlured_;
    cv::Mat imgEdges_;
    cv::Mat imgGradient_;
    cv::Mat imgDirection_;
    cv::Mat imgSobelDx_;
    cv::Mat imgSobelDy_;
    cv::Mat_<cv::Point2f>  lookupUndistor_;
    std::list<Pose> perception_;
    std::list<Marker> markers_;
    unsigned long loop_count;    
    V4R::Contour contour_detector_;
    V4R::Camera camera_;
};
};
#endif // V4R_ELLIPSES_H
