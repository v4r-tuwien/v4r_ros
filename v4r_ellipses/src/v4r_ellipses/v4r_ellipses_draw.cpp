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

#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <v4r_ellipses/v4r_ellipses.h>
#include <v4r_ellipses/v4r_ellipses_defaults.h>
#include <boost/foreach.hpp>

using namespace V4R;
/*
void undistort(const cv::Point2f &src, cv::Point2f &des, const cv::Mat_<double> &srcCameraMatrix, const cv::Mat_<double> &distCoeffs, const cv::Mat_<double> &desCameraMatrix, double epsylon, int max_interation = 3) {
  
  double &r = epsylon;
  int i = 0;
  std::vector<cv::Point2f> neigbours(4);
  for (int  i = 0; (r >= epsylon ) && (i < max_interation); i++){
    
    neigbours
    
    [x_c1, y_c1] = distor( x_d + r, y_d    , Intrinsic, Distortion);
    [x_c2, y_c2] = distor( x_d - r, y_d    , Intrinsic, Distortion);
    [x_c3, y_c3] = distor( x_d    , y_d + r, Intrinsic, Distortion);
    [x_c4, y_c4] = distor( x_d    , y_d - r, Intrinsic, Distortion);
    
    dd1 = [x, y] - [x_c1, y_c1];
    dd2 = [x, y] - [x_c2, y_c2];
    dd3 = [x, y] - [x_c3, y_c3];
    dd4 = [x, y] - [x_c4, y_c4];
    
    r1 = norm(dd1);
    r2 = norm(dd2);
    r3 = norm(dd3);
    r4 = norm(dd4);
    
    scale = (2*r) / norm([x_c1-x_c2, y_c1-y_c2]);
    
    if     (r1 < r2) && (r1 < r3) && (r1 < r4)
        r = r1 * scale;
        x_d = x_d + r;
        y_d = y_d;
    elseif (r2 < r3) && (r2 < r4) 
        r = r2 * scale;
        x_d = x_d - r;
        y_d = y_d;
    elseif (r3 < r4) 
        r = r3 * scale;
        x_d = x_d;
        y_d = y_d + r;
    else
        r = r4 * scale;
        x_d = x_d;
        y_d = y_d - r;   
    end
    
end
  
}
*/

void EllipsesDetection::draw_ellipses(cv::Mat &img) {
    std::vector<cv::Point2f> vtx(4);
    std::vector<cv::Point2f> vtxDis;
    cv::Scalar colourContour(255,0,255);
    cv::Scalar colourEllipse(0,0,255);
    char text[0xFF];
    for(unsigned int i = 0; i < ellipses_.size(); i++) {
        Ellipse &ellipse = ellipses_[i];
        if(ellipse.detection != VALID) continue;
        //cv::drawContours(img, contours_, (int) i, colourContour, 1, 8);
        cv::RotatedRect box1, box2;
        camera_.distort(ellipse.boxEllipse, box1);
        cv::ellipse(img, box1, colourEllipse, 1, CV_AA);
        camera_.distort(ellipses_[ellipse.innerRing].boxEllipse, box2);
        if(ellipse.innerRing >= 0) cv::ellipse(img, box2, colourEllipse, 1, CV_AA);
        box1.points(&vtx[0]);
        for( int j = 0; j < 4; j++ ) {
            cv::line(img, vtx[j], vtx[(j+2)%4], cv::Scalar(0,255,0), 1, CV_AA);
            sprintf(text, "%i", j);
            //cv::putText(img, text, vtx[j], cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(0), 1, CV_AA);
        }
        camera_.distort(*ellipse.contourUndistort, vtxDis);
        BOOST_FOREACH(const cv::Point2f &p, vtxDis ){
          img.at<cv::Vec3b>(p) = cv::Vec3b(0,255,255);
        }

        if(ellipse.cone.T.empty() == false) {
            cv::Mat pi, pw(ellipse.cone.translations);
            cv::Mat pin , pn = pw + cv::Mat(ellipse.cone.normals)*param_->circle_diameter;
            cv::projectPoints(pw, cv::Mat_<double>::zeros(1,3), cv::Mat_<double>::zeros(1,3), camera_.cameraMatrix, camera_.distCoeffs, pi);
            cv::projectPoints(pn, cv::Mat_<double>::zeros(1,3), cv::Mat_<double>::zeros(1,3), camera_.cameraMatrix, camera_.distCoeffs, pin);
            for(int j = 0; j < pi.rows; j++) {
                cv::Point p0(pi.at<double>(j,0),pi.at<double>(j,1));
                cv::circle(img, p0,2, cv::Scalar(125*j,125*j,255));
                cv::Point p1(pin.at<double>(j,0),pin.at<double>(j,1));
                cv::line(img, p0, p1, cv::Scalar(125*j,125*j,255), 1, CV_AA);
            }
        }

        /// Matlab Debug output
        if(param_->debug) {
            std::cout << "% === " << std::setw(4) <<  loop_count;
            std::cout << "  ===== ellipse " << std::setw(4) << i << " ===" << std::endl;
            std::cout << "ellipse.center     = " <<  ellipse.boxEllipse.center << "; " << std::endl;
            std::cout << "ellipse.size       = " <<  (cv::Point2f) ellipse.boxEllipse.size << "; " << std::endl;
            std::cout << "ellipse.angle      = " <<   M_PI/180.0 *ellipse.boxEllipse.angle << "; " << std::endl;
            std::cout << "ellipse.C          = " << ellipse.cone.C << "; " << "  % Ellipse Image" << std::endl;
            std::cout << "ellipse.Q          = " << ellipse.cone.Q << "; " << "  % Cone Imageplane" << std::endl;
            std::cout << "ellipse.E          = " << ellipse.cone.E << "; " << " % Eigenvalues" << std::endl;
            std::cout << "ellipse.V          = " << ellipse.cone.V << "; " << "  % Eigenvectors" << std::endl;
            std::cout << "ellipse.radius     = " << param_->circle_diameter/2. << "; " << std::endl;
            std::cout << "ellipse.nr_of_edges=" <<  ellipse.contourUndistort->size() << "; " << std::endl;
            std::cout << "ellipse.contour    = " << cv::Mat(*ellipse.contourUndistort) << "; " << std::endl;
            std::cout << "ellipse.T          = " << ellipse.cone.T << "; " << " % Translations" << std::endl;
            std::cout << "ellipse.N          = " << ellipse.cone.N << "; " << "  % Normals" << std::endl;
            std::cout << "ellipse.Pi         = " << ellipse.cone.Pi << "; " << " % Projection" << std::endl;
            std::cout << "camera.intrinsic   = " << camera_.cameraMatrix << "; " << std::endl;
            std::cout << "camera.distortions = " << camera_.distCoeffs << "; " << std::endl;
            std::cout << "camera.projectionMatrix = " << camera_.projectionMatrix << "; " << std::endl;
        }
    }
    if(param_->debug) {
        contour_detector_.Draw(img.data);
    }
}