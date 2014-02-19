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


#include <opencv/cv.h>
#include <numeric>      // std::inner_product
#include <v4r_ellipses/v4r_ellipses.h>
#include <v4r_ellipses/v4r_ellipses_defaults.h>

EllipsesDetection::~EllipsesDetection()
{
    if(param_ != NULL) delete param_;
}

EllipsesDetection::EllipsesDetection(Parameters *param)
    :param_(param) {
}

void EllipsesDetection::createEllipseCanditates ( ) {
  ellipses_.clear();
  ellipses_.resize(contours_.size());
  for(unsigned int i = 0; i < contours_.size(); i++){
      std::vector<cv::Point> &contour = contours_[i];
      unsigned int sizeContour = contour.size();
      Ellipse &ellipse = ellipses_[i];
      ellipse.init();
      ellipse.id = i;
      cv::approxPolyDP( cv::Mat(contour), *ellipse.polygon, param_->threshold_polygon, true );
      ellipse.boxContour = cv::boundingRect( cv::Mat(*ellipse.polygon) );
      cv::minEnclosingCircle( (cv::Mat)*ellipse.polygon, ellipse.centerContour, ellipse.radiusContour );
      ellipse.contour->resize(sizeContour);
      for(unsigned int j = 0; j < sizeContour; j++) ellipse.contour->at(j) = contour[j];
  }  
}


EllipsesDetection::DetectionState EllipsesDetection::filterContour (Ellipse &ellipse) {
      if(ellipse.contour->size() < param_->threshold_contour_min_points){
	ellipse.detection = INVALID_CONTOUR_POINTS;
	return ellipse.detection;
      }
      if(param_->filter_convex && !cv::isContourConvex(*ellipse.polygon)){
	ellipse.detection = INVALID_CONTOUR_CONVEX;
	return ellipse.detection;
      }
      return ellipse.detection;
}

void EllipsesDetection::fit_ellipses_opencv ( cv::Mat &m ) {
    serach_windows_.push_back(m >= param_->threshold_edge_detection);
    cv::Mat &bimage = serach_windows_.back();
    contours_.clear();
    cv::findContours ( bimage, contours_, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
    createEllipseCanditates();
    for(unsigned int i = 0; i < ellipses_.size(); i++){
        Ellipse &ellipse = ellipses_[i];
        if(filterContour (ellipse) != VALID) continue;
        cv::Mat pointsf;
        cv::Mat ( contours_[ellipse.id] ).convertTo ( pointsf, CV_32F );
        ellipse.boxEllipse = fitEllipse ( pointsf );
        if(filterEllipse (ellipse) != VALID) continue;
    }
}

EllipsesDetection::DetectionState EllipsesDetection::filterEllipse(Ellipse &ellipse) {
        float boxEllipseRatio = (float) ellipse.boxEllipse.size.width / (float) ellipse.boxEllipse.size.height;
	if(boxEllipseRatio > 1)  {
	  boxEllipseRatio = (float) ellipse.boxEllipse.size.height / (float) ellipse.boxEllipse.size.width;
	}
        if ( boxEllipseRatio < param_->threshold_ring_ratio ){
            ellipse.detection = INVALID_ROTATED_RECT_RATIO;
	    return ellipse.detection;
	}
	return ellipse.detection;

  /*
    for(unsigned int i = 0; i < ellipses_.size(); i++) {
        Ellipse &ellipse = ellipses_[i];
        if(param_->filter_ellipse_mean_sample_steps > 0){
	  computeDistancesContourToEllipse(ellipse);
	  ellipse.detection = fabs(ellipse.distance_mean - 1.) < param_->threshold_ellipse_mean;
	}
        if(param_->filter_rings) {
            ellipse.detection = filterCheckIsRing(ellipse);
        }
    }
    */
}

bool EllipsesDetection::filterCheckIsRing(const Ellipse &ellipse) {
    const Ellipse &a = ellipse;
    bool filterRatio = (param_->threshold_ring_ratio > 0);
    for(unsigned int i = 0; i < ellipses_.size(); i++) {
      /*
        const Ellipse &b = ellipses_[i];
        float threshold_center = a.boxMax * param_->threshold_ring_center;
        if((a.id != b.id) && (a.boxMax > b.boxMax)) {
            float d = cv::norm(a.boxEllipse.center - b.boxEllipse.center);
            if(d < threshold_center) {
                if(!filterRatio) return true;
                float tmp = ((a.boxRatio > b.boxRatio) ? (a.boxRatio / b.boxRatio):(b.boxRatio / a.boxRatio)) ;
                float ratioDiff = fabs(tmp-1);
                if((ratioDiff < param_->threshold_ring_ratio)) {
                    float tmp = ((a.boxMax < b.boxMax) ? (a.boxMax / b.boxMax):(b.boxMax / a.boxMax));
                    float radiusDiff = fabs(tmp-0.5);
                    if((radiusDiff < param_->threshold_ring_ratio)) {
                        return true;
                    }
                }
            }
        }
        */
    }
    return false;
}

void EllipsesDetection::next() {
    contours_.clear();
    ellipses_.clear();
    serach_windows_.clear();
}

void EllipsesDetection::computeDistancesContourToEllipse(Ellipse &ellipse) {
    const std::vector<cv::Point2f> &contour = *(ellipse.contour.get());
    ellipse.distances = boost::shared_ptr<std::vector<float> >(new std::vector<float> );
    std::vector<float > &distances = *(ellipse.distances.get());
    const cv::Point2f &pc = ellipse.boxEllipse.center;
    float angle = M_PI/180.0 * (float) ellipse.boxEllipse.angle;
    float ca = cos(angle), sa = sin(angle);
    float a = ellipse.boxEllipse.size.width/2., b = ellipse.boxEllipse.size.height/2.;
    float sum = 0;
    distances.reserve(contour.size());
    for(unsigned int i = 0; i < contour.size(); i = i + param_->filter_ellipse_mean_sample_steps){
      float dx = contour[i].x - pc.x, dy = contour[i].y - pc.y;
      float u = ca*dx + sa*dy, v = -sa*dx + ca*dy;
      cv::Point2f p(contour[i].x - pc.x, contour[i].y - pc.y);
      float d = (u*u)/(a*a) + (v*v)/(b*b);
      distances.push_back(d);
      sum += d; 
      // http://stackoverflow.com/questions/11041547/finding-the-distance-of-a-point-to-an-ellipse-wether-its-inside-or-outside-of-e
    }
    ellipse.distance_mean = sum / distances.size();
    float sq_sum = std::inner_product(distances.begin(), distances.end(), distances.begin(), 0.0);
    ellipse.distance_stdev = std::sqrt(sq_sum / distances.size() - ellipse.distance_mean * ellipse.distance_mean);
}
void EllipsesDetection::draw_ellipses(cv::Mat &img) {
    cv::Point2f vtx[4];
    for(unsigned int i = 0; i < ellipses_.size(); i++) {
        Ellipse &ellipse = ellipses_[i];
        if(ellipse.detection != VALID) continue;
        cv::drawContours(img, contours_, (int) i, cv::Scalar::all(255), 1, 8);
        cv::ellipse(img, ellipse.boxEllipse, cv::Scalar(0,0,255), 1, CV_AA);
        ellipse.boxEllipse.points(vtx);
        for( int j = 0; j < 4; j++ ) cv::line(img, vtx[j], vtx[(j+1)%4], cv::Scalar(0,255,0), 1, CV_AA);
    }
}
