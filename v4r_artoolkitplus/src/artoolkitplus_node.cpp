/***************************************************************************
 *   Copyright (C) 2013 by Markus Bader                                    *
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

#include <v4r_artoolkitplus/artoolkitplus_node.h>
#include <v4r_artoolkitplus/artoolkitplus_defaults.h>

#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "ARToolKitPlus/TrackerSingleMarkerImpl.h"
#include "ARToolKitPlus/TrackerMultiMarkerImpl.h"
#include <ARToolKitPlus/CameraAdvImpl.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "arMarker");
    ros::NodeHandle n;
    ARToolKitPlusNode ar(n);
    ros::spin();
    return 0;
}


int ARToolKitPlusNode::matrix2Tf(const ARFloat M[3][4], tf::Transform &transform) {
    tf::Matrix3x3 R(M[0][0], M[0][1], M[0][2], M[1][0], M[1][1], M[1][2], M[2][0], M[2][1], M[2][2]);
    tf::Vector3 T(M[0][3], M[1][3], M[2][3]);
    transform = tf::Transform(R, T);
}
;

ARToolKitPlusNode::ARToolKitPlusNode(ros::NodeHandle & n) :
        n_(n), n_param_("~"), callback_counter_(0), imageTransport_(n_), trackerSingleMarker_(NULL), trackerMultiMarker_(NULL), logger_(NULL) {

    sprintf(namespace_, "%s", n_param_.getNamespace().c_str());
    debugWndName_ = std::string("Debug: ") + n_param_.getNamespace();
    readParam();
    init();
    cameraSubscriber_ = imageTransport_.subscribeCamera( IMAGE_SRC, 1, &ARToolKitPlusNode::imageCallback, this);
}
class MyLogger: public ARToolKitPlus::Logger {
    void artLog(const char* nStr) {
        printf("%s", nStr);
    }
};


class ARCamera: public ARToolKitPlus::CameraAdvImpl {
public:
    virtual ~ARCamera() {
    }
    ;
    ARCamera(const sensor_msgs::CameraInfoConstPtr& _camer_info, int _undist_iterations, bool _input_distorted) {

        // ARToolKitPlus::CameraAdvImpl Parameter
        if (_input_distorted) {
            // using the ros Intrinsic camera matrix for the raw (distorted) images.
            this->fc[0] = (ARFloat) _camer_info->K[0];
            this->fc[1] = (ARFloat) _camer_info->K[4];
            this->cc[0] = (ARFloat) _camer_info->K[2];
            this->cc[1] = (ARFloat) _camer_info->K[5];

            undist_iterations = _undist_iterations;

            this->kc[0] = (ARFloat) _camer_info->D[0];
            this->kc[1] = (ARFloat) _camer_info->D[1];
            this->kc[2] = (ARFloat) _camer_info->D[2];
            this->kc[3] = (ARFloat) _camer_info->D[3];
            this->kc[4] = (ARFloat) _camer_info->D[4];
            this->kc[5] = (ARFloat) 0.;

        } else {
            // using the ros Projection/camera matrix
            this->fc[0] = (ARFloat) _camer_info->P[0];
            this->fc[1] = (ARFloat) _camer_info->P[5];
            this->cc[0] = (ARFloat) _camer_info->P[2];
            this->cc[1] = (ARFloat) _camer_info->P[6];

            undist_iterations = 1;

            for (int i = 0; i < 6; i++)
                this->kc[i] = (ARFloat) 0.;

        }

        // ARToolKitPlus::Camera Parameter
        // fileName

        // ARToolKit::ARParam Parameter
        xsize = _camer_info->width;
        ysize = _camer_info->height;

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 4; j++)
                this->mat[i][j] = (ARFloat) 0.;

        mat[0][0] = fc[0]; // fc_x
        mat[1][1] = fc[1]; // fc_y
        mat[0][2] = cc[0]; // cc_x
        mat[1][2] = cc[1]; // cc_y
        mat[2][2] = 1.0;

        if (_input_distorted == false) {
            // using the ros Projection/camera matrix
            mat[0][3] = (ARFloat) _camer_info->P[3];
            mat[1][3] = (ARFloat) _camer_info->P[7];
        }

        for (int i = 0; i < 4; i++)
            this->dist_factor[i] = this->kc[i];
    }
};



ARToolKitPlusNode::~ARToolKitPlusNode() {
    if (trackerSingleMarker_ != NULL)
        delete trackerSingleMarker_;
    if (logger_ != NULL)
        delete logger_;
}



void ARToolKitPlusNode::initTrackerSingleMarker(const sensor_msgs::CameraInfoConstPtr& camer_info) {
    if(trackerMultiMarker_) delete trackerSingleMarker_;
    trackerSingleMarker_ = new ARToolKitPlus::TrackerSingleMarkerImpl<AR_TRACKER_PARAM>(camer_info->width, camer_info->height);
    const char* description = trackerSingleMarker_->getDescription();
    ROS_INFO("%s: compile-time information:\n%s", namespace_, description);

// set a logger so we can output error messages
    trackerSingleMarker_->setLogger(logger_);
    trackerSingleMarker_->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

    ARCamera *camera = new ARCamera(camer_info, param_.undist_iterations, param_.input_distorted);
     if (!trackerSingleMarker_->init(camera, 1.0f, 1000.0f)) {
        ROS_ERROR("ERROR: init() failed");
    }

// define size of the marker
    trackerSingleMarker_->setPatternWidth(param_.patternWidth);

// the marker in the BCH test image has a thin border...
    if (param_.borderWidth > 0) {
        trackerSingleMarker_->setBorderWidth(param_.borderWidth);
    } else {
        trackerSingleMarker_->setBorderWidth(param_.useBCH ? 0.125f : 0.250f);
    }

// set a threshold. alternatively we could also activate automatic thresholding
    if (param_.threshold > 0) {
        trackerSingleMarker_->activateAutoThreshold(false);
        trackerSingleMarker_->setThreshold(param_.threshold);
    } else {
        trackerSingleMarker_->activateAutoThreshold(true);
    }
// let's use lookup-table undistortion for high-speed
// note: LUT only works with images up to 1024x1024
    trackerSingleMarker_->setUndistortionMode((ARToolKitPlus::UNDIST_MODE) param_.undist_mode);

// RPP is more robust than ARToolKit's standard pose estimator
    trackerSingleMarker_->setPoseEstimator((ARToolKitPlus::POSE_ESTIMATOR) param_.pose_estimation_mode);

// switch to simple ID based markers
// use the tool in tools/IdPatGen to generate markers
    trackerSingleMarker_->setMarkerMode(param_.useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);

// do the OpenGL camera setup
//glMatrixMode(GL_PROJECTION)
//glLoadMatrixf(tracker->getProjectionMatrix());
}

void ARToolKitPlusNode::initTrackerMultiMarker(const sensor_msgs::CameraInfoConstPtr& camer_info) {
    if(trackerMultiMarker_) delete trackerMultiMarker_;
    trackerMultiMarker_ = new ARToolKitPlus::TrackerMultiMarkerImpl<AR_TRACKER_PARAM>(camer_info->width, camer_info->height);
    const char* description = trackerMultiMarker_->getDescription();
    ROS_INFO("%s: compile-time information:\n%s", namespace_, description);

// set a logger so we can output error messages
    if(logger_ == NULL) logger_ = new MyLogger();
    trackerMultiMarker_->setLogger(logger_);
    trackerMultiMarker_->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

    ARCamera *camera = new ARCamera(camer_info, param_.undist_iterations, param_.input_distorted);
    if (!trackerMultiMarker_->init(camera, "/home/max/projects/catkin_ws/src/repo/v4r/v4r_artoolkitplus/cfg/markerboard_0000-0011.cfg", 1.0f, 1000.0f)) {
        ROS_ERROR("ERROR: init() failed");
    }

// the marker in the BCH test image has a thin border...
    if (param_.borderWidth > 0) {
        trackerMultiMarker_->setBorderWidth(param_.borderWidth);
    } else {
        trackerMultiMarker_->setBorderWidth(param_.useBCH ? 0.125f : 0.250f);
    }

// set a threshold. alternatively we could also activate automatic thresholding
    if (param_.threshold > 0) {
        trackerMultiMarker_->activateAutoThreshold(false);
        trackerMultiMarker_->setThreshold(param_.threshold);
    } else {
        trackerMultiMarker_->activateAutoThreshold(true);
    }
// let's use lookup-table undistortion for high-speed
// note: LUT only works with images up to 1024x1024
    trackerMultiMarker_->setUndistortionMode((ARToolKitPlus::UNDIST_MODE) param_.undist_mode);

// RPP is more robust than ARToolKit's standard pose estimator
    trackerMultiMarker_->setPoseEstimator((ARToolKitPlus::POSE_ESTIMATOR) param_.pose_estimation_mode);

// switch to simple ID based markers
// use the tool in tools/IdPatGen to generate markers
    trackerMultiMarker_->setMarkerMode(param_.useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);

// do the OpenGL camera setup
//glMatrixMode(GL_PROJECTION)
//glLoadMatrixf(tracker->getProjectionMatrix());


}


void ARToolKitPlusNode::imageCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& camer_info_) {
    callback_counter_++;
    if((callback_counter_ % (param_.skip_frames+1) ) != 0) {
        return;
    }
    cv_bridge::CvImagePtr img;
    try {
        img = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if (param_.tracker_single_marker) {
        if(trackerSingleMarker_ == NULL) initTrackerSingleMarker(camer_info_);
        ARToolKitPlus::ARMarkerInfo* arMarkerInfo;
        int nNumMarkers;
        int markerId = trackerSingleMarker_->calc(img->image.data, param_.nPattern, param_.nUpdateMatrix, &arMarkerInfo, &nNumMarkers);
        float conf = (float) trackerSingleMarker_->getConfidence();
        arSingleMarkerInfo_.resize(nNumMarkers);
        for(int i = 0; i < nNumMarkers; i++) arSingleMarkerInfo_[i] = arMarkerInfo[i];
    }
    if (param_.tracker_multi_marker) {
        if(trackerMultiMarker_ == NULL) initTrackerMultiMarker(camer_info_);
        ARToolKitPlus::ARMarkerInfo* arMarkerInfo;
        int nNumMarkers = trackerMultiMarker_->calc(img->image.data);
        arMultiMarkerInfo_.resize(nNumMarkers);
        for(int i = 0; i < nNumMarkers; i++) arMultiMarkerInfo_[i] = trackerMultiMarker_->getDetectedMarker(i);
    }
    publishTf(image_msg->header);

    if (show_camera_image_) {
        cv::Mat img_debug;
        cvtColor(img->image, img_debug, CV_GRAY2BGR);
        generateDebugImage(img_debug);
        cv::imshow( DEBUG_WINDOWS_NAME, img_debug);
        cv::waitKey(5);
    }
}

void ARToolKitPlusNode::publishTf(const std_msgs::Header &header) {

    ARFloat center[2];
    center[0] = 0;
    center[1] = 0;
    ARFloat pose[3][4];
    tf::Transform trans;
    tf::StampedTransform st;
    char frame[0xFF];

    for (int j = 0; j < arSingleMarkerInfo_.size(); j++) {
        ARToolKitPlus::ARMarkerInfo &artag = arSingleMarkerInfo_[j];
        if (artag.id < 0)
            continue;
        sprintf(frame, "t%i", artag.id);
        trackerSingleMarker_->executeSingleMarkerPoseEstimator(&artag, center, param_.patternWidth, pose);

        matrix2Tf(pose, trans);
        std::string child_frame = tf::resolve(param_.tf_prefix, frame);
        st = tf::StampedTransform(trans, header.stamp, header.frame_id, child_frame);
        transformBroadcaster_.sendTransform(st);
    }
    for (int j = 0; j < arMultiMarkerInfo_.size(); j++) {
        const ARFloat *p = trackerMultiMarker_->getModelViewMatrix();
        for(int r = 0; r < 3; r++){
            pose[r][0] = p[r+0];
            pose[r][1] = p[r+4];
            pose[r][2] = p[r+8];
            pose[r][3] = p[r+12];
        }
        matrix2Tf(pose, trans);
        std::string child_frame = tf::resolve(param_.tf_prefix, param_.pattern_frame);
        st = tf::StampedTransform(trans, header.stamp, header.frame_id, child_frame);
        transformBroadcaster_.sendTransform(st);
    }
}

void ARToolKitPlusNode::generateDebugImage(cv::Mat &img) {
    char text[0xFF];
    cv::Point a, b, c;
    cv::Scalar green(0, 255, 0);
    cv::Scalar red(0, 0, 255);
    cv::Scalar blue(255, 0, 0);
    int fondFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    for (int i = 0; i < arSingleMarkerInfo_.size(); i++) {
        ARToolKitPlus::ARMarkerInfo &artag = arSingleMarkerInfo_[i];
        cv::Scalar lineColour = green;
        if (artag.id < 0) lineColour = red;
        a = cv::Point(artag.vertex[3][0], artag.vertex[3][1]);
        for (int v = 0; v < 4; v++) {
            b = cv::Point(artag.vertex[v][0], artag.vertex[v][1]);
            cv::line(img, a, b, lineColour, 1);
            a = b;
        }
        sprintf(text, "%i", artag.id);
        cv::putText(img, text, cv::Point(artag.pos[0], artag.pos[1]), fondFace, 0.2, green);

    }
    for (int i = 0; i < arMultiMarkerInfo_.size(); i++) {
        ARToolKitPlus::ARMarkerInfo &artag = arMultiMarkerInfo_[i];
        cv::Scalar lineColour = green;
        if (artag.id < 0) lineColour = red;
        a = cv::Point(artag.vertex[0][0], artag.vertex[0][1]);
        b = cv::Point(artag.vertex[2][0], artag.vertex[2][1]);
        c = cv::Point(a.x + (b.x - a.x)/2, a.y + (b.y - a.y)/2);
        cv::line(img, a, b, lineColour, 1);
        a = cv::Point(artag.vertex[1][0], artag.vertex[1][1]);
        b = cv::Point(artag.vertex[3][0], artag.vertex[3][1]);
        cv::line(img, a, b, lineColour, 1);
        sprintf(text, "%i", artag.id);
        cv::putText(img, text, c, fondFace, 0.2, green);

    }
}

void ARToolKitPlusNode::readParam() {

    std::string tmp;

    n_param_.param<int>("skip_frames", param_.skip_frames, DEFAULT_SKIP_FRAMES);
    ROS_INFO("%s: skip_frames: %i", namespace_, param_.skip_frames);

    n_param_.param<bool>("show_camera_image", show_camera_image_,
    DEFAULT_SHOW_CAMERA_IMAGE);
    ROS_INFO("%s: show_camera_image:  %s", namespace_, ((show_camera_image_) ? "true" : "false"));

    n_param_.param<bool>("tracker_single_marker", param_.tracker_single_marker,
    DEFAULT_TRACKER_SINGLE_MARKER);
    ROS_INFO("%s: tracker_single_marker:  %s", namespace_, ((param_.tracker_single_marker) ? "true" : "false"));

    n_param_.param<bool>("tracker_multi_marker", param_.tracker_multi_marker,
    DEFAULT_TRACKER_MULTI_MARKER);
    ROS_INFO("%s: tracker_multi_marker:  %s", namespace_, ((param_.tracker_multi_marker) ? "true" : "false"));

    if(!param_.tracker_multi_marker && !param_.tracker_single_marker){
        ROS_ERROR("%s: at least tracker_multi_marker or tracker_single_marker must be true", namespace_);
    }

    n_param_.param<std::string>("pattern_frame", param_.pattern_frame, DEFAULT_PATTERN_FRAME);
    ROS_INFO("%s: pattern_frame: %s", namespace_, param_.pattern_frame.c_str());

    n_param_.param<std::string>("pattern_file", param_.pattern_file, DEFAULT_PATTERN_FILE);
    ROS_INFO("%s: pattern_file: %s", namespace_, param_.pattern_file.c_str());
    if(!param_.tracker_multi_marker && !param_.pattern_file.empty()){
        ROS_WARN("%s: tracker_multi_marker must be true in order to use mutli patterns with a pattern file", namespace_);
    }

    n_param_.param<std::string>("tf_prefix", param_.tf_prefix, n_param_.getNamespace());
    ROS_INFO("%s: tf_prefix: %s", namespace_, param_.tf_prefix.c_str());


    n_param_.param<std::string>("marker_mode", tmp, DEFAULT_MARKER_MODE);
    if ((tmp.compare(MARKER_MODE_BCH) == 0) || (tmp.compare(MARKER_MODE_SIMPEL) == 0)) {
        if (tmp.compare(MARKER_MODE_BCH) == 0)
            param_.useBCH = true;
        else
            param_.useBCH = false;
        ROS_INFO("%s: marker_mode:  %s", namespace_, tmp.c_str());
    } else {
        ROS_ERROR("%s: marker_mode:  %s does not match any known type use %s or %s", namespace_, tmp.c_str(), MARKER_MODE_SIMPEL, MARKER_MODE_BCH);
    }

    n_param_.param<double>("pattern_width", param_.patternWidth, DEFAULT_PATTERN_WITH);
    ROS_INFO("%s: pattern_width: %4.3f [m] only for single marker", namespace_, param_.patternWidth);

    n_param_.param<int>("threshold", param_.threshold, DEFAULT_THRESHOLD);
    ROS_INFO("%s: threshold: %i, (0 = auto threshold)", namespace_, param_.threshold);

    n_param_.param<double>("border_width", param_.borderWidth, DEFAULT_BOARDER_WIDTH);
    ROS_INFO("%s: border_width: %4.3f, (0 = auto threshold)", namespace_, param_.borderWidth);

    n_param_.param<int>("undist_iterations", param_.undist_iterations, DEFAULT_UNDIST_INTERATIONS);
    ROS_INFO("%s: undist_iterations: %i", namespace_, param_.undist_iterations);

    n_param_.param<bool>("input_distorted", param_.input_distorted, DEFAULT_INPUT_DISTORTED);
    ROS_INFO("%s: input_distorted:  %s", namespace_, ((param_.input_distorted) ? "true" : "false"));

    n_param_.param<std::string>("undist_mode", tmp, DEFAULT_UNDIST_MODE);
    if ((tmp.compare(UNDIST_MODE_NONE) == 0) || (tmp.compare(UNDIST_MODE_STD) == 0) || (tmp.compare(UNDIST_MODE_LUT) == 0)) {
        if (tmp.compare(UNDIST_MODE_NONE) == 0)
            param_.undist_mode = ARToolKitPlus::UNDIST_NONE;
        if (tmp.compare(UNDIST_MODE_STD) == 0)
            param_.undist_mode = ARToolKitPlus::UNDIST_STD;
        if (tmp.compare(UNDIST_MODE_LUT) == 0)
            param_.undist_mode = ARToolKitPlus::UNDIST_LUT;
        ROS_INFO("%s: undist_mode:  %s", namespace_, tmp.c_str());
    } else {
        ROS_ERROR("%s: undist_mode:  %s does not match any known type use %s, %s  or %s", namespace_, tmp.c_str(), UNDIST_MODE_NONE, UNDIST_MODE_STD,
                UNDIST_MODE_LUT);
    }

    n_param_.param<std::string>("pose_estimation_mode", tmp, DEFAULT_POSE_ESTIMATION_MODE);
    if ((tmp.compare(POSE_ESTIMATION_MODE_NORMAL) == 0) || (tmp.compare(POSE_ESTIMATION_MODE_CONT) == 0) || (tmp.compare(POSE_ESTIMATION_MODE_RPP) == 0)) {
        if (tmp.compare(POSE_ESTIMATION_MODE_NORMAL) == 0)
            param_.pose_estimation_mode = ARToolKitPlus::POSE_ESTIMATOR_ORIGINAL;
        if (tmp.compare(POSE_ESTIMATION_MODE_CONT) == 0)
            param_.pose_estimation_mode = ARToolKitPlus::POSE_ESTIMATOR_ORIGINAL_CONT;
        if (tmp.compare(POSE_ESTIMATION_MODE_RPP) == 0)
            param_.pose_estimation_mode = ARToolKitPlus::POSE_ESTIMATOR_RPP;
        ROS_INFO("%s: pose_estimation_mode:  %s", namespace_, tmp.c_str());
    } else {
        ROS_ERROR("%s: pose_estimation_mode:  %s does not match any known type use %s, %s  or %s", namespace_, tmp.c_str(), POSE_ESTIMATION_MODE_NORMAL,
                POSE_ESTIMATION_MODE_CONT, POSE_ESTIMATION_MODE_RPP);
    }

    param_.nPattern = -1;
    param_.nUpdateMatrix = true;
}

void ARToolKitPlusNode::init() {
    if (show_camera_image_) {
        cv::namedWindow( DEBUG_WINDOWS_NAME, 1);
    }
}
