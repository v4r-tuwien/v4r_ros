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

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <sstream>

#include <dynamic_reconfigure/server.h>
#include <v4r_laser_filter/LineFilterConfig.h>

#ifndef V4R_LASER_LINE_FILTER_NODE
#define V4R_LASER_LINE_FILTER_NODE


#define MX_LASER_LINE_FILTER_PUBLISH_MARKER false
#define MX_LASER_LINE_FILTER_THRESHOLD_SPLIT 0.02
#define MX_LASER_LINE_FILTER_MIN_LENGTH 0.2
#define MX_LASER_LINE_FILTER_MIN_POINTS_PER_LINE 10
#define MX_LASER_LINE_FILTER_MIN_POINTS_PER_METER 10
#define MX_LASER_LINE_FILTER_FIT_LINES false
#define MX_LASER_LINE_FILTER_SPLIT_SCAN true

/// ROS Node
class LaserLineFilterNode {
public:
    struct Parameters {
        Parameters()
            : publish_marker(MX_LASER_LINE_FILTER_PUBLISH_MARKER)
            , threshold_split(MX_LASER_LINE_FILTER_THRESHOLD_SPLIT)
	    , threshold_split_neighbor(true)
            , min_length(MX_LASER_LINE_FILTER_MIN_LENGTH)
            , min_points_per_line(MX_LASER_LINE_FILTER_MIN_POINTS_PER_LINE)
            , min_points_per_meter(MX_LASER_LINE_FILTER_MIN_POINTS_PER_METER)
            , split_scan(MX_LASER_LINE_FILTER_SPLIT_SCAN)
            , fit_lines(MX_LASER_LINE_FILTER_FIT_LINES)
            , write_scan(false)
            , read_scan(false)
            , scan_filename("/tmp/scan.bin") {};
        bool publish_marker;
        float threshold_split;
	bool threshold_split_neighbor;
        float min_length;
        int min_points_per_line;
        float min_points_per_meter;
        bool split_scan;
        bool fit_lines;
        bool write_scan;
        bool read_scan;
        std::string scan_filename;
    };
    class Point {
    public:
        Point() {};
        Point(float _x, float _y): x(_x), y(_y) {};
        float x, y;
        float L2(const Point &p) {
            float dx = x-p.x, dy = y-p.y;
	    return sqrt(dx*dx+dy*dy);
        }
        float L1(const Point &p) {
            return fabs(x-p.x) + fabs(y-p.y);
        }
    };
    class Beam {
    public:
        Beam() {};
        void set(float _alpha, float _range) {
            alpha = _alpha, range = _range;
        }
        float alpha;
        float range;
    };
    class Measurment : public Point {
    public:
        Measurment(): Point() {};
        void set(float _alpha, float _range) {
            if(isnan(_range) || isinf(_range)) {
                valid = false;
            } else {
                x = cos(_alpha) * _range;
                y = sin(_alpha) * _range;
                valid = true;
            }
        }
        bool valid;
    };
    class LineEq {
    public:
        LineEq() {};
        LineEq(float _a, float _b, float _c): a(_a), b(_b), c(_c) {};
        void set(const Point &p0, const Point &p1);
        float distance(const Point &p);
        float a, b, c;
    };
    class Line {
    public:
        Line() {};
        Line(const Point &_p0, const Point &_p1);
        void set(const Point &_p0, const Point &_p1);
        float length();
        Point p0, p1;
        LineEq eq;
    };
    class LineSegment : public Line {
    public:
        LineSegment() :Line(), id(0) {};
        void set(unsigned int _idx0, unsigned int _idx1, const std::vector<Measurment> &measurments);
        void updatePoints(const std::vector<Measurment> &measurments);
        bool isSupportPoint(int idx);
        unsigned int nrSupportPoint();
        unsigned int id;
        unsigned int idx0, idx1;
        std::vector<Point> points;
    };
    LaserLineFilterNode ( ros::NodeHandle &n );
    void callback (const sensor_msgs::LaserScan::ConstPtr& msg);
    void callbackParameters ( v4r_laser_filter::LineFilterConfig &config, uint32_t level );
private:
    void split(LineSegment &line);
    void splitStart();
    void readScan(const std::string &filename, sensor_msgs::LaserScan &msg );
    void writeScan(const std::string &filename, const sensor_msgs::LaserScan &msg );
    void lineFitStart();
    void publish_marker();
    void theilsen(const std::vector<Point> &points, Point &start, Point &end);
private: // variables
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    Parameters param_;
    ros::Publisher pub_laser_line_split_;
    ros::Publisher pub_laser_line_fit_;
    ros::Publisher pub_laser_input_;
    ros::Publisher pub_marker_;
    ros::Subscriber sub_;
    dynamic_reconfigure::Server<v4r_laser_filter::LineFilterConfig> reconfigureServer_;
    dynamic_reconfigure::Server<v4r_laser_filter::LineFilterConfig>::CallbackType reconfigureFnc_;
    sensor_msgs::LaserScan msg_scan_;
    visualization_msgs::Marker msg_line_list_;
    std::vector<Beam> beams_;
    std::vector<Measurment> measurments_;
    std::vector<std::pair<unsigned int, unsigned int> > connectedMeasurments_;
    std::vector<LineSegment> lineSegments_;
    std::vector<Line> lines_;
    float  angle_increment_;
    float  angle_increment_sin_;

};

#endif // V4R_LASER_LINE_FILTER_NODE
