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


#ifndef MX_TEMPLATE_NODE_H
#define MX_TEMPLATE_NODE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "template/template.h"

/// ROS Node
class Scan2LineNode : public Scan2Line {
public:
	struct ParametersNode : public Parameters{
		ParametersNode();
		void update(const unsigned long &loop_count);
	    ros::NodeHandle node;
	    double rate;
	    int parameter_update_skip;
	    bool publish;
	};
    Scan2LineNode ( ros::NodeHandle &n );
    ~Scan2LineNode();
    void init ();
    void loop ();
    void callbackString (const std_msgs::String::ConstPtr& msg);
private: //functions
    ParametersNode *param();
    void update ();
    ros::Publisher pubString_;
    ros::Subscriber subString_;
private: // variables
    ros::NodeHandle n_;
    unsigned long loop_count_;

};

#endif // MX_TEMPLATE_NODE_H
