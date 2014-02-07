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

#include "template_node.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "Scan2Line");
	ros::NodeHandle n;
	Scan2LineNode my_node(n);
	my_node.init();
	my_node.loop();
	return 0;
}

Scan2LineNode::~Scan2LineNode() {
}

Scan2LineNode::Scan2LineNode(ros::NodeHandle &n) :
		Scan2Line(new Scan2LineNode::ParametersNode()), n_(n), loop_count_(0) {

}

Scan2LineNode::ParametersNode *Scan2LineNode::param() {
	return (Scan2LineNode::ParametersNode*) param_;
}

void Scan2LineNode::init() {
	if (param()->publish) {
		pubString_ = n_.advertise<std_msgs::String>("chatter", 1);
	} else {
		subString_ = n_.subscribe("chatter", 1000, &Scan2LineNode::callbackString, this);
	}

}

void Scan2LineNode::loop() {
	for (ros::Rate rate(param()->rate); ros::ok(); loop_count_++) {
		param()->update(loop_count_);
		if (param()->publish) {
			std_msgs::String str;
			char text[0xFF];
			sprintf(text, "%4lu Hallo World!", loop_count_);
			str.data = text;
			pubString_.publish(str);
		}
		ros::spinOnce();
		rate.sleep();
	}
}

void Scan2LineNode::callbackString(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("callbackString:  %s", msg->data.c_str());
}
