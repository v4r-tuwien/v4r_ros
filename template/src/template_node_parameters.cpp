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
#include "template_node_defaults.h"

Scan2LineNode::ParametersNode::ParametersNode()
: Parameters(), node("~") {
	node.param<double>("rate", rate, MX_TEMPLATE_NODE_DEFAULT_RATE);
    ROS_INFO("rate: %f", rate);
    node.param<int>("parameter_update_skip", parameter_update_skip, MX_TEMPLATE_NODE_DEFAULT_PARAMETER_UPDATE_SKIP);
    ROS_INFO("parameter_update_skip: %i", parameter_update_skip);
	node.getParam("publish", publish);
	ROS_INFO("publish:  %s", (publish ? "true" : "false"));

}

void Scan2LineNode::ParametersNode::update(const unsigned long &loop_count){
	if(loop_count % parameter_update_skip) return;
	node.getParam("debug", debug);
	if(loop_count == 0) ROS_INFO("debug:  %s", (debug ? "true" : "false"));

}
