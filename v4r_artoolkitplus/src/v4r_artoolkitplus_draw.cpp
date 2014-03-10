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

#include <v4r_artoolkitplus/v4r_artoolkitplus.h>

void ARToolKitPlusNode::generateDebugImage(cv::Mat &img) {
    char text[0xFF];
    cv::Point a, b, c;
    cv::Scalar green(0, 255, 0);
    cv::Scalar red(0, 0, 255);
    cv::Scalar blue(255, 0, 0);
    int fondFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    for (int i = 0; i < arMarkerInfo_.size(); i++) {
        ARToolKitPlus::ARMarkerInfo &artag = arMarkerInfo_[i];
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
}