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

#ifndef ARTOOLKITPLUS_NODE_DEFAULTS_H
#define ARTOOLKITPLUS_NODE_DEFAULTS_H

#define IMAGE_SRC "image"
#define DEBUG_WINDOWS_NAME "artoolkitplus"
#define DEFAULT_SKIP_FRAMES 0
#define DEFAULT_SHOW_CAMERA_IMAGE true
#define DEFAULT_BASE_FRAME "camera"
#define DEFAULT_PATTERN_FRAME "pattern"
#define DEFAULT_PATTERN_FILE ""
#define DEFAULT_TF_PREFIX ""

#define MARKER_MODE_BCH "bch"
#define MARKER_MODE_SIMPEL "simple"
#define DEFAULT_MARKER_MODE MARKER_MODE_BCH // "SIMPLE
#define DEFAULT_PATTERN_WITH 0.1
#define DEFAULT_THRESHOLD 0
#define DEFAULT_BOARDER_WIDTH 0
#define DEFAULT_UNDIST_INTERATIONS 10
#define UNDIST_MODE_NONE "none"
#define UNDIST_MODE_STD "std"
#define UNDIST_MODE_LUT "lut"
#define DEFAULT_UNDIST_MODE UNDIST_MODE_LUT
#define DEFAULT_INPUT_DISTORTED true
#define POSE_ESTIMATION_MODE_NORMAL "normal"
#define POSE_ESTIMATION_MODE_CONT "cont"
#define POSE_ESTIMATION_MODE_RPP "rpp"
#define DEFAULT_POSE_ESTIMATION_MODE POSE_ESTIMATION_MODE_RPP
#define DEFAULT_TRACKER_SINGLE_MARKER true
#define DEFAULT_TRACKER_MULTI_MARKER false

#endif // ARTOOLKITPLUS_NODE_DEFAULTS_H
