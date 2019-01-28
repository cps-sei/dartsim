/*******************************************************************************
 * DARTSim Mission Simulator
 *
 * Copyright 2019 Carnegie Mellon University. All Rights Reserved.
 * NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
 * INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
 * UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR IMPLIED, AS
 * TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF FITNESS FOR PURPOSE
 * OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF THE
 * MATERIAL. CARNEGIE MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY KIND
 * WITH RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT INFRINGEMENT.
 * 
 * Released under a BSD (SEI)-style license, please see license.txt or contact
 * permission@sei.cmu.edu for full terms.
 * 
 * [DISTRIBUTION STATEMENT A] This material has been approved for public release
 * and unlimited distribution. Please see Copyright notice for non-US Government
 * use and distribution.
 * 
 * Carnegie Mellon® is registered in the U.S. Patent and Trademark Office by
 * Carnegie Mellon University.
 * 
 * This Software includes and/or makes use of Third-Party Software, each subject
 * to its own license. See license.txt.
 * 
 * DM19-0045
 ******************************************************************************/

#pragma once

#include <vector>
#include <iostream>

namespace dart {
namespace sim {

using CoordT = int;

/**
 * Coordinate in 2D
 */
class Coordinate {
public:
	CoordT x;
	CoordT y;
	Coordinate(CoordT x = 0, CoordT y = 0) : x(x), y(y) {};
	bool operator==(const Coordinate& b) const {
		return x == b.x && y == b.y;
	}

	/**
	 * This defines a strict ordering
	 */
	bool operator<(const Coordinate& b) const {
		return x < b.x || (x == b.x && y < b.y);
	}

	/**
	 * @param b corner opposite from the origin, defining the rectangle
	 * @return true if coordinate is inside the rectangle defined by 0,0 and b
	 */
	bool isInsideRect(const Coordinate& b) const {
		return x < b.x && y < b.y;
	}

    void printOn(std::ostream& os) const;
    friend std::ostream& operator<<(std::ostream& os, const Coordinate& coord);
};


/**
 * A route is a vector of Coordinate
 *
 * The points in the route are not necessarily contiguous, so this class can be
 * used to represent a route with waypoints for example.
 */
class Route : public std::vector<Coordinate> {
public:
	Route() {};
	Route(Coordinate origin, double directionX, double directionY, unsigned length);
	virtual ~Route();
};

} /* namespace sim */
} /* namespace dart */
