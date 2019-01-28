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

#include <dartsim/Route.h>

namespace dart {
namespace sim {

Route::Route(Coordinate origin, double directionX, double directionY, unsigned length) {
	// operate in doubles so that fractional directions can be used
	double x = origin.x;
	double y = origin.y;

	while (length > 0) {
		push_back(Coordinate(x, y));
		x += directionX;
		y += directionY;
		length--;
	}
}

Route::~Route() {
}



void Coordinate::printOn(std::ostream& os) const {
	os << x << ';' << y;
}

std::ostream& operator<<(std::ostream& os, const Coordinate& coord) {
	coord.printOn(os);
	return os;
}


} /* namespace sim */
} /* namespace dart */
