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

#include "RealEnvironmentSparse.h"
#include "RandomSeed.h"
#include <cassert>
#include <list>


using namespace std;

namespace dart {
namespace sim {

RealEnvironmentSparse::RealEnvironmentSparse() {
	// TODO Auto-generated constructor stub

}


void RealEnvironmentSparse::populate(Coordinate size, unsigned numOfObjects, unsigned horizon) {
	assert(size.y == 1); // only for linear maps
	assert(size.x / (horizon * 2 - 1) >= numOfObjects);

	this->size = size;
	envMap.clear();

	std::default_random_engine gen(RandomSeed::getNextSeed());

	list<unsigned> availablePositions;
	for (unsigned i = 0; i < (unsigned) size.x; i++) {
		availablePositions.push_back(i);
	}

	while (numOfObjects > 0) {
//		cout << "threat " << numOfObjects << " chkpt 1" << endl;
		std::uniform_int_distribution<> unifX(0, availablePositions.size() - 1);
		unsigned idx_x = unifX(gen);
		auto it = availablePositions.begin();
		assert(it != availablePositions.end());
		for (unsigned i = 0; i < idx_x; i++) {
			it++;
			assert(it != availablePositions.end());
		}
		unsigned x = *it;
//		cout << "threat " << numOfObjects << " chkpt 2" << endl;

		assert(!envMap[Coordinate(x,0)]);
		envMap[Coordinate(x,0)] = true;

		// remove neighboring available positions
		unsigned upperEnd = x + horizon - 1;
		unsigned lowerEnd = (x > (horizon - 1)) ? x - (horizon - 1) : 0u;
		while (it != availablePositions.end() && *it <= upperEnd) {
			it = availablePositions.erase(it);
		}
//		cout << "threat " << numOfObjects << " chkpt 3" << endl;
		while (it != availablePositions.begin()) {
			it--;
			if (*it < lowerEnd) {
				break;
			}
			it = availablePositions.erase(it);
		}
//		cout << "threat " << numOfObjects << " chkpt 4" << endl;
		numOfObjects--;
	}
}

RealEnvironmentSparse::~RealEnvironmentSparse() {
	// TODO Auto-generated destructor stub
}

} /* namespace sim */
} /* namespace dart */
