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

#include "Parameters.h"
#include <cstdlib>
#include <sstream>

namespace dart {
namespace am2 {

AdaptationManagerParams::AdaptationManagerParams() {
	
	// try to set good defaults for path dependent properties
	
	// check to see if PLADAPT environment variable is defined
	char* pladaptPath = std::getenv("PLADAPT");
	if (pladaptPath) {
		std::ostringstream newReachPath;
		newReachPath << pladaptPath;
		newReachPath << '/';
		newReachPath << reachPath;
		reachPath = newReachPath.str();

		std::ostringstream newReachModel;
		newReachModel << pladaptPath;
		newReachModel << '/';
		newReachModel << reachModel;
		reachModel = newReachModel.str();
	}
}

} /* namespace am2 */
} /* namespace dart */
