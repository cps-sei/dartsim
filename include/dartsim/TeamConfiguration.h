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

namespace dart {
namespace sim {


/**
 * Configuration of the team
 */
class TeamConfiguration {
public:
	enum Formation { LOOSE, TIGHT };

	unsigned altitudeLevel; /**< Altitude level of the team */
	Formation formation; /**< Current formation */
	bool ecm; /**< Whether electronic countermeasures (ECM) are on */
	unsigned ttcIncAlt; /**< time in periods to complete altitude increase, 0 means not executing */
	unsigned ttcDecAlt; /**< time in periods to complete altitude decrease, 0 means not executing */
	unsigned ttcIncAlt2; /**< time in periods to complete altitude increase 2, 0 means not executing */
	unsigned ttcDecAlt2; /**< time in periods to complete altitude decrease 2, 0 means not executing */
};

} /* namespace sim */
} /* namespace dart */

