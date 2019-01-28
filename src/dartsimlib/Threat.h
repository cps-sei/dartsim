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

#include "RealEnvironment.h"
#include <dartsim/TeamConfiguration.h>
#include <random>

namespace dart {
namespace sim {

/**
 * Simulates the effect of threats present in the environment
 */
class Threat {
public:
	Threat(double range, double destructionFormationFactor);
	virtual ~Threat();

	/**
	 * Simulates the effect of a (possible) threat
	 *
	 * @param threatEnv real threat environment
	 * @param config current team configuration
	 * @param location current team location
	 * @return true if destroyed by threat
	 */
	virtual bool isDestroyed(const RealEnvironment& threatEnv, const TeamConfiguration& config, const Coordinate& location);

	/**
	 * Computes probability of destruction given that there is a threat
	 */
	virtual double getProbabilityOfDestruction(const TeamConfiguration& config);

protected:
	double range;
	double destructionFormationFactor;
	std::uniform_real_distribution<> uniform;
	std::default_random_engine randomGenerator;
};

} /* namespace sim */
} /* namespace dart */
