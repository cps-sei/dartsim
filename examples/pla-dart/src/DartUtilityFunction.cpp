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

#include "DartUtilityFunction.h"
#include "DartConfiguration.h"
#include "DartEnvironment.h"
#include <algorithm>
#include <iostream>
#include <random>
#include <memory>

using namespace pladapt;
using namespace std;

namespace dart {
namespace am2 {

static const double INJECTED_RMSE = 0; //0.3 * 0.6;
std::unique_ptr<std::normal_distribution<>> pDist;
std::default_random_engine randomGenerator;

DartUtilityFunction::DartUtilityFunction(double targetDetectionRange, double detectionFormationFactor,
		double threatRange, double destructionFormationFactor,
		double finalReward,
		bool deterministic)
	: targetDetectionRange(targetDetectionRange), detectionFormationFactor(detectionFormationFactor),
		threatRange(threatRange), destructionFormationFactor(destructionFormationFactor),
		finalReward(finalReward), deterministic(deterministic)
{
	if (INJECTED_RMSE > 0) {
		//randomGenerator.seed(RandomSeed::getNextSeed());
		pDist.reset(new normal_distribution<>(0, INJECTED_RMSE));
		//pDist.reset(new normal_distribution<>(INJECTED_RMSE / 2, INJECTED_RMSE));
	}
}


/**
 * computes g(c_t) in the paper
 */
double DartUtilityFunction::getAdditiveUtility(
		const pladapt::Configuration& config, const pladapt::Environment& env,
		int time) const {
    auto& dartConfig = dynamic_cast<const DartConfiguration&>(config);
    auto dartEnv = DartEnvironment(dynamic_cast<const JointEnvironment&>(env));

	double probOfDetection = getProbabilityOfDetection(dartConfig);

	if (INJECTED_RMSE > 0) {
    	double error = pDist->operator ()(randomGenerator);
    	probOfDetection += error;
    	if (probOfDetection < 0) {
    		probOfDetection = 0;
    	} else if (probOfDetection > 1) {
    		probOfDetection = 1;
    	}
    }

	double utility = dartEnv.getProbOfTarget() * probOfDetection;

    return utility;
}

/**
 * computes s(c_t) in the paper
 */
double DartUtilityFunction::getMultiplicativeUtility(
		const pladapt::Configuration& config, const pladapt::Environment& env,
		int time) const {

	auto& dartConfig = dynamic_cast<const DartConfiguration&>(config);
    auto dartEnv = DartEnvironment(dynamic_cast<const JointEnvironment&>(env));

	double probOfDestruction = dartEnv.getProbOfThreat()
			* getProbabilityOfDestruction(dartConfig);

    return 1 - probOfDestruction;
}


double DartUtilityFunction::getFinalReward(
		const pladapt::Configuration& config, const pladapt::Environment& env,
		int time) const {
	return finalReward;
}


double DartUtilityFunction::getProbabilityOfDetection(const DartConfiguration& config) const {
	double probOfDetection =
			((config.getFormation() == DartConfiguration::Formation::LOOSE) ? 1.0 : 1 / detectionFormationFactor)
			* max(0.0, targetDetectionRange - (config.getAltitudeLevel() + 1)) / targetDetectionRange; // +1 because level 0 is one level above ground

	// ECM reduces the prob of detection
	if (config.getEcm()) {
		probOfDetection *= 0.25;
	}

	if (deterministic) {
		probOfDetection = (probOfDetection > 0.0) ? 1.0 : 0.0;
	}

	return probOfDetection;
}

double DartUtilityFunction::getProbabilityOfDestruction(const DartConfiguration& config) const {
	double probOfDestruction =
			((config.getFormation() == DartConfiguration::Formation::LOOSE) ? 1.0 : (1.0 / destructionFormationFactor))
			* max(0.0, threatRange - (config.getAltitudeLevel() + 1)) / threatRange; // +1 because level 0 is one level above ground

	// ECM reduces the prob of destruction
	if (config.getEcm()) {
		probOfDestruction *= 0.25;
	}

	if (deterministic) {
		probOfDestruction = (probOfDestruction > 0.0) ? 1.0 : 0.0;
	}

	return probOfDestruction;
}

DartUtilityFunction::~DartUtilityFunction() {
}

} /* namespace am2 */
} /* namespace dart */
