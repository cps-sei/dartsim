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

#ifndef PARAMETERS_H_
#define PARAMETERS_H_


#include <string>
#include "DartDTMCEnvironment.h"
#include <dartsim/Simulator.h>

namespace dart {
namespace am2 {


struct LongRangeSensorParams {
	int threatObservationsPerCycle = 4;
	int targetObservationsPerCycle = 4;
};

struct ConfigurationSpaceParams {
	bool twoLevelTactics = false;
	bool hasEcm = false;
};

struct AdaptationManagerParams {

	AdaptationManagerParams();

	std::string mgr = "sdpra"; /**< identifier for the adaptation manager to use */

	unsigned horizon = 5;
	bool nonLatencyAware = false;
	DartDTMCEnvironment::DistributionApproximation distributionApproximation = DartDTMCEnvironment::DistributionApproximation::E_PT;
	std::string reachPath = "reach/reach.sh";
	std::string reachModel = "reach/model/dart2";
	std::string reachPrefix = "";
	double probabilityBound = 0.90; /**< lower bound on the probability of survival */
	double finalReward = 0.00001; // so that all else being equal, it'll favor surviving
	std::string prismTemplate = "model/dart2";

#if DART_USE_CE
	//-- ce solver parameters
	int ce_samples = 100;
	double ce_alpha = 0.3;
	double ce_precision = 0.01;
	int ce_maxIterations = 100;
	double ce_hintWeight = 0.0;
	bool ce_incremental = true;
#endif

};

struct Params {
	LongRangeSensorParams longRangeSensor;
	dart::sim::SimulationParams simulationParams;
	AdaptationManagerParams adaptationManager;
	ConfigurationSpaceParams configurationSpace;
};


} /* namespace am2 */
} /* namespace dart */



#endif /* PARAMETERS_H_ */
