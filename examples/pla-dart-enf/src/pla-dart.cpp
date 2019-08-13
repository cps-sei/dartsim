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
#include <dartsim/Simulator.h>
#include "Parameters.h"
#include <iostream>
#include <getopt.h>
#include <cstdlib>
#include <chrono>
#include "DartDetectionUtilityFunction.h"
#include "DartSurvivalUtilityFunction.h"
#include "DartAdaptationManager.h"

// set this to 1 for testing
#define FIXED2DSPACE 0

// set to 1 to make random results compatible with original version
#define RANDOMSEED_COMPATIBILITY 0
#define SUPPORT_OPTIMALITY_TEST 1

using namespace dart::am2;
using namespace std;

using myclock = chrono::high_resolution_clock;


enum ARGS {
	LOOKAHEAD_horizon,
	reachPath,
	reachModel,
	DISTRIB_APPROX,
	NON_LATENCY_AWARE,
	PROBABILITY_BOUND,
	IMPROVEMENT_THRESHOLD,
	STAY_ALIVE_REWARD,
	NO_FORMATION,
	ECM,
	TWO_LEVEL_TACTICS,
	ADAPT_MGR,
	prismTemplate,
#if DART_USE_CE
	,
	CE_NONINCREMENTAL,
	CE_HINT_WEIGHT,
	CE_SAMPLES,
	CE_ALPHA,
	CE_PRECISION,
	CE_MAX_ITERATIONS
#endif
};

static struct option long_options[] = {
    {"lookahead-horizon",  required_argument, 0,  LOOKAHEAD_horizon },
    {"reach-path",  required_argument, 0,  reachPath },
    {"reach-model",  required_argument, 0,  reachModel },
	{"distrib-approx", required_argument, 0, DISTRIB_APPROX },
	{"non-latency-aware", no_argument, 0, NON_LATENCY_AWARE },
	{"probability-bound", required_argument, 0, PROBABILITY_BOUND },
	{"improvement-threshold", required_argument, 0, IMPROVEMENT_THRESHOLD },
	{"stay-alive-reward", required_argument, 0, STAY_ALIVE_REWARD },
	{"no-formation", no_argument, 0, NO_FORMATION },
	{"ecm", no_argument, 0, ECM },
	{"two-level-tactics", no_argument, 0, TWO_LEVEL_TACTICS },
	{"adapt-mgr", required_argument, 0, ADAPT_MGR },
    {"prism-template",  required_argument, 0,  prismTemplate },
#if DART_USE_CE
	{"ce-nonincremental", no_argument, 0, CE_NONINCREMENTAL },
	{"ce-hint-weight", required_argument, 0, CE_HINT_WEIGHT },
	{"ce-samples", required_argument, 0, CE_SAMPLES },
	{"ce-alpha", required_argument, 0, CE_ALPHA },
	{"ce-precision", required_argument, 0, CE_PRECISION },
	{"ce-max-iterations", required_argument, 0, CE_MAX_ITERATIONS },
#endif
    {0, 0, 0, 0 }
};

static void usage() {
	cout << "options: " << endl;
	cout << "\t[simulator options] [-- [adaptation manager options]]" << endl;
	dart::sim::Simulator::usage();
	cout << "valid adaptation manager options are:" << endl;
	int opt = 0;
	while (long_options[opt].name != 0) {
		cout << "\t--" << long_options[opt].name;
		if (long_options[opt].has_arg == required_argument) {
			cout << "=value";
		}
		cout << endl;
		opt++;
	}
	exit(EXIT_FAILURE);
}

int main(int argc, char** argv) {

	// instantiate sim first

	/*
	 * Split all command-line options first
	 * All the options before a -- arg are for the sim, the rest are for
	 * the adaptation manager
	 */
	int simArgc = 0;

	while (simArgc < argc) {
		if (strcmp(argv[simArgc++], "--") == 0) {
			simArgc--;
			argv[simArgc] = nullptr;
			break;
		}
	}

	dart::sim::Simulator *simp = dart::sim::Simulator::createInstance(simArgc, argv);
	if (!simp) {
		usage();
	}

	dart::sim::Simulator &sim = *simp;

	dart::am2::Params adaptParams;

	argv[simArgc] = argv[0];
	int amArgc = argc - simArgc;
	char **amArgv = argv + simArgc;

	optind = 1; // reset getopt scanning

	while (1) {
		int option_index = 0;

		auto c = getopt_long(amArgc, amArgv, "", long_options, &option_index);

		if (c == -1) {
			break;
		}

		switch (c) {
		case LOOKAHEAD_horizon:
			adaptParams.adaptationManager.horizon = atoi(optarg);
			break;
		case reachModel:
			adaptParams.adaptationManager.reachModel = optarg;
			break;
		case reachPath:
			adaptParams.adaptationManager.reachPath = optarg;
			break;
		case DISTRIB_APPROX:
			adaptParams.adaptationManager.distributionApproximation =
					(DartDTMCEnvironment::DistributionApproximation) atoi(optarg);
			break;
		case NON_LATENCY_AWARE:
			adaptParams.adaptationManager.nonLatencyAware = true;
			break;
		case PROBABILITY_BOUND:
			adaptParams.adaptationManager.probabilityBound = atof(optarg);
			break;
		case IMPROVEMENT_THRESHOLD:
			adaptParams.adaptationManager.improvementThreshold = atof(optarg);
			break;
		case STAY_ALIVE_REWARD:
			adaptParams.adaptationManager.finalReward = atof(optarg);
			break;
		case NO_FORMATION:
			adaptParams.adaptationManager.reachModel += "-formation-disabled";
			break;
		case ECM:
			adaptParams.configurationSpace.hasEcm = true;
			break;
		case TWO_LEVEL_TACTICS:
			adaptParams.configurationSpace.twoLevelTactics = true;
			break;
		case ADAPT_MGR:
			adaptParams.adaptationManager.mgr = optarg;
			break;
		case prismTemplate:
			adaptParams.adaptationManager.prismTemplate = optarg;
			break;
#if DART_USE_CE
		case CE_NONINCREMENTAL:
			adaptParams.adaptationManager.ce_incremental = false;
			break;
		case CE_HINT_WEIGHT:
			adaptParams.adaptationManager.ce_hintWeight = atof(optarg);
			break;
		case CE_SAMPLES:
			adaptParams.adaptationManager.ce_samples = atoi(optarg);
			break;
		case CE_ALPHA:
			adaptParams.adaptationManager.ce_alpha = atof(optarg);
			break;
		case CE_PRECISION:
			adaptParams.adaptationManager.ce_precision = atof(optarg);
			break;
		case CE_MAX_ITERATIONS:
			adaptParams.adaptationManager.ce_maxIterations = atoi(optarg);
			break;
#endif
		default:
			usage();
		}
	}

	if (optind < amArgc) {
		usage();
	}

	if (adaptParams.configurationSpace.twoLevelTactics) {
		adaptParams.adaptationManager.reachModel += "-2l";
	}

	if (adaptParams.configurationSpace.hasEcm) {
		adaptParams.adaptationManager.reachModel += "-ecm";
	}


	adaptParams.simulationParams = sim.getParameters();

#if SUPPORT_OPTIMALITY_TEST
	/*
	 *  optimal testing
	 *  This makes a single adaptation decision for the whole route, computing
	 *  the complete strategy along the route. It then uses that strategy for
	 *  the whole simulation.
	 */

	// change parameters
	if (adaptParams.simulationParams.optimalityTest) {
		if (adaptParams.simulationParams.squareMap) {
			cout << "error: optimality test not supported with square map" << endl;
			usage();
		}
		adaptParams.adaptationManager.horizon = adaptParams.simulationParams.mapSize;
		adaptParams.adaptationManager.distributionApproximation = DartDTMCEnvironment::DistributionApproximation::POINT;
	}

	std::shared_ptr<pladapt::Strategy> strategy;
	pladapt::Strategy::iterator strategyIterator;
	bool gotStrategy = false;
#endif

	/* initialize adaptation manager */
	DartAdaptationManager adaptMgr;
	adaptMgr.initialize(adaptParams,
			unique_ptr<pladapt::UtilityFunction>(
					new DartDetectionUtilityFunction(adaptParams.simulationParams.downwardLookingSensor.targetSensorRange,
							adaptParams.simulationParams.downwardLookingSensor.targetDetectionFormationFactor,
							adaptParams.simulationParams.threat.threatRange,
							adaptParams.simulationParams.threat.destructionFormationFactor,
							adaptParams.adaptationManager.finalReward,
							adaptParams.simulationParams.optimalityTest)),
			unique_ptr<pladapt::UtilityFunction>(
					new DartSurvivalUtilityFunction(
							adaptParams.simulationParams.threat.threatRange,
							adaptParams.simulationParams.threat.destructionFormationFactor,
							adaptParams.simulationParams.optimalityTest))
							);

	if (adaptParams.simulationParams.optimalityTest && !adaptMgr.supportsStrategy()) {
		throw std::invalid_argument("selected adaptation manager does not support full strategies");
	}

	/* create environment monitors */
	EnvironmentMonitor envThreatMonitor;
	EnvironmentMonitor envTargetMonitor;

	while (!sim.finished()) {
		auto simState = sim.getState();

		cout << "current position: " << simState.position << endl;

		DartMonitoringInfo monitoringInfo;
		monitoringInfo.position.x = simState.position.x;
		monitoringInfo.position.y = simState.position.y;
		monitoringInfo.altitudeLevel = simState.config.altitudeLevel - 1;
		monitoringInfo.directionX = simState.directionX;
		monitoringInfo.directionY = simState.directionY;

		monitoringInfo.formation =
				(simState.config.formation
						== dart::sim::TeamConfiguration::Formation::LOOSE) ?
						DartConfiguration::Formation::LOOSE :
						DartConfiguration::Formation::TIGHT;
		monitoringInfo.ttcFormationChange = 0;
		monitoringInfo.ttcIncAlt = simState.config.ttcIncAlt;
		monitoringInfo.ttcDecAlt = simState.config.ttcDecAlt;
		monitoringInfo.ttcIncAlt2 = simState.config.ttcIncAlt2;
		monitoringInfo.ttcDecAlt2 = simState.config.ttcDecAlt2;
		monitoringInfo.ecm = simState.config.ecm;

		/* monitor environment */
		dart::sim::Route senseRoute(monitoringInfo.position, monitoringInfo.directionX, monitoringInfo.directionY, adaptParams.adaptationManager.horizon);
#if !RANDOMSEED_COMPATIBILITY

		// unless compatibility is required, this is preferred (more efficient)

		envThreatMonitor.clear();
		for (int i = 0l; i < adaptParams.longRangeSensor.threatObservationsPerCycle; i++) {
			envThreatMonitor.processSensorReadings(senseRoute, sim.readForwardThreatSensor(senseRoute.size()));
		}
		envTargetMonitor.clear();
		for (int i = 0l; i < adaptParams.longRangeSensor.targetObservationsPerCycle; i++) {
			envTargetMonitor.processSensorReadings(senseRoute, sim.readForwardTargetSensor(senseRoute.size()));
		}
#else
		envThreatMonitor.clear();
		auto sensed = sim.readForwardThreatSensor(senseRoute.size(), adaptParams.longRangeSensor.threatObservationsPerCycle);
		for (int i = 0; i < adaptParams.longRangeSensor.threatObservationsPerCycle; i++) {
			std::vector<bool> oneObservation;
			auto pos = senseRoute.begin();
			for (const auto& obsVector : sensed) {
				oneObservation.push_back(obsVector[i]);
				pos++;
			}
			envThreatMonitor.processSensorReadings(senseRoute, oneObservation);
		}
		envTargetMonitor.clear();
		sensed = sim.readForwardTargetSensor(senseRoute.size(), adaptParams.longRangeSensor.targetObservationsPerCycle);
		for (int i = 0; i < adaptParams.longRangeSensor.targetObservationsPerCycle; i++) {
			std::vector<bool> oneObservation;
			for (const auto& obsVector : sensed) {
				oneObservation.push_back(obsVector[i]);
			}
			envTargetMonitor.processSensorReadings(senseRoute, oneObservation);
		}
#endif
		monitoringInfo.threatSensing = envThreatMonitor.getResults(senseRoute);
		monitoringInfo.targetSensing = envTargetMonitor.getResults(senseRoute);

		pladapt::TacticList tactics;

		double deltaMsec = 0.0; // adaptation decision time
#if SUPPORT_OPTIMALITY_TEST
		if (!adaptParams.simulationParams.optimalityTest || !gotStrategy) {
#endif
			auto startTime = myclock::now();
			tactics  = adaptMgr.decideAdaptation(monitoringInfo);
			auto delta = myclock::now() - startTime;
			deltaMsec = chrono::duration_cast<chrono::duration<double, std::milli>>(delta).count();

#if SUPPORT_OPTIMALITY_TEST
			if (adaptParams.simulationParams.optimalityTest) {
				gotStrategy = true;
				strategy = adaptMgr.getStrategy();
				strategyIterator = strategy->begin();
			}
		} else {
			strategyIterator++;
			assert(strategyIterator != strategy->end());
			tactics = *strategyIterator;
		}
#endif
		sim.step(tactics, deltaMsec);
	}

	auto results = sim.getResults();
	if (!results.destroyed) {
		cout << "Total targets detected: " << results.targetsDetected << endl;
	}

	cout << sim.getScreenOutput();

	const std::string RESULTS_PREFIX = "out:";
	cout << RESULTS_PREFIX << "destroyed=" << results.destroyed << endl;
	cout << RESULTS_PREFIX << "targetsDetected=" << results.targetsDetected << endl;
	cout << RESULTS_PREFIX << "missionSuccess=" << results.missionSuccess << endl;

	cout << "csv," << results.targetsDetected << ',' << results.destroyed
			<< ',' << results.whereDestroyed.x
			<< ',' << results.missionSuccess
			<< ',' << results.decisionTimeAvg
			<< ',' << results.decisionTimeVar
			<<  endl;

	delete simp;

	return 0;
}
