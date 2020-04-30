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
#include "SimulatorImpl.h"
#include <iostream>
#include <getopt.h>
#include <cstdlib>
#include <string.h>
#include "RandomSeed.h"


/**
 * If >0, it uses a sparse environment, meaning that there is no more than one
 * threat in any portion of the route of the length given by this constant. This only works with
 * a non-square map. This value is intended to match the look-ahead horizon of the adaptation
 * manager if it is desired to have only one threat in the range of the look-ahead at a time.
 *
 * This constant has to be 0 for a normal build.
 */
#define SPARSE_ENVIRONMENT 0


#if SPARSE_ENVIRONMENT
#include "RealEnvironmentSparse.h"
#endif

using namespace std;

namespace dart {
namespace sim {

const string Simulator::INC_ALTITUDE = "IncAlt";
const string Simulator::DEC_ALTITUDE = "DecAlt";
const string Simulator::INC_ALTITUDE2 = "IncAlt2";
const string Simulator::DEC_ALTITUDE2 = "DecAlt2";
const string Simulator::GO_TIGHT = "GoTight";
const string Simulator::GO_LOOSE = "GoLoose";
const string Simulator::ECM_ON = "EcmOn";
const string Simulator::ECM_OFF = "EcmOff";

enum ARGS {
	MAP_SIZE,
	SQUARE_MAP,
	NUM_TARGETS,
	NUM_THREATS,
	ALTITUDE_LEVELS,
	THREAT_RANGE,
	THREAT_SENSOR_FPR,
	THREAT_SENSOR_FNR,
	TARGET_SENSOR_FPR,
	TARGET_SENSOR_FNR,
	DL_TARGET_SENSOR_RANGE,
	AUTO_RANGE,
	CHANGE_ALT_LATENCY_PERIODS,
	SEED,
	OPT_TEST
};

static struct option long_options[] = {
	{"map-size", required_argument, 0,  MAP_SIZE },
	{"square-map", no_argument, 0,  SQUARE_MAP },
	{"num-targets", required_argument, 0,  NUM_TARGETS },
	{"num-threats", required_argument, 0,  NUM_THREATS },
	{"altitude-levels", required_argument, 0,  ALTITUDE_LEVELS },
	{"threat-range", required_argument, 0,  THREAT_RANGE },
    {"threat-sensor-fpr", required_argument, 0,  THREAT_SENSOR_FPR },
    {"threat-sensor-fnr",  required_argument, 0,  THREAT_SENSOR_FNR },
    {"target-sensor-fpr", required_argument, 0,  TARGET_SENSOR_FPR },
    {"target-sensor-fnr",  required_argument, 0,  TARGET_SENSOR_FNR },
	{"dl-target-sensor-range", required_argument, 0,  DL_TARGET_SENSOR_RANGE },
	{"auto-range", no_argument, 0,  AUTO_RANGE },
	{"change-alt-latency", required_argument, 0, CHANGE_ALT_LATENCY_PERIODS },
	{"seed", required_argument, 0, SEED },
	{"opt-test", no_argument, 0, OPT_TEST },
    {0, 0, 0, 0 }
};

void Simulator::usage() {
	cout << "valid simulator options are:" << endl;
	int opt = 0;
	while (long_options[opt].name != 0) {
		cout << "\t--" << long_options[opt].name;
		if (long_options[opt].has_arg == required_argument) {
			cout << "=value";
		}
		cout << endl;
		opt++;
	}
}

Simulator* Simulator::createInstance(int argc, char** argv) {
	dart::sim::SimulationParams simParams;
	bool autoRange = false;
	unsigned numThreats = 6;
	unsigned numTargets = 4;

	// split options
	int simArgc = 0;

	while (simArgc < argc) {
		if (strcmp(argv[simArgc++], "--") == 0) {
			simArgc--;
			argv[simArgc] = nullptr;
			break;
		}
	}

	while (1) {
		int option_index = 0;

		auto c = getopt_long(simArgc, argv, "", long_options, &option_index);

		if (c == -1) {
			break;
		}

		switch (c) {
		case MAP_SIZE:
			simParams.mapSize = atoi(optarg);
			break;
		case SQUARE_MAP:
			simParams.squareMap = true;
			break;
		case NUM_TARGETS:
			numTargets = atoi(optarg);
			break;
		case NUM_THREATS:
			numThreats = atoi(optarg);
			break;
		case ALTITUDE_LEVELS:
			simParams.altitudeLevels = atoi(optarg);
			break;
		case THREAT_RANGE:
			simParams.threat.threatRange = atoi(optarg);
			break;
		case THREAT_SENSOR_FPR:
			simParams.longRangeSensor.threatSensorFPR = atof(optarg);
			break;
		case THREAT_SENSOR_FNR:
			simParams.longRangeSensor.threatSensorFNR = atof(optarg);
			break;
		case TARGET_SENSOR_FPR:
			simParams.longRangeSensor.targetSensorFPR = atof(optarg);
			break;
		case TARGET_SENSOR_FNR:
			simParams.longRangeSensor.targetSensorFNR = atof(optarg);
			break;
		case DL_TARGET_SENSOR_RANGE:
			simParams.downwardLookingSensor.targetSensorRange = atoi(optarg);
			break;
		case AUTO_RANGE:
			autoRange = true;
			break;
		case CHANGE_ALT_LATENCY_PERIODS:
			simParams.changeAltitudeLatencyPeriods = atoi(optarg);
			break;
		case SEED:
			dart::sim::RandomSeed::seed(atoi(optarg));
			break;
		case OPT_TEST:
			simParams.optimalityTest = true;
			break;
		default:
			return nullptr;
		}
	}

	if (optind < simArgc) {
		return nullptr;
	}

	if (autoRange) {
		simParams.downwardLookingSensor.targetSensorRange = simParams.altitudeLevels;
		simParams.threat.threatRange = simParams.altitudeLevels * 3 / 4;
		cout << "autorange: threat range is " << simParams.threat.threatRange << endl;
	}

	if (numTargets > simParams.mapSize) {
		cout << "error: number of targets cannot be larger than map size" << endl;
		return nullptr;
	}

	if (numThreats > simParams.mapSize) {
		cout << "error: number of threats cannot be larger than map size" << endl;
		return nullptr;
	}


	// generate environment
#if FIXED2DSPACE
	RealEnvironment threatEnv;
	threatEnv.populate(Coordinate(10, 10), 0);

	RealEnvironment targetEnv;
	targetEnv.populate(Coordinate(10, 10), 0);

	threatEnv.setAt(Coordinate(2,2), true);
	threatEnv.setAt(Coordinate(3,2), true);
	threatEnv.setAt(Coordinate(6,6), true);
	threatEnv.setAt(Coordinate(7,7), true);

	targetEnv.setAt(Coordinate(5,2), true);
	targetEnv.setAt(Coordinate(7,2), true);
	targetEnv.setAt(Coordinate(7,5), true);
#else
#if SPARSE_ENVIRONMENT
	cout << "warning: built with SPARSE_ENVIRONMENT=" << SPARSE_ENVIRONMENT << endl;
	dart::sim::RealEnvironmentSparse threatEnv;
#else
	dart::sim::RealEnvironment threatEnv;
#endif
	dart::sim::RealEnvironment targetEnv;

	if (simParams.squareMap) {

		/* generate true environment */
#if SPARSE_ENVIRONMENT
		threatEnv.populate(dart::sim::Coordinate(simParams.mapSize, simParams.mapSize), numThreats, SPARSE_ENVIRONMENT);
#else
		threatEnv.populate(dart::sim::Coordinate(simParams.mapSize, simParams.mapSize), numThreats);
#endif		
		targetEnv.populate(dart::sim::Coordinate(simParams.mapSize, simParams.mapSize), numTargets);
	} else {

		/* generate true environment */
#if SPARSE_ENVIRONMENT
		threatEnv.populate(dart::sim::Coordinate(simParams.mapSize, 1), numThreats, SPARSE_ENVIRONMENT);
#else
		threatEnv.populate(dart::sim::Coordinate(simParams.mapSize, 1), numThreats);
#endif
		targetEnv.populate(dart::sim::Coordinate(simParams.mapSize, 1), numTargets);
	}
#endif


	// generate route
	dart::sim::Route route;

#if FIXED2DSPACE
	unsigned x = 2;
	unsigned y = 2;
	while (x < 7) {
		route.push_back(Coordinate(x,y));
		x++;
	}
	while (y <= 6) {
		route.push_back(Coordinate(x,y));
		y++;
	}
#else
	if (simParams.squareMap) {
		for (unsigned y = 0; y < simParams.mapSize; y++) {
			if (y % 2) {
				for (unsigned x = simParams.mapSize; x > 0; x--) {
					route.push_back(dart::sim::Coordinate(x - 1, y));
				}
			} else {
				for (unsigned x = 0; x < simParams.mapSize; x++) {
					route.push_back(dart::sim::Coordinate(x, y));
				}
			}
		}
	} else {
		route = dart::sim::Route(dart::sim::Coordinate(0,0), 1.0, 0.0, simParams.mapSize);
	}
#endif

	// change parameters if doing optimality test
	if (simParams.optimalityTest) {
		simParams.longRangeSensor.targetSensorFNR = 0;
		simParams.longRangeSensor.targetSensorFPR = 0;
		simParams.longRangeSensor.threatSensorFNR = 0;
		simParams.longRangeSensor.threatSensorFPR = 0;

		// autorange
		simParams.downwardLookingSensor.targetSensorRange = simParams.altitudeLevels / 2.0;
		simParams.threat.threatRange = simParams.altitudeLevels * 3.0 / 4;
		cout << "ranges sensor=" << simParams.downwardLookingSensor.targetSensorRange
				<< " threat=" << simParams.threat.threatRange << endl;
	}


	// instantiate adaptation manager
	// replace seeds that would have been consumed by the sensor and threat used in the old utility fc
	dart::sim::RandomSeed::getNextSeed();
	dart::sim::RandomSeed::getNextSeed();
	dart::sim::RandomSeed::getNextSeed();
	dart::sim::RandomSeed::getNextSeed();

	unsigned missionSuccessTargetThreshold = numTargets / 2.0;
	return new SimulatorImpl(simParams, threatEnv, targetEnv,
			route, missionSuccessTargetThreshold);
}


Simulator::~Simulator() {
}

} /* namespace sim */
} /* namespace dart */

