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
#include "DeterministicThreat.h"
#include "DeterministicTargetSensor.h"
#include <math.h>
#include <sstream>

using namespace std;

namespace dart {
namespace sim {

SimulatorImpl::SimulatorImpl(const SimulationParams& simParams,
		const RealEnvironment& threatEnv, const RealEnvironment& targetEnv,
		const Route& route, unsigned missionSuccessTargetThreshold)
	: params(simParams),
	  threatEnv(threatEnv), targetEnv(targetEnv), route(route),
	  screen(route.size(), vector<char>(simParams.altitudeLevels + 2, ' ')),
	  currentConfig({simParams.altitudeLevels, TeamConfiguration::Formation::LOOSE, false, 0, 0, 0, 0}),
	  routeIt(this->route.begin()),
	  changeAltitudeLatencyPeriods(simParams.changeAltitudeLatencyPeriods),
	  position(*routeIt),
	  MISSION_SUCCESS_THRESHOLD(missionSuccessTargetThreshold),
	  SCREEN_THREATS(simParams.altitudeLevels),
	  SCREEN_TARGETS(simParams.altitudeLevels + 1)
{

	/* create simulators of target sensors and threats */
	pTargetSensor = createTargetSensor(simParams);
	pThreatSim = createThreatSim(simParams);

	/* create forward-looking sensors */
	pFwdThreatSensor = make_unique<Sensor>(simParams.longRangeSensor.threatSensorFPR,
			simParams.longRangeSensor.threatSensorFNR);
	pFwdTargetSensor = make_unique<Sensor>(simParams.longRangeSensor.targetSensorFPR,
			simParams.longRangeSensor.targetSensorFNR);

	updateDirection();

	/* initialize screen */
	for (unsigned p = 0; p < route.size(); p++) {
		for (unsigned h = 0; h < simParams.altitudeLevels; h++) {
			screen[p][h] = ' ';
		}
		if (threatEnv.isObjectAt(route.at(p))) {
			screen[p][SCREEN_THREATS] = '^';
		} else {
			screen[p][SCREEN_THREATS] = ' ';
		}
		if (targetEnv.isObjectAt(route.at(p))) {
			screen[p][SCREEN_TARGETS] = 'T';
		} else {
			screen[p][SCREEN_TARGETS] = ' ';
		}
	}

}

SimulationParams SimulatorImpl::getParameters() const {
	return params;
}


shared_ptr<Threat> SimulatorImpl::createThreatSim(const SimulationParams& simParams) {
	shared_ptr<Threat> pThreatSim;
	if (simParams.optimalityTest) {
		pThreatSim = make_shared<DeterministicThreat>(
				simParams.threat.threatRange,
				simParams.threat.destructionFormationFactor);
	} else {
		pThreatSim = make_shared<Threat>(
				simParams.threat.threatRange,
				simParams.threat.destructionFormationFactor);
	}
	return pThreatSim;
}

shared_ptr<TargetSensor> SimulatorImpl::createTargetSensor(const SimulationParams& simParams) {
	shared_ptr<TargetSensor> pTargetSensor;
	if (simParams.optimalityTest) {
		pTargetSensor = make_shared<DeterministicTargetSensor>(
				simParams.downwardLookingSensor.targetSensorRange,
				simParams.downwardLookingSensor.targetDetectionFormationFactor);
	} else {
		pTargetSensor = make_shared<TargetSensor>(
				simParams.downwardLookingSensor.targetSensorRange,
				simParams.downwardLookingSensor.targetDetectionFormationFactor);
	}
	return pTargetSensor;
}

bool SimulatorImpl::finished() const {
	return destroyed || routeIt == route.end();
}

SimulationResults SimulatorImpl::getResults() {
	SimulationResults results;
	results.destroyed = destroyed;
	results.targetsDetected = targetsDetected;
	results.whereDestroyed = position;
	results.missionSuccess = !destroyed && targetsDetected >= MISSION_SUCCESS_THRESHOLD;
	results.decisionTimeAvg = boost::accumulators::mean(decisionTimeStats);
	results.decisionTimeVar = boost::accumulators::moment<2>(decisionTimeStats);
	return results;
}

TeamState SimulatorImpl::getState() {
	TeamState state;
	state.position = position;
	state.config = currentConfig;
	state.directionX = directionX;
	state.directionY = directionY;
	return state;
}

std::vector<bool> SimulatorImpl::readForwardSensor(const RealEnvironment& environment,
		Sensor* pSensor, unsigned cells) {
	std::vector<bool> sensed;

	// compute forward sensing route
	Route route(position, directionX, directionY, cells);

	for (const auto& pos : route) {
		if (pos.isInsideRect(environment.getSize())) {
			sensed.push_back(pSensor->sense(environment.isObjectAt(pos)));
		} else {
			break; // the route is a straight line and the environment is convex
		}
	}

	return sensed;
}


std::vector<bool> SimulatorImpl::readForwardThreatSensor(
		unsigned cells) {
	return readForwardSensor(threatEnv, pFwdThreatSensor.get(), cells);
}

std::vector<bool> SimulatorImpl::readForwardTargetSensor(
		unsigned cells) {
	return readForwardSensor(targetEnv, pFwdTargetSensor.get(), cells);
}

std::vector<std::vector<bool>> SimulatorImpl::readForwardSensor(const RealEnvironment& environment,
		Sensor* pSensor, unsigned cells, unsigned numOfObservations) {
	std::vector<std::vector<bool>> sensed;

	// compute forward sensing route
	Route route(position, directionX, directionY, cells);

	for (const auto& pos : route) {
		if (pos.isInsideRect(environment.getSize())) {
			std::vector<bool> values;
			for (unsigned c = 0; c < numOfObservations; c++) {
				values.push_back(pSensor->sense(environment.isObjectAt(pos)));
			}
			sensed.push_back(values);
		} else {
			break; // the route is a straight line and the environment is convex
		}
	}

	return sensed;
}


std::vector<std::vector<bool>> SimulatorImpl::readForwardThreatSensor(unsigned cells, unsigned numOfObservations) {
	return readForwardSensor(threatEnv, pFwdThreatSensor.get(), cells, numOfObservations);
}

std::vector<std::vector<bool>> SimulatorImpl::readForwardTargetSensor(unsigned cells, unsigned numOfObservations) {
	return readForwardSensor(targetEnv, pFwdTargetSensor.get(), cells, numOfObservations);
}

void SimulatorImpl::updateDirection() {
	directionX = 0;
	directionY = 0;
	if (routeIt != route.end()) {
		auto nextPos = routeIt + 1;
		directionX = nextPos->x - position.x;
		directionY = nextPos->y - position.y;
	}
}

bool SimulatorImpl::step(const TacticList& tactics, double decisionTimeMsec) {
	bool targetDetectedInThisStep = false;
	if (finished()) {
		return targetDetectedInThisStep;
	}

	// collect decision time
	decisionTimeStats(decisionTimeMsec);

	for (auto tactic : tactics) {
		currentConfig = executeTactic(tactic, currentConfig);
	}

	configInLastStep = currentConfig;

	/* update display */
	screen[screenPosition][currentConfig.altitudeLevel - 1] =
			(currentConfig.formation
					== TeamConfiguration::Formation::LOOSE) ?
					(currentConfig.ecm ? '@' : '#') :
					(currentConfig.ecm ? '0' : '*');

	/* simulate threats */
	threatInLastStep = threatEnv.isObjectAt(position);
	destroyed = pThreatSim->isDestroyed(threatEnv, currentConfig, position);
	if (destroyed) {
		cout << "Team destroyed at position " << position << endl;
		return targetDetectedInThisStep;
	}

	/* simulate target detection */
	if (pTargetSensor->sense(currentConfig, targetEnv.isObjectAt(position))) {
		cout << "Target detected at " << position << endl;
		targetsDetected++;
		targetDetectedInThisStep = true;
		screen[screenPosition][SCREEN_TARGETS] = 'X';
	}

	/* system evolution */
	routeIt++;
	if (routeIt != route.end()) {
		position = *routeIt;
	}
	updateDirection();
	screenPosition++;

	/* update tactic progress */
	auto ttcIncAlt = currentConfig.ttcIncAlt;
	if (ttcIncAlt > 0) {
		currentConfig.ttcIncAlt = --ttcIncAlt;
		if (ttcIncAlt == 0) {
			currentConfig.altitudeLevel = currentConfig.altitudeLevel + 1;
		}
	}

	auto ttcDecAlt = currentConfig.ttcDecAlt;
	if (ttcDecAlt > 0) {
		currentConfig.ttcDecAlt = --ttcDecAlt;
		if (ttcDecAlt == 0) {
			currentConfig.altitudeLevel = currentConfig.altitudeLevel - 1;
		}
	}

	auto ttcIncAlt2 = currentConfig.ttcIncAlt2;
	if (ttcIncAlt2 > 0) {
		currentConfig.ttcIncAlt2 = --ttcIncAlt2;
		if (ttcIncAlt2 == 0) {
			currentConfig.altitudeLevel = currentConfig.altitudeLevel + 2;
		}
	}

	auto ttcDecAlt2 = currentConfig.ttcDecAlt2;
	if (ttcDecAlt2 > 0) {
		currentConfig.ttcDecAlt2 = --ttcDecAlt2;
		if (ttcDecAlt2 == 0) {
			currentConfig.altitudeLevel = currentConfig.altitudeLevel - 2;
		}
	}

	return targetDetectedInThisStep;
}

TeamConfiguration SimulatorImpl::executeTactic(string tactic, const TeamConfiguration& config) {
	auto newConfig = config;
	cout << "executing tactic " << tactic << endl;
	if (tactic == INC_ALTITUDE) {
		if (changeAltitudeLatencyPeriods > 0) {
			newConfig.ttcIncAlt = changeAltitudeLatencyPeriods;
		} else {
			newConfig.altitudeLevel = newConfig.altitudeLevel + 1;
		}
	} else if (tactic == DEC_ALTITUDE) {
		if (changeAltitudeLatencyPeriods > 0) {
			newConfig.ttcDecAlt = changeAltitudeLatencyPeriods;
		} else {
			newConfig.altitudeLevel = newConfig.altitudeLevel - 1;
		}
	} else if (tactic == INC_ALTITUDE2) {
		if (changeAltitudeLatencyPeriods > 0) {
			newConfig.ttcIncAlt2 = changeAltitudeLatencyPeriods;
		} else {
			newConfig.altitudeLevel = newConfig.altitudeLevel + 2;
		}
	} else if (tactic == DEC_ALTITUDE2) {
		if (changeAltitudeLatencyPeriods > 0) {
			newConfig.ttcDecAlt2 = changeAltitudeLatencyPeriods;
		} else {
			newConfig.altitudeLevel = newConfig.altitudeLevel - 2;
		}
	} else if (tactic == GO_TIGHT) {
		newConfig.formation = TeamConfiguration::Formation::TIGHT;
	} else if (tactic == GO_LOOSE) {
		newConfig.formation = TeamConfiguration::Formation::LOOSE;
	} else if (tactic == ECM_ON) {
		newConfig.ecm = true;
	} else if (tactic == ECM_OFF) {
		newConfig.ecm = false;
	} else {
		throw std::runtime_error(string("unknown tactic ") + tactic);
	}
	return newConfig;
}

bool SimulatorImpl::wasThereAThreat(TeamConfiguration* pTeamConfig) const {
	if (pTeamConfig) {
		*pTeamConfig = configInLastStep;
	}
	return threatInLastStep;
}

std::string SimulatorImpl::getScreenOutput() {
	ostringstream out;
	for (int h = SCREEN_THREATS; h > 0 ; h--) {
		for (unsigned p = 0; p < screen.size(); p++) {
			out << screen[p][h - 1];
		}
		out << endl;
	}
	for (int h = SCREEN_THREATS; h < SCREEN_THREATS + 2; h++) {
		for (unsigned p = 0; p < screen.size(); p++) {
			out << screen[p][h];
		}
		out << endl;
	}
	return out.str();
}

SimulatorImpl::~SimulatorImpl() {
}

} /* namespace sim */
} /* namespace dart */

