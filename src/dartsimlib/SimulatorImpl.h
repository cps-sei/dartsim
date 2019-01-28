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
#include <dartsim/Simulator.h>
#include "RealEnvironment.h"
#include "Sensor.h"
#include "Threat.h"
#include "TargetSensor.h"
#include <memory>
#include <vector>
#include <string>
#include <set>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>

using Stats = boost::accumulators::accumulator_set<double,
        boost::accumulators::stats<boost::accumulators::tag::mean,
                boost::accumulators::tag::moment<2> > >;

namespace dart {
namespace sim {

class SimulatorImpl : public Simulator {
	SimulationParams params;
	RealEnvironment threatEnv;
	RealEnvironment targetEnv;
	Route route;

	std::unique_ptr<Sensor> pFwdThreatSensor;
	std::unique_ptr<Sensor> pFwdTargetSensor;

	std::shared_ptr<TargetSensor> pTargetSensor;
	std::shared_ptr<Threat> pThreatSim;

	std::vector<std::vector<char> > screen;
	unsigned screenPosition = 0;
	Stats decisionTimeStats;
	TeamConfiguration currentConfig;
	unsigned targetsDetected = 0;
	bool destroyed = false;

	/**
	 * Iterator over route
	 *
	 * It is always one ahead of the position so that the
	 * direction of the team can be computed relative
	 * to where the team is going next
	 */
	Route::const_iterator routeIt;

	unsigned changeAltitudeLatencyPeriods;

	// TODO this may be removed and use only routeIt instead
	Coordinate position; /**< current team position */


	const unsigned MISSION_SUCCESS_THRESHOLD;

	const int SCREEN_THREATS;
	const int SCREEN_TARGETS;

	int directionX = 0; /**< -1, 0 or +1 to indicate the horizontal direction of travel */
	int directionY = 0; /**< -1, 0 or +1 to indicate the vertical direction of travel */

public:
	typedef std::set<std::string> TacticList; /**< a set of tactic labels */

	SimulatorImpl(const SimulationParams& simParams,
			const RealEnvironment& threatEnv, const RealEnvironment& targetEnv,
			const Route& route, unsigned missionSuccessTargetThreshold);

	SimulationParams getParameters() const;

	bool finished() const;

	TeamState getState();

	/**
	 * Read the forward-looking threat sensor
	 *
	 * The sensor can only look forward, so, even if the team is about to
	 * turn a corner, the sensing is done straight ahead.
	 * For the cells that are outside of the map, it returns false;
	 *
	 * @return vector of booleans of size cells indicating whether a threat was sensed
	 * 	in each cell in the straight forward path
	 */
	std::vector<bool> readForwardThreatSensor(unsigned cells);
	std::vector<bool> readForwardTargetSensor(unsigned cells);

	std::vector<std::vector<bool> > readForwardThreatSensor(unsigned cells, unsigned numOfObservations);
	std::vector<std::vector<bool> > readForwardTargetSensor(unsigned cells, unsigned numOfObservations);


	/**
	 * Executes one simulation step
	 *
	 * @return true if target was detected
	 */
	bool step(const TacticList& tactics, double decisionTimeMsec = 0.0);

	SimulationResults getResults();

	/**
	 * Get text rendering of screen output
	 *
	 * @return string with screen output
	 */
	std::string getScreenOutput();

	virtual ~SimulatorImpl();

private:
	std::vector<bool> readForwardSensor(const RealEnvironment& environment,
			Sensor* pSensor,
			unsigned cells);

	std::vector<std::vector<bool> > readForwardSensor(const RealEnvironment& environment,
			Sensor* pSensor,
			unsigned cells, unsigned numOfObservations);

	static std::shared_ptr<Threat> createThreatSim(const SimulationParams& simParams);
	static std::shared_ptr<TargetSensor> createTargetSensor(const SimulationParams& simParams);
	TeamConfiguration executeTactic(std::string tactic, const TeamConfiguration& config);
	void updateDirection();
};

} /* namespace sim */
} /* namespace dart */
