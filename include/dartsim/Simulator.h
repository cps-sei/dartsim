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

#include <dartsim/Route.h>
#include <dartsim/TeamConfiguration.h>
#include <memory>
#include <vector>
#include <string>
#include <set>

namespace dart {
namespace sim {

/**
 * Parameters for the long-range forward-looking sensors
 *
 * False positive and false negative rates for the sensors that are
 * used to monitor the environment in front of the team
 */
struct LongRangeSensorParams {
	double threatSensorFPR = 0.10;
	double threatSensorFNR = 0.15;
	double targetSensorFPR = 0.10;
	double targetSensorFNR = 0.15;
};


/**
 * Parameters for the downward-looking sensor used for target detection
 */
struct DownwardLookingSensorParams {

	/**
	 * When the team flies in tight formation, the probability of detecting
	 * a target is reduced. This factor specifies how much the detection
	 * probability is reduced when the team flies in tight formation.
	 * For example, if p is the detection probability in loose formation,
	 * the detection probability in tight formation is
	 * p / targetDetectionFormationFactor
	 */
	double targetDetectionFormationFactor = 1.2;

	/**
	 * The detection range for the sensor specified in altitude levels.
	 *
	 * The detection probability is inversely proportional to the altitude
	 * level at which the team is flying. When the team is flying at a level
	 * equal or greater than the range, the probability of detection is 0.
	 */
	unsigned targetSensorRange = 4;
};

/**
 * Parameters for the threats
 */
struct ThreatParams {

	/**
	 * When the team flies in tight formation, the probability of being hit by
	 * a threat is reduced. This factor specifies how much the destruction
	 * probability is reduced when the team flies in tight formation.
	 * For example, if p is the destruction probability in loose formation,
	 * the descruction probability in tight formation is
	 * p / destructionFormationFactor
	 */
	double destructionFormationFactor = 1.5;

	/**
	 * The range for a threat specified in altitude levels.
	 *
	 * The destruction probability is inversely proportional to the altitude
	 * level at which the team is flying. When the team is flying at a level
	 * equal or greater than the range, the probability of destruction is 0.
	 */
	unsigned threatRange = 3; // in altitude levels
};


/**
 * These parameters are known, in the sense that the adaptation manager
 * can use them to make decisions
 */
struct SimulationParams {

	/**
	 * This is the size of the map in route segments.
	 *
	 * If squareMap == false, this is the length of a straight route.
	 * If squareMap == true, this is the length of each side of the square,
	 * with the route being a lawn-mower pattern that visits every cell in
	 * the map.
	 */
	unsigned mapSize = 40;

	/**
	 * Whether the map is square.
	 *
	 * If the map is not square, it is simple a sequence of route segments in
	 * a straight line.
	 */
	bool squareMap = false;

	/**
	 * This is the number of altitude levels. The team's altitude levels are
	 * in the integer range [0,...,altitudeLevels -1].
	 */
	unsigned altitudeLevels = 4;

	/**
	 * Latency of the tactic to change altitude
	 *
	 * This is specified in periods (aka simulation steps)
	 */
	unsigned changeAltitudeLatencyPeriods = 1;

	/**
	 * When this is true, the sensors behave deterministically, in the sense that
	 * if a target is in range it is detected. The same with threats.
	 */
	bool optimalityTest = false;

	/**
	 * Parameters for the long-range forward-looking sensors
	 */
	LongRangeSensorParams longRangeSensor;

	/**
	 * Parameters for the downward-looking target detection sensor
	 */
	DownwardLookingSensorParams downwardLookingSensor;

	/**
	 * Parameters of threats
	 */
	ThreatParams threat;
};

/**
 * Simulation results
 *
 * Simulation results at the end of the simulation.
 */
struct SimulationResults {

	/**
	 * True if the team was destroyed
	 */
	bool destroyed;

	/**
	 * If the team was destroyed, where it that happened.
	 */
	Coordinate whereDestroyed;

	/**
	 * Number of targets detected
	 */
	unsigned targetsDetected;

	/**
	 * True if mission was completed successfully
	 *
	 * Success requires surviving the mission and detecting at least
	 * half of the targets present in the environment.
	 */
	bool missionSuccess;

	/**
	 * Average decision time (if reported)
	 */
	double decisionTimeAvg;

	/**
	 * Variance of decision time (if reported)
	 */
	double decisionTimeVar;
};


/**
 * Team state
 */
struct TeamState {

	/**
	 * Team position in the map
	 */
	Coordinate position;

	/**
	 * Horizontal direction of the team
	 *
	 * This is also the horizontal direction of the forward-looking sensors
	 *
	 * -1, 0 or +1 to indicate the horizontal direction of travel
	 */
	int directionX;

	/**
	 * Vertical direction of the team
	 *
	 * This is also the vertical direction of the forward-looking sensors
	 *
	 * -1, 0 or +1 to indicate the vertical direction of travel
	 */
	int directionY;

	/**
	 * Configuration of the team
	 */
	TeamConfiguration config;
};

/**
 * Main simulator class
 *
 * This is the interface that adaptation managers use to monitor the state of
 * the team and the environment, and to execute adaptation tactics
 */
class Simulator {

public:
	typedef std::set<std::string> TacticList; /**< a set of tactic labels */

	static const std::string INC_ALTITUDE;
	static const std::string DEC_ALTITUDE;
	static const std::string INC_ALTITUDE2;
	static const std::string DEC_ALTITUDE2;
	static const std::string GO_TIGHT;
	static const std::string GO_LOOSE;
	static const std::string ECM_ON;
	static const std::string ECM_OFF;

	/**
	 * Create an instance of the simulator.
	 *
	 * Creates an instance of the simulator parameterized with arguments that
	 * can be passed directly from the command line arguments received by
	 * main(argc, argv). If main() can also receive command line arguments
	 * that are not options for the simulator, such as arguments for the
	 * adaptation manager, those arguments have to be removed from the
	 * arguments passed to this method first. The pla-dart example shows
	 * how this can be done.
	 *
	 * @param argc number of arguments counting argv[0], which is the name
	 * 	of the executable.
	 * @param argv null-terminated array of arguments.
	 * @return pointer to simulator instance or nullptr if there was a problem
	 * 	instantiating the simulator (the method usage() can be used to print
	 * 	help about the supported arguments.
	 */
	static Simulator* createInstance(int argc, char** argv);

	/**
	 * Print help about the supported arguments for the simulator.
	 */
	static void usage();

	/**
	 * Return parameters of the simulator
	 *
	 * @return simulator parameters
	 */
	virtual SimulationParams getParameters() const = 0;

	/**
	 * Check if the simulation has finished.
	 *
	 * The simulation finishes either when the team gets to the end of the
	 * route, or when it is destroyed.
	 *
	 * @return true if simulation has finished
	 */
	virtual bool finished() const = 0;

	/**
	 * Return state of the team.
	 *
	 * @return state of the team
	 */
	virtual TeamState getState() = 0;

	/**
	 * Read the forward-looking threat sensor
	 *
	 * The sensor can only look forward, so, even if the team is about to
	 * turn a corner, the sensing is done straight ahead.
	 * For the cells that are outside of the map, it returns false.
	 *
	 * @param number of cells to sense
	 * @return vector of booleans of size cells indicating whether a threat was sensed
	 * 	in each cell in the straight forward path
	 */
	virtual std::vector<bool> readForwardThreatSensor(unsigned cells) = 0;

	/**
	 * Read the forward-looking target sensor
	 *
	 * The sensor can only look forward, so, even if the team is about to
	 * turn a corner, the sensing is done straight ahead.
	 * For the cells that are outside of the map, it returns false.
	 *
	 * @param number of cells to sense
	 * @return vector of booleans of size cells indicating whether a target was sensed
	 * 	in each cell in the straight forward path
	 */
	virtual std::vector<bool> readForwardTargetSensor(unsigned cells) = 0;

	/**
	 * Read several observations with the forward-looking threat sensor
	 *
	 * This method is for convenience, and required to implement adaptation
	 * managers compatible with the original version of DARTSim. It gets
	 * multiple observations of each cell instead of just one.
	 *
	 * The sensor can only look forward, so, even if the team is about to
	 * turn a corner, the sensing is done straight ahead.
	 * For the cells that are outside of the map, it returns false.
	 *
	 * @param number of cells to sense
	 * @param numOfObservations number of observations to take for each cell
	 * @return vector of vectors of booleans. The inner vector has
	 * 	numObservations of the same cell. The outer vector has one entry for
	 * 	each cell. The size of the outer vector could be less than cells if
	 * 	the number of cells in the straight path ahead of the team that are
	 *  inside of the map is less than the number of requested cells.
	 */
	virtual std::vector<std::vector<bool> > readForwardThreatSensor(unsigned cells, unsigned numOfObservations) = 0;

	/**
	 * Read several observations with the forward-looking target sensor
	 *
	 * This method is for convenience, and required to implement adaptation
	 * managers compatible with the original version of DARTSim. It gets
	 * multiple observations of each cell instead of just one.
	 *
	 * The sensor can only look forward, so, even if the team is about to
	 * turn a corner, the sensing is done straight ahead.
	 * For the cells that are outside of the map, it returns false.
	 *
	 * @param number of cells to sense
	 * @param numOfObservations number of observations to take for each cell
	 * @return vector of vectors of booleans. The inner vector has
	 * 	numObservations of the same cell. The outer vector has one entry for
	 * 	each cell. The size of the outer vector could be less than cells if
	 * 	the number of cells in the straight path ahead of the team that are
	 *  inside of the map is less than the number of requested cells.
	 */
	virtual std::vector<std::vector<bool> > readForwardTargetSensor(unsigned cells, unsigned numOfObservations) = 0;


	/**
	 * Executes one simulation step
	 *
	 * @param tactics tactics to execute (can be empty)
	 * @param decisionTimeMsec the amount of time in milliseconds that the
	 * 	adaptation manager took to make the adaptation decision
	 * @return true if a target was detected with the downward-looking sensor
	 */
	virtual bool step(const TacticList& tactics, double decisionTimeMsec = 0.0) = 0;


	/**
	 * Query if there was a threat in segment traversed in the previous step.
	 *
	 * @param pTeamConfig if not null, is filled with the team configuration
	 * 	(a subset of the team state) in the previous step.
	 *
	 * @return true if there was a threat in the segment of the last step.
	 */
	virtual bool wasThereAThreat(TeamConfiguration* pTeamConfig) const = 0;

	/**
	 * Get results of the simulation
	 *
	 * @return simulation results
	 */
	virtual SimulationResults getResults() = 0;

	/**
	 * Get text rendering of screen output
	 *
	 * @return string with multi-line screen output
	 */
	virtual std::string getScreenOutput() = 0;

	virtual ~Simulator();
};

} /* namespace sim */
} /* namespace dart */
