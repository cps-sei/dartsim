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

#include "DartPMCHelper.h"
#include "DartConfiguration.h"
#include <sstream>

using namespace std;

namespace dart {
namespace am2 {


DartPMCHelper::DartPMCHelper(const Params& params)
	: evaluationPeriod(1.0), // because latency is in periods
	  changeAltitudeLatency(params.simulationParams.changeAltitudeLatencyPeriods),
	  maxAltitudeLevel(params.simulationParams.altitudeLevels - 1),
	  destructionFormationFactor(params.simulationParams.threat.destructionFormationFactor),
	  threatRange(params.simulationParams.threat.threatRange),
	  detectionFormationFactor(params.simulationParams.downwardLookingSensor.targetDetectionFormationFactor),
	  sensorRange(params.simulationParams.downwardLookingSensor.targetSensorRange)
{
}


DartPMCHelper::~DartPMCHelper() {
}


std::string DartPMCHelper::generateInitializations(const pladapt::Configuration& currentConfigObj,
            const pladapt::UtilityFunction& utilityFunction, unsigned horizon) const {

    auto& config = dynamic_cast<const DartConfiguration&>(currentConfigObj);

	stringstream initialState;
	initialState << "const double PERIOD = " << evaluationPeriod << ';' << endl;
	initialState << "const int horizon = " << horizon << ';' << endl;
	initialState << "const double IncAlt_LATENCY = " << changeAltitudeLatency
			<< ';' << endl;
	initialState << "const double DecAlt_LATENCY = " << changeAltitudeLatency
			<< ';' << endl;
	initialState << "const int MAX_ALT_LEVEL = " << maxAltitudeLevel << ';'
			<< endl;
	initialState << "const double destructionFormationFactor = "
			<< destructionFormationFactor << ';' << endl;
	initialState << "const double threatRange = " << threatRange << ';' << endl;
	initialState << "const double detectionFormationFactor = "
			<< detectionFormationFactor << ';' << endl;
	initialState << "const double sensorRange = " << sensorRange << ';' << endl;
	initialState << "const init_a = " << config.getAltitudeLevel() << ';'
			<< endl;
	initialState << "const init_f = " << config.getFormation() << ';' << endl;
	initialState << "const int ini_IncAlt_state = " << config.getTtcIncAlt()
			<< ';' << endl;
	initialState << "const int ini_DecAlt_state = " << config.getTtcDecAlt()
			<< ';' << endl;

    return initialState.str();
}

} /* namespace am2 */
} /* namespace dart */
