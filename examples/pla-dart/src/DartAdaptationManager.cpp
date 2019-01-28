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

#include "DartAdaptationManager.h"
#include "DartConfigurationManager.h"
#include <pladapt/Utils.h>
#include <pladapt/SDPRAAdaptationManager.h>
#include <pladapt/PMCRAAdaptationManager.h>
#include "DartPMCHelper.h"
#include <math.h>

#if DART_USE_CE
#include <pladapt/CEAdaptationManager.h>
#endif

using namespace std;

namespace dart {
namespace am2 {

const string ADAPT_MGR_SDP = "sdp";
const string ADAPT_MGR_SDPRA = "sdpra";
const string ADAPT_MGR_PMC = "pmc";
#if DART_USE_CE
const string ADAPT_MGR_CE = "ce";
#endif

void DartAdaptationManager::instantiateAdaptationMgr(const Params& params) {
	cout << "Initializing adapt mgr...";

	auto changeAltitudePeriods = params.simulationParams.changeAltitudeLatencyPeriods;

	// initialize config manager
	configManager = std::make_shared<DartConfigurationManager>(
			params.simulationParams.altitudeLevels,
			params.simulationParams.changeAltitudeLatencyPeriods,
			params.configurationSpace.hasEcm, params.configurationSpace.twoLevelTactics);

	// instantiate and initialize appropriate adapt mgr
	if (params.adaptationManager.mgr == ADAPT_MGR_PMC) {
	    YAML::Node amParams;
	    amParams[pladapt::PMCAdaptationManager::NO_LATENCY] = (params.adaptationManager.nonLatencyAware || changeAltitudePeriods == 0);
	    amParams[pladapt::PMCAdaptationManager::TEMPLATE_PATH] = params.adaptationManager.prismTemplate;
	    amParams[pladapt::PMCRAAdaptationManager::PROBABILITY_BOUND] = params.adaptationManager.probabilityBound;

	    auto pAdaptMgr = new pladapt::PMCAdaptationManager;
		pAdaptMgr->initialize(configManager, amParams, std::make_shared<const DartPMCHelper>(params));
		adaptMgr.reset(pAdaptMgr);
	} else { // SDP or derived
	    YAML::Node amParams;
	    amParams[pladapt::SDPAdaptationManager::NO_LATENCY] = (params.adaptationManager.nonLatencyAware || changeAltitudePeriods == 0);
	    amParams[pladapt::SDPAdaptationManager::REACH_OPTIONS] = "-c ConfigDart2";
		amParams[pladapt::SDPAdaptationManager::REACH_PATH] = params.adaptationManager.reachPath;
		if (!params.adaptationManager.reachPrefix.empty()) {
			amParams[pladapt::SDPAdaptationManager::REACH_PREFIX] = params.adaptationManager.reachPrefix;
		}
	    if (params.adaptationManager.nonLatencyAware && changeAltitudePeriods > 0) {
	    	amParams[pladapt::SDPAdaptationManager::REACH_MODEL] = params.adaptationManager.reachModel + "-nla";
	    } else {
	    	amParams[pladapt::SDPAdaptationManager::REACH_MODEL] = params.adaptationManager.reachModel;
	    }

	    stringstream scope;
	    scope << "A=" << params.simulationParams.altitudeLevels;
	    scope << " F=2";
	    if (changeAltitudePeriods > 0) {
	        scope << " TPIA#=" <<  changeAltitudePeriods << " TPDA#=" <<  changeAltitudePeriods;
	        if (params.configurationSpace.twoLevelTactics) {
	        	scope << " TPIA2#=" <<  changeAltitudePeriods << " TPDA2#=" <<  changeAltitudePeriods;
	        }
	    }

	    amParams[pladapt::SDPAdaptationManager::REACH_SCOPE] = scope.str();

	    amParams[pladapt::SDPRAAdaptationManager::PROBABILITY_BOUND] = params.adaptationManager.probabilityBound;

#if DART_USE_CE
	    if (params.adaptationManager.mgr == ADAPT_MGR_CE) {
	    	amParams[pladapt::CEAdaptationManager::CE_INCREMENTAL] = params.adaptationManager.ce_incremental;
	    	amParams[pladapt::CEAdaptationManager::CE_HINT_WEIGHT] = params.adaptationManager.ce_hintWeight;
	    	amParams[pladapt::CEAdaptationManager::CE_SAMPLES] = params.adaptationManager.ce_samples;
			amParams[pladapt::CEAdaptationManager::CE_ALPHA] = params.adaptationManager.ce_alpha;
			amParams[pladapt::CEAdaptationManager::CE_PRECISION] = params.adaptationManager.ce_precision;
			amParams[pladapt::CEAdaptationManager::CE_MAX_ITERATIONS] = params.adaptationManager.ce_maxIterations;

			auto pAdaptMgr = new pladapt::CEAdaptationManager;
			pAdaptMgr->initialize(configManager, amParams);
			adaptMgr.reset(pAdaptMgr);
		} else
#endif
		if (params.adaptationManager.mgr == ADAPT_MGR_SDPRA) {
			auto pAdaptMgr = new pladapt::SDPRAAdaptationManager;
			pAdaptMgr->initialize(configManager, amParams);
			adaptMgr.reset(pAdaptMgr);
		} else if (params.adaptationManager.mgr == ADAPT_MGR_SDP) {
			auto pAdaptMgr = new pladapt::SDPAdaptationManager;
			pAdaptMgr->initialize(configManager, amParams);
			adaptMgr.reset(pAdaptMgr);
		} else {
			ostringstream msg;
			msg << "Error: adaptation manager ";
			msg << params.adaptationManager.mgr;
			msg << " not supported.";
			throw std::invalid_argument(msg.str());
		}
	}

	cout << "done" << endl;
}

void DartAdaptationManager::initialize(const Params& params, std::unique_ptr<pladapt::UtilityFunction> utilityFunction) {
	this->params = params;
	pEnvThreatMonitor.reset(
			new EnvironmentMonitor);
	pEnvTargetMonitor.reset(
			new EnvironmentMonitor);

	instantiateAdaptationMgr(params);

	pUtilityFunction = std::move(utilityFunction);
}

pladapt::TacticList DartAdaptationManager::decideAdaptation(
		const DartMonitoringInfo& monitoringInfo) {

	/* update environment */
	pEnvThreatMonitor->update(monitoringInfo.threatSensing);
	pEnvTargetMonitor->update(monitoringInfo.targetSensing);

	/* build env model with information collected so far */
	dart::sim::Route senseRoute(monitoringInfo.position, monitoringInfo.directionX, monitoringInfo.directionY, params.adaptationManager.horizon);
	DartDTMCEnvironment threatDTMC(*pEnvThreatMonitor, senseRoute, params.adaptationManager.distributionApproximation);
	DartDTMCEnvironment targetDTMC(*pEnvTargetMonitor, senseRoute, params.adaptationManager.distributionApproximation);
	pladapt::EnvironmentDTMCPartitioned jointEnv = pladapt::EnvironmentDTMCPartitioned::createJointDTMC(threatDTMC, targetDTMC);

	/* make adaptation decision */
	//adaptMgr->setDebug(monitoringInfo.position.x == 4);
	return adaptMgr->evaluate(convertToDiscreteConfiguration(monitoringInfo), jointEnv, *pUtilityFunction, params.adaptationManager.horizon);
}

DartConfiguration DartAdaptationManager::convertToDiscreteConfiguration(const DartMonitoringInfo& info) const {
	DartConfiguration::Formation formation =
			(info.formation == 0) ?
					DartConfiguration::Formation::LOOSE :
					DartConfiguration::Formation::TIGHT;

	return DartConfiguration(info.altitudeLevel, formation, info.ttcIncAlt,
			info.ttcDecAlt, info.ttcIncAlt2, info.ttcDecAlt2, info.ecm);
}

bool DartAdaptationManager::supportsStrategy() const {
	return adaptMgr->supportsStrategy();
}

std::shared_ptr<pladapt::Strategy> DartAdaptationManager::getStrategy() {
	return adaptMgr->getStrategy();
}


DartAdaptationManager::~DartAdaptationManager() {
}


} /* namespace am2 */
} /* namespace dart */
