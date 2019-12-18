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
#include <cmath>
#include <algorithm>

#if DART_USE_CE
#include <pladapt/CEAdaptationManager.h>
#endif

using namespace std;

namespace dart {
namespace am2 {

const string ADAPT_MGR_SDP = "sdp";
const string ADAPT_MGR_SDPRA = "sdpra";

void DartAdaptationManager::instantiateAdaptationMgr(const Params& params) {
	cout << "Initializing adapt mgr...";

	auto changeAltitudePeriods = params.simulationParams.changeAltitudeLatencyPeriods;

	// initialize config manager
	configManager = std::make_shared<DartConfigurationManager>(
			params.simulationParams.altitudeLevels,
			params.simulationParams.changeAltitudeLatencyPeriods,
			params.configurationSpace.hasEcm, params.configurationSpace.twoLevelTactics);

	YAML::Node amParams;
	amParams[pladapt::SDPAdaptationManager::NO_LATENCY] = (params.adaptationManager.nonLatencyAware || changeAltitudePeriods == 0);
	amParams[pladapt::SDPAdaptationManager::REACH_OPTIONS] = "-c ConfigDart2";
	amParams[pladapt::SDPAdaptationManager::REACH_PATH] = params.adaptationManager.reachPath;
	amParams[pladapt::SDPAdaptationManager::IMPROVEMENT_THRESHOLD] = params.adaptationManager.improvementThreshold;
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

	auto pSurvivalAdaptMgr = new pladapt::SDPAdaptationManager;
	pSurvivalAdaptMgr->initialize(configManager, amParams);
	survivabilityAdaptMgr.reset(pSurvivalAdaptMgr);

	auto pAdaptMgr = new pladapt::SDPAdaptationManager;
	//amParams[pladapt::SDPAdaptationManager::REACH_MODEL] = params.adaptationManager.reachModel + "mission";
	pAdaptMgr->initialize(configManager, amParams);
	missionAdaptMgr.reset(pAdaptMgr);

	cout << "done" << endl;
}

void DartAdaptationManager::initialize(const Params& params, std::unique_ptr<pladapt::UtilityFunction> missionUtilityFunction,
		std::unique_ptr<pladapt::UtilityFunction> survivalUtilityFunction) {
	this->params = params;
	pEnvThreatMonitor.reset(
			new EnvironmentMonitor);
	pEnvTargetMonitor.reset(
			new EnvironmentMonitor);

	instantiateAdaptationMgr(params);

	pUtilityFunction = std::move(missionUtilityFunction);
	pSurvivalUtilityFunction = std::move(survivalUtilityFunction);
}

pladapt::TacticList DartAdaptationManager::decideAdaptation(
		const DartMonitoringInfo& monitoringInfo, unsigned L) {

	/* update environment */
	pEnvThreatMonitor->update(monitoringInfo.threatSensing);
	pEnvTargetMonitor->update(monitoringInfo.targetSensing);

	/* build env model with information collected so far */
	dart::sim::Route senseRoute(monitoringInfo.position, monitoringInfo.directionX, monitoringInfo.directionY, params.adaptationManager.horizon);
	DartDTMCEnvironment threatDTMC(*pEnvThreatMonitor, senseRoute, params.adaptationManager.distributionApproximation);
	DartDTMCEnvironment targetDTMC(*pEnvTargetMonitor, senseRoute, params.adaptationManager.distributionApproximation);
	pladapt::EnvironmentDTMCPartitioned jointEnv = pladapt::EnvironmentDTMCPartitioned::createJointDTMC(threatDTMC, targetDTMC);

	/* make adaptation decision */
//	missionAdaptMgr->setDebug(monitoringInfo.position.x == 9);
//	survivabilityAdaptMgr->setDebug(monitoringInfo.position.x == 9);

	pladapt::TacticList tactics = missionAdaptMgr->evaluate(convertToDiscreteConfiguration(monitoringInfo), jointEnv, *pUtilityFunction, params.adaptationManager.horizon);

	// enf P v2
	double requiredP = pow(params.adaptationManager.probabilityBound, 1.0 / 2.0);

	auto tacticsOut = survivabilityEnforcer(tactics, monitoringInfo, jointEnv, requiredP);

//	missionAdaptMgr->printPolicy(tacticsOut);

	return tacticsOut;
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
	return missionAdaptMgr->supportsStrategy();
}

std::shared_ptr<pladapt::Strategy> DartAdaptationManager::getStrategy() {
	return missionAdaptMgr->getStrategy();
}

#define SDPRA 0


using InfoType = pladapt::SDPAdaptationManager::ExpectedUtilityInfo;
#define PROBABILITY utility

pladapt::TacticList DartAdaptationManager::survivabilityEnforcer(const pladapt::TacticList& inputTactics,
		const DartMonitoringInfo& monitoringInfo,
		pladapt::EnvironmentDTMCPartitioned& jointEnv, double requiredP) {

#if 1
	cout << "survivabilityEnforcer([";
	for (const auto& t : inputTactics) {
		cout << ' ' << t;
	}
	cout << "]) >=" << requiredP << endl;
#endif
	//survivabilityAdaptMgr->setDebug(monitoringInfo.position.x == 4);
//	survivabilityAdaptMgr->setDebug(false);

//	auto survTactics = missionAdaptMgr->evaluate(convertToDiscreteConfiguration(monitoringInfo), jointEnv, *pSurvivalUtilityFunction, params.adaptationManager.horizon);
//	auto survInfo = missionAdaptMgr->getExpectedUtilityInfo();
	auto survTactics = survivabilityAdaptMgr->evaluate(convertToDiscreteConfiguration(monitoringInfo), jointEnv, *pSurvivalUtilityFunction, params.adaptationManager.horizon);
	auto survInfo = survivabilityAdaptMgr->getExpectedUtilityInfo();

	double expectedP = 0;

	for (const auto& entry : survInfo) {
		if (entry.tactics == inputTactics && entry.PROBABILITY >= requiredP) {
			expectedP = entry.PROBABILITY;
			decisionTimeStats(expectedP);
			return inputTactics; // OK pass through
		}
	}

	/*
	 * if nop tactic satisfies P, return that
	 * else, if theere are actions that satisfies P, return one with the lowest prob
	 * else return action with highest prob
	 */
	assert(survInfo.begin() != survInfo.end());
	const InfoType* pBest = &(*survInfo.begin());
	const InfoType* pBestPassing = nullptr;
	const pladapt::TacticList* pResult = nullptr;

	for (auto& entry : survInfo) {
		if (entry.PROBABILITY >= requiredP) {
			if (entry.tactics.empty()) {
				pResult = &entry.tactics;
				expectedP = entry.PROBABILITY;
			}

			if (pBestPassing == nullptr || entry.PROBABILITY < pBestPassing->PROBABILITY) {
				pBestPassing = &entry;
			}
		}

		if (entry.PROBABILITY > pBest->PROBABILITY) {
			pBest = &entry;
		}
	}

	if (pResult == nullptr) {
		if (pBestPassing) {
			pResult = &pBestPassing->tactics;
			expectedP = pBestPassing->PROBABILITY;
		} else {
			pResult = &pBest->tactics;
			expectedP = pBest->PROBABILITY;
		}
	}

	decisionTimeStats(expectedP);

	cout << "survivabilityEnforcer()=";
	for (const auto& t : *pResult) {
		cout << ' ' << t;
	}
	cout << endl;

//	cout << ">>> new strategy evaluation:" << endl;
//	origStrategy->front() = *pResult;
//	double val = missionAdaptMgr->evaluateStrategy(convertToDiscreteConfiguration(monitoringInfo),
//			jointEnv, *pSurvivalUtilityFunction, params.adaptationManager.horizon, origStrategy);
//	cout << "val = " << val << endl;
//	cout << "<<< new strategy evaluation:" << endl;

	return *pResult;
}


DartAdaptationManager::~DartAdaptationManager() {
}


} /* namespace am2 */
} /* namespace dart */
