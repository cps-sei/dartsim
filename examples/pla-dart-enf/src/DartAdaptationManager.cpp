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
#include <math.h>

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

	auto pAdaptMgr = new pladapt::SDPAdaptationManager;
	pAdaptMgr->initialize(configManager, amParams);
	missionAdaptMgr.reset(pAdaptMgr);

	auto pSurvivalAdaptMgr = new pladapt::SDPRAAdaptationManager;
	pSurvivalAdaptMgr->initialize(configManager, amParams);
	survivabilityAdaptMgr.reset(pSurvivalAdaptMgr);

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
//	missionAdaptMgr->setDebug(true);
	auto tactics = missionAdaptMgr->evaluate(convertToDiscreteConfiguration(monitoringInfo), jointEnv, *pUtilityFunction, params.adaptationManager.horizon);

	auto tacticsOut = survivabilityEnforcer(tactics, monitoringInfo, jointEnv);

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


pladapt::TacticList DartAdaptationManager::survivabilityEnforcer(const pladapt::TacticList& inputTactics,
		const DartMonitoringInfo& monitoringInfo,
		pladapt::EnvironmentDTMCPartitioned& jointEnv) {

#if 0
	cout << "survivabilityEnforcer([";
	for (const auto& t : inputTactics) {
		cout << ' ' << t;
	}
	cout << "])" << endl;
#endif
//	survivabilityAdaptMgr->setDebug(true);

	auto survTactics = survivabilityAdaptMgr->evaluate(convertToDiscreteConfiguration(monitoringInfo), jointEnv, *pSurvivalUtilityFunction, params.adaptationManager.horizon);
	auto survInfo = survivabilityAdaptMgr->getSurvivalInfo();

#if 0
	for (const auto& entry : survInfo) {
		cout << "surv [";
		for (auto& tactic : entry.tactics) {
			cout << ' ' << tactic;
		}
		cout << "]=" << entry.probability << endl;
	}
#endif

	for (const auto& entry : survInfo) {
		if (entry.tactics == inputTactics && entry.probability >= params.adaptationManager.probabilityBound) {
			return inputTactics; // OK pass through
		}
	}

	/*
	 * if nop tactic satisfies P, return that
	 * if there are actions with no Alt tactic that satisfies P, return one with highest prob
	 * from actions with Alt tactic that satisfies P, return one with highest prob
	 * else return action with highest prob
	 *
	 * TODO add preference for fewer tactics
	 */
	assert(survInfo.begin() != survInfo.end());
	const pladapt::SDPRAAdaptationManager::SurvivalInfo* pBest = &(*survInfo.begin());
	const pladapt::SDPRAAdaptationManager::SurvivalInfo* pBestWithAltTactic = nullptr;
	const pladapt::SDPRAAdaptationManager::SurvivalInfo* pBestWithoutAltTactic = nullptr;
	const pladapt::TacticList* pResult = nullptr;

	for (auto& entry : survInfo) {
		if (entry.probability >= params.adaptationManager.probabilityBound) {
			if (entry.tactics.empty()) {
				pResult = &entry.tactics;
			}

			bool hasAltTactics = false;
			for (auto& tactic : entry.tactics) {
				if (tactic.find("Alt") != std::string::npos) {
					hasAltTactics = true;
					break;
				}
			}

			if (hasAltTactics) {
				if (pBestWithAltTactic == nullptr || entry.probability > pBestWithAltTactic->probability) {
					pBestWithAltTactic = &entry;
				}
			} else {
				if (pBestWithoutAltTactic == nullptr || entry.probability > pBestWithoutAltTactic->probability) {
					pBestWithoutAltTactic = &entry;
				}
			}
		}

		if (entry.probability > pBest->probability) {
			pBest = &entry;
		}
	}

	if (pResult == nullptr) {
		if (pBestWithoutAltTactic) {
			pResult = &pBestWithoutAltTactic->tactics;
		} else if (pBestWithAltTactic) {
			pResult = &pBestWithAltTactic->tactics;
		} else {
			pResult = &pBest->tactics;
		}
	}

	cout << "survivabilityEnforcer()=";
	for (const auto& t : *pResult) {
		cout << ' ' << t;
	}
	cout << endl;

	return *pResult;
}


DartAdaptationManager::~DartAdaptationManager() {
}


} /* namespace am2 */
} /* namespace dart */
