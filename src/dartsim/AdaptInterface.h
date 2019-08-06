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
#include <string>
#include <boost/asio.hpp>
#include <json11.hpp>
#include <vector>
#include <memory>
#include <string>


namespace dart {
namespace sim {

class AdaptInterface {
private:
	dart::sim::Simulator* mSimulatorP;
	const unsigned mPort;
	boost::asio::io_service* mIOServiceP;
	boost::asio::ip::tcp::tcp::endpoint* mEndPointP;
	boost::asio::ip::tcp::tcp::acceptor* mAcceptorP;
	boost::asio::ip::tcp::tcp::socket* mSocketP;
	std::map<std::string, std::function<std::string(const std::vector<std::string>&)>> mCommandHandlers;

	std::shared_ptr<std::string> readCmd() const;
	void sendBytes(const std::string& bytes) const;
	json11::Json convertTeamStateToJson(const dart::sim::TeamState& state) const;
	json11::Json convertSimulationResultsToJson(const dart::sim::SimulationResults& simResults) const;
	json11::Json convertSimulationParamsToJson(const dart::sim::SimulationParams& simResults) const;

	std::string cmdFinished(const std::vector<std::string>& args);
	std::string cmdGetState(const std::vector<std::string>& args);
	std::string cmdReadForwardThreatSensor(const std::vector<std::string>& args);
	std::string cmdReadForwardTargetSensor(const std::vector<std::string>& args);
	std::string cmdReadForwardThreatSensorForObservations(const std::vector<std::string>& args);
	std::string cmdReadForwardTargetSensorForObservations(const std::vector<std::string>& args);
	std::string cmdStep(const std::vector<std::string>& args);
	std::string cmdGetResults(const std::vector<std::string>& args);
	std::string cmdGetScreenOutput(const std::vector<std::string>& args);
	std::string cmdGetParameters(const std::vector<std::string>& args);

public:
	AdaptInterface(dart::sim::Simulator* simulatorP, unsigned port = 5418);
	void connectToClient();
	void serviceClient();
	void handleClientCmd(const std::string& cmd);
	virtual ~AdaptInterface();
};

}
}
