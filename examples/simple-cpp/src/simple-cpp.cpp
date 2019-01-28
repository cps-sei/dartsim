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
#include <iostream>
#include <getopt.h>
#include <cstdlib>
#include <chrono>
#include <algorithm>
#include <cstring>

using namespace std;
using namespace dart::sim;

using myclock = chrono::high_resolution_clock;

enum ARGS {
	LOOKAHEAD_horizon
};

static struct option long_options[] = {
    {"lookahead-horizon",  required_argument, 0,  LOOKAHEAD_horizon },
    {0, 0, 0, 0 }
};

static void usage() {
	cout << "options: " << endl;
	cout << "\t[simulator options] [-- [adaptation manager options]]" << endl;
	Simulator::usage();
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
	int horizon = 5;

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
			break;
		}
	}

	int amArgc = argc - simArgc;
	if (amArgc) {
		argv[simArgc] = argv[0];
		char **amArgv = argv + simArgc;


		while (1) {
			int option_index = 0;

			auto c = getopt_long(amArgc, amArgv, "", long_options, &option_index);

			if (c == -1) {
				break;
			}

			switch (c) {
			case LOOKAHEAD_horizon:
				horizon = atoi(optarg);
				if (horizon < 1) {
					cout << "error: horizon must be >= 1" << endl;
					usage();
				}
				break;
			default:
				usage();
			}
		}

		if (optind < amArgc) {
			usage();
		}
	}

	optind = 1; // reset getopt scanning
	argv[simArgc] = nullptr;

	Simulator *dartsim = Simulator::createInstance(simArgc, argv);
	if (!dartsim) {
		usage();
	}

	auto simParams = dartsim->getParameters();
	const unsigned minAltitude = 1;
	const unsigned maxAltitude = simParams.altitudeLevels;

	while (!dartsim->finished()) {
		auto startTime = myclock::now();
		auto state = dartsim->getState();
		cout << "current position: " << state.position << endl;
		auto threats = dartsim->readForwardThreatSensor(horizon);
		auto targets = dartsim->readForwardTargetSensor(horizon);

		Simulator::TacticList tactics;
		bool threatAhead = any_of(threats.begin(), threats.end(), [](bool p){return p;});
		if (threatAhead && state.config.altitudeLevel < maxAltitude) {
			tactics.insert(Simulator::INC_ALTITUDE);
		} else {
			bool targetAhead = any_of(targets.begin(), targets.end(), [](bool p){return p;});
			if (targetAhead && state.config.altitudeLevel > minAltitude) {
				tactics.insert(Simulator::DEC_ALTITUDE);
			}
		}

		if (threats[0]) { // is there an immediate threat?
			if (state.config.formation != TeamConfiguration::Formation::TIGHT) {
				tactics.insert(Simulator::GO_TIGHT);
			}
		} else if (state.config.formation != TeamConfiguration::Formation::LOOSE) {
			tactics.insert(Simulator::GO_LOOSE);
		}

		auto delta = myclock::now() - startTime;
		double deltaMsec = chrono::duration_cast<chrono::duration<double, std::milli>>(delta).count();

		dartsim->step(tactics, deltaMsec);
	}

	auto results = dartsim->getResults();
	if (!results.destroyed) {
		cout << "Total targets detected: " << results.targetsDetected << endl;
	}

	cout << dartsim->getScreenOutput();

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

	delete dartsim;

	return 0;
}
