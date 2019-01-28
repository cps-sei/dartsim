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
import java.util.ArrayList;
import java.io.IOException;


public class SimpleAdaptationManager {

	/**
	 * @param args
	 * @throws IOException 
	 */
	public static void main(String[] args) throws IOException {
		String host = "localhost";
		int port = 5418;
		
        if (args.length > 2) {
        	System.out.println("error: invalid options");
        	System.out.println("valid options: [host [port]]");
        	return;
        } else if (args.length > 0) {
        	host = args[0];
        	if (args.length > 1) {
            	port = Integer.parseInt(args[1]);
        	}
        }

        DartSimClient dartsim = new DartSimClient(host, port);
    
        simpleAdaptationManager(dartsim);
	}
	
	static void simpleAdaptationManager(DartSimClient dartsim) {
		dartsim.connect();
		DartSimClient.SimulationParams simParams = dartsim.getParameters();

		int horizon = 5;
		int minAltitude = 1;
		int maxAltitude = simParams.altitudeLevels;

		while(dartsim.isConnected() && !dartsim.finished()) {
			long start = System.currentTimeMillis();
			DartSimClient.TeamState state = dartsim.getState();
			ArrayList<Boolean> threats = dartsim.readForwardThreatSensor(horizon);
			ArrayList<Boolean> targets = dartsim.readForwardTargetSensor(horizon);

			ArrayList<String> tacticList = new ArrayList<String>();
			boolean threatAhead = threats.contains(true);
			
			if (threatAhead && state.config.altitudeLevel < maxAltitude) {
				tacticList.add(dartsim.INC_ALTITUDE);
			} else {
				boolean targetAhead = targets.contains(true);
				if (targetAhead && state.config.altitudeLevel > minAltitude) {
					tacticList.add(dartsim.DEC_ALTITUDE);
				}
			}

			if (threats.get(0) == true) { // is there an immediate threat?
				if (state.config.formation == DartSimClient.TeamConfiguration.Formation.TIGHT) {
					tacticList.add(dartsim.GO_TIGHT);
				}
			} else if (state.config.formation == DartSimClient.TeamConfiguration.Formation.LOOSE) {
				tacticList.add(dartsim.GO_LOOSE);
			}

			double decisionTimeMsec = System.currentTimeMillis() - start;
			dartsim.step(tacticList, decisionTimeMsec);
		}

		String output = dartsim.getScreenOutput();
		System.out.println(output);

		DartSimClient.SimulationResults simResults = dartsim.getResults();
		System.out.println("Destroyed = " + simResults.destroyed);
		System.out.println("Targets detected = " + simResults.targetsDetected);
		System.out.println("Mission success = " + simResults.missionSuccess);
		System.out.println("Decision time agv = " + simResults.decisionTimeAvg
				+ "  var = " + simResults.decisionTimeVar);
	}
}
