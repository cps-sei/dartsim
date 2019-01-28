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
import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList; 
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import com.google.gson.Gson;
import com.google.gson.JsonSyntaxException;

public class DartSimClient {
	public String INC_ALTITUDE = "IncAlt";
	public String DEC_ALTITUDE = "DecAlt";
	public String INC_ALTITUDE2 = "IncAlt2";
	public String DEC_ALTITUDE2 = "DecAlt2";
	public String GO_TIGHT = "GoTight";
	public String GO_LOOSE = "GoLoose";
	public String ECM_ON = "EcmOn";
	public String ECM_OFF = "EcmOff";

private
	
	Socket mClientSocket = null;
	boolean mDebug = false;
	String mHost = "";
	int mPort = Integer.MAX_VALUE;
	
	public class LongRangeSensorParams {
		double threatSensorFPR = 0.10;
		double threatSensorFNR = 0.15;
		double targetSensorFPR = 0.10;
		double targetSensorFNR = 0.15;
	};

	public class DownwardLookingSensorParams {
		// this is the parameter for the short range sensor
		double targetDetectionFormationFactor = 1.2;
		int targetSensorRange = 4; // in altitude levels
	};


	public class ThreatParams {
		// this is the parameter for the threats
		double destructionFormationFactor = 1.5;
		int threatRange = 3; // in altitude levels
	};
	
	public class SimulationParams {
		int mapSize = 40;
		boolean squareMap = false;
		int altitudeLevels = 4;
		int changeAltitudeLatencyPeriods = 1;
		boolean optimalityTest = false;
		LongRangeSensorParams longRangeSensor;
		DownwardLookingSensorParams downwardLookingSensor;
		ThreatParams threat;
		
		public SimulationParams() {
			longRangeSensor = new LongRangeSensorParams();
			downwardLookingSensor = new DownwardLookingSensorParams();
			threat = new ThreatParams();
		}
	};

	public class Coordinate {
		int X;
		int Y;
		
		public Coordinate(int x, int y) {
	        X = x;
			Y = y;
		};
		public Coordinate() {
	        X = Integer.MAX_VALUE;
			Y = Integer.MAX_VALUE;
		};
	}

	public static class TeamConfiguration {
		public enum Formation {
			LOOSE,
			TIGHT 
		};

		int altitudeLevel;
		Formation formation;
		boolean ecm;
		int ttcIncAlt;
		int ttcDecAlt;
		int ttcIncAlt2;
		int ttcDecAlt2;
	}

	public class TeamState {
		Coordinate position; /**< current team position */
		int directionX;
		int directionY;
		TeamConfiguration config;
		
		public TeamState() {
			position = new Coordinate();
			config = new TeamConfiguration();
		}
	}

	public class SimulationResults {
		boolean destroyed;
		Coordinate whereDestroyed;
		int targetsDetected;
		boolean missionSuccess;
		double decisionTimeAvg;
		double decisionTimeVar;
		
		public SimulationResults() {
			whereDestroyed = new Coordinate();
		}
	}

	public DartSimClient(String host, int port) {
		mHost = host;
		mPort = port;
	}

	public void connect() {
		try {
			mClientSocket = new Socket(mHost, mPort);
		} catch (UnknownHostException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public boolean isConnected() {
		return mClientSocket.isConnected();
	}
	
	public void closeConnection() {
		try {
			mClientSocket.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public String sendCommand(String cmd) {
		if (!isConnected()) {
			connect();
		}
		
		DataOutputStream outToServer = null;
		try {
			outToServer = new DataOutputStream(mClientSocket.getOutputStream());
		} catch (IOException e) {
			e.printStackTrace();
		}

		try {
			if (mDebug) {
				System.out.println("Sending Command: " + cmd);
			}
			outToServer.writeBytes(cmd + '\n');
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		BufferedReader inFromServer = null;
		try {
			inFromServer = new BufferedReader(new InputStreamReader(mClientSocket.getInputStream()));
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		String jsonOutput = null;
		try {
			jsonOutput = inFromServer.readLine();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		if (mDebug) {
			System.out.println("FROM SERVER: " + jsonOutput);
		}
		
		return jsonOutput;
	}

	public boolean finished() {
		String result = sendCommand("finished");
		return Boolean.parseBoolean(result);
	}

	public TeamState getState() {
		String result = sendCommand("getState");
		JSONObject json = null;
		
		try {
			json = new JSONObject(result);
			
			if (mDebug) {
				System.out.println("Json parsed: " + json.getInt("positionX"));
			}
		} catch (JSONException e1) {
			e1.printStackTrace();
		}
		
		TeamState state = new TeamState();
		try {
			state.position.X = json.getInt("positionX");
		} catch (JSONException e1) {
			e1.printStackTrace();
		}
		try {
			state.position.Y = json.getInt("positionY");
		} catch (JSONException e1) {
			e1.printStackTrace();
		}
		
		try {
			state.directionX = json.getInt("directionX");
		} catch (JSONException e1) {
			e1.printStackTrace();
		}
		try {
			state.directionY = json.getInt("directionY");
		} catch (JSONException e1) {
			e1.printStackTrace();
		}

		try {
			state.config.altitudeLevel = json.getInt("altitudeLevel");
		} catch (JSONException e1) {
			e1.printStackTrace();
		}
		
		try {
			if (json.getInt("formation") == 0) {
				state.config.formation = TeamConfiguration.Formation.LOOSE;
			} else {
				state.config.formation = TeamConfiguration.Formation.TIGHT;
			}
		} catch (JSONException e) {
			e.printStackTrace();
		}
		
		try {
			state.config.ecm = json.getBoolean("ecm");
		} catch (JSONException e1) {
			e1.printStackTrace();
		}
		
		try {
			state.config.ttcIncAlt = json.getInt("ttcIncAlt");
		} catch (JSONException e) {
			e.printStackTrace();
		}
		try {
			state.config.ttcDecAlt = json.getInt("ttcDecAlt");
		} catch (JSONException e) {
			e.printStackTrace();
		}
		try {
			state.config.ttcIncAlt2 = json.getInt("ttcIncAlt2");
		} catch (JSONException e1) {
			e1.printStackTrace();
		}
		try {
			state.config.ttcDecAlt2 = json.getInt("ttcDecAlt2");
		} catch (JSONException e) {
			e.printStackTrace();
		}
		
		return state;
	}

	public ArrayList<Boolean> readForwardThreatSensor(int cells) {
		String cmd = "readForwardThreatSensor";
		String args = Integer.toString(cells);
		String result = sendCommand(cmd + " " + args);
		
		JSONArray arr = null;
		
		try {
			arr = new JSONArray(result);
		} catch (JSONException e) {
			e.printStackTrace();
		}

		ArrayList<Boolean> threats =  new ArrayList<Boolean>();

		try {
			int index = 0;

			for (; index < arr.length(); ++index) {
				threats.add(arr.getBoolean(index));
			}
		} catch (JSONException e) {
			e.printStackTrace();
		}

		return threats;
	}

	public ArrayList<Boolean> readForwardTargetSensor(int cells) {
		String cmd = "readForwardTargetSensor";
		String args = Integer.toString(cells);
		String result = sendCommand(cmd + " " + args);
		
		JSONArray arr = null;
		
		try {
			arr = new JSONArray(result);
		} catch (JSONException e) {
			e.printStackTrace();
		}

		ArrayList<Boolean> targets =  new ArrayList<Boolean>();

		try {
			int index = 0;

			for (; index < arr.length(); ++index) {
				targets.add(arr.getBoolean(index));
			}
		} catch (JSONException e) {
			e.printStackTrace();
		}

		return targets;
	}
	
	public ArrayList<ArrayList<Boolean>> readForwardThreatSensor(int cells, int numOfObservations) {
		String cmd = "readForwardThreatSensorForObservations";
		String args = Integer.toString(cells) + " " + Integer.toString(numOfObservations);
		String result = sendCommand(cmd + " " + args);

		JSONArray arr = null;
		
		try {
			arr = new JSONArray(result);
		} catch (JSONException e) {
			e.printStackTrace();
		}

		ArrayList<ArrayList<Boolean>> threats = new ArrayList<ArrayList<Boolean>>();

		try {
			int index1 = 0;

			for (; index1 < arr.length(); ++index1) {
				if (mDebug) {
					System.out.println("Json parsed: " + arr.getJSONArray(index1));
				}
				
				int index2 = 0;
				JSONArray jsonArr = arr.getJSONArray(index1);
				ArrayList<Boolean> readings = new ArrayList<Boolean>();
				for (; index2 < jsonArr.length(); ++index2) {
					readings.add(jsonArr.getBoolean(index2));
				}
				threats.add(readings);
			}
		} catch (JSONException e) {
			e.printStackTrace();
		}

		return threats;
	}
	
	public ArrayList<ArrayList<Boolean>> readForwardTargetSensor(int cells, int numOfObservations) {
		String cmd = "readForwardTargetSensorForObservations";
		String args = Integer.toString(cells) + " " + Integer.toString(numOfObservations);
		String result = sendCommand(cmd + " " + args);

		JSONArray arr = null;

		try {
			arr = new JSONArray(result);
		} catch (JSONException e) {
			e.printStackTrace();
		}

		ArrayList<ArrayList<Boolean>> targets = new ArrayList<ArrayList<Boolean>>();

		try {
			int index1 = 0;

			for (; index1 < arr.length(); ++index1) {
				if (this.mDebug) {
					System.out.println("Json parsed: " + arr.getJSONArray(index1));
				}

				int index2 = 0;
				JSONArray jsonArr = arr.getJSONArray(index1);
				ArrayList<Boolean> readings = new ArrayList<Boolean>();
				for (; index2 < jsonArr.length(); ++index2) {
					readings.add(jsonArr.getBoolean(index2));
				}
				targets.add(readings);
			}
		} catch (JSONException e) {
			e.printStackTrace();
		}

		return targets;
	}

	public boolean step(ArrayList<String> tacticList, double decisionTimeMsec) {
		String cmd = "step";
		String args = new Gson().toJson(tacticList);
		String result = sendCommand(cmd + " " + args + " " + decisionTimeMsec);
		
		return Boolean.parseBoolean(result);
	}

	public SimulationResults getResults() {
		String cmd = "getResults";
		String result = sendCommand(cmd);
		SimulationResults results = null;

		if (result != null) {
			JSONObject json = null;

			try {
				json = new JSONObject(result);
			} catch (JSONException e1) {
				e1.printStackTrace();
			}

			results = new SimulationResults();

			try {
				results.destroyed = json.getBoolean("destroyed");
			} catch (JSONException e) {
				e.printStackTrace();
			}

			try {
				results.whereDestroyed.X = json.getInt("destruction positionX");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			try {
				results.whereDestroyed.Y = json.getInt("destruction positionX");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			try {
				results.targetsDetected = json.getInt("targetsDetected");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			try {
				results.missionSuccess = json.getBoolean("missionSuccess");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			try {
				results.decisionTimeAvg = json.getDouble("decisionTimeAvg");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			try {
				results.decisionTimeVar = json.getDouble("decisionTimeVar");
			} catch (JSONException e) {
				e.printStackTrace();
			}
		}
		
		return results;
	}

	public String getScreenOutput() {
		String result = sendCommand("getScreenOutput");
		String output = "";

		try {
			output = new Gson().fromJson(result, String.class);
		} catch (JsonSyntaxException e1) {
			e1.printStackTrace();
		}
		
		return output;
	}
	
	public SimulationParams getParameters() {
		String cmd = "getParameters";
		String result = sendCommand(cmd);
		SimulationParams params = null;

		if (result != null) {
			JSONObject json = null;

			try {
				json = new JSONObject(result);
			} catch (JSONException e1) {
				e1.printStackTrace();
			}

			params = new SimulationParams();

			try {
				params.mapSize = json.getInt("mapSize");
			} catch (JSONException e) {
				e.printStackTrace();
			}

			try {
				params.squareMap = json.getBoolean("squareMap");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			
			try {
				params.altitudeLevels = json.getInt("altitudeLevels");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			
			try {
				params.changeAltitudeLatencyPeriods = json.getInt("changeAltitudeLatencyPeriods");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			
			try {
				params.optimalityTest = json.getBoolean("optimalityTest");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			
			try {
				params.longRangeSensor.targetSensorFNR = json.getDouble("targetSensorFNR");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			
			try {
				params.longRangeSensor.targetSensorFPR = json.getDouble("targetSensorFPR");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			
			try {
				params.longRangeSensor.threatSensorFNR = json.getDouble("threatSensorFNR");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			
			try {
				params.longRangeSensor.threatSensorFPR = json.getDouble("threatSensorFPR");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			
			try {
				params.downwardLookingSensor.targetDetectionFormationFactor = json.getDouble("targetDetectionFormationFactor");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			
			try {
				params.downwardLookingSensor.targetSensorRange = json.getInt("targetSensorRange");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			
			try {
				params.threat.destructionFormationFactor = json.getDouble("destructionFormationFactor");
			} catch (JSONException e) {
				e.printStackTrace();
			}
			
			try {
				params.threat.threatRange = json.getInt("threatRange");
			} catch (JSONException e) {
				e.printStackTrace();
			}
		}

		return params;
	}
}
