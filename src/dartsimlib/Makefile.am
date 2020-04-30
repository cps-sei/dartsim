lib_LIBRARIES = libdartsim.a

AM_CPPFLAGS = -std=c++14 -I$(top_srcdir)/include -O3 -Wall -g
ARFLAGS = cr

libdartsim_a_SOURCES = RealEnvironment.cpp RealEnvironmentSparse.cpp TargetSensor.cpp \
	DeterministicTargetSensor.cpp Route.cpp \
	DeterministicThreat.cpp Sensor.cpp Threat.cpp \
	RandomSeed.cpp Simulator.cpp SimulatorImpl.cpp
