bin_PROGRAMS = pla_dart

DARTSIMLIB_PATH = $(top_srcdir)/../..

AM_CPPFLAGS = -std=c++14 -I$(DARTSIMLIB_PATH)/include -I$(PLADAPT)/include -O3 -Wall -g

pla_dart_SOURCES = DartAdaptationManager.cpp DartConfiguration.cpp \
	DartConfigurationManager.cpp DartDTMCEnvironment.cpp DartEnvironment.cpp \
	DartPMCHelper.cpp DartSimpleEnvironment.cpp DartUtilityFunction.cpp \
	EnvironmentMonitor.cpp pla-dart.cpp Parameters.cpp
pla_dart_LDADD = $(DARTSIMLIB_PATH)/build/src/dartsimlib/libdartsim.a $(PLADAPT)/build/src/libadaptmgr.a -lboost_system \
	-lboost_filesystem -lboost_serialization -lyaml-cpp
