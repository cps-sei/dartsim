bin_PROGRAMS = pla_dart_enf

DARTSIMLIB_PATH = $(top_srcdir)/../..

PLADAPT = $(top_srcdir)/../../../pladapt

PLADAPTLIB = $(PLADAPT)/build/src/libadaptmgr.a

AM_CPPFLAGS = -std=c++14 -I$(DARTSIMLIB_PATH)/include -I$(PLADAPT)/include -O3 -Wall -g -DSDPRA=0

pla_dart_enf_SOURCES = DartAdaptationManager.cpp DartConfiguration.cpp \
	DartConfigurationManager.cpp DartDTMCEnvironment.cpp DartEnvironment.cpp \
	DartSimpleEnvironment.cpp DartSurvivalUtilityFunction.cpp \
	EnvironmentMonitor.cpp pla-dart.cpp Parameters.cpp DartDetectionUtilityFunction.cpp
pla_dart_enf_LDADD = $(DARTSIMLIB_PATH)/build/src/dartsimlib/libdartsim.a $(PLADAPTLIB) -lboost_system \
	-lboost_filesystem -lboost_serialization -lyaml-cpp
