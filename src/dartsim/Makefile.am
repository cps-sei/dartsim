bin_PROGRAMS = dartsim
dartsim_SOURCES = dartsimmain.cpp AdaptInterface.cpp
dartsim_LDADD = ../dartsimlib/libdartsim.a ../../libraries/json11/libjson11.a -lboost_system
AM_CPPFLAGS = -std=c++14 -I$(top_srcdir)/include -I$(top_srcdir)/libraries/json11 -O3 -Wall -fmessage-length=0 -g
