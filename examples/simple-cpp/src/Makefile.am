bin_PROGRAMS = simple_cpp

DARTSIMLIB_PATH = $(top_srcdir)/../..

AM_CPPFLAGS = -std=c++14 -I$(DARTSIMLIB_PATH)/include -O3 -Wall -g

simple_cpp_SOURCES = simple-cpp.cpp
simple_cpp_LDADD = $(DARTSIMLIB_PATH)/build/src/dartsimlib/libdartsim.a \
	-lboost_system \
	-lboost_filesystem -lboost_serialization -lyaml-cpp
