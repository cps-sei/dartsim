#!/bin/sh
PWD=`pwd`
BASEDIR=`dirname $0`
cd $BASEDIR 
CP="bin:$(for i in lib/*.jar ; do printf $i: ; done)" 
java -cp $CP SimpleAdaptationManager $*
cd $PWD
