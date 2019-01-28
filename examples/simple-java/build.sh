#!/bin/sh
if command -v curl; then
    GETCMD="curl -o"
else
    GETCMD="wget -nc -O"
fi 

GSONJAR=gson-2.8.2.jar
GSONURL=https://search.maven.org/remotecontent?filepath=com/google/code/gson/gson/2.8.2/gson-2.8.2.jar
JSONJAR=java-json.jar
JSONURL=http://central.maven.org/maven2/org/json/json/20180813/json-20180813.jar

BASEDIR=$(dirname "$0")
LIBDIR=${BASEDIR}/lib
mkdir -p ${LIBDIR}

GSONJARPATH=${LIBDIR}/${GSONJAR}
JSONJARPATH=${LIBDIR}/${JSONJAR}

if [ ! -f ${GSONJARPATH} ]; then
    $GETCMD ${GSONJARPATH} ${GSONURL}
fi

if [ ! -f ${JSONJARPATH} ]; then
    $GETCMD ${JSONJARPATH} ${JSONURL}
fi

ant -buildfile ${BASEDIR}/build.xml
