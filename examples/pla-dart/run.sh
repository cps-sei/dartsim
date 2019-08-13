#!/bin/sh
PWD=`pwd`
cd `dirname $0`
BASEDIR=`pwd`

# try to guess PLADAPT if not set
if [ -z "$PLADAPT" ]; then
    GUESS=../../../pladapt
    if [ -d "$GUESS" ]; then
	cd $GUESS
	export PLADAPT=`pwd`
	cd $BASEDIR
	echo PLADAPT env var undefined. Set to: $PLADAPT
    else
	echo PLADAPT env var undefined. It must be set.
	exit 1
    fi
fi

# find location of executable. Either in . or build
EXECBIN=src/pla_dart
BUILD=.
if [ -f "build/$EXECBIN" ]; then
    BUILD=build
fi

if [ "`basename $0`" = "docker_run.sh" ]; then
    docker run --mount type=bind,src=$BASEDIR,target=$BASEDIR --mount type=bind,src=$PLADAPT,target=$PLADAPT pladevbase $BASEDIR/run.sh $*
else
    $BUILD/$EXECBIN $*
fi    
cd $PWD
