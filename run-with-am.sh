#!/bin/sh

if [ $# -lt 1 ] || [ ! -f $1 ]; then
	echo "usage: $0 adaptmgrpath [simoptions [-- adaptmgroptions]]"
	echo example:
	echo "  $0 examples/simple-java/run.sh --seed=1 -- localhost"
	exit 1
fi

ADAPTMGR=$1
shift

SIMOPTS=
ADAPTMGROPTS=
while [ "$#" -gt 0 ]; do

    if [ "$1" = "--" ]; then
	shift
	ADAPTMGROPTS=$@
	break
    else
	SIMOPTS="$SIMOPTS $1"
	shift
    fi
done

echo "simulator options: $SIMOPTS"
echo "adaptation manager options: $ADAPTMGROPTS"

BASEDIR=`dirname $0`
{ $BASEDIR/run.sh $SIMOPTS ; }&
SIMPID=$!; 

trap 'kill $SIMPID;' INT

# give time to the simulator to start
sleep 2

# start the adaptation manager
echo "Starting adaptation manager..."
$ADAPTMGR $ADAPTMGROPTS

# wait for the simulator to end
echo "Waiting for simulator to end..."
wait $SIMPID

echo "done!"
