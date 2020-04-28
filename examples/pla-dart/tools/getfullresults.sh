#!/bin/bash
BASEDIR=`dirname $0`

LOGDIR="log"

if [ $# -ge 2 ]; then
    LOGDIR="$2"
fi

TRANSPOSE="awk -f $BASEDIR/transpose.awk -"

echo true_seed,targetsDetected,destroyed,wheredestroyed,lookahead.horizon,decisionTimeAvg,threatsSurvived
for s in $(eval echo "{$1}")
do
    logfile="$LOGDIR/exp-s${s}.log"
    # csv,detected,destroyed,wheredestroyed,mission_success,decision_time_avg,decision_time_var
    #   0,       1,        2,             3,              4,                5,                6
    csv=`cat $logfile | grep csv`
    vals=( `echo -n $csv | tr "," " "` )
    horizon=`cat $logfile | grep "lookahead horizon=" | cut -d= -f2`
    destroyed=${vals[2]}
    wheredestroyed=${vals[3]}

    if [ "$destroyed" == "1" ]; then
	# wheredestroyed is 0 based, so it's equal to the last alive position in a 1-based indexing
	wherealive=$wheredestroyed
	threatsSurvived=`cat $logfile | grep "\^" | cut -c1-$wherealive | $TRANSPOSE | grep "\^" | wc -l`
    else
	threatsSurvived=`cat $logfile | grep "\^" | $TRANSPOSE | grep "\^" | wc -l`
    fi
    decisiontimeavg=${vals[5]}
    echo $s,${vals[1]},$destroyed,$wheredestroyed,$horizon,$decisiontimeavg,$threatsSurvived
done
