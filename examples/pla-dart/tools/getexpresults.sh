#!/bin/bash


echo true_seed,targetsDetected,destroyed,lookahead.horizon,decisionTimeAvg
for s in $(eval echo "{$1}")
do
    logfile="log/exp-s${s}.log"
    # csv,detected,destroyed,wheredestroyed,mission_success,decision_time_avg,decision_time_var
    #   0,       1,        2,             3,              4,                5,                6
    csv=`cat $logfile | grep csv`
    vals=( `echo -n $csv | tr "," " "` )
    horizon=`cat $logfile | grep "lookahead horizon=" | cut -d= -f2`
    echo $s,${vals[1]},${vals[2]},$horizon,${vals[5]}
done
