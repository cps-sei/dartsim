#!/bin/bash
PWD=`pwd`
cd `dirname $0`
BASEDIR=`pwd`

mkdir -p log

#BASEPARAMS=""
BASEPARAMS="--map-size 100 --num-targets 20 --altitude-levels=10 --auto-range --num-threats 7"
#BASEPARAMS="--map-size 25 --num-targets 5 --altitude-levels=10 --auto-range --num-threats 2"
#EXTRAPARAMS=""
#EXTRAPARAMS="-- --lookahead-horizon 2"
#EXTRAPARAMS="-- --distrib-approx=1"
#EXTRAPARAMS="--threat-sensor-fpr 0 --threat-sensor-fnr 0 -- --lookahead-horizon 40"
#EXTRAPARAMS="--threat-sensor-fpr 0 --threat-sensor-fnr 0 --target-sensor-fpr 0 --target-sensor-fnr 0 -- --distrib-approx=1 --probability-bound=0.9503228 --lookahead-horizon=7"
EXTRAPARAMS="-- --probability-bound=0.985 --lookahead-horizon=7 --improvement-threshold=0.05"
#EXTRAPARAMS="-- --probability-bound=0.6 --improvement-threshold=0"

for s in $(eval echo "{$1}")
do
    echo Experiment $s
    $BASEDIR/run.sh --seed $s $BASEPARAMS $EXTRAPARAMS > log/exp-s${s}.log
done

cd $PWD
