#!/bin/bash
PWD=`pwd`
cd `dirname $0`
BASEDIR=`pwd`

mkdir -p log

#BASEPARAMS=""
BASEPARAMS="--map-size 100 --num-targets 20 --altitude-levels=10 --auto-range --num-threats 7"
EXTRAPARAMS=""
#EXTRAPARAMS="-- --lookahead-horizon 2"
#EXTRAPARAMS="-- --probability-bound=0.9486833"

#EXTRAPARAMS="-- --probability-bound=0.90 --stay-alive-reward=0"

#EXTRAPARAMS="--threat-sensor-fpr 0 --threat-sensor-fnr 0 -- --lookahead-horizon 40"

for s in $(eval echo "{$1}")
do
    echo Experiment $s
    $BASEDIR/run.sh --seed $s $BASEPARAMS $EXTRAPARAMS > log/exp-s${s}.log
done

cd $PWD
