#!/bin/bash
PWD=`pwd`
cd `dirname $0`
BASEDIR=`pwd`

mkdir -p log

EXTRAPARAMS=""
#EXTRAPARAMS="-- --distrib-approx=1"
#EXTRAPARAMS="--threat-sensor-fpr 0 --threat-sensor-fnr 0 -- --lookahead-horizon 40"

for s in $(eval echo "{$1}")
do
    echo Experiment $s
    $BASEDIR/run.sh --seed $s --map-size 100 --num-targets 20 --altitude-levels=10 --auto-range --num-threats 7 $EXTRAPARAMS > log/exp-s${s}.log
done

cd $PWD
