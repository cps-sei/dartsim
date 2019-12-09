#!/bin/bash
PWD=`pwd`
cd `dirname $0`
BASEDIR=`pwd`

mkdir -p log

for s in $(eval echo "{$1}")
do
    echo Experiment $s
    $BASEDIR/run.sh --seed $s --map-size 100 --num-targets 20 --altitude-levels=10 --auto-range --num-threats 7 -- --probability-bound=0.9486833 > log/exp-s${s}.log
done

cd $PWD
