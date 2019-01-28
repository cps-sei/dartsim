PWD=`pwd`
BASEDIR=`dirname $0`
cd $BASEDIR 
build/src/pla_dart $*
cd $PWD
