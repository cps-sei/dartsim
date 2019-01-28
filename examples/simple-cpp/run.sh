PWD=`pwd`
BASEDIR=`dirname $0`
cd $BASEDIR 
build/src/simple_cpp $*
cd $PWD
