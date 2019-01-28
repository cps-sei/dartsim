# Simple C++ Adaptation Manager for DARTSim
This is a simple adaptation manager for DARTSim written in C++. It provides an example of an adaptation for DARTSim that links with the DARTSim library to create a single executable.

This adaptation manager has only been tested on Ubuntu 16.04.

## Building the Example
In the top-level directory of this example, where this README is, execute the following commands to compile the adaptation manager.

```
autoreconf -i
mkdir build; cd build
../configure
make
```

## Executing the Example
There are several options to run the simulation and configure the adaptation manager.
Running `./run.sh --help` will list them. There are two kinds of options supported:
the ones for DARTSim, and the ones for the adaptation manager. Every option that precedes the `--` argument is an option for DARTSim, the ones that follow the `--` argument are options for the adaptation manager.

```
./run.sh --seed 1234 -- --lookahead-horizon=4 
```

The output will log the tactics executed at different times.
The end of the output looks similar to this:

```
Total targets detected: 2
### * # # # * # ##            ## * * #*#
   # # # # # * *  ##         #  # # #   
                    #    #  #           
                     #### ##            
    ^        ^ ^              ^  ^    ^ 
          TT     T                 T    
out:destroyed=0
out:targetsDetected=2
out:missionSuccess=1
csv,2,0,39,1,4.26153,18.3524
```
The part with all the symbols is a 2D side view of the route of the team of drones.
The two-bottom lines represent the ground, and the lines before them represent the altitude level of the drones at the different positions in the route. The symbols have the following meaning.

Symbol | Meaning
-------|-------------------------
\#      | loose formation
\*      | tight formation
@      | loose formation, ECM on
0      | tight formation, ECM on
^      | threat
T      | target (not detected)
X      | target (detected)

The last line has a summary of the results in csv format:
```
csv, targets detected, team destroyed, last team position, mission success, decision time avg, decision time variance
```

