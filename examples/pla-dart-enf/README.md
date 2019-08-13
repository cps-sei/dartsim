# PLA-DART: PLA-Based Adaptation Manager for DARTSim
This is an adaptation manager for DARTSim based on PLADAPT, the Proactive Latency-Aware Adaptation Manager. It provides an example of an adaptation for DARTSim that links with the DARTSim library to create a single executable.

This adaptation manager has only been tested on Ubuntu 16.04.

## Building PLA-DART

### Install Dependencies
This adaptation manager requires PLADAT to be built first. It can be downloaded from [here](https://github.com/cps-sei/pladapt). Follow the instructions to build the PLADAPT library. Note that it is not necessary to compile the Java Wrapper. Make sure that the `PLADAPT` environment variable is set to the path of the top-level directory of PLADAPT as explained in its [README](https://github.com/cps-sei/pladapt/blob/master/README.md).

### Compiling PLA-DART
In the following examples `$PLADART` refers to the top-level directory of this example, where this README is.

Execute the following commands to compile the adaptation manager.

```
cd $PLADART
autoreconf -i
mkdir build; cd build
../configure
make
```

## Executing PLA-DART
There are several options to run the simulation and configure the adaptation manager.
Running `./run.sh --help` will list them. There are two kinds of options supported:
the ones for DARTSim, and the ones for the adaptation manager. Every option
that precedes the `--` argument is an option for DARTSim, the ones that follow
the `--` argument are options for the adaptation manager. DARTSim options are
described in the DARTSim `README.md` file, and adaptation manager options are
listed at the end of this document.

```
./run.sh --seed 1234 -- --adapt-mgr sdp --stay-alive-reward=0.5 
```

The output will log the tactics executed at different times.
The end of the output looks similar to this:

```
Total targets detected: 2
#                                       
 # #    ##   ## #   # # # ## # # # ## *#
  * *  #  * #  # # # * # #  # # # #  *  
     **    #      *                     
 ^      ^       ^   ^ ^    ^            
       T   X     X          T           
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

To run the same example with PLA-SDP extended with probabilistic requirements, we can specify the required probability of surviving the mission, instead of having to select a reward for staying alive.
```
./run.sh --seed 1234 -- --adapt-mgr sdpra --probability-bound 0.95
```

# Adaptation Manager Options
The following are options that can be used to configure the adaptation manager.

## General Options

### `--lookahead-horizon=value`
Set the decision horizon

### `--distrib-approx=value`
Select probability distribution approximation used to model environment.
0 selects Extended Pearson-Tuckey (3-point discretization), and 1 selects a
point estimate approximation.

### `--adapt-mgr=value`
Choose the adaptation manager that you want to use. Currently supports:
*   `pmc` for PLA-PMC
*   `sdp` for PLA-SDP
*   `sdpra` for PLA-SDP with probabilistic survivability requirement

## SDP Options

### `--stay-alive-reward=value`
Set reward for the team of drones surviving the mission.

### `--non-latency-aware`
Makes the adaptation manager ignore the latency of its adaptations.
(It assumes all actions are instantaneous)

### `--ecm`
Use the tactics to turn ECM (Electronic Countermeasures) on and off. When ECM
is turned on, it reduces the chance of both target detection and destruction
by threats.

### `--two-level-tactics`
Allow the adaptation manager to use tactics that climb or descend 2 altitude
levels.

### `--no-formation`
Remove the option for loose and tight formation tactics.

### `--reach-path=value`
Set path of `reach.sh` script of PLADAPT.

### `--reach-model=value`
Set path prefix for Alloy models.

### `--probability-bound=value`
Set the minimum survival probability that the SDPRA adaptation manager will try
to achieve.

## PMC Options

### `--prism-template=value`
Give the path to the PRISM template. Defaults to the path to `model/dart2.prism`
