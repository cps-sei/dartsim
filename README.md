# DARTSim: An Exemplar for Evaluation and Comparison of Self-Adaptation Approaches for Smart Cyber-Physical Systems

DARTSim is an exemplar smart cyber-physical system (sCPS) to enable the
evaluation of self-adaptive adaptation managers in this challenging domain.
DARTSim is a high-level simulation of a team of UAVs performing a reconnaissance
mission in an unknown hostile environment. Designed for ease of use by the
research community, DARTSim provides a TCP-based interface for easy integration
with external adaption managers, three example adaption managers (in C++ and
Java), and is provided as a [Docker](https://www.docker.com/) container for 
portability.

# Installation

## Installing Docker

DARTSim uses Docker to automate deployment into a container. Docker can be
downloaded for free from the [Docker website](https://www.docker.com/community-edition#/download)
(the community edition is sufficient). It must be installed before following the
rest of the instructions.

## Creating and Running the DARTSim Container

DARTSim is hosted on Docker Hub, a repository for hosting Docker containers.
Once Docker is installed, the following command will automatically download
all required content from Docker Hub, and will start the container.

```
docker run -d -p 5901:5901 -p 6901:6901 --name dartsim gabrielmoreno/dartsim:1.0
```

**Note:** Don't do this now, but when you are done using the container, you
can stop the container by running `docker stop dartsim`. You can start the
container again with `docker start dartsim`.

## Connecting to the Container

Once the container is started (either by the previous `run` command or after
the `start` command, the container is ready to accept connections.
The container opens two ports on your host (5901 and 6901) that are mapped to 
the same ports in the container. These ports allow two modes of connection with
DARTSim: via HTTP or via a VNC client.

If you have a VNC client installed, you can connect through it to the host:
`vnc://localhost:5901`, and using the password `vncpassword`. To connect
with HTTP, open a HTML5 compatible browser, and open the address
http://localhost:6901 This will also prompt you for the password 
(which is the same as above).

# Running DARTSim

DARTSim is designed to be controlled by an adaptation manager. The interaction
between the adaptation manager and DARTSim can be done in two ways.
1. Linking DARTSim as a library. In this case both the adaptation manager and
DARTSim execute in the same process.
2. Connecting to DARTSim through its TCP interface. In this case, the
adaptation manager and DARTSim run in different processes.

The following instructions explain how to run DARTSim in these two modes using
the included example adaption managers.

## Running DARTSim as a Library with Simple C++ Example
The `simple-cpp` example included with DARTSim uses it as a library. To run
this example, follow these steps.

1. Start a Terminal Emulator from the `Applications` menu in the top-left
corner or with the desktop icon. This opens a terminal within the container. 
2. In this terminal, type the commands 
```
   > cd ~/dartsim/examples/simple-cpp/
   > ./run.sh
```

You should see output similar to this showing what the adaptation manager
commanded the simulator to do at each position in the route.
```
current position: 0;0
executing tactic DecAlt
current position: 1;0
executing tactic GoTight
executing tactic IncAlt
current position: 2;0
executing tactic DecAlt
executing tactic GoLoose
...
```

When the simulation completes, summary information is shown, along with a
text diagram showing a side view of the execution of the simulation, as
shown here.

```
Total targets detected: 1
### * *###### # # ## * # # #* *         
   * #       # # *  # # # *  # #        
                                #      #
                                 ###### 
   ^  ^          ^        ^ ^ ^         
              T           T      X    T 
out:destroyed=0
out:targetsDetected=1
out:missionSuccess=0
csv,1,0,39,0,0.00568157,8.35444e-05
```

The part with all the symbols is a 2D side view of the route of the team of
drones. The two-bottom lines represent the ground, and the lines before them
represent the altitude level of the drones at the different positions in the
route. The symbols have the following meaning.

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

The `run.sh` script supports passing options as follows:
```
./run.sh [simoptions] [-- [--lookahead-horizon=value]]
```
The simulator options are described at the end of this document. The
`--lookahead-horizon` option allows specifying the length of the horizon
(greater than or equal to 1) used by the this adaptation manager. For example,
to run it with a horizon of length 5 use use:
``` 
   > ./run.sh -- --lookahead-horizon=5
```

## Running DARTSim Controlled through its TCP Interface

DARTSim provides a simple adaptation manager implemented in Java for
instructional purposes. Note that this adaption manger does not make the best
decisions, but it does provide an example of how an adaptation manager can
integrate with DARTSim over its TCP interface.

When DARTSim is controlled through its TCP interface, DARTSim and the
adaptation manager run in different processes. Follow the following steps
to execute the `simple-java` example.

1. Start a Terminal Emulator from the `Applications` menu in the top-left
corner or with the desktop icon. This opens a terminal within the container. 
2. In this terminal, type the commands 
```
   > cd ~/dartsim
   > ./run.sh
```

You should see `Simulator instantiated`, this means that the simulator is
running and listening on port 5418. It is now ready for an adaption
manager to give commands over the TCP interface. Note that the simulation
does not proceed unless the adaptation manager sends a command to do so.
Now, we are ready to start the adaptation manager.

3. Start another Terminal Emulator from the `Applications` menu in the
top-left corner or with the desktop icon. 
4. In this terminal, type the commands 
```
   > cd ~/dartsim/examples/simple-java/
   > ./run.sh
```

You should see output on the first terminal logging tactic executions like the
following:

```
Simulation client connected
executing tactic GoLoose
executing tactic GoLoose
executing tactic DecAlt
executing tactic GoLoose
```

When the simulation terminates, you should see output in the second
terminal showing the results of the simulation, including a text-based
representation of the simulation trace, like below:

```
###               ###                   
   ##           ##   #####              
     # #  ######          #    #####    
      # ##                 ####         
      ^  ^     ^ ^      ^          ^    
    X            T  T       X           

Destroyed = true
Targets detected = 2
Mission success = false
Decision time agv = 131.11111111111111  var = 17209.5
```

### Launching DARTSim and Adaptation Manager with a Single Command
Another script in the top-level directory of DARTSim, `run-with-am.sh`, is
provided for convenience. This script will start DARTSim as well as a
specified adaption manager, allowing for both processes to be started with one
command.

```
./run-with-am.sh adaptmgrpath [simoptions [-- adaptmgroptions]]
```

where `adaptmgrpath` is the path of the executable or shell script to launch
the adaptation manager, `simoptions` are options for DARTSim, and
`adaptmgroptions` are options passed to the adaptation manager.

See the below instructions on running the simple-java example for an example of
using this script. To run the `simple-java` example in this way, follow the
following steps.
1. Start a Terminal Emulator from the `Applications` menu in the top-left
corner or with the desktop icon. This opens a terminal within the container. 
2. In this terminal, type the commands 
```
   > cd ~/dartsim
   > ./run-with-am.sh examples/simple-java/run.sh
```

## PLA Example

A third example included with DARTSim is `pla-dart` an adaption manager based
on PLA-SDP, described in the following publications:

[1] Gabriel A. Moreno, Javier Camara, David Garlan and Bradley Schmerl. "Efficient Decision-Making under Uncertainty for Proactive Self-Adaptation." _Proc. of the International Conference on Autonomic Computing_ (2016) [[link]](http://works.bepress.com/gabriel_moreno/28/)

[2] Gabriel A. Moreno. _Adaptation Timing in Self-Adaptive Systems_. PhD Thesis, Carnegie Mellon University (2017) [[link]](http://works.bepress.com/gabriel_moreno/31/)

This adaption manager is written in C++ and compiles to a single executable
binary, proving another example of using DARTSim as a library. To run the
`pla-dart` example, follow the following steps.
1. Start a Terminal Emulator from the `Applications` menu in the top-left
corner or with the desktop icon. This opens a terminal within the container. 
2. In this terminal, type the commands 
```
   > cd ~/dartsim/examples/pla-dart
   > ./run.sh
```

The output is similar to that of the `simple-cpp` example.

# Source Code and Building

DARTSim and its examples are already compiled in the container. Nevertheless,
a development environment is included in the Docker container, for exploring
its source code and experimenting modifying the examples.

The development environment, Eclipse, can be launched using the icon on the
container's desktop. DARTSim can be built from source from Eclipse.

DARTSim and its examples can also be built from the command line. Instructions
for doing so are included in the file `README-source.md`

## DARTSim Command Line Options
The following are options that can be used to configure DARTSim.

### `--help`
Lists all command line arguments.

### `--square-map`
Creates a square map for the drones to travel. The drones cover the map using
a lawn-mover pattern. The sharp turns at the end of each side of the square
add uncertainty because the forward-looking sensors can only sense in a
straight line.

### `--map-size=value`
Set the length of the route when the map is not square. For a square map, this
sets the length of the sides of the map.

### `--num-targets=value`
Set the number of targets generated.

### `--num-threats=value`
Set the number of threats generated.

### `--altitude-levels=value`
Set the number of altitude levels that the drones can use.

### `--threat-range=value`
Set the range of the threats in altitude levels. Use `--auto-range` to
automatically adjust this to 75% of the number of altitude levels.

### `--dl-target-sensor-range=value`
Set the range from which targets can be detected on the ground when the drones
fly over them. Use `--auto-range` to automatically adjust this to the number
of altitude levels.

### `--auto-range`
Automatically sets the target and threat sensors to 100% and 75% of the number
of altitude levels respectively.

### `--threat-sensor-fpr=value`, `--threat-sensor-fnr=value`
Set the false positive and false negative rates (respectively) for the
forward-looking threat sensor.

### `--target-sensor-fpr=value`,`--target-sensor-fnr=value`
Set the false positive and false negative rates (respectively) for the
forward-looking target sensor.

### `--change-alt-latency=value`
Adjusts the latency of the tactics for changing altitude. Given in periods
(a.k.a. simulation steps). This defaults to 1 period.

### `--seed=value`
Set the random seed for the master random generator that controls all the
random behavior in the simulation. Using the same seed value allows
replicating the same conditions in multiple runs of the simulator.

### `--opt-test`
Run an optimality test if the adaptation manager supports it. Generates a
single plan at the beginning and runs it throughout the simulation.
Only the `pla-dart` example with the SDP adaptation manager supports this
option.
