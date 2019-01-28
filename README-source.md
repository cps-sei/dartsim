# DARTSim: An Exemplar for Evaluation and Comparison of Self-Adaptation Approaches for Smart Cyber-Physical Systems

This explains how to build DARTSim in Ubuntu 16.04

## Install Tools and libraries

Install required tools and libraries:
```
sudo apt-get install libboost-all-dev libyaml-cpp-dev make automake autoconf g++ default-jdk ant wget libtool
```

## Build DARTSim
In the top-level directory of DARTSim, where this file is, execute the following commands.

```
autoreconf -i
mkdir build; cd build
../configure
make
```

## Building and Running Examples
There are examples included with DARTSim in the `examples` directory. There is a `README.md` in each explaining how to build the examples.

Instructions for how to run the examples are in [`README.md`](README.md).

