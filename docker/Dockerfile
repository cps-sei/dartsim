FROM consol/ubuntu-xfce-vnc:1.2.3
LABEL Description="Docker image for DART Simulator with VNC"

USER root

# first install dependencies
RUN apt update && apt install -y --no-install-recommends git libboost-all-dev \
libyaml-cpp-dev make automake autoconf g++ default-jdk ant wget libtool gdb curl parallel evince

# now install eclipse
# download eclipse-cpp
RUN wget http://ftp.osuosl.org/pub/eclipse/technology/epp/downloads/release/2018-12/R/eclipse-cpp-2018-12-R-linux-gtk-x86_64.tar.gz
# extract eclipse to /opt
RUN tar --directory=/opt -xzf eclipse-cpp-2018-12-R-linux-gtk-x86_64.tar.gz && \
rm eclipse-cpp-2018-12-R-linux-gtk-x86_64.tar.gz
# add eclipse to PATH
ENV PATH $PATH:/opt/eclipse

# now clone and build DARTSim
WORKDIR $HOME
RUN git clone https://github.com/cps-sei/dartsim.git
WORKDIR dartsim

# build DART
RUN autoreconf -i && mkdir build; cd build; ../configure && make

# build the examples
WORKDIR examples/pla-dart
# first clone and build the PLADAPT dependency
RUN git clone https://github.com/cps-sei/pladapt.git
WORKDIR pladapt
RUN autoreconf -i && mkdir build; cd build; ../configure && make
#optional build commands for PLADAPT
# if PLA-SDP is desired, build the script to generate reachability functions
RUN reach/build.sh
# compile the Java wrapper
#RUN cd java; make

#build pla-dart example
# set env variable for building pla-dart
ENV PLADAPT $HOME/dartsim/examples/pla-dart/pladapt
WORKDIR ..

RUN autoreconf -i; mkdir build; cd build; ../configure && make

# build the simple-java example
WORKDIR ../simple-java
RUN ./build.sh

# build the simple-cpp example
WORKDIR ../simple-cpp
RUN autoreconf -i; mkdir build; cd build; ../configure && make

# add projects to eclipse workspace
WORKDIR $HOME/dartsim
RUN /opt/eclipse/eclipse -nosplash -data /root/eclipse-workspace -application org.eclipse.cdt.managedbuilder.core.headlessbuild -import . -import examples/simple-cpp

# create desktop shortcuts
ADD eclipse.desktop /headless/Desktop
ADD exo-terminal-emulator.desktop /headless/Desktop
RUN ln $HOME/dartsim/docs/tcp-interface.pdf /headless/Desktop

# remove unneeded clutter from desktop
RUN cd $HOME/Desktop; rm firefox.desktop; rm chromium-browser.desktop; \
chmod +x *.desktop

# Cleanup
RUN apt-get clean && \
  rm -rf /var/lib/apt

WORKDIR $HOME
