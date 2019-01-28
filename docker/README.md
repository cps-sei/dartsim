# Docker

To provide a portable and consistent build environment, DARTSim is provided in a
[Docker](https://www.docker.com/) container. DARTSim is hosted on Docker Hub,
and can be run by executing the following command:

```
docker run -d -p 5901:5901 -p 6901:6901 --name dartsim gabrielmoreno/dartsim:1.0
```
 
If you would like to build the container locally rather than using the version
hosted on Docker Hub, you can do so with the following instructions.

First, in a terminal, move to the `dartsim/docker` directory, i.e., `cd
dartsim/docker` from the directory that you clone the DARTSim repository.

Next, build the image with the following command:

`docker build . -t gabrielmoreno/dartsim:1.0`

Building the image may take a few minutes as the required files are downloaded,
packages installed, and DARTSim is built from source. When this process
completes, you can run the container with the following command:

`docker run -d -p 5901:5901 -p 6901:6901 --name dartsim gabrielmoreno/dartsim:1.0`

You should now be able to connect to the container over VNC using a VNC client
at port 5901, or over the HTTP interface by pointing a web browser on the host
machine to `127.0.0.1:6901`. The password is `vncpassword`.

You can stop the running container with `docker stop dartsim` and start again
with `docker start dartsim`.
