# What
Setup a Docker container to run OpenCV 4.5.2, integrating it with VSCode for development.

# How
We need to:
1. build a container with all the dependencies needed
2. run it on several terminals at once
3. integrate VSCode to be able to program with OpenCV

## Step 1: build the Docker container
This repository contains a file named Dockerfile which instructs Docker on how to setup a container. Once built, it will contain all relevant dependencies to program with OpenCV.
Build the container as follows:
- change to the directory containing Dockerfile
- run `$ docker build .` This will take some time but you should do it only once.

## Step 2: run the Docker container
Now we want to run the container, instanting it. It is pretty straightforward but there are a few nuances to consider:
- run `$ docker images` to list all the Docker images you have in your laptop. You should see at least two. One named hello-world (precreated when installing Docker) and one created at Step 1. Take note of the latter's `IMAGE ID`
- run `$ xhost +local:*` to open access your X server (Linux only *)
- run `$ docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix/ -it <YOUR IMAGE ID>` to finally instantiate the containter, connecting it to your display (Linux only *)

## Step 3: check everything works
- run `$ pkg-config --modversion opencv4` and the installed version should popup (`4.2.0`, not sure why not `4.5.2` as I intended 🤔)

## Step 4: connect more terminals
- open another terminal on the host machine
- run `$ docker ps` to list all running containers. Take note of the `NAMES` of your container
- run `$ docker exec -it <YOUR NAMES> bin/bash`. You will get your new terminal

## Step 5: integrate VSCode
- download Dev-Containers
- `Ctrl+P`, then type `> Dev-Containers: Attach to Running Container...`
- select your container
- after a quick setup your terminals in VSCode will belong to the Docker container, and your folders too

## (*) For non-Linux laptops
Everything should work the same, but you may need a different woraround to share your display with the Docker container:
- Windows: use VcXsrv Windows X Server, somehow
- MacOS: use XQuartz
  - download from https://www.xquartz.org/
  - after installation, restart you Mac. This is mandatory. Logging out is not enough.
  - open XQuartz, go to Preferences > Security > check `Allow connections from network clients`
  - open a terminal and run `ifconfig en0`. Take note of you local IP address (it's something like 192.168.<>.<>)
  - from XQuartz' terminal, run `xhost <YOUR LOCAL IP ADDRESS>`
  - now from a terminal you can finally run the container sharing your Mac's display, with `docker run -e DISPLAY=$<YOUR_LOCAL_IP_ADDRESS>:0 -it <YOUR IMAGE ID>`
  - now you can keep going with Step 3 and on
