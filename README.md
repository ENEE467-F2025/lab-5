# ENEE 467 Fall 2025: Robotics Project Laboratory
## Lab 5: Collision-Free Kinematic Planning in ROS 2

This repository contains a Docker container for Lab 5 (Collision-Free Kinematic Planning in ROS 2) as well as the necessary code templates for completing the exercises. Software for both parts is provided in this repo; the manuals will specify which packages to run for which labs.

## Overview

Motion planning is the robotics subfield concerned with computing a continuous, collision-free motion that takes a robot from a start configuration to a goal configuration. Unlike the forward and inverse kinematics problems that are only concerned with the robot's geometry and not its workspace, motion planning entails reasoning about the robot's motion relative to other artifacts in its environment, including objects to be grasped and workspace obstacles, ensuring that joint limits, dynamic constraints, and collision avoidance requirements are respected throughout the motion. As such, motion planning forms a critical component for robots operating in semi-controlled or unstructured human–machine environments. Thus, this lab is designed to equip you with the skills required to both effectively utilize de facto modern motion planning software and to develop independent and ROS 2 compliant motion planning programs of your own.

## Lab Software

To avoid software conflicts and increase portability, all lab software will be packaged as a Docker container. Follow the instructions below to get started.

## Building the Container

First check to see if the image is prebuilt on the lab computer by running the following command
```
docker image ls
```
If you see the image named `lab-5-image` in the list then you can **skip** the build process.

To build the Docker container, ensure that you have [Docker](https://www.docker.com/get-started/) installed and the Docker daemon running.
* Clone this repository and navigate to the `docker` folder
    ```
    cd ~/Labs
    git clone https://github.com/ENEE467-F2025/lab-5.git
    cd lab-5/docker
    ```
* Build the image with Docker compose
    ```
    userid=$(id -u) groupid=$(id -g) docker compose -f lab-5-compose.yml build
    ```

## Starting the Container

The lab computers contain a prebuild image so you will not have to build the image.
* Clone this repo to get the lab-5 code if you haven't done so already
    ```
    cd ~/Labs
    git clone https://github.com/ENEE467-F2025/lab-5.git
    cd lab-5/docker
    ```
* Run the Docker container
    ```
    userid=$(id -u) groupid=$(id -g) docker compose -f lab-5-compose.yml run --rm lab-5-docker
    ```
* Once inside the container, you should be greeted with the following prompt indicating that the container is running
    ```
    (lab-5) robot@docker-desktop:~$
    ```
* Edit the lab-5 Python (ROS 2) code  within the `lab-5/src` and `lab-5/src/exercise` folders from a VS Code editor on the host machine. The repo directory `lab-5/src`  is mounted to the docker container located at `/home/robot/ros2_ws/src` so all changes will be reflected **inside** the container.

* Test your setup by running the following script from a terminal within the Docker container:
    ```
    cd ~/ros2_ws/src
    python3 test_docker.py
    ```
    This should print the following output to the terminal (if the message doesn’t appear, stop and contact your TA, otherwise proceed with the lab procedure): 
    ```txt 
    All packages for Lab 5 found. Docker setup is correct.
    ```
## Attaching the Docker Container to VSCode
To enable type hints and IntelliSense, after starting the container, run the following command from a new terminal on the lab machine to attach the running container to VSCode:
```bash
code --folder-uri vscode-remote://attached-container+$(printf "$(docker ps -q --filter ancestor=lab-5-image)" | od -A n -t x1 | sed 's/ *//g' | tr -d '\n')/home/robot/ros2_ws/src
```
Once this is done, verify that the Python extension is installed in the container on VSCode, otherwise install it to enable type hints.

## Lab Instructions

Please follow the [lab manual](Lab_5_Motion-Planning-Part-1.pdf) closely. All instructions are contained inside the lab manual.