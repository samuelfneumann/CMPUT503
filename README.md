# CMPUT503

Welcome to my CMPUT503 repository! A repository to store both code and a
minimal amount of notes from _CMPUT503: Experimental Mobile Robotics_ at the
University of Alberta.

## Structure

In this repository, you'll find a few main directories:

- `Code`: holds the code we write/run in the labs
- `Notes`: holds some notes from the class
- `Survey`: `tex` files to generate a pdf document of a survey of GVFs in robotics

Within the `Code` directory, each lab has its own sub-directory named `LabX`,
where `X` is the lab number. This directory holds all the data, code, and
other files for the lab exercise, as well as a `pdf` file of the actual
exercise instructions, found in a file called `labX.pdf`.

As previously mentioned, these labs run on DuckieBots, which means we are using
DuckieTown projects to run the code. If you are unfamiliar with DuckieTown
projects, see [this post](https://docs.duckietown.org/daffy/duckietown-robotics-development/out/dt_infrastructure.html).

## Running the Code

The code is meant to be run on a DuckieBot. To run the code for each lab
(except for lab 1), you'll first need to build the code on a DuckieBot and
then actually run the code on the DuckieBot using the `dts` command.

To build to code for `LabX`, first `cd` into the directory then build the code
with `dts`:

```bash
cd Code/LabXXJk
dts devel build -f -H DUCKIEBOT_NAME
```

where `DUCKIEBOT_NAME` is replaced with the name of your DuckieBot. Then, you
can run the code using:

```bash
dts devel run -H DUCKIEBOT_NAME
```

## Description of Labs

The code for all labs is in the `Code` directory. This section describes each
lab and outlines how to run each lab on a DuckieBot. In the instructions below,
replace `MY_ROBOT` with your robot's hostname.

### Lab 1

The code for lab 1 is in `./Code/Lab1`. This lab was mostly concerned with
building the DuckieBot then being able to communicate with it over a network.
We did write a color detection program to detect colours from the robot's
camera. To run this code:

```bash
cd ./Code/Lab1
dts devel build -f --arch arm64v8 -H MY_ROBOT.local
docker -H MY_ROBOT.local run -it --rm --net=host duckietown/my-program:latest-arm64v8
```

We use Docker to run the lab rather than `dts` because the lab doesn't use a
DuckieTown project.

### Lab 2

The code for lab 2 is in `./Code/Lab2`. Lab 2 was concerned with two things:

1. Driving the DuckieBot with your own code
2. Estimating the DuckieBot's odometry

The lab instructions can be found at `./Code/Lab2/lab2.pdf`. See those
instructions for more information about what we actually did for the lab. To
run the code for the lab:

```bash
cd ./Code/Lab2
dts devel build -f -H MY_ROBOT.local
dts devel run -f -H MY_ROBOT.local
```

## More Information

To see these labs running in action, or for more information on these labs,
see the corresponding post on my [website](https://samuelfneumann.github.io).
