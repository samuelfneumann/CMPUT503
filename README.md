# CMPUT503

Welcome to my CMPUT503 repository! A repository to store both code and a
minimal amount of notes from _CMPUT503: Experimental Mobile Robotics_ at the
University of Alberta.

## Structure

In this repository, you'll find two main directories:

- `Notes`: holds some notes from the class
- `Code`: holds the code we write/run in the labs

Within the `Code` directory, each lab has its own sub-directory named `LabX`,
where `X` is the lab number. This directory holds all the data, code, and
other files for the lab exercise, as well as a `pdf` file of the actual
exercise instructions, found in a file called `labX.pdf`.

As previously mentioned, these labs run on DuckieBots, which means we are using
DuckieTown projects to run the code. If you are unfamiliar with DuckieTown
projects, see [this post](https://docs.duckietown.org/daffy/duckietown-robotics-development/out/dt_infrastructure.html).

## Running the Code

The code is meant to be run on a DuckieBot. To run the code for each lab,
you'll first need to build the code on a DuckieBot, and then actually run the
code on the DuckieBot.

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

## More Information

To see these labs running in action, or for more information on these labs,
see the corresponding post on my [website](https://samuelfneumann.github.io).
