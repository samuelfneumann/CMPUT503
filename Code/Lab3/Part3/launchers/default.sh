#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
# dt-exec roslaunch deadreckoning deadreckoning_node.launch veh:=$VEHICLE_NAME
# dt-exec roslaunch camera_image camera_image_node.launch veh:=$VEHICLE_NAME
dt-exec roslaunch lab3 lab3.launch veh:=$VEHICLE_NAME


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
