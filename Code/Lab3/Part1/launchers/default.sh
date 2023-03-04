#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
dt-exec roslaunch apriltag apriltag_detector_node.launch veh:=$VEHICLE_NAME
dt-exec roslaunch apriltag apriltag_postprocessing_node.launch veh:=$VEHICLE_NAME
dt-exec roslaunch led_emitter led_emitter_node.launch veh:=$VEHICLE_NAME
# dt-exec roslaunch augmented_reality_basics nodes.launch veh:=$VEHICLE_NAME map_file:="/data/map/calibration_pattern.yaml"


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
