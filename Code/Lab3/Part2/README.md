# Part 2

In Part 2 of Lab 3, we implement lane following in the American and English
driving styles.

There are two ways to implement the English driving style. The first is to use
the `d_offset` in the `lane_control` package to set the desired offset of the
lane. In this way, the target (middle of the lane) can be offset to the left
hand lane.

The second way is to do something similar to [the project on dynamic obstacle
avoidance](https://github.com/duckietown-ethz/proj-lfvop). Here, they changed
the `lane_filter` package to accept a lane offset. This is also the recommended
way to implement the English driving style
[here](https://docs.duckietown.org/daffy/duckietown-classical-robotics/out/exercise_localization.html).

I've implemented both of these methods, but the former method worked better (at
least for me).

Part 2 of Lab 3 (the code in this directory) is taken and adapted from
[dt-core](https://github.com/duckietown/dt-core) and [a project on dynamic
obstacle avoidance](https://github.com/duckietown-ethz/proj-lfvop).
