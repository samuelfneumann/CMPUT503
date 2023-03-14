# numpy needs to be pinned to avoid this: https://staging-ci.duckietown.org/view/Wall%20-%20daffy-staging/job/Docker%20Autobuild%20-%20daffy-staging%20-%20dt-core%20-%20arm64v8/43/consoleFull
python3 -m pip install -U opencv-python "numpy<=1.20.0"
