SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

set -ex

# update lists
apt-get update

# install libraries
apt-get install -y \
    python3-matplotlib \
    python3-scipy \
    python3-ruamel.yaml \
    python3-yaml \
    python3-opencv \
    python3-cffi \
    python3-psutil \
    python3-setproctitle

# cleanup
apt-get clean
rm -r /var/lib/apt/lists/*

# add fake egg for opencv
cp -r ${SCRIPT_DIR}/fake-opencv-info /usr/lib/python3.8/opencv_python-4.2.0.dist-info
