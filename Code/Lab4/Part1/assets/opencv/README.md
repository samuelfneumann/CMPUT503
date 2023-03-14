# Nov 27 2022
OpenCV recently stopped building for `arm32v7` and binaries are not available on `pip`. 
While binaries are available on `apt` for all three archs, the version available on apt is older than those 
available via pip.

Also, `python3-opencv` installed via apt is not detected by `pip` because of a missing egg-info descriptor, 
so `pip` tries to install it (and build for `arm32v7`) again.

The solution is to break this into three different scripts,
 
- amd64: install via `pip`;
- arm64v8: install via `pip`;
- arm32v7: install (older version) via `apt` and add a fake egg-info to tell `pip` not to install it again;
