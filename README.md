# David Fu and John Wallace's 2022 SPRE Project

## Major Dependency Installation Guide

### Ubuntu

#### Installing Librealsense

Run the following terminal commands.

```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
```

To open the Realsense GUI, run `realsense-viewer`.

#### Installing Pyrealsense

Run `pip3 install pyrealsense2`

#### Installing opencv

Run `pip3 install opencv-python`

##### Other dependencies are likely needed, but those can be installed with simple `pip3` terminal commands.
