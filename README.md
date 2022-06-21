# David Fu and John Wallace's 2022 SPRE Project

## Raspberry Pi Ubuntu 20.04 Installation Guide

Format an SD card using MS-DOS/FAT32 formatting option using Disk Utility. Flash the card with Ubuntu Server 20.04 LTS (64 bit) using the Raspberry Pi Imager.

Follow the steps in this [guide](https://linuxhint.com/install-ubuntu-desktop-20-04-lts-on-raspberry-pi-4/). Ignore the section on flashing to the SD Card and start from "Powering on the Rasperry Pi 4". To connect to servicenet, use `GROUP=sudo`, `ssid=servicenet`, and `key_mgmt=NONE`. Do not provide a `psk` option. You may need to reinitialize `wpa_supplicant.conf` after rebooting and before installing ubuntu desktop.

## Major Dependency Installation Guide

### Ubuntu (aarch64)

#### SSH Guide

Run the following commands on the Pi to initialize an SSH server.

```
sudo apt-get install openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
```

This server's IP address can be found using `hostname -i`.

SSH into the Pi from a remote terminal using the following command and entering the Pi's password.

`ssh ubunti@[PI's IP ADDRESS]`

#### Installing Miniconda3

###### We do not recommend using conda environments to install packages as there are compatibility issues with aarch64 and PyPi packages. If you get the issue `Illegal Instruction: core dumped` when trying to import a library while running `python3` in a conda environment, this is probably the reason.

First download the latest shell script from the Miniconda repo. You can check the [repo](https://repo.anaconda.com/miniconda/) for the latest version by scrolling to the bottom. Find the latest version for Linux-aarch64 (replace "Miniconda3-py39_4.9.2-Linux-aarch64.sh" with the newer filename.)
```
wget https://repo.anaconda.com/miniconda/Miniconda3-py39_4.9.2-Linux-aarch64.sh
```
Then enter
```
sha256sum filename
bash filename
```

#### Installing Intel Realsense SDK and Python wrapper

Follow the steps in this [guide]([(https://github.com/IntelRealSense/librealsense/blob/c94410a420b74e5fb6a414bd12215c05ddd82b69/doc/installation.md)]. Be sure to run the scripts to set Realsense permissions and the relevant patches. 

Here is a full list of packages we found necessary for installation:
```
sudo apt install cmake git build-essential pkg-config
sudo apt install libglfw3 libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
sudo apt install python3.8-dev
sudo apt install openssl libssl-dev
sudo apt install libusb-1.0-0-dev
sudo apt install libcurl4 libcurl4-openssl-dev
sudo apt install libxcursor1 libxcursor-dev
sudo apt install libudev-dev libgtk-3-dev 
```

When running Cmake, use 'cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_PYTHON_BINDINGS=true` For a full list of cmake options see the [Intel documentation][(https://dev.intelrealsense.com/docs/build-configuration)]. 

Troubleshooting for `cmake` step.
`No CMAKE_CXX_COMPILER`: Running `sudo apt-get install build-essential` should solve the problem.

`Python config failure`: Try removing the Cmake cache with `rm CMakeCacheText.txt` Otherwise, the issue is probably with the python interpreter. Use the option `-DPYTHON_EXECUTABLE=(path of python interpreter)`. Python 3.8 has been verified to work.

To use pyrealsense2, copy the `.so` files found in `librealsense/build/python` next to the Python script you want to run. Or copy them to `/usr/lib/python3/dist-packages`.

#### Installing Crazyflie Software

##### Install the dependencies from source:

###### cflib
```
git clone https://github.com/bitcraze/crazyflie-lib-python.git
cd crazyflie-lib-python
sudo pip3 install -e .
```

###### libusb

```
git clone https://github.com/karpierz/libusb.git libusb
python3 -m pip install --editable ./libusb
```

###### cfclient

##### Follow [these](https://github.com/bitcraze/crazyflie-lib-python/blob/master/docs/installation/usb_permissions.md) instructions to ensure the cf radio will work

### Ubuntu (x86)

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
