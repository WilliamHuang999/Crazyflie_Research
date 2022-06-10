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
