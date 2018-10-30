# Moxa Kernel Repository Guidelines

## Table Of Contents
* Getting Started
* How to update kernel
* Products List
* Appendix

---
## Getting Started

### Check out the kernel source:

#### If you want to build the latest am335x-linux-4.4 kernel source code. Please make sure you are at develop branch.
```
# git branch -a
```

#### If you want to build the specific version of am335x-linux-4.4 kernel source code. Please checkout the kernel source using tag.

##### To show all the tags.
```
# git tag
```

##### Checkout kernel source by using tag:
```
# git checkout <product>_V<version>
```
### Steps to compile the kernel:

The detail model support list, default kernel configuration, dts file, its file and kernel type information can be found at "Product List" below.


#### 1. Set default config.
```
# make <product>_defconfig
```
Example:
```
# make uc2100_defconfig
```

#### 2. Compile kernel and create zImage file.
```
# make -j[jobs]
```
Example:
```
# make -j16
```

Our kernel source code will build dtb file automatically. If you want to re-build dtb file by your self.
```
# make <product dts name>.dtb
```
Example:
```
# make am335x-moxa-uc-8100.dtb
```

#### 4. Strip and install kernel modules:
```
# make INSTALL_MOD_STRIP=1 modules_install INSTALL_MOD_PATH=<YOUR DIRECTORY>
```
Example:
```
# make INSTALL_MOD_STRIP=1 modules_install INSTALL_MOD_PATH=/tmp
```

#### 5. Create uImage file or build itb file.

To build itb file, please make sure all the <product> dtb and zImage are exist. (For itb kernel type product.)
```
# ./its/am335x-moxa-<product>
```
Example:
```
# ./its/am335x-moxa-uc-2100
```

To create uImage file. (For uImage kernel type product.)
```
# make uImage
```

---
## How to update kernel
### Steps to update kernel

#### 1. Upload all your compiled results to the device
```
# scp -r <Username of device system>@<IP address of device>:<File path>
```

To upload kernel modules:
Example:
```
# scp -r /tmp/lib/modules/4.4.0-cip-uc2100/kernel/ moxa@192.168.3.100:/tmp
# scp /tmp/lib/modules/4.4.0-cip-uc2100/modules.* moxa@192.168.3.100:/tmp
```

To upload uImage and dtb files:
```
# scp arch/arm/boot/uImage moxa@192.168.3.100:/tmp
# scp arch/arm/boot/am335x-moxa-uc-8100.dtb moxa@192.168.3.100:/tmp
```

To upload itb files:
```
# scp its/am335x-moxa-uc-2100.itb moxa@192.168.3.100:/tmp
```

#### 2. Replace the kernel modules on the device with new files

Backup the original kernel modules and update kernel modules with new files

Example:
```
# mv /lib/modules/4.4.0-cip-uc2100/ /lib/modules/4.4.0-cip-uc2100_bak/
# mkdir -p /lib/modules/4.4.0-cip-uc2100/
# cp -arf /tmp/kernel/ /lib/modules/4.4.0-cip-uc2100/
# cp -arf /tmp/modules.* /lib/modules/4.4.0-cip-uc2100/
# sync
```

#### 3. Replace the kernel file on the device with new file

The kernel file is placed in operation system storage partition 1.
The storage device name can be checked by `lsblk` command:

![lsblk](https://github.com/Moxa-Linux/resize-image/blob/develop/lsblk.PNG?raw=true)

In this example, root is mounted at "/dev/mmcblk0p2", so the system storage device is "/dev/mmcblk0". Partition 1 is "/dev/mmcblk0p1".

To mount operation system storage partition 1

Example:
```
# mount /dev/mmcblk0p1 /mnt
# cd /mnt
```

Backup the original kernel file and update it with new file

Example:
```
# mv am335x-moxa-uc-2100.itb am335x-moxa-uc-2100.itb_bak
# cp /tmp/am335x-moxa-uc-2100.itb .
# sync
```

#### 3. Reboot the device
```
# reboot
```

---
## Products List
### Product kernel configuration:
There following are the list of product kernel configuration files. defconfig is placed in the arch/arm/configs folder, dts file is placed in arch/arm/boot/dts folder, its file is placed in its folder.

#### Kernel Type: itb file
##### UC-2100 series
* models: UC-2101-LX, UC-2102-LX, UC-2104-LX, UC-2111-LX, UC-2112-LX, UC-2112-T-LX, UC-2114-T-LX, UC-2116-T-LX
* defconfig: uc2100_defconfig
* dts: am335x-moxa-uc-2101.dts, am335x-moxa-uc-2102.dts, am335x-moxa-uc-2104.dts, am335x-moxa-uc-2111.dts, am335x-moxa-uc-2112.dts, am335x-moxa-uc-2114.dts, am335x-moxa-uc-2116.dts
* its: am335x-moxa-uc-2100.its

##### UC-3100 series
* models: UC-3101-T-US-LX, UC-3101-T-EU-LX, UC-3101-T-AU-LX, UC-3111-T-US-LX, UC-3111-T-EU-LX, UC-3111-T-AU-LX, UC-3121-T-US-LX, UC-3121-T-EU-LX, UC-3121-T-AU-LX
* defconfig: uc3100_defconfig
* dts: am335x-moxa-uc-3101.dts, am335x-moxa-uc-3111.dts, am335x-moxa-uc-3121.dts
* its: am335x-moxa-uc-3100.its

##### UC-5100 series
* models: UC-5101-LX, UC-5101-T-LX, UC-5102-LX, UC-5102-T-LX, UC-5111-LX, UC-5111-T-LX, UC-5112-LX, UC-5112-T-LX
* defconfig: uc5100_defconfig
* dts: am335x-moxa-uc-5101.dts, am335x-moxa-uc-5102.dts, am335x-moxa-uc-5111.dts, am335x-moxa-uc-5112.dts
* its: am335x-moxa-uc-5100.its

#### Kernel Type: uImage and dtb file

##### UC-8100 series
* models: UC-8131-LX, UC-8132-LX, UC-8162-LX, UC-8112-LX, UC-8112-LX1
* defconfig: uc8100_defconfig
* dts: am335x-moxa-uc-8100.dts, moxa-uc8100.dts
Note: 	moxa-uc8100.dts is for FWR version 1.x and 2.x
	am335x-moxa-uc-8100.dts is for FWR version 3.x or above

##### UC-8100-ME series
* models: UC-8112-ME-T-LX, UC-8112-ME-T-LX1, UC-8112-ME-T-LX-US-LTE, UC-8112-ME-T-US-LTE-LX1
* defconfig: uc8100me_defconfig
* dts: moxa-uc8100-me.dts

---
## Appendix

### How to create a customized image?
To create a customized image, please refer to the following link and see "Steps to create a customized image" section.
[https://github.com/Moxa-Linux/resize-image](https://github.com/Moxa-Linux/resize-image)

### About the kernel version magic name
For standard Linux kernel source code, if the source is maintained in the git repository, the plus sign "+" will be automatically appended to the kerenl version magic.

Example:
	4.4.0-cip-uc2100+

If you switch the kernel source code to a tag or a source build that has never been maintained in git, the plus sign "+" will not be appended.

Example:
	4.4.0-cip-uc2100
