# Building Moxa Linux Kernel am335x-linux-4.4 project

Below you will find instructions to build and install the am335x-linux-4.4 project.

## Version Tags, Branch, Kernel Sourece Table

| Tags | Branch | Kernel Sourece | Note |
| ---- | ------ | -------------- | ---- |
| UC-2100_V1.11<br>UC-2100_V1.12<br>UC-3100_V1.5<br>UC-5100_V1.4<br>UC-3100_V1.6<br>UC-8100_V3.5<br>UC-8100-ME_V3.1<br>UC-8100A-ME_V1.6 | [4.4.285-cip63-rt36/stretch-am335x/master](https://github.com/Moxa-Linux/linux-4.4/tree/4.4.285-cip63-rt36/stretch-am335x/master) | [linux-4.4](https://github.com/Moxa-Linux/linux-4.4/) (`Latest`) | |
| UC-2100_V1.6<br>UC-3100_V1.2<br>UC-5100_V1.2<br>UC-8100_V3.2 | [4.4.190-cip36-rt25/stretch/master](https://github.com/Moxa-Linux/am335x-linux-4.4/tree/4.4.190-cip36-rt25/stretch/master) | [am335x-linux-4.4](https://github.com/Moxa-Linux/am335x-linux-4.4) (*This repository*) (`Outdated`) | |
| UC-3100_V1.0<br>UC-3100_V1.1<br>UC-5100_V1.0<br>UC-5100_V1.1<br>UC-8100_V3.0<br>UC-8100_V3.1<br>UC-2100_V1.2<br>UC-2100_V1.3<br>UC-2100_V1.4<br>UC-2100_V1.5<br>UC-8100-ME_V3.0<br>UC-8100A-ME_V1.0<br>UC-8100A-ME_V1.1<br>UC-8100A-ME_V1.2<br>UC-8100A-ME_V1.3 | [4.4.190-cip36-rt25/stretch/master](https://github.com/Moxa-Linux/am335x-linux-4.4/tree/4.4.190-cip36-rt25/stretch/master) | [am335x-linux-4.4](https://github.com/Moxa-Linux/am335x-linux-4.4) (*This repository*) (`Outdated`) | [Past Instructions](OLD_GUIDELINE.md) |

## Download source

To obtain the am335x-linux-4.4 sources you must clone them as below:

```
git clone https://github.com/Moxa-Linux/am335x-linux-4.4
```

> 📘 Refer to https://github.com/Moxa-Linux/linux-4.4.git for newer kernel source and building flow.

## Dependencies

To build am335x-linux-4.4, we provide [moxa-dockerfiles](https://github.com/Moxa-Linux/moxa-dockerfiles) to create build environment.


## Building

### Create docker container

To create a docker container execute the following commands from the directory which source in:

```
# sudo docker run -d -it -v ${PWD}:/workspace moxa-package-builder:1.0.0 bash
d103e6df5f719f9430056f9c23cf4e3e518d4a4f8b5b65e55889b90c258886c6
```

After execute commands, you will get a string like `d103e6df5f719f9430056f9c23cf4e3e518d4a4f8b5b65e55889b90c258886c6` which called `<container_id>` and we will use it in next step.

### Build kernel package

To build kernel package execute the following commands:

```
# docker start -ia <container_id>
# cd /workspace/am335x-linux-4.4
# apt-get build-dep -aarmhf .
# dpkg-buildpackage -us -uc -b -aarmhf
```

Once build process complete, you can find `.deb` files under `/workspace` directory.

## Updating

After build the kernel packages, now you can update your device.

Below are instructions to update the kernel packages on `UC-5100`.

### Upload the kernel packages to the device

To upload kernel package to the device execute the following commands:

```
# scp uc5100-kernel*.deb uc5100-modules*.deb moxa@192.168.3.127:/tmp
```

### Install the kernel packages

To install kernel package to the device execute the following commands:

```
# cd /tmp
# dpkg -i *.deb
# sync
```

**NOTE: Remember to reboot the device after install the kernel package!**
