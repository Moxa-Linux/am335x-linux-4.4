# Moxa Kernel Repository Guidelines

## Table Of Contents
    * Getting Started
    * Products List

---
## Getting Started
    * Check out the kernel source:
	* To show all the tags:
	$ git tag

	* Checkout kernel source by using tag:
	$ git checkout <product>_V<version>

    * Steps to compile the kernel:
	1. Set default config:
	$ make <product>_defconfig
	e.g. $ make uc2100_defconfig

	2. Compile kernel:
	$ make

	3. Build dtb:
	$ make <product dts name>.dtb
	e.g $ make am335x-moxa-uc-8100.dtb

	4. Create uImage:
	$ make uImage

	5. Strip and install kernel modules:
	$ make INSTALL_MOD_STRIP=1 modules_install INSTALL_MOD_PATH=<YOUR DIRECTORY>
	e.g. make INSTALL_MOD_STRIP=1 modules_install INSTALL_MOD_PATH=/tmp

	6. Build itb:
	$ ./its/am335x-moxa-<product>
	e.g. $ ./its/am335x-moxa-uc-2100
	Note: To build itb file, please make sure all the <product> dtb and uImage are exist.
	      Here are the list of products that support to build itb file: UC-2100, UC-3100, UC-5100
		
---
## Products List
    * Product kernel configuration:
	There following are the list of product kernel configuration files. defconfig is placed in the arch/arm/configs folder, dts file is placed in arch/arm/boot/dts folder, its file is placed in its folder. If the following products have an itb file listed, it supports itb technology on his behalf. If not, it means that the product uses dtb technology.

	* UC-2100 series
    		* defconfig: uc2100_defconfig
		* dts: am335x-moxa-uc-2101.dts, am335x-moxa-uc-2102.dts, am335x-moxa-uc-2104.dts, am335x-moxa-uc-2111.dts, am335x-moxa-uc-2112.dts
		* its: am335x-moxa-uc-2100.its

	* UC-3100 series
    		* defconfig: uc3100_defconfig
		* dts: am335x-moxa-uc-3101.dts, am335x-moxa-uc-3111.dts, am335x-moxa-uc-3121.dts
		* its: am335x-moxa-uc-3100.its

	* UC-5100 series
    		* defconfig: uc5100_defconfig
		* dts: am335x-moxa-uc-5101.dts, am335x-moxa-uc-5102.dts, am335x-moxa-uc-5111.dts, am335x-moxa-uc-5112.dts
		* its: am335x-moxa-uc-5100.its

	* UC-8100-LX series
    		* defconfig: uc8100_defconfig
		* dts: am335x-moxa-uc-8100.dts, moxa-uc8100.dts
		Note: moxa-uc8100.dts is for version 1.x and 2.x product
		      am335x-moxa-uc-8100.dts is for version 3.x or above

	* UC-8100-ME-T-LX series
    		* defconfig: uc8100me_defconfig
		* dts: am335x-moxa-uc-8100-me.dts, moxa-uc8100-me.dts
		Note: moxa-uc8100-me.dts is for version 1.x and 2.x.
		      am335x-moxa-uc-8100-me.dts is for version 3.x or above.
