#ifndef __MXUSB_SERIAL_H_
#define __MXUSB_SERIAL_H_

#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,12,0))
#include "mxusb-serial-0.h"
#else
#include "mxusb-serial-12.h"
#endif

#endif /* #ifndef __MXUSB_SERIAL_H_ */
