/*
 * TI 3410/5052 USB Serial Driver
 *
 * Copyright (C) 2004 Texas Instruments
 *
 * This driver is based on the Linux io_ti driver, which is
 *   Copyright (C) 2000-2002 Inside Out Networks
 *   Copyright (C) 2001-2002 Greg Kroah-Hartman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * For questions or problems with this driver, contact Texas Instruments
 * technical support, or Al Borchers <alborchers@steinerpoint.com>, or
 * Peter Berger <pberger@brimson.com>.
 */
/*
 * Add MOXA ID
 * Add MOXA 3 Mode feature
 * Harry 2017.07.07
*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/ioctl.h>
#include <linux/serial.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#define VICTOR_DEBUG
#include <linux/moxa_driver.h>

/* Configuration ids */
#define TI_BOOT_CONFIG			1
#define TI_ACTIVE_CONFIG		2

/* Vendor and product ids */
#define TI_VENDOR_ID			0x0451
#define IBM_VENDOR_ID			0x04b3
#define TI_3410_PRODUCT_ID		0x3410
#define IBM_4543_PRODUCT_ID		0x4543
#define IBM_454B_PRODUCT_ID		0x454b
#define IBM_454C_PRODUCT_ID		0x454c
#define TI_3410_EZ430_ID		0xF430  /* TI ez430 development tool */
#define TI_5052_BOOT_PRODUCT_ID		0x5052	/* no EEPROM, no firmware */
#define TI_5152_BOOT_PRODUCT_ID		0x5152	/* no EEPROM, no firmware */
#define TI_5052_EEPROM_PRODUCT_ID	0x505A	/* EEPROM, no firmware */
#define TI_5052_FIRMWARE_PRODUCT_ID	0x505F	/* firmware is running */
#define FRI2_PRODUCT_ID			0x5053  /* Fish River Island II */

/* Multi-Tech vendor and product ids */
#define MTS_VENDOR_ID			0x06E0
#define MTS_GSM_NO_FW_PRODUCT_ID	0xF108
#define MTS_CDMA_NO_FW_PRODUCT_ID	0xF109
#define MTS_CDMA_PRODUCT_ID		0xF110
#define MTS_GSM_PRODUCT_ID		0xF111
#define MTS_EDGE_PRODUCT_ID		0xF112
#define MTS_MT9234MU_PRODUCT_ID		0xF114
#define MTS_MT9234ZBA_PRODUCT_ID	0xF115
#define MTS_MT9234ZBAOLD_PRODUCT_ID	0x0319

/* Abbott Diabetics vendor and product ids */
#define ABBOTT_VENDOR_ID		0x1a61
#define ABBOTT_STEREO_PLUG_ID		0x3410
#define ABBOTT_PRODUCT_ID		ABBOTT_STEREO_PLUG_ID
#define ABBOTT_STRIP_PORT_ID		0x3420

/* Honeywell vendor and product IDs */
#define HONEYWELL_VENDOR_ID		0x10ac
#define HONEYWELL_HGI80_PRODUCT_ID	0x0102  /* Honeywell HGI80 */

/* Moxa UPORT 11x0 vendor and product IDs */
#define MXU1_VENDOR_ID				0x110a
#define MXU1_TI_VENDOR_ID			0x0451
#define MXU1_1110_PRODUCT_ID			0x1110
#define MXU1_1130_PRODUCT_ID			0x1130
#define MXU1_1150_PRODUCT_ID			0x1150
#define MXU1_1151_PRODUCT_ID			0x1151
#define MXU1_1131_PRODUCT_ID			0x1131
#define MXU1_TI_PRODUCT_ID			0x3410

/* Commands */
#define TI_GET_VERSION			0x01
#define TI_GET_PORT_STATUS		0x02
#define TI_GET_PORT_DEV_INFO		0x03
#define TI_GET_CONFIG			0x04
#define TI_SET_CONFIG			0x05
#define TI_OPEN_PORT			0x06
#define TI_CLOSE_PORT			0x07
#define TI_START_PORT			0x08
#define TI_STOP_PORT			0x09
#define TI_TEST_PORT			0x0A
#define TI_PURGE_PORT			0x0B
#define TI_RESET_EXT_DEVICE		0x0C
#define TI_WRITE_DATA			0x80
#define TI_READ_DATA			0x81
#define TI_REQ_TYPE_CLASS		0x82

/* Module identifiers */
#define TI_I2C_PORT			0x01
#define TI_IEEE1284_PORT		0x02
#define TI_UART1_PORT			0x03
#define TI_UART2_PORT			0x04
#define TI_RAM_PORT			0x05

/* Modem status */
#define TI_MSR_DELTA_CTS		0x01
#define TI_MSR_DELTA_DSR		0x02
#define TI_MSR_DELTA_RI			0x04
#define TI_MSR_DELTA_CD			0x08
#define TI_MSR_CTS			0x10
#define TI_MSR_DSR			0x20
#define TI_MSR_RI			0x40
#define TI_MSR_CD			0x80
#define TI_MSR_DELTA_MASK		0x0F
#define TI_MSR_MASK			0xF0

/* Line status */
#define TI_LSR_OVERRUN_ERROR		0x01
#define TI_LSR_PARITY_ERROR		0x02
#define TI_LSR_FRAMING_ERROR		0x04
#define TI_LSR_BREAK			0x08
#define TI_LSR_ERROR			0x0F
#define TI_LSR_RX_FULL			0x10
#define TI_LSR_TX_EMPTY			0x20

/* Line control */
#define TI_LCR_BREAK			0x40

/* Modem control */
#define TI_MCR_LOOP			0x04
#define TI_MCR_DTR			0x10
#define TI_MCR_RTS			0x20

/* Mask settings */
#define TI_UART_ENABLE_RTS_IN		0x0001
#define TI_UART_DISABLE_RTS		0x0002
#define TI_UART_ENABLE_PARITY_CHECKING	0x0008
#define TI_UART_ENABLE_DSR_OUT		0x0010
#define TI_UART_ENABLE_CTS_OUT		0x0020
#define TI_UART_ENABLE_X_OUT		0x0040
#define TI_UART_ENABLE_XA_OUT		0x0080
#define TI_UART_ENABLE_X_IN		0x0100
#define TI_UART_ENABLE_DTR_IN		0x0800
#define TI_UART_DISABLE_DTR		0x1000
#define TI_UART_ENABLE_MS_INTS		0x2000
#define TI_UART_ENABLE_AUTO_START_DMA	0x4000

/* Parity */
#define TI_UART_NO_PARITY		0x00
#define TI_UART_ODD_PARITY		0x01
#define TI_UART_EVEN_PARITY		0x02
#define TI_UART_MARK_PARITY		0x03
#define TI_UART_SPACE_PARITY		0x04

/* Stop bits */
#define TI_UART_1_STOP_BITS		0x00
#define TI_UART_1_5_STOP_BITS		0x01
#define TI_UART_2_STOP_BITS		0x02

/* Bits per character */
#define TI_UART_5_DATA_BITS		0x00
#define TI_UART_6_DATA_BITS		0x01
#define TI_UART_7_DATA_BITS		0x02
#define TI_UART_8_DATA_BITS		0x03

/* 232/485 modes */
#define TI_UART_232			0x00
#define TI_UART_485_RECEIVER_DISABLED	0x01
#define TI_UART_485_RECEIVER_ENABLED	0x02

/* Pipe transfer mode and timeout */
#define TI_PIPE_MODE_CONTINUOUS		0x01
#define TI_PIPE_MODE_MASK		0x03
#define TI_PIPE_TIMEOUT_MASK		0x7C
#define TI_PIPE_TIMEOUT_ENABLE		0x80

/* Config struct */
struct ti_uart_config {
	__be16	wBaudRate;
	__be16	wFlags;
	u8	bDataBits;
	u8	bParity;
	u8	bStopBits;
	char	cXon;
	char	cXoff;
	u8	bUartMode;
} __packed;

/* Get port status */
struct ti_port_status {
	u8 bCmdCode;
	u8 bModuleId;
	u8 bErrorCode;
	u8 bMSR;
	u8 bLSR;
} __packed;

/* Purge modes */
#define TI_PURGE_OUTPUT			0x00
#define TI_PURGE_INPUT			0x80

/* Read/Write data */
#define TI_RW_DATA_ADDR_SFR		0x10
#define TI_RW_DATA_ADDR_IDATA		0x20
#define TI_RW_DATA_ADDR_XDATA		0x30
#define TI_RW_DATA_ADDR_CODE		0x40
#define TI_RW_DATA_ADDR_GPIO		0x50
#define TI_RW_DATA_ADDR_I2C		0x60
#define TI_RW_DATA_ADDR_FLASH		0x70
#define TI_RW_DATA_ADDR_DSP		0x80

#define TI_RW_DATA_UNSPECIFIED		0x00
#define TI_RW_DATA_BYTE			0x01
#define TI_RW_DATA_WORD			0x02
#define TI_RW_DATA_DOUBLE_WORD		0x04

struct ti_write_data_bytes {
	u8	bAddrType;
	u8	bDataType;
	u8	bDataCounter;
	__be16	wBaseAddrHi;
	__be16	wBaseAddrLo;
	u8	bData[0];
} __packed;

struct ti_read_data_request {
	__u8	bAddrType;
	__u8	bDataType;
	__u8	bDataCounter;
	__be16	wBaseAddrHi;
	__be16	wBaseAddrLo;
} __packed;

struct ti_read_data_bytes {
	__u8	bCmdCode;
	__u8	bModuleId;
	__u8	bErrorCode;
	__u8	bData[0];
} __packed;

/* Interrupt struct */
struct ti_interrupt {
	__u8	bICode;
	__u8	bIInfo;
} __packed;

/* Interrupt codes */
#define TI_CODE_HARDWARE_ERROR		0xFF
#define TI_CODE_DATA_ERROR		0x03
#define TI_CODE_MODEM_STATUS		0x04

/* Download firmware max packet size */
#define TI_DOWNLOAD_MAX_PACKET_SIZE	64

/* Firmware image header */
struct ti_firmware_header {
	__le16	wLength;
	u8	bCheckSum;
} __packed;

/* UART addresses */
#define TI_UART1_BASE_ADDR		0xFFA0	/* UART 1 base address */
#define TI_UART2_BASE_ADDR		0xFFB0	/* UART 2 base address */
#define TI_UART_OFFSET_LCR		0x0002	/* UART MCR register offset */
#define TI_UART_OFFSET_MCR		0x0004	/* UART MCR register offset */

#define TI_DRIVER_AUTHOR	"Al Borchers <alborchers@steinerpoint.com>"
#define TI_DRIVER_DESC		"TI USB 3410/5052 Serial Driver"

#define TI_FIRMWARE_BUF_SIZE	16284

#define TI_TRANSFER_TIMEOUT	2

#if 0	/* FIXME: Victor Yu. 2017/6/22, change value */
#define TI_DEFAULT_CLOSING_WAIT	4000		/* in .01 secs */
#else
#define TI_DEFAULT_CLOSING_WAIT	300		/* in .01 secs */
#endif

/* supported setserial flags */
#define TI_SET_SERIAL_FLAGS	0

/* read urb states */
#define TI_READ_URB_RUNNING	0
#define TI_READ_URB_STOPPING	1
#define TI_READ_URB_STOPPED	2

#define TI_EXTRA_VID_PID_COUNT	5
/* add by Harry */
#define MXU1_MODEL_1110		0x01
#define MXU1_MODEL_1130		0x02
#define MXU1_MODEL_1150		0x03
#define MXU1_MODEL_1151		0x04
#define MXU1_MODEL_1131		0x05
#define MXU1_UART_232                           0x00
#define MXU1_UART_485_RECEIVER_DISABLED         0x01
#define MXU1_UART_485_RECEIVER_ENABLED          0x02
#define MXU1_RS232		0
#define MXU1_RS4852W		1
#define MXU1_RS422		2
#define MXU1_RS4854W		3
/* add by Harry */

struct ti_port {
	int			tp_is_open;
	u8			tp_msr;
	u8			tp_shadow_mcr;
	u8			tp_uart_mode;	/* 232 or 485 modes */
	/* add by Harry */
	u8			tp_user_get_uart_mode;
	/* add by Harry */
	unsigned int		tp_uart_base_addr;
	/* add by Harry */
	int			tp_flags;
	/* add by Harry */
	struct ti_device	*tp_tdev;
	struct usb_serial_port	*tp_port;
	spinlock_t		tp_lock;
	int			tp_read_urb_state;
	int			tp_write_urb_in_use;
#if 0	/* FIXME: Victor Yu. 2017/7/3 */
	/* just for debuging used */
#define DEBUG_ERR_URB	1
	int			err_write_urb_cnt;
	int			err_read_urb_cnt;
#endif
};

struct ti_device {
	struct mutex		td_open_close_lock;
	int			td_open_port_count;
	struct usb_serial	*td_serial;
	int			td_is_3410;
	bool			td_rs485_only;
};

static int ti_startup(struct usb_serial *serial);
static void ti_release(struct usb_serial *serial);
static int ti_port_probe(struct usb_serial_port *port);
static int ti_port_remove(struct usb_serial_port *port);
static int ti_open(struct tty_struct *tty, struct usb_serial_port *port);
static void ti_close(struct usb_serial_port *port);
static int ti_write(struct tty_struct *tty, struct usb_serial_port *port,
		const unsigned char *data, int count);
static int ti_write_room(struct tty_struct *tty);
static int ti_chars_in_buffer(struct tty_struct *tty);
static bool ti_tx_empty(struct usb_serial_port *port);
static void ti_throttle(struct tty_struct *tty);
static void ti_unthrottle(struct tty_struct *tty);
static int ti_ioctl(struct tty_struct *tty,
		unsigned int cmd, unsigned long arg);
static void ti_set_termios(struct tty_struct *tty,
		struct usb_serial_port *port, struct ktermios *old_termios);
static int ti_tiocmget(struct tty_struct *tty);
static int ti_tiocmset(struct tty_struct *tty,
		unsigned int set, unsigned int clear);
static void ti_break(struct tty_struct *tty, int break_state);
static void ti_interrupt_callback(struct urb *urb);
static void ti_bulk_in_callback(struct urb *urb);
static void ti_bulk_out_callback(struct urb *urb);

static void ti_recv(struct usb_serial_port *port, unsigned char *data,
		int length);
static void ti_send(struct ti_port *tport);
static int ti_set_mcr(struct ti_port *tport, unsigned int mcr);
static int ti_get_lsr(struct ti_port *tport, u8 *lsr);
static int ti_get_serial_info(struct ti_port *tport,
	struct serial_struct __user *ret_arg);
static int ti_set_serial_info(struct tty_struct *tty, struct ti_port *tport,
	struct serial_struct __user *new_arg);
static void ti_handle_new_msr(struct ti_port *tport, u8 msr);

static void ti_stop_read(struct ti_port *tport, struct tty_struct *tty);
static int ti_restart_read(struct ti_port *tport, struct tty_struct *tty);

static int ti_command_out_sync(struct ti_device *tdev, __u8 command,
	__u16 moduleid, __u16 value, __u8 *data, int size);
static int ti_command_in_sync(struct ti_device *tdev, __u8 command,
	__u16 moduleid, __u16 value, __u8 *data, int size);

static int ti_write_byte(struct usb_serial_port *port, struct ti_device *tdev,
			 unsigned long addr, u8 mask, u8 byte);

static int ti_download_firmware(struct ti_device *tdev);

static int closing_wait = TI_DEFAULT_CLOSING_WAIT;

static const struct usb_device_id ti_id_table_3410[] = {
	{ USB_DEVICE(TI_VENDOR_ID, TI_3410_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_3410_EZ430_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_GSM_NO_FW_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_CDMA_NO_FW_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_CDMA_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_GSM_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_EDGE_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_MT9234MU_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_MT9234ZBA_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_MT9234ZBAOLD_PRODUCT_ID) },
	{ USB_DEVICE(IBM_VENDOR_ID, IBM_4543_PRODUCT_ID) },
	{ USB_DEVICE(IBM_VENDOR_ID, IBM_454B_PRODUCT_ID) },
	{ USB_DEVICE(IBM_VENDOR_ID, IBM_454C_PRODUCT_ID) },
	{ USB_DEVICE(ABBOTT_VENDOR_ID, ABBOTT_STEREO_PLUG_ID) },
	{ USB_DEVICE(ABBOTT_VENDOR_ID, ABBOTT_STRIP_PORT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, FRI2_PRODUCT_ID) },
	{ USB_DEVICE(HONEYWELL_VENDOR_ID, HONEYWELL_HGI80_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1110_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1130_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1131_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1150_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1151_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_TI_VENDOR_ID, MXU1_TI_PRODUCT_ID) },
	{ }	/* terminator */
};

static const struct usb_device_id ti_id_table_5052[] = {
	{ USB_DEVICE(TI_VENDOR_ID, TI_5052_BOOT_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5152_BOOT_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5052_EEPROM_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5052_FIRMWARE_PRODUCT_ID) },
	{ }
};

static const struct usb_device_id ti_id_table_combined[] = {
	{ USB_DEVICE(TI_VENDOR_ID, TI_3410_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_3410_EZ430_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_GSM_NO_FW_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_CDMA_NO_FW_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_CDMA_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_GSM_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_EDGE_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_MT9234MU_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_MT9234ZBA_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_MT9234ZBAOLD_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5052_BOOT_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5152_BOOT_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5052_EEPROM_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5052_FIRMWARE_PRODUCT_ID) },
	{ USB_DEVICE(IBM_VENDOR_ID, IBM_4543_PRODUCT_ID) },
	{ USB_DEVICE(IBM_VENDOR_ID, IBM_454B_PRODUCT_ID) },
	{ USB_DEVICE(IBM_VENDOR_ID, IBM_454C_PRODUCT_ID) },
	{ USB_DEVICE(ABBOTT_VENDOR_ID, ABBOTT_PRODUCT_ID) },
	{ USB_DEVICE(ABBOTT_VENDOR_ID, ABBOTT_STRIP_PORT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, FRI2_PRODUCT_ID) },
	{ USB_DEVICE(HONEYWELL_VENDOR_ID, HONEYWELL_HGI80_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1110_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1130_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1131_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1150_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1151_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_TI_VENDOR_ID, MXU1_TI_PRODUCT_ID) },
	{ }	/* terminator */
};

/* FIXME: Victor Yu. 2017/6/7 */
#if 0
static void
ti_init_termios(struct tty_struct *tty)
{
	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	tty->termios = tty_std_termios;
}
#endif

static struct usb_serial_driver ti_1port_device = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "ti_usb_3410_5052_1",
	},
	.description		= "TI USB 3410 1 port adapter",
	.id_table		= ti_id_table_3410,
	.num_ports		= 1,
	.attach			= ti_startup,
	.release		= ti_release,
	.port_probe		= ti_port_probe,
	.port_remove		= ti_port_remove,
	.open			= ti_open,
	.close			= ti_close,
	.write			= ti_write,
	.write_room		= ti_write_room,
	.chars_in_buffer	= ti_chars_in_buffer,
	.tx_empty		= ti_tx_empty,
	.throttle		= ti_throttle,
	.unthrottle		= ti_unthrottle,
	.ioctl			= ti_ioctl,
	.set_termios		= ti_set_termios,
	.tiocmget		= ti_tiocmget,
	.tiocmset		= ti_tiocmset,
	.tiocmiwait		= usb_serial_generic_tiocmiwait,
	.get_icount		= usb_serial_generic_get_icount,
	.break_ctl		= ti_break,
	.read_int_callback	= ti_interrupt_callback,
	.read_bulk_callback	= ti_bulk_in_callback,
	.write_bulk_callback	= ti_bulk_out_callback,
#if 0	/* FIXME: Victor Yu. 2017/6/7 */
	.init_termios		= ti_init_termios,
#endif
};

static struct usb_serial_driver ti_2port_device = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "ti_usb_3410_5052_2",
	},
	.description		= "TI USB 5052 2 port adapter",
	.id_table		= ti_id_table_5052,
	.num_ports		= 2,
	.attach			= ti_startup,
	.release		= ti_release,
	.port_probe		= ti_port_probe,
	.port_remove		= ti_port_remove,
	.open			= ti_open,
	.close			= ti_close,
	.write			= ti_write,
	.write_room		= ti_write_room,
	.chars_in_buffer	= ti_chars_in_buffer,
	.tx_empty		= ti_tx_empty,
	.throttle		= ti_throttle,
	.unthrottle		= ti_unthrottle,
	.ioctl			= ti_ioctl,
	.set_termios		= ti_set_termios,
	.tiocmget		= ti_tiocmget,
	.tiocmset		= ti_tiocmset,
	.tiocmiwait		= usb_serial_generic_tiocmiwait,
	.get_icount		= usb_serial_generic_get_icount,
	.break_ctl		= ti_break,
	.read_int_callback	= ti_interrupt_callback,
	.read_bulk_callback	= ti_bulk_in_callback,
	.write_bulk_callback	= ti_bulk_out_callback,
#if 0	/* FIXME: Victor Yu. 2017/6/7 */
	.init_termios		= ti_init_termios,
#endif
};

static struct usb_serial_driver * const serial_drivers[] = {
	&ti_1port_device, &ti_2port_device, NULL
};

MODULE_AUTHOR(TI_DRIVER_AUTHOR);
MODULE_DESCRIPTION(TI_DRIVER_DESC);
MODULE_LICENSE("GPL");

MODULE_FIRMWARE("ti_3410.fw");
MODULE_FIRMWARE("ti_5052.fw");
MODULE_FIRMWARE("mts_cdma.fw");
MODULE_FIRMWARE("mts_gsm.fw");
MODULE_FIRMWARE("mts_edge.fw");
MODULE_FIRMWARE("mts_mt9234mu.fw");
MODULE_FIRMWARE("mts_mt9234zba.fw");
MODULE_FIRMWARE("moxa/moxa-1110.fw");
MODULE_FIRMWARE("moxa/moxa-1130.fw");
MODULE_FIRMWARE("moxa/moxa-1131.fw");
MODULE_FIRMWARE("moxa/moxa-1150.fw");
MODULE_FIRMWARE("moxa/moxa-1151.fw");

module_param(closing_wait, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(closing_wait,
    "Maximum wait for data to drain in close, in .01 secs, default is 4000");

MODULE_DEVICE_TABLE(usb, ti_id_table_combined);

module_usb_serial_driver(serial_drivers, ti_id_table_combined);

static int ti_startup(struct usb_serial *serial)
{
	struct ti_device *tdev;
	struct usb_device *dev = serial->dev;
	struct usb_host_interface *cur_altsetting;
	int num_endpoints;
	u16 vid, pid;
	int status;

	dgprintk(HIGH_DEBUG_LEVEL, "num_ports=%d\n", serial->num_ports);
	dev_dbg(&dev->dev,
		"%s - product 0x%4X, num configurations %d, configuration value %d\n",
		__func__, le16_to_cpu(dev->descriptor.idProduct),
		dev->descriptor.bNumConfigurations,
		dev->actconfig->desc.bConfigurationValue);

	tdev = kzalloc(sizeof(struct ti_device), GFP_KERNEL);
	if (!tdev)
		return -ENOMEM;

	mutex_init(&tdev->td_open_close_lock);
	tdev->td_serial = serial;
	usb_set_serial_data(serial, tdev);

	/* determine device type */
	if (serial->type == &ti_1port_device)
		tdev->td_is_3410 = 1;
	dev_dbg(&dev->dev, "%s - device type is %s\n", __func__,
		tdev->td_is_3410 ? "3410" : "5052");

	vid = le16_to_cpu(dev->descriptor.idVendor);
	pid = le16_to_cpu(dev->descriptor.idProduct);
	if (vid == MXU1_VENDOR_ID) {
		switch (pid) {
		case MXU1_1130_PRODUCT_ID:
		case MXU1_1131_PRODUCT_ID:
			tdev->td_rs485_only = true;
			break;
		}
	}

	cur_altsetting = serial->interface->cur_altsetting;
	num_endpoints = cur_altsetting->desc.bNumEndpoints;

	/* if we have only 1 configuration and 1 endpoint, download firmware */
	if (dev->descriptor.bNumConfigurations == 1 && num_endpoints == 1) {
		dgprintk(HIGH_DEBUG_LEVEL, "\n");
		status = ti_download_firmware(tdev);

		if (status != 0)
			goto free_tdev;

		/* 3410 must be reset, 5052 resets itself */
		if (tdev->td_is_3410) {
			msleep_interruptible(100);
			usb_reset_device(dev);
		}

		status = -ENODEV;
		goto free_tdev;
	}

	/* the second configuration must be set */
	if (dev->actconfig->desc.bConfigurationValue == TI_BOOT_CONFIG) {
		dgprintk(HIGH_DEBUG_LEVEL, "\n");
		status = usb_driver_set_configuration(dev, TI_ACTIVE_CONFIG);
		status = status ? status : -ENODEV;
		goto free_tdev;
	}

	return 0;

free_tdev:
	kfree(tdev);
	usb_set_serial_data(serial, NULL);
	return status;
}


static void ti_release(struct usb_serial *serial)
{
	struct ti_device *tdev = usb_get_serial_data(serial);

	dgprintk(HIGH_DEBUG_LEVEL, "\n");

#if 1	/* FIXME: Victor Yu. 2017/6/9, just protect it */
	usb_set_serial_data(serial, NULL);
#endif

	kfree(tdev);
}

static int ti_port_probe(struct usb_serial_port *port)
{
	struct ti_port *tport;

	dgprintk(HIGH_DEBUG_LEVEL, "port->port_number=%d\n", port->port_number);
	tport = kzalloc(sizeof(*tport), GFP_KERNEL);
	if (!tport)
		return -ENOMEM;

	spin_lock_init(&tport->tp_lock);
	if (port == port->serial->port[0])
		tport->tp_uart_base_addr = TI_UART1_BASE_ADDR;
	else
		tport->tp_uart_base_addr = TI_UART2_BASE_ADDR;
	port->port.closing_wait = msecs_to_jiffies(10 * closing_wait);
	tport->tp_port = port;
	tport->tp_tdev = usb_get_serial_data(port->serial);

	if (tport->tp_tdev->td_rs485_only)
		tport->tp_uart_mode = TI_UART_485_RECEIVER_DISABLED;
	else
		tport->tp_uart_mode = TI_UART_232;

	usb_set_serial_port_data(port, tport);
#if 0	/* FIXME: Victor Yu. 2017/6/21, change to more time */
	port->port.drain_delay = 3;
#else
	port->port.drain_delay = 128;
#endif

#ifdef DEBUG_ERR_URB	
	/* FIXME: Victor Yu. 2017/6/8, add it just for debugging */
	tport->err_write_urb_cnt = 0;
	tport->err_read_urb_cnt = 0;
#endif

	return 0;
}

static int ti_port_remove(struct usb_serial_port *port)
{
	struct ti_port *tport;

	dgprintk(HIGH_DEBUG_LEVEL, "\n");
	tport = usb_get_serial_port_data(port);

#if 1	/* FIXME: Victor Yu. 2017/6/9, just protect it */
	usb_set_serial_port_data(port, NULL);
#endif

	kfree(tport);

	return 0;
}

static int ti_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct ti_port *tport = usb_get_serial_port_data(port);
	struct ti_device *tdev;
	struct usb_device *dev;
	struct urb *urb;
	int port_number;
	int status;
	u16 open_settings;

	dgprintk(NORMAL_DEBUG_LEVEL, "tty->index=%d,port=%d\n", 
		tty->index, port->port_number);
	open_settings = (TI_PIPE_MODE_CONTINUOUS |
			 TI_PIPE_TIMEOUT_ENABLE |
			 (TI_TRANSFER_TIMEOUT << 2));

	dev = port->serial->dev;
	tdev = tport->tp_tdev;

	/* only one open on any port on a device at a time */
	if (mutex_lock_interruptible(&tdev->td_open_close_lock))
		return -ERESTARTSYS;

#if 1	/* FIXME: Victor Yu. 2017/6/2, I should be done first time opened */
	if (tdev->td_open_port_count > 0) {
		dgprintk(RAW_DEBUG_LEVEL, "not first time open\n");
		goto open_again;
	}
#endif
	dgprintk(RAW_DEBUG_LEVEL, "first time open\n");

	/* FIXME: Victor Yu. 2017/6/2, I should be done first time opened */
	port_number = port->port_number;
	tport->tp_msr = 0;
	tport->tp_shadow_mcr |= (TI_MCR_RTS | TI_MCR_DTR);

	/* start interrupt urb the first time a port is opened on this device */
	/* FIXME: Victor Yu. 2017/6/7 */
	/* above has checked it is first time opening */
	/*if (tdev->td_open_port_count == 0) { */
		dev_dbg(&port->dev, "%s - start interrupt in urb\n", __func__);
		urb = tdev->td_serial->port[0]->interrupt_in_urb;
		if (!urb) {
			dev_err(&port->dev, "%s - no interrupt urb\n", __func__);
			status = -EINVAL;
			goto release_lock;
		}
		urb->context = tdev;
		status = usb_submit_urb(urb, GFP_KERNEL);
		if (status) {
			dev_err(&port->dev, 
				"%s - submit interrupt urb failed, %d\n", 
				__func__, status);
			goto release_lock;
		}
	/*} */

#if 0	/* FIXME: Victor Yu. masked it 2917/6/21 */
	/* TODO: why before setting USB port-open do this ? */
	/* could I do it after port-open ? Victor Yu. 2017/6/21 */
	if (tty)
		ti_set_termios(tty, port, &tty->termios);

	status = ti_command_out_sync(tdev, TI_OPEN_PORT,
		(__u8)(TI_UART1_PORT + port_number), open_settings, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot send open command, %d\n",
			__func__, status);
		goto unlink_int_urb;
	}

	status = ti_command_out_sync(tdev, TI_START_PORT,
		(__u8)(TI_UART1_PORT + port_number), 0, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot send start command, %d\n",
							__func__, status);
		goto unlink_int_urb;
	}

	status = ti_command_out_sync(tdev, TI_PURGE_PORT,
		(__u8)(TI_UART1_PORT + port_number), TI_PURGE_INPUT, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot clear input buffers, %d\n",
							__func__, status);
		goto unlink_int_urb;
	}
	status = ti_command_out_sync(tdev, TI_PURGE_PORT,
		(__u8)(TI_UART1_PORT + port_number), TI_PURGE_OUTPUT, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot clear output buffers, %d\n",
							__func__, status);
		goto unlink_int_urb;
	}
#endif

	/* reset the data toggle on the bulk endpoints to work around bug in
	 * host controllers where things get out of sync some times */
#if 0	/* FIXME: Victor Yu. 2017/6/8, mask it just for debugging */
	/* I make sure these actions will let the receiving lost data
	 * at the beginning */
	usb_clear_halt(dev, port->write_urb->pipe);
	usb_clear_halt(dev, port->read_urb->pipe);
#endif

	/* FIXME: Victor Yu. 2017/6/2, 
	 * why set termios,open,start port again ? */
#if 1	/* Victor Yu. 2017/6/2 */
	if (tty)
		ti_set_termios(tty, port, &tty->termios);
#endif

	status = ti_command_out_sync(tdev, TI_OPEN_PORT,
		(__u8)(TI_UART1_PORT + port_number), open_settings, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot send open command (2), %d\n",
							__func__, status);
		goto unlink_int_urb;
	}

	status = ti_command_out_sync(tdev, TI_START_PORT,
		(__u8)(TI_UART1_PORT + port_number), 0, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot send start command (2), %d\n",
							__func__, status);
		goto unlink_int_urb;
	}

#if 1	/* FIXME: Victor Yu. added 2017/6/2 */
	status = ti_command_out_sync(tdev, TI_PURGE_PORT,
		(__u8)(TI_UART1_PORT + port_number), TI_PURGE_OUTPUT, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot clear output buffers, %d\n",
			__func__, status);
		goto unlink_int_urb;
	}
	status = ti_command_out_sync(tdev, TI_PURGE_PORT,
		(__u8)(TI_UART1_PORT + port_number), TI_PURGE_INPUT, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot clear input buffers, %d\n",
			__func__, status);
		goto unlink_int_urb;
	}
#endif

	/* start read urb */
	urb = port->read_urb;
	if (!urb) {
		dev_err(&port->dev, "%s - no read urb\n", __func__);
		status = -EINVAL;
		goto unlink_int_urb;
	}
	tport->tp_read_urb_state = TI_READ_URB_RUNNING;
	urb->context = tport;
	status = usb_submit_urb(urb, GFP_KERNEL);
	if (status) {
		dev_err(&port->dev, "%s - submit read urb failed, %d\n",
			__func__, status);
		goto unlink_int_urb;
	}

	tport->tp_is_open = 1;

open_again:
	++tdev->td_open_port_count;
	dgprintk(RAW_DEBUG_LEVEL, "open finished, tport->tp_shadow_mcr=0x%x\n",
		tport->tp_shadow_mcr);
	goto release_lock;

unlink_int_urb:
	if (tdev->td_open_port_count == 0)
		usb_kill_urb(port->serial->port[0]->interrupt_in_urb);

release_lock:
	mutex_unlock(&tdev->td_open_close_lock);
	return status;
}


static void ti_close(struct usb_serial_port *port)
{
	struct ti_device *tdev;
	struct ti_port *tport;
	int port_number;
	int status;
	int do_unlock;
	unsigned long flags;

	dgprintk(NORMAL_DEBUG_LEVEL, "port->port.tty->index=%d\n", 
		port->port.tty->index);
	tdev = usb_get_serial_data(port->serial);
	tport = usb_get_serial_port_data(port);

#if 0	/* FIXME: Victor Yu. 2017/6/2,
	 * I think it should be done the last close */
	tport->tp_is_open = 0;

	usb_kill_urb(port->read_urb);
	usb_kill_urb(port->write_urb);
	tport->tp_write_urb_in_use = 0;
	spin_lock_irqsave(&tport->tp_lock, flags);
	kfifo_reset_out(&port->write_fifo);
	spin_unlock_irqrestore(&tport->tp_lock, flags);

	port_number = port->port_number;

	status = ti_command_out_sync(tdev, TI_CLOSE_PORT,
		     (__u8)(TI_UART1_PORT + port_number), 0, NULL, 0);
	if (status)
		dev_err(&port->dev,
			"%s - cannot send close port command, %d\n"
			, __func__, status);

#endif
	/* if mutex_lock is interrupted, continue anyway */
	/* FIXME: Victor Yu. 2017/6/2 */
	/* here continue is right ? If it is opened at this time, 
	 * then the count may be error */
#if 0	/* Victor Yu. 2017/6/2 */
	do_unlock = !mutex_lock_interruptible(&tdev->td_open_close_lock);
#else
	while (mutex_lock_interruptible(&tdev->td_open_close_lock))
		msleep_interruptible(100);
#endif
	--tport->tp_tdev->td_open_port_count;
	if (tport->tp_tdev->td_open_port_count <= 0) {
		dgprintk(RAW_DEBUG_LEVEL, "\n");
		/* last port is closed, shut down interrupt urb */
		tport->tp_tdev->td_open_port_count = 0;

#if 1	/* FIXME: Victor Yu. 2017/6/2,
	 * I think it should be done the last close */
//#define LATE_KILL_URB	1
	#ifndef LATE_KILL_URB	/* FIXME: Victor Yu. 2017/6/26 */
		usb_kill_urb(port->serial->port[0]->interrupt_in_urb);
		usb_kill_urb(port->read_urb);
		usb_kill_urb(port->write_urb);
	#endif
		spin_lock_irqsave(&tport->tp_lock, flags);
		tport->tp_is_open = 0;
		tport->tp_write_urb_in_use = 0;
		tport->tp_read_urb_state = TI_READ_URB_STOPPED;
		kfifo_reset_out(&port->write_fifo);
		spin_unlock_irqrestore(&tport->tp_lock, flags);

		port_number = port->port_number;
		status = ti_command_out_sync(tdev, TI_CLOSE_PORT,
			(__u8)(TI_UART1_PORT + port_number), 0, NULL, 0);
		if (status)
			dev_err(&port->dev,
				"%s - cannot send close port command, %d\n"
				, __func__, status);
	#ifdef LATE_KILL_URB	/* FIXME: Victor Yu. 2017/6/23 */
	/* I found these actions do after close port will be more stable */
	/* But I don't know why ? I just found it will let xHCI host */
	/* controller TRB queue maintain more stable */
		usb_kill_urb(port->serial->port[0]->interrupt_in_urb);
		usb_kill_urb(port->read_urb);
		usb_kill_urb(port->write_urb);
		msleep(300);
	#endif
#endif
	}
#if 0	/* FIXME: Victor Yu. 2017/6/22 */
	if (do_unlock)
#endif
		mutex_unlock(&tdev->td_open_close_lock);
	dgprintk(RAW_DEBUG_LEVEL, "\n");
}


static int ti_write(struct tty_struct *tty, struct usb_serial_port *port,
			const unsigned char *data, int count)
{
	struct ti_port *tport = usb_get_serial_port_data(port);

	dgprintk(NORMAL_DEBUG_LEVEL, "tty->index=%d,count=%d\n", 
		tty->index, count);
	if (count == 0) {
		return 0;
	}

	if (!tport->tp_is_open)
		return -ENODEV;

	count = kfifo_in_locked(&port->write_fifo, data, count,
					&tport->tp_lock);
	dgprintk(RAW_DEBUG_LEVEL, "write in fifo=%d\n", count);
	ti_send(tport);

	return count;
}


static int ti_write_room(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ti_port *tport = usb_get_serial_port_data(port);
	int room = 0;
	unsigned long flags;

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	spin_lock_irqsave(&tport->tp_lock, flags);
	room = kfifo_avail(&port->write_fifo);
	spin_unlock_irqrestore(&tport->tp_lock, flags);

	dev_dbg(&port->dev, "%s - returns %d\n", __func__, room);
	return room;
}


static int ti_chars_in_buffer(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ti_port *tport = usb_get_serial_port_data(port);
	int chars = 0;
	unsigned long flags;

	spin_lock_irqsave(&tport->tp_lock, flags);
	chars = kfifo_len(&port->write_fifo);
	spin_unlock_irqrestore(&tport->tp_lock, flags);
	dgprintk(NORMAL_DEBUG_LEVEL, "tty->index=%d,in bytes=%d\n", 
		tty->index, chars);

	dev_dbg(&port->dev, "%s - returns %d\n", __func__, chars);
	return chars;
}

static bool ti_tx_empty(struct usb_serial_port *port)
{
	struct ti_port *tport = usb_get_serial_port_data(port);
	int ret;
	u8 lsr;

	ret = ti_get_lsr(tport, &lsr);
	if (!ret && !(lsr & TI_LSR_TX_EMPTY)) {
		dgprintk(NORMAL_DEBUG_LEVEL, "tx is not empty\n");
		return false;
	}

	dgprintk(NORMAL_DEBUG_LEVEL, "tx is empty\n");
	return true;
}

static void ti_throttle(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ti_port *tport = usb_get_serial_port_data(port);

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	if (I_IXOFF(tty) || C_CRTSCTS(tty))
		ti_stop_read(tport, tty);

}


static void ti_unthrottle(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ti_port *tport = usb_get_serial_port_data(port);
	int status;

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	if (I_IXOFF(tty) || C_CRTSCTS(tty)) {
		status = ti_restart_read(tport, tty);
		if (status)
			dev_err(&port->dev, "%s - cannot restart read, %d\n",
							__func__, status);
	}
}

static int ti_ioctl(struct tty_struct *tty,
	unsigned int cmd, unsigned long arg)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ti_port *tport = usb_get_serial_port_data(port);

	dgprintk(NORMAL_DEBUG_LEVEL, "tty->index=%d\n", tty->index);
	switch (cmd) {
	case TIOCGSERIAL:
		dgprintk(RAW_DEBUG_LEVEL, "TIOCGSERIAL\n");
		return ti_get_serial_info(tport,
				(struct serial_struct __user *)arg);
	case TIOCSSERIAL:
		dgprintk(RAW_DEBUG_LEVEL, "TIOCSSERIAL\n");
		return ti_set_serial_info(tty, tport,
				(struct serial_struct __user *)arg);
	}
	dgprintk(RAW_DEBUG_LEVEL, "unkonwn command\n");
	return -ENOIOCTLCMD;
}

/*
 * @brief set the serial port communication configuration
 */
static void ti_set_termios(struct tty_struct *tty,
		struct usb_serial_port *port, struct ktermios *old_termios)
{
	struct ti_port *tport = usb_get_serial_port_data(port);
	struct ti_uart_config *config;
	tcflag_t cflag, iflag;
	int baud;
	int status;
	int port_number = port->port_number;
	unsigned int mcr;
	u16 wbaudrate;
	u16 wflags = 0;

	dgprintk(NORMAL_DEBUG_LEVEL, "tty->index=%d\n", tty->index);
	cflag = tty->termios.c_cflag;
	iflag = tty->termios.c_iflag;

	dev_dbg(&port->dev, "%s - cflag %08x, iflag %08x\n", __func__, 
		cflag, iflag);
	dev_dbg(&port->dev, "%s - old clfag %08x, old iflag %08x\n", __func__,
		old_termios->c_cflag, old_termios->c_iflag);

	config = kmalloc(sizeof(*config), GFP_KERNEL);
	if (!config)
		return;

	/* these flags must be set */
	wflags |= TI_UART_ENABLE_MS_INTS;
	wflags |= TI_UART_ENABLE_AUTO_START_DMA;
	config->bUartMode = tport->tp_uart_mode;

	switch (C_CSIZE(tty)) {
	case CS5:
		    config->bDataBits = TI_UART_5_DATA_BITS;
		    break;
	case CS6:
		    config->bDataBits = TI_UART_6_DATA_BITS;
		    break;
	case CS7:
		    config->bDataBits = TI_UART_7_DATA_BITS;
		    break;
	default:
	case CS8:
		    config->bDataBits = TI_UART_8_DATA_BITS;
		    break;
	}

	/* CMSPAR isn't supported by this driver */
	tty->termios.c_cflag &= ~CMSPAR;

	if (C_PARENB(tty)) {
		if (C_PARODD(tty)) {
			wflags |= TI_UART_ENABLE_PARITY_CHECKING;
			config->bParity = TI_UART_ODD_PARITY;
		} else {
			wflags |= TI_UART_ENABLE_PARITY_CHECKING;
			config->bParity = TI_UART_EVEN_PARITY;
		}
	} else {
		wflags &= ~TI_UART_ENABLE_PARITY_CHECKING;
		config->bParity = TI_UART_NO_PARITY;
	}

	if (C_CSTOPB(tty))
		config->bStopBits = TI_UART_2_STOP_BITS;
	else
		config->bStopBits = TI_UART_1_STOP_BITS;

	if (C_CRTSCTS(tty)) {
		/* RTS flow control must be off to drop RTS for baud rate B0 */
		if ((C_BAUD(tty)) != B0)
			wflags |= TI_UART_ENABLE_RTS_IN;
		wflags |= TI_UART_ENABLE_CTS_OUT;
	} else {
		ti_restart_read(tport, tty);
	}

	if (I_IXOFF(tty) || I_IXON(tty)) {
		config->cXon  = START_CHAR(tty);
		config->cXoff = STOP_CHAR(tty);

		/* TODO: Victor Yu. 2017/6/8, here is something wrong */
		if (I_IXOFF(tty))
			wflags |= TI_UART_ENABLE_X_IN;
		else
			ti_restart_read(tport, tty);

		if (I_IXON(tty))
			wflags |= TI_UART_ENABLE_X_OUT;
	}

	baud = tty_get_baud_rate(tty);
	dgprintk(RAW_DEBUG_LEVEL, "buadrate is %d\n", baud);
	if (!baud)
		baud = 9600;
	if (tport->tp_tdev->td_is_3410)
		wbaudrate = (923077 + baud/2) / baud;
	else
		wbaudrate = (461538 + baud/2) / baud;

	/* FIXME: Should calculate resulting baud here and report it back */
	if ((C_BAUD(tty)) != B0)
		tty_encode_baud_rate(tty, baud, baud);

	dev_dbg(&port->dev,
		"%s - BaudRate=%d, wBaudRate=%d, wFlags=0x%04X, bDataBits=%d, bParity=%d, bStopBits=%d, cXon=%d, cXoff=%d, bUartMode=%d\n",
		__func__, baud, wbaudrate, wflags,
		config->bDataBits, config->bParity, config->bStopBits,
		config->cXon, config->cXoff, config->bUartMode);

	config->wBaudRate = cpu_to_be16(wbaudrate);
	config->wFlags = cpu_to_be16(wflags);

	status = ti_command_out_sync(tport->tp_tdev, TI_SET_CONFIG,
		(__u8)(TI_UART1_PORT + port_number), 0, (__u8 *)config,
		sizeof(*config));
	/* FIXME: Victor Yu. 2017/6/2 */
	/* It should be stoped when the status has error ? */
	if (status)
		dev_err(&port->dev, "%s - cannot set config on port %d, %d\n",
					__func__, port_number, status);

	/* SET_CONFIG asserts RTS and DTR, reset them correctly */
	mcr = tport->tp_shadow_mcr;
	/* if baud rate is B0, clear RTS and DTR */
#if 0	/* FIXME: Victor Yu. 2017/6/8, why ? I don't understand. */
	/* so I remove it */
	if (C_BAUD(tty) == B0)
		mcr &= ~(TI_MCR_DTR | TI_MCR_RTS);
#endif
	dgprintk(RAW_DEBUG_LEVEL, "tport->tp_shadow_mcr=0x%x\n", 
		tport->tp_shadow_mcr);
	status = ti_set_mcr(tport, mcr);
	/* FIXME: Victor Yu. 2017/6/2 */
	/* It should be stoped when the status has error ? */
	if (status)
		dev_err(&port->dev,
			"%s - cannot set modem control on port %d, %d\n",
						__func__, port_number, status);

	kfree(config);
}

static int ti_tiocmget(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ti_port *tport = usb_get_serial_port_data(port);
	unsigned int result;
	unsigned int msr;
	unsigned int mcr;
	unsigned long flags;

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	/* FIXME: Victor Yu. 2017/6/2 */
	/* I don't think it needs spin_lock_irqsave */
	/* spin_lock_irqsave(&tport->tp_lock, flags); */
	msr = tport->tp_msr;
	mcr = tport->tp_shadow_mcr;
	/*spin_unlock_irqrestore(&tport->tp_lock, flags);*/

	result = ((mcr & TI_MCR_DTR) ? TIOCM_DTR : 0)
		| ((mcr & TI_MCR_RTS) ? TIOCM_RTS : 0)
		| ((mcr & TI_MCR_LOOP) ? TIOCM_LOOP : 0)
		| ((msr & TI_MSR_CTS) ? TIOCM_CTS : 0)
		| ((msr & TI_MSR_CD) ? TIOCM_CAR : 0)
		| ((msr & TI_MSR_RI) ? TIOCM_RI : 0)
		| ((msr & TI_MSR_DSR) ? TIOCM_DSR : 0);

	dev_dbg(&port->dev, "%s - 0x%04X\n", __func__, result);

	return result;
}


static int ti_tiocmset(struct tty_struct *tty,
				unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ti_port *tport = usb_get_serial_port_data(port);
	unsigned int mcr;
	unsigned long flags;

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	/* FIXME: Victor Yu. 2017/6/2 */
	/* I don't think it needs spin_lock_irqsave */
	/* spin_lock_irqsave(&tport->tp_lock, flags); */
	mcr = tport->tp_shadow_mcr;

	if (set & TIOCM_RTS)
		mcr |= TI_MCR_RTS;
	if (set & TIOCM_DTR)
		mcr |= TI_MCR_DTR;
	if (set & TIOCM_LOOP)
		mcr |= TI_MCR_LOOP;

	if (clear & TIOCM_RTS)
		mcr &= ~TI_MCR_RTS;
	if (clear & TIOCM_DTR)
		mcr &= ~TI_MCR_DTR;
	if (clear & TIOCM_LOOP)
		mcr &= ~TI_MCR_LOOP;
	/* spin_unlock_irqrestore(&tport->tp_lock, flags); */

	return ti_set_mcr(tport, mcr);
}


static void ti_break(struct tty_struct *tty, int break_state)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ti_port *tport = usb_get_serial_port_data(port);
	int status;

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	dev_dbg(&port->dev, "%s - state = %d\n", __func__, break_state);

	status = ti_write_byte(port, tport->tp_tdev,
		tport->tp_uart_base_addr + TI_UART_OFFSET_LCR,
		TI_LCR_BREAK, break_state == -1 ? TI_LCR_BREAK : 0);

	if (status)
		dev_dbg(&port->dev, "%s - error setting break, %d\n", __func__, status);
}

static int ti_get_port_from_code(unsigned char code)
{
	return (code >> 4) - 3;
}

static int ti_get_func_from_code(unsigned char code)
{
	return code & 0x0f;
}

static void ti_interrupt_callback(struct urb *urb)
{
	struct ti_device *tdev = urb->context;
	struct usb_serial_port *port;
	struct usb_serial *serial = tdev->td_serial;
	struct ti_port *tport;
	struct device *dev = &urb->dev->dev;
	unsigned char *data = urb->transfer_buffer;
	int length = urb->actual_length;
	int port_number;
	int function;
	int status = urb->status;
	int retval;
	u8 msr;

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	switch (status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		dgprintk(RAW_DEBUG_LEVEL, "shutdown error !\n");
		dev_dbg(dev, "%s - urb shutting down, %d\n", __func__, status);
		return;
	default:
		dgprintk(RAW_DEBUG_LEVEL, "others error !\n");
		dev_err(dev, "%s - nonzero urb status, %d\n", __func__, status);
		goto exit;
	}

	if (length != 2) {
		dgprintk(RAW_DEBUG_LEVEL, "bad packet size !\n");
		dev_dbg(dev, "%s - bad packet size, %d\n", __func__, length);
		goto exit;
	}

	if (data[0] == TI_CODE_HARDWARE_ERROR) {
		dgprintk(RAW_DEBUG_LEVEL, "code hardwar eerror !\n");
		dev_err(dev, "%s - hardware error, %d\n", __func__, data[1]);
		goto exit;
	}

	port_number = ti_get_port_from_code(data[0]);
	function = ti_get_func_from_code(data[0]);

	dgprintk(RAW_DEBUG_LEVEL, "\n");
	dev_dbg(dev, "%s - port_number %d, function %d, data 0x%02X\n",
		__func__, port_number, function, data[1]);

	if (port_number >= serial->num_ports) {
		dgprintk(RAW_DEBUG_LEVEL, "\n");
		dev_err(dev, "%s - bad port number, %d\n",
				__func__, port_number);
		goto exit;
	}

	port = serial->port[port_number];
	dgprintk(RAW_DEBUG_LEVEL, "port->port.tty->index=%d\n", 
		port->port.tty->index);

	tport = usb_get_serial_port_data(port);
	if (!tport) {
		dgprintk(RAW_DEBUG_LEVEL, "\n");
		goto exit;
	}

	switch (function) {
	case TI_CODE_DATA_ERROR:
		dgprintk(RAW_DEBUG_LEVEL, "code data error !\n");
		dev_warn(dev, "%s - DATA ERROR, port %d, data 0x%02X\n",
			__func__, port_number, data[1]);
		break;

	case TI_CODE_MODEM_STATUS:
		msr = data[1];
		dgprintk(RAW_DEBUG_LEVEL, "code modem status,msr=0x%02x\n", msr);
		dev_dbg(dev, "%s - port %d, msr 0x%02X\n", __func__, 
			port_number, msr);
		ti_handle_new_msr(tport, msr);
		break;

	default:
		dgprintk(RAW_DEBUG_LEVEL, "unknown interrupt code !\n");
		dev_err(dev, "%s - unknown interrupt code, 0x%02X\n",
			__func__, data[1]);
		break;
	}

exit:
	dgprintk(RAW_DEBUG_LEVEL, "submit interrupt urb again\n");
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		dev_err(dev, "%s - resubmit interrupt urb failed, %d\n",
			__func__, retval);
}


static void ti_bulk_in_callback(struct urb *urb)
{
	struct ti_port *tport = urb->context;
	struct usb_serial_port *port = tport->tp_port;
	struct device *dev = &urb->dev->dev;
	int status = urb->status;

	dgprintk(NORMAL_DEBUG_LEVEL, "port->port.tty->index=%d,urb->actual_length=%d\n", port->port.tty->index, urb->actual_length);
	switch (status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		dgprintk(RAW_DEBUG_LEVEL, "shutdown error !\n");
		dev_dbg(dev, "%s - urb shutting down, %d\n", __func__, status);
		return;
	default:
		dgprintk(RAW_DEBUG_LEVEL, "others error !\n");
		dev_err(dev, "%s - nonzero urb status, %d\n",
			__func__, status);
	}

	if (status == -EPIPE) {
		dgprintk(RAW_DEBUG_LEVEL, "pipe error !\n");
		goto exit;
	}

	if (status) {
		dgprintk(RAW_DEBUG_LEVEL, "stopping read !\n");
		dev_err(dev, "%s - stopping read!\n", __func__);
		return;
	}

	if (urb->actual_length) {
		usb_serial_debug_data(dev, __func__, urb->actual_length,
				      urb->transfer_buffer);
#ifdef VICTOR_DEBUG
		if ( debug_level <= ALL_DEBUG_LEVEL ) {
			int __c;
			unsigned char *__buf;
			for ( __c=0,__buf=urb->transfer_buffer;
				__c<urb->actual_length; __c++, __buf++ )
				printk("%c", *__buf);
			printk("\n");
		}
#endif

		if (!tport->tp_is_open) {
			dgprintk(RAW_DEBUG_LEVEL, "port is closed !\n");
			dev_dbg(dev, "%s - port closed, dropping data\n",
				__func__);
#if 1	/* FIXME: Victor Yu. 2017/6/22 */
			return;
#endif
		} else
			ti_recv(port, urb->transfer_buffer, urb->actual_length);
		/* FIXME: I don't think it needs to be locked */
		/* spin_lock(&tport->tp_lock); */
		port->icount.rx += urb->actual_length;
		/* spin_unlock(&tport->tp_lock); */
	}

exit:
	/* continue to read unless stopping */
	/* FIXME: Victor Yu. 2017/6/5 */
	/* here should spin_lock_irqsave, becuase the others do it ? */
	{
	unsigned long flags;
	/* spin_lock(&tport->tp_lock); */
	spin_lock_irqsave(&tport->tp_lock, flags);
#if 1	/* FIXME: Victor Yu. 2017/6/22 */
	if ( tport->tp_is_open == 0 ) {
		spin_unlock_irqrestore(&tport->tp_lock, flags);
		return;
	}
#endif
	if (tport->tp_read_urb_state == TI_READ_URB_RUNNING) {
		spin_unlock_irqrestore(&tport->tp_lock, flags);
		status = usb_submit_urb(urb, GFP_ATOMIC);
#if 1	/* FIXME: Victor Yu. 2017/6/9, move here */
		if (status) {
			dev_err(dev, "%s - resubmit read urb failed, %d\n",
				__func__, status);
#ifdef DEBUG_ERR_URB
	/* FIXME: Victor Yu. 2017/7/3, just for debuging */
			myprintk("port %d,read err count=%d\n", 
				tport->tp_port->port.tty->index,
				++tport->err_read_urb_cnt);
#endif
		}
#endif
		return;
	} else if (tport->tp_read_urb_state == TI_READ_URB_STOPPING) {
		tport->tp_read_urb_state = TI_READ_URB_STOPPED;
	}

	/*spin_unlock(&tport->tp_lock);*/
	spin_unlock_irqrestore(&tport->tp_lock, flags);
	}

#if 0	/* FIXME: Victor Yu. 2017/6/9, move above */
	if (retval)
		dev_err(dev, "%s - resubmit read urb failed, %d\n",
			__func__, retval);
#endif
}


static void ti_bulk_out_callback(struct urb *urb)
{
	struct ti_port *tport = urb->context;
	struct usb_serial_port *port = tport->tp_port;
	int status = urb->status;

	dgprintk(RAW_DEBUG_LEVEL, "port->port.tty->index=%d\n",
		port->port.tty->index);
	/* FIXME: Victor YU/ 2017/6/5 */
	/* Here should spin_locked it ? */
	{
	unsigned long flags;
	spin_lock_irqsave(&tport->tp_lock, flags);
	tport->tp_write_urb_in_use = 0;
	spin_unlock_irqrestore(&tport->tp_lock, flags);
	}

	switch (status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		dgprintk(RAW_DEBUG_LEVEL, "shutdown error !\n");
		dev_dbg(&port->dev, "%s - urb shutting down, %d\n", __func__, 
			status);
		return;
	default:
		/* TODO; Victor Yu. 2017/6/8, here should we need to do
		 * something ? */
		dgprintk(RAW_DEBUG_LEVEL, "others error !\n");
		dev_err_console(port, "%s - nonzero urb status, %d\n",
			__func__, status);
	}

	/* send any buffered data */
	ti_send(tport);
}


static void ti_recv(struct usb_serial_port *port, unsigned char *data,
		int length)
{
	int cnt;

	dgprintk(RAW_DEBUG_LEVEL, "port->port.tty->index=%d,length=%d\n",
		port->port.tty->index, length);
	do {
		cnt = tty_insert_flip_string(&port->port, data, length);
		if (cnt < length) {
			dev_err(&port->dev, 
				"%s - dropping data, %d bytes lost\n", 
				__func__, length - cnt);
			if (cnt == 0)
				break;
		}
		tty_flip_buffer_push(&port->port);
		data += cnt;
		length -= cnt;
	} while (length > 0);
#if 1	/* Just debuging, Victor Yu. 2017/6/8 */
	if ( length > 0 )
		dgprintk(RAW_DEBUG_LEVEL, "receive not completed=%d\n", length);
#endif
}


static void ti_send(struct ti_port *tport)
{
	int count, result;
	struct usb_serial_port *port = tport->tp_port;
	unsigned long flags;

	dgprintk(RAW_DEBUG_LEVEL, "port->port.tty->index=%d\n",
		port->port.tty->index);
	spin_lock_irqsave(&tport->tp_lock, flags);
#if 1	/* FIXME: Victor Yu. 2017/6/22 */
	if ( tport->tp_is_open == 0 )
		goto unlock;
#endif

	if (tport->tp_write_urb_in_use) {
		dgprintk(RAW_DEBUG_LEVEL, "write urb in use\n");
		goto unlock;
	}

	count = kfifo_out(&port->write_fifo,
				port->write_urb->transfer_buffer,
				port->bulk_out_size);

	if (count == 0) {
		dgprintk(RAW_DEBUG_LEVEL, "no data to be send !\n");
		goto unlock;
	}

	tport->tp_write_urb_in_use = 1;

	spin_unlock_irqrestore(&tport->tp_lock, flags);

	usb_serial_debug_data(&port->dev, __func__, count,
			      port->write_urb->transfer_buffer);

	usb_fill_bulk_urb(port->write_urb, port->serial->dev,
			   usb_sndbulkpipe(port->serial->dev,
					    port->bulk_out_endpointAddress),
			   port->write_urb->transfer_buffer, count,
			   ti_bulk_out_callback, tport);

	result = usb_submit_urb(port->write_urb, GFP_ATOMIC);
	if (result) {
		dgprintk(RAW_DEBUG_LEVEL, "submit write urb fail !\n");
		dev_err_console(port, "%s - submit write urb failed, %d\n",
					__func__, result);
#ifdef DEBUG_ERR_URB
	/* FIXME: Victor Yu. 2017/7/3, just for debuging */
		myprintk("port %d,write err count=%d\n", 
			tport->tp_port->port.tty->index,
			++tport->err_write_urb_cnt);
#endif
		/* FIXME: Victor Yu. 2017/6/5 */
		/* Here should spin_locked ? */
		spin_lock_irqsave(&tport->tp_lock, flags);
		tport->tp_write_urb_in_use = 0;
		spin_unlock_irqrestore(&tport->tp_lock, flags);
		/* TODO: reschedule ti_send */
	} else {
		dgprintk(RAW_DEBUG_LEVEL, "submit write urb count=%d\n",
			count);
		/* FIXME: I don't think it needs to be locked */
		/* spin_lock_irqsave(&tport->tp_lock, flags); */
		port->icount.tx += count;
		/* spin_unlock_irqrestore(&tport->tp_lock, flags); */
	}

	/* more room in the buffer for new writes, wakeup */
	tty_port_tty_wakeup(&port->port);

	return;

unlock:
	spin_unlock_irqrestore(&tport->tp_lock, flags);
	return;
}


static int ti_set_mcr(struct ti_port *tport, unsigned int mcr)
{
	unsigned long flags;
	int status;

	dgprintk(NORMAL_DEBUG_LEVEL, "mcr=0x%x\n", mcr);
	status = ti_write_byte(tport->tp_port, tport->tp_tdev,
		tport->tp_uart_base_addr + TI_UART_OFFSET_MCR,
		TI_MCR_RTS | TI_MCR_DTR | TI_MCR_LOOP, mcr);

	/* FIXME: Victor Yu. 2017/6/2 */
	/* I don't think it needs spin_lock_irqsave */
	/* spin_lock_irqsave(&tport->tp_lock, flags); */
	if (!status)
		tport->tp_shadow_mcr = mcr;
	/* spin_unlock_irqrestore(&tport->tp_lock, flags); */
	dgprintk(RAW_DEBUG_LEVEL, "tport->tp_shadow_mcr=0x%x\n", 
		tport->tp_shadow_mcr);

	return status;
}


static int ti_get_lsr(struct ti_port *tport, u8 *lsr)
{
	int size, status;
	struct ti_device *tdev = tport->tp_tdev;
	struct usb_serial_port *port = tport->tp_port;
	int port_number = port->port_number;
	struct ti_port_status *data;

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	size = sizeof(struct ti_port_status);
	data = kmalloc(size, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	status = ti_command_in_sync(tdev, TI_GET_PORT_STATUS,
		(__u8)(TI_UART1_PORT+port_number), 0, (__u8 *)data, size);
	if (status) {
		dev_err(&port->dev,
			"%s - get port status command failed, %d\n",
			__func__, status);
		goto free_data;
	}

	dev_dbg(&port->dev, "%s - lsr 0x%02X\n", __func__, data->bLSR);

	*lsr = data->bLSR;

free_data:
	kfree(data);
	return status;
}


static int ti_get_serial_info(struct ti_port *tport,
	struct serial_struct __user *ret_arg)
{
	struct usb_serial_port *port = tport->tp_port;
	struct serial_struct ret_serial;
	unsigned cwait;

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	if (!ret_arg)
		return -EFAULT;

	cwait = port->port.closing_wait;
	if (cwait != ASYNC_CLOSING_WAIT_NONE)
		cwait = jiffies_to_msecs(cwait) / 10;

	memset(&ret_serial, 0, sizeof(ret_serial));

	ret_serial.type = PORT_16550A;
	ret_serial.line = port->minor;
	ret_serial.port = port->port_number;
	/* Add by Harry */
	ret_serial.port = tport->tp_user_get_uart_mode;
	/* Add by Harry */
	ret_serial.xmit_fifo_size = kfifo_size(&port->write_fifo);
	ret_serial.baud_base = tport->tp_tdev->td_is_3410 ? 921600 : 460800;
	ret_serial.closing_wait = cwait;

	if (copy_to_user(ret_arg, &ret_serial, sizeof(*ret_arg)))
		return -EFAULT;

	return 0;
}

//#define SET_INTERFACE_BY_GPIO	1
static int ti_set_serial_info(struct tty_struct *tty, struct ti_port *tport,
	struct serial_struct __user *new_arg)
{
	struct serial_struct new_serial;
	unsigned cwait;

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	if (copy_from_user(&new_serial, new_arg, sizeof(new_serial)))
		return -EFAULT;

	/* add by Harry */
	tport->tp_flags = new_serial.flags & TI_SET_SERIAL_FLAGS;
	/* add by Harry */

	cwait = new_serial.closing_wait;
	if (cwait != ASYNC_CLOSING_WAIT_NONE)
		cwait = msecs_to_jiffies(10 * new_serial.closing_wait);

	tport->tp_port->port.closing_wait = cwait;

	/* add by Harry */	
	if(true){ //UPort 1130, 1150, 1150I
		switch(new_serial.port){
			case MXU1_RS232: //UPort 1150, 1150I
				dgprintk(NORMAL_DEBUG_LEVEL, "RS232\n");
				if(tport->tp_tdev->td_is_3410 == MXU1_MODEL_1130 || tport->tp_tdev->td_is_3410 == MXU1_MODEL_1131){
					return -EINVAL;
				}
#ifndef SET_INTERFACE_BY_GPIO	/* FIXME: Victor Yu. 2017/6/9, UC-8580 set interface by GPIO */
				tport->tp_uart_mode = MXU1_UART_232;
#endif
				tport->tp_user_get_uart_mode = MXU1_RS232;
				break;
			case MXU1_RS422: //UPort 1130, 1150, 1150I
				dgprintk(NORMAL_DEBUG_LEVEL, "RS422\n");
#ifndef SET_INTERFACE_BY_GPIO	/* FIXME: Victor Yu. 2017/6/9, UC-8580 set interface by GPIO */
				tport->tp_uart_mode = MXU1_UART_485_RECEIVER_ENABLED;
#endif
				tport->tp_user_get_uart_mode = MXU1_RS422;
				break;
			case MXU1_RS4854W: //UPort 1130, 1150, 1150I
				dgprintk(NORMAL_DEBUG_LEVEL, "RS4854W\n");
#ifndef SET_INTERFACE_BY_GPIO	/* FIXME: Victor Yu. 2017/6/9, UC-8580 set interface by GPIO */
				tport->tp_uart_mode = MXU1_UART_485_RECEIVER_ENABLED;
#endif
				tport->tp_user_get_uart_mode = MXU1_RS4854W;
				break;
			case MXU1_RS4852W: //UPort 1130, 1150, 1150I
				dgprintk(NORMAL_DEBUG_LEVEL, "RS4852W\n");
#ifndef SET_INTERFACE_BY_GPIO	/* FIXME: Victor Yu. 2017/6/9, UC-8580 set interface by GPIO */
				tport->tp_uart_mode = MXU1_UART_485_RECEIVER_DISABLED;
#endif
				tport->tp_user_get_uart_mode = MXU1_RS4852W;
				break;
			default:
				dgprintk(NORMAL_DEBUG_LEVEL, "unknown error\n");
			        return -EINVAL;
                }
        }else{ //UPort 1110
                if(new_serial.port != MXU1_RS232) {
			dgprintk(NORMAL_DEBUG_LEVEL, "just support RS232 !\n");
                        return -EINVAL;
		}
        }
	/* add by Harry */

#ifndef SET_INTERFACE_BY_GPIO	/* FIXME: Victor Yu. 2017/6/9, UC-8580 set interface by GPIO */
	if (tty)
		ti_set_termios(tty, tport->tp_port, &tty->termios);
#endif

	return 0;
}


static void ti_handle_new_msr(struct ti_port *tport, u8 msr)
{
	struct async_icount *icount;
	struct tty_struct *tty;
	unsigned long flags;

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	dev_dbg(&tport->tp_port->dev, "%s - msr 0x%02X\n", __func__, msr);

	if (msr & TI_MSR_DELTA_MASK) {
		/* FIXME: Victor Yu. 2017/6/2 */
		/* I don't think it needs spin_lock_irqsave here */
		/*spin_lock_irqsave(&tport->tp_lock, flags);*/
		icount = &tport->tp_port->icount;
		if (msr & TI_MSR_DELTA_CTS)
			icount->cts++;
		if (msr & TI_MSR_DELTA_DSR)
			icount->dsr++;
		if (msr & TI_MSR_DELTA_CD)
			icount->dcd++;
		if (msr & TI_MSR_DELTA_RI)
			icount->rng++;
		wake_up_interruptible(&tport->tp_port->port.delta_msr_wait);
		/*spin_unlock_irqrestore(&tport->tp_lock, flags);*/
	}

	tport->tp_msr = msr & TI_MSR_MASK;

	/* handle CTS flow control */
	tty = tty_port_tty_get(&tport->tp_port->port);
	if (tty && C_CRTSCTS(tty)) {
		if (msr & TI_MSR_CTS)
			tty_wakeup(tty);
	}
	tty_kref_put(tty);
}


static void ti_stop_read(struct ti_port *tport, struct tty_struct *tty)
{
	unsigned long flags;

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	spin_lock_irqsave(&tport->tp_lock, flags);

	if (tport->tp_read_urb_state == TI_READ_URB_RUNNING)
		tport->tp_read_urb_state = TI_READ_URB_STOPPING;

	spin_unlock_irqrestore(&tport->tp_lock, flags);
}


static int ti_restart_read(struct ti_port *tport, struct tty_struct *tty)
{
	struct urb *urb;
	int status = 0;
	unsigned long flags;

	spin_lock_irqsave(&tport->tp_lock, flags);
#if 1	/* FIXME: Victor Yu. 2017/6/22 */
	if ( tport->tp_is_open == 0 ) {
		spin_unlock_irqrestore(&tport->tp_lock, flags);
		return status;
	}
#endif

	if (tport->tp_read_urb_state == TI_READ_URB_STOPPED) {
		tport->tp_read_urb_state = TI_READ_URB_RUNNING;
		urb = tport->tp_port->read_urb;
		spin_unlock_irqrestore(&tport->tp_lock, flags);
		urb->context = tport;
		status = usb_submit_urb(urb, GFP_KERNEL);
		if (status) {
			dev_err(&tport->tp_port->dev, 
				"%s - resubmit read urb failed, %d\n",
				__func__, status);
#ifdef DEBUG_ERR_URB
	/* FIXME: Victor Yu. 2017/7/3, just for debuging */
			myprintk("port %d,read err count=%d\n", 
				tport->tp_port->port.tty->index,
				++tport->err_read_urb_cnt);
#endif
		}
		dgprintk(NORMAL_DEBUG_LEVEL, "restart reading\n");
	} else  {
		tport->tp_read_urb_state = TI_READ_URB_RUNNING;
		spin_unlock_irqrestore(&tport->tp_lock, flags);
		dgprintk(NORMAL_DEBUG_LEVEL, "continue reading\n");
	}

	return status;
}


static int ti_command_out_sync(struct ti_device *tdev, __u8 command,
	__u16 moduleid, __u16 value, __u8 *data, int size)
{
	int status;

	status = usb_control_msg(tdev->td_serial->dev,
		usb_sndctrlpipe(tdev->td_serial->dev, 0), command,
		(USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT),
		value, moduleid, data, size, 1000);

	if (status == size)
		status = 0;

	if (status > 0)
		status = -ECOMM;

	dgprintk(RAW_DEBUG_LEVEL, "return status=%d\n", status);
	return status;
}


static int ti_command_in_sync(struct ti_device *tdev, __u8 command,
	__u16 moduleid, __u16 value, __u8 *data, int size)
{
	int status;

	status = usb_control_msg(tdev->td_serial->dev,
		usb_rcvctrlpipe(tdev->td_serial->dev, 0), command,
		(USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN),
		value, moduleid, data, size, 1000);

	if (status == size)
		status = 0;

	if (status > 0)
		status = -ECOMM;

	dgprintk(NORMAL_DEBUG_LEVEL, "return status=%d\n", status);
	return status;
}

/*
 * @brief write a byte to usb-to-serial port hardware register
 */
static int ti_write_byte(struct usb_serial_port *port,
			 struct ti_device *tdev, unsigned long addr,
			 u8 mask, u8 byte)
{
	int status;
	unsigned int size;
	struct ti_write_data_bytes *data;

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	dev_dbg(&port->dev, "%s - addr 0x%08lX, mask 0x%02X, byte 0x%02X\n", 
		__func__, addr, mask, byte);

	size = sizeof(struct ti_write_data_bytes) + 2;
	data = kmalloc(size, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->bAddrType = TI_RW_DATA_ADDR_XDATA;
	data->bDataType = TI_RW_DATA_BYTE;
	data->bDataCounter = 1;
	data->wBaseAddrHi = cpu_to_be16(addr>>16);
	data->wBaseAddrLo = cpu_to_be16(addr);
	data->bData[0] = mask;
	data->bData[1] = byte;

	status = ti_command_out_sync(tdev, TI_WRITE_DATA, TI_RAM_PORT, 0,
		(__u8 *)data, size);

	if (status < 0)
		dev_err(&port->dev, "%s - failed, %d\n", __func__, status);

	kfree(data);

	return status;
}

static int ti_do_download(struct usb_device *dev, int pipe,
						u8 *buffer, int size)
{
	int pos;
	u8 cs = 0;
	int done;
	struct ti_firmware_header *header;
	int status = 0;
	int len;

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	for (pos = sizeof(struct ti_firmware_header); pos < size; pos++)
		cs = (u8)(cs + buffer[pos]);

	header = (struct ti_firmware_header *)buffer;
	header->wLength = cpu_to_le16(size - sizeof(*header));
	header->bCheckSum = cs;

	dev_dbg(&dev->dev, "%s - downloading firmware\n", __func__);
	for (pos = 0; pos < size; pos += done) {
		len = min(size - pos, TI_DOWNLOAD_MAX_PACKET_SIZE);
		status = usb_bulk_msg(dev, pipe, buffer + pos, len,
								&done, 1000);
		if (status)
			break;
	}
	return status;
}

static int ti_download_firmware(struct ti_device *tdev)
{
	int status;
	int buffer_size;
	u8 *buffer;
	struct usb_device *dev = tdev->td_serial->dev;
	unsigned int pipe = usb_sndbulkpipe(dev,
		tdev->td_serial->port[0]->bulk_out_endpointAddress);
	const struct firmware *fw_p;
	char buf[32];

	dgprintk(NORMAL_DEBUG_LEVEL, "\n");
	if ((le16_to_cpu(dev->descriptor.idVendor) == MXU1_VENDOR_ID) || (le16_to_cpu(dev->descriptor.idVendor) == MXU1_TI_VENDOR_ID)) {
		snprintf(buf,
			sizeof(buf),
			"moxa/moxa-%04x.fw",
			le16_to_cpu(dev->descriptor.idProduct));

		status = request_firmware(&fw_p, buf, &dev->dev);
		goto check_firmware;
	}

	/* try ID specific firmware first, then try generic firmware */
	sprintf(buf, "ti_usb-v%04x-p%04x.fw",
			le16_to_cpu(dev->descriptor.idVendor),
			le16_to_cpu(dev->descriptor.idProduct));
	status = request_firmware(&fw_p, buf, &dev->dev);

	if (status != 0) {
		buf[0] = '\0';
		if (le16_to_cpu(dev->descriptor.idVendor) == MTS_VENDOR_ID) {
			switch (le16_to_cpu(dev->descriptor.idProduct)) {
			case MTS_CDMA_PRODUCT_ID:
				strcpy(buf, "mts_cdma.fw");
				break;
			case MTS_GSM_PRODUCT_ID:
				strcpy(buf, "mts_gsm.fw");
				break;
			case MTS_EDGE_PRODUCT_ID:
				strcpy(buf, "mts_edge.fw");
				break;
			case MTS_MT9234MU_PRODUCT_ID:
				strcpy(buf, "mts_mt9234mu.fw");
				break;
			case MTS_MT9234ZBA_PRODUCT_ID:
				strcpy(buf, "mts_mt9234zba.fw");
				break;
			case MTS_MT9234ZBAOLD_PRODUCT_ID:
				strcpy(buf, "mts_mt9234zba.fw");
				break;			}
		}
		if (buf[0] == '\0') {
			if (tdev->td_is_3410)
				strcpy(buf, "ti_3410.fw");
			else
				strcpy(buf, "ti_5052.fw");
		}
		status = request_firmware(&fw_p, buf, &dev->dev);
	}

check_firmware:
	if (status) {
		dev_err(&dev->dev, "%s - firmware not found\n", __func__);
		return -ENOENT;
	}
	if (fw_p->size > TI_FIRMWARE_BUF_SIZE) {
		dev_err(&dev->dev, "%s - firmware too large %zu\n", __func__, fw_p->size);
		release_firmware(fw_p);
		return -ENOENT;
	}

	buffer_size = TI_FIRMWARE_BUF_SIZE + sizeof(struct ti_firmware_header);
	buffer = kmalloc(buffer_size, GFP_KERNEL);
	if (buffer) {
		memcpy(buffer, fw_p->data, fw_p->size);
		memset(buffer + fw_p->size, 0xff, buffer_size - fw_p->size);
		status = ti_do_download(dev, pipe, buffer, fw_p->size);
		kfree(buffer);
	} else {
		status = -ENOMEM;
	}
	release_firmware(fw_p);
	if (status) {
		dev_err(&dev->dev, "%s - error downloading firmware, %d\n",
							__func__, status);
		return status;
	}

	dev_dbg(&dev->dev, "%s - download successful\n", __func__);

	return 0;
}

