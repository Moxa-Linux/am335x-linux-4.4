/* Copyright (C) MOXA Inc. All rights reserved.

   This is free software distributed under the terms of the
   GNU Public License.  See the file COPYING-GPL for details.
 */
/*
 *   mx-uport.h - MOXA UPort series drvier definitions
 *
 */
#ifndef __MX_USB_SERIAL_H
#define __MX_USB_SERIAL_H

/*
 *  Definitions for driver info
 */
#include "mx_ver.h"
#define DRIVER_AUTHOR               "Danny Lin <danny.lin@moxa.com>"
#define DRIVER_DESC                 "MOXA UPort series driver"
#define DRIVER_LICENSE              "GPL"

/*
 * Definitions for the vendor ID and device ID 
 */
#define MX_USBSERIAL_VID	        0x110A
#define MX_UPORT1250_PID	        0x1250
#define MX_UPORT1251_PID	        0x1251
#define MX_UPORT1410_PID	        0x1410
#define MX_UPORT1450_PID	        0x1450
#define MX_UPORT1451_PID	        0x1451
#define MX_UPORT1618_PID	        0x1618
#define MX_UPORT1658_PID	        0x1658
#define MX_UPORT1613_PID	        0x1613
#define MX_UPORT1653_PID	        0x1653

/*
 *  Definitions for USB info
 */
#define HEADER_SIZE                 4
#define MAX_EVENT_LENGTH            8
#define MAX_NAME_LEN		    64
#define DOWN_BLOCK_SIZE             64
#define TTY_THRESHOLD_THROTTLE      128
#define TRIGGER_SEND_NEXT           4096
#define MAX_QUEUE_SIZE              (1024 * 32) 
#define HIGH_WATER_SIZE             (1024 * 5) 
#define LOW_WATER_SIZE              (1024 * 2) 
#define WAIT_MSR_TIMEOUT            (5*HZ)

/*
 *  Definitions for firmware info
 */
#define VER_ADDR_1                  0x20              
#define VER_ADDR_2                  0x24
#define VER_ADDR_3                  0x28

/*
 *  Definitions for USB vendor request
 */
#define RQ_VENDOR_NONE                      0x00
#define RQ_VENDOR_SET_BAUD                  0x01	/* set baudrate */
#define RQ_VENDOR_SET_LINE                  0x02 	/* set line status */
#define RQ_VENDOR_SET_CHARS                 0x03 	/* set Xon/Xoff chars */
#define RQ_VENDOR_SET_RTS                   0x04	/* set RTS */
#define RQ_VENDOR_SET_DTR                   0x05	/* set DTR */
#define RQ_VENDOR_SET_XONXOFF               0x06	/* set auto Xon/Xoff */
#define RQ_VENDOR_SET_RX_HOST_EN            0x07	/* set RX host enable */
#define RQ_VENDOR_SET_OPEN                  0x08	/* set open/close port */
#define RQ_VENDOR_PURGE                     0x09	/* purge Rx/Tx buffer */
#define RQ_VENDOR_SET_MCR                   0x0A	/* set MCR register */
#define RQ_VENDOR_SET_BREAK                 0x0B	/* set Break signal */

#define RQ_VENDOR_START_FW_DOWN             0x0C	/* start firmware download */
#define RQ_VENDOR_STOP_FW_DOWN              0x0D	/* stop firmware download */
#define RQ_VENDOR_QUERY_FW_READY            0x0E	/* query if new firmware ready */

#define RQ_VENDOR_SET_FIFO_DISABLE          0x0F	/* set fifo disable */
#define RQ_VENDOR_SET_INTERFACE             0x10	/* set interface */
#define RQ_VENDOR_SET_HIGH_PERFOR           0x11	/* set hi-performance */

#define RQ_VENDOR_ERASE_BLOCK               0x12	/* erase flash block */
#define RQ_VENDOR_WRITE_PAGE                0x13	/* write flash page */
#define RQ_VENDOR_PREPARE_WRITE             0x14	/* prepare write flash */
#define RQ_VENDOR_CONFIRM_WRITE             0x15	/* confirm write flash */
#define RQ_VENDOR_LOCATE                    0x16	/* locate the device */

#define RQ_VENDOR_START_ROM_DOWN            0x17	/* start firmware download */
#define RQ_VENDOR_ROM_DATA                  0x18	/* rom file data */
#define RQ_VENDOR_STOP_ROM_DOWN             0x19	/* stop firmware download */
#define RQ_VENDOR_FW_DATA                   0x20    /* firmware data */

#define RQ_VENDOR_GET_VERSION               0x81	/* get firmware version */
#define RQ_VENDOR_GET_PAGE                  0x82	/* read flash page */
#define RQ_VENDOR_GET_ROM_PROC              0x83	/* get ROM process state */

#define RQ_VENDOR_GET_INQUEUE               0x84    /* get data remaining in the input buffer */
#define RQ_VENDOR_GET_OUTQUEUE              0x85    /* get data remaining in the output buffer */

#define RQ_VENDOR_RESET_DEVICE              0x23
#define RQ_VENDOR_QUERY_FW_CONFIG           0x24
#define RQ_VENDOR_GET_MSR                   0x86

/*
 *  Definitions for UPort event type
 */ 
#define	UPORT_EVENT_NONE                    0       /* None */
#define	UPORT_EVNET_TXBUF_THRESHOLD         1       /* Tx buffer threshold */
#define	UPORT_EVNET_SEND_NEXT               2       /* Send next */
#define	UPORT_EVNET_MSR	                    3       /* Modem status */
#define	UPORT_EVNET_LSR	                    4       /* Line status */
#define	UPORT_EVNET_MCR                     5       /* Modem control */

/*
 *  Definitions for serial event type
 */ 
#define	SERIAL_EV_CTS		        0x0008  /* CTS changed state */
#define	SERIAL_EV_DSR		        0x0010  /* DSR changed state */
#define	SERIAL_EV_RLSD		        0x0020  /* RLSD changed state */

/*
 *  Definitions for line control of communication
 */
#define	MX_WORDLENGTH_5             5
#define	MX_WORDLENGTH_6             6
#define	MX_WORDLENGTH_7             7
#define	MX_WORDLENGTH_8             8

#define	MX_PARITY_NONE              0
#define	MX_PARITY_ODD               1
#define	MX_PARITY_EVEN              2
#define	MX_PARITY_MARK              3
#define	MX_PARITY_SPACE             4

#define	MX_STOP_BITS_1              0
#define	MX_STOP_BITS_1_5            1
#define	MX_STOP_BITS_2              2

#define MX_NO_FLOWCTRL              0x0
#define MX_HW_FLOWCTRL              0x1
#define MX_XON_FLOWCTRL             0x2
#define MX_XOFF_FLOWCTRL	    0x4

#define MX_INT_RS232                0
#define MX_INT_2W_RS485             1
#define MX_INT_RS422                2
#define MX_INT_4W_RS485             3

#define MX_START_CHAR               0x11
#define MX_STOP_CHAR                0x13

/*
 *  Definitions for holding reason
 */
#define MX_WAIT_FOR_CTS             0x0001
#define MX_WAIT_FOR_DSR             0x0002
#define MX_WAIT_FOR_DCD             0x0004
#define MX_WAIT_FOR_XON             0x0008
#define MX_WAIT_FOR_START_TX        0x0010
#define MX_WAIT_FOR_UNTHROTTLE      0x0020
#define MX_WAIT_FOR_LOW_WATER       0x0040
#define MX_WAIT_FOR_SEND_NEXT       0x0080

/*
 *  Definitions for vendor I/O control code
 */
#define MOXA                        0x404
#define MOXA_SET_INTERFACE          (MOXA + 1)
#define MOXA_SET_SPECIAL_BAUD       (MOXA + 2)

/*
 *  Definitions for fitting some kernel change prototype
 */

#define DBG_BURNIN_DATA		0	

/* 
 *  This structure holds all of the individual device information 
 */
struct mxuport_serial
{
	__u16               productid;              /* product ID */
	char                dev_name[MAX_NAME_LEN];	/* string name of this device */

	int					bulkout_size;
	int					open_cnt;
	__u8				write_bulkout_addr;     /* bulk out address for write data */
	__u8                read_bulkin_addr;       /* bulk in address for read data */
	__u8                event_bulkin_addr;      /* bulk in address for receive event */

	struct mxusb_serial   *serial;                /* loop back to the owner of this object */
	struct semaphore open_close_sem;
};

/* 
 *  This structure holds all of the local port information 
 */
struct mxuport_port
{
	int                     portno;             /* indicate the actual port number */
	int                     opened;             /* port open status */
	int						flags;              /* for async_struct and serial_struct flags field */
	int                     send_len;           /* for trigger SEND-NEXT length */
	int			set_B0;
	int			open_cnt;

	__u8			mcr_state;	    /* last MCR state */
	__u8			msr_state;	    /* last MSR state */
	__u8		        lsr_state;          /* last LSR state */

#if DBG_BURNIN_DATA
	int	                txerr;          /* use for debug burnin data */ 
	int 	                txcnt;          /* use for debug burnin data */ 
#endif	

	unsigned long           hold_reason;        
	unsigned long           comm_err;           /* for line status error */

	struct async_icount	    icount;

	struct urb		*write_urb;			/* write URB for this port */
	struct mxusb_serial_port	*port;			    /* loop back to the owner of this object */
	struct mx_uart_config   *uart_cfg;          /* configuration of UART */
	struct mx_queue         *write_q;           /* transimit buffer */
	struct mx_queue         *read_q;            /* receive buffer */

	atomic_t                write_in_progress;  /* flag for current wirte state */
	atomic_t                read_in_progress;   /* flag for current read state */

	atomic_t                wait_msr;           /* flag for waiting MSR event */
	wait_queue_head_t	    delta_msr_wait;		/* waiting for MSR change */

	spinlock_t		        port_splock;
	spinlock_t		        read_splock;
	spinlock_t		        write_splock;
	spinlock_t			cnt_splock;
	unsigned char ch;
	unsigned char ch1;
	int			type;		/* UART type */
};

/*
 *  Configuration of UART
 */
struct mx_uart_config	
{
	long  wBaudRate;	/* baud rate                        */
	__u8  bDataBits;	/* 5..8 - data bits per character   */
	__u8  bParity;		/* parity settings                  */
	__u8  bStopBits;	/* stop bits settings               */
	__u8  bFlowCtrl;    /* flow control settings            */
	char  cXon;		    /* XON character                    */
	char  cXoff;		/* XOFF character                   */
	int  interface;	/* interface is defined             */
}; 

/*
 *  Structure of circular queue
 */
struct mx_queue
{
	int           portno;       /* owner of this queue */
	atomic_t      add_sn;       /* flag for requesting SEND-NEXT */
	atomic_t      qfull;        /* flag for determining queue is full */
	unsigned int  size;         /* queue size */
	unsigned char *front;       /* front of queue */
	unsigned char *rear;        /* rear of queue */
	unsigned char *buffer;      /* buffer of queue */
};    

/*
 *  Function prototypes for the usbserial callbacks
 */
static int mxuport_port_probe(struct mxusb_serial_port *port);
static int mxuport_port_remove(struct mxusb_serial_port *port);
static int  mxuport_startup(struct mxusb_serial *serial);
//static void mxuport_shutdown(struct mxusb_serial *serial);
#if 1
static int mxuport_open(struct tty_struct *tty, struct mxusb_serial_port *port);
static void mxuport_close(struct mxusb_serial_port *port);
static int mxuport_tiocmset (struct tty_struct *tty, unsigned int set, unsigned int clear);
static int mxuport_tiocmget(struct tty_struct *tty);
static void mxuport_break_ctl (struct tty_struct *tty, int break_state);
static int mxuport_chars_in_buffer(struct tty_struct *tty);
static int mxuport_write_room (struct tty_struct *tty);
static void mxuport_throttle (struct tty_struct *tty);
static void mxuport_unthrottle (struct tty_struct *tty);
static int mxuport_ioctl (struct tty_struct *tty, unsigned int cmd, unsigned long arg);
static int mxuport_write(struct tty_struct *tty, struct mxusb_serial_port *port, const unsigned char *data, int count);
static void mxuport_set_termios (struct tty_struct *tty, struct mxusb_serial_port *port, struct ktermios *old);
#else
static int  mxuport_open(struct mxusb_serial_port *port, struct file *filp);
static void mxuport_close(struct mxusb_serial_port *port, struct file * filp);
static int  mxuport_tiocmset(struct mxusb_serial_port *port, 
		unsigned int set, unsigned int clear);
static int  mxuport_tiocmget(struct mxusb_serial_port *port);
static void mxuport_break_ctl (struct mxusb_serial_port *port, int break_state);
static int  mxuport_chars_in_buffer (struct mxusb_serial_port *port);
static int  mxuport_write_room (struct mxusb_serial_port *port);
static void mxuport_throttle (struct mxusb_serial_port *port);
static void mxuport_unthrottle (struct mxusb_serial_port *port);
static int  mxuport_ioctl(struct mxusb_serial_port *port, unsigned int cmd, unsigned long arg);
static int  mxuport_write (struct mxusb_serial_port *port, const unsigned char *data, int count);
static void mxuport_set_termios(struct mxusb_serial_port *port, struct ktermios *old_termios);
#endif

static void mxuport_write_bulk_callback (struct urb *urb);
static void mxuport_read_bulk_callback (struct urb *urb);
static void mxuport_event_bulk_callback (struct urb *urb);
static void mxuport_control_callback (struct urb *urb);

/*
 *  Function prototypes for our local functions 
 */
static void  mx_load_firmware(struct mxuport_serial *mxserial);
static int  mx_init_port(struct mxusb_serial_port *port);
static int  mx_get_string(struct usb_device *dev, int Id, char *string);
static void mx_unicode_to_ascii(char *string, __le16 *unicode, int unicode_size);
static struct mx_queue* mx_init_queue(int portno, unsigned int buflen);
static void mx_free_queue(struct mx_queue *mx_q);
static void mx_clear_queue(struct mx_queue *mx_q);
static int  mx_data_in_queue(struct mx_queue *mx_q);
static int  mx_space_in_queue(struct mx_queue *mx_q);
static int  mx_insert_queue(struct mx_queue *mx_q, const char *buf, int count);
static int  mx_remove_tx_queue(struct mx_queue *mx_q, unsigned char *buf, int count);
static int  mx_remove_rx_queue(struct mx_queue *mx_q, unsigned char *buf, int count);
static void mx_pack_header(struct mx_queue *mx_q, unsigned char *buf, int count);
static int mx_process_txrx_fifo(struct mxuport_port *mxport, int type);
static int  mx_change_port_settings(struct mxuport_port *mxport, int async_spd_baud);
static int  mx_set_modem_info(struct tty_struct *tty, unsigned int cmd, unsigned long arg);
static void mx_tty_recv(struct tty_struct *tty, struct mxuport_port *mxport);
static void mx_send_break(struct mxusb_serial_port *port, int period);
static void mx_wait_fw_until_sent(struct mxusb_serial_port *port, int timeout);
static int  mx_check_close_port(struct mxuport_port *mxport, int portnum);
static void mx_update_recv_event(struct mxuport_port *mxport, unsigned long event, unsigned char *buf, unsigned long idx);
static void mx_prepare_write(struct mxusb_serial_port *port);
static int  mx_write_bulkout_urb (struct mxusb_serial_port *port, unsigned char *buffer, int length);
static void mx_send_async_ctrl_urb(struct usb_device *dev, __u8 request, __u16 value,
		__u16 index, u8 *data, int size);


static int  mx_send_ctrl_urb(struct usb_device *dev, __u8 request, __u16 value,
		__u16 index, u8 *data, int size);
static int  mx_recv_ctrl_urb(struct usb_device *dev, __u8 request, __u16 value,
		__u16  index,u8 *data, int size);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,20,0))
static int  mx_get_serial_info2(struct tty_struct *tty,
		struct serial_struct __user *ret_arg);
static int  mx_set_serial_info2(struct tty_struct *tty,
		struct serial_struct __user *new_arg);
#endif
static int  mx_get_serial_info(struct mxuport_port *mxport,
		struct serial_struct __user *ret_arg);
static int  mx_set_serial_info(struct mxuport_port *mxport,
		struct serial_struct __user *new_arg);

#if DBG_BURNIN_DATA 
static void mx_dbg_burnin_data(struct mxuport_port *mxport, unsigned char *buf, int count);
#endif
#endif
