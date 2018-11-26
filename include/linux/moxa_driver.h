/*
 * @file moxa_driver.h
 * @brief define some independent driver, let it can be used in all driver
 *	  driver can define VICTOR_DEBUG for debug functions, but it must
 *	  be defined before <linux/moxa_driver.h> (I have put it in 
 *        includ/linux directory), eg:
 *	  #define VICTOR_DEBUG
 *	  #include <linux/moxa_driver.h>
 *	  you can change the default debug_level setting eg:
 *	  static int __init	module_init(void)
 *	  {
 *	  #ifdef VICTOR_DEBUG
 *	  	debug_level = ALL_DEBUG_LEVEL;
 *	  #endif
 *		...... // others code
 *	  }
 *	  another thing, the default will create a file in 
 *        /sys/module/driver_name/parameters/debug_level
 *	  you can change the debug level by dynamic, or you can call 
 *        create_sys_debug_level() to create
 *	  a sys file in your device driver directory, it is used the same 
 *        above. But you have call remove_sys_debug_level() when you exit driver
 * 
 * History:
 * Date		Author		Comment
 * 2017/1/16	Victor Yu. 	Initialize it.
 * 2017/2/21	Victor Yu.	Add class attribute and change name.
 * 2017/2/23	Victor Yu.	Add ring buffer control.
 * 2017/3/9	Victor Yu.	Add a register API in i2c multiplexer driver 
 *                              for other drivers. When the the multiplexer 
 *                              driver dectected some module change, it will 
 *                              call these registered driver by work queue.
 */
#ifndef _MOXA_DRIVER_H
#define _MOXA_DRIVER_H

/*
 * here is my private debug function
 */
#ifdef VICTOR_DEBUG
        #define ALL_DEBUG_LEVEL         0 /*output all debug message*/
        #define RAW_DEBUG_LEVEL         3 /*output raw level debug message*/
        #define NORMAL_DEBUG_LEVEL      6 /*output normal level debug message*/
        #define HIGH_DEBUG_LEVEL        9 /*output high level debug message*/
	/* no debug message output */
        #define NO_DEBUG_LEVEL          (HIGH_DEBUG_LEVEL+1)    

	/* set the debug level */
        static int      __attribute__((unused))debug_level=NO_DEBUG_LEVEL; 
        #define dgprintk(level,fmt, ...) { \
                if ( (level) >= debug_level ) \
                        printk("%s,%s,%d: " fmt, __FILE__, __func__, \
				__LINE__, ##__VA_ARGS__); \
        }
#else   /* VICTOR_DEBUG */
        #define dgprintk(level,fmt, ...)
#endif  /* VICTOR_DEBUG */
#define myprintk(fmt, ...)      printk("%s,%s,%d: " fmt, __FILE__, __func__, \
					__LINE__, ##__VA_ARGS__)

#define BYTE_BITS	8 /* 8 bits for one byte */

/*
 * MOXA NET SPI FPGA Format
 */
#ifdef CONFIG_SPI_MOXA_EDS_FPGA
#define DECLARE_MOXA_FPGA_SPI_TRANSFER(_l,_bpw)	\
struct spi_transfer      spi_t = {		\
        .len            = _l,			\
        .bits_per_word  = _bpw,			\
        .tx_nbits       = SPI_NBITS_SINGLE,	\
        .rx_nbits       = SPI_NBITS_SINGLE,	\
}
#endif //CONFIG_SPI_MOXA_EDS_FPGA

/*
 * there are sysfs text information for int with dec or hex output
 * the interface is struct kobject and struct kobj_attribute
 */
#include <linux/kobject.h>
#define my_kobj_hex_show(_var) \
static ssize_t  _var##_kobj_hex_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) \
{ \
        return sprintf(buf, "0x%x", _var); \
}
#define my_kobj_int_show(_var) \
static ssize_t  _var##_kobj_int_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) \
{ \
        return sprintf(buf, "%d", _var); \
}
#define my_kobj_int_store(_var) \
static ssize_t  _var##_kobj_int_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) \
{ \
        int     t; \
        if ( unlikely(!count) ) \
                return -EINVAL; \
        if ( buf[0] == '0' && buf[1] == 'x' ) \
                sscanf(buf, "0x%x", &t); \
        else \
                sscanf(buf, "%d", &t); \
        _var = t; \
        return count; \
}
#define MY_KOBJ_INT_ATTR_RW(_name) \
        my_kobj_int_show(_name) \
        my_kobj_int_store(_name) \
        static struct kobj_attribute    kobj_attr_##_name = { \
                .attr   = { \
                        .name   = __stringify(_name), \
                        .mode   = S_IWUSR | S_IRUGO, \
                }, \
                .show   = _name##_kobj_int_show, \
                .store  = _name##_kobj_int_store, \
        };
#define MY_KOBJ_INT_ATTR_RO(_name) \
        my_kobj_int_show(_name) \
        static struct kobj_attribute    kobj_attr_##_name = { \
                .attr   = { \
                        .name   = __stringify(_name), \
                        .mode   = S_IRUGO, \
                }, \
                .show   = _name##_kobj_int_show, \
                .store  = NULL, \
        };
#define MY_KOBJ_INT_ATTR_WO(_name) \
        my_kobj_int_store(_name) \
        static struct kobj_attribute    kobj_attr_##_name = { \
                .attr   = { \
                        .name   = __stringify(_name), \
                        .mode   = S_IWUSR, \
                }, \
                .show   = NULL, \
                .store  = _name##_kobj_int_store, \
        };
#define MY_KOBJ_HEX_ATTR_RW(_name) \
        my_kobj_hex_show(_name) \
        my_kobj_int_store(_name) \
        static struct kobj_attribute    kobj_attr_##_name = { \
                .attr   = { \
                        .name   = __stringify(_name), \
                        .mode   = S_IWUSR | S_IRUGO, \
                }, \
                .show   = _name##_kobj_hex_show, \
                .store  = _name##_kobj_int_store, \
        };
#define MY_KOBJ_HEX_ATTR_RO(_name) \
        my_kobj_hex_show(_name) \
        static struct kobj_attribute    kobj_attr_##_name = { \
                .attr   = { \
                        .name   = __stringify(_name), \
                        .mode   = S_IRUGO, \
                }, \
                .show   = _name##_kobj_hex_show, \
                .store  = NULL, \
        };
#define MY_KOBJ_HEX_ATTR_WO(_name) \
        my_kobj_int_store(_name) \
        static struct kobj_attribute    kobj_attr_##_name = { \
                .attr   = { \
                        .name   = __stringify(_name), \
                        .mode   = S_IWUSR, \
                }, \
                .show   = NULL, \
                .store  = _name##_kobj_int_store, \
        };
#define MY_KOBJ_ATTR_LIST(_name) &kobj_attr_##_name.attr

/*
 * there are sysfs text information for int with dec or hex output
 * the interface is struct class and struct class_attribute
 */
#define my_cls_hex_show(_var) \
static ssize_t  _var##_cls_hex_show(struct class *cls, struct class_attribute *attr, char *buf) \
{ \
        return sprintf(buf, "0x%x", _var); \
}
#define my_cls_int_show(_var) \
static ssize_t  _var##_cls_int_show(struct class *cls, struct class_attribute *attr, char *buf) \
{ \
        return sprintf(buf, "%d", _var); \
}
#define my_cls_int_store(_var) \
static ssize_t  _var##_cls_int_store(struct class *kobj, struct class_attribute *attr, const char *buf, size_t count) \
{ \
        int     t; \
        if ( unlikely(!count) ) \
                return -EINVAL; \
        if ( buf[0] == '0' && buf[1] == 'x' ) \
                sscanf(buf, "0x%x", &t); \
        else \
                sscanf(buf, "%d", &t); \
        _var = t; \
        return count; \
}
#define MY_CLS_INT_ATTR_RW(_name) \
        my_cls_int_show(_name) \
        my_cls_int_store(_name) \
        static struct class_attribute    cls_attr_##_name = { \
                .attr   = { \
                        .name   = __stringify(_name), \
                        .mode   = S_IWUSR | S_IRUGO, \
                }, \
                .show   = _name##_cls_int_show, \
                .store  = _name##_cls_int_store, \
        };
#define MY_CLS_INT_ATTR_RO(_name) \
        my_cls_int_show(_name) \
        static struct class_attribute    cls_attr_##_name = { \
                .attr   = { \
                        .name   = __stringify(_name), \
                        .mode   = S_IRUGO, \
                }, \
                .show   = _name##_cls_int_show, \
                .store  = NULL, \
        };
#define MY_CLS_INT_ATTR_WO(_name) \
        my_cls_int_store(_name) \
        static struct class_attribute    cls_attr_##_name = { \
                .attr   = { \
                        .name   = __stringify(_name), \
                        .mode   = S_IWUSR | S_IRUGO, \
                }, \
                .show   = NULL, \
                .store  = _name##_cls_int_store, \
        };
#define MY_CLS_HEX_ATTR_RW(_name) \
        my_cls_hex_show(_name) \
        my_cls_int_store(_name) \
        static struct class_attribute    cls_attr_##_name = { \
                .attr   = { \
                        .name   = __stringify(_name), \
                        .mode   = S_IWUSR | S_IRUGO, \
                }, \
                .show   = _name##_cls_hex_show, \
                .store  = _name##_cls_int_store, \
        };
#define MY_CLS_HEX_ATTR_RO(_name) \
        my_cls_hex_show(_name) \
        static struct class_attribute    cls_attr_##_name = { \
                .attr   = { \
                        .name   = __stringify(_name), \
                        .mode   = S_IRUGO, \
                }, \
                .show   = _name##_cls_hex_show, \
                .store  = NULL, \
        };
#define MY_CLS_HEX_ATTR_WO(_name) \
        my_cls_int_store(_name) \
        static struct class_attribute    cls_attr_##_name = { \
                .attr   = { \
                        .name   = __stringify(_name), \
                        .mode   = S_IWUSR | S_IRUGO, \
                }, \
                .show   = NULL, \
                .store  = _name##_cls_int_store, \
        };
#define MY_CLS_ATTR_LIST(_name) &cls_attr_##_name.attr

#define DEVICE_ATTR_LIST(_name) &dev_attr_##_name.attr

#define MS_TO_JIFFIES(m)        (((m) * HZ) / 1000)

#ifdef VICTOR_DEBUG
	module_param(debug_level, int, 0644); // this parameter will be created in /sys/module/driver_name/parameters/debug_level

	/* here to create a sysfs file in driver directory */
        my_kobj_int_show(debug_level)
        my_kobj_int_store(debug_level)
        static struct kobj_attribute    __attribute__((unused))kobj_attr_debug_level = {
                .attr   = {
                        .name   = __stringify(debug_level),
                        .mode   = S_IWUSR | S_IRUGO,
                },
                .show   = debug_level_kobj_int_show,
                .store  = debug_level_kobj_int_store,
        };
	#define create_sysfs_debug_level(_kobj)	sysfs_create_file_ns((_kobj), &kobj_attr_debug_level.attr, NULL)
	#define remove_sysfs_debug_level(_kobj)	sysfs_remove_file_ns((_kobj), &kobj_attr_debug_level.attr, NULL)
#endif

/*
 * ring buffer maintain macro
 */
#include <linux/spinlock.h>
#include <linux/slab.h>
typedef struct moxa_ring_buffer_struct {
	unsigned int	wptr;
	unsigned int	rptr;
	unsigned int	size;
	spinlock_t	spinlock;
	unsigned char	buffer[0];
} moxa_ring_buffer_t;

#define free_ring_no(_w,_r,_s)  (((_w) >= (_r)) ? ((_s)-(_w)+(_r)-1) : ((_r)-(_w)-1))
#define in_ring_no(_w,_r,_s)    (((_w) >= (_r)) ? ((_w)-(_r)) : ((_s)-(_r)+(_w)))
#define INC_PTR(_v,_s)          ((++(_v) >= (_s) ) ? (_v) = 0 : (_v))
#define ADD_PTR(_v, _c, _s)	{(_v) += (_c); ((_v) >= (_s)) ? (_v) -= (_s): (_v);}
#define abort_ring(_w,_r)	((_w)=(_r)=0)

#define moxa_ring_buffer_free(_ringbuf)	free_ring_no(_ringbuf->wptr, \
					_ringbuf->rptr, _ringbuf->size)
#define moxa_ring_buffer_in(_ringbuf)	in_ring_no(_ringbuf->wptr, \
					 _ringbuf->rptr, _ringbuf->size)
#define moxa_ring_buffer_destory(_ringbuf)	kfree(_ringbuf)
#define moxa_ring_buffer_abort(_ringbuf)	(_ringbuf->wptr=_ringbuf->rptr=0)
#define moxa_ring_buffer_buffer(_ringbuf)	_ringbuf->buffer
#define moxa_ring_buffer_container_of(_buf)	(_buf-sizeof(moxa_ring_buffer_t)

/*
 * @func static struct moxa_ring_buffer_struct   __attribute__((unused))*moxa_ring_buffer_init(unsigned int size)
 * @brief allocate the moxa ring buffer data and initialize it
 * @param unsigned int size	want to allocate buffer size, suggest it is 2 power value
 * @return allocated buffer pointer, NULL is fail
 */
static struct moxa_ring_buffer_struct	__attribute__((unused))*moxa_ring_buffer_init(unsigned int size)
{
	moxa_ring_buffer_t	*ringbuf;

	ringbuf = kzalloc(sizeof(moxa_ring_buffer_t)+size, GFP_KERNEL);
	if ( ringbuf ) {
		spin_lock_init(&ringbuf->spinlock);
		ringbuf->size = size;
	}
	return ringbuf;
}

/*
 * @func static int      __attribute__((unused))moxa_ring_buffer_put(moxa_ring_buffer_t *ringbuf, char *data, int count)
 * @brief put user data to the moxa ring buffer
 * @param moxa_ring_buffer_t *ringbuf	the moxa ring buffer data structure
 * 	  char *data			user data which want to save in moxa ring buffer
 *	  int count			want to save count
 * @return saved bytes
 */
static int	__attribute__((unused))moxa_ring_buffer_put(moxa_ring_buffer_t *ringbuf, char *data, int count)
{
	int	ret, i;

	/* TODO : should we spin lock here */
	ret = moxa_ring_buffer_free(ringbuf);
	if ( count > ret )
		count = ret;
	ret = count;
	if ( (count+ringbuf->wptr) >= ringbuf->size ) {
		i = ringbuf->size - ringbuf->wptr;
		memcpy(&ringbuf->buffer[ringbuf->wptr], data, i);
		data += i;
		count -= i;
		ringbuf->wptr = 0;
	}
	if ( count > 0 ) {
		memcpy(&ringbuf->buffer[ringbuf->wptr], data, count);
		ADD_PTR(ringbuf->wptr, count, ringbuf->size);
	}
	/* TODO : should we spin unlock here */
	return ret;
}

/*
 * @func static int      __attribute__((unused))moxa_ring_buffer_get(moxa_ring_buffer_t *ringbuf, char *buf, int count)
 * @brief to get data from moxa ring buffer
 * @param moxa_ring_buffer_t *ringbuf	the moxa ring buffer data structure
 *	  char *buf			want to save buffer from moxa ring buffer
 *	  int count			want to get count
 * @return got bytes
 */
static int	__attribute__((unused))moxa_ring_buffer_get(moxa_ring_buffer_t *ringbuf, char *buf, int count)
{
	int	ret, i;

	/* TODO : should we spin lock here */
	ret = moxa_ring_buffer_in(ringbuf);
	if ( count < ret )
		count = ret;
	ret = count;
	if ( (count+ringbuf->rptr) >= ringbuf->size ) {
		i = ringbuf->size - ringbuf->rptr;
		memcpy(buf, &ringbuf->buffer[ringbuf->rptr], i);
		buf += i;
		count -= i;
		ringbuf->rptr = 0;
	}
	if ( count > 0 ) {
		memcpy(buf, &ringbuf->buffer[ringbuf->rptr], count);
		ADD_PTR(ringbuf->rptr, count, ringbuf->size);
	}
	/* TODO : should we spin unlock here */
	return ret;
}

#ifdef CONFIG_MOXA_SYSFS
/*
 * include moxa_sysfs kobject, this is locale in /sys/moxa directory
 * this implement in drivers/base/moxa_sysfs.c
 */
extern struct kobject	moxa_sysfs_kobj; // /sys/moxa kobject
extern struct kobj_type	moxa_sysfs_ktype; // /sys/moxa ktype for this kobject
/*
 * @func create_moxa_sysfs_dir(_kobj, ...)
 * @brief create a subdirectory in /sys/moxa directory
 * @param _kobj		the struct kobject data
 * @return 0 is OK, others is fail
 */
#define create_moxa_sysfs_subdir(_kobj, ...)	kobject_init_and_add((_kobj), &moxa_sysfs_ktype, &moxa_sysfs_kobj, ##__VA_ARGS__)

/*
 * @func remove_moxa_sysfs_dir(_kobj)
 * @brief remove a subdirectory in /sys/moxa directory
 * @param _kobj		the struct kobject data
 * @return none
 */
#define remove_moxa_sysfs_subdir(_kobj)		kobject_del((_kobj))
#endif // CONFIG_MOXA_SYSFS

/* this implement in drivers/i2c/muxes/i2c-mux-moxa-eds.c */
#if defined(CONFIG_I2C_MUX_MOXA_EDS) || defined(CONFIG_I2C_MUX_MOXA_EDS_MODULE)
/*
 * here is my private register function in my i2c multiplexer driver
 * this implement in drivers/i2c/muxes/i2c-mux-moxa-eds.c
 */
#include <linux/list.h>
#include <linux/workqueue.h>
#define CHANGE_NO       64 // must be 2 power
#define CHANGE_NO_MASK  (CHANGE_NO-1)
typedef struct   moxaeds_module_change_struct {
        u32     change; // which change
        u32     old;    // before change old value (old inserted_module)
        u32     new;    // after change new value
} moxaeds_module_change_t;

/* module information define */
#define MAX_MODULES	16
#define CPU_MODULE	0 // CPU is located this module
#define POWER_MODULE	8 // power module index
#define CONTROL_MODULE	15 // control module index
enum moxaeds_module_type_enum {
        NETWORK1=0,
        NETWORK2,
        NETWORK3,
        POWER=POWER_MODULE,
        CONTROL=CONTROL_MODULE,
        UNKNOWN=-1,
};

static int __attribute__((unused))no_fpga_module_lists[] = {7, POWER_MODULE};
#define NO_FPGA_MODULE_LISTS_NO	(sizeof(no_fpga_module_lists)/sizeof(int))

typedef struct moxaeds_module_info_struct {
        int slotno; /* 0 - 7, 7 is the most close to CPU module */
        enum moxaeds_module_type_enum   type;
        int eeprom_data_len;
#ifdef CONFIG_MOXA_SYSFS
        struct kobject module_sysfs_kobj;
#endif
#define MAX_EEPROM_DATA_LEN     2048
        u8 eeprom_data[0]; /* this member must be last in this structure */
} moxaeds_module_info_t;

typedef struct moxaeds_module_dect_struct {
	struct list_head	list;
	/* 
	 * allocate/maintain in i2c-mux-moxa-eds.c driver, the other 
	 * drivers just read it
	 */
	moxaeds_module_info_t	**module_info; 

	/* when the i2c-mux-moxa-eds.c shutdown, it will be called */
	void (*exit)(struct moxaeds_module_dect_struct *md); 
	int (*inserted)(struct moxaeds_module_dect_struct *md, int mno);
	int (*removed)(struct moxaeds_module_dect_struct *md, int mno);
	u8 priv[0]; /* the others private data */
} moxaeds_module_dect_t;

extern int     register_moxaeds_module_dect(moxaeds_module_dect_t *md);
extern void    unregister_moxaeds_module_dect(moxaeds_module_dect_t *md);
extern moxaeds_module_dect_t   *alloc_moxaeds_module_dect(int privsize);

#define get_moxaeds_module_dect_priv(_md)	(_md)->priv
#define free_moxaeds_module_dect(_md)		kfree((_md))
#endif /* CONFIG_I2C_MUX_MOXA_EDS */

#endif /* _MOXA_DRIVER_H */

