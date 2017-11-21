#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/reboot.h>
#include <linux/watchdog.h>

#define DS1374_REG_WDALM0	0x04 /* Watchdog/Alarm */
#define DS1374_REG_CR		0x07 /* Control */
#define DS1374_REG_CR_AIE	0x01 /* Alarm Int. Enable */
#define DS1374_REG_CR_WDSTR	0x08
#define DS1374_REG_CR_WDALM	0x20 /* 1=Watchdog, 0=Alarm */
#define DS1374_REG_CR_WACE	0x40 /* WD/Alarm counter enable */

/* Default nowayout */
#define DEFAULT_NOWAYOUT        WATCHDOG_NOWAYOUT
/* Default timeout margin (1/4096 second) */
#define DEFAULT_MARGIN          (60 * 4096)

static int timer_margin = DEFAULT_MARGIN;
static bool nowayout = DEFAULT_NOWAYOUT;

extern struct i2c_client *save_client;
extern int ds1374_write_rtc(struct i2c_client *client, u32 time, int reg, 
                            int nbytes);
extern int ds1374_read_rtc(struct i2c_client *client, u32 *time, int reg, 
                            int nbytes);


struct ds1374 {
	struct i2c_client *client;
	struct rtc_device *rtc;
	struct work_struct work;

	/* The mutex protects alarm operations, and prevents a race
	 * between the enable_irq() in the workqueue and the free_irq()
	 * in the remove function.
	 */
	struct mutex mutex;
	int exiting;
};

/* Default margin */
#define DRV_NAME "DS1374 Watchdog"
static unsigned long wdt_is_open;

module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started, default="
                __MODULE_STRING(WATCHDOG_NOWAYOUT));
module_param(timer_margin, int, 0);
MODULE_PARM_DESC(timer_margin, "Watchdog timeout in seconds (default 60s)");

static const struct watchdog_info ds1374_wdt_info = {
	.identity       = "DS1374 WTD",
	.options        = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
						WDIOF_MAGICCLOSE,
};
static char expect_close;
static int ds1374_wdt_settimeout(unsigned int timeout)
{
	int ret = -ENOIOCTLCMD;
	int cr;

	ret = cr = i2c_smbus_read_byte_data(save_client, DS1374_REG_CR);
	if (ret < 0)
		goto out;

	/* Disable any existing watchdog/alarm before setting the new one */
	cr &= ~DS1374_REG_CR_WACE;

	ret = i2c_smbus_write_byte_data(save_client, DS1374_REG_CR, cr);
	if (ret < 0)
		goto out;

	/* Set new watchdog time */
	ret = ds1374_write_rtc(save_client, timeout, DS1374_REG_WDALM0, 3);
	if (ret) {
		pr_info("ds1374_wdt: Couldn't set new watchdog time\n");
		goto out;
	}

	/* Enable watchdog timer */
	cr |= DS1374_REG_CR_WACE | DS1374_REG_CR_WDALM;
	cr &= ~DS1374_REG_CR_WDSTR;
	cr &= ~DS1374_REG_CR_AIE;

	ret = i2c_smbus_write_byte_data(save_client, DS1374_REG_CR, cr);
	if (ret < 0)
		goto out;

	return 0;
out:
	return ret;
}


/*
 * Reload the watchdog timer.  (ie, pat the watchdog)
 */
static void ds1374_wdt_ping(void)
{
	u32 val;
	int ret = 0;

	ret = ds1374_read_rtc(save_client, &val, DS1374_REG_WDALM0, 3);
	if (ret)
		pr_info("ds1374_wdt: WD TICK FAIL!!!!!!!!!! %i\n", ret);
}

static void ds1374_wdt_disable(void)
{
	int ret = -ENOIOCTLCMD;
	int cr;

	cr = i2c_smbus_read_byte_data(save_client, DS1374_REG_CR);
	/* Disable watchdog timer */
	cr &= ~DS1374_REG_CR_WACE;

	ret = i2c_smbus_write_byte_data(save_client, DS1374_REG_CR, cr);
}

/*
 * Watchdog device is opened, and watchdog starts running.
 */
static int ds1374_wdt_open(struct inode *inode, struct file *file)
{
	struct ds1374 *ds1374 = i2c_get_clientdata(save_client);

	if (MINOR(inode->i_rdev) == WATCHDOG_MINOR) {
		mutex_lock(&ds1374->mutex);
		if (test_and_set_bit(0, &wdt_is_open)) {
			mutex_unlock(&ds1374->mutex);
			return -EBUSY;
		}
		/*
		 *      Activate
		 */
		wdt_is_open = 1;
		mutex_unlock(&ds1374->mutex);
	        	
                ds1374_wdt_settimeout(timer_margin);
                return nonseekable_open(inode, file);
	}
	return -ENODEV;
}

/*
 * Close the watchdog device.
 */
static int ds1374_wdt_release(struct inode *inode, struct file *file)
{
	if (MINOR(inode->i_rdev) == WATCHDOG_MINOR)
		clear_bit(0, &wdt_is_open);

	/*
	 *  Check NOWAYOUT's setting and whether do magic close 
	 */
	if (!nowayout && (expect_close == 42)) {
               	ds1374_wdt_disable();
               	printk(KERN_CRIT "ds1374_wdt: Magic close, shut off the timer!\n");
	} else {
        	printk(KERN_CRIT "ds1374_wdt: Unexpected close, not stopping!\n");
        }
	expect_close = 0;

	return 0;
}

/*
 * Pat the watchdog whenever device is written to.
 */
static ssize_t ds1374_wdt_write(struct file *file, const char __user *data,
				size_t len, loff_t *ppos)
{
	int ofs;
        
        if (len) {
		ds1374_wdt_settimeout(timer_margin);
		ds1374_wdt_ping();
                if (!nowayout) {
                        for (ofs = 0; ofs != len; ofs++) {
                                char c;
                                if (get_user(c, data + ofs))
                                        return -EFAULT;
                                if (c == 'V') 
                                        expect_close = 42;
                        }
                }
        }
	return len;
}

static ssize_t ds1374_wdt_read(struct file *file, char __user *data,
				size_t len, loff_t *ppos)
{
	return 0;
}

/*
 * Handle commands from user-space.
 */
static long ds1374_wdt_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	int new_margin, options;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info __user *)arg,
		&ds1374_wdt_info, sizeof(ds1374_wdt_info)) ? -EFAULT : 0;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, (int __user *)arg);
	case WDIOC_KEEPALIVE:
		ds1374_wdt_ping();
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, (int __user *)arg))
			return -EFAULT;

		if (new_margin < 1 || new_margin > 16777216)
			return -EINVAL;

		timer_margin = (new_margin * 4096);
		ds1374_wdt_settimeout(timer_margin);
		ds1374_wdt_ping();
		/* fallthrough */
	case WDIOC_GETTIMEOUT:
		return put_user(timer_margin / 4096, (int __user *)arg);
	case WDIOC_SETOPTIONS:
		if (copy_from_user(&options, (int __user *)arg, sizeof(int)))
			return -EFAULT;

		if (options & WDIOS_DISABLECARD & !nowayout ) {
			pr_info("ds1374_wdt: disable watchdog\n");
			ds1374_wdt_disable();
		}

		if (options & WDIOS_ENABLECARD) {
			pr_info("ds1374_wdt: enable watchdog\n");
			ds1374_wdt_settimeout(timer_margin);
			ds1374_wdt_ping();
		}

		return -EINVAL;
	}
	return -ENOTTY;
}

static long ds1374_wdt_unlocked_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int ret;
	struct ds1374 *ds1374 = i2c_get_clientdata(save_client);

	mutex_lock(&ds1374->mutex);
	ret = ds1374_wdt_ioctl(file, cmd, arg);
	mutex_unlock(&ds1374->mutex);

	return ret;
}

static int ds1374_wdt_notify_sys(struct notifier_block *this,
			unsigned long code, void *unused)
{
	/* use ds1374 reboot instead of CPU watchdog reboot */
	if (code == SYS_HALT) {
 	/* Disable Watchdog */
		ds1374_wdt_disable(); 
	} else if (code == SYS_RESTART) {
	/* one second */
		ds1374_wdt_settimeout(1*4096);
		ds1374_wdt_ping();
 	}
	return NOTIFY_DONE;
}

static const struct file_operations ds1374_wdt_fops = {
	.owner			= THIS_MODULE,
	.read			= ds1374_wdt_read,
	.unlocked_ioctl		= ds1374_wdt_unlocked_ioctl,
	.write			= ds1374_wdt_write,
	.open                   = ds1374_wdt_open,
	.release                = ds1374_wdt_release,
	.llseek			= no_llseek,
};

static struct miscdevice ds1374_miscdev = {
	.minor          = WATCHDOG_MINOR,
	.name           = "watchdog",
	.fops           = &ds1374_wdt_fops,
};

static struct notifier_block ds1374_wdt_notifier = {
	.notifier_call = ds1374_wdt_notify_sys,
};

static int __init ds1374_wdt_init(void)
{
	int ret;
        
        ret = misc_register(&ds1374_miscdev);
	if (ret)
		return ret;
	ret = register_reboot_notifier(&ds1374_wdt_notifier);
	if (ret) {
		misc_deregister(&ds1374_miscdev);
		return ret;
	}
	if (timer_margin != DEFAULT_MARGIN)
                timer_margin *= 4096; 
        pr_info("ds1374_wdt: Watchdog timer initial timeout %d sec\n",
                        timer_margin/4096);
 
	if (nowayout)
		 pr_info("ds1374_wdt: Nowayout mode\n");
	else
		 pr_info("ds1374_wdt: Common mode\n"); 
	       
        return 0;
}

static void __exit ds1374_wdt_exit(void)
{
	ds1374_wdt_disable();
	misc_deregister(&ds1374_miscdev);
	ds1374_miscdev.parent = NULL;
	unregister_reboot_notifier(&ds1374_wdt_notifier);
}

module_init(ds1374_wdt_init);
module_exit(ds1374_wdt_exit);

MODULE_AUTHOR("Scott Wood <scottwood@freescale.com>");
MODULE_DESCRIPTION("Maxim/Dallas DS1374 WDT Driver");
MODULE_LICENSE("GPL");

