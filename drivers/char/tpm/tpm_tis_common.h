#ifndef __TIS_COMMON__H
#define __TIS_COMMON__H

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pnp.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/acpi.h>
#include <linux/freezer.h>
#include "tpm.h"
#include "tpm_tis_common.h"
enum tis_access {
	TPM_ACCESS_VALID = 0x80,
	TPM_ACCESS_ACTIVE_LOCALITY = 0x20,
	TPM_ACCESS_REQUEST_PENDING = 0x04,
	TPM_ACCESS_REQUEST_USE = 0x02,
};

enum tis_status {
	TPM_STS_VALID = 0x80,
	TPM_STS_COMMAND_READY = 0x40,
	TPM_STS_GO = 0x20,
	TPM_STS_DATA_AVAIL = 0x10,
	TPM_STS_DATA_EXPECT = 0x08,
};

enum tis_int_flags {
	TPM_GLOBAL_INT_ENABLE = 0x80000000,
	TPM_INTF_BURST_COUNT_STATIC = 0x100,
	TPM_INTF_CMD_READY_INT = 0x080,
	TPM_INTF_INT_EDGE_FALLING = 0x040,
	TPM_INTF_INT_EDGE_RISING = 0x020,
	TPM_INTF_INT_LEVEL_LOW = 0x010,
	TPM_INTF_INT_LEVEL_HIGH = 0x008,
	TPM_INTF_LOCALITY_CHANGE_INT = 0x004,
	TPM_INTF_STS_VALID_INT = 0x002,
	TPM_INTF_DATA_AVAIL_INT = 0x001,
};

enum tis_defaults {
	TIS_MEM_BASE = 0xFED40000,
	TIS_MEM_LEN = 0x5000,
	TIS_SHORT_TIMEOUT = 750,	/* ms */
	TIS_LONG_TIMEOUT = 2000,	/* 2 sec */
};

/* Some timeout values are needed before it is known whether the chip is
 * TPM 1.0 or TPM 2.0.
 */
#define TIS_TIMEOUT_A_MAX	max(TIS_SHORT_TIMEOUT, TPM2_TIMEOUT_A)
#define TIS_TIMEOUT_B_MAX	max(TIS_LONG_TIMEOUT, TPM2_TIMEOUT_B)
#define TIS_TIMEOUT_C_MAX	max(TIS_SHORT_TIMEOUT, TPM2_TIMEOUT_C)
#define TIS_TIMEOUT_D_MAX	max(TIS_SHORT_TIMEOUT, TPM2_TIMEOUT_D)

#define	TPM_ACCESS(l)			(0x0000 | ((l) << 12))
#define	TPM_INT_ENABLE(l)		(0x0008 | ((l) << 12))
#define	TPM_INT_VECTOR(l)		(0x000C | ((l) << 12))
#define	TPM_INT_STATUS(l)		(0x0010 | ((l) << 12))
#define	TPM_INTF_CAPS(l)		(0x0014 | ((l) << 12))
#define	TPM_STS(l)			(0x0018 | ((l) << 12))
#define	TPM_STS3(l)			(0x001b | ((l) << 12))
#define	TPM_DATA_FIFO(l)		(0x0024 | ((l) << 12))

#define	TPM_DID_VID(l)			(0x0F00 | ((l) << 12))
#define	TPM_RID(l)			(0x0F04 | ((l) << 12))

struct priv_data {
	bool irq_tested;
};
struct tis_vendor_timeout_override {
	u32 did_vid;
	unsigned long timeout_us[4];
};

static const struct tis_vendor_timeout_override vendor_timeout_overrides[] = {
	/* Atmel 3204 */
	{ 0x32041114, { (TIS_SHORT_TIMEOUT * 1000), (TIS_LONG_TIMEOUT * 1000),
			(TIS_SHORT_TIMEOUT * 1000), (TIS_SHORT_TIMEOUT * 1000) } },
};

u8 tpm_tis_status(struct tpm_chip *chip);
void tpm_tis_ready(struct tpm_chip *chip);
int tpm_tis_recv(struct tpm_chip *chip, u8 *buf, size_t count);
int tpm_tis_send(struct tpm_chip *chip, u8 *buf, size_t len);
bool tpm_tis_update_timeouts(struct tpm_chip *chip,
				    unsigned long *timeout_cap);
bool tpm_tis_req_canceled(struct tpm_chip *chip, u8 status);
int tpm_tis_resume(struct device *dev);
void tpm_tis_remove(struct tpm_chip *chip);

/* Helpers - read */
static inline void read_tpm_bytes(struct tpm_chip *chip, u32 addr, u8 len, u8 *res)
{
	chip->ops->read_bytes(chip, addr, len, 1, res);
}
static inline void read_tpm_byte(struct tpm_chip *chip, u32 addr, u8 *res)
{
	chip->ops->read_bytes(chip, addr, 1, 1, res);
}

static inline void read_tpm_word(struct tpm_chip *chip, u32 addr, u16 *res)
{
	chip->ops->read_bytes(chip, addr, 1, 2, (u8 *)res);
}

static inline void read_tpm_dword(struct tpm_chip *chip, u32 addr, u32 *res)
{
	chip->ops->read_bytes(chip, addr, 1, 4, (u8 *)res);
}

/* Helpers - write */
static inline void write_tpm_bytes(struct tpm_chip *chip, u32 addr, u8 len, u8 *value)
{
	chip->ops->write_bytes(chip, addr, len, 1, value);
}
static inline void write_tpm_byte(struct tpm_chip *chip, u32 addr, u8 value)
{
	u8 tmp = value;

	chip->ops->write_bytes(chip, addr, 1, 1, &tmp);
}
static inline void write_tpm_dword(struct tpm_chip *chip, u32 addr, u32 value)
{
	u32 tmp = value;

	chip->ops->write_bytes(chip, addr, 1, 4, (u8 *)&tmp);
}

int tpm_tis_init_generic(struct device *dev, struct tpm_chip *chip, unsigned int irq, bool enable_interrupts, bool itpm);
#endif
