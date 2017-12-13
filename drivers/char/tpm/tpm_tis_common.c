#include <linux/init.h>
#include <linux/pnp.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/freezer.h>
#include "tpm.h"
#include "tpm_tis_common.h"

/* Before we attempt to access the TPM we must see that the valid bit is set.
 * The specification says that this bit is 0 at reset and remains 0 until the
 * 'TPM has gone through its self test and initialization and has established
 * correct values in the other bits.' */
static int wait_startup(struct tpm_chip *chip, int l)
{
	u8 access;
	unsigned long stop = jiffies + chip->vendor.timeout_a;

	do {
		read_tpm_byte(chip, TPM_ACCESS(l), &access);
		if (access & TPM_ACCESS_VALID)
			return 0;
		msleep(TPM_TIMEOUT);
	} while (time_before(jiffies, stop));
	return -1;
}

static int check_locality(struct tpm_chip *chip, int l)
{
	u8 access;

	read_tpm_byte(chip, TPM_ACCESS(l), &access);
	if ((access & (TPM_ACCESS_ACTIVE_LOCALITY | TPM_ACCESS_VALID)) ==
	    (TPM_ACCESS_ACTIVE_LOCALITY | TPM_ACCESS_VALID))
		return chip->vendor.locality = l;

	return -1;
}

static void release_locality(struct tpm_chip *chip, int l, int force)
{
	u8 access;

	read_tpm_byte(chip, TPM_ACCESS(l), &access);
	if (force ||
	    (access & (TPM_ACCESS_REQUEST_PENDING | TPM_ACCESS_VALID)) ==
	    (TPM_ACCESS_REQUEST_PENDING | TPM_ACCESS_VALID))
		write_tpm_byte(chip, TPM_ACCESS(l), TPM_ACCESS_ACTIVE_LOCALITY);
}
//Todo inline?

static int request_locality(struct tpm_chip *chip, int l)
{
	unsigned long stop, timeout;
	long rc;

	if (check_locality(chip, l) >= 0)
		return l;

	write_tpm_byte(chip, TPM_ACCESS(l), TPM_ACCESS_REQUEST_USE);

	stop = jiffies + chip->vendor.timeout_a;

	if (chip->vendor.irq) {
again:
		timeout = stop - jiffies;
		if ((long)timeout <= 0)
			return -1;
		rc = wait_event_interruptible_timeout(chip->vendor.int_queue,
						      (check_locality
						       (chip, l) >= 0),
						      timeout);
		if (rc > 0)
			return l;
		if (rc == -ERESTARTSYS && freezing(current)) {
			clear_thread_flag(TIF_SIGPENDING);
			goto again;
		}
	} else {
		/* wait for burstcount */
		do {
			if (check_locality(chip, l) >= 0)
				return l;
			msleep(TPM_TIMEOUT);
		} while (time_before(jiffies, stop));
	}
	return -1;
}

u8 tpm_tis_status(struct tpm_chip *chip)
{
	u8 status;

	read_tpm_byte(chip, TPM_STS(chip->vendor.locality), &status);
	return status;
}
EXPORT_SYMBOL(tpm_tis_status); //TODO inline?

void tpm_tis_ready(struct tpm_chip *chip)
{
	/* this causes the current command to be aborted */
	write_tpm_byte(chip, TPM_STS(chip->vendor.locality), TPM_STS_COMMAND_READY);
}
EXPORT_SYMBOL(tpm_tis_ready);

static int get_burstcount(struct tpm_chip *chip)
{
	unsigned long stop;
	int burstcnt;
	u8 tmp;
	/* wait for burstcount */
	/* which timeout value, spec has 2 answers (c & d) */
	stop = jiffies + chip->vendor.timeout_d;
	do {
		read_tpm_byte(chip, TPM_STS(chip->vendor.locality) + 1, &tmp);
		burstcnt = tmp;
		read_tpm_byte(chip, TPM_STS(chip->vendor.locality) + 2, &tmp);
		burstcnt += tmp << 8;
		//read_tpm_word(chip, TPM_STS(chip->vendor.locality) + 1, &burstcount2);//TODO revisit
		if (burstcnt)
			return min_t(int,burstcnt, 64); //SPI framesize TODO revisit
		msleep(TPM_TIMEOUT);
	} while (time_before(jiffies, stop));
	return -EBUSY;
}

static int recv_data(struct tpm_chip *chip, u8 *buf, size_t count)
{
	size_t size = 0, burstcnt, transfer_size;

	while (size < count &&
	       wait_for_tpm_stat(chip,
				 TPM_STS_DATA_AVAIL | TPM_STS_VALID,
				 chip->vendor.timeout_c,
				 &chip->vendor.read_queue, true)
	       == 0) {
		burstcnt = get_burstcount(chip);
		transfer_size = min_t (size_t, (count - size), burstcnt);
//		pr_err("size %d count %d bct %d tsize %d\n", size, count, burstcnt, transfer_size);
		read_tpm_bytes(chip, TPM_DATA_FIFO(chip->vendor.locality), transfer_size, &buf[size]);
		size += transfer_size;
	}
	return size;
}

int tpm_tis_recv(struct tpm_chip *chip, u8 *buf, size_t count)
{
	int size = 0;
	int expected, status;

	if (count < TPM_HEADER_SIZE) {
		size = -EIO;
		goto out;
	}

	/* read first 10 bytes, including tag, paramsize, and result */
	size = recv_data(chip, buf, TPM_HEADER_SIZE);
	if (size < TPM_HEADER_SIZE) {
		dev_err(chip->pdev, "Unable to read header\n");
		goto out;
	}

	expected = be32_to_cpu(*(__be32 *)(buf + 2));
	if (expected > count) {
		size = -EIO;
		goto out;
	}

	size += recv_data(chip, &buf[TPM_HEADER_SIZE], expected - TPM_HEADER_SIZE);
	if (size < expected) {
		dev_err(chip->pdev, "Unable to read remainder of result\n");
		size = -ETIME;
		goto out;
	}

	wait_for_tpm_stat(chip, TPM_STS_VALID, chip->vendor.timeout_c,
			  &chip->vendor.int_queue, false);
	status = tpm_tis_status(chip);
	if (status & TPM_STS_DATA_AVAIL) {	/* retry? */
		dev_err(chip->pdev, "Error left over data\n");
		size = -EIO;
		goto out;
	}

out:
	tpm_tis_ready(chip);
	release_locality(chip, chip->vendor.locality, 0);
	return size;
}
EXPORT_SYMBOL(tpm_tis_recv);

/*
 * If interrupts are used (signaled by an irq set in the vendor structure)
 * tpm.c can skip polling for the data to be available as the interrupt is
 * waited for here
 */
static int tpm_tis_send_data(struct tpm_chip *chip, u8 *buf, size_t len, bool itpm)
{
	int rc, status;
	size_t count = 0, burstcnt, transfer_size;

	if (request_locality(chip, 0) < 0)
		return -EBUSY;

	status = tpm_tis_status(chip);
	if ((status & TPM_STS_COMMAND_READY) == 0) {
		tpm_tis_ready(chip);
		if (wait_for_tpm_stat
		    (chip, TPM_STS_COMMAND_READY, chip->vendor.timeout_b,
		     &chip->vendor.int_queue, false) < 0) {
			rc = -ETIME;
			goto out_err;
		}
	}

	while (count < len - 1) {
		burstcnt = get_burstcount(chip);
		transfer_size = min_t (size_t, len - count - 1, burstcnt);
		write_tpm_bytes(chip,  TPM_DATA_FIFO(chip->vendor.locality), transfer_size, &buf[count]);
		count +=  transfer_size;

		wait_for_tpm_stat(chip, TPM_STS_VALID, chip->vendor.timeout_c,
				  &chip->vendor.int_queue, false);
		status = tpm_tis_status(chip);
		if (!itpm && (status & TPM_STS_DATA_EXPECT) == 0) {
			rc = -EIO;
			goto out_err;
		}
	}

	/* write last byte */
	write_tpm_byte(chip, TPM_DATA_FIFO(chip->vendor.locality), buf[count]);
	wait_for_tpm_stat(chip, TPM_STS_VALID, chip->vendor.timeout_c,
			  &chip->vendor.int_queue, false);
	status = tpm_tis_status(chip);
	if ((status & TPM_STS_DATA_EXPECT) != 0) {
		rc = -EIO;
		goto out_err;
	}

	return 0;

out_err:
	tpm_tis_ready(chip);
	release_locality(chip, chip->vendor.locality, 0);
	return rc;
}

static void disable_interrupts(struct tpm_chip *chip)
{
	u32 intmask;

	read_tpm_dword(chip, TPM_INT_ENABLE(chip->vendor.locality), &intmask);
	intmask &= ~TPM_GLOBAL_INT_ENABLE;
	write_tpm_dword(chip, TPM_INT_ENABLE(chip->vendor.locality), intmask);
	free_irq(chip->vendor.irq, chip);
	chip->vendor.irq = 0;
}

/*
 * If interrupts are used (signaled by an irq set in the vendor structure)
 * tpm.c can skip polling for the data to be available as the interrupt is
 * waited for here
 */
static int tpm_tis_send_main(struct tpm_chip *chip, u8 *buf, size_t len, bool itpm)
{
	int rc;
	u32 ordinal;
	unsigned long dur;

	rc = tpm_tis_send_data(chip, buf, len, itpm);
	if (rc < 0)
		return rc;

	/* go and do it */
	write_tpm_byte(chip, TPM_STS(chip->vendor.locality), TPM_STS_GO);

	if (chip->vendor.irq) {
		ordinal = be32_to_cpu(*((__be32 *)(buf + 6)));

		if (chip->flags & TPM_CHIP_FLAG_TPM2)
			dur = tpm2_calc_ordinal_duration(chip, ordinal);
		else
			dur = tpm_calc_ordinal_duration(chip, ordinal);

		if (wait_for_tpm_stat
		    (chip, TPM_STS_DATA_AVAIL | TPM_STS_VALID, dur,
		     &chip->vendor.read_queue, false) < 0) {
			rc = -ETIME;
			goto out_err;
		}
	}
	return len;
out_err:
	tpm_tis_ready(chip);
	release_locality(chip, chip->vendor.locality, 0);
	return rc;
}

int tpm_tis_send(struct tpm_chip *chip, u8 *buf, size_t len)
{
	int rc, irq;
	struct priv_data *priv = chip->vendor.priv;

	if (!chip->vendor.irq || priv->irq_tested)
		return tpm_tis_send_main(chip, buf, len, false);//TODO ITPM

	/* Verify receipt of the expected IRQ */
	irq = chip->vendor.irq;
	chip->vendor.irq = 0;
	rc = tpm_tis_send_main(chip, buf, len, false); //TODO ITPM
	chip->vendor.irq = irq;
	if (!priv->irq_tested)
		msleep(1);
	if (!priv->irq_tested) {
		disable_interrupts(chip);
		dev_err(chip->pdev,
			FW_BUG "TPM interrupt not working, polling instead\n");
	}
	priv->irq_tested = true;
	return rc;
}
EXPORT_SYMBOL(tpm_tis_send);

bool tpm_tis_update_timeouts(struct tpm_chip *chip, unsigned long *timeout_cap)
{
	int i;
	u32 did_vid;

	read_tpm_dword(chip, TPM_DID_VID(0), &did_vid);

	for (i = 0; i != ARRAY_SIZE(vendor_timeout_overrides); i++) {
		if (vendor_timeout_overrides[i].did_vid != did_vid)
			continue;
		memcpy(timeout_cap, vendor_timeout_overrides[i].timeout_us,
		       sizeof(vendor_timeout_overrides[i].timeout_us));
		return true;
	}

	return false;
}
EXPORT_SYMBOL(tpm_tis_update_timeouts);

bool tpm_tis_req_canceled(struct tpm_chip *chip, u8 status)
{
	switch (chip->vendor.manufacturer_id) {
	case TPM_VID_WINBOND:
		return ((status == TPM_STS_VALID) ||
			(status == (TPM_STS_VALID | TPM_STS_COMMAND_READY)));
	case TPM_VID_STM:
		return (status == (TPM_STS_VALID | TPM_STS_COMMAND_READY));
	default:
		return (status == TPM_STS_COMMAND_READY);
	}
}
EXPORT_SYMBOL(tpm_tis_req_canceled); //TODO inline?

static irqreturn_t tis_int_probe(int irq, void *dev_id)
{
	struct tpm_chip *chip = dev_id;
	u32 interrupt;

	read_tpm_dword(chip, TPM_INT_STATUS(chip->vendor.locality), &interrupt);

	if (interrupt == 0)
		return IRQ_NONE;

	chip->vendor.probed_irq = irq;

	/* Clear interrupts handled with TPM_EOI */
	write_tpm_dword(chip, TPM_INT_STATUS(chip->vendor.locality), interrupt);
	return IRQ_HANDLED;
}

static irqreturn_t tis_int_handler(int dummy, void *dev_id)
{
	struct tpm_chip *chip = dev_id;
	u32 interrupt;
	int i;

	read_tpm_dword(chip, TPM_INT_STATUS(chip->vendor.locality), &interrupt);

	if (interrupt == 0)
		return IRQ_NONE;

	((struct priv_data *)chip->vendor.priv)->irq_tested = true; //TODO CHECK
	if (interrupt & TPM_INTF_DATA_AVAIL_INT)
		wake_up_interruptible(&chip->vendor.read_queue);
	if (interrupt & TPM_INTF_LOCALITY_CHANGE_INT)
		for (i = 0; i < 5; i++)
			if (check_locality(chip, i) >= 0)
				break;
	if (interrupt &
	    (TPM_INTF_LOCALITY_CHANGE_INT | TPM_INTF_STS_VALID_INT |
	     TPM_INTF_CMD_READY_INT))
		wake_up_interruptible(&chip->vendor.int_queue);

	/* Clear interrupts handled with TPM_EOI */
	write_tpm_dword(chip, TPM_INT_STATUS(chip->vendor.locality), interrupt);
	read_tpm_dword(chip, TPM_INT_STATUS(chip->vendor.locality), &interrupt);
	return IRQ_HANDLED;
}

void tpm_tis_remove(struct tpm_chip *chip)
{
	u32 interrupt;

	if (chip->flags & TPM_CHIP_FLAG_TPM2)
		tpm2_shutdown(chip, TPM2_SU_CLEAR);

	read_tpm_dword(chip, TPM_INT_ENABLE(chip->vendor.locality), &interrupt);
	interrupt = interrupt & ~TPM_GLOBAL_INT_ENABLE;

	write_tpm_dword(chip, TPM_INT_ENABLE(chip->vendor.locality), interrupt);
	release_locality(chip, chip->vendor.locality, 1);
}
EXPORT_SYMBOL(tpm_tis_remove);

#ifdef CONFIG_PM_SLEEP //TODO check
static void tpm_tis_reenable_interrupts(struct tpm_chip *chip)
{
	u32 intmask;

	/* reenable interrupts that device may have lost or
	   BIOS/firmware may have disabled */
	write_tpm_byte(chip, TPM_INT_VECTOR(chip->vendor.locality), chip->vendor.irq);

	read_tpm_dword(chip, TPM_INT_ENABLE(chip->vendor.locality), &intmask);

	intmask |= TPM_INTF_CMD_READY_INT
	    | TPM_INTF_LOCALITY_CHANGE_INT | TPM_INTF_DATA_AVAIL_INT
	    | TPM_INTF_STS_VALID_INT | TPM_GLOBAL_INT_ENABLE;

	write_tpm_dword(chip, TPM_INT_ENABLE(chip->vendor.locality), intmask);
}

int tpm_tis_resume(struct device *dev)
{
	struct tpm_chip *chip = dev_get_drvdata(dev);
	int ret;

	if (chip->vendor.irq)
		tpm_tis_reenable_interrupts(chip);

	ret = tpm_pm_resume(dev);
	if (ret)
		return ret;

	/* TPM 1.2 requires self-test on resume. This function actually returns
	 * an error code but for unknown reason it isn't handled.
	 */
	if (!(chip->flags & TPM_CHIP_FLAG_TPM2))
		tpm_do_selftest(chip);

	return 0;
}
EXPORT_SYMBOL(tpm_tis_resume);
#endif
/*
 * Early probing for iTPM with STS_DATA_EXPECT flaw.
 * Try sending command without itpm flag set and if that
 * fails, repeat with itpm flag set.
 */
static int probe_itpm(struct tpm_chip *chip, bool *itpm)
{
	int rc = 0;
	u8 cmd_getticks[] = {
		0x00, 0xc1, 0x00, 0x00, 0x00, 0x0a,
		0x00, 0x00, 0x00, 0xf1
	};
	size_t len = sizeof(cmd_getticks);
	bool rem_itpm = *itpm;
	u16 vendor;

	read_tpm_word(chip, TPM_DID_VID(0), &vendor);

	/* probe only iTPMS */
	if (vendor != TPM_VID_INTEL)
		return 0;

	*itpm = false;

	rc = tpm_tis_send_data(chip, cmd_getticks, len, *itpm);
	if (rc == 0)
		goto out;

	tpm_tis_ready(chip);
	release_locality(chip, chip->vendor.locality, 0);

	*itpm = true;

	rc = tpm_tis_send_data(chip, cmd_getticks, len, *itpm);
	if (rc == 0) {
		dev_info(chip->pdev, "Detected an iTPM.\n");
		rc = 1;
	} else {
		rc = -EFAULT;
	}
out:
	*itpm = rem_itpm;
	tpm_tis_ready(chip);
	release_locality(chip, chip->vendor.locality, 0);

	return rc;
}

int tpm_tis_init_generic(struct device *dev, struct tpm_chip *chip, unsigned int irq, bool enable_interrupts, bool itpm)
{
	u32 vendor, intfcaps, intmask, interrupts;
	int rc, i, irq_e, probe;
	u8 rid, irq_s;

	/* Maximum timeouts */
	chip->vendor.timeout_a = TIS_TIMEOUT_A_MAX;
	chip->vendor.timeout_b = TIS_TIMEOUT_B_MAX;
	chip->vendor.timeout_c = TIS_TIMEOUT_C_MAX;
	chip->vendor.timeout_d = TIS_TIMEOUT_D_MAX;

	if (wait_startup(chip, 0) != 0) {
		rc = -ENODEV;
		goto out_err;
	}

	if (request_locality(chip, 0) != 0) {
		rc = -ENODEV;
		goto out_err;
	}

	rc = tpm2_probe(chip);
	if (rc)
		goto out_err;
//	pr_err("DID VID %x\n", vendor);
	read_tpm_dword(chip, TPM_DID_VID(0), &vendor);
//	pr_err("DID VID %x\n", vendor);
	chip->vendor.manufacturer_id = vendor;
	read_tpm_byte(chip, TPM_RID(0), &rid);

	dev_info(dev, "%s TPM (device-id 0x%X, rev-id %d)\n",
		 (chip->flags & TPM_CHIP_FLAG_TPM2) ? "2.0" : "1.2",
		 vendor >> 16, rid);
	if (!itpm) {
		probe = probe_itpm(chip, &itpm);
		if (probe < 0) {
			rc = -ENODEV;
			goto out_err;
		}
		itpm = !!probe;
	}

	if (itpm)
		dev_info(dev, "Intel iTPM workaround enabled\n");

	/* Figure out the capabilities */
	read_tpm_dword(chip, TPM_INTF_CAPS(chip->vendor.locality), &intfcaps);
	dev_dbg(dev, "TPM interface capabilities (0x%x):\n",
		intfcaps);
	if (intfcaps & TPM_INTF_BURST_COUNT_STATIC)
		dev_dbg(dev, "\tBurst Count Static\n");
	if (intfcaps & TPM_INTF_CMD_READY_INT)
		dev_dbg(dev, "\tCommand Ready Int Support\n");
	if (intfcaps & TPM_INTF_INT_EDGE_FALLING)
		dev_dbg(dev, "\tInterrupt Edge Falling\n");
	if (intfcaps & TPM_INTF_INT_EDGE_RISING)
		dev_dbg(dev, "\tInterrupt Edge Rising\n");
	if (intfcaps & TPM_INTF_INT_LEVEL_LOW)
		dev_dbg(dev, "\tInterrupt Level Low\n");
	if (intfcaps & TPM_INTF_INT_LEVEL_HIGH)
		dev_dbg(dev, "\tInterrupt Level High\n");
	if (intfcaps & TPM_INTF_LOCALITY_CHANGE_INT)
		dev_dbg(dev, "\tLocality Change Int Support\n");
	if (intfcaps & TPM_INTF_STS_VALID_INT)
		dev_dbg(dev, "\tSts Valid Int Support\n");
	if (intfcaps & TPM_INTF_DATA_AVAIL_INT)
		dev_dbg(dev, "\tData Avail Int Support\n");

	/* INTERRUPT Setup */
	init_waitqueue_head(&chip->vendor.read_queue);
	init_waitqueue_head(&chip->vendor.int_queue);

	read_tpm_dword(chip, TPM_INT_ENABLE(chip->vendor.locality), &intmask);

	intmask |= TPM_INTF_CMD_READY_INT
	    | TPM_INTF_LOCALITY_CHANGE_INT | TPM_INTF_DATA_AVAIL_INT
	    | TPM_INTF_STS_VALID_INT;

	write_tpm_dword(chip, TPM_INT_ENABLE(chip->vendor.locality), intmask);
	if (enable_interrupts)
		chip->vendor.irq = irq;
	if (enable_interrupts && !chip->vendor.irq) {
		read_tpm_byte(chip, TPM_INT_VECTOR(chip->vendor.locality), &irq_s);
		if (irq_s) {
			irq_e = irq_s;
		} else {
			irq_s = 3;
			irq_e = 15;
		}

		for (i = irq_s; i <= irq_e && chip->vendor.irq == 0; i++) {
			write_tpm_byte(chip, TPM_INT_VECTOR(chip->vendor.locality), i);
			if (devm_request_irq
			    (dev, i, tis_int_probe, IRQF_SHARED,
			     chip->devname, chip) != 0) {
				dev_info(chip->pdev,
					 "Unable to request irq: %d for probe\n",
					 i);
				continue;
			}

			/* Clear all existing */
			read_tpm_dword(chip, TPM_INT_STATUS(chip->vendor.locality), &interrupts);
			write_tpm_dword(chip, TPM_INT_STATUS(chip->vendor.locality), interrupts);

			/* Turn on */
			write_tpm_dword(chip, TPM_INT_ENABLE(chip->vendor.locality), intmask | TPM_GLOBAL_INT_ENABLE);

			chip->vendor.probed_irq = 0;

			/* Generate Interrupts */
			if (chip->flags & TPM_CHIP_FLAG_TPM2)
				tpm2_gen_interrupt(chip);
			else
				tpm_gen_interrupt(chip);

			chip->vendor.irq = chip->vendor.probed_irq;

			/* free_irq will call into tis_int_probe;
			   clear all irqs we haven't seen while doing
			   tpm_gen_interrupt */
			read_tpm_dword(chip, TPM_INT_STATUS(chip->vendor.locality), &interrupts);
			write_tpm_dword(chip, TPM_INT_STATUS(chip->vendor.locality), interrupts);

			/* Turn off */
			write_tpm_dword(chip, TPM_INT_ENABLE(chip->vendor.locality), intmask);
		}
	}
	if (chip->vendor.irq) {
		write_tpm_byte(chip, TPM_INT_VECTOR(chip->vendor.locality), chip->vendor.irq);
		if (devm_request_irq
		    (dev, chip->vendor.irq, tis_int_handler, IRQF_SHARED,
		     chip->devname, chip) != 0) {
			dev_info(chip->pdev,
				 "Unable to request irq: %d for use\n",
				 chip->vendor.irq);
			chip->vendor.irq = 0;
		} else {
			/* Clear all existing */
			read_tpm_dword(chip, TPM_INT_STATUS(chip->vendor.locality), &interrupts);
			write_tpm_dword(chip, TPM_INT_STATUS(chip->vendor.locality), interrupts);

			/* Turn on */
			write_tpm_dword(chip, TPM_INT_ENABLE(chip->vendor.locality), intmask | TPM_GLOBAL_INT_ENABLE);
		}
	}

	if (chip->flags & TPM_CHIP_FLAG_TPM2) {
		chip->vendor.timeout_a = msecs_to_jiffies(TPM2_TIMEOUT_A);
		chip->vendor.timeout_b = msecs_to_jiffies(TPM2_TIMEOUT_B);
		chip->vendor.timeout_c = msecs_to_jiffies(TPM2_TIMEOUT_C);
		chip->vendor.timeout_d = msecs_to_jiffies(TPM2_TIMEOUT_D);
		chip->vendor.duration[TPM_SHORT] =
			msecs_to_jiffies(TPM2_DURATION_SHORT);
		chip->vendor.duration[TPM_MEDIUM] =
			msecs_to_jiffies(TPM2_DURATION_MEDIUM);
		chip->vendor.duration[TPM_LONG] =
			msecs_to_jiffies(TPM2_DURATION_LONG);

		rc = tpm2_do_selftest(chip);
		if (rc == TPM2_RC_INITIALIZE) {
			dev_warn(dev, "Firmware has not started TPM\n");
			rc  = tpm2_startup(chip, TPM2_SU_CLEAR);
			if (!rc)
				rc = tpm2_do_selftest(chip);
		}

		if (rc) {
			dev_err(dev, "TPM self test failed\n");
			if (rc > 0)
				rc = -ENODEV;
			goto out_err;
		}
	} else {
		if (tpm_get_timeouts(chip)) {
			dev_err(dev, "Could not get TPM timeouts and durations\n");
			rc = -ENODEV;
			goto out_err;
		}

		if (tpm_do_selftest(chip)) {
			dev_err(dev, "TPM self test failed\n");
			rc = -ENODEV;
			goto out_err;
		}
	}

	return tpm_chip_register(chip);
out_err:
	tpm_tis_remove(chip);
	return rc;
}
EXPORT_SYMBOL(tpm_tis_init_generic);
