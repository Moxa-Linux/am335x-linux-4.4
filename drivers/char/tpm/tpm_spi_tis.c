/*
 * Copyright (C) 2015 Infineon Technologies AG
 *
 * Authors:
 * Peter Huewe <peter.huewe@infineon.com>
 *
 * Maintained by: <tpmdd-devel@lists.sourceforge.net>
 *
 * Device driver for TCG/TCPA TPM (trusted platform module).
 * Specifications at www.trustedcomputinggroup.org
 *
 * This device driver implements the TPM interface as defined in
 * the TCG TPM Interface Spec version 1.3, revision 27 via _raw/native
 * SPI access_.
 *
 * It is based on the original tpm_tis device driver from Leendert van
 * Dorn and Kyleen Hall and Jarko Sakkinnen.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2 of the
 * License.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/acpi.h>
#include <linux/freezer.h>

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/tpm.h>
#include "tpm.h"
#include "tpm_tis_common.h"

#define MAX_SPI_FRAMESIZE 64

struct spi_comms {
	bool irq_tested;
	struct spi_device *spi_device;
	struct spi_transfer spi_xfer;
	u8 tx_buf[MAX_SPI_FRAMESIZE+4];
	u8 rx_buf[MAX_SPI_FRAMESIZE+4];
};


static void read_spi_bytes(struct tpm_chip *chip, u32 addr, u8 len, u8 size, u8 *result){
	struct spi_comms *comms  = TPM_VPRIV(chip);
	if (len > MAX_SPI_FRAMESIZE){
		pr_err("too large1\n");
		return; // ADD error
}
	memset(comms->tx_buf, 0, 4 + len);
	comms->spi_xfer.len = 4 + len ;
	comms->tx_buf[0] = 0x80 | (len-1); //0x80 = read  | 0=1byte
	comms->tx_buf[1] = (addr>>16) & 0xFF;
	comms->tx_buf[2] = (addr>>8)  & 0xFF;
	comms->tx_buf[3] = (addr)     & 0xFF;

	spi_sync_transfer(comms->spi_device, &comms->spi_xfer, 1);
	memcpy(result, &comms->rx_buf[4], len);
	memset(comms->rx_buf, 0, 4 + len);
}

static void write_spi_bytes(struct tpm_chip *chip, u32 addr, u8 len, u8 size, u8 *value){
	struct spi_comms *comms = TPM_VPRIV(chip);
	if (len > MAX_SPI_FRAMESIZE) {
		pr_err("too large1\n");
		return; // ADD error
}
	memset(comms->tx_buf, 0, 4 + len);
	comms->spi_xfer.len = 4 + len ;
	comms->tx_buf[0] = 0x00 | (len-1); //0x00 = write | 0 = 1byte
	comms->tx_buf[1] = (addr>>16) & 0xFF;
	comms->tx_buf[2] = (addr>>8)  & 0xFF;
	comms->tx_buf[3] = (addr)     & 0xFF;
	memcpy(&comms->tx_buf[4], value, len);

	spi_sync_transfer(comms->spi_device, &comms->spi_xfer, 1);
	memset(comms->rx_buf, 0, 4 + len);
}

static const struct tpm_class_ops tpm_tis = {
	.status = tpm_tis_status,
	.recv = tpm_tis_recv,
	.send = tpm_tis_send,
	.cancel = tpm_tis_ready,
	.req_complete_mask = TPM_STS_DATA_AVAIL | TPM_STS_VALID,
	.req_complete_val = TPM_STS_DATA_AVAIL | TPM_STS_VALID,
	.req_canceled = tpm_tis_req_canceled,
	.read_bytes = read_spi_bytes,
	.write_bytes = write_spi_bytes,
};

static bool interrupts = false;
module_param(interrupts, bool, 0444);
MODULE_PARM_DESC(interrupts, "Enable interrupts");

static SIMPLE_DEV_PM_OPS(tpm_spi_tis_pm, tpm_pm_suspend, tpm_tis_resume);

static int
tpm_tis_spi_probe(struct spi_device *dev)
{
	//int ret;
	struct spi_comms *comms;
	struct tpm_chip *chip;

	/* Check SPI platform functionnalities */
	if (!dev) {
		pr_err("%s: dev is NULL. Device is not accessible.\n",
				__func__);
		return -ENODEV;
	}
	comms = devm_kzalloc(&dev->dev, sizeof(struct spi_comms),
			   GFP_KERNEL);
	if (!comms)
		return -ENOMEM;

	chip = tpmm_chip_alloc(&dev->dev, &tpm_tis);
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	comms->spi_device = dev;
	comms->spi_xfer.tx_buf = comms->tx_buf;
	comms->spi_xfer.rx_buf = comms->rx_buf;

	TPM_VPRIV(chip) = comms;
	return tpm_tis_init_generic(&dev->dev, chip, 0, interrupts, false);
}

static int tpm_tis_spi_remove(struct spi_device *dev)
{
	struct tpm_chip *chip = spi_get_drvdata(dev);
	tpm_chip_unregister(chip);
	return 0;
}

static const struct spi_device_id tpm_tis_spi_id[] = {
	{"tpm_spi_tis", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, tpm_tis_spi_id);

#ifdef CONFIG_OF
static const struct of_device_id of_tis_spi_match[] = {
	{ .compatible = "infineon,slb9670", },
	{ .compatible = "tcg,tpm_spi_tis", },
	{ .compatible = "tcg,tpm_tis-spi", },
	{}
};
MODULE_DEVICE_TABLE(of, of_tis_spi_match);
#endif

static struct spi_driver tpm_tis_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tpm_spi_tis",
		.pm = &tpm_spi_tis_pm,
		.of_match_table = of_match_ptr(of_tis_spi_match),
	},
	.probe = tpm_tis_spi_probe,
	.remove = tpm_tis_spi_remove,
	.id_table = tpm_tis_spi_id,
};

module_spi_driver(tpm_tis_spi_driver);
MODULE_AUTHOR("Peter Huewe (peter.huewe@infineon.com)");
MODULE_DESCRIPTION("TPM Driver for native SPI access");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
