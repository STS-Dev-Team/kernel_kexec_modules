#ifndef _OMAP2_MCSPI_H
#define _OMAP2_MCSPI_H

#define OMAP2_MCSPI_MASTER		0
#define OMAP2_MCSPI_SLAVE		1

#define OMAP2_MCSPI_REV 0
#define OMAP3_MCSPI_REV 1
#define OMAP4_MCSPI_REV 2

#define OMAP4_MCSPI_REG_OFFSET 0x100

#define OMAP2_MCSPI_MAX_FREQ		48000000
#define OMAP2_MCSPI_MAX_FIFODEPTH	64

// Moved from drivers/spi/omap2_mcspi.c for EMU_UART
/* OMAP2 has 3 SPI controllers, while OMAP3 has 4 */
#define OMAP2_MCSPI_MAX_CTRL 		4

#define OMAP2_MCSPI_REVISION		0x00
#define OMAP2_MCSPI_SYSCONFIG		0x10
#define OMAP2_MCSPI_SYSSTATUS		0x14
#define OMAP2_MCSPI_IRQSTATUS		0x18
#define OMAP2_MCSPI_IRQENABLE		0x1c
#define OMAP2_MCSPI_WAKEUPENABLE	0x20
#define OMAP2_MCSPI_SYST		0x24
#define OMAP2_MCSPI_MODULCTRL		0x28

/* per-channel banks, 0x14 bytes each, first is: */
#define OMAP2_MCSPI_CHCONF0		0x2c
#define OMAP2_MCSPI_CHSTAT0		0x30
#define OMAP2_MCSPI_CHCTRL0		0x34
#define OMAP2_MCSPI_TX0			0x38
#define OMAP2_MCSPI_RX0			0x3c
#define OMAP2_MCSPI_XFERLEVEL		0x7c


/* per-register bitmasks: */
#define OMAP2_MCSPI_SYSCONFIG_SMARTIDLE	BIT(4)
#define OMAP2_MCSPI_SYSCONFIG_NOIDLE	BIT(3)
#define OMAP2_MCSPI_SYSCONFIG_ENAWAKEUP	BIT(2)
#define OMAP2_MCSPI_SYSCONFIG_AUTOIDLE	BIT(0)
#define OMAP2_MCSPI_SYSCONFIG_SOFTRESET	BIT(1)

#define OMAP2_MCSPI_SYSSTATUS_RESETDONE	BIT(0)

#define OMAP2_MCSPI_MODULCTRL_SINGLE	BIT(0)
#define OMAP2_MCSPI_MODULCTRL_MS	BIT(2)
#define OMAP2_MCSPI_MODULCTRL_STEST	BIT(3)

#define OMAP2_MCSPI_CHCONF_PHA		BIT(0)
#define OMAP2_MCSPI_CHCONF_POL		BIT(1)
#define OMAP2_MCSPI_CHCONF_CLKD_MASK	(0x0f << 2)
#define OMAP2_MCSPI_CHCONF_EPOL		BIT(6)
#define OMAP2_MCSPI_CHCONF_WL_MASK	(0x1f << 7)
#define OMAP2_MCSPI_CHCONF_TRM_RX_ONLY	BIT(12)
#define OMAP2_MCSPI_CHCONF_TRM_TX_ONLY	BIT(13)
#define OMAP2_MCSPI_CHCONF_TRM_MASK	(0x03 << 12)
#define OMAP2_MCSPI_CHCONF_DMAW		BIT(14)
#define OMAP2_MCSPI_CHCONF_DMAR		BIT(15)
#define OMAP2_MCSPI_CHCONF_DPE0		BIT(16)
#define OMAP2_MCSPI_CHCONF_DPE1		BIT(17)
#define OMAP2_MCSPI_CHCONF_IS		BIT(18)
#define OMAP2_MCSPI_CHCONF_TURBO	BIT(19)
#define OMAP2_MCSPI_CHCONF_FORCE	BIT(20)
#define OMAP2_MCSPI_CHCONF_TCS0		BIT(25)
#define OMAP2_MCSPI_CHCONF_TCS1		BIT(26)
#define OMAP2_MCSPI_CHCONF_TCS_MASK	(0x03 << 25)
#define OMAP2_MCSPI_CHCONF_FFET		BIT(27)
#define OMAP2_MCSPI_CHCONF_FFER		BIT(28)

#define OMAP2_MCSPI_CHSTAT_RXS		BIT(0)
#define OMAP2_MCSPI_CHSTAT_TXS		BIT(1)
#define OMAP2_MCSPI_CHSTAT_EOT		BIT(2)

#define OMAP2_MCSPI_IRQ_EOW		BIT(17)

#define OMAP2_MCSPI_CHCTRL_EN		BIT(0)

#define OMAP2_MCSPI_WAKEUPENABLE_WKEN	BIT(0)


struct omap2_mcspi_platform_config {
	unsigned short	num_cs;
	unsigned short	mode;
	unsigned short	dma_mode;
	unsigned short	force_cs_mode;
	unsigned short	fifo_depth;
	unsigned int	regs_offset;
	unsigned int	*regs_data;
};

struct omap2_mcspi_dev_attr {
	unsigned short num_chipselect;
};

struct omap2_mcspi_device_config {
	unsigned turbo_mode:1;

	/* Do we want one channel enabled at the same time? */
	unsigned single_channel:1;
	/* Swap data lines */
	unsigned swap_datalines;
};

#endif
