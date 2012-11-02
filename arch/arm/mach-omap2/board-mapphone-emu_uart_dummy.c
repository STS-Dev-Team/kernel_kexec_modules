#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kallsyms.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pm_qos_params.h>

#include <linux/spi/spi.h>
#include <plat/system.h>
#include <linux/irq.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>

#include <plat/dma.h>
#include <plat/clock.h>
#include <plat/board-mapphone-emu_uart.h>
#include <plat/hardware.h>
#include <plat/omap_hwmod.h>
#include <plat/omap34xx.h>
#include <plat/omap44xx.h>
#include <plat/mcspi.h>

#include <linux/module.h>
#include <linux/workqueue.h>

void printascii(const char *c)
{
#if 0
	while(*c)
	{
		if(*c=='\n')
			printch('\r');
		printch(*c);
		c++;
	}
#endif
}
EXPORT_SYMBOL(printascii);

static int module_emu_uart_init(void)
{

        printk(KERN_INFO "Dummy UART enabled.\n");
        return 0;
}

static void module_emu_uart_exit(void)
{
        printk(KERN_INFO "Dummy UART disabled.\n");
}

module_init(module_emu_uart_init);
module_exit(module_emu_uart_exit);

MODULE_LICENSE("GPL");

