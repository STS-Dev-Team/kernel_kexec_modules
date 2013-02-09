/* drivers/android/ram_console.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/console.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/platform_data/ram_console.h>
#include <linux/kallsyms.h>
#include "../../../../arch/arm/mach-omap2/omap_ram_console.h"

#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
#include <linux/rslib.h>
#endif

#include <asm/bootinfo.h>

struct ram_console_buffer {
	uint32_t    sig;
	uint32_t    start;
	uint32_t    size;
	uint8_t     data[0];
};

#define RAM_CONSOLE_SIG (0x43474244) /* DBGC */
#define OMAP_KEXEC_RAM_CONSOLE_START_DEFAULT	(0x80000000 + SZ_256M)
#define OMAP_KEXEC_RAM_CONSOLE_SIZE_DEFAULT	SZ_2M

// Function Pointers
long (*kexec_memblock_remove)(phys_addr_t base, phys_addr_t size);
int (*kexec_decode_rs8)(struct rs_control *rs, uint8_t *data, uint16_t *par, int len,
		uint16_t *s, int no_eras, int *eras_pos, uint16_t invmsk,
		uint16_t *corr);
struct rs_control *(*kexec_init_rs)(int symsize, int gfpoly, int fcr, int prim,
		int nroots);

// proc entry
struct proc_dir_entry *entry;

static char *ram_console_old_log;
static size_t ram_console_old_log_size;

static struct ram_console_buffer *ram_console_buffer;
static size_t ram_console_buffer_size;
static char *ram_console_par_buffer;
static struct rs_control *ram_console_rs_decoder;
static int ram_console_corrected_bytes;
static int ram_console_bad_blocks;
#define ECC_BLOCK_SIZE CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION_DATA_SIZE
#define ECC_SIZE CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION_ECC_SIZE
#define ECC_SYMSIZE CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION_SYMBOL_SIZE
#define ECC_POLY CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION_POLYNOMIAL

static int ram_console_decode_rs8(void *data, size_t len, uint8_t *ecc)
{
	int i;
	uint16_t par[ECC_SIZE];
	for (i = 0; i < ECC_SIZE; i++)
		par[i] = ecc[i];
	return kexec_decode_rs8(ram_console_rs_decoder, data, par, len,
				NULL, 0, NULL, 0, NULL);
}

static void __init
ram_console_save_old(struct ram_console_buffer *buffer, const char *bootinfo,
	char *dest)
{
	size_t old_log_size = buffer->size;
	size_t bootinfo_size = 0;
	size_t total_size = old_log_size;
	char *ptr;
	const char *bootinfo_label = "Boot info:\n";

	uint8_t *block;
	uint8_t *par;
	char strbuf[80];
	int strbuf_len = 0;

	block = buffer->data;
	par = ram_console_par_buffer;
	while (block < buffer->data + buffer->size) {
		int numerr;
		int size = ECC_BLOCK_SIZE;
		if (block + size > buffer->data + ram_console_buffer_size)
			size = buffer->data + ram_console_buffer_size - block;
		numerr = ram_console_decode_rs8(block, size, par);
		if (numerr > 0) {
			printk(KERN_INFO "ram_console: error in block %p, %d\n",
			       block, numerr);
			ram_console_corrected_bytes += numerr;
		} else if (numerr < 0) {
			printk(KERN_INFO "ram_console: uncorrectable error in "
			       "block %p\n", block);
			ram_console_bad_blocks++;
		}
		block += ECC_BLOCK_SIZE;
		par += ECC_SIZE;
	}
	if (ram_console_corrected_bytes || ram_console_bad_blocks)
		strbuf_len = snprintf(strbuf, sizeof(strbuf),
			"\n%d Corrected bytes, %d unrecoverable blocks\n",
			ram_console_corrected_bytes, ram_console_bad_blocks);
	else
		strbuf_len = snprintf(strbuf, sizeof(strbuf),
				      "\nNo errors detected\n");
	if (strbuf_len >= sizeof(strbuf))
		strbuf_len = sizeof(strbuf) - 1;
	total_size += strbuf_len;

	if (bootinfo)
		bootinfo_size = strlen(bootinfo) + strlen(bootinfo_label);
	total_size += bootinfo_size;

	if (dest == NULL) {
		dest = kmalloc(total_size, GFP_KERNEL);
		if (dest == NULL) {
			printk(KERN_ERR
			       "ram_console: failed to allocate buffer\n");
			return;
		}
	}

	ram_console_old_log = dest;
	ram_console_old_log_size = total_size;
	memcpy(ram_console_old_log,
	       &buffer->data[buffer->start], buffer->size - buffer->start);
	memcpy(ram_console_old_log + buffer->size - buffer->start,
	       &buffer->data[0], buffer->start);
	ptr = ram_console_old_log + old_log_size;
	memcpy(ptr, strbuf, strbuf_len);
	ptr += strbuf_len;
	if (bootinfo) {
		memcpy(ptr, bootinfo_label, strlen(bootinfo_label));
		ptr += strlen(bootinfo_label);
		memcpy(ptr, bootinfo, bootinfo_size);
		ptr += bootinfo_size;
	}
}

static int __init ram_console_init(struct ram_console_buffer *buffer,
				   size_t buffer_size, const char *bootinfo,
				   char *old_buf)
{
	int numerr;
	uint8_t *par;
	ram_console_buffer = buffer;
	ram_console_buffer_size =
		buffer_size - sizeof(struct ram_console_buffer);

	if (ram_console_buffer_size > buffer_size) {
		pr_err("ram_console: buffer %p, invalid size %zu, "
		       "datasize %zu\n", buffer, buffer_size,
		       ram_console_buffer_size);
		return 0;
	}

	ram_console_buffer_size -= (DIV_ROUND_UP(ram_console_buffer_size,
						ECC_BLOCK_SIZE) + 1) * ECC_SIZE;

	if (ram_console_buffer_size > buffer_size) {
		pr_err("ram_console: buffer %p, invalid size %zu, "
		       "non-ecc datasize %zu\n",
		       buffer, buffer_size, ram_console_buffer_size);
		return 0;
	}

	ram_console_par_buffer = buffer->data + ram_console_buffer_size;


	/* first consecutive root is 0
	 * primitive element to generate roots = 1
	 */
	ram_console_rs_decoder = kexec_init_rs(ECC_SYMSIZE, ECC_POLY, 0, 1, ECC_SIZE);
	if (ram_console_rs_decoder == NULL) {
		printk(KERN_INFO "ram_console: init_rs failed\n");
		return 0;
	}

	ram_console_corrected_bytes = 0;
	ram_console_bad_blocks = 0;

	par = ram_console_par_buffer +
	      DIV_ROUND_UP(ram_console_buffer_size, ECC_BLOCK_SIZE) * ECC_SIZE;

	numerr = ram_console_decode_rs8(buffer, sizeof(*buffer), par);

	if (numerr > 0) {
		printk(KERN_INFO "ram_console: error in header, %d\n", numerr);
		ram_console_corrected_bytes += numerr;
	} else if (numerr < 0) {
		printk(KERN_INFO
		       "ram_console: uncorrectable error in header\n");
		ram_console_bad_blocks++;
	}

	if (buffer->size > ram_console_buffer_size
	    || buffer->start > buffer->size)
		printk(KERN_INFO "ram_console: found existing invalid "
		       "buffer, size %d, start %d\n",
		       buffer->size, buffer->start);
	else {
		printk(KERN_INFO "ram_console: found existing buffer, "
		       "size %d, start %d\n",
		       buffer->size, buffer->start);
		ram_console_save_old(buffer, bootinfo, old_buf);
	}

	if (buffer->sig != RAM_CONSOLE_SIG)
		printk(KERN_INFO "ram_console: no valid data in buffer "
		       "(sig = 0x%08x)\n", buffer->sig);

	buffer->sig = RAM_CONSOLE_SIG;
	buffer->start = 0;
	buffer->size = 0;

	return 0;
}

static ssize_t kexec_ram_console_read_old(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

	if (pos >= ram_console_old_log_size)
		return 0;

	count = min(len, (size_t)(ram_console_old_log_size - pos));
	if (copy_to_user(buf, ram_console_old_log + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static const struct file_operations kexec_ram_console_file_ops = {
	.owner = THIS_MODULE,
	.read = kexec_ram_console_read_old,
};

static struct resource kexec_ram_console_resources[] = {
	{
		.flags  = IORESOURCE_MEM,
		.start  = OMAP_KEXEC_RAM_CONSOLE_START_DEFAULT,
		.end    = OMAP_KEXEC_RAM_CONSOLE_START_DEFAULT +
			  OMAP_KEXEC_RAM_CONSOLE_SIZE_DEFAULT - 1,
	},
};

static struct ram_console_platform_data kexec_ram_console_pdata;

static struct platform_device kexec_ram_console_device = {
	.name		= "kexec_ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(kexec_ram_console_resources),
	.resource	= kexec_ram_console_resources,
	.dev		= {
	.platform_data	= &kexec_ram_console_pdata,
	},
};

static int __init kexec_ram_console_late_init(void)
{
	int ret;
	struct resource *res;
	size_t start;
	size_t buffer_size;
	void *buffer;

	kexec_init_rs = (void *)kallsyms_lookup_name("init_rs");
	kexec_decode_rs8 = (void *)kallsyms_lookup_name("decode_rs8");
	if (kexec_decode_rs8 == NULL || kexec_init_rs == NULL) {
		printk(KERN_ERR
		       "ram_console: failed to find init_rs or decode_rs8\n");
		return 0;
	}
	kexec_memblock_remove = (void *)kallsyms_lookup_name("memblock_remove");
	if (kexec_memblock_remove == NULL) {
		printk(KERN_ERR
		       "ram_console: failed to find memblock_remove\n");
		return 0;
	}

	printk(KERN_INFO
	       "ram_console: Using buffer settings: start=0x%x, size=%u\n",
			OMAP_KEXEC_RAM_CONSOLE_START_DEFAULT,
			OMAP_KEXEC_RAM_CONSOLE_SIZE_DEFAULT);

	/* Remove the ram console region from kernel's map */
	ret = kexec_memblock_remove(OMAP_KEXEC_RAM_CONSOLE_START_DEFAULT, OMAP_KEXEC_RAM_CONSOLE_SIZE_DEFAULT);
	if (ret) {
		pr_err("%s: unable to remove memory for ram console:"
			"start=0x%08x, size=0x%08x, ret=%d\n",
			__func__, (u32)OMAP_KEXEC_RAM_CONSOLE_START_DEFAULT, (u32)OMAP_KEXEC_RAM_CONSOLE_SIZE_DEFAULT, ret);
		return ret;
	}

	ret = platform_device_register(&kexec_ram_console_device);
	if (ret) {
		pr_err("%s: unable to register kexec ram console device:"
			"start=0x%08x, end=0x%08x, ret=%d\n",
			__func__, (u32)kexec_ram_console_resources[0].start,
			(u32)kexec_ram_console_resources[0].end, ret);
	}

	res = &kexec_ram_console_resources[0];

	buffer_size = res->end - res->start + 1;
	start = res->start;
	printk(KERN_INFO "ram_console: got buffer at %zx, size %zx\n",
	       start, buffer_size);
	buffer = ioremap(res->start, buffer_size);
	if (buffer == NULL) {
		printk(KERN_ERR "ram_console: failed to map memory\n");
		return -ENOMEM;
	}

	ram_console_init(buffer, buffer_size, NULL, NULL/* allocate */);

	entry = create_proc_entry("last_kmsg", S_IFREG | S_IRUGO, NULL);
	if (!entry) {
		printk(KERN_ERR "ram_console: failed to create proc entry\n");
		kfree(ram_console_old_log);
		ram_console_old_log = NULL;
		return 0;
	}

	entry->proc_fops = &kexec_ram_console_file_ops;
	entry->size = ram_console_old_log_size;
	return 0;
}

static void __exit kexec_ram_console_exit(void) {
	if (entry)
		remove_proc_entry("last_kmsg", NULL);
	platform_device_unregister(&kexec_ram_console_device);
	entry = NULL;
}

module_init(kexec_ram_console_late_init);
module_exit(kexec_ram_console_exit);

MODULE_LICENSE("GPL");
