/*
 * machine_kexec.c - handle transition of Linux booting another kernel
 */

#include <linux/module.h>
#include <linux/mm.h>
#include <linux/kexec.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/kallsyms.h>
#include <linux/smp.h>
#include <linux/uaccess.h>
#include <linux/cpu.h>

#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/mmu_context.h>
#include <asm/cacheflush.h>
#include <asm/mach-types.h>
#include <asm/kexec.h>
#include <asm/cputype.h>
#include <asm/hardware/cache-l2x0.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <plat/omap44xx.h>

//#define CONFIG_DIRTYKEXEC //TODO: WTF

extern void relocate_new_kernel(void);
extern const unsigned int relocate_new_kernel_size;

extern unsigned long kexec_start_address;
extern unsigned long kexec_indirection_page;
extern unsigned long kexec_mach_type;
extern unsigned long kexec_boot_atags;

void kexec_cpu_proc_fin(void);
void kexec_cpu_reset(void);

extern void call_with_stack(void (*fn)(void *), void *arg, void *sp);
typedef void (*phys_reset_t)(void);

static DEFINE_SPINLOCK(main_lock);
void __iomem *myL2CacheBase = NULL;
static uint32_t kexec_l2x0_way_mask = (1 << 16 ) - 1;  /* Bitmask of active PL310 ways (on RAZR..) */

void flushcachesinit(void);
void setwayflush(void);

void pulse(void)
{
	asm volatile(
		"ldr	r3, =0x4a310000		\n\t"
		"mov	r4, #0x10000000		\n\t"
		"str	r4, [r3, #0x190]	\n\t"
		"str	r4, [r3, #0x194]	\n\t"
		::: "r3", "r4"
	);
}

/*
 * A temporary stack to use for CPU reset. This is static so that we
 * don't clobber it with the identity mapping. When running with this
 * stack, any references to the current task *will not work* so you
 * should really do as little as possible before jumping to your reset
 * code.
 */
static u32 soft_restart_stack[256];

static void kexec_info(struct kimage *image)
{
	int i;
	printk("kexec information\n");
	for (i = 0; i < image->nr_segments; i++) {
	        printk("  segment[%d]: 0x%08x - 0x%08x (0x%08x)\n",
		       i,
		       (unsigned int)image->segment[i].mem,
		       (unsigned int)image->segment[i].mem +
				     image->segment[i].memsz,
		       (unsigned int)image->segment[i].memsz);
	}
	printk("  start     : 0x%08x\n", (unsigned int)image->start);
	printk("  atags     : 0x%08x\n", (unsigned int)image->start - KEXEC_ARM_ZIMAGE_OFFSET + KEXEC_ARM_ATAGS_OFFSET);
	printk("machine_arch_type: %04x\n", machine_arch_type);
}

/*
 * Provide a dummy crash_notes definition while crash dump arrives to arm.
 * This prevents breakage of crash_notes attribute in kernel/ksysfs.c.
 */

int machine_kexec_prepare(struct kimage *image)
{
	kexec_info(image);
	return 0;
}
EXPORT_SYMBOL(machine_kexec_prepare);

void machine_kexec_cleanup(struct kimage *image)
{
}
EXPORT_SYMBOL(machine_kexec_cleanup);

void machine_shutdown(void)
{
}
EXPORT_SYMBOL(machine_shutdown);

void machine_crash_shutdown(struct pt_regs *regs)
{
}
EXPORT_SYMBOL(machine_crash_shutdown);

int cpu_architecture(void)
{
	int cpu_arch;

	if ((read_cpuid_id() & 0x0008f000) == 0) {
		cpu_arch = CPU_ARCH_UNKNOWN;
	} else if ((read_cpuid_id() & 0x0008f000) == 0x00007000) {
		cpu_arch = (read_cpuid_id() & (1 << 23)) ? CPU_ARCH_ARMv4T : CPU_ARCH_ARMv3;
	} else if ((read_cpuid_id() & 0x00080000) == 0x00000000) {
		cpu_arch = (read_cpuid_id() >> 16) & 7;
		if (cpu_arch)
			cpu_arch += CPU_ARCH_ARMv3;
	} else if ((read_cpuid_id() & 0x000f0000) == 0x000f0000) {
		unsigned int mmfr0;

		/* Revised CPUID format. Read the Memory Model Feature
		 * Register 0 and check for VMSAv7 or PMSAv7 */
		asm("mrc	p15, 0, %0, c0, c1, 4"
		    : "=r" (mmfr0));
		if ((mmfr0 & 0x0000000f) >= 0x00000003 ||
		    (mmfr0 & 0x000000f0) == 0x00000030)
			cpu_arch = CPU_ARCH_ARMv7;
		else if ((mmfr0 & 0x0000000f) == 0x00000002 ||
			 (mmfr0 & 0x000000f0) == 0x00000020)
			cpu_arch = CPU_ARCH_ARMv6;
		else
			cpu_arch = CPU_ARCH_UNKNOWN;
	} else
		cpu_arch = CPU_ARCH_UNKNOWN;

	return cpu_arch;
}

/*
 * In order to soft-boot, we need to insert a 1:1 mapping in place of
 * the user-mode pages.  This will then ensure that we have predictable
 * results when turning the mmu off
 */
void identity_map(unsigned long phys_addr)
{
	pgd_t *pgd;
	pmd_t *pmd;

	int prot = PMD_SECT_AP_WRITE | PMD_SECT_AP_READ | PMD_TYPE_SECT;
	unsigned long phys = phys_addr & PMD_MASK;

	if (cpu_architecture() <= CPU_ARCH_ARMv5TEJ && !cpu_is_xscale())
		prot |= PMD_BIT4;

	/*
	 * We need to access to user-mode page tables here. For kernel threads
	 * we don't have any user-mode mappings so we use the context that we
	 * "borrowed".
	 */

	pgd = pgd_offset(current->active_mm, phys);
	pmd = pmd_offset(pgd, phys);
	pmd[0] = __pmd(phys | prot);
	pmd[1] = __pmd((phys + (1 << (PGDIR_SHIFT - 1))) | prot);

	flush_pmd_entry(pmd);

	local_flush_tlb_all();
}

#define UART3_BASE 0x48020000 
#define UART_LSR  0x14
#define UART_THR   0
#define UART_LSR_THRE	0x20

extern void printascii(char*);

static void (*kex_setup_mm_for_reboot)(char mode);
static void __soft_restart(void *addr)
{
	phys_reset_t phys_reset = (phys_reset_t)addr;

	while(((*(volatile u32*)(UART3_BASE+UART_LSR))&UART_LSR_THRE)==0)
	{}
	*(volatile u32*)(UART3_BASE+UART_THR)='A';


	while(((*(volatile u32*)(UART3_BASE+UART_LSR))&UART_LSR_THRE)==0)
	{}
	*(volatile u32*)(UART3_BASE+UART_THR)='B';

	/* Clean and invalidate caches */
	flush_cache_all();

//	while(((*(volatile u32*)(UART3_BASE+UART_LSR))&UART_LSR_THRE)==0)
//	{}
//	*(volatile u32*)(UART3_BASE+UART_THR)='C';

	/* Turn off caching */
	kexec_cpu_proc_fin();

//	while(((*(volatile u32*)(UART3_BASE+UART_LSR))&UART_LSR_THRE)==0)
//	{}
//	*(volatile u32*)(UART3_BASE+UART_THR)='D';


	/* Push out any further dirty data, and ensure cache is empty */
	flush_cache_all();

	while(((*(volatile u32*)(UART3_BASE+UART_LSR))&UART_LSR_THRE)==0)
	{}
	*(volatile u32*)(UART3_BASE+UART_THR)='E';


//	phys_reset = (phys_reset_t)(unsigned long)virt_to_phys(cpu_reset);
	phys_reset();//(unsigned long)addr);

	while(((*(volatile u32*)(UART3_BASE+UART_LSR))&UART_LSR_THRE)==0)
	{}
	*(volatile u32*)(UART3_BASE+UART_THR)='Z';


	/* Should never get here. */
	BUG();
}

void outer_disable(void) 
{
	omap_smc1(0x102, 0x0);
}

static inline void kexec_cache_wait_way(void __iomem *reg, unsigned long mask)
{
        /* wait for cache operation by line or way to complete */
        while (readl_relaxed(reg) & mask)
                ;
}

static inline void kexec_cache_sync_always(void)
{
        void __iomem *base = myL2CacheBase;
        writel_relaxed(0, base + L2X0_CACHE_SYNC);
        kexec_cache_wait_way(base + L2X0_CACHE_SYNC, 1);
}

static inline void kexec_l2x0_inv_all(void)
{
        unsigned long flags;

        /* invalidate all ways
         * Since the L2 is disabled, exclusice accessors may
         * not be available to us, so avoid taking any locks.
         * Don't do spin_lock_irqsave(&l2x0_lock, flags);
         */
		BUG_ON(readl(myL2CacheBase + L2X0_CTRL) & 1);
        writel_relaxed(kexec_l2x0_way_mask, myL2CacheBase + L2X0_INV_WAY);
        kexec_cache_wait_way(myL2CacheBase + L2X0_INV_WAY, kexec_l2x0_way_mask);
        kexec_cache_sync_always();

        /* And don't do spin_unlock_irqrestore(&l2x0_lock, flags); */
}

static inline void kexec_l2x0_flush_all(void)
{
	writel_relaxed(kexec_l2x0_way_mask, myL2CacheBase + L2X0_CLEAN_INV_WAY);
	kexec_cache_wait_way(myL2CacheBase + L2X0_CLEAN_INV_WAY, kexec_l2x0_way_mask);
	kexec_cache_sync_always();
}

static inline void invalidate_icache_all(void)
{
#ifdef CONFIG_ARM_ERRATA_411920
	extern void v6_icache_inval_all(void);
	v6_icache_inval_all();
#elif defined(CONFIG_SMP) && __LINUX_ARM_ARCH__ >= 7

	asm volatile ("mcr p15, 0, %0, c7, c5, 0" : : "r" (0));

	/* Invalidate entire branch predictor array */

	asm volatile ("mcr p15, 0, %0, c7, c5, 6" : : "r" (0));

#else
	asm("mcr	p15, 0, %0, c7, c5, 0	@ invalidate I-cache\n"
	    :
	    : "r" (0));
#endif
}

static inline void invalidate_foo_all(void)
{

	asm volatile ("mcr p15, 0, %0, c7, c5, 0" : : "r" (0));

	/* Invalidate entire branch predictor array */

	asm volatile ("mcr p15, 0, %0, c7, c5, 6" : : "r" (0));

	
}

void soft_restart(unsigned long addr)
{
	u32 *stack = (u32 *)(soft_restart_stack + ARRAY_SIZE(soft_restart_stack));
	unsigned long flags;
	kex_setup_mm_for_reboot = (void *)kallsyms_lookup_name("setup_mm_for_reboot");

	//Invalidate flush and disable l1


	printascii("L2X0 Access\n");
//Do this first so the ioremap will be around
	flushcachesinit(); 

	printascii("L2X0 Disable\n");
	/* Disable the L2 if we're the last man standing. */
	if (num_online_cpus() == 1) 
	{	
		kexec_l2x0_flush_all();
		outer_disable();
		kexec_l2x0_flush_all();

		kexec_l2x0_inv_all();
	}
	else
	{
		printascii("***********Multiple CPU's online!!!***************\n");
	}

	printascii("Switch mm\n");

	/* Take out a flat memory mapping. */
	kex_setup_mm_for_reboot(0);

	printascii("Switch stack\n");
	/* Change to the new stack and continue with the reset. */
	call_with_stack(__soft_restart, (void *)addr, (void *)stack);

	/* Should never get here. */
	BUG();
}


void flushcachesinit(void)
{
	myL2CacheBase=ioremap(OMAP44XX_L2CACHE_BASE,SZ_4K);
	BUG_ON(!myL2CacheBase);

	set_fs(KERNEL_DS);
}

void setwayflush(void)
{
	__cpuc_flush_kern_all();
		__asm__("dsb\n"
		"dmb\n"
		"isb\n"
		"nop\n"
		"nop\n"
		"ldr     r0, =0xffff\n"
		"str     r0, [%0, %2]\n"
		"2:\n"
		"ldr     r0, [%0, %2]\n"
		"cmp     r0, #0\n"
		"bne     2b\n"
		"nop\n"
		"nop\n"
		"mov     r0, #0x0\n"
		"str     r0, [%0, %1]\n"
		"1:\n"
		"ldr     r0, [%0, %1]\n"
		"ands    r0, r0, #0x1\n"
		"bne     1b\n"
		"nop\n"
		"nop\n"
		:
		: "r"(myL2CacheBase), "J"(L2X0_CACHE_SYNC), "J"(L2X0_CLEAN_INV_WAY)
		: "r0");
	__cpuc_flush_kern_all();
	__asm__("dsb\n");
}


void machine_kexec(struct kimage *image)
{
	unsigned long page_list;
	unsigned long reboot_code_buffer_phys;
	void *reboot_code_buffer;
	char buf[256];

	page_list = image->head & PAGE_MASK;

	/* Disable preemption */
	preempt_disable();

	printascii("Disable IRQ's\n");
	/* Disable interrupts first */
	local_irq_disable();
	local_fiq_disable();

	/* we need both effective and real address here */
	reboot_code_buffer_phys =
	    page_to_pfn(image->control_code_page) << PAGE_SHIFT;
	reboot_code_buffer = page_address(image->control_code_page);

	sprintf(buf, "va: %08x\n", (int)reboot_code_buffer);
	printascii(buf);
	sprintf(buf, "pa: %08x\n", (int)reboot_code_buffer_phys);
	printascii(buf);

	/* Prepare parameters for reboot_code_buffer*/
	kexec_start_address = image->start;
	kexec_indirection_page = page_list;
	kexec_mach_type = machine_arch_type;
	kexec_boot_atags = image->start - KEXEC_ARM_ZIMAGE_OFFSET + KEXEC_ARM_ATAGS_OFFSET;

	printascii("Bye!\n");

	/* Take out a flat memory mapping. */
	identity_map(UART3_BASE); // GPIO for pulse
	identity_map(reboot_code_buffer_phys);

//TODO: wtf does this do? copy relocate onto itself?
	memcpy(reboot_code_buffer, relocate_new_kernel, relocate_new_kernel_size);

	printascii("Flush Icache\n");
	invalidate_icache_all();

	soft_restart(reboot_code_buffer_phys);
}

static int __init arm_kexec_init(void)
{
	void (*set_cpu_online_ptr)(unsigned int cpu, bool online) = (void *)kallsyms_lookup_name("set_cpu_online");
	void (*set_cpu_present_ptr)(unsigned int cpu, bool present) = (void *)kallsyms_lookup_name("set_cpu_present");
	void (*set_cpu_possible_ptr)(unsigned int cpu, bool possible) = (void *)kallsyms_lookup_name("set_cpu_possible");
	
	int (*disable_nonboot_cpus)(void) = (void *)kallsyms_lookup_name("disable_nonboot_cpus");
	int nbcval = 0;

	nbcval = disable_nonboot_cpus();
	if (nbcval < 0)
		printk(KERN_INFO "!!!WARNING!!! disable_nonboot_cpus have FAILED!\n \
				  Continuing to boot anyway: something can go wrong!\n");

	set_cpu_online_ptr(1, false);
	set_cpu_present_ptr(1, false);
	set_cpu_possible_ptr(1, false);

	return 0;
}

static void __exit arm_kexec_exit(void)
{
}

module_init(arm_kexec_init);
module_exit(arm_kexec_exit);

EXPORT_SYMBOL(machine_kexec);
MODULE_LICENSE("GPL");
