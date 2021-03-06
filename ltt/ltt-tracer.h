/*
 * Copyright (C) 2005,2006,2008 Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * This contains the definitions for the Linux Trace Toolkit tracer.
 *
 * Dual LGPL v2.1/GPL v2 license.
 */

#ifndef _LTT_TRACER_H
#define _LTT_TRACER_H

#include <stdarg.h>
#include <linux/types.h>
#include <linux/limits.h>
#include <linux/list.h>
#include <linux/cache.h>
#include <linux/kernel.h>
#include <linux/timex.h>
#include <linux/wait.h>
#include <linux/marker.h>
#include <linux/trace-clock.h>
#include <linux/ltt-channels.h>
#include <asm/atomic.h>
#include <asm/local.h>

#include "ltt-tracer-core.h"
#include "ltt-relay.h"

/* Number of bytes to log with a read/write event */
#define LTT_LOG_RW_SIZE			32L

/* Interval (in jiffies) at which the LTT per-CPU timer fires */
#define LTT_PERCPU_TIMER_INTERVAL	1

#ifndef LTT_ARCH_TYPE
#define LTT_ARCH_TYPE			LTT_ARCH_TYPE_UNDEFINED
#endif

#ifndef LTT_ARCH_VARIANT
#define LTT_ARCH_VARIANT		LTT_ARCH_VARIANT_NONE
#endif

struct ltt_active_marker;

/* Maximum number of callbacks per marker */
#define LTT_NR_CALLBACKS	10

struct ltt_serialize_closure {
	ltt_serialize_cb *callbacks;
	long cb_args[LTT_NR_CALLBACKS];
	unsigned int cb_idx;
};

size_t ltt_serialize_data(struct ltt_chanbuf *buf, size_t buf_offset,
			  struct ltt_serialize_closure *closure,
			  void *serialize_private, unsigned int stack_pos_ctx,
			  int *largest_align, const char *fmt, va_list *args);

struct ltt_available_probe {
	const char *name;		/* probe name */
	const char *format;
	marker_probe_func *probe_func;
	ltt_serialize_cb callbacks[LTT_NR_CALLBACKS];
	struct list_head node;		/* registered probes list */
};

enum ltt_channels {
	LTT_CHANNEL_METADATA,
	LTT_CHANNEL_FD_STATE,
	LTT_CHANNEL_GLOBAL_STATE,
	LTT_CHANNEL_IRQ_STATE,
	LTT_CHANNEL_MODULE_STATE,
	LTT_CHANNEL_NETIF_STATE,
	LTT_CHANNEL_SOFTIRQ_STATE,
	LTT_CHANNEL_SWAP_STATE,
	LTT_CHANNEL_SYSCALL_STATE,
	LTT_CHANNEL_TASK_STATE,
	LTT_CHANNEL_VM_STATE,
	LTT_CHANNEL_FS,
	LTT_CHANNEL_INPUT,
	LTT_CHANNEL_IPC,
	LTT_CHANNEL_KERNEL,
	LTT_CHANNEL_MM,
	LTT_CHANNEL_RCU,
	LTT_CHANNEL_DEFAULT,
};

struct ltt_active_marker {
	struct list_head node;		/* active markers list */
	const char *channel;
	const char *name;
	const char *format;
	struct ltt_available_probe *probe;
};

extern void ltt_vtrace(const struct marker *mdata, void *probe_data,
		       void *call_data, const char *fmt, va_list *args);
extern void ltt_trace(const struct marker *mdata, void *probe_data,
		      void *call_data, const char *fmt, ...);

size_t ltt_serialize_printf(struct ltt_chanbuf *buf, unsigned long buf_offset,
			    size_t *msg_size, char *output, size_t outlen,
			    const char *fmt);

/*
 * Unique ID assigned to each registered probe.
 */
enum marker_id {
	MARKER_ID_SET_MARKER_ID = 0,	/* Static IDs available (range 0-7) */
	MARKER_ID_SET_MARKER_FORMAT,
	MARKER_ID_COMPACT,		/* Compact IDs (range: 8-127)	    */
	MARKER_ID_DYNAMIC,		/* Dynamic IDs (range: 128-65535)   */
};

/* static ids 0-1 reserved for internal use. */
#define MARKER_CORE_IDS		2
static __inline__ enum marker_id marker_id_type(uint16_t id)
{
	if (id < MARKER_CORE_IDS)
		return (enum marker_id)id;
	else
		return MARKER_ID_DYNAMIC;
}

struct user_dbg_data {
	unsigned long avail_size;
	unsigned long write;
	unsigned long read;
};

struct ltt_trace_ops {
	/* First 32 bytes cache-hot cacheline */
	void (*wakeup_channel) (struct ltt_chan *chan);
	int (*user_blocking) (struct ltt_trace *trace, unsigned int index,
			      size_t data_size, struct user_dbg_data *dbg);
	/* End of first 32 bytes cacheline */
	int (*create_dirs) (struct ltt_trace *new_trace);
	void (*remove_dirs) (struct ltt_trace *new_trace);
	int (*create_channel) (const char *channel_name, struct ltt_chan *chan,
			       struct dentry *parent, size_t sb_size,
			       size_t n_sb, int overwrite,
			       struct ltt_trace *trace);
	void (*finish_channel) (struct ltt_chan *chan);
	void (*remove_channel) (struct kref *kref);
	void (*remove_channel_files) (struct ltt_chan *chan);
	void (*user_errors) (struct ltt_trace *trace, unsigned int index,
			     size_t data_size, struct user_dbg_data *dbg,
			     int cpu);
	void (*start_switch_timer) (struct ltt_chan *chan);
	void (*stop_switch_timer) (struct ltt_chan *chan);
#ifdef CONFIG_HOTPLUG_CPU
	int (*handle_cpuhp) (struct notifier_block *nb, unsigned long action,
			     void *hcpu, struct ltt_trace *trace);
#endif
};

struct ltt_transport {
	char *name;
	struct module *owner;
	struct list_head node;
	struct ltt_trace_ops ops;
};

enum trace_mode { LTT_TRACE_NORMAL, LTT_TRACE_FLIGHT, LTT_TRACE_HYBRID };

#define CHANNEL_FLAG_ENABLE	(1U<<0)
#define CHANNEL_FLAG_OVERWRITE	(1U<<1)

/* Per-trace information - each trace/flight recorder represented by one */
struct ltt_trace {
	/* First 32 bytes cache-hot cacheline */
	struct list_head list;
	struct ltt_chan *channels;
	unsigned int nr_channels;
	int active;
	/* Second 32 bytes cache-hot cacheline */
	struct ltt_trace_ops *ops;
	u32 freq_scale;
	u64 start_freq;
	u64 start_tsc;
	unsigned long long start_monotonic;
	struct timeval		start_time;
	struct ltt_channel_setting *settings;
	struct {
		struct dentry			*trace_root;
		struct dentry			*ascii_root;
	} dentry;
	struct kref kref; /* Each channel has a kref of the trace struct */
	struct ltt_transport *transport;
	struct kref ltt_transport_kref;
	wait_queue_head_t kref_wq; /* Place for ltt_trace_destroy to sleep */
	char trace_name[NAME_MAX];
} ____cacheline_aligned;

/* Hardcoded event headers
 *
 * event header for a trace with active heartbeat : 27 bits timestamps
 *
 * headers are 32-bits aligned. In order to insure such alignment, a dynamic per
 * trace alignment value must be done.
 *
 * Remember that the C compiler does align each member on the boundary
 * equivalent to their own size.
 *
 * As relay subbuffers are aligned on pages, we are sure that they are 4 and 8
 * bytes aligned, so the buffer header and trace header are aligned.
 *
 * Event headers are aligned depending on the trace alignment option.
 *
 * Note using C structure bitfields for cross-endianness and portability
 * concerns.
 */

#define LTT_RESERVED_EVENTS	3
#define LTT_EVENT_BITS		5
#define LTT_FREE_EVENTS		((1 << LTT_EVENT_BITS) - LTT_RESERVED_EVENTS)
#define LTT_TSC_BITS		27
#define LTT_TSC_MASK		((1 << LTT_TSC_BITS) - 1)

struct ltt_event_header {
	u32 id_time;		/* 5 bits event id (MSB); 27 bits time (LSB) */
};

/* Reservation flags */
#define	LTT_RFLAG_ID			(1 << 0)
#define	LTT_RFLAG_ID_SIZE		(1 << 1)
#define	LTT_RFLAG_ID_SIZE_TSC		(1 << 2)

#define LTT_MAX_SMALL_SIZE		0xFFFFU

/*
 * We use asm/timex.h : cpu_khz/HZ variable in here : we might have to deal
 * specifically with CPU frequency scaling someday, so using an interpolation
 * between the start and end of buffer values is not flexible enough. Using an
 * immediate frequency value permits to calculate directly the times for parts
 * of a buffer that would be before a frequency change.
 *
 * Keep the natural field alignment for _each field_ within this structure if
 * you ever add/remove a field from this header. Packed attribute is not used
 * because gcc generates poor code on at least powerpc and mips. Don't ever
 * let gcc add padding between the structure elements.
 */
struct ltt_subbuffer_header {
	uint64_t cycle_count_begin;	/* Cycle count at subbuffer start */
	uint64_t cycle_count_end;	/* Cycle count at subbuffer end */
	uint32_t magic_number;		/*
					 * Trace magic number.
					 * contains endianness information.
					 */
	uint8_t major_version;
	uint8_t minor_version;
	uint8_t arch_size;		/* Architecture pointer size */
	uint8_t alignment;		/* LTT data alignment */
	uint64_t start_time_sec;	/* NTP-corrected start time */
	uint64_t start_time_usec;
	uint64_t start_freq;		/*
					 * Frequency at trace start,
					 * used all along the trace.
					 */
	uint32_t freq_scale;		/* Frequency scaling (divisor) */
	uint32_t data_size;		/* Size of data in subbuffer */
	uint32_t sb_size;		/* Subbuffer size (include padding) */
	uint32_t events_lost;		/*
					 * Events lost in this subbuffer since
					 * the beginning of the trace.
					 * (may overflow)
					 */
	uint32_t subbuf_corrupt;	/*
					 * Corrupted (lost) subbuffers since
					 * the begginig of the trace.
					 * (may overflow)
					 */
	uint8_t header_end[0];		/* End of header */
};

/**
 * ltt_sb_header_size - called on buffer-switch to a new sub-buffer
 *
 * Return header size without padding after the structure. Don't use packed
 * structure because gcc generates inefficient code on some architectures
 * (powerpc, mips..)
 */
static __inline__ size_t ltt_sb_header_size(void)
{
	return offsetof(struct ltt_subbuffer_header, header_end);
}

/*
 * ltt_get_header_size
 *
 * Calculate alignment offset to 32-bits. This is the alignment offset of the
 * event header.
 *
 * Important note :
 * The event header must be 32-bits. The total offset calculated here :
 *
 * Alignment of header struct on 32 bits (min arch size, header size)
 * + sizeof(header struct)  (32-bits)
 * + (opt) u16 (ext. event id)
 * + (opt) u16 (event_size)
 *             (if event_size == LTT_MAX_SMALL_SIZE, has ext. event size)
 * + (opt) u32 (ext. event size)
 * + (opt) u64 full TSC (aligned on min(64-bits, arch size))
 *
 * The payload must itself determine its own alignment from the biggest type it
 * contains.
 * */
static __inline__
unsigned char ltt_get_header_size(struct ltt_chan *chan, size_t offset,
				  size_t data_size, size_t *before_hdr_pad,
				  unsigned int rflags)
{
	size_t orig_offset = offset;
	size_t padding;

	BUILD_BUG_ON(sizeof(struct ltt_event_header) != sizeof(u32));

	padding = ltt_align(offset, sizeof(struct ltt_event_header));
	offset += padding;
	offset += sizeof(struct ltt_event_header);

	if (unlikely(rflags)) {
		switch (rflags) {
		case LTT_RFLAG_ID_SIZE_TSC:
			offset += sizeof(u16) + sizeof(u16);
			if (data_size >= LTT_MAX_SMALL_SIZE)
				offset += sizeof(u32);
			offset += ltt_align(offset, sizeof(u64));
			offset += sizeof(u64);
			break;
		case LTT_RFLAG_ID_SIZE:
			offset += sizeof(u16) + sizeof(u16);
			if (data_size >= LTT_MAX_SMALL_SIZE)
				offset += sizeof(u32);
			break;
		case LTT_RFLAG_ID:
			offset += sizeof(u16);
			break;
		}
	}

	*before_hdr_pad = padding;
	return offset - orig_offset;
}

extern
size_t ltt_write_event_header_slow(struct ltt_chanbuf_alloc *bufa,
				   struct ltt_chan_alloc *chana,
				   long buf_offset, u16 eID, u32 event_size,
				   u64 tsc, unsigned int rflags);

/*
 * ltt_write_event_header
 *
 * Writes the event header to the offset (already aligned on 32-bits).
 *
 * @buf : buffer to write to.
 * @chan : pointer to the channel structure..
 * @buf_offset : buffer offset to write to (aligned on 32 bits).
 * @eID : event ID
 * @event_size : size of the event, excluding the event header.
 * @tsc : time stamp counter.
 * @rflags : reservation flags.
 *
 * returns : offset where the event data must be written.
 */
static __inline__
size_t ltt_write_event_header(struct ltt_chanbuf_alloc *bufa,
			      struct ltt_chan_alloc *chana,
			      long buf_offset, u16 eID, u32 event_size, u64 tsc,
			      unsigned int rflags)
{
	struct ltt_event_header header;

	if (unlikely(rflags))
		goto slow_path;

	header.id_time = eID << LTT_TSC_BITS;
	header.id_time |= (u32)tsc & LTT_TSC_MASK;
	ltt_relay_write(bufa, chana, buf_offset, &header, sizeof(header));
	buf_offset += sizeof(header);

	return buf_offset;

slow_path:
	return ltt_write_event_header_slow(bufa, chana, buf_offset,
					   eID, event_size, tsc, rflags);
}

/*
 * ltt_read_event_header
 * buf_offset must aligned on 32 bits
 */
static __inline__
size_t ltt_read_event_header(struct ltt_chanbuf_alloc *bufa, long buf_offset,
			     u64 *tsc, u32 *event_size, u16 *eID,
			     unsigned int *rflags)
{
	struct ltt_event_header header;
	u16 small_size;

	ltt_relay_read(bufa, buf_offset, &header, sizeof(header));
	buf_offset += sizeof(header);

	*event_size = INT_MAX;
	*eID = header.id_time >> LTT_TSC_BITS;
	*tsc = header.id_time & LTT_TSC_MASK;

	switch (*eID) {
	case 29:
		*rflags = LTT_RFLAG_ID_SIZE_TSC;
		ltt_relay_read(bufa, buf_offset, eID, sizeof(u16));
		buf_offset += sizeof(u16);
		ltt_relay_read(bufa, buf_offset, &small_size, sizeof(u16));
		buf_offset += sizeof(u16);
		if (small_size == LTT_MAX_SMALL_SIZE) {
			ltt_relay_read(bufa, buf_offset, event_size,
					sizeof(u32));
			buf_offset += sizeof(u32);
		} else
			*event_size = small_size;
		buf_offset += ltt_align(buf_offset, sizeof(u64));
		ltt_relay_read(bufa, buf_offset, tsc, sizeof(u64));
		buf_offset += sizeof(u64);
		break;
	case 30:
		*rflags = LTT_RFLAG_ID_SIZE;
		ltt_relay_read(bufa, buf_offset, eID, sizeof(u16));
		buf_offset += sizeof(u16);
		ltt_relay_read(bufa, buf_offset, &small_size, sizeof(u16));
		buf_offset += sizeof(u16);
		if (small_size == LTT_MAX_SMALL_SIZE) {
			ltt_relay_read(bufa, buf_offset, event_size,
					sizeof(u32));
			buf_offset += sizeof(u32);
		} else
			*event_size = small_size;
		break;
	case 31:
		*rflags = LTT_RFLAG_ID;
		ltt_relay_read(bufa, buf_offset, eID, sizeof(u16));
		buf_offset += sizeof(u16);
		break;
	default:
		*rflags = 0;
		break;
	}

	return buf_offset;
}

/* Lockless LTTng */

/* Buffer offset macros */

/*
 * BUFFER_TRUNC zeroes the subbuffer offset and the subbuffer number parts of
 * the offset, which leaves only the buffer number.
 */
#define BUFFER_TRUNC(offset, chan) \
	((offset) & (~((chan)->a.buf_size - 1)))
#define BUFFER_OFFSET(offset, chan) ((offset) & ((chan)->a.buf_size - 1))
#define SUBBUF_OFFSET(offset, chan) ((offset) & ((chan)->a.sb_size - 1))
#define SUBBUF_ALIGN(offset, chan) \
	(((offset) + (chan)->a.sb_size) & (~((chan)->a.sb_size - 1)))
#define SUBBUF_TRUNC(offset, chan) \
	((offset) & (~((chan)->a.sb_size - 1)))
#define SUBBUF_INDEX(offset, chan) \
	(BUFFER_OFFSET((offset), chan) >> (chan)->a.sb_size_order)

/*
 * Control channels :
 * control/metadata
 * control/interrupts
 * control/...
 *
 * cpu channel :
 * cpu
 */
#define LTT_RELAY_ROOT			"ltt"
#define LTT_RELAY_LOCKED_ROOT		"ltt-locked"

#define LTT_METADATA_CHANNEL		"metadata_state"
#define LTT_FD_STATE_CHANNEL		"fd_state"
#define LTT_GLOBAL_STATE_CHANNEL	"global_state"
#define LTT_IRQ_STATE_CHANNEL		"irq_state"
#define LTT_MODULE_STATE_CHANNEL	"module_state"
#define LTT_NETIF_STATE_CHANNEL		"netif_state"
#define LTT_SOFTIRQ_STATE_CHANNEL	"softirq_state"
#define LTT_SWAP_STATE_CHANNEL		"swap_state"
#define LTT_SYSCALL_STATE_CHANNEL	"syscall_state"
#define LTT_TASK_STATE_CHANNEL		"task_state"
#define LTT_VM_STATE_CHANNEL		"vm_state"
#define LTT_FS_CHANNEL			"fs"
#define LTT_INPUT_CHANNEL		"input"
#define LTT_IPC_CHANNEL			"ipc"
#define LTT_KERNEL_CHANNEL		"kernel"
#define LTT_MM_CHANNEL			"mm"
#define LTT_RCU_CHANNEL			"rcu"

#define LTT_FLIGHT_PREFIX		"flight-"

#define LTT_ASCII			"ascii"

/* Tracer properties */
#define LTT_DEFAULT_SUBBUF_SIZE_LOW	65536
#define LTT_DEFAULT_N_SUBBUFS_LOW	2
#define LTT_DEFAULT_SUBBUF_SIZE_MED	262144
#define LTT_DEFAULT_N_SUBBUFS_MED	2
#define LTT_DEFAULT_SUBBUF_SIZE_HIGH	1048576
#define LTT_DEFAULT_N_SUBBUFS_HIGH	2
#define LTT_TRACER_MAGIC_NUMBER		0x00D6B7ED
#define LTT_TRACER_VERSION_MAJOR	2
#define LTT_TRACER_VERSION_MINOR	6

/**
 * ltt_write_trace_header - Write trace header
 * @trace: Trace information
 * @header: Memory address where the information must be written to
 */
static __inline__
void ltt_write_trace_header(struct ltt_trace *trace,
			    struct ltt_subbuffer_header *header)
{
	header->magic_number = LTT_TRACER_MAGIC_NUMBER;
	header->major_version = LTT_TRACER_VERSION_MAJOR;
	header->minor_version = LTT_TRACER_VERSION_MINOR;
	header->arch_size = sizeof(void *);
	header->alignment = ltt_get_alignment();
	header->start_time_sec = trace->start_time.tv_sec;
	header->start_time_usec = trace->start_time.tv_usec;
	header->start_freq = trace->start_freq;
	header->freq_scale = trace->freq_scale;
}

/*
 * Size reserved for high priority events (interrupts, NMI, BH) at the end of a
 * nearly full buffer. User space won't use this last amount of space when in
 * blocking mode. This space also includes the event header that would be
 * written by this user space event.
 */
#define LTT_RESERVE_CRITICAL		4096

/* Register and unregister function pointers */

enum ltt_module_function {
	LTT_FUNCTION_RUN_FILTER,
	LTT_FUNCTION_FILTER_CONTROL,
	LTT_FUNCTION_STATEDUMP
};

extern int ltt_module_register(enum ltt_module_function name, void *function,
			       struct module *owner);
extern void ltt_module_unregister(enum ltt_module_function name);

void ltt_transport_register(struct ltt_transport *transport);
void ltt_transport_unregister(struct ltt_transport *transport);

/* Exported control function */

enum ltt_control_msg {
	LTT_CONTROL_START,
	LTT_CONTROL_STOP,
	LTT_CONTROL_CREATE_TRACE,
	LTT_CONTROL_DESTROY_TRACE
};

union ltt_control_args {
	struct {
		enum trace_mode mode;
		unsigned int subbuf_size_low;
		unsigned int n_subbufs_low;
		unsigned int subbuf_size_med;
		unsigned int n_subbufs_med;
		unsigned int subbuf_size_high;
		unsigned int n_subbufs_high;
	} new_trace;
};

int _ltt_trace_setup(const char *trace_name);
int ltt_trace_setup(const char *trace_name);
struct ltt_trace *_ltt_trace_find_setup(const char *trace_name);
int ltt_trace_set_type(const char *trace_name, const char *trace_type);
int ltt_trace_set_channel_subbufsize(const char *trace_name,
				     const char *channel_name,
				     unsigned int size);
int ltt_trace_set_channel_subbufcount(const char *trace_name,
				      const char *channel_name,
				      unsigned int cnt);
int ltt_trace_set_channel_switch_timer(const char *trace_name,
				       const char *channel_name,
				       unsigned long interval);
int ltt_trace_set_channel_enable(const char *trace_name,
				 const char *channel_name,
				 unsigned int enable);
int ltt_trace_set_channel_overwrite(const char *trace_name,
				    const char *channel_name,
				    unsigned int overwrite);
int ltt_trace_alloc(const char *trace_name);
int ltt_trace_destroy(const char *trace_name);
int ltt_trace_start(const char *trace_name);
int ltt_trace_stop(const char *trace_name);

extern int ltt_control(enum ltt_control_msg msg, const char *trace_name,
		       const char *trace_type, union ltt_control_args args);

enum ltt_filter_control_msg {
	LTT_FILTER_DEFAULT_ACCEPT,
	LTT_FILTER_DEFAULT_REJECT
};

extern int ltt_filter_control(enum ltt_filter_control_msg msg,
			      const char *trace_name);

extern struct dentry *get_filter_root(void);

void ltt_core_register(int (*function)(u8, void *));

void ltt_core_unregister(void);

void ltt_release_trace(struct kref *kref);
void ltt_release_transport(struct kref *kref);

extern int ltt_probe_register(struct ltt_available_probe *pdata);
extern int ltt_probe_unregister(struct ltt_available_probe *pdata);
extern int ltt_marker_connect(const char *channel, const char *mname,
			      const char *pname);
extern int ltt_marker_disconnect(const char *channel, const char *mname,
				 const char *pname);
extern void ltt_dump_marker_state(struct ltt_trace *trace);

void ltt_lock_traces(void);
void ltt_unlock_traces(void);

extern int ltt_ascii_create_dir(struct ltt_trace *new_trace);
extern void ltt_ascii_remove_dir(struct ltt_trace *trace);
extern int ltt_ascii_create(struct ltt_chan *chan);
extern void ltt_ascii_remove(struct ltt_chan *chan);

extern
void ltt_statedump_register_kprobes_dump(void (*callback)(void *call_data));
extern
void ltt_statedump_unregister_kprobes_dump(void (*callback)(void *call_data));

extern void ltt_dump_softirq_vec(void *call_data);

#ifdef CONFIG_HAVE_LTT_DUMP_TABLES
extern void ltt_dump_sys_call_table(void *call_data);
extern void ltt_dump_idt_table(void *call_data);
#else
static inline void ltt_dump_sys_call_table(void *call_data)
{
}

static inline void ltt_dump_idt_table(void *call_data)
{
}
#endif

/* Relay IOCTL */

/* Get the next sub-buffer that can be read. */
#define RELAY_GET_SB			_IOR(0xF5, 0x00, __u32)
/* Release the oldest reserved (by "get") sub-buffer. */
#define RELAY_PUT_SB			_IOW(0xF5, 0x01, __u32)
/* returns the number of sub-buffers in the per cpu channel. */
#define RELAY_GET_N_SB			_IOR(0xF5, 0x02, __u32)
/* returns the size of the current sub-buffer. */
#define RELAY_GET_SB_SIZE		_IOR(0xF5, 0x03, __u32)
/* returns the maximum size for sub-buffers. */
#define RELAY_GET_MAX_SB_SIZE		_IOR(0xF5, 0x04, __u32)

#endif /* _LTT_TRACER_H */
