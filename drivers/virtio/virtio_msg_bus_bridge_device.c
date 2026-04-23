// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message Linux <-> userspace bridge core (Step 5 slice).
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#define pr_fmt(fmt) "virtio-msg-bridge: " fmt

#include <linux/atomic.h>
#include <linux/bitmap.h>
#include <linux/eventfd.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/poll.h>
#include <linux/refcount.h>
#include <linux/shmem_fs.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/virtio_msg_bus_bridge_device.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/xarray.h>
#include <uapi/linux/virtio_msg_bus_bridge.h>

#define VM_LOG_COMPONENT VM_LOG_COMPONENT_BRIDGE
#include "virtio_msg_debug.h"

/*
 * Step 1 shrinks the exported bridge headers ahead of the bridge-core cleanup
 * that will delete these paths. Keep the removed definitions private here
 * until the follow-on implementation steps land.
 */
struct vmsg_bus_handle_entry;
struct vmsg_bus_file_ctx;

struct vmsg_bus_ring_state {
	struct vmsg_bridge_uapi_ring_info info;
	u64 end;
	struct folio **folios;
	struct page **pages;
	void *vaddr;
	unsigned int nr_folios;
	unsigned int nr_pages;
	size_t first_page_offset;
};

struct vmsg_bus_control_entry {
	struct list_head node;
	struct vmsg_bridge_uapi_control_msg msg;
};

enum vmsg_bus_map_record_state {
	VMSG_BUS_MAP_RECORD_MMAP_READY = 1,
	VMSG_BUS_MAP_RECORD_ACTIVE = 2,
	VMSG_BUS_MAP_RECORD_DEL_REQ_PENDING = 3,
};

struct vmsg_bus_map_record {
	u64 map_id;
	u64 epoch;
	u64 bus_addr;
	u64 length;
	u64 mmap_offset;
	u64 mmap_length;
	u32 flags;
	enum vmsg_bus_map_record_state state;
	bool del_req_queued;
};

struct vmsg_bus_map_event_entry {
	struct list_head node;
	struct vmsg_bridge_uapi_map_event event;
	bool delivered;
	unsigned long deliver_after_jiffies;
};

struct vmsg_bus_rx_publish_entry {
	struct list_head node;
	u16 msg_size;
	u8 data[];
};

enum vmsg_bus_session_action {
	VMSG_BUS_SESSION_ACT_TX_DRAIN = BIT(0),
	VMSG_BUS_SESSION_ACT_RX_PUBLISH = BIT(1),
	VMSG_BUS_SESSION_ACT_WAKE = BIT(2),
};

struct vmsg_bus_session {
	refcount_t refs;
	u32 endpoint_id;
	struct vmsg_bus_file_ctx *ctx;
	struct virtio_msg_bus_bridge_device *endpoint;
	struct file *ring_file;
	struct eventfd_ctx *kick_evt;
	struct eventfd_ctx *call_evt;
	struct vmsg_bus_ring_state tx;
	struct vmsg_bus_ring_state rx;
	struct virtio_msg_bus_bridge_device_caps caps;
	u32 attach_vmm_max_msg_size;
	u32 endpoint_max_msg_size;
	struct list_head map_event_queue;
	/* Protects map queue, map records, and map sequence/id allocation. */
	struct mutex map_lock;
	u32 map_event_queue_len;
	u64 map_event_next_seq;
	u64 map_event_next_id;
	u64 map_event_epoch;
	/* Protects relay metadata keyed by (dev_num, token) -> request state. */
	struct mutex relay_lock;
	struct xarray relay_meta;
	bool detached;
	struct xarray map_records_by_id;
	struct xarray map_records_by_offset;
	struct list_head control_queue;
	/* Protects control_queue and control_next_seq updates. */
	struct mutex control_lock;
	u64 control_next_seq;
	/* Protects executor action bits, kick hook/retry state, and RX publish queue. */
	spinlock_t exec_lock;
	unsigned long exec_actions;
	struct work_struct exec_work;
	struct delayed_work tx_retry_work;
	wait_queue_entry_t kick_wait;
	poll_table kick_pt;
	bool kick_hooked;
	bool tx_retry_armed;
	struct list_head rx_publish_queue;
	u32 rx_publish_queue_len;
	wait_queue_head_t poll_waitq;
	bool ring_fault;
};

struct vmsg_bus_relay_meta {
	u16 dev_num;
	u16 token;
	u8 msg_id;
	u64 relay_seq;
};

struct vmsg_bus_file_ctx {
	refcount_t refs;
	/* Serializes bridge-fd ioctls and endpoint-driven invalidation. */
	struct mutex lock;
	struct vmsg_bus_session *session;
};

struct vmsg_bus_handle_entry {
	struct vmsg_bridge_uapi_endpoint_addr addr;
	struct virtio_msg_bus_bridge_device *endpoint;
	struct vmsg_bus_session *session;
	u32 resolve_refs;
};

struct vmsg_bus_endpoint_topology {
	struct xarray devices;
	DECLARE_BITMAP(devices_bitmap, U16_MAX + 1U);
};

struct vmsg_bus_resolver_entry {
	struct list_head node;
	struct vmsg_bus_resolver *resolver;
};

static atomic_t vmsg_bus_next_handle = ATOMIC_INIT(1);
static DEFINE_XARRAY(vmsg_bus_handles);
static DEFINE_MUTEX(vmsg_bus_handles_lock);

static DEFINE_XARRAY(vmsg_bus_endpoint_topologies);
static DEFINE_MUTEX(vmsg_bus_endpoint_topologies_lock);

static LIST_HEAD(vmsg_bus_resolvers);
static DEFINE_MUTEX(vmsg_bus_resolvers_lock);

static DEFINE_XARRAY(vmsg_bus_cleanup_busy_endpoints);
static DEFINE_MUTEX(vmsg_bus_cleanup_busy_lock);

#define VMSG_BUS_CLEANUP_STATE_IDLE	xa_mk_value(1)
#define VMSG_BUS_CLEANUP_STATE_BUSY	xa_mk_value(2)
#define VMSG_BUS_MAP_EVENT_QUEUE_MAX	1024U
#define VMSG_BUS_MAP_EVENT_RETRY_DELAY_MS	10U
#define VMSG_BUS_SESSION_TX_RETRY_DELAY_MS	10U
#define VMSG_BUS_RX_PUBLISH_QUEUE_MAX	1024U
#define VMSG_BUS_TOPOLOGY_REMOVED_BATCH	64U

static int vmsg_bus_cleanup_busy_entry_init(u32 handle);
static void vmsg_bus_cleanup_busy_entry_destroy(u32 handle);
static void vmsg_bus_map_event_purge_locked(struct vmsg_bus_session *session);
static u32 vmsg_bus_map_records_purge_locked(struct vmsg_bus_session *session);
static void vmsg_bus_session_caps_clear(struct vmsg_bus_session *session);
static void vmsg_bus_session_exec_queue(struct vmsg_bus_session *session,
					unsigned long actions);
static void vmsg_bus_session_exec_sync_disable(struct vmsg_bus_session *session);
static void vmsg_bus_session_tx_retry_workfn(struct work_struct *work);
static void vmsg_bus_session_tx_retry_arm(struct vmsg_bus_session *session);
static void vmsg_bus_session_tx_disconnect(struct vmsg_bus_session *session);
static int vmsg_bus_session_caps_refresh(struct vmsg_bus_session *session);
static struct vmsg_bus_session *
vmsg_bus_session_get(struct vmsg_bus_session *session);
static void vmsg_bus_session_put(struct vmsg_bus_session *session);
static unsigned long vmsg_bus_relay_key(u16 dev_num, u16 token);
static int vmsg_bus_relay_insert(struct vmsg_bus_session *session, u16 dev_num,
				 u16 token, u8 msg_id, u64 relay_seq);
static int vmsg_bus_relay_take(struct vmsg_bus_session *session, u16 dev_num,
			       u16 token, u8 *msg_id, u64 *relay_seq);
static void vmsg_bus_relay_purge_session(struct vmsg_bus_session *session);
static int
vmsg_bus_validate_transport_msg_type(const struct virtio_msg *msg);
static int
vmsg_bus_dispatch_validate_transport_target
		(struct virtio_msg_bus_bridge_device *endpoint,
		 const struct virtio_msg *msg);
static struct vmsg_bus_endpoint_topology *
vmsg_bus_endpoint_topology_lookup_locked(struct virtio_msg_bus_bridge_device *endpoint);
static void vmsg_bus_publish_endpoint_online_control(struct vmsg_bus_session *session);
static void vmsg_bus_publish_endpoint_offline_control(struct vmsg_bus_session *session,
						      u32 endpoint_offline_reason);
static void
vmsg_bus_session_rebind_online(struct vmsg_bus_session *session,
			       struct virtio_msg_bus_bridge_device *endpoint);
static void vmsg_bus_session_mark_offline(struct vmsg_bus_session *session,
					  u32 endpoint_offline_reason);
static bool vmsg_bus_session_invalidate(struct vmsg_bus_session *session);
static int
vmsg_bus_bridge_endpoint_resolve(struct virtio_msg_bus_bridge_device *endpoint,
				 u32 *out_handle,
				 struct vmsg_bus_session **out_session);
static void
vmsg_bus_bridge_endpoint_quiesce(struct virtio_msg_bus_bridge_device *endpoint);
static void
vmsg_bus_bridge_endpoint_mark_session_offline(struct vmsg_bus_session *session,
					      u32 endpoint_offline_reason);
static void
vmsg_bus_topology_clear_notify_removed
		(struct virtio_msg_bus_bridge_device *endpoint);
static int vmsg_bus_ring_entry_size_calc(u32 vmm_max_msg_size,
					 u32 *entry_size);

static struct vmsg_bus_file_ctx *
vmsg_bus_file_ctx_get(struct vmsg_bus_file_ctx *ctx)
{
	if (!ctx)
		return NULL;

	if (!refcount_inc_not_zero(&ctx->refs))
		return NULL;

	return ctx;
}

static void vmsg_bus_file_ctx_put(struct vmsg_bus_file_ctx *ctx)
{
	if (!ctx)
		return;

	if (refcount_dec_and_test(&ctx->refs))
		kfree(ctx);
}

static unsigned long vmsg_bus_session_exec_fetch(struct vmsg_bus_session *session)
{
	unsigned long actions;
	unsigned long irqflags;

	spin_lock_irqsave(&session->exec_lock, irqflags);
	actions = session->exec_actions;
	session->exec_actions = 0;
	spin_unlock_irqrestore(&session->exec_lock, irqflags);

	return actions;
}

static void vmsg_bus_session_exec_queue(struct vmsg_bus_session *session,
					unsigned long actions)
{
	unsigned long irqflags;
	bool detached;

	if (!session || !actions)
		return;

	spin_lock_irqsave(&session->exec_lock, irqflags);
	detached = READ_ONCE(session->detached);
	if (!detached)
		session->exec_actions |= actions;
	spin_unlock_irqrestore(&session->exec_lock, irqflags);

	if (!detached)
		schedule_work(&session->exec_work);
}

static void vmsg_bus_rx_publish_queue_purge(struct vmsg_bus_session *session)
{
	struct vmsg_bus_rx_publish_entry *entry;
	struct vmsg_bus_rx_publish_entry *tmp;
	LIST_HEAD(purge);
	unsigned long irqflags;

	if (!session)
		return;

	spin_lock_irqsave(&session->exec_lock, irqflags);
	list_splice_init(&session->rx_publish_queue, &purge);
	session->rx_publish_queue_len = 0;
	spin_unlock_irqrestore(&session->exec_lock, irqflags);

	list_for_each_entry_safe(entry, tmp, &purge, node)
		kfree(entry);
}

static bool vmsg_bus_rx_publish_queue_pending(struct vmsg_bus_session *session)
{
	unsigned long irqflags;
	bool pending;

	spin_lock_irqsave(&session->exec_lock, irqflags);
	pending = !list_empty(&session->rx_publish_queue);
	spin_unlock_irqrestore(&session->exec_lock, irqflags);

	return pending;
}

static void vmsg_bus_ring_state_release(struct vmsg_bus_ring_state *state)
{
	if (!state)
		return;

	if (state->vaddr) {
		vm_unmap_ram(state->vaddr, state->nr_pages);
		state->vaddr = NULL;
	}
	kfree(state->pages);
	state->pages = NULL;
	if (state->folios)
		unpin_folios(state->folios, state->nr_folios);
	kfree(state->folios);
	state->folios = NULL;
	state->nr_folios = 0;
	state->nr_pages = 0;
	state->first_page_offset = 0;
}

static int vmsg_bus_ring_state_map(struct file *ring_file,
				   struct vmsg_bus_ring_state *state)
{
	struct folio **folios = NULL;
	struct page **pages = NULL;
	pgoff_t first_folio_offset;
	u64 covered_bytes;
	unsigned int nr_pages;
	unsigned int pages_filled = 0;
	unsigned int i;
	long nr_folios;
	int ret = 0;

	if (!ring_file || !state)
		return -EINVAL;
	if (!state->info.bytes)
		return -EINVAL;

	covered_bytes = (u64)offset_in_page(state->info.offset) + state->info.bytes;
	if (covered_bytes > (u64)UINT_MAX * PAGE_SIZE)
		return -EOVERFLOW;

	nr_pages = DIV_ROUND_UP_ULL(covered_bytes, PAGE_SIZE);
	folios = kcalloc(nr_pages, sizeof(*folios), GFP_KERNEL);
	pages = kcalloc(nr_pages, sizeof(*pages), GFP_KERNEL);
	if (!folios || !pages) {
		ret = -ENOMEM;
		goto out_free;
	}

	nr_folios = memfd_pin_folios(ring_file, state->info.offset, state->end - 1,
				     folios, nr_pages, &first_folio_offset);
	if (nr_folios <= 0) {
		ret = nr_folios ? (int)nr_folios : -EINVAL;
		goto out_free;
	}

	for (i = 0; i < nr_folios; i++) {
		pgoff_t folio_offset = i == 0 ? ALIGN_DOWN(first_folio_offset,
							  PAGE_SIZE) : 0;
		size_t folio_bytes = folio_size(folios[i]);

		for (; folio_offset < folio_bytes && pages_filled < nr_pages;
		     folio_offset += PAGE_SIZE) {
			pages[pages_filled++] = folio_page(folios[i],
							   folio_offset >>
							   PAGE_SHIFT);
		}
	}

	if (pages_filled != nr_pages) {
		ret = -EINVAL;
		goto out_unpin;
	}

	state->vaddr = vm_map_ram(pages, nr_pages, NUMA_NO_NODE);
	if (!state->vaddr) {
		ret = -ENOMEM;
		goto out_unpin;
	}

	state->folios = folios;
	state->pages = pages;
	state->nr_folios = nr_folios;
	state->nr_pages = nr_pages;
	state->first_page_offset = offset_in_page(state->info.offset);

	return 0;

out_unpin:
	unpin_folios(folios, nr_folios);
out_free:
	kfree(pages);
	kfree(folios);
	return ret;
}

static int vmsg_bus_ring_ptr(const struct vmsg_bus_ring_state *state, u64 offset,
			     size_t len, void **out_ptr)
{
	u64 rel_offset;
	u64 rel_end;
	u64 map_offset;
	u64 map_end;

	if (!state || !state->vaddr || !out_ptr)
		return -EINVAL;
	if (offset < state->info.offset)
		return -EINVAL;

	rel_offset = offset - state->info.offset;
	if (check_add_overflow(rel_offset, (u64)len, &rel_end) ||
	    rel_end > state->info.bytes)
		return -EOVERFLOW;
	if (check_add_overflow((u64)state->first_page_offset, rel_offset,
			       &map_offset) ||
	    check_add_overflow(map_offset, (u64)len, &map_end))
		return -EOVERFLOW;
	if (map_end > (u64)state->nr_pages * PAGE_SIZE || map_offset > SIZE_MAX)
		return -EOVERFLOW;

	*out_ptr = (u8 *)state->vaddr + (size_t)map_offset;
	return 0;
}

static int vmsg_bus_ring_hdr_read(const struct vmsg_bus_ring_state *state,
				  struct vmsg_bridge_uapi_ring_hdr *hdr)
{
	void *src;
	int ret;

	if (!state || !hdr)
		return -EINVAL;

	ret = vmsg_bus_ring_ptr(state, state->info.offset, sizeof(*hdr), &src);
	if (ret)
		return ret;

	memcpy(hdr, src, sizeof(*hdr));

	if (hdr->entries != state->info.entries ||
	    hdr->entry_size != state->info.entry_size ||
	    hdr->flags || hdr->reserved)
		return -EPROTO;

	return 0;
}

static int vmsg_bus_ring_slot_offset(const struct vmsg_bus_ring_state *state, u32 cursor,
				     u64 *out_offset)
{
	u32 idx;
	u64 rel = sizeof(struct vmsg_bridge_uapi_ring_hdr);
	u64 slot_bytes;
	u64 abs;
	u64 slot_end;

	if (!state || !out_offset || !state->info.entries)
		return -EINVAL;

	idx = cursor % state->info.entries;
	if (check_mul_overflow((u64)idx, (u64)state->info.entry_size, &slot_bytes))
		return -EOVERFLOW;
	if (check_add_overflow(rel, slot_bytes, &rel))
		return -EOVERFLOW;
	if (check_add_overflow(state->info.offset, rel, &abs))
		return -EOVERFLOW;
	if (check_add_overflow(abs, (u64)state->info.entry_size, &slot_end))
		return -EOVERFLOW;
	if (slot_end > state->end)
		return -EOVERFLOW;

	*out_offset = abs;
	return 0;
}

static int vmsg_bus_ring_slot_read(const struct vmsg_bus_ring_state *state,
				   u32 cursor, void *buf)
{
	u64 offset;
	void *src;
	int ret;

	ret = vmsg_bus_ring_slot_offset(state, cursor, &offset);
	if (ret)
		return ret;

	ret = vmsg_bus_ring_ptr(state, offset, state->info.entry_size, &src);
	if (ret)
		return ret;

	memcpy(buf, src, state->info.entry_size);
	return 0;
}

static int vmsg_bus_ring_slot_write(const struct vmsg_bus_ring_state *state, u32 cursor,
				    const void *buf, size_t len)
{
	u64 offset;
	void *dst;
	int ret;

	if (!state || len > state->info.entry_size)
		return -EMSGSIZE;

	ret = vmsg_bus_ring_slot_offset(state, cursor, &offset);
	if (ret)
		return ret;

	ret = vmsg_bus_ring_ptr(state, offset, len, &dst);
	if (ret)
		return ret;

	memcpy(dst, buf, len);
	return 0;
}

static int vmsg_bus_ring_occupancy(const struct vmsg_bridge_uapi_ring_hdr *hdr, u32 *out_used)
{
	u32 used;

	if (!hdr || !out_used)
		return -EINVAL;

	used = hdr->prod - hdr->cons;
	if (used > hdr->entries)
		return -EPROTO;

	*out_used = used;
	return 0;
}

static int vmsg_bus_ring_publish_hdr(const struct vmsg_bus_ring_state *state,
				     const struct vmsg_bridge_uapi_ring_hdr *hdr)
{
	struct vmsg_bridge_uapi_ring_hdr out = *hdr;
	void *dst;
	int ret;

	out.entries = state->info.entries;
	out.entry_size = state->info.entry_size;
	out.flags = 0;
	out.reserved = 0;
	/* Wrappers define direction; publish prior ring updates before header. */
	smp_wmb();
	ret = vmsg_bus_ring_ptr(state, state->info.offset, sizeof(out), &dst);
	if (ret)
		return ret;

	memcpy(dst, &out, sizeof(out));
	return 0;
}

static int vmsg_bus_ring_publish_cons(const struct vmsg_bus_ring_state *state,
				      const struct vmsg_bridge_uapi_ring_hdr *hdr)
{
	/* Publish consumed slot handling before userspace observes new cons. */
	return vmsg_bus_ring_publish_hdr(state, hdr);
}

static int vmsg_bus_ring_publish_prod(const struct vmsg_bus_ring_state *state,
				      const struct vmsg_bridge_uapi_ring_hdr *hdr)
{
	/* Publish slot payload writes before userspace observes new prod. */
	return vmsg_bus_ring_publish_hdr(state, hdr);
}

static void vmsg_bus_session_mark_ring_fault(struct vmsg_bus_session *session)
{
	if (!session)
		return;

	if (!READ_ONCE(session->ring_fault)) {
		WRITE_ONCE(session->ring_fault, true);
		vmsg_bus_session_exec_queue(session, VMSG_BUS_SESSION_ACT_WAKE);
	}
}

static void vmsg_bus_session_kick_ptable_queue_proc(struct file *file,
						    wait_queue_head_t *wqh,
						    poll_table *pt)
{
	struct vmsg_bus_session *session;

	(void)file;
	session = container_of(pt, struct vmsg_bus_session, kick_pt);
	add_wait_queue(wqh, &session->kick_wait);
}

static int vmsg_bus_session_kick_wakeup(wait_queue_entry_t *wait, unsigned int mode,
					int sync, void *key)
{
	struct vmsg_bus_session *session;
	__poll_t events = key_to_poll(key);
	u64 cnt = 0;

	(void)mode;
	(void)sync;

	session = container_of(wait, struct vmsg_bus_session, kick_wait);
	if (events & EPOLLIN) {
		eventfd_ctx_do_read(session->kick_evt, &cnt);
		if (cnt)
			vmsg_bus_session_exec_queue
				(session, VMSG_BUS_SESSION_ACT_TX_DRAIN);
	}
	if (events & (EPOLLERR | EPOLLHUP))
		vmsg_bus_session_exec_queue(session, VMSG_BUS_SESSION_ACT_WAKE);

	return 0;
}

static int vmsg_bus_session_kick_hook_install(struct vmsg_bus_session *session, int kick_fd)
{
	struct file *kick_file;
	__poll_t events;
	unsigned long irqflags;

	if (!session || !session->kick_evt)
		return -EINVAL;

	kick_file = eventfd_fget(kick_fd);
	if (IS_ERR(kick_file))
		return PTR_ERR(kick_file);

	init_waitqueue_func_entry(&session->kick_wait, vmsg_bus_session_kick_wakeup);
	init_poll_funcptr(&session->kick_pt, vmsg_bus_session_kick_ptable_queue_proc);
	events = vfs_poll(kick_file, &session->kick_pt);
	fput(kick_file);

	spin_lock_irqsave(&session->exec_lock, irqflags);
	session->kick_hooked = true;
	spin_unlock_irqrestore(&session->exec_lock, irqflags);

	if (events & EPOLLIN)
		vmsg_bus_session_exec_queue(session, VMSG_BUS_SESSION_ACT_TX_DRAIN);
	if (events & (EPOLLERR | EPOLLHUP))
		vmsg_bus_session_exec_queue(session, VMSG_BUS_SESSION_ACT_WAKE);

	return 0;
}

static int
vmsg_bus_session_rx_publish_enqueue_gfp(struct vmsg_bus_session *session,
					const struct virtio_msg *msg,
					u16 msg_size, gfp_t gfp,
					int alloc_fail_ret)
{
	struct vmsg_bus_rx_publish_entry *entry;
	unsigned long irqflags;
	bool detached;
	int ret = 0;

	if (!session || !msg || msg_size < sizeof(*msg))
		return -EINVAL;
	if (msg_size > session->rx.info.entry_size || msg_size > VIRTIO_MSG_MAX_SIZE)
		return -EMSGSIZE;

	entry = kmalloc(struct_size(entry, data, msg_size), gfp);
	if (!entry)
		return alloc_fail_ret;

	entry->msg_size = msg_size;
	memcpy(entry->data, msg, msg_size);

	spin_lock_irqsave(&session->exec_lock, irqflags);
	detached = READ_ONCE(session->detached);
	if (detached) {
		ret = -ENOTCONN;
	} else if (session->rx_publish_queue_len >= VMSG_BUS_RX_PUBLISH_QUEUE_MAX) {
		ret = -ENOSPC;
	} else {
		list_add_tail(&entry->node, &session->rx_publish_queue);
		session->rx_publish_queue_len++;
	}
	spin_unlock_irqrestore(&session->exec_lock, irqflags);

	if (ret) {
		kfree(entry);
		return ret;
	}

	vmsg_bus_session_exec_queue(session, VMSG_BUS_SESSION_ACT_RX_PUBLISH |
				    VMSG_BUS_SESSION_ACT_WAKE);
	return 0;
}

static int vmsg_bus_session_rx_publish_enqueue(struct vmsg_bus_session *session,
					       const struct virtio_msg *msg,
					       u16 msg_size)
{
	return vmsg_bus_session_rx_publish_enqueue_gfp(session, msg, msg_size,
						       GFP_KERNEL, -ENOMEM);
}

static int
vmsg_bus_session_rx_publish_enqueue_nonblock(struct vmsg_bus_session *session,
					     const struct virtio_msg *msg,
					     u16 msg_size)
{
	return vmsg_bus_session_rx_publish_enqueue_gfp(session, msg, msg_size,
						       GFP_ATOMIC, -ENOSPC);
}

static void vmsg_bus_session_tx_retry_workfn(struct work_struct *work)
{
	struct vmsg_bus_session *session;
	bool queue_retry;
	unsigned long irqflags;

	session = container_of(to_delayed_work(work), struct vmsg_bus_session,
			       tx_retry_work);

	spin_lock_irqsave(&session->exec_lock, irqflags);
	session->tx_retry_armed = false;
	queue_retry = session->kick_hooked &&
		      !READ_ONCE(session->detached) &&
		      !READ_ONCE(session->ring_fault) &&
		      READ_ONCE(session->endpoint);
	spin_unlock_irqrestore(&session->exec_lock, irqflags);

	if (queue_retry)
		vmsg_bus_session_exec_queue(session, VMSG_BUS_SESSION_ACT_TX_DRAIN);
}

static unsigned long vmsg_bus_relay_key(u16 dev_num, u16 token)
{
	return ((unsigned long)dev_num << 16) | token;
}

static int vmsg_bus_relay_insert(struct vmsg_bus_session *session, u16 dev_num,
				 u16 token, u8 msg_id, u64 relay_seq)
{
	struct vmsg_bus_relay_meta *meta;
	int ret;

	if (!session || !relay_seq)
		return -EINVAL;

	meta = kmalloc(sizeof(*meta), GFP_KERNEL);
	if (!meta)
		return -ENOMEM;

	meta->dev_num = dev_num;
	meta->token = token;
	meta->msg_id = msg_id;
	meta->relay_seq = relay_seq;

	mutex_lock(&session->relay_lock);
	ret = xa_err(xa_store(&session->relay_meta, vmsg_bus_relay_key(dev_num, token),
			      meta, GFP_KERNEL));
	mutex_unlock(&session->relay_lock);
	if (ret)
		kfree(meta);

	return ret;
}

static int vmsg_bus_relay_take(struct vmsg_bus_session *session, u16 dev_num,
			       u16 token, u8 *msg_id, u64 *relay_seq)
{
	struct vmsg_bus_relay_meta *meta;
	unsigned long key;

	if (!session || !relay_seq)
		return -EINVAL;

	key = vmsg_bus_relay_key(dev_num, token);
	mutex_lock(&session->relay_lock);
	meta = xa_erase(&session->relay_meta, key);
	mutex_unlock(&session->relay_lock);
	if (!meta)
		return -ENOENT;

	if (msg_id)
		*msg_id = meta->msg_id;
	*relay_seq = meta->relay_seq;
	kfree(meta);
	return 0;
}

static void vmsg_bus_relay_purge_session(struct vmsg_bus_session *session)
{
	struct vmsg_bus_relay_meta *meta;
	unsigned long index;

	if (!session)
		return;

	mutex_lock(&session->relay_lock);
	xa_for_each(&session->relay_meta, index, meta) {
		xa_erase(&session->relay_meta, index);
		kfree(meta);
	}
	xa_destroy(&session->relay_meta);
	xa_init(&session->relay_meta);
	mutex_unlock(&session->relay_lock);
}

int virtio_msg_bus_bridge_device_relay_drop(u32 handle, u16 dev_num,
					    u16 token)
{
	struct vmsg_bus_handle_entry *entry;
	struct vmsg_bus_session *session;
	u64 relay_seq = 0;
	int ret;

	if (!handle)
		return -EINVAL;

	mutex_lock(&vmsg_bus_handles_lock);
	entry = xa_load(&vmsg_bus_handles, handle);
	if (!entry || !entry->session) {
		mutex_unlock(&vmsg_bus_handles_lock);
		return -ENODEV;
	}

	session = vmsg_bus_session_get(entry->session);
	mutex_unlock(&vmsg_bus_handles_lock);
	if (!session)
		return -ENODEV;

	if (session->endpoint_id != handle || READ_ONCE(session->detached) ||
	    !READ_ONCE(session->endpoint)) {
		ret = -ENOTCONN;
		goto out_put;
	}

	ret = vmsg_bus_relay_take(session, dev_num, token, NULL, &relay_seq);
	if (ret == -ENOENT)
		ret = 0;

out_put:
	vmsg_bus_session_put(session);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_relay_drop);

static int
vmsg_bus_validate_transport_msg_type(const struct virtio_msg *msg)
{
	u8 type;

	if (!msg)
		return -EINVAL;

	type = msg->type;
	if (type & ~(VIRTIO_MSG_TYPE_RESPONSE | VIRTIO_MSG_TYPE_BUS))
		return -EINVAL;
	if (type & VIRTIO_MSG_TYPE_BUS)
		return -EOPNOTSUPP;

	return 0;
}

static int
vmsg_bus_dispatch_validate_transport_target
		(struct virtio_msg_bus_bridge_device *endpoint,
		 const struct virtio_msg *msg)
{
	struct vmsg_bus_endpoint_topology *topology;
	u16 dev_num;
	int ret;

	if (!endpoint || !msg)
		return -EINVAL;

	ret = vmsg_bus_validate_transport_msg_type(msg);
	if (ret)
		return ret;

	dev_num = le16_to_cpu(msg->dev_num);
	topology = READ_ONCE(endpoint->topology);
	if (!topology || !test_bit(dev_num, topology->devices_bitmap))
		ret = -ENODEV;

	return ret;
}

static void vmsg_bus_session_tx_retry_arm(struct vmsg_bus_session *session)
{
	unsigned long irqflags;

	if (!session)
		return;

	spin_lock_irqsave(&session->exec_lock, irqflags);
	if (!session->kick_hooked || READ_ONCE(session->detached) ||
	    READ_ONCE(session->ring_fault) || session->tx_retry_armed) {
		spin_unlock_irqrestore(&session->exec_lock, irqflags);
		return;
	}
	session->tx_retry_armed = true;
	if (!schedule_delayed_work(&session->tx_retry_work,
				   msecs_to_jiffies(VMSG_BUS_SESSION_TX_RETRY_DELAY_MS)))
		session->tx_retry_armed = false;
	spin_unlock_irqrestore(&session->exec_lock, irqflags);
}

static void vmsg_bus_session_tx_disconnect(struct vmsg_bus_session *session)
{
	unsigned long irqflags;

	if (!session)
		return;

	spin_lock_irqsave(&session->exec_lock, irqflags);
	session->tx_retry_armed = false;
	spin_unlock_irqrestore(&session->exec_lock, irqflags);
	cancel_delayed_work_sync(&session->tx_retry_work);
	vmsg_bus_session_mark_offline(session,
				      VMSG_BRIDGE_UAPI_ENDPOINT_OFFLINE_REASON_REMOVED);
}

static void vmsg_bus_session_tx_drain(struct vmsg_bus_session *session)
{
	struct vmsg_bridge_uapi_ring_hdr hdr;
	u32 used;
	u8 *slot_buf;
	int ret;

	if (!session || READ_ONCE(session->ring_fault))
		return;

	slot_buf = kmalloc(session->tx.info.entry_size, GFP_KERNEL);
	if (!slot_buf)
		return;

	for (;;) {
		const struct virtio_msg *msg = (const struct virtio_msg *)slot_buf;
		const struct virtio_msg_dispatch_ctx *dctx_ptr;
		struct virtio_msg_dispatch_ctx dctx = { 0 };
		u64 relay_seq = 0;
		u16 msg_size;

		if (READ_ONCE(session->detached) || READ_ONCE(session->ring_fault))
			break;

		ret = vmsg_bus_ring_hdr_read(&session->tx, &hdr);
		if (ret)
			goto out_fault;

		ret = vmsg_bus_ring_occupancy(&hdr, &used);
		if (ret)
			goto out_fault;
		if (!used)
			break;

		ret = vmsg_bus_ring_slot_read(&session->tx, hdr.cons, slot_buf);
		if (ret)
			goto out_fault;

		msg_size = le16_to_cpu(msg->msg_size);
		if (msg_size < sizeof(*msg) ||
		    msg_size > session->tx.info.entry_size ||
		    msg_size > VIRTIO_MSG_MAX_SIZE)
			goto out_fault;

		dctx_ptr = NULL;
		if (msg->type & VIRTIO_MSG_TYPE_RESPONSE) {
			ret = vmsg_bus_relay_take(session,
						  le16_to_cpu(msg->dev_num),
						  le16_to_cpu(msg->token),
						  NULL,
						  &relay_seq);
			if (ret == -ENOENT) {
				hdr.cons++;
				ret = vmsg_bus_ring_publish_cons(&session->tx, &hdr);
				if (ret)
					goto out_fault;
				continue;
			}
			if (ret)
				goto out_fault;
			dctx.flags = VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE;
			dctx.relay_seq = relay_seq;
			dctx_ptr = &dctx;
		}

		ret = virtio_msg_bus_bridge_device_rx(session->endpoint_id, msg,
						      dctx_ptr);
		if (!ret) {
			hdr.cons++;
			ret = vmsg_bus_ring_publish_cons(&session->tx, &hdr);
			if (ret)
				goto out_fault;
			continue;
		}
		if ((msg->type & VIRTIO_MSG_TYPE_RESPONSE) && ret == -ENOENT) {
			hdr.cons++;
			ret = vmsg_bus_ring_publish_cons(&session->tx, &hdr);
			if (ret)
				goto out_fault;
			continue;
		}
		if (ret == -EAGAIN) {
			vmsg_bus_session_tx_retry_arm(session);
			break;
		}
		if (ret == -ENODEV || ret == -ENOTCONN || ret == -ESHUTDOWN) {
			vmsg_bus_session_tx_disconnect(session);
			break;
		}
		goto out_fault;
	}

	kfree(slot_buf);
	return;

out_fault:
	kfree(slot_buf);
	vmsg_bus_session_mark_ring_fault(session);
}

static bool vmsg_bus_session_rx_publish_drain(struct vmsg_bus_session *session)
{
	struct vmsg_bus_rx_publish_entry *entry;
	struct vmsg_bridge_uapi_ring_hdr hdr;
	u32 used;
	bool published = false;
	unsigned long irqflags;
	int ret;

	if (!session || READ_ONCE(session->ring_fault))
		return false;

	for (;;) {
		if (READ_ONCE(session->detached) || READ_ONCE(session->ring_fault))
			break;

		ret = vmsg_bus_ring_hdr_read(&session->rx, &hdr);
		if (ret)
			goto out_fault;

		ret = vmsg_bus_ring_occupancy(&hdr, &used);
		if (ret)
			goto out_fault;
		if (used == hdr.entries)
			break;

		spin_lock_irqsave(&session->exec_lock, irqflags);
		entry = list_first_entry_or_null(&session->rx_publish_queue,
						 struct vmsg_bus_rx_publish_entry,
						 node);
		spin_unlock_irqrestore(&session->exec_lock, irqflags);
		if (!entry)
			break;

		ret = vmsg_bus_ring_slot_write(&session->rx, hdr.prod,
					       entry->data, entry->msg_size);
		if (ret)
			goto out_fault;

		hdr.prod++;
		ret = vmsg_bus_ring_publish_prod(&session->rx, &hdr);
		if (ret)
			goto out_fault;

		spin_lock_irqsave(&session->exec_lock, irqflags);
		if (!list_empty(&session->rx_publish_queue) &&
		    list_first_entry(&session->rx_publish_queue,
				     struct vmsg_bus_rx_publish_entry,
				     node) == entry) {
			list_del(&entry->node);
			session->rx_publish_queue_len--;
		}
		spin_unlock_irqrestore(&session->exec_lock, irqflags);

		kfree(entry);
		published = true;
	}

	return published;

out_fault:
	vmsg_bus_session_mark_ring_fault(session);
	return published;
}

static void vmsg_bus_session_exec_workfn(struct work_struct *work)
{
	struct vmsg_bus_session *session;
	unsigned long actions;

	session = container_of(work, struct vmsg_bus_session, exec_work);

	for (;;) {
		bool published = false;
		bool do_wake = false;
		__poll_t poll_mask = EPOLLIN | EPOLLRDNORM;

		actions = vmsg_bus_session_exec_fetch(session);
		if (!actions)
			break;
		if (READ_ONCE(session->detached))
			break;

		if (actions & VMSG_BUS_SESSION_ACT_TX_DRAIN)
			vmsg_bus_session_tx_drain(session);
		if (actions & VMSG_BUS_SESSION_ACT_RX_PUBLISH) {
			published = vmsg_bus_session_rx_publish_drain(session);
			if (published)
				do_wake = true;
		}
		if (actions & VMSG_BUS_SESSION_ACT_WAKE)
			do_wake = true;

		if (READ_ONCE(session->ring_fault))
			poll_mask |= EPOLLERR;
		if (do_wake) {
			if (session->call_evt)
				eventfd_signal(session->call_evt);
			wake_up_interruptible_poll(&session->poll_waitq, poll_mask);
		}
	}
}

static void vmsg_bus_session_exec_sync_disable(struct vmsg_bus_session *session)
{
	bool kick_hooked;
	unsigned long irqflags;
	u64 cnt;

	if (!session)
		return;

	spin_lock_irqsave(&session->exec_lock, irqflags);
	kick_hooked = session->kick_hooked;
	session->kick_hooked = false;
	session->tx_retry_armed = false;
	session->exec_actions = 0;
	spin_unlock_irqrestore(&session->exec_lock, irqflags);

	if (kick_hooked && session->kick_evt)
		eventfd_ctx_remove_wait_queue(session->kick_evt, &session->kick_wait,
					      &cnt);
	cancel_delayed_work_sync(&session->tx_retry_work);
	cancel_work_sync(&session->exec_work);
	vmsg_bus_rx_publish_queue_purge(session);
	vmsg_bus_relay_purge_session(session);
}

static void vmsg_bus_session_release_io(struct vmsg_bus_session *session)
{
	if (!session)
		return;

	vmsg_bus_ring_state_release(&session->tx);
	vmsg_bus_ring_state_release(&session->rx);

	if (session->kick_evt) {
		eventfd_ctx_put(session->kick_evt);
		session->kick_evt = NULL;
	}

	if (session->call_evt) {
		eventfd_ctx_put(session->call_evt);
		session->call_evt = NULL;
	}

	if (session->ring_file) {
		fput(session->ring_file);
		session->ring_file = NULL;
	}
}

static struct vmsg_bus_session *
vmsg_bus_session_get(struct vmsg_bus_session *session)
{
	if (!session)
		return NULL;

	if (!refcount_inc_not_zero(&session->refs))
		return NULL;

	return session;
}

static void vmsg_bus_session_free(struct vmsg_bus_session *session)
{
	struct vmsg_bus_control_entry *entry;
	struct vmsg_bus_control_entry *tmp;

	if (!session)
		return;

	vmsg_bus_session_exec_sync_disable(session);

	mutex_lock(&session->control_lock);
	list_for_each_entry_safe(entry, tmp, &session->control_queue, node) {
		list_del(&entry->node);
		kfree(entry);
	}
	mutex_unlock(&session->control_lock);

	mutex_lock(&session->map_lock);
	vmsg_bus_map_event_purge_locked(session);
	vmsg_bus_map_records_purge_locked(session);
	mutex_unlock(&session->map_lock);

	xa_destroy(&session->relay_meta);

	if (session->ctx) {
		vmsg_bus_file_ctx_put(session->ctx);
		session->ctx = NULL;
	}

	vmsg_bus_session_release_io(session);
	kfree(session);
}

static void vmsg_bus_session_put(struct vmsg_bus_session *session)
{
	if (!session)
		return;

	if (refcount_dec_and_test(&session->refs))
		vmsg_bus_session_free(session);
}

static struct vmsg_bus_session *
vmsg_bus_endpoint_publish_session_get(struct virtio_msg_bus_bridge_device *endpoint)
{
	struct vmsg_bus_session *session = NULL;
	unsigned long irqflags;

	if (!endpoint)
		return NULL;

	spin_lock_irqsave(&endpoint->publish_lock, irqflags);
	session = vmsg_bus_session_get(endpoint->publish_session);
	spin_unlock_irqrestore(&endpoint->publish_lock, irqflags);

	return session;
}

static void
vmsg_bus_endpoint_publish_session_set(struct virtio_msg_bus_bridge_device *endpoint,
				      struct vmsg_bus_session *session)
{
	struct vmsg_bus_session *prev;
	unsigned long irqflags;

	if (!endpoint || !session)
		return;

	session = vmsg_bus_session_get(session);
	if (!session)
		return;

	spin_lock_irqsave(&endpoint->publish_lock, irqflags);
	prev = endpoint->publish_session;
	endpoint->publish_session = session;
	spin_unlock_irqrestore(&endpoint->publish_lock, irqflags);

	vmsg_bus_session_put(prev);
}

static void
vmsg_bus_endpoint_publish_session_clear(struct virtio_msg_bus_bridge_device *endpoint,
					struct vmsg_bus_session *session)
{
	struct vmsg_bus_session *prev;
	unsigned long irqflags;

	if (!endpoint)
		return;

	spin_lock_irqsave(&endpoint->publish_lock, irqflags);
	prev = endpoint->publish_session;
	if (prev && (!session || prev == session))
		endpoint->publish_session = NULL;
	else
		prev = NULL;
	spin_unlock_irqrestore(&endpoint->publish_lock, irqflags);

	vmsg_bus_session_put(prev);
}

static int vmsg_bus_ctrl_payload_size_validate(u16 ctrl_type, u32 payload_len)
{
	switch (ctrl_type) {
	case VMSG_BRIDGE_UAPI_CTRL_TYPE_ENDPOINT_ONLINE:
		if (payload_len !=
		    sizeof(struct vmsg_bridge_uapi_ctrl_endpoint_online))
			return -EINVAL;
		break;
	case VMSG_BRIDGE_UAPI_CTRL_TYPE_ENDPOINT_OFFLINE:
		if (payload_len !=
		    sizeof(struct vmsg_bridge_uapi_ctrl_endpoint_offline))
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int vmsg_bus_control_enqueue(struct vmsg_bus_session *session, u16 ctrl_type,
				    const void *payload, u32 payload_len)
{
	struct vmsg_bus_control_entry *entry;

	if (!session)
		return -EINVAL;
	if (!payload && payload_len)
		return -EINVAL;
	if (payload_len > VMSG_BRIDGE_UAPI_CTRL_PAYLOAD_MAX)
		return -EINVAL;
	if (vmsg_bus_ctrl_payload_size_validate(ctrl_type, payload_len))
		return -EINVAL;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	mutex_lock(&session->control_lock);
	entry->msg.seq = ++session->control_next_seq;
	entry->msg.type = ctrl_type;
	entry->msg.flags = 0;
	entry->msg.payload_len = payload_len;
	if (payload_len)
		memcpy(entry->msg.payload, payload, payload_len);
	list_add_tail(&entry->node, &session->control_queue);
	mutex_unlock(&session->control_lock);

	vmsg_bus_session_exec_queue(session, VMSG_BUS_SESSION_ACT_WAKE);

	return 0;
}

static int vmsg_bus_map_id_to_index(u64 map_id, unsigned long *index)
{
	if (!map_id || !index)
		return -EINVAL;
	if (map_id > (u64)ULONG_MAX)
		return -EOVERFLOW;

	*index = (unsigned long)map_id;
	return 0;
}

static int vmsg_bus_map_offset_to_index(u64 mmap_offset, unsigned long *index)
{
	u64 shifted;

	if (!PAGE_ALIGNED(mmap_offset) || !index)
		return -EINVAL;

	shifted = mmap_offset >> PAGE_SHIFT;
	if (shifted > (u64)ULONG_MAX)
		return -EOVERFLOW;

	*index = (unsigned long)shifted;
	return 0;
}

static void vmsg_bus_session_caps_clear(struct vmsg_bus_session *session)
{
	if (!session)
		return;

	memset(&session->caps, 0, sizeof(session->caps));
	session->endpoint_max_msg_size = 0;
}

static int vmsg_bus_ring_entry_size_calc(u32 vmm_max_msg_size, u32 *entry_size)
{
	u64 aligned_entry_size;

	if (!entry_size)
		return -EINVAL;
	if (vmm_max_msg_size < VIRTIO_MSG_MIN_SIZE ||
	    vmm_max_msg_size > VIRTIO_MSG_MAX_SIZE)
		return -EINVAL;

	aligned_entry_size = ALIGN((u64)vmm_max_msg_size,
				   VMSG_BRIDGE_UAPI_RING_ENTRY_ALIGN);
	if (aligned_entry_size > U32_MAX)
		return -EOVERFLOW;

	*entry_size = (u32)aligned_entry_size;
	return 0;
}

static struct vmsg_bus_map_record *
vmsg_bus_map_record_lookup_by_id_locked(struct vmsg_bus_session *session, u64 map_id)
{
	unsigned long index;

	lockdep_assert_held(&session->map_lock);

	if (vmsg_bus_map_id_to_index(map_id, &index))
		return NULL;

	return xa_load(&session->map_records_by_id, index);
}

static struct vmsg_bus_map_record *
vmsg_bus_map_record_lookup_by_offset_locked(struct vmsg_bus_session *session,
					    u64 mmap_offset)
{
	unsigned long index;

	lockdep_assert_held(&session->map_lock);

	if (vmsg_bus_map_offset_to_index(mmap_offset, &index))
		return NULL;

	return xa_load(&session->map_records_by_offset, index);
}

static int
vmsg_bus_map_event_alloc_seq_locked(struct vmsg_bus_session *session, u64 *seq)
{
	lockdep_assert_held(&session->map_lock);

	if (!session->map_event_next_seq || !seq)
		return -EOVERFLOW;

	*seq = session->map_event_next_seq;
	if (session->map_event_next_seq == U64_MAX)
		session->map_event_next_seq = 0;
	else
		session->map_event_next_seq++;

	return 0;
}

static int
vmsg_bus_map_event_alloc_id_locked(struct vmsg_bus_session *session, u64 *map_id)
{
	lockdep_assert_held(&session->map_lock);

	if (!session->map_event_next_id || !map_id)
		return -EOVERFLOW;

	*map_id = session->map_event_next_id;
	if (session->map_event_next_id == U64_MAX)
		session->map_event_next_id = 0;
	else
		session->map_event_next_id++;

	return 0;
}

static int vmsg_bus_map_record_insert_locked(struct vmsg_bus_session *session,
					     struct vmsg_bus_map_record *record)
{
	unsigned long map_id_index;
	unsigned long offset_index;
	int ret;

	lockdep_assert_held(&session->map_lock);

	ret = vmsg_bus_map_id_to_index(record->map_id, &map_id_index);
	if (ret)
		return ret;
	ret = vmsg_bus_map_offset_to_index(record->mmap_offset, &offset_index);
	if (ret)
		return ret;

	if (xa_load(&session->map_records_by_id, map_id_index))
		return -EEXIST;
	if (xa_load(&session->map_records_by_offset, offset_index))
		return -EEXIST;

	ret = xa_err(xa_store(&session->map_records_by_id, map_id_index, record,
			      GFP_KERNEL));
	if (ret)
		return ret;

	ret = xa_err(xa_store(&session->map_records_by_offset, offset_index, record,
			      GFP_KERNEL));
	if (ret) {
		xa_erase(&session->map_records_by_id, map_id_index);
		return ret;
	}

	return 0;
}

static void
vmsg_bus_map_record_remove_locked(struct vmsg_bus_session *session,
				  struct vmsg_bus_map_record *record)
{
	unsigned long map_id_index;
	unsigned long offset_index;

	lockdep_assert_held(&session->map_lock);

	if (!record)
		return;

	if (!vmsg_bus_map_id_to_index(record->map_id, &map_id_index))
		xa_erase(&session->map_records_by_id, map_id_index);
	if (!vmsg_bus_map_offset_to_index(record->mmap_offset, &offset_index))
		xa_erase(&session->map_records_by_offset, offset_index);

	kfree(record);
}

static u32 vmsg_bus_map_records_purge_locked(struct vmsg_bus_session *session)
{
	struct vmsg_bus_map_record *record;
	unsigned long index;
	u32 purged = 0;

	lockdep_assert_held(&session->map_lock);

	xa_for_each(&session->map_records_by_id, index, record) {
		xa_erase(&session->map_records_by_id, index);
		kfree(record);
		purged++;
	}

	xa_destroy(&session->map_records_by_id);
	xa_init(&session->map_records_by_id);
	xa_destroy(&session->map_records_by_offset);
	xa_init(&session->map_records_by_offset);

	return purged;
}

static struct vmsg_bus_map_event_entry *
vmsg_bus_map_event_head_locked(struct vmsg_bus_session *session)
{
	lockdep_assert_held(&session->map_lock);

	return list_first_entry_or_null(&session->map_event_queue,
					struct vmsg_bus_map_event_entry, node);
}

static int
vmsg_bus_map_event_enqueue_locked(struct vmsg_bus_session *session,
				  const struct vmsg_bridge_uapi_map_event *event)
{
	struct vmsg_bus_map_event_entry *entry;

	lockdep_assert_held(&session->map_lock);

	if (session->map_event_queue_len >= VMSG_BUS_MAP_EVENT_QUEUE_MAX)
		return -ENOSPC;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	INIT_LIST_HEAD(&entry->node);
	entry->event = *event;
	entry->delivered = false;
	entry->deliver_after_jiffies = jiffies;
	list_add_tail(&entry->node, &session->map_event_queue);
	session->map_event_queue_len++;

	return 0;
}

static void vmsg_bus_map_event_purge_locked(struct vmsg_bus_session *session)
{
	struct vmsg_bus_map_event_entry *entry;
	struct vmsg_bus_map_event_entry *tmp;

	lockdep_assert_held(&session->map_lock);

	list_for_each_entry_safe(entry, tmp, &session->map_event_queue, node) {
		list_del(&entry->node);
		kfree(entry);
	}
	session->map_event_queue_len = 0;
}

static int
vmsg_bus_ring_info_validate(struct file *ring_file,
			    const struct vmsg_bridge_uapi_ring_info *info,
			    u32 vmm_max_msg_size,
			    struct vmsg_bus_ring_state *state)
{
	u64 data_bytes;
	u64 min_bytes;
	u64 end;
	loff_t ring_file_size;
	u32 entry_size;
	int ret;

	if (!ring_file || !info || !state)
		return -EINVAL;
	if (info->reserved)
		return -EINVAL;
	if (!info->entries || !info->entry_size)
		return -EINVAL;
	if (info->entries != VMSG_BRIDGE_UAPI_RING_ENTRIES_DEFAULT)
		return -EINVAL;

	ret = vmsg_bus_ring_entry_size_calc(vmm_max_msg_size, &entry_size);
	if (ret)
		return ret;
	if (info->entry_size != entry_size)
		return -EINVAL;

	if (check_mul_overflow((u64)info->entries, (u64)info->entry_size,
			       &data_bytes))
		return -EINVAL;
	if (check_add_overflow((u64)sizeof(struct vmsg_bridge_uapi_ring_hdr),
			       data_bytes, &min_bytes))
		return -EINVAL;
	if (info->bytes < min_bytes)
		return -EINVAL;
	if (check_add_overflow(info->offset, info->bytes, &end))
		return -EINVAL;

	ring_file_size = i_size_read(file_inode(ring_file));
	if (end > (u64)ring_file_size)
		return -EINVAL;

	state->info = *info;
	state->end = end;
	return 0;
}

static bool vmsg_bus_ring_ranges_overlap(const struct vmsg_bus_ring_state *tx,
					 const struct vmsg_bus_ring_state *rx)
{
	return tx->info.offset < rx->end && rx->info.offset < tx->end;
}

static unsigned long
vmsg_bus_endpoint_key(const struct virtio_msg_bus_bridge_device *endpoint)
{
	return (unsigned long)endpoint;
}

static struct vmsg_bus_endpoint_topology *
vmsg_bus_endpoint_topology_lookup_locked(struct virtio_msg_bus_bridge_device *endpoint)
{
	lockdep_assert_held(&vmsg_bus_endpoint_topologies_lock);

	return xa_load(&vmsg_bus_endpoint_topologies, vmsg_bus_endpoint_key(endpoint));
}

static struct vmsg_bus_endpoint_topology *
vmsg_bus_endpoint_topology_get_or_create_locked(struct virtio_msg_bus_bridge_device *endpoint,
						int *ret)
{
	struct vmsg_bus_endpoint_topology *topology;

	lockdep_assert_held(&vmsg_bus_endpoint_topologies_lock);

	topology = vmsg_bus_endpoint_topology_lookup_locked(endpoint);
	if (topology) {
		WRITE_ONCE(endpoint->topology, topology);
		return topology;
	}

	topology = kzalloc(sizeof(*topology), GFP_KERNEL);
	if (!topology) {
		*ret = -ENOMEM;
		return NULL;
	}

	xa_init(&topology->devices);
	bitmap_zero(topology->devices_bitmap, U16_MAX + 1U);
	*ret = xa_err(xa_store(&vmsg_bus_endpoint_topologies,
			       vmsg_bus_endpoint_key(endpoint), topology,
			       GFP_KERNEL));
	if (*ret) {
		xa_destroy(&topology->devices);
		kfree(topology);
		return NULL;
	}

	WRITE_ONCE(endpoint->topology, topology);
	return topology;
}

static void
vmsg_bus_endpoint_topology_destroy(struct vmsg_bus_endpoint_topology *topology)
{
	if (!topology)
		return;

	xa_destroy(&topology->devices);
	kfree(topology);
}

static void
vmsg_bus_endpoint_topology_remove_locked(struct virtio_msg_bus_bridge_device *endpoint)
{
	struct vmsg_bus_endpoint_topology *topology;

	lockdep_assert_held(&vmsg_bus_endpoint_topologies_lock);

	WRITE_ONCE(endpoint->topology, NULL);
	topology = xa_erase(&vmsg_bus_endpoint_topologies,
			    vmsg_bus_endpoint_key(endpoint));
	vmsg_bus_endpoint_topology_destroy(topology);
}

static int
vmsg_bus_bridge_topology_add_locked(struct vmsg_bus_endpoint_topology *topology,
				    u16 dev_num)
{
	int ret;

	lockdep_assert_held(&vmsg_bus_endpoint_topologies_lock);

	if (xa_load(&topology->devices, dev_num))
		return -EEXIST;

	/*
	 * RX membership checks are lockless and bitmap-only, so publish add
	 * intent in the bitmap first and roll it back if xa_store() fails.
	 */
	set_bit(dev_num, topology->devices_bitmap);
	ret = xa_err(xa_store(&topology->devices, dev_num, xa_mk_value(1),
			      GFP_KERNEL));
	if (ret)
		clear_bit(dev_num, topology->devices_bitmap);

	return ret;
}

static int
vmsg_bus_bridge_topology_remove_locked(struct vmsg_bus_endpoint_topology *topology,
				       u16 dev_num)
{
	lockdep_assert_held(&vmsg_bus_endpoint_topologies_lock);

	if (!xa_load(&topology->devices, dev_num))
		return -ENOENT;

	/*
	 * Make lockless bitmap readers reject immediately once removal starts.
	 */
	clear_bit(dev_num, topology->devices_bitmap);
	xa_erase(&topology->devices, dev_num);
	return 0;
}

static void
vmsg_bus_bridge_topology_clear_locked(struct vmsg_bus_endpoint_topology *topology)
{
	void *entry;
	unsigned long index;

	lockdep_assert_held(&vmsg_bus_endpoint_topologies_lock);

	/*
	 * bitmap_zero() is not safe against concurrent lockless test_bit()
	 * readers. Clear each published device bit with atomic bitops first.
	 */
	xa_for_each(&topology->devices, index, entry)
		clear_bit((u16)index, topology->devices_bitmap);

	xa_destroy(&topology->devices);
	xa_init(&topology->devices);
}

static int
vmsg_bus_endpoint_addr_build(struct vmsg_bridge_uapi_endpoint_addr *addr,
			     const char *bus_name, const char *bus_id)
{
	size_t bus_name_len;
	size_t bus_id_len;

	if (!addr || !bus_name || !bus_id)
		return -EINVAL;

	bus_name_len = strnlen(bus_name, VMSG_BRIDGE_UAPI_BUS_NAME_LEN);
	if (!bus_name_len || bus_name_len >= VMSG_BRIDGE_UAPI_BUS_NAME_LEN)
		return -EINVAL;

	bus_id_len = strnlen(bus_id, VMSG_BRIDGE_UAPI_BUS_ID_LEN);
	if (bus_id_len >= VMSG_BRIDGE_UAPI_BUS_ID_LEN)
		return -EINVAL;

	memset(addr, 0, sizeof(*addr));
	memcpy(addr->bus_name, bus_name, bus_name_len);
	memcpy(addr->bus_id, bus_id, bus_id_len);

	return 0;
}

static int
vmsg_bus_endpoint_addr_validate_uapi(struct vmsg_bridge_uapi_endpoint_addr *addr)
{
	size_t bus_name_len;
	size_t bus_id_len;
	size_t bus_name_tail;
	size_t bus_id_tail;

	if (!addr)
		return -EINVAL;

	bus_name_len = strnlen(addr->bus_name, VMSG_BRIDGE_UAPI_BUS_NAME_LEN);
	if (!bus_name_len || bus_name_len >= VMSG_BRIDGE_UAPI_BUS_NAME_LEN)
		return -EINVAL;

	bus_id_len = strnlen(addr->bus_id, VMSG_BRIDGE_UAPI_BUS_ID_LEN);
	if (bus_id_len >= VMSG_BRIDGE_UAPI_BUS_ID_LEN)
		return -EINVAL;

	bus_name_tail = VMSG_BRIDGE_UAPI_BUS_NAME_LEN - bus_name_len - 1;
	if (bus_name_tail &&
	    memchr_inv(addr->bus_name + bus_name_len + 1, 0, bus_name_tail))
		return -EINVAL;

	bus_id_tail = VMSG_BRIDGE_UAPI_BUS_ID_LEN - bus_id_len - 1;
	if (bus_id_tail &&
	    memchr_inv(addr->bus_id + bus_id_len + 1, 0, bus_id_tail))
		return -EINVAL;

	return 0;
}

static struct vmsg_bus_resolver_entry *
vmsg_bus_find_resolver_locked(const char *bus_name)
{
	struct vmsg_bus_resolver_entry *entry;

	lockdep_assert_held(&vmsg_bus_resolvers_lock);

	list_for_each_entry(entry, &vmsg_bus_resolvers, node) {
		if (!strncmp(entry->resolver->name, bus_name,
			     VMSG_BRIDGE_UAPI_BUS_NAME_LEN))
			return entry;
	}

	return NULL;
}

static int
vmsg_bus_resolver_validate_addr(struct vmsg_bridge_uapi_endpoint_addr *addr,
				bool strict_unknown_bus)
{
	struct vmsg_bus_resolver_entry *entry;
	struct vmsg_bus_resolver *resolver;
	char bus_id[VMSG_BRIDGE_UAPI_BUS_ID_LEN];
	int ret;

	if (!addr)
		return -EINVAL;

	mutex_lock(&vmsg_bus_resolvers_lock);
	entry = vmsg_bus_find_resolver_locked(addr->bus_name);
	resolver = entry ? entry->resolver : NULL;
	if (!resolver) {
		mutex_unlock(&vmsg_bus_resolvers_lock);
		return strict_unknown_bus ? -ENOENT : -ENODEV;
	}
	if (!resolver->validate) {
		mutex_unlock(&vmsg_bus_resolvers_lock);
		return 0;
	}

	memset(bus_id, 0, sizeof(bus_id));
	strscpy(bus_id, addr->bus_id, sizeof(bus_id));
	ret = resolver->validate(resolver, bus_id);
	mutex_unlock(&vmsg_bus_resolvers_lock);
	if (ret)
		return ret;

	memset(addr->bus_id, 0, sizeof(addr->bus_id));
	strscpy(addr->bus_id, bus_id, sizeof(addr->bus_id));

	return 0;
}

static struct vmsg_bus_handle_entry *
vmsg_bus_lookup_handle_entry_by_addr_locked(const struct vmsg_bridge_uapi_endpoint_addr *addr,
					    u32 *out_handle)
{
	struct vmsg_bus_handle_entry *entry;
	unsigned long handle;

	lockdep_assert_held(&vmsg_bus_handles_lock);

	xa_for_each(&vmsg_bus_handles, handle, entry) {
		if (memcmp(&entry->addr, addr, sizeof(*addr)))
			continue;

		if (out_handle)
			*out_handle = (u32)handle;
		return entry;
	}

	if (out_handle)
		*out_handle = 0;
	return NULL;
}

static u32 vmsg_bus_alloc_handle(void)
{
	u32 handle;

	lockdep_assert_held(&vmsg_bus_handles_lock);

	for (;;) {
		handle = (u32)atomic_inc_return(&vmsg_bus_next_handle);
		if (!handle)
			continue;

		if (!xa_load(&vmsg_bus_handles, handle))
			return handle;
	}
}

static void vmsg_bus_handle_try_free_locked(u32 handle,
					    struct vmsg_bus_handle_entry *entry)
{
	lockdep_assert_held(&vmsg_bus_handles_lock);

	if (!entry)
		return;
	if (entry->resolve_refs)
		return;
	if (entry->endpoint)
		return;
	if (entry->session)
		return;

	xa_erase(&vmsg_bus_handles, handle);
	kfree(entry);
	vmsg_bus_cleanup_busy_entry_destroy(handle);
}

static int
vmsg_bus_resolve_addr_locked(struct vmsg_bridge_uapi_endpoint_addr *addr,
			     bool create, u32 *out_handle)
{
	struct vmsg_bus_handle_entry *entry;
	u32 handle;
	int ret;

	lockdep_assert_held(&vmsg_bus_handles_lock);

	entry = vmsg_bus_lookup_handle_entry_by_addr_locked(addr, &handle);
	if (entry) {
		if (entry->resolve_refs == U32_MAX)
			return -EOVERFLOW;

		entry->resolve_refs++;
		*out_handle = handle;
		return 0;
	}

	if (!create)
		return -ENODEV;

	handle = vmsg_bus_alloc_handle();
	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->addr = *addr;
	entry->resolve_refs = 1;
	ret = xa_err(xa_store(&vmsg_bus_handles, handle, entry, GFP_KERNEL));
	if (ret) {
		kfree(entry);
		return ret;
	}

	ret = vmsg_bus_cleanup_busy_entry_init(handle);
	if (ret) {
		xa_erase(&vmsg_bus_handles, handle);
		kfree(entry);
		return ret;
	}

	*out_handle = handle;
	return 0;
}

static void vmsg_bus_release_handle_locked(u32 handle)
{
	struct vmsg_bus_handle_entry *entry;

	lockdep_assert_held(&vmsg_bus_handles_lock);

	entry = xa_load(&vmsg_bus_handles, handle);
	if (!entry)
		return;

	if (entry->resolve_refs)
		entry->resolve_refs--;

	vmsg_bus_handle_try_free_locked(handle, entry);
}

static int
vmsg_bus_bridge_endpoint_resolve(struct virtio_msg_bus_bridge_device *endpoint,
				 u32 *out_handle,
				 struct vmsg_bus_session **out_session)
{
	struct vmsg_bus_handle_entry *entry;
	struct vmsg_bus_session *session = NULL;
	u32 handle;
	int ret = 0;

	if (!endpoint)
		return -EINVAL;

	handle = READ_ONCE(endpoint->handle);
	if (!handle)
		return -ENODEV;

	mutex_lock(&vmsg_bus_handles_lock);
	entry = xa_load(&vmsg_bus_handles, handle);
	if (!entry || entry->endpoint != endpoint) {
		ret = -ENODEV;
		goto out_unlock_handles;
	}

	session = vmsg_bus_session_get(entry->session);
	if (out_handle)
		*out_handle = handle;
	if (out_session)
		*out_session = session;
	else
		vmsg_bus_session_put(session);

out_unlock_handles:
	mutex_unlock(&vmsg_bus_handles_lock);
	return ret;
}

static void
vmsg_bus_bridge_endpoint_quiesce(struct virtio_msg_bus_bridge_device *endpoint)
{
	if (!endpoint)
		return;

	WRITE_ONCE(endpoint->rx_unregistered, true);
	if (endpoint->ops && endpoint->ops->synchronize_cbs)
		endpoint->ops->synchronize_cbs(endpoint);
	wait_event(endpoint->rx_waitq, !atomic_read(&endpoint->rx_inflight));
	mutex_lock(&vmsg_bus_endpoint_topologies_lock);
	WRITE_ONCE(endpoint->topology, NULL);
	mutex_unlock(&vmsg_bus_endpoint_topologies_lock);
}

static void
vmsg_bus_bridge_endpoint_mark_session_offline(struct vmsg_bus_session *session,
					      u32 endpoint_offline_reason)
{
	struct vmsg_bus_file_ctx *ctx;

	if (!session)
		return;

	ctx = vmsg_bus_file_ctx_get(READ_ONCE(session->ctx));
	if (ctx)
		mutex_lock(&ctx->lock);
	if (ctx && session->ctx == ctx && ctx->session == session)
		vmsg_bus_session_mark_offline(session, endpoint_offline_reason);
	if (ctx)
		mutex_unlock(&ctx->lock);
	vmsg_bus_file_ctx_put(ctx);
}

int vmsg_bus_resolver_register(struct vmsg_bus_resolver *resolver)
{
	struct vmsg_bus_resolver_entry *entry;
	struct vmsg_bus_resolver_entry *iter;

	if (!resolver || !resolver->name[0] || !resolver->validate ||
	    !resolver->match)
		return -EINVAL;
	if (strnlen(resolver->name, VMSG_BRIDGE_UAPI_BUS_NAME_LEN) >=
	    VMSG_BRIDGE_UAPI_BUS_NAME_LEN)
		return -EINVAL;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->resolver = resolver;
	mutex_lock(&vmsg_bus_resolvers_lock);
	list_for_each_entry(iter, &vmsg_bus_resolvers, node) {
		if (iter->resolver == resolver ||
		    !strncmp(iter->resolver->name, resolver->name,
			     VMSG_BRIDGE_UAPI_BUS_NAME_LEN)) {
			mutex_unlock(&vmsg_bus_resolvers_lock);
			kfree(entry);
			return -EEXIST;
		}
	}
	list_add_tail(&entry->node, &vmsg_bus_resolvers);
	mutex_unlock(&vmsg_bus_resolvers_lock);
	vm_info(NULL, "bridge resolver registered: bus='%s'\n", resolver->name);
	return 0;
}
EXPORT_SYMBOL_GPL(vmsg_bus_resolver_register);

void vmsg_bus_resolver_unregister(struct vmsg_bus_resolver *resolver)
{
	struct vmsg_bus_resolver_entry *entry;
	struct vmsg_bus_resolver_entry *tmp;

	if (!resolver)
		return;

	mutex_lock(&vmsg_bus_resolvers_lock);
	list_for_each_entry_safe(entry, tmp, &vmsg_bus_resolvers, node) {
		if (entry->resolver != resolver)
			continue;

		list_del(&entry->node);
		kfree(entry);
		vm_info(NULL, "bridge resolver unregistered: bus='%s'\n",
			resolver->name);
		break;
	}
	mutex_unlock(&vmsg_bus_resolvers_lock);
}
EXPORT_SYMBOL_GPL(vmsg_bus_resolver_unregister);

int virtio_msg_bus_bridge_device_register(const char *bus_name,
					  const char *bus_id,
					  struct virtio_msg_bus_bridge_device *endpoint)
{
	struct vmsg_bus_handle_entry *entry;
	struct vmsg_bus_session *session = NULL;
	struct vmsg_bridge_uapi_endpoint_addr addr;
	u32 handle = 0;
	bool created = false;
	int ret;

	if (!endpoint || !endpoint->ops || !endpoint->ops->get_caps ||
	    !endpoint->ops->tx_msg)
		return -EINVAL;

	ret = vmsg_bus_endpoint_addr_build(&addr, bus_name, bus_id);
	if (ret)
		return ret;
	ret = vmsg_bus_endpoint_addr_validate_uapi(&addr);
	if (ret)
		return ret;
	ret = vmsg_bus_resolver_validate_addr(&addr, true);
	if (ret)
		return ret;

	mutex_lock(&vmsg_bus_handles_lock);
	entry = vmsg_bus_lookup_handle_entry_by_addr_locked(&addr, &handle);
	if (entry) {
		if (entry->endpoint) {
			ret = -EEXIST;
			goto out_unlock_handles;
		}
		entry->endpoint = endpoint;
		session = vmsg_bus_session_get(entry->session);
	} else {
		handle = vmsg_bus_alloc_handle();
		entry = kzalloc(sizeof(*entry), GFP_KERNEL);
		if (!entry) {
			ret = -ENOMEM;
			goto out_unlock_handles;
		}

		entry->addr = addr;
		entry->endpoint = endpoint;
		entry->resolve_refs = 0;
		ret = xa_err(xa_store(&vmsg_bus_handles, handle, entry,
				      GFP_KERNEL));
		if (ret) {
			kfree(entry);
			goto out_unlock_handles;
		}
		created = true;
	}

	ret = vmsg_bus_cleanup_busy_entry_init(handle);
	if (ret) {
		entry->endpoint = NULL;
		if (created) {
			xa_erase(&vmsg_bus_handles, handle);
			kfree(entry);
		}
		goto out_unlock_handles;
	}

	endpoint->handle = handle;
	WRITE_ONCE(endpoint->rx_unregistered, false);
	mutex_unlock(&vmsg_bus_handles_lock);
	vmsg_bus_endpoint_publish_session_clear(endpoint, NULL);

	vm_info(NULL, "endpoint registered: bus='%s' handle=%u\n",
		bus_name, handle);

	if (session)
		vmsg_bus_session_rebind_online(session, endpoint);

	vmsg_bus_session_put(session);

	return 0;

out_unlock_handles:
	mutex_unlock(&vmsg_bus_handles_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_register);

void virtio_msg_bus_bridge_device_unregister(const char *bus_name,
					     const char *bus_id,
					     struct virtio_msg_bus_bridge_device *endpoint)
{
	struct vmsg_bus_handle_entry *entry;
	struct vmsg_bus_session *session = NULL;
	struct vmsg_bridge_uapi_endpoint_addr addr;
	u32 handle = 0;
	const u32 endpoint_offline_reason =
		VMSG_BRIDGE_UAPI_ENDPOINT_OFFLINE_REASON_REMOVED;
	bool registered = false;

	if (!endpoint)
		return;

	if (vmsg_bus_endpoint_addr_build(&addr, bus_name, bus_id))
		return;

	vm_info(NULL, "endpoint unregister: bus='%s' handle=%u\n",
		bus_name, endpoint->handle);

	mutex_lock(&vmsg_bus_handles_lock);
	entry = vmsg_bus_lookup_handle_entry_by_addr_locked(&addr, &handle);
	if (!entry)
		goto out_unlock_handles;
	if (entry->endpoint != endpoint)
		goto out_unlock_handles;

	registered = true;
	session = vmsg_bus_session_get(entry->session);
	entry->endpoint = NULL;
	endpoint->handle = 0;
	vmsg_bus_handle_try_free_locked(handle, entry);

out_unlock_handles:
	mutex_unlock(&vmsg_bus_handles_lock);
	if (registered) {
		vmsg_bus_endpoint_publish_session_clear(endpoint, NULL);
		vmsg_bus_bridge_endpoint_quiesce(endpoint);
		mutex_lock(&vmsg_bus_endpoint_topologies_lock);
		vmsg_bus_endpoint_topology_remove_locked(endpoint);
		mutex_unlock(&vmsg_bus_endpoint_topologies_lock);
	}
	if (session) {
		vmsg_bus_bridge_endpoint_mark_session_offline(session,
							      endpoint_offline_reason);
		vmsg_bus_session_put(session);
	}
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_unregister);

int virtio_msg_bus_bridge_endpoint_set_offline(struct virtio_msg_bus_bridge_device
					       *endpoint,
					       enum
					       virtio_msg_bus_bridge_endpoint_offline_reason
					       reason)
{
	struct vmsg_bus_session *session = NULL;
	int ret;

	switch (reason) {
	case VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_OFFLINE_REASON_REMOVED:
	case VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_OFFLINE_REASON_RESET:
		break;
	default:
		return -EINVAL;
	}

	ret = vmsg_bus_bridge_endpoint_resolve(endpoint, NULL, &session);
	if (ret)
		return ret;

	vmsg_bus_bridge_endpoint_quiesce(endpoint);
	if (session) {
		vmsg_bus_bridge_endpoint_mark_session_offline(session, reason);
		vmsg_bus_session_put(session);
	}
	vmsg_bus_endpoint_publish_session_clear(endpoint, NULL);

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_endpoint_set_offline);

int virtio_msg_bus_bridge_endpoint_set_online(struct virtio_msg_bus_bridge_device *endpoint)
{
	struct vmsg_bus_endpoint_topology *topology;
	struct vmsg_bus_session *session = NULL;
	int ret;

	ret = vmsg_bus_bridge_endpoint_resolve(endpoint, NULL, &session);
	if (ret)
		return ret;

	mutex_lock(&vmsg_bus_endpoint_topologies_lock);
	topology = vmsg_bus_endpoint_topology_lookup_locked(endpoint);
	WRITE_ONCE(endpoint->topology, topology);
	mutex_unlock(&vmsg_bus_endpoint_topologies_lock);

	WRITE_ONCE(endpoint->rx_unregistered, false);
	if (session) {
		vmsg_bus_session_rebind_online(session, endpoint);
		vmsg_bus_session_put(session);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_endpoint_set_online);

int
virtio_msg_bus_bridge_topology_add(struct virtio_msg_bus_bridge_device *endpoint,
				   u16 dev_num)
{
	struct vmsg_bus_endpoint_topology *topology;
	int ret;

	if (!endpoint)
		return -EINVAL;

	mutex_lock(&vmsg_bus_endpoint_topologies_lock);
	topology = vmsg_bus_endpoint_topology_get_or_create_locked(endpoint, &ret);
	if (topology)
		ret = vmsg_bus_bridge_topology_add_locked(topology, dev_num);
	mutex_unlock(&vmsg_bus_endpoint_topologies_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_topology_add);

int
virtio_msg_bus_bridge_topology_remove(struct virtio_msg_bus_bridge_device *endpoint,
				      u16 dev_num)
{
	struct vmsg_bus_endpoint_topology *topology;
	int ret;

	if (!endpoint)
		return -EINVAL;

	mutex_lock(&vmsg_bus_endpoint_topologies_lock);
	topology = vmsg_bus_endpoint_topology_lookup_locked(endpoint);
	if (!topology)
		ret = -ENOENT;
	else
		ret = vmsg_bus_bridge_topology_remove_locked(topology, dev_num);
	mutex_unlock(&vmsg_bus_endpoint_topologies_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_topology_remove);

void
virtio_msg_bus_bridge_topology_clear(struct virtio_msg_bus_bridge_device *endpoint)
{
	struct vmsg_bus_endpoint_topology *topology;

	if (!endpoint)
		return;

	mutex_lock(&vmsg_bus_endpoint_topologies_lock);
	topology = vmsg_bus_endpoint_topology_lookup_locked(endpoint);
	if (topology)
		vmsg_bus_bridge_topology_clear_locked(topology);
	mutex_unlock(&vmsg_bus_endpoint_topologies_lock);
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_topology_clear);

int virtio_msg_bus_bridge_topology_get_devices_window(struct virtio_msg_bus_bridge_device *endpoint,
						      u16 offset, u16 count,
						      u8 *bitmap, size_t bitmap_len,
						      u16 *out_num, u16 *out_next_offset)
{
	struct vmsg_bus_endpoint_topology *topology;
	void *entry;
	unsigned long index;
	size_t req_bitmap_len;
	u32 window_end;
	u16 next_offset = 0;

	if (!endpoint || !bitmap || !out_num || !out_next_offset)
		return -EINVAL;
	if (!count || (count & 0x7) || (offset & 0x7))
		return -EINVAL;

	req_bitmap_len = count >> 3;
	if (req_bitmap_len > bitmap_len)
		return -EMSGSIZE;
	if (check_add_overflow((u32)offset, (u32)count, &window_end) ||
	    window_end > (u32)U16_MAX + 1U)
		return -EINVAL;

	memset(bitmap, 0, req_bitmap_len);

	mutex_lock(&vmsg_bus_endpoint_topologies_lock);
	topology = vmsg_bus_endpoint_topology_lookup_locked(endpoint);
	if (topology) {
		index = offset;
		while (index < window_end) {
			entry = xa_find(&topology->devices, &index,
					window_end - 1, XA_PRESENT);
			if (!entry)
				break;

			bitmap[(index - offset) >> 3] |= BIT((index - offset) & 0x7);
			index++;
		}

		if (window_end <= U16_MAX) {
			index = window_end;
			entry = xa_find(&topology->devices, &index, U16_MAX,
					XA_PRESENT);
			if (entry)
				next_offset = (u16)index;
		}
	}
	mutex_unlock(&vmsg_bus_endpoint_topologies_lock);

	*out_num = count;
	*out_next_offset = next_offset;
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_topology_get_devices_window);

int virtio_msg_bus_bridge_endpoint_cleanup_begin(u32 endpoint_id)
{
	void *state;
	int ret = 0;

	mutex_lock(&vmsg_bus_cleanup_busy_lock);
	state = xa_load(&vmsg_bus_cleanup_busy_endpoints, endpoint_id);
	if (!state) {
		ret = xa_err(xa_store(&vmsg_bus_cleanup_busy_endpoints,
				      endpoint_id, VMSG_BUS_CLEANUP_STATE_BUSY,
				      GFP_NOWAIT));
		goto out_unlock;
	}
	if (state == VMSG_BUS_CLEANUP_STATE_BUSY)
		goto out_unlock;

	ret = xa_err(xa_store(&vmsg_bus_cleanup_busy_endpoints, endpoint_id,
			      VMSG_BUS_CLEANUP_STATE_BUSY, GFP_NOWAIT));

out_unlock:
	mutex_unlock(&vmsg_bus_cleanup_busy_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_endpoint_cleanup_begin);

void virtio_msg_bus_bridge_endpoint_cleanup_end(u32 endpoint_id)
{
	mutex_lock(&vmsg_bus_cleanup_busy_lock);
	if (xa_load(&vmsg_bus_cleanup_busy_endpoints, endpoint_id))
		xa_store(&vmsg_bus_cleanup_busy_endpoints, endpoint_id,
			 VMSG_BUS_CLEANUP_STATE_IDLE, GFP_NOWAIT);
	mutex_unlock(&vmsg_bus_cleanup_busy_lock);
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_endpoint_cleanup_end);

static int
vmsg_bus_map_event_publish_add_locked(struct vmsg_bus_session *session,
				      u64 bus_addr, u64 length,
				      u64 mmap_offset, u64 mmap_length,
				      u32 flags, u64 *out_map_id)
{
	struct vmsg_bus_map_record *record;
	struct vmsg_bridge_uapi_map_event event = { 0 };
	u64 end;
	int ret;

	lockdep_assert_held(&session->map_lock);

	if (session->detached || !session->endpoint ||
	    READ_ONCE(session->endpoint->rx_unregistered))
		return -ENOTCONN;
	if (flags)
		return -EINVAL;
	if (!bus_addr || !length || !mmap_offset || !mmap_length)
		return -EINVAL;
	if (!PAGE_ALIGNED(bus_addr) || !PAGE_ALIGNED(length) ||
	    !PAGE_ALIGNED(mmap_offset) || !PAGE_ALIGNED(mmap_length))
		return -EINVAL;
	if (mmap_length < length)
		return -EINVAL;
	if (check_add_overflow(bus_addr, length, &end))
		return -EOVERFLOW;
	if (check_add_overflow(mmap_offset, mmap_length, &end))
		return -EOVERFLOW;

	record = kzalloc(sizeof(*record), GFP_KERNEL);
	if (!record)
		return -ENOMEM;

	ret = vmsg_bus_map_event_alloc_id_locked(session, &record->map_id);
	if (ret)
		goto err_free_record;

	record->epoch = session->map_event_epoch;
	record->bus_addr = bus_addr;
	record->length = length;
	record->mmap_offset = mmap_offset;
	record->mmap_length = mmap_length;
	record->flags = flags;
	record->state = VMSG_BUS_MAP_RECORD_MMAP_READY;

	ret = vmsg_bus_map_record_insert_locked(session, record);
	if (ret)
		goto err_free_record;

	ret = vmsg_bus_map_event_alloc_seq_locked(session, &event.seq);
	if (ret)
		goto err_remove_record;

	event.map_id = record->map_id;
	event.epoch = record->epoch;
	event.bus_addr = record->bus_addr;
	event.length = record->length;
	event.mmap_offset = record->mmap_offset;
	event.mmap_length = record->mmap_length;
	event.type = VMSG_BRIDGE_UAPI_MAP_EVENT_ADD;
	event.flags = record->flags;

	ret = vmsg_bus_map_event_enqueue_locked(session, &event);
	if (ret)
		goto err_remove_record;

	if (out_map_id)
		*out_map_id = record->map_id;

	return 0;

err_remove_record:
	vmsg_bus_map_record_remove_locked(session, record);
	return ret;
err_free_record:
	kfree(record);
	return ret;
}

static int
vmsg_bus_map_event_publish_del_req_locked(struct vmsg_bus_session *session, u64 map_id)
{
	struct vmsg_bus_map_record *record;
	struct vmsg_bridge_uapi_map_event event = { 0 };
	int ret;

	lockdep_assert_held(&session->map_lock);

	if (session->detached || !session->endpoint ||
	    READ_ONCE(session->endpoint->rx_unregistered))
		return -ENOTCONN;
	if (!map_id)
		return -EINVAL;

	record = vmsg_bus_map_record_lookup_by_id_locked(session, map_id);
	if (!record)
		return -ENOENT;
	if (record->state == VMSG_BUS_MAP_RECORD_DEL_REQ_PENDING ||
	    record->del_req_queued)
		return -EALREADY;
	if (record->state != VMSG_BUS_MAP_RECORD_MMAP_READY &&
	    record->state != VMSG_BUS_MAP_RECORD_ACTIVE)
		return -EBUSY;

	ret = vmsg_bus_map_event_alloc_seq_locked(session, &event.seq);
	if (ret)
		return ret;

	event.map_id = record->map_id;
	event.epoch = record->epoch;
	event.bus_addr = record->bus_addr;
	event.length = record->length;
	event.mmap_offset = record->mmap_offset;
	event.mmap_length = record->mmap_length;
	event.type = VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_REQ;
	event.flags = record->flags;

	ret = vmsg_bus_map_event_enqueue_locked(session, &event);
	if (ret)
		return ret;

	if (record->state == VMSG_BUS_MAP_RECORD_MMAP_READY)
		record->del_req_queued = true;
	else
		record->state = VMSG_BUS_MAP_RECORD_DEL_REQ_PENDING;
	return 0;
}

int virtio_msg_bus_bridge_device_map_event_add(u32 handle, u64 bus_addr,
					       u64 length, u64 mmap_offset,
					       u64 mmap_length, u32 flags,
					       u64 *map_id)
{
	struct vmsg_bus_handle_entry *entry;
	struct vmsg_bus_session *session;
	int ret;

	if (!handle)
		return -EINVAL;

	if (map_id)
		*map_id = 0;

	mutex_lock(&vmsg_bus_handles_lock);
	entry = xa_load(&vmsg_bus_handles, handle);
	if (!entry || !entry->session) {
		mutex_unlock(&vmsg_bus_handles_lock);
		return -ENODEV;
	}

	session = vmsg_bus_session_get(entry->session);
	mutex_unlock(&vmsg_bus_handles_lock);
	if (!session)
		return -ENODEV;

	mutex_lock(&session->map_lock);
	if (session->endpoint_id != handle || session->detached || !session->endpoint) {
		ret = -ENOTCONN;
		goto out_unlock_map;
	}

	ret = vmsg_bus_map_event_publish_add_locked(session, bus_addr, length,
						    mmap_offset, mmap_length,
						    flags, map_id);

out_unlock_map:
	mutex_unlock(&session->map_lock);
	if (!ret)
		vmsg_bus_session_exec_queue(session, VMSG_BUS_SESSION_ACT_WAKE);
	vmsg_bus_session_put(session);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_map_event_add);

int virtio_msg_bus_bridge_device_map_event_del_req(u32 handle, u64 map_id)
{
	struct vmsg_bus_handle_entry *entry;
	struct vmsg_bus_session *session;
	int ret;

	if (!handle)
		return -EINVAL;

	mutex_lock(&vmsg_bus_handles_lock);
	entry = xa_load(&vmsg_bus_handles, handle);
	if (!entry || !entry->session) {
		mutex_unlock(&vmsg_bus_handles_lock);
		return -ENODEV;
	}

	session = vmsg_bus_session_get(entry->session);
	mutex_unlock(&vmsg_bus_handles_lock);
	if (!session)
		return -ENODEV;

	mutex_lock(&session->map_lock);
	if (session->endpoint_id != handle || session->detached || !session->endpoint) {
		ret = -ENOTCONN;
		goto out_unlock_map;
	}

	ret = vmsg_bus_map_event_publish_del_req_locked(session, map_id);

out_unlock_map:
	mutex_unlock(&session->map_lock);
	if (!ret)
		vmsg_bus_session_exec_queue(session, VMSG_BUS_SESSION_ACT_WAKE);
	vmsg_bus_session_put(session);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_map_event_del_req);

int virtio_msg_bus_bridge_device_rx(u32 handle, const struct virtio_msg *msg,
				    const struct virtio_msg_dispatch_ctx *dctx)
{
	struct vmsg_bus_handle_entry *entry;
	struct virtio_msg_bus_bridge_device *endpoint;
	struct vmsg_bus_session *session = NULL;
	u16 msg_size;
	u32 dflags = 0;
	bool nonblock = false;
	bool record_relay = false;
	bool event_msg;
	u16 relay_dev_num = 0;
	u16 relay_token = 0;
	int ret;

	if (!handle || !msg)
		return -EINVAL;

	msg_size = le16_to_cpu(msg->msg_size);
	if (msg_size < sizeof(*msg) || msg_size > VIRTIO_MSG_MAX_SIZE)
		return -EMSGSIZE;

	if (dctx) {
		dflags = dctx->flags;
		if (dctx->reserved0 || dctx->reserved1)
			return -EINVAL;
		if (dflags & ~(VIRTIO_MSG_DISPATCH_F_NONBLOCK |
			       VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE))
			return -EINVAL;
		if ((dflags & VIRTIO_MSG_DISPATCH_F_NONBLOCK) &&
		    (dflags & VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE))
			return -EINVAL;
		if ((dflags & VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE) &&
		    !dctx->relay_seq)
			return -EINVAL;
		if (!(dflags & VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE) &&
		    dctx->relay_seq)
			return -EINVAL;
	}
	nonblock = dflags & VIRTIO_MSG_DISPATCH_F_NONBLOCK;
	record_relay = (dflags & VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE) &&
		       !(msg->type & VIRTIO_MSG_TYPE_RESPONSE) &&
		       !(msg->msg_id & VIRTIO_MSG_ID_EVENT_BIT);
	event_msg = msg->msg_id & VIRTIO_MSG_ID_EVENT_BIT;

	if (event_msg)
		vm_trace(NULL,
			 "rx event: handle=%u msg_id=0x%02x dev=%u type=0x%02x nonblock=%u\n",
			 handle, msg->msg_id, le16_to_cpu(msg->dev_num),
			 msg->type, nonblock);

	if (nonblock) {
		if (!mutex_trylock(&vmsg_bus_handles_lock))
			return -EAGAIN;
	} else {
		mutex_lock(&vmsg_bus_handles_lock);
	}
	entry = xa_load(&vmsg_bus_handles, handle);
	if (!entry || !entry->endpoint) {
		ret = -ENODEV;
		goto out_unlock;
	}
	endpoint = entry->endpoint;
	if (READ_ONCE(endpoint->rx_unregistered)) {
		ret = -ENODEV;
		goto out_unlock;
	}
	if (record_relay) {
		session = vmsg_bus_session_get(entry->session);
		if (!session) {
			ret = -ENODEV;
			goto out_unlock;
		}
	}
	ret = vmsg_bus_dispatch_validate_transport_target(endpoint, msg);
	if (ret)
		goto out_unlock;

	if (!endpoint->ops || !endpoint->ops->tx_msg) {
		ret = -EOPNOTSUPP;
		goto out_unlock;
	}
	atomic_inc(&endpoint->rx_inflight);

	mutex_unlock(&vmsg_bus_handles_lock);

	if (record_relay) {
		relay_dev_num = le16_to_cpu(msg->dev_num);
		relay_token = le16_to_cpu(msg->token);

		ret = vmsg_bus_relay_insert(session, relay_dev_num, relay_token,
					    msg->msg_id, dctx->relay_seq);
		if (ret) {
			if (atomic_dec_and_test(&endpoint->rx_inflight))
				wake_up_all(&endpoint->rx_waitq);
			vmsg_bus_session_put(session);
			return ret;
		}
	}

	ret = endpoint->ops->tx_msg(endpoint, msg, msg_size, dctx);
		if (ret) {
			if (event_msg)
				vm_trace(NULL,
					 "rx event dispatch failed: handle=%u msg_id=0x%02x ret=%d\n",
					 handle, msg->msg_id, ret);
			else
				vm_err_rl(NULL,
					  "rx dispatch failed: handle=%u msg_id=0x%02x ret=%d\n",
					  handle, msg->msg_id, ret);
		}
	if (record_relay && ret) {
		u64 tmp_seq;

		vmsg_bus_relay_take(session, relay_dev_num, relay_token, NULL,
				    &tmp_seq);
	}
	if (atomic_dec_and_test(&endpoint->rx_inflight))
		wake_up_all(&endpoint->rx_waitq);
	if (session)
		vmsg_bus_session_put(session);

	return ret;

out_unlock:
	mutex_unlock(&vmsg_bus_handles_lock);
	if (session)
		vmsg_bus_session_put(session);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_rx);

int virtio_msg_bus_bridge_device_report_error(u32 handle, u16 dev_num,
					      u16 token, u8 msg_id,
					      int error)
{
	struct vmsg_bus_handle_entry *entry;
	struct virtio_msg_bus_bridge_device *endpoint;
	struct vmsg_bus_relay_meta *meta;
	struct vmsg_bus_session *session;
	struct virtio_msg_dispatch_ctx dctx = {
		.flags = VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE,
	};
	unsigned long key;
	int ret;

	if (!handle || token <= VIRTIO_MSG_TOKEN_FIXED || error >= 0)
		return -EINVAL;
	if (msg_id & VIRTIO_MSG_ID_EVENT_BIT)
		return -EINVAL;

	vm_trace(NULL, "report_error: hdl=%u dev=%u tok=%u id=0x%02x err=%d\n",
		 handle, dev_num, token, msg_id, error);

	mutex_lock(&vmsg_bus_handles_lock);
	entry = xa_load(&vmsg_bus_handles, handle);
	if (!entry || !entry->session || !entry->endpoint) {
		mutex_unlock(&vmsg_bus_handles_lock);
		return -ENODEV;
	}

	session = vmsg_bus_session_get(entry->session);
	if (!session) {
		mutex_unlock(&vmsg_bus_handles_lock);
		return -ENODEV;
	}

	endpoint = entry->endpoint;
	mutex_unlock(&vmsg_bus_handles_lock);

	if (session->endpoint_id != handle || READ_ONCE(session->detached) ||
	    !READ_ONCE(session->endpoint)) {
		ret = -ENOTCONN;
		goto out_put;
	}
	if (!endpoint->ops || !endpoint->ops->report_error) {
		ret = -EOPNOTSUPP;
		goto out_put;
	}

	key = vmsg_bus_relay_key(dev_num, token);
	mutex_lock(&session->relay_lock);
	meta = xa_load(&session->relay_meta, key);
	if (!meta) {
		ret = -ENOENT;
		goto out_unlock_relay;
	}
	if (meta->msg_id != msg_id) {
		ret = -EPROTO;
		goto out_unlock_relay;
	}

	meta = xa_erase(&session->relay_meta, key);
	if (!meta) {
		ret = -ENOENT;
		goto out_unlock_relay;
	}

	dctx.relay_seq = meta->relay_seq;
	mutex_unlock(&session->relay_lock);

	ret = endpoint->ops->report_error(endpoint, dev_num, token, msg_id,
					  error, &dctx);
	if (ret) {
		int restore_ret;

		mutex_lock(&session->relay_lock);
		if (!xa_load(&session->relay_meta, key)) {
			restore_ret = xa_err(xa_store(&session->relay_meta, key,
						      meta, GFP_KERNEL));
			if (restore_ret)
				kfree(meta);
		} else {
			kfree(meta);
		}
		mutex_unlock(&session->relay_lock);
		goto out_put;
	}

	kfree(meta);
	goto out_put;

out_unlock_relay:
	mutex_unlock(&session->relay_lock);
out_put:
	vmsg_bus_session_put(session);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_report_error);

int virtio_msg_bus_bridge_device_publish_rx(u32 handle, const struct virtio_msg *msg)
{
	struct vmsg_bus_handle_entry *entry;
	struct vmsg_bus_session *session;
	u16 msg_size;
	int ret;

	if (!handle || !msg)
		return -EINVAL;

	msg_size = le16_to_cpu(msg->msg_size);
	if (msg_size < sizeof(*msg) || msg_size > VIRTIO_MSG_MAX_SIZE)
		return -EMSGSIZE;
	ret = vmsg_bus_validate_transport_msg_type(msg);
	if (ret)
		return ret;

	mutex_lock(&vmsg_bus_handles_lock);
	entry = xa_load(&vmsg_bus_handles, handle);
	if (!entry || !entry->session) {
		mutex_unlock(&vmsg_bus_handles_lock);
		return -ENODEV;
	}

	session = vmsg_bus_session_get(entry->session);
	mutex_unlock(&vmsg_bus_handles_lock);
	if (!session)
		return -ENODEV;

	mutex_lock(&session->map_lock);
	if (session->endpoint_id != handle || session->detached || !session->endpoint ||
	    READ_ONCE(session->endpoint->rx_unregistered)) {
		ret = -ENOTCONN;
		goto out_unlock_map;
	}
	ret = vmsg_bus_session_rx_publish_enqueue(session, msg, msg_size);

out_unlock_map:
	mutex_unlock(&session->map_lock);
	vmsg_bus_session_put(session);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_publish_rx);

int virtio_msg_bus_bridge_device_publish_rx_nonblock(struct virtio_msg_bus_bridge_device
						     *endpoint,
						     const struct virtio_msg *msg)
{
	struct vmsg_bus_session *session;
	u16 msg_size;
	int ret;

	if (!endpoint || !msg)
		return -EINVAL;

	msg_size = le16_to_cpu(msg->msg_size);
	if (msg_size < sizeof(*msg) || msg_size > VIRTIO_MSG_MAX_SIZE)
		return -EMSGSIZE;

	ret = vmsg_bus_validate_transport_msg_type(msg);
	if (ret)
		return ret;

	if (READ_ONCE(endpoint->rx_unregistered))
		return -ENOTCONN;

	session = vmsg_bus_endpoint_publish_session_get(endpoint);
	if (!session)
		return -ENOTCONN;

	if (READ_ONCE(session->detached) || READ_ONCE(session->endpoint) != endpoint ||
	    READ_ONCE(endpoint->rx_unregistered)) {
		ret = -ENOTCONN;
		goto out_put;
	}

	ret = vmsg_bus_session_rx_publish_enqueue_nonblock(session, msg, msg_size);

out_put:
	vmsg_bus_session_put(session);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_publish_rx_nonblock);

static int
vmsg_bus_attach_req_validate(struct vmsg_bridge_uapi_attach *req)
{
	u32 entry_size;
	size_t i;

	if (!req)
		return -EINVAL;
	if (!req->endpoint_id || req->flags)
		return -EINVAL;
	if (req->ring_fd < 0 || req->kick_fd < 0 || req->call_fd < 0)
		return -EINVAL;
	if (memchr_inv(&req->caps, 0, sizeof(req->caps)))
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(req->reserved1); i++) {
		if (req->reserved1[i])
			return -EINVAL;
	}
	if (vmsg_bus_ring_entry_size_calc(req->vmm_max_msg_size, &entry_size))
		return -EINVAL;

	return 0;
}

static int
vmsg_bus_session_caps_refresh(struct vmsg_bus_session *session)
{
	struct virtio_msg_bus_bridge_device_caps caps;
	struct virtio_msg_bus_bridge_device *endpoint;
	size_t name_len;
	int ret;

	if (!session)
		return -EINVAL;

	endpoint = READ_ONCE(session->endpoint);
	if (!endpoint || !endpoint->ops || !endpoint->ops->get_caps)
		return -ENODEV;

	memset(&caps, 0, sizeof(caps));
	ret = endpoint->ops->get_caps(endpoint, &caps);
	if (ret)
		return ret;
	if (!caps.name)
		return -EINVAL;

	name_len = strnlen(caps.name, VMSG_BRIDGE_UAPI_BUS_NAME_LEN);
	if (!name_len || name_len >= VMSG_BRIDGE_UAPI_BUS_NAME_LEN)
		return -EINVAL;
	if (!caps.msg_size || caps.msg_size > VIRTIO_MSG_MAX_SIZE)
		return -EINVAL;

	session->caps = caps;
	session->endpoint_max_msg_size = caps.msg_size;
	session->caps.msg_size = min_t(u32, session->attach_vmm_max_msg_size,
				       session->endpoint_max_msg_size);
	return 0;
}

static int
vmsg_bus_attach_fill_caps(struct vmsg_bus_session *session,
			  struct vmsg_bridge_uapi_attach *req)
{
	int ret = 0;

	vmsg_bus_session_caps_clear(session);
	memset(&req->caps, 0, sizeof(req->caps));
	req->caps.uapi_version = VMSG_BRIDGE_UAPI_VERSION;
	if (!READ_ONCE(session->endpoint))
		return 0;

	ret = vmsg_bus_session_caps_refresh(session);
	if (ret)
		return ret;

	req->caps.revision = session->caps.revision;
	req->caps.max_msg_size = session->caps.msg_size;
	req->caps.transport_features = session->caps.transport_features;
	req->caps.bridge_features = session->caps.bridge_features;
	req->caps.flags |= VMSG_BRIDGE_UAPI_CAP_F_ENDPOINT_ONLINE;

	return 0;
}

static void
vmsg_bus_publish_endpoint_online_control(struct vmsg_bus_session *session)
{
	struct vmsg_bridge_uapi_ctrl_endpoint_online endpoint_online;

	if (!session)
		return;

	memset(&endpoint_online, 0, sizeof(endpoint_online));
	endpoint_online.endpoint_id = session->endpoint_id;
	strscpy(endpoint_online.bus_name, session->caps.name,
		sizeof(endpoint_online.bus_name));
	endpoint_online.max_msg_size = session->endpoint_max_msg_size;
	vmsg_bus_control_enqueue(session,
				 VMSG_BRIDGE_UAPI_CTRL_TYPE_ENDPOINT_ONLINE,
				 &endpoint_online, sizeof(endpoint_online));
	vm_trace(NULL, "endpoint online published: endpoint_id=%u\n",
		 session->endpoint_id);
}

static void
vmsg_bus_publish_endpoint_offline_control(struct vmsg_bus_session *session,
					  u32 endpoint_offline_reason)
{
	struct vmsg_bridge_uapi_ctrl_endpoint_offline endpoint_offline;

	if (!session)
		return;

	memset(&endpoint_offline, 0, sizeof(endpoint_offline));
	endpoint_offline.endpoint_id = session->endpoint_id;
	endpoint_offline.reason = endpoint_offline_reason;
	vmsg_bus_control_enqueue(session,
				 VMSG_BRIDGE_UAPI_CTRL_TYPE_ENDPOINT_OFFLINE,
				 &endpoint_offline, sizeof(endpoint_offline));
	vm_trace(NULL, "endpoint offline published: endpoint_id=%u reason=%u\n",
		 session->endpoint_id, endpoint_offline_reason);
}

static void vmsg_bus_session_mark_offline(struct vmsg_bus_session *session,
					  u32 endpoint_offline_reason)
{
	struct virtio_msg_bus_bridge_device *endpoint = NULL;
	bool was_online = false;

	if (!session)
		return;

	mutex_lock(&session->map_lock);
	if (!session->detached && session->endpoint) {
		endpoint = session->endpoint;
		WRITE_ONCE(session->endpoint, NULL);
		vmsg_bus_map_event_purge_locked(session);
		vmsg_bus_map_records_purge_locked(session);
		was_online = true;
	}
	mutex_unlock(&session->map_lock);
	vmsg_bus_session_caps_clear(session);

	if (was_online && endpoint) {
		vm_info(NULL,
			"client offline: bus='%s' endpoint=%u reason=%u\n",
			session->caps.name, session->endpoint_id,
			endpoint_offline_reason);
		vmsg_bus_endpoint_publish_session_clear(endpoint, session);
		vmsg_bus_topology_clear_notify_removed(endpoint);
		vmsg_bus_publish_endpoint_offline_control(session,
							  endpoint_offline_reason);
	}
}

static void
vmsg_bus_session_rebind_online(struct vmsg_bus_session *session,
			       struct virtio_msg_bus_bridge_device *endpoint)
{
	struct vmsg_bus_file_ctx *ctx;
	bool rebind = false;
	int ret;

	if (!session || !endpoint)
		return;

	ctx = vmsg_bus_file_ctx_get(READ_ONCE(session->ctx));
	if (!ctx)
		return;

	mutex_lock(&ctx->lock);
	if (session->ctx != ctx || ctx->session != session)
		goto out_unlock_ctx;

	mutex_lock(&session->map_lock);
	if (!session->detached && !session->endpoint) {
		WRITE_ONCE(session->endpoint, endpoint);
		rebind = true;
	}
	mutex_unlock(&session->map_lock);
	if (!rebind)
		goto out_unlock_ctx;

	vmsg_bus_endpoint_publish_session_set(endpoint, session);

	ret = vmsg_bus_session_caps_refresh(session);
	if (ret) {
		vm_err_rl(NULL, "rebind caps refresh failed endpoint=%u ret=%d\n",
			  session->endpoint_id, ret);
		mutex_lock(&session->map_lock);
		if (!session->detached && session->endpoint == endpoint)
			WRITE_ONCE(session->endpoint, NULL);
		mutex_unlock(&session->map_lock);
		vmsg_bus_session_caps_clear(session);
		vmsg_bus_endpoint_publish_session_clear(endpoint, session);
		goto out_unlock_ctx;
	}

	vmsg_bus_publish_endpoint_online_control(session);
	vm_info(NULL, "client rebind online: bus='%s' endpoint=%u\n",
		session->caps.name, session->endpoint_id);

out_unlock_ctx:
	mutex_unlock(&ctx->lock);
	vmsg_bus_file_ctx_put(ctx);
}

static bool vmsg_bus_session_invalidate(struct vmsg_bus_session *session)
{
	struct virtio_msg_bus_bridge_device *endpoint = NULL;
	bool newly_detached = false;

	if (!session)
		return false;

	mutex_lock(&session->map_lock);
	if (!session->detached) {
		session->detached = true;
		vmsg_bus_map_event_purge_locked(session);
		vmsg_bus_map_records_purge_locked(session);
		newly_detached = true;
	}
	endpoint = READ_ONCE(session->endpoint);
	WRITE_ONCE(session->endpoint, NULL);
	mutex_unlock(&session->map_lock);
	vmsg_bus_session_caps_clear(session);

	vmsg_bus_endpoint_publish_session_clear(endpoint, session);
	vmsg_bus_topology_clear_notify_removed(endpoint);

	if (!newly_detached)
		return false;

	vmsg_bus_relay_purge_session(session);
	wake_up_interruptible_poll(&session->poll_waitq, EPOLLERR | EPOLLHUP);

	return true;
}

static void vmsg_bus_session_unlink_handle(struct vmsg_bus_session *session)
{
	struct vmsg_bus_handle_entry *entry;

	mutex_lock(&vmsg_bus_handles_lock);
	entry = xa_load(&vmsg_bus_handles, session->endpoint_id);
	if (entry && entry->session == session) {
		entry->session = NULL;
		vmsg_bus_session_put(session);
		vmsg_bus_release_handle_locked(session->endpoint_id);
	}
	mutex_unlock(&vmsg_bus_handles_lock);
}

static void vmsg_bus_session_drop_locked(struct vmsg_bus_file_ctx *ctx,
					 struct vmsg_bus_session *session)
{
	u32 endpoint_id;
	char bus_name[VMSG_BRIDGE_UAPI_BUS_NAME_LEN];

	if (!ctx || !session)
		return;
	endpoint_id = session->endpoint_id;
	strscpy(bus_name, session->caps.name, sizeof(bus_name));

	if (ctx->session == session)
		ctx->session = NULL;
	if (session->ctx == ctx) {
		vmsg_bus_file_ctx_put(session->ctx);
		session->ctx = NULL;
	}

	vmsg_bus_session_invalidate(session);

	vmsg_bus_session_exec_sync_disable(session);
	vmsg_bus_session_release_io(session);
	vmsg_bus_session_unlink_handle(session);
	vm_info(NULL, "client detached: bus='%s' endpoint=%u\n",
		bus_name, endpoint_id);
}

static long vmsg_bus_ioctl_attach(struct file *file, unsigned long arg)
{
	struct vmsg_bus_file_ctx *ctx = file->private_data;
	struct vmsg_bridge_uapi_attach req;
	struct vmsg_bus_handle_entry *entry;
	struct virtio_msg_bus_bridge_device *endpoint = NULL;
	struct vmsg_bus_session *session = NULL;
	struct vmsg_bus_session *copy_session = NULL;
	struct vmsg_bus_ring_state tx_state;
	struct vmsg_bus_ring_state rx_state;
	struct file *ring_file = NULL;
	struct eventfd_ctx *kick_evt = NULL;
	struct eventfd_ctx *call_evt = NULL;
	int ret;

	if (!ctx)
		return -EINVAL;
	if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
		return -EFAULT;

	ret = vmsg_bus_attach_req_validate(&req);
	if (ret)
		return ret;

	ring_file = fget(req.ring_fd);
	if (!ring_file)
		return -EBADF;
	if (!shmem_file(ring_file)) {
		ret = -EINVAL;
		goto out_put_ring;
	}
	if (!(ring_file->f_mode & FMODE_READ) ||
	    !(ring_file->f_mode & FMODE_WRITE)) {
		ret = -EINVAL;
		goto out_put_ring;
	}

	ret = vmsg_bus_ring_info_validate(ring_file, &req.tx,
					  req.vmm_max_msg_size, &tx_state);
	if (ret)
		goto out_put_ring;
	ret = vmsg_bus_ring_info_validate(ring_file, &req.rx,
					  req.vmm_max_msg_size, &rx_state);
	if (ret)
		goto out_put_ring;
	if (vmsg_bus_ring_ranges_overlap(&tx_state, &rx_state)) {
		ret = -EINVAL;
		goto out_put_ring;
	}

	kick_evt = eventfd_ctx_fdget(req.kick_fd);
	if (IS_ERR(kick_evt)) {
		ret = PTR_ERR(kick_evt);
		kick_evt = NULL;
		goto out_put_ring;
	}

	call_evt = eventfd_ctx_fdget(req.call_fd);
	if (IS_ERR(call_evt)) {
		ret = PTR_ERR(call_evt);
		call_evt = NULL;
		goto out_put_kick;
	}

	session = kzalloc(sizeof(*session), GFP_KERNEL);
	if (!session) {
		ret = -ENOMEM;
		goto out_put_call;
	}

	refcount_set(&session->refs, 1);
	mutex_init(&session->map_lock);
	mutex_init(&session->control_lock);
	INIT_LIST_HEAD(&session->map_event_queue);
	INIT_LIST_HEAD(&session->control_queue);
	xa_init(&session->map_records_by_id);
	xa_init(&session->map_records_by_offset);
	session->map_event_next_seq = 1;
	session->map_event_next_id = 1;
	session->map_event_epoch = 1;
	mutex_init(&session->relay_lock);
	xa_init(&session->relay_meta);
	spin_lock_init(&session->exec_lock);
	INIT_WORK(&session->exec_work, vmsg_bus_session_exec_workfn);
	INIT_DELAYED_WORK(&session->tx_retry_work,
			  vmsg_bus_session_tx_retry_workfn);
	INIT_LIST_HEAD(&session->rx_publish_queue);
	init_waitqueue_head(&session->poll_waitq);
	session->endpoint_id = req.endpoint_id;
	session->attach_vmm_max_msg_size = req.vmm_max_msg_size;
	session->ring_file = ring_file;
	session->kick_evt = kick_evt;
	session->call_evt = call_evt;
	session->tx = tx_state;
	session->rx = rx_state;

	ret = vmsg_bus_ring_state_map(ring_file, &session->tx);
	if (ret)
		goto out_put_session;
	ret = vmsg_bus_ring_state_map(ring_file, &session->rx);
	if (ret)
		goto out_put_session;

	ring_file = NULL;
	kick_evt = NULL;
	call_evt = NULL;

	ret = vmsg_bus_session_kick_hook_install(session, req.kick_fd);
	if (ret)
		goto out_put_session;

	mutex_lock(&ctx->lock);
	if (ctx->session) {
		ret = -EBUSY;
		goto out_drop_new_session;
	}

	mutex_lock(&vmsg_bus_handles_lock);
	entry = xa_load(&vmsg_bus_handles, req.endpoint_id);
	if (!entry) {
		ret = -ENODEV;
	} else if (entry->session) {
		ret = -EBUSY;
	} else if (!entry->resolve_refs) {
		ret = -ENOENT;
	} else if (entry->resolve_refs == U32_MAX) {
		ret = -EOVERFLOW;
	} else {
		session->ctx = vmsg_bus_file_ctx_get(ctx);
		if (!session->ctx) {
			ret = -ENODEV;
		} else {
			endpoint = entry->endpoint;
			session->endpoint = endpoint;
			entry->resolve_refs++;
			entry->session = session;
			vmsg_bus_session_get(session);
			ctx->session = session;
		}
	}
	mutex_unlock(&vmsg_bus_handles_lock);
	if (ret)
		goto out_unlock_ctx;
	if (endpoint)
		vmsg_bus_endpoint_publish_session_set(endpoint, session);

	ret = vmsg_bus_attach_fill_caps(session, &req);
	if (ret)
		goto out_drop_attached_session;

	copy_session = vmsg_bus_session_get(session);
	if (!copy_session) {
		ret = -ENOTCONN;
		goto out_drop_attached_session;
	}

	mutex_unlock(&ctx->lock);
	if (copy_to_user((void __user *)arg, &req, sizeof(req))) {
		bool dropped_attached_session = false;

		ret = -EFAULT;
		mutex_lock(&ctx->lock);
		if (ctx->session == session && session->ctx == ctx) {
			vmsg_bus_session_drop_locked(ctx, session);
			dropped_attached_session = true;
		}
		mutex_unlock(&ctx->lock);
		vmsg_bus_session_put(copy_session);
		if (dropped_attached_session)
			vmsg_bus_session_put(session);
		return ret;
	}

	vm_info(NULL,
		"client attached: bus='%s' endpoint=%u vmm_max_msg_size=%u\n",
		copy_session->caps.name, req.endpoint_id, req.vmm_max_msg_size);
	vmsg_bus_session_put(copy_session);
	return 0;

out_drop_attached_session:
	vmsg_bus_session_drop_locked(ctx, session);
out_unlock_ctx:
	mutex_unlock(&ctx->lock);
	vmsg_bus_session_put(session);
	return ret;

out_drop_new_session:
	mutex_unlock(&ctx->lock);
	vmsg_bus_session_put(session);
	return ret;

out_put_session:
	vmsg_bus_session_put(session);
	return ret;

out_put_call:
	eventfd_ctx_put(call_evt);
out_put_kick:
	eventfd_ctx_put(kick_evt);
out_put_ring:
	fput(ring_file);
	return ret;
}

static long vmsg_bus_ioctl_detach(struct file *file)
{
	struct vmsg_bus_file_ctx *ctx = file->private_data;
	struct vmsg_bus_session *session;
	u32 endpoint_id = 0;
	int ret;

	if (!ctx)
		return -EINVAL;

	mutex_lock(&ctx->lock);
	session = ctx->session;
	if (!session) {
		ret = -ENOTCONN;
		goto out_unlock_ctx;
	}

	ret = virtio_msg_bus_bridge_endpoint_cleanup_begin(session->endpoint_id);
	if (ret)
		goto out_unlock_ctx;
	endpoint_id = session->endpoint_id;

	vmsg_bus_session_drop_locked(ctx, session);
	virtio_msg_bus_bridge_endpoint_cleanup_end(session->endpoint_id);
	vmsg_bus_session_put(session);
	vm_info(NULL, "client detach requested: endpoint=%u\n",
		endpoint_id);
	ret = 0;

out_unlock_ctx:
	mutex_unlock(&ctx->lock);
	return ret;
}

static long vmsg_bus_ioctl_control_recv(struct file *file, unsigned long arg)
{
	struct vmsg_bus_file_ctx *ctx = file->private_data;
	struct vmsg_bus_session *session;
	struct vmsg_bus_session *copy_session = NULL;
	struct vmsg_bus_control_entry *entry;
	struct vmsg_bus_control_entry *candidate = NULL;
	struct vmsg_bridge_uapi_control_msg msg;
	u64 seq;
	int ret;

	if (!ctx)
		return -EINVAL;

	mutex_lock(&ctx->lock);
	session = ctx->session;
	if (!session) {
		ret = -ENOTCONN;
		goto out_unlock_ctx;
	}

	mutex_lock(&session->control_lock);
	candidate = list_first_entry_or_null(&session->control_queue,
					     struct vmsg_bus_control_entry,
					     node);
	if (!candidate) {
		ret = -EAGAIN;
		goto out_unlock_control;
	}

	msg = candidate->msg;
	seq = candidate->msg.seq;
	copy_session = vmsg_bus_session_get(session);
	if (!copy_session) {
		ret = -ENOTCONN;
		goto out_unlock_control;
	}
	mutex_unlock(&session->control_lock);
	mutex_unlock(&ctx->lock);

	if (copy_to_user((void __user *)arg, &msg, sizeof(msg))) {
		ret = -EFAULT;
		goto out_put_session;
	}

	mutex_lock(&ctx->lock);
	if (ctx->session != session || session->ctx != ctx) {
		ret = -EAGAIN;
		goto out_unlock_ctx;
	}

	mutex_lock(&session->control_lock);
	entry = list_first_entry_or_null(&session->control_queue,
					 struct vmsg_bus_control_entry,
					 node);
	if (!entry || entry->msg.seq != seq) {
		ret = -EAGAIN;
		goto out_unlock_control;
	}
	list_del(&entry->node);
	kfree(entry);
	ret = 0;

out_unlock_control:
	mutex_unlock(&session->control_lock);
out_unlock_ctx:
	mutex_unlock(&ctx->lock);
out_put_session:
	vmsg_bus_session_put(copy_session);
	return ret;
}

static int
vmsg_bus_ioctl_msg_error_validate(const struct vmsg_bridge_uapi_msg_error *req)
{
	if (!req)
		return -EINVAL;
	if (!req->msg_id || req->flags || req->error >= 0)
		return -EINVAL;
	if (req->reserved0[0] || req->reserved0[1] || req->reserved0[2])
		return -EINVAL;
	if (req->reserved1[0] || req->reserved1[1])
		return -EINVAL;
	if (req->token <= VIRTIO_MSG_TOKEN_FIXED)
		return -EINVAL;
	if (req->msg_id & VIRTIO_MSG_ID_EVENT_BIT)
		return -EINVAL;

	return 0;
}

static long vmsg_bus_ioctl_msg_error(struct file *file, unsigned long arg)
{
	struct vmsg_bus_file_ctx *ctx = file->private_data;
	struct vmsg_bridge_uapi_msg_error req;
	struct vmsg_bus_session *session;
	int ret;

	if (!ctx)
		return -EINVAL;
	if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
		return -EFAULT;

	ret = vmsg_bus_ioctl_msg_error_validate(&req);
	if (ret)
		return ret;

	mutex_lock(&ctx->lock);
	session = ctx->session;
	if (!session || session->detached || !session->endpoint) {
		ret = -ENOTCONN;
		goto out_unlock_ctx;
	}

	ret = virtio_msg_bus_bridge_device_report_error
		(session->endpoint_id, req.dev_num, req.token, req.msg_id,
		 req.error);

out_unlock_ctx:
	mutex_unlock(&ctx->lock);
	return ret;
}

static int
vmsg_bus_validate_map_event_recv_request(const struct vmsg_bridge_uapi_map_event_recv *recv)
{
	const struct vmsg_bridge_uapi_map_event *event;
	size_t i;

	if (!recv)
		return -EINVAL;
	if (recv->flags || recv->reserved0)
		return -EINVAL;

	event = &recv->event;
	if (event->flags || event->reserved0)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(event->reserved1); i++) {
		if (event->reserved1[i])
			return -EINVAL;
	}
	for (i = 0; i < ARRAY_SIZE(recv->reserved1); i++) {
		if (recv->reserved1[i])
			return -EINVAL;
	}
	if (event->type)
		return -EINVAL;
	if (event->seq || event->map_id || event->epoch ||
	    event->bus_addr || event->length ||
	    event->mmap_offset || event->mmap_length)
		return -EINVAL;

	return 0;
}

static int
vmsg_bus_validate_map_event_ack_request(const struct vmsg_bridge_uapi_map_event_ack *ack)
{
	size_t i;

	if (!ack || !ack->seq)
		return -EINVAL;
	if (ack->type < VMSG_BRIDGE_UAPI_MAP_EVENT_ADD ||
	    ack->type > VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_REQ)
		return -EINVAL;
	if (ack->status < VMSG_BRIDGE_UAPI_MAP_ACK_OK ||
	    ack->status > VMSG_BRIDGE_UAPI_MAP_ACK_REJECT)
		return -EINVAL;
	if (ack->flags)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(ack->reserved1); i++) {
		if (ack->reserved1[i])
			return -EINVAL;
	}

	return 0;
}

static int
vmsg_bus_map_event_ack_notify(struct vmsg_bus_session *session,
			      const struct vmsg_bridge_uapi_map_event *event,
			      u32 ack_status)
{
	struct virtio_msg_bus_bridge_device *endpoint;

	if (!event)
		return -EINVAL;
	if (event->type != VMSG_BRIDGE_UAPI_MAP_EVENT_ADD &&
	    event->type != VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_REQ)
		return 0;

	endpoint = session->endpoint;
	if (!endpoint || !endpoint->ops || !endpoint->ops->map_event_ack)
		return 0;

	return endpoint->ops->map_event_ack(endpoint, event, ack_status);
}

static long vmsg_bus_ioctl_map_event_recv(struct file *file, unsigned long arg)
{
	struct vmsg_bus_file_ctx *ctx = file->private_data;
	struct vmsg_bus_session *session;
	struct vmsg_bus_session *copy_session = NULL;
	struct vmsg_bus_map_event_entry *entry;
	struct vmsg_bridge_uapi_map_event_recv recv = { 0 };
	u64 seq;
	int ret;

	if (!ctx)
		return -EINVAL;
	if (copy_from_user(&recv, (void __user *)arg, sizeof(recv)))
		return -EFAULT;

	ret = vmsg_bus_validate_map_event_recv_request(&recv);
	if (ret)
		return ret;

	mutex_lock(&ctx->lock);
	session = ctx->session;
	if (!session) {
		ret = -ENOTCONN;
		goto out_unlock_ctx;
	}

	mutex_lock(&session->map_lock);
	if (session->detached) {
		ret = -ENOTCONN;
		goto out_unlock_map;
	}
	if (!session->endpoint) {
		ret = -ENODEV;
		goto out_unlock_map;
	}

	entry = vmsg_bus_map_event_head_locked(session);
	if (!entry || entry->delivered ||
	    !time_after_eq(jiffies, entry->deliver_after_jiffies)) {
		ret = -EAGAIN;
		goto out_unlock_map;
	}

	recv.event = entry->event;
	seq = entry->event.seq;
	copy_session = vmsg_bus_session_get(session);
	if (!copy_session) {
		ret = -ENOTCONN;
		goto out_unlock_map;
	}
	mutex_unlock(&session->map_lock);
	mutex_unlock(&ctx->lock);

	if (copy_to_user((void __user *)arg, &recv, sizeof(recv))) {
		ret = -EFAULT;
		goto out_put_session;
	}

	mutex_lock(&ctx->lock);
	if (ctx->session != session || session->ctx != ctx) {
		ret = -EAGAIN;
		goto out_unlock_ctx;
	}

	mutex_lock(&session->map_lock);
	if (session->detached) {
		ret = -ENOTCONN;
		goto out_unlock_map;
	}
	if (!session->endpoint) {
		ret = -ENODEV;
		goto out_unlock_map;
	}

	entry = vmsg_bus_map_event_head_locked(session);
	if (!entry || entry->delivered || entry->event.seq != seq) {
		ret = -EAGAIN;
		goto out_unlock_map;
	}

	entry->delivered = true;
	ret = 0;

out_unlock_map:
	mutex_unlock(&session->map_lock);
out_unlock_ctx:
	mutex_unlock(&ctx->lock);
out_put_session:
	vmsg_bus_session_put(copy_session);
	return ret;
}

static long vmsg_bus_ioctl_map_event_ack(struct file *file, unsigned long arg)
{
	struct vmsg_bus_file_ctx *ctx = file->private_data;
	struct vmsg_bus_session *session;
	struct vmsg_bus_map_event_entry *entry;
	struct vmsg_bus_map_record *record;
	struct vmsg_bridge_uapi_map_event_ack ack;
	struct vmsg_bridge_uapi_map_event event;
	bool trigger_detach = false;
	bool notify_endpoint;
	int ret;

	if (!ctx)
		return -EINVAL;
	if (copy_from_user(&ack, (void __user *)arg, sizeof(ack)))
		return -EFAULT;

	ret = vmsg_bus_validate_map_event_ack_request(&ack);
	if (ret)
		return ret;

	mutex_lock(&ctx->lock);
	session = ctx->session;
	if (!session) {
		ret = -ENOTCONN;
		goto out_unlock_ctx;
	}

	mutex_lock(&session->map_lock);
	if (session->detached) {
		ret = -ENOTCONN;
		goto out_unlock_map;
	}
	if (!session->endpoint) {
		ret = -ENODEV;
		goto out_unlock_map;
	}

	entry = vmsg_bus_map_event_head_locked(session);
	if (!entry || entry->event.seq != ack.seq) {
		ret = -ENOENT;
		goto out_unlock_map;
	}
	if (!entry->delivered) {
		ret = -EPROTO;
		goto out_unlock_map;
	}
	if (entry->event.type != ack.type) {
		ret = -EINVAL;
		goto out_unlock_map;
	}

	event = entry->event;
	notify_endpoint = event.type == VMSG_BRIDGE_UAPI_MAP_EVENT_ADD ||
			  event.type == VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_REQ;
	mutex_unlock(&session->map_lock);

	if (notify_endpoint) {
		ret = vmsg_bus_map_event_ack_notify(session, &event, ack.status);
		if (ret)
			goto out_unlock_ctx;
	}

	mutex_lock(&session->map_lock);
	if (session->detached) {
		ret = -ENOTCONN;
		goto out_unlock_map;
	}
	if (!session->endpoint) {
		ret = -ENODEV;
		goto out_unlock_map;
	}

	entry = vmsg_bus_map_event_head_locked(session);
	if (!entry || entry->event.seq != ack.seq) {
		ret = -ENOENT;
		goto out_unlock_map;
	}
	if (!entry->delivered) {
		ret = -EPROTO;
		goto out_unlock_map;
	}
	if (entry->event.type != ack.type) {
		ret = -EINVAL;
		goto out_unlock_map;
	}

	switch (ack.status) {
	case VMSG_BRIDGE_UAPI_MAP_ACK_OK:
		if (entry->event.type == VMSG_BRIDGE_UAPI_MAP_EVENT_ADD) {
			record = vmsg_bus_map_record_lookup_by_id_locked(session,
									 entry->event.map_id);
			if (!record) {
				ret = -ENOENT;
				break;
			}
			if (record->state == VMSG_BUS_MAP_RECORD_MMAP_READY) {
				if (record->del_req_queued) {
					record->del_req_queued = false;
					record->state = VMSG_BUS_MAP_RECORD_DEL_REQ_PENDING;
				} else {
					record->state = VMSG_BUS_MAP_RECORD_ACTIVE;
				}
			} else if (record->state !=
				   VMSG_BUS_MAP_RECORD_DEL_REQ_PENDING) {
				ret = -ENOENT;
				break;
			}
		} else if (entry->event.type == VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_REQ) {
			record = vmsg_bus_map_record_lookup_by_id_locked(session,
									 entry->event.map_id);
			if (!record ||
			    record->state != VMSG_BUS_MAP_RECORD_DEL_REQ_PENDING) {
				ret = -ENOENT;
				break;
			}
			vmsg_bus_map_record_remove_locked(session, record);
		}

		list_del(&entry->node);
		if (session->map_event_queue_len)
			session->map_event_queue_len--;
		kfree(entry);
		ret = 0;
		break;
	case VMSG_BRIDGE_UAPI_MAP_ACK_RETRY:
		entry->delivered = false;
		entry->deliver_after_jiffies =
			jiffies + msecs_to_jiffies(VMSG_BUS_MAP_EVENT_RETRY_DELAY_MS);
		ret = 0;
		break;
	case VMSG_BRIDGE_UAPI_MAP_ACK_REJECT:
		list_del(&entry->node);
		if (session->map_event_queue_len)
			session->map_event_queue_len--;
		kfree(entry);
		trigger_detach = true;
		ret = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

out_unlock_map:
	mutex_unlock(&session->map_lock);
	if (!ret && trigger_detach) {
		ret = virtio_msg_bus_bridge_endpoint_cleanup_begin(session->endpoint_id);
		if (!ret) {
			vmsg_bus_session_drop_locked(ctx, session);
			virtio_msg_bus_bridge_endpoint_cleanup_end(session->endpoint_id);
			vmsg_bus_session_put(session);
		}
	}
out_unlock_ctx:
	mutex_unlock(&ctx->lock);
	return ret;
}

static long vmsg_bus_ioctl_resolve_endpoint(struct file *file, unsigned long arg)
{
	struct vmsg_bridge_uapi_resolve_endpoint req;
	bool create;
	u32 handle;
	int ret;

	(void)file;
	if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
		return -EFAULT;
	if (req.reserved[0] || req.reserved[1])
		return -EINVAL;
	if (req.flags & ~VMSG_BRIDGE_UAPI_RESOLVE_F_CREATE)
		return -EINVAL;

	ret = vmsg_bus_endpoint_addr_validate_uapi(&req.addr);
	if (ret)
		return ret;

	ret = vmsg_bus_resolver_validate_addr(&req.addr, true);
	if (ret)
		return ret;

	create = !!(req.flags & VMSG_BRIDGE_UAPI_RESOLVE_F_CREATE);
	mutex_lock(&vmsg_bus_handles_lock);
	ret = vmsg_bus_resolve_addr_locked(&req.addr, create, &handle);
	mutex_unlock(&vmsg_bus_handles_lock);
	if (ret)
		return ret;

	req.handle = handle;
	if (copy_to_user((void __user *)arg, &req, sizeof(req))) {
		mutex_lock(&vmsg_bus_handles_lock);
		vmsg_bus_release_handle_locked(handle);
		mutex_unlock(&vmsg_bus_handles_lock);
		return -EFAULT;
	}

	return 0;
}

static long vmsg_bus_ioctl_release_endpoint(struct file *file, unsigned long arg)
{
	struct vmsg_bridge_uapi_release_endpoint req;

	(void)file;
	if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
		return -EFAULT;
	if (!req.handle || req.flags || req.reserved)
		return -EINVAL;

	mutex_lock(&vmsg_bus_handles_lock);
	vmsg_bus_release_handle_locked(req.handle);
	mutex_unlock(&vmsg_bus_handles_lock);

	return 0;
}

static int
vmsg_bus_ioctl_topology_ctx_get(struct vmsg_bus_file_ctx *ctx, u32 endpoint_id,
				struct virtio_msg_bus_bridge_device **out_endpoint)
{
	struct vmsg_bus_session *session;
	struct virtio_msg_bus_bridge_device *endpoint;

	if (!ctx || !out_endpoint)
		return -EINVAL;

	session = ctx->session;
	if (!session || session->detached)
		return -ENOTCONN;
	if (!endpoint_id || endpoint_id != session->endpoint_id)
		return -EPERM;

	endpoint = READ_ONCE(session->endpoint);
	if (!endpoint || READ_ONCE(endpoint->rx_unregistered))
		endpoint = NULL;

	*out_endpoint = endpoint;
	return 0;
}

static int
vmsg_bus_ioctl_set_devices_validate(const struct vmsg_bridge_uapi_set_devices *req,
				    u32 *out_bits)
{
	u32 bits;
	u32 bytes;

	if (!req || !out_bits)
		return -EINVAL;
	if (!req->endpoint_id || req->reserved0)
		return -EINVAL;
	if (req->flags & ~VMSG_BRIDGE_UAPI_SET_DEVICES_F_CLEAR)
		return -EINVAL;
	if (req->first_dev_num > req->last_dev_num)
		return -EINVAL;

	bits = (u32)req->last_dev_num - (u32)req->first_dev_num + 1U;
	bytes = DIV_ROUND_UP(bits, 8U);
	if (bytes > VMSG_BRIDGE_UAPI_SET_DEVICES_BITMAP_MAX)
		return -EINVAL;
	if (req->bitmap_bytes != bytes || !req->bitmap_ptr)
		return -EINVAL;

	*out_bits = bits;
	return 0;
}

static bool vmsg_bus_bitmap_test(const u8 *bitmap, u32 bit)
{
	return bitmap[bit >> 3] & BIT(bit & 0x7);
}

static void vmsg_bus_bitmap_set(u8 *bitmap, u32 bit)
{
	bitmap[bit >> 3] |= BIT(bit & 0x7);
}

static void
vmsg_bus_topology_snapshot_locked(struct vmsg_bus_endpoint_topology *topology,
				  u8 *bitmap, size_t bitmap_len)
{
	void *entry;
	unsigned long index = 0;

	lockdep_assert_held(&vmsg_bus_endpoint_topologies_lock);

	if (!topology || !bitmap)
		return;

	memset(bitmap, 0, bitmap_len);
	while (index <= U16_MAX) {
		entry = xa_find(&topology->devices, &index, U16_MAX, XA_PRESENT);
		if (!entry)
			break;

		vmsg_bus_bitmap_set(bitmap, (u32)index);
		index++;
	}
}

static void
vmsg_bus_topology_notify_device_event(struct virtio_msg_bus_bridge_device *endpoint,
				      u16 dev_num, u16 dev_state)
{
	int ret;

	if (!endpoint || !endpoint->ops || !endpoint->ops->notify_device_event)
		return;

	ret = endpoint->ops->notify_device_event(endpoint, dev_num, dev_state);
	if (ret)
		vm_warn_rl(NULL,
			   "virtio_msg bridge topology notify failed handle=%u dev_num=%u state=%u ret=%d\n",
			   endpoint->handle, dev_num, dev_state, ret);
}

static void
vmsg_bus_topology_notify_bitmap_diff(struct virtio_msg_bus_bridge_device *endpoint,
				     const u8 *old_bitmap,
				     const u8 *new_bitmap, size_t bitmap_len)
{
	u32 bit;

	if (!old_bitmap || !new_bitmap)
		return;

	for (bit = 0; bit < bitmap_len * 8U; bit++) {
		bool old_present = vmsg_bus_bitmap_test(old_bitmap, bit);
		bool new_present = vmsg_bus_bitmap_test(new_bitmap, bit);

		if (old_present == new_present)
			continue;

		vmsg_bus_topology_notify_device_event
			(endpoint, (u16)bit,
			 new_present ? VIRTIO_MSG_BUS_EVENT_DEV_STATE_READY :
				       VIRTIO_MSG_BUS_EVENT_DEV_STATE_REMOVED);
	}
}

static void
vmsg_bus_topology_clear_notify_removed(struct virtio_msg_bus_bridge_device *endpoint)
{
	struct vmsg_bus_endpoint_topology *topology;
	u8 *old_bitmap;
	u8 *new_bitmap;
	u16 removed_devs[VMSG_BUS_TOPOLOGY_REMOVED_BATCH];
	bool clear_endpoint_topology;
	bool notify = false;
	u32 removed;
	u32 i;

	if (!endpoint)
		return;

	clear_endpoint_topology = READ_ONCE(endpoint->rx_unregistered);
	old_bitmap = kcalloc(VMSG_BRIDGE_UAPI_SET_DEVICES_BITMAP_MAX,
			     sizeof(*old_bitmap), GFP_KERNEL);
	new_bitmap = kcalloc(VMSG_BRIDGE_UAPI_SET_DEVICES_BITMAP_MAX,
			     sizeof(*new_bitmap), GFP_KERNEL);
	if (!old_bitmap || !new_bitmap)
		goto fallback_no_bitmap;

	mutex_lock(&vmsg_bus_endpoint_topologies_lock);
	topology = vmsg_bus_endpoint_topology_lookup_locked(endpoint);
	if (topology) {
		vmsg_bus_topology_snapshot_locked
			(topology, old_bitmap,
			 VMSG_BRIDGE_UAPI_SET_DEVICES_BITMAP_MAX);
		vmsg_bus_bridge_topology_clear_locked(topology);
		notify = true;
	}
	if (clear_endpoint_topology)
		WRITE_ONCE(endpoint->topology, NULL);
	mutex_unlock(&vmsg_bus_endpoint_topologies_lock);

	if (notify)
		vmsg_bus_topology_notify_bitmap_diff
			(endpoint, old_bitmap, new_bitmap,
			 VMSG_BRIDGE_UAPI_SET_DEVICES_BITMAP_MAX);
	goto out_free;

fallback_no_bitmap:
	/*
	 * Keep teardown deterministic under memory pressure: remove in lock-
	 * protected batches, then emit REMOVED callbacks after dropping the lock.
	 */
	for (;;) {
		unsigned long index = 0;
		void *entry;

		removed = 0;
		mutex_lock(&vmsg_bus_endpoint_topologies_lock);
		topology = vmsg_bus_endpoint_topology_lookup_locked(endpoint);
		while (topology && removed < ARRAY_SIZE(removed_devs)) {
			entry = xa_find(&topology->devices, &index, U16_MAX,
					XA_PRESENT);
			if (!entry)
				break;

			removed_devs[removed++] = (u16)index;
			vmsg_bus_bridge_topology_remove_locked(topology,
							       (u16)index);
			index++;
		}
		if (clear_endpoint_topology)
			WRITE_ONCE(endpoint->topology, NULL);
		mutex_unlock(&vmsg_bus_endpoint_topologies_lock);

		if (!removed)
			break;

		for (i = 0; i < removed; i++)
			vmsg_bus_topology_notify_device_event
				(endpoint, removed_devs[i],
				 VIRTIO_MSG_BUS_EVENT_DEV_STATE_REMOVED);
	}

out_free:
	kfree(new_bitmap);
	kfree(old_bitmap);
}

static int
vmsg_bus_topology_apply_bitmap_online(struct virtio_msg_bus_bridge_device *endpoint,
				      u16 first_dev_num, u32 bits, u32 flags,
				      const u8 *bitmap)
{
	u8 *new_bitmap;
	u8 *old_bitmap;
	struct vmsg_bus_endpoint_topology *topology;
	u32 bit;
	int ret = 0;

	old_bitmap = kcalloc(VMSG_BRIDGE_UAPI_SET_DEVICES_BITMAP_MAX,
			     sizeof(*old_bitmap), GFP_KERNEL);
	new_bitmap = kcalloc(VMSG_BRIDGE_UAPI_SET_DEVICES_BITMAP_MAX,
			     sizeof(*new_bitmap), GFP_KERNEL);
	if (!old_bitmap || !new_bitmap) {
		ret = -ENOMEM;
		goto out_free_notify;
	}

	mutex_lock(&vmsg_bus_endpoint_topologies_lock);
	topology = vmsg_bus_endpoint_topology_get_or_create_locked(endpoint, &ret);
	if (!topology)
		goto out_unlock;

	vmsg_bus_topology_snapshot_locked(topology, old_bitmap,
					  VMSG_BRIDGE_UAPI_SET_DEVICES_BITMAP_MAX);
	if (flags & VMSG_BRIDGE_UAPI_SET_DEVICES_F_CLEAR)
		vmsg_bus_bridge_topology_clear_locked(topology);

	for (bit = 0; bit < bits; bit++) {
		u16 dev_num = (u16)((u32)first_dev_num + bit);

		if (vmsg_bus_bitmap_test(bitmap, bit)) {
			ret = vmsg_bus_bridge_topology_add_locked(topology,
								  dev_num);
			if (ret == -EEXIST)
				ret = 0;
		} else {
			ret = vmsg_bus_bridge_topology_remove_locked(topology,
								     dev_num);
			if (ret == -ENOENT)
				ret = 0;
		}
		if (ret)
			break;
	}
	if (!ret)
		vmsg_bus_topology_snapshot_locked(topology, new_bitmap,
						  VMSG_BRIDGE_UAPI_SET_DEVICES_BITMAP_MAX);

out_unlock:
	mutex_unlock(&vmsg_bus_endpoint_topologies_lock);
	if (!ret)
		vmsg_bus_topology_notify_bitmap_diff
			(endpoint, old_bitmap, new_bitmap,
			 VMSG_BRIDGE_UAPI_SET_DEVICES_BITMAP_MAX);

out_free_notify:
	kfree(new_bitmap);
	kfree(old_bitmap);
	return ret;
}

static long vmsg_bus_ioctl_set_devices(struct file *file, unsigned long arg)
{
	struct vmsg_bus_file_ctx *ctx = file->private_data;
	struct vmsg_bridge_uapi_set_devices req;
	struct virtio_msg_bus_bridge_device *endpoint;
	u8 *bitmap;
	u32 bits;
	int ret;

	if (!ctx)
		return -EINVAL;
	if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
		return -EFAULT;

	ret = vmsg_bus_ioctl_set_devices_validate(&req, &bits);
	if (ret)
		return ret;

	bitmap = memdup_user(u64_to_user_ptr(req.bitmap_ptr), req.bitmap_bytes);
	if (IS_ERR(bitmap))
		return PTR_ERR(bitmap);

	mutex_lock(&ctx->lock);
	ret = vmsg_bus_ioctl_topology_ctx_get(ctx, req.endpoint_id, &endpoint);
	if (ret)
		goto out_unlock_ctx;

	if (!endpoint) {
		ret = -ENODEV;
		goto out_unlock_ctx;
	}

	ret = vmsg_bus_topology_apply_bitmap_online(endpoint,
						    req.first_dev_num, bits,
						    req.flags, bitmap);

out_unlock_ctx:
	mutex_unlock(&ctx->lock);
	kfree(bitmap);
	return ret;
}

static long vmsg_bus_unlocked_ioctl(struct file *file, unsigned int cmd,
				    unsigned long arg)
{
	switch (cmd) {
	case VMSG_BRIDGE_IOCTL_ATTACH:
		return vmsg_bus_ioctl_attach(file, arg);
	case VMSG_BRIDGE_IOCTL_DETACH:
		return vmsg_bus_ioctl_detach(file);
	case VMSG_BRIDGE_IOCTL_MAP_EVENT_RECV:
		return vmsg_bus_ioctl_map_event_recv(file, arg);
	case VMSG_BRIDGE_IOCTL_MAP_EVENT_ACK:
		return vmsg_bus_ioctl_map_event_ack(file, arg);
	case VMSG_BRIDGE_IOCTL_CONTROL_RECV:
		return vmsg_bus_ioctl_control_recv(file, arg);
	case VMSG_BRIDGE_IOCTL_MSG_ERROR:
		return vmsg_bus_ioctl_msg_error(file, arg);
	case VMSG_BRIDGE_IOCTL_RESOLVE_ENDPOINT:
		return vmsg_bus_ioctl_resolve_endpoint(file, arg);
	case VMSG_BRIDGE_IOCTL_RELEASE_ENDPOINT:
		return vmsg_bus_ioctl_release_endpoint(file, arg);
	case VMSG_BRIDGE_IOCTL_SET_DEVICES:
		return vmsg_bus_ioctl_set_devices(file, arg);
	default:
		return -ENOTTY;
	}
}

static bool vmsg_bus_control_queue_readable_locked(struct vmsg_bus_session *session)
{
	lockdep_assert_held(&session->control_lock);

	return !list_empty(&session->control_queue);
}

static bool vmsg_bus_map_event_queue_readable_locked(struct vmsg_bus_session *session)
{
	struct vmsg_bus_map_event_entry *entry;

	lockdep_assert_held(&session->map_lock);

	entry = vmsg_bus_map_event_head_locked(session);
	if (!entry || entry->delivered)
		return false;

	return !time_before(jiffies, entry->deliver_after_jiffies);
}

static int vmsg_bus_rx_ring_has_unread(struct vmsg_bus_session *session, bool *has_unread)
{
	struct vmsg_bridge_uapi_ring_hdr hdr;
	u32 used;
	int ret;

	if (!session || !has_unread)
		return -EINVAL;

	ret = vmsg_bus_ring_hdr_read(&session->rx, &hdr);
	if (ret)
		return ret;

	ret = vmsg_bus_ring_occupancy(&hdr, &used);
	if (ret)
		return ret;

	*has_unread = used != 0;
	return 0;
}

static __poll_t vmsg_bus_poll(struct file *file, poll_table *wait)
{
	struct vmsg_bus_file_ctx *ctx = file->private_data;
	struct vmsg_bus_session *session;
	bool control_ready = false;
	bool map_ready = false;
	bool rx_ready = false;
	__poll_t mask = 0;
	int ret;

	if (!ctx)
		return EPOLLERR | EPOLLHUP;

	mutex_lock(&ctx->lock);
	session = ctx->session;
	if (!session) {
		mask = EPOLLERR | EPOLLHUP;
		goto out_unlock_ctx;
	}

	poll_wait(file, &session->poll_waitq, wait);
	if (READ_ONCE(session->detached)) {
		mask = EPOLLERR | EPOLLHUP;
		goto out_unlock_ctx;
	}

	mutex_lock(&session->control_lock);
	control_ready = vmsg_bus_control_queue_readable_locked(session);
	mutex_unlock(&session->control_lock);

	mutex_lock(&session->map_lock);
	if (session->detached) {
		mask = EPOLLERR | EPOLLHUP;
		mutex_unlock(&session->map_lock);
		goto out_unlock_ctx;
	}
	map_ready = vmsg_bus_map_event_queue_readable_locked(session);
	mutex_unlock(&session->map_lock);

	ret = vmsg_bus_rx_ring_has_unread(session, &rx_ready);
	if (ret)
		vmsg_bus_session_mark_ring_fault(session);

	if (READ_ONCE(session->ring_fault))
		mask |= EPOLLERR;
	if (control_ready || map_ready || rx_ready)
		mask |= EPOLLIN | EPOLLRDNORM;

	if (vmsg_bus_rx_publish_queue_pending(session))
		vmsg_bus_session_exec_queue(session, VMSG_BUS_SESSION_ACT_RX_PUBLISH);

out_unlock_ctx:
	mutex_unlock(&ctx->lock);
	return mask;
}

static int vmsg_bus_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct vmsg_bus_file_ctx *ctx = file->private_data;
	struct vmsg_bus_session *session;
	struct virtio_msg_bus_bridge_device *endpoint;
	struct vmsg_bus_map_record *record;
	u64 mmap_offset;
	u64 mmap_len;
	u32 handle;
	int ret;

	if (!ctx || !vma) {
		vm_trace(NULL, "mmap reject: ctx=%p vma=%p\n", ctx, vma);
		return -EINVAL;
	}

	mmap_len = (u64)(vma->vm_end - vma->vm_start);
	if (!mmap_len) {
		vm_trace(NULL, "mmap reject: zero length pgoff=%lu\n",
			 vma->vm_pgoff);
		return -EINVAL;
	}

	if ((u64)vma->vm_pgoff > (U64_MAX >> PAGE_SHIFT)) {
		vm_trace(NULL, "mmap reject: pgoff overflow pgoff=%lu\n",
			 vma->vm_pgoff);
		return -EOVERFLOW;
	}
	mmap_offset = (u64)vma->vm_pgoff << PAGE_SHIFT;

	mutex_lock(&ctx->lock);
	session = ctx->session;
	if (!session) {
		vm_trace(NULL, "mmap reject: no session offset=%#llx len=%#llx\n",
			 (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len);
		ret = -ENOTCONN;
		goto out_unlock_ctx;
	}

	mutex_lock(&session->map_lock);
	handle = session->endpoint ? session->endpoint->handle : 0;
	if (session->detached) {
		vm_trace(NULL,
			 "mmap reject: detached session handle=%u offset=%#llx len=%#llx\n",
			 handle, (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len);
		ret = -ENOTCONN;
		goto out_unlock_map;
	}

	endpoint = session->endpoint;
	if (!endpoint) {
		vm_trace(NULL,
			 "mmap reject: endpoint offline handle=%u offset=%#llx len=%#llx\n",
			 handle, (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len);
		ret = -ENODEV;
		goto out_unlock_map;
	}

	record = vmsg_bus_map_record_lookup_by_offset_locked(session, mmap_offset);
	if (!record || mmap_len > record->mmap_length ||
	    (record->state != VMSG_BUS_MAP_RECORD_MMAP_READY &&
	     record->state != VMSG_BUS_MAP_RECORD_ACTIVE)) {
		vm_trace(NULL,
			 "mmap reject: handle=%u offset=%#llx len=%#llx record=%p map_id=%#llx record_len=%#llx state=%u\n",
			 handle, (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len, record,
			 record ? (unsigned long long)record->map_id : 0,
			 record ? (unsigned long long)record->mmap_length : 0,
			 record ? record->state : 0);
		ret = -ENOENT;
		goto out_unlock_map;
	}

	if (!endpoint->ops || !endpoint->ops->mmap) {
		vm_trace(NULL,
			 "mmap reject: endpoint mmap unsupported handle=%u endpoint=%p ops=%p\n",
			 handle, endpoint, endpoint ? endpoint->ops : NULL);
		ret = -EOPNOTSUPP;
		goto out_unlock_map;
	}

	mutex_unlock(&session->map_lock);
	ret = endpoint->ops->mmap(endpoint, mmap_offset, vma);
	if (ret)
		vm_trace(NULL,
			 "mmap endpoint failed: handle=%u offset=%#llx len=%#llx ret=%d\n",
			 handle, (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len, ret);
	mutex_unlock(&ctx->lock);
	return ret;

out_unlock_map:
	mutex_unlock(&session->map_lock);
out_unlock_ctx:
	mutex_unlock(&ctx->lock);
	return ret;
}

static int vmsg_bus_open(struct inode *inode, struct file *file)
{
	struct vmsg_bus_file_ctx *ctx;

	(void)inode;
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	refcount_set(&ctx->refs, 1);
	mutex_init(&ctx->lock);
	file->private_data = ctx;
	return 0;
}

static int vmsg_bus_release(struct inode *inode, struct file *file)
{
	struct vmsg_bus_file_ctx *ctx = file->private_data;
	struct vmsg_bus_session *session;
	int ret;

	(void)inode;
	if (!ctx)
		return 0;

	mutex_lock(&ctx->lock);
	session = ctx->session;
	if (session) {
		ret = virtio_msg_bus_bridge_endpoint_cleanup_begin(session->endpoint_id);
		if (ret)
			vm_warn_rl(NULL, "forced session drop endpoint=%u ret=%d\n",
				   session->endpoint_id, ret);

		vmsg_bus_session_drop_locked(ctx, session);
		if (!ret)
			virtio_msg_bus_bridge_endpoint_cleanup_end(session->endpoint_id);
		vmsg_bus_session_put(session);
	}
	mutex_unlock(&ctx->lock);

	vmsg_bus_file_ctx_put(ctx);
	file->private_data = NULL;
	return 0;
}

static const struct file_operations vmsg_bus_fops = {
	.owner = THIS_MODULE,
	.open = vmsg_bus_open,
	.release = vmsg_bus_release,
	.poll = vmsg_bus_poll,
	.mmap = vmsg_bus_mmap,
	.unlocked_ioctl = vmsg_bus_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vmsg_bus_unlocked_ioctl,
#endif
	.llseek = noop_llseek,
};

static struct miscdevice vmsg_bus_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "virtio-msg-bridge",
	.fops = &vmsg_bus_fops,
};

static int vmsg_bus_cleanup_busy_entry_init(u32 handle)
{
	int ret = 0;

	mutex_lock(&vmsg_bus_cleanup_busy_lock);
	if (!xa_load(&vmsg_bus_cleanup_busy_endpoints, handle)) {
		ret = xa_err(xa_store(&vmsg_bus_cleanup_busy_endpoints, handle,
				      VMSG_BUS_CLEANUP_STATE_IDLE, GFP_KERNEL));
	}
	mutex_unlock(&vmsg_bus_cleanup_busy_lock);

	return ret;
}

static void vmsg_bus_cleanup_busy_entry_destroy(u32 handle)
{
	mutex_lock(&vmsg_bus_cleanup_busy_lock);
	xa_erase(&vmsg_bus_cleanup_busy_endpoints, handle);
	mutex_unlock(&vmsg_bus_cleanup_busy_lock);
}

static int __init vmsg_bus_init(void)
{
	return misc_register(&vmsg_bus_miscdev);
}

static void __exit vmsg_bus_exit(void)
{
	struct vmsg_bus_handle_entry *handle_entry;
	struct vmsg_bus_endpoint_topology *topology;
	struct vmsg_bus_resolver_entry *resolver;
	unsigned long index;

	misc_deregister(&vmsg_bus_miscdev);

	mutex_lock(&vmsg_bus_handles_lock);
	xa_for_each(&vmsg_bus_handles, index, handle_entry) {
		if (handle_entry->session) {
			vmsg_bus_session_put(handle_entry->session);
			handle_entry->session = NULL;
		}
		kfree(handle_entry);
	}
	xa_destroy(&vmsg_bus_handles);
	mutex_unlock(&vmsg_bus_handles_lock);

	mutex_lock(&vmsg_bus_endpoint_topologies_lock);
	xa_for_each(&vmsg_bus_endpoint_topologies, index, topology)
		vmsg_bus_endpoint_topology_destroy(topology);
	xa_destroy(&vmsg_bus_endpoint_topologies);
	mutex_unlock(&vmsg_bus_endpoint_topologies_lock);

	mutex_lock(&vmsg_bus_cleanup_busy_lock);
	xa_destroy(&vmsg_bus_cleanup_busy_endpoints);
	mutex_unlock(&vmsg_bus_cleanup_busy_lock);

	mutex_lock(&vmsg_bus_resolvers_lock);
	while (!list_empty(&vmsg_bus_resolvers)) {
		resolver = list_first_entry(&vmsg_bus_resolvers,
					    struct vmsg_bus_resolver_entry,
					    node);
		list_del(&resolver->node);
		kfree(resolver);
	}
	mutex_unlock(&vmsg_bus_resolvers_lock);
}

module_init(vmsg_bus_init);
module_exit(vmsg_bus_exit);

MODULE_DESCRIPTION("virtio-msg userspace bridge core");
MODULE_LICENSE("GPL");
