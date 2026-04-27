// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message Linux <-> userspace bridge core (Step 5 slice).
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#define pr_fmt(fmt) "virtio-msg-bridge: " fmt

#include <linux/atomic.h>
#include <linux/bitmap.h>
#include <linux/build_bug.h>
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
#include <linux/rcupdate.h>
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

enum vmsg_bus_map_record_state {
	VMSG_BUS_MAP_RECORD_ADD_PENDING = 1,
	VMSG_BUS_MAP_RECORD_ACTIVE = 2,
	VMSG_BUS_MAP_RECORD_DEL_PENDING = 3,
	VMSG_BUS_MAP_RECORD_REMOTE_RELEASED = 4,
	VMSG_BUS_MAP_RECORD_FAILED = 5,
	VMSG_BUS_MAP_RECORD_STALE = 6,
};

struct vmsg_bus_endpoint_topology {
	struct xarray devices;
	DECLARE_BITMAP(devices_bitmap, U16_MAX + 1U);
	struct rcu_head rcu;
};

enum vmsg_bus_session_state {
	VMSG_BUS_SESSION_ATTACHED_OFFLINE = 1,
	VMSG_BUS_SESSION_ATTACHED_ONLINE_NEEDS_TOPOLOGY = 2,
	VMSG_BUS_SESSION_ATTACHED_ONLINE_ACTIVE = 3,
	VMSG_BUS_SESSION_ATTACHED_FAILED = 4,
};

struct vmsg_bus_map_record {
	u64 map_id;
	u64 epoch;
	u64 map_seq;
	u16 request_token;
	u64 bus_addr;
	u64 length;
	u64 mmap_offset;
	u64 mmap_length;
	u32 flags;
	enum vmsg_bus_map_record_state state;
};

struct vmsg_bus_map_event_entry {
	struct list_head node;
	struct vmsg_bridge_uapi_map_event event;
	u16 request_token;
	bool published;
};

struct vmsg_bus_stale_map_event {
	struct list_head node;
	struct vmsg_bridge_uapi_map_event event;
};

enum vmsg_bus_session_action {
	VMSG_BUS_SESSION_ACT_TX_DRAIN = BIT(0),
	VMSG_BUS_SESSION_ACT_WAKE = BIT(1),
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
	enum vmsg_bus_session_state state;
	u64 online_epoch;
	u64 online_epoch_next;
	struct vmsg_bus_endpoint_topology staged_topology;
	u64 staged_topology_epoch;
	u32 staged_topology_num_devs;
	bool staged_topology_valid;
	struct list_head map_event_queue;
	/* Protects map queue, map records, and map sequence/id allocation. */
	struct mutex map_lock;
	u32 map_event_queue_len;
	u64 map_event_next_seq;
	u64 map_event_next_id;
	u16 map_request_next_token;
	/* Protects relay metadata keyed by (dev_num, token) -> request state. */
	struct mutex relay_lock;
	struct xarray relay_meta;
	bool detached;
	struct xarray map_records_by_id;
	struct xarray map_records_by_offset;
	/* Protects executor action bits and kick hook state. */
	spinlock_t exec_lock;
	unsigned long exec_actions;
	struct delayed_work exec_work;
	wait_queue_entry_t kick_wait;
	poll_table kick_pt;
	bool kick_hooked;
	/* Serializes RX ring publishers and preserves publication order. */
	struct mutex rx_publish_lock;
	wait_queue_head_t rx_publish_waitq;
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

struct vmsg_bus_resolver_entry {
	struct list_head node;
	struct virtio_msg_bus_bridge_resolver *resolver;
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
#define VMSG_BUS_RX_PUBLISH_TIMEOUT_MS	5000U
#define VMSG_BUS_TOPOLOGY_REMOVED_BATCH	64U
#define VMSG_BUS_TOPOLOGY_BITMAP_BYTES	8192U
#define VMSG_BUS_RX_TYPE_MASK		(VIRTIO_MSG_TYPE_RESPONSE | \
					 VIRTIO_MSG_TYPE_BUS)
#define VMSG_BUS_BRIDGE_LOCAL_MAX_PAYLOAD_SIZE \
	sizeof(struct virtio_msg_bus_bridge_map_add_req)
#define VMSG_BUS_BRIDGE_LOCAL_MAX_MSG_SIZE \
	(sizeof(struct virtio_msg) + \
	 VMSG_BUS_BRIDGE_LOCAL_MAX_PAYLOAD_SIZE)

static_assert(sizeof(struct virtio_msg_bus_bridge_endpoint_online) <=
	      VMSG_BUS_BRIDGE_LOCAL_MAX_PAYLOAD_SIZE);
static_assert(sizeof(struct virtio_msg_bus_bridge_endpoint_offline) <=
	      VMSG_BUS_BRIDGE_LOCAL_MAX_PAYLOAD_SIZE);
static_assert(sizeof(struct virtio_msg_bus_bridge_topology_reset) <=
	      VMSG_BUS_BRIDGE_LOCAL_MAX_PAYLOAD_SIZE);
static_assert(sizeof(struct virtio_msg_bus_bridge_topology_dev_add) <=
	      VMSG_BUS_BRIDGE_LOCAL_MAX_PAYLOAD_SIZE);
static_assert(sizeof(struct virtio_msg_bus_bridge_topology_commit) <=
	      VMSG_BUS_BRIDGE_LOCAL_MAX_PAYLOAD_SIZE);
static_assert(sizeof(struct virtio_msg_bus_bridge_map_add_req) <=
	      VMSG_BUS_BRIDGE_LOCAL_MAX_PAYLOAD_SIZE);
static_assert(sizeof(struct virtio_msg_bus_bridge_map_del_req) <=
	      VMSG_BUS_BRIDGE_LOCAL_MAX_PAYLOAD_SIZE);
static_assert(sizeof(struct virtio_msg_bus_bridge_map_add_resp) <=
	      VMSG_BUS_BRIDGE_LOCAL_MAX_PAYLOAD_SIZE);
static_assert(sizeof(struct virtio_msg_bus_bridge_map_del_resp) <=
	      VMSG_BUS_BRIDGE_LOCAL_MAX_PAYLOAD_SIZE);
static_assert(sizeof(struct virtio_msg_bus_bridge_map_released) <=
	      VMSG_BUS_BRIDGE_LOCAL_MAX_PAYLOAD_SIZE);
static_assert(sizeof(struct virtio_msg_bus_bridge_error) <=
	      VMSG_BUS_BRIDGE_LOCAL_MAX_PAYLOAD_SIZE);
static_assert(VMSG_BUS_BRIDGE_LOCAL_MAX_MSG_SIZE <= VIRTIO_MSG_MAX_SIZE);
static_assert(sizeof(struct virtio_msg_bus_bridge_endpoint_online) == 32);
static_assert(sizeof(struct virtio_msg_bus_bridge_endpoint_offline) == 16);
static_assert(sizeof(struct virtio_msg_bus_bridge_topology_reset) == 16);
static_assert(sizeof(struct virtio_msg_bus_bridge_topology_dev_add) == 16);
static_assert(sizeof(struct virtio_msg_bus_bridge_topology_commit) == 16);
static_assert(sizeof(struct virtio_msg_bus_bridge_map_add_req) == 64);
static_assert(sizeof(struct virtio_msg_bus_bridge_map_del_req) == 48);
static_assert(sizeof(struct virtio_msg_bus_bridge_map_add_resp) == 32);
static_assert(sizeof(struct virtio_msg_bus_bridge_map_del_resp) == 32);
static_assert(sizeof(struct virtio_msg_bus_bridge_map_released) == 40);
static_assert(sizeof(struct virtio_msg_bus_bridge_error) == 8);
static_assert(sizeof(struct vmsg_bridge_uapi_endpoint_addr) == 64);
static_assert(sizeof(struct vmsg_bridge_uapi_resolve_endpoint) == 88);
static_assert(sizeof(struct vmsg_bridge_uapi_release_endpoint) == 16);
static_assert(sizeof(struct vmsg_bridge_uapi_ring_hdr) == 24);
static_assert(sizeof(struct vmsg_bridge_uapi_ring_info) == 32);
static_assert(sizeof(struct vmsg_bridge_uapi_caps) == 48);
static_assert(sizeof(struct vmsg_bridge_uapi_attach) == 168);
static_assert(sizeof(struct vmsg_bridge_uapi_map_event) == 80);

static int vmsg_bus_cleanup_busy_entry_init(u32 handle);
static void vmsg_bus_cleanup_busy_entry_destroy(u32 handle);
static void vmsg_bus_map_event_purge_locked(struct vmsg_bus_session *session);
static u32 vmsg_bus_map_records_purge_locked(struct vmsg_bus_session *session);
static void vmsg_bus_session_caps_clear(struct vmsg_bus_session *session);
static void vmsg_bus_session_exec_queue(struct vmsg_bus_session *session,
					unsigned long actions);
static void vmsg_bus_session_exec_sync_disable(struct vmsg_bus_session *session);
static int
vmsg_bus_session_rx_publish_msg_locked(struct vmsg_bus_session *session,
				       u32 handle,
				       struct virtio_msg_bus_bridge_device *endpoint,
				       const struct virtio_msg *msg,
				       u16 msg_size, bool active_required,
				       bool nonblock);
static int
vmsg_bus_session_rx_publish_map_locked(struct vmsg_bus_session *session,
				       u16 request_token);
static void vmsg_bus_session_tx_disconnect(struct vmsg_bus_session *session);
static int vmsg_bus_session_caps_refresh(struct vmsg_bus_session *session);
static struct vmsg_bus_session *
vmsg_bus_session_get(struct vmsg_bus_session *session);
static void vmsg_bus_session_put(struct vmsg_bus_session *session);
static unsigned long vmsg_bus_relay_key(u16 dev_num, u16 token);
static int vmsg_bus_relay_insert(struct vmsg_bus_session *session, u16 dev_num,
				 u16 token, u8 msg_id, u64 relay_seq);
static int vmsg_bus_relay_consume(struct vmsg_bus_session *session,
				  u16 dev_num, u16 token, u8 *msg_id,
				  u64 *relay_seq);
static int vmsg_bus_relay_consume_msg_id(struct vmsg_bus_session *session,
					 u16 dev_num, u16 token,
					 u8 expected_msg_id, u64 *relay_seq);
static void vmsg_bus_relay_purge_session(struct vmsg_bus_session *session);
static int
vmsg_bus_validate_transport_msg_type(const struct virtio_msg *msg);
static int
vmsg_bus_dispatch_validate_transport_target
		(struct virtio_msg_bus_bridge_device *endpoint,
		 const struct virtio_msg *msg);
static int
vmsg_bus_bridge_device_rx_dispatch
		(struct virtio_msg_bus_bridge_device *endpoint,
		 const struct virtio_msg *msg, u16 msg_size,
		 const struct virtio_msg_dispatch_ctx *dctx,
		 enum virtio_msg_bus_bridge_dispatch_origin origin);
static unsigned long
vmsg_bus_endpoint_key(const struct virtio_msg_bus_bridge_device *endpoint);
static struct vmsg_bus_endpoint_topology *
vmsg_bus_endpoint_topology_lookup_locked(struct virtio_msg_bus_bridge_device *endpoint);
static struct vmsg_bus_endpoint_topology *
vmsg_bus_endpoint_topology_get_or_create_locked(struct virtio_msg_bus_bridge_device *endpoint,
						int *ret);
static void
vmsg_bus_endpoint_topology_destroy(struct vmsg_bus_endpoint_topology *topology);
static void
vmsg_bus_endpoint_topology_destroy_rcu(struct vmsg_bus_endpoint_topology *topology);
static void
vmsg_bus_bridge_topology_clear_locked(struct vmsg_bus_endpoint_topology *topology);
static int
vmsg_bus_bridge_topology_add_locked(struct vmsg_bus_endpoint_topology *topology,
				    u16 dev_num);
static int
vmsg_bus_bridge_topology_remove_locked(struct vmsg_bus_endpoint_topology *topology,
				       u16 dev_num);
static int
vmsg_bus_map_event_bridge_msg_build_locked(struct vmsg_bus_session *session,
					   u16 request_token, u8 *buf,
					   size_t buf_size,
					   u16 *out_msg_size);
static int
vmsg_bus_map_event_mark_published_locked(struct vmsg_bus_session *session,
					 u16 request_token);
static int
vmsg_bus_session_map_msg_handle(struct vmsg_bus_session *session,
				const struct virtio_msg *msg, u16 msg_size);
static int
vmsg_bus_session_bridge_error_handle(struct vmsg_bus_session *session,
				     const struct virtio_msg *msg,
				     u16 msg_size);
static int
vmsg_bus_map_event_ack_notify(struct vmsg_bus_session *session,
			      const struct vmsg_bridge_uapi_map_event *event,
			      u32 ack_status);
static void vmsg_bus_publish_endpoint_online(struct vmsg_bus_session *session);
static void vmsg_bus_publish_endpoint_offline(struct vmsg_bus_session *session,
					      u32 endpoint_offline_reason,
					      u64 online_epoch);
static int vmsg_bus_session_online_epoch_start_locked(struct vmsg_bus_session *session);
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
static void
vmsg_bus_topology_snapshot_locked(struct vmsg_bus_endpoint_topology *topology,
				  u8 *bitmap, size_t bitmap_len);
static void
vmsg_bus_topology_notify_bitmap_diff(struct virtio_msg_bus_bridge_device *endpoint,
				     const u8 *old_bitmap,
				     const u8 *new_bitmap, size_t bitmap_len);
static bool vmsg_bus_bitmap_test(const u8 *bitmap, u32 bit);
static void vmsg_bus_bitmap_set(u8 *bitmap, u32 bit);
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
		mod_delayed_work(system_wq, &session->exec_work, 0);
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
		WRITE_ONCE(session->state, VMSG_BUS_SESSION_ATTACHED_FAILED);
		wake_up_interruptible(&session->rx_publish_waitq);
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
		wake_up_interruptible(&session->rx_publish_waitq);
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

	if (events & EPOLLIN) {
		wake_up_interruptible(&session->rx_publish_waitq);
		vmsg_bus_session_exec_queue(session, VMSG_BUS_SESSION_ACT_TX_DRAIN);
	}
	if (events & (EPOLLERR | EPOLLHUP))
		vmsg_bus_session_exec_queue(session, VMSG_BUS_SESSION_ACT_WAKE);

	return 0;
}

static int
vmsg_bus_session_rx_publish_state_locked
		(struct vmsg_bus_session *session, u32 handle,
		 struct virtio_msg_bus_bridge_device *endpoint,
		 bool active_required)
{
	lockdep_assert_held(&session->map_lock);

	if (!session)
		return -EINVAL;
	if (READ_ONCE(session->ring_fault) ||
	    session->state == VMSG_BUS_SESSION_ATTACHED_FAILED)
		return -EIO;
	if (session->detached)
		return -ENOTCONN;
	if (handle && session->endpoint_id != handle)
		return -ENOTCONN;
	if (endpoint && session->endpoint != endpoint)
		return -ENOTCONN;

	if (!active_required)
		return 0;
	if (!session->endpoint ||
	    READ_ONCE(session->endpoint->rx_unregistered))
		return -ENOTCONN;
	if (session->state == VMSG_BUS_SESSION_ATTACHED_ONLINE_NEEDS_TOPOLOGY)
		return -EAGAIN;
	if (session->state != VMSG_BUS_SESSION_ATTACHED_ONLINE_ACTIVE)
		return -ENOTCONN;
	return 0;
}

static int vmsg_bus_session_rx_publish_try_write(struct vmsg_bus_session *session,
						 const struct virtio_msg *msg,
						 u16 msg_size)
{
	struct vmsg_bridge_uapi_ring_hdr hdr;
	u32 used;
	int ret;

	if (!session || !msg || msg_size < sizeof(*msg))
		return -EINVAL;
	if (msg_size > session->caps.msg_size ||
	    msg_size > session->rx.info.entry_size ||
	    msg_size > VIRTIO_MSG_MAX_SIZE)
		return -EMSGSIZE;

	ret = vmsg_bus_ring_hdr_read(&session->rx, &hdr);
	if (ret)
		return ret;

	ret = vmsg_bus_ring_occupancy(&hdr, &used);
	if (ret)
		return ret;
	if (used == hdr.entries)
		return -ENOSPC;

	ret = vmsg_bus_ring_slot_write(&session->rx, hdr.prod, msg, msg_size);
	if (ret)
		return ret;

	hdr.prod++;
	return vmsg_bus_ring_publish_prod(&session->rx, &hdr);
}

static bool vmsg_bus_session_rx_publish_ready(struct vmsg_bus_session *session,
					      bool active_required)
{
	struct vmsg_bridge_uapi_ring_hdr hdr;
	u32 used;

	if (!session || READ_ONCE(session->detached) ||
	    READ_ONCE(session->ring_fault))
		return true;
	if (active_required &&
	    READ_ONCE(session->state) != VMSG_BUS_SESSION_ATTACHED_ONLINE_ACTIVE)
		return true;
	if (vmsg_bus_ring_hdr_read(&session->rx, &hdr))
		return true;
	if (vmsg_bus_ring_occupancy(&hdr, &used))
		return true;

	return used < hdr.entries;
}

static int
vmsg_bus_session_rx_publish_wait_locked(struct vmsg_bus_session *session,
					long *timeout, bool active_required)
{
	long ret;

	lockdep_assert_held(&session->map_lock);

	if (!timeout || *timeout <= 0)
		return -ETIMEDOUT;

	mutex_unlock(&session->map_lock);
	ret = wait_event_interruptible_timeout
		(session->rx_publish_waitq,
		 vmsg_bus_session_rx_publish_ready(session, active_required),
		 *timeout);
	mutex_lock(&session->map_lock);

	if (ret < 0)
		return ret;
	if (!ret)
		return -ETIMEDOUT;

	*timeout = ret;
	return 0;
}

static int
vmsg_bus_session_rx_publish_msg_locked(struct vmsg_bus_session *session,
				       u32 handle,
				       struct virtio_msg_bus_bridge_device *endpoint,
				       const struct virtio_msg *msg,
				       u16 msg_size, bool active_required,
				       bool nonblock)
{
	long timeout = msecs_to_jiffies(VMSG_BUS_RX_PUBLISH_TIMEOUT_MS);
	int ret;

	lockdep_assert_held(&session->rx_publish_lock);
	lockdep_assert_held(&session->map_lock);

	if (!msg || msg_size < sizeof(*msg))
		return -EINVAL;
	if (msg_size > session->caps.msg_size ||
	    msg_size > session->rx.info.entry_size ||
	    msg_size > VIRTIO_MSG_MAX_SIZE)
		return -EMSGSIZE;

	for (;;) {
		ret = vmsg_bus_session_rx_publish_state_locked
			(session, handle, endpoint, active_required);
		if (ret)
			return ret;

		ret = vmsg_bus_session_rx_publish_try_write(session, msg, msg_size);
		if (!ret) {
			if (session->call_evt)
				eventfd_signal(session->call_evt);
			return 0;
		}
		if (ret != -ENOSPC) {
			vmsg_bus_session_mark_ring_fault(session);
			return ret;
		}
		if (nonblock)
			return -ENOSPC;

		ret = vmsg_bus_session_rx_publish_wait_locked
			(session, &timeout, active_required);
		if (ret)
			return ret;
	}
}

static int
vmsg_bus_session_rx_publish_map_locked(struct vmsg_bus_session *session,
				       u16 request_token)
{
	u8 buf[VMSG_BUS_BRIDGE_LOCAL_MAX_MSG_SIZE];
	struct virtio_msg *msg = (struct virtio_msg *)buf;
	long timeout = msecs_to_jiffies(VMSG_BUS_RX_PUBLISH_TIMEOUT_MS);
	u16 msg_size;
	int ret;

	lockdep_assert_held(&session->rx_publish_lock);
	lockdep_assert_held(&session->map_lock);

	for (;;) {
		ret = vmsg_bus_session_rx_publish_state_locked(session, 0, NULL,
							       true);
		if (ret)
			return ret;

		ret = vmsg_bus_map_event_bridge_msg_build_locked
			(session, request_token, buf, sizeof(buf), &msg_size);
		if (ret == -EAGAIN)
			return -ENOTCONN;
		if (ret)
			return ret;

		ret = vmsg_bus_session_rx_publish_try_write(session, msg,
							    msg_size);
		if (!ret) {
			ret = vmsg_bus_map_event_mark_published_locked
				(session, request_token);
			if (ret)
				return ret;
			if (session->call_evt)
				eventfd_signal(session->call_evt);
			return 0;
		}
		if (ret != -ENOSPC) {
			vmsg_bus_session_mark_ring_fault(session);
			return ret;
		}

		ret = vmsg_bus_session_rx_publish_wait_locked(session, &timeout,
							      true);
		if (ret)
			return ret;
	}
}

static int
vmsg_bus_session_rx_publish_bridge_event(struct vmsg_bus_session *session,
					 u8 msg_id, const void *payload,
					 u16 payload_size)
{
	u8 buf[VMSG_BUS_BRIDGE_LOCAL_MAX_MSG_SIZE];
	struct virtio_msg *msg = (struct virtio_msg *)buf;
	u16 msg_size;
	int ret;

	if (!session || !payload)
		return -EINVAL;

	msg_size = sizeof(*msg) + payload_size;
	if (msg_size > sizeof(buf))
		return -EMSGSIZE;

	memset(buf, 0, msg_size);
	msg->type = VIRTIO_MSG_TYPE_BUS;
	msg->msg_id = msg_id;
	msg->dev_num = 0;
	msg->token = 0;
	msg->msg_size = cpu_to_le16(msg_size);
	memcpy(msg->payload, payload, payload_size);

	mutex_lock(&session->rx_publish_lock);
	mutex_lock(&session->map_lock);
	ret = vmsg_bus_session_rx_publish_msg_locked(session, 0, NULL, msg,
						     msg_size, false, false);
	mutex_unlock(&session->map_lock);
	mutex_unlock(&session->rx_publish_lock);

	return ret;
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

static int __vmsg_bus_relay_consume(struct vmsg_bus_session *session,
				    u16 dev_num, u16 token,
				    const u8 *expected_msg_id, u8 *msg_id,
				    u64 *relay_seq)
{
	struct vmsg_bus_relay_meta *meta;
	unsigned long key;
	int ret = 0;

	if (!session || !relay_seq)
		return -EINVAL;

	key = vmsg_bus_relay_key(dev_num, token);
	mutex_lock(&session->relay_lock);
	meta = xa_load(&session->relay_meta, key);
	if (!meta) {
		ret = -ENOENT;
		goto out_unlock;
	}
	if (expected_msg_id && meta->msg_id != *expected_msg_id) {
		ret = -EPROTO;
		goto out_unlock;
	}

	xa_erase(&session->relay_meta, key);
	if (msg_id)
		*msg_id = meta->msg_id;
	*relay_seq = meta->relay_seq;

out_unlock:
	mutex_unlock(&session->relay_lock);
	if (!ret)
		kfree(meta);
	return ret;
}

static int vmsg_bus_relay_consume(struct vmsg_bus_session *session,
				  u16 dev_num, u16 token, u8 *msg_id,
				  u64 *relay_seq)
{
	return __vmsg_bus_relay_consume(session, dev_num, token, NULL, msg_id,
					relay_seq);
}

static int vmsg_bus_relay_consume_msg_id(struct vmsg_bus_session *session,
					 u16 dev_num, u16 token,
					 u8 expected_msg_id, u64 *relay_seq)
{
	return __vmsg_bus_relay_consume(session, dev_num, token,
					&expected_msg_id, NULL, relay_seq);
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

	ret = vmsg_bus_relay_consume(session, dev_num, token, NULL, &relay_seq);
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

	type = msg->type & VMSG_BUS_RX_TYPE_MASK;
	if (type & VIRTIO_MSG_TYPE_BUS)
		return -EOPNOTSUPP;

	return 0;
}

static bool vmsg_bus_transport_msg_is_response(const struct virtio_msg *msg)
{
	return (msg->type & VMSG_BUS_RX_TYPE_MASK) & VIRTIO_MSG_TYPE_RESPONSE;
}

static enum virtio_msg_bus_bridge_dispatch_origin
vmsg_bus_transport_dispatch_origin(const struct virtio_msg *msg,
				   const struct virtio_msg_dispatch_ctx *dctx)
{
	if (!dctx)
		return VIRTIO_MSG_BUS_BRIDGE_DISPATCH_ORIGIN_USERSPACE_TX;
	if (dctx->flags & VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE) {
		if (vmsg_bus_transport_msg_is_response(msg))
			return VIRTIO_MSG_BUS_BRIDGE_DISPATCH_ORIGIN_RELAY_RESPONSE;

		return VIRTIO_MSG_BUS_BRIDGE_DISPATCH_ORIGIN_RELAY_REQUEST;
	}
	if (dctx->flags & VIRTIO_MSG_DISPATCH_F_NONBLOCK)
		return VIRTIO_MSG_BUS_BRIDGE_DISPATCH_ORIGIN_RELAY_EVENT;

	return VIRTIO_MSG_BUS_BRIDGE_DISPATCH_ORIGIN_USERSPACE_TX;
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
	rcu_read_lock();
	topology = rcu_dereference(endpoint->topology);
	if (!topology || !test_bit(dev_num, topology->devices_bitmap))
		ret = -ENODEV;
	rcu_read_unlock();

	return ret;
}

static int
vmsg_bus_bridge_device_rx_dispatch
		(struct virtio_msg_bus_bridge_device *endpoint,
		 const struct virtio_msg *msg, u16 msg_size,
		 const struct virtio_msg_dispatch_ctx *dctx,
		 enum virtio_msg_bus_bridge_dispatch_origin origin)
{
	const struct virtio_msg_bus_bridge_device_ops *ops = endpoint->ops;

	switch (origin) {
	case VIRTIO_MSG_BUS_BRIDGE_DISPATCH_ORIGIN_USERSPACE_TX:
		if (ops->tx_userspace_msg)
			return ops->tx_userspace_msg(endpoint, msg, msg_size,
						     dctx);
		break;
	case VIRTIO_MSG_BUS_BRIDGE_DISPATCH_ORIGIN_RELAY_REQUEST:
		if (ops->relay_request)
			return ops->relay_request(endpoint, msg, msg_size, dctx);
		break;
	case VIRTIO_MSG_BUS_BRIDGE_DISPATCH_ORIGIN_RELAY_EVENT:
		if (ops->relay_event)
			return ops->relay_event(endpoint, msg, msg_size, dctx);
		break;
	case VIRTIO_MSG_BUS_BRIDGE_DISPATCH_ORIGIN_RELAY_RESPONSE:
		if (ops->relay_response)
			return ops->relay_response(endpoint, msg, msg_size, dctx);
		break;
	}

	return ops->tx_msg(endpoint, msg, msg_size, dctx);
}

static void vmsg_bus_session_tx_disconnect(struct vmsg_bus_session *session)
{
	if (!session)
		return;

	vmsg_bus_session_mark_offline(session,
				      VMSG_BRIDGE_UAPI_ENDPOINT_OFFLINE_REASON_REMOVED);
}

static void vmsg_bus_topology_init(struct vmsg_bus_endpoint_topology *topology)
{
	xa_init(&topology->devices);
	bitmap_zero(topology->devices_bitmap, U16_MAX + 1U);
}

static void vmsg_bus_topology_clear(struct vmsg_bus_endpoint_topology *topology)
{
	void *entry;
	unsigned long index;

	if (!topology)
		return;

	xa_for_each(&topology->devices, index, entry)
		clear_bit((u16)index, topology->devices_bitmap);

	xa_destroy(&topology->devices);
	vmsg_bus_topology_init(topology);
}

static struct vmsg_bus_endpoint_topology *vmsg_bus_topology_alloc(gfp_t gfp)
{
	struct vmsg_bus_endpoint_topology *topology;

	topology = kzalloc(sizeof(*topology), gfp);
	if (topology)
		vmsg_bus_topology_init(topology);

	return topology;
}

static int
vmsg_bus_topology_add_unpublished(struct vmsg_bus_endpoint_topology *topology,
				  u16 dev_num, gfp_t gfp)
{
	int ret;

	if (xa_load(&topology->devices, dev_num))
		return -EEXIST;

	ret = xa_err(xa_store(&topology->devices, dev_num, xa_mk_value(1),
			      gfp));
	if (ret)
		return ret;

	set_bit(dev_num, topology->devices_bitmap);
	return 0;
}

static void vmsg_bus_session_topology_stage_clear_locked(struct vmsg_bus_session *session)
{
	lockdep_assert_held(&session->map_lock);

	vmsg_bus_topology_clear(&session->staged_topology);
	session->staged_topology_epoch = 0;
	session->staged_topology_num_devs = 0;
	session->staged_topology_valid = false;
}

static bool vmsg_bus_tx_bus_msg_discard(const struct virtio_msg *msg)
{
	if (!(msg->type & VIRTIO_MSG_TYPE_BUS))
		return false;

	switch (msg->msg_id) {
	case VIRTIO_MSG_BUS_GET_DEVICES:
	case VIRTIO_MSG_BUS_PING:
	case VIRTIO_MSG_BUS_EVENT_DEVICE:
		return true;
	case VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_RESET:
	case VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_DEV_ADD:
	case VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_COMMIT:
	case VIRTIO_MSG_BUS_BRIDGE_ERROR:
	case VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_ONLINE:
	case VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_OFFLINE:
	case VIRTIO_MSG_BUS_BRIDGE_MAP_ADD:
	case VIRTIO_MSG_BUS_BRIDGE_MAP_DEL:
	case VIRTIO_MSG_BUS_BRIDGE_MAP_RELEASED:
		return false;
	default:
		return true;
	}
}

static int
vmsg_bus_tx_topology_msg_validate(const struct virtio_msg *msg, u16 msg_size,
				  u8 msg_id, u16 payload_size)
{
	if (msg->type != VIRTIO_MSG_TYPE_BUS || msg->msg_id != msg_id ||
	    le16_to_cpu(msg->dev_num) || le16_to_cpu(msg->token))
		return -EPROTO;
	if (msg_size != sizeof(*msg) + payload_size)
		return -EPROTO;

	return 0;
}

static int
vmsg_bus_session_epoch_check_locked(struct vmsg_bus_session *session,
				    u64 online_epoch, bool *stale)
{
	lockdep_assert_held(&session->map_lock);

	*stale = false;
	if (!online_epoch)
		return -EPROTO;

	if (!session->online_epoch) {
		if (session->state == VMSG_BUS_SESSION_ATTACHED_OFFLINE &&
		    (!session->online_epoch_next ||
		     online_epoch < session->online_epoch_next)) {
			*stale = true;
			return 0;
		}

		return -EPROTO;
	}

	if (online_epoch < session->online_epoch) {
		*stale = true;
		return 0;
	}
	if (online_epoch > session->online_epoch)
		return -EPROTO;
	if (session->state == VMSG_BUS_SESSION_ATTACHED_OFFLINE ||
	    session->state == VMSG_BUS_SESSION_ATTACHED_FAILED)
		return -EPROTO;

	return 0;
}

static int
vmsg_bus_session_topology_epoch_check_locked(struct vmsg_bus_session *session,
					     u64 online_epoch,
					     bool *stale)
{
	lockdep_assert_held(&session->map_lock);

	return vmsg_bus_session_epoch_check_locked(session, online_epoch,
						   stale);
}

static int
vmsg_bus_session_topology_reset_locked(struct vmsg_bus_session *session,
				       const struct virtio_msg *msg,
				       u16 msg_size)
{
	const struct virtio_msg_bus_bridge_topology_reset *reset;
	bool stale;
	u64 online_epoch;
	int ret;

	lockdep_assert_held(&session->map_lock);

	ret = vmsg_bus_tx_topology_msg_validate
		(msg, msg_size, VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_RESET,
		 sizeof(*reset));
	if (ret)
		return ret;

	reset = (const void *)msg->payload;
	online_epoch = le64_to_cpu(reset->online_epoch);
	ret = vmsg_bus_session_topology_epoch_check_locked(session,
							   online_epoch,
							   &stale);
	if (ret || stale)
		return ret;

	vmsg_bus_session_topology_stage_clear_locked(session);
	session->staged_topology_epoch = online_epoch;
	session->staged_topology_valid = true;
	return 0;
}

static int
vmsg_bus_session_topology_add_locked(struct vmsg_bus_session *session,
				     const struct virtio_msg *msg,
				     u16 msg_size)
{
	const struct virtio_msg_bus_bridge_topology_dev_add *add;
	bool stale;
	u64 online_epoch;
	u16 dev_num;
	int ret;

	lockdep_assert_held(&session->map_lock);

	ret = vmsg_bus_tx_topology_msg_validate
		(msg, msg_size, VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_DEV_ADD,
		 sizeof(*add));
	if (ret)
		return ret;

	add = (const void *)msg->payload;
	online_epoch = le64_to_cpu(add->online_epoch);
	ret = vmsg_bus_session_topology_epoch_check_locked(session,
							   online_epoch,
							   &stale);
	if (ret || stale)
		return ret;
	if (!session->staged_topology_valid ||
	    session->staged_topology_epoch != online_epoch)
		return -EPROTO;

	dev_num = le16_to_cpu(add->dev_num);
	if (test_bit(dev_num, session->staged_topology.devices_bitmap))
		return -EPROTO;

	ret = xa_err(xa_store(&session->staged_topology.devices, dev_num,
			      xa_mk_value(1), GFP_KERNEL));
	if (ret)
		return ret;
	set_bit(dev_num, session->staged_topology.devices_bitmap);
	session->staged_topology_num_devs++;
	return 0;
}

static int
vmsg_bus_session_topology_active_replace_locked(struct vmsg_bus_session *session)
{
	struct virtio_msg_bus_bridge_device *endpoint = session->endpoint;
	struct vmsg_bus_endpoint_topology *active = NULL;
	struct vmsg_bus_endpoint_topology *replacement;
	void *entry;
	unsigned long index;
	u8 *old_bitmap;
	u8 *new_bitmap;
	int ret = 0;

	lockdep_assert_held(&session->map_lock);

	if (!endpoint || READ_ONCE(endpoint->rx_unregistered))
		return -ENOTCONN;

	old_bitmap = kcalloc(VMSG_BUS_TOPOLOGY_BITMAP_BYTES,
			     sizeof(*old_bitmap), GFP_KERNEL);
	new_bitmap = kcalloc(VMSG_BUS_TOPOLOGY_BITMAP_BYTES,
			     sizeof(*new_bitmap), GFP_KERNEL);
	if (!old_bitmap || !new_bitmap) {
		ret = -ENOMEM;
		goto out_free;
	}

	replacement = vmsg_bus_topology_alloc(GFP_KERNEL);
	if (!replacement) {
		ret = -ENOMEM;
		goto out_free;
	}
	xa_for_each(&session->staged_topology.devices, index, entry) {
		ret = vmsg_bus_topology_add_unpublished(replacement,
							(u16)index,
							GFP_KERNEL);
		if (ret)
			goto out_destroy_replacement;
		vmsg_bus_bitmap_set(new_bitmap, (u32)index);
	}

	mutex_lock(&vmsg_bus_endpoint_topologies_lock);
	if (READ_ONCE(endpoint->rx_unregistered)) {
		ret = -ENOTCONN;
		goto out_unlock;
	}

	active = vmsg_bus_endpoint_topology_lookup_locked(endpoint);
	vmsg_bus_topology_snapshot_locked(active, old_bitmap,
					  VMSG_BUS_TOPOLOGY_BITMAP_BYTES);

	entry = xa_store(&vmsg_bus_endpoint_topologies,
			 vmsg_bus_endpoint_key(endpoint), replacement,
			 GFP_KERNEL);
	ret = xa_err(entry);
	if (ret)
		goto out_unlock;

	active = entry;
	rcu_assign_pointer(endpoint->topology, replacement);
	replacement = NULL;
out_unlock:
	mutex_unlock(&vmsg_bus_endpoint_topologies_lock);
	if (!ret) {
		vmsg_bus_endpoint_topology_destroy_rcu(active);
		vmsg_bus_topology_notify_bitmap_diff
			(endpoint, old_bitmap, new_bitmap,
			 VMSG_BUS_TOPOLOGY_BITMAP_BYTES);
	}
out_destroy_replacement:
	vmsg_bus_endpoint_topology_destroy(replacement);
out_free:
	kfree(new_bitmap);
	kfree(old_bitmap);
	return ret;
}

static int
vmsg_bus_session_topology_commit_locked(struct vmsg_bus_session *session,
					const struct virtio_msg *msg,
					u16 msg_size)
{
	const struct virtio_msg_bus_bridge_topology_commit *commit;
	bool stale;
	u64 online_epoch;
	u16 num_devs;
	int ret;

	lockdep_assert_held(&session->map_lock);

	ret = vmsg_bus_tx_topology_msg_validate
		(msg, msg_size, VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_COMMIT,
		 sizeof(*commit));
	if (ret)
		return ret;

	commit = (const void *)msg->payload;
	online_epoch = le64_to_cpu(commit->online_epoch);
	ret = vmsg_bus_session_topology_epoch_check_locked(session,
							   online_epoch,
							   &stale);
	if (ret || stale)
		return ret;
	if (!session->staged_topology_valid ||
	    session->staged_topology_epoch != online_epoch)
		return -EPROTO;

	num_devs = le16_to_cpu(commit->num_devs);
	if (num_devs != session->staged_topology_num_devs)
		return -EPROTO;

	ret = vmsg_bus_session_topology_active_replace_locked(session);
	if (ret)
		return ret;

	vmsg_bus_session_topology_stage_clear_locked(session);
	WRITE_ONCE(session->state, VMSG_BUS_SESSION_ATTACHED_ONLINE_ACTIVE);
	return 0;
}

static int
vmsg_bus_bridge_error_validate(const struct virtio_msg *msg, u16 msg_size,
			       const struct virtio_msg_bus_bridge_error **out_error,
			       int *out_errno)
{
	const struct virtio_msg_bus_bridge_error *error;
	s32 error_code;

	if (!msg || !out_error || !out_errno)
		return -EINVAL;
	if (msg->type != (VIRTIO_MSG_TYPE_BUS | VIRTIO_MSG_TYPE_RESPONSE) ||
	    msg->msg_id != VIRTIO_MSG_BUS_BRIDGE_ERROR ||
	    le16_to_cpu(msg->dev_num))
		return -EPROTO;
	if (msg_size != sizeof(*msg) + sizeof(*error))
		return -EPROTO;

	error = (const void *)msg->payload;
	if (error->original_msg_id & VIRTIO_MSG_ID_EVENT_BIT)
		return -EPROTO;

	error_code = (s32)le32_to_cpu(error->error);
	if (error_code >= 0)
		return -EPROTO;

	*out_error = error;
	*out_errno = error_code;
	return 0;
}

static int
vmsg_bus_session_bridge_error_handle(struct vmsg_bus_session *session,
				     const struct virtio_msg *msg,
				     u16 msg_size)
{
	const struct virtio_msg_bus_bridge_error *error;
	int error_code;
	int ret;

	ret = vmsg_bus_bridge_error_validate(msg, msg_size, &error,
					     &error_code);
	if (ret)
		return ret;

	return virtio_msg_bus_bridge_device_report_error
		(session->endpoint_id, le16_to_cpu(error->original_dev_num),
		 le16_to_cpu(msg->token), error->original_msg_id, error_code);
}

static int
vmsg_bus_session_tx_handle_bus_msg(struct vmsg_bus_session *session,
				   const struct virtio_msg *msg,
				   u16 msg_size, bool *handled)
{
	struct virtio_msg_bus_bridge_device *online_endpoint = NULL;
	int ret;

	*handled = false;
	if (!(msg->type & VIRTIO_MSG_TYPE_BUS))
		return 0;

	*handled = true;
	if (vmsg_bus_tx_bus_msg_discard(msg))
		return 0;

	if (msg->msg_id == VIRTIO_MSG_BUS_BRIDGE_ERROR)
		return vmsg_bus_session_bridge_error_handle(session, msg,
							    msg_size);

	mutex_lock(&session->map_lock);
	switch (msg->msg_id) {
	case VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_RESET:
		ret = vmsg_bus_session_topology_reset_locked(session, msg,
							     msg_size);
		break;
	case VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_DEV_ADD:
		ret = vmsg_bus_session_topology_add_locked(session, msg,
							   msg_size);
		break;
	case VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_COMMIT:
		ret = vmsg_bus_session_topology_commit_locked(session, msg,
							      msg_size);
		if (!ret && session->endpoint && session->endpoint->ops &&
		    session->endpoint->ops->endpoint_online)
			online_endpoint = session->endpoint;
		break;
	case VIRTIO_MSG_BUS_BRIDGE_MAP_ADD:
	case VIRTIO_MSG_BUS_BRIDGE_MAP_DEL:
	case VIRTIO_MSG_BUS_BRIDGE_MAP_RELEASED:
		mutex_unlock(&session->map_lock);
		return vmsg_bus_session_map_msg_handle(session, msg, msg_size);
	default:
		ret = -EPROTO;
		break;
	}
	mutex_unlock(&session->map_lock);
	if (online_endpoint)
		online_endpoint->ops->endpoint_online(online_endpoint);

	return ret;
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
		bool handled;

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
		    msg_size > session->caps.msg_size ||
		    msg_size > session->tx.info.entry_size ||
		    msg_size > VIRTIO_MSG_MAX_SIZE)
			goto out_fault;

		ret = vmsg_bus_session_tx_handle_bus_msg(session, msg, msg_size,
							 &handled);
		if (handled) {
			if (!ret) {
				hdr.cons++;
				ret = vmsg_bus_ring_publish_cons(&session->tx,
								 &hdr);
				if (ret)
					goto out_fault;
				continue;
			}
			if (ret == -ENODEV || ret == -ENOTCONN ||
			    ret == -ESHUTDOWN) {
				vmsg_bus_session_tx_disconnect(session);
				break;
			}
			goto out_fault;
		}

		dctx_ptr = NULL;
		if (vmsg_bus_transport_msg_is_response(msg)) {
			ret = vmsg_bus_relay_consume_msg_id(session,
							    le16_to_cpu(msg->dev_num),
							    le16_to_cpu(msg->token),
							    msg->msg_id,
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
		if (vmsg_bus_transport_msg_is_response(msg) && ret == -ENOENT) {
			hdr.cons++;
			ret = vmsg_bus_ring_publish_cons(&session->tx, &hdr);
			if (ret)
				goto out_fault;
			continue;
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

static void vmsg_bus_session_exec_workfn(struct work_struct *work)
{
	struct vmsg_bus_session *session;
	unsigned long actions;

	session = container_of(to_delayed_work(work), struct vmsg_bus_session,
			       exec_work);

	for (;;) {
		bool do_wake = false;

		actions = vmsg_bus_session_exec_fetch(session);
		if (!actions)
			break;
		if (READ_ONCE(session->detached))
			break;

		if (actions & VMSG_BUS_SESSION_ACT_TX_DRAIN)
			vmsg_bus_session_tx_drain(session);
		if (actions & VMSG_BUS_SESSION_ACT_WAKE)
			do_wake = true;

		if (do_wake) {
			if (session->call_evt)
				eventfd_signal(session->call_evt);
			if (READ_ONCE(session->ring_fault))
				wake_up_interruptible_poll(&session->poll_waitq,
							   EPOLLERR);
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
	session->exec_actions = 0;
	spin_unlock_irqrestore(&session->exec_lock, irqflags);

	if (kick_hooked && session->kick_evt)
		eventfd_ctx_remove_wait_queue(session->kick_evt, &session->kick_wait,
					      &cnt);
	wake_up_interruptible(&session->rx_publish_waitq);
	cancel_delayed_work_sync(&session->exec_work);
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
	if (!session)
		return;

	vmsg_bus_session_exec_sync_disable(session);

	mutex_lock(&session->map_lock);
	vmsg_bus_map_event_purge_locked(session);
	vmsg_bus_map_records_purge_locked(session);
	vmsg_bus_session_topology_stage_clear_locked(session);
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
vmsg_bus_map_event_lookup_token_locked(struct vmsg_bus_session *session,
				       u16 token)
{
	struct vmsg_bus_map_event_entry *entry;

	lockdep_assert_held(&session->map_lock);

	if (!token)
		return NULL;

	list_for_each_entry(entry, &session->map_event_queue, node) {
		if (entry->request_token == token)
			return entry;
	}

	return NULL;
}

static bool
vmsg_bus_map_event_token_busy_locked(struct vmsg_bus_session *session,
				     u16 token)
{
	struct vmsg_bus_map_event_entry *entry;

	lockdep_assert_held(&session->map_lock);

	list_for_each_entry(entry, &session->map_event_queue, node) {
		if (entry->request_token == token)
			return true;
	}

	return false;
}

static int
vmsg_bus_map_request_token_alloc_locked(struct vmsg_bus_session *session,
					u16 *token)
{
	u16 candidate;
	unsigned int i;

	lockdep_assert_held(&session->map_lock);

	if (!token)
		return -EINVAL;

	candidate = session->map_request_next_token;
	if (!candidate)
		candidate = 1;

	for (i = 0; i < U16_MAX; i++) {
		if (candidate && !vmsg_bus_map_event_token_busy_locked(session,
								       candidate)) {
			*token = candidate;
			session->map_request_next_token = candidate + 1;
			if (!session->map_request_next_token)
				session->map_request_next_token = 1;
			return 0;
		}
		candidate++;
		if (!candidate)
			candidate = 1;
	}

	return -EBUSY;
}

static int
vmsg_bus_map_event_entry_alloc(const struct vmsg_bridge_uapi_map_event *event,
			       u16 request_token,
			       struct vmsg_bus_map_event_entry **out_entry)
{
	struct vmsg_bus_map_event_entry *entry;

	if (!event || !request_token || !out_entry)
		return -EINVAL;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	INIT_LIST_HEAD(&entry->node);
	entry->event = *event;
	entry->request_token = request_token;
	entry->published = false;
	*out_entry = entry;
	return 0;
}

static int
vmsg_bus_map_event_enqueue_entry_locked(struct vmsg_bus_session *session,
					struct vmsg_bus_map_event_entry *entry)
{
	int ret;

	lockdep_assert_held(&session->rx_publish_lock);
	lockdep_assert_held(&session->map_lock);

	if (!entry)
		return -EINVAL;
	if (session->map_event_queue_len >= VMSG_BUS_MAP_EVENT_QUEUE_MAX) {
		kfree(entry);
		return -ENOSPC;
	}

	list_add_tail(&entry->node, &session->map_event_queue);
	session->map_event_queue_len++;

	ret = vmsg_bus_session_rx_publish_map_locked(session, entry->request_token);
	if (ret) {
		if (vmsg_bus_map_event_lookup_token_locked
				(session, entry->request_token)) {
			list_del(&entry->node);
			if (session->map_event_queue_len)
				session->map_event_queue_len--;
			kfree(entry);
		}
		return ret;
	}

	return 0;
}

static int
vmsg_bus_map_event_enqueue_locked(struct vmsg_bus_session *session,
				  const struct vmsg_bridge_uapi_map_event *event,
				  u16 request_token)
{
	struct vmsg_bus_map_event_entry *entry;
	int ret;

	lockdep_assert_held(&session->map_lock);

	ret = vmsg_bus_map_event_entry_alloc(event, request_token, &entry);
	if (ret)
		return ret;

	ret = vmsg_bus_map_event_enqueue_entry_locked(session, entry);
	if (ret)
		return ret;

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
vmsg_bus_map_event_bridge_msg_build_locked(struct vmsg_bus_session *session,
					   u16 request_token, u8 *buf,
					   size_t buf_size,
					   u16 *out_msg_size)
{
	struct vmsg_bus_map_event_entry *entry;
	struct virtio_msg_bus_bridge_map_add_req add;
	struct virtio_msg_bus_bridge_map_del_req del_req;
	const struct vmsg_bridge_uapi_map_event *event;
	struct virtio_msg *msg = (struct virtio_msg *)buf;
	const void *payload;
	u16 payload_size;
	u8 msg_id;

	lockdep_assert_held(&session->map_lock);

	if (!request_token || !buf || !out_msg_size || buf_size < sizeof(*msg))
		return -EINVAL;

	entry = vmsg_bus_map_event_lookup_token_locked(session, request_token);
	if (!entry || entry->published)
		return -EAGAIN;

	event = &entry->event;
	switch (event->type) {
	case VMSG_BRIDGE_UAPI_MAP_EVENT_ADD_RESP:
		memset(&add, 0, sizeof(add));
		add.online_epoch = cpu_to_le64(event->epoch);
		add.map_seq = cpu_to_le64(event->map_seq);
		add.map_id = cpu_to_le64(event->map_id);
		add.bus_addr = cpu_to_le64(event->bus_addr);
		add.length = cpu_to_le64(event->length);
		add.mmap_offset = cpu_to_le64(event->mmap_offset);
		add.mmap_length = cpu_to_le64(event->mmap_length);
		add.flags = cpu_to_le32(event->flags);
		msg_id = VIRTIO_MSG_BUS_BRIDGE_MAP_ADD;
		payload = &add;
		payload_size = sizeof(add);
		break;
	case VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP:
		memset(&del_req, 0, sizeof(del_req));
		del_req.online_epoch = cpu_to_le64(event->epoch);
		del_req.map_seq = cpu_to_le64(event->map_seq);
		del_req.map_id = cpu_to_le64(event->map_id);
		del_req.bus_addr = cpu_to_le64(event->bus_addr);
		del_req.length = cpu_to_le64(event->length);
		msg_id = VIRTIO_MSG_BUS_BRIDGE_MAP_DEL;
		payload = &del_req;
		payload_size = sizeof(del_req);
		break;
	default:
		return -EINVAL;
	}

	*out_msg_size = sizeof(*msg) + payload_size;
	if (*out_msg_size > session->caps.msg_size ||
	    *out_msg_size > buf_size ||
	    *out_msg_size > session->rx.info.entry_size)
		return -EMSGSIZE;

	memset(buf, 0, *out_msg_size);
	msg->type = VIRTIO_MSG_TYPE_BUS;
	msg->msg_id = msg_id;
	msg->dev_num = 0;
	msg->token = cpu_to_le16(entry->request_token);
	msg->msg_size = cpu_to_le16(*out_msg_size);
	memcpy(msg->payload, payload, payload_size);

	return 0;
}

static int
vmsg_bus_map_event_mark_published_locked(struct vmsg_bus_session *session,
					 u16 request_token)
{
	struct vmsg_bus_map_event_entry *entry;

	lockdep_assert_held(&session->map_lock);

	entry = vmsg_bus_map_event_lookup_token_locked(session, request_token);
	if (!entry || entry->published)
		return -EAGAIN;

	entry->published = true;
	return 0;
}

static u8
vmsg_bus_map_event_msg_id(const struct vmsg_bridge_uapi_map_event *event)
{
	switch (event->type) {
	case VMSG_BRIDGE_UAPI_MAP_EVENT_ADD_RESP:
		return VIRTIO_MSG_BUS_BRIDGE_MAP_ADD;
	case VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP:
		return VIRTIO_MSG_BUS_BRIDGE_MAP_DEL;
	default:
		return 0;
	}
}

struct vmsg_bus_map_response {
	u8 msg_id;
	u16 token;
	u64 online_epoch;
	u64 map_seq;
	u64 map_id;
	s32 status;
};

static int
vmsg_bus_map_response_validate(const struct virtio_msg *msg, u16 msg_size,
			       struct vmsg_bus_map_response *response)
{
	const struct virtio_msg_bus_bridge_map_add_resp *add;
	const struct virtio_msg_bus_bridge_map_del_resp *del;
	s32 status;

	if (!msg || !response)
		return -EINVAL;
	if (msg->type != (VIRTIO_MSG_TYPE_BUS | VIRTIO_MSG_TYPE_RESPONSE) ||
	    le16_to_cpu(msg->dev_num) || !le16_to_cpu(msg->token))
		return -EPROTO;

	memset(response, 0, sizeof(*response));
	response->msg_id = msg->msg_id;
	response->token = le16_to_cpu(msg->token);

	switch (msg->msg_id) {
	case VIRTIO_MSG_BUS_BRIDGE_MAP_ADD:
		if (msg_size != sizeof(*msg) + sizeof(*add))
			return -EPROTO;
		add = (const void *)msg->payload;
		if (add->reserved)
			return -EPROTO;
		status = (s32)le32_to_cpu(add->status);
		response->online_epoch = le64_to_cpu(add->online_epoch);
		response->map_seq = le64_to_cpu(add->map_seq);
		response->map_id = le64_to_cpu(add->map_id);
		break;
	case VIRTIO_MSG_BUS_BRIDGE_MAP_DEL:
		if (msg_size != sizeof(*msg) + sizeof(*del))
			return -EPROTO;
		del = (const void *)msg->payload;
		if (del->reserved)
			return -EPROTO;
		status = (s32)le32_to_cpu(del->status);
		response->online_epoch = le64_to_cpu(del->online_epoch);
		response->map_seq = le64_to_cpu(del->map_seq);
		response->map_id = le64_to_cpu(del->map_id);
		break;
	default:
		return -EPROTO;
	}

	if (status > 0)
		return -EPROTO;

	response->status = status;
	return 0;
}

static int
vmsg_bus_map_response_match_locked(struct vmsg_bus_session *session,
				   const struct vmsg_bus_map_response *response,
				   struct vmsg_bus_map_event_entry **out_entry)
{
	struct vmsg_bus_map_event_entry *entry;
	struct vmsg_bus_map_record *record;
	bool stale;
	int ret;

	lockdep_assert_held(&session->map_lock);

	if (!response || !out_entry)
		return -EINVAL;

	*out_entry = NULL;
	entry = vmsg_bus_map_event_lookup_token_locked(session, response->token);

	ret = vmsg_bus_session_epoch_check_locked(session,
						  response->online_epoch,
						  &stale);
	if (ret || stale)
		goto out_stale;
	if (session->detached || !session->endpoint)
		return -ENOTCONN;

	if (entry) {
		if (!entry->published)
			return -EPROTO;
		if (response->online_epoch < entry->event.epoch ||
		    response->map_seq < entry->event.map_seq)
			goto out_stale;
		if (response->online_epoch > entry->event.epoch ||
		    response->map_seq > entry->event.map_seq)
			return -EPROTO;
		if (entry->event.map_id != response->map_id ||
		    vmsg_bus_map_event_msg_id(&entry->event) !=
			    response->msg_id)
			return -EPROTO;
	}

	record = vmsg_bus_map_record_lookup_by_id_locked(session,
							 response->map_id);
	if (!record)
		goto out_stale;
	if (response->map_seq < record->map_seq)
		goto out_stale;
	if (response->map_seq > record->map_seq ||
	    response->online_epoch != record->epoch)
		return -EPROTO;

	if (!entry)
		return -EPROTO;

	*out_entry = entry;
	return 0;

out_stale:
	if (!ret)
		*out_entry = NULL;
	return ret;
}

static void
vmsg_bus_map_event_entry_remove_locked(struct vmsg_bus_session *session,
				       struct vmsg_bus_map_event_entry *entry)
{
	lockdep_assert_held(&session->map_lock);

	if (!entry)
		return;

	list_del(&entry->node);
	if (session->map_event_queue_len)
		session->map_event_queue_len--;
	kfree(entry);
}

static void
vmsg_bus_map_event_remove_map_locked(struct vmsg_bus_session *session,
				     u64 map_id)
{
	struct vmsg_bus_map_event_entry *entry;
	struct vmsg_bus_map_event_entry *tmp;

	lockdep_assert_held(&session->map_lock);

	list_for_each_entry_safe(entry, tmp, &session->map_event_queue, node) {
		if (entry->event.map_id == map_id)
			vmsg_bus_map_event_entry_remove_locked(session, entry);
	}
}

static int
vmsg_bus_map_response_apply_locked(struct vmsg_bus_session *session,
				   struct vmsg_bus_map_event_entry *entry,
				   const struct vmsg_bus_map_response *response,
				   struct vmsg_bridge_uapi_map_event *event,
				   u32 *ack_status)
{
	struct vmsg_bus_map_record *record;

	lockdep_assert_held(&session->map_lock);

	if (!entry || !response || !event || !ack_status)
		return -EINVAL;

	record = vmsg_bus_map_record_lookup_by_id_locked(session,
							 entry->event.map_id);
	if (!record)
		return -ENOENT;

	*event = entry->event;
	event->status = response->status;
	*ack_status = response->status ? VMSG_BRIDGE_UAPI_MAP_ACK_REJECT :
					 VMSG_BRIDGE_UAPI_MAP_ACK_OK;

	switch (entry->event.type) {
	case VMSG_BRIDGE_UAPI_MAP_EVENT_ADD_RESP:
		if (record->state != VMSG_BUS_MAP_RECORD_ADD_PENDING &&
		    record->state != VMSG_BUS_MAP_RECORD_DEL_PENDING)
			return -ENOENT;
		if (response->status) {
			record->state = VMSG_BUS_MAP_RECORD_FAILED;
			vmsg_bus_map_event_remove_map_locked(session,
							     record->map_id);
			vmsg_bus_map_record_remove_locked(session, record);
			return 0;
		}
		if (record->state == VMSG_BUS_MAP_RECORD_ADD_PENDING)
			record->state = VMSG_BUS_MAP_RECORD_ACTIVE;
		vmsg_bus_map_event_entry_remove_locked(session, entry);
		return 0;
	case VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP:
		if (response->status)
			return -EPROTO;
		if (record->state != VMSG_BUS_MAP_RECORD_DEL_PENDING &&
		    record->state != VMSG_BUS_MAP_RECORD_REMOTE_RELEASED)
			return -ENOENT;
		vmsg_bus_map_event_remove_map_locked(session, record->map_id);
		vmsg_bus_map_record_remove_locked(session, record);
		return 0;
	default:
		return -EPROTO;
	}
}

static int
vmsg_bus_session_map_response_handle(struct vmsg_bus_session *session,
				     const struct virtio_msg *msg,
				     u16 msg_size)
{
	struct vmsg_bus_map_response response;
	struct vmsg_bus_map_event_entry *entry;
	struct vmsg_bridge_uapi_map_event event;
	u32 ack_status;
	bool stale = false;
	int ret;

	ret = vmsg_bus_map_response_validate(msg, msg_size, &response);
	if (ret)
		return ret;

	mutex_lock(&session->map_lock);
	ret = vmsg_bus_map_response_match_locked(session, &response, &entry);
	if (ret || !entry) {
		stale = !ret && !entry;
		goto out_unlock;
	}
	ret = vmsg_bus_map_response_apply_locked(session, entry, &response,
						 &event, &ack_status);
	mutex_unlock(&session->map_lock);
	if (ret)
		return ret;

	return vmsg_bus_map_event_ack_notify(session, &event, ack_status);

out_unlock:
	mutex_unlock(&session->map_lock);
	return stale ? 0 : ret;
}

static int
vmsg_bus_map_released_validate(const struct virtio_msg *msg, u16 msg_size,
			       const struct virtio_msg_bus_bridge_map_released **out)
{
	if (!msg || !out)
		return -EINVAL;
	if (msg->type != VIRTIO_MSG_TYPE_BUS ||
	    msg->msg_id != VIRTIO_MSG_BUS_BRIDGE_MAP_RELEASED ||
	    le16_to_cpu(msg->dev_num) || le16_to_cpu(msg->token))
		return -EPROTO;
	if (msg_size != sizeof(*msg) +
			sizeof(struct virtio_msg_bus_bridge_map_released))
		return -EPROTO;

	*out = (const void *)msg->payload;
	return 0;
}

static int
vmsg_bus_session_map_released_handle(struct vmsg_bus_session *session,
				     const struct virtio_msg *msg,
				     u16 msg_size)
{
	const struct virtio_msg_bus_bridge_map_released *released;
	struct vmsg_bus_map_record *record;
	struct vmsg_bridge_uapi_map_event event = { 0 };
	u64 online_epoch;
	u64 map_seq;
	u64 map_id;
	u64 bus_addr;
	u64 length;
	bool notify = false;
	bool stale;
	int ret;

	ret = vmsg_bus_map_released_validate(msg, msg_size, &released);
	if (ret)
		return ret;

	online_epoch = le64_to_cpu(released->online_epoch);
	map_seq = le64_to_cpu(released->map_seq);
	map_id = le64_to_cpu(released->map_id);
	bus_addr = le64_to_cpu(released->bus_addr);
	length = le64_to_cpu(released->length);

	mutex_lock(&session->map_lock);
	ret = vmsg_bus_session_epoch_check_locked(session, online_epoch, &stale);
	if (ret || stale)
		goto out_unlock;
	if (session->detached || !session->endpoint) {
		ret = -ENOTCONN;
		goto out_unlock;
	}

	record = vmsg_bus_map_record_lookup_by_id_locked(session, map_id);
	if (!record)
		goto out_stale;
	if (map_seq < record->map_seq)
		goto out_stale;
	if (map_seq > record->map_seq || online_epoch != record->epoch ||
	    bus_addr != record->bus_addr || length != record->length) {
		ret = -EPROTO;
		goto out_unlock;
	}

	event.map_seq = record->map_seq;
	event.map_id = record->map_id;
	event.epoch = record->epoch;
	event.bus_addr = record->bus_addr;
	event.length = record->length;
	event.mmap_offset = record->mmap_offset;
	event.mmap_length = record->mmap_length;
	event.flags = record->flags;
	event.status = 0;

	if (record->state == VMSG_BUS_MAP_RECORD_DEL_PENDING) {
		event.type = VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP;
		vmsg_bus_map_event_remove_map_locked(session, map_id);
		vmsg_bus_map_record_remove_locked(session, record);
		notify = true;
	} else if (record->state == VMSG_BUS_MAP_RECORD_ACTIVE) {
		record->state = VMSG_BUS_MAP_RECORD_REMOTE_RELEASED;
		event.type = VMSG_BRIDGE_UAPI_MAP_EVENT_RELEASED;
		notify = true;
	} else {
		ret = -EPROTO;
	}

out_unlock:
	mutex_unlock(&session->map_lock);
	if (ret)
		return ret;
	if (!notify)
		return 0;

	return vmsg_bus_map_event_ack_notify
		(session, &event, VMSG_BRIDGE_UAPI_MAP_ACK_OK);

out_stale:
	mutex_unlock(&session->map_lock);
	return 0;
}

static int
vmsg_bus_session_map_msg_handle(struct vmsg_bus_session *session,
				const struct virtio_msg *msg, u16 msg_size)
{
	if (msg->msg_id == VIRTIO_MSG_BUS_BRIDGE_MAP_RELEASED)
		return vmsg_bus_session_map_released_handle(session, msg,
							    msg_size);

	return vmsg_bus_session_map_response_handle(session, msg, msg_size);
}

static void vmsg_bus_stale_map_events_free(struct list_head *stale_events)
{
	struct vmsg_bus_stale_map_event *event;
	struct vmsg_bus_stale_map_event *tmp;

	if (!stale_events)
		return;

	list_for_each_entry_safe(event, tmp, stale_events, node) {
		list_del(&event->node);
		kfree(event);
	}
}

static void
vmsg_bus_stale_map_events_notify(struct virtio_msg_bus_bridge_device *endpoint,
				 struct list_head *stale_events)
{
	struct vmsg_bus_stale_map_event *event;

	if (!endpoint || !endpoint->ops || !endpoint->ops->map_event_ack ||
	    !stale_events)
		goto out_free;

	list_for_each_entry(event, stale_events, node)
		endpoint->ops->map_event_ack(endpoint, &event->event,
					     VMSG_BRIDGE_UAPI_MAP_ACK_REJECT);

out_free:
	vmsg_bus_stale_map_events_free(stale_events);
}

static int
vmsg_bus_map_records_stale_collect_locked(struct vmsg_bus_session *session,
					  int status,
					  struct list_head *stale_events)
{
	struct vmsg_bus_stale_map_event *event;
	struct vmsg_bus_map_record *record;
	unsigned long index;

	lockdep_assert_held(&session->map_lock);

	if (!stale_events)
		return -EINVAL;
	if (!session->endpoint || !session->endpoint->ops ||
	    !session->endpoint->ops->map_event_ack)
		return 0;

	xa_for_each(&session->map_records_by_id, index, record) {
		event = kzalloc(sizeof(*event), GFP_KERNEL);
		if (!event)
			return -ENOMEM;

		event->event.map_seq = record->map_seq;
		event->event.map_id = record->map_id;
		event->event.epoch = record->epoch;
		event->event.bus_addr = record->bus_addr;
		event->event.length = record->length;
		event->event.mmap_offset = record->mmap_offset;
		event->event.mmap_length = record->mmap_length;
		event->event.flags = record->flags;
		event->event.status = status;

		if (record->state == VMSG_BUS_MAP_RECORD_DEL_PENDING)
			event->event.type = VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP;
		else
			event->event.type = VMSG_BRIDGE_UAPI_MAP_EVENT_ADD_RESP;

		record->state = VMSG_BUS_MAP_RECORD_STALE;
		list_add_tail(&event->node, stale_events);
	}

	return 0;
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
		rcu_assign_pointer(endpoint->topology, topology);
		return topology;
	}

	topology = vmsg_bus_topology_alloc(GFP_KERNEL);
	if (!topology) {
		*ret = -ENOMEM;
		return NULL;
	}

	*ret = xa_err(xa_store(&vmsg_bus_endpoint_topologies,
			       vmsg_bus_endpoint_key(endpoint), topology,
			       GFP_KERNEL));
	if (*ret) {
		xa_destroy(&topology->devices);
		kfree(topology);
		return NULL;
	}

	rcu_assign_pointer(endpoint->topology, topology);
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

static void vmsg_bus_endpoint_topology_free_rcu(struct rcu_head *rcu)
{
	struct vmsg_bus_endpoint_topology *topology;

	topology = container_of(rcu, struct vmsg_bus_endpoint_topology, rcu);
	kfree(topology);
}

static void
vmsg_bus_endpoint_topology_destroy_rcu(struct vmsg_bus_endpoint_topology *topology)
{
	if (!topology)
		return;

	/*
	 * RCU readers only consume devices_bitmap. Drop the xarray before the
	 * RCU callback so xa_destroy() does not run from softirq context.
	 */
	xa_destroy(&topology->devices);
	call_rcu(&topology->rcu, vmsg_bus_endpoint_topology_free_rcu);
}

static void
vmsg_bus_endpoint_topology_remove_locked(struct virtio_msg_bus_bridge_device *endpoint)
{
	struct vmsg_bus_endpoint_topology *topology;

	lockdep_assert_held(&vmsg_bus_endpoint_topologies_lock);

	RCU_INIT_POINTER(endpoint->topology, NULL);
	topology = xa_erase(&vmsg_bus_endpoint_topologies,
			    vmsg_bus_endpoint_key(endpoint));
	vmsg_bus_endpoint_topology_destroy_rcu(topology);
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
	struct virtio_msg_bus_bridge_resolver *resolver;
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
	RCU_INIT_POINTER(endpoint->topology, NULL);
	mutex_unlock(&vmsg_bus_endpoint_topologies_lock);
}

static void
vmsg_bus_bridge_endpoint_mark_session_offline(struct vmsg_bus_session *session,
					      u32 endpoint_offline_reason)
{
	struct vmsg_bus_file_ctx *ctx;
	bool mark_offline = false;

	if (!session)
		return;

	ctx = vmsg_bus_file_ctx_get(READ_ONCE(session->ctx));
	if (ctx)
		mutex_lock(&ctx->lock);
	if (ctx && session->ctx == ctx && ctx->session == session)
		mark_offline = true;
	if (ctx)
		mutex_unlock(&ctx->lock);
	vmsg_bus_file_ctx_put(ctx);

	if (mark_offline)
		vmsg_bus_session_mark_offline(session, endpoint_offline_reason);
}

int virtio_msg_bus_bridge_resolver_register(struct virtio_msg_bus_bridge_resolver *resolver)
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
	pr_info("bridge resolver registered: bus='%s'\n", resolver->name);
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_resolver_register);

void virtio_msg_bus_bridge_resolver_unregister(struct virtio_msg_bus_bridge_resolver *resolver)
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
		pr_info("bridge resolver unregistered: bus='%s'\n",
			resolver->name);
		break;
	}
	mutex_unlock(&vmsg_bus_resolvers_lock);
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_resolver_unregister);

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

	pr_info("endpoint registered: bus='%s' handle=%u\n", bus_name, handle);

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

	pr_info("endpoint unregister: bus='%s' handle=%u\n", bus_name,
		endpoint->handle);

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
	rcu_assign_pointer(endpoint->topology, topology);
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
	size_t out_bitmap_len;
	u32 window_end;
	u16 num = 0;
	u16 next_offset = 0;
	int ret = 0;

	if (!endpoint || !out_num || !out_next_offset)
		return -EINVAL;

	if (!count) {
		*out_num = 0;
		*out_next_offset = 0;
		return 0;
	}

	window_end = min_t(u32, (u32)offset + count, (u32)U16_MAX + 1U);

	mutex_lock(&vmsg_bus_endpoint_topologies_lock);
	topology = vmsg_bus_endpoint_topology_lookup_locked(endpoint);
	if (topology) {
		index = offset;
		while (index < window_end) {
			entry = xa_find(&topology->devices, &index,
					window_end - 1, XA_PRESENT);
			if (!entry)
				break;

			num = index - offset + 1;
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

	out_bitmap_len = DIV_ROUND_UP(num, 8);
	if (out_bitmap_len) {
		if (!bitmap) {
			ret = -EINVAL;
			goto out_unlock;
		}
		if (out_bitmap_len > bitmap_len) {
			ret = -EMSGSIZE;
			goto out_unlock;
		}

		memset(bitmap, 0, out_bitmap_len);
		if (topology) {
			index = offset;
			while (index < (u32)offset + num) {
				entry = xa_find(&topology->devices, &index,
						(u32)offset + num - 1,
						XA_PRESENT);
				if (!entry)
					break;

				bitmap[(index - offset) >> 3] |=
					BIT((index - offset) & 0x7);
				index++;
			}
		}
	}

out_unlock:
	mutex_unlock(&vmsg_bus_endpoint_topologies_lock);
	if (ret)
		return ret;

	*out_num = num;
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
				      u32 flags,
				      const struct virtio_msg_bus_bridge_map_add_install *install,
				      u64 *out_map_id)
{
	struct vmsg_bus_map_event_entry *event_entry;
	struct vmsg_bus_map_record *record;
	struct vmsg_bridge_uapi_map_event event = { 0 };
	u16 request_token;
	u64 end;
	int ret;

	lockdep_assert_held(&session->map_lock);

	if (session->detached || !session->endpoint ||
	    READ_ONCE(session->endpoint->rx_unregistered))
		return -ENOTCONN;
	if (!session->online_epoch ||
	    session->state == VMSG_BUS_SESSION_ATTACHED_OFFLINE)
		return -ENOTCONN;
	if (session->state == VMSG_BUS_SESSION_ATTACHED_FAILED)
		return -EIO;
	if (session->state == VMSG_BUS_SESSION_ATTACHED_ONLINE_NEEDS_TOPOLOGY)
		return -EAGAIN;
	if (session->state != VMSG_BUS_SESSION_ATTACHED_ONLINE_ACTIVE)
		return -ENOTCONN;
	if (flags & ~VIRTIO_MSG_BUS_BRIDGE_MAP_F_RETENTION_REQUESTED)
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

	record->epoch = session->online_epoch;
	record->bus_addr = bus_addr;
	record->length = length;
	record->mmap_offset = mmap_offset;
	record->mmap_length = mmap_length;
	record->flags = flags;
	record->state = VMSG_BUS_MAP_RECORD_ADD_PENDING;

	ret = vmsg_bus_map_event_alloc_seq_locked(session, &record->map_seq);
	if (ret)
		goto err_free_record;
	ret = vmsg_bus_map_request_token_alloc_locked(session, &request_token);
	if (ret)
		goto err_free_record;
	record->request_token = request_token;

	ret = vmsg_bus_map_record_insert_locked(session, record);
	if (ret)
		goto err_free_record;

	event.map_seq = record->map_seq;
	event.map_id = record->map_id;
	event.epoch = record->epoch;
	event.bus_addr = record->bus_addr;
	event.length = record->length;
	event.mmap_offset = record->mmap_offset;
	event.mmap_length = record->mmap_length;
	event.type = VMSG_BRIDGE_UAPI_MAP_EVENT_ADD_RESP;
	event.flags = record->flags;

	ret = vmsg_bus_map_event_entry_alloc(&event, request_token,
					     &event_entry);
	if (ret)
		goto err_remove_record;

	if (install && install->install) {
		ret = install->install(session->endpoint, record->map_id,
				       install->data);
		if (ret)
			goto err_free_event;
	}

	ret = vmsg_bus_map_event_enqueue_entry_locked(session, event_entry);
	if (ret)
		goto err_remove_record_by_id;

	if (out_map_id)
		*out_map_id = record->map_id;

	return 0;

err_free_event:
	kfree(event_entry);
err_remove_record:
	vmsg_bus_map_record_remove_locked(session, record);
	return ret;
err_remove_record_by_id:
	record = vmsg_bus_map_record_lookup_by_id_locked(session, event.map_id);
	if (record)
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
	u16 request_token;
	int ret;

	lockdep_assert_held(&session->map_lock);

	if (session->detached || !session->endpoint ||
	    READ_ONCE(session->endpoint->rx_unregistered))
		return -ENOTCONN;
	if (!session->online_epoch ||
	    session->state == VMSG_BUS_SESSION_ATTACHED_OFFLINE)
		return -ENOTCONN;
	if (session->state == VMSG_BUS_SESSION_ATTACHED_FAILED)
		return -EIO;
	if (session->state == VMSG_BUS_SESSION_ATTACHED_ONLINE_NEEDS_TOPOLOGY)
		return -EAGAIN;
	if (session->state != VMSG_BUS_SESSION_ATTACHED_ONLINE_ACTIVE)
		return -ENOTCONN;
	if (!map_id)
		return -EINVAL;

	record = vmsg_bus_map_record_lookup_by_id_locked(session, map_id);
	if (!record)
		return -ENOENT;
	if (record->state == VMSG_BUS_MAP_RECORD_DEL_PENDING)
		return -EALREADY;
	if (record->state == VMSG_BUS_MAP_RECORD_REMOTE_RELEASED ||
	    record->state == VMSG_BUS_MAP_RECORD_FAILED ||
	    record->state == VMSG_BUS_MAP_RECORD_STALE) {
		vmsg_bus_map_event_remove_map_locked(session, record->map_id);
		vmsg_bus_map_record_remove_locked(session, record);
		return 0;
	}
	if (record->state != VMSG_BUS_MAP_RECORD_ADD_PENDING &&
	    record->state != VMSG_BUS_MAP_RECORD_ACTIVE)
		return -EBUSY;

	ret = vmsg_bus_map_request_token_alloc_locked(session, &request_token);
	if (ret)
		return ret;

	event.map_seq = record->map_seq;
	event.map_id = record->map_id;
	event.epoch = record->epoch;
	event.bus_addr = record->bus_addr;
	event.length = record->length;
	event.mmap_offset = record->mmap_offset;
	event.mmap_length = record->mmap_length;
	event.type = VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP;
	event.flags = record->flags;

	ret = vmsg_bus_map_event_enqueue_locked(session, &event, request_token);
	if (ret)
		return ret;

	record->request_token = request_token;
	record->state = VMSG_BUS_MAP_RECORD_DEL_PENDING;
	return 0;
}

int virtio_msg_bus_bridge_device_map_add_install(u32 handle, u64 bus_addr,
						 u64 length, u64 mmap_offset,
						 u64 mmap_length, u32 flags,
						 const struct virtio_msg_bus_bridge_map_add_install
							*install,
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

	mutex_lock(&session->rx_publish_lock);
	mutex_lock(&session->map_lock);
	if (session->endpoint_id != handle || session->detached || !session->endpoint) {
		ret = -ENOTCONN;
		goto out_unlock_map;
	}

	ret = vmsg_bus_map_event_publish_add_locked(session, bus_addr, length,
						    mmap_offset, mmap_length,
						    flags, install, map_id);

out_unlock_map:
	mutex_unlock(&session->map_lock);
	mutex_unlock(&session->rx_publish_lock);
	vmsg_bus_session_put(session);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_map_add_install);

int virtio_msg_bus_bridge_device_map_add(u32 handle, u64 bus_addr,
					 u64 length, u64 mmap_offset,
					 u64 mmap_length, u32 flags,
					 u64 *map_id)
{
	return virtio_msg_bus_bridge_device_map_add_install
		(handle, bus_addr, length, mmap_offset, mmap_length, flags,
		 NULL, map_id);
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_map_add);

int virtio_msg_bus_bridge_device_map_del(u32 handle, u64 map_id)
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

	mutex_lock(&session->rx_publish_lock);
	mutex_lock(&session->map_lock);
	if (session->endpoint_id != handle || session->detached || !session->endpoint) {
		ret = -ENOTCONN;
		goto out_unlock_map;
	}

	ret = vmsg_bus_map_event_publish_del_req_locked(session, map_id);

out_unlock_map:
	mutex_unlock(&session->map_lock);
	mutex_unlock(&session->rx_publish_lock);
	vmsg_bus_session_put(session);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_map_del);

int
virtio_msg_bus_bridge_device_map_event_add_install(u32 handle, u64 bus_addr,
						   u64 length, u64 mmap_offset,
						   u64 mmap_length, u32 flags,
						  const struct virtio_msg_bus_bridge_map_add_install
							*install,
						   u64 *map_id)
{
	return virtio_msg_bus_bridge_device_map_add_install
		(handle, bus_addr, length, mmap_offset, mmap_length, flags,
		 install, map_id);
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_map_event_add_install);

int virtio_msg_bus_bridge_device_map_event_add(u32 handle, u64 bus_addr,
					       u64 length, u64 mmap_offset,
					       u64 mmap_length, u32 flags,
					       u64 *map_id)
{
	return virtio_msg_bus_bridge_device_map_add
		(handle, bus_addr, length, mmap_offset, mmap_length, flags,
		 map_id);
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_map_event_add);

int virtio_msg_bus_bridge_device_map_event_del_req(u32 handle, u64 map_id)
{
	return virtio_msg_bus_bridge_device_map_del(handle, map_id);
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_bridge_device_map_event_del_req);

int virtio_msg_bus_bridge_device_rx(u32 handle, const struct virtio_msg *msg,
				    const struct virtio_msg_dispatch_ctx *dctx)
{
	enum virtio_msg_bus_bridge_dispatch_origin origin;
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
		       !vmsg_bus_transport_msg_is_response(msg) &&
		       !(msg->msg_id & VIRTIO_MSG_ID_EVENT_BIT);
	event_msg = msg->msg_id & VIRTIO_MSG_ID_EVENT_BIT;

	if (event_msg)
		pr_debug("rx event: handle=%u msg_id=0x%02x dev=%u type=0x%02x nonblock=%u\n",
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
	if (entry->session && msg_size > entry->session->caps.msg_size) {
		ret = -EMSGSIZE;
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

	origin = vmsg_bus_transport_dispatch_origin(msg, dctx);
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

	ret = vmsg_bus_bridge_device_rx_dispatch(endpoint, msg, msg_size, dctx,
						 origin);
	if (ret) {
		if (event_msg)
			pr_debug("rx event dispatch failed: handle=%u msg_id=0x%02x ret=%d\n",
				 handle, msg->msg_id, ret);
		else
			pr_err_ratelimited("rx dispatch failed: handle=%u msg_id=0x%02x ret=%d\n",
					   handle, msg->msg_id, ret);
	}
	if (record_relay && ret) {
		u64 tmp_seq;

		vmsg_bus_relay_consume(session, relay_dev_num, relay_token,
				       NULL, &tmp_seq);
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

	if (!handle || error >= 0)
		return -EINVAL;
	if (msg_id & VIRTIO_MSG_ID_EVENT_BIT)
		return -EINVAL;

	pr_debug("report_error: hdl=%u dev=%u tok=%u id=0x%02x err=%d\n",
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

	mutex_lock(&session->rx_publish_lock);
	mutex_lock(&session->map_lock);
	ret = vmsg_bus_session_rx_publish_msg_locked(session, handle, NULL, msg,
						     msg_size, true, false);

	mutex_unlock(&session->map_lock);
	mutex_unlock(&session->rx_publish_lock);
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

	if (!mutex_trylock(&session->rx_publish_lock)) {
		ret = -EAGAIN;
		goto out_put;
	}
	if (!mutex_trylock(&session->map_lock)) {
		ret = -EAGAIN;
		goto out_unlock_publish;
	}
	ret = vmsg_bus_session_rx_publish_msg_locked(session, 0, endpoint, msg,
						     msg_size, true, true);

	mutex_unlock(&session->map_lock);
out_unlock_publish:
	mutex_unlock(&session->rx_publish_lock);
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
	if (req->vmm_max_msg_size < VMSG_BUS_BRIDGE_LOCAL_MAX_MSG_SIZE)
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
	if (caps.msg_size < VMSG_BUS_BRIDGE_LOCAL_MAX_MSG_SIZE ||
	    caps.msg_size > VIRTIO_MSG_MAX_SIZE)
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
	if (!READ_ONCE(session->endpoint)) {
		mutex_lock(&session->map_lock);
		session->online_epoch = 0;
		WRITE_ONCE(session->state,
			   VMSG_BUS_SESSION_ATTACHED_OFFLINE);
		mutex_unlock(&session->map_lock);
		return 0;
	}

	ret = vmsg_bus_session_caps_refresh(session);
	if (ret)
		return ret;

	mutex_lock(&session->map_lock);
	ret = vmsg_bus_session_online_epoch_start_locked(session);
	mutex_unlock(&session->map_lock);
	if (ret)
		return ret;

	req->caps.revision = session->caps.revision;
	req->caps.max_msg_size = session->caps.msg_size;
	req->caps.transport_features = session->caps.transport_features;
	req->caps.bridge_features = session->caps.bridge_features;
	req->caps.online_epoch = session->online_epoch;
	req->caps.flags |= VMSG_BRIDGE_UAPI_CAP_F_ENDPOINT_ONLINE;

	return 0;
}

static int vmsg_bus_session_online_epoch_start_locked(struct vmsg_bus_session *session)
{
	lockdep_assert_held(&session->map_lock);

	if (!session->online_epoch_next)
		return -EOVERFLOW;

	vmsg_bus_session_topology_stage_clear_locked(session);
	session->online_epoch = session->online_epoch_next;
	if (session->online_epoch_next == U64_MAX)
		session->online_epoch_next = 0;
	else
		session->online_epoch_next++;

	WRITE_ONCE(session->state,
		   VMSG_BUS_SESSION_ATTACHED_ONLINE_NEEDS_TOPOLOGY);
	return 0;
}

static void vmsg_bus_publish_endpoint_online(struct vmsg_bus_session *session)
{
	struct virtio_msg_bus_bridge_endpoint_online endpoint_online;

	if (!session)
		return;

	memset(&endpoint_online, 0, sizeof(endpoint_online));
	endpoint_online.online_epoch = cpu_to_le64(session->online_epoch);
	strscpy(endpoint_online.bus_name, session->caps.name,
		sizeof(endpoint_online.bus_name));
	endpoint_online.max_msg_size =
		cpu_to_le32(session->endpoint_max_msg_size);
	vmsg_bus_session_rx_publish_bridge_event
		(session, VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_ONLINE,
		 &endpoint_online, sizeof(endpoint_online));
	pr_debug("endpoint online published: endpoint_id=%u\n",
		 session->endpoint_id);
}

static void vmsg_bus_publish_endpoint_offline(struct vmsg_bus_session *session,
					      u32 endpoint_offline_reason,
					      u64 online_epoch)
{
	struct virtio_msg_bus_bridge_endpoint_offline endpoint_offline;

	if (!session)
		return;

	memset(&endpoint_offline, 0, sizeof(endpoint_offline));
	endpoint_offline.online_epoch = cpu_to_le64(online_epoch);
	endpoint_offline.reason = cpu_to_le32(endpoint_offline_reason);
	vmsg_bus_session_rx_publish_bridge_event
		(session, VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_OFFLINE,
		 &endpoint_offline, sizeof(endpoint_offline));
	pr_debug("endpoint offline published: endpoint_id=%u reason=%u\n",
		 session->endpoint_id, endpoint_offline_reason);
}

static void vmsg_bus_session_mark_offline(struct vmsg_bus_session *session,
					  u32 endpoint_offline_reason)
{
	struct virtio_msg_bus_bridge_device *endpoint = NULL;
	char bus_name[VMSG_BRIDGE_UAPI_BUS_NAME_LEN] = "";
	LIST_HEAD(stale_events);
	u64 offline_epoch = 0;
	bool was_online = false;
	int ret;

	if (!session)
		return;

	mutex_lock(&session->map_lock);
	if (!session->detached && session->endpoint) {
		endpoint = session->endpoint;
		if (session->caps.name)
			strscpy(bus_name, session->caps.name, sizeof(bus_name));
		ret = vmsg_bus_map_records_stale_collect_locked
				(session, -ENOTCONN, &stale_events);
		if (ret)
			pr_debug("stale map notify collect failed endpoint=%u ret=%d\n",
				 session->endpoint_id, ret);
		WRITE_ONCE(session->endpoint, NULL);
		vmsg_bus_map_event_purge_locked(session);
		vmsg_bus_map_records_purge_locked(session);
		vmsg_bus_session_topology_stage_clear_locked(session);
		WRITE_ONCE(session->state, VMSG_BUS_SESSION_ATTACHED_OFFLINE);
		offline_epoch = session->online_epoch;
		session->online_epoch = 0;
		was_online = true;
	}
	mutex_unlock(&session->map_lock);
	wake_up_interruptible(&session->rx_publish_waitq);
	vmsg_bus_stale_map_events_notify(endpoint, &stale_events);

	if (was_online && endpoint) {
		pr_info("client offline: bus='%s' endpoint=%u reason=%u\n",
			bus_name, session->endpoint_id, endpoint_offline_reason);
		vmsg_bus_endpoint_publish_session_clear(endpoint, session);
		vmsg_bus_topology_clear_notify_removed(endpoint);
		vmsg_bus_publish_endpoint_offline(session,
						  endpoint_offline_reason,
						  offline_epoch);
		vmsg_bus_session_caps_clear(session);
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
		pr_err_ratelimited("rebind caps refresh failed endpoint=%u ret=%d\n",
				   session->endpoint_id, ret);
		mutex_lock(&session->map_lock);
		if (!session->detached && session->endpoint == endpoint)
			WRITE_ONCE(session->endpoint, NULL);
		mutex_unlock(&session->map_lock);
		vmsg_bus_session_caps_clear(session);
		vmsg_bus_endpoint_publish_session_clear(endpoint, session);
		goto out_unlock_ctx;
	}

	mutex_lock(&session->map_lock);
	ret = vmsg_bus_session_online_epoch_start_locked(session);
	if (ret) {
		WRITE_ONCE(session->endpoint, NULL);
		WRITE_ONCE(session->state, VMSG_BUS_SESSION_ATTACHED_FAILED);
		mutex_unlock(&session->map_lock);
		vmsg_bus_session_caps_clear(session);
		vmsg_bus_endpoint_publish_session_clear(endpoint, session);
		goto out_unlock_ctx;
	}
	mutex_unlock(&session->map_lock);

	vmsg_bus_publish_endpoint_online(session);
	pr_info("client rebind online: bus='%s' endpoint=%u\n",
		session->caps.name, session->endpoint_id);

out_unlock_ctx:
	mutex_unlock(&ctx->lock);
	vmsg_bus_file_ctx_put(ctx);
}

static bool vmsg_bus_session_invalidate(struct vmsg_bus_session *session)
{
	struct virtio_msg_bus_bridge_device *endpoint = NULL;
	LIST_HEAD(stale_events);
	bool newly_detached = false;
	int ret;

	if (!session)
		return false;

	mutex_lock(&session->map_lock);
	if (!session->detached) {
		session->detached = true;
		WRITE_ONCE(session->state, VMSG_BUS_SESSION_ATTACHED_FAILED);
		ret = vmsg_bus_map_records_stale_collect_locked
				(session, -ENOTCONN, &stale_events);
		if (ret)
			pr_debug("stale map notify collect failed endpoint=%u ret=%d\n",
				 session->endpoint_id, ret);
		vmsg_bus_map_event_purge_locked(session);
		vmsg_bus_map_records_purge_locked(session);
		vmsg_bus_session_topology_stage_clear_locked(session);
		newly_detached = true;
	}
	endpoint = READ_ONCE(session->endpoint);
	WRITE_ONCE(session->endpoint, NULL);
	mutex_unlock(&session->map_lock);
	wake_up_interruptible(&session->rx_publish_waitq);
	vmsg_bus_stale_map_events_notify(endpoint, &stale_events);
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
	pr_info("client detached: bus='%s' endpoint=%u\n", bus_name,
		endpoint_id);
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
	INIT_LIST_HEAD(&session->map_event_queue);
	xa_init(&session->map_records_by_id);
	xa_init(&session->map_records_by_offset);
	vmsg_bus_topology_init(&session->staged_topology);
	session->map_event_next_seq = 1;
	session->map_event_next_id = 1;
	session->map_request_next_token = 1;
	session->state = VMSG_BUS_SESSION_ATTACHED_OFFLINE;
	session->online_epoch = 0;
	session->online_epoch_next = 1;
	mutex_init(&session->relay_lock);
	xa_init(&session->relay_meta);
	spin_lock_init(&session->exec_lock);
	INIT_DELAYED_WORK(&session->exec_work, vmsg_bus_session_exec_workfn);
	mutex_init(&session->rx_publish_lock);
	init_waitqueue_head(&session->rx_publish_waitq);
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

	pr_info("client attached: bus='%s' endpoint=%u vmm_max_msg_size=%u\n",
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
	pr_info("client detach requested: endpoint=%u\n", endpoint_id);
	ret = 0;

out_unlock_ctx:
	mutex_unlock(&ctx->lock);
	return ret;
}

static int
vmsg_bus_map_event_ack_notify(struct vmsg_bus_session *session,
			      const struct vmsg_bridge_uapi_map_event *event,
			      u32 ack_status)
{
	struct virtio_msg_bus_bridge_device *endpoint;

	if (!event)
		return -EINVAL;
	if (event->type != VMSG_BRIDGE_UAPI_MAP_EVENT_ADD_RESP &&
	    event->type != VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP &&
	    event->type != VMSG_BRIDGE_UAPI_MAP_EVENT_RELEASED)
		return 0;

	endpoint = session->endpoint;
	if (!endpoint || !endpoint->ops || !endpoint->ops->map_event_ack)
		return 0;

	return endpoint->ops->map_event_ack(endpoint, event, ack_status);
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
		pr_warn_ratelimited("virtio_msg bridge topology notify failed handle=%u dev_num=%u state=%u ret=%d\n",
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
			 new_present ? VIRTIO_MSG_BUS_EVENT_DEV_STATE_ADDED :
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
	old_bitmap = kcalloc(VMSG_BUS_TOPOLOGY_BITMAP_BYTES,
			     sizeof(*old_bitmap), GFP_KERNEL);
	new_bitmap = kcalloc(VMSG_BUS_TOPOLOGY_BITMAP_BYTES,
			     sizeof(*new_bitmap), GFP_KERNEL);
	if (!old_bitmap || !new_bitmap)
		goto fallback_no_bitmap;

	mutex_lock(&vmsg_bus_endpoint_topologies_lock);
	topology = vmsg_bus_endpoint_topology_lookup_locked(endpoint);
	if (topology) {
		vmsg_bus_topology_snapshot_locked
			(topology, old_bitmap,
			 VMSG_BUS_TOPOLOGY_BITMAP_BYTES);
		vmsg_bus_bridge_topology_clear_locked(topology);
		notify = true;
	}
	if (clear_endpoint_topology)
		RCU_INIT_POINTER(endpoint->topology, NULL);
	mutex_unlock(&vmsg_bus_endpoint_topologies_lock);

	if (notify)
		vmsg_bus_topology_notify_bitmap_diff
			(endpoint, old_bitmap, new_bitmap,
			 VMSG_BUS_TOPOLOGY_BITMAP_BYTES);
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
			RCU_INIT_POINTER(endpoint->topology, NULL);
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

static long vmsg_bus_unlocked_ioctl(struct file *file, unsigned int cmd,
				    unsigned long arg)
{
	switch (cmd) {
	case VMSG_BRIDGE_IOCTL_ATTACH:
		return vmsg_bus_ioctl_attach(file, arg);
	case VMSG_BRIDGE_IOCTL_DETACH:
		return vmsg_bus_ioctl_detach(file);
	case VMSG_BRIDGE_IOCTL_RESOLVE_ENDPOINT:
		return vmsg_bus_ioctl_resolve_endpoint(file, arg);
	case VMSG_BRIDGE_IOCTL_RELEASE_ENDPOINT:
		return vmsg_bus_ioctl_release_endpoint(file, arg);
	default:
		return -ENOTTY;
	}
}

static __poll_t vmsg_bus_poll(struct file *file, poll_table *wait)
{
	struct vmsg_bus_file_ctx *ctx = file->private_data;
	struct vmsg_bus_session *session;
	__poll_t mask = 0;

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

	if (READ_ONCE(session->ring_fault))
		mask |= EPOLLERR;

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
		pr_debug("mmap reject: ctx=%p vma=%p\n", ctx, vma);
		return -EINVAL;
	}

	mmap_len = (u64)(vma->vm_end - vma->vm_start);
	if (!mmap_len) {
		pr_debug("mmap reject: zero length pgoff=%lu\n", vma->vm_pgoff);
		return -EINVAL;
	}

	if ((u64)vma->vm_pgoff > (U64_MAX >> PAGE_SHIFT)) {
		pr_debug("mmap reject: pgoff overflow pgoff=%lu\n",
			 vma->vm_pgoff);
		return -EOVERFLOW;
	}
	mmap_offset = (u64)vma->vm_pgoff << PAGE_SHIFT;

	mutex_lock(&ctx->lock);
	session = ctx->session;
	if (!session) {
		pr_debug("mmap reject: no session offset=%#llx len=%#llx\n",
			 (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len);
		ret = -ENOTCONN;
		goto out_unlock_ctx;
	}

	mutex_lock(&session->map_lock);
	handle = session->endpoint ? session->endpoint->handle : 0;
	if (session->detached) {
		pr_debug("mmap reject: detached session handle=%u offset=%#llx len=%#llx\n",
			 handle, (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len);
		ret = -ENOTCONN;
		goto out_unlock_map;
	}

	endpoint = session->endpoint;
	if (!endpoint) {
		pr_debug("mmap reject: endpoint offline handle=%u offset=%#llx len=%#llx\n",
			 handle, (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len);
		ret = -ENODEV;
		goto out_unlock_map;
	}

	record = vmsg_bus_map_record_lookup_by_offset_locked(session, mmap_offset);
	if (!record || mmap_len > record->mmap_length ||
	    (record->state != VMSG_BUS_MAP_RECORD_ADD_PENDING &&
	     record->state != VMSG_BUS_MAP_RECORD_ACTIVE)) {
		pr_debug("mmap reject: handle=%u offset=%#llx len=%#llx record=%p map_id=%#llx record_len=%#llx state=%u\n",
			 handle, (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len, record,
			 record ? (unsigned long long)record->map_id : 0,
			 record ? (unsigned long long)record->mmap_length : 0,
			 record ? record->state : 0);
		ret = -ENOENT;
		goto out_unlock_map;
	}

	if (!endpoint->ops || !endpoint->ops->mmap) {
		pr_debug("mmap reject: endpoint mmap unsupported handle=%u endpoint=%p ops=%p\n",
			 handle, endpoint, endpoint ? endpoint->ops : NULL);
		ret = -EOPNOTSUPP;
		goto out_unlock_map;
	}

	mutex_unlock(&session->map_lock);
	ret = endpoint->ops->mmap(endpoint, mmap_offset, vma);
	if (ret)
		pr_debug("mmap endpoint failed: handle=%u offset=%#llx len=%#llx ret=%d\n",
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
			pr_warn_ratelimited("forced session drop endpoint=%u ret=%d\n",
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
