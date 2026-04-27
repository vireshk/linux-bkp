.. SPDX-License-Identifier: GPL-2.0
.. Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.

=================================
Virtio-message bridge userspace API
=================================

Overview
========

The virtio-message bridge provides a character-device interface for a VMM or
userspace device process.

- Device node: ``/dev/virtio-msg-bridge``
- UAPI header: ``include/uapi/linux/virtio_msg_bus_bridge.h``
- Bridge-local protocol definitions: ``include/uapi/linux/virtio_msg_bus_bridge.h``

The bridge ABI covers endpoint resolution, attach or detach lifecycle, shared
TX/RX rings, and ``mmap()`` of DMA-visible windows. Runtime control is carried
as ordered bridge-local bus messages on the same TX/RX rings as ordinary
transport-class virtio-message frames.

Interface surface
=================

Userspace works on one file descriptor returned by
``open("/dev/virtio-msg-bridge")``.

- ``ioctl()``: ``RESOLVE_ENDPOINT``, ``RELEASE_ENDPOINT``, ``ATTACH``, and
  ``DETACH`` remain synchronous operations.
- ``mmap()``: map DMA-visible windows on the attached bridge file descriptor,
  using bridge-local map-message ``mmap_offset`` and ``mmap_length`` values.
- ``eventfd``: userspace TX kick and kernel RX call notifications configured by
  ``ATTACH``.

There is no data-path ``read()``/``write()`` interface. Runtime messages are
delivered only through the shared rings, with kernel-to-userspace runtime
readiness signaled through ``call_fd``.

Session and endpoint model
==========================

A session is bound to one file descriptor. ``ATTACH`` binds that session to one
resolved endpoint handle. ``DETACH`` or ``close()`` removes the binding and
releases session resources.

One endpoint supports one attached session at a time. Concurrent second attach
returns ``-EBUSY``.

Endpoint unregister or register transitions do not force detach. If the
endpoint goes offline, the same attached session stays bound to the same handle.
Userspace keeps the session attached, stops endpoint-scoped traffic, and waits
for the next ``ENDPOINT_ONLINE`` bridge-local ring event or detaches
voluntarily.

Endpoint addressing and handle lifecycle
========================================

Endpoints are addressed by:

- ``bus_name``: backend name, for example ``loopback`` or ``ffa``
- ``bus_id``: bridge-driver-defined identifier string

Userspace resolves ``(bus_name, bus_id)`` to a numeric handle via
``VMSG_BRIDGE_IOCTL_RESOLVE_ENDPOINT`` and later drops references via
``VMSG_BRIDGE_IOCTL_RELEASE_ENDPOINT``.

``RESOLVE_ENDPOINT`` rules
--------------------------

- ``addr.bus_name`` must be non-empty and NUL-terminated.
- ``addr.bus_id`` must be NUL-terminated and zero-padded after NUL.
- ``addr.bus_id`` parsing is owned by the selected bus backend.
- FF-A accepts both decimal and ``0x`` hexadecimal VM-ID strings; bridge core
  stores the backend-canonicalized form.
- Only ``VMSG_BRIDGE_UAPI_RESOLVE_F_CREATE`` is valid in ``flags``.
- ``reserved[]`` must be zero.
- Success returns a non-zero ``handle``.

Strict unknown-bus behavior:

- Unknown ``bus_name`` returns ``-ENOENT``, including with ``CREATE``.

Known-bus behavior:

- Known bus plus malformed ``bus_id`` returns the backend validation error.
- Known ``ffa`` bus plus ``bus_id`` equal to the local host partition ID
  returns ``-EPERM``.
- Known bus plus unresolved valid endpoint:

  - no ``CREATE`` returns ``-ENODEV``
  - with ``CREATE`` creates a handle that can later be attached

Attach snapshot
===============

``ATTACH`` validates FDs, ring geometry, and backend caps; then binds the
session to ``endpoint_id``.

Userspace supplies these attach-time sizing inputs:

- ``attach.vmm_max_msg_size``: the VMM-side raw virtio-msg size ceiling for
  this attach
- ``attach.tx.entries`` and ``attach.rx.entries``: both must be ``256``
- ``attach.tx.entry_size`` and ``attach.rx.entry_size``: both must equal
  ``attach.vmm_max_msg_size`` rounded up to the next ``128``-byte boundary
- ``attach.caps``: output-only storage, passed zero-initialized and filled by
  the kernel on success

``attach.vmm_max_msg_size`` is the raw frame limit before ring-slot alignment.
It must be in the transport range and must be large enough to carry a raw
``VIRTIO_MSG_BUS_BRIDGE_MAP_ADD`` frame: ``struct virtio_msg`` plus
``struct virtio_msg_bus_bridge_map_add_req``. The current bridge-local minimum
is 72 bytes.

``attach.caps`` is the authoritative initial endpoint-state snapshot:

- ``caps.flags & VMSG_BRIDGE_UAPI_CAP_F_ENDPOINT_ONLINE`` set:

  - endpoint is online at attach time
  - ``caps.revision``, ``caps.transport_features``, and
    ``caps.bridge_features`` describe the endpoint
  - ``caps.max_msg_size`` is the negotiated raw frame size shared by
    transport-class frames and bridge-local bus frames
  - ``caps.online_epoch`` is non-zero and identifies the initial online epoch

- ``caps.flags`` clear:

  - endpoint is offline at attach time
  - endpoint capability fields are zeroed
  - ``caps.online_epoch`` is zero

If attach succeeds while endpoint is already online, no synthetic
``ENDPOINT_ONLINE`` ring event follows. Userspace uses the attach snapshot as
the initial online state and publishes topology for ``caps.online_epoch`` on the
TX ring.

TX/RX shared-ring contract
==========================

The bridge data path uses two shared rings backed by ``ring_fd``:

- TX ring: userspace producer, kernel consumer.
- RX ring: kernel producer, userspace consumer.
- Each ring uses ``256`` entries.
- Each ring slot size is derived from the attach-time
  ``vmm_max_msg_size``, rounded up to the next ``128``-byte boundary.

Each slot carries exactly one raw ``struct virtio_msg`` frame. A slot may carry
either:

- a transport-class virtio-message frame, or
- a bridge-local implementation-defined bus message.

``caps.max_msg_size`` is the negotiated maximum raw frame size for both classes
of frames. Slot size may be larger because it is aligned for ring geometry.

For both rings, ``struct vmsg_bridge_uapi_ring_hdr`` cursors follow this
contract:

- ``prod`` and ``cons`` are 32-bit monotonic cursors, not modulo indices.
- Producers and consumers increment their own cursor by exactly 1 per frame.
- Slot index is computed as ``cursor % entries``.
- Empty ring: ``prod == cons``.
- Full ring: ``prod - cons == entries`` computed in 32-bit unsigned arithmetic.
- Valid occupancy range is ``0 .. entries``. If occupancy exceeds ``entries``,
  ring state is invalid and the session is faulted.

Ordering requirements:

- Producer must fully initialize slot payload first, then publish cursor update
  to ``prod`` with release ordering.
- Consumer must read ``prod`` with acquire ordering before consuming slot
  payload, and only publish ``cons`` after slot processing is complete.
- Receivers process entries strictly in ring order.
- A bridge-local bus message is fully applied before any later slot is
  processed.

Bus-message handling on bridge rings
====================================

The bridge uses implementation-defined bus IDs for runtime bridge state:

- ``VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_ONLINE``
- ``VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_OFFLINE``
- ``VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_RESET``
- ``VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_DEV_ADD``
- ``VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_COMMIT``
- ``VIRTIO_MSG_BUS_BRIDGE_MAP_ADD``
- ``VIRTIO_MSG_BUS_BRIDGE_MAP_DEL``
- ``VIRTIO_MSG_BUS_BRIDGE_MAP_RELEASED``
- ``VIRTIO_MSG_BUS_BRIDGE_ERROR``

Standardized bus messages such as ``GET_DEVICES``, ``PING``, and
``EVENT_DEVICE`` are not part of the bridge ring protocol. Reserved bus-message
IDs are also not part of the bridge ring protocol. Receivers discard these
messages without forwarding them to generic virtio-message transport or
provider paths.

Malformed virtio-message headers and malformed bridge-local payload sizes fault
the attached session.

Online epoch
============

The bridge uses a session-scoped ``online_epoch`` generated by Linux.

- The epoch is zero while attached offline.
- Linux increments the epoch before each transition to online.
- Linux includes ``online_epoch`` in every runtime bridge event that belongs to
  a specific online epoch.
- Userspace includes ``online_epoch`` in every runtime bridge event it sends
  back to Linux.
- A message for an older epoch is stale and is ignored.
- A message for a future epoch is a protocol violation and faults the session.

Topology publication
====================

Userspace publishes a full topology snapshot for each online epoch using TX
ring bridge-local bus messages:

1. ``TOPOLOGY_RESET(online_epoch)``
2. zero or more ``TOPOLOGY_DEV_ADD(online_epoch, dev_num)`` messages
3. ``TOPOLOGY_COMMIT(online_epoch, num_devs)``

Linux stages the snapshot while consuming those slots and transitions the
session to data-active only when it consumes ``TOPOLOGY_COMMIT`` for the current
epoch.

Userspace must not enqueue transport TX frames that depend on topology before
the matching topology commit has been enqueued.

DMA map lifecycle
=================

Linux publishes DMA map lifecycle requests on the RX ring:

- ``MAP_ADD`` makes a DMA-visible window available to userspace.
- ``MAP_DEL`` requests revocation of a previously published mapping.

``MAP_ADD`` and ``MAP_DEL`` use normal virtio-message bus request/response
headers. Requests have ``dev_num = 0`` and a Linux-generated ``token``.
Responses use ``dev_num = 0``, the same ``msg_id`` as the request, and the
matching request ``token``. A non-zero response status is a signed negative
Linux errno.

Userspace maps a ``MAP_ADD`` window with
``mmap(dev_fd, mmap_offset, ...)`` on the attached bridge file descriptor. The
mapping length must not exceed the published ``mmap_length``. The request
payload includes ``online_epoch``, ``map_seq``, ``map_id``, ``bus_addr``,
``length``, ``mmap_offset``, ``mmap_length``, and ``flags``.

``map_seq`` identifies one publication instance of a map record. Linux uses it
with ``online_epoch`` and ``map_id`` to ignore stale responses or events and to
prevent ABA reuse when the same logical mapping record is later republished.

``VIRTIO_MSG_BUS_BRIDGE_MAP_F_RETENTION_REQUESTED`` is a retention hint. Linux
sets it when keeping the userspace mapping around after it becomes idle would
avoid later ``mmap()`` and setup cost. It is not a hard promise from userspace:
userspace may still release the mapping and then report that release with
``MAP_RELEASED``.

Userspace completes ``MAP_ADD`` by mapping the provider window and then
enqueueing the matching ``MAP_ADD`` response on the TX ring. On success, the
window is visible to later transport frames ordered after the request. On
failure, Linux treats the map publication as failed and completes the dependent
provider operation through provider-specific cleanup or fault handling.

Userspace, such as QEMU, completes ``MAP_DEL`` only after visibility has been
revoked, references have drained, and the provider ``mmap()`` has been removed.
The matching ``MAP_DEL`` response tells Linux that the mapping may be reclaimed
and that userspace will not access the DMA-visible range again through that
publication instance.

``MAP_RELEASED`` is a userspace-to-Linux event, not a response. For QEMU, this
is a QEMU-to-Linux event. It has ``dev_num = 0`` and ``token = 0`` and carries
``online_epoch``, ``map_seq``, ``map_id``, ``bus_addr``, and ``length``. The
normal use is idle release of non-retained temporary mappings. For retained
mappings, ``MAP_RELEASED`` is reserved for exceptional cleanup or fault paths.
If userspace releases an active retained mapping, Linux removes the affected
virtio device; Linux escalates to endpoint removal only when it cannot identify
the owning device.

Linux may publish multiple map requests on the RX ring before earlier requests
have completed. Userspace consumes map requests in RX ring order and must
enqueue matching responses in the same order. Userspace must enqueue the
matching response before it enqueues any later transport TX frame that depends
on the new map state.

For FF-A-backed endpoints, an ``AREA_SHARE`` success may be returned before the
later userspace ``MAP_ADD`` response. The bridge FIFO provides the required
ordering: Linux queues the ``MAP_ADD`` before dependent transport traffic, and
userspace processes the FIFO strictly in order before observing later frames
that depend on the mapping.

Transport request failure
=========================

If userspace consumes a transport request that expected a response and cannot
produce a valid transport response, it enqueues ``BRIDGE_ERROR`` on the TX
ring.

``BRIDGE_ERROR`` is a response-class bus message:

- ``type = VIRTIO_MSG_TYPE_BUS | VIRTIO_MSG_TYPE_RESPONSE``
- ``msg_id = VIRTIO_MSG_BUS_BRIDGE_ERROR``
- ``dev_num = 0``
- ``token`` copied from the failing transport request

The payload echoes the original transport ``dev_num`` and ``msg_id`` and
contains a signed negative Linux errno. Linux correlates the error response to
the pending transport request and no additional response follows.

Lifecycle flow
==============

Recommended loop per endpoint handle:

1. Resolve handle with ``RESOLVE_ENDPOINT``.
2. ``ATTACH`` once and read ``caps.flags`` and ``caps.online_epoch``.
3. If the attach snapshot says online, publish a full topology snapshot on the
   TX ring for ``caps.online_epoch``.
4. Start transport and map flows only after topology publication is enqueued.
5. On ``ENDPOINT_OFFLINE``, stop endpoint-scoped data and map activity and
   discard old-epoch state.
6. On ``ENDPOINT_ONLINE``, publish a new full topology snapshot for that event
   epoch before resuming transport traffic.
7. ``DETACH`` and ``RELEASE_ENDPOINT`` when done.

Errno model and caller actions
==============================

.. list-table:: errno matrix
   :header-rows: 1

   * - errno
     - Typical meaning
     - Caller action
   * - ``-EINVAL``
     - Invalid flags/reserved fields/alignment/ranges, malformed attach sizing,
       or malformed synchronous ioctl request
     - Fix request construction
   * - ``-ENOENT``
     - Unknown bus or missing entry
     - Fix address or state
   * - ``-ENODEV``
     - Valid handle unresolved without create, or endpoint unavailable
     - Wait for availability/lifecycle transition
   * - ``-ENOTCONN``
     - Session detached or invalidated
     - Reattach
   * - ``-EBUSY``
     - Endpoint already attached by another session
     - Retry after owner detaches
   * - ``-EPERM``
     - Session ownership mismatch, or ``ffa`` host-self endpoint target
     - Use correct owner session/endpoint handle, or a non-self ``ffa`` target
   * - ``-EOPNOTSUPP``
     - Feature not negotiated or backend unsupported
     - Disable unsupported path
   * - ``-EPROTO``
     - Protocol mismatch or malformed ring state
     - Fault and recreate the session
   * - ``-EAGAIN``
     - Nonblocking path has no immediately available work
     - Wait for notification and retry
   * - ``-ENOSPC``
     - Queue or ring temporarily full on producer side
     - Drain the matching consumer side and retry
