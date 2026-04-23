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

The bridge ABI covers endpoint resolution, attach or detach lifecycle,
userspace-published topology snapshots, userspace-reported request failures,
control-event delivery, map-event delivery, and shared TX or RX ring handoff.
Bridge map events are the only DMA-sharing path exposed to userspace: the
kernel publishes DMA-visible windows with ``ADD`` events and later retires them
with ``DEL_REQ`` events. Those windows can cover the initial virtqueue rings
and later transport-owned DMA mappings. Wire-level virtio-message payload
semantics are defined by the transport specification, not by this bridge ABI.

Interface surface
=================

Userspace works on one file descriptor returned by
``open("/dev/virtio-msg-bridge")``.

- ``ioctl()``: resolve or release endpoint handles, attach or detach one
  session, publish one topology snapshot, report one userspace-side transport
  request failure, receive control events, receive ordered map events, and
  acknowledge map events.
- ``mmap()``: map DMA-visible windows published by ``ADD`` events, using the
  ``mmap_offset`` and ``mmap_length`` carried by each event.
- ``poll()``/``epoll``: wait for control/map-event readability and RX-ring
  unread data readiness.
- ``eventfd``: userspace TX kick and kernel RX call notifications configured by
  ``ATTACH``.

There is no data-path ``read()``/``write()`` interface.
There is also no query-style shared-memory discovery ioctl. Userspace learns
every DMA-visible window only from ordered ``ADD``/``DEL_REQ`` map events.

Session and endpoint model
==========================

A session is bound to one file descriptor. ``ATTACH`` binds that session to one
resolved endpoint handle. ``DETACH`` (or ``close()``) removes the binding and
releases session resources.

One endpoint supports one attached session at a time. Concurrent second attach
returns ``-EBUSY``.

Endpoint unregister or register transitions do not force detach. If the
endpoint goes offline, the same attached session stays bound to the same
handle. Userspace keeps the session attached, stops data or map traffic, and
republishes full topology after the next ``ENDPOINT_ONLINE`` transition.

Endpoint addressing and handle lifecycle
========================================

Endpoints are addressed by:

- ``bus_name``: backend name (for example ``loopback`` or ``ffa``)
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

- Unknown ``bus_name`` returns ``-ENOENT`` (including with ``CREATE``).

Known-bus behavior:

- Known bus + malformed ``bus_id`` returns backend validation error
  (for current in-tree backends this is ``-EINVAL``).
- Known ``ffa`` bus + ``bus_id`` equal to local host partition ID returns
  ``-EPERM``.
- Known bus + unresolved valid endpoint:

  - no ``CREATE`` -> ``-ENODEV``
  - with ``CREATE`` -> handle is created and can later be attached

Attach snapshot and online/offline semantics
============================================

``ATTACH`` validates FDs, ring geometry, and backend caps; then binds the
session to ``endpoint_id``.

Userspace supplies these attach-time sizing inputs:

- ``attach.vmm_max_msg_size``: the VMM-side raw virtio-msg size ceiling for
  this attach epoch
- ``attach.tx.entries`` and ``attach.rx.entries``: both must be ``256``
- ``attach.tx.entry_size`` and ``attach.rx.entry_size``: both must equal
  ``attach.vmm_max_msg_size`` rounded up to the next ``128``-byte boundary
- ``attach.caps``: output-only storage, passed zero-initialized and filled by
  the kernel on success

``attach.caps`` is the authoritative initial endpoint-state snapshot:

- ``caps.flags & VMSG_BRIDGE_UAPI_CAP_F_ENDPOINT_ONLINE`` set:

  - endpoint is online at attach time
  - capability tuple is populated (revision/features and negotiated
    ``max_msg_size``)
  - ``caps.max_msg_size`` is the smaller of the attach-time
    ``vmm_max_msg_size`` and the current provider-reported endpoint limit

- ``caps.flags`` clear:

  - endpoint is offline at attach time
  - capability tuple is zeroed

This snapshot is the initial state source. Control events describe later
endpoint transitions.

Online or offline transition contract:

- If attach succeeds while endpoint is offline, ``ENDPOINT_ONLINE`` is emitted
  when the endpoint later becomes online for the same attach epoch.
- If attach succeeds while endpoint is already online, no immediate synthetic
  ``ENDPOINT_ONLINE`` is emitted; userspace uses the attach snapshot as the
  initial online state.
- ``ENDPOINT_ONLINE`` carries only ``endpoint_id``, the current provider
  ``bus_name``, and the current provider-reported endpoint ``max_msg_size``.
  It does not repeat revision or feature bits.
- Userspace republishes topology with ``SET_DEVICES`` after each
  ``ENDPOINT_ONLINE`` transition.
- ``ENDPOINT_OFFLINE`` means the endpoint left the online state for the current
  attach epoch. Userspace stops topology, map, and data activity until the next
  ``ENDPOINT_ONLINE`` transition.

TX/RX shared-ring cursor contract
=================================

The bridge data path uses two shared rings backed by ``ring_fd``:

- TX ring: userspace producer, kernel consumer.
- RX ring: kernel producer, userspace consumer.
- Each ring uses ``256`` entries in v1.
- Each ring slot size is derived from the attach-time
  ``vmm_max_msg_size``, rounded up to the next ``128``-byte boundary.

Raw data-path ownership:

- Backend code never reads or writes ring headers/slots directly.
- Userspace->backend raw dispatch is owned by bridge core TX-ring drain.
- Backend->userspace raw publication is queued through
  ``virtio_msg_bus_bridge_device_publish_rx()`` and emitted by bridge core
  RX-ring publication.

For both rings, ``struct vmsg_bridge_uapi_ring_hdr`` cursors follow this
contract:

- ``prod`` and ``cons`` are 32-bit monotonic cursors, not modulo indices.
- Producers and consumers increment their own cursor by exactly 1 per frame.
- Slot index is computed as ``cursor % entries``.
- Empty ring: ``prod == cons``.
- Full ring: ``prod - cons == entries`` (computed in 32-bit unsigned
  arithmetic).
- Valid occupancy range is ``0 .. entries``. If occupancy exceeds ``entries``,
  ring state is invalid/corrupted and processing is aborted.

Ordering requirements:

- Producer must fully initialize slot payload first, then publish cursor update
  to ``prod`` with release ordering.
- Consumer must read ``prod`` with acquire ordering before consuming slot
  payload, and only publish ``cons`` after slot processing is complete.

Practical userspace guidance:

- Do not write modulo-reduced values into ``prod``/``cons``.
- Do not reset cursors after wrap; natural ``u32`` wrap is part of the ABI.
- Use C11 atomics (or equivalent) for acquire/release updates of ring cursors.

Bridge executor and ``poll()`` contract
---------------------------------------

One ordered kernel executor runs per attached session. It owns TX-ring drain,
queued backend RX publication, and control/map wake signaling.

``poll()``/``epoll`` read readiness (``POLLIN|POLLRDNORM``) is asserted when
any of the following is true:

- control queue has at least one deliverable event
- map-event queue head is deliverable
- RX ring has unread entries (``prod != cons`` with valid occupancy)

``poll()`` error readiness:

- detached/invalidated session: ``POLLERR|POLLHUP``
- invalid/corrupted ring header/occupancy detected by bridge core: ``POLLERR``

Data path and map-event/mmap behavior while offline
===================================================

When attached endpoint is offline, bridge core rejects endpoint operations with
``-ENODEV`` (without calling backend callbacks):

- TX data path
- ``MAP_EVENT_RECV``
- ``MAP_EVENT_ACK``
- ``mmap()`` on bridge mapping tokens

Kernel performs no retry loops for endpoint availability. Userspace should
continue from control-event transitions. No new DMA-visible windows are
published while the endpoint is offline; userspace waits for the next
``ENDPOINT_ONLINE`` transition, republishes topology, and then resumes map and
data traffic.

Transport-only ring contract
============================

The bridge TX and RX rings carry only transport-class virtio-msg frames.
Bus-class traffic does not traverse the rings.

Rules:

- Any frame with ``msg.type & VIRTIO_MSG_TYPE_BUS`` is invalid on the bridge
  rings.
- Userspace must not place bus-class frames on the TX ring. Bridge core rejects
  them as invalid ring traffic.
- Bridge core never publishes bus-class frames on the RX ring.
- Transport frames must target a ``dev_num`` that is currently present in the
  topology snapshot published through ``SET_DEVICES``.
- Transport ``dev_num`` 0 is valid if userspace published it in the live
  topology snapshot.

Bridge userspace ABI for non-transport state:

- topology publication uses ``SET_DEVICES``
- endpoint online or offline transitions use ``CONTROL_RECV``
- DMA visibility changes use map-event delivery and ``MAP_EVENT_ACK``

Topology publication model
==========================

``SET_DEVICES`` is the only topology publication ioctl. It applies one bitmap
snapshot window over endpoint ``dev_num`` space.

Rules:

- session must be attached
- ``endpoint_id`` must match session-bound handle
- endpoint must remain online
- userspace publishes the full current topology snapshot after each
  ``ENDPOINT_ONLINE`` transition
- ``VMSG_BRIDGE_UAPI_SET_DEVICES_F_CLEAR`` clears the live snapshot before the
  supplied window is applied, which allows chunked full-snapshot publication

If ``SET_DEVICES`` is issued on an attached but offline endpoint, it fails with
``-ENODEV``.

Userspace request-failure reporting
===================================

``VMSG_BRIDGE_IOCTL_MSG_ERROR`` lets userspace fail one outstanding transport
request when it cannot produce a valid transport response on the TX ring.

Rules:

- The ioctl is valid only for transport requests. Events, responses, and
  bus-class frames do not use it.
- The report is scoped to the same attached file descriptor session that owns
  the outstanding request relay entry.
- Userspace must fill ``struct vmsg_bridge_uapi_msg_error`` with the original
  ``dev_num``, ``token``, and ``msg_id`` from the request and a negative Linux
  errno in ``error``.
- ``flags`` and all reserved fields must be zero in v1.
- If userspace can produce the normal protocol response, it should do that on
  the TX ring instead of using ``MSG_ERROR``.
- Bridge core correlates the report against the attached session's outstanding
  relay entry by ``(dev_num, token)`` and then validates ``msg_id``.

Typical uses:

- the request targets a transport ``dev_num`` that is not currently present or
  cannot be served by userspace
- the request payload, opcode, or state is invalid for the addressed device and
  userspace cannot produce a valid transport response
- userspace hits a local processing failure while handling the request
- userspace must reject the request because completing it would violate local
  device or transport invariants

Bridge-core correlation rules:

- unknown outstanding request key returns ``-ENOENT``
- mismatched ``msg_id`` for the same ``(dev_num, token)`` returns ``-EPROTO``
- malformed reports return ``-EINVAL``

Attach-then-topology bootstrap sequence
=======================================

The intended bridge bootstrap order is:

1. Resolve ``(bus_name, bus_id)`` to one endpoint handle.
2. Attach shared rings and notification eventfds.
3. If the attach snapshot says online, publish available ``dev_num`` values
   with ``SET_DEVICES``.
4. Start consuming ordered map events. Initial ``ADD`` events typically publish
   the DMA-visible windows needed to bootstrap the endpoint, such as virtqueue
   rings.
5. Wait for kernel-originated raw transport requests on the RX ring, answer
   them on the TX ring, and continue processing later ``ADD``/``DEL_REQ`` map
   traffic for transport-owned DMA buffers.

Userspace owns device-number publication. Kernel transport bootstrap does not
invent ``dev_num`` values or derive them from endpoint capabilities. A bridge
peer publishes topology first, then the loopback frontend registers the
transport device for each ready ``dev_num`` and emits ``GET_DEVICE_INFO`` for
that published number. Bridge map events remain the only userspace-visible DMA
publication channel for the rings and buffers owned by those transport devices.

Control-event queue ABI
=======================

Control queue is FIFO with monotonic ``seq`` values.

- ``CONTROL_RECV`` returns current head and retires it from the queue.
- There is no ``CONTROL_ACK`` ioctl.
- ``control_msg.flags`` is reserved and must be zero.

Non-blocking behavior:

- ``CONTROL_RECV`` on empty queue with ``O_NONBLOCK`` returns ``-EAGAIN``.

Event types and payloads
------------------------

The public control-event contract contains only bridge-generated endpoint
transitions:

- ``VMSG_BRIDGE_UAPI_CTRL_TYPE_ENDPOINT_ONLINE``
  with ``struct vmsg_bridge_uapi_ctrl_endpoint_online``
  carrying ``endpoint_id``, ``bus_name``, and the provider-reported
  ``max_msg_size``
- ``VMSG_BRIDGE_UAPI_CTRL_TYPE_ENDPOINT_OFFLINE``
  with ``struct vmsg_bridge_uapi_ctrl_endpoint_offline``

Lifecycle state machine for VMMs
================================

Recommended loop per endpoint handle:

1. Resolve handle with ``RESOLVE_ENDPOINT``.
2. ``ATTACH`` once and read ``caps.flags`` snapshot.
3. If snapshot says online, publish topology with ``SET_DEVICES`` and start
   data or map flows.
4. If snapshot says offline, keep the session attached and wait on the control
   queue.
5. On ``ENDPOINT_ONLINE`` transition, republish full topology with
   ``SET_DEVICES`` and then enable data or map flows.
6. On ``ENDPOINT_OFFLINE`` transition, stop data or map flows and wait for the
   next ``ENDPOINT_ONLINE``.
7. ``DETACH`` and ``RELEASE_ENDPOINT`` when done.

Retry policy:

- do not busy-loop retries in kernel-facing ioctls
- userspace should block/poll on control queue and react to transitions

Errno model and caller actions
==============================

.. list-table:: errno matrix
   :header-rows: 1

   * - errno
     - Typical meaning
     - Caller action
   * - ``-EINVAL``
     - Invalid flags/reserved fields/alignment/ranges, or malformed
       ``MSG_ERROR`` report
     - Fix request construction
   * - ``-ENOENT``
     - Unknown bus, missing entry, or no matching outstanding request for
       ``MSG_ERROR``
     - Fix address or state
   * - ``-ENODEV``
     - Valid handle unresolved without create, or endpoint offline runtime path
     - Wait for availability/lifecycle transition
   * - ``-ENOTCONN``
     - Session detached
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
     - Correlation mismatch, for example ``MSG_ERROR`` reported with the wrong
       ``msg_id`` for the outstanding ``(dev_num, token)``
     - Fix userspace request tracking
   * - ``-EAGAIN``
     - Nonblocking receive with empty queue
     - Poll and retry
   * - ``-ENOSPC``
     - Control or map queue full on producer side
     - Increase consume/ack rate; retry

Practical flows
===============

Online topology publication:

1. Open bridge device.
2. Resolve one endpoint handle.
3. Size both rings for ``256`` entries with slot size derived from the chosen
   ``vmm_max_msg_size``, then ``ATTACH`` once and read ``caps.flags``.
4. If online, publish one full snapshot with ``SET_DEVICES``.
5. Begin transport traffic only after topology publication completes.

Transport request handling:

1. Read one kernel-originated transport request from the RX ring.
2. Track the request ``dev_num``, ``token``, and ``msg_id`` until completion.
3. If userspace can generate the normal protocol response, write that response
   on the TX ring.
4. If userspace cannot generate a valid transport response, issue
   ``VMSG_BRIDGE_IOCTL_MSG_ERROR`` on the same attached bridge fd with the
   original request correlation tuple and a negative errno.

Control loop:

1. ``poll()`` for readable bridge fd.
2. ``CONTROL_RECV``.
3. Process event by ``type`` and payload.
4. Repeat.

DMA-visible windows:

1. Attach and publish topology for an online endpoint.
2. ``MAP_EVENT_RECV`` and inspect ``event.type``, ``event.bus_addr``,
   ``event.length``, ``event.mmap_offset``, and ``event.mmap_length``.
3. On ``ADD``, record the event metadata and ``mmap()`` the published window
   with ``event.mmap_offset`` and a length not exceeding
   ``event.mmap_length``.
4. Use the mapping for the transport-owned DMA window identified by that
   event. Early ``ADD`` traffic can publish virtqueue rings; later ``ADD``
   traffic can publish other DMA buffers created by the transport/provider
   stack.
5. ``MAP_EVENT_ACK`` each delivered map event in sequence after userspace has
   accepted or rejected it.
6. On ``DEL_REQ``, stop using the window, ``munmap()`` any live alias, retire
   local bookkeeping for that published range, and ACK the event.
