# Cotton HostController Trait & RP2040 Implementation Reference

This document captures the full patterns from `cotton-usb-host` that must be replicated
for the i.MX RT 1062 implementation.

## 1. HostController Trait (host_controller.rs)

```rust
pub trait HostController {
    type InterruptPipe: Stream<Item = InterruptPacket> + Unpin;
    type DeviceDetect: Stream<Item = DeviceStatus>;

    fn device_detect(&self) -> Self::DeviceDetect;
    fn reset_root_port(&self, rst: bool);

    fn control_transfer(
        &self, address: u8, transfer_extras: TransferExtras,
        packet_size: u8, setup: SetupPacket, data_phase: DataPhase<'_>,
    ) -> impl Future<Output = Result<usize, UsbError>>;

    fn bulk_in_transfer(
        &self, address: u8, endpoint: u8, packet_size: u16,
        data: &mut [u8], transfer_type: TransferType, data_toggle: &Cell<bool>,
    ) -> impl Future<Output = Result<usize, UsbError>>;

    fn bulk_out_transfer(
        &self, address: u8, endpoint: u8, packet_size: u16,
        data: &[u8], transfer_type: TransferType, data_toggle: &Cell<bool>,
    ) -> impl Future<Output = Result<usize, UsbError>>;

    fn alloc_interrupt_pipe(
        &self, address: u8, transfer_extras: TransferExtras,
        endpoint: u8, max_packet_size: u16, interval_ms: u8,
    ) -> impl Future<Output = Self::InterruptPipe>;

    fn try_alloc_interrupt_pipe(
        &self, address: u8, transfer_extras: TransferExtras,
        endpoint: u8, max_packet_size: u16, interval_ms: u8,
    ) -> Result<Self::InterruptPipe, UsbError>;
}
```

## 2. Key Types

### UsbError
```rust
pub enum UsbError {
    Stall,
    Timeout,
    Overflow,
    BitStuffError,
    CrcError,
    DataSeqError,
    BufferTooSmall,
    AllPipesInUse,
    ProtocolError,
    TooManyDevices,
    NoSuchEndpoint,
}
```

### DeviceStatus / UsbSpeed
```rust
pub enum DeviceStatus {
    Present(UsbSpeed),
    Absent,
}

pub enum UsbSpeed {
    Low1_5,
    Full12,
    High480,
}
```

### TransferExtras / DataPhase / TransferType
```rust
pub enum TransferExtras {
    Normal,
    WithPreamble,  // For low-speed devices behind full-speed hubs
}

pub enum DataPhase<'a> {
    In(&'a mut [u8]),
    Out(&'a [u8]),
    None,
}

pub enum TransferType {
    FixedSize,
    VariableSize,
}
```

### InterruptPacket
```rust
pub struct InterruptPacket {
    pub address: u8,
    pub endpoint: u8,
    pub size: u8,
    pub data: [u8; 64],
}

impl Deref for InterruptPacket {
    type Target = [u8];
    fn deref(&self) -> &Self::Target {
        &self.data[0..(self.size as usize)]
    }
}
```

### SetupPacket (from wire.rs)
```rust
#[repr(C)]
pub struct SetupPacket {
    pub bmRequestType: u8,
    pub bRequest: u8,
    pub wValue: u16,
    pub wIndex: u16,
    pub wLength: u16,
}
```

### Direction / EndpointType (from wire.rs)
```rust
pub enum Direction { In, Out }
pub enum EndpointType { Control = 0, Isochronous = 1, Bulk = 2, Interrupt = 3 }
```

## 3. RP2040 Architecture Pattern

### 3.1 Three-struct split

```rust
// Shared between ISR and async task — must be 'static
pub struct UsbShared {
    device_waker: CriticalSectionWakerRegistration,
    pipe_wakers: [CriticalSectionWakerRegistration; 16],
}

// Static pools — must be 'static but not shared with ISR
pub struct UsbStatics {
    bulk_pipes: Pool,      // Pool::new(15) — 15 bulk/interrupt pipes
    control_pipes: Pool,   // Pool::new(1)  — 1 control pipe
}

// The actual HostController — borrows 'static refs + owns HW registers
pub struct Rp2040HostController {
    shared: &'static UsbShared,
    statics: &'static UsbStatics,
    regs: pac::USBCTRL_REGS,     // Owned PAC register block
    dpram: pac::USBCTRL_DPRAM,   // Owned PAC register block
}
```

All three are `const fn new()` constructible — placed in `static` storage.

### 3.2 ISR pattern (on_irq)

The ISR in UsbShared::on_irq():

1. Reads interrupt status (`ints`)
2. For buffer status bits: wakes the corresponding pipe waker
3. For connection events: clears the interrupt flag, wakes device_waker
4. For transaction/error events (bits 0x458): wakes pipe_waker[0] (control pipe)
5. **CRITICAL**: Disables remaining interrupts to prevent IRQ storm:
   ```rust
   let bits = regs.ints().read().bits();
   regs.inte().modify(|r, w| w.bits(r.bits() & !bits));
   ```

### 3.3 Re-enable-on-poll pattern

Every poll function follows this pattern:
1. Register the waker
2. Check if the condition is met (register read)
3. If ready: re-enable relevant interrupts, return `Poll::Ready`
4. If not ready: re-enable relevant interrupts, return `Poll::Pending`

The interrupt is re-enabled in BOTH paths (ready and pending) so that
the next event will fire.

Example from DeviceDetect:
```rust
fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
    self.waker.register(cx.waker());
    let status = regs.sie_status().read();
    let device_status = match status.speed().bits() { ... };
    if device_status != self.status {
        regs.inte().modify(|_, w| w.host_conn_dis().set_bit());  // re-enable
        self.status = device_status;
        Poll::Ready(Some(device_status))
    } else {
        regs.inte().modify(|_, w| w.host_conn_dis().set_bit());  // re-enable
        Poll::Pending
    }
}
```

Example from ControlEndpoint:
```rust
fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
    self.waker.register(cx.waker());
    let status = regs.sie_status().read();
    let intr = regs.intr().read();
    if (intr.bits() & 0x458) != 0 {  // transaction complete or error
        regs.sie_status().write(|w| unsafe { w.bits(0xFF08_0000) });
        Poll::Ready(status)
    } else {
        regs.sie_status().write(|w| unsafe { w.bits(0xFF08_0000) });
        regs.inte().modify(|_, w| {
            w.stall().set_bit()
             .error_rx_timeout().set_bit()
             .trans_complete().set_bit()
        });
        Poll::Pending
    }
}
```

### 3.4 Pool-based pipe allocation

```rust
// Pipe wraps a Pool allocation + offset
struct Pipe {
    _pooled: Pooled<'static>,  // Drop glue returns to pool
    which: u8,
}

// Allocation from pool
async fn alloc_pipe(&self, endpoint_type: EndpointType) -> Pipe {
    if endpoint_type == EndpointType::Control {
        Pipe::new(self.statics.control_pipes.alloc().await, 0)
    } else {
        Pipe::new(self.statics.bulk_pipes.alloc().await, 1)
    }
}
```

Pool is a simple async resource allocator:
- `Pool::new(n)` — creates pool with n resources (0..n-1)
- `pool.alloc().await` — returns `Pooled` (waits if none available)
- `pool.try_alloc()` — returns `Option<Pooled>` (immediate)
- When `Pooled` is dropped, resource returns to pool and wakes waiters

### 3.5 DeviceDetect stream

```rust
pub struct Rp2040DeviceDetect {
    waker: &'static CriticalSectionWakerRegistration,
    status: DeviceStatus,  // tracks last-reported status to detect changes
}

impl Stream for Rp2040DeviceDetect {
    type Item = DeviceStatus;
    fn poll_next(...) -> Poll<Option<Self::Item>> {
        self.waker.register(cx.waker());
        // Read current hardware state
        let device_status = match regs.sie_status().read().speed().bits() {
            0 => DeviceStatus::Absent,
            1 => DeviceStatus::Present(UsbSpeed::Low1_5),
            _ => DeviceStatus::Present(UsbSpeed::Full12),
        };
        // Only yield when status CHANGES
        if device_status != self.status {
            regs.inte().modify(|_, w| w.host_conn_dis().set_bit());
            self.status = device_status;
            Poll::Ready(Some(device_status))
        } else {
            regs.inte().modify(|_, w| w.host_conn_dis().set_bit());
            Poll::Pending
        }
    }
}
```

Key pattern: Stores previous status and only returns `Ready` on state change.

### 3.6 reset_root_port

```rust
fn reset_root_port(&self, rst: bool) {
    if rst {
        self.regs.sie_ctrl().modify(|_, w| w.reset_bus().set_bit());
    }
    // SIE_CTRL.RESET_BUS clears itself when done
}
```

Note: The RP2040 `reset_bus` bit auto-clears. The `rst: bool` parameter is
called with `true` to start reset, then `false` to end it (but RP2040 only
acts on `true`). For EHCI, we'd set/clear PORTSC1.PR.

### 3.7 control_transfer

The RP2040 control_transfer follows USB 2.0 section 8.5.3:

```rust
async fn control_transfer(
    &self, address: u8, transfer_extras: TransferExtras,
    packet_size: u8, setup: SetupPacket, data_phase: DataPhase<'a>,
) -> Result<usize, UsbError> {
    // 1. Allocate a control pipe (waits if busy)
    let _pipe = self.alloc_pipe(EndpointType::Control).await;

    // 2. Send SETUP packet
    self.send_setup(address, transfer_extras, &setup).await?;

    // 3. Data + Status phases depend on direction
    match data_phase {
        DataPhase::In(buf) => {
            let sz = self.control_transfer_in(address, packet_size,
                setup.wLength as usize, buf).await?;
            // Status phase (OUT zero-length)
            self.control_transfer_out(address, packet_size, 0, &[]).await?;
            Ok(sz)
        }
        DataPhase::Out(buf) => {
            let sz = self.control_transfer_out(address, packet_size,
                setup.wLength as usize, buf).await?;
            // Status phase (IN zero-length)
            self.control_transfer_in(address, packet_size, 0, &mut []).await?;
            Ok(sz)
        }
        DataPhase::None => {
            // Status phase only (IN zero-length)
            self.control_transfer_in(address, packet_size, 0, &mut []).await
        }
    }
}
```

The SETUP + DATA + STATUS phases are all separate async operations.
For EHCI, we'll chain these as qTDs in a single async schedule entry.

### 3.8 InterruptPipe stream

```rust
pub struct Rp2040InterruptPipe {
    shared: &'static UsbShared,
    pipe: Pipe,              // Owns the pipe allocation (RAII)
    max_packet_size: u16,
    data_toggle: Cell<bool>, // Tracks DATA0/DATA1 toggle
}

impl Stream for Rp2040InterruptPipe {
    type Item = InterruptPacket;
    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        self.set_waker(cx.waker());
        if let Some(packet) = self.poll() {
            Poll::Ready(Some(packet))
        } else {
            Poll::Pending
        }
    }
}
```

When `Rp2040InterruptPipe` is dropped:
1. `Pipe` is dropped, which drops `Pooled`
2. `Pooled::drop()` returns the resource to the pool
3. Pool wakes any waiters

### 3.9 Bulk transfers

Bulk transfers use the same `control_transfer_inner()` mechanism as control
transfers (the RP2040 hardware uses endpoint 0 for everything). The key
differences:
- Endpoint number can be non-zero
- Data toggle is tracked via the `Cell<bool>` parameter
- No SETUP phase
- Uses the endpoint type field to select Bulk

```rust
async fn bulk_in_transfer(
    &self, address: u8, endpoint: u8, packet_size: u16,
    data: &mut [u8], transfer_type: TransferType, data_toggle: &Cell<bool>,
) -> Result<usize, UsbError> {
    let _pipe = self.alloc_pipe(EndpointType::Control).await;
    let mut packetiser = InPacketiser::new(
        data.len() as u16, packet_size as u16,
        data_toggle.get(),
        match transfer_type {
            TransferType::FixedSize => ZeroLengthPacket::Never,
            TransferType::VariableSize => ZeroLengthPacket::AsNeeded,
        },
    );
    let length = data.len() as u16;
    let mut depacketiser = InDepacketiser::new(length, data);
    self.control_transfer_inner(
        address, endpoint, packet_size as u8,
        Direction::In, length as usize,
        &mut packetiser, &mut depacketiser,
    ).await?;
    data_toggle.set(data_toggle.get() ^ depacketiser.packet_parity);
    Ok(depacketiser.total())
}
```

## 4. Mapping to i.MX RT EHCI

### DeviceDetect → PORTSC1 polling
- `PORTSC1::CCS` (bit 0) = device present
- `PORTSC1::PSPD` (bits 27:26) = speed: 0=FS, 1=LS, 2=HS
- `PORTSC1::CSC` (bit 1) = connect status change (W1C)
- ISR watches `USBSTS::PCI` (port change interrupt)
- Poll reads PORTSC1, compares to stored status

### reset_root_port → PORTSC1.PR
- `rst=true`: Set `PORTSC1::PR` (bit 8) = 1
- `rst=false`: Clear `PORTSC1::PR` (bit 8) = 0 (but EHCI may auto-clear)
- After reset completes: `PORTSC1::PE` (bit 2) = 1 indicates port enabled

### control_transfer → qTD chain on async schedule
- EHCI handles SETUP+DATA+STATUS as a chain of qTDs linked to a QH
- QH goes on the async schedule (`ASYNCLISTADDR`)
- `USBCMD::ASE` enables the async schedule
- ISR watches `USBSTS::UI` (USB interrupt = transfer complete)
- qTD status bits map to UsbError variants

### Pipe allocation → QH pool
- Each QH can be used for one endpoint at a time
- Pool manages QH allocation (control QHs, bulk QHs)
- Control pipe: allocate QH, build qTD chain, submit, wait, free QH
- Interrupt pipe: allocate QH, put on periodic schedule, keep until dropped

### InterruptPipe → periodic schedule QH
- QH goes on the periodic schedule frame list
- ISR wakes pipe waker on `USBSTS::UI`
- Poll checks qTD completion, re-arms qTD, returns packet
- Drop removes QH from periodic schedule

### Data toggle
- EHCI tracks data toggle in QH overlay area automatically
- For bulk transfers, the `Cell<bool>` data_toggle is managed by the QH's DT bit
- We may need to save/restore data toggle when reusing QHs

## 5. Async Pool (async_pool.rs)

```rust
pub struct Pool {
    total: u8,
    allocated: Cell<BitSet>,      // Which resources are in use
    waker: RefCell<Option<Waker>>, // Waker for next waiter
}

pub struct Pooled<'a> {
    n: u8,
    pool: &'a Pool,
}

impl Drop for Pooled<'_> {
    fn drop(&mut self) {
        self.pool.dealloc_internal(self.n);  // returns to pool + wakes waiter
    }
}
```

Key: `Pooled` has drop glue that returns the resource. Used for RAII pipe ownership.
