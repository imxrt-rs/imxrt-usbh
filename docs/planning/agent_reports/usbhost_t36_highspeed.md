# Reference: USBHost_t36 High Speed Handling

**Date**: 2026-02-18
**Source**: Analysis of `USBHost_t36` codebase at `C:\Users\tacer\GitHub\USBHost_t36\`

## 1. ENHOSTDISCONDETECT PHY Configuration - When It's Set

**Answer: ENHOSTDISCONDETECT is set AFTER a High Speed device is detected during port reset**

- **Location:** `ehci.cpp`, lines 405-408
- **Code:**
```c
if (USBHS_PORTSC1 & USBHS_PORTSC_HSP) {
    // turn on high-speed disconnect detector
    USBPHY_CTRL_SET = USBPHY_CTRL_ENHOSTDISCONDETECT;
}
```
- **When:** This is set in the ISR port change handler when:
  1. Port state is `PORT_STATE_RESET`
  2. Port becomes enabled (`USBHS_PORTSC_PE` bit set) - this happens after the reset sequence completes
  3. The HSP (High Speed Port) bit is detected in PORTSC1
  4. The device is transitioning to `PORT_STATE_RECOVERY`

- **Cleared:** ENHOSTDISCONDETECT is cleared when the device disconnects:
  - **Location:** `ehci.cpp`, line 391
  - **Code:**
```c
USBPHY_CTRL_CLR = USBPHY_CTRL_ENHOSTDISCONDETECT;
```

## 2. Full Speed -> High Speed Transition During Port Reset (Chirp Protocol)

**Answer: The code does NOT explicitly handle chirp protocol - the EHCI hardware handles it**

The reset sequence is very simple and relies on hardware to detect the speed:

- **Port Reset Initiation:** `ehci.cpp`, line 424
```c
USBHS_PORTSC1 |= USBHS_PORTSC_PR; // begin reset sequence
```

- **Speed Detection:** After reset completes, the speed is read from PORTSC1:
  - **Location:** `ehci.cpp`, line 430
  - **Code (root port):**
```c
uint32_t speed = (USBHS_PORTSC1 >> 26) & 3;
// 0=FS, 1=LS, 2=HS
```

  - **Location:** `hub.cpp`, lines 403-405
  - **Code (hub port):**
```c
uint8_t speed=0;
if (status & 0x0200) speed = 1;  // LS
else if (status & 0x0400) speed = 2;  // HS
```

- **No explicit chirp handling:** The EHCI controller hardware handles the chirp protocol automatically. The software only observes the HSP bit after reset completes.

## 3. QH Configuration for High Speed Bulk or Control Endpoints

**Answer: High Speed endpoints use the SAME QH structure with speed encoded in capabilities**

- **Location:** `ehci.cpp`, lines 579-594
- **QH Capabilities Functions:**

```c
static uint32_t QH_capabilities1(uint32_t nak_count_reload, uint32_t control_endpoint_flag,
    uint32_t max_packet_length, uint32_t head_of_list, uint32_t data_toggle_control,
    uint32_t speed, uint32_t endpoint_number, uint32_t inactivate, uint32_t address)
{
    return ( (nak_count_reload << 28) | (control_endpoint_flag << 27) |
        (max_packet_length << 16) | (head_of_list << 15) |
        (data_toggle_control << 14) | (speed << 12) | (endpoint_number << 8) |
        (inactivate << 7) | (address << 0) );
}

static uint32_t QH_capabilities2(uint32_t high_bw_mult, uint32_t hub_port_number,
    uint32_t hub_address, uint32_t split_completion_mask, uint32_t interrupt_schedule_mask)
{
    return ( (high_bw_mult << 30) | (hub_port_number << 23) | (hub_address << 16) |
        (split_completion_mask << 8) | (interrupt_schedule_mask << 0) );
}
```

- **Pipe Setup:** `ehci.cpp`, lines 659-662
```c
pipe->qh.capabilities[0] = QH_capabilities1(15, c, maxlen, 0,
    dtc, dev->speed, endpoint, 0, dev->address);
pipe->qh.capabilities[1] = QH_capabilities2(1, dev->hub_port,
    dev->hub_address, pipe->complete_mask, pipe->start_mask);
```

**Key points:**
- Speed is passed directly from device structure (`dev->speed`): 0=FS, 1=LS, 2=HS
- Speed is encoded at bits [13:12] of capabilities[0] (see line 585)
- For control endpoints: `c = 1` only for FS/LS devices (line 651); HS control endpoints have `c = 0`
- For bulk endpoints: no special handling - same structure used for all speeds
- NAK count reload = 15 (same as our implementation)

## 4. Delays and Timing After Port Reset for High Speed Devices

**Answer: Same timing for HS and FS devices - timing is USB 2.0 spec compliant**

- **100ms Debounce:** `ehci.cpp`, line 381
```c
// 100 ms debounce (USB 2.0: TATTDB, page 150 & 188)
USBHS_GPTIMER0LD = 100000; // microseconds
```

- **Port Reset Recovery - 10ms:** `ehci.cpp`, lines 402-404
```c
// 10 ms reset recover (USB 2.0: TRSTRCY, page 151 & 188)
USBHS_GPTIMER0LD = 10000; // microseconds
USBHS_GPTIMER0CTL = USBHS_GPTIMERCTL_RST | USBHS_GPTIMERCTL_RUN;
```

- **Reset Recovery Timer (Hub):** `hub.cpp`, line 407
```c
resettimer.start(25000);  // 25ms recovery timer
```

**Timing sequence:**
1. Device connect detected → 100ms debounce
2. After debounce → Initiate port reset (PR bit set)
3. Port reset completes → Port Enable (PE) event
4. On PE event → Start 10ms recovery timer (root port)
5. On hub ports → Start 25ms recovery timer
6. After recovery → Device enumeration begins

**Note:** USBHost_t36 uses a **100ms debounce** before port reset. Our
cotton-usb-host flow has **no explicit debounce** — the `device_events_no_hubs`
method immediately resets after `Present(speed)`.

## 5. Port Reset Sequence - Reset Duration HS vs FS

**Answer: The code does NOT explicitly vary reset duration by speed - Hardware handles it**

The port reset sequence is very straightforward:

- **Location:** `ehci.cpp`, lines 415-425
```c
if (stat & USBHS_USBSTS_TI0) { // timer 0
    if (port_state == PORT_STATE_DEBOUNCE) {
        port_state = PORT_STATE_RESET;
        // Begin reset sequence
        USBHS_PORTSC1 |= USBHS_PORTSC_PR; // begin reset sequence
        println("  begin reset");
```

**Key points:**
- The code simply sets `USBHS_PORTSC_PR` (Port Reset bit)
- The EHCI controller hardware automatically holds reset and completes it
- The hardware handles speed detection during reset (FS vs HS chirp)
- The port enable (PE) interrupt indicates reset completion
- **No explicit reset duration timing:** The i.MX RT 1062 EHCI controller applies the reset and automatically detects speed through the LS/HS handshake

## 6. Port State Machine (ISR-driven)

USBHost_t36 uses a state machine in the ISR for port management:

```
PORT_STATE_DISCONNECTED
    → (CCS=1) → PORT_STATE_DEBOUNCE (100ms timer)
    → (timer) → PORT_STATE_RESET (set PR)
    → (PE=1)  → PORT_STATE_RECOVERY (10ms timer, set ENHOSTDISCONDETECT if HSP)
    → (timer) → PORT_STATE_ACTIVE (begin enumeration)
    → (CCS=0) → PORT_STATE_DISCONNECTED (clear ENHOSTDISCONDETECT)
```

**Contrast with our implementation:** We use cotton-usb-host's `device_events_no_hubs`
which drives the state machine from async task context rather than the ISR. The
flow is: detect → reset → delay → enumerate. No separate debounce state.

## 7. Implications for Our Implementation

### ENHOSTDISCONDETECT Must Be Deferred
Setting ENHOSTDISCONDETECT during `init()` is wrong. It must be set only after
confirming HSP=1 in PORTSC1. This is the most likely cause of HS control
transfer failures.

### QH Configuration Matches
Our `qh_characteristics()` function produces the same layout as USBHost_t36's
`QH_capabilities1()`. Speed, C-bit, NAK_RL, DTC all match for HS devices.

### No Debounce May Cause Issues
USBHost_t36 uses a 100ms debounce. Our flow (via cotton-usb-host) has no
debounce. This may cause issues with contact bounce on some USB ports, but
is unlikely to be the root cause of consistent HS failures.
