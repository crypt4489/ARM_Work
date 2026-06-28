# STM32F4 Bare-Metal Peripheral Drivers (ARM Assembly)
 
Hand-written ARMv7-M (Cortex-M4) assembly drivers for STM32F4-series peripherals — no CMSIS, no HAL, no C runtime. Registers are addressed directly via `.equ` offsets, and each peripheral driver is built as a set of `.global` entry points meant to be called from C or from interrupt vector stubs.
 
This is low-level firmware code: it talks straight to the CAN, SPI, USART, and DMA controllers on the chip with raw memory-mapped I/O.
 
## Target
 
- **MCU family:** STM32F4 (Cortex-M4, `armv7e-m`, Thumb-2)
- **Toolchain:** GNU `as`/`gcc`
- **No dependencies:** no CMSIS headers, no ST HAL/LL, no libc
## File Overview
 
| File | Purpose |
|---|---|
| `canbus.s` | Full bxCAN driver: peripheral/GPIO init, mailbox-based init/normal mode sequencing, 32-bit filter bank configuration, NVIC enable/disable macros for TX and RX interrupts, and a software FSM (`CAN1FSM`/`CAN2FSM`) that manages ring buffers for receive, transmit, and intermediate byte data per CAN instance. Includes TX/RX0/RX1 interrupt handlers for both CAN1 and CAN2, plus block-based message create/retrieve and "pump" routines to drain queued messages into mailboxes. |
| `spi.s` | SPI1 driver, master mode: GPIO alternate-function setup (PA4–PA7), `SPI_CR1`/`CR2` configuration from a parameter struct, DMA2 stream setup for SPI TX/RX, a polled single-word copy routine, and a global SPI interrupt/DMA clear handler. |
| `usart.s` | USART2 driver: GPIO/clock setup, a baud-rate generator (`CalculateBRR`, mantissa/fraction divider math with oversampling support), polled single-word transmit, and DMA1-driven block transmit with completion/flag handling. |
 
## Architecture Notes
 
- **Register access pattern:** every peripheral's registers are defined as `.equ` offsets from a base address (e.g. `CAN_MCR`, `USART_SR`, `DMA_S0CR`), included from `addresses.s` / `canbus_addresses.s`, then accessed with `ldr`/`str [base, OFFSET]`.
- **CAN software FSM:** each bxCAN instance has an in-memory FSM block (`CAN1FSM`/`CAN2FSM`) carved into three ring buffers — receive blocks, raw byte storage, and transmit blocks — sized at init time from caller-supplied buffer memory and message counts (`canFSMInit` / `can1FSMInit` / `can2FSMInit`).
- **Interrupt-safe queue mutation:** TX/RX queue operations that are also touched by interrupt handlers (e.g. `createTransmitMessageBlkCAN1`, `retrieveMessageBlockCAN1`) wrap the underlying call in `DISABLE*INTERRUPT`/`ENABLE*INTERRUPT` macros rather than relying on a full critical section, to avoid races with the ISRs.
- **DMA usage:** SPI1 uses DMA2 streams 0 (RX) and 3 (TX); USART2 uses DMA1 stream 6 — consistent with the DMA stream-to-peripheral mapping on the STM32F4.
- **No dynamic allocation:** all buffers (CAN FSM ring buffers, SPI/USART DMA targets) are caller-provided memory regions passed in by address; nothing is heap-allocated.

## To Do

1. **Finish SPI and USART.** Have both use DMA copy patterns and make them configurable in the way bxCAN is currently is. Create possible superstructure to allow receives to be managed by the driver, a la bxCAN driver.
2. **Create SPI master session and make it configurable for different SPI hubs on STM MCU** Currently, window messaging passes note between to a USART to Winodws connection from a embedded Linux to STM SPI connection. Create a Full Duplex STM session and create multi slave setup with SPI2/SPI3
3. **Generalize the CAN1/CAN2 selector pattern.** The `ENABLE/DISABLERXINTERRUPT` and TX equivalents take a magic `#0`/`#1` "which CAN" argument; consider deriving NVIC IRQ numbers from the CAN base address instead of duplicating call sites per-instance.
4. **Resolve dead/duplicate code paths.** `can2FSMInit` has two `bx lr` in a row, and the commented-out `canBUSTransmit`/`canBUSReceive` block in `canbus.s` should either be removed or reconciled with the FSM-based equivalents.
5. **Write a top-level integration example.** None of these files show the expected init order (clocks → GPIO → peripheral → NVIC → FSM) end-to-end for a board; a small `main`-equivalent or sequence diagram would make the driver call order unambiguous to a new reader.
6. **Add NVIC specific layer to configure the ARM chip to user specifications**
7. **Add RTC and RCC specific layer to configure clock and set up peripherals** Generate functions that will set up the RTC and RCC registers per the specification of user and by usage of the driver
8. **Add Error Handling and navigation for bxCAN, USART, and SPI**
