.syntax unified
.cpu cortex-m4
.thumb

.include "spi_addresses.s"
.include "addresses.s"



.global spi1Init
.global spiImmediateCopy
.global spi1SetupDMAStreams
.global spi1StartDMATransfer
.global spi1StartDMAReception
.global spi1ReceptionDMAPoll
.global spiGlobalInterrupt
.global spi1RXGlobalInterrupt
/*

struct FSM
{
	u32 TransmitBufferAddr
	u32 TrasnmitBufferRead
	u32 TransmitBufferCurr
	u32 TransmitBufferSize
	u32 ReceiveBufferAddr
	u32 ReceiveBufferCurr
	u32 ReceiveBufferRead
	u32 ReceiveBufferSize
	u32 ReceiveBufferCount
};



NSS output enable (SSM=0,SSOE = 1): this configuration is only used when the
MCU is set as master. The NSS pin is managed by the hardware. The NSS signal
is driven low as soon as the SPI is enabled in master mode (SPE=1), and is kept
low until the SPI is disabled (SPE =0).

NSS output disable (SSM=0, SSOE = 0): if the microcontroller is acting as the
master on the bus, this configuration allows multimaster capability. If the NSS pin
is pulled low in this mode, the SPI enters master mode fault state and the device is
automatically reconfigured in slave mode. In slave mode, the NSS pin works as a
standard “chip select” input and the slave is selected while NSS line is at low level.

*/


.section .text.spiGlobalInterrupt
	.type spiGlobalInterrupt, %function
spiGlobalInterrupt:
//r0 DMA_BASE
//r1 dma stream index
push {r2-r6}
cmp r1, #3
ITTEE gt
movgt r2, DMA_HISR
movgt r3, DMA_HIFCR
movle r2, DMA_LISR
movle r3, DMA_LIFCR
and r4, r1, #1
and r5, r1, #2
lsr r5, r5, #1
mov r6, #16
mul r5, r5, r6
mov r6, #6
mul r4, r4, r6
mov r6, #0x3d
add r4, r5, r4
lsl r6, r6, r4
ldr r4, [r0, r2]
and r4, r4, r6
str r4, [r0, r3]
pop {r2-r6}
bx lr

.size spiGlobalInterrupt, .-spiGlobalInterrupt


.section .text.spi1RXGlobalInterrupt
	.type spi1RXGlobalInterrupt, %function
spi1RXGlobalInterrupt:

push {r0-r1}
ldr r0, =NVIC_ICPR1
ldr r1, [r0]
orr r1, r1, #(0b1 << 24)
str r1, [r0]

ldr r0, =DMA2_BASE
ldr r1, =0

push {lr}

bl spiGlobalInterrupt

pop {lr}

pop {r0-r1}
bx lr

.size spi1RXGlobalInterrupt, .-spi1RXGlobalInterrupt


.section .text.spi1SetupDMAStreams
	.type spi1SetupDMAStreams, %function
spi1SetupDMAStreams:

 push {r0-r1}
 ldr r0, =RCC_AHB1ENR       //gpioa enable clock peripheral
 ldr r1, [r0]
 orr r1, r1, #(0b1 << 22) // enabled DMA2 controller (only one for memory to memory)
 str r1, [r0]
 pop {r0-r1}
 bx lr

.size spi1SetupDMAStreams, .-spi1SetupDMAStreams


.section .text.spi1StartDMATransfer
	.type spi1StartDMATransfer, %function
spi1StartDMATransfer:

//r0 ADDRESS OF DATA
//r1 SIZE OF DATA
 push {r2-r3}
 ldr r2, =DMA2_BASE  // load dma2 base
 ldr r3, =SPI1
 add r3, r3, SPI_DR
 str r3, [r2, DMA_S3PAR] // dma write is SPI address
 str r0, [r2, DMA_S3M0AR] //dma read is ADDRESS OF DATA

 str r1, [r2, DMA_S3NDTR] // number of n-bytes to write
 mov r3, #0x0440 // auto increment read address and keep fixed write address, mem-to-per
 movt r3, #0x0600 //dma channel 3
 str r3, [r2, DMA_S3CR] //configure dma transfer
 ldr r3, [r2, DMA_S3FCR]
 orr r3, r3, #(0b11)
 str r3, [r2, DMA_S3FCR] //full fifo
 ldr r3, [r2, DMA_S3CR]
 orr r3, r3, #1
 str r3, [r2, DMA_S3CR] //start dma transfer
 pop {r2-r3}
 bx lr

.size spi1StartDMATransfer, .-spi1StartDMATransfer


.section .text.spi1StartDMAReception
	.type spi1StartDMAReception, %function
spi1StartDMAReception:


//r0 ADDRESS OF DATA
//r1 SIZE OF DATA
 push {r2-r4}
 ldr r2, =DMA2_BASE  // load dma2 base
 ldr r3, =SPI1
 add r3, r3, SPI_DR
 str r3, [r2, DMA_S0PAR] // dma write is SPI address
 str r0, [r2, DMA_S0M0AR] //dma read is ADDRESS OF DATA

 str r1, [r2, DMA_S0NDTR] // number of n-bytes to write
 mov r3, #0x0410 // auto increment read address and keep fixed write address, per-to-mem
 movt r3, #0x0600 //dma channel 3
 ldr r4, [r2, DMA_S0CR]
 orr r4, r4, r3
 str r4, [r2, DMA_S0CR] //configure dma transfer
 ldr r3, [r2, DMA_S0FCR]
 orr r3, r3, #(0b11)
 str r3, [r2, DMA_S0FCR] //full fifo
 ldr r3, [r2, DMA_S0CR]
 orr r3, r3, #1
 str r3, [r2, DMA_S0CR] //start dma transfer
 pop {r2-r4}
 bx lr
.size spi1StartDMAReception, .-spi1StartDMAReception


//

//this will be used as master of bus but with

/*PA4 - SPI1_NSS
PA5 - SPI1_SCK
PA6 - SPI_MISO
PA7 - SPI_MOSI */

.section .text.spi1Init
	.type spi1Init, %function

//IN : r0 address to data structure

//r1 baudrate
//r2 lsb/msb frame format
//r3 data frame format
//r4 ssm and ssoe
//r5 master/slave selection
//r6 CPOL/CPHA, what value it is when low/which clock part is used when transmitting
//r7 DMA TX and RX
//r8 DMA BIMODE and associated
spi1Init:

	push {r1-r10}



	ldr r7, =RCC_AHB1ENR
	ldr r8, [r7]
	orr r8, r8, #1
	str r8, [r7] //enable Gpioa

	ldr r7, =RCC_APB2ENR
	ldr r8, [r7]
	orr r8, r8, #(1 << 12)
	str r8, [r7] //enable spi1

	ldr r7, [r0, #24]
	ldr r6, [r0, #20]
	ldr r5, [r0, #16]
	ldr r4, [r0, #12]
	ldr r3, [r0, #8]
	ldr r2, [r0, #4]
	ldr r1, [r0]


	ldr r8, =GPIOA_BASE
	ldr r9, [r8, GPIO_MODER]
	bic r9, r9, #(0xFF << 8)
	orr r9, r9, #(0xAA << 8)
	str r9, [r8, GPIO_MODER] //alternate fucntion mode

	ldr r9, [r8, GPIO_OSPEEDR]
	bic r9, r9, #(0xFF << 8)
	orr r9, r9, #(0xAA << 8)
	str r9, [r8, GPIO_OSPEEDR] //fast speed

	ldr r9, [r8, GPIO_PUPDR]
	bic r9, r9, #(0xFF << 8)
	orr r9, r9, #(0xAA << 8)
	str r9, [r8, GPIO_PUPDR] //pull down resistor

	mov r9, #0xFFFF
	lsl r9, r9, #16
	ldr r10, [r8, GPIO_AFRL]
	bic r10, r10, r9
	mov r9, #0x5555
	lsl r9, r9, #16
	orr r9, r10, r9
	str r9, [r8, GPIO_AFRL] //set alternate function as 5 for SPI


	ldr r8, =SPI1
	ldr r9, =0

	orr r9, r9, r3
	lsl r9, r9, #11 //data frame
	and r10, r4, #1
	lsl r10, r10, #9
	orr r9, r9, r10 //ssm
	and r10, r2, #1
	lsl r10, r10, #7
	orr r9, r9, r10 //lsb/msb
	and r10, r1, #7
	lsl r10, r10, #3
	orr r9, r9, r10 //baud rate
	and r10, r5, #1
	lsl r10, r10, #2
	orr r9, r9, r10 //master or slave
	and r10, r6, #3
	orr r9, r9, r10 // CPOL/CPHA
	orr r9, r9, #(1 << 6) //enable
	ldr r1, [r0, #28]
	and r1, r1, #3
	lsl r1, r1, #14
	orr r9, r9, r1

	and r10, r4, #2
	lsl r10, r10, #1
	orr r10, r10, r7

	str r10, [r8, SPI_CR2]

	str r9, [r8, SPI_CR1]

	pop {r1-r10}

	bx lr

.size spi1Init, .-spi1Init


.section .text.spi1ReceptionDMAPoll
	.type spi1ReceptionDMAPoll, %function
spi1ReceptionDMAPoll:
  push {r0-r1}
  ldr r0, =DMA2_BASE
L_001:
  ldr r1, [r0, DMA_LISR]
  and r1, r1, #0x20
  cmp r1, #0
  beq L_001

  orr r1, r1, #0x10
  str r1, [r0, DMA_LIFCR]
  pop {r0-r1}
  bx lr
.size spi1ReceptionDMAPoll, .-spi1ReceptionDMAPoll

.section .text.spiImmediateCopy
	.type spiImmediateCopy, %function
spiImmediateCopy:
//r0 SPI ADDR
//r1 DATA

	push {r2}

L_SPICOPYLOOP:
	ldr r2, [r0, SPI_SR]
	and r2, r2, #2
	cmp r2, #0
	beq L_SPICOPYLOOP

	str r1, [r0, SPI_DR]

	pop {r2}
	bx lr

.size spiImmediateCopy, .-spiImmediateCopy


