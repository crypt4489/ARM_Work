.syntax unified
.cpu cortex-m4
.thumb

.include "spi_addresses.s"
.include "addresses.s"



.global spi1Init


/*


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


//

//this will be used as master of bus but with

/*PA4 - SPI1_NSS
PA5 - SPI1_SCK
PA6 - SPI_MISO
PA7 - SPI_MOSI */

.section .text.spi1Init
	.type spi1Init, %function

//IN : r0 address to data structure

//r0 baudrate
//r1 lsb/msb frame format
//r2 data frame format
//r3 ssm and ssoe
//r4 master/slave selection
//r5 CPOL/CPHA, what value it is when low/which clock part is used when transmitting

spi1Init:

	push {r7-r9}

	ldr r5, [r0, #20]
	ldr r4, [r0, #16]
	ldr r3, [r0, #12]
	ldr r2, [r0, #8]
	ldr r1, [r0, #4]
	ldr r0, [r0]


	ldr r7, =RCC_AHB1ENR
	ldr r8, [r7]
	orr r8, r8, #1
	str r8, [r7] //enable Gpioa

	ldr r7, =RCC_APB2ENR
	ldr r8, [r7]
	orr r8, r8, #(1 << 12)
	str r8, [r7] //enable spi1

	ldr r7, =GPIOA_BASE
	ldr r8, [r7, GPIO_MODER]
	orr r8, r8, #(0xAA << 8)
	str r8, [r7, GPIO_MODER] //alternate fucntion mode

	ldr r8, [r7, GPIO_OSPEEDR]
	orr r8, r8, #(0xAA << 8)
	str r8, [r7, GPIO_OSPEEDR] //fast speed

	ldr r8, [r7, GPIO_PUPDR]
	orr r8, r8, #(0xAA << 8)
	str r8, [r7, GPIO_PUPDR] //pull down resistor

	mov r9, #0x5555
	lsl r9, r9, #16
	ldr r8, [r7, GPIO_AFRL]
	orr r8, r8, r9
	str r8, [r7, GPIO_AFRL] //set alternate function as 5 for SPI


	ldr r7, =SPI1
	ldr r9, =0

	orr r9, r9, r2
	lsl r9, r9, #11 //data frame
	and r10, r3, #1
	lsl r10, r10, #9
	orr r9, r9, r10 //ssm
	and r10, r1, #1
	lsl r10, r10, #7
	orr r9, r9, r10 //lsb/msb
	and r10, r0, #7
	lsl r10, r10, #3
	orr r9, r9, r10 //baud rate
	and r10, r4, #1
	lsl r10, r10, #2
	orr r9, r9, r10 //master or slave
	and r10, r5, #3
	orr r9, r9, r10 // CPOL/CPHA
	orr r9, r9, #(1 << 6) //enable


	and r8, r3, #2
	lsl r8, r8, #1
	str r8, [r7, SPI_CR2]

	str r9, [r7, SPI_CR1]

	mov r9, #0xC3

	str r9, [r7, SPI_DR]

	ldr r7, [r7, SPI_DR]

	pop {r7-r9}

	bx lr

.size spi1Init, .-spi1Init

