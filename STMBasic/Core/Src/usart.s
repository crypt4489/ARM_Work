/*
 * usart.s
 *
 *  Created on: Apr 16, 2025
 *      Author: dflet
 */
.syntax unified
.cpu cortex-m4
.thumb
.global EstablishUSART2
.global USART2_IRQHandler
.global EstablishUSART2_INT
.global DMATest


 .equ USART1_BASE, 0x40011000
 .equ USART2_BASE, 0x40004400
 .equ USART3_BASE, 0x40004800
 .equ UART4_BASE, 0x40004C00
 .equ UART5_BASE, 0x40005000
 .equ USART6_BASE, 0x40014000
 .equ USART_SR, 0x00
 .equ USART_DR, 0x04
 .equ USART_BRR, 0x08
 .equ USART_CR1, 0x0c
 .equ USART_CR2, 0x10
 .equ USART_CR3, 0x14
 .equ USART_GTPR, 0x18
 .equ RCC_APB1ENR, 0x40023840
 .equ RCC_AHB1ENR, 0x40023830
 .equ GPIOA_BASE, 0x40020000
 .equ NVIC_ISER0, 0xE000E100 // interrupt enable nvic
 .equ NVIC_ICPR0, 0xE000E280 //clear pending nvic

 .equ DMA1_BASE, 0x40026000
 .equ DMA2_BASE, 0x40026400
 .equ DMA_LISR, 0x00
 .equ DMA_HISR, 0x04
 .equ DMA_LIFCR, 0x08
 .equ DMA_HIFCR, 0x0C
 .equ DMA_S0CR, 0x10
.equ DMA_S0NDTR, 0x14
.equ DMA_S0PAR, 0x18
.equ DMA_S0M0AR, 0x1c
.equ DMA_S0M1AR, 0x20
.equ DMA_S0FCR, 0x24
.equ DMA_S1CR, 0x28
.equ DMA_S1NDTR, 0x2c
.equ DMA_S1PAR, 0x30
.equ DMA_S1M0AR, 0x34
.equ DMA_S1M1AR, 0x38
.equ DMA_S1FCR, 0x3c
.equ DMA_S2CR, 0x40
.equ DMA_S2NDTR, 0x44
.equ DMA_S2PAR, 0x48
.equ DMA_S2M0AR, 0x4c
.equ DMA_S2M1AR, 0x50
.equ DMA_S2FCR, 0x54
.equ DMA_S3CR, 0x58
.equ DMA_S3NDTR, 0x5c
.equ DMA_S3PAR, 0x60
.equ DMA_S3M0AR, 0x64
.equ DMA_S3M1AR, 0x68
.equ DMA_S3FCR, 0x6c
.equ DMA_S4CR, 0x70
.equ DMA_S4NDTR, 0x74
.equ DMA_S4PAR, 0x78
.equ DMA_S4M0AR, 0x7c
.equ DMA_S4M1AR, 0x80
.equ DMA_S4FCR, 0x84
.equ DMA_S5CR, 0x88
.equ DMA_S5NDTR, 0x8c
.equ DMA_S5PAR, 0x90
.equ DMA_S5M0AR, 0x94
.equ DMA_S5M1AR, 0x98
.equ DMA_S5FCR, 0x9c
.equ DMA_S6CR, 0xa0
.equ DMA_S6NDTR, 0xa4
.equ DMA_S6PAR, 0xa8
.equ DMA_S6M0AR, 0xac
.equ DMA_S6M1AR, 0xb0
.equ DMA_S6FCR, 0xb4
.equ DMA_S7CR, 0xb8
.equ DMA_S7NDTR, 0xbc
.equ DMA_S7PAR, 0xc0
.equ DMA_S7M0AR, 0xc4
.equ DMA_S7M1AR, 0xc8
.equ DMA_S7FCR, 0xcc

 .section .data
OutputString:
    .asciz "Hello, Woorld!"
OutputStringSize:
	.word 14
OutputStringIterator:
	.word 0
TestNumber:
	.word 55
TestNumber2:
	.word 66
Array:
	.byte 0, 1, 2, 3, 4, 5



 .section  .text.EstablishUSART2
 .type EstablishUSART2, %function

 EstablishUSART2:


 ldr r0, =RCC_AHB1ENR       //gpiob enable clock peripheral
 ldr r1, [r0]
 orr r1, r1, #0x0001
 str r1, [r0]

   ldr r0, =GPIOA_BASE


  ldr r1, [r0]
  bic r1, r1, #(0b11 << (2 * 2))   // Clear PB6
  orr r1, r1, #(0b10 << (2 * 2))   // Set PB6 to alternate function
  str r1, [r0]


  ldr r1, [r0, #0xC]
  bic r1, r1, #(0x3 << (2 * 2)) // clear pull up down resiter
  orr r1, r1, #(0b01 << (2 * 2))   // Set PB6 tro pull up
  str r1, [r0, #0xC]


  ldr r1, [r0, #0x20]       //; GPIOx_AFRH
  bic r1, r1, #(0xF << (2 * 4))
  orr r1, r1, #(0x7 << (2 * 4))
  str r1, [r0, #0x20]


 ldr r0, =RCC_APB1ENR
 ldr r1, =#0x20000
 str r1, [r0]


 ldr r0, =USART2_BASE



 ldr r1, [r0, USART_CR1]
 orr r1, r1, #0x2000 // 8 bit word and Start USART
 str r1, [r0, USART_CR1]

 mov r1, #0x124f
 str r1, [r0, USART_BRR] // set baud rate of 9600


 ldr r1, [r0, USART_CR1]
 orr r1, r1, #0x0008 // 8 bit word and Start USART
 str r1, [r0, USART_CR1]

 mov r1, #0x44
 mov r3, #0x80
loop:
 ldr r2, [r0, USART_SR]
 and r2, r2, r3
 cmp r2, r3
 bne loop
 str r1, [r0, USART_DR]
 b loop

 bx lr

 .size  EstablishUSART2, .-EstablishUSART2

 .section  .text.EstablishUSART2_INT
 .type EstablishUSART2_INT, %function
 EstablishUSART2_INT:


 ldr r0, =RCC_AHB1ENR       //gpioa enable clock peripheral
 ldr r1, [r0]
 orr r1, r1, #0x0001
 str r1, [r0]

   ldr r0, =GPIOA_BASE


  ldr r1, [r0]
  bic r1, r1, #(0b11 << (2 * 2))   // Clear PA2
  orr r1, r1, #(0b10 << (2 * 2))   // Set PA2 transmitter
  str r1, [r0]


  ldr r1, [r0, #0xC]
  bic r1, r1, #(0x3 << (2 * 2)) // clear pull up down resiter
  orr r1, r1, #(0b01 << (2 * 2))   // Set PA@ tro pull up
  str r1, [r0, #0xC]


  ldr r1, [r0, #0x20]       //; GPIOA_AFRH
  bic r1, r1, #(0xF << (2 * 4))
  orr r1, r1, #(0x7 << (2 * 4)) //USART2
  str r1, [r0, #0x20]


 ldr r0, =RCC_APB1ENR
 ldr r1, =#0x20000  //usart2 clock enable
 str r1, [r0]


 ldr r0, =USART2_BASE



 ldr r1, [r0, USART_CR1]
 orr r1, r1, #0x2000 // 8 bit word and Start USART
 str r1, [r0, USART_CR1]

 mov r1, #0x124f
 str r1, [r0, USART_BRR] // set baud rate of 9600


 ldr r1, [r0, USART_CR1]
 orr r1, r1, #0x0088 // 8 bit word and Start USART
 str r1, [r0, USART_CR1]

  ldr r1, =NVIC_ISER0 // interrupt set enable for NVIC
  ldr r2, [r1, 0x4]
  orr r2, r2, #1 << 6
  str r2, [r1, 0x4]

 bx lr

 .size  EstablishUSART2_INT, .-EstablishUSART2_INT


 .section  .text.USART2_IRQHandler
 .type USART2_IRQHandler, %function
USART2_IRQHandler:

	push { r0-r4 }
	ldr r0, =NVIC_ICPR0
	ldr r1, [r0, 0x4]
    orr r1, r1, #(1 << 6)
    str r1, [r0, 0x4]
	ldr r0, =OutputString
	ldr r4, =OutputStringIterator
	ldr r1, [r4]
	ldr r2, =OutputStringSize
	ldr r2, [r2]
	cmp r1, r2
	bne USARTINTWriteData
	ldr r0, =USART2_BASE
	ldr r1, [r0, USART_CR1]
	bic r1, r1, #(0x88) // disable interrupts
	str r1, [r0, USART_CR1]
	b USARTINTEND
USARTINTWriteData:
	ldr r2, =USART2_BASE
  	ldr r0, [r0, r1]
  	str r0, [r2, USART_DR]
  	add r1, r1, #1
  	str r1, [r4]
USARTINTEND:
	pop {r0-r4}
	bx lr

.size  USART2_IRQHandler, .-USART2_IRQHandler

  .section  .text.DMATest
 .type DMATest, %function
DMATest:

 ldr r0, =RCC_AHB1ENR       //gpioa enable clock peripheral
 ldr r1, [r0]
 orr r1, r1, #(0b1 << 22) // enabled DMA2 controller (only one for memory to memory)
 str r1, [r0]
 ldr r0, =[DMA2_BASE]  // load dma2 base
 ldr r1, =TestNumber // address to be written
 ldr r2, =Array //address writing to
 str r1, [r0, DMA_S0PAR] // dma read
 str r2, [r0, DMA_S0M0AR] //dma write
 mov r1, #6
 str r1, [r0, DMA_S0NDTR] // number of n-bytes to write
 mov r1, #0x0481 // auto increment write address and keep fixed read address, mem-to-mem
 str r1, [r0, DMA_S0CR] //start dma transfer
 bx lr

 .size  DMATest, .-DMATest
