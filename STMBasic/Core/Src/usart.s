/*
 * usart.s
 *
 *  Created on: Apr 16, 2025
 *      Author: dflet
 */
.syntax unified
.cpu cortex-m4
.thumb


.include "addresses.s"


.global EstablishUSART2
.global USART2_IRQHandler
.global EstablishUSART2_INT
.global DMATest
.global DMAUSART
.global OutputStringSize
.global DMAUSARTReceive
.global USARTBitBanging
.global USARTBitBangingInt



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

BitBangString:
	.asciz "a"
BitBangTransmit:
	.byte 0
BitBangIterator:
	.byte 0
BitBangIdle:
	.byte 8

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
 orr r1, r1, #0x0088 // 8 bit word and Start USART and set INT for TIX
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


 .section  .text.DMAUSART
 .type DMAUSART, %function
DMAUSART:
 ldr r0, =RCC_AHB1ENR
 ldr r1, [r0]
 orr r1, r1, #(0b1 << 21) // enabled DMA1 controller (only one for memory to memory)
 orr r1, r1, #0x0001 //enable GPIOA clock
 str r1, [r0]

 ldr r0, =GPIOA_BASE


 ldr r1, [r0]
 bic r1, r1, #(0b11 << (2 * 2))   // Clear PA2
 orr r1, r1, #(0b10 << (2 * 2))   // Set PA2 transmitter
 str r1, [r0]


 ldr r1, [r0, #0xC]
 bic r1, r1, #(0x3 << (2 * 2)) // clear pull up down resiter
 orr r1, r1, #(0b01 << (2 * 2))   // Set PA2 to pull up
 str r1, [r0, #0xC]


 ldr r1, [r0, #0x20]       //; GPIOA_AFRH
 bic r1, r1, #(0xF << (2 * 4))
 orr r1, r1, #(0x7 << (2 * 4)) //USART2
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

 ldr r1, [r0, USART_CR3]
 orr r1, r1, #0x0080 // Enable DMA Transfer
 str r1, [r0, USART_CR3]


 ldr r1, =[DMA1_BASE]  // load dma1 base
 ldr r2, =OutputString // address to be written
 add r3, r0, USART_DR
 str r3, [r1, DMA_S6PAR] // dma write is USART address
 str r2, [r1, DMA_S6M0AR] //dma read is outputString
 ldr r2, =OutputStringSize
 ldr r2, [r2]

 str r2, [r1, DMA_S6NDTR] // number of n-bytes to write
 mov r2, #0x0440 // auto increment read address and keep fixed write address, mem-to-per
 movt r2, #0x0800
 str r2, [r1, DMA_S6CR] //configure dma transfer
 ldr r2, [r1, DMA_S6FCR]
 orr r2, r2, #(0b11)
 str r2, [r1, DMA_S6FCR]



 mov r4, r1
 eor r1, r1
 str r1, [r0, USART_SR]

 ldr r1, [r0, USART_CR1]
 orr r1, r1, #0x0008 // 8 bit word and Start USART
 str r1, [r0, USART_CR1]
 ldr r1, [r4, DMA_S6CR]
 orr r1, r1, #1
 str r1, [r4, DMA_S6CR] //start dma transfer

 ldr r0, =[DMA1_BASE] // this will just write the string to the buffer over and over again
 ldr r1, =OutputStringSize
 ldr r1, [r1]
 mov r3, #0
 movt r3, #(1 << 5)
dmausartloop:
 ldr r2, [r0, DMA_HISR]
 and r4, r2, r3
 cmp r4, r3
 bne dmausartloop
 eor r4, r4
 movt r4, #(0b110001) //fifo error is for when the buffer is cleared, but no more data is there
 str r4, [r0, DMA_HIFCR]
 str r1, [r0, DMA_S6NDTR]
 ldr r2, [r0, DMA_S6CR]
 orr r2, r2, #1
 str r2, [r0, DMA_S6CR]
 b dmausartloop
 bx lr

 .size DMAUSART, .-DMAUSART


  .section  .text.DMAUSARTReceive
 .type DMAUSARTReceive, %function
DMAUSARTReceive:
 ldr r0, =RCC_AHB1ENR
 ldr r1, [r0]
 orr r1, r1, #(0b1 << 21) // enabled DMA1 controller (only one for memory to memory)
 orr r1, r1, #0x0001 //enable GPIOA clock
 str r1, [r0]

 ldr r0, =GPIOA_BASE


 ldr r1, [r0]
 bic r1, r1, #(0b11 << (3 * 2))   // Clear PA3
 orr r1, r1, #(0b10 << (3 * 2))   // Set PA3 Receiver
 str r1, [r0]


 ldr r1, [r0, #0xC]
 bic r1, r1, #(0x3 << (3 * 2)) // clear pull up down resiter
 orr r1, r1, #(0b01 << (3 * 2))   // Set PA3 to pull up
 str r1, [r0, #0xC]


 ldr r1, [r0, #0x20]       //; GPIOA_AFRH
 bic r1, r1, #(0xF << (3 * 4))
 orr r1, r1, #(0x7 << (3 * 4)) //USART2
 str r1, [r0, #0x20]

 ldr r0, =RCC_APB1ENR
 ldr r1, =#0x20000 //enable usart2 clock
 str r1, [r0]


 ldr r0, =USART2_BASE



 ldr r1, [r0, USART_CR1]
 orr r1, r1, #0x2000 // 8 bit word and Start USART
 str r1, [r0, USART_CR1]

 mov r1, #0x124f
 str r1, [r0, USART_BRR] // set baud rate of 9600

 ldr r1, [r0, USART_CR3]
 orr r1, r1, #0x0040 // Enable USART Receive DMAR
 str r1, [r0, USART_CR3]


 ldr r1, =[DMA1_BASE]  // load dma1 base
 ldr r2, =OutputString // address to be written to
 add r3, r0, USART_DR
 str r3, [r1, DMA_S5PAR] // dma read is USART address
 str r2, [r1, DMA_S5M0AR] //dma write is outputString
 ldr r2, =OutputStringSize
 ldr r2, [r2]

 str r2, [r1, DMA_S5NDTR] // number of n-bytes to write
 mov r2, #0x0400 // auto increment write address and keep fixed write address, mem-to-per
 movt r2, #0x0800
 str r2, [r1, DMA_S5CR] //configure dma transfer

 mov r4, r1
 eor r1, r1
 str r1, [r0, USART_SR]

 ldr r1, [r0, USART_CR1]
 orr r1, r1, #0x0004 // 8 bit word and Start USART
 str r1, [r0, USART_CR1]
 ldr r1, [r4, DMA_S5CR]
 orr r1, r1, #1
 str r1, [r4, DMA_S5CR] //start dma transfer

 bx lr

 .size DMAUSARTReceive, .-DMAUSARTReceive


  .section  .text.USARTBitBanging
 .type USARTBitBanging, %function
USARTBitBanging:

 ldr r0, =RCC_AHB1ENR

 ldr r1, [r0]
 orr r1, r1, #0x0001 //enable GPIOA clock
 str r1, [r0]

 ldr r0, =RCC_APB1ENR

 ldr r1, [r0]
 orr r1, r1, #0x4 //enable tim4
 str r1, [r0]

 ldr r0, =GPIOA_BASE

 ldr r1, [r0]
 bic r1, r1, #(0b11 << (2 * 2))   // Clear PA2
 orr r1, r1, #(0b01 << (2 * 2))   // Set PA2 GPIO
 str r1, [r0]

 ldr r1, [r0, #0x8]
 bic r1, r1, #(0b11 << (2 * 2))
 orr r1, r1, #(0b11 << (2 * 2))
 str r1, [r0, 0x8]


 ldr r1, [r0, #0xC]
 bic r1, r1, #(0x3 << (2 * 2)) // clear pull up down resiter
 orr r1, r1, #(0b01 << (2 * 2))   // Set PA2 to pull up
 str r1, [r0, #0xC]


 ldr r0, =TIM4_BASE

 mov r1, #200                //; Prescaler = 83
 str r1, [r0, TIM4_PSC]            // ; Store PSC

 mov r1, #93
 str r1, [r0, TIM4_ARR]

  ldr r2, [r0]  //TIM4_CR1 ARR preload
  orr r2, r2, #0x0080
  str r2, [r0]

  ldr r2, [r0, TIM4_EGR]  //TIM4_CR1 write to shadow regs
  orr r2, r2, #0x0001
  str r2, [r0, TIM4_EGR]


  ldr r1, [r0, TIM4_DIER]
  orr r1, r1, #0x0001
  str r1, [r0, TIM4_DIER]


  ldr r2, [r0]  //TIM9_CR1 enable timer
  orr r2, r2, #0x0001
  str r2, [r0]

  ldr r1, =NVIC_ISER0 // interrupt set enable for NVIC
  mov r2, #1 << 30
  str r2, [r1]


 bx lr
.size USARTBitBanging, .-USARTBitBanging

.section .text.USARTBitBangingInt
  	.type USARTBitBangingInt, %function
USARTBitBangingInt:

LDR r0, =TIM4_BASE
  LDR r1, [r0, TIM4_SR]
  BIC r1, r1, #0x0001
  STR r1, [r0, TIM4_SR]
  LDR r0, =NVIC_ICPR0
  MOV r1, #(1 << 30)
  STR r1, [r0]

  ldr r0, =BitBangIterator
  ldr r1, [r0] // load iterator



  ldr r4, =BitBangTransmit
  ldr r5, [r4]

  ldr r6, =GPIOA_BASE
  ldr r2, =BitBangIdle
  ldrb r3, [r2]
 // cmp r3, #10
 // beq updateandout
  cmp r3, #0
  beq starttransmission
  ldr r7, [r6, GPIO_BSRR]
  orr r7, r7, #(1 << 2)
  str r7, [r6, GPIO_BSRR]
  sub r3, r3, 1
  strb r3, [r2]
  b updateandout
starttransmission:

  cmp r5, #0
  bne transmitdatabyte
sendstartbit:
  mov r5, #1
  str r5, [r4]
  ldr r7, [r6, GPIO_BSRR]
  orr r7, r7, #(1 << 18)
  str r7, [r6, GPIO_BSRR]
  b updateandout
transmitdatabyte:
  cmp r1, #8
  beq sendendbit
  ldr r2, =BitBangString
  ldrb r3, [r2]
  lsr r3, r3, r1
  and r3, r3, #1
  ldr r7, [r6, GPIO_ODR]
  lsl r3, r3, #2
  bic r7, r7, #4
  orr r7, r7, r3
  str r7, [r6, GPIO_ODR]
  add r1, r1, #1
  b updateandout

sendendbit:
  mov r5, #0
  str r5, [r4]
  mov r1, #0
  ldr r7, [r6, GPIO_BSRR]
  orr r7, r7, #(1 << 2)
  str r7, [r6, GPIO_BSRR]
  mov r3, #8
  strb r3, [r2]
updateandout:
  strb r1, [r0] // iterator update


  bx lr
.size USARTBitBangingInt, .-USARTBitBangingInt
