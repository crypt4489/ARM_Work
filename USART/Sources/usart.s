 .syntax unified
.cpu cortex-m4
.thumb

.include "addresses.s"

.global InitializeDMA1
.global InitializeUSART2
.global WriteToUSART
.global WriteToUSARTDMA


 .section  .text.InitializeDMA1
 .type InitializeDMA1, %function
 InitializeDMA1:
 push {r0-r1}
 ldr r0, =RCC_AHB1ENR
 ldr r1, [r0]
 orr r1, r1, #(0b1 << 21) // enabled DMA1 controller (only one for memory to memory)
 str r1, [r0]
 pop {r0-r1}
 bx lr
 .size  InitializeDMA1, .-InitializeDMA1



 .section  .text.InitializeUSART2
 .type InitializeUSART2, %function
//r0 Data for USART
//r0 data format /0
//r1 baud rate /4
//r2 tx/rx enable /8
//r3 tx/rx dma /12
//r4 parity selection /16
 InitializeUSART2:

 push {r1-r5}
 ldr r1, =RCC_AHB1ENR
 ldr r2, [r1]
 orr r2, r2, #0x0001
 str r2, [r1]

 ldr r1, =GPIOA_BASE


 ldr r2, [r1, GPIO_MODER]
 bic r2, r2, #(0b11 << (2 * 2))   // Clear PB6
 orr r2, r2, #(0b10 << (2 * 2))   // Set PB6 to alternate function
 str r2, [r1, GPIO_MODER]


  ldr r2, [r1, GPIO_PUPDR]
  bic r2, r2, #(0x3 << (2 * 2)) // clear pull up down resiter
  orr r2, r2, #(0b01 << (2 * 2))   // Set PB6 tro pull up
  str r2, [r1, GPIO_PUPDR]


  ldr r2, [r1, GPIO_AFRL]
  bic r2, r2, #(0xF << (2 * 4))
  orr r2, r2, #(0x7 << (2 * 4))
  str r2, [r1, GPIO_AFRL]


 ldr r1, =RCC_APB1ENR
 ldr r3, =#0x20000  //enable USART2
 ldr r2, [r1]
 orr r2, r2, r3
 str r2, [r1]


 ldr r1, =USART2_BASE
 ldr r2, [r0, #0]

 ldr r3, [r0, #8]
 ldr r5, [r0, #16]
 eor r4, r4, r4
 lsl r2, r2, #12
 lsl r3, r3, #2
 lsl r5, r5, #9
 orr r4, r3, r2
 orr r4, r5, r4
 str r4, [r1, USART_CR1]

 ldr r4, [r0, #4]

 //mov r1, #0x124f
 str r4, [r1, USART_BRR] // set baud rate of 9600

 ldr r3, [r0, #12]
 lsl r3, r3, #6
 ldr r2, [r1, USART_CR3]
 orr r2, r2, r3
 str r2, [r1, USART_CR3]

 ldr r2, [r1, USART_CR1]

 orr r2, r2, #0x2000 // 8 bit word and Start USART
 str r2, [r1, USART_CR1]
 pop {r1-r5}
 bx lr

 .size  InitializeUSART2, .-InitializeUSART2


 .section  .text.WriteToUSART
 .type WriteToUSART, %function
//r0 USART BASE
//r1 DATA TO SEND
 WriteToUSART:
 push {r2}
L_001:
 ldr r2, [r0, USART_SR]
 and r2, r2, #0x80
 cmp r2, #0
 beq L_001
 str r1, [r0, USART_DR]
 pop {r2}
 bx lr

  .size  WriteToUSART, .-WriteToUSART

  .section  .text.WriteToUSARTDMA
 .type WriteToUSARTDMA, %function
  WriteToUSARTDMA:
/*
//r0 addresses
--//r0 DMA BASE
--//r1 DMA Stream Offset
--//r2 USART BASE
//r1 address of data
//r2 size of data
*/

push {r3-r6}

ldr r3, [r0]
ldr r4, [r0, #4]
ldr r5, [r0, #8]

 ldr r6, [r3, DMA_HISR]
 str r6, [r3, DMA_HIFCR]

L_002:
 ldr r6, [r5, USART_SR]
 and r6, r6, #0x80
 cmp r6, #0
 beq L_002


add r6, r5, USART_DR
add r4, r4, r3
 str r6, [r4, DMA_PAR] // dma write is USART address
 str r1, [r4, DMA_M0AR] //dma read is outputString

 str r2, [r4, DMA_NDTR] // number of n-bytes to write
 mov r5, #0x0440 // auto increment read address and keep fixed write address, mem-to-per
 movt r5, #0x0803
 str r5, [r4, DMA_CR] //configure dma transfer
 ldr r5, [r4, DMA_FCR]
 bic r5, r5, #(0b100)
 orr r5, r5, #(0b101)
 str r5, [r4, DMA_FCR]
 ldr r5, [r4, DMA_CR]
 orr r5, r5, #1
 str r5, [r4, DMA_CR] //start dma transfer

L_003:
 ldr r5, [r4, DMA_CR]
 and r5, r5, #1
 cmp r5, #0
 bne L_003

ldr r3, [r0, #8]

L_004:
ldr r4, [r3, USART_SR]
and r5, r4, #(0x40)
cmp r5, #0
beq L_004
bic r4, r4, #(0x40)
str r4, [r3, USART_SR]

ldr r3, [r0]
 ldr r6, [r3, DMA_HISR]
 str r6, [r3, DMA_HIFCR]

pop {r3-r6}
bx lr


.size  WriteToUSARTDMA, .-WriteToUSARTDMA
