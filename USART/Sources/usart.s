 .syntax unified
.cpu cortex-m4
.thumb

.include "addresses.s"

.global InitializeUSART2
.global WriteToUSART

 .section  .text.InitializeUSART2
 .type InitializeUSART2, %function
//r0 Data for USART
//r0 data format
//r1 baud rate
//r2 tx/rx dma
 InitializeUSART2:

 push {r0-r2}
 ldr r0, =RCC_AHB1ENR
 ldr r1, [r0]
 orr r1, r1, #0x0001
 str r1, [r0]

 ldr r0, =GPIOA_BASE


 ldr r1, [r0, GPIO_MODER]
 bic r1, r1, #(0b11 << (2 * 2))   // Clear PB6
 orr r1, r1, #(0b10 << (2 * 2))   // Set PB6 to alternate function
 str r1, [r0, GPIO_MODER]


  ldr r1, [r0, GPIO_PUPDR]
  bic r1, r1, #(0x3 << (2 * 2)) // clear pull up down resiter
  orr r1, r1, #(0b01 << (2 * 2))   // Set PB6 tro pull up
  str r1, [r0, GPIO_PUPDR]


  ldr r1, [r0, GPIO_AFRL]
  bic r1, r1, #(0xF << (2 * 4))
  orr r1, r1, #(0x7 << (2 * 4))
  str r1, [r0, GPIO_AFRL]


 ldr r0, =RCC_APB1ENR
 ldr r2, =#0x20000  //enable USART2
 ldr r1, [r0]
 orr r1, r1, r2
 str r1, [r0]


 ldr r0, =USART2_BASE

 ldr r1, [r0, USART_CR1]
 orr r1, r1, #0x2000 // 8 bit word
 str r1, [r0, USART_CR1]

 mov r1, #0x124f
 str r1, [r0, USART_BRR] // set baud rate of 9600


 ldr r1, [r0, USART_CR1]
 orr r1, r1, #0x0008 // Start USART
 str r1, [r0, USART_CR1]
 pop {r0-r1}
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
