.syntax unified
.cpu cortex-m4
.thumb


.include "addresses.s"
.include "spi_addresses.s"

 .section .data
STRING:
    .asciz "AAAABBBBCCCCDDDDEEEEFFFFGGGG"

 .section  .text.EstablishClockSignal
 .type EstablishClockSignal, %function

EstablishClockSignal:
ldr r0, =RCC_APB1ENR
eor r1, r1
movt r1, #(0b1 << 12)
str r1, [r0] // enable pwr enable clock
ldr r0, =PWR_BASE
ldr r1, [r0]
orr r1, r1, #(0x3 << 14) // set the scale mode for the voltage regulartor to 1
str r1, [r0]
ldr r0, =RCC_BASE
ldr r1, [r0, RCC_CR]
mov r2, #(0x0001) // enable the HSI
orr r1, r1, r2
str r1, [r0, RCC_CR]
eor r1, r1
mov r1, #(0x0002)
checkHSI:
ldr r2, [r0, RCC_CR]
and r2, r2, r1
cmp r2, r1
bne checkHSI //wait for HSIRDY
ldr r2, [r0, RCC_CR]
orr r2, r2, #(0x10 << 3) //set calibration
ldr r2, [r0, RCC_CR]
bic r2, r2, #(1 << 24)
str r2, [r0, RCC_CR] //clear PLLENable
eor r1, r1
movt r1, #(0x0002 << 8)
checkPLL:
ldr r2, [r0, RCC_CR]
and r2, r2, r1
cmp r2, r1
beq checkPLL //wait for PLLRDY
eor r1, r1
orr r1, r1, #(0x0010) //pllm
orr r1, r1, #(0x168 << 6) //ppln
orr r1, r1, #(7 << 24) //pplq
orr r1, r1, #(6 << 28) //pplr
str r1, [r0, RCC_PLLCFGR]
ldr r1, [r0, RCC_CR]
orr r1, r1, #(0x1 << 24)
str r1, [r0, RCC_CR] // start PLLEnable
ldr r0, =PWR_BASE
ldr r1, [r0]
 /* Activate the OverDrive to reach the 180 MHz Frequency */
orr r1, r1, #(0x1 << 16) // set the overdrive enable, probably need to wait till ready
str r1, [r0]
ldr r0, =RCC_BASE
ldr r1, [r0, RCC_CFGR]
bic r1, r1, #(0xF3)
bic r1, r1, #(0xFC << 8)
orr r1, r1, #(0x02) // divide APB1 by 4, divide APB2 by 1, AHB divide by 1, SW = PLL_p
orr r1, r1, #(0x14 << 8)
str r1, [r0, RCC_CFGR]
ldr r0, =FIR_BASE
mov r1, #5
strb r1, [r0, FLASH_ACR] //set the flash latency
bx lr

.size  EstablishClockSignal, .-EstablishClockSignal

 .section  .text.GPIOSetup

  .type  GPIOSetup, %function
GPIOSetup:
  ldr r0, =GPIOA_BASE
  ldr r1, [r0, GPIO_MODER]
  bic r1, r1, #(0b11 << (2 * 9))   // Clear PA11
  orr r1, r1, #(0b01 << (2 * 9))   // Set PA3 togpio
  str r1, [r0, GPIO_MODER]

   ldr r1, [r0, GPIO_OSPEEDR]
  bic r1, r1, #(0b11 << (2 * 9))   // Clear PA3
  orr r1, r1, #(0b10 << (2 * 9))   // Set to high speed
  str r1, [r0, GPIO_OSPEEDR]

  ldr r1, [r0, GPIO_PUPDR]
  bic r1, r1, #(0b11 << (2 * 9))   // Clear PA11
  orr r1, r1, #(0b01 << (2 * 9))   // Set to pull up
  str r1, [r0, GPIO_PUPDR]

bx lr

.size  GPIOSetup, .-GPIOSetup


  .section  .text.main
  .weak  main
  .type  main, %function
main:

 ldr r0, =SCB
 eor r1, r1
 movt r1, #(0x00F0)  // give full access to fpu registers
 str r1, [r0, SCB_CPACR]

 push {lr}

 bl EstablishClockSignal

  bl InitializeDMA1

pop {lr}

  ldr r1, =0
  ldr r2, =#9600
  ldr r3, =#3
  ldr r4, =#45000000
  ldr r5, =#0

  push {r1-r5}

  mov r0, sp

  bl InitializeUSART2

  pop {r1-r5}

  bl GPIOSetup

  bl spi1SetupDMAStreams

  ldr r0, =6
  ldr r1, =0
  ldr r2, =0
  ldr r3, =0
  ldr r4, =0
  ldr r5, =0
  ldr r6, =3
  ldr r7, =0

  push {r0-r7}

  mov r0, sp

  bl spi1Init

  pop {r0-r7}



  sub sp, sp, #4


L_005:
  mov r0, sp
  mov r1, #4

  bl spi1StartDMAReception

  ldr r0, =DMA2_BASE
L_004:
  ldr r1, [r0, DMA_LISR]
  and r1, r1, #0x20
  cmp r1, #0
  beq L_004

  orr r1, r1, #0x10
  str r1, [r0, DMA_LIFCR]

  ldr r0, =STRING
  ldr r1, [sp]
  mov r4, r1

  bl spi1StartDMAReception

  ldr r0, =GPIOA_BASE
  ldr r1, =#(1 << 9)
  str r1, [r0, GPIO_BSRR]




  ldr r0, =DMA2_BASE
L_006:
  ldr r1, [r0, DMA_LISR]
  and r1, r1, #0x20
  cmp r1, #0
  beq L_006

  orr r1, r1, #0x10
  str r1, [r0, DMA_LIFCR]

  ldr r1, =DMA1_BASE
  ldr r2, =DMA_S6CR
  ldr r3, =USART2_BASE

  push {r1-r3}

  mov r0, sp

  ldr r1, =STRING
  mov r2, r4



  bl WriteToUSARTDMA

  pop {r1-r3}

  ldr r0, =GPIOA_BASE
  ldr r1, =#(1 << 25)
  str r1, [r0, GPIO_BSRR]

  b L_005

.size  main, .-main
