/**
  ******************************************************************************
  * @file      startup_stm32f446xx.s
  * @author    MCD Application Team
  * @brief     STM32F446xx Devices vector table for GCC based toolchains.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M4 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m4
  .fpu fpv4-sp-d16
  .thumb

.include "addresses.s"

.global  g_pfnVectors
.global  Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word  _sidata
/* start address for the .data section. defined in linker script */
.word  _sdata
/* end address for the .data section. defined in linker script */
.word  _edata
/* start address for the .bss section. defined in linker script */
.word  _sbss
/* end address for the .bss section. defined in linker script */
.word  _ebss
/* stack used for SystemInit_ExtMemCtl; always internal RAM used */


    .section  .text.TIM9_Handler
TIM9_Handler:
	PUSH {r0-r1, lr}
	LDR r0, =TIM9_BASE
    LDR r1, [r0, TIM9_SR]
    BIC r1, r1, #0x0001
    STR r1, [r0, TIM9_SR]
    LDR r0, =NVIC_ICPR0
	MOV r1, #(1 << 24)
	STR r1, [r0]
	ldr r1, =0x00000020
	ldr r0, =0x40020000
	//bl  HAL_GPIO_TogglePin
	pop {r0-r1, lr}
	bx lr
  .size  TIM9_Handler, .-TIM9_Handler



  	.section .text.ToggleHandler
  	.weak ToggleHandler
  	.type ToggleHandler, %function
 ToggleHandler:
  ldr r0, =RCC_APB2ENR 				//enable the tim9 clock
  ldr r1, [r0]
  orr r1, r1, #0x10000
  str r1, [r0]
  ldr r0, =TIM9_BASE
  									//; Set Prescaler (PSC) → 1 µs tick with 84 MHz clock
  mov r1, #0xff00                   //; Prescaler = 83
  str r1, [r0, TIM9_PSC]            // ; Store PSC

  									// Set Auto-Reload (ARR) → 1 ms overflow
  mov r1, #1000                 	// ARR = 999
  str r1, [r0, TIM9_ARR]             // Store ARR
  ldr r2, [r0, TIM9_DIER]           //Interrupt Setting for TIM9
  orr r2, r2, #0x0001
  str r2, [r0, TIM9_DIER]
  mov r2, #0
  str r2, [r0, TIM9_SMCR] // disable slave mode
  ldr r2, [r0] //TIM9_CR1 enable timer
  orr r2, r2, #0x0001
  str r2, [r0]
  ldr r0, =NVIC_ISER0 // interrupt set enable for NVIC
  mov r1, #1 << 24
  str r1, [r0]
  bx lr
  	.size ToggleHandler, .-ToggleHandler

// this fails because sb13 and sb14 are fused
  	.section .text.PWMTIM9
  	.weak PWMTIM9
  	.type PWMTIM9, %function
PWMTIM9:

  ldr r0, =GPIOA_BASE
  ldr r1, [r0, #0x20]       //; GPIOx_AFRL
  bic r1, r1, #(0xF << (4 * 3))     //; Clear AF bits for PA2
  orr r1, r1, #(0x3 << (4 * 3))     //; AF3 for TIM9_CH1
  str r1, [r0, #0x20]

  ldr r1, [r0]
  bic r1, r1, #(0b11 << (2 * 3))   // Clear PA3
  orr r1, r1, #(0b10 << (2 * 3))   // Set PA3 to alternate function
  str r1, [r0]

  ldr r0, =RCC_APB2ENR 				//enable the tim9 clock
  ldr r1, [r0]
  orr r1, r1, #0x10000
  str r1, [r0]
  ldr r0, =TIM9_BASE

  mov r1, #83                   //; Prescaler = 83
  str r1, [r0, TIM9_PSC]            // ; Store PSC

  									// Set Auto-Reload (ARR) → 1 ms overflow
  mov r1, #16000                	// ARR = 999
  str r1, [r0, TIM9_ARR]             // Store ARR

  mov r1, #8000
  str r1, [r0, TIM9_CCR2]


  mov r1, #(0x0068 << 8)
  str r1, [r0, TIM9_CCMR1]


  ldr r2, [r0, TIM9_CCER]
  orr r2, r2, #0x0010
  str r2, [r0, TIM9_CCER] //TIM9_CCER ARP prenable

  ldr r2, [r0]  //TIM9_CR1 ARR preload
  orr r2, r2, #0x0080
  str r2, [r0]

  ldr r2, [r0, TIM9_EGR]  //TIM9_CR1 write to shadow regs
  orr r2, r2, #0x0001
  str r2, [r0, TIM9_EGR]



  ldr r2, [r0]  //TIM9_CR1 enable timer
  orr r2, r2, #0x0001
  str r2, [r0]



  bx lr

  	.size PWMTIM9, .-PWMTIM9



  	.section .text.PWMTIM4
  	.weak PWMTIM4
  	.type PWMTIM4, %function
PWMTIM4:



  ldr r0, =RCC_APB1ENR 	    //enable the tim4 clock
  ldr r1, [r0]
  orr r1, r1, #0x0004
  str r1, [r0]

  ldr r0, =RCC_AHB1ENR       //gpiob enable clock peripheral
  ldr r1, [r0]
  orr r1, r1, #0x0002
  str r1, [r0]


  ldr r0, =GPIOB_BASE


  ldr r1, [r0]
  bic r1, r1, #(0b11 << (2 * 6))   // Clear PB6
  orr r1, r1, #(0b10 << (2 * 6))   // Set PB6 to alternate function
  str r1, [r0]


  ldr r1, [r0, #0xC]
  bic r1, r1, #(0x3 << (2*6)) // clear pull up down resiter
  orr r1, r1, #(0b01 << (2 * 6))   // Set PB6 tro pull up
  str r1, [r0, #0xC]


  ldr r1, [r0, #0x20]       //; GPIOx_AFRH
  bic r1, r1, #(0xF << (6 * 4))
  orr r1, r1, #(0x2 << (6 * 4))
  str r1, [r0, #0x20]


  ldr r0, =TIM4_BASE

  mov r1, #10000                 //; Prescaler = 83
  str r1, [r0, TIM4_PSC]            // ; Store PSC

  									// Set Auto-Reload (ARR) → 1 ms overflow
  mov r1, #16000               	// ARR = 999
  str r1, [r0, TIM4_ARR]             // Store ARR

  ldr r2, [r0]  //TIM4_CR1 ARR preload
  orr r2, r2, #0x0080
  str r2, [r0]

  ldr r2, [r0, TIM4_EGR]  //TIM4_CR1 write to shadow regs
  orr r2, r2, #0x0001
  str r2, [r0, TIM4_EGR]



  mov r1, #8000
  str r1, [r0, TIM4_CCR1] //50 duty cycle

  mov r1, #(0x0068)
  str r1, [r0, TIM4_CCMR1] // pwm mode 6

  ldr r1, [r0, TIM4_DIER]
  orr r1, r1, #0x0001
  str r1, [r0, TIM4_DIER]

  ldr r2, [r0, TIM4_CCER]
  orr r2, r2, #0x0001
  str r2, [r0, TIM4_CCER] //TIM4_CCER ARP prenable

   ldr r2, [r0]  //TIM9_CR1 enable timer
  orr r2, r2, #0x0001
  str r2, [r0]

  ldr r1, =NVIC_ISER0 // interrupt set enable for NVIC
  mov r2, #1 << 30
  str r2, [r1]





  bx lr







//pa7 adc in7
  	.section .text.ADCSETUP
  	.weak ADCSETUP
  	.type ADCSETUP, %function
ADCSETUP:



  ldr r0, =RCC_AHB1ENR       //gpioa enable clock peripheral
  ldr r1, [r0]
  orr r1, r1, #0x0001
  str r1, [r0]

  ldr r0, =RCC_APB2ENR
  ldr r1, [r0]
  orr r1, r1, #0x0100 // enable ADC1 clock
  str r1, [r0]

  ldr r0, =GPIOA_BASE
  ldr r1, [r0]
  orr r1, r1, #(0b11 << (2 * 7))   // Set PA0 to analog function
  str r1, [r0]

  ldr r0, =ADC_BASE
  ldr r1, [r0, ADC_CCR]
  bic r1, r1, #(0b11 << 16) // set prescalar for adc all config
  orr r1, r1, #(0b10 << 16)
  str r1, [r0, ADC_CCR]
  ldr r1, [r0, ADC_SMPR2]
  //bic r1, r1, #(0b111 << (3 * 7))   // Clear adc_in7
  orr r1, r1, #(0b111 << (3 * 7))   // set sampling period
  str r1, [r0, ADC_SMPR2]
  ldr r1, [r0, ADC_SQR1]
  bic r1, r1, #(0xf << 20); // set total reg count to 1 (0)
  str r1, [r0, ADC_SQR1]
  ldr r1, [r0, ADC_SQR3]
  orr r1, r1, #0x7  //set the position in conversion sequence for first poll
  str r1, [r0, ADC_SQR3]
  ldr r1, [r0, ADC_CR2]
  orr r1, r1, #3
  str r1, [r0, ADC_CR2] // cont and adc on
  orr r1, r1, #(0b1 << 30) // start regular conversion
  str r1, [r0, ADC_CR2]
  mov r1, #0b00010
  vmov.f32 s1, #0.5

 loop:

  ldr r2, [r0, ADC_SR]
  and r2, r2, #0b00010
  //cbz r2, loop //too far away
  cmp r2, r1
  bne loop
  mov r2, #0
  str r2, [r0, ADC_SR]
  ldr r2, [r0, ADC_DR]
  vmov s0, r2
  vcvt.f32.u32 s0, s0
  vmul.f32 s2, s0, s1
  b loop
  bx lr

  .size ADCSETUP, .-ADCSETUP



	.section .text.ADC1EnableFlicker
	.weak ADC1EnableFlicker
	.type ADC1EnableFlicker, %function
ADC1EnableFlicker:

  PUSH { lr }

  bl PWMTIM4

  POP { lr }

  ldr r0, =RCC_AHB1ENR       //gpioa enable clock peripheral
  ldr r1, [r0]
  orr r1, r1, #0x0001
  str r1, [r0]
  ldr r0, =RCC_APB2ENR
  ldr r1, [r0]
  orr r1, r1, #0x0100 // enable ADC1 clock
  str r1, [r0]
  ldr r0, =GPIOA_BASE
  ldr r1, [r0]
  orr r1, r1, #(0b11 << (2 * 7))   // Set PA0 to analog function
  str r1, [r0]
  ldr r0, =ADC_BASE
  ldr r1, [r0, ADC_CCR]
  bic r1, r1, #(0b11 << 16) // set prescalar for adc all config
  orr r1, r1, #(0b10 << 16)
  str r1, [r0, ADC_CCR]
  ldr r1, [r0, ADC_SMPR2]
  orr r1, r1, #(0b111 << (3 * 7))   // set sampling period
  str r1, [r0, ADC_SMPR2]
  ldr r1, [r0, ADC_SQR1]
  bic r1, r1, #(0xf << 20); // set total reg count to 1 (0)
  str r1, [r0, ADC_SQR1]
  ldr r1, [r0, ADC_SQR3]
  orr r1, r1, #0x7  //set the position in conversion sequence for first poll
  str r1, [r0, ADC_SQR3]
  ldr r1, [r0, ADC_CR2]
  orr r1, r1, #3
  str r1, [r0, ADC_CR2] // cont and adc on
  orr r1, r1, #(0b1 << 30) // start regular conversion
  str r1, [r0, ADC_CR2]



  bx lr

	.size ADC1EnableFlicker, .-ADC1EnableFlicker

	.section .text.TIM4_IRQHandler
  	.weak TIM4_IRQHandler
  	.type TIM4_IRQHandler, %function
TIM4_IRQHandler:
  PUSH { r0-r5 }
  VPUSH { s0-s3 }
  LDR r0, =TIM4_BASE
  LDR r1, [r0, TIM4_SR]
  BIC r1, r1, #0x0001
  STR r1, [r0, TIM4_SR]
  LDR r0, =NVIC_ICPR0
  MOV r1, #(1 << 30)
  STR r1, [r0]
  ldr r0, =ADC_BASE
  ldr r2, [r0, ADC_SR]
  mov r1, #(0b00010)
  and r2, r2, r1
  cmp r2, r1
  bne LABEL2  // check if adc has finsihed conversion eoc


  ldr r3, =#4000
  mov r5, #12000
  vmov s4, r5
  vcvt.f32.u32 s4, s4
  mov r4, #0xfff
  vmov s3, r4
  vcvt.f32.u32 s3, s3


  mov r2, #0
  str r2, [r0, ADC_SR] // clear eoc
  ldr r2, [r0, ADC_DR]
  vmov s0, r2
  vcvt.f32.u32 s0, s0
  vdiv.f32 s1, s0, s3 // ADC_DR / 2^12-1
  vmul.f32 s2, s1, s4 // adc% * 16000-12000
  vcvt.u32.f32 s2, s2
  vmov r2, s2
  add r2, r2, r3
  ldr r0, =TIM4_BASE
  str r2, [r0, TIM4_ARR]             // Store ARR
  lsr r2, r2, #1
  str r2, [r0, TIM4_CCR1] //50 duty cycle
 LABEL2:

  VPOP { s0-s3 }
  POP {r0-r5 }
  bx lr

  	.size TIM4_IRQHandler, .-TIM4_IRQHandler

 	.section .text.PA11TEST
  	.weak PA11TEST
  	.type PA11TEST, %function
PA11TEST:
  ldr r0, =GPIOA_BASE
  ldr r1, [r0]
  bic r1, r1, #(0b11 << (2 * 11))   // Clear PA11
  orr r1, r1, #(0b01 << (2 * 11))   // Set PA3 togpio
  str r1, [r0]

   ldr r1, [r0, 0x08]
  bic r1, r1, #(0b11 << (2 * 11))   // Clear PA3
  orr r1, r1, #(0b10 << (2 * 11))   // Set to high speed
  str r1, [r0, #0x08]

  ldr r1, [r0, 0x0C]
  bic r1, r1, #(0b11 << (2 * 11))   // Clear PA11
  orr r1, r1, #(0b01 << (2 * 11))   // Set to pull up
  str r1, [r0, #0x0C]


  ldr r1, =#(1 << 11)
  str r1, [r0, #0x18]

  bx lr

 // ldr r1, [r0, #0x24]       //; GPIOx_AFRH
//  bic r1, r1, #(0xF << (3 * 4))
 // orr r1, r1, #(0x2 << (3 * 4))
 // str r1, [r0, #0x24]

 	.size PA11TEST, .-PA11TEST

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

    .section  .text.AllBeginning
  .weak  AllBeginning
  .type  AllBeginning, %function
AllBeginning:
  ldr   sp, =_estack      /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

 ldr r0, =SCB
 eor r1, r1
 movt r1, #(0x00F0)  // give full access to fpu registers
 str r1, [r0, SCB_CPACR]

 bl EstablishClockSignal

 bl canBUSInit

 mov r0, 55

 bl canBUSTransmit

LABEL:
  b LABEL
.size  AllBeginning, .-AllBeginning

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 * @param  None
 * @retval None
*/
    .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b  Infinite_Loop
  .size  Default_Handler, .-Default_Handler



/******************************************************************************
*
* The minimal vector table for a Cortex M3. Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
*******************************************************************************/
   .section  .isr_vector,"a",%progbits
  .type  g_pfnVectors, %object


g_pfnVectors:
  .word  _estack
  .word  AllBeginning

  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler

  /* External Interrupts */
  .word     WWDG_IRQHandler                   /* Window WatchDog              */
  .word     PVD_IRQHandler                    /* PVD through EXTI Line detection */
  .word     TAMP_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */
  .word     RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */
  .word     FLASH_IRQHandler                  /* FLASH                        */
  .word     RCC_IRQHandler                    /* RCC                          */
  .word     EXTI0_IRQHandler                  /* EXTI Line0                   */
  .word     EXTI1_IRQHandler                  /* EXTI Line1                   */
  .word     EXTI2_IRQHandler                  /* EXTI Line2                   */
  .word     EXTI3_IRQHandler                  /* EXTI Line3                   */
  .word     EXTI4_IRQHandler                  /* EXTI Line4                   */
  .word     DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */
  .word     DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */
  .word     DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */
  .word     DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */
  .word     DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */
  .word     DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */
  .word     DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */
  .word     ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */
  .word     CAN1_TX_IRQHandler                /* CAN1 TX                      */
  .word     CAN1_RX0_IRQHandler               /* CAN1 RX0                     */
  .word     CAN1_RX1_IRQHandler               /* CAN1 RX1                     */
  .word     CAN1_SCE_IRQHandler               /* CAN1 SCE                     */
  .word     EXTI9_5_IRQHandler                /* External Line[9:5]s          */
  .word     TIM1_BRK_TIM9_IRQHandler          /* TIM1 Break and TIM9          */
  .word     TIM1_UP_TIM10_IRQHandler          /* TIM1 Update and TIM10        */
  .word     TIM1_TRG_COM_TIM11_IRQHandler     /* TIM1 Trigger and Commutation and TIM11 */
  .word     TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */
  .word     TIM2_IRQHandler                   /* TIM2                         */
  .word     TIM3_IRQHandler                   /* TIM3                         */
  .word     TIM4_IRQHandler                   /* TIM4                         */
  .word     I2C1_EV_IRQHandler                /* I2C1 Event                   */
  .word     I2C1_ER_IRQHandler                /* I2C1 Error                   */
  .word     I2C2_EV_IRQHandler                /* I2C2 Event                   */
  .word     I2C2_ER_IRQHandler                /* I2C2 Error                   */
  .word     SPI1_IRQHandler                   /* SPI1                         */
  .word     SPI2_IRQHandler                   /* SPI2                         */
  .word     USART1_IRQHandler                 /* USART1                       */
  .word     USART2_IRQHandler                 /* USART2                       */
  .word     USART3_IRQHandler                 /* USART3                       */
  .word     EXTI15_10_IRQHandler              /* External Line[15:10]s        */
  .word     RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */
  .word     OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI line */
  .word     TIM8_BRK_TIM12_IRQHandler         /* TIM8 Break and TIM12         */
  .word     TIM8_UP_TIM13_IRQHandler          /* TIM8 Update and TIM13        */
  .word     TIM8_TRG_COM_TIM14_IRQHandler     /* TIM8 Trigger and Commutation and TIM14 */
  .word     TIM8_CC_IRQHandler                /* TIM8 Capture Compare         */
  .word     DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */
  .word     FMC_IRQHandler                    /* FMC                          */
  .word     SDIO_IRQHandler                   /* SDIO                         */
  .word     TIM5_IRQHandler                   /* TIM5                         */
  .word     SPI3_IRQHandler                   /* SPI3                         */
  .word     UART4_IRQHandler                  /* UART4                        */
  .word     UART5_IRQHandler                  /* UART5                        */
  .word     TIM6_DAC_IRQHandler               /* TIM6 and DAC1&2 underrun errors */
  .word     TIM7_IRQHandler                   /* TIM7                         */
  .word     DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */
  .word     DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */
  .word     DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */
  .word     DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */
  .word     DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */
  .word     0                                 /* Reserved                     */
  .word     0                                 /* Reserved                     */
  .word     CAN2_TX_IRQHandler                /* CAN2 TX                      */
  .word     CAN2_RX0_IRQHandler               /* CAN2 RX0                     */
  .word     CAN2_RX1_IRQHandler               /* CAN2 RX1                     */
  .word     CAN2_SCE_IRQHandler               /* CAN2 SCE                     */
  .word     OTG_FS_IRQHandler                 /* USB OTG FS                   */
  .word     DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */
  .word     DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */
  .word     DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */
  .word     USART6_IRQHandler                 /* USART6                       */
  .word     I2C3_EV_IRQHandler                /* I2C3 event                   */
  .word     I2C3_ER_IRQHandler                /* I2C3 error                   */
  .word     OTG_HS_EP1_OUT_IRQHandler         /* USB OTG HS End Point 1 Out   */
  .word     OTG_HS_EP1_IN_IRQHandler          /* USB OTG HS End Point 1 In    */
  .word     OTG_HS_WKUP_IRQHandler            /* USB OTG HS Wakeup through EXTI */
  .word     OTG_HS_IRQHandler                 /* USB OTG HS                   */
  .word     DCMI_IRQHandler                   /* DCMI                         */
  .word     0                                 /* Reserved                     */
  .word     0                                 /* Reserved                     */
  .word     FPU_IRQHandler                    /* FPU                          */
  .word     0                                 /* Reserved                     */
  .word     0                                 /* Reserved                     */
  .word     SPI4_IRQHandler                   /* SPI4                         */
  .word     0                                 /* Reserved                     */
  .word     0                                 /* Reserved                     */
  .word     SAI1_IRQHandler                   /* SAI1                         */
  .word     0                                 /* Reserved                     */
  .word     0                                 /* Reserved                     */
  .word     0                                 /* Reserved                     */
  .word     SAI2_IRQHandler                   /* SAI2                         */
  .word     QUADSPI_IRQHandler                /* QuadSPI                      */
  .word     CEC_IRQHandler                    /* CEC                          */
  .word     SPDIF_RX_IRQHandler               /* SPDIF RX                     */
  .word     FMPI2C1_EV_IRQHandler          /* FMPI2C 1 Event               */
  .word     FMPI2C1_ER_IRQHandler          /* FMPI2C 1 Error               */


  .size  g_pfnVectors, .-g_pfnVectors

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/
   .weak      NMI_Handler
   .thumb_set NMI_Handler,Default_Handler

   .weak      HardFault_Handler
   .thumb_set HardFault_Handler,Default_Handler

   .weak      MemManage_Handler
   .thumb_set MemManage_Handler,Default_Handler

   .weak      BusFault_Handler
   .thumb_set BusFault_Handler,Default_Handler

   .weak      UsageFault_Handler
   .thumb_set UsageFault_Handler,Default_Handler

   .weak      SVC_Handler
   .thumb_set SVC_Handler,Default_Handler

   .weak      DebugMon_Handler
   .thumb_set DebugMon_Handler,Default_Handler

   .weak      PendSV_Handler
   .thumb_set PendSV_Handler,Default_Handler

   .weak      SysTick_Handler
   .thumb_set SysTick_Handler,Default_Handler

   .weak      WWDG_IRQHandler
   .thumb_set WWDG_IRQHandler,Default_Handler

   .weak      PVD_IRQHandler
   .thumb_set PVD_IRQHandler,Default_Handler

   .weak      TAMP_STAMP_IRQHandler
   .thumb_set TAMP_STAMP_IRQHandler,Default_Handler

   .weak      RTC_WKUP_IRQHandler
   .thumb_set RTC_WKUP_IRQHandler,Default_Handler

   .weak      FLASH_IRQHandler
   .thumb_set FLASH_IRQHandler,Default_Handler

   .weak      RCC_IRQHandler
   .thumb_set RCC_IRQHandler,Default_Handler

   .weak      EXTI0_IRQHandler
   .thumb_set EXTI0_IRQHandler,Default_Handler

   .weak      EXTI1_IRQHandler
   .thumb_set EXTI1_IRQHandler,Default_Handler

   .weak      EXTI2_IRQHandler
   .thumb_set EXTI2_IRQHandler,Default_Handler

   .weak      EXTI3_IRQHandler
   .thumb_set EXTI3_IRQHandler,Default_Handler

   .weak      EXTI4_IRQHandler
   .thumb_set EXTI4_IRQHandler,Default_Handler

   .weak      DMA1_Stream0_IRQHandler
   .thumb_set DMA1_Stream0_IRQHandler,Default_Handler

   .weak      DMA1_Stream1_IRQHandler
   .thumb_set DMA1_Stream1_IRQHandler,Default_Handler

   .weak      DMA1_Stream2_IRQHandler
   .thumb_set DMA1_Stream2_IRQHandler,Default_Handler

   .weak      DMA1_Stream3_IRQHandler
   .thumb_set DMA1_Stream3_IRQHandler,Default_Handler

   .weak      DMA1_Stream4_IRQHandler
   .thumb_set DMA1_Stream4_IRQHandler,Default_Handler

   .weak      DMA1_Stream5_IRQHandler
   .thumb_set DMA1_Stream5_IRQHandler,Default_Handler

   .weak      DMA1_Stream6_IRQHandler
   .thumb_set DMA1_Stream6_IRQHandler,Default_Handler

   .weak      ADC_IRQHandler
   .thumb_set ADC_IRQHandler,Default_Handler

   .weak      CAN1_TX_IRQHandler
   .thumb_set CAN1_TX_IRQHandler,Default_Handler

   .weak      CAN1_RX0_IRQHandler
   .thumb_set CAN1_RX0_IRQHandler,Default_Handler

   .weak      CAN1_RX1_IRQHandler
   .thumb_set CAN1_RX1_IRQHandler,Default_Handler

   .weak      CAN1_SCE_IRQHandler
   .thumb_set CAN1_SCE_IRQHandler,Default_Handler

   .weak      EXTI9_5_IRQHandler
   .thumb_set EXTI9_5_IRQHandler,Default_Handler

   /*.weak      TIM1_BRK_TIM9_IRQHandler
   .thumb_set TIM1_BRK_TIM9_IRQHandler,Default_Handler*/

   .weak      TIM1_BRK_TIM9_IRQHandler
   .thumb_set TIM1_BRK_TIM9_IRQHandler,TIM9_Handler

   .weak      TIM1_UP_TIM10_IRQHandler
   .thumb_set TIM1_UP_TIM10_IRQHandler,Default_Handler

   .weak      TIM1_TRG_COM_TIM11_IRQHandler
   .thumb_set TIM1_TRG_COM_TIM11_IRQHandler,Default_Handler

   .weak      TIM1_CC_IRQHandler
   .thumb_set TIM1_CC_IRQHandler,Default_Handler

   .weak      TIM2_IRQHandler
   .thumb_set TIM2_IRQHandler,Default_Handler

   .weak      TIM3_IRQHandler
   .thumb_set TIM3_IRQHandler,Default_Handler

   .weak      TIM4_IRQHandler
   .thumb_set TIM4_IRQHandler,USARTBitBangingInt

   .weak      I2C1_EV_IRQHandler
   .thumb_set I2C1_EV_IRQHandler,Default_Handler

   .weak      I2C1_ER_IRQHandler
   .thumb_set I2C1_ER_IRQHandler,Default_Handler

   .weak      I2C2_EV_IRQHandler
   .thumb_set I2C2_EV_IRQHandler,Default_Handler

   .weak      I2C2_ER_IRQHandler
   .thumb_set I2C2_ER_IRQHandler,Default_Handler

   .weak      SPI1_IRQHandler
   .thumb_set SPI1_IRQHandler,Default_Handler

   .weak      SPI2_IRQHandler
   .thumb_set SPI2_IRQHandler,Default_Handler

   .weak      USART1_IRQHandler
   .thumb_set USART1_IRQHandler,Default_Handler

   .weak      USART2_IRQHandler
   .thumb_set USART2_IRQHandler,Default_Handler

   .weak      USART3_IRQHandler
   .thumb_set USART3_IRQHandler,Default_Handler

   .weak      EXTI15_10_IRQHandler
   .thumb_set EXTI15_10_IRQHandler,Default_Handler

   .weak      RTC_Alarm_IRQHandler
   .thumb_set RTC_Alarm_IRQHandler,Default_Handler

   .weak      OTG_FS_WKUP_IRQHandler
   .thumb_set OTG_FS_WKUP_IRQHandler,Default_Handler

   .weak      TIM8_BRK_TIM12_IRQHandler
   .thumb_set TIM8_BRK_TIM12_IRQHandler,Default_Handler

   .weak      TIM8_UP_TIM13_IRQHandler
   .thumb_set TIM8_UP_TIM13_IRQHandler,Default_Handler

   .weak      TIM8_TRG_COM_TIM14_IRQHandler
   .thumb_set TIM8_TRG_COM_TIM14_IRQHandler,Default_Handler

   .weak      TIM8_CC_IRQHandler
   .thumb_set TIM8_CC_IRQHandler,Default_Handler

   .weak      DMA1_Stream7_IRQHandler
   .thumb_set DMA1_Stream7_IRQHandler,Default_Handler

   .weak      FMC_IRQHandler
   .thumb_set FMC_IRQHandler,Default_Handler

   .weak      SDIO_IRQHandler
   .thumb_set SDIO_IRQHandler,Default_Handler

   .weak      TIM5_IRQHandler
   .thumb_set TIM5_IRQHandler,Default_Handler

   .weak      SPI3_IRQHandler
   .thumb_set SPI3_IRQHandler,Default_Handler

   .weak      UART4_IRQHandler
   .thumb_set UART4_IRQHandler,Default_Handler

   .weak      UART5_IRQHandler
   .thumb_set UART5_IRQHandler,Default_Handler

   .weak      TIM6_DAC_IRQHandler
   .thumb_set TIM6_DAC_IRQHandler,Default_Handler

   .weak      TIM7_IRQHandler
   .thumb_set TIM7_IRQHandler,Default_Handler

   .weak      DMA2_Stream0_IRQHandler
   .thumb_set DMA2_Stream0_IRQHandler,Default_Handler

   .weak      DMA2_Stream1_IRQHandler
   .thumb_set DMA2_Stream1_IRQHandler,Default_Handler

   .weak      DMA2_Stream2_IRQHandler
   .thumb_set DMA2_Stream2_IRQHandler,Default_Handler

   .weak      DMA2_Stream3_IRQHandler
   .thumb_set DMA2_Stream3_IRQHandler,Default_Handler

   .weak      DMA2_Stream4_IRQHandler
   .thumb_set DMA2_Stream4_IRQHandler,Default_Handler

   .weak      CAN2_TX_IRQHandler
   .thumb_set CAN2_TX_IRQHandler,Default_Handler

   .weak      CAN2_RX0_IRQHandler
   .thumb_set CAN2_RX0_IRQHandler,Default_Handler

   .weak      CAN2_RX1_IRQHandler
   .thumb_set CAN2_RX1_IRQHandler,Default_Handler

   .weak      CAN2_SCE_IRQHandler
   .thumb_set CAN2_SCE_IRQHandler,Default_Handler

   .weak      OTG_FS_IRQHandler
   .thumb_set OTG_FS_IRQHandler,Default_Handler

   .weak      DMA2_Stream5_IRQHandler
   .thumb_set DMA2_Stream5_IRQHandler,Default_Handler

   .weak      DMA2_Stream6_IRQHandler
   .thumb_set DMA2_Stream6_IRQHandler,Default_Handler

   .weak      DMA2_Stream7_IRQHandler
   .thumb_set DMA2_Stream7_IRQHandler,Default_Handler

   .weak      USART6_IRQHandler
   .thumb_set USART6_IRQHandler,Default_Handler

   .weak      I2C3_EV_IRQHandler
   .thumb_set I2C3_EV_IRQHandler,Default_Handler

   .weak      I2C3_ER_IRQHandler
   .thumb_set I2C3_ER_IRQHandler,Default_Handler

   .weak      OTG_HS_EP1_OUT_IRQHandler
   .thumb_set OTG_HS_EP1_OUT_IRQHandler,Default_Handler

   .weak      OTG_HS_EP1_IN_IRQHandler
   .thumb_set OTG_HS_EP1_IN_IRQHandler,Default_Handler

   .weak      OTG_HS_WKUP_IRQHandler
   .thumb_set OTG_HS_WKUP_IRQHandler,Default_Handler

   .weak      OTG_HS_IRQHandler
   .thumb_set OTG_HS_IRQHandler,Default_Handler

   .weak      DCMI_IRQHandler
   .thumb_set DCMI_IRQHandler,Default_Handler

   .weak      FPU_IRQHandler
   .thumb_set FPU_IRQHandler,Default_Handler

   .weak      SPI4_IRQHandler
   .thumb_set SPI4_IRQHandler,Default_Handler

   .weak      SAI1_IRQHandler
   .thumb_set SAI1_IRQHandler,Default_Handler

   .weak      SAI2_IRQHandler
   .thumb_set SAI2_IRQHandler,Default_Handler

   .weak      QUADSPI_IRQHandler
   .thumb_set QUADSPI_IRQHandler,Default_Handler

   .weak      CEC_IRQHandler
   .thumb_set CEC_IRQHandler,Default_Handler

   .weak      SPDIF_RX_IRQHandler
   .thumb_set SPDIF_RX_IRQHandler,Default_Handler

   .weak      FMPI2C1_EV_IRQHandler
   .thumb_set FMPI2C1_EV_IRQHandler,Default_Handler

   .weak      FMPI2C1_ER_IRQHandler
   .thumb_set FMPI2C1_ER_IRQHandler,Default_Handler
