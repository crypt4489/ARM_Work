/*
 * clock.s
 *
 *  Created on: Apr 22, 2025
 *      Author: dflet
 */
.syntax unified
.cpu cortex-m4
.thumb

.global EstablishClockSignal
.include "addresses.s"

/* RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 6;

 Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; */

//enabled clock (mostly hal refactor)
//establish a MAX Overdrive frequency with PLL as clock source and
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


