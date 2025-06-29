.syntax unified
.cpu cortex-m4
.thumb

.include "canbus_addresses.s"
.include "addresses.s"

.global canBUSPeripheralInit
.global canBUSInit
.global canBUSTransmit
.global canExitInit
.global canFilterSet


.section .text.canBUSPeripheralInit
  	.type canBUSPeripheralInit, %function
canBUSPeripheralInit:


 	ldr r1, =RCC_AHB1ENR

 	cmp r0, #0

 	bne CANBUSBPINIT

CANBUSAPINIT:

 	ldr r2, [r1]
 	orr r2, r2, #0x0001 //enable GPIOA clock
 	str r2, [r1]

 	ldr r0, =GPIOA_BASE

 	ldr r1, [r0]
 	bic r1, r1, #(0b1111 << (11 * 2))   // Clear PA3
    orr r1, r1, #(0b1010 << (11 * 2))   // Set PA3 Receiver
    str r1, [r0]


 	ldr r1, [r0, #0xC]
 	bic r1, r1, #(0x33 << (11 * 2)) // clear pull up resiter
 	orr r1, r1, #(0b0101 << (11 * 2))   // Set PA3 to pull up
 	str r1, [r0, #0xC]

 	ldr r1, [r0, GPIO_AFRH]

 	mov r2, #(9 << 12)
 	movt r2, #9

 	orr r1, r1, r2
 	str r1, [r0, GPIO_AFRH]

 	ldr r0, =RCC_APB1ENR
	ldr r1, [r0]
	orr r1, r1, #(1 << 25) // enable CANBUSA Clock
	str r1, [r0]

	b CANBUSPEND

CANBUSBPINIT:
	ldr r2, [r1]
 	orr r2, r2, #0x0002 //enable GPIOB clock
 	str r2, [r1]

 	ldr r0, =GPIOB_BASE

 	ldr r1, [r0, GPIO_MODER]
 	bic r1, r1, #(0b1111 << (12 * 2))   // Clear Pb12/13
    orr r1, r1, #(0b1010 << (12 * 2))   // Set pb12/13 alternate
    str r1, [r0, GPIO_MODER]


 	ldr r1, [r0, GPIO_PUPDR]
 	bic r1, r1, #(0x33 << (12 * 2)) // clear pull up resiter
 	orr r1, r1, #(0b0101 << (12 * 2))   // Set pb12/13 to pull up
 	str r1, [r0, GPIO_PUPDR]

 	ldr r1, [r0, GPIO_AFRH]
 	movt r2, #0x99
 	orr r1, r1, r2
 	str r1, [r0, GPIO_AFRH]

 	ldr r0, =RCC_APB1ENR
	ldr r1, [r0]
	orr r1, r1, #(1 << 26) // enable CANBUSB Clock
	str r1, [r0]

CANBUSPEND:
 	bx lr

 .size  canBUSPeripheralInit, .-canBUSPeripheralInit

.section .text.canBUSInit
  	.type canBUSInit, %function
canBUSInit:

	ldr r1, [r0, CAN_MCR]
	bic r1, r1, #0x2
	str r1, [r0, CAN_MCR] //clear sleep

SLEEPLOOP:
	ldr r1, [r0, CAN_MSR]
	and r2, r1, #2
	cmp r2, #0
	beq SLEEPLOOP

	ldr r1, [r0, CAN_MCR]
	orr r1, r1, #0x0001
	str r1, [r0, CAN_MCR] // request initialization
ACKLoop:
	ldr r1, [r0, CAN_MSR]
	and r2, r1, #1
	cmp r2, #0
	beq ACKLoop //now in initializationb mode

	ldr r1, =#0x403c0004  //prescalar of 5 (4+1), TS1 of 14 (14+1) and TS2 of 2(3+1), SJW is 1
	str r1, [r0, CAN_BTR] // set up for 500 k bit and set in loop back mode






	bx lr

 .size  canBUSInit, .-canBUSInit

.section .text.canFilterSet
  .type canFilterSet, %function
canFilterSet:
//r0 can address,
//r1 address of following values

//r1 filter number
//r2 identifier mode
//r3 mode config
//r4 fifo assignment
//r5 fitler id
//r6 filter mask

	ldr r6, [r1, #20]
	ldr r5, [r1, #16]
	ldr r4, [r1, #12]
	ldr r3, [r1, #8]
	ldr r2, [r1, #4]
	ldr r1, [r1]

	ldr r7, [r0, CAN_FA1R]
	mov r8, #1
	lsl r8, r8, r1
	bic r7, r7, r8
	str r7, [r0, CAN_FA1R]


	ldr r7, [r0, CAN_FM1R]
	ldr r9, [r0, CAN_FS1R]
	ldr r11, [r0, CAN_FFA1R]
	mov r8, #1
	lsl r8, r8, r1
	bic r7, r7, r8
	bic r9, r9, r8
	bic r11, r11, r8
	lsl r8, r2, r1
	lsl r10, r3, r1
	lsl r12, r4, r1
	orr r7, r7, r8
	orr r9, r9, r10
	orr r11, r11, r12
	str r7, [r0, CAN_FM1R] // set identifier mode to mask mde
	str r9, [r0, CAN_FS1R] // set to single 32-bit config
	str r11, [r0, CAN_FFA1R]

	ldr r2, =CAN_F0R1
	mov r3, #8
	mul r3, r3, r1 //address of canfilter register
	add r3, r3, r2

	lsl r5, r5, #21
	str r5, [r0, r3] //set fdilter id of only one

	add r3, r3, #4
	lsl r6, r6, #21
	str r6, [r0, r3] //set can filter to match all 11 bits

	ldr r2, [r0, CAN_FA1R]
	mov r3, #1
	lsl r3, r3, r1
	orr r2, r2, r3
	str r2, [r0, CAN_FA1R] //activate filer

	bx lr

.size  canFilterSet, .-canFilterSet



.section .text.canExitInit
  .type canExitInit, %function

canExitInit:

	ldr r1, [r0, CAN_FMR]
	bic r1, r1, #1
	str r1, [r0, CAN_FMR] //end the filter setup init

  	ldr r1, [r0, CAN_MCR]
	bic r1, r1, #1
	str r1, [r0, CAN_MCR] //request normal mode
ACKLoop2:
	ldr r1, [r0, CAN_MSR]
	and r2, r1, #1
	cmp r2, #0
	bne ACKLoop2 // entered normal mode
	bx lr

 .size  canExitInit, .-canExitInit


.section .text.canBUSTransmit
  .type canBUSTransmit, %function
canBUSTransmit:
	ldr r1, =CAN1_BASE
transmitempty:
	ldr r2, [r1, CAN_TI0R]
	and r3, r2, #1
	cmp r3, #1
	beq transmitempty

	mov r2, #1
	str r2, [r1, CAN_TDT0R] //set data frame length
	and r0, r0, #0xff
	str r0, [r1, CAN_TDL0R] //set data

	mov r2, #1
	movt r2, #0x0020
	//mov r2, #0
	//str r2, [r1, CAN_RI0R] // setup identifier

	str r2, [r1, CAN_TI0R] // start message with identifier of 1
	bx lr

 .size  canBUSTransmit, .-canBUSTransmit


