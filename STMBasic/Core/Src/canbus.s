.syntax unified
.cpu cortex-m4
.thumb

.include "canbus_addresses.s"
.include "addresses.s"

.global canBUSInit
.global canBUSTransmit


.section .text.canBUSInit
  	.type canBUSInit, %function
canBUSInit:

	ldr r0, =RCC_AHB1ENR

 	ldr r1, [r0]
 	orr r1, r1, #0x0001 //enable GPIOA clock
 	str r1, [r0]

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
	orr r1, r1, #(1 << 25)
	str r1, [r0]

	ldr r0, =CAN1_BASE
	ldr r1, [r0, CAN_MCR]
	bic r1, r1, #0x2

	str r1, [r0, CAN_MCR] //request initiialization
	orr r1, r1, #0x0001
	str r1, [r0, CAN_MCR]
ACKLoop:
	ldr r1, [r0, CAN_MSR]
	and r2, r1, #1
	cmp r2, #0
	beq ACKLoop //now in initializationb mode
	//ldr r1, =#0x00000
	//ldr r2, [r0, CAN_MCR]
	//movt r2, 0
	//str r2, [r0, CAN_MCR] // setup debug mode

	ldr r1, =#0x403c0004  //prescalar of 5 (4+1), TS1 of 14 (14+1) and TS2 of 2(3+1), SJW is 1
	str r1, [r0, CAN_BTR] // set up for 500 k bit and set in loop back mode


	ldr r1, [r0, CAN_FM1R]
	bic r1, r1, #1
	str r1, [r0, CAN_FM1R] // set identifier mode to mask mde

	ldr r1, [r0, CAN_FS1R]
	orr r1, r1, #1
	str r1, [r0, CAN_FS1R] // set to single 32-bit config

	ldr r1, [r0, CAN_FFA1R]
	bic r1, r1, #1
	str r1, [r0, CAN_FFA1R] // set to single 32-bit config

	mov r1, #0
	movt r1, (0x1 << 5)
	str r1, [r0, CAN_F0R1] //set fdilter id of only one

	mov r1, #0
	movt r1, (0x7ff << 5)
	str r1, [r0, CAN_F0R2] //set can filter to match all 11 bits

	ldr r1, [r0, CAN_FA1R]
	orr r1, r1, #0x1
	str r1, [r0, CAN_FA1R] //activate filer

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

 .size  canBUSInit, .-canBUSInit

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


