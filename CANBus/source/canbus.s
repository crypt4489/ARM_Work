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
.global canBUSReceive
.global canFSMInit
.global moveFSMCurrForward
.global packReceivedMessageInBuffer
.global createTransmitMessageBlk
.global copyBlockToCanBUSTransmitter
.global copyBlockFSM
.global manageTransmitBuffer

.equ CANBUSRECEIVEBLKSIZE, 0x10
.equ CANBUSTRANSMITBLKSIZE, 0x10

.equ CANBUSRECEIVETS, 0x00
.equ CANBUSRECEIVEFID, 0x04
.equ CANBUSRECEIVEDL, 0x08
.equ CANBUSRECEIVEADDR, 0x0C

.equ CANBUSTRANSMITSIZE, 0x00
.equ CANBUSTRANSMITADDR, 0x04
.equ CANBUSTRANSMITCURR, 0x08
.equ CANBUSTRANSMITFID, 0x0C

.equ RECEIVEBUFFERADDR, 0x00
.equ RECEIVEBUFFERCURR, 0x04
.equ RECEIVEBUFFERSIZE, 0x08
.equ RECEIVEBUFFERREAD, 0x0C

.equ BYTEBUFFERADDR, 0x00
.equ BYTEBUFFERCURR, 0x04
.equ BYTEBUFFERSIZE, 0x08

.equ TRANSMITBUFFERADDR, 0x00
.equ TRANSMITBUFFERCURR, 0x04
.equ TRANSMITBUFFERSIZE, 0x08
.equ TRANSMITBUFFERREAD, 0x0C

/* struct receiveblock {
  u32 timestamp
  u32 filter id
  u32 data length
  u32 addrofdata
  }
*/

/* struct transmitblk {
	u32 datasize
	u32 addr
	u32 currptr
	u32 filter
	}
*/
.section .canbus
ReceiveBuffer:
	.word 0x02
ReceiveBufferPointer:
	.word 0x00
ReceiveBufferSize:
	.word 0x00
ReceiveBufferRead:
	.word 0x00
ByteBuffer:
	.word 0x00
ByteBufferPointer:
	.word 0x00
ByteBufferSize:
	.word 0x00
TransmitBuffer:
	.word 0xFFFFFFFF
TransmitBufferPointer:
	.word 0xFFFFEEEE
TransmitBufferSize:
	.word 0xFFFFDDDD
TransmitBufferRead:
	.word 0xFFFFCCCC



.section .text.canFSMInit
	.type canFSMInit, %function
canFSMInit:
// r0 Base Buffer address
// r1 number of messages to store

	push {r2-r7}
	ldr r3, =ReceiveBuffer
	ldr r4, =ByteBuffer
	ldr r5, =TransmitBuffer
	mov r6, CANBUSTRANSMITBLKSIZE
	mul r7, r1, r6 //size of buffer
	str r0, [r5]
	str r0, [r5, TRANSMITBUFFERCURR]
	str r0, [r5, TRANSMITBUFFERREAD]
	str r7, [r5, TRANSMITBUFFERSIZE] //init tranmsit
	add r2, r0, r7
	str r2, [r3]
	str r2, [r3, RECEIVEBUFFERCURR]
	str r2, [r3, RECEIVEBUFFERREAD]
	mov r6, CANBUSRECEIVEBLKSIZE
	mul r7, r1, r6 //size of receive buffer
	str r7, [r3, RECEIVEBUFFERSIZE]
	add r2, r2, r7
	str r2, [r4]
	str r2, [r4, BYTEBUFFERCURR]
	mov r6, #8
	mul r7, r1, r6 //size of byte buffer
	str r7, [r4, BYTEBUFFERSIZE]
	pop {r2-r7}
	bx lr
.size canFSMInit, .-canFSMInit



.section .text.moveFSMCurrForward
	.type moveFSMCurrForward, %function
moveFSMCurrForward:
// r0 buffer base pointer
//r1 message size
//r2 offset
    push {r3-r6}
	ldr r3, [r0] //buffer base pointer
	ldr r4, [r0, r2] //buffer curr ptr
	ldr r5, [r0, #8] //buffersize
	add r6, r3, r5
	add r4, r4, r1
	cmp r4, r6
	bne storebuffptr
	mov r4, r3
storebuffptr:
	str r4, [r0, r2]
	pop {r3-r6}
	bx lr

.size moveFSMCurrForward, .-moveFSMCurrForward


.section .text.packReceivedMessageInBuffer
	.type packReceivedMessageInBuffer, %function
packReceivedMessageInBuffer:
//r0 canbase
//r1 receive index
	push {r2-r11}
	ldr r2, =CAN_RI0R
	mov r3, #0x10
	add r2, r2, r0
	mla r2, r3, r1, r2 //get reception address

	ldr r4, =ReceiveBufferPointer //cuurent pointer
	ldr r4, [r4]
	ldr r5, =ByteBufferPointer
	ldr r5, [r5]
	ldr r3, [r2]
	lsr r3, r3, #21
	str r3, [r4, CANBUSRECEIVEFID]
	ldr r3, [r2, #4]
	and r6, r3, #0xf
	lsr r3, r3, #16
	str r3, [r4, CANBUSRECEIVETS]
	str r6, [r4, CANBUSRECEIVEDL]
	str r5, [r4, CANBUSRECEIVEADDR] //store in message buffer

	mov r3, r5 //set up temp ptr
	add r8, r2, #8
	mov r2, r6
	b packloopcondition
packcopydataloop:
    ldr r9, [r8]
    add r8, r8, #4
    mov r10, #4
packbytecopyloop:
	uxtb r11, r9
	lsr r9, r9, #8
	sub r10, r10, #1
	strb r11, [r3]
	add r3, r3, #1
	sub r6, r6, #1
	cmp r6, #0
	beq packreceiveend
	cmp r10, #0
	bne packbytecopyloop
packloopcondition:
	cmp r6, #0
	bne packcopydataloop
packreceiveend:

	ldr r0, =ReceiveBuffer
	mov r1, CANBUSRECEIVEBLKSIZE
	mov r2, RECEIVEBUFFERCURR
	push {r0-r2, lr}
	bl moveFSMCurrForward
	pop {r0-r2, lr}
	ldr r0, =ByteBuffer
	mov r1, #8
	mov r2, BYTEBUFFERCURR
	push {r0-r2, lr}
	bl moveFSMCurrForward
	pop {r0-r2, lr}
	pop {r2-r11}
	bx lr

.size packReceivedMessageInBuffer, .-packReceivedMessageInBuffer

// add a blk to the buffer
.section .text.createTransmitMessageBlk
	.type createTransmitMessageBlk, %function
createTransmitMessageBlk:
//r0 data addr
//r1 data size
//r2 filter

	push {r3-r4}
	ldr r3, =TransmitBuffer
	ldr r4, [r3, #4]
	str r1, [r4, CANBUSTRANSMITSIZE]
	str r0, [r4, CANBUSTRANSMITADDR]
	str r0, [r4, CANBUSTRANSMITCURR]
	str r2, [r4, CANBUSTRANSMITFID]
	push {r0-r2, lr}
	mov r0, r3
	mov r1, CANBUSTRANSMITBLKSIZE
	mov r2, TRANSMITBUFFERCURR
	bl moveFSMCurrForward
	pop {r0-r2, lr}
	pop {r3-r4}
	bx lr

.size createTransmitMessageBlk, .-createTransmitMessageBlk


//copy block to transmitter
.section .text.copyBlockToCanBUSTransmitter
	.type copyBlockToCanBUSTransmitter, %function
copyBlockToCanBUSTransmitter:

//r0 canaddr
//r1 transmitter block 0, 1, 2
//r2 block address
	push {r3-r12}

	mov r3, #0x10
	mov r4, CAN_TI0R
	mla r4, r1, r3, r4

	add r4, r0, r4 // address of mailbox identifier register
	ldr r5, [r2, CANBUSTRANSMITFID] //filter
	lsl r6, r5, #21
	str r6, [r4]
	ldr r5, [r2, CANBUSTRANSMITSIZE] //data size
	ldr r6, [r2, CANBUSTRANSMITADDR] //datat start ptr
	ldr r7, [r2, CANBUSTRANSMITCURR] //data curr pointer
	add r8, r6, r5
	sub r9, r8, r7 //distance from end
	cmp r9, #8
	IT gt
	movgt r9, #8

	mov r5, CAN_TDT0R
	mla r5, r1, r3, r5
	add r5, r0, r5 // address of data length ctrl
	strb r9, [r5]

	mov r5, CAN_TDL0R
	mla r5, r1, r3, r5
	add r5, r0, r5 // address of mailbox low

	mov r6, #0
	mov r11, r9

bccopytransmitloop:
	ldrb r12, [r7]
	lsl r12, r12, r6
	add r10, r10, r12
	add r6, r6, #8
    cmp r6, #32
    bne bctransmitmovenext
    str r10, [r5]
    add r5, r5, #4
    mov r6, #0
    mov r10, #0
bctransmitmovenext:
	add r7, r7, #1
	sub r11, r11, #1
	cmp r7, r8
	beq bctransmitend
	cmp r11, #0
	bne bccopytransmitloop
	bal bcstarttransmission
bctransmitend:
    cmp r11, #0
	beq bcstarttransmission
	str r10, [r5]

bcstarttransmission:
	str r7, [r2, CANBUSTRANSMITCURR]
	mov r3, #1
	ldr r5, [r4]
	orr r5, r5, r3
	str r5, [r4]
	pop {r3-r12}
	bx lr



.size copyBlockToCanBUSTransmitter, .-copyBlockToCanBUSTransmitter


.section .text.copyBlockFSM
	.type copyBlockFSM, %function
copyBlockFSM:
//r0 cann addr
//r1 transmit blk to copy to

	push {r2-r6}

	ldr r2, =TransmitBuffer
	ldr r2, [r2, TRANSMITBUFFERREAD]

	push {r0-r2, lr}
	bl copyBlockToCanBUSTransmitter

	pop {r0-r2, lr}

	ldr r3, [r2, CANBUSTRANSMITADDR]
	ldr r4, [r2, CANBUSTRANSMITCURR]
	ldr r5, [r2, CANBUSTRANSMITSIZE]
	add r3, r5, r3
	mov r6, #0
	cmp r4, r3
	bne readheadtransmove

	ldr r0, =TransmitBuffer
	mov r1, CANBUSTRANSMITBLKSIZE
	mov r2, TRANSMITBUFFERREAD
	push {r0-r2, lr}
	bl moveFSMCurrForward
	pop {r0-r2, lr}
	mov r6, #1
readheadtransmove:
	mov r0, r6
	pop {r2-r6}
	bx lr
.size copyBlockFSM, .-copyBlockFSM

.section .text.manageTransmitBuffer
	.type manageTransmitBuffer, %function
manageTransmitBuffer:

// r0 CAN ADDR

	push {r1-r5}
	add r1, r0, CAN_TSR
	ldr r2, [r1]
	mov r3, #0
	movt r3, #0x1C00
	and r2, r3, r2
	lsr r2, r2, #26
	movs r4, #-1
manageloop:
	adds r4, r4, #1
	and r5, r2, #1
	cmp r5, #0
	bne callcopyblockmanage
	lsr r2, r2, #1
checkmanageloop:
	cmp r2, #0
	bne manageloop
	movs r0, #-1
	b endmanage
callcopyblockmanage:
	mov r1, r4
	push {r0-r1, lr}
	bl copyBlockFSM
	pop {r0-r1, lr}
	mov r0, #0
endmanage:
	pop {r1-r5}
	bx lr

.size manageTransmitBuffer, .-manageTransmitBuffer


.section .text.canBUSPeripheralInit
  	.type canBUSPeripheralInit, %function
canBUSPeripheralInit:

	push {r1-r2}
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
	pop {r1-r2}
 	bx lr

 .size  canBUSPeripheralInit, .-canBUSPeripheralInit



.section .text.canBUSInit
  	.type canBUSInit, %function
canBUSInit:
	push {r1-r2}
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




	pop {r1-r2}

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
	push {r2-r12}
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

	pop {r2-r12}
	bx lr

.size  canFilterSet, .-canFilterSet



.section .text.canExitInit
  .type canExitInit, %function

canExitInit:
	push {r1-r2}
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
	pop {r1-r2}
	bx lr

 .size  canExitInit, .-canExitInit


.section .text.canBUSTransmit
  .type canBUSTransmit, %function
canBUSTransmit:

// r0 cnabase
// r1 mailbox
// r2 identifier
// r3 datalength
// r4 address of data

	push {r5-r12}
	add r5, r0, CAN_TI0R
	mov r6, #0x10
	mla r7, r6, r1, r5
transmitempty:
	ldr r8, [r7]
	and r8, r8, #1
	cmp r8, #1
	beq transmitempty

	add r5, r0, CAN_TDT0R
	mla r8, r6, r1, r5


	and r3, r3, #0xf

	str r3, [r8] //set data frame length

	add r5, r0, CAN_TDL0R
	mla r8, r6, r1, r5
	mov r9, r4
	mov r11, #0
	mov r12, #0
	b copytransmitloopcheck
copytransmitloop:
	ldrb r10, [r9]
	lsl r10, r10, r12
	add r11, r11, r10
	add r12, r12, #8
    cmp r12, #32
    bne transmitmovenext
    str r11, [r8]
    add r8, r8, #4
    mov r12, #0
    mov r11, #0
transmitmovenext:
	add r9, r9, #1
	sub r3, r3, #1
copytransmitloopcheck:
	cmp r3, #0
	bne copytransmitloop
    cmp r12, #0
    beq transmitend
	str r11, [r8]
transmitend:
	mov r8, #1
	lsl r9, r2, #21
    orr r9, r9, r8


	str r9, [r7] // start message with identifier of 1

	pop {r5-r12}
	bx lr

 .size  canBUSTransmit, .-canBUSTransmit



 .section .text.canBUSReceive
  .type canBUSReceive, %function
canBUSReceive:

//r0 can base
//r1 address to write data
//r2 mailbox id
//r3 identifer
	push {r4-r11}
	ldr r4, =CAN_RI0R

	mov r5, #0x10
	cmp r2, #0
	blt byidentifier
	mla r4, r5, r2, r4
	add r5, r4, r0
	b read_data

byidentifier:
	add r7, r4, r0
	ldr r6, [r7]
	lsr r6, r6, #21
	cmp r6, r3
	beq endidentifier
byidentifier2:
   	add r4, r5, r4
   	add r7, r4, r0
   	ldr r6, [r7]
	lsr r6, r6, #21
	cmp r6, r3
	beq endidentifier
	pop {r5-r12}
	mov r0, #-1
	bx lr
endidentifier:
	add r5, r0, r4

read_data:
	ldr r6, [r5, #4]
	and r6, r6, #0xf
	mov r7, r1
	add r8, r5, #8
	b loopcondition
copydataloop:
    ldr r9, [r8]
    add r8, r8, #4
    mov r10, #4
bytecopyloop:

	uxtb r11, r9
	lsr r9, r9, #8
	sub r10, r10, #1
	strb r11, [r7]
	add r7, r7, #1
	sub r6, r6, #1
	cmp r6, #0
	beq receiveend
	cmp r10, #0
	bne bytecopyloop

loopcondition:
	cmp r6, #0
	bne copydataloop
receiveend:
	pop {r5-r12}
	mov r0, #0
	bx lr

 .size  canBUSReceive, .-canBUSReceive


