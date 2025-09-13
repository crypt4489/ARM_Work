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
.global canInterruptSetup
.global canBUSReceive
.global canFSMInit
.global moveFSMCurrForward
.global packReceivedMessageInBuffer
.global createTransmitMessageBlk
.global copyBlockToCanBUSTransmitter
.global copyBlockFSM
.global manageTransmitBuffer
.global manageReceiveBuffer
.global manageReceiveBufferINT
.global retrieveMessageBlock
.global can1TXInterrupt
.global can1RX0Interrupt
.global initializeCAN1NVIC
.global canInterruptSetup

.equ CANBUSRECEIVEBLKSIZE, 0x10
.equ CANBUSTRANSMITBLKSIZE, 0x10

.equ CANBUSRECEIVETS, 0x00
.equ CANBUSRECEIVEFID, 0x04
.equ CANBUSRECEIVESIZE, 0x08
.equ CANBUSRECEIVEADDR, 0x0C

.equ CANBUSTRANSMITSIZE, 0x00
.equ CANBUSTRANSMITADDR, 0x04
.equ CANBUSTRANSMITCURR, 0x08
.equ CANBUSTRANSMITFID, 0x0C

.equ CANBUSRECEIVERETBLKID, 0x00
.equ CANBUSRECEIVERETBLKSIZE, 0x04
.equ CANBUSRECEIVERETBLKDATA, 0x08

.equ RECEIVEBUFFERADDR, 0x00
.equ RECEIVEBUFFERCURR, 0x04
.equ RECEIVEBUFFERSIZE, 0x08
.equ RECEIVEBUFFERREAD, 0x0C
.equ RECEIVEBUFFERCOUNT, 0x10

.equ BYTEBUFFERADDR, 0x00
.equ BYTEBUFFERCURR, 0x04
.equ BYTEBUFFERSIZE, 0x08
.equ BYTEBUFFERCOUNT, 0x0c

.equ TRANSMITBUFFERADDR, 0x00
.equ TRANSMITBUFFERCURR, 0x04
.equ TRANSMITBUFFERSIZE, 0x08
.equ TRANSMITBUFFERREAD, 0x0C
.equ TRANSMITBUFFERCOUNT, 0x10

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

/*
struct receivepopblovk
{
	u32 id
	u32 length
	u8 data[8]
};



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
ReceiveBufferCount:
	.word 0x0
ByteBuffer:
	.word 0xFFEECCDD
ByteBufferPointer:
	.word 0xAABBCCDD
ByteBufferSize:
	.word 0x00
ByteBufferCount:
	.word 0x00
TransmitBuffer:
	.word 0xFFFFFFFF
TransmitBufferPointer:
	.word 0xFFFFEEEE
TransmitBufferSize:
	.word 0xFFFFDDDD
TransmitBufferRead:
	.word 0xFFFFCCCC
TransmitBufferCount:
	.word 0x0
CAN1OR2:
	.word 0x0


.macro ENABLERXINTERRUPT IntReg1:req, IntReg2:req, IntReg3:req, IntNum:req

	ldr \IntReg1, =CAN1OR2
	ldr \IntReg1, [\IntReg1]
	ldr \IntReg2, =NVIC_ISER0
	cmp \IntReg1, #0
	ITTE ne
	movne \IntReg1, #0
	addne \IntReg2, \IntReg2, #8
	moveq \IntReg1, #20

	add \IntReg1, \IntReg1, \IntNum //which rx number
	mov \IntReg3, #1
	lsl \IntReg1, \IntReg3, \IntReg1
	ldr \IntReg3, [\IntReg2]
	orr \IntReg3, \IntReg3, \IntReg1
	str \IntReg3, [\IntReg2]

.endm


.macro DISABLERXINTERRUPT IntReg1:req, IntReg2:req, IntReg3:req, IntNum:req

	ldr \IntReg1, =CAN1OR2
	ldr \IntReg1, [\IntReg1]
	ldr \IntReg2, =NVIC_ICER0
	cmp \IntReg1, #0
	ITTE ne
	movne \IntReg1, #0
	addne \IntReg2, \IntReg2, #8
	moveq \IntReg1, #20
	mov \IntReg3, #1
	lsl \IntReg1, \IntReg3, \IntReg1
	ldr \IntReg3, =#0
	orr \IntReg3, \IntReg3, \IntReg1
	str \IntReg3, [\IntReg2]

.endm

.macro ENABLETXINTERRUPT IntReg1:req, IntReg2:req, IntReg3:req

	ldr \IntReg1, =CAN1OR2
	ldr \IntReg1, [\IntReg1]
	ldr \IntReg2, =NVIC_ISER0
	cmp \IntReg1, #0
	ITTE ne
	movtne \IntReg1, #0x8000
	addne \IntReg2, \IntReg2, #8
	movteq \IntReg1, #0x0008

	ldr \IntReg3, [\IntReg2]
	orr \IntReg3, \IntReg3, \IntReg1
	str \IntReg3, [\IntReg2]

.endm


.macro DISABLETXINTERRUPT IntReg1:req, IntReg2:req, IntReg3:req

	ldr \IntReg1, =CAN1OR2
	ldr \IntReg1, [\IntReg1]
	ldr \IntReg2, =NVIC_ICER0
	cmp \IntReg1, #0
	ITTTE ne
	movne \IntReg1, #0
	movtne \IntReg1, #0x8000
	addne \IntReg2, \IntReg2, #8
	movteq \IntReg1, #0x0008

	ldr \IntReg3, =0
	orr \IntReg3, \IntReg3, \IntReg1
	str \IntReg3, [\IntReg2]

.endm


.section .text.initializeCAN1NVIC
	.type initializeCAN1NVIC, %function
initializeCAN1NVIC:
//r0 Interrupt Counter
	push {r1-r2}
	ldr r1, =NVIC_ISER0
	ldr r2, [r1]
	cmp r0, #0
	beq endinitializecan1nvic
	orr r2, r2, #(1 << 19)
	lsr r0, r0, #1
	cmp r0, #0
	beq endinitializecan1nvic
	orr r2, r2, #(1 << 20)
	lsr r0, r0, #1
	cmp r0, #0
	beq endinitializecan1nvic
	orr r2, r2, #(1 << 21)


endinitializecan1nvic:
	str r2, [r1]
	pop {r1-r2}
	bx lr

.size initializeCAN1NVIC, .-initializeCAN1NVIC


.section .text.can1TXInterrupt
	.type can1TXInterrupt, %function
can1TXInterrupt:


	push {r0-r3}
	ldr r0, =NVIC_ICPR0
	ldr r1, [r0]
	orr r1, r1, #(0x1 << 19)
	str r1, [r0]

	ldr r0, =CAN1_BASE
	ldr r1, [r0, CAN_TSR]
	mov r2, #0x0101
	movt r2, #1
	orr r1, r1, r2
	str r1, [r0, CAN_TSR]
	ldr r2, =TransmitBufferPointer
	ldr r3, =TransmitBufferRead
	ldr r2, [r2]
	ldr r3, [r3]
	cmp r2, r3
	beq can1TXIntteruptEnd

	push {r0, lr}

	bl manageTransmitBuffer

	pop { r0, lr }

can1TXIntteruptEnd:


	pop {r0-r3}
	bx lr
.size can1TXInterrupt, .-can1TXInterrupt


.section .text.can1RX0Interrupt
	.type can1RX0Interrupt, %function
can1RX0Interrupt:

	push {r0-r3}
	ldr r0, =NVIC_ICPR0
	ldr r1, [r0]
	orr r1, r1, #(0x1 << 20)
	str r1, [r0]

	ldr r0, =CAN1_BASE
	ldr r1, =CAN_RF0R
	mov r2, #0


	push { lr }

	bl manageReceiveBufferINT  //pop all messages off fifo queue

	pop { lr }

can1RX0IntteruptEnd:


	pop {r0-r3}
	bx lr

.size can1RX0Interrupt, .-can1RX0Interrupt

.section .text.can1RX1Interrupt
	.type can1RX1Interrupt, %function
can1RX1Interrupt:

	push {r0-r3}
	ldr r0, =NVIC_ICPR0
	ldr r1, [r0]
	orr r1, r1, #(0x1 << 21)
	str r1, [r0]

	ldr r0, =CAN1_BASE
	ldr r1, =CAN_RF1R
	mov r2, #0


	push { lr }

	bl manageReceiveBufferINT  //pop all messages off fifo queue

	pop { lr }

can1RX1IntteruptEnd:


	pop {r0-r3}
	bx lr

.size can1RX1Interrupt, .-can1RX1Interrupt


.section .text.can2TXInterrupt
	.type can2TXInterrupt, %function
can2TXInterrupt:

	push {r0-r3}
	ldr r0, =NVIC_ICPR1
	ldr r1, [r0]
	bic r1, r1, #(0x1 << 31)

	ldr r2, =TransmitBufferPointer
	ldr r3, =TransmitBufferRead
	ldr r2, [r2]
	ldr r3, [r3]
	cmp r2, r3
	beq can2TXIntteruptEnd
	push {r0-r1, lr}
	ldr r0, =CAN2_BASE
	bl manageTransmitBuffer
	pop {r0-r1, lr}
can2TXIntteruptEnd:
	str r1, [r0]
	pop {r0-r3}
	bx lr

.size can2TXInterrupt, .-can2TXInterrupt


.section .text.can2RX0Interrupt
	.type can2RX0Interrupt, %function
can2RX0Interrupt:

.size can2RX0Interrupt, .-can2RX0Interrupt

.section .text.can2RX1Interrupt
	.type can2RX1Interrupt, %function
can2RX1Interrupt:

.size can2RX1Interrupt, .-can2RX1Interrupt


.section .text.canFSMInit
	.type canFSMInit, %function
canFSMInit:
// r0 Base Buffer address
// r1 number of messages to store
// r2 can #
	push {r3-r8}
	ldr r3, =ReceiveBuffer
	ldr r4, =ByteBuffer
	ldr r5, =TransmitBuffer
	mov r6, CANBUSTRANSMITBLKSIZE
	mul r7, r1, r6 //size of buffer
	str r0, [r5]
	str r0, [r5, TRANSMITBUFFERCURR]
	str r0, [r5, TRANSMITBUFFERREAD]
	str r7, [r5, TRANSMITBUFFERSIZE] //init tranmsit
	add r8, r0, r7
	str r8, [r3]
	str r8, [r3, RECEIVEBUFFERCURR]
	str r8, [r3, RECEIVEBUFFERREAD]
	mov r6, CANBUSRECEIVEBLKSIZE
	mul r7, r1, r6 //size of receive buffer
	str r7, [r3, RECEIVEBUFFERSIZE]
	add r8, r8, r7
	str r8, [r4]
	str r8, [r4, BYTEBUFFERCURR]
	mov r6, #8
	mul r7, r1, r6 //size of byte buffer
	str r7, [r4, BYTEBUFFERSIZE]

	ldr r3, =CAN1OR2
	str r2, [r3]
	pop {r3-r8}
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

	ldr r4, =ReceiveBuffer //cuurent pointer
	ldr r2, [r4, RECEIVEBUFFERCOUNT]
	mov r3, CANBUSRECEIVEBLKSIZE
	mul r3, r3, r2
	ldr r2, [r4, RECEIVEBUFFERSIZE]
	cmp r2, r3
	bne readmessagrbuffer

	movs r0, #-1
	b packreceivefinal


readmessagrbuffer:
	ldr r2, =CAN_RI0R
	mov r3, #0x10
	add r2, r2, r0
	mla r2, r3, r1, r2 //get reception address

	ldr r4, [r4, RECEIVEBUFFERCURR]
	ldr r5, =ByteBuffer
	ldr r5, [r5, BYTEBUFFERCURR]
	ldr r3, [r2]
	lsr r3, r3, #21
	str r3, [r4, CANBUSRECEIVEFID]
	ldr r3, [r2, #4]
	and r6, r3, #0xf
	lsr r3, r3, #16
	str r3, [r4, CANBUSRECEIVETS]
	str r6, [r4, CANBUSRECEIVESIZE]
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
	ldr r1, [r0, RECEIVEBUFFERCOUNT]
	add r1, r1, #1
	str r1, [r0, RECEIVEBUFFERCOUNT]
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
	mov r0, #0
packreceivefinal:
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

	push {r3-r5}
	DISABLETXINTERRUPT r3, r4, r5
	ldr r3, =TransmitBuffer
	ldr r4, [r3, TRANSMITBUFFERCOUNT]
	mov r5, CANBUSTRANSMITBLKSIZE
	mul r5, r5, r4
	ldr r5, [r3, TRANSMITBUFFERSIZE]
	cmp r4, r5
	bne nooverflowoftbuffer
	mov r0, #-1
	b endofcreatetransmit

nooverflowoftbuffer:
	ldr r4, [r3, TRANSMITBUFFERCURR]
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
	ldr r2, [r3, TRANSMITBUFFERCOUNT]
	add r2, r2, #1
	str r2, [r3, TRANSMITBUFFERCOUNT]

	ldr r0, =NVIC_ISPR0
	ldr r1, [r0]
	orr r1, r1, #(1 << 19)
	str r1, [r0]


	mov r0, #0

endofcreatetransmit:
	ENABLETXINTERRUPT r3, r4, r5
	pop {r3-r5}
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
	ldr r1, [r0, TRANSMITBUFFERCOUNT]
	sub r1, r1, #1
	str r1, [r0, TRANSMITBUFFERCOUNT]
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
	b checktmanageloop
tmanageloop:
	adds r4, r4, #1
	and r5, r2, #1
	cmp r5, #0
	bne callcopyblocktmanage
	lsr r2, r2, #1
checktmanageloop:
	cmp r2, #0
	bne tmanageloop
	movs r0, #-1
	b endtmanage
callcopyblocktmanage:
	mov r1, r4
	push {r0-r1, lr}
	bl copyBlockFSM
	pop {r0-r1, lr}
	mov r0, r1
endtmanage:
	pop {r1-r5}
	bx lr

.size manageTransmitBuffer, .-manageTransmitBuffer


.section .text.manageReceiveBuffer
	.type manageReceiveBuffer, %function
manageReceiveBuffer:

// r0 CAN_ADDR
	push {r1-r6}
	add r1, r0, CAN_RF0R
	mov r2, #0 //
	ldr r3, [r1]
	and r3, r3, #3
	cmp r3, #0
	bne getmessagesfromrfifo
	add r2, r2, #1
	add r1, r1, #4
	ldr r3, [r1]
	and r3, r3, #3
	cmp r3, #0
	bne getmessagesfromrfifo

	movs r0, #-1
	b checkrmessagesend

getmessagesfromrfifo:
	mov r4, r1
	mov r1, r2
	mov r2, r3
	movt r5, #0
	mov r5, #0x0020
rmessagesloop:
	push {r0-r1, lr}
	bl packReceivedMessageInBuffer
	pop {r0-r1, lr}
	ldr r6, [r4]
	orr r6, r5, r6
	str r6, [r4]
checkrmessagesloop:
	sub r2, r2, #1
	cmp r2, #0
	bne rmessagesloop
	mov r0, #0
checkrmessagesend:
	pop {r1-r6}
 	bx lr
.size manageReceiveBuffer, .-manageReceiveBuffer


.section .text.manageReceiveBufferINT
	.type manageReceiveBufferINT, %function
manageReceiveBufferINT:

// r0 CAN_ADDR
// r1 CAN_RF#R
// r2 RECEIVE INDEX
	push {r3-r6}
	add r1, r0, r1
	ldr r3, [r1]
	and r3, r3, #3

	mov r4, r1
	mov r1, r2
	mov r2, r3
	movt r5, #0
	mov r5, #0x0020
rmessagesloopint:
	push {r0-r1, lr}
	bl packReceivedMessageInBuffer
	pop {r0-r1, lr}
	ldr r6, [r4]
	orr r6, r5, r6
	str r6, [r4]

	sub r2, r2, #1
	cmp r2, #0
	bne rmessagesloopint
	mov r0, #0
	pop {r3-r6}
 	bx lr
.size manageReceiveBufferINT, .-manageReceiveBufferINT



.section .text.retrieveMessageBlock
	.type retrieveMessageBlock, %function
retrieveMessageBlock:
//r0 address to move the structure to
	push {r1-r5}
	DISABLERXINTERRUPT r1, r2, r3, #0
	DISABLERXINTERRUPT r1, r2, r3, #1
	ldr r1, =ReceiveBufferPointer
	ldr r2, [r1, RECEIVEBUFFERCOUNT]
	cmp r2, #0
	bne proceedwithpoppingrblk
	movs r0, #-1
	b retrieveMessageBlockEnd

proceedwithpoppingrblk:
	sub r2, r2, #1
	str r2, [r1, RECEIVEBUFFERCOUNT]
	ldr r2, [r1, RECEIVEBUFFERREAD]
	ldr r3, [r2, CANBUSRECEIVEFID]
	str r3, [r0, CANBUSRECEIVERETBLKID]
	ldr r3, [r2, CANBUSRECEIVERETBLKSIZE]
	str r3, [r0, CANBUSRECEIVERETBLKID]
	add r0, r0, CANBUSRECEIVERETBLKDATA
	ldr r4, [r2, CANBUSRECEIVEADDR]
	cmp r3, #4
	ldr r5, [r4], #4
	str r5, [r0], #4
	ble poppingend
morereceivedatapop:
	ldr r5, [r4]
	str r5, [r0]
poppingend:
	mov r0, #0
retrieveMessageBlockEnd:
	ENABLERXINTERRUPT r1, r2, r3, #0
	ENABLERXINTERRUPT r1, r2, r3, #1
	pop {r1-r5}
	bx lr
.size retrieveMessageBlock, .-retrieveMessageBlock

.section .text.canInterruptSetup
	.type canInterruptSetup, %function
canInterruptSetup:
//r0 can base
//r1 can mask

	add r2, r0, CAN_IER
	ldr r3, [r2]
	orr r3, r3, r1
	str r3, [r2]
	bx lr

.size canInterruptSetup, .-canInterruptSetup

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
	mov r0, #-1
	b receiveend
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
	mov r0, #0
receiveend:
	pop {r5-r12}

	bx lr

 .size  canBUSReceive, .-canBUSReceive


