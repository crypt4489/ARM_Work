.syntax unified
.cpu cortex-m4
.thumb

.include "canbus_addresses.s"
.include "addresses.s"

.global canBUSPeripheralInit
.global canBUSInit
.global canExitInit
.global canFilterSet
.global canInterruptSetup
.global can1FSMInit
.global can2FSMInit
.global createTransmitMessageBlkCAN1
.global createTransmitMessageBlkCAN2
.global retrieveMessageBlockCAN1
.global retrieveMessageBlockCAN2
.global can1TXInterrupt
.global can1RX0Interrupt
.global can1RX1Interrupt
.global can2TXInterrupt
.global can2RX0Interrupt
.global can2RX1Interrupt
.global initializeCAN1NVIC
.global initializeCAN2NVIC
.global canInterruptSetup
.global can1PumpMessages
.global can2PumpMessages

.equ CANBUSRECEIVEBLKSIZE, 0x10
.equ CANBUSTRANSMITBLKSIZE, 0x10
.equ CANBUSFSMDATASIZE, 0x38

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

.equ CANFSMRECEIVE, 0x00
.equ CANFSMBYTE, 0x14
.equ CANFSMTRANSMIT, 0x24

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


struct CANFSMBLock
{
	u32 ReceiveBuffer
	u32 ReceiveBufferPtr
	u32 ReceiveBufferSize
	u32 ReceiveBufferRead
	u32 ReceiveBufferCount
	u32 ByteBuffer
	u32 ByteBufferPointer
	u32 ByteBufferSize
	u32 ByteBufferCount
	u32 TransmitBuffer
	u32 TransmitBufferPointer
	u32 TransmitBufferSize
	u32 TransmitBufferRead
	u32 TransmitBufferCount
}

*/
.section .canbus
.align 2
CAN1FSM:
	.word 0x0
CAN2FSM:
	.word 0x0


.macro ENABLERXINTERRUPT IntReg1:req, IntReg2:req, IntReg3:req, IntNum:req, CAN1O2:req
	ldr \IntReg2, =NVIC_ISER0
	mov \IntReg1, \CAN1O2
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


.macro DISABLERXINTERRUPT IntReg1:req, IntReg2:req, IntReg3:req, IntNum:req, CAN1O2:req

	ldr \IntReg2, =NVIC_ICER0
	mov \IntReg1, \CAN1O2
	cmp \IntReg1, #0
	ITTE ne
	movne \IntReg1, #0
	addne \IntReg2, \IntReg2, #8
	moveq \IntReg1, #20
	add \IntReg1, \IntReg1, \IntNum //which rx number
	mov \IntReg3, #1
	lsl \IntReg1, \IntReg3, \IntReg1
	ldr \IntReg3, =#0
	orr \IntReg3, \IntReg3, \IntReg1
	str \IntReg3, [\IntReg2]

.endm

.macro ENABLETXINTERRUPT IntReg1:req, IntReg2:req, IntReg3:req, CAN1O2:req

	ldr \IntReg2, =NVIC_ISER0
	mov \IntReg1, \CAN1O2
	cmp \IntReg1, #0
	ITTE ne
	movtne \IntReg1, #0x8000
	addne \IntReg2, \IntReg2, #4
	movteq \IntReg1, #0x0008

	ldr \IntReg3, [\IntReg2]
	orr \IntReg3, \IntReg3, \IntReg1
	str \IntReg3, [\IntReg2]

.endm


.macro DISABLETXINTERRUPT IntReg1:req, IntReg2:req, IntReg3:req, CAN1O2:req

	ldr \IntReg2, =NVIC_ICER0
	mov \IntReg1, \CAN1O2
	cmp \IntReg1, #0
	ITTTE ne
	movne \IntReg1, #0
	movtne \IntReg1, #0x8000
	addne \IntReg2, \IntReg2, #4
	movteq \IntReg1, #0x0008

	ldr \IntReg3, =0
	orr \IntReg3, \IntReg3, \IntReg1
	str \IntReg3, [\IntReg2]

.endm

 .section .text.canTXInterrupt
	.type canTXInterrupt, %function
canTXInterrupt:

	push {r1-r5}
	ldr r1, [r0, CAN_TSR]
	mov r2, #0x0001
	//movt r2, #1
	mov r3, #0
	movt r3, #0x1C00
	and r3, r3, r1
	lsr r3, r3, #26
	mov r4, #0
	movt r4, #0
L_CAN1PUMPLOOP:
	and r5, r3, #1
	lsr r3, r3, #1
	cmp r5, #0
	beq L_MOVETONEXT
	orr r4, r4, r2
	lsl r2, r2, #8
L_MOVETONEXT:
	cmp r3, #0
	bne L_CAN1PUMPLOOP


	orr r1, r1, r4
	str r1, [r0, CAN_TSR]

	pop {r1-r5}
	bx lr

.size canTXInterrupt, .-canTXInterrupt


.section .text.can1TXInterrupt
	.type can1TXInterrupt, %function
can1TXInterrupt:


	push {r0-r1}

	ldr r0, =NVIC_ICPR0
	ldr r1, [r0]
	orr r1, r1, #(0x1 << 19)
	str r1, [r0]


	ldr r0, =CAN1_BASE

	push {r0, lr}
	bl canTXInterrupt
	pop {r0, lr}

	pop {r0-r1}
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
	ldr r1, =CAN1FSM
	mov r2, CAN_RF0R
	mov r3, CAN_RI0R


	push { lr }

	bl manageReceiveBufferINT  //pop all messages off fifo queue

	pop { lr }


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
	ldr r1, =CAN1FSM
	mov r2, CAN_RF1R
	mov r3, CAN_RI1R


	push { lr }

	bl manageReceiveBufferINT  //pop all messages off fifo queue

	pop { lr }



	pop {r0-r3}
	bx lr

.size can1RX1Interrupt, .-can1RX1Interrupt


.section .text.can2TXInterrupt
	.type can2TXInterrupt, %function
can2TXInterrupt:

	push {r0-r1}
	ldr r0, =NVIC_ICPR1
	ldr r1, [r0]
	bic r1, r1, #(0x1 << 31)


	ldr r0, =CAN2_BASE

	push {r0, lr}
	bl canTXInterrupt
	pop {r0, lr}

	pop {r0-r1}
	bx lr

.size can2TXInterrupt, .-can2TXInterrupt


.section .text.can2RX0Interrupt
	.type can2RX0Interrupt, %function
can2RX0Interrupt:

	push {r0-r3}

	ldr r0, =NVIC_ICPR2
	ldr r1, [r0]
	orr r1, r1, #(0x1 << 0)
	str r1, [r0]

	ldr r0, =CAN2_BASE
	ldr r1, =CAN2FSM
	mov r2, CAN_RF0R
	mov r3, CAN_RI0R


	push { lr }

	bl manageReceiveBufferINT  //pop all messages off fifo queue

	pop { lr }

	pop {r0-r3}
	bx lr

.size can2RX0Interrupt, .-can2RX0Interrupt

.section .text.can2RX1Interrupt
	.type can2RX1Interrupt, %function
can2RX1Interrupt:


	push {r0-r3}

	ldr r0, =NVIC_ICPR2
	ldr r1, [r0]
	orr r1, r1, #(0x1 << 1)
	str r1, [r0]

	ldr r0, =CAN2_BASE
	ldr r1, =CAN2FSM
	mov r2, CAN_RF1R
	mov r3, CAN_RI1R


	push { lr }

	bl manageReceiveBufferINT  //pop all messages off fifo queue

	pop { lr }

	pop {r0-r3}
	bx lr

.size can2RX1Interrupt, .-can2RX1Interrupt




/* ---------- INITIALIZATION OF FSM --------- */


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


.section .text.initializeCAN2NVIC
	.type initializeCAN2NVIC, %function
initializeCAN2NVIC:
//r0 Interrupt Counter
	push {r1-r2}

	ldr r1, =NVIC_ISER1
	ldr r2, [r1]
	cmp r0, #0
	beq endinitializecan2nvic
	orr r2, r2, #(1 << 31)

	lsr r0, r0, #1
	cmp r0, #0
	beq endinitializecan2nvic
	str r2, [r1]
	add r1, r1, #4
	ldr r2, [r1]
	orr r2, r2, #(1 << 0)
	lsr r0, r0, #1
	cmp r0, #0
	beq endinitializecan2nvic
	orr r2, r2, #(1 << 1)


endinitializecan2nvic:


	str r2, [r1]
	pop {r1-r2}
	bx lr

.size initializeCAN2NVIC, .-initializeCAN2NVIC

.section .text.canFSMInit
	.type canFSMInit, %function
canFSMInit:
// r0 Base Buffer address
// r1 number of messages to store
// r2 CAN FSM Location
	push {r2-r7}

	mov r3, CANBUSFSMDATASIZE
	mov r4, r0
	ldr r5, =0

clearfsminitloopcan:
	str r5, [r4]
	adds r4, r4, #4
	sub r3, r3, #4
	cmp r3, r5
	bne clearfsminitloopcan


	str r0, [r2]

	add r2, r0, CANFSMRECEIVE //receivebuffer
	add r3, r0, CANFSMBYTE //bytebuffer
	add r4, r0, CANFSMTRANSMIT //transmitbuffer
	add r0, r0, CANBUSFSMDATASIZE

	mov r5, CANBUSTRANSMITBLKSIZE
	mul r7, r1, r5 //size of buffer
	str r0, [r4]
	str r0, [r4, TRANSMITBUFFERCURR]
	str r0, [r4, TRANSMITBUFFERREAD]
	str r7, [r4, TRANSMITBUFFERSIZE] //init tranmsit
	add r7, r0, r7
	str r7, [r2]
	str r7, [r2, RECEIVEBUFFERCURR]
	str r7, [r2, RECEIVEBUFFERREAD]
	mov r6, CANBUSRECEIVEBLKSIZE
	mul r6, r1, r6 //size of receive buffer
	str r6, [r2, RECEIVEBUFFERSIZE]
	add r7, r7, r6
	str r7, [r3]
	str r7, [r3, BYTEBUFFERCURR]
	mov r6, #8
	mul r7, r1, r6 //size of byte buffer
	str r7, [r3, BYTEBUFFERSIZE]

	pop {r2-r7}
	bx lr

.size canFSMInit, .-canFSMInit


.section .text.can1FSMInit
	.type can1FSMInit, %function
can1FSMInit:
// r0 Base Buffer address
// r1 number of messages to store
	push {r2}
	ldr r2, =CAN1FSM
	push {lr}
	bl canFSMInit
	pop {lr}
	pop {r2}
	bx lr
.size can1FSMInit, .-can1FSMInit

.section .text.can2FSMInit
	.type can2FSMInit, %function
can2FSMInit:
// r0 Base Buffer address
// r1 number of messages to store

	push {r2}
	ldr r2, =CAN2FSM
	push {lr}
	bl canFSMInit
	pop {lr}
	pop {r2}
	bx lr
	bx lr
.size can2FSMInit, .-can2FSMInit



/* ------------ UTILITY -------------- */



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



/* --------- TRANSMISSION --------------- */


.section .text.createTransmitMessageBlk
	.type createTransmitMessageBlk, %function
createTransmitMessageBlk:
//r0 data addr
//r1 data size
//r2 filter
//r3 CANFSMDATA
	push {r4-r6}
	ldr r3, [r3]
	add r4, r3, CANFSMTRANSMIT
	ldr r5, [r4, TRANSMITBUFFERCOUNT]
	mov r6, CANBUSTRANSMITBLKSIZE
	mul r6, r6, r5
	ldr r5, [r4, TRANSMITBUFFERSIZE]
	cmp r5, r6
	bne nooverflowoftbuffercan
	mov r0, #-1
	b endofcreatetransmitcan

nooverflowoftbuffercan:
	ldr r5, [r4, TRANSMITBUFFERCURR]
	str r1, [r5, CANBUSTRANSMITSIZE]
	str r0, [r5, CANBUSTRANSMITADDR]
	str r0, [r5, CANBUSTRANSMITCURR]
	str r2, [r5, CANBUSTRANSMITFID]
	push {r0-r2, lr}
	mov r0, r4
	mov r1, CANBUSTRANSMITBLKSIZE
	mov r2, TRANSMITBUFFERCURR
	bl moveFSMCurrForward
	pop {r0-r2, lr}
	ldr r2, [r4, TRANSMITBUFFERCOUNT]
	add r2, r2, #1
	str r2, [r4, TRANSMITBUFFERCOUNT]

	mov r0, #0
endofcreatetransmitcan:
	pop {r4-r6}
	bx lr


.size createTransmitMessageBlk, .-createTransmitMessageBlk


// add a blk to the buffer
.section .text.createTransmitMessageBlkCAN1
	.type createTransmitMessageBlkCAN1, %function
createTransmitMessageBlkCAN1:
//r0 data addr
//r1 data size
//r2 filter

	push {r4-r6}
	DISABLETXINTERRUPT r4, r5, r6, #0
	ldr r3, =CAN1FSM
	push {r0-r3, lr}
	bl createTransmitMessageBlk
	mov r6, r0
	pop {r0-r3, lr}

	mov r0, r6
	ENABLETXINTERRUPT r4, r5, r6, #0
	pop {r4-r6}
	bx lr

.size createTransmitMessageBlkCAN1, .-createTransmitMessageBlkCAN1


.section .text.createTransmitMessageBlkCAN2
	.type createTransmitMessageBlkCAN2, %function
createTransmitMessageBlkCAN2:
//r0 data addr
//r1 data size
//r2 filter
	push {r4-r6}
	DISABLETXINTERRUPT r4, r5, r6, #1
	ldr r3, =CAN2FSM
	push {r0-r3, lr}
	bl createTransmitMessageBlk
	mov r6, r0
	pop {r0-r3, lr}
	mov r0, r6
	ENABLETXINTERRUPT r4, r5, r6, #1
	pop {r4-r6}
	bx lr

.size createTransmitMessageBlkCAN2, .-createTransmitMessageBlkCAN2


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
    cmp r6, #32
	beq bcstarttransmission
	str r10, [r5]

bcstarttransmission:
	str r7, [r2, CANBUSTRANSMITCURR]
	pop {r3-r12}
	bx lr



.size copyBlockToCanBUSTransmitter, .-copyBlockToCanBUSTransmitter


.section .text.copyBlockFSM
	.type copyBlockFSM, %function
copyBlockFSM:
//r0 cann addr
//r1 transmit blk to copy to
//r2 CANFSMDATALOCATION
	push {r3-r6}

	add r3, r2, CANFSMTRANSMIT
	ldr r4, [r3, TRANSMITBUFFERCURR]
	ldr r3, [r3, TRANSMITBUFFERREAD]
	mov r6, #-1
	cmp r3, r4
	beq readheadtransmove
	push {r0-r2, lr}
	mov r2, r3
	bl copyBlockToCanBUSTransmitter

	pop {r0-r2, lr}

	ldr r4, [r3, CANBUSTRANSMITADDR]
	ldr r5, [r3, CANBUSTRANSMITCURR]
	ldr r6, [r3, CANBUSTRANSMITSIZE]
	add r4, r6, r4
	mov r6, #0
	cmp r5, r4
	bne readheadtransmove

	add r0, r2, CANFSMTRANSMIT

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
	pop {r3-r6}
	bx lr
.size copyBlockFSM, .-copyBlockFSM



.section .text.manageTransmitBufferInt
	.type manageTransmitBufferInt, %function
manageTransmitBufferInt:

// r0 CAN ADDR
// r1 CANFSMDATA

	push {r2-r7}
	add r2, r0, CAN_TSR
	ldr r3, [r2]
	mov r4, #0
	movt r4, #0x1C00
	and r3, r4, r3
	lsr r3, r3, #26
	movs r4, #-1
	ldr r2, [r1]
	ldr r7, =0
	mov r6, #1
	b checktmanageloop
tmanageloop:
	adds r4, r4, #1
	and r5, r3, #1
	lsr r3, r3, #1
	cmp r5, #0
	bne callcopyblocktmanage
checktmanageloop:
	cmp r3, #0
	bne tmanageloop

	b endtmanage
callcopyblocktmanage:

	mov r1, r4

	push {r0-r2, lr}
	bl copyBlockFSM
	cmp r0, #0
	pop {r0-r2, lr}
	blt endtmanage
	mov r6, #1
	lsl r6, r6, r4
	orr r7, r7, r6
	b checktmanageloop
endtmanage:
	mov r0, r7
	pop {r2-r7}
	bx lr

.size manageTransmitBufferInt, .-manageTransmitBufferInt


.section .text.canPumpMessages
	.type canPumpMessages, %function
canPumpMessages:
//r0 CAN1DATA
//r1 CAN1FSMDATA
	push {r2-r6}
	ldr r2, [r1]
	add r2, r2, CANFSMTRANSMIT
	ldr r3, [r2, TRANSMITBUFFERREAD]
	ldr r2, [r2, TRANSMITBUFFERCURR]
	cmp r2, r3
	beq canPumpMessagesEnd

	push {r0-r1, lr}

	bl manageTransmitBufferInt

	mov r2, r0

	pop { r0-r1, lr }

	add r0, r0, CAN_TI0R
	mov r1, #16

	mov r4, r2
	mov r5, #1
L_TRANSMIT:
	and r6, r5, r4
	lsr r4, r4, #1
	cmp r6, #0
	beq L_LOOPCHECK
	ldr r3, [r0]
	orr r3, r3, r5
	str r3, [r0]
L_LOOPCHECK:
	add r0, r0, r1
	cmp r4, #0
	bne L_TRANSMIT
canPumpMessagesEnd:
	pop {r2-r6}
	bx lr

.size canPumpMessages, .-canPumpMessages


.section .text.can1PumpMessages
	.type can1PumpMessages, %function
can1PumpMessages:

	push {r0-r1, r4-r6}

	DISABLETXINTERRUPT r4, r5, r6, #0
	ldr r0, =CAN1_BASE
	ldr r1, =CAN1FSM

	push { lr }
	bl canPumpMessages
	pop { lr }

	ENABLETXINTERRUPT r4, r5, r6, #0
	pop {r0-r1, r4-r6}
	bx lr
.size can1PumpMessages, .-can1PumpMessages


.section .text.can2PumpMessages
	.type can2PumpMessages, %function
can2PumpMessages:

	push {r0-r1, r4-r6}

	DISABLETXINTERRUPT r4, r5, r6, #1
	ldr r0, =CAN2_BASE
	ldr r1, =CAN2FSM

	push { lr }
	bl canPumpMessages
	pop { lr }

	ENABLETXINTERRUPT r4, r5, r6, #1
	pop {r0-r1, r4-r6}
	bx lr
.size can2PumpMessages, .-can2PumpMessages


/* --------------RECEIVER LOGIC------------------------ */
.section .text.packReceivedMessageInBuffer
	.type packReceivedMessageInBuffer, %function
packReceivedMessageInBuffer:
//r0 canbase + canrior
//r1 fsmdata location
	push {r2-r11}

	add r4, r1, CANFSMRECEIVE //receive buffer ptr
	ldr r2, [r4, RECEIVEBUFFERCOUNT]
	mov r3, CANBUSRECEIVEBLKSIZE
	mul r3, r3, r2
	ldr r2, [r4, RECEIVEBUFFERSIZE]
	cmp r2, r3
	bne readmessagebuffer

	movs r0, #-1
	b packreceivefinal


readmessagebuffer:
	
	ldr r4, [r4, RECEIVEBUFFERCURR]
	add r5, r1, CANFSMBYTE
	ldr r5, [r5, BYTEBUFFERCURR]
	ldr r3, [r0]
	lsr r3, r3, #21
	str r3, [r4, CANBUSRECEIVEFID] //get filter id and store it
	ldr r3, [r0, #4]
	and r6, r3, #0xf //dlc
	lsr r3, r3, #16 //timestamp
	str r3, [r4, CANBUSRECEIVETS]
	str r6, [r4, CANBUSRECEIVESIZE]
	str r5, [r4, CANBUSRECEIVEADDR] //store in message buffer

	mov r3, r5 //set up temp ptr
	add r8, r0, #8
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

	add r0, r1, CANFSMRECEIVE
	add r4, r1, CANFSMBYTE

	ldr r1, [r0, RECEIVEBUFFERCOUNT]
	add r1, r1, #1
	str r1, [r0, RECEIVEBUFFERCOUNT]

	mov r1, CANBUSRECEIVEBLKSIZE
	mov r2, RECEIVEBUFFERCURR
	push {r0-r2, lr}
	bl moveFSMCurrForward
	pop {r0-r2, lr}

	mov r0, r4
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

.section .text.manageReceiveBuffer
	.type manageReceiveBuffer, %function
manageReceiveBuffer:

// r0 CAN_ADDR
// r1 CANFSMDATA
	push {r2-r7}
	
	add r2, r0, CAN_RF0R
	mov r3, #0
	
	ldr r4, [r2]
	and r4, r4, #3
	cmp r4, #0
	bne getmessagesfromrfifo
	
	add r3, r3, #1
	add r2, r2, #4
	
	ldr r4, [r2]
	and r4, r4, #3
	cmp r4, #0
	bne getmessagesfromrfifo

	movs r0, #-1
	b checkrmessagesend

getmessagesfromrfifo:
	add r0, r0, CAN_RI0R
	mov r6, #16
	
	mla r0, r3, r6, r0
	
	mov r6, #0x0020
	movt r6, #0
	ldr r1, [r1]
rmessagesloop:
	push {r0-r1, lr}
	bl packReceivedMessageInBuffer
	pop {r0-r1, lr}

	ldr r7, [r2]
	orr r7, r7, r6
	str r7, [r2]
checkrmessagesloop:
	
	sub r4, r4, #1
	cmp r4, #0
	bne rmessagesloop

	mov r0, #0
checkrmessagesend:
	pop {r2-r7}
 	bx lr
.size manageReceiveBuffer, .-manageReceiveBuffer


.section .text.manageReceiveBufferINT
	.type manageReceiveBufferINT, %function
manageReceiveBufferINT:

// r0 CAN_ADDR 
// r1 CANFSMDATA
// r2 CAN_RF#R
// r3 CAN_RI#R
	push {r4-r6}
	ldr r4, [r0, r2]
	and r4, r4, #3 //get count of number of fifo queued


	mov r5, #0x0020
	movt r5, #0
	ldr r1, [r1]
rmessagesloopint:
	push {r0-r1, lr}
	add r0, r0, r3
	bl packReceivedMessageInBuffer
	pop {r0-r1, lr}

	ldr r6, [r0, r2]
	orr r6, r5, r6
	str r6, [r0, r2]

	ldr r4, [r0, r2]
	and r4, r4, #3
	cmp r4, #0
	bne rmessagesloopint
	mov r0, #0
	pop {r4-r6}
 	bx lr
.size manageReceiveBufferINT, .-manageReceiveBufferINT


.section .text.retrieveMessageBlock
	.type retrieveMessageBlock, %function
retrieveMessageBlock:
	push {r2-r5}
	ldr r1, [r1]
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
	ldr r3, [r2, CANBUSRECEIVESIZE]
	str r3, [r0, CANBUSRECEIVERETBLKSIZE]
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
	add r2, r2, CANBUSRECEIVEBLKSIZE
	ldr r3, [r1, RECEIVEBUFFERADDR]
	ldr r4, [r1, RECEIVEBUFFERSIZE]
	add r4, r3, r4
	cmp r2, r4
	bne receivewraparound
	mov r2, r3
receivewraparound:
	str r2, [r1, RECEIVEBUFFERREAD]
	mov r0, #0
retrieveMessageBlockEnd:
	pop {r2-r5}
	bx lr

.size retrieveMessageBlock, .-retrieveMessageBlock


.section .text.retrieveMessageBlockCAN1
	.type retrieveMessageBlockCAN1, %function
retrieveMessageBlockCAN1:
//r0 address to move the structure to
	push {r1-r5}
	DISABLERXINTERRUPT r1, r2, r3, #0, #0
	DISABLERXINTERRUPT r1, r2, r3, #1, #0
	ldr r1, =CAN1FSM
	push {r1, lr}
	bl retrieveMessageBlock
	pop {r1, lr}
	ENABLERXINTERRUPT r1, r2, r3, #0, #0
	ENABLERXINTERRUPT r1, r2, r3, #1, #0
	pop {r1-r5}
	bx lr
.size retrieveMessageBlockCAN1, .-retrieveMessageBlockCAN1


.section .text.retrieveMessageBlockCAN2
	.type retrieveMessageBlockCAN2, %function
retrieveMessageBlockCAN2:
//r0 address to move the structure to
	push {r1-r5}
	DISABLERXINTERRUPT r1, r2, r3, #0, #1
	DISABLERXINTERRUPT r1, r2, r3, #1, #1
	ldr r1, =CAN2FSM
	push {r1, lr}
	bl retrieveMessageBlock
	pop {r1, lr}
	ENABLERXINTERRUPT r1, r2, r3, #0, #1
	ENABLERXINTERRUPT r1, r2, r3, #1, #1
	pop {r1-r5}
	bx lr
.size retrieveMessageBlockCAN2, .-retrieveMessageBlockCAN2


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


	cmp r0, #0


/* might only need RCC_APB1ENR and GPIOA enabled to work can 2 */


	ldr r1, =RCC_AHB1ENR
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




 	beq CANBUSPEND

CANBUSBPINIT:

	ldr r1, =RCC_AHB1ENR
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

  // r0 CAN Data Address
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
//r0 address of following values

//r1 filter number
//r2 identifier mode
//r3 mode config
//r4 fifo assignment
//r5 fitler id
//r6 filter mask
	push {r1-r12}
	ldr r6, [r0, #20]
	ldr r5, [r0, #16]
	ldr r4, [r0, #12]
	ldr r3, [r0, #8]
	ldr r2, [r0, #4]
	ldr r1, [r0]

	ldr r0, =CAN1_BASE

	ldr r7, [r7, CAN_FA1R]
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

	pop {r1-r12}
	bx lr

.size  canFilterSet, .-canFilterSet



.section .text.canExitInit
  .type canExitInit, %function
//r0 CAN ADDR
// r1 CANSB mask
canExitInit:
	push {r2-r4}

	ldr r3, =#0x3F01
	ldr r4, =CAN1_BASE
	ldr r2, [r4, CAN_FMR]
	bic r2, r2, r3
	lsl r1, r1, #8
	orr r2, r2, r1
	str r2, [r4, CAN_FMR] //end the filter setup init

  	ldr r2, [r0, CAN_MCR]
	bic r2, r2, #1
	str r2, [r0, CAN_MCR] //request normal mode
ACKLoop2:
	ldr r2, [r0, CAN_MSR]
	and r3, r2, #1
	cmp r3, #0
	bne ACKLoop2 // entered normal mode
	pop {r2-r4}
	bx lr

 .size  canExitInit, .-canExitInit

/*
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

*/
