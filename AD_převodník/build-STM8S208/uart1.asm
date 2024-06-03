;--------------------------------------------------------
; File Created by SDCC : free open source ISO C Compiler 
; Version 4.4.0 #14620 (Linux)
;--------------------------------------------------------
	.module uart1
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _getchar
	.globl _putchar
	.globl _UART1_GetFlagStatus
	.globl _UART1_SendData8
	.globl _UART1_ReceiveData8
	.globl _UART1_Cmd
	.globl _UART1_Init
	.globl _UART1_DeInit
	.globl _init_uart1
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)

; default segment ordering for linker
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area CONST
	.area INITIALIZER
	.area CODE

;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	./src/uart1.c: 13: void init_uart1(void)
; genLabel
;	-----------------------------------------
;	 function init_uart1
;	-----------------------------------------
;	Register assignment is optimal.
;	Stack space usage: 0 bytes.
_init_uart1:
;	./src/uart1.c: 15: UART1_DeInit();
; genCall
	call	_UART1_DeInit
;	./src/uart1.c: 17: UART1_Init((uint32_t) 115200,       // - BaudRate = 115200 baud  
; genIPush
	push	#0x0c
; genIPush
	push	#0x80
; genIPush
	push	#0x00
; genIPush
	push	#0x00
; genIPush
	push	#0x00
; genIPush
	push	#0x00
	push	#0xc2
	push	#0x01
	push	#0x00
; genCall
	call	_UART1_Init
;	./src/uart1.c: 24: UART1_Cmd(ENABLE);
; genSend
	ld	a, #0x01
; genCall
	jp	_UART1_Cmd
; genLabel
00101$:
;	./src/uart1.c: 25: }
; genEndFunction
	ret
;	./src/uart1.c: 28: PUTCHAR_PROTOTYPE {
; genLabel
;	-----------------------------------------
;	 function putchar
;	-----------------------------------------
;	Register assignment is optimal.
;	Stack space usage: 0 bytes.
_putchar:
; genReceive
;	./src/uart1.c: 30: UART1_SendData8(c);
; genCast
; genAssign
	ld	a, xl
; genSend
	pushw	x
; genCall
	call	_UART1_SendData8
	popw	x
;	./src/uart1.c: 32: while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET){
; genLabel
00101$:
; genSend
	pushw	x
	ldw	x, #0x0080
; genCall
	call	_UART1_GetFlagStatus
	popw	x
; genIfx
	tnz	a
	jrne	00120$
	jp	00101$
00120$:
;	./src/uart1.c: 35: return (c);
; genReturn
; genLabel
00104$:
;	./src/uart1.c: 36: }
; genEndFunction
	ret
;	./src/uart1.c: 43: GETCHAR_PROTOTYPE {
; genLabel
;	-----------------------------------------
;	 function getchar
;	-----------------------------------------
;	Register assignment is optimal.
;	Stack space usage: 0 bytes.
_getchar:
;	./src/uart1.c: 50: while (UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET) {
; genLabel
00101$:
; genSend
	ldw	x, #0x0020
; genCall
	call	_UART1_GetFlagStatus
; genIfx
	tnz	a
	jrne	00120$
	jp	00101$
00120$:
;	./src/uart1.c: 53: c = UART1_ReceiveData8();
; genCall
	call	_UART1_ReceiveData8
; genCast
; genAssign
	clrw	x
;	./src/uart1.c: 54: return (c);
; genReturn
	ld	xl, a
; genLabel
00104$:
;	./src/uart1.c: 55: }
; genEndFunction
	ret
	.area CODE
	.area CONST
	.area INITIALIZER
	.area CABS (ABS)
