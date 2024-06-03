;--------------------------------------------------------
; File Created by SDCC : free open source ISO C Compiler 
; Version 4.4.0 #14620 (Linux)
;--------------------------------------------------------
	.module main
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _init_uart1
	.globl _ADC2_Startup_Wait
	.globl _ADC2_AlignConfig
	.globl _ADC2_Select_Channel
	.globl _ADC_get
	.globl _printf
	.globl _milis
	.globl _init_milis
	.globl _GPIO_Init
	.globl _CLK_HSIPrescalerConfig
	.globl _ADC2_SchmittTriggerConfig
	.globl _ADC2_PrescalerConfig
	.globl _ADC2_Cmd
	.globl _init
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
;--------------------------------------------------------
; Stack segment in internal ram
;--------------------------------------------------------
	.area SSEG
__start__stack:
	.ds	1

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
; interrupt vector
;--------------------------------------------------------
	.area HOME
__interrupt_vect:
	int s_GSINIT ; reset
	int _TRAP_IRQHandler ; trap
	int _TLI_IRQHandler ; int0
	int _AWU_IRQHandler ; int1
	int _CLK_IRQHandler ; int2
	int _EXTI_PORTA_IRQHandler ; int3
	int _EXTI_PORTB_IRQHandler ; int4
	int _EXTI_PORTC_IRQHandler ; int5
	int _EXTI_PORTD_IRQHandler ; int6
	int _EXTI_PORTE_IRQHandler ; int7
	int _CAN_RX_IRQHandler ; int8
	int _CAN_TX_IRQHandler ; int9
	int _SPI_IRQHandler ; int10
	int _TIM1_UPD_OVF_TRG_BRK_IRQHandler ; int11
	int _TIM1_CAP_COM_IRQHandler ; int12
	int _TIM2_UPD_OVF_BRK_IRQHandler ; int13
	int _TIM2_CAP_COM_IRQHandler ; int14
	int _TIM3_UPD_OVF_BRK_IRQHandler ; int15
	int _TIM3_CAP_COM_IRQHandler ; int16
	int _UART1_TX_IRQHandler ; int17
	int _UART1_RX_IRQHandler ; int18
	int _I2C_IRQHandler ; int19
	int _UART3_TX_IRQHandler ; int20
	int _UART3_RX_IRQHandler ; int21
	int _ADC2_IRQHandler ; int22
	int _TIM4_UPD_OVF_IRQHandler ; int23
	int _EEPROM_EEC_IRQHandler ; int24
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
	call	___sdcc_external_startup
	tnz	a
	jreq	__sdcc_init_data
	jp	__sdcc_program_startup
__sdcc_init_data:
; stm8_genXINIT() start
	ldw x, #l_DATA
	jreq	00002$
00001$:
	clr (s_DATA - 1, x)
	decw x
	jrne	00001$
00002$:
	ldw	x, #l_INITIALIZER
	jreq	00004$
00003$:
	ld	a, (s_INITIALIZER - 1, x)
	ld	(s_INITIALIZED - 1, x), a
	decw	x
	jrne	00003$
00004$:
; stm8_genXINIT() end
	.area GSFINAL
	jp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
__sdcc_program_startup:
	jp	_main
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	inc/delay.h: 18: static @inline void _delay_cycl( unsigned short __ticks )
; genLabel
;	-----------------------------------------
;	 function _delay_cycl
;	-----------------------------------------
;	Register assignment is optimal.
;	Stack space usage: 0 bytes.
__delay_cycl:
; genReceive
;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
;	genInline
	nop
	nop
;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
; genAssign
; genLabel
00101$:
;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
; genMinus
	decw	x
;	inc/delay.h: 32: } while ( __ticks );
; genIfx
	tnzw	x
	jreq	00121$
	jp	00101$
00121$:
;	inc/delay.h: 33: __asm__("nop\n");
;	genInline
	nop
; genLabel
00104$:
;	inc/delay.h: 43: }
; genEndFunction
	ret
;	inc/delay.h: 45: static @inline void _delay_us( const unsigned short __us ){
; genLabel
;	-----------------------------------------
;	 function _delay_us
;	-----------------------------------------
;	Register assignment might be sub-optimal.
;	Stack space usage: 0 bytes.
__delay_us:
; genReceive
;	inc/delay.h: 46: _delay_cycl( (unsigned short)( T_COUNT(__us) ));
; genCast
; genAssign
	clrw	y
; genIPush
	pushw	x
	pushw	y
; genIPush
	push	#0x00
	push	#0x24
	push	#0xf4
	push	#0x00
; genCall
	call	__mullong
	addw	sp, #8
; genCast
; genAssign
; genIPush
	push	#0x40
	push	#0x42
	push	#0x0f
	push	#0x00
; genIPush
	pushw	x
	pushw	y
; genCall
	call	__divulong
	addw	sp, #8
; genRightShiftLiteral
	srlw	y
	rrcw	x
	srlw	y
	rrcw	x
	srlw	y
	rrcw	x
; genCast
; genAssign
; genPlus
	incw	x
;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
;	genInline
	nop
	nop
;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
; genAssign
; genLabel
00101$:
;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
; genMinus
	decw	x
;	inc/delay.h: 32: } while ( __ticks );
; genIfx
	tnzw	x
	jreq	00122$
	jp	00101$
00122$:
;	inc/delay.h: 33: __asm__("nop\n");
;	genInline
	nop
;	inc/delay.h: 46: _delay_cycl( (unsigned short)( T_COUNT(__us) ));
; genLabel
00105$:
;	inc/delay.h: 47: }
; genEndFunction
	ret
;	./src/main.c: 11: void init(void) {
; genLabel
;	-----------------------------------------
;	 function init
;	-----------------------------------------
;	Register assignment is optimal.
;	Stack space usage: 0 bytes.
_init:
;	./src/main.c: 12: CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // taktovani MCU na 16MHz
; genSend
	clr	a
; genCall
	call	_CLK_HSIPrescalerConfig
;	./src/main.c: 14: GPIO_Init(LED_PORT, LED_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
; genIPush
	push	#0xc0
; genSend
	ld	a, #0x20
; genSend
	ldw	x, #0x500a
; genCall
	call	_GPIO_Init
;	./src/main.c: 16: GPIO_Init(BTN_PORT, BTN_PIN, GPIO_MODE_IN_FL_NO_IT);
; genIPush
	push	#0x00
; genSend
	ld	a, #0x10
; genSend
	ldw	x, #0x5014
; genCall
	call	_GPIO_Init
;	./src/main.c: 18: ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL14, DISABLE);
; genIPush
	push	#0x00
; genSend
	ld	a, #0x0e
; genCall
	call	_ADC2_SchmittTriggerConfig
;	./src/main.c: 19: ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL15, DISABLE);
; genIPush
	push	#0x00
; genSend
	ld	a, #0x0f
; genCall
	call	_ADC2_SchmittTriggerConfig
;	./src/main.c: 22: ADC2_PrescalerConfig(ADC2_PRESSEL_FCPU_D4);
; genSend
	ld	a, #0x20
; genCall
	call	_ADC2_PrescalerConfig
;	./src/main.c: 24: ADC2_AlignConfig(ADC2_ALIGN_RIGHT);
; genSend
	ld	a, #0x08
; genCall
	call	_ADC2_AlignConfig
;	./src/main.c: 26: ADC2_Select_Channel(ADC2_CHANNEL_14);
; genSend
	ld	a, #0x0e
; genCall
	call	_ADC2_Select_Channel
;	./src/main.c: 28: ADC2_Cmd(ENABLE);
; genSend
	ld	a, #0x01
; genCall
	call	_ADC2_Cmd
;	./src/main.c: 30: ADC2_Startup_Wait();
; genCall
	call	_ADC2_Startup_Wait
;	./src/main.c: 32: init_milis();
; genCall
	call	_init_milis
;	./src/main.c: 33: init_uart1();
; genCall
	jp	_init_uart1
; genLabel
00101$:
;	./src/main.c: 34: }
; genEndFunction
	ret
;	./src/main.c: 36: int main(void) {
; genLabel
;	-----------------------------------------
;	 function main
;	-----------------------------------------
;	Register assignment might be sub-optimal.
;	Stack space usage: 12 bytes.
_main:
	sub	sp, #12
;	./src/main.c: 37: uint32_t time = 0;
; genAssign
	clrw	x
	ldw	(0x07, sp), x
	ldw	(0x05, sp), x
;	./src/main.c: 39: init();
; genCall
	call	_init
;	./src/main.c: 41: while (1) {
; genLabel
00104$:
;	./src/main.c: 42: if (milis() - time > 1111) {
; genCall
	call	_milis
	ldw	(0x03, sp), x
	ldw	(0x01, sp), y
; genMinus
	ldw	x, (0x03, sp)
	subw	x, (0x07, sp)
	ldw	(0x0b, sp), x
	ld	a, (0x02, sp)
	sbc	a, (0x06, sp)
	ld	(0x0a, sp), a
	ld	a, (0x01, sp)
	sbc	a, (0x05, sp)
	ld	(0x09, sp), a
; genCmp
; genCmpTnz
	ldw	x, #0x0457
	cpw	x, (0x0b, sp)
	clr	a
	sbc	a, (0x0a, sp)
	clr	a
	sbc	a, (0x09, sp)
	jrc	00122$
	jp	00104$
00122$:
; skipping generated iCode
;	./src/main.c: 43: time = milis();
; genCall
	call	_milis
	ldw	(0x07, sp), x
	ldw	(0x05, sp), y
;	./src/main.c: 45: vref = ADC_get(CHANNEL_VREF) * (uint32_t)(5000 + 512)/ 1023;
; genSend
	ld	a, #0x0f
; genCall
	call	_ADC_get
; genAssign
; genIPush
	pushw	x
; genSend
	ldw	x, #0x1588
; genCall
	call	___muluint2ulong
	addw	sp, #2
; genIPush
	push	#0xff
	push	#0x03
	push	#0x00
	push	#0x00
; genIPush
	pushw	x
	pushw	y
; genCall
	call	__divulong
	addw	sp, #8
; genCast
; genAssign
	ldw	(0x09, sp), x
;	./src/main.c: 46: ADC_get(CHANNEL_VTEMP);
; genSend
	ld	a, #0x0e
; genCall
	call	_ADC_get
;	./src/main.c: 47: ADC_get(CHANNEL_VTEMP);
; genSend
	ld	a, #0x0e
; genCall
	call	_ADC_get
;	./src/main.c: 48: ADC_get(CHANNEL_VTEMP);
; genSend
	ld	a, #0x0e
; genCall
	call	_ADC_get
;	./src/main.c: 49: ADC_get(CHANNEL_VTEMP);
; genSend
	ld	a, #0x0e
; genCall
	call	_ADC_get
;	./src/main.c: 50: ADC_get(CHANNEL_VTEMP);
; genSend
	ld	a, #0x0e
; genCall
	call	_ADC_get
;	./src/main.c: 51: ADC_get(CHANNEL_VTEMP);
; genSend
	ld	a, #0x0e
; genCall
	call	_ADC_get
;	./src/main.c: 52: ADC_get(CHANNEL_VTEMP);
; genSend
	ld	a, #0x0e
; genCall
	call	_ADC_get
;	./src/main.c: 53: ADC_get(CHANNEL_VTEMP);
; genSend
	ld	a, #0x0e
; genCall
	call	_ADC_get
;	./src/main.c: 54: vtemp = (uint32_t)ADC_get(CHANNEL_VTEMP) * (5000L + 512) / 1023;
; genSend
	ld	a, #0x0e
; genCall
	call	_ADC_get
; genAssign
; genIPush
	pushw	x
; genSend
	ldw	x, #0x1588
; genCall
	call	___muluint2ulong
	addw	sp, #2
; genIPush
	push	#0xff
	push	#0x03
	push	#0x00
	push	#0x00
; genIPush
	pushw	x
	pushw	y
; genCall
	call	__divulong
	addw	sp, #8
; genCast
; genAssign
	ldw	(0x0b, sp), x
;	./src/main.c: 56: temp = (100L*vtemp -40000L)/195;
; genCast
; genAssign
	ldw	y, (0x0b, sp)
	clrw	x
; genIPush
	pushw	y
	pushw	x
; genIPush
	push	#0x64
	clrw	x
	pushw	x
	push	#0x00
; genCall
	call	__mullong
	addw	sp, #8
; genMinus
	subw	x, #0x9c40
	jrnc	00123$
	decw	y
00123$:
; genIPush
	push	#0xc3
	push	#0x00
	push	#0x00
	push	#0x00
; genIPush
	pushw	x
	pushw	y
; genCall
	call	__divslong
	addw	sp, #8
; genCast
; genAssign
;	./src/main.c: 57: printf("%u mV, %u mV,%u,%u ˚C\n", vref, vtemp,temp/10,temp%10);
; genCast
; genAssign
; genDivMod
	pushw	x
	ldw	y, #0x000a
	divw	x, y
	popw	x
; genDivMod
	pushw	y
	ldw	y, #0x000a
	divw	x, y
	popw	y
; skipping iCode since result will be rematerialized
; skipping iCode since result will be rematerialized
; genIPush
	pushw	y
; genIPush
	pushw	x
; genIPush
	ldw	x, (0x0f, sp)
	pushw	x
; genIPush
	ldw	x, (0x0f, sp)
	pushw	x
; genIPush
	push	#<(___str_0+0)
	push	#((___str_0+0) >> 8)
; genCall
	call	_printf
	addw	sp, #10
; genGoto
	jp	00104$
; genLabel
00106$:
;	./src/main.c: 60: }
; genEndFunction
	addw	sp, #12
	ret
	.area CODE
	.area CONST
	.area CONST
___str_0:
	.ascii "%u mV, %u mV,%u,%u "
	.db 0xcb
	.db 0x9a
	.ascii "C"
	.db 0x0a
	.db 0x00
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
