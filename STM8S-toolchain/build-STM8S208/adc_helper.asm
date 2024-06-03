;--------------------------------------------------------
; File Created by SDCC : free open source ISO C Compiler 
; Version 4.4.0 #14620 (Linux)
;--------------------------------------------------------
	.module adc_helper
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _ADC2_GetConversionValue
	.globl _ADC_get
	.globl _ADC2_Select_Channel
	.globl _ADC2_AlignConfig
	.globl _ADC2_Startup_Wait
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
;	./src/adc_helper.c: 10: uint16_t ADC_get(ADC2_Channel_TypeDef ADC2_Channel)
; genLabel
;	-----------------------------------------
;	 function ADC_get
;	-----------------------------------------
;	Register assignment is optimal.
;	Stack space usage: 0 bytes.
_ADC_get:
; genReceive
;	./src/adc_helper.c: 12: ADC2_Select_Channel(ADC2_Channel);  // vybere kanál / nastavuje analogový multiplexer
; genSend
; genCall
	call	_ADC2_Select_Channel
;	./src/adc_helper.c: 13: ADC2->CR1 |= ADC2_CR1_ADON; // Start Conversion (ADON must be SET before => ADC must be enabled !)
; genPointerGet
	ld	a, 0x5401
; genOr
	or	a, #0x01
; genPointerSet
	ld	0x5401, a
;	./src/adc_helper.c: 14: while (!(ADC2->CSR & ADC2_CSR_EOC));        // čeká na dokončení převodu (End Of Conversion)
; genLabel
00101$:
; genPointerGet
	ld	a, 0x5400
; genAnd
	tnz	a
	jrmi	00120$
	jp	00101$
00120$:
; skipping generated iCode
;	./src/adc_helper.c: 15: ADC2->CSR &= ~ADC2_CSR_EOC; // maže vlajku 
; genPointerGet
	ld	a, 0x5400
; genAnd
	and	a, #0x7f
; genPointerSet
	ld	0x5400, a
;	./src/adc_helper.c: 16: return ADC2_GetConversionValue();   // vrací výsledek
; genCall
	jp	_ADC2_GetConversionValue
; genReturn
; genLabel
00104$:
;	./src/adc_helper.c: 17: }
; genEndFunction
	ret
;	./src/adc_helper.c: 22: void ADC2_Select_Channel(ADC2_Channel_TypeDef ADC2_Channel)
; genLabel
;	-----------------------------------------
;	 function ADC2_Select_Channel
;	-----------------------------------------
;	Register assignment is optimal.
;	Stack space usage: 1 bytes.
_ADC2_Select_Channel:
	push	a
; genReceive
	ld	xl, a
;	./src/adc_helper.c: 24: uint8_t tmp = (ADC2->CSR) & (~ADC2_CSR_CH);
; genPointerGet
	ld	a, 0x5400
; genAnd
	and	a, #0xf0
	ld	(0x01, sp), a
;	./src/adc_helper.c: 25: tmp |= ADC2_Channel | ADC2_CSR_EOC;
; genOr
	ld	a, xl
	or	a, #0x80
; genOr
	or	a, (0x01, sp)
;	./src/adc_helper.c: 26: ADC2->CSR = tmp;
; genPointerSet
	ld	0x5400, a
; genLabel
00101$:
;	./src/adc_helper.c: 27: }
; genEndFunction
	pop	a
	ret
;	./src/adc_helper.c: 32: void ADC2_AlignConfig(ADC2_Align_TypeDef ADC2_Align)
; genLabel
;	-----------------------------------------
;	 function ADC2_AlignConfig
;	-----------------------------------------
;	Register assignment is optimal.
;	Stack space usage: 1 bytes.
_ADC2_AlignConfig:
	push	a
; genReceive
	ld	(0x01, sp), a
;	./src/adc_helper.c: 35: ADC2->CR2 |= (uint8_t) (ADC2_Align);
; genPointerGet
	ld	a, 0x5402
;	./src/adc_helper.c: 34: if (ADC2_Align) {
; genIfx
	tnz	(0x01, sp)
	jrne	00113$
	jp	00102$
00113$:
;	./src/adc_helper.c: 35: ADC2->CR2 |= (uint8_t) (ADC2_Align);
; genOr
	or	a, (0x01, sp)
; genPointerSet
	ld	0x5402, a
; genGoto
	jp	00104$
; genLabel
00102$:
;	./src/adc_helper.c: 37: ADC2->CR2 &= (uint8_t) (~ADC2_CR2_ALIGN);
; genAnd
	and	a, #0xf7
; genPointerSet
	ld	0x5402, a
; genLabel
00104$:
;	./src/adc_helper.c: 39: }
; genEndFunction
	pop	a
	ret
;	./src/adc_helper.c: 43: void ADC2_Startup_Wait(void)
; genLabel
;	-----------------------------------------
;	 function ADC2_Startup_Wait
;	-----------------------------------------
;	Register assignment is optimal.
;	Stack space usage: 0 bytes.
_ADC2_Startup_Wait:
;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
;	genInline
	nop
	nop
;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
; genAssign
	ldw	x, #0x000f
; genLabel
00101$:
;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
; genMinus
	decw	x
;	inc/delay.h: 32: } while ( __ticks );
; genIfx
	tnzw	x
	jreq	00123$
	jp	00101$
00123$:
;	inc/delay.h: 33: __asm__("nop\n");
;	genInline
	nop
;	./src/adc_helper.c: 45: _delay_us(ADC_TSTAB);
; genLabel
00106$:
;	./src/adc_helper.c: 46: }
; genEndFunction
	ret
	.area CODE
	.area CONST
	.area INITIALIZER
	.area CABS (ABS)
