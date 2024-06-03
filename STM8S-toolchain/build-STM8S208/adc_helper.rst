                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ISO C Compiler 
                                      3 ; Version 4.4.0 #14620 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module adc_helper
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _ADC2_GetConversionValue
                                     12 	.globl _ADC_get
                                     13 	.globl _ADC2_Select_Channel
                                     14 	.globl _ADC2_AlignConfig
                                     15 	.globl _ADC2_Startup_Wait
                                     16 ;--------------------------------------------------------
                                     17 ; ram data
                                     18 ;--------------------------------------------------------
                                     19 	.area DATA
                                     20 ;--------------------------------------------------------
                                     21 ; ram data
                                     22 ;--------------------------------------------------------
                                     23 	.area INITIALIZED
                                     24 ;--------------------------------------------------------
                                     25 ; absolute external ram data
                                     26 ;--------------------------------------------------------
                                     27 	.area DABS (ABS)
                                     28 
                                     29 ; default segment ordering for linker
                                     30 	.area HOME
                                     31 	.area GSINIT
                                     32 	.area GSFINAL
                                     33 	.area CONST
                                     34 	.area INITIALIZER
                                     35 	.area CODE
                                     36 
                                     37 ;--------------------------------------------------------
                                     38 ; global & static initialisations
                                     39 ;--------------------------------------------------------
                                     40 	.area HOME
                                     41 	.area GSINIT
                                     42 	.area GSFINAL
                                     43 	.area GSINIT
                                     44 ;--------------------------------------------------------
                                     45 ; Home
                                     46 ;--------------------------------------------------------
                                     47 	.area HOME
                                     48 	.area HOME
                                     49 ;--------------------------------------------------------
                                     50 ; code
                                     51 ;--------------------------------------------------------
                                     52 	.area CODE
                                     53 ;	inc/delay.h: 18: static @inline void _delay_cycl( unsigned short __ticks )
                                     54 ; genLabel
                                     55 ;	-----------------------------------------
                                     56 ;	 function _delay_cycl
                                     57 ;	-----------------------------------------
                                     58 ;	Register assignment is optimal.
                                     59 ;	Stack space usage: 0 bytes.
      0080C8                         60 __delay_cycl:
                                     61 ; genReceive
                                     62 ;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
                                     63 ;	genInline
      0080C8 9D               [ 1]   64 	nop
      0080C9 9D               [ 1]   65 	nop
                                     66 ;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
                                     67 ; genAssign
                                     68 ; genLabel
      0080CA                         69 00101$:
                                     70 ;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
                                     71 ; genMinus
      0080CA 5A               [ 2]   72 	decw	x
                                     73 ;	inc/delay.h: 32: } while ( __ticks );
                                     74 ; genIfx
      0080CB 5D               [ 2]   75 	tnzw	x
      0080CC 27 03            [ 1]   76 	jreq	00121$
      0080CE CC 80 CA         [ 2]   77 	jp	00101$
      0080D1                         78 00121$:
                                     79 ;	inc/delay.h: 33: __asm__("nop\n");
                                     80 ;	genInline
      0080D1 9D               [ 1]   81 	nop
                                     82 ; genLabel
      0080D2                         83 00104$:
                                     84 ;	inc/delay.h: 43: }
                                     85 ; genEndFunction
      0080D2 81               [ 4]   86 	ret
                                     87 ;	inc/delay.h: 45: static @inline void _delay_us( const unsigned short __us ){
                                     88 ; genLabel
                                     89 ;	-----------------------------------------
                                     90 ;	 function _delay_us
                                     91 ;	-----------------------------------------
                                     92 ;	Register assignment might be sub-optimal.
                                     93 ;	Stack space usage: 0 bytes.
      0080D3                         94 __delay_us:
                                     95 ; genReceive
                                     96 ;	inc/delay.h: 46: _delay_cycl( (unsigned short)( T_COUNT(__us) ));
                                     97 ; genCast
                                     98 ; genAssign
      0080D3 90 5F            [ 1]   99 	clrw	y
                                    100 ; genIPush
      0080D5 89               [ 2]  101 	pushw	x
      0080D6 90 89            [ 2]  102 	pushw	y
                                    103 ; genIPush
      0080D8 4B 00            [ 1]  104 	push	#0x00
      0080DA 4B 24            [ 1]  105 	push	#0x24
      0080DC 4B F4            [ 1]  106 	push	#0xf4
      0080DE 4B 00            [ 1]  107 	push	#0x00
                                    108 ; genCall
      0080E0 CD 88 3B         [ 4]  109 	call	__mullong
      0080E3 5B 08            [ 2]  110 	addw	sp, #8
                                    111 ; genCast
                                    112 ; genAssign
                                    113 ; genIPush
      0080E5 4B 40            [ 1]  114 	push	#0x40
      0080E7 4B 42            [ 1]  115 	push	#0x42
      0080E9 4B 0F            [ 1]  116 	push	#0x0f
      0080EB 4B 00            [ 1]  117 	push	#0x00
                                    118 ; genIPush
      0080ED 89               [ 2]  119 	pushw	x
      0080EE 90 89            [ 2]  120 	pushw	y
                                    121 ; genCall
      0080F0 CD 86 08         [ 4]  122 	call	__divulong
      0080F3 5B 08            [ 2]  123 	addw	sp, #8
                                    124 ; genRightShiftLiteral
      0080F5 90 54            [ 2]  125 	srlw	y
      0080F7 56               [ 2]  126 	rrcw	x
      0080F8 90 54            [ 2]  127 	srlw	y
      0080FA 56               [ 2]  128 	rrcw	x
      0080FB 90 54            [ 2]  129 	srlw	y
      0080FD 56               [ 2]  130 	rrcw	x
                                    131 ; genCast
                                    132 ; genAssign
                                    133 ; genPlus
      0080FE 5C               [ 1]  134 	incw	x
                                    135 ;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
                                    136 ;	genInline
      0080FF 9D               [ 1]  137 	nop
      008100 9D               [ 1]  138 	nop
                                    139 ;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
                                    140 ; genAssign
                                    141 ; genLabel
      008101                        142 00101$:
                                    143 ;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
                                    144 ; genMinus
      008101 5A               [ 2]  145 	decw	x
                                    146 ;	inc/delay.h: 32: } while ( __ticks );
                                    147 ; genIfx
      008102 5D               [ 2]  148 	tnzw	x
      008103 27 03            [ 1]  149 	jreq	00122$
      008105 CC 81 01         [ 2]  150 	jp	00101$
      008108                        151 00122$:
                                    152 ;	inc/delay.h: 33: __asm__("nop\n");
                                    153 ;	genInline
      008108 9D               [ 1]  154 	nop
                                    155 ;	inc/delay.h: 46: _delay_cycl( (unsigned short)( T_COUNT(__us) ));
                                    156 ; genLabel
      008109                        157 00105$:
                                    158 ;	inc/delay.h: 47: }
                                    159 ; genEndFunction
      008109 81               [ 4]  160 	ret
                                    161 ;	./src/adc_helper.c: 10: uint16_t ADC_get(ADC2_Channel_TypeDef ADC2_Channel)
                                    162 ; genLabel
                                    163 ;	-----------------------------------------
                                    164 ;	 function ADC_get
                                    165 ;	-----------------------------------------
                                    166 ;	Register assignment is optimal.
                                    167 ;	Stack space usage: 0 bytes.
      00810A                        168 _ADC_get:
                                    169 ; genReceive
                                    170 ;	./src/adc_helper.c: 12: ADC2_Select_Channel(ADC2_Channel);  // vybere kanál / nastavuje analogový multiplexer
                                    171 ; genSend
                                    172 ; genCall
      00810A CD 81 2A         [ 4]  173 	call	_ADC2_Select_Channel
                                    174 ;	./src/adc_helper.c: 13: ADC2->CR1 |= ADC2_CR1_ADON; // Start Conversion (ADON must be SET before => ADC must be enabled !)
                                    175 ; genPointerGet
      00810D C6 54 01         [ 1]  176 	ld	a, 0x5401
                                    177 ; genOr
      008110 AA 01            [ 1]  178 	or	a, #0x01
                                    179 ; genPointerSet
      008112 C7 54 01         [ 1]  180 	ld	0x5401, a
                                    181 ;	./src/adc_helper.c: 14: while (!(ADC2->CSR & ADC2_CSR_EOC));        // čeká na dokončení převodu (End Of Conversion)
                                    182 ; genLabel
      008115                        183 00101$:
                                    184 ; genPointerGet
      008115 C6 54 00         [ 1]  185 	ld	a, 0x5400
                                    186 ; genAnd
      008118 4D               [ 1]  187 	tnz	a
      008119 2B 03            [ 1]  188 	jrmi	00120$
      00811B CC 81 15         [ 2]  189 	jp	00101$
      00811E                        190 00120$:
                                    191 ; skipping generated iCode
                                    192 ;	./src/adc_helper.c: 15: ADC2->CSR &= ~ADC2_CSR_EOC; // maže vlajku 
                                    193 ; genPointerGet
      00811E C6 54 00         [ 1]  194 	ld	a, 0x5400
                                    195 ; genAnd
      008121 A4 7F            [ 1]  196 	and	a, #0x7f
                                    197 ; genPointerSet
      008123 C7 54 00         [ 1]  198 	ld	0x5400, a
                                    199 ;	./src/adc_helper.c: 16: return ADC2_GetConversionValue();   // vrací výsledek
                                    200 ; genCall
      008126 CC 87 FA         [ 2]  201 	jp	_ADC2_GetConversionValue
                                    202 ; genReturn
                                    203 ; genLabel
      008129                        204 00104$:
                                    205 ;	./src/adc_helper.c: 17: }
                                    206 ; genEndFunction
      008129 81               [ 4]  207 	ret
                                    208 ;	./src/adc_helper.c: 22: void ADC2_Select_Channel(ADC2_Channel_TypeDef ADC2_Channel)
                                    209 ; genLabel
                                    210 ;	-----------------------------------------
                                    211 ;	 function ADC2_Select_Channel
                                    212 ;	-----------------------------------------
                                    213 ;	Register assignment is optimal.
                                    214 ;	Stack space usage: 1 bytes.
      00812A                        215 _ADC2_Select_Channel:
      00812A 88               [ 1]  216 	push	a
                                    217 ; genReceive
      00812B 97               [ 1]  218 	ld	xl, a
                                    219 ;	./src/adc_helper.c: 24: uint8_t tmp = (ADC2->CSR) & (~ADC2_CSR_CH);
                                    220 ; genPointerGet
      00812C C6 54 00         [ 1]  221 	ld	a, 0x5400
                                    222 ; genAnd
      00812F A4 F0            [ 1]  223 	and	a, #0xf0
      008131 6B 01            [ 1]  224 	ld	(0x01, sp), a
                                    225 ;	./src/adc_helper.c: 25: tmp |= ADC2_Channel | ADC2_CSR_EOC;
                                    226 ; genOr
      008133 9F               [ 1]  227 	ld	a, xl
      008134 AA 80            [ 1]  228 	or	a, #0x80
                                    229 ; genOr
      008136 1A 01            [ 1]  230 	or	a, (0x01, sp)
                                    231 ;	./src/adc_helper.c: 26: ADC2->CSR = tmp;
                                    232 ; genPointerSet
      008138 C7 54 00         [ 1]  233 	ld	0x5400, a
                                    234 ; genLabel
      00813B                        235 00101$:
                                    236 ;	./src/adc_helper.c: 27: }
                                    237 ; genEndFunction
      00813B 84               [ 1]  238 	pop	a
      00813C 81               [ 4]  239 	ret
                                    240 ;	./src/adc_helper.c: 32: void ADC2_AlignConfig(ADC2_Align_TypeDef ADC2_Align)
                                    241 ; genLabel
                                    242 ;	-----------------------------------------
                                    243 ;	 function ADC2_AlignConfig
                                    244 ;	-----------------------------------------
                                    245 ;	Register assignment is optimal.
                                    246 ;	Stack space usage: 1 bytes.
      00813D                        247 _ADC2_AlignConfig:
      00813D 88               [ 1]  248 	push	a
                                    249 ; genReceive
      00813E 6B 01            [ 1]  250 	ld	(0x01, sp), a
                                    251 ;	./src/adc_helper.c: 35: ADC2->CR2 |= (uint8_t) (ADC2_Align);
                                    252 ; genPointerGet
      008140 C6 54 02         [ 1]  253 	ld	a, 0x5402
                                    254 ;	./src/adc_helper.c: 34: if (ADC2_Align) {
                                    255 ; genIfx
      008143 0D 01            [ 1]  256 	tnz	(0x01, sp)
      008145 26 03            [ 1]  257 	jrne	00113$
      008147 CC 81 52         [ 2]  258 	jp	00102$
      00814A                        259 00113$:
                                    260 ;	./src/adc_helper.c: 35: ADC2->CR2 |= (uint8_t) (ADC2_Align);
                                    261 ; genOr
      00814A 1A 01            [ 1]  262 	or	a, (0x01, sp)
                                    263 ; genPointerSet
      00814C C7 54 02         [ 1]  264 	ld	0x5402, a
                                    265 ; genGoto
      00814F CC 81 57         [ 2]  266 	jp	00104$
                                    267 ; genLabel
      008152                        268 00102$:
                                    269 ;	./src/adc_helper.c: 37: ADC2->CR2 &= (uint8_t) (~ADC2_CR2_ALIGN);
                                    270 ; genAnd
      008152 A4 F7            [ 1]  271 	and	a, #0xf7
                                    272 ; genPointerSet
      008154 C7 54 02         [ 1]  273 	ld	0x5402, a
                                    274 ; genLabel
      008157                        275 00104$:
                                    276 ;	./src/adc_helper.c: 39: }
                                    277 ; genEndFunction
      008157 84               [ 1]  278 	pop	a
      008158 81               [ 4]  279 	ret
                                    280 ;	./src/adc_helper.c: 43: void ADC2_Startup_Wait(void)
                                    281 ; genLabel
                                    282 ;	-----------------------------------------
                                    283 ;	 function ADC2_Startup_Wait
                                    284 ;	-----------------------------------------
                                    285 ;	Register assignment is optimal.
                                    286 ;	Stack space usage: 0 bytes.
      008159                        287 _ADC2_Startup_Wait:
                                    288 ;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
                                    289 ;	genInline
      008159 9D               [ 1]  290 	nop
      00815A 9D               [ 1]  291 	nop
                                    292 ;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
                                    293 ; genAssign
      00815B AE 00 0F         [ 2]  294 	ldw	x, #0x000f
                                    295 ; genLabel
      00815E                        296 00101$:
                                    297 ;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
                                    298 ; genMinus
      00815E 5A               [ 2]  299 	decw	x
                                    300 ;	inc/delay.h: 32: } while ( __ticks );
                                    301 ; genIfx
      00815F 5D               [ 2]  302 	tnzw	x
      008160 27 03            [ 1]  303 	jreq	00123$
      008162 CC 81 5E         [ 2]  304 	jp	00101$
      008165                        305 00123$:
                                    306 ;	inc/delay.h: 33: __asm__("nop\n");
                                    307 ;	genInline
      008165 9D               [ 1]  308 	nop
                                    309 ;	./src/adc_helper.c: 45: _delay_us(ADC_TSTAB);
                                    310 ; genLabel
      008166                        311 00106$:
                                    312 ;	./src/adc_helper.c: 46: }
                                    313 ; genEndFunction
      008166 81               [ 4]  314 	ret
                                    315 	.area CODE
                                    316 	.area CONST
                                    317 	.area INITIALIZER
                                    318 	.area CABS (ABS)
