                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ISO C Compiler 
                                      3 ; Version 4.4.0 #14620 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module main
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _main
                                     12 	.globl _init_uart1
                                     13 	.globl _ADC2_Startup_Wait
                                     14 	.globl _ADC2_AlignConfig
                                     15 	.globl _ADC2_Select_Channel
                                     16 	.globl _ADC_get
                                     17 	.globl _printf
                                     18 	.globl _milis
                                     19 	.globl _init_milis
                                     20 	.globl _GPIO_Init
                                     21 	.globl _CLK_HSIPrescalerConfig
                                     22 	.globl _ADC2_SchmittTriggerConfig
                                     23 	.globl _ADC2_PrescalerConfig
                                     24 	.globl _ADC2_Cmd
                                     25 	.globl _init
                                     26 ;--------------------------------------------------------
                                     27 ; ram data
                                     28 ;--------------------------------------------------------
                                     29 	.area DATA
                                     30 ;--------------------------------------------------------
                                     31 ; ram data
                                     32 ;--------------------------------------------------------
                                     33 	.area INITIALIZED
                                     34 ;--------------------------------------------------------
                                     35 ; Stack segment in internal ram
                                     36 ;--------------------------------------------------------
                                     37 	.area SSEG
      009166                         38 __start__stack:
      009166                         39 	.ds	1
                                     40 
                                     41 ;--------------------------------------------------------
                                     42 ; absolute external ram data
                                     43 ;--------------------------------------------------------
                                     44 	.area DABS (ABS)
                                     45 
                                     46 ; default segment ordering for linker
                                     47 	.area HOME
                                     48 	.area GSINIT
                                     49 	.area GSFINAL
                                     50 	.area CONST
                                     51 	.area INITIALIZER
                                     52 	.area CODE
                                     53 
                                     54 ;--------------------------------------------------------
                                     55 ; interrupt vector
                                     56 ;--------------------------------------------------------
                                     57 	.area HOME
      008000                         58 __interrupt_vect:
      008000 82 00 80 6F             59 	int s_GSINIT ; reset
      008004 82 00 83 E8             60 	int _TRAP_IRQHandler ; trap
      008008 82 00 83 E9             61 	int _TLI_IRQHandler ; int0
      00800C 82 00 83 EA             62 	int _AWU_IRQHandler ; int1
      008010 82 00 83 EB             63 	int _CLK_IRQHandler ; int2
      008014 82 00 83 EC             64 	int _EXTI_PORTA_IRQHandler ; int3
      008018 82 00 83 ED             65 	int _EXTI_PORTB_IRQHandler ; int4
      00801C 82 00 83 EE             66 	int _EXTI_PORTC_IRQHandler ; int5
      008020 82 00 83 EF             67 	int _EXTI_PORTD_IRQHandler ; int6
      008024 82 00 83 F0             68 	int _EXTI_PORTE_IRQHandler ; int7
      008028 82 00 83 F1             69 	int _CAN_RX_IRQHandler ; int8
      00802C 82 00 83 F2             70 	int _CAN_TX_IRQHandler ; int9
      008030 82 00 83 F3             71 	int _SPI_IRQHandler ; int10
      008034 82 00 83 F4             72 	int _TIM1_UPD_OVF_TRG_BRK_IRQHandler ; int11
      008038 82 00 83 F5             73 	int _TIM1_CAP_COM_IRQHandler ; int12
      00803C 82 00 83 F6             74 	int _TIM2_UPD_OVF_BRK_IRQHandler ; int13
      008040 82 00 83 F7             75 	int _TIM2_CAP_COM_IRQHandler ; int14
      008044 82 00 83 F8             76 	int _TIM3_UPD_OVF_BRK_IRQHandler ; int15
      008048 82 00 83 F9             77 	int _TIM3_CAP_COM_IRQHandler ; int16
      00804C 82 00 83 FA             78 	int _UART1_TX_IRQHandler ; int17
      008050 82 00 83 FB             79 	int _UART1_RX_IRQHandler ; int18
      008054 82 00 83 FC             80 	int _I2C_IRQHandler ; int19
      008058 82 00 83 FD             81 	int _UART3_TX_IRQHandler ; int20
      00805C 82 00 83 FE             82 	int _UART3_RX_IRQHandler ; int21
      008060 82 00 83 FF             83 	int _ADC2_IRQHandler ; int22
      008064 82 00 84 00             84 	int _TIM4_UPD_OVF_IRQHandler ; int23
      008068 82 00 84 1A             85 	int _EEPROM_EEC_IRQHandler ; int24
                                     86 ;--------------------------------------------------------
                                     87 ; global & static initialisations
                                     88 ;--------------------------------------------------------
                                     89 	.area HOME
                                     90 	.area GSINIT
                                     91 	.area GSFINAL
                                     92 	.area GSINIT
      00806F CD 86 90         [ 4]   93 	call	___sdcc_external_startup
      008072 4D               [ 1]   94 	tnz	a
      008073 27 03            [ 1]   95 	jreq	__sdcc_init_data
      008075 CC 80 6C         [ 2]   96 	jp	__sdcc_program_startup
      008078                         97 __sdcc_init_data:
                                     98 ; stm8_genXINIT() start
      008078 AE 00 00         [ 2]   99 	ldw x, #l_DATA
      00807B 27 07            [ 1]  100 	jreq	00002$
      00807D                        101 00001$:
      00807D 72 4F 00 00      [ 1]  102 	clr (s_DATA - 1, x)
      008081 5A               [ 2]  103 	decw x
      008082 26 F9            [ 1]  104 	jrne	00001$
      008084                        105 00002$:
      008084 AE 00 04         [ 2]  106 	ldw	x, #l_INITIALIZER
      008087 27 09            [ 1]  107 	jreq	00004$
      008089                        108 00003$:
      008089 D6 80 C3         [ 1]  109 	ld	a, (s_INITIALIZER - 1, x)
      00808C D7 00 00         [ 1]  110 	ld	(s_INITIALIZED - 1, x), a
      00808F 5A               [ 2]  111 	decw	x
      008090 26 F7            [ 1]  112 	jrne	00003$
      008092                        113 00004$:
                                    114 ; stm8_genXINIT() end
                                    115 	.area GSFINAL
      008092 CC 80 6C         [ 2]  116 	jp	__sdcc_program_startup
                                    117 ;--------------------------------------------------------
                                    118 ; Home
                                    119 ;--------------------------------------------------------
                                    120 	.area HOME
                                    121 	.area HOME
      00806C                        122 __sdcc_program_startup:
      00806C CC 82 B4         [ 2]  123 	jp	_main
                                    124 ;	return from main will return to caller
                                    125 ;--------------------------------------------------------
                                    126 ; code
                                    127 ;--------------------------------------------------------
                                    128 	.area CODE
                                    129 ;	inc/delay.h: 18: static @inline void _delay_cycl( unsigned short __ticks )
                                    130 ; genLabel
                                    131 ;	-----------------------------------------
                                    132 ;	 function _delay_cycl
                                    133 ;	-----------------------------------------
                                    134 ;	Register assignment is optimal.
                                    135 ;	Stack space usage: 0 bytes.
      00822E                        136 __delay_cycl:
                                    137 ; genReceive
                                    138 ;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
                                    139 ;	genInline
      00822E 9D               [ 1]  140 	nop
      00822F 9D               [ 1]  141 	nop
                                    142 ;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
                                    143 ; genAssign
                                    144 ; genLabel
      008230                        145 00101$:
                                    146 ;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
                                    147 ; genMinus
      008230 5A               [ 2]  148 	decw	x
                                    149 ;	inc/delay.h: 32: } while ( __ticks );
                                    150 ; genIfx
      008231 5D               [ 2]  151 	tnzw	x
      008232 27 03            [ 1]  152 	jreq	00121$
      008234 CC 82 30         [ 2]  153 	jp	00101$
      008237                        154 00121$:
                                    155 ;	inc/delay.h: 33: __asm__("nop\n");
                                    156 ;	genInline
      008237 9D               [ 1]  157 	nop
                                    158 ; genLabel
      008238                        159 00104$:
                                    160 ;	inc/delay.h: 43: }
                                    161 ; genEndFunction
      008238 81               [ 4]  162 	ret
                                    163 ;	inc/delay.h: 45: static @inline void _delay_us( const unsigned short __us ){
                                    164 ; genLabel
                                    165 ;	-----------------------------------------
                                    166 ;	 function _delay_us
                                    167 ;	-----------------------------------------
                                    168 ;	Register assignment might be sub-optimal.
                                    169 ;	Stack space usage: 0 bytes.
      008239                        170 __delay_us:
                                    171 ; genReceive
                                    172 ;	inc/delay.h: 46: _delay_cycl( (unsigned short)( T_COUNT(__us) ));
                                    173 ; genCast
                                    174 ; genAssign
      008239 90 5F            [ 1]  175 	clrw	y
                                    176 ; genIPush
      00823B 89               [ 2]  177 	pushw	x
      00823C 90 89            [ 2]  178 	pushw	y
                                    179 ; genIPush
      00823E 4B 00            [ 1]  180 	push	#0x00
      008240 4B 24            [ 1]  181 	push	#0x24
      008242 4B F4            [ 1]  182 	push	#0xf4
      008244 4B 00            [ 1]  183 	push	#0x00
                                    184 ; genCall
      008246 CD 88 3B         [ 4]  185 	call	__mullong
      008249 5B 08            [ 2]  186 	addw	sp, #8
                                    187 ; genCast
                                    188 ; genAssign
                                    189 ; genIPush
      00824B 4B 40            [ 1]  190 	push	#0x40
      00824D 4B 42            [ 1]  191 	push	#0x42
      00824F 4B 0F            [ 1]  192 	push	#0x0f
      008251 4B 00            [ 1]  193 	push	#0x00
                                    194 ; genIPush
      008253 89               [ 2]  195 	pushw	x
      008254 90 89            [ 2]  196 	pushw	y
                                    197 ; genCall
      008256 CD 86 08         [ 4]  198 	call	__divulong
      008259 5B 08            [ 2]  199 	addw	sp, #8
                                    200 ; genRightShiftLiteral
      00825B 90 54            [ 2]  201 	srlw	y
      00825D 56               [ 2]  202 	rrcw	x
      00825E 90 54            [ 2]  203 	srlw	y
      008260 56               [ 2]  204 	rrcw	x
      008261 90 54            [ 2]  205 	srlw	y
      008263 56               [ 2]  206 	rrcw	x
                                    207 ; genCast
                                    208 ; genAssign
                                    209 ; genPlus
      008264 5C               [ 1]  210 	incw	x
                                    211 ;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
                                    212 ;	genInline
      008265 9D               [ 1]  213 	nop
      008266 9D               [ 1]  214 	nop
                                    215 ;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
                                    216 ; genAssign
                                    217 ; genLabel
      008267                        218 00101$:
                                    219 ;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
                                    220 ; genMinus
      008267 5A               [ 2]  221 	decw	x
                                    222 ;	inc/delay.h: 32: } while ( __ticks );
                                    223 ; genIfx
      008268 5D               [ 2]  224 	tnzw	x
      008269 27 03            [ 1]  225 	jreq	00122$
      00826B CC 82 67         [ 2]  226 	jp	00101$
      00826E                        227 00122$:
                                    228 ;	inc/delay.h: 33: __asm__("nop\n");
                                    229 ;	genInline
      00826E 9D               [ 1]  230 	nop
                                    231 ;	inc/delay.h: 46: _delay_cycl( (unsigned short)( T_COUNT(__us) ));
                                    232 ; genLabel
      00826F                        233 00105$:
                                    234 ;	inc/delay.h: 47: }
                                    235 ; genEndFunction
      00826F 81               [ 4]  236 	ret
                                    237 ;	./src/main.c: 11: void init(void) {
                                    238 ; genLabel
                                    239 ;	-----------------------------------------
                                    240 ;	 function init
                                    241 ;	-----------------------------------------
                                    242 ;	Register assignment is optimal.
                                    243 ;	Stack space usage: 0 bytes.
      008270                        244 _init:
                                    245 ;	./src/main.c: 12: CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // taktovani MCU na 16MHz
                                    246 ; genSend
      008270 4F               [ 1]  247 	clr	a
                                    248 ; genCall
      008271 CD 86 EC         [ 4]  249 	call	_CLK_HSIPrescalerConfig
                                    250 ;	./src/main.c: 14: GPIO_Init(LED_PORT, LED_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
                                    251 ; genIPush
      008274 4B C0            [ 1]  252 	push	#0xc0
                                    253 ; genSend
      008276 A6 20            [ 1]  254 	ld	a, #0x20
                                    255 ; genSend
      008278 AE 50 0A         [ 2]  256 	ldw	x, #0x500a
                                    257 ; genCall
      00827B CD 84 60         [ 4]  258 	call	_GPIO_Init
                                    259 ;	./src/main.c: 16: GPIO_Init(BTN_PORT, BTN_PIN, GPIO_MODE_IN_FL_NO_IT);
                                    260 ; genIPush
      00827E 4B 00            [ 1]  261 	push	#0x00
                                    262 ; genSend
      008280 A6 10            [ 1]  263 	ld	a, #0x10
                                    264 ; genSend
      008282 AE 50 14         [ 2]  265 	ldw	x, #0x5014
                                    266 ; genCall
      008285 CD 84 60         [ 4]  267 	call	_GPIO_Init
                                    268 ;	./src/main.c: 18: ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL14, DISABLE);
                                    269 ; genIPush
      008288 4B 00            [ 1]  270 	push	#0x00
                                    271 ; genSend
      00828A A6 0E            [ 1]  272 	ld	a, #0x0e
                                    273 ; genCall
      00828C CD 88 BC         [ 4]  274 	call	_ADC2_SchmittTriggerConfig
                                    275 ;	./src/main.c: 19: ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL15, DISABLE);
                                    276 ; genIPush
      00828F 4B 00            [ 1]  277 	push	#0x00
                                    278 ; genSend
      008291 A6 0F            [ 1]  279 	ld	a, #0x0f
                                    280 ; genCall
      008293 CD 88 BC         [ 4]  281 	call	_ADC2_SchmittTriggerConfig
                                    282 ;	./src/main.c: 22: ADC2_PrescalerConfig(ADC2_PRESSEL_FCPU_D4);
                                    283 ; genSend
      008296 A6 20            [ 1]  284 	ld	a, #0x20
                                    285 ; genCall
      008298 CD 86 92         [ 4]  286 	call	_ADC2_PrescalerConfig
                                    287 ;	./src/main.c: 24: ADC2_AlignConfig(ADC2_ALIGN_RIGHT);
                                    288 ; genSend
      00829B A6 08            [ 1]  289 	ld	a, #0x08
                                    290 ; genCall
      00829D CD 81 3D         [ 4]  291 	call	_ADC2_AlignConfig
                                    292 ;	./src/main.c: 26: ADC2_Select_Channel(ADC2_CHANNEL_14);
                                    293 ; genSend
      0082A0 A6 0E            [ 1]  294 	ld	a, #0x0e
                                    295 ; genCall
      0082A2 CD 81 2A         [ 4]  296 	call	_ADC2_Select_Channel
                                    297 ;	./src/main.c: 28: ADC2_Cmd(ENABLE);
                                    298 ; genSend
      0082A5 A6 01            [ 1]  299 	ld	a, #0x01
                                    300 ; genCall
      0082A7 CD 85 9F         [ 4]  301 	call	_ADC2_Cmd
                                    302 ;	./src/main.c: 30: ADC2_Startup_Wait();
                                    303 ; genCall
      0082AA CD 81 59         [ 4]  304 	call	_ADC2_Startup_Wait
                                    305 ;	./src/main.c: 32: init_milis();
                                    306 ; genCall
      0082AD CD 83 C7         [ 4]  307 	call	_init_milis
                                    308 ;	./src/main.c: 33: init_uart1();
                                    309 ; genCall
      0082B0 CC 84 1B         [ 2]  310 	jp	_init_uart1
                                    311 ; genLabel
      0082B3                        312 00101$:
                                    313 ;	./src/main.c: 34: }
                                    314 ; genEndFunction
      0082B3 81               [ 4]  315 	ret
                                    316 ;	./src/main.c: 36: int main(void) {
                                    317 ; genLabel
                                    318 ;	-----------------------------------------
                                    319 ;	 function main
                                    320 ;	-----------------------------------------
                                    321 ;	Register assignment might be sub-optimal.
                                    322 ;	Stack space usage: 12 bytes.
      0082B4                        323 _main:
      0082B4 52 0C            [ 2]  324 	sub	sp, #12
                                    325 ;	./src/main.c: 37: uint32_t time = 0;
                                    326 ; genAssign
      0082B6 5F               [ 1]  327 	clrw	x
      0082B7 1F 07            [ 2]  328 	ldw	(0x07, sp), x
      0082B9 1F 05            [ 2]  329 	ldw	(0x05, sp), x
                                    330 ;	./src/main.c: 39: init();
                                    331 ; genCall
      0082BB CD 82 70         [ 4]  332 	call	_init
                                    333 ;	./src/main.c: 41: while (1) {
                                    334 ; genLabel
      0082BE                        335 00104$:
                                    336 ;	./src/main.c: 42: if (milis() - time > 1111) {
                                    337 ; genCall
      0082BE CD 83 A7         [ 4]  338 	call	_milis
      0082C1 1F 03            [ 2]  339 	ldw	(0x03, sp), x
      0082C3 17 01            [ 2]  340 	ldw	(0x01, sp), y
                                    341 ; genMinus
      0082C5 1E 03            [ 2]  342 	ldw	x, (0x03, sp)
      0082C7 72 F0 07         [ 2]  343 	subw	x, (0x07, sp)
      0082CA 1F 0B            [ 2]  344 	ldw	(0x0b, sp), x
      0082CC 7B 02            [ 1]  345 	ld	a, (0x02, sp)
      0082CE 12 06            [ 1]  346 	sbc	a, (0x06, sp)
      0082D0 6B 0A            [ 1]  347 	ld	(0x0a, sp), a
      0082D2 7B 01            [ 1]  348 	ld	a, (0x01, sp)
      0082D4 12 05            [ 1]  349 	sbc	a, (0x05, sp)
      0082D6 6B 09            [ 1]  350 	ld	(0x09, sp), a
                                    351 ; genCmp
                                    352 ; genCmpTnz
      0082D8 AE 04 57         [ 2]  353 	ldw	x, #0x0457
      0082DB 13 0B            [ 2]  354 	cpw	x, (0x0b, sp)
      0082DD 4F               [ 1]  355 	clr	a
      0082DE 12 0A            [ 1]  356 	sbc	a, (0x0a, sp)
      0082E0 4F               [ 1]  357 	clr	a
      0082E1 12 09            [ 1]  358 	sbc	a, (0x09, sp)
      0082E3 25 03            [ 1]  359 	jrc	00122$
      0082E5 CC 82 BE         [ 2]  360 	jp	00104$
      0082E8                        361 00122$:
                                    362 ; skipping generated iCode
                                    363 ;	./src/main.c: 43: time = milis();
                                    364 ; genCall
      0082E8 CD 83 A7         [ 4]  365 	call	_milis
      0082EB 1F 07            [ 2]  366 	ldw	(0x07, sp), x
      0082ED 17 05            [ 2]  367 	ldw	(0x05, sp), y
                                    368 ;	./src/main.c: 45: vref = ADC_get(CHANNEL_VREF) * (uint32_t)(5000 + 512)/ 1023;
                                    369 ; genSend
      0082EF A6 0F            [ 1]  370 	ld	a, #0x0f
                                    371 ; genCall
      0082F1 CD 81 0A         [ 4]  372 	call	_ADC_get
                                    373 ; genAssign
                                    374 ; genIPush
      0082F4 89               [ 2]  375 	pushw	x
                                    376 ; genSend
      0082F5 AE 15 88         [ 2]  377 	ldw	x, #0x1588
                                    378 ; genCall
      0082F8 CD 84 EA         [ 4]  379 	call	___muluint2ulong
      0082FB 5B 02            [ 2]  380 	addw	sp, #2
                                    381 ; genIPush
      0082FD 4B FF            [ 1]  382 	push	#0xff
      0082FF 4B 03            [ 1]  383 	push	#0x03
      008301 4B 00            [ 1]  384 	push	#0x00
      008303 4B 00            [ 1]  385 	push	#0x00
                                    386 ; genIPush
      008305 89               [ 2]  387 	pushw	x
      008306 90 89            [ 2]  388 	pushw	y
                                    389 ; genCall
      008308 CD 86 08         [ 4]  390 	call	__divulong
      00830B 5B 08            [ 2]  391 	addw	sp, #8
                                    392 ; genCast
                                    393 ; genAssign
      00830D 1F 09            [ 2]  394 	ldw	(0x09, sp), x
                                    395 ;	./src/main.c: 46: ADC_get(CHANNEL_VTEMP);
                                    396 ; genSend
      00830F A6 0E            [ 1]  397 	ld	a, #0x0e
                                    398 ; genCall
      008311 CD 81 0A         [ 4]  399 	call	_ADC_get
                                    400 ;	./src/main.c: 47: ADC_get(CHANNEL_VTEMP);
                                    401 ; genSend
      008314 A6 0E            [ 1]  402 	ld	a, #0x0e
                                    403 ; genCall
      008316 CD 81 0A         [ 4]  404 	call	_ADC_get
                                    405 ;	./src/main.c: 48: ADC_get(CHANNEL_VTEMP);
                                    406 ; genSend
      008319 A6 0E            [ 1]  407 	ld	a, #0x0e
                                    408 ; genCall
      00831B CD 81 0A         [ 4]  409 	call	_ADC_get
                                    410 ;	./src/main.c: 49: ADC_get(CHANNEL_VTEMP);
                                    411 ; genSend
      00831E A6 0E            [ 1]  412 	ld	a, #0x0e
                                    413 ; genCall
      008320 CD 81 0A         [ 4]  414 	call	_ADC_get
                                    415 ;	./src/main.c: 50: ADC_get(CHANNEL_VTEMP);
                                    416 ; genSend
      008323 A6 0E            [ 1]  417 	ld	a, #0x0e
                                    418 ; genCall
      008325 CD 81 0A         [ 4]  419 	call	_ADC_get
                                    420 ;	./src/main.c: 51: ADC_get(CHANNEL_VTEMP);
                                    421 ; genSend
      008328 A6 0E            [ 1]  422 	ld	a, #0x0e
                                    423 ; genCall
      00832A CD 81 0A         [ 4]  424 	call	_ADC_get
                                    425 ;	./src/main.c: 52: ADC_get(CHANNEL_VTEMP);
                                    426 ; genSend
      00832D A6 0E            [ 1]  427 	ld	a, #0x0e
                                    428 ; genCall
      00832F CD 81 0A         [ 4]  429 	call	_ADC_get
                                    430 ;	./src/main.c: 53: ADC_get(CHANNEL_VTEMP);
                                    431 ; genSend
      008332 A6 0E            [ 1]  432 	ld	a, #0x0e
                                    433 ; genCall
      008334 CD 81 0A         [ 4]  434 	call	_ADC_get
                                    435 ;	./src/main.c: 54: vtemp = (uint32_t)ADC_get(CHANNEL_VTEMP) * (5000L + 512) / 1023;
                                    436 ; genSend
      008337 A6 0E            [ 1]  437 	ld	a, #0x0e
                                    438 ; genCall
      008339 CD 81 0A         [ 4]  439 	call	_ADC_get
                                    440 ; genAssign
                                    441 ; genIPush
      00833C 89               [ 2]  442 	pushw	x
                                    443 ; genSend
      00833D AE 15 88         [ 2]  444 	ldw	x, #0x1588
                                    445 ; genCall
      008340 CD 84 EA         [ 4]  446 	call	___muluint2ulong
      008343 5B 02            [ 2]  447 	addw	sp, #2
                                    448 ; genIPush
      008345 4B FF            [ 1]  449 	push	#0xff
      008347 4B 03            [ 1]  450 	push	#0x03
      008349 4B 00            [ 1]  451 	push	#0x00
      00834B 4B 00            [ 1]  452 	push	#0x00
                                    453 ; genIPush
      00834D 89               [ 2]  454 	pushw	x
      00834E 90 89            [ 2]  455 	pushw	y
                                    456 ; genCall
      008350 CD 86 08         [ 4]  457 	call	__divulong
      008353 5B 08            [ 2]  458 	addw	sp, #8
                                    459 ; genCast
                                    460 ; genAssign
      008355 1F 0B            [ 2]  461 	ldw	(0x0b, sp), x
                                    462 ;	./src/main.c: 56: temp = (100L*vtemp -40000L)/195;
                                    463 ; genCast
                                    464 ; genAssign
      008357 16 0B            [ 2]  465 	ldw	y, (0x0b, sp)
      008359 5F               [ 1]  466 	clrw	x
                                    467 ; genIPush
      00835A 90 89            [ 2]  468 	pushw	y
      00835C 89               [ 2]  469 	pushw	x
                                    470 ; genIPush
      00835D 4B 64            [ 1]  471 	push	#0x64
      00835F 5F               [ 1]  472 	clrw	x
      008360 89               [ 2]  473 	pushw	x
      008361 4B 00            [ 1]  474 	push	#0x00
                                    475 ; genCall
      008363 CD 88 3B         [ 4]  476 	call	__mullong
      008366 5B 08            [ 2]  477 	addw	sp, #8
                                    478 ; genMinus
      008368 1D 9C 40         [ 2]  479 	subw	x, #0x9c40
      00836B 24 02            [ 1]  480 	jrnc	00123$
      00836D 90 5A            [ 2]  481 	decw	y
      00836F                        482 00123$:
                                    483 ; genIPush
      00836F 4B C3            [ 1]  484 	push	#0xc3
      008371 4B 00            [ 1]  485 	push	#0x00
      008373 4B 00            [ 1]  486 	push	#0x00
      008375 4B 00            [ 1]  487 	push	#0x00
                                    488 ; genIPush
      008377 89               [ 2]  489 	pushw	x
      008378 90 89            [ 2]  490 	pushw	y
                                    491 ; genCall
      00837A CD 85 E1         [ 4]  492 	call	__divslong
      00837D 5B 08            [ 2]  493 	addw	sp, #8
                                    494 ; genCast
                                    495 ; genAssign
                                    496 ;	./src/main.c: 57: printf("%u mV, %u mV,%u,%u ˚C\n", vref, vtemp,temp/10,temp%10);
                                    497 ; genCast
                                    498 ; genAssign
                                    499 ; genDivMod
      00837F 89               [ 2]  500 	pushw	x
      008380 90 AE 00 0A      [ 2]  501 	ldw	y, #0x000a
      008384 65               [ 2]  502 	divw	x, y
      008385 85               [ 2]  503 	popw	x
                                    504 ; genDivMod
      008386 90 89            [ 2]  505 	pushw	y
      008388 90 AE 00 0A      [ 2]  506 	ldw	y, #0x000a
      00838C 65               [ 2]  507 	divw	x, y
      00838D 90 85            [ 2]  508 	popw	y
                                    509 ; skipping iCode since result will be rematerialized
                                    510 ; skipping iCode since result will be rematerialized
                                    511 ; genIPush
      00838F 90 89            [ 2]  512 	pushw	y
                                    513 ; genIPush
      008391 89               [ 2]  514 	pushw	x
                                    515 ; genIPush
      008392 1E 0F            [ 2]  516 	ldw	x, (0x0f, sp)
      008394 89               [ 2]  517 	pushw	x
                                    518 ; genIPush
      008395 1E 0F            [ 2]  519 	ldw	x, (0x0f, sp)
      008397 89               [ 2]  520 	pushw	x
                                    521 ; genIPush
      008398 4B 95            [ 1]  522 	push	#<(___str_0+0)
      00839A 4B 80            [ 1]  523 	push	#((___str_0+0) >> 8)
                                    524 ; genCall
      00839C CD 86 DB         [ 4]  525 	call	_printf
      00839F 5B 0A            [ 2]  526 	addw	sp, #10
                                    527 ; genGoto
      0083A1 CC 82 BE         [ 2]  528 	jp	00104$
                                    529 ; genLabel
      0083A4                        530 00106$:
                                    531 ;	./src/main.c: 60: }
                                    532 ; genEndFunction
      0083A4 5B 0C            [ 2]  533 	addw	sp, #12
      0083A6 81               [ 4]  534 	ret
                                    535 	.area CODE
                                    536 	.area CONST
                                    537 	.area CONST
      008095                        538 ___str_0:
      008095 25 75 20 6D 56 2C 20   539 	.ascii "%u mV, %u mV,%u,%u "
             25 75 20 6D 56 2C 25
             75 2C 25 75 20
      0080A8 CB                     540 	.db 0xcb
      0080A9 9A                     541 	.db 0x9a
      0080AA 43                     542 	.ascii "C"
      0080AB 0A                     543 	.db 0x0a
      0080AC 00                     544 	.db 0x00
                                    545 	.area CODE
                                    546 	.area INITIALIZER
                                    547 	.area CABS (ABS)
