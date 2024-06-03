                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ISO C Compiler 
                                      3 ; Version 4.4.0 #14620 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module delay
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _delay_us
                                     12 	.globl _delay_ms
                                     13 ;--------------------------------------------------------
                                     14 ; ram data
                                     15 ;--------------------------------------------------------
                                     16 	.area DATA
                                     17 ;--------------------------------------------------------
                                     18 ; ram data
                                     19 ;--------------------------------------------------------
                                     20 	.area INITIALIZED
                                     21 ;--------------------------------------------------------
                                     22 ; absolute external ram data
                                     23 ;--------------------------------------------------------
                                     24 	.area DABS (ABS)
                                     25 
                                     26 ; default segment ordering for linker
                                     27 	.area HOME
                                     28 	.area GSINIT
                                     29 	.area GSFINAL
                                     30 	.area CONST
                                     31 	.area INITIALIZER
                                     32 	.area CODE
                                     33 
                                     34 ;--------------------------------------------------------
                                     35 ; global & static initialisations
                                     36 ;--------------------------------------------------------
                                     37 	.area HOME
                                     38 	.area GSINIT
                                     39 	.area GSFINAL
                                     40 	.area GSINIT
                                     41 ;--------------------------------------------------------
                                     42 ; Home
                                     43 ;--------------------------------------------------------
                                     44 	.area HOME
                                     45 	.area HOME
                                     46 ;--------------------------------------------------------
                                     47 ; code
                                     48 ;--------------------------------------------------------
                                     49 	.area CODE
                                     50 ;	inc/delay.h: 18: static @inline void _delay_cycl( unsigned short __ticks )
                                     51 ; genLabel
                                     52 ;	-----------------------------------------
                                     53 ;	 function _delay_cycl
                                     54 ;	-----------------------------------------
                                     55 ;	Register assignment is optimal.
                                     56 ;	Stack space usage: 0 bytes.
      008167                         57 __delay_cycl:
                                     58 ; genReceive
                                     59 ;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
                                     60 ;	genInline
      008167 9D               [ 1]   61 	nop
      008168 9D               [ 1]   62 	nop
                                     63 ;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
                                     64 ; genAssign
                                     65 ; genLabel
      008169                         66 00101$:
                                     67 ;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
                                     68 ; genMinus
      008169 5A               [ 2]   69 	decw	x
                                     70 ;	inc/delay.h: 32: } while ( __ticks );
                                     71 ; genIfx
      00816A 5D               [ 2]   72 	tnzw	x
      00816B 27 03            [ 1]   73 	jreq	00121$
      00816D CC 81 69         [ 2]   74 	jp	00101$
      008170                         75 00121$:
                                     76 ;	inc/delay.h: 33: __asm__("nop\n");
                                     77 ;	genInline
      008170 9D               [ 1]   78 	nop
                                     79 ; genLabel
      008171                         80 00104$:
                                     81 ;	inc/delay.h: 43: }
                                     82 ; genEndFunction
      008171 81               [ 4]   83 	ret
                                     84 ;	inc/delay.h: 45: static @inline void _delay_us( const unsigned short __us ){
                                     85 ; genLabel
                                     86 ;	-----------------------------------------
                                     87 ;	 function _delay_us
                                     88 ;	-----------------------------------------
                                     89 ;	Register assignment might be sub-optimal.
                                     90 ;	Stack space usage: 0 bytes.
      008172                         91 __delay_us:
                                     92 ; genReceive
                                     93 ;	inc/delay.h: 46: _delay_cycl( (unsigned short)( T_COUNT(__us) ));
                                     94 ; genCast
                                     95 ; genAssign
      008172 90 5F            [ 1]   96 	clrw	y
                                     97 ; genIPush
      008174 89               [ 2]   98 	pushw	x
      008175 90 89            [ 2]   99 	pushw	y
                                    100 ; genIPush
      008177 4B 00            [ 1]  101 	push	#0x00
      008179 4B 24            [ 1]  102 	push	#0x24
      00817B 4B F4            [ 1]  103 	push	#0xf4
      00817D 4B 00            [ 1]  104 	push	#0x00
                                    105 ; genCall
      00817F CD 88 3B         [ 4]  106 	call	__mullong
      008182 5B 08            [ 2]  107 	addw	sp, #8
                                    108 ; genCast
                                    109 ; genAssign
                                    110 ; genIPush
      008184 4B 40            [ 1]  111 	push	#0x40
      008186 4B 42            [ 1]  112 	push	#0x42
      008188 4B 0F            [ 1]  113 	push	#0x0f
      00818A 4B 00            [ 1]  114 	push	#0x00
                                    115 ; genIPush
      00818C 89               [ 2]  116 	pushw	x
      00818D 90 89            [ 2]  117 	pushw	y
                                    118 ; genCall
      00818F CD 86 08         [ 4]  119 	call	__divulong
      008192 5B 08            [ 2]  120 	addw	sp, #8
                                    121 ; genRightShiftLiteral
      008194 90 54            [ 2]  122 	srlw	y
      008196 56               [ 2]  123 	rrcw	x
      008197 90 54            [ 2]  124 	srlw	y
      008199 56               [ 2]  125 	rrcw	x
      00819A 90 54            [ 2]  126 	srlw	y
      00819C 56               [ 2]  127 	rrcw	x
                                    128 ; genCast
                                    129 ; genAssign
                                    130 ; genPlus
      00819D 5C               [ 1]  131 	incw	x
                                    132 ;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
                                    133 ;	genInline
      00819E 9D               [ 1]  134 	nop
      00819F 9D               [ 1]  135 	nop
                                    136 ;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
                                    137 ; genAssign
                                    138 ; genLabel
      0081A0                        139 00101$:
                                    140 ;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
                                    141 ; genMinus
      0081A0 5A               [ 2]  142 	decw	x
                                    143 ;	inc/delay.h: 32: } while ( __ticks );
                                    144 ; genIfx
      0081A1 5D               [ 2]  145 	tnzw	x
      0081A2 27 03            [ 1]  146 	jreq	00122$
      0081A4 CC 81 A0         [ 2]  147 	jp	00101$
      0081A7                        148 00122$:
                                    149 ;	inc/delay.h: 33: __asm__("nop\n");
                                    150 ;	genInline
      0081A7 9D               [ 1]  151 	nop
                                    152 ;	inc/delay.h: 46: _delay_cycl( (unsigned short)( T_COUNT(__us) ));
                                    153 ; genLabel
      0081A8                        154 00105$:
                                    155 ;	inc/delay.h: 47: }
                                    156 ; genEndFunction
      0081A8 81               [ 4]  157 	ret
                                    158 ;	./src/delay.c: 4: void delay_us(uint8_t us)
                                    159 ; genLabel
                                    160 ;	-----------------------------------------
                                    161 ;	 function delay_us
                                    162 ;	-----------------------------------------
                                    163 ;	Register assignment might be sub-optimal.
                                    164 ;	Stack space usage: 4 bytes.
      0081A9                        165 _delay_us:
      0081A9 52 04            [ 2]  166 	sub	sp, #4
                                    167 ; genReceive
                                    168 ;	./src/delay.c: 6: _delay_us(us);
                                    169 ; genCast
                                    170 ; genAssign
                                    171 ;	inc/delay.h: 46: _delay_cycl( (unsigned short)( T_COUNT(__us) ));
                                    172 ; genCast
                                    173 ; genAssign
      0081AB 5F               [ 1]  174 	clrw	x
      0081AC 0F 01            [ 1]  175 	clr	(0x01, sp)
                                    176 ; genIPush
      0081AE 88               [ 1]  177 	push	a
      0081AF 89               [ 2]  178 	pushw	x
      0081B0 4F               [ 1]  179 	clr	a
      0081B1 88               [ 1]  180 	push	a
                                    181 ; genIPush
      0081B2 4B 00            [ 1]  182 	push	#0x00
      0081B4 4B 24            [ 1]  183 	push	#0x24
      0081B6 4B F4            [ 1]  184 	push	#0xf4
      0081B8 4B 00            [ 1]  185 	push	#0x00
                                    186 ; genCall
      0081BA CD 88 3B         [ 4]  187 	call	__mullong
      0081BD 5B 08            [ 2]  188 	addw	sp, #8
                                    189 ; genCast
                                    190 ; genAssign
                                    191 ; genIPush
      0081BF 4B 40            [ 1]  192 	push	#0x40
      0081C1 4B 42            [ 1]  193 	push	#0x42
      0081C3 4B 0F            [ 1]  194 	push	#0x0f
      0081C5 4B 00            [ 1]  195 	push	#0x00
                                    196 ; genIPush
      0081C7 89               [ 2]  197 	pushw	x
      0081C8 90 89            [ 2]  198 	pushw	y
                                    199 ; genCall
      0081CA CD 86 08         [ 4]  200 	call	__divulong
      0081CD 5B 08            [ 2]  201 	addw	sp, #8
                                    202 ; genRightShiftLiteral
      0081CF 90 54            [ 2]  203 	srlw	y
      0081D1 56               [ 2]  204 	rrcw	x
      0081D2 90 54            [ 2]  205 	srlw	y
      0081D4 56               [ 2]  206 	rrcw	x
      0081D5 90 54            [ 2]  207 	srlw	y
      0081D7 56               [ 2]  208 	rrcw	x
                                    209 ; genCast
                                    210 ; genAssign
                                    211 ; genPlus
      0081D8 5C               [ 1]  212 	incw	x
                                    213 ;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
                                    214 ;	genInline
      0081D9 9D               [ 1]  215 	nop
      0081DA 9D               [ 1]  216 	nop
                                    217 ;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
                                    218 ; genAssign
                                    219 ; genLabel
      0081DB                        220 00101$:
                                    221 ;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
                                    222 ; genMinus
      0081DB 5A               [ 2]  223 	decw	x
                                    224 ;	inc/delay.h: 32: } while ( __ticks );
                                    225 ; genIfx
      0081DC 5D               [ 2]  226 	tnzw	x
      0081DD 27 03            [ 1]  227 	jreq	00123$
      0081DF CC 81 DB         [ 2]  228 	jp	00101$
      0081E2                        229 00123$:
                                    230 ;	inc/delay.h: 33: __asm__("nop\n");
                                    231 ;	genInline
      0081E2 9D               [ 1]  232 	nop
                                    233 ;	./src/delay.c: 6: _delay_us(us);
                                    234 ; genLabel
      0081E3                        235 00106$:
                                    236 ;	./src/delay.c: 7: }
                                    237 ; genEndFunction
      0081E3 5B 04            [ 2]  238 	addw	sp, #4
      0081E5 81               [ 4]  239 	ret
                                    240 ;	./src/delay.c: 9: void delay_ms(uint16_t ms)
                                    241 ; genLabel
                                    242 ;	-----------------------------------------
                                    243 ;	 function delay_ms
                                    244 ;	-----------------------------------------
                                    245 ;	Register assignment might be sub-optimal.
                                    246 ;	Stack space usage: 2 bytes.
      0081E6                        247 _delay_ms:
      0081E6 89               [ 2]  248 	pushw	x
                                    249 ; genReceive
      0081E7 1F 01            [ 2]  250 	ldw	(0x01, sp), x
                                    251 ;	./src/delay.c: 11: for (int16_t i = 0; i < ms; i++) {
                                    252 ; genAssign
      0081E9 90 5F            [ 1]  253 	clrw	y
                                    254 ; genLabel
      0081EB                        255 00123$:
                                    256 ; genCast
                                    257 ; genAssign
      0081EB 93               [ 1]  258 	ldw	x, y
                                    259 ; genCmp
                                    260 ; genCmpTnz
      0081EC 13 01            [ 2]  261 	cpw	x, (0x01, sp)
      0081EE 25 03            [ 1]  262 	jrc	00202$
      0081F0 CC 82 2C         [ 2]  263 	jp	00125$
      0081F3                        264 00202$:
                                    265 ; skipping generated iCode
                                    266 ;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
                                    267 ;	genInline
      0081F3 9D               [ 1]  268 	nop
      0081F4 9D               [ 1]  269 	nop
                                    270 ;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
                                    271 ; genAssign
      0081F5 AE 01 F5         [ 2]  272 	ldw	x, #0x01f5
                                    273 ; genLabel
      0081F8                        274 00102$:
                                    275 ;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
                                    276 ; genMinus
      0081F8 5A               [ 2]  277 	decw	x
                                    278 ;	inc/delay.h: 32: } while ( __ticks );
                                    279 ; genIfx
      0081F9 5D               [ 2]  280 	tnzw	x
      0081FA 27 03            [ 1]  281 	jreq	00203$
      0081FC CC 81 F8         [ 2]  282 	jp	00102$
      0081FF                        283 00203$:
                                    284 ;	inc/delay.h: 33: __asm__("nop\n");
                                    285 ;	genInline
      0081FF 9D               [ 1]  286 	nop
                                    287 ;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
                                    288 ;	genInline
      008200 9D               [ 1]  289 	nop
      008201 9D               [ 1]  290 	nop
                                    291 ;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
                                    292 ; genAssign
      008202 AE 01 F5         [ 2]  293 	ldw	x, #0x01f5
                                    294 ; genLabel
      008205                        295 00107$:
                                    296 ;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
                                    297 ; genMinus
      008205 5A               [ 2]  298 	decw	x
                                    299 ;	inc/delay.h: 32: } while ( __ticks );
                                    300 ; genIfx
      008206 5D               [ 2]  301 	tnzw	x
      008207 27 03            [ 1]  302 	jreq	00204$
      008209 CC 82 05         [ 2]  303 	jp	00107$
      00820C                        304 00204$:
                                    305 ;	inc/delay.h: 33: __asm__("nop\n");
                                    306 ;	genInline
      00820C 9D               [ 1]  307 	nop
                                    308 ;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
                                    309 ;	genInline
      00820D 9D               [ 1]  310 	nop
      00820E 9D               [ 1]  311 	nop
                                    312 ;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
                                    313 ; genAssign
      00820F AE 01 F5         [ 2]  314 	ldw	x, #0x01f5
                                    315 ; genLabel
      008212                        316 00112$:
                                    317 ;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
                                    318 ; genMinus
      008212 5A               [ 2]  319 	decw	x
                                    320 ;	inc/delay.h: 32: } while ( __ticks );
                                    321 ; genIfx
      008213 5D               [ 2]  322 	tnzw	x
      008214 27 03            [ 1]  323 	jreq	00205$
      008216 CC 82 12         [ 2]  324 	jp	00112$
      008219                        325 00205$:
                                    326 ;	inc/delay.h: 33: __asm__("nop\n");
                                    327 ;	genInline
      008219 9D               [ 1]  328 	nop
                                    329 ;	inc/delay.h: 29: __asm__("nop\n nop\n"); 
                                    330 ;	genInline
      00821A 9D               [ 1]  331 	nop
      00821B 9D               [ 1]  332 	nop
                                    333 ;	inc/delay.h: 30: do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
                                    334 ; genAssign
      00821C AE 01 E7         [ 2]  335 	ldw	x, #0x01e7
                                    336 ; genLabel
      00821F                        337 00117$:
                                    338 ;	inc/delay.h: 31: __ticks--;//      2c;                 1c;     2c    ; 1/2c   
                                    339 ; genMinus
      00821F 5A               [ 2]  340 	decw	x
                                    341 ;	inc/delay.h: 32: } while ( __ticks );
                                    342 ; genIfx
      008220 5D               [ 2]  343 	tnzw	x
      008221 27 03            [ 1]  344 	jreq	00206$
      008223 CC 82 1F         [ 2]  345 	jp	00117$
      008226                        346 00206$:
                                    347 ;	inc/delay.h: 33: __asm__("nop\n");
                                    348 ;	genInline
      008226 9D               [ 1]  349 	nop
                                    350 ;	./src/delay.c: 11: for (int16_t i = 0; i < ms; i++) {
                                    351 ; genPlus
      008227 90 5C            [ 1]  352 	incw	y
                                    353 ; genGoto
      008229 CC 81 EB         [ 2]  354 	jp	00123$
                                    355 ; genLabel
      00822C                        356 00125$:
                                    357 ;	./src/delay.c: 17: }
                                    358 ; genEndFunction
      00822C 85               [ 2]  359 	popw	x
      00822D 81               [ 4]  360 	ret
                                    361 	.area CODE
                                    362 	.area CONST
                                    363 	.area INITIALIZER
                                    364 	.area CABS (ABS)
