@   file core_asm.s
@   core asm routines
@   author cearn
@   Modified by Legolas for fpc4gba use
@
@ === NOTES ===
@ * 20050924: Lower overhead for all; reduced i-count for u16 loops.
@ * These are 16/32bit memset and memcpy. The 32bit versions are in 
@   iwram for maximum effect and pretty much do what CpuFastSet does, 
@   except that it'll work for non multiples of 8 words too. Speed 
@   is as good as CpuFastSet, but with a little less overhead.
@ * The 16bit versions call the 32bit ones if possible and/or desirable. 
@   They are thumb/ROM functions but did them in asm anyway because 
@   GCC goes haywire with the use of registers resulting in a much
@   higher overhead (i.e., detrimental for low counts)
@ * Crossover with inline while(nn--) loops (not for(ii++), which are
@   much slower):
@		memcpy32: ~4
@		memset32: ~5
@		memcpy16: ~8
@		memset16: ~8

	.file "core_asm.s"

@ === procedure memcpy32(dest: pointer; const src: pointer; wcount: u32); ======
@     Fast-copy by words.
@	  param dest Destination address.
@	  param src Source address.
@	  param wcount Number of words.
@	  note: src and dst must be word aligned.
@	  note: r0 and r1 return as dst + wdn and src + wdn.

@ Reglist:
@   r0, r1: dst, src
@   r2: wcount, then wcount>>3
@   r3-r10: data buffer
@   r12: wcount&7

	.text @ ?!?!?
@	.section .iwram,"ax", %progbits
	.align	2
	.code	32
	.global	memcpy32
memcpy32:
	and		r12, r2, #7
	movs	r2, r2, lsr #3
	beq		.Lres_cpy32
	stmfd	sp!, {r4-r10}
	@ copy 32byte chunks with 8fold xxmia
.Lmain_cpy32:
		ldmia	r1!, {r3-r10}	
		stmia	r0!, {r3-r10}
		subs	r2, r2, #1
		bhi		.Lmain_cpy32
	ldmfd	sp!, {r4-r10}
	@ and the residual 0-7 words
.Lres_cpy32:
		subs	r12, r12, #1
		ldmcsia	r1!, {r3}
		stmcsia	r0!, {r3}
		bcs		.Lres_cpy32
	bx	lr

@ === procedure memset32(dest: pointer; wd: u32; wcount: u32); =================
@     Fast-fill by words.
@   param dest Destination address.
@   param src Fill word (not address).
@   param wcount Number of words to fill.
@   note: dst must be word aligned.
@   note: r0 returns as dst + wcount.

@ Reglist:
@   r0, r1: dst, src
@   r2: wcount, then wcount>>3
@   r3-r10: data buffer
@   r12: wcount&7

  .text @?!?!?
@	.section .iwram,"ax", %progbits
	.align	2
	.code	32
	.global	memset32
memset32:
	and		r12, r2, #7
	movs	r2, r2, lsr #3
	beq		.Lres_set32
	stmfd	sp!, {r4-r10}
	@ set 32byte chunks with 8fold xxmia
	mov		r3, r1
	mov		r4, r1
	mov		r5, r1
	mov		r6, r1
	mov		r7, r1
	mov		r8, r1
	mov		r9, r1
	mov		r10, r1
.Lmain_set32:
		stmia	r0!, {r3-r10}
		subs	r2, r2, #1
		bhi		.Lmain_set32
	ldmfd	sp!, {r4-r10}
	@ residual 0-7 words
.Lres_set32:
		subs	r12, r12, #1
		stmcsia	r0!, {r1}
		bcs		.Lres_set32
	bx	lr

@ === procedure memcpy16(dest: pointer; const src: pointer; hwcount: u32); =====
@     Copy for halfwords.
@   	Uses memcpy32() if hwcount>6 and src and dst are aligned equally.
@   param dest Destination address.
@   param src Source address.
@   param hwcount Number of halfwords to fill.
@   note: dst and src must be halfword aligned.
@   note: r0 and r1 return as dst + hwcount and src + hwcount.

@ Reglist:
@   r0, r1: dst, src
@   r2, r4: hwcount
@   r3: tmp; and data buffer

	.text
	.align	2
	.code	16
	.global memcpy16
	.thumb_func
memcpy16:
	push	{r4, lr}
	@ under 5 hwords -> std cpy
	cmp		r2, #5
	bls		.Ltail_cpy16
	@ unreconcilable alignment -> std cpy
	@ if (dst^src)&2 -> alignment impossible
	mov		r3, r0
	eor		r3, r1
	lsl		r3, r3, #31		@ (dst^src), bit 1 into carry
	bcs		.Ltail_cpy16	@ (dst^src)&2 : must copy by halfword
	@ src and dst have same alignment -> word align
	lsl		r3, r0, #31
	bcc		.Lmain_cpy16	@ ~src&2 : already word aligned
	@ aligning is necessary: copy 1 hword and align
		ldrh	r3, [r1]
		strh	r3, [r0]
		add		r0, #2
		add		r1, #2
		sub		r2, r2, #1
	@ right, and for the REAL work, we're gonna use memcpy32
.Lmain_cpy16:
	lsl		r4, r2, #31
	lsr		r2, r2, #1
	ldr		r3, .Lpool_cpy16
  bx    r3
  nop

	@ NOTE: r0,r1 are altered by memcpy32, but in exactly the right 
	@ way, so we can use them as is.
	lsr		r2, r4, #31
	beq		.Lend_cpy16
.Ltail_cpy16:
	sub		r2, #1
	bcc		.Lend_cpy16		@ r2 was 0, bug out
	lsl		r2, r2, #1
.Lres_cpy16:
		ldrh	r3, [r1, r2]
		strh	r3, [r0, r2]
		sub		r2, r2, #2
		bcs		.Lres_cpy16
.Lend_cpy16:
	pop		{r4}
	pop		{r3}
	bx	r3
	.align 2
.Lpool_cpy16:
	.word memcpy32


@ === procedure memset16(dest: pointer; hw: u16; hwcount: u32); ================
@     Fill for halfwords.
@   	Uses memset32() if hwcount>5
@   param dest Destination address.
@   param hw Source halfword (not address).
@   param hwcount Number of halfwords to fill.
@   note: dest must be halfword aligned.
@   note: r0 returns as dest + hwcount.

@ Reglist:
@   r0, r1: dst, hw
@   r2, r4: hwcount
@   r3: tmp; and data buffer

	.text
	.align	2
	.code 16
	.global memset16
	.thumb_func
memset16:
	push	{r4, lr}
	@ under 6 hwords -> std set
	cmp		r2, #5
	bls		.Ltail_set16
	@ dst not word aligned: copy 1 hword and align
	lsl		r3, r0, #31
	bcc		.Lmain_set16
		strh	r1, [r0]
		add		r0, #2
		sub		r2, r2, #1
	@ Again, memset32 does the real work
.Lmain_set16:
	lsl		r4, r1, #16
	orr		r1, r4
	lsl		r4, r2, #31
	lsr		r2, r2, #1
	ldr		r3, .Lpool_set16
  bx    r3
  nop

	@ NOTE: r0 is altered by memset32, but in exactly the right 
	@ way, so we can use is as is. r1 is now doubled though.
	lsr		r2, r4, #31
	beq		.Lend_set16
	lsr		r1, #16
.Ltail_set16:
	sub		r2, #1
	bcc		.Lend_set16		@ r2 was 0, bug out
	lsl		r2, r2, #1
.Lres_set16:
		strh	r1, [r0, r2]
		sub		r2, r2, #2
		bcs		.Lres_set16
.Lend_set16:
	pop		{r4}
	pop		{r3}
	bx	r3
	.align 2
.Lpool_set16:
	.word memset32
