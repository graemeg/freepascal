    _start:
{$ifdef CPUAVR_16_REGS}
        clr r17
{$else CPUAVR_16_REGS}
        clr r1
{$endif CPUAVR_16_REGS}

        // load stack pointer
        ldi r30,lo8(_stack_top)
        out 0x3d,r30
        ldi r30,hi8(_stack_top)
        out 0x3e,r30

        // Initialize .data section
        ldi XL,lo8(_data)
        ldi XH,hi8(_data)
        ldi YH,hi8(_edata)
        ldi ZL,lo8(_etext)
{$ifdef CPUAVR_16_REGS}
        ldi ZH,hi8(_etext)+(0x40) // program memory mapped to $4000 in data space
{$else CPUAVR_16_REGS}
        ldi ZH,hi8(_etext)
{$endif CPUAVR_16_REGS}

        rjmp .LCopyDataLoopEntry
.LCopyDataLoop:
{$ifdef CPUAVR_16_REGS}
        ld r16, Z+
{$else CPUAVR_16_REGS}
        lpm r16, Z+
{$endif CPUAVR_16_REGS}
        st X+, r16

.LCopyDataLoopEntry:
        cpi XL, lo8(_edata)
        cpc XH, YH
        brne .LCopyDataLoop

        // Zero .bss section
        ldi XL,lo8(_bss_start)
        ldi XH,hi8(_bss_start)
        ldi YH,hi8(_bss_end)

{$ifdef RELBRANCHES}
        rjmp .LZeroBssLoopEntry
{$else RELBRANCHES}
        jmp .LZeroBssLoopEntry
{$endif RELBRANCHES}
.LZeroBssLoop:
{$ifdef CPUAVR_16_REGS}
        st X+, r17
{$else CPUAVR_16_REGS}
        st X+, r1
{$endif CPUAVR_16_REGS}

.LZeroBssLoopEntry:
        cpi XL, lo8(_bss_end)
        cpc XH, YH
        brne .LZeroBssLoop

{$ifdef RELBRANCHES}
        rjmp PASCALMAIN
{$else RELBRANCHES}
        jmp PASCALMAIN
{$endif RELBRANCHES}
        .text

