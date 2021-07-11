{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl

    This unit contains some routines to get informations about the
    processor

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
unit cpu;

  interface

  {$ifdef freebsd}                 // FreeBSD 7/8 have binutils version that don't support cmpxchg16b
			           // Unless overridebinutils is defined (for ports usage), use db instead of the instruction
     {$ifndef overridebinutils}
       {$define oldbinutils}
     {$endif}
  {$endif}

    uses
      sysutils;

    function InterlockedCompareExchange128Support : boolean;inline;
    function AESSupport : boolean;inline;
    function AVXSupport : boolean;inline;
    function AVX2Support: boolean;inline;
    function AVX512FSupport: boolean;inline;    
    function AVX512DQSupport: boolean;inline;    
    function AVX512IFMASupport: boolean;inline;    
    function AVX512PFSupport: boolean;inline;    
    function AVX512ERSupport: boolean;inline;    
    function AVX512CDSupport: boolean;inline;    
    function AVX512BWSupport: boolean;inline;    
    function AVX512VLSupport: boolean;inline;    
    function FMASupport: boolean;inline;
    function POPCNTSupport: boolean;inline;
    function SSE41Support: boolean;inline;
    function SSE42Support: boolean;inline;
    function MOVBESupport: boolean;inline;
    function F16CSupport: boolean;inline;
    function RDRANDSupport: boolean;inline;
    function RTMSupport: boolean;inline;
    function BMI1Support: boolean;inline;
    function BMI2Support: boolean;inline;

    var
      is_sse3_cpu : boolean = false;

    function InterlockedCompareExchange128(var Target: Int128Rec; NewValue: Int128Rec; Comperand: Int128Rec): Int128Rec;

  implementation

{$asmmode att}

    var
      _AESSupport,
      _AVXSupport,
      _InterlockedCompareExchange128Support,
      _AVX2Support,
      _AVX512FSupport,
      _AVX512DQSupport,
      _AVX512IFMASupport,
      _AVX512PFSupport,
      _AVX512ERSupport,
      _AVX512CDSupport,
      _AVX512BWSupport,
      _AVX512VLSupport,
      _FMASupport,
      _POPCNTSupport,
      _SSE41Support,
      _SSE42Support,
      _MOVBESupport,
      _F16CSupport,
      _RDRANDSupport,
      _RTMSupport,
      _BMI1Support,
      _BMI2Support: boolean;

    function InterlockedCompareExchange128(var Target: Int128Rec; NewValue: Int128Rec; Comperand: Int128Rec): Int128Rec; assembler;
     {
        win64:
          rcx ... pointer to result
          rdx ... target
          r8  ... NewValue
          r9  ... Comperand
      }
    {$ifdef win64}
      asm
        pushq %rbx

        { store result pointer for later use }
        pushq %rcx

        { load new value }
        movq (%r8),%rbx
        movq 8(%r8),%rcx

        { save target pointer for later use }
        movq %rdx,%r8

        { load comperand }
        movq (%r9),%rax
        movq 8(%r9),%rdx

        {$ifdef oldbinutils}
           .byte 0xF0,0x49,0x0F,0xC7,0x08
        {$else}
        lock cmpxchg16b (%r8)
        {$endif}
        { restore result pointer }
        popq %rcx

        { store result }
        movq %rax,(%rcx)
        movq %rdx,8(%rcx)

        popq %rbx
      end;
    {$else win64}
    {
      linux:
        rdi       ... target
        [rsi:rdx] ... NewValue
        [rcx:r8]  ... Comperand
        [rdx:rax] ... result
    }
      asm
        pushq %rbx

        movq %rsi,%rbx          // new value low
        movq %rcx,%rax          // comperand low
        movq %rdx,%rcx          // new value high
        movq %r8,%rdx           // comperand high
        {$ifdef oldbinutils}
        .byte 0xF0,0x48,0x0F,0xC7,0x0F
        {$else}
        lock cmpxchg16b (%rdi)
        {$endif}

        popq %rbx
      end;
    {$endif win64}


    function XGETBV(i : dword) : int64;assembler;
      asm
    {$ifndef win64}
        movq %rdi,%rcx
    {$endif win64}
        // older FPCs don't know the xgetbv opcode
        .byte 0x0f,0x01,0xd0
        andl $0xffffffff,%eax
        shlq $32,%rdx
        orq %rdx,%rax
      end;


    procedure SetupSupport;
      var
        _ecx,
        _ebx,maxcpuidvalue : longint;
      begin
        asm
           movl $0x0,%eax
           cpuid
           movl %eax,maxcpuidvalue
        end ['rax','rbx','rcx','rdx'];
        asm
           movl $0x00000001,%eax
           cpuid
           movl %ecx,_ecx
        end ['rax','rbx','rcx','rdx'];
        _InterlockedCompareExchange128Support:=(_ecx and $2000)<>0;
        _AESSupport:=(_ecx and $2000000)<>0;
        _POPCNTSupport:=(_ecx and $800000)<>0;
        _SSE41Support:=(_ecx and $80000)<>0;
        _SSE42Support:=(_ecx and $100000)<>0;
        _MOVBESupport:=(_ecx and $400000)<>0;
        _F16CSupport:=(_ecx and $20000000)<>0;
        _RDRANDSupport:=(_ecx and $40000000)<>0;

        _AVXSupport:=
          { XGETBV suspport? }
          ((_ecx and $08000000)<>0) and
          { xmm and ymm state enabled? }
          ((XGETBV(0) and %110)=%110) and
          { avx supported? }
          ((_ecx and $10000000)<>0);

        is_sse3_cpu:=(_ecx and $1)<>0;

        _FMASupport:=_AVXSupport and ((_ecx and $1000)<>0);

        { very early x86-64 CPUs might not support eax=7 }
        if maxcpuidvalue>=7 then
          begin
            asm
              movl $7,%eax
              movl $0,%ecx
              cpuid
              movl %ebx,_ebx
            end ['rax','rbx','rcx','rdx'];
            _AVX2Support:=_AVXSupport and ((_ebx and $20)<>0);
            _AVX512FSupport:=(_ebx and $10000)<>0;
            _AVX512DQSupport:=(_ebx and $20000)<>0;
            _AVX512IFMASupport:=(_ebx and $200000)<>0;
            _AVX512PFSupport:=(_ebx and $4000000)<>0;
            _AVX512ERSupport:=(_ebx and $8000000)<>0;
            _AVX512CDSupport:=(_ebx and $10000000)<>0;
            _AVX512BWSupport:=(_ebx and $40000000)<>0;
            _AVX512VLSupport:=(_ebx and $80000000)<>0;
            _BMI1Support:=(_ebx and $8)<>0;
            _BMI2Support:=(_ebx and $100)<>0;
            _RTMSupport:=(_ebx and $800)<>0;
          end;
      end;


    function InterlockedCompareExchange128Support : boolean;inline;
      begin
        result:=_InterlockedCompareExchange128Support;
      end;


    function AESSupport : boolean;inline;
      begin
        result:=_AESSupport;
      end;


    function AVXSupport: boolean;inline;
      begin
        result:=_AVXSupport;
      end;


    function AVX2Support: boolean;inline;
      begin
        result:=_AVX2Support;
      end;


    function AVX512FSupport: boolean;inline;
      begin
        result:=_AVX512FSupport;
      end;


    function AVX512DQSupport: boolean;inline;
      begin
        result:=_AVX512DQSupport;
      end;


    function AVX512IFMASupport: boolean;inline;    
      begin
        result:=_AVX512IFMASupport;
      end;


    function AVX512PFSupport: boolean;inline;    
      begin
        result:=_AVX512PFSupport;
      end;


    function AVX512ERSupport: boolean;inline;    
      begin
        result:=_AVX512ERSupport;
      end;


    function AVX512CDSupport: boolean;inline;    
      begin
        result:=_AVX512CDSupport;
      end;


    function AVX512BWSupport: boolean;inline;    
      begin
        result:=_AVX512BWSupport;
      end;


    function AVX512VLSupport: boolean;inline;    
      begin
        result:=_AVX512VLSupport;
      end;


    function FMASupport: boolean;inline;
      begin
        result:=_FMASupport;
      end;


    function POPCNTSupport: boolean;inline;
      begin
        result:=_POPCNTSupport;
      end;

    function SSE41Support: boolean;inline;
      begin
        result:=_SSE41Support;
      end;


    function SSE42Support: boolean;inline;
      begin
        result:=_SSE42Support;
      end;


    function MOVBESupport: boolean;inline;
      begin
        result:=_MOVBESupport;
      end;


    function F16CSupport: boolean;inline;
      begin
        result:=_F16CSupport;
      end;


    function RDRANDSupport: boolean;inline;
      begin
        result:=_RDRANDSupport;
      end;


    function RTMSupport: boolean;inline;
      begin
        result:=_RTMSupport;
      end;


    function BMI1Support: boolean;inline;
      begin
        result:=_BMI1Support;
      end;


    function BMI2Support: boolean;inline;
      begin
        result:=_BMI2Support;
      end;


begin
  SetupSupport;
end.
