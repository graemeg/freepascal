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
{$goto on}
unit cpu;

  interface

    uses
      sysutils;

    { returns true, if the processor supports the cpuid instruction }
    function cpuid_support : boolean;

    { returns true, if floating point is done by an emulator }
    function floating_point_emulation : boolean;

    { returns the contents of the cr0 register }
    function cr0 : longint;

    function InterlockedCompareExchange128Support : boolean;
    function AESSupport : boolean;inline;
    function AVXSupport: boolean;inline;
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

{$ASMMODE INTEL}
    var
      _AVXSupport,
      _AVX2Support,
      _AVX512FSupport,
      _AVX512DQSupport,
      _AVX512IFMASupport,
      _AVX512PFSupport,
      _AVX512ERSupport,
      _AVX512CDSupport,
      _AVX512BWSupport,
      _AVX512VLSupport,
      _AESSupport,
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

{$ASMMODE ATT}

    function InterlockedCompareExchange128(var Target: Int128Rec; NewValue: Int128Rec; Comperand: Int128Rec): Int128Rec;
      begin
{$if FPC_FULLVERSION >= 30101}
{$ifndef FPC_PIC}
        if _RTMSupport then
          begin
            asm
{$ifdef USE_REAL_INSTRUCTIONS}
         .Lretry:
              xbegin .Lretry
{$else}
{   3d:	c7 f8 fa ff ff ff    	xbegin    }
         .byte 0xc7,0xf8, 0xfa, 0xff, 0xff, 0xff
{$endif}
            end;
            Result:=Target;
            if (Result.Lo=Comperand.Lo) and (Result.Hi=Comperand.Hi) then
              Target:=NewValue;
            asm
{$ifdef USE_REAL_INSTRUCTIONS}
              xend
{$else}
  { 8a:	0f 01 d5             	xend    }
         .byte 0x0f, 0x01, 0xd5
{$endif}
            end;
          end
        else
{$endif FPC_PIC}
{$endif FPC_FULLVERSION >= 30101}
          RunError(217);
      end;

{$ASMMODE INTEL}

    function cpuid_support : boolean;assembler;
      {
        Check if the ID-flag can be changed, if changed then CpuID is supported.
        Tested under go32v1 and Linux on c6x86 with CpuID enabled and disabled (PFV)
      }
      asm
         push    ebx
         pushfd
         pushfd
         pop     eax
         mov     ebx,eax
         xor     eax,200000h
         push    eax
         popfd
         pushfd
         pop     eax
         popfd
         and     eax,200000h
         and     ebx,200000h
         cmp     eax,ebx
         setnz   al
         pop     ebx
      end;


    function cr0 : longint;assembler;
      asm
{$ifdef USE_REAL_INSTRUCTIONS}
         mov eax,cr0
{$else}
         DB 0Fh,20h,0C0h
{$endif}
         { mov eax,cr0
           special registers are not allowed in the assembler
                parsers }
      end;


    function floating_point_emulation : boolean;
      begin
         {!!!! I don't know currently the position of the EM flag }
         { $4 after Ralf Brown's list }
         floating_point_emulation:=(cr0 and $4)<>0;
      end;


{$ASMMODE ATT}
    function XGETBV(i : dword) : int64;assembler;
      asm
        movl %eax,%ecx
{$ifdef USE_REAL_INSTRUCTIONS}
        xgetbv
{$else}
        // older FPCs don't know the xgetbv opcode
        .byte 0x0f,0x01,0xd0
{$endif}
      end;


    procedure SetupSupport;
      var
         _ecx,_ebx,maxcpuidvalue : longint;
      begin
        is_sse3_cpu:=false;
         if cpuid_support then
           begin
              asm
                 pushl %ebx
                 movl $0,%eax
                 cpuid
                 movl %eax,maxcpuidvalue
                 popl %ebx
              end;
              asm
                 pushl %ebx
                 movl $1,%eax
                 cpuid
                 movl %ecx,_ecx
                 popl %ebx
              end;
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

              if maxcpuidvalue>=7 then
                begin
                  asm
                    pushl %ebx
                    movl $7,%eax
                    movl $0,%ecx
                    cpuid
                    movl %ebx,_ebx
                    popl %ebx
                  end;
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
                  _RTMSupport:=((_ebx and $800)<>0);
                end;
           end;
      end;


    function InterlockedCompareExchange128Support : boolean;
      begin
        { 32 Bit CPUs have no 128 Bit interlocked exchange support,
          but it can simulated using RTM }
        result:=_RTMSupport;
      end;


    function AESSupport : boolean;
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
