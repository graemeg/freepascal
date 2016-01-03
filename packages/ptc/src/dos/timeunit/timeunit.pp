{$MODE objfpc}
{$ASMMODE intel}
{$goto on}

Unit timeunit;

Interface

Type
  TGetClockTics = Function : QWord;

Var
  TimerResolution : Double;
  CPS : Double;
  GetClockTics : TGetClockTics;

Implementation

Var
  UseRDTSC : Boolean;
  Clk1Lo, Clk1Hi, Clk2Lo, Clk2Hi : DWord;
  Clk1, Clk2 : QWord;
  ClkDelta : QWord;
  CpuFlags : DWord;

Function GetClockTics_RDTSC : QWord; Assembler;

Asm
  rdtsc
End;

Function GetClockTics_LAME : QWord;

Begin
  GetClockTics_LAME := MemL[$46C];
End;

Procedure DetectCPUSpeed_RDTSC;

Begin
  {word absolute $46C}
  Asm
    mov di, fs:[046Ch]
@@1:
    cmp di, fs:[046Ch]
    je @@1
    rdtsc
    mov ebx, eax
    mov ecx, edx
    mov di, fs:[046Ch]
@@2:
    mov ax, fs:[046Ch]
    sub ax, di
    cmp ax, 32
    jb @@2
    rdtsc
    mov [Clk1Lo], ebx
    mov [Clk1Hi], ecx
    mov [Clk2Lo], eax
    mov [Clk2Hi], edx
  End ['EAX','EBX','ECX','EDX','EDI'];
{  Clk1 := Clk1Lo Or (QWord(Clk1Hi) Shl 32);
  Clk2 := Clk2Lo Or (QWord(Clk2Hi) Shl 32);}
  Clk1 := Clk1Hi;
  Clk1 := Clk1 Shl 32;
  Clk1 := Clk1 + Clk1Lo;
  Clk2 := Clk2Hi;
  Clk2 := Clk2 Shl 32;
  Clk2 := Clk2 + Clk2Lo;
  ClkDelta := Clk2 - Clk1;
  CPS := (ClkDelta * 18.2) / 32;
  TimerResolution := 1 / CPS;
End;

Procedure _CPU; Assembler;

Label
  nocpuid;

Asm
  mov CpuFlags, 0
  pushf
  pop eax
  mov ecx, eax
  xor eax, 40000h
  push eax
  popf
  pushf
  pop eax
  xor eax, ecx
  jz nocpuid
  push ecx
  popf
  mov eax, ecx
  xor eax, 200000h
  push eax
  popf
  pushf
  pop eax
  xor eax, ecx
  je nocpuid

  pusha
  mov eax, 1
  cpuid
  mov CpuFlags, edx
  popa

nocpuid:
End;

Procedure DetectCPU;

Begin
  _CPU;
  If (CpuFlags And $10) <> 0 Then
    UseRDTSC := True
  Else
    UseRDTSC := False;

  If UseRDTSC Then
  Begin
    DetectCPUSpeed_RDTSC;
    GetClockTics := @GetClockTics_RDTSC;
  End
  Else
  Begin
    TimerResolution := 1 / 18.2;
    GetClockTics := @GetClockTics_LAME;
  End;
End;

Initialization

Begin
  DetectCPU;
End;

End.
