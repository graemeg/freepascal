{ %CPU=x86_64,i386 }
{$mode objfpc}{$H+}
{$asmmode INTEL}

function f: longint; assembler;
asm
  mov ecx, ebx {shift by initial common exponent e}
{$ifdef cpui386}  
  vaddpd              XMM0 {k1} {z}, XMM0, [EAX + EDI + $10] {1to2}
{$else cpui386}
  vaddpd              XMM0 {k1} {z}, XMM0, [RAX + RDI + $10] {1to2}
{$endif cpui386}
end;


begin
end.
