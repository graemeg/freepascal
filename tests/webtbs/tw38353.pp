{ %OPT=-Cg -O2 }
{ %CPU=x86_64 }

{ -Cg and -O2 options together lead to 
  the generation of instruction:
  testq   $15,U_$P$VECTORCALL_HVA_TEST1_$$_HVA@GOTPCREL(%rip)
  for which the relocation was not correctly generated
  in the internal assembler }

program tw38353;

{$IFNDEF CPUX86_64}
  {$FATAL This test program can only be compiled on Windows or Linux 64-bit with an Intel processor }
{$ENDIF}

{$ASMMODE Intel}
{$PUSH}
{$CODEALIGN RECORDMIN=16}
{$PACKRECORDS C}
type
  TM128 = record
    case Byte of
      0: (M128_F32: array[0..3] of Single);
      1: (M128_F64: array[0..1] of Double);
  end;
{$POP}

var
  HVA: TM128;

begin
{$ifdef verbose}
  writeln('@HVA=',hexstr(ptruint(@HVA),2*sizeof(ptruint)));
{$endif verbose}
  if (PtrUInt(@HVA) and $F) <> 0 then
  begin
{$ifdef verbose}
    WriteLn('FAIL: HVA is not correctly aligned.');
{$endif verbose}
    Halt(1);
  end;
end.
