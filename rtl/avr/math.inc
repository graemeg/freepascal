{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 by the Free Pascal development team.

    Implementation of mathematical Routines (only for real)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$i divide.inc}


{$ifndef FPC_SYSTEM_HAS_DIV_BYTE}
{$define FPC_SYSTEM_HAS_DIV_BYTE}

// z (dividend) = q(quotient) x n(divisor) + p(remainder)

// z in Ra, n in Rb, 0 in Rp
function fpc_div_byte(n, z: byte): byte; assembler; nostackframe;
{$ifdef FPC_IS_SYSTEM}[public,alias: 'FPC_DIV_BYTE'];{$endif}
asm
{$ifdef CPUAVR_16_REGS}
  cp R24, R17
{$else CPUAVR_16_REGS}
  cp R24, R1
{$endif CPUAVR_16_REGS}  
  brne .LNonZero
{$ifdef CPUAVR_HAS_JMP_CALL}
  call fpc_divbyzero
.LNonZero:
  call fpc_divmod_byte
{$else  CPUAVR_HAS_JMP_CALL}
  rcall fpc_divbyzero
.LNonZero:
  rcall fpc_divmod_byte
{$endif CPUAVR_HAS_JMP_CALL}

  mov R24, R22
end;

{It is a compilerproc (systemh.inc), make an alias for internal use.}
{$ifdef FPC_IS_SYSTEM}
function fpc_div_byte(n, z: byte): byte; external name 'FPC_DIV_BYTE';
{$endif FPC_IS_SYSTEM}
{$endif FPC_SYSTEM_HAS_DIV_BYTE}


{$ifndef FPC_SYSTEM_HAS_MOD_BYTE}
{$define FPC_SYSTEM_HAS_MOD_BYTE}

// z in Ra, n in Rb, 0 in Rp
function fpc_mod_byte(n, z: byte): byte; assembler; nostackframe;
{$ifdef FPC_IS_SYSTEM}[public,alias: 'FPC_MOD_BYTE'];{$endif}
asm
{$ifdef CPUAVR_16_REGS}
  cp R24, R17
{$else CPUAVR_16_REGS}
  cp R24, R1
{$endif CPUAVR_16_REGS}
  brne .LNonZero
{$ifdef CPUAVR_HAS_JMP_CALL}
  call fpc_divbyzero
.LNonZero:
  call fpc_divmod_byte
{$else  CPUAVR_HAS_JMP_CALL}
  rcall fpc_divbyzero
.LNonZero:
  rcall fpc_divmod_byte
{$endif CPUAVR_HAS_JMP_CALL}

  mov R24, R20
end;

{It is a compilerproc (systemh.inc), make an alias for internal use.}
{$ifdef FPC_IS_SYSTEM}
function fpc_mod_byte(n, z: byte): byte; external name 'FPC_MOD_BYTE';
{$endif FPC_IS_SYSTEM}
{$endif FPC_SYSTEM_HAS_MOD_BYTE}


{$ifndef FPC_SYSTEM_HAS_DIV_WORD}
{$define FPC_SYSTEM_HAS_DIV_WORD}

// z in Ra, n in Rb, 0 in Rp
function fpc_div_word(n, z: word): word; assembler; nostackframe;
{$ifdef FPC_IS_SYSTEM}[public,alias: 'FPC_DIV_WORD'];{$endif}
asm
{$ifdef CPUAVR_16_REGS}
  cp R24, R17
  cpc R25, R17
{$else CPUAVR_16_REGS}
  cp R24, R1
  cpc R25, R1
{$endif CPUAVR_16_REGS}
  brne .LNonZero
{$ifdef CPUAVR_HAS_JMP_CALL}
  call fpc_divbyzero
.LNonZero:
  call fpc_divmod_word
{$else  CPUAVR_HAS_JMP_CALL}
  rcall fpc_divbyzero
.LNonZero:
  rcall fpc_divmod_word
{$endif CPUAVR_HAS_JMP_CALL}

{$if not(defined(CPUAVR_HAS_MOVW))}
  mov R24, R20
  mov R25, R21
{$else CPUAVR_16_REGS}
  movw R24, R22
{$endif CPUAVR_16_REGS}
end;

{It is a compilerproc (systemh.inc), make an alias for internal use.}
{$ifdef FPC_IS_SYSTEM}
function fpc_div_word(n, z: word): word; external name 'FPC_DIV_WORD';
{$endif FPC_IS_SYSTEM}
{$endif FPC_SYSTEM_HAS_DIV_WORD}


{$ifndef FPC_SYSTEM_HAS_MOD_WORD}
{$define FPC_SYSTEM_HAS_MOD_WORD}

// z in Ra, n in Rb, 0 in Rp
function fpc_mod_word(n, z: word): word; assembler; nostackframe;
{$ifdef FPC_IS_SYSTEM}[public,alias: 'FPC_MOD_WORD'];{$endif}
asm
{$ifdef CPUAVR_16_REGS}
  cp R24, R17
  cpc R25, R17
{$else CPUAVR_16_REGS}
  cp R24, R1
  cpc R25, R1
{$endif CPUAVR_16_REGS}
  brne .LNonZero
{$ifdef CPUAVR_HAS_JMP_CALL}
  call fpc_divbyzero
.LNonZero:
  call fpc_divmod_word
{$else  CPUAVR_HAS_JMP_CALL}
  rcall fpc_divbyzero
.LNonZero:
  rcall fpc_divmod_word
{$endif CPUAVR_HAS_JMP_CALL}

{$if not(defined(CPUAVR_HAS_MOVW))}
  mov R24, R20
  mov R25, R21
{$else not(defined(CPUAVR_HAS_MOVW))}
  movw R24, R20
{$endif not(defined(CPUAVR_HAS_MOVW))}
end;

{It is a compilerproc (systemh.inc), make an alias for internal use.}
{$ifdef FPC_IS_SYSTEM}
function fpc_mod_word(n, z: word): word; external name 'FPC_MOD_WORD';
{$endif FPC_IS_SYSTEM}
{$endif FPC_SYSTEM_HAS_MOD_WORD}


{$ifndef FPC_SYSTEM_HAS_DIV_DWORD}
{$define FPC_SYSTEM_HAS_DIV_DWORD}

// z in Ra, n in Rb, 0 in Rp
function fpc_div_dword(n, z: dword): dword; assembler; nostackframe;
{$ifdef FPC_IS_SYSTEM}[public,alias: 'FPC_DIV_DWORD'];{$endif}
asm
{$ifdef CPUAVR_16_REGS}
  cp R24, R17
  cpc R25, R17
  cpc R22, R17
  cpc R23, R17
{$else CPUAVR_16_REGS}
  cp R24, R1
  cpc R25, R1
  cpc R22, R1
  cpc R23, R1
{$endif CPUAVR_16_REGS}
  brne .LNonZero
{$ifdef CPUAVR_HAS_JMP_CALL}
  call fpc_divbyzero
.LNonZero:
  call fpc_divmod_dword
{$else  CPUAVR_HAS_JMP_CALL}
  rcall fpc_divbyzero
.LNonZero:
  rcall fpc_divmod_dword
{$endif CPUAVR_HAS_JMP_CALL}

{$if not(defined(CPUAVR_HAS_MOVW))}
  mov R22, R18    // Move result from R18:R21 to R22:R25
  mov R23, R19    // Move result from R18:R21 to R22:R25
  mov R24, R20
  mov R25, R21
{$else not(defined(CPUAVR_HAS_MOVW))}
  movw R22, R18    // Move result from R18:R21 to R22:R25
  movw R24, R20
{$endif not(defined(CPUAVR_HAS_MOVW))}
end;

{It is a compilerproc (systemh.inc), make an alias for internal use.}
{$ifdef FPC_IS_SYSTEM}
function fpc_div_dword(n, z: dword): dword; external name 'FPC_DIV_DWORD';
{$endif FPC_IS_SYSTEM}
{$endif FPC_SYSTEM_HAS_DIV_DWORD}


{$ifndef FPC_SYSTEM_HAS_MOD_DWORD}
{$define FPC_SYSTEM_HAS_MOD_DWORD}

// z in Ra, n in Rb, 0 in Rp
function fpc_mod_dword(n, z: dword): dword; assembler; nostackframe;
{$ifdef FPC_IS_SYSTEM}[public,alias: 'FPC_MOD_DWORD'];{$endif}
asm
{$ifdef CPUAVR_16_REGS}
  cp R24, R17
  cpc R25, R17
  cpc R22, R17
  cpc R23, R17
{$else CPUAVR_16_REGS}
  cp R24, R1
  cpc R25, R1
  cpc R22, R1
  cpc R23, R1
{$endif CPUAVR_16_REGS}
  brne .LNonZero
{$ifdef CPUAVR_HAS_JMP_CALL}
  call fpc_divbyzero
.LNonZero:
  call fpc_divmod_dword
{$else  CPUAVR_HAS_JMP_CALL}
  rcall fpc_divbyzero
.LNonZero:
  rcall fpc_divmod_dword
{$endif CPUAVR_HAS_JMP_CALL}
end;

{It is a compilerproc (systemh.inc), make an alias for internal use.}
{$ifdef FPC_IS_SYSTEM}
function fpc_mod_dword(n, z: dword): dword; external name 'FPC_MOD_DWORD';
{$endif FPC_IS_SYSTEM}
{$endif FPC_SYSTEM_HAS_MOD_DWORD}


{$ifndef FPC_SYSTEM_HAS_BSF_BYTE}
{$define FPC_SYSTEM_HAS_BSF_BYTE}

function BsfByte(Const AValue: Byte): Byte;
  var
    i, value: byte;
  begin
    Value:=AValue;
    for i:=0 to 7 do
      begin
        if odd(Value) then
          exit(i);
        Value:=Value shr 1;
      end;
    result:=$FF;
  end;

{$endif}


{$ifndef FPC_SYSTEM_HAS_BSR_BYTE}
{$define FPC_SYSTEM_HAS_BSR_BYTE}

function BsrByte(Const AValue: Byte): Byte;
  var
    i, value: byte;
  begin
    Value:=AValue;
    for i:=0 to 7 do
      begin
        if (Value and $80)<>0 then
          exit(i);
        Value:=Value shl 1;
      end;
    result:=$FF;
  end;

{$endif}
