{
   This file is part of the Free Pascal run time library.
   Copyright (c) 2002,2018 by Florian Klaempfl
   members of the Free Pascal development team.

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}

procedure ABITag;nostackframe;assembler;
asm
  .section ".note.ABI-tag", "a", %note
  .align 4
  .long 4
  .long 16
  .long  1
  .asciz "GNU"
  .align 4
  .long 0
  // oldest supported kernel version, just a guess for now (FK)
  .long 2,6,0
  .align 4

  .section ".note.GNU-stack","",@progbits
  .text
end;

