{
  Copyright (c) 2004, John O'Harrow (john@almcrest.demon.co.uk)

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the
use of this software.

Permission is granted to anyone to use this software for any purpose, including
commercial applications, and to alter it and redistribute it freely, subject to
the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim
   that you wrote the original software. If you use this software in a product,
   an acknowledgment in the product documentation would be appreciated but is
   not required.

2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.

3. This notice may not be removed or altered from any source distribution.

-------------------------------------------------------------------------------

Version: 1.40 - 16-SEP-2004
}

{$ifdef USE_FASTMOVE}

{$ifndef FPC_SYSTEM_HAS_MOVE}
{$define FPC_SYSTEM_HAS_MOVE}

{$asmmode intel}

{-------------------------------------------------------------------------}
(*
{Just to show that a good Pascal algorithm can beat the default BASM}
procedure MoveJOH_PAS_3(const Source; var Dest; Count : Integer);
var
  S, D       : PtrUInt;
  Temp, C, I : PtrInt;
  L          : PPtrInt;
begin
  S := Cardinal(@Source);
  D := Cardinal(@Dest);
  if S = D then
    Exit;
  if Count <= 4 then
    case Count of
      1 : PByte(@Dest)^ := PByte(S)^;
      2 : PWord(@Dest)^ := PWord(S)^;
      3 : if D > S then
            begin
              PByte(Integer(@Dest)+2)^ := PByte(S+2)^;
              PWord(@Dest)^ := PWord(S)^;
            end
          else
            begin
              PWord(@Dest)^ := PWord(S)^;
              PByte(Integer(@Dest)+2)^ := PByte(S+2)^;
            end;
      4 : PInteger(@Dest)^ := PInteger(S)^
      else Exit; {Count <= 0}
    end
  else
    if D > S then
      begin
        Temp := PInteger(S)^;
        I := Integer(@Dest);
        C := Count - 4;
        L := PInteger(Integer(@Dest) + C);
        Inc(S, C);
        repeat
          L^ := PInteger(S)^;
          if Count <= 8 then
            Break;
          Dec(Count, 4);
          Dec(S, 4);
          Dec(L);
        until False;
        PInteger(I)^ := Temp;
      end
    else
      begin
        C := Count - 4;
        Temp := PInteger(S + Cardinal(C))^;
        I := Integer(@Dest) + C;
        L := @Dest;
        repeat
          L^ := PInteger(S)^;
          if Count <= 8 then
            Break;
          Dec(Count, 4);
          Inc(S, 4);
          Inc(L);
        until False;
        PInteger(I)^ := Temp;
      end;
end; {MoveJOH_PAS}
*)

const
  SMALLMOVESIZE = 36;

{-------------------------------------------------------------------------}
{Perform Forward Move of 0..36 Bytes}
{On Entry, ECX = Count, EAX = Source+Count, EDX = Dest+Count.  Destroys ECX}
procedure SmallForwardMove_3;assembler;nostackframe;
asm
  jmp     dword ptr @@FwdJumpTable[ecx*4]
  align   16
@@FwdJumpTable:
  dd      @@Done {Removes need to test for zero size move}
  dd      @@Fwd01,@@Fwd02,@@Fwd03,@@Fwd04,@@Fwd05,@@Fwd06,@@Fwd07,@@Fwd08
  dd      @@Fwd09,@@Fwd10,@@Fwd11,@@Fwd12,@@Fwd13,@@Fwd14,@@Fwd15,@@Fwd16
  dd      @@Fwd17,@@Fwd18,@@Fwd19,@@Fwd20,@@Fwd21,@@Fwd22,@@Fwd23,@@Fwd24
  dd      @@Fwd25,@@Fwd26,@@Fwd27,@@Fwd28,@@Fwd29,@@Fwd30,@@Fwd31,@@Fwd32
  dd      @@Fwd33,@@Fwd34,@@Fwd35,@@Fwd36
@@Fwd36:
  mov     ecx,[eax-36]
  mov     [edx-36],ecx
@@Fwd32:
  mov     ecx,[eax-32]
  mov     [edx-32],ecx
@@Fwd28:
  mov     ecx,[eax-28]
  mov     [edx-28],ecx
@@Fwd24:
  mov     ecx,[eax-24]
  mov     [edx-24],ecx
@@Fwd20:
  mov     ecx,[eax-20]
  mov     [edx-20],ecx
@@Fwd16:
  mov     ecx,[eax-16]
  mov     [edx-16],ecx
@@Fwd12:
  mov     ecx,[eax-12]
  mov     [edx-12],ecx
@@Fwd08:
  mov     ecx,[eax-8]
  mov     [edx-8],ecx
@@Fwd04:
  mov     ecx,[eax-4]
  mov     [edx-4],ecx
  ret
@@Fwd35:
  mov     ecx,[eax-35]
  mov     [edx-35],ecx
@@Fwd31:
  mov     ecx,[eax-31]
  mov     [edx-31],ecx
@@Fwd27:
  mov     ecx,[eax-27]
  mov     [edx-27],ecx
@@Fwd23:
  mov     ecx,[eax-23]
  mov     [edx-23],ecx
@@Fwd19:
  mov     ecx,[eax-19]
  mov     [edx-19],ecx
@@Fwd15:
  mov     ecx,[eax-15]
  mov     [edx-15],ecx
@@Fwd11:
  mov     ecx,[eax-11]
  mov     [edx-11],ecx
@@Fwd07:
  mov     ecx,[eax-7]
  mov     [edx-7],ecx
  mov     ecx,[eax-4]
  mov     [edx-4],ecx
  ret
@@Fwd03:
  movzx   ecx, word ptr [eax-3]
  mov     [edx-3],cx
  movzx   ecx, byte ptr [eax-1]
  mov     [edx-1],cl
  ret
@@Fwd34:
  mov     ecx,[eax-34]
  mov     [edx-34],ecx
@@Fwd30:
  mov     ecx,[eax-30]
  mov     [edx-30],ecx
@@Fwd26:
  mov     ecx,[eax-26]
  mov     [edx-26],ecx
@@Fwd22:
  mov     ecx,[eax-22]
  mov     [edx-22],ecx
@@Fwd18:
  mov     ecx,[eax-18]
  mov     [edx-18],ecx
@@Fwd14:
  mov     ecx,[eax-14]
  mov     [edx-14],ecx
@@Fwd10:
  mov     ecx,[eax-10]
  mov     [edx-10],ecx
@@Fwd06:
  mov     ecx,[eax-6]
  mov     [edx-6],ecx
@@Fwd02:
  movzx   ecx, word ptr [eax-2]
  mov     [edx-2],cx
  ret
@@Fwd33:
  mov     ecx,[eax-33]
  mov     [edx-33],ecx
@@Fwd29:
  mov     ecx,[eax-29]
  mov     [edx-29],ecx
@@Fwd25:
  mov     ecx,[eax-25]
  mov     [edx-25],ecx
@@Fwd21:
  mov     ecx,[eax-21]
  mov     [edx-21],ecx
@@Fwd17:
  mov     ecx,[eax-17]
  mov     [edx-17],ecx
@@Fwd13:
  mov     ecx,[eax-13]
  mov     [edx-13],ecx
@@Fwd09:
  mov     ecx,[eax-9]
  mov     [edx-9],ecx
@@Fwd05:
  mov     ecx,[eax-5]
  mov     [edx-5],ecx
@@Fwd01:
  movzx   ecx, byte ptr [eax-1]
  mov     [edx-1],cl
@@Done:
end; {SmallForwardMove}

{-------------------------------------------------------------------------}
{Perform Backward Move of 0..36 Bytes}
{On Entry, ECX = Count, EAX = Source, EDX = Dest.  Destroys ECX}
procedure SmallBackwardMove_3;assembler;nostackframe;
asm
  jmp     dword ptr @@BwdJumpTable[ecx*4]
  align   16
@@BwdJumpTable:
  dd      @@Done {Removes need to test for zero size move}
  dd      @@Bwd01,@@Bwd02,@@Bwd03,@@Bwd04,@@Bwd05,@@Bwd06,@@Bwd07,@@Bwd08
  dd      @@Bwd09,@@Bwd10,@@Bwd11,@@Bwd12,@@Bwd13,@@Bwd14,@@Bwd15,@@Bwd16
  dd      @@Bwd17,@@Bwd18,@@Bwd19,@@Bwd20,@@Bwd21,@@Bwd22,@@Bwd23,@@Bwd24
  dd      @@Bwd25,@@Bwd26,@@Bwd27,@@Bwd28,@@Bwd29,@@Bwd30,@@Bwd31,@@Bwd32
  dd      @@Bwd33,@@Bwd34,@@Bwd35,@@Bwd36
@@Bwd36:
  mov     ecx,[eax+32]
  mov     [edx+32],ecx
@@Bwd32:
  mov     ecx,[eax+28]
  mov     [edx+28],ecx
@@Bwd28:
  mov     ecx,[eax+24]
  mov     [edx+24],ecx
@@Bwd24:
  mov     ecx,[eax+20]
  mov     [edx+20],ecx
@@Bwd20:
  mov     ecx,[eax+16]
  mov     [edx+16],ecx
@@Bwd16:
  mov     ecx,[eax+12]
  mov     [edx+12],ecx
@@Bwd12:
  mov     ecx,[eax+8]
  mov     [edx+8],ecx
@@Bwd08:
  mov     ecx,[eax+4]
  mov     [edx+4],ecx
@@Bwd04:
  mov     ecx,[eax]
  mov     [edx],ecx
  ret
@@Bwd35:
  mov     ecx,[eax+31]
  mov     [edx+31],ecx
@@Bwd31:
  mov     ecx,[eax+27]
  mov     [edx+27],ecx
@@Bwd27:
  mov     ecx,[eax+23]
  mov     [edx+23],ecx
@@Bwd23:
  mov     ecx,[eax+19]
  mov     [edx+19],ecx
@@Bwd19:
  mov     ecx,[eax+15]
  mov     [edx+15],ecx
@@Bwd15:
  mov     ecx,[eax+11]
  mov     [edx+11],ecx
@@Bwd11:
  mov     ecx,[eax+7]
  mov     [edx+7],ecx
@@Bwd07:
  mov     ecx,[eax+3]
  mov     [edx+3],ecx
  mov     ecx,[eax]
  mov     [edx],ecx
  ret
@@Bwd03:
  movzx   ecx, word ptr [eax+1]
  mov     [edx+1],cx
  movzx   ecx, byte ptr [eax]
  mov     [edx],cl
  ret
@@Bwd34:
  mov     ecx,[eax+30]
  mov     [edx+30],ecx
@@Bwd30:
  mov     ecx,[eax+26]
  mov     [edx+26],ecx
@@Bwd26:
  mov     ecx,[eax+22]
  mov     [edx+22],ecx
@@Bwd22:
  mov     ecx,[eax+18]
  mov     [edx+18],ecx
@@Bwd18:
  mov     ecx,[eax+14]
  mov     [edx+14],ecx
@@Bwd14:
  mov     ecx,[eax+10]
  mov     [edx+10],ecx
@@Bwd10:
  mov     ecx,[eax+6]
  mov     [edx+6],ecx
@@Bwd06:
  mov     ecx,[eax+2]
  mov     [edx+2],ecx
@@Bwd02:
  movzx   ecx, word ptr [eax]
  mov     [edx],cx
  ret
@@Bwd33:
  mov     ecx,[eax+29]
  mov     [edx+29],ecx
@@Bwd29:
  mov     ecx,[eax+25]
  mov     [edx+25],ecx
@@Bwd25:
  mov     ecx,[eax+21]
  mov     [edx+21],ecx
@@Bwd21:
  mov     ecx,[eax+17]
  mov     [edx+17],ecx
@@Bwd17:
  mov     ecx,[eax+13]
  mov     [edx+13],ecx
@@Bwd13:
  mov     ecx,[eax+9]
  mov     [edx+9],ecx
@@Bwd09:
  mov     ecx,[eax+5]
  mov     [edx+5],ecx
@@Bwd05:
  mov     ecx,[eax+1]
  mov     [edx+1],ecx
@@Bwd01:
  movzx   ecx, byte ptr[eax]
  mov     [edx],cl
@@Done:
end; {SmallBackwardMove}


{ at least valgrind up to 3.3 has a bug which prevents the default code to
  work so we use a rather simple implementation here
}
procedure Forwards_Valgrind;assembler;nostackframe;
asm
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_ENABLED_CLD}
  cld
{$endif FPC_ENABLED_CLD}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  push    esi
  push    edi
  mov     esi,eax
  mov     edi,edx
  rep     movsb
  pop     edi
  pop     esi
end;

{ at least valgrind up to 3.3 has a bug which prevents the default code to
  work so we use a rather simple implementation here
}
procedure Backwards_Valgrind;assembler;nostackframe;
asm
  push    esi
  push    edi
  lea     esi,[eax+ecx-1]
  lea     edi,[edx+ecx-1]
@@repeat:
  mov     al,[esi]
  mov     [edi],al
  dec     esi
  dec     edi
  dec     ecx
  jnz     @@repeat
  pop     edi
  pop     esi
end;

{-------------------------------------------------------------------------}
{Move ECX Bytes from EAX to EDX, where EAX > EDX and ECX > 36 (SMALLMOVESIZE)}
procedure Forwards_IA32_3;assembler;nostackframe;
asm
  push    ebx
  mov     ebx,edx
  fild    qword ptr [eax]
  add     eax,ecx {QWORD Align Writes}
  add     ecx,edx
  add     edx,7
  and     edx,-8
  sub     ecx,edx
  add     edx,ecx {Now QWORD Aligned}
  sub     ecx,16
  neg     ecx
@FwdLoop:
  fild    qword ptr [eax+ecx-16]
  fistp   qword ptr [edx+ecx-16]
  fild    qword ptr [eax+ecx-8]
  fistp   qword ptr [edx+ecx-8]
  add     ecx,16
  jle     @FwdLoop
  fistp   qword ptr [ebx]
  neg     ecx
  add     ecx,16
  pop     ebx
  jmp     SmallForwardMove_3
end; {Forwards_IA32}

{-------------------------------------------------------------------------}
{Move ECX Bytes from EAX to EDX, where EAX < EDX and ECX > 36 (SMALLMOVESIZE)}
procedure Backwards_IA32_3;assembler;nostackframe;
asm
  push    ebx
  fild    qword ptr [eax+ecx-8]
  lea     ebx,[edx+ecx] {QWORD Align Writes}
  and     ebx,7
  sub     ecx,ebx
  add     ebx,ecx {Now QWORD Aligned, EBX = Original Length}
  sub     ecx,16
@BwdLoop:
  fild    qword ptr [eax+ecx]
  fild    qword ptr [eax+ecx+8]
  fistp   qword ptr [edx+ecx+8]
  fistp   qword ptr [edx+ecx]
  sub     ecx,16
  jge     @BwdLoop
  fistp   qword ptr [edx+ebx-8]
  add     ecx,16
  pop     ebx
  jmp     SmallBackwardMove_3
end; {Backwards_IA32}

{-------------------------------------------------------------------------}
{Move ECX Bytes from EAX to EDX, where EAX > EDX and ECX > 36 (SMALLMOVESIZE)}
procedure Forwards_MMX_3;assembler;nostackframe;
const
  LARGESIZE = 1024;
asm
  cmp     ecx,LARGESIZE
  jge     @FwdLargeMove
  cmp     ecx,72 {Size at which using MMX becomes worthwhile}
  jl      Forwards_IA32_3
  push    ebx
  mov     ebx,edx
  movq    mm0,[eax] {First 8 Characters}
  {QWORD Align Writes}
  add     eax,ecx
  add     ecx,edx
  add     edx,7
  and     edx,-8
  sub     ecx,edx
  add     edx,ecx
  {Now QWORD Aligned}
  sub     ecx,32
  neg     ecx
@FwdLoopMMX:
  movq    mm1,[eax+ecx-32]
  movq    mm2,[eax+ecx-24]
  movq    mm3,[eax+ecx-16]
  movq    mm4,[eax+ecx- 8]
  movq    [edx+ecx-32],mm1
  movq    [edx+ecx-24],mm2
  movq    [edx+ecx-16],mm3
  movq    [edx+ecx- 8],mm4
  add     ecx,32
  jle     @FwdLoopMMX
  movq    [ebx],mm0 {First 8 Characters}
  emms
  pop     ebx
  neg     ecx
  add     ecx,32
  jmp     SmallForwardMove_3
@FwdLargeMove:
  push    ebx
  mov     ebx,ecx
  test    edx,15
  jz      @FwdAligned
  {16 byte Align Destination}
  mov     ecx,edx
  add     ecx,15
  and     ecx,-16
  sub     ecx,edx
  add     eax,ecx
  add     edx,ecx
  sub     ebx,ecx
  {Destination now 16 Byte Aligned}
  call    SmallForwardMove_3
@FwdAligned:
  mov     ecx,ebx
  and     ecx,-16
  sub     ebx,ecx {EBX = Remainder}
  push    esi
  push    edi
  mov     esi,eax          {ESI = Source}
  mov     edi,edx          {EDI = Dest}
  mov     eax,ecx          {EAX = Count}
  and     eax,-64          {EAX = No of Bytes to Blocks Moves}
  and     ecx,$3F          {ECX = Remaining Bytes to Move (0..63)}
  add     esi,eax
  add     edi,eax
  shr     eax,3            {EAX = No of QWORD's to Block Move}
  neg     eax
@MMXcopyloop:
  movq    mm0,[esi+eax*8   ]
  movq    mm1,[esi+eax*8+ 8]
  movq    mm2,[esi+eax*8+16]
  movq    mm3,[esi+eax*8+24]
  movq    mm4,[esi+eax*8+32]
  movq    mm5,[esi+eax*8+40]
  movq    mm6,[esi+eax*8+48]
  movq    mm7,[esi+eax*8+56]
  movq    [edi+eax*8   ],mm0
  movq    [edi+eax*8+ 8],mm1
  movq    [edi+eax*8+16],mm2
  movq    [edi+eax*8+24],mm3
  movq    [edi+eax*8+32],mm4
  movq    [edi+eax*8+40],mm5
  movq    [edi+eax*8+48],mm6
  movq    [edi+eax*8+56],mm7
  add     eax,8
  jnz     @MMXcopyloop
  emms                   {Empty MMX State}
{$ifdef FPC_ENABLED_CLD}
  cld
{$endif FPC_ENABLED_CLD}
  add     ecx,ebx
  shr     ecx,2
  rep     movsd
  mov     ecx,ebx
  and     ecx,3
  rep     movsb
  pop     edi
  pop     esi
  pop     ebx
end; {Forwards_MMX}

{-------------------------------------------------------------------------}
{Move ECX Bytes from EAX to EDX, where EAX < EDX and ECX > 36 (SMALLMOVESIZE)}
procedure Backwards_MMX_3;assembler;nostackframe;
asm
  cmp     ecx,72 {Size at which using MMX becomes worthwhile}
  jl      Backwards_IA32_3
  push    ebx
  movq    mm0,[eax+ecx-8] {Get Last QWORD}
  {QWORD Align Writes}
  lea     ebx,[edx+ecx]
  and     ebx,7
  sub     ecx,ebx
  add     ebx,ecx
  {Now QWORD Aligned}
  sub     ecx,32
@BwdLoopMMX:
  movq    mm1,[eax+ecx   ]
  movq    mm2,[eax+ecx+ 8]
  movq    mm3,[eax+ecx+16]
  movq    mm4,[eax+ecx+24]
  movq    [edx+ecx+24],mm4
  movq    [edx+ecx+16],mm3
  movq    [edx+ecx+ 8],mm2
  movq    [edx+ecx   ],mm1
  sub     ecx,32
  jge     @BwdLoopMMX
  movq    [edx+ebx-8], mm0 {Last QWORD}
  emms
  add     ecx,32
  pop     ebx
  jmp     SmallBackwardMove_3
end; {Backwards_MMX}

{-------------------------------------------------------------------------}
{Dest MUST be 16-Byes Aligned, Count MUST be multiple of 16 }
procedure AlignedFwdMoveSSE_3(const Source; var Dest; Count: Integer);assembler;nostackframe;
const
  Prefetch = 512;
asm
  push    esi
  mov     esi,eax             {ESI = Source}
  mov     eax,ecx             {EAX = Count}
  and     eax,-128            {EAX = No of Bytes to Block Move}
  add     esi,eax
  add     edx,eax
  shr     eax,3               {EAX = No of QWORD's to Block Move}
  neg     eax
  cmp     eax, -(32*1024)     {Count > 256K}
  jl      @Large
@Small: {Count<=256K}
  test    esi,15              {Check if Both Source/Dest Aligned}
  jnz     @SmallUnaligned
@SmallAligned:                {Both Source and Dest 16-Byte Aligned}
@SmallAlignedLoop:
  movaps  xmm0,[esi+8*eax]
  movaps  xmm1,[esi+8*eax+16]
  movaps  xmm2,[esi+8*eax+32]
  movaps  xmm3,[esi+8*eax+48]
  movaps  [edx+8*eax],xmm0
  movaps  [edx+8*eax+16],xmm1
  movaps  [edx+8*eax+32],xmm2
  movaps  [edx+8*eax+48],xmm3
  movaps  xmm4,[esi+8*eax+64]
  movaps  xmm5,[esi+8*eax+80]
  movaps  xmm6,[esi+8*eax+96]
  movaps  xmm7,[esi+8*eax+112]
  movaps  [edx+8*eax+64],xmm4
  movaps  [edx+8*eax+80],xmm5
  movaps  [edx+8*eax+96],xmm6
  movaps  [edx+8*eax+112],xmm7
  add     eax,16
  js      @SmallAlignedLoop
  jmp     @Remainder
@SmallUnaligned:              {Source Not 16-Byte Aligned}
@SmallUnalignedLoop:
  movups  xmm0,[esi+8*eax]
  movups  xmm1,[esi+8*eax+16]
  movups  xmm2,[esi+8*eax+32]
  movups  xmm3,[esi+8*eax+48]
  movaps  [edx+8*eax],xmm0
  movaps  [edx+8*eax+16],xmm1
  movaps  [edx+8*eax+32],xmm2
  movaps  [edx+8*eax+48],xmm3
  movups  xmm4,[esi+8*eax+64]
  movups  xmm5,[esi+8*eax+80]
  movups  xmm6,[esi+8*eax+96]
  movups  xmm7,[esi+8*eax+112]
  movaps  [edx+8*eax+64],xmm4
  movaps  [edx+8*eax+80],xmm5
  movaps  [edx+8*eax+96],xmm6
  movaps  [edx+8*eax+112],xmm7
  add     eax,16
  js      @SmallUnalignedLoop
  jmp     @Remainder
@Large: {Count>256K}
  test    esi,15              {Check if Both Source/Dest Aligned}
  jnz     @LargeUnaligned
@LargeAligned:                {Both Source and Dest 16-Byte Aligned}
@LargeAlignedLoop:
  prefetchnta  [esi+8*eax+Prefetch]
  prefetchnta  [esi+8*eax+Prefetch+64]
  movaps  xmm0,[esi+8*eax]
  movaps  xmm1,[esi+8*eax+16]
  movaps  xmm2,[esi+8*eax+32]
  movaps  xmm3,[esi+8*eax+48]
  movntps [edx+8*eax],xmm0
  movntps [edx+8*eax+16],xmm1
  movntps [edx+8*eax+32],xmm2
  movntps [edx+8*eax+48],xmm3
  movaps  xmm4,[esi+8*eax+64]
  movaps  xmm5,[esi+8*eax+80]
  movaps  xmm6,[esi+8*eax+96]
  movaps  xmm7,[esi+8*eax+112]
  movntps [edx+8*eax+64],xmm4
  movntps [edx+8*eax+80],xmm5
  movntps [edx+8*eax+96],xmm6
  movntps [edx+8*eax+112],xmm7
  add     eax,16
  js      @LargeAlignedLoop
  sfence
  jmp     @Remainder
@LargeUnaligned:              {Source Not 16-Byte Aligned}
@LargeUnalignedLoop:
  prefetchnta  [esi+8*eax+Prefetch]
  prefetchnta  [esi+8*eax+Prefetch+64]
  movups  xmm0,[esi+8*eax]
  movups  xmm1,[esi+8*eax+16]
  movups  xmm2,[esi+8*eax+32]
  movups  xmm3,[esi+8*eax+48]
  movntps [edx+8*eax],xmm0
  movntps [edx+8*eax+16],xmm1
  movntps [edx+8*eax+32],xmm2
  movntps [edx+8*eax+48],xmm3
  movups  xmm4,[esi+8*eax+64]
  movups  xmm5,[esi+8*eax+80]
  movups  xmm6,[esi+8*eax+96]
  movups  xmm7,[esi+8*eax+112]
  movntps [edx+8*eax+64],xmm4
  movntps [edx+8*eax+80],xmm5
  movntps [edx+8*eax+96],xmm6
  movntps [edx+8*eax+112],xmm7
  add     eax,16
  js      @LargeUnalignedLoop
  sfence
@Remainder:
  and     ecx,$7F {ECX = Remainder (0..112 - Multiple of 16)}
  jz      @Done
  add     esi,ecx
  add     edx,ecx
  neg     ecx
@RemainderLoop:
  movups  xmm0,[esi+ecx]
  movaps  [edx+ecx],xmm0
  add     ecx,16
  jnz     @RemainderLoop
@Done:
  pop     esi
end; {AlignedFwdMoveSSE}

{-------------------------------------------------------------------------}
{Move ECX Bytes from EAX to EDX, where EAX > EDX and ECX > 36 (SMALLMOVESIZE)}
procedure Forwards_SSE_3;assembler;nostackframe;
const
  LARGESIZE = 2048;
asm
  cmp     ecx,LARGESIZE
  jge     @FwdLargeMove
  cmp     ecx,SMALLMOVESIZE+32
  movups  xmm0,[eax]
  jg      @FwdMoveSSE
  movups  xmm1,[eax+16]
  movups  [edx],xmm0
  movups  [edx+16],xmm1
  add     eax,ecx
  add     edx,ecx
  sub     ecx,32
  jmp     SmallForwardMove_3
@FwdMoveSSE:
  push    ebx
  mov     ebx,edx
  {Align Writes}
  add     eax,ecx
  add     ecx,edx
  add     edx,15
  and     edx,-16
  sub     ecx,edx
  add     edx,ecx
  {Now Aligned}
  sub     ecx,32
  neg     ecx
@FwdLoopSSE:
  movups  xmm1,[eax+ecx-32]
  movups  xmm2,[eax+ecx-16]
  movaps  [edx+ecx-32],xmm1
  movaps  [edx+ecx-16],xmm2
  add     ecx,32
  jle     @FwdLoopSSE
  movups  [ebx],xmm0 {First 16 Bytes}
  neg     ecx
  add     ecx,32
  pop     ebx
  jmp     SmallForwardMove_3
@FwdLargeMove:
  push    ebx
  mov     ebx,ecx
  test    edx,15
  jz      @FwdLargeAligned
  {16 byte Align Destination}
  mov     ecx,edx
  add     ecx,15
  and     ecx,-16
  sub     ecx,edx
  add     eax,ecx
  add     edx,ecx
  sub     ebx,ecx
  {Destination now 16 Byte Aligned}
  call    SmallForwardMove_3
  mov     ecx,ebx
@FwdLargeAligned:
  and     ecx,-16
  sub     ebx,ecx {EBX = Remainder}
  push    edx
  push    eax
  push    ecx
  call    AlignedFwdMoveSSE_3
  pop     ecx
  pop     eax
  pop     edx
  add     ecx,ebx
  add     eax,ecx
  add     edx,ecx
  mov     ecx,ebx
  pop     ebx
  jmp     SmallForwardMove_3
end; {Forwards_SSE}

{-------------------------------------------------------------------------}
{Move ECX Bytes from EAX to EDX, where EAX < EDX and ECX > 36 (SMALLMOVESIZE)}
procedure Backwards_SSE_3;assembler;nostackframe;
asm
  cmp     ecx,SMALLMOVESIZE+32
  jg      @BwdMoveSSE
  sub     ecx,32
  movups  xmm1,[eax+ecx]
  movups  xmm2,[eax+ecx+16]
  movups  [edx+ecx],xmm1
  movups  [edx+ecx+16],xmm2
  jmp     SmallBackwardMove_3
@BwdMoveSSE:
  push    ebx
  movups  xmm0,[eax+ecx-16] {Last 16 Bytes}
  {Align Writes}
  lea     ebx,[edx+ecx]
  and     ebx,15
  sub     ecx,ebx
  add     ebx,ecx
  {Now Aligned}
  sub     ecx,32
@BwdLoop:
  movups  xmm1,[eax+ecx]
  movups  xmm2,[eax+ecx+16]
  movaps  [edx+ecx],xmm1
  movaps  [edx+ecx+16],xmm2
  sub     ecx,32
  jge     @BwdLoop
  movups  [edx+ebx-16],xmm0  {Last 16 Bytes}
  add     ecx,32
  pop     ebx
  jmp     SmallBackwardMove_3
end; {Backwards_SSE}

const
   fastmoveproc_forward : pointer = @Forwards_IA32_3;
   fastmoveproc_backward : pointer = @Backwards_IA32_3;

procedure Move(const source;var dest;count:SizeInt);[public, alias: 'FPC_MOVE'];assembler;nostackframe;
asm
  cmp     ecx,SMALLMOVESIZE
  ja      @Large
  cmp     eax,edx
  lea     eax,[eax+ecx]
  jle     @SmallCheck
@SmallForward:
  add     edx,ecx
  jmp     SmallForwardMove_3
@SmallCheck:
  je      @Done {For Compatibility with Delphi's move for Source = Dest}
  sub     eax,ecx
  jmp     SmallBackwardMove_3
@Large:
  jng     @Done {For Compatibility with Delphi's move for Count < 0}
  cmp     eax,edx
  jg      @moveforward
  je      @Done {For Compatibility with Delphi's move for Source = Dest}
  push    eax
  add     eax,ecx
  cmp     eax,edx
  pop     eax
  jg      @movebackward
@moveforward:
  jmp     dword ptr fastmoveproc_forward
@movebackward:
  jmp     dword ptr fastmoveproc_backward {Source/Dest Overlap}
@Done:
end;

{$asmmode att}
<<<<<<< HEAD
=======
{$ifdef FPC_HAS_VALGRINDBOOL}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_HAS_INDIRECT_MAIN_INFORMATION}
var
  valgrind_used : boolean;external name '__fpc_valgrind';
{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{$endif FPC_HAS_VALGRINDBOOL}
>>>>>>> graemeg/fixes_2_2

procedure setup_fastmove;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    { workaround valgrind bug }
{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}
    if EntryInformation.valgrind_used then
{$else FPC_HAS_INDIRECT_MAIN_INFORMATION}
    if valgrind_used then
{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
      begin
        fastmoveproc_forward:=@Forwards_Valgrind;
        fastmoveproc_backward:=@Backwards_Valgrind;
      end
    else
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{$endif FPC_HAS_VALGRINDBOOL}
>>>>>>> graemeg/fixes_2_2

    if has_sse_support then
      begin
        fastmoveproc_forward:=@Forwards_SSE_3;
        fastmoveproc_backward:=@Backwards_SSE_3;
      end
   else if has_mmx_support then
      begin
        fastmoveproc_forward:=@Forwards_MMX_3;
        fastmoveproc_backward:=@Backwards_MMX_3;
      end;
  end;

{$endif  FPC_SYSTEM_HAS_MOVE}

{$endif}
