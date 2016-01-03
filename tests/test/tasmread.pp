{ Checks for qualified variable support
   in assembler reader }
program test;

{$ifdef cpui386}
{$asmmode intel}
{$endif}

var l: longint;

begin
{$ifdef cpui386}
  asm
{$ifndef FPC_PIC}
     mov test.l, 5
{$else FPC_PIC}
     call @@LPIC
@@LPIC:
     pop ecx
{$ifdef darwin}
     mov [test.l-@@LPIC+ecx],5
{$else darwin}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     add ecx, _GLOBAL_OFFSET_TABLE_+1
=======
     add ecx, _GLOBAL_OFFSET_TABLE_
>>>>>>> graemeg/cpstrnew
=======
     add ecx, _GLOBAL_OFFSET_TABLE_
>>>>>>> graemeg/cpstrnew
=======
     add ecx, _GLOBAL_OFFSET_TABLE_
>>>>>>> graemeg/cpstrnew
=======
     add ecx, _GLOBAL_OFFSET_TABLE_
>>>>>>> origin/cpstrnew
     mov [test.l + ecx],5
=======
     add ecx, @_GLOBAL_OFFSET_TABLE_
     mov [ecx].OFFSET test.l,5
>>>>>>> graemeg/fixes_2_2
=======
     add ecx, @_GLOBAL_OFFSET_TABLE_
     mov [ecx].OFFSET test.l,5
>>>>>>> origin/fixes_2_2
{$endif darwin}
{$endif FPC_PIC}
  end;
{$endif cpui386}
{$ifdef cpu68k}
  asm
     move.l  test.l,d0
  end;
{$endif cpu68k}
{$ifdef arm}
  asm
    ld  r0,test.l
  end;
{$endif arm}

end.
