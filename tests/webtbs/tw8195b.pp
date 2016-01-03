{ %cpu=i386 }
{ %OPT=-Cg- }

{$APPTYPE CONSOLE}

{$ifdef fpc}
  {$mode delphi}
{$endif}

function Expression1: Integer;
asm
<<<<<<< HEAD
<<<<<<< HEAD
  mov eax, 4 * 3 - 2 + (-1) / 2
=======
//  mov eax, 4 * 3 - 2 + (-1) / 2
>>>>>>> graemeg/fixes_2_2
=======
//  mov eax, 4 * 3 - 2 + (-1) / 2
>>>>>>> origin/fixes_2_2
end;

function Expression2: Integer;
asm
  mov eax, NOT 4 OR 3 AND 2 XOR 1 MOD 6 SHL 4 SHR 2
end;


begin
  WriteLn('Expression1: ', Expression1);
  WriteLn('Expression2: ', Expression2);
  if (Expression1<>10) or (Expression2<>-1) then
    halt(1);
end.

