{$mode objfpc}

program testbitscan;

function test_byte: boolean;
var
  x8,f,r: byte;
  i: integer;
begin
  for i:=0 to 7 do
  begin
    x8:=1 shl i;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    f:=BsfByte(x8 or ((7-i) shl i));
    if (f<>i) then
    begin
      writeln('BsfByte($',hexstr(x8 or ((7-i) shl i),2),') returned ',f,', should be ',i);
      exit(false);
    end;
    r:=BsrByte(x8 or i);
    if r<>i then
    begin
      writeln('BsrByte($',hexstr(x8 or i,2),') returned ',r,', should be ',i);
      exit(false);
    end;
  end;
  x8:=0;
  f:=BsfByte(x8);
  if (f<>$ff) then
  begin
    writeln('BsfByte(',x8,') returned ',f,', should be ',$ff);
    exit(false);
  end;
  r:=BsrByte(x8);
  if r<>$ff then
  begin
    writeln('BsrByte(',x8,') returned ',r,', should be ',$ff);
    exit(false);
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    f:=BsfByte(x8);
    if (f<>i) then
    begin
      writeln('BsfByte(',x8,') returned ',f,', should be ',i);
      exit(false);
    end;
    r:=BsrByte(x8);
    if r<>i then
    begin
      writeln('BsrByte(',x8,') returned ',f,', should be ',i);
      exit(false);
    end;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  result:=true;
end;

function test_word: boolean;
var
  x16: word;
  i,f,r: integer;
begin
  for i:=0 to 15 do
  begin
    x16:=1 shl i;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    f:=BsfWord(x16 or ((15-i) shl i));
=======
    f:=BsfWord(x16);
>>>>>>> graemeg/cpstrnew
=======
    f:=BsfWord(x16);
>>>>>>> graemeg/cpstrnew
=======
    f:=BsfWord(x16);
>>>>>>> graemeg/cpstrnew
=======
    f:=BsfWord(x16);
>>>>>>> origin/cpstrnew
    if (f<>i) then
    begin
      writeln('BsfWord(',x16,') returned ',f,', should be ',i);
      exit(false);
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    r:=BsrWord(x16 or i);
    if r<>i then
    begin
      writeln('BsrWord(',x16,') returned ',r,', should be ',i);
      exit(false);
    end;
  end;
  x16:=0;
  f:=BsfWord(x16);
  if (f<>$ff) then
  begin
    writeln('BsfWord(',x16,') returned ',f,', should be ',$ff);
    exit(false);
  end;
  r:=BsrWord(x16);
  if r<>$ff then
  begin
    writeln('BsrWord(',x16,') returned ',r,', should be ',$ff);
    exit(false);
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    r:=BsrWord(x16);
    if r<>i then
    begin
      writeln('BsrWord(',x16,') returned ',f,', should be ',i);
      exit(false);
    end;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  result:=true;
end;

function test_dword: boolean;
var
  x32: cardinal;
  i,f,r: integer;
begin
  for i:=0 to 31 do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    x32:=cardinal(1) shl i;
    f:=BsfDWord(x32 or ((31-i) shl i));
=======
    x32:=1 shl i;
    f:=BsfDWord(x32);
>>>>>>> graemeg/cpstrnew
=======
    x32:=1 shl i;
    f:=BsfDWord(x32);
>>>>>>> graemeg/cpstrnew
=======
    x32:=1 shl i;
    f:=BsfDWord(x32);
>>>>>>> graemeg/cpstrnew
=======
    x32:=1 shl i;
    f:=BsfDWord(x32);
>>>>>>> origin/cpstrnew
    if (f<>i) then
    begin
      writeln('BsfDWord(',x32,') returned ',f,', should be ',i);
      exit(false);
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    r:=BsrDWord(x32 or i);
    if r<>i then
    begin
      writeln('BsrDWord(',x32,') returned ',r,', should be ',i);
      exit(false);
    end;
  end;
  x32:=0;
  f:=BsfDWord(x32);
  if (f<>$ff) then
  begin
    writeln('BsfDWord(',x32,') returned ',f,', should be ',$ff);
    exit(false);
  end;
  r:=BsrDWord(x32);
  if r<>$ff then
  begin
    writeln('BsrDWord(',x32,') returned ',r,', should be ',$ff);
    exit(false);
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    r:=BsrDWord(x32);
    if r<>i then
    begin
      writeln('BsrDWord(',x32,') returned ',f,', should be ',i);
      exit(false);
    end;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  result:=true;
end;

function test_qword: boolean;
var
  x64: qword;
  i, f, r: integer;
begin
  for i:=0 to 63 do
  begin
    x64:=uint64(1) shl i;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    f:=BsfQWord(x64 or (uint64(63-i) shl i));
=======
    f:=BsfQWord(x64);
>>>>>>> graemeg/cpstrnew
=======
    f:=BsfQWord(x64);
>>>>>>> graemeg/cpstrnew
=======
    f:=BsfQWord(x64);
>>>>>>> graemeg/cpstrnew
=======
    f:=BsfQWord(x64);
>>>>>>> origin/cpstrnew
    if f<>i then begin
      writeln('BsfQWord(',x64,') returned ',f,', should be ',i);
      exit(false);
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    r:=BsrQWord(x64 or i);
=======
    r:=BsrQWord(x64);
>>>>>>> graemeg/cpstrnew
=======
    r:=BsrQWord(x64);
>>>>>>> graemeg/cpstrnew
=======
    r:=BsrQWord(x64);
>>>>>>> graemeg/cpstrnew
=======
    r:=BsrQWord(x64);
>>>>>>> origin/cpstrnew
    if r<>i then begin
      writeln('BsrQWord(',x64,') returned ',r,', should be ',i);
      exit(false);
    end;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  x64:=0;
  f:=BsfQWord(x64);
  if (f<>$ff) then
  begin
    writeln('BsfQWord(',x64,') returned ',f,', should be ',$ff);
    exit(false);
  end;
  r:=BsrQWord(x64);
  if r<>$ff then
  begin
    writeln('BsrQWord(',x64,') returned ',r,', should be ',$ff);
    exit(false);
  end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  result:=true;
end;

begin
  if test_byte then writeln('passed') else halt(1);
  if test_word then writeln('passed') else halt(1);
  if test_dword then writeln('passed') else halt(1);
  if test_qword then writeln('passed') else halt(1);
end.
