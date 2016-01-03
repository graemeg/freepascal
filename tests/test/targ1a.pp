<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %NEEDEDAFTER }
=======
>>>>>>> origin/fixes_2.4
program go32v2_crash;

const
  MAX_SIZE = 256;
  SIZE_INC = 8;

type
  TMemArray = array [0..MAX_SIZE div SIZE_INC] of pointer;

var
  i : longint;
  MemArray : TMemArray;

function Size(i: longint) : longint;
begin
  Size:=1+SIZE_INC*i;
end;

begin
  FillChar(MemArray,Sizeof(MemArray),#0);
  for i:=0 to MAX_SIZE div SIZE_INC do
    begin
      GetMem(MemArray[i],Size(i));
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  for i:=0 to MAX_SIZE div SIZE_INC do
    begin
      FreeMem(MemArray[i],Size(i));
    end;
  Writeln(stderr,'Everything is fine');
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
  for i:=1 to MAX_SIZE div SIZE_INC do
    begin
      FreeMem(MemArray[i],Size(i));
    end;
  Writeln(stderr,'Everthing is fine');
<<<<<<< HEAD
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

end.
=======
=======
>>>>>>> origin/fixes_2_2
program go32v2_crash;

const
  MAX_SIZE = 256;
  SIZE_INC = 8;

type
  TMemArray = array [0..MAX_SIZE div SIZE_INC] of pointer;

var
  i : longint;
  MemArray : TMemArray;

function Size(i: longint) : longint;
begin
  Size:=1+SIZE_INC*i;
end;

begin
  FillChar(MemArray,Sizeof(MemArray),#0);
  for i:=0 to MAX_SIZE div SIZE_INC do
    begin
      GetMem(MemArray[i],Size(i));
    end;
  for i:=1 to MAX_SIZE div SIZE_INC do
    begin
      FreeMem(MemArray[i],Size(i));
    end;
  Writeln(stderr,'Everthing is fine');

end.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

end.
>>>>>>> origin/fixes_2.4
