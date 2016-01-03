{ This unit tests the basic routines         }
{ which are usually coded in assembler       }
{ Mainly used in porting to other processors }
{********************************************}
{ Tested against Delphi 6 and Delphi 3       }
{********************************************}
program tmem;

const
  MAX_TABLE = 69;  { this value should not be even ! }
  DEFAULT_VALUE = $55;
  FILL_VALUE = $33;


var
  dst_arraybyte : array[1..MAX_TABLE] of byte;
  src_arraybyte : array[1..MAX_TABLE] of byte;
  dst_arrayword : array[1..MAX_TABLE] of word;
  dst_arraylongword : array[1..MAX_TABLE] of longword;
  i: integer;


procedure test(value, required: longint);
begin
  if value <> required then
    begin
      writeln('Got ',value,' instead of ',required);
      halt(1);
    end;
end;




procedure test_fillchar;
 var
  i: integer;
 begin
  { non-aligned count }
  write('testing fillchar (non-aligned size)...');
  for i := 1 to MAX_TABLE do
    dst_arraybyte[i] := DEFAULT_VALUE;
  fillchar(dst_arraybyte, MAX_TABLE-2, FILL_VALUE);
  test(dst_arraybyte[MAX_TABLE], DEFAULT_VALUE);
  test(dst_arraybyte[MAX_TABLE-1], DEFAULT_VALUE);
  for i := 1 to MAX_TABLE-2 do
    test(dst_arraybyte[i], FILL_VALUE);
  writeln('Passed!');
  { modulo 2 count fill }
  write('testing fillchar (aligned size)...');
  for i := 1 to MAX_TABLE do
    dst_arraybyte[i] := DEFAULT_VALUE;
  fillchar(dst_arraybyte, MAX_TABLE-1, FILL_VALUE);
  test(dst_arraybyte[MAX_TABLE], DEFAULT_VALUE);
  for i := 1 to MAX_TABLE-1 do
    test(dst_arraybyte[i], FILL_VALUE);
  writeln('Passed!');
  { test zero fillchar count }
  write('testing fillchar (zero count)...');
  for i := 1 to MAX_TABLE do
    dst_arraybyte[i] := DEFAULT_VALUE;
  fillchar(dst_arraybyte, 0, FILL_VALUE);
  for i := 1 to MAX_TABLE do
    test(dst_arraybyte[i], DEFAULT_VALUE);
  writeln('Passed!');
  { test negative fillchar count }
  write('testing fillchar (negative count)...');
  for i := 1 to MAX_TABLE do
    dst_arraybyte[i] := DEFAULT_VALUE;
  fillchar(dst_arraybyte, -1, FILL_VALUE);
  for i := 1 to MAX_TABLE do
    test(dst_arraybyte[i], DEFAULT_VALUE);
  writeln('Passed!');
 end;


procedure test_move;
begin
  { non-aligned count }
  write('testing move (non-aligned size)...');
  for i := 1 to MAX_TABLE do
  begin
    dst_arraybyte[i] := DEFAULT_VALUE;
    src_arraybyte[i] := FILL_VALUE;
  end;
  move(src_arraybyte, dst_arraybyte, MAX_TABLE-2);
  test(dst_arraybyte[MAX_TABLE], DEFAULT_VALUE);
  test(dst_arraybyte[MAX_TABLE-1], DEFAULT_VALUE);
  for i:= 1 to MAX_TABLE-2 do
    test(dst_arraybyte[i], FILL_VALUE);
  writeln('Passed!');
  { modulo 2 count fill }
  { non-aligned count }
  write('testing move (aligned size)...');
  for i := 1 to MAX_TABLE do
  begin
    dst_arraybyte[i] := DEFAULT_VALUE;
    src_arraybyte[i] := FILL_VALUE;
  end;
  move(src_arraybyte, dst_arraybyte, MAX_TABLE-1);
  test(dst_arraybyte[MAX_TABLE], DEFAULT_VALUE);
  for i:= 1 to MAX_TABLE-1 do
    test(dst_arraybyte[i], FILL_VALUE);
  writeln('Passed!');
  { zero move count }
  write('testing move (zero count)...');
  for i := 1 to MAX_TABLE do
  begin
    dst_arraybyte[i] := DEFAULT_VALUE;
    src_arraybyte[i] := FILL_VALUE;
  end;
  move(src_arraybyte,dst_arraybyte, 0);
  for i:= 1 to MAX_TABLE do
    test(dst_arraybyte[i], DEFAULT_VALUE);
  writeln('Passed!');
  { negative move count }
  write('testing move (negative count)...');
  move(src_arraybyte,dst_arraybyte,-12);
  writeln('Passed!');
end;


procedure test_move_large(size: longint);
var
  src, dst: PLongInt;
  i: LongInt;
begin
  GetMem(src, size*sizeof(LongInt));
  GetMem(dst, size*sizeof(LongInt));
  write('testing move of ',size,' dwords ...');
  for i := 0 to size-1 do
  begin
    src[i] := i;
    dst[i] := -1;
  end;
  move(src[0], dst[2], (size-4)*sizeof(LongInt));
  test(dst[0], -1);
  test(dst[1], -1);
  test(dst[size-1], -1);
  test(dst[size-2], -1);
  for i := 2 to size-3 do
    test(dst[i], i-2);
  writeln('Passed!');

  // repeat with source and dest swapped (maybe move in opposite direction)
  // current implementations detect that regions don't overlap and move forward,
  // so this test is mostly useless. But it won't harm anyway.
  write('testing move of ',size,' dwords, opposite direction...');
  for i := 0 to size-1 do
  begin
    dst[i] := i;
    src[i] := -1;
  end;
  move(dst[0], src[2], (size-4)*sizeof(LongInt));
  test(src[0], -1);
  test(src[1], -1);
  test(src[size-1], -1);
  test(src[size-2], -1);
  for i := 2 to size-3 do
    test(src[i], i-2);
  writeln('Passed!');

  write('testing move of ',size,' dwords, overlapping forward...');
  for i := 0 to size-1 do
    src[i] := i;
  move(src[0], src[100], (size-100)*sizeof(LongInt));
  for i := 0 to 99 do
    test(src[i], i);
  for i := 100 to size-101 do
    test(src[i], i-100);
  writeln('Passed!');

  write('testing move of ',size,' dwords, overlapping backward...');
  for i := 0 to size-1 do
    src[i] := i;
  move(src[100], src[0], (size-100)*sizeof(LongInt));
  for i := 0 to size-101 do
    test(src[i], i+100);
  for i := size-100 to size-1 do
    test(src[i], i);
  writeln('Passed!');
  FreeMem(dst);
  FreeMem(src);
end;

{$ifdef fpc}
procedure test_fillword;
 var
  i: integer;
 begin
  { non-aligned count }
  write('testing fillword (non-aligned size)...');
  for i := 1 to MAX_TABLE do
    dst_arrayword[i] := DEFAULT_VALUE;
  fillword(dst_arrayword, MAX_TABLE-2, FILL_VALUE);
  test(dst_arrayword[MAX_TABLE], DEFAULT_VALUE);
  test(dst_arrayword[MAX_TABLE-1], DEFAULT_VALUE);
  for i := 1 to MAX_TABLE-2 do
    test(dst_arrayword[i], FILL_VALUE);
  writeln('Passed!');
  { modulo 2 count fill }
  write('testing fillword (aligned size)...');
  for i := 1 to MAX_TABLE do
    dst_arrayword[i] := DEFAULT_VALUE;
  fillword(dst_arrayword, MAX_TABLE-1, FILL_VALUE);
  test(dst_arrayword[MAX_TABLE], DEFAULT_VALUE);
  for i := 1 to MAX_TABLE-1 do
    test(dst_arrayword[i], FILL_VALUE);
  writeln('Passed!');
  { test zero fillword count }
  write('testing fillword (zero count)...');
  for i := 1 to MAX_TABLE do
    dst_arrayword[i] := DEFAULT_VALUE;
  fillword(dst_arrayword, 0, FILL_VALUE);
  for i := 1 to MAX_TABLE do
    test(dst_arrayword[i], DEFAULT_VALUE);
  writeln('Passed!');
  { test negative fillword count }
  write('testing fillword (negative count)...');
  for i := 1 to MAX_TABLE do
    dst_arrayword[i] := DEFAULT_VALUE;
  fillword(dst_arrayword, -1, FILL_VALUE);
  writeln('Passed!');
 end;


procedure test_filldword;
 var
  i: integer;
 begin
  { non-aligned count }
  write('testing filldword (non-aligned size)...');
  for i := 1 to MAX_TABLE do
    dst_arraylongword[i] := DEFAULT_VALUE;
  filldword(dst_arraylongword, MAX_TABLE-2, FILL_VALUE);
  test(dst_arraylongword[MAX_TABLE], DEFAULT_VALUE);
  test(dst_arraylongword[MAX_TABLE-1], DEFAULT_VALUE);
  for i := 1 to MAX_TABLE-2 do
    test(dst_arraylongword[i], FILL_VALUE);
  writeln('Passed!');
  { modulo 2 count fill }
  write('testing filldword (aligned size)...');
  for i := 1 to MAX_TABLE do
    dst_arraylongword[i] := DEFAULT_VALUE;
  filldword(dst_arraylongword, MAX_TABLE-1, FILL_VALUE);
  test(dst_arraylongword[MAX_TABLE], DEFAULT_VALUE);
  for i := 1 to MAX_TABLE-1 do
    test(dst_arraylongword[i], FILL_VALUE);
  writeln('Passed!');
  { test zero filldword count }
  write('testing filldword (zero count)...');
  for i := 1 to MAX_TABLE do
    dst_arraylongword[i] := DEFAULT_VALUE;
  filldword(dst_arraylongword, 0, FILL_VALUE);
  for i := 1 to MAX_TABLE do
    test(dst_arraylongword[i], DEFAULT_VALUE);
  writeln('Passed!');
  { test negative filldword count }
  write('testing filldword (negative count)...');
  for i := 1 to MAX_TABLE do
    dst_arraylongword[i] := DEFAULT_VALUE;
  filldword(dst_arraylongword, -1, FILL_VALUE);
  writeln('Passed!');
 end;


procedure test_movechar0;
begin
  { non-aligned count }
  write('testing movechar0 (non-aligned size)...');
  for i := 1 to MAX_TABLE do
  begin
    dst_arraybyte[i] := DEFAULT_VALUE;
    src_arraybyte[i] := FILL_VALUE;
  end;
  movechar0(src_arraybyte, dst_arraybyte, MAX_TABLE-2);
  test(dst_arraybyte[MAX_TABLE], DEFAULT_VALUE);
  test(dst_arraybyte[MAX_TABLE-1], DEFAULT_VALUE);
  for i:= 1 to MAX_TABLE-2 do
    test(dst_arraybyte[i], FILL_VALUE);
  writeln('Passed!');
  { modulo 2 count fill }
  { non-aligned count }
  write('testing movechar0 (aligned size)...');
  for i := 1 to MAX_TABLE do
  begin
    dst_arraybyte[i] := DEFAULT_VALUE;
    src_arraybyte[i] := FILL_VALUE;
  end;
  movechar0(src_arraybyte, dst_arraybyte, MAX_TABLE-1);
  test(dst_arraybyte[MAX_TABLE], DEFAULT_VALUE);
  for i:= 1 to MAX_TABLE-1 do
    test(dst_arraybyte[i], FILL_VALUE);
  writeln('Passed!');
  { zero movechar0 count }
  write('test movechar0 (zero count)...');
  for i := 1 to MAX_TABLE do
  begin
    dst_arraybyte[i] := DEFAULT_VALUE;
    src_arraybyte[i] := FILL_VALUE;
  end;
  movechar0(src_arraybyte,dst_arraybyte, 0);
  for i:= 1 to MAX_TABLE do
    test(dst_arraybyte[i], DEFAULT_VALUE);
  writeln('Passed!');
  { withh null value as first value in index }
  write('test movechar0 with null character...');
  for i := 1 to MAX_TABLE do
  begin
    dst_arraybyte[i] := DEFAULT_VALUE;
    src_arraybyte[i] := 0;
  end;
  movechar0(src_arraybyte, dst_arraybyte, MAX_TABLE-1);
  { nothing should have been moved }
  for i:= 1 to MAX_TABLE do
    test(dst_arraybyte[i], DEFAULT_VALUE);
  writeln('Passed!');
  { with null value as second value in index }
  write('test movechar0 with null character (and char)...');
  for i := 1 to MAX_TABLE do
  begin
    dst_arraybyte[i] := DEFAULT_VALUE;
  end;
  src_arraybyte[1] := FILL_VALUE;
  src_arraybyte[2] := 0;
  movechar0(src_arraybyte, dst_arraybyte, MAX_TABLE-1);
  test(dst_arraybyte[1], FILL_VALUE);
  { the rest should normally not have bene touched }
  test(dst_arraybyte[2], DEFAULT_VALUE);
  writeln('Passed!');
end;
{$endif}


begin
  test_fillchar;
  test_move;
  test_move_large(500);   // 512 longints=2048 bytes
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef CPU16}
  test_move_large(500000);
{$endif CPU16}
=======
  test_move_large(500000);
>>>>>>> graemeg/cpstrnew
=======
  test_move_large(500000);
>>>>>>> graemeg/cpstrnew
=======
  test_move_large(500000);
>>>>>>> graemeg/cpstrnew
=======
  test_move_large(500000);
>>>>>>> origin/cpstrnew
{$ifdef fpc}
  test_fillword;
  test_filldword;
  test_movechar0;
{$endif}
end.
