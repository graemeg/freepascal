program sha1test;
{$mode objfpc}{$h+}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses SysUtils, sha1;

function performTest: cardinal;
// Runs test and returns result code (0=success)
var
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
uses sha1;

var
  code: cardinal;
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
=======
>>>>>>> origin/fixes_2.4
  s, sdig: string;
  i: integer;
  ctx: TSHA1Context;
  d: TSHA1Digest;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  result := 0;
  sdig := SHA1Print(SHA1String('abc'));
  if sdig <> 'a9993e364706816aba3e25717850c26c9cd0d89d' then
    result := result or 1;

  sdig := SHA1Print(SHA1String('abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq'));
  if sdig <> '84983e441c3bd26ebaae4aa1f95129e5e54670f1' then
    result := result or 2;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
  code := 0;
  sdig := SHA1Print(SHA1String('abc'));
  if sdig <> 'a9993e364706816aba3e25717850c26c9cd0d89d' then
    code := code or 1;
    
  sdig := SHA1Print(SHA1String('abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq'));
  if sdig <> '84983e441c3bd26ebaae4aa1f95129e5e54670f1' then
    code := code or 2;
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
=======
>>>>>>> origin/fixes_2.4

  // SHA-1 of a million 'a' symbols
  SetLength(s, 1000);
  for i := 1 to 1000 do s[i] := 'a';
  SHA1Init(ctx);
  for i := 0 to 999 do
    SHA1Update(ctx, PChar(s)^, 1000);
  SHA1Final(ctx, d);
  sdig := SHA1Print(d);
  if sdig <> '34aa973cd4c4daa4f61eeb2bdbad27316534016f' then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    result := result or 4;
end;

var
  StartTime, EndTime: TDateTime;
  code: cardinal;
begin
  writeln('Performing basic SHA-1 test...');
  code:=performTest;
=======
    code := code or 4;
>>>>>>> graemeg/cpstrnew
=======
    code := code or 4;
>>>>>>> graemeg/cpstrnew
=======
    code := code or 4;
>>>>>>> graemeg/cpstrnew
=======
    code := code or 4;
>>>>>>> origin/cpstrnew
=======
    code := code or 4;
>>>>>>> origin/fixes_2.4

  if code = 0 then
    writeln('Basic SHA-1 tests passed')
  else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  begin
    writeln('SHA-1 tests failed: ', code);
  end;
  Halt(code);	
end.
=======
    writeln('SHA-1 tests failed: ', code);
  Halt(code);
end.
>>>>>>> graemeg/cpstrnew
=======
    writeln('SHA-1 tests failed: ', code);
  Halt(code);
end.
>>>>>>> graemeg/cpstrnew
=======
    writeln('SHA-1 tests failed: ', code);
  Halt(code);
end.
>>>>>>> graemeg/cpstrnew
=======
    writeln('SHA-1 tests failed: ', code);
  Halt(code);
end.
>>>>>>> origin/cpstrnew
=======
    writeln('SHA-1 tests failed: ', code);
  Halt(code);
end.
>>>>>>> origin/fixes_2.4
