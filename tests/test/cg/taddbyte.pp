{ Program to test Code generator secondadd()                 }
{ with Byte values                                       }
{ FUNCTIONAL PRE-REQUISITES:                                 }
{   - assignments function correctly.                        }
{   - if statements function correctly.                      }
{   - subroutine calls function correctly.                   }

procedure fail;
begin
  WriteLn('Failed!');
  halt(1);
end;


procedure ByteTestAdd;
var
 i: Byte;
 j: Byte;
 result : boolean;
begin
 Write('Byte + Byte test...');
 result := true;
 i:=0;
 j:=0;
 i := i + 100;
 if i <> 100 then
  result := false;
 j := 32;
 i := i + j;
 if i <> 132 then
  result := false;
 i := i + j + 50;
 if i <> 214 then
  result := false;
 i:=0;
 j:=10;
 i:= i + j + j + i + j;
 if i <> 30 then
  result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;


procedure ByteTestSub;
var
 i, j, k : Byte;
 result : boolean;
begin
 Write('Byte - Byte test...');
 result := true;
 i:=100;
 j:=4;
 k:=6;
 i:= i - 10;
 if i <> 90 then
  result := false;
 i := i - j - k - 11;
 if i <> 69 then
  result := false;
 i:=10;
 j:=100;
 k:=10;
 i:= j - i - k;
 if i <> 80 then
  result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;


procedure ByteTestMul;
var
 i : Byte;
 j : Byte;
 k: Byte;
 result: boolean;
begin
 Write('Byte * Byte test...');
 result := true;
 i:=0;
 j:=0;
 i:=i * 32;
 if i <> 0 then
   result := false;
 i:=10;
 i:=i * 16;
 if i <> 160 then
    result := false;
 j:=10;
 i:=10;
 i:=i * j;
 if i <> 100 then
    result := false;
 i:=1;
 j:=10;
 k:=16;
 i := i * j * k;
 if i <> 160 then
    result := false;
 i := 1;
 j := 2;
 k := 4;
 i := i * 2 * j * i * j * 4 * k;
 if i <> 128 then
    result := false;
 i := 50;
 j := 5;
 i := i * j;
 if i <> 250 then
   result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;

procedure ByteTestXor;
var
 i, j : Byte;
 result : boolean;
begin
 Write('Byte XOR Byte test...');
 result := true;
 i := 0;
 j := 0;
 i := i xor $11;
 if i <> $11 then
   result := false;
 i:=0;
 j:=$11;
 i:=i xor j;
 if i <> $11 then
   result := false;

 i := 0;
 j := $55;
 i := i xor j xor $AA;
 if i <> $FF then
   result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;


procedure ByteTestOr;
var
 i,j : Byte;
 result : boolean;
Begin
 Write('Byte OR Byte test...');
 result := true;
 i := 0;
 j := 0;
 i := i or $11;
 if i <> $11 then
   result := false;
 i:=0;
 j:=$11;
 i:=i or j;
 if i <> $11 then
   result := false;

 i := 0;
 j := $55;
 i := i or j or $AA;
 if i <> $FF then
   result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;



procedure ByteTestAnd;
var
 i,j : Byte;
 result : boolean;
Begin
 Write('Byte AND Byte test...');
 result := true;
 i := $11;
 j := 0;
 i := i and $11;
 if i <> $11 then
   result := false;
 i:=0;
 j:=$11;
 i:=i and j;
 if i <> 0 then
   result := false;

 i := $FF;
 j := $55;
 i := i and j;
 if i <> $55 then
   result := false;
 i := $FF;
 i := i and $AA;
 if i <> $AA then
   result := false;

 i := 0;
 j := $55;
 i := i and j and $AA;
 if i <> 0 then
   result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;

procedure ByteTestEqual;
var
 i,j : Byte;
 result : boolean;
Begin
 Write('Byte = Byte test...');
 result := true;
 i := $11;
 j := 0;
 if i = 0 then
   result := false;
 if i = j then
  result := false;
 if j = i then
  result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;


procedure ByteTestNotEqual;
var
 i,j : Byte;
 result : boolean;
Begin
 Write('Byte <> Byte test...');
 result := true;
 i := $11;
 j := $11;
 if i <> $11 then
   result := false;
 if i <> j then
  result := false;
 if j <> i then
  result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;

procedure ByteTestLE;
var
 i, j: Byte;
 result : boolean;
begin
{$ifndef tp}
 Write('Byte <= Byte test...');
 result := true;
 i := $FF;
 j := $FE;
 if i <= j then
   result := false;
 i := $FE;
 j := $F;
 if i <= j then
   result := false;
 i := $FF;
 if i <= $FE then
    result := false;
 j := $FF;
 if i <= j then
  begin
    if result then
      WriteLn('Success.')
    else
      Fail;
  end
 else
  Fail;
{$endif}
end;


procedure ByteTestGE;
var
 i, j: Byte;
 result : boolean;
begin
{$ifndef tp}
 Write('Byte >= Byte test...');
 result := true;
 i := $FE;
 j := $FF;
 if i >= j then
   result := false;
 i := $FE;
 j := $FF;
 if i > j then
   result := false;
 i := $FE;
 if i > $FE then
    result := false;
 i := $FF;
 j := $FF;
 if i >= j then
  begin
    if result then
      WriteLn('Success.')
    else
      Fail;
  end
 else
  Fail;
{$endif}
end;


Begin
  { These should be tested first, since if they do not }
  { work, they will false all other results.           }
  ByteTestEqual;
  ByteTestNotEqual;
  ByteTestAdd;
  ByteTestMul;
  ByteTestOr;
  ByteTestAnd;
  ByteTestXor;
  ByteTestLe;
  ByteTestGe;
  ByteTestSub;
end.
