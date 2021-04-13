{ Program to test Code generator secondadd()                 }
{ with cardinal values                                       }
{ FUNCTIONAL PRE-REQUISITES:                                 }
{   - assignments function correctly.                        }
{   - if statements function correctly.                      }
{   - subroutine calls function correctly.                   }
{$ifdef tp}
  type cardinal = longint;
{$endif}

procedure fail;
begin
  WriteLn('Failed!');
  halt(1);
end;


procedure CardinalTestAdd;
var
 i: cardinal;
 j: cardinal;
 result : boolean;
begin
 Write('Cardinal + Cardinal test...');
 result := true;
 i:=0;
 j:=0;
 i := i + 10000;
 if i <> 10000 then
  result := false;
 j := 32767;
 i := i + j;
 if i <> 42767 then
  result := false;
 i := i + j + 50000;
 if i <> 125534 then
  result := false;
 i:=0;
 j:=10000;
 i:= i + j + j + i + j;
 if i <> 30000 then
  result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;


procedure CardinalTestSub;
var
 i, j, k : cardinal;
 result : boolean;
begin
 Write('Cardinal - Cardinal test...');
 result := true;
 i:=100000;
 j:=54;
 k:=56;
 i:= i - 100;
 if i <> 99900 then
  result := false;
 i := i - j - k - 100;
 if i <> 99690 then
  result := false;
 i:=100;
 j:=1000;
 k:=100;
 i:= j - i - k;
 if i <> 800 then
  result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;


procedure CardinalTestMul;
var
 i : cardinal;
 j : cardinal;
 k: cardinal;
 result: boolean;
begin
 Write('Cardinal * Cardinal test...');
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
 i:=$10;
 i:=i * $100010;
 if i <> $1000100 then
    result := false;
 j:=10000;
 i:=10000;
 i:=i * j;
 if i <> 100000000 then
    result := false;
 i:=1;
 j:=10;
 k:=16;
 i := i * j * k;
 if i <> 160 then
    result := false;
 i := 1;
 j := 10;
 k := 16;
 i := i * 10 * j * i * j * 16 * k;
 if i <> 256000 then
    result := false;
 i := 100000;
 j := 100;
 i := i * j;
 if i <> 10000000 then
   result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;

procedure CardinalTestXor;
var
 i, j : cardinal;
 result : boolean;
begin
 Write('Cardinal XOR Cardinal test...');
 result := true;
 i := 0;
 j := 0;
 i := i xor $1000001;
 if i <> $1000001 then
   result := false;
 i:=0;
 j:=$10000001;
 i:=i xor j;
 if i <> $10000001 then
   result := false;

 i := 0;
 j := $55555555;
 i := i xor j xor $AAAAAAAA;
 if i <> $FFFFFFFF then
   result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;


procedure CardinalTestOr;
var
 i,j : Cardinal;
 result : boolean;
Begin
 Write('Cardinal OR Cardinal test...');
 result := true;
 i := 0;
 j := 0;
 i := i or $1000001;
 if i <> $1000001 then
   result := false;
 i:=0;
 j:=$10000001;
 i:=i or j;
 if i <> $10000001 then
   result := false;

 i := 0;
 j := $55555555;
 i := i or j or $AAAAAAAA;
 if i <> $FFFFFFFF then
   result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;



procedure CardinalTestAnd;
var
 i,j : Cardinal;
 result : boolean;
Begin
 Write('Cardinal AND Cardinal test...');
 result := true;
 i := $1000001;
 j := 0;
 i := i and $1000001;
 if i <> $1000001 then
   result := false;
 i:=0;
 j:=$10000001;
 i:=i and j;
 if i <> 0 then
   result := false;

 i := $FFFFFFFF;
 j := $55555555;
 i := i and j;
 if i <> $55555555 then
   result := false;
 i := $FFFFFFFF;
 i := i and $AAAAAAAA;
 if i <> $AAAAAAAA then
   result := false;

 i := 0;
 j := $55555555;
 i := i and j and $AAAAAAAA;
 if i <> 0 then
   result := false;
 if not result then
  Fail
 else
  WriteLn('Success.');
end;

procedure CardinalTestEqual;
var
 i,j : Cardinal;
 result : boolean;
Begin
 Write('Cardinal = Cardinal test...');
 result := true;
 i := $1000001;
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


procedure CardinalTestNotEqual;
var
 i,j : Cardinal;
 result : boolean;
Begin
 Write('Cardinal <> Cardinal test...');
 result := true;
 i := $1000001;
 j := $1000001;
 if i <> $1000001 then
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

procedure CardinalTestLE;
var
 i, j: cardinal;
 result : boolean;
begin
{$ifndef tp}
 Write('Cardinal <= Cardinal test...');
 result := true;
 i := $FFFFFFFF;
 j := $FFFFFFFE;
 if i <= j then
   result := false;
 i := $FFFFFFFE;
 j := $FFFF;
 if i <= j then
   result := false;
 i := $FFFFFFFF;
 if i <= $FFFFFFFE then
    result := false;
 j := $FFFFFFFF;
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


procedure CardinalTestGE;
var
 i, j: cardinal;
 result : boolean;
begin
{$ifndef tp}
 Write('Cardinal >= Cardinal test...');
 result := true;
 i := $FFFFFFFE;
 j := $FFFFFFFF;
 if i >= j then
   result := false;
 i := $FFFFFFFE;
 j := $FFFFFFFF;
 if i > j then
   result := false;
 i := $FFFFFFFE;
 if i > $FFFFFFFE then
    result := false;
 i := $FFFFFFFF;
 j := $FFFFFFFF;
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
  CardinalTestEqual;
  CardinalTestNotEqual;
  CardinalTestAdd;
  CardinalTestMul;
  CardinalTestOr;
  CardinalTestAnd;
  CardinalTestXor;
  CardinalTestLe;
  CardinalTestGe;
  CardinalTestSub;
end.
