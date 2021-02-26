program ValidateStrToInt;

{$APPTYPE CONSOLE}

{$IFDEF FPC}
  {$mode delphi}
{$ENDIF}

{$if defined(cpuarm) or defined(cpuavr) or defined(cpui8086) or defined(cpum68k) or defined(cpumips) or defined(cpuz80)}
  {$define slowcpu}
{$endif}
{$ifdef android}
  {$define slowcpu}
{$endif}

uses
  SysUtils;

const
  AllowSlow = False;

const
 VALIDATE8MIN : Integer = {$ifdef slowcpu}-20000{$else}-80000{$endif};
 VALIDATE8MAX : Integer = {$ifdef slowcpu}20000{$else}80000{$endif};
 VALIDATE9MAX : Integer = {$ifdef slowcpu}50000{$else}200000{$endif};
 VALIDATE10MAX : Integer = {$ifdef slowcpu}50000{$else}200000{$endif};
 VALIDATE13MIN : Integer = {$ifdef slowcpu}-12345{$else}-1234678{$endif};
 VALIDATE13MAX : Integer = {$ifdef slowcpu}12345{$else}123457{$endif};
 VALIDATE14MIN : Integer = {$ifdef slowcpu}-12345{$else}-1234567{$endif};
 VALIDATE14MAX : Integer = {$ifdef slowcpu}12345{$else}1234568{$endif};
 VALIDATE15MIN : Integer = {$ifdef slowcpu}-22356{$else}-2235678{$endif};
 VALIDATE15MAX : Integer = {$ifdef slowcpu}23457{$else}234578{$endif};
 VALIDATE16MIN : Integer = {$ifdef slowcpu}-11234{$else}-1123478{$endif};
 VALIDATE16MAX : Integer = 45678;
 VALIDATE29OFFSETMAX : Integer = 400000;
 VALIDATE30OFFSETMAX : Integer = 400000;
 VALIDATE39MAX : Integer = 40000;
 VALIDATE39MAXSTRLEN : Integer = 1234;
 VALIDATE40OFFSETMAX : Integer = 100000;
 VALIDATE43MAX : Integer = 70000;

procedure ErrorTrap(ValidateNo : Integer; S : AnsiString);
var
 I : Integer;
begin
 I := StrToInt(S);
 WriteLn('Function failed in Validate' + IntToStr(ValidateNo) + ' String: ' + S + 'Result: ' + IntToStr(I));
end;

//One simple testcase

function Validate1 : Boolean;
const
 VALIDATENO : Cardinal = 1;
var
 S : string;
 I : Integer;
begin
  WriteLn('Validate1');
 try
  Result := True;
  S := '92';
  I := StrToInt(S);
  if I <> 92 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//one simple testcase where StrToInt must throw an EConvertError exception

function Validate2 : Boolean;
var
 S : string;
const
 VALIDATENO : Cardinal = 2;

begin
  WriteLn('Validate2');
 Result := False;
 S := '';
 try
  StrToInt(S);
  ErrorTrap(VALIDATENO, S);
 except
  on E:EConvertError do
    Result := True;
 end;
end;

//A number of simple testcases with positive decimal numbers

function Validate3 : Boolean;
var
 S : string;
 I : Integer;
const
 VALIDATENO : Cardinal = 3;

begin
  WriteLn('Validate3');
 try
  Result := True;
  S := '1';
  I := StrToInt(S);
  if I <> 1 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '10';
  I := StrToInt(S);
  if I <> 10 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '210';
  I := StrToInt(S);
  if I <> 210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '3210';
  I := StrToInt(S);
  if I <> 3210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '43210';
  I := StrToInt(S);
  if I <> 43210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '543210';
  I := StrToInt(S);
  if I <> 543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '6543210';
  I := StrToInt(S);
  if I <> 6543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '76543210';
  I := StrToInt(S);
  if I <> 76543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '876543210';
  I := StrToInt(S);
  if I <> 876543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
   end;
  S := '2147483647';
  I := StrToInt(S);
  if I <> MaxInt then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//A number of simple testcases with negative decimal numbers

function Validate4 : Boolean;
var
 S : string;
 I : Integer;
const
 VALIDATENO : Cardinal = 4;

begin
  WriteLn('Validate4');
 try
  Result := True;
  S := '-1';
  I := StrToInt(S);
  if I <> -1 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-10';
  I := StrToInt(S);
  if I <> -10 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-210';
  I := StrToInt(S);
  if I <> -210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-3210';
  I := StrToInt(S);
  if I <> -3210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-43210';
  I := StrToInt(S);
  if I <> -43210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-543210';
  I := StrToInt(S);
  if I <> -543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-6543210';
  I := StrToInt(S);
  if I <> -6543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-76543210';
  I := StrToInt(S);
  if I <> -76543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-876543210';
  I := StrToInt(S);
  if I <> -876543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-2147483648';
  I := StrToInt(S);
  if I <> Low(I) then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Author:            Lars Bloch Gravengaard
//Date:              17/9 2006
//Validate           Hexadecimal values start with a '$'
//                   Hexadecimal values start with a '-$'
//                   Hexadecimal values start with a '0x'
//                   Hexadecimal values start with a '-0x'
//                   Hexadecimal values start with a 'x'
//                   Hexadecimal values start with a '-x'
//                   Leading blanks are ignored

function Validate5 : Boolean;
var
 S : string;
 I : Integer;
const
 VALIDATENO : Cardinal = 5;

begin
  WriteLn('Validate5');
 try
  Result := True;
  S := '$1E';
  I := StrToInt(S);
  if I <> 30 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-$1E';
  I := StrToInt(S);
  if I <> -30 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '            -10';
  I := StrToInt(S);
  if I <> -10 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-0x1E';
  I := StrToInt(S);
  if I <> -30 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '0x1E';
  I := StrToInt(S);
  if I <> 30 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-x1E';
  I := StrToInt(S);
  if I <> -30 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := 'x1E';
  I := StrToInt(S);
  if I <> 30 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := ' 3210';
  I := StrToInt(S);
  if I <> 3210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '  43210';
  I := StrToInt(S);
  if I <> 43210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '   543210';
  I := StrToInt(S);
  if I <> 543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '    6543210';
  I := StrToInt(S);
  if I <> 6543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '                                                       76543210';
  I := StrToInt(S);
  if I <> 76543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates conversion of all decimal numbers from VALIDATE8MIN to VALIDATE8MAX

function Validate8 : Boolean;
var
 S : string;
 I1, I2 : Integer;
const
 VALIDATENO : Cardinal = 8;

begin
  WriteLn('Validate8');
 try
  Result := True;
  for I1 := VALIDATE8MIN to VALIDATE8MAX do
   begin
    S := IntToStr(I1);
    I2 := StrToInt(S);
    if I2 <> I1 then
     begin
      ErrorTrap(VALIDATENO, S);
      Result := False;
      Exit;
     end;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates VALIDATE9MAX random decimal numbers from 0 to MaxInt

function Validate9 : Boolean;
var
 S : string;
 I1, I2, I3 : Integer;
const
 VALIDATENO : Cardinal = 9;

begin
  WriteLn('Validate9');
 try
  Result := True;
  for I1 := 1 to VALIDATE9MAX do
   begin
    I3 := Random(MaxInt);
    S := IntToStr(I3);
    I2 := StrToInt(S);
    if I2 <> I3 then
     begin
      ErrorTrap(VALIDATENO, S);
      Result := False;
      Exit;
     end;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates VALIDATE10MAX random decimal numbers from -MaxInt to 0

function Validate10 : Boolean;
var
 S : string;
 I1, I2, I3 : Integer;
const
 VALIDATENO : Cardinal = 10;

begin
  WriteLn('Validate10');
 try
  Result := True;
  for I1 := 1 to VALIDATE10MAX do
   begin
    I3 := -Random(MaxInt);
    S := IntToStr(I3);
    I2 := StrToInt(S);
    if I2 <> I3 then
     begin
      ErrorTrap(VALIDATENO, S);
      Result := False;
      Exit;
     end;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Test all invalid digits in leftmost digit for string lengths = 1->12

function Validate11 : Boolean;
var
 S1, S2 : string;
 I1, I2, I4 : Integer;
const
 VALIDATENO : Cardinal = 11;

begin
  WriteLn('Validate11');
 Result := False;
 S1 := '';
 for I4 := 1 to 12 do  //11 digits in '-2,147,483,648';
  begin
   for I1 := 0 to 255 do
    begin
     S2 := Char(I1) + S1;
     if not TryStrToInt(S2, I2) then
      begin
       try
        StrToInt(S2);
        ErrorTrap(VALIDATENO, S2);
        Result := False;
        Exit;
       except
        on E:EConvertError do
          Result := True;
        //All other exception types are bad
        on E:EIntOverflow do
         begin
          ErrorTrap(VALIDATENO, S2);
          Result := False;
          Exit;
         end;
       end;
      end;
    end;
   S1 := S1 + '0';
  end;
end;

//Test all valid digits in leftmost digit for string lengths = 1->12 against the RTL function

function Validate12 : Boolean;
var
 S1, S2 : string;
 I1, I2, I3, I4, Iref : Integer;
const
 VALIDATENO : Cardinal = 12;

begin
  WriteLn('Validate12');
 Result := True;
 S1 := '';
 for I4 := 1 to 12 do  //11 digits in '-2,147,483,648';
  begin
   for I1 := 0 to 255 do
    begin
     S2 := Char(I1) + S1;
     if TryStrToInt(S2, I2) then
      begin
       try
        Iref := StrToInt(S2);
        I3 := StrToInt(S2);
        if I3 <> Iref then
         begin
          ErrorTrap(VALIDATENO, S2);
          Result := False;
          Exit;
         end;
       except
        ErrorTrap(VALIDATENO, S2);
        Result := False;
       end;
      end;
    end;
   S1 := S1 + '0';
  end;
end;

//Validates conversion of all '$' prefixed hexadecimal numbers from VALIDATE13MIN VALIDATE13MAX

function Validate13 : Boolean;
var
 S : string;
 I1, I2 : Integer;
const
 VALIDATENO : Cardinal = 13;

begin
  WriteLn('Validate13');
 try
  Result := True;
  for I1 := VALIDATE13MIN to VALIDATE13MAX do
   begin
    S := '$' + IntToHex(I1,1);
    I2 := StrToInt(S);
    if I2 <> I1 then
     begin
      ErrorTrap(VALIDATENO, S);
      Result := False;
      Exit;
     end;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates conversion of all '0x' prefixed hexadecimal numbers from VALIDATE14MIN VALIDATE14MAX

function Validate14 : Boolean;
var
 S : string;
 I1, I2 : Integer;
const
 VALIDATENO : Cardinal = 14;

begin
  WriteLn('Validate14');
 try
  Result := True;
  for I1 := VALIDATE14MIN to VALIDATE14MAX do
   begin
    S := '0x' + IntToHex(I1,1);
    I2 := StrToInt(S);
    if I2 <> I1 then
     begin
      ErrorTrap(VALIDATENO, S);
      Result := False;
      Exit;
     end;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates conversion of all 'x' prefixed hexadecimal numbers from VALIDATE15MIN VALIDATE15MAX

function Validate15 : Boolean;
var
 S : string;
 I1, I2 : Integer;
const
 VALIDATENO : Cardinal = 15;

begin
  WriteLn('Validate15');
 try
  Result := True;
  for I1 := VALIDATE15MIN to VALIDATE15MAX do
   begin
    S := 'x' + IntToHex(I1,1);
    I2 := StrToInt(S);
    if I2 <> I1 then
     begin
      ErrorTrap(VALIDATENO, S);
      Result := False;
      Exit;
     end;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates conversion of all 'X' prefixed hexadecimal numbers from VALIDATE16MIN VALIDATE16MAX

function Validate16 : Boolean;
var
 S : string;
 I1, I2 : Integer;
const
 VALIDATENO : Cardinal = 16;

begin
  WriteLn('Validate16');
 try
  Result := True;
  for I1 := VALIDATE16MIN to VALIDATE16MAX do
   begin
    S := 'X' + IntToHex(I1,1);
    I2 := StrToInt(S);
    if I2 <> I1 then
     begin
      ErrorTrap(VALIDATENO, S);
      Result := False;
      Exit;
     end;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates conversion of all '$' prefixed 8 digit hexadecimal numbers
//Only two least significant digits are never changed

function Validate27 : Boolean;
var
 S : string;
 I1, I2, I3, I4, I5, I6, Iref, Ires : Integer;
const
 VALIDATENO : Cardinal = 27;
 CharArray : array[1..22] of Char = ('0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f','A','B','C','D','E','F');
begin
  WriteLn('Validate27');
 try
  Result := True;
  for I1 := Low(CharArray) to High(CharArray) do
   begin
    for I2 := Low(CharArray) to High(CharArray) do
     begin
      for I3 := Low(CharArray) to High(CharArray) do
       begin
        for I4 := Low(CharArray) to High(CharArray) do
         begin
          for I5 := Low(CharArray) to High(CharArray) do
           begin
            for I6 := Low(CharArray) to High(CharArray) do
             begin
              S := '$' + CharArray[I1] + CharArray[I2] + CharArray[I3]
                       + CharArray[I4] + CharArray[I5] + CharArray[I6]
                       + 'FF';
              Iref := StrToInt(S);
              Ires := StrToInt(S);
              if Iref <> Ires then
               begin
                ErrorTrap(VALIDATENO, S);
                Result := False;
                Exit;
               end;
             end;
           end;
         end;
       end;
     end;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates  non-nil zero length zero terminated string.

var
 GlobalStrValidate28: AnsiString;

function Validate28 : Boolean;
const
 VALIDATENO : Cardinal = 28;

var
  c: char;
begin
  WriteLn('Validate28');
 c:=#0;
 // make sure the statement after this does not write to a read-only location
 GlobalStrValidate28 := c;
 PSizeInt(Pointer(GlobalStrValidate28)-sizeof(SizeInt))^ := 0; //Set Length to 0
 try
  StrToInt(GlobalStrValidate28);
  ErrorTrap(VALIDATENO, GlobalStrValidate28);
  Result := False;
 except
  on E:EConvertError do
Result := True;
  else
   //All other exception types are bad -
   begin
    ErrorTrap(VALIDATENO, GlobalStrValidate28);
    Result := False;
    Exit;
   end;
 end;
end;

//Validates that proper exception is raised for all numbers MaxInt -> MaxInt+VALIDATE29OFFSETMAX

function Validate29 : Boolean;
var
 S : string;
 I : Integer;
 I64 : Int64;
const
 VALIDATENO : Cardinal = 29;

begin
  WriteLn('Validate29');
 Result := True;
 for I := 1 to VALIDATE29OFFSETMAX do
  begin
   try
    I64 := Int64(MaxInt) + Int64(I);
    S := IntToStr(I64);
    StrToInt(S);
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   except
    on E:EConvertError do
      Result := True;
    else
    //All other exception types are bad -
     begin
      ErrorTrap(VALIDATENO, S);
      Result := False;
      Exit;
     end;
   end;
  end;
end;

//Validates that proper exception is raised for all numbers MinInt -> MinInt-VALIDATE30OFFSETMAX

function Validate30 : Boolean;
var
 S : string;
 I : Integer;
 I64 : Int64;
const
 VALIDATENO : Cardinal = 30;

begin
  WriteLn('Validate30');
 Result := True;
 for I := 1 to VALIDATE30OFFSETMAX do
  begin
   try
    I64 := Int64(Low(Integer)) - Int64(I);
    S := IntToStr(I64);
    StrToInt(S);
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   except
    on E:EConvertError do
      Result := True;
    else
    //All other exception types are bad -
     begin
      ErrorTrap(VALIDATENO, S);
      Result := False;
      Exit;
     end;
   end;
  end;
end;

//Validates all $ prefixed hexadecimal numbers
//Adopted from Val B&V made by John O'Harrow

function Validate31 : Boolean;
var
 Lookup: array[0..$FFFF] of array[1..4] of Char; {'0000'..'FFFF'}
 I, J, Res : Integer;
 S: string;
 P1, P2: PInteger;
const
 VALIDATENO : Cardinal = 31;

 procedure SetupLookup;
 var
  I: integer;
  S: string;
 begin
  for I := 0 to $FFFF do
   begin
    S := IntToHex(I,4);
    Move(S[1], Lookup[I], 4);
   end;
 end;

begin
  WriteLn('Validate31');
 try
  Result := True;
  SetupLookup;
  S := '$00000000';
  P1 := @S[2];
  P2 := @S[6];
  for I := 0 to $FFFF do
   begin
    P1^ := Pinteger(@Lookup[I])^; {Set First 4 Chars in S}
    for J := 0 to $FFFF do
     begin
      P2^ := Pinteger(@Lookup[J])^; {Set Last 4 Chars in S}
      Res := StrToInt(S);
      if (Res <> (I shl 16) + J) then
       begin
        ErrorTrap(VALIDATENO, S);
        Result := False;
        Exit;
       end;
     end;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates all decimal numbers
//Adopted from Val B&V made by John O'Harrow

function Validate32 : Boolean;
var
 Lookup: packed array[0..9999] of array[1..4] of Char; {'0000'..'9999'}
 I, Max, Min, M, Res : Integer;
 S: string;
 P: PInteger;
const
 VALIDATENO : Cardinal = 32;

  procedure SetupLookup;
  var
   I : Integer;
   S : string;

  begin
   for I := 0 to 9999 do
    begin
     S := IntToStr(I);
     if Length(S) = 1 then
      S := '000' + S
     else
      if Length(S) = 2 then
       S := '00' + S
      else
       if Length(S) = 3 then
        S := '0' + S;
     Move(S[1], Lookup[I], 4);
    end;
  end;

begin
  WriteLn('Validate32');
 try
  Result := True;
  SetupLookup;
  I := -10000; {test -10000..+10000}
  repeat
   S := IntToStr(I);
   Res := StrToInt(S);
   if (Res <> I) then
    begin
     ErrorTrap(VALIDATENO, S);
     Result := False;
     Exit;
    end;
   Inc(I);
  until I > 10000;
  I := MaxInt; {Test Last MaxInt MOD 10000}
  repeat
   S := IntToStr(I);
   Res := StrToInt(S);
   if (Res <> I) then
    begin
     ErrorTrap(VALIDATENO, S);
     Result := False;
     Exit;
    end;
   Dec(I);
  until I mod 10000 = 0;
  S := IntToStr(I);
  Res := StrToInt(S);
  if (Res <> I) then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  Max := I;
  I := -MaxInt-1; {Test Last MinInt MOD 10000}
  repeat
   S := IntToStr(I);
   Res := StrToInt(S);
   if (Res <> I) then
    begin
     ErrorTrap(VALIDATENO, S);
     Result := False;
     Exit;
    end;
   Inc(I);
  until I mod 10000 = 0;
  S := IntToStr(I);
  Res := StrToInt(S);
  if (Res <> I) then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  Min := I;
  I := 10000;
  while I < Max do {Test All Positive Values from 10000 and up}
   begin
    S := IntToStr(I);
    P := @S[Length(S)-3];
    for M := 0 to 9999 do
     begin
      P^ := Pinteger(@Lookup[M])^; {Set Last 4 Digits in S}
      Res := StrToInt(S);
      if (Res <> I+M) then
       begin
        ErrorTrap(VALIDATENO, S);
        Result := False;
        Exit;
       end;
     end;
    Inc(I,10000);
   end;
  I := -10000;
  while I > Min do {Test All Negative Values}
   begin
    S := IntToStr(I);
    P := @S[Length(S)-3];
    for M := 0 to 9999 do
     begin
      P^ := Pinteger(@Lookup[M])^; {Set Last 4 Digits in S}
      Res := StrToInt(S);
      if (Res <> I-M) then
       begin
        ErrorTrap(VALIDATENO, S);
        Result := False;
        Exit;
       end;
     end;
    Dec(I,10000);
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates all 0x prefixed hexadecimal numbers
//Adopted from Val B&V made by John O'Harrow

function Validate33 : Boolean;
var
 Lookup: array[0..$FFFF] of array[1..4] of Char; {'0000'..'FFFF'}
 I, J, Res : Integer;
 S: string;
 P1, P2: PInteger;
const
 VALIDATENO : Cardinal = 33;

 procedure SetupLookup;
 var
  I: integer;
  S: string;
 begin
  for I := 0 to $FFFF do
   begin
    S := IntToHex(I,4);
    Move(S[1], Lookup[I], 4);
   end;
 end;

begin
  WriteLn('Validate33');
 try
  Result := True;
  SetupLookup;
  S := '0x00000000';
  P1 := @S[3];
  P2 := @S[7];
  for I := 0 to $FFFF do
   begin
    P1^ := Pinteger(@Lookup[I])^; {Set First 4 Chars in S}
    for J := 0 to $FFFF do
     begin
      P2^ := Pinteger(@Lookup[J])^; {Set Last 4 Chars in S}
      Res := StrToInt(S);
      if (Res <> (I shl 16) + J) then
       begin
        ErrorTrap(VALIDATENO, S);
        Result := False;
        Exit;
       end;
     end;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates all x prefixed hexadecimal numbers
//Adopted from Val B&V made by John O'Harrow

function Validate34 : Boolean;
var
 Lookup: array[0..$FFFF] of array[1..4] of Char; {'0000'..'FFFF'}
 I, J, Res : Integer;
 S: string;
 P1, P2: PInteger;
const
 VALIDATENO : Cardinal = 34;

 procedure SetupLookup;
 var
  I: integer;
  S: string;
 begin
  for I := 0 to $FFFF do
   begin
    S := IntToHex(I,4);
    Move(S[1], Lookup[I], 4);
   end;
 end;

begin
  WriteLn('Validate34');
 try
  Result := True;
  SetupLookup;
  S := 'x00000000';
  P1 := @S[2];
  P2 := @S[6];
  for I := 0 to $FFFF do
   begin
    P1^ := Pinteger(@Lookup[I])^; {Set First 4 Chars in S}
    for J := 0 to $FFFF do
     begin
      P2^ := Pinteger(@Lookup[J])^; {Set Last 4 Chars in S}
      Res := StrToInt(S);
      if (Res <> (I shl 16) + J) then
       begin
        ErrorTrap(VALIDATENO, S);
        Result := False;
        Exit;
       end;
     end;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates all 0X prefixed hexadecimal numbers
//Adopted from Val B&V made by John O'Harrow

function Validate35 : Boolean;
var
 Lookup: array[0..$FFFF] of array[1..4] of Char; {'0000'..'FFFF'}
 I, J, Res : Integer;
 S: string;
 P1, P2: PInteger;
const
 VALIDATENO : Cardinal = 35;

 procedure SetupLookup;
 var
  I: integer;
  S: string;
 begin
  for I := 0 to $FFFF do
   begin
    S := IntToHex(I,4);
    Move(S[1], Lookup[I], 4);
   end;
 end;

begin
  WriteLn('Validate35');
 try
  Result := True;
  SetupLookup;
  S := '0X00000000';
  P1 := @S[3];
  P2 := @S[7];
  for I := 0 to $FFFF do
   begin
    P1^ := Pinteger(@Lookup[I])^; {Set First 4 Chars in S}
    for J := 0 to $FFFF do
     begin
      P2^ := Pinteger(@Lookup[J])^; {Set Last 4 Chars in S}
      Res := StrToInt(S);
      if (Res <> (I shl 16) + J) then
       begin
        ErrorTrap(VALIDATENO, S);
        Result := False;
        Exit;
       end;
     end;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates all X prefixed hexadecimal numbers
//Adopted from Val B&V made by John O'Harrow

function Validate36 : Boolean;
var
 Lookup: array[0..$FFFF] of array[1..4] of Char; {'0000'..'FFFF'}
 I, J, Res : Integer;
 S: string;
 P1, P2: PInteger;
const
 VALIDATENO : Cardinal = 36;

 procedure SetupLookup;
 var
  I: integer;
  S: string;
 begin
  for I := 0 to $FFFF do
   begin
    S := IntToHex(I,4);
    Move(S[1], Lookup[I], 4);
   end;
 end;

begin
  WriteLn('Validate36');
 try
  Result := True;
  SetupLookup;
  S := 'X00000000';
  P1 := @S[2];
  P2 := @S[6];
  for I := 0 to $FFFF do
   begin
    P1^ := Pinteger(@Lookup[I])^; {Set First 4 Chars in S}
    for J := 0 to $FFFF do
     begin
      P2^ := Pinteger(@Lookup[J])^; {Set Last 4 Chars in S}
      Res := StrToInt(S);
      if (Res <> (I shl 16) + J) then
       begin
        ErrorTrap(VALIDATENO, S);
        Result := False;
        Exit;
       end;
     end;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

//Validates that proper exception is raised for all numbers MinInt -> MinInt-VALIDATE30OFFSETMAX

function Validate40 : Boolean;
var
 S : string;
 I, I1, I2 : Integer;
 I64 : Int64;
const
 VALIDATENO : Cardinal = 40;

begin
  WriteLn('Validate40');
 Result := True;
 for I := 1 to VALIDATE40OFFSETMAX do
  begin
   try
    I1 := Random(MaxInt);
    I2 := Random(MaxInt);
    I64 := Int64(I1) * Int64(I2);
    if I64 <= MaxInt then
     Continue;
    S := IntToStr(I64);
    StrToInt(S);
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   except
    on E:EConvertError do
      Result := True;
    else
    //All other exception types are bad
     begin
      ErrorTrap(VALIDATENO, S);
      Result := False;
      Exit;
     end;
   end;
  end;
end;

function Validate41 : Boolean;
var
 S : string;
 I1, I2, I3, I4, I5, I6 : Integer;
const
 VALIDATENO : Cardinal = 41;

begin
  WriteLn('Validate41');
 Result := True;
 for I6 := 0 to 10 do
  begin
   for I5 := 0 to 10 do
    begin
     for I4 := 0 to 10 do
      begin
       for I3 := 0 to 10 do
        begin
         S := Char(I3) + Char(I4) + Char(I5) + Char(I6);
         try
          I1 := StrToInt(S);
          try
           I2 := StrToInt(S);
           if I2 <> I1 then
            begin
             ErrorTrap(VALIDATENO, S);
             Result := False;
             Exit;
            end;
          except
           ErrorTrap(VALIDATENO, S);
           Result := False;
           Exit;
          end;
         except
          //StrToInt raised exception - so must our function - did RTL function raise E:EConvertError? Yes - otherwise it would fail this validation
          try
           StrToInt(S);
          except
           on E:EConvertError do
             Result := True;
           else
            //All other exception types are bad -
            begin
             ErrorTrap(VALIDATENO, S);
             Result := False;
             Exit;
            end;
          end;
         end;
        end;
      end;
    end;
  end;
end;

//A number of simple testcases with positive decimal numbers

function Validate42 : Boolean;
var
 S : string;
 I : Integer;
const
 VALIDATENO : Cardinal = 42;

begin
  WriteLn('Validate42');
 Result := False;
 try
   S := #0;
   I := StrToInt(S);
 except
  Result := True;
 end;
 if not Result then
   begin
     Writeln('#0 is not recognized as an invalid integer');
     halt(1);
   end;

 try
  Result := True;
  S := '0'#0;
  I := StrToInt(S);
  if I <> 0 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '10'#0;
  I := StrToInt(S);
  if I <> 10 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '210'#0;
  I := StrToInt(S);
  if I <> 210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '3210'#0'ABFG';
  I := StrToInt(S);
  if I <> 3210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '43210'#0'12';
  I := StrToInt(S);
  if I <> 43210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-543210'#0;
  I := StrToInt(S);
  if I <> -543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '6543210'#0;
  I := StrToInt(S);
  if I <> 6543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '76543210'#0;
  I := StrToInt(S);
  if I <> 76543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
    Exit;
   end;
  S := '-876543210'#0;
  I := StrToInt(S);
  if I <> -876543210 then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
   end;
  S := '2147483647'#0;
  I := StrToInt(S);
  if I <> MaxInt then
   begin
    ErrorTrap(VALIDATENO, S);
    Result := False;
   end;
 except
  ErrorTrap(VALIDATENO, S);
  Result := False;
 end;
end;

begin
  if not Validate1 then Halt(1);
  if not Validate2 then Halt(2);
  if not Validate3 then Halt(3);
  if not Validate4 then Halt(4);
  if not Validate5 then Halt(5);
  if not Validate8 then Halt(8);
  if not Validate9 then Halt(9);
  if not Validate10 then Halt(10);
  if not Validate11 then Halt(11);
  if not Validate12 then Halt(12);
  if not Validate13 then Halt(13);
  if not Validate14 then Halt(14);
  if not Validate15 then Halt(15);
  if not Validate16 then Halt(16);
  if AllowSlow then if not Validate27 then Halt(27);
  if not Validate28 then Halt(28);
  if AllowSlow then if not Validate29 then Halt(29);
  if AllowSlow then if not Validate30 then Halt(30);
  if AllowSlow then if not Validate31 then Halt(31);
  if AllowSlow then if not Validate32 then Halt(32);
  if AllowSlow then if not Validate33 then Halt(33);
  if AllowSlow then if not Validate34 then Halt(34);
  if AllowSlow then if not Validate35 then Halt(35);
  if AllowSlow then if not Validate36 then Halt(36);
  if AllowSlow then if not Validate40 then Halt(40);
  if AllowSlow then if not Validate41 then Halt(41);
  if not Validate42 then Halt(42);
  WriteLn('Done');
end.
