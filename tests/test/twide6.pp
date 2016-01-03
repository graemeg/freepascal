{%skiptarget=wince}
{$codepage utf-8}
uses
 {$ifdef unix}
<<<<<<< HEAD
<<<<<<< HEAD
 {$ifdef darwin}iosxwstr{$else}cwstring{$endif},
 {$endif}
  sysutils;

// {$define print}

=======
=======
>>>>>>> origin/fixes_2_2
 cwstring,
 {$endif}
  sysutils;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure doerror(i : integer);
  begin
    writeln('Error: ',i);
    halt(i);
  end;


<<<<<<< HEAD
<<<<<<< HEAD
{ normal upper case testing (widestring) }
procedure testupper;
var
<<<<<<< HEAD
  w1,w2: widestring;
=======
  s: ansistring;
  w1,w2,w3,w4: widestring;
  u1,u2,u3,u4: unicodestring;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  i: longint;
>>>>>>> graemeg/cpstrnew
begin
  w1:='aé'#0'èàł'#$d87e#$dc04;
  w2:='AÉ'#0'ÈÀŁ'#$d87e#$dc04;
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original upper: ',w2);
{$endif print}
  w1:=wideuppercase(w1);
{$ifdef print}
  writeln('wideupper: ',w1);
{$endif print}
  if (w1 <> w2) then
    doerror(1);

  w1:='aéèàł'#$d87e#$dc04;
  w2:='AÉÈÀŁ'#$d87e#$dc04;
  w1:=wideuppercase(w1);
{$ifdef print}
  writeln('wideupper: ',w1);
{$endif print}
  if (w1 <> w2) then
    doerror(21);
<<<<<<< HEAD
end;


=======
  if (w4 <> w2) then
    doerror(22);
end;


{ normal upper case testing (unicodestring) }
procedure testupperu;
var
  s: ansistring;
  w1,w2,w3,w4: widestring;
  u1,u2,u3,u4: unicodestring;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
{ normal upper case testing }
procedure testupper;
var
  s: ansistring;
  w1,w2,w3,w4: widestring;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  i: longint;
begin
  w1:='aé'#0'èàł'#$d87e#$dc04;
  w2:='AÉ'#0'ÈÀŁ'#$d87e#$dc04;
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original upper: ',w2);
{$endif print}
  s:=w1;
{$ifdef print}
  writeln('ansi: ',s);
{$endif print}
  w3:=s;
  w4:=AnsiUpperCase(s);
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
<<<<<<< HEAD
<<<<<<< HEAD
  w1:=unicodeuppercase(w1);
=======
  w1:=wideuppercase(w1);
>>>>>>> graemeg/fixes_2_2
{$ifdef print}
  writeln('wideupper: ',w1);
  writeln('original upper: ',w2);
  writeln('ansiupper: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(1);
  if (w4 <> w2) then
    doerror(2);

  w1:='aéèàł'#$d87e#$dc04;
  w2:='AÉÈÀŁ'#$d87e#$dc04;
  s:=w1;
  w3:=s;
  w4:=AnsiStrUpper(pchar(s));
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
<<<<<<< HEAD
  w1:=unicodeuppercase(w1);
{$ifdef print}
  writeln('unicodeupper: ',w1);
  writeln('ansistrupper: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(21);
  if (w4 <> w2) then
    doerror(22);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;


{ normal upper case testing (unicodestring) }
procedure testupperu;
var
  s: ansistring;
  w1,w2,w3,w4: widestring;
  u1,u2,u3,u4: unicodestring;
  i: longint;
begin
  w1:='aé'#0'èàł'#$d87e#$dc04;
  w2:='AÉ'#0'ÈÀŁ'#$d87e#$dc04;
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original upper: ',w2);
{$endif print}
  s:=w1;
{$ifdef print}
  writeln('ansi: ',s);
{$endif print}
  w3:=s;
  w4:=AnsiUpperCase(s);
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
  w1:=unicodeuppercase(w1);
=======
  w1:=wideuppercase(w1);
>>>>>>> origin/fixes_2_2
{$ifdef print}
  writeln('wideupper: ',w1);
  writeln('original upper: ',w2);
  writeln('ansiupper: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(1);
  if (w4 <> w2) then
    doerror(2);

  w1:='aéèàł'#$d87e#$dc04;
  w2:='AÉÈÀŁ'#$d87e#$dc04;
  s:=w1;
  w3:=s;
  w4:=AnsiStrUpper(pchar(s));
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
<<<<<<< HEAD
  w1:=unicodeuppercase(w1);
{$ifdef print}
  writeln('unicodeupper: ',w1);
=======
  w1:=wideuppercase(w1);
{$ifdef print}
  writeln('wideupper: ',w1);
>>>>>>> graemeg/fixes_2_2
=======
  w1:=wideuppercase(w1);
{$ifdef print}
  writeln('wideupper: ',w1);
>>>>>>> origin/fixes_2_2
  writeln('ansistrupper: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(21);
  if (w4 <> w2) then
    doerror(22);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
{ normal lower case testing (widestring) }
procedure testlower;
var
  w1,w2: widestring;
begin
  w1:='AÉ'#0'ÈÀŁ'#$d87e#$dc04;
  w2:='aé'#0'èàł'#$d87e#$dc04;
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original lower: ',w2);
{$endif print}
  w1:=widelowercase(w1);
{$ifdef print}
  writeln('widelower: ',w1);
{$endif print}
  if (w1 <> w2) then
    doerror(3);


  w1:='AÉÈÀŁ'#$d87e#$dc04;
  w2:='aéèàł'#$d87e#$dc04;
  w1:=widelowercase(w1);
{$ifdef print}
  writeln('widelower: ',w1);
{$endif print}
  if (w1 <> w2) then
    doerror(3);
end;


<<<<<<< HEAD
=======
{ normal lower case testing (unicodestring) }
procedure testloweru;
var
  s: ansistring;
  w1,w2,w3,w4: unicodestring;
=======
=======
>>>>>>> origin/fixes_2_2

end;


{ normal lower case testing }
procedure testlower;
var
  s: ansistring;
  w1,w2,w3,w4: widestring;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  i: longint;
begin
  w1:='AÉ'#0'ÈÀŁ'#$d87e#$dc04;
  w2:='aé'#0'èàł'#$d87e#$dc04;
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original lower: ',w2);
{$endif print}
  s:=w1;
  w3:=s;
  w4:=AnsiLowerCase(s);
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
<<<<<<< HEAD
<<<<<<< HEAD
  w1:=unicodelowercase(w1);
{$ifdef print}
  writeln('unicodelower: ',w1);
=======
  w1:=widelowercase(w1);
{$ifdef print}
  writeln('widelower: ',w1);
>>>>>>> graemeg/fixes_2_2
=======
  w1:=widelowercase(w1);
{$ifdef print}
  writeln('widelower: ',w1);
>>>>>>> origin/fixes_2_2
  writeln('ansilower: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(3);
  if (w4 <> w2) then
    doerror(4);


  w1:='AÉÈÀŁ'#$d87e#$dc04;
  w2:='aéèàł'#$d87e#$dc04;
  s:=w1;
  w3:=s;
  w4:=AnsiStrLower(pchar(s));
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
<<<<<<< HEAD
<<<<<<< HEAD
  w1:=unicodelowercase(w1);
{$ifdef print}
  writeln('unicodelower: ',w1);
=======
  w1:=widelowercase(w1);
{$ifdef print}
  writeln('widelower: ',w1);
>>>>>>> graemeg/fixes_2_2
=======
  w1:=widelowercase(w1);
{$ifdef print}
  writeln('widelower: ',w1);
>>>>>>> origin/fixes_2_2
  writeln('ansistrlower: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(3);
  if (w4 <> w2) then
    doerror(4);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======

{ normal lower case testing (unicodestring) }
procedure testloweru;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
var
  s: ansistring;
  w1,w2,w3,w4: unicodestring;
  i: longint;
begin
  w1:='AÉ'#0'ÈÀŁ'#$d87e#$dc04;
  w2:='aé'#0'èàł'#$d87e#$dc04;
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original lower: ',w2);
{$endif print}
  s:=w1;
  w3:=s;
  w4:=AnsiLowerCase(s);
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
  w1:=unicodelowercase(w1);
{$ifdef print}
  writeln('unicodelower: ',w1);
  writeln('ansilower: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(3);
  if (w4 <> w2) then
    doerror(4);


  w1:='AÉÈÀŁ'#$d87e#$dc04;
  w2:='aéèàł'#$d87e#$dc04;
  s:=w1;
  w3:=s;
  w4:=AnsiStrLower(pchar(s));
=======
=======
>>>>>>> origin/fixes_2_2


{ upper case testing with a missing utf-16 pair at the end }
procedure testupperinvalid;
var
  s: ansistring;
  w1,w2,w3,w4: widestring;
  i: longint;
begin
  { missing utf-16 pair at end }
  w1:='aé'#0'èàł'#$d87e;
  w2:='AÉ'#0'ÈÀŁ'#$d87e;
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original upper: ',w2);
{$endif print}
  s:=w1;
  w3:=s;
  w4:=AnsiUpperCase(s);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
<<<<<<< HEAD
<<<<<<< HEAD
  w1:=unicodelowercase(w1);
{$ifdef print}
  writeln('unicodelower: ',w1);
  writeln('ansistrlower: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(3);
  if (w4 <> w2) then
    doerror(4);
end;

{ upper case testing with a missing utf-16 pair at the end }
procedure testupperinvalid;
>>>>>>> origin/cpstrnew
var
  s: ansistring;
  w1,w2,w3,w4: unicodestring;
  i: longint;
begin
  w1:='AÉ'#0'ÈÀŁ'#$d87e#$dc04;
  w2:='aé'#0'èàł'#$d87e#$dc04;
=======
=======
>>>>>>> origin/fixes_2_2
  w1:=wideuppercase(w1);
{$ifdef print}
  writeln('wideupper: ',w1);
  writeln('ansiupper: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(5);
  if (w4 <> w2) then
    doerror(6);
end;


{ lower case testing with a missing utf-16 pair at the end }
procedure testlowerinvalid;
var
  s: ansistring;
  w1,w2,w3,w4: widestring;
  i: longint;
begin
  { missing utf-16 pair at end}
  w1:='AÉ'#0'ÈÀŁ'#$d87e;
  w2:='aé'#0'èàł'#$d87e;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original lower: ',w2);
{$endif print}
  s:=w1;
  w3:=s;
  w4:=AnsiLowerCase(s);
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
<<<<<<< HEAD
<<<<<<< HEAD
  w1:=unicodelowercase(w1);
{$ifdef print}
  writeln('unicodelower: ',w1);
  writeln('ansilower: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(3);
  if (w4 <> w2) then
    doerror(4);


  w1:='AÉÈÀŁ'#$d87e#$dc04;
  w2:='aéèàł'#$d87e#$dc04;
  s:=w1;
  w3:=s;
  w4:=AnsiStrLower(pchar(s));
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
  w1:=unicodelowercase(w1);
{$ifdef print}
  writeln('unicodelower: ',w1);
  writeln('ansistrlower: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(3);
  if (w4 <> w2) then
    doerror(4);
end;

{ upper case testing with a missing utf-16 pair at the end }
procedure testupperinvalid;
>>>>>>> graemeg/cpstrnew
var
  s: ansistring;
  w1,w2,w3,w4: unicodestring;
  i: longint;
begin
  w1:='AÉ'#0'ÈÀŁ'#$d87e#$dc04;
  w2:='aé'#0'èàł'#$d87e#$dc04;
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original lower: ',w2);
{$endif print}
  s:=w1;
  w3:=s;
  w4:=AnsiLowerCase(s);
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
  w1:=unicodelowercase(w1);
{$ifdef print}
  writeln('unicodelower: ',w1);
  writeln('ansilower: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(3);
  if (w4 <> w2) then
    doerror(4);


  w1:='AÉÈÀŁ'#$d87e#$dc04;
  w2:='aéèàł'#$d87e#$dc04;
  s:=w1;
  w3:=s;
  w4:=AnsiStrLower(pchar(s));
=======
=======
>>>>>>> origin/fixes_2_2
  w1:=widelowercase(w1);
{$ifdef print}
  writeln('widelower: ',w1);
  writeln('ansilower: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(7);
  if (w4 <> w2) then
    doerror(8);
end;



{ upper case testing with a missing utf-16 pair at the end, followed by a normal char }
procedure testupperinvalid1;
var
  s: ansistring;
  w1,w2,w3,w4: widestring;
  i: longint;
begin
  { missing utf-16 pair at end with char after it}
  w1:='aé'#0'èàł'#$d87e'j';
  w2:='AÉ'#0'ÈÀŁ'#$d87e'J';
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original upper: ',w2);
{$endif print}
  s:=w1;
  w3:=s;
  w4:=AnsiUpperCase(s);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
<<<<<<< HEAD
<<<<<<< HEAD
  w1:=unicodelowercase(w1);
{$ifdef print}
  writeln('unicodelower: ',w1);
  writeln('ansistrlower: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(3);
  if (w4 <> w2) then
    doerror(4);
end;

>>>>>>> graemeg/cpstrnew
{ upper case testing with a missing utf-16 pair at the end }
procedure testupperinvalid;
var
  w1,w2: widestring;
begin
  { missing utf-16 pair at end }
  w1:='aé'#0'èàł'#$d87e;
  w2:='AÉ'#0'ÈÀŁ'#$d87e;
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original upper: ',w2);
{$endif print}
  w1:=wideuppercase(w1);
{$ifdef print}
  writeln('wideupper: ',w1);
{$endif print}
  if (w1 <> w2) then
    doerror(5);
end;


{ lower case testing with a missing utf-16 pair at the end }
procedure testlowerinvalid;
var
  w1,w2: widestring;
begin
  { missing utf-16 pair at end}
  w1:='AÉ'#0'ÈÀŁ'#$d87e;
  w2:='aé'#0'èàł'#$d87e;
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original lower: ',w2);
{$endif print}
  w1:=widelowercase(w1);
{$ifdef print}
  writeln('widelower: ',w1);
{$endif print}
  if (w1 <> w2) then
    doerror(7);
end;


{ upper case testing with a missing utf-16 pair at the end, followed by a normal char }
procedure testupperinvalid1;
var
  w1,w2: widestring;
begin
  { missing utf-16 pair at end with char after it}
  w1:='aé'#0'èàł'#$d87e'j';
  w2:='AÉ'#0'ÈÀŁ'#$d87e'J';
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original upper: ',w2);
{$endif print}
  w1:=wideuppercase(w1);
{$ifdef print}
  writeln('wideupper: ',w1);
{$endif print}
  if (w1 <> w2) then
    doerror(9);
=======
=======
>>>>>>> origin/fixes_2_2
  w1:=wideuppercase(w1);
{$ifdef print}
  writeln('wideupper: ',w1);
  writeln('ansiupper: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(9);
  if (w4 <> w2) then
    doerror(10);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


{ lower case testing with a missing utf-16 pair at the end, followed by a normal char }
procedure testlowerinvalid1;
var
<<<<<<< HEAD
<<<<<<< HEAD
  w1,w2: widestring;
=======
  s: ansistring;
  w1,w2,w3,w4: widestring;
  i: longint;
>>>>>>> graemeg/fixes_2_2
=======
  s: ansistring;
  w1,w2,w3,w4: widestring;
  i: longint;
>>>>>>> origin/fixes_2_2
begin
  { missing utf-16 pair at end with char after it}
  w1:='AÉ'#0'ÈÀŁ'#$d87e'J';
  w2:='aé'#0'èàł'#$d87e'j';
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original lower: ',w2);
{$endif print}
<<<<<<< HEAD
<<<<<<< HEAD
  w1:=widelowercase(w1);
{$ifdef print}
  writeln('widelower: ',w1);
{$endif print}
  if (w1 <> w2) then
    doerror(11);
=======
=======
>>>>>>> origin/fixes_2_2
  s:=w1;
  w3:=s;
  w4:=AnsiLowerCase(s);
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
  w1:=widelowercase(w1);
{$ifdef print}
  writeln('widelower: ',w1);
  writeln('ansilower: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(11);
  if (w4 <> w2) then
    doerror(12);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


{ upper case testing with corrupting the utf-8 string after conversion }
procedure testupperinvalid2;
var
<<<<<<< HEAD
<<<<<<< HEAD
  w1,w2: widestring;
=======
  s: ansistring;
  w1,w2,w3,w4: widestring;
  i: longint;
>>>>>>> graemeg/fixes_2_2
=======
  s: ansistring;
  w1,w2,w3,w4: widestring;
  i: longint;
>>>>>>> origin/fixes_2_2
begin
  w1:='aé'#0'èàł'#$d87e#$dc04'ö';
  w2:='AÉ'#0'ÈÀŁ'#$d87e#$dc04'Ö';
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original upper: ',w2);
{$endif print}
<<<<<<< HEAD
<<<<<<< HEAD
  w1:=wideuppercase(w1);
{$ifdef print}
  writeln('wideupper: ',w1);
{$endif print}
  if (w1 <> w2) then
    doerror(13);
=======
=======
>>>>>>> origin/fixes_2_2
  s:=w1;
  { truncate the last utf-8 character }
  setlength(s,length(s)-1);
  w3:=s;
  { adjust checking values for new length due to corruption }
  if length(w3)<>length(w2) then
    begin
      setlength(w2,length(w3));
      setlength(w1,length(w3));
    end;
  w4:=AnsiUpperCase(s);
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
  w1:=wideuppercase(w1);
{$ifdef print}
  writeln('wideupper: ',w1);
  writeln('ansiupper: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(13);
  if (w4 <> w2) then
    doerror(14);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


{ lower case testing with corrupting the utf-8 string after conversion }
procedure testlowerinvalid2;
var
<<<<<<< HEAD
<<<<<<< HEAD
  w1,w2: widestring;
=======
  s: ansistring;
  w1,w2,w3,w4: widestring;
  i: longint;
>>>>>>> graemeg/fixes_2_2
=======
  s: ansistring;
  w1,w2,w3,w4: widestring;
  i: longint;
>>>>>>> origin/fixes_2_2
begin
  w1:='AÉ'#0'ÈÀŁ'#$d87e#$dc04'Ö';
  w2:='aé'#0'èàł'#$d87e#$dc04'ö';
{$ifdef print}
// the utf-8 output can confuse the testsuite parser
  writeln('original: ',w1);
  writeln('original lower: ',w2);
{$endif print}
<<<<<<< HEAD
<<<<<<< HEAD
  w1:=widelowercase(w1);
{$ifdef print}
  writeln('widelower: ',w1);
{$endif print}
  if (w1 <> w2) then
    doerror(15);
end;


begin
  testupper;
  writeln;
  testupperu;
  writeln;
  testlower;
  writeln;
  testloweru;
  writeln;
=======
=======
>>>>>>> origin/fixes_2_2
  s:=w1;
  { truncate the last utf-8 character }
  setlength(s,length(s)-1);
  w3:=s;
  { adjust checking values for new length due to corruption }
  if length(w3)<>length(w2) then
    begin
      setlength(w2,length(w3));
      setlength(w1,length(w3));
    end;
  w4:=AnsiLowerCase(s);
  { filter out unsupported characters }
  for i:=1 to length(w3) do
    if w3[i]='?' then
      begin
        w2[i]:='?';
        w1[i]:='?';
      end;
  w1:=widelowercase(w1);
{$ifdef print}
  writeln('widelower: ',w1);
  writeln('ansilower: ',w4);
{$endif print}
  if (w1 <> w2) then
    doerror(15);
  if (w4 <> w2) then
    doerror(16);
end;



begin
  testupper;
  writeln;
  testlower;
  writeln;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  writeln;
  testupperinvalid;
  writeln;
  testlowerinvalid;
  writeln;
  writeln;
  testupperinvalid1;
  writeln;
  testlowerinvalid1;
  writeln;
  writeln;
  testupperinvalid2;
  writeln;
  testlowerinvalid2;
end.
