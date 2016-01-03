<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef unix}
uses
  {$ifdef darwin}iosxwstr{$else}cwstring{$endif};
{$endif}

type
  tcpstr1 = type AnsiString(1253);
  tcpstr2 = type AnsiString(1251);
var
  a1 : tcpstr1;
  a2 : utf8string;
  a3 : tcpstr2;
  u1 : unicodestring;
begin
  a1:=' ';
  a1[1]:=char($80); // Euro symbol in cp1253
  a2:=a1;
  if ord(a2[1])<>$E2 then
    halt(1);
  if ord(a2[2])<>$82 then
    halt(2);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$CODEPAGE cp437}
type
  tcpstr437 = string<437>;
  tcpstr850 = string<850>;
var
  a1 : tcpstr437;
  a2 : utf8string;
  a3 : tcpstr850;
  u1 : unicodestring;
begin
  a1:=#132;
  a2:=a1;
  if ord(a2[1])<>195 then
    halt(1);
  if ord(a2[2])<>164 then
    halt(1);
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

  writeln('---');

  a3:=a1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if ord(a3[1])<>$88 then
    halt(3);
=======
  if ord(a3[1])<>132 then
    halt(1);
>>>>>>> graemeg/cpstrnew
=======
  if ord(a3[1])<>132 then
    halt(1);
>>>>>>> graemeg/cpstrnew
=======
  if ord(a3[1])<>132 then
    halt(1);
>>>>>>> graemeg/cpstrnew
=======
  if ord(a3[1])<>132 then
    halt(1);
>>>>>>> origin/cpstrnew

  writeln('---');

  u1:=a1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if ord(u1[1])<>$20AC then
    halt(4);
=======
  if ord(u1[1])<>228 then
    halt(1);
>>>>>>> graemeg/cpstrnew
=======
  if ord(u1[1])<>228 then
    halt(1);
>>>>>>> graemeg/cpstrnew
=======
  if ord(u1[1])<>228 then
    halt(1);
>>>>>>> graemeg/cpstrnew
=======
  if ord(u1[1])<>228 then
    halt(1);
>>>>>>> origin/cpstrnew

  writeln('ok');
end.
