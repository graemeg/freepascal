uses
{$ifdef unix}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef darwin}iosxwstr{$else}cwstring{$endif},
=======
  cwstring,
>>>>>>> graemeg/cpstrnew
=======
  cwstring,
>>>>>>> graemeg/cpstrnew
=======
  cwstring,
>>>>>>> graemeg/cpstrnew
=======
  cwstring,
>>>>>>> origin/cpstrnew
{$endif unix}
  SysUtils;
  
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ts1253 = type AnsiString(1253);
  ts1251 = type AnsiString(1251);
var
  s1 : ts1253;
  s2 : ts1251;
  au : unicodestring;
begin
  au := #$20AC; // Euro symbol
  s1 := au;
  s2 := au;
  if (s1<>s2) then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  ts850 = type string<850>;
  ts1251 = type string<1251>;  
var
  a850:ts850;
  a1251 : ts1251;  
  au : utf8string;
begin 
  au := #$00AE#$00A7;
  a850 := au; 
  a1251 := au;
  if (a850<>a1251) then
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
    halt(1);
  writeln('ok');
end.
