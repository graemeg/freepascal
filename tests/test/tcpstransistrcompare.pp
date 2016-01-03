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
  if Ord(s1[1]) = Ord(s2[1]) then
    halt(5);
  if (s1>s2) then
    halt(1);
  if (s1<s2) then
    halt(2);
    
  s1 := s1 + 'a';
  if (s1<=s2) then
    halt(3);
  if (s2>=s1) then
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
  if (a850>a1251) then
    halt(1);
  if (a850<a1251) then
    halt(2);
    
  a850 := a850 + 'a'; 
  if (a850<=a1251) then
    halt(3);
  if (a1251>=a850) then
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
    halt(4);
  
  writeln('ok');
end.
