{$APPTYPE CONSOLE}
// test "fpc_AnsiStr_Concat_multi" with a differant types(encodings) 
uses
{$ifdef unix}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef darwin}iosxwstr{$else}cwstring{$endif},
{$endif unix}
  SysUtils;

const
{$ifdef android}
  cp1 = 1251;
  cp2 = 1252;
  cp3 = 65001;
{$else}
  cp1 = 866;
  cp2 = 850;
  cp3 = 1251;
{$endif}

type
  ts1 = type AnsiString(cp1);
  ts2 = type AnsiString(cp2);
  ts3 = type AnsiString(cp3);
var
  a : ts3;
  b : ts2;
  c, d : ts1;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  cwstring,
{$endif unix}
  SysUtils;

type
  ts866 = type string<866>;
  ts850 = type string<850>;
  ts1251 = type string<1251>;
var
  a : ts1251;
  b : ts850; 
  c, d : ts866;
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
begin
  a := 'al';
  b := 'b2';
  c := 'c3';
  
  //without "DestS" in the array
  d := a + b + c; 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if (StringCodePage(d) <> cp1) then
=======
  if (StringCodePage(d) <> 866) then
>>>>>>> graemeg/cpstrnew
=======
  if (StringCodePage(d) <> 866) then
>>>>>>> graemeg/cpstrnew
=======
  if (StringCodePage(d) <> 866) then
>>>>>>> graemeg/cpstrnew
=======
  if (StringCodePage(d) <> 866) then
>>>>>>> origin/cpstrnew
    halt(1);
  //with empty "DestS" in the array
  d := '';
  d := d + a + b + c; 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if (StringCodePage(d) <> cp1) then
    halt(2);
  //with "DestS" in the array at the start
  d := d + a + b + c; 
  if (StringCodePage(d) <> cp1) then
    halt(3);
  //with "DestS" in the array, not at the start 
  d := a + b + d + c; 
  if (StringCodePage(d) <> cp1) then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if (StringCodePage(d) <> 866) then
    halt(2);
  //with "DestS" in the array at the start
  d := d + a + b + c; 
  if (StringCodePage(d) <> 866) then
    halt(3);
  //with "DestS" in the array, not at the start 
  d := a + b + d + c; 
  if (StringCodePage(d) <> 866) then
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
    
  WriteLn('ok');
end.
