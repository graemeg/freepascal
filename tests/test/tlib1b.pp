<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %skiptarget=go32v2,os2 }
=======
{ %skiptarget=go32v2 }
>>>>>>> graemeg/cpstrnew
=======
{ %skiptarget=go32v2 }
>>>>>>> graemeg/cpstrnew
=======
{ %skiptarget=go32v2 }
>>>>>>> graemeg/cpstrnew
=======
{ %skiptarget=go32v2 }
>>>>>>> origin/cpstrnew
{ %opt=-gl -O- }
{ %needlibrary }
{ %delfiles=tlib1a }
{$goto on}

{ test lineinfo in libraries, to get always proper results, the optimizer
  must be turned off with -O- }
uses
  {$ifdef unix}dl,{$endif unix}sysutils;

{$ifdef darwin}
{$linklib tlib1a}
{$endif darwin}

procedure p(var a : pointer);external 'tlib1a' name 'p';

var
  s1,s2 : string;
  l : longint;
  a : pointer;
label
  w;
begin
w:
  { library }
  p(a);
  GetLineInfo(PtrUInt(a),s1,s2,l);
  s2:=ExtractFilename(s2);
  writeln( 'Func: ',s1,' Source: ',s2,' Line: ',l);
  { GetLineInfo of dwarf doesn't return the function name }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  { gdb reports line number 12 instead of 10 for the label
    (the real one is 11; lineinfo matches it with the previous
     line, gdb with the next one) }
  if { (s1<>'P') or } (s2<>'tlib1a.pp') or ((l<>10) and (l<>12)) then
=======
  if { (s1<>'P') or } (s2<>'tlib1a.pp') or (l<>10) then
>>>>>>> graemeg/cpstrnew
=======
  if { (s1<>'P') or } (s2<>'tlib1a.pp') or (l<>10) then
>>>>>>> graemeg/cpstrnew
=======
  if { (s1<>'P') or } (s2<>'tlib1a.pp') or (l<>10) then
>>>>>>> graemeg/cpstrnew
=======
  if { (s1<>'P') or } (s2<>'tlib1a.pp') or (l<>10) then
>>>>>>> origin/cpstrnew
    halt(1);

  { main program }
  GetLineInfo(PtrUInt(@w),s1,s2,l);
  s2:=ExtractFilename(s2);
  writeln( 'Func: ',s1,' Source: ',s2,' Line: ',l);
  { GetLineInfo of dwarf doesn't return the function name }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if { (s1<>'P') or } (s2<>'tlib1b.pp') or ((l<>24) and (l<>27)) then
=======
  if { (s1<>'P') or } (s2<>'tlib1b.pp') or (l<>24) then
>>>>>>> graemeg/cpstrnew
=======
  if { (s1<>'P') or } (s2<>'tlib1b.pp') or (l<>24) then
>>>>>>> graemeg/cpstrnew
=======
  if { (s1<>'P') or } (s2<>'tlib1b.pp') or (l<>24) then
>>>>>>> graemeg/cpstrnew
=======
  if { (s1<>'P') or } (s2<>'tlib1b.pp') or (l<>24) then
>>>>>>> origin/cpstrnew
    halt(1);

  writeln('ok');
end.
