{$codepage utf-8}

uses
{$ifdef unix}
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef darwin}iosxwstr{$else}cwstring{$endif},
=======
  cwstring,
>>>>>>> graemeg/fixes_2_2
=======
  cwstring,
>>>>>>> origin/fixes_2_2
{$endif unix}
  sysutils;

procedure testwcmp;
var
  w1,w2: widestring;
  s: ansistring;
begin
  w1:='aécde';
  { filter unsupported characters }
  s:=w1;
  w1:=s;
  w2:=w1;
  
  if (w1<>w2) then
    halt(1);
  w1[2]:='f';
  if (w1=w2) or
     WideSameStr(w1,w2) or
     (WideCompareText(w1,w2)=0) or
     (WideCompareStr(w1,w2)<0) or
     (WideCompareStr(w2,w1)>0) then
    halt(2);
  w1[2]:=#0;
  w2[2]:=#0;
  if (w1<>w2) or
     not WideSameStr(w1,w2) or
     (WideCompareStr(w1,w2)<>0) or
     (WideCompareText(w1,w2)<>0) then
    halt(3);
  w1[3]:='m';
  if WideSameStr(w1,w2) or
     (WideCompareText(w1,w2)=0) or
     (WideCompareStr(w1,w2)<0) or
     (WideCompareStr(w2,w1)>0) then
    halt(4);
end;


begin
  testwcmp;
end.