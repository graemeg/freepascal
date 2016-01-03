
{$ifdef fpc}
{$mode objfpc}
{$endif fpc}

uses
{$ifdef unix}
<<<<<<< HEAD
<<<<<<< HEAD
  cthreads, {$ifdef darwin}iosxwstr{$else}cwstring{$endif},
=======
  cthreads, cwstring,
>>>>>>> graemeg/fixes_2_2
=======
  cthreads, cwstring,
>>>>>>> origin/fixes_2_2
{$endif}
  Classes, SysUtils;

type
  tc = class(tthread)
    orgstr: ansistring;
    cnvstr: widestring;
    constructor create(const s: ansistring; const w: widestring);
    procedure execute; override;
  end;

const
  // string with an invalid utf-8 code sequence
  str1 = #$c1#$34'Życie'#$c1#$34' jest jak papier '#$c1#$34'toaletowy'#$c1#$34' : długie, szare i '#$c1#$34'do'#$c1#$34' dupy';
  str2 = 'Życie '#$c1#$34'jest'#$c1#$34' jak papier toaletowy : '#$c1#$34'długie'#$c1#$34', szare i do '#$c1#$34'dupy'#$c1#$34'222222222222222222222222222222222222222222222222';
  str3 = 'Życie jest '#$c1#$34'jak'#$c1#$34' papier 333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333 toaletowy : długie, '#$c1#$34'szare'#$c1#$34' i do dupy';
  str4 = 'Życie jest 4444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444 jak '#$c1#$34'papier'#$c1#$34' toaletowy : długie, szare '#$c1#$34'i'#$c1#$34' do dupy';
  count = 20000;

var
  wstr: widestring;
//  cnvstr: ansistring;
  error: boolean;


constructor tc.create(const s: ansistring; const w: widestring);
begin
  orgstr:=s;
  cnvstr:=w;
  inherited create(true);
end;


procedure tc.execute;
var
  i: longint;
  w: widestring;
begin
  for i := 1 to count do
    begin
      w:=orgstr;
      if (w<>cnvstr) then
        error:=true;
    end;
end;

var
  a: array[1..4] of tc;
  w1,w2,w3,w4: widestring;
  cnvstr: ansistring;
begin
  error:=false;
  cnvstr:=str1;
  w1:=cnvstr;
  cnvstr:=str2;
  w2:=cnvstr;
  cnvstr:=str3;
  w3:=cnvstr;
  cnvstr:=str4;
  w4:=cnvstr;
  writeln(w1);
  writeln(w2);
  writeln(w3);
  writeln(w4);
  a[1]:=tc.create(str1,w1);
  a[2]:=tc.create(str2,w2);
  a[3]:=tc.create(str3,w3);
  a[4]:=tc.create(str4,w4);
  a[1].resume;
  a[2].resume;
  a[3].resume;
  a[4].resume;
  a[1].waitfor;
  a[2].waitfor;
  a[3].waitfor;
  a[4].waitfor;
  a[1].free;
  a[2].free;
  a[3].free;
  a[4].free;
  
  if error then
    halt(1);  
end.
