<<<<<<< HEAD
<<<<<<< HEAD
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos,aix,android }
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
>>>>>>> graemeg/fixes_2_2
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
>>>>>>> origin/fixes_2_2
{ %norun }
{ %needlibrary }

library tw9089b;

{$mode objfpc}{$H+}

const
{$ifdef windows}
  libname='tw9089a.dll';
{$else}
  libname='tw9089a';
  {$linklib tw9089a}
{$endif}

var
<<<<<<< HEAD
<<<<<<< HEAD
  myvar: longint; cvar; external {$ifdef windows}libname{$endif windows};
=======
  myvar: longint; cvar; external;
>>>>>>> graemeg/fixes_2_2
=======
  myvar: longint; cvar; external;
>>>>>>> origin/fixes_2_2

function Test: Integer; cdecl; export;
begin
  Result := 0;
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
  Writeln('Test');
end;

exports
  Test;

var
  t: text;

initialization
  Writeln('INIT2');
  if (myvar<>-1) then
    halt(3);

finalization
  Writeln('FINI2');
  myvar:=1;
  { so tw9089d can check whether the finalization has run at all }
  assign(t,'tw9089b.txt');
  rewrite(t);
  close(t);
end.
