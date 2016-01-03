{ Test for resources support. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{%TARGET=win32,win64,wince,linux,freebsd,darwin,netbsd,openbsd,solaris,haiku,aix,android}

{$mode objfpc}

uses sysutils;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{%TARGET=win32,win64,wince,linux,freebsd,darwin,netbsd,openbsd,solaris,haiku}

{$mode objfpc}

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
=======
{%TARGET=win32,win64,wince,linux,freebsd,darwin,netbsd,openbsd,solaris}

{$mode objfpc}

>>>>>>> origin/fixes_2.4
{$R tres1.res}

procedure Fail(const Msg: string);
begin
  writeln(Msg);
  Halt(1);
end;

function GetResource(ResourceName, ResourceType: PChar; PResSize: PLongInt = nil): pointer;
var
  hRes: TFPResourceHandle;
  gRes: TFPResourceHGLOBAL;
begin
  hRes:=FindResource(HINSTANCE, ResourceName, ResourceType);
  if hRes = 0 then
    Fail('FindResource failed.');
  gRes:=LoadResource(HINSTANCE, hRes);
  if gRes = 0 then
    Fail('LoadResource failed.');
  if PResSize <> nil then begin
    PResSize^:=SizeofResource(HINSTANCE, hRes);
    if PResSize^ = 0 then
      Fail('SizeofResource failed.');
  end;
  Result:=LockResource(gRes);
  if Result = nil then
    Fail('LockResource failed.');
end;

procedure DoTest;
var
  s: string;
  p: PChar;
  sz: longint;
begin
  p:=GetResource('TestFile', 'FILE', @sz);
  SetString(s, p, sz);
  if s <> 'test file.' then
    Fail('Invalid resource loaded.');
  writeln(s);
  
  p:=GetResource('Test', 'TEXT', @sz);
  SetString(s, p, sz);
  if s <> 'Another test file.' then
    Fail('Invalid resource loaded.');
  writeln(s);
end;

begin
  writeln('Resources test.');
  DoTest;
  writeln('Done.');
end.
