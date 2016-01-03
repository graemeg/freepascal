{ Test for FindResourceEx function. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{%TARGET=win32,win64,linux,freebsd,darwin,netbsd,openbsd,solaris,haiku,aix,android }
=======
{%TARGET=win32,win64,linux,freebsd,darwin,netbsd,openbsd,solaris,haiku}
>>>>>>> graemeg/cpstrnew
=======
{%TARGET=win32,win64,linux,freebsd,darwin,netbsd,openbsd,solaris,haiku}
>>>>>>> graemeg/cpstrnew
=======
{%TARGET=win32,win64,linux,freebsd,darwin,netbsd,openbsd,solaris,haiku}
>>>>>>> graemeg/cpstrnew
=======
{%TARGET=win32,win64,linux,freebsd,darwin,netbsd,openbsd,solaris,haiku}
>>>>>>> origin/cpstrnew

{$mode objfpc}

uses
  sysutils;

{$R tresb.res}

procedure Fail(const Msg: string);
begin
  writeln(Msg);
  Halt(1);
end;

function GetResource(ResourceType, ResourceName: PChar; ResLang : word; PResSize: PLongInt = nil): pointer;
var
  hRes: TFPResourceHandle;
  gRes: TFPResourceHGLOBAL;
begin
  writeln('trying ',ResourceType,':',ResourceName,':',IntToHex(ResLang,4));
  hRes:=FindResourceEx(HINSTANCE, ResourceType,ResourceName,ResLang);
  if hRes = 0 then
    Fail('FindResourceEx failed.');
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
const
  LANG_ENGLISH = $09;
  SUBLANG_ENGLISH_US = $01;
  LANG_ITALIAN = $10;
  SUBLANG_ITALIAN = $01;
  SUBLANG_ITALIAN_SWISS = $02;
  LANG_GERMAN = $07;
  SUBLANG_GERMAN = $01;
var
  s: string;
  p: PChar;
  sz: longint;
begin
  //us english, exact match
  p:=GetResource('FILE','TestFile', MakeLangID(LANG_ENGLISH,SUBLANG_ENGLISH_US), @sz);
  SetString(s, p, sz);
  if s <> 'test file.' then
    Fail('Invalid resource loaded.');
  writeln(s);
  
  //italian, exact match
  p:=GetResource('FILE','TestFile', MakeLangID(LANG_ITALIAN,SUBLANG_ITALIAN), @sz);
  SetString(s, p, sz);
  if s <> 'test file (italian).' then
    Fail('Invalid resource loaded.');
  writeln(s);

  { On Windows, FindResourceEx behaviour varies between versions, so we
    can't rely on the following tests }
  {$IFNDEF WINDOWS}
  //swiss italian , should fallback to italian
  p:=GetResource('FILE','TestFile', MakeLangID(LANG_ITALIAN,SUBLANG_ITALIAN_SWISS), @sz);
  SetString(s, p, sz);
  if s <> 'test file (italian).' then
    Fail('Invalid resource loaded.');
  writeln(s);

  //german, should fallback on the first resource found (english)
  p:=GetResource('FILE','TestFile', MakeLangID(LANG_GERMAN,SUBLANG_GERMAN), @sz);
  SetString(s, p, sz);
  if s <> 'test file.' then
    Fail('Invalid resource loaded.');
  writeln(s);
  {$ENDIF}
end;

begin
  writeln('Resources test.');
  DoTest;
  writeln('Done.');
end.
