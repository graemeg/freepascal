{ Test for resource enumeration functions. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{%TARGET=win32,win64,wince,linux,freebsd,darwin,netbsd,openbsd,solaris,haiku,aix,android}
=======
{%TARGET=win32,win64,wince,linux,freebsd,darwin,netbsd,openbsd,solaris,haiku}
>>>>>>> graemeg/cpstrnew
=======
{%TARGET=win32,win64,wince,linux,freebsd,darwin,netbsd,openbsd,solaris,haiku}
>>>>>>> graemeg/cpstrnew
=======
{%TARGET=win32,win64,wince,linux,freebsd,darwin,netbsd,openbsd,solaris,haiku}
>>>>>>> graemeg/cpstrnew
=======
{%TARGET=win32,win64,wince,linux,freebsd,darwin,netbsd,openbsd,solaris,haiku}
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

type
  TResInfo = record
    name : pchar;
    _type : pchar;
    langid : word;
    found : boolean;
  end;

const
  rescount = 3;

var
  reslst : array[1..rescount] of TResInfo =
  (
  (name : 'TESTFILE'; _type : 'FILE'; langid : $0409; found : false),
  (name : 'TEST'; _type : 'TEXT'; langid : $0409; found : false),
  (name : 'TESTFILE'; _type : 'FILE'; langid : $0410; found : false)
  );

function CompareDesc(d1, d2 : PChar) : boolean;
begin
  if Is_IntResource(d1) then
    Result:=PtrUInt(d1)=PtrUInt(d2)
  else
    Result:=CompareChar0(d1[0],d2[0],MaxInt)=0;
end;

procedure ResFound(ResourceType, ResourceName : PChar; IDLanguage : word);
var i : integer;
begin
  for i:=1 to rescount do
  begin
    if CompareDesc(reslst[i].name,ResourceName) and
       CompareDesc(reslst[i]._type,ResourceType) and
       (reslst[i].langid=IDLanguage) then
         if reslst[i].found then
           Fail('Resource found twice!')
         else
         begin
           reslst[i].found:=true;
           exit;
         end;
  end;
  Fail('Resource not found!');
end;

function ResLangProc(ModuleHandle : TFPResourceHMODULE; ResourceType, ResourceName : PChar; IDLanguage : word; lParam : PtrInt) : LongBool; stdcall;
begin
  writeln('        Lang: ',IntToHex(IDLanguage,4));
  Result:=true;
  ResFound(ResourceType,ResourceName,IDLanguage);
end;

function ResNameProc(ModuleHandle : TFPResourceHMODULE; ResourceType, ResourceName : PChar; lParam : PtrInt) : LongBool; stdcall;
begin
  if Is_IntResource(ResourceName) then
    writeln('    Name: ',PtrUint(ResourceName))
  else
    writeln('    Name: ',ResourceName);
  EnumResourceLanguages(ModuleHandle,ResourceType,ResourceName,@ResLangProc,lParam);
  Result:=true;
end;

function ResTypeProc(ModuleHandle : TFPResourceHMODULE; ResourceType : PChar; lParam : PtrInt) : LongBool; stdcall;
begin
  if Is_IntResource(ResourceType) then
    writeln('Type: ',PtrUint(ResourceType))
  else
    writeln('Type: ',ResourceType);
  EnumResourceNames(ModuleHandle,ResourceType,@ResNameProc,lParam);
  Result:=true;
end;

procedure CheckFound;
var i : integer;
begin
  for i:=1 to rescount do
  begin
    if not reslst[i].found then
      Fail('Resource #'+IntToStr(i)+' was not found!');
  end;
end;

procedure DoTest;
begin
  EnumResourceTypes(HINSTANCE,@ResTypeProc,0);
end;

begin
  writeln('Resources test.');
  DoTest;
  writeln('Done.');
end.
