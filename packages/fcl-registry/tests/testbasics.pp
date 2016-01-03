unit TestBasics;

{$IFDEF FPC}
  {$mode objfpc}{$H+}
{$ENDIF}

interface

uses
  fpcunit, testutils, testregistry, testdecorator,
  Classes, SysUtils;

type

  { TTestBasics }

  TTestBasics = class(TTestCase)
  private
<<<<<<< HEAD
<<<<<<< HEAD
    procedure DeleteUserXmlFile;
  protected
  published
    procedure TestSimpleWinRegistry;
    procedure TestDoubleWrite;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure bug16395;
    procedure TestAdv;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
  protected
  published
    procedure TestSimpleWinRegistry;
>>>>>>> graemeg/fixes_2_2
=======
  protected
  published
    procedure TestSimpleWinRegistry;
>>>>>>> origin/fixes_2_2
  end;

implementation

uses
<<<<<<< HEAD
<<<<<<< HEAD
  registry
{$ifdef windows}
  , tregistry2
{$endif windows}
  ;

{ TTestBasics }

procedure TTestBasics.DeleteUserXmlFile;
{$ifndef windows}
var
  fn: string;
{$endif}
begin
{$ifdef windows}
  with TRegistry.Create do
    try
      DeleteKey('FirstNode');
    finally
      Free;
    end;
{$else}
  FN:=includetrailingpathdelimiter(GetAppConfigDir(False))+'reg.xml';
  if FileExists(FN) then
    AssertTrue(DeleteFile(FN));
{$endif}
end;

=======
=======
>>>>>>> origin/fixes_2_2
  registry;

{ TTestBasics }

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TTestBasics.TestSimpleWinRegistry;
var
  Registry : TRegistry;
begin
  Registry := TRegistry.Create(KEY_READ);
  Registry.RootKey:=HKEY_LOCAL_MACHINE;

  // use a hopefully non existing key
  AssertFalse(Registry.KeyExists('FPC1234'));
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef windows}
  AssertTrue(Registry.KeyExists('SOFTWARE'));
{$endif}  

  Registry.Free;
end;

procedure TTestBasics.TestDoubleWrite;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  DeleteUserXmlFile;
  with TRegistry.Create do
    try
      OpenKey('FirstNode', true);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$ifndef windows}
Var
  FN : String;
{$endif}

begin
{$ifndef windows}
  FN:=includetrailingpathdelimiter(GetAppConfigDir(False))+'reg.xml';
  if FileExists(FN) then
    AssertTrue(DeleteFile(FN));
{$endif}
  with TRegistry.Create do
    try
      OpenKey('test', true);
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
      WriteString('LAYOUT', '');
      CloseKey;
    finally
      Free;
    end;
  with TRegistry.Create do
    try
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      OpenKey('FirstNode', true);
=======
      OpenKey('test', true);
>>>>>>> graemeg/cpstrnew
=======
      OpenKey('test', true);
>>>>>>> graemeg/cpstrnew
=======
      OpenKey('test', true);
>>>>>>> graemeg/cpstrnew
=======
      OpenKey('test', true);
>>>>>>> origin/cpstrnew
      WriteString('LAYOUT', '');
      CloseKey;
    finally
      Free;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  DeleteUserXmlFile;
end;

procedure TTestBasics.bug16395;
var
  r: TRegistry;
  s: string;
begin
  DeleteUserXmlFile;
  
  r := TRegistry.Create;
  try
    r.RootKey := HKEY_CURRENT_USER;
    r.OpenKey('FirstNode', true);
    r.WriteString('string1', '');
    r.CloseKey;
  finally
    r.Free;
  end;

  // verify that empty value can be changed to non-empty one
  r := TRegistry.Create;
  try
    r.RootKey := HKEY_CURRENT_USER;
    r.OpenKey('FirstNode',false);
    s := r.ReadString('string1');
    AssertEquals('Failed to read back an empty string', '', s);
    r.WriteString('string1', 'string_value_1');
    r.CloseKey;
  finally
    r.Free;
  end;

  // verify that non-empty value can be changed to empty one
  r := TRegistry.Create;
  try
    r.RootKey := HKEY_CURRENT_USER;
    r.OpenKey('FirstNode',false);
    s := r.ReadString('string1');
    AssertEquals('Failed chaning empty string value to non-empty one', 'string_value_1',s);

    r.WriteString('string1', '');
    r.CloseKey;
  finally
    r.Free;
  end;

  r := TRegistry.Create;
  try
    r.RootKey := HKEY_CURRENT_USER;
    r.OpenKey('FirstNode',false);
    s := r.ReadString('string1');
    AssertEquals('Failed changing non-empty string value to empty one', '', s);
    r.CloseKey;
  finally
    r.Free;
  end;

  DeleteUserXmlFile;
end;

procedure TTestBasics.TestAdv;
begin
{$ifdef windows}
  DoRegTest2;
{$endif windows}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifndef windows}
  FN:=includetrailingpathdelimiter(GetAppConfigDir(False))+'reg.xml';
  if FileExists(FN) then
    AssertTrue(DeleteFile(FN));
{$endif}
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
=======
>>>>>>> origin/fixes_2_2

  AssertTrue(Registry.KeyExists('SOFTWARE'));

  // Registry.OpenKey('FPC', False);
  // Result:=Registry.ReadString('VALUE1');

  Registry.Free;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

initialization
  RegisterTest(TTestBasics);
end.
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
