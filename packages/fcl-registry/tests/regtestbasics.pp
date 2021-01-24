unit RegTestBasics;

{$IFDEF FPC}
  {$mode objfpc}{$H+}
{$ENDIF}

interface

uses
  fpcunit, testutils, testregistry, testdecorator, Classes, SysUtils;

type

  { TTestBasics }

  TTestBasics = class(TTestCase)
  private
    procedure DeleteUserXmlFile;
  protected
  published
    procedure TestSimpleWinRegistry;
    procedure TestDoubleWrite;
    procedure bug16395;
    procedure TestStringList;
    Procedure TestInt64;
    Procedure TestDeleteSubkey;
  end;

implementation

uses
  registry;

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

procedure TTestBasics.TestSimpleWinRegistry;
var
  Registry : TRegistry;
begin
  Registry := TRegistry.Create(KEY_READ);
  Registry.RootKey:=HKEY_LOCAL_MACHINE;

  // use a hopefully non existing key
  AssertFalse(Registry.KeyExists('FPC1234'));
{$ifdef windows}
  AssertTrue(Registry.KeyExists('SOFTWARE'));
{$endif}  

  Registry.Free;
end;

procedure TTestBasics.TestDoubleWrite;
begin
  DeleteUserXmlFile;
  with TRegistry.Create do
    try
      OpenKey('FirstNode', true);
      WriteString('LAYOUT', '');
      CloseKey;
    finally
      Free;
    end;
  with TRegistry.Create do
    try
      OpenKey('FirstNode', true);
      WriteString('LAYOUT', '');
      CloseKey;
    finally
      Free;
    end;
  DeleteUserXmlFile;
end;

procedure TTestBasics.bug16395;
var
  r: TRegistry;
  s,t: string;
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

  r := TRegistry.Create;
  try
    r.RootKey := HKEY_CURRENT_USER;
    r.OpenKey('LongNode',true);
    t:=StringOfChar('*',4000);
    r.WriteString('LongString',T);
    s := r.ReadString('LongString');
    AssertEquals('Writing long string works OK', t, s);
    r.CloseKey;
  finally
    r.Free;
  end;

  DeleteUserXmlFile;
end;

Procedure TTestBasics.TestStringList;

Var
  SL : TStringList;
  I : Integer;

begin
  With TRegistry.Create do
    try
      RootKey:=HKEY_CURRENT_USER;
      OpenKey('Software/Test',True);
      SL:=TstringList.Create;
      For I:=0 to 10 do
        SL.Add(IntToStr(I));
      WriteStringList('me',SL);
      SL.Clear;
      ReadStringList('me',SL);
      For I:=0 to 10 do
        AssertEquals('String '+IntToStr(i),IntToStr(I),SL[I]);
    finally
      Free;
    end;
end;


procedure TTestBasics.TestInt64;

  

Var
  Def,I64 : Int64;
    
begin
  def:=MaxInt*1024; 
  With TRegistry.Create do
    try
      RootKey:=HKEY_CURRENT_USER;
      OpenKey('Software/Test',True);
      WriteInt64('you',Def);
      I64:=ReadInt64('you');
      AssertEquals('Value written and read',Def,I64);
    finally
      Free;
    end;  
end;

procedure TTestBasics.TestDeleteSubKey;

const
  BugID = 'Bug35132';
  Base = 'Software\' + BugID;
  One = 'One';
  OneFull = Base + '\' + One;
  Two = 'Two';
  TwoFull = OneFull + '\' + Two;


procedure CleanUp(AssertionFailed: Boolean);
var
  R: TRegistry;
  B: Boolean;
begin
  R := TRegistry.Create(KEY_ALL_ACCESS);
  try
  R.RootKey := HKEY_CURRENT_USER;
  if R.KeyExists(TwoFull) then
  begin
    B := R. DeleteKey(TwoFull);
    if B then B := R.DeleteKey(OneFull);
    if B then B := R.DeleteKey(Base);
    AssertTrue('cleanup OK',B);
  end;
  finally
    R.Free;
  end;
end;

var
  R: TRegistry;
  B: Boolean;
begin
  R := TRegistry.Create(KEY_ALL_ACCESS);
  try
    R.RootKey := HKEY_CURRENT_USER;

    B := R.OpenKey(Base, True);
    AssertTrue(format('OpenKey(''%s'') failed.',[Base]),B);

    B := R.OpenKey('One',True);
    AssertTrue(format('OpenKey(''%s'') failed.',[OneFull]),B);

    B := R.OpenKey('Two',True);
    AssertTrue(format('OpenKey(''%s'') failed.',[TwoFull]),B);

    R.CloseKey;

    B := R.KeyExists(TwoFull);
    AssertTrue(format('KeyExists(''%s'') failed.',[TwoFull]),B);

    R.CloseKey;
    B := R.OpenKey(Base,False);
    AssertTrue(format('OpenKey(''%s'') failed.',[Base]),B);

    B := R.DeleteKey('One');
    AssertTrue(format('DeleteKey(''%s'') should have succeeded, but it failed.',[OneFull]),B);
  finally
    R.Free;
    CleanUp(ExceptObject <> nil);
  end;
end;


initialization
  RegisterTest(TTestBasics);
end.

