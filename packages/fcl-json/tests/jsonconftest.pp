unit jsonconftest;

{$mode objfpc}{$H+}
<<<<<<< HEAD
<<<<<<< HEAD
{$codepage utf8}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

interface

uses
  Classes, SysUtils, fpcunit, testutils, testregistry, jsonconf;

type

  { TTestJSONConfig }

  TTestJSONConfig= class(TTestCase)
  Private
<<<<<<< HEAD
<<<<<<< HEAD
    procedure AssertStrings(Msg: String; L: TStrings;
      const Values: array of string);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Function CreateConf(AFileName : String) : TJSONCOnfig;
    Procedure DeleteConf(C : TJSONConfig; DeleteConfFile : Boolean = true);
  published
    procedure TestDataTypes;
    procedure TestSubNodes;
    procedure TestEnumSubkeys;
    procedure TestEnumValues;
    procedure TestClear;
    procedure TestKey;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure TestStrings;
    procedure TestUnicodeStrings;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

implementation

function TTestJSONConfig.CreateConf(AFileName: String): TJSONCOnfig;
begin
  Result:=TJSONConfig.Create(Nil);
  Result.FileName:=AFileName;
end;

procedure TTestJSONConfig.DeleteConf(C: TJSONConfig; DeleteConfFile: Boolean);

Var
  FN : String;

begin
  If DeleteConfFile then
    FN:=C.FileName;
  FreeAndNil(C);
  If DeleteConfFile then
    DeleteFile(FN);
end;

procedure TTestJSONConfig.TestDataTypes;

Const
<<<<<<< HEAD
<<<<<<< HEAD
  A = Integer(1);
  B = 'A string';
  C = 1.23;
  D = True;
  E = Int64($FFFFFFFFFFFFF);
=======
=======
>>>>>>> origin/fixes_2_2
  A = 1;
  B = 'A string';
  C = 1.23;
  D = True;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Var
  Co : TJSONCOnfig;

begin
  Co:=CreateConf('test.json');
  try
    Co.SetValue('a',a);
    AssertEquals('Integer read/Write',a,Co.GetValue('a',0));
    Co.SetValue('b',b);
    AssertEquals('String read/Write',b,Co.GetValue('b',''));
    Co.SetValue('c',C);
    AssertEquals('Float read/Write',c,Co.GetValue('c',0.0),0.01);
    Co.SetValue('d',d);
    AssertEquals('Boolean read/Write',d,Co.GetValue('d',False));
<<<<<<< HEAD
<<<<<<< HEAD
    Co.SetValue('e',E);
    AssertEquals('Int64 read/Write',e,Co.GetValue('e',Int64(0)));
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Co.Flush;
  finally
    DeleteConf(Co,True);
  end;
end;

procedure TTestJSONConfig.TestSubNodes;

Var
  C : TJSONCOnfig;

begin
  C:=CreateConf('test.json');
  try
    C.SetValue('a',1);
    AssertEquals('Read at root',1,C.GetValue('a',0));
    C.SetValue('b/a',2);
    AssertEquals('Read at root',2,C.GetValue('b/a',2));
    C.SetValue('b/c/a',3);
    AssertEquals('Read at root',3,C.GetValue('b/c/a',3));
  finally
    DeleteConf(C,True);
  end;
end;

procedure TTestJSONConfig.TestEnumSubkeys;
Var
  C : TJSONCOnfig;
  L : TStringList;
  
begin
  C:=CreateConf('test.json');
  try
    C.SetValue('/a',1);
    C.SetValue('/b/a',2);
    C.SetValue('/b/b',2);
    C.SetValue('/c/a',3);
    C.SetValue('/c/b/a',4);
    C.SetValue('/c/c/a',4);
    C.SetValue('/c/d/d',4);
    L:=TStringList.Create;
    try
      C.EnumSubKeys('/',L);
      If (L.Count<>2) then
        Fail('EnumSubkeys count');
      If (L[0]<>'b') then
        Fail('EnumSubkeys first element');
      If (L[1]<>'c') then
        Fail('EnumSubkeys second element');
    finally
      L.Free;
    end;
  finally
    DeleteConf(C,True);
  end;
end;

procedure TTestJSONConfig.TestEnumValues;
Var
  C : TJSONCOnfig;
  L : TStringList;

begin
  C:=CreateConf('test.json');
  try
    C.SetValue('/a',1);
    C.SetValue('/b/a',2);
    C.SetValue('/b/b',2);
    C.SetValue('/c/a',3);
    C.SetValue('/c/b/a',4);
    C.SetValue('/c/c/a',4);
    C.SetValue('/c/d/d',4);
    L:=TStringList.Create;
    try
      C.EnumValues('/',L);
      If (L.Count<>1) then
        Fail('EnumValues count');
      If (L[0]<>'a') then
        Fail('EnumValues first element');
      L.Clear;
      C.EnumValues('/b',L);
      If (L.Count<>2) then
        Fail('EnumValues subkey count');
      If (L[0]<>'a') then
        Fail('EnumValues subkey first element');
      If (L[1]<>'b') then
        Fail('EnumValues subkey second element');
    finally
      L.Free;
    end;
  finally
    DeleteConf(C,True);
  end;
end;

procedure TTestJSONConfig.TestClear;

Var
  C : TJSONCOnfig;

begin
  C:=CreateConf('test.json');
  try
    C.SetValue('a',1);
    C.DeleteValue('a');
    AssertEquals('Delete value',0,C.GetValue('a',0));
    C.SetValue('b/a',1);
    C.SetValue('b/c',2);
    C.DeleteValue('b/a');
    AssertEquals('Delete value in subkey',0,C.GetValue('a',0));
    AssertEquals('Delete value only clears deleted value',2,C.GetValue('b/c',0));
    C.SetValue('b/a',1);
    C.DeletePath('b');
    AssertEquals('Delete path',0,C.GetValue('b/a',0));
    AssertEquals('Delete path deletes all values',0,C.GetValue('b/c',0));
    C.Clear;
    AssertEquals('Clear',0,C.GetValue('/a',0));
  finally
    DeleteConf(C,True);
  end;
end;

procedure TTestJSONConfig.TestKey;

Var
  C : TJSONCOnfig;
  L : TStrings;
  
begin
  C:=CreateConf('test.json');
  try
    C.SetValue('a',1);
    C.SetValue('b/a',2);
    C.SetValue('b/b',2);
    C.SetValue('b/c/a',3);
    C.SetValue('b/c/b',3);
    C.OpenKey('/b',False);
    AssertEquals('Read relative to key a',2,C.GetValue('a',0));
    AssertEquals('Read relative to key b',2,C.GetValue('b',0));
    AssertEquals('Read in subkey relative to key a',3,C.GetValue('c/a',0));
    AssertEquals('Read in subkey relative to key b',3,C.GetValue('c/b',0));
    AssertEquals('Read absolute, disregarding key',1,C.GetValue('/a',0));
    AssertEquals('Read absolute in subkey, disregarding key',2,C.GetValue('/b/a',0));
    AssertEquals('Read absolute in subkeys, disregarding key',3,C.GetValue('/b/c/a',0));
    C.CloseKey;
    AssertEquals('Closekey',1,C.GetValue('a',0));
    C.OpenKey('b',False);
    C.OpenKey('c',False);
    AssertEquals('Open relative key',3,C.GetValue('a',0));
    C.ResetKey;
    AssertEquals('ResetKey',1,C.GetValue('a',0));
    C.Clear;
    L:=TStringList.Create;
    try
      C.EnumSubKeys('/',L);
      If (L.Count<>0) then
        Fail('clear failed');
      C.OpenKey('/a/b/c/d',true);
      C.EnumSubKeys('/a',L);
      If (L.Count<>1) then
        Fail('Open key with allowcreate, level 1');
      If (L[0]<>'b') then
        Fail('Open key with allowcreate, level 1');
      L.Clear;
      C.EnumSubKeys('/a/b',L);
      If (L.Count<>1) then
        Fail('Open key with allowcreate, level 2');
      If (L[0]<>'c') then
        Fail('Open key with allowcreate, level 2');
      L.Clear;
      C.EnumSubKeys('/a/b/c',L);
      If (L.Count<>1) then
        Fail('Open key with allowcreate, level 3');
      If (L[0]<>'d') then
        Fail('Open key with allowcreate, level 3');
    finally
      L.Free;
    end;
  finally
    DeleteConf(C,True);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TTestJSONConfig.AssertStrings(Msg: String; L: TStrings;
  const Values: array of string);

Var
  I : Integer;
begin
  Msg:=Msg+': ';
  AssertNotNull(Msg+'Have strings',L);
  AssertEquals(Msg+'Correct element count',Length(Values),L.Count);
  For I:=0 to L.Count-1 do
    AssertEquals(Msg+'element '+IntToStr(i),Values[i],l[i]);
end;

procedure TTestJSONConfig.TestStrings;

Var
  C : TJSONCOnfig;
  L,LD : TStrings;

begin
  L:=Nil;
  LD:=Nil;
  C:=CreateConf('test.json');
  try
    L:=TStringList.Create;
    LD:=TStringList.Create;
    L.Add('abc');
    C.GetValue('list',L,'');
    AssertStrings('Clear, no default.',L,[]);
    C.GetValue('list',L,'text');
    AssertStrings('Use default.',L,['text']);
    L.Clear;
    L.Add('abc');
    L.Add('def');
    C.SetValue('a',L);
    C.GetValue('a',LD,'');
    AssertStrings('List',LD,['abc','def']);
    L.Clear;
    L.Add('abc=1');
    L.Add('def=2');
    C.SetValue('a',L,True);
    LD.Clear;
    C.GetValue('a',LD,'');
    AssertStrings('List',LD,['abc=1','def=2']);
    C.SetValue('a','abc');
    C.GetValue('a',L,'');
    AssertStrings('String',L,['abc']);
    C.SetValue('a',Integer(1));
    C.GetValue('a',L,'');
    AssertStrings('integer',L,['1']);
    C.SetValue('a',True);
    C.GetValue('a',L,'');
    AssertStrings('integer',L,['True']);
    C.SetValue('a',Int64(1));
    C.GetValue('a',L,'');
    AssertStrings('int64',L,['1']);
  finally
    L.Free;
    DeleteConf(C,True);
  end;
end;

procedure TTestJSONConfig.TestUnicodeStrings;

Const
  utf8str = 'Größe ÄÜÖ ㎰ す 가';
  utf8path = 'Größe/す가';

Var
  Co : TJSONCOnfig;


begin
  Co:=CreateConf('test.json');
  try
    Co.SetValue('a',utf8str);
    Co.SetValue(utf8path,'something');
    Co.Flush;
  finally
    co.Free;
  end;
  Co:=CreateConf('test.json');
  try
    AssertEquals('UTF8 string read/Write',utf8str,utf8encode(Co.GetValue('a','')));
    AssertEquals('UTF8 path read/Write','something',Co.GetValue(utf8path,'something'));
  finally
    DeleteConf(Co,True);
  end;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

initialization

  RegisterTest(TTestJSONConfig); 
end.

