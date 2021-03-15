{
    This file is part of the Free Component Library

    JSON FPCUNit test for data structures
    Copyright (c) 2007 by Michael Van Canneyt michael@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}
unit testjsondata; 

interface

uses
  Classes, SysUtils, fpcunit, testregistry, fpjson;

type
   TMyNull     = Class(TJSONNull);
   TMyInteger  = Class(TJSONIntegerNumber);
   TMyInt64    = Class(TJSONInt64Number);
   TMyQWord    = Class(TJSONQWordNumber);
   TMyFloat    = Class(TJSONFloatNumber);
   TMyString   = Class(TJSONString);
   TMyBoolean  = Class(TJSONBoolean);
   TMyArray    = Class(TJSONArray);
   TMyObject   = Class(TJSONObject);

  { TTestJSONString }

  TTestJSONString = Class(TTestCase)
  Private
    Procedure TestTo(Const Src,Dest : String; Strict : Boolean = False);
    Procedure TestFrom(Const Src,Dest : String);
  Published
    Procedure TestJSONStringToString;
    Procedure TestStringToJSONString;
  end;
  
  { TTestJSON }
  
  TTestJSON = Class(TTestCase)
  private
  Protected
    procedure SetDefaultInstanceTypes;
    procedure SetMyInstanceTypes;
    Procedure SetUp; override;
    Procedure TestItemCount(J : TJSONData;Expected : Integer);
    Procedure TestJSONType(J : TJSONData;Expected : TJSONType);
    Procedure TestJSON(J : TJSONData;Expected : TJSONStringType);
    Procedure TestIsNull(J : TJSONData;Expected : Boolean);
    Procedure TestAsBoolean(J : TJSONData;Expected : Boolean; ExpectError : boolean = False);
    Procedure TestAsInteger(J : TJSONData; Expected : Integer; ExpectError : boolean = False);
    Procedure TestAsInt64(J : TJSONData; Expected : Int64; ExpectError : boolean = False);
    Procedure TestAsQWord(J : TJSONData; Expected : QWord; ExpectError : boolean = False);
    Procedure TestAsString(J : TJSONData; Expected : String; ExpectError : boolean = False);
    Procedure TestAsFloat(J : TJSONData; Expected : TJSONFloat; ExpectError : boolean = False);
  end;
  
  { TTestNull }

  TTestNull = class(TTestJSON)
  published
    procedure TestNull;
    Procedure TestClone;
    Procedure TestMyClone;
    Procedure TestFormat;
  end;
  
  { TTestBoolean }

  TTestBoolean = class(TTestJSON)
  published
    procedure TestTrue;
    procedure TestFalse;
    Procedure TestClone;
    Procedure TestMyClone;
    Procedure TestFormat;
  end;
  
  { TTestInteger }

  TTestInteger = class(TTestJSON)
  Private
    Procedure DoTest(I : Integer);
  published
    procedure TestPositive;
    procedure TestNegative;
    procedure TestZero;
    Procedure TestClone;
    Procedure TestMyClone;
    Procedure TestFormat;
  end;

  { TTestInt64 }

  TTestInt64 = class(TTestJSON)
  Private
    Procedure DoTest(I : Int64);
  published
    procedure TestPositive;
    procedure TestNegative;
    procedure TestZero;
    Procedure TestClone;
    Procedure TestMyClone;
    Procedure TestFormat;
  end;

  { TTestQword }

  TTestQword = class(TTestJSON)
  Private
    Procedure DoTest(Q : QWord);
  published
    procedure TestPositive;
    procedure TestZero;
    Procedure TestClone;
    Procedure TestMyClone;
    Procedure TestFormat;
  end;

  { TTestFloat }

  TTestFloat = class(TTestJSON)
  Private
    Procedure DoTest(F : TJSONFloat);
  published
    procedure TestPositive;
    procedure TestNegative;
    procedure TestZero;
    Procedure TestClone;
    Procedure TestMyClone;
    Procedure TestFormat;
  end;

  { TTestString }

  TTestString = class(TTestJSON)
  private
    procedure DoTestFloat(F: TJSOnFloat; S: String; OK: Boolean);
  published
    procedure TestString;
    procedure TestControlString;
    procedure TestSolidus;
    procedure TestInteger;
    procedure TestNegativeInteger;
    procedure TestFloat;
    procedure TestNegativeFloat;
    Procedure TestBooleanTrue;
    Procedure TestBooleanFalse;
    Procedure TestClone;
    Procedure TestMyClone;
    Procedure TestFormat;
  end;
  
  { TTestArray }

  TTestArray = class(TTestJSON)
  private
    procedure TestAddBoolean(B : Boolean);
    procedure TestInsertBoolean(B : Boolean);
  published
    Procedure TestCreate;
    Procedure TestCreateString;
    Procedure TestCreatePchar;
    procedure TestCreateStrings;
    procedure TestCreateStringsCompressed;
    procedure TestCreateInteger;
    procedure TestCreateInt64;
    procedure TestCreateFloat;
    procedure TestCreateBoolean;
    procedure TestCreateObject;
    procedure TestCreateJSONString;
    procedure TestCreateJSONObject;
    procedure TestCreateNilPointer;
    procedure TestCreatePointer;
    procedure TestAddInteger;
    procedure TestAddInt64;
    procedure TestAddFloat;
    procedure TestAddBooleanTrue;
    procedure TestAddBooleanFalse;
    procedure TestAddString;
    procedure TestAddNull;
    procedure TestAddObject;
    procedure TestAddArray;
    procedure TestInsertInteger;
    procedure TestInsertInt64;
    procedure TestInsertFloat;
    procedure TestInsertBooleanTrue;
    procedure TestInsertBooleanFalse;
    procedure TestInsertString;
    procedure TestInsertNull;
    procedure TestInsertObject;
    procedure TestInsertArray;
    procedure TestMove;
    procedure TestExchange;
    procedure TestDelete;
    procedure TestRemove;
    Procedure TestClone;
    Procedure TestMyClone;
    Procedure TestFormat;
    Procedure TestFormatNil;
  end;
  
  { TTestObject }

  TTestObject = class(TTestJSON)
  private
    FJ: TJSONObject;
    procedure AppendA;
  protected
    Procedure Setup; override;
    Procedure TearDown; override;
    procedure TestAddBoolean(B : Boolean);
    Procedure TestAccessError;
    Property J : TJSONObject Read FJ;
  published
    Procedure TestCreate;
    Procedure TestCreateString;
    Procedure TestCreateStringUnquoted;
    Procedure TestCreatePchar;
    Procedure TestCreatePcharUnquoted;
    procedure TestCreateStrings;
    procedure TestCreateStringsCompressed;
    procedure TestCreateStringsCompressedUnquoted;
    procedure TestCreateInteger;
    procedure TestCreateIntegerUnquoted;
    procedure TestCreateInt64;
    procedure TestCreateInt64Unquoted;
    procedure TestCreateFloat;
    procedure TestCreateFloatUnquoted;
    procedure TestCreateBoolean;
    procedure TestCreateBooleanUnquoted;
    procedure TestCreateObject;
    procedure TestCreateJSONUnicodeString;
    procedure TestCreateJSONWideString;
    procedure TestCreateJSONString;
    procedure TestCreateJSONStringUnquoted;
    procedure TestCreateJSONObject;
    procedure TestCreateJSONObjectUnquoted;
    procedure TestCreateNilPointer;
    procedure TestCreatePointer;
    procedure TestAddInteger;
    procedure TestAddInt64;
    procedure TestAddFloat;
    procedure TestAddBooleanTrue;
    procedure TestAddBooleanFalse;
    procedure TestAddString;
    procedure TestAddNull;
    procedure TestAddObject;
    procedure TestAddArray;
    procedure TestDelete;
    procedure TestRemove;
    procedure TestClone;
    procedure TestMyClone;
    procedure TestExtract;
    Procedure TestNonExistingAccessError;
    Procedure TestFormat;
    Procedure TestFormatNil;
    Procedure TestFind;
    Procedure TestIfFind;
    Procedure TestDuplicate;
  end;

  { TTestJSONPath }

  TTestJSONPath = class(TTestJSON)
  private
    FData: TJSONData;
  Protected
    Procedure TearDown; override;
    Property Data : TJSONData read FData Write FData;
  Published
    Procedure TestNullEmpty;
    Procedure TestNullGet;
    Procedure TestNullNonExisting;
    Procedure TestNullNotEmpty;
    Procedure TestBooleanEmpty;
    Procedure TestBooleanNotEmpty;
    Procedure TestIntegerEmpty;
    Procedure TestIntegerNotEmpty;
    Procedure TestInt64Empty;
    Procedure TestInt64NotEmpty;
    Procedure TestFloatEmpty;
    Procedure TestFloatNotEmpty;
    Procedure TestStringEmpty;
    Procedure TestStringNotEmpty;
    Procedure TestArrayEmpty;
    Procedure TestArrayNotIndex;
    Procedure TestArrayIncompleteIndex;
    Procedure TestArrayNonNumericalIndex;
    Procedure TestArrayOutOfRangeIndex;
    Procedure TestArrayCorrectIndex;
    Procedure TestArrayRecursiveArray;
    Procedure TestArrayRecursiveObject;
    Procedure TestObjectEmpty;
    Procedure TestObjectDots;
    Procedure TestObjectExisting;
    Procedure TestObjectNonExisting;
    Procedure TestObjectTrailingDot;
    Procedure TestObjectRecursiveArray;
    Procedure TestObjectRecursiveObject;
    Procedure TestDeepRecursive;
  end;

  { TTestFactory }

  TTestFactory = class(TTestJSON)
  Private
    FType : TJSONInstanceType;
    FClass : TJSONDataClass;
    FData: TJSONData;
  Protected
    Procedure DoSet;
    Procedure TearDown; override;
    Procedure AssertElement0(AClass : TJSONDataClass);
    Procedure AssertElementA(AClass : TJSONDataClass);
    Property Data : TJSONData read FData Write FData;
  Published
    Procedure TestSet;
    Procedure TestSetInvalid;
    Procedure CreateNull;
    Procedure CreateInteger;
    Procedure CreateInt64;
    Procedure CreateFloat;
    Procedure CreateBoolean;
    Procedure CreateString;
    Procedure CreateArray;
    Procedure CreateObject;
    Procedure ArrayAddNull;
    Procedure ArrayAddInteger;
    Procedure ArrayAddInt64;
    Procedure ArrayAddFloat;
    Procedure ArrayAddBoolean;
    Procedure ArrayAddString;
    Procedure ArrayCreateNull;
    Procedure ArrayCreateInteger;
    Procedure ArrayCreateInt64;
    Procedure ArrayCreateFloat;
    Procedure ArrayCreateBoolean;
    Procedure ArrayCreateString;
    Procedure ObjectAddNull;
    Procedure ObjectAddInteger;
    Procedure ObjectAddInt64;
    Procedure ObjectAddFloat;
    Procedure ObjectAddBoolean;
    Procedure ObjectAddString;
    Procedure ObjectCreateNull;
    Procedure ObjectCreateInteger;
    Procedure ObjectCreateInt64;
    Procedure ObjectCreateFloat;
    Procedure ObjectCreateBoolean;
    Procedure ObjectCreateString;
  end;

  { TTestIterator }

  TTestIterator = class(TTestJSON)
  private
    FData: TJSONData;
  Protected
    Procedure TearDown; override;
    Procedure TestSingle;
    Procedure TestLoop(ACount : Integer);
    Property Data : TJSONData Read FData Write FData;
  Published
    Procedure TestNull;
    Procedure TestInteger;
    Procedure TestInt64;
    Procedure TestFloat;
    Procedure TestBoolean;
    Procedure TestString;
    Procedure TestArray;
    Procedure TestObject;
  end;


implementation

{ TTestIterator }

procedure TTestIterator.TearDown;
begin
  FreeAndNil(FData);
  inherited TearDown;
end;

procedure TTestIterator.TestSingle;

Var
  F : TJSONEnum;
  C : Integer;

begin
  C:=0;
  For F in Data do
   begin
   Inc(C);
   If C>1 then
     Fail(Data.ClassName+' loops more than once');
   AssertEquals(Data.ClassName+' has empty key','',F.Key);
   AssertEquals(Data.ClassName+' has empty numerical key',0,F.KeyNum);
   AssertSame(Data.ClassName+' returns data',Data,F.Value);
   end;
  If C<1 then
    Fail(Data.ClassName+' Loops not even once');
end;

procedure TTestIterator.TestLoop(ACount: Integer);
Var
  F : TJSONEnum;
  C : Integer;

begin
  C:=0;
  For F in Data do
   begin
   AssertEquals(Data.ClassName+' has correct string key',IntToStr(C),F.Key);
   AssertEquals(Data.ClassName+' has correct numerical key',C,F.KeyNum);
   AssertSame(Data.ClassName+' returns correct data',Data.Items[C],F.Value);
   Inc(C);
   end;
  AssertEquals(Data.ClassName+' correct loop count',ACount,C);
end;

procedure TTestIterator.TestNull;
begin
  Data:=TJSONNull.Create;
  TestSingle;
end;

procedure TTestIterator.TestInteger;
begin
  Data:=TJSONIntegerNumber.Create(1);
  TestSingle;
end;

procedure TTestIterator.TestInt64;
begin
  Data:=TJSONInt64Number.Create(1);
  TestSingle;
end;

procedure TTestIterator.TestFloat;
begin
  Data:=TJSONFloatNumber.Create(1.2);
  TestSingle;
end;

procedure TTestIterator.TestBoolean;
begin
  Data:=TJSONBoolean.Create(True);
  TestSingle;
end;

procedure TTestIterator.TestString;
begin
  Data:=TJSONString.Create('Data');
  TestSingle;
end;

procedure TTestIterator.TestArray;
begin
  Data:=TJSONArray.Create([1,2,3]);
  TestLoop(3);
end;

procedure TTestIterator.TestObject;
begin
  Data:=TJSONObject.Create(['0',1,'1',2,'2',3]);
  TestLoop(3);
end;

{ TTestFactory }

procedure TTestFactory.DoSet;
begin
  SetJSONInstanceType(FType,FClass);
end;

procedure TTestFactory.TearDown;
begin
  FreeAndNil(FData);
  inherited TearDown;
end;

procedure TTestFactory.AssertElement0(AClass: TJSONDataClass);
begin
  AssertEquals('Correct class',TMyArray,Data.ClassType);
  AssertEquals('Have 1 element',1,Data.Count);
  AssertEquals('Correct class',AClass,(Data as TJSONArray)[0].ClassType);
end;

procedure TTestFactory.AssertElementA(AClass: TJSONDataClass);
begin
  AssertEquals('Correct class',TMyObject,Data.ClassType);
  AssertEquals('Have element a',0,TMyObject(Data).IndexOfName('a'));
  AssertEquals('Correct class',AClass,(Data as TJSONObject).Elements['a'].ClassType);
end;

procedure TTestFactory.TestSet;
begin
  SetMyInstanceTypes;
  AssertEquals('Correct type for unknown',TJSONData,GetJSONInstanceType(jitUnknown));
  AssertEquals('Correct type for integer',TMyInteger,GetJSONInstanceType(jitNumberInteger));
  AssertEquals('Correct type for int64',TMyInt64,GetJSONInstanceType(jitNumberInt64));
  AssertEquals('Correct type for float',TMyFloat,GetJSONInstanceType(jitNumberFloat));
  AssertEquals('Correct type for boolean',TMyBoolean,GetJSONInstanceType(jitBoolean));
  AssertEquals('Correct type for null',TMyNull,GetJSONInstanceType(jitNUll));
  AssertEquals('Correct type for String',TMyString,GetJSONInstanceType(jitString));
  AssertEquals('Correct type for Array',TMyArray,GetJSONInstanceType(jitArray));
  AssertEquals('Correct type for Object',TMyObject,GetJSONInstanceType(jitObject));
end;

procedure TTestFactory.TestSetInvalid;

Const
  MyJSONInstanceTypes :
    Array [TJSONInstanceType] of TJSONDataClass = (TJSONData, TMyInteger,
    TMyInt64,TMyQWord,TMyFloat, TMyString, TMyBoolean, TMyNull, TMyArray,
    TMyObject);

Var
  Ti : TJSONInstanceType;

begin
  For ti:=Succ(Low(TJSONInstanceType)) to High(TJSONInstanceType) do
    begin
    FType:=Ti;
    FClass:=MyJSONInstanceTypes[Pred(ti)];
    AssertException('Set '+FClass.ClassName,EJSON,@DoSet);
    end;
  FType:=jitString;
  FClass:=Nil;
  AssertException('Set Nil',EJSON,@DoSet);
end;

procedure TTestFactory.CreateNull;
begin
  SetMyInstanceTypes;
  Data:=CreateJSON;
  AssertEquals('Correct class',TMyNull,Data.ClassType);
end;

procedure TTestFactory.CreateInteger;
begin
  SetMyInstanceTypes;
  Data:=CreateJSON(1);
  AssertEquals('Correct class',TMyInteger,Data.ClassType);
end;

procedure TTestFactory.CreateInt64;
begin
  SetMyInstanceTypes;
  Data:=CreateJSON(Int64(1));
  AssertEquals('Correct class',TMyInt64,Data.ClassType);
end;

procedure TTestFactory.CreateFloat;
begin
  SetMyInstanceTypes;
  Data:=CreateJSON(1.2);
  AssertEquals('Correct class',TMyFloat,Data.ClassType);
end;

procedure TTestFactory.CreateBoolean;
begin
  SetMyInstanceTypes;
  Data:=CreateJSON(True);
  AssertEquals('Correct class',TMyBoolean,Data.ClassType);
end;

procedure TTestFactory.CreateString;
begin
  SetMyInstanceTypes;
  Data:=CreateJSON('True');
  AssertEquals('Correct class',TMyString,Data.ClassType);
end;

procedure TTestFactory.CreateArray;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONArray(['True']);
  AssertEquals('Correct class',TMyArray,Data.ClassType);
end;

procedure TTestFactory.CreateObject;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONObject(['a','True']);
  AssertEquals('Correct class',TMyObject,Data.ClassType);
end;

procedure TTestFactory.ArrayAddNull;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONArray([]);
  TJSONArray(Data).Add();
  AssertElement0(TMyNull);
end;

procedure TTestFactory.ArrayAddInteger;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONArray([]);
  TJSONArray(Data).Add(1);
  AssertElement0(TMyInteger);
end;

procedure TTestFactory.ArrayAddInt64;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONArray([]);
  TJSONArray(Data).Add(Int64(1));
  AssertElement0(TMyInt64);
end;

procedure TTestFactory.ArrayAddFloat;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONArray([]);
  TJSONArray(Data).Add(1.2);
  AssertElement0(TMyFloat);
end;

procedure TTestFactory.ArrayAddBoolean;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONArray([]);
  TJSONArray(Data).Add(True);
  AssertElement0(TMyBoolean);
end;

procedure TTestFactory.ArrayAddString;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONArray([]);
  TJSONArray(Data).Add('True');
  AssertElement0(TMyString);
end;

procedure TTestFactory.ArrayCreateNull;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONArray([Nil]);
  AssertElement0(TMyNull);
end;

procedure TTestFactory.ArrayCreateInteger;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONArray([1]);
  AssertElement0(TMyInteger);
end;

procedure TTestFactory.ArrayCreateInt64;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONArray([int64(1)]);
  AssertElement0(TMyInt64);
end;

procedure TTestFactory.ArrayCreateFloat;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONArray([1.2]);
  AssertElement0(TMyFloat);
end;

procedure TTestFactory.ArrayCreateBoolean;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONArray([True]);
  AssertElement0(TMyBoolean);
end;

procedure TTestFactory.ArrayCreateString;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONArray(['true']);
  AssertElement0(TMyString);
end;

procedure TTestFactory.ObjectAddNull;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONObject([]);
  TJSONObject(Data).Add('a');
  AssertElementA(TMyNull);
end;

procedure TTestFactory.ObjectAddInteger;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONObject([]);
  TJSONObject(Data).Add('a',1);
  AssertElementA(TMyInteger);
end;

procedure TTestFactory.ObjectAddInt64;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONObject([]);
  TJSONObject(Data).Add('a',Int64(1));
  AssertElementA(TMyInt64);
end;

procedure TTestFactory.ObjectAddFloat;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONObject([]);
  TJSONObject(Data).Add('a',1.2);
  AssertElementA(TMyFloat);
end;

procedure TTestFactory.ObjectAddBoolean;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONObject([]);
  TJSONObject(Data).Add('a',True);
  AssertElementA(TMyBoolean);
end;

procedure TTestFactory.ObjectAddString;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONObject([]);
  TJSONObject(Data).Add('a','True');
  AssertElementA(TMyString);
end;

procedure TTestFactory.ObjectCreateNull;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONObject(['a',Nil]);
  AssertElementA(TMyNull);
end;

procedure TTestFactory.ObjectCreateInteger;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONObject(['a',1]);
  AssertElementA(TMyInteger);
end;

procedure TTestFactory.ObjectCreateInt64;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONObject(['a',int64(1)]);
  AssertElementA(TMyInt64);
end;

procedure TTestFactory.ObjectCreateFloat;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONObject(['a',1.2]);
  AssertElementA(TMyFloat);
end;

procedure TTestFactory.ObjectCreateBoolean;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONObject(['a',True]);
  AssertElementA(TMyBoolean);
end;

procedure TTestFactory.ObjectCreateString;
begin
  SetMyInstanceTypes;
  Data:=CreateJSONObject(['a','true']);
  AssertElementA(TMyString);
end;

{ TTestJSONPath }

procedure TTestJSONPath.TearDown;
begin
  FreeAndNil(FData);
  inherited TearDown;
end;

procedure TTestJSONPath.TestNullEmpty;
begin
  Data:=TJSONNull.Create;
  AssertSame('Empty on NULL returns object itself',Data,Data.FIndPath(''));
end;

procedure TTestJSONPath.TestNullGet;
begin
  Data:=TJSONNull.Create;
  AssertSame('Empty get on NULL returns object itself',Data,Data.GetPath(''));
end;

procedure TTestJSONPath.TestNullNonExisting;

Var
  Msg : String;

begin
  Data:=TJSONNull.Create;
  try
    Data.GetPath('a.b.c');
    Msg:='No exception raised'
  except
    on E : Exception do
      begin
      If Not (E is EJSON) then
        Msg:='Wrong exception class. Got '+E.ClassName+' instead of EJSON'
      else
        If E.Message<>'Path "a.b.c" invalid: element "a.b.c" not found.' then
          Msg:='Wrong exception message, expected: "Path "a.b.c" invalid: element "a.b.c" not found.", actual: "'+E.Message+'"';
      end;
  end;
  If (Msg<>'') then
    Fail(Msg);
end;

procedure TTestJSONPath.TestNullNotEmpty;
begin
  Data:=TJSONNull.Create;
  AssertNull('Not empty on NULL returns nil',Data.FindPath('a'));
end;

procedure TTestJSONPath.TestBooleanEmpty;
begin
  Data:=TJSONBoolean.Create(true);
  AssertSame('Empty on Boolean returns object itself',Data,Data.FIndPath(''));
end;

procedure TTestJSONPath.TestBooleanNotEmpty;
begin
  Data:=TJSONBoolean.Create(True);
  AssertNull('Not empty on Boolean returns nil',Data.FindPath('a'));
end;

procedure TTestJSONPath.TestIntegerEmpty;
begin
  Data:=TJSONIntegerNumber.Create(1);
  AssertSame('Empty on integer returns object itself',Data,Data.FIndPath(''));
end;

procedure TTestJSONPath.TestIntegerNotEmpty;
begin
  Data:=TJSONIntegerNumber.Create(1);
  AssertNull('Not Empty on integer returns object itself',Data.FIndPath('a'));
end;

procedure TTestJSONPath.TestInt64Empty;
begin
  Data:=TJSONInt64Number.Create(1);
  AssertSame('Empty on Int64 returns object itself',Data,Data.FIndPath(''));
end;

procedure TTestJSONPath.TestInt64NotEmpty;
begin
  Data:=TJSONInt64Number.Create(1);
  AssertNull('Not Empty on Int64 returns object itself',Data.FIndPath('a'));
end;

procedure TTestJSONPath.TestFloatEmpty;
begin
  Data:=TJSONFloatNumber.Create(1);
  AssertSame('Empty on Float returns object itself',Data,Data.FIndPath(''));
end;

procedure TTestJSONPath.TestFloatNotEmpty;
begin
  Data:=TJSONFloatNumber.Create(1);
  AssertNull('Not Empty on Float returns object itself',Data.FIndPath('a'));
end;

procedure TTestJSONPath.TestStringEmpty;
begin
  Data:=TJSONString.Create('1');
  AssertSame('Empty on String returns object itself',Data,Data.FIndPath(''));
end;

procedure TTestJSONPath.TestStringNotEmpty;
begin
  Data:=TJSONString.Create('1');
  AssertNull('Not Empty on String returns object itself',Data.FIndPath('a'));
end;

procedure TTestJSONPath.TestArrayEmpty;
begin
  Data:=TJSONArray.Create([1,2,3]);
  AssertSame('Empty on array returns object itself',Data,Data.FIndPath(''));
end;

procedure TTestJSONPath.TestArrayNotIndex;
begin
  Data:=TJSONArray.Create([1,2,3]);
  AssertNull('Not index indication on array returns object itself',Data.FindPath('oo'));
end;

procedure TTestJSONPath.TestArrayIncompleteIndex;
begin
  Data:=TJSONArray.Create([1,2,3]);
  AssertNull('Not complete index indication on array returns object itself',Data.FindPath('[1'));
  AssertNull('Not complete index indication on array returns object itself',Data.FindPath('['));
end;

procedure TTestJSONPath.TestArrayNonNumericalIndex;
begin
  Data:=TJSONArray.Create([1,2,3]);
  AssertNull('Not complete index indication on array returns object itself',Data.FindPath('[a]'));
end;

procedure TTestJSONPath.TestArrayOutOfRangeIndex;
begin
  Data:=TJSONArray.Create([1,2,3]);
  AssertNull('Not complete index indication on array returns object itself',Data.FindPath('[-1]'));
  AssertNull('Not complete index indication on array returns object itself',Data.FindPath('[3]'));
end;

procedure TTestJSONPath.TestArrayCorrectIndex;
begin
  Data:=TJSONArray.Create([1,2,3]);
  AssertSame('Index 0 on array returns item 0',Data.Items[0],Data.FindPath('[0]'));
  AssertSame('Index 1 on array returns item 1',Data.Items[1],Data.FindPath('[1]'));
  AssertSame('Index 2 on array returns item 2',Data.Items[2],Data.FindPath('[2]'));
end;

procedure TTestJSONPath.TestArrayRecursiveArray;

Var
  A : TJSONArray;

begin
  A:=TJSONArray.Create([1,2,3]);
  Data:=TJSONArray.Create([A,1,2,3]);
  AssertSame('Index [0][0] on array returns item 0',A.Items[0],Data.FindPath('[0][0]'));
  AssertSame('Index [0][1] on array returns item 1',A.Items[1],Data.FindPath('[0][1]'));
  AssertSame('Index [0][2] on array returns item 2',A.Items[2],Data.FindPath('[0][2]'));
end;

procedure TTestJSONPath.TestArrayRecursiveObject;

Var
  A : TJSONObject;

begin
  A:=TJSONObject.Create(['a',1,'b',2,'c',3]);
  Data:=TJSONArray.Create([A,1,2,3]);
  AssertSame('[0]a on array returns element a of item 0',A.Elements['a'],Data.FindPath('[0]a'));
  AssertSame('[0]b on array returns element b of item 0',A.Elements['b'],Data.FindPath('[0]b'));
  AssertSame('[0]c on array returns element c of item 0',A.Elements['c'],Data.FindPath('[0]c'));
  AssertSame('[0].a on array returns element a of item 0',A.Elements['a'],Data.FindPath('[0].a'));
  AssertSame('[0].b on array returns element b of item 0',A.Elements['b'],Data.FindPath('[0].b'));
  AssertSame('[0].c on array returns element c of item 0',A.Elements['c'],Data.FindPath('[0].c'));
end;

procedure TTestJSONPath.TestObjectEmpty;
begin
  Data:=TJSONObject.Create(['a',1,'b',2,'c',3]);
  AssertSame('Empty on object returns object',Data,Data.FindPath(''));
end;

procedure TTestJSONPath.TestObjectDots;
begin
  Data:=TJSONObject.Create(['a',1,'b',2,'c',3]);
  AssertSame('Dot on object returns object',Data,Data.FindPath('.'));
  AssertSame('2 Dots on object returns object',Data,Data.FindPath('..'));
  AssertSame('3 Dots on object returns object',Data,Data.FindPath('...'));
end;

procedure TTestJSONPath.TestObjectExisting;
begin
  Data:=TJSONObject.Create(['a',1,'b',2,'c',3]);
  AssertSame('a on object returns element a',TJSONObject(Data).Elements['a'],Data.FindPath('a'));
  AssertSame('.a on object returns element a',TJSONObject(Data).Elements['a'],Data.FindPath('.a'));
  AssertSame('..a on object returns element a',TJSONObject(Data).Elements['a'],Data.FindPath('..a'));
end;

procedure TTestJSONPath.TestObjectNonExisting;
begin
  Data:=TJSONObject.Create(['a',1,'b',2,'c',3]);
  AssertNull('d on object returns nil',Data.FindPath('d'));
end;

procedure TTestJSONPath.TestObjectTrailingDot;
begin
  Data:=TJSONObject.Create(['a',1,'b',2,'c',3]);
  AssertNull('a. on object returns nil',Data.FindPath('a.'));
end;

procedure TTestJSONPath.TestObjectRecursiveArray;

Var
  A : TJSONArray;

begin
  A:=TJSONArray.Create([1,2,3]);
  Data:=TJSONObject.Create(['a',A,'b',2,'c',3]);
  AssertSame('a[0] returns item 0 of array a',A.Items[0],Data.FindPath('a[0]'));
end;

procedure TTestJSONPath.TestObjectRecursiveObject;
Var
  O : TJSONObject;
  D : TJSONData;
begin
  D :=TJSONIntegerNumber.Create(1);
  O:=TJSONObject.Create(['b',D]);
  Data:=TJSONObject.Create(['a',O]);
  AssertSame('a.b returns correct data ',D,Data.FindPath('a.b'));
  AssertSame('a..b returns correct data ',D,Data.FindPath('a..b'));
end;

procedure TTestJSONPath.TestDeepRecursive;
Var
  A : TJSONArray;
  D : TJSONData;
begin
  D :=TJSONIntegerNumber.Create(1);
  A:=TJSONArray.Create([0,'string',TJSONObject.Create(['b',D])]);
  Data:=TJSONObject.Create(['a',TJSONObject.Create(['c',A])]);
  AssertSame('a.c[2].b returns correct data ',D,Data.FindPath('a.c[2].b'));
  AssertSame('a.c[2]b returns correct data ',D,Data.FindPath('a.c[2]b'));
  AssertNull('a.c[2]d returns nil ',Data.FindPath('a.c[2]d'));
end;

{ TTestJSON }

Const
  DefJSONInstanceTypes :
    Array [TJSONInstanceType] of TJSONDataClass = (TJSONData, TJSONIntegerNumber,
    TJSONInt64Number,TJSONQWordNumber,TJSONFloatNumber, TJSONString, TJSONBoolean,
    TJSONNull, TJSONArray, TJSONObject);

Const
  MyJSONInstanceTypes :
    Array [TJSONInstanceType] of TJSONDataClass = (TJSONData, TMyInteger,
    TMyInt64, TMyQWord,TMyFloat, TMyString, TMyBoolean, TMyNull, TMyArray,
    TMyObject);

procedure TTestJSON.SetDefaultInstanceTypes;

Var
  Ti : TJSONInstanceType;

begin
  For ti:=Low(TJSONInstanceType) to High(TJSONInstanceType) do
   SetJSONInstanceType(Ti,DefJSONInstanceTypes[ti]);
end;

procedure TTestJSON.SetMyInstanceTypes;

Var
  Ti : TJSONInstanceType;

begin
  For ti:=Low(TJSONInstanceType) to High(TJSONInstanceType) do
    AssertEquals('Previous value is returned by SetJSONInstanceType',DefJSONInstanceTypes[ti],SetJSONInstanceType(Ti,MyJSONInstanceTypes[ti]));
end;

Procedure TTestJSON.SetUp;


begin
  inherited SetUp;
  SetDefaultInstanceTypes;
  TJSONData.CompressedJSON:=False;
  TJSONObject.UnquotedMemberNames:=False;
end;

Procedure TTestJSON.TestItemCount(J: TJSONData; Expected: Integer);
begin
  AssertEquals(J.ClassName+'.ItemCount',Expected,J.Count);
end;

Procedure TTestJSON.TestJSONType(J: TJSONData; Expected: TJSONType);
begin
  AssertEquals(J.ClassName+'.JSONType',Ord(Expected),Ord(J.JSONType));
end;

Procedure TTestJSON.TestJSON(J: TJSONData; Expected: TJSONStringType);
begin
  AssertEquals(J.ClassName+'.AsJSON',Expected,J.AsJSON);
end;

Procedure TTestJSON.TestIsNull(J: TJSONData; Expected: Boolean);
begin
  AssertEquals(J.ClassName+'.IsNull',Expected,J.IsNull);
end;

Procedure TTestJSON.TestAsBoolean(J: TJSONData; Expected: Boolean;
  ExpectError: boolean);

Var
  B : Boolean;
  AssignOK : Boolean;
  Msg : String;
  
begin
  AssignOK:=False;
  Try
    B:=J.AsBoolean;
    AssignOK:=True;
    If Not ExpectError then
      AssertEquals(J.Classname+'.AsBoolean',Expected,B);
  except
    On E : Exception do
      begin
      AssignOK:=False;
      Msg:=E.Message;
      end;
  end;
  If ExpectError then
    begin
    If AssignOK then
      Fail(J.ClassName+'.AsBoolean must raise error');
    end
  else
    begin
    If not AssignOK then
      Fail(J.ClassName+'.AsBoolean raised unexpected exception: '+Msg)
    end;
end;

Procedure TTestJSON.TestAsInteger(J: TJSONData; Expected: Integer;
  ExpectError: boolean);

Var
  I : Integer;
  AssignOK : Boolean;
  Msg : String;

begin
  AssignOK:=False;
  Try
    I:=J.AsInteger;
    AssignOK:=True;
    If Not ExpectError then
      AssertEquals(J.Classname+'.AsInteger',Expected,I);
  except
    On E : Exception do
      begin
      AssignOK:=False;
      Msg:=E.Message;
      end;
  end;
  If ExpectError then
    begin
    If AssignOK then
      Fail(J.ClassName+'.AsInteger must raise error');
    end
  else
    begin
    If not AssignOK then
      Fail(J.ClassName+'.AsInteger raised unexpected exception: '+Msg)
    end;
end;

Procedure TTestJSON.TestAsInt64(J: TJSONData; Expected: Int64;
  ExpectError: boolean);

Var
  I : Int64;
  AssignOK : Boolean;
  Msg : String;

begin
  AssignOK:=False;
  Try
    I:=J.AsInt64;
    AssignOK:=True;
    If Not ExpectError then
      AssertEquals(J.Classname+'.AsInt64',Expected,I);
  except
    On E : Exception do
      begin
      AssignOK:=False;
      Msg:=E.Message;
      end;
  end;
  If ExpectError then
    begin
    If AssignOK then
      Fail(J.ClassName+'.AsInt64 must raise error');
    end
  else
    begin
    If not AssignOK then
      Fail(J.ClassName+'.AsInt64 raised unexpected exception: '+Msg)
    end;
end;

Procedure TTestJSON.TestAsQWord(J: TJSONData; Expected: QWord;
  ExpectError: boolean);
Var
  Q : QWord;
  AssignOK : Boolean;
  Msg : String;

begin
  AssignOK:=False;
  Try
    Q:=J.AsQWord;
    AssignOK:=True;
    If Not ExpectError then
      AssertEquals(J.Classname+'.AsQWord',IntToStr(Expected),IntToStr(Q));
  except
    On E : Exception do
      begin
      AssignOK:=False;
      Msg:=E.Message;
      end;
  end;
  If ExpectError then
    begin
    If AssignOK then
      Fail(J.ClassName+'.AsQWord must raise error');
    end
  else
    begin
    If not AssignOK then
      Fail(J.ClassName+'.AsInt64 raised unexpected exception: '+Msg)
    end;
end;

Procedure TTestJSON.TestAsString(J: TJSONData; Expected: String;
  ExpectError: boolean);
  
Var
  S : String;
  AssignOK : Boolean;
  Msg : String;

begin
  AssignOK:=False;
  Try
    S:=J.AsString;
    AssignOK:=True;
    If Not ExpectError then
      AssertEquals(J.Classname+'.AsString',Expected,S);
  except
    On E : Exception do
      begin
      AssignOK:=False;
      Msg:=E.Message;
      end;
  end;
  If ExpectError then
    begin
    If AssignOK then
      Fail(J.ClassName+'.AsString must raise error');
    end
  else
    begin
    If not AssignOK then
      Fail(J.ClassName+'.AsString raised unexpected exception: '+Msg)
    end;
end;

Procedure TTestJSON.TestAsFloat(J: TJSONData; Expected: TJSONFloat;
  ExpectError: boolean);
  
Var
  F : TJSONFloat;
  AssignOK : Boolean;
  Msg : String;

begin
  AssignOK:=False;
  Try
    F:=J.AsFloat;
    AssignOK:=True;
    If Not ExpectError then
      AssertEquals(J.Classname+'.AsFloat',Expected,F);
  except
    On E : Exception do
      begin
      AssignOK:=False;
      Msg:=E.Message;
      end;
  end;
  If ExpectError then
    begin
    If AssignOK then
      Fail(J.ClassName+'.AsFloat must raise error');
    end
  else
    begin
    If not AssignOK then
      Fail(J.ClassName+'.AsFloat raised unexpected exception: '+Msg)
    end;
end;

{ TTestBoolean }

procedure TTestBoolean.TestTrue;

Var
  J : TJSONBoolean;

begin
  J:=TJSONBoolean.Create(True);
  try
    TestJSONType(J,jtBoolean);
    TestItemCount(J,0);
    TestJSON(J,'true');
    TestIsNull(J,False);
    TestAsBoolean(J,True);
    TestAsInteger(J,1);
    TestAsInt64(J,1);
    TestAsQword(J,1);
    TestAsString(J,BoolToStr(True,'true','false'));
    TestAsFloat(J,1.0);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestBoolean.TestFalse;

Var
  J : TJSONBoolean;

begin
  J:=TJSONBoolean.Create(False);
  try
    TestJSONType(J,jtBoolean);
    TestItemCount(J,0);
    TestJSON(J,'false');
    TestIsNull(J,False);
    TestAsBoolean(J,False);
    TestAsInteger(J,0);
    TestAsInt64(J,0);
    TestAsQWord(J,0);
    TestAsString(J,BoolToStr(False,'true','false'));
    TestAsFloat(J,0.0);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestBoolean.TestClone;

Var
  B : TJSONBoolean;
  D : TJSONData;

begin
  B:=TJSONBoolean.Create(true);
  try
    D:=B.Clone;
    try
     TestJSONType(D,jtBoolean);
     TestAsBoolean(D,true);
    finally
      D.Free;
    end;
  finally
    FreeAndNil(B);
  end;
end;

procedure TTestBoolean.TestMyClone;
Var
  B : TMyBoolean;
  D : TJSONData;

begin
  B:=TMyBoolean.Create(true);
  try
    D:=B.Clone;
    try
     TestJSONType(D,jtBoolean);
     AssertEquals('Correct class',TMyBoolean,D.ClassType);
     TestAsBoolean(D,true);
    finally
      D.Free;
    end;
  finally
    FreeAndNil(B);
  end;
end;

procedure TTestBoolean.TestFormat;

Var
  B : TJSONBoolean;

begin
  B:=TJSONBoolean.Create(true);
  try
    AssertEquals('FormatJSON same as asJSON',B.asJSON,B.FormatJSON);
  finally
    B.Free;
  end;
end;



{ TTestNull }

procedure TTestNull.TestNull;

Var
  J : TJSONNull;

begin
  J:=TJSONNull.Create;
  try
    TestJSONType(J,jtNull);
    TestItemCount(J,0);
    TestJSON(J,'null');
    TestIsNull(J,True);
    TestAsBoolean(J,False,True);
    TestAsInteger(J,0,true);
    TestAsInt64(J,0,true);
    TestAsQWord(J,0,true);
    TestAsString(J,BoolToStr(False),true);
    TestAsFloat(J,0.0,true);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestNull.TestClone;

Var
  J : TJSONNull;
  D : TJSONData;

begin
  J:=TJSONNull.Create;
  try
    D:=J.Clone;
    try
      TestIsNull(D,True);
    finally
      D.Free;
    end;
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestNull.TestMyClone;
Var
  J : TMyNull;
  D : TJSONData;

begin
  J:=TMyNull.Create;
  try
    D:=J.Clone;
    try
      TestIsNull(D,True);
      AssertEquals('Correct class',TMyNull,D.ClassType);
    finally
      D.Free;
    end;
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestNull.TestFormat;
Var
  J : TJSONNull;
begin
  J:=TJSONNull.Create;
  try
    AssertEquals('FormatJSON same as asJSON',J.asJSON,J.FormatJSON);
  finally
    J.Free;
  end;
end;


{ TTestString }

procedure TTestString.TestString;

Const
  S = 'A string';

Var
  J : TJSONString;

begin
  J:=TJSONString.Create(S);
  try
    TestJSONType(J,jtString);
    TestItemCount(J,0);
    TestJSON(J,'"'+S+'"');
    TestIsNull(J,False);
    TestAsBoolean(J,False,True);
    TestAsInteger(J,0,true);
    TestAsInt64(J,0,true);
    TestAsQWord(J,0,true);
    TestAsString(J,S);
    TestAsFloat(J,0.0,true);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestString.TestControlString;
Var
  J : TJSONString;
  I : Integer;
  T : String;

begin
  J:=TJSONString.Create('');
  try
    For I:=0 to 31 do
      begin
      J.AsString:='-->'+Char(I)+'<--';
      Case I of
       8  : T:='\b';
       9  : T:='\t';
       10 : T:='\n';
       12 : T:='\f';
       13 : T:='\r';
      else
        T:='\u'+HexStr(I,4);
      end;
      AssertEquals('Control char','"-->'+T+'<--"',J.AsJSON);
      end;
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestString.TestSolidus;
Var
  J : TJSONString;

begin
  J:=TJSONString.Create('');
  try
    J.AsString:='http://www.json.org/';
    TJSONString.StrictEscaping:=True;
    TestJSON(J,'"http:\/\/www.json.org\/"');
    TJSONString.StrictEscaping:=False;
    TestJSON(J,'"http://www.json.org/"');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestString.TestInteger;

Const
  S = '1';

Var
  J : TJSONString;

begin
  J:=TJSONString.Create(S);
  try
    TestJSONType(J,jtString);
    TestItemCount(J,0);
    TestJSON(J,'"'+S+'"');
    TestIsNull(J,False);
    TestAsBoolean(J,True,False);
    TestAsInteger(J,1,False);
    TestAsInt64(J,1,False);
    TestAsQWord(J,1,False);
    TestAsString(J,S);
    TestAsFloat(J,1.0,False);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestString.TestNegativeInteger;

Const
  S = '-1';

Var
  J : TJSONString;

begin
  J:=TJSONString.Create(S);
  try
    TestJSONType(J,jtString);
    TestItemCount(J,0);
    TestJSON(J,'"'+S+'"');
    TestIsNull(J,False);
    TestAsBoolean(J,True,False);
    TestAsInteger(J,-1,False);
    TestAsInt64(J,-1,False);
    TestAsQWord(J,QWord(-1),True);
    TestAsString(J,S);
    TestAsFloat(J,-1.0,False);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestString.TestFloat;

begin
  DoTestFloat(1.0,'1.0',True);
  DoTestFloat(1.0,'1',True);
  DoTestFloat(1.0,'1e0',True);
  DoTestFloat(1.2,'1.2',True);
  DoTestFloat(12.0,'1.2e1',True);
end;

procedure TTestString.TestNegativeFloat;
begin
  DoTestFloat(-1.0,'-1.0',True);
  DoTestFloat(-1.0,'-1',True);
  DoTestFloat(-1.0,'-1e0',True);
  DoTestFloat(-1.2,'-1.2',True);
  DoTestFloat(-12.0,'-1.2e1',True);
end;

procedure TTestString.TestBooleanTrue;

Const
  S = 'true';

Var
  J : TJSONString;

begin
  J:=TJSONString.Create(S);
  try
    TestJSONType(J,jtString);
    TestItemCount(J,0);
    TestJSON(J,'"'+S+'"');
    TestIsNull(J,False);
    TestAsBoolean(J,True,False);
    TestAsInteger(J,-1,True);
    TestAsInt64(J,-1,True);
    TestAsQWord(J,QWord(-1),True);
    TestAsString(J,S);
    TestAsFloat(J,-1.0,True);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestString.TestBooleanFalse;

Const
  S = 'false';

Var
  J : TJSONString;

begin
  J:=TJSONString.Create(S);
  try
    TestJSONType(J,jtString);
    TestItemCount(J,0);
    TestJSON(J,'"'+S+'"');
    TestIsNull(J,False);
    TestAsBoolean(J,False,False);
    TestAsInteger(J,0,True);
    TestAsInt64(J,0,True);
    TestAsQWord(J,0,True);
    TestAsString(J,S);
    TestAsFloat(J,0,True);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestString.TestClone;

Var
  S : TJSONString;
  D : TJSONData;

begin
  S:=TJSONString.Create('aloha');
  try
    D:=S.Clone;
    try
     TestJSONType(D,jtString);
     TestAsString(D,'aloha');
    finally
      D.Free;
    end;
  finally
    FreeAndNil(S);
  end;
end;

procedure TTestString.TestMyClone;
Var
  S : TMyString;
  D : TJSONData;

begin
  S:=TMyString.Create('aloha');
  try
    D:=S.Clone;
    try
      AssertEquals('Correct class',TMyString,D.ClassType);
     TestJSONType(D,jtString);
     TestAsString(D,'aloha');
    finally
      D.Free;
    end;
  finally
    FreeAndNil(S);
  end;
end;

procedure TTestString.TestFormat;
Var
  S : TJSONString;

begin
  S:=TJSONString.Create('aloha');
  try
    AssertEquals('FormatJSON equals JSON',S.AsJSON,S.FormatJSOn);
  finally
    FreeAndNil(S);
  end;
end;

procedure TTestString.DoTestFloat(F : TJSOnFloat;S : String; OK : Boolean);

Var
  J : TJSONString;

begin
  J:=TJSONString.Create(S);
  try
    TestJSONType(J,jtString);
    TestItemCount(J,0);
    TestJSON(J,'"'+S+'"');
    TestIsNull(J,False);
    TestAsBoolean(J,(F<>0),Not OK);
    TestAsInteger(J,Round(F),(Pos('.',S)<>0) or (Pos('E',UpperCase(S))<>0));
    TestAsInt64(J,Round(F),(Pos('.',S)<>0) or (Pos('E',UpperCase(S))<>0));
    if F>0 then
      TestAsQword(J,Round(F),(Pos('.',S)<>0) or (Pos('E',UpperCase(S))<>0));
    TestAsString(J,S);
    TestAsFloat(J,F,Not OK);
  finally
    FreeAndNil(J);
  end;
end;


{ TTestInteger }

procedure TTestInteger.DoTest(I: Integer);

Var
  J : TJSONIntegerNumber;

begin
  J:=TJSONIntegerNumber.Create(I);
  try
    TestJSONType(J,jtNumber);
    TestItemCount(J,0);
    AssertEquals('Numbertype is ntInteger',ord(ntInteger),Ord(J.NumberType));
    TestJSON(J,IntToStr(i));
    TestIsNull(J,False);
    TestAsBoolean(J,(I<>0));
    TestAsInteger(J,I);
    TestAsInt64(J,I);
    TestAsQword(J,I);
    TestAsString(J,IntToStr(I));
    TestAsFloat(J,I);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestInteger.TestPositive;

begin
  DoTest(1);
end;

procedure TTestInteger.TestNegative;
begin
  DoTest(-1);
end;

procedure TTestInteger.TestZero;
begin
  DoTest(0);
end;

procedure TTestInteger.TestClone;

Var
  I : TJSONIntegerNumber;
  D : TJSONData;

begin
  I:=TJSONIntegerNumber.Create(99);
  try
    D:=I.Clone;
    try
     TestJSONType(D,jtNumber);
     TestAsInteger(D,99);
    finally
      D.Free;
    end;
  finally
    FreeAndNil(I);
  end;

end;

procedure TTestInteger.TestMyClone;
Var
  I : TMyInteger;
  D : TJSONData;

begin
  I:=TMyInteger.Create(99);
  try
    D:=I.Clone;
    try
     AssertEquals('Correct class',TMyInteger,D.ClassType);
     TestJSONType(D,jtNumber);
     TestAsInteger(D,99);
    finally
      D.Free;
    end;
  finally
    FreeAndNil(I);
  end;
end;

procedure TTestInteger.TestFormat;

Var
  I : TJSONIntegerNumber;

begin
  I:=TJSONIntegerNumber.Create(99);
  try
    AssertEquals('FormatJSON equal to JSON',I.AsJSON,I.FormatJSON);
  finally
    FreeAndNil(I);
  end;
end;

{ TTestInt64 }

procedure TTestInt64.DoTest(I: Int64);

Var
  J : TJSONInt64Number;

begin
  J:=TJSONInt64Number.Create(I);
  try
    TestJSONType(J,jtNumber);
    TestItemCount(J,0);
    AssertEquals('Numbertype is ntInt64',ord(ntInt64),Ord(J.NumberType));
    TestJSON(J,IntToStr(i));
    TestIsNull(J,False);
    TestAsBoolean(J,(I<>0));
    TestAsInteger(J,I);
    TestAsInt64(J,I);
    TestAsQword(J,I);
    TestAsString(J,IntToStr(I));
    TestAsFloat(J,I);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestInt64.TestPositive;

begin
  DoTest(1);
end;

procedure TTestInt64.TestNegative;
begin
  DoTest(-1);
end;

procedure TTestInt64.TestZero;
begin
  DoTest(0);
end;

procedure TTestInt64.TestClone;

Var
  I : TJSONInt64Number;
  D : TJSONData;

begin
  I:=TJSONInt64Number.Create(99);
  try
    D:=I.Clone;
    try
     TestJSONType(D,jtNumber);
     AssertEquals('Numbertype is ntInt64',ord(ntInt64),Ord(TJSONInt64Number(D).NumberType));
     TestAsInteger(D,99);
    finally
      D.Free;
    end;
  finally
    FreeAndNil(I);
  end;

end;

procedure TTestInt64.TestMyClone;
Var
  I : TMyInt64;
  D : TJSONData;

begin
  I:=TMyInt64.Create(99);
  try
    D:=I.Clone;
    try
      AssertEquals('Correct class',TMyInt64,D.ClassType);
     TestJSONType(D,jtNumber);
     AssertEquals('Numbertype is ntInt64',ord(ntInt64),Ord(TMyInt64(D).NumberType));
     TestAsInteger(D,99);
    finally
      D.Free;
    end;
  finally
    FreeAndNil(I);
  end;
end;

procedure TTestInt64.TestFormat;
Var
  I : TJSONInt64Number;

begin
  I:=TJSONInt64Number.Create(99);
  try
    AssertEquals('FormatJSON equal to JSON',I.AsJSON,I.FormatJSON);
  finally
    FreeAndNil(I);
  end;
end;

{ TTestQWord }

procedure TTestQWord.DoTest(Q: QWord);

Var
  J : TJSONQWordNumber;

begin
  J:=TJSONQWordNumber.Create(Q);
  try
    TestJSONType(J,jtNumber);
    TestItemCount(J,0);
    AssertEquals('Numbertype is ntQWord',ord(ntQWord),Ord(J.NumberType));
    TestJSON(J,IntToStr(Q));
    TestIsNull(J,False);
    TestAsBoolean(J,(Q<>0));
    TestAsInteger(J,Q);
    TestAsInt64(J,Q);
    TestAsQword(J,Q);
    TestAsString(J,IntToStr(Q));
    TestAsFloat(J,Q);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestQWord.TestPositive;

begin
  DoTest(1);
end;


procedure TTestQWord.TestZero;
begin
  DoTest(0);
end;

procedure TTestQWord.TestClone;

Var
  I : TJSONQWordNumber;
  D : TJSONData;

begin
  I:=TJSONQWordNumber.Create(99);
  try
    D:=I.Clone;
    try
     TestJSONType(D,jtNumber);
     AssertEquals('Numbertype is ntQWord',ord(ntQWord),Ord(TJSONQWordNumber(D).NumberType));
     TestAsInteger(D,99);
    finally
      D.Free;
    end;
  finally
    FreeAndNil(I);
  end;

end;

procedure TTestQWord.TestMyClone;
Var
  I : TMyQWord;
  D : TJSONData;

begin
  I:=TMyQWord.Create(99);
  try
    D:=I.Clone;
    try
      AssertEquals('Correct class',TMyQWord,D.ClassType);
     TestJSONType(D,jtNumber);
     AssertEquals('Numbertype is ntQWord',ord(ntQWord),Ord(TMyQWord(D).NumberType));
     TestAsInteger(D,99);
    finally
      D.Free;
    end;
  finally
    FreeAndNil(I);
  end;
end;

procedure TTestQWord.TestFormat;
Var
  I : TJSONQWordNumber;

begin
  I:=TJSONQWordNumber.Create(99);
  try
    AssertEquals('FormatJSON equal to JSON',I.AsJSON,I.FormatJSON);
  finally
    FreeAndNil(I);
  end;
end;

{ TTestFloat }

procedure TTestFloat.DoTest(F: TJSONFloat);

Var
  J : TJSONFloatNumber;
  S : String;
  
begin
  Str(F,S);
  If S[1]=' ' then
    Delete(S,1,1);
  J:=TJSONFloatNumber.Create(F);
  try
    TestJSONType(J,jtNumber);
    TestItemCount(J,0);
    AssertEquals('Numbertype is ntFloat',ord(ntFloat),Ord(J.NumberType));
    TestJSON(J,S);
    TestIsNull(J,False);
    TestAsBoolean(J,(F<>0));
    TestAsInteger(J,Round(F));
    TestAsInt64(J,Round(F));
    TestAsQword(J,Round(F));
    TestAsString(J,S);
    TestAsFloat(J,F);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestFloat.TestPositive;
begin
  DoTest(1.0);
  DoTest(1.2);
  DoTest(1.2e1);
  DoTest(1.2e-1);
  DoTest(1.2e10);
  DoTest(1.2e-10);
end;

procedure TTestFloat.TestNegative;
begin
  DoTest(-1.0);
  DoTest(-1.2);
  DoTest(-1.2e1);
  DoTest(-1.2e-1);
  DoTest(-1.2e10);
  DoTest(-1.2e-10);
end;

procedure TTestFloat.TestZero;
begin
  DoTest(0.0);
end;

procedure TTestFloat.TestClone;

Var
  F : TJSONFloatNumber;
  D : TJSONData;

begin
  F:=TJSONFloatNumber.Create(1.23);
  try
    D:=F.Clone;
    try
     TestJSONType(D,jtNumber);
     AssertEquals('Numbertype is ntFloat',ord(ntFloat),Ord(TJSONFloatNumber(D).NumberType));
     TestAsFloat(D,1.23);
    finally
      D.Free;
    end;
  finally
    FreeAndNil(F);
  end;

end;

procedure TTestFloat.TestMyClone;

Var
  F : TMyFloat;
  D : TJSONData;

begin
  F:=TMyFloat.Create(1.23);
  try
    D:=F.Clone;
    try
     AssertEquals('Correct class',TMyFloat,D.ClassType);
     TestJSONType(D,jtNumber);
     AssertEquals('Numbertype is ntFloat',ord(ntFloat),Ord(TMyFloat(D).NumberType));
     TestAsFloat(D,1.23);
    finally
      D.Free;
    end;
  finally
    FreeAndNil(F);
  end;
end;

procedure TTestFloat.TestFormat;

Var
  F : TJSONFloatNumber;


begin
  F:=TJSONFloatNumber.Create(1.23);
  try
    AssertEquals('FormatJSON equals asJSON',F.AsJSON,F.FormatJSON);
  finally
    FreeAndNil(F);
  end;
end;

{ TTestArray }

procedure TTestArray.TestCreate;

Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    TestJSONType(J,jtArray);
    TestItemCount(J,0);
    TestJSON(J,'[]');
    TestIsNull(J,False);
    TestAsBoolean(J,False,True);
    TestAsInteger(J,1,True);
    TestAsInt64(J,1,True);
    TestAsQWord(J,1,True);
    TestAsString(J,'',True);
    TestAsFloat(J,0.0,True);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestCreateString;

Const
  S = 'A string';

Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create([S]);
  try
    TestJSONType(J,jtArray);
    TestItemCount(J,1);
    TestJSONType(J[0],jtString);
    TestJSON(J,'["'+S+'"]');
    TestIsNull(J,False);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestCreatePchar;

Const
  S = 'A string';

Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create([Pchar(S)]);
  try
    TestJSONType(J,jtArray);
    TestItemCount(J,1);
    TestJSONType(J[0],jtString);
    TestJSON(J,'["'+S+'"]');
    TestIsNull(J,False);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestCreateStrings;

Const
  S = 'A string';
  T = 'B string';
  
Var
  J : TJSONArray;

begin
  J:=TJSONArray.Create([S,T]);
  try
    TestJSONType(J,jtArray);
    TestItemCount(J,2);
    TestJSONType(J[0],jtString);
    TestJSONType(J[1],jtString);
    TestJSON(J,'["'+S+'", "'+T+'"]');
    TestIsNull(J,False);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestCreateStringsCompressed;
Const
  S = 'A string';
  T = 'B string';

Var
  J : TJSONArray;

begin
  TJSONData.CompressedJSON:=True;
  J:=TJSONArray.Create([S,T]);
  try
    TestJSONType(J,jtArray);
    TestItemCount(J,2);
    TestJSONType(J[0],jtString);
    TestJSONType(J[1],jtString);
    TestJSON(J,'["'+S+'","'+T+'"]');
    TestIsNull(J,False);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestCreateInteger;

Const
  S = 3;

Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create([S]);
  try
    TestJSONType(J,jtArray);
    TestItemCount(J,1);
    TestJSONType(J[0],jtNumber);
    TestJSON(J,'[3]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestCreateFloat;

Const
  S : double = 1.2;

Var
  J : TJSONArray;
  r : String;
  
begin
  J:=TJSonArray.Create([S]);
  try
    TestJSONType(J,jtArray);
    TestItemCount(J,1);
    TestJSONType(J[0],jtNumber);
    Str(S,R);
    Delete(R,1,1);
    TestJSON(J,'['+R+']');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestCreateInt64;

Const
  S : Int64 = $FFFFFFFFFFFFF;

Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create([S]);
  try
    TestJSONType(J,jtArray);
    TestItemCount(J,1);
    TestJSONType(J[0],jtNumber);
    TestJSON(J,'['+IntToStr(S)+']');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestCreateBoolean;

Const
  S = True;

Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create([S]);
  try
    TestJSONType(J,jtArray);
    TestItemCount(J,1);
    TestJSONType(J[0],jtBoolean);
    TestJSON(J,'[true]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestCreateJSONObject;

Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create([TJSONObject.Create]);
  try
    TestItemCount(J,1);
    TestJSONType(J[0],jtObject);
    TestJSON(J,'[{}]');
  finally
    FreeAndNil(J);
  end;
end;
procedure TTestArray.TestCreateJSONString;

Const
  S = 'A string';

Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create([TJSONString.Create(S)]);
  try
    TestItemCount(J,1);
    TestJSONType(J[0],jtString);
    TestJSON(J,'["'+S+'"]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestCreateObject;

Var
  J : TJSONArray;
  O : TObject;
  
begin
  J:=Nil;
  try
    Try
      O:=TObject.Create;
      J:=TJSONArray.Create([O]);
      Fail('Array constructor accepts only TJSONData');
    finally
      FreeAndNil(J);
      FreeAndNil(O);
    end;
  except
    // Should be OK.
  end;
end;

procedure TTestArray.TestCreateNilPointer;

Var
  J : TJSONArray;
  P : Pointer;
  
begin
  J:=Nil;
  P:=Nil;
  Try
    J:=TJSONArray.Create([P]);
    TestJSONType(J[0],jtNull);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestCreatePointer;

Var
  J : TJSONArray;
  P : Pointer;
  
begin
  J:=Nil;
  P:=@Self;
  try
    Try
      J:=TJSONArray.Create([P]);
      Fail('Array constructor accepts only NIL pointers');
    finally
      FreeAndNil(J);
    end;
  except
    // Should be OK.
  end;
end;

procedure TTestArray.TestAddInteger;

Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    J.Add(Integer(0));
    TestItemCount(J,1);
    TestJSONType(J[0],jtNumber);
    AssertEquals('J[0] is TJSONIntegerNumber',J[0].ClassType,TJSONIntegerNumber);
    AssertEquals('j.Types[0]=jtNumber',ord(J.Types[0]),Ord(jtNumber));
    AssertEquals('J.Integers[0]=0',0,J.integers[0]);
    TestAsInteger(J[0],0);
    TestAsInt64(J[0],0);
    TestAsQword(J[0],0);
    TestJSON(J,'[0]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestAddInt64;

Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    J.Add(Int64(0));
    TestItemCount(J,1);
    TestJSONType(J[0],jtNumber);
    AssertEquals('J[0] is TJSONInt64Number',J[0].ClassType,TJSONInt64Number);
    AssertEquals('j.Types[0]=jtNumber',ord(J.Types[0]),Ord(jtNumber));
    AssertEquals('J.Int64s[0]=0',0,J.Int64s[0]);
    TestAsInteger(J[0],0);
    TestAsInt64(J[0],0);
    TestAsQword(J[0],0);
    TestJSON(J,'[0]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestAddFloat;

Var
  J : TJSONArray;
  S : String;
  F : TJSONFloat;
begin
  F:=1.2;
  J:=TJSonArray.Create;
  try
    J.Add(F);
    TestItemCount(J,1);
    TestJSONType(J[0],jtNumber);
    AssertEquals('J[0] is TJSONFloatNumber',TJSONfloatNumber,J[0].ClassType);
    AssertEquals('j.Types[0]=jtNumber',Ord(jtNumber),ord(J.Types[0]));
    AssertEquals('J.Floats[0]='+FloatToStr(F),F,J.Floats[0]);
    TestAsFloat(J[0],F);
    Str(F,S);
    Delete(S,1,1);
    TestJSON(J,'['+S+']');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestAddBoolean(B : Boolean);

Var
  J : TJSONArray;

begin
  B:=True;
  J:=TJSonArray.Create;
  try
    J.Add(B);
    TestItemCount(J,1);
    TestJSONType(J[0],jtBoolean);
    AssertEquals('J[0] is TJSONBoolean',TJSONBoolean,J[0].ClassType);
    TestAsBoolean(J[0],B);
    AssertEquals('J.Booleans[0]='+BoolToStr(B)+'"',B,J.Booleans[0]);
    If B then
      TestJSON(J,'[true]')
    else
      TestJSON(J,'[false]');
  finally
    FreeAndNil(J);
  end;

end;

procedure TTestArray.TestInsertBoolean(B: Boolean);
Var
  J : TJSONArray;

begin
  B:=True;
  J:=TJSonArray.Create;
  try
    J.Add(Not B);
    J.Insert(0,B);
    TestItemCount(J,2);
    TestJSONType(J[0],jtBoolean);
    AssertEquals('J[0] is TJSONBoolean',TJSONBoolean,J[0].ClassType);
    TestAsBoolean(J[0],B);
    AssertEquals('J.Booleans[0]='+BoolToStr(B)+'"',B,J.Booleans[0]);
    If B then
      TestJSON(J,'[true, false]')
    else
      TestJSON(J,'[false, true]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestAddBooleanTrue;

begin
  TestAddBoolean(True);
end;

procedure TTestArray.TestAddBooleanFalse;

begin
  TestAddBoolean(False);
end;

procedure TTestArray.TestAddString;

Var
  J : TJSONArray;
  S : String;
  
begin
  S:='A string';
  J:=TJSonArray.Create;
  try
    J.Add(S);
    TestItemCount(J,1);
    TestJSONType(J[0],jtString);
    AssertEquals('J[0] is TJSONString',TJSONString,J[0].ClassType);
    TestAsString(J[0],S);
    AssertEquals('J.Strings[0]="'+S+'"',S,J.Strings[0]);
    TestJSON(J,'["'+StringToJSONString(S)+'"]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestAddNull;

Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    J.Add;
    TestItemCount(J,1);
    TestJSONType(J[0],jtNull);
    AssertEquals('J[0] is TJSONNull',TJSONNull,J[0].ClassType);
    AssertEquals('J.Nulls[0]=True',True,J.Nulls[0]);
    TestIsNull(J[0],true);
    TestJSON(J,'[null]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestAddArray;

Var
  J,J2 : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    J2:=TJSonArray.Create;
    J2.Add(0);
    J2.Add(1);
    J.Add(J2);
    TestItemCount(J,1);
    TestJSONType(J[0],jtArray);
    AssertEquals('J[0] is TJSONArray',TJSONArray,J[0].ClassType);
    AssertEquals('J.Arrays[0] is TJSONArray',TJSONArray,J.Arrays[0].ClassType);
    TestAsInteger(J.Arrays[0][0],0);
    TestAsInteger(J.Arrays[0][1],1);
    TestAsInt64(J.Arrays[0][0],0);
    TestAsInt64(J.Arrays[0][1],1);
    TestAsQword(J.Arrays[0][0],0);
    TestAsQword(J.Arrays[0][1],1);
    TestJSON(J,'[[0, 1]]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestInsertInteger;
Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    J.Add(Integer(1));
    J.Insert(0,Integer(0));
    TestItemCount(J,2);
    TestJSONType(J[0],jtNumber);
    AssertEquals('J[0] is TJSONIntegerNumber',J[0].ClassType,TJSONIntegerNumber);
    AssertEquals('j.Types[0]=jtNumber',ord(J.Types[0]),Ord(jtNumber));
    AssertEquals('J.Integers[0]=0',0,J.integers[0]);
    TestAsInteger(J[0],0);
    TestAsInt64(J[0],0);
    TestAsQword(J[0],0);
    TestJSON(J,'[0, 1]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestInsertInt64;
Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    J.Add(Int64(1));
    J.Insert(0,Int64(0));
    TestItemCount(J,2);
    TestJSONType(J[0],jtNumber);
    AssertEquals('J[0] is TJSONInt64Number',J[0].ClassType,TJSONInt64Number);
    AssertEquals('j.Types[0]=jtNumber',ord(J.Types[0]),Ord(jtNumber));
    AssertEquals('J.Int64s[0]=0',0,J.Int64s[0]);
    TestAsInteger(J[0],0);
    TestAsInt64(J[0],0);
    TestAsQword(J[0],0);
    TestJSON(J,'[0, 1]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestInsertFloat;
Var
  J : TJSONArray;
  S,S2 : String;
  F : TJSONFloat;
begin
  F:=1.2;
  J:=TJSonArray.Create;
  try
    J.Add(2.3);
    J.Insert(0,F);
    TestItemCount(J,2);
    TestJSONType(J[0],jtNumber);
    AssertEquals('J[0] is TJSONFloatNumber',TJSONfloatNumber,J[0].ClassType);
    AssertEquals('j.Types[0]=jtNumber',Ord(jtNumber),ord(J.Types[0]));
    AssertEquals('J.Floats[0]='+FloatToStr(F),F,J.Floats[0]);
    TestAsFloat(J[0],F);
    Str(F,S);
    Delete(S,1,1);
    F:=2.3;
    Str(F,S2);
    Delete(S2,1,1);
    TestJSON(J,'['+S+', '+S2+']');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestInsertBooleanTrue;
begin
  TestInsertBoolean(True);
end;

procedure TTestArray.TestInsertBooleanFalse;
begin
  TestInsertBoolean(False);
end;

procedure TTestArray.TestInsertString;

Var
  J : TJSONArray;
  S : String;

begin
  S:='A string';
  J:=TJSonArray.Create;
  try
    J.Add('Another string');
    J.Insert(0,S);
    TestItemCount(J,2);
    TestJSONType(J[0],jtString);
    AssertEquals('J[0] is TJSONString',TJSONString,J[0].ClassType);
    TestAsString(J[0],S);
    AssertEquals('J.Strings[0]="'+S+'"',S,J.Strings[0]);
    TestJSON(J,'["'+StringToJSONString(S)+'", "Another string"]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestInsertNull;
Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    J.Add(123);
    J.Insert(0);
    TestItemCount(J,2);
    TestJSONType(J[0],jtNull);
    AssertEquals('J[0] is TJSONNull',TJSONNull,J[0].ClassType);
    AssertEquals('J.Nulls[0]=True',True,J.Nulls[0]);
    TestIsNull(J[0],true);
    TestJSON(J,'[null, 123]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestInsertObject;
Const
  A = 'a';
  B = 'b';

Var
  J : TJSONArray;
  J2 : TJSONObject;

begin
  J:=TJSonArray.Create;
  try
    J.Add('A string');
    J2:=TJSonObject.Create;
    J2.Add(A,0);
    J2.Add(B,1);
    J.Insert(0,J2);
    TestItemCount(J,2);
    TestJSONType(J[0],jtObject);
    AssertEquals('J[0] is TJSONObject',TJSONObject,J[0].ClassType);
    AssertEquals('J.Objects[0] is TJSONObject',TJSONObject,J.Objects[0].ClassType);
    TestAsInteger(J.Objects[0][A],0);
    TestAsInteger(J.Objects[0][B],1);
    TestAsInt64(J.Objects[0][A],0);
    TestAsInt64(J.Objects[0][B],1);
    TestAsQword(J.Objects[0][A],0);
    TestAsQword(J.Objects[0][B],1);
    TestJSON(J,'[{ "a" : 0, "b" : 1 }, "A string"]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestInsertArray;
Var
  J,J2 : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    J.Add('Something nice');
    J2:=TJSonArray.Create;
    J2.Add(0);
    J2.Add(1);
    J.Insert(0,J2);
    TestItemCount(J,2);
    TestJSONType(J[0],jtArray);
    AssertEquals('J[0] is TJSONArray',TJSONArray,J[0].ClassType);
    AssertEquals('J.Arrays[0] is TJSONArray',TJSONArray,J.Arrays[0].ClassType);
    TestAsInteger(J.Arrays[0][0],0);
    TestAsInteger(J.Arrays[0][1],1);
    TestAsInt64(J.Arrays[0][0],0);
    TestAsInt64(J.Arrays[0][1],1);
    TestAsQWord(J.Arrays[0][0],0);
    TestAsQWord(J.Arrays[0][1],1);
    TestJSON(J,'[[0, 1], "Something nice"]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestMove;
Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    J.Add('First string');
    J.Add('Second string');
    J.Add('Third string');
    J.Move(2,1);
    TestItemCount(J,3);
    AssertEquals('J[2] is TJSONString',TJSONString,J[1].ClassType);
    AssertEquals('J[1] is TJSONString',TJSONString,J[2].ClassType);
    TestAsString(J[1],'Third string');
    TestAsString(J[2],'Second string');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestExchange;
Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    J.Add('First string');
    J.Add('Second string');
    J.Add('Third string');
    J.Exchange(2,0);
    TestItemCount(J,3);
    AssertEquals('J[2] is TJSONString',TJSONString,J[0].ClassType);
    AssertEquals('J[1] is TJSONString',TJSONString,J[2].ClassType);
    TestAsString(J[0],'Third string');
    TestAsString(J[2],'First string');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestAddObject;

Const
  A = 'a';
  B = 'b';
  
Var
  J : TJSONArray;
  J2 : TJSONObject;

begin
  J:=TJSonArray.Create;
  try
    J2:=TJSonObject.Create;
    J2.Add(A,0);
    J2.Add(B,1);
    J.Add(J2);
    TestItemCount(J,1);
    TestJSONType(J[0],jtObject);
    AssertEquals('J[0] is TJSONObject',TJSONObject,J[0].ClassType);
    AssertEquals('J.Objects[0] is TJSONObject',TJSONObject,J.Objects[0].ClassType);
    TestAsInteger(J.Objects[0][A],0);
    TestAsInteger(J.Objects[0][B],1);
    TestAsInt64(J.Objects[0][A],0);
    TestAsInt64(J.Objects[0][B],1);
    TestAsQword(J.Objects[0][A],0);
    TestAsQword(J.Objects[0][B],1);
    TestJSON(J,'[{ "a" : 0, "b" : 1 }]');
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestDelete;

Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    J.Add(0);
    J.Add(1);
    TestItemCount(J,2);
    TestJSONType(J[0],jtNumber);
    TestJSONType(J[1],jtNumber);
    TestJSON(J,'[0, 1]');
    J.Delete(1);
    TestItemCount(J,1);
    J.Delete(0);
    TestItemCount(J,0);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestRemove;

Var
  J : TJSONArray;
  I : TJSONData;

begin
  J:=TJSonArray.Create;
  try
    J.Add(0);
    J.Add(1);
    J.Add(2);
    TestItemCount(J,3);
    TestJSONType(J[0],jtNumber);
    TestJSONType(J[1],jtNumber);
    TestJSONType(J[2],jtNumber);
    TestJSON(J,'[0, 1, 2]');
    I:=J[1];
    J.Remove(I);
    TestItemCount(J,2);
    TestAsInteger(J[0],0);
    TestAsInteger(J[1],2);
    TestAsInt64(J[0],0);
    TestAsInt64(J[1],2);
    TestAsQWord(J[0],0);
    TestAsQWord(J[1],2);
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestClone;

Var
  J,J2 : TJSONArray;
  D : TJSONData;

begin
  J:=TJSonArray.Create;
  try
    J.Add(1);
    J.Add('aloha');
    D:=J.Clone;
    try
      TestJSONType(D,jtArray);
      J2:=TJSonArray(D);
      TestItemCount(J2,2);
      TestJSONType(J2[0],jtNumber);
      TestJSONType(J2[1],jtString);
      TestAsInteger(J2[0],1);
      TestAsString(J2[1],'aloha');
    finally
      D.Free;
    end;
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestMyClone;
Var
  J : TMyArray;
  D : TJSONData;

begin
  J:=TMyArray.Create;
  try
    J.Add(1);
    J.Add('aloha');
    D:=J.Clone;
    try
      TestJSONType(D,jtArray);
      AssertEquals('Correct class',TMyArray,D.ClassType);
    finally
      D.Free;
    end;
  finally
    FreeAndNil(J);
  end;
end;

procedure TTestArray.TestFormat;
Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    J.Add(0);
    J.Add(1);
    J.Add(2);
    TestItemCount(J,3);
    TestJSONType(J[0],jtNumber);
    TestJSONType(J[1],jtNumber);
    TestJSONType(J[2],jtNumber);
    TestJSON(J,'[0, 1, 2]');
    AssertEquals('FormatJSON, single line',J.AsJSON,J.FormatJSON([foSingleLineArray],1));
    AssertEquals('FormatJSON, default','['+sLinebreak+'  0,'+sLinebreak+'  1,'+sLinebreak+'  2'+sLinebreak+']',J.FormatJSON());
    AssertEquals('FormatJSON, use tab','['+sLinebreak+#9'0,'+sLinebreak+#9'1,'+sLinebreak+#9'2'+sLinebreak+']',J.FormatJSON([foUseTabChar],1));
    J.Add(TJSONObject.Create(['x',1,'y',2]));
    AssertEquals('FormatJSON, use tab indentsize 1','['+sLinebreak+#9'0,'+sLinebreak+#9'1,'+sLinebreak+#9'2,'+sLinebreak+#9'{'+sLineBreak+#9#9'"x" : 1,'+sLineBreak+#9#9'"y" : 2'+sLinebreak+#9'}'+sLineBreak+']',J.FormatJSON([foUseTabChar],1));
  finally
    J.Free
  end;
end;

procedure TTestArray.TestFormatNil;

Var
  J : TJSONArray;

begin
  J:=TJSonArray.Create;
  try
    J.Add(1);
    J.Add(TJSONObject(Nil));
    TestJSON(J,'[1, null]');
    AssertEquals('FormatJSON, single line',J.AsJSON,J.FormatJSON([foSingleLineArray],1));
  finally
    J.Free;
  end;
end;

{ TTestObject }

procedure TTestObject.TestCreate;

begin
  TestJSONType(J,jtObject);
  TestItemCount(J,0);
  TestJSON(J,'{}');
  TestIsNull(J,False);
  TestAsBoolean(J,False,True);
  TestAsInteger(J,1,True);
  TestAsInt64(J,1,True);
  TestAsQword(J,1,True);
  TestAsString(J,'',True);
  TestAsFloat(J,0.0,True);
end;

procedure TTestObject.TestAddInteger;

Const
  A = 'a';

begin
  J.Add(A,Integer(0));
  TestItemCount(J,1);
  TestJSONType(J[A],jtNumber);
  AssertEquals('J[''a''] is TJSONIntegerNumber',J[A].ClassType,TJSONIntegerNumber);
  AssertEquals('j.Types[''a'']=jtNumber',ord(J.Types[A]),Ord(jtNumber));
  AssertEquals('J.Integers[''a'']=0',0,J.integers[A]);
  TestAsInteger(J[A],0);
  TestAsInt64(J[A],0);
  TestAsQword(J[A],0);
  TestJSON(J,'{ "'+A+'" : 0 }');
end;

procedure TTestObject.TestAddInt64;

Const
  A = 'a';

begin
  J.Add(A,Int64(0));
  TestItemCount(J,1);
  TestJSONType(J[A],jtNumber);
  AssertEquals('J[''a''] is TJSONInt64Number',J[A].ClassType,TJSONInt64Number);
  AssertEquals('j.Types[''a'']=jtNumber',ord(J.Types[A]),Ord(jtNumber));
  AssertEquals('J.Int64s[''a'']=0',0,J.Int64s[A]);
  TestAsInteger(J[A],0);
  TestAsInt64(J[A],0);
  TestAsQword(J[A],0);
  TestJSON(J,'{ "'+A+'" : 0 }');
end;

procedure TTestObject.TestAddFloat;

Const
  A = 'a';

Var
  S : String;
  F : TJSONFloat;
begin
  F:=1.2;
  J.Add(A,F);
  TestItemCount(J,1);
  TestJSONType(J[A],jtNumber);
  AssertEquals('J[''a''] is TJSONFloatNumber',TJSONfloatNumber,J[a].ClassType);
  AssertEquals('j.Types[''a'']=jtNumber',Ord(jtNumber),ord(J.Types[a]));
  AssertEquals('J.Floats[''a'']='+FloatToStr(F),F,J.Floats[a]);
  TestAsFloat(J[A],F);
  Str(F,S);
  TestJSON(J,'{ "'+a+'" :'+S+' }');
end;

procedure TTestObject.Setup;
begin
  inherited Setup;
  FJ:=TJSONObject.Create;
end;

procedure TTestObject.TearDown;
begin
  FreeAndNil(FJ);
  inherited TearDown;
end;

procedure TTestObject.TestAddBoolean(B : Boolean);

Const
  A = 'a';
  
begin
  B:=True;
  J.Add(A,B);
  TestItemCount(J,1);
  TestJSONType(J[A],jtBoolean);
  AssertEquals('J[''a''] is TJSONBoolean',TJSONBoolean,J[a].ClassType);
  TestAsBoolean(J[a],B);
  AssertEquals('J.Booleans[''a'']='+BoolToStr(B)+'"',B,J.Booleans[a]);
  If B then
    TestJSON(J,'{ "'+a+'" : true }')
  else
    TestJSON(J,'{ "'+a+'" : false }');
end;

procedure TTestObject.TestAccessError;

begin
  J.Strings['NonExist'];
end;

procedure TTestObject.TestAddBooleanTrue;

begin
  TestAddBoolean(True);
end;

procedure TTestObject.TestAddBooleanFalse;

begin
  TestAddBoolean(False);
end;

procedure TTestObject.TestAddString;

Const
  A = 'a';

Var
  S : String;

begin
  S:='A string';
    J.Add(A,S);
    TestItemCount(J,1);
    TestJSONType(J[a],jtString);
    AssertEquals('J[''a''] is TJSONString',TJSONString,J[A].ClassType);
    TestAsString(J[a],S);
    AssertEquals('J.Strings[''a'']="'+S+'"',S,J.Strings[A]);
    TestJSON(J,'{ "'+a+'" : "'+StringToJSONString(S)+'" }');
end;

procedure TTestObject.TestAddNull;

Const
  A = 'a';

begin
  J.Add(a);
  TestItemCount(J,1);
  TestJSONType(J[a],jtNull);
  AssertEquals('J[''a''] is TJSONNull',TJSONNull,J[A].ClassType);
  AssertEquals('J.Nulls[''a'']=True',True,J.Nulls[A]);
  TestIsNull(J[a],true);
  TestJSON(J,'{ "'+a+'" : null }');
end;

procedure TTestObject.TestAddObject;

Const
  A = 'a';
  B = 'b';
  C = 'c';

Var
  J2 : TJSONObject;

begin
  J2:=TJSonObject.Create;
  J2.Add(B,0);
  J2.Add(C,1);
  J.Add(A,J2);
  TestItemCount(J,1);
  TestJSONType(J[A],jtObject);
  AssertEquals('J[''a''] is TJSONObject',TJSONObject,J[A].ClassType);
  AssertEquals('J.Objects[''a''] is TJSONObject',TJSONObject,J.Objects[A].ClassType);
  TestAsInteger(J.Objects[A][B],0);
  TestAsInteger(J.Objects[A][C],1);
  TestAsInt64(J.Objects[A][B],0);
  TestAsInt64(J.Objects[A][C],1);
  TestAsQword(J.Objects[A][B],0);
  TestAsQword(J.Objects[A][C],1);
  TestJSON(J,'{ "a" : { "b" : 0, "c" : 1 } }');
end;

procedure TTestObject.TestAddArray;

Const
  A = 'a';

Var
  J2 : TJSONArray;

begin
  J2:=TJSonArray.Create;
  J2.Add(0);
  J2.Add(1);
  J.Add(A,J2);
  TestItemCount(J,1);
  TestJSONType(J[A],jtArray);
  AssertEquals('J[''a''] is TJSONArray',TJSONArray,J[A].ClassType);
  AssertEquals('J.Arrays[0] is TJSONArray',TJSONArray,J.Arrays[A].ClassType);
  TestAsInteger(J.Arrays[A][0],0);
  TestAsInteger(J.Arrays[A][1],1);
  TestAsInt64(J.Arrays[A][0],0);
  TestAsInt64(J.Arrays[A][1],1);
  TestAsQword(J.Arrays[A][0],0);
  TestAsQword(J.Arrays[A][1],1);
  TestJSON(J,'{ "a" : [0, 1] }');
end;

procedure TTestObject.TestDelete;

Const
  A = 'a';
  B = 'b';
  
begin
  J.Add(A,0);
  J.Add(B,1);
  TestItemCount(J,2);
  TestJSONType(J[A],jtNumber);
  TestJSONType(J[A],jtNumber);
  TestJSON(J,'{ "a" : 0, "b" : 1 }');
  J.Delete(1);
  TestItemCount(J,1);
  J.Delete(0);
  TestItemCount(J,0);
end;

procedure TTestObject.TestRemove;

Const
  A = 'a';
  B = 'b';
  C = 'c';
  
Var
  I : TJSONData;

begin
  J.Add(A,1);
  J.Add(B,2);
  J.Add(C,3);
  TestItemCount(J,3);
  TestJSONType(J[A],jtNumber);
  TestJSONType(J[B],jtNumber);
  TestJSONType(J[C],jtNumber);
  TestJSON(J,'{ "a" : 1, "b" : 2, "c" : 3 }');
  I:=J[b];
  J.Remove(I);
  TestItemCount(J,2);
  TestAsInteger(J[a],1);
  TestAsInteger(J[c],3);
  TestAsInt64(J[a],1);
  TestAsInt64(J[c],3);
  TestAsQword(J[a],1);
  TestAsQword(J[c],3);
end;

procedure TTestObject.TestClone;

Var
  J2 : TJSONObject;
  D : TJSONData;

begin
  J.Add('p1',1);
  J.Add('p2','aloha');
  D:=J.Clone;
  try
    TestJSONType(D,jtObject);
    J2:=TJSonObject(D);
    TestItemCount(J2,2);
    TestJSONType(J2['p1'],jtNumber);
    TestJSONType(J2['p2'],jtString);
    TestAsInteger(J2['p1'],1);
    TestAsString(J2['p2'],'aloha');
  finally
    D.Free;
  end;
end;

procedure TTestObject.TestMyClone;

Var
  D : TJSONData;
  O : TMyObject;

begin
  D:=Nil;
  O:=TMyObject.Create;
  try
    O.Add('p1',1);
    O.Add('p2','aloha');
    D:=O.Clone;
    TestJSONType(D,jtObject);
    AssertEquals('Correct class',TMYObject,D.ClassType);
  finally
    D.Free;
    O.Free;
  end;
end;

procedure TTestObject.TestExtract;

Const
  A = 'a';
  B = 'b';

Var
  JA,JB : TJSONData;
  E : TJSONData;

begin
  J.Add(A,0);
  J.Add(B,1);
  TestItemCount(J,2);
  JA:=J[A];
  JB:=J[B];
  TestJSONType(JA,jtNumber);
  TestJSONType(JB,jtNumber);
  TestJSON(J,'{ "a" : 0, "b" : 1 }');
  E:=J.Extract(1);
  AssertSame('Extracted JA',JB,E);
  E.Free;
  TestItemCount(J,1);
  E:=J.Extract(0);
  AssertSame('Extracted JB',JA,E);
  E.Free;
  TestItemCount(J,0);
end;

procedure TTestObject.TestNonExistingAccessError;
begin
  AssertException(EJSON,@TestAccessError);
end;

procedure TTestObject.TestFormat;

Var
  O : TJSONObject;

begin
  O:=TJSONObject.Create(['x',1,'y',2]);
  try
    TestJSON(O,'{ "x" : 1, "y" : 2 }');
    AssertEquals('Format equals JSON',O.AsJSON,O.FormatJSON([foSingleLineObject]));
    AssertEquals('Format using SkipWhiteSpace','{"x":1,"y":2}',O.FormatJSON([foSingleLineObject,foSkipWhiteSpace]));
    AssertEquals('Format using SkipWhiteSpace,foSkipWhiteSpaceOnlyLeading','{"x": 1,"y": 2}',O.FormatJSON([foSingleLineObject,foSkipWhiteSpace,foSkipWhiteSpaceOnlyLeading]));
    AssertEquals('Format using SkipWhiteSpace,unquotednames','{x:1,y:2}',O.FormatJSON([foSingleLineObject,foSkipWhiteSpace,foDoNotQuoteMembers]));
    AssertEquals('Format []','{'+sLineBreak+'  "x" : 1,'+sLineBreak+'  "y" : 2'+sLineBreak+'}',O.FormatJSON([]));
    AssertEquals('Format [foDoNotQuoteMembers]','{'+sLineBreak+'  x : 1,'+sLineBreak+'  y : 2'+sLineBreak+'}',O.FormatJSON([foDoNotQuoteMembers]));
    AssertEquals('Format [foUseTabChar,foDoNotQuoteMembers]','{'+sLineBreak+#9'x : 1,'+sLineBreak+#9'y : 2'+sLineBreak+'}',O.FormatJSON([foUseTabChar,foDoNotQuoteMembers],1));
    O.Add('s',TJSONObject.Create(['w',10,'h',20]));
    AssertEquals('Format [foUseTabChar,foDoNotQuoteMembers] 2','{'+sLineBreak+#9'x : 1,'+sLineBreak+#9'y : 2,'+sLineBreak+#9's : {'+sLineBreak+#9#9'w : 10,'+sLineBreak+#9#9'h : 20'+sLineBreak+#9'}'+sLineBreak+'}',O.FormatJSON([foUseTabChar,foDoNotQuoteMembers],1));
  finally
    O.Free;
  end;
end;

procedure TTestObject.TestFormatNil;

begin
  J.Add('a',1);
  J.Add('b',TJSONObject(Nil));
  TestJSON(J,'{ "a" : 1, "b" : null }');
  AssertEquals('FormatJSON, single line',J.AsJSON,J.FormatJSON([foSingleLineObject],1));
end;

procedure TTestObject.TestFind;

Const
  A = 'A';
  S = 'A string';
  B = 'a';
  S2 = 'Another string';
  C = 'c';
  S3 = 'Yet Another string';

begin
  J.Add(A,S);
  J.Add(B,S2);
  J.Add(C,S3);
  TestJSONType(J,jtObject);
  TestIsNull(J,False);
  TestItemCount(J,3);
  TestJSONType(J[A],jtString);
  TestJSONType(J[B],jtString);
  TestJSON(J,'{ "A" : "'+S+'", "a" : "'+S2+'", "c" : "'+S3+'" }');
  AssertEquals('Nonexisting, case sensitive',-1,J.IndexOfName('D'));
  AssertEquals('Nonexisting, case insensitive',-1,J.IndexOfName('D',True));
  AssertEquals('1 Existing , case sensitive',0,J.IndexOfName(A));
  AssertEquals('2 Existing exact match, case insensitive',0,J.IndexOfName(A,true));
  AssertEquals('3 Existing , case sensitive',1,J.IndexOfName(B));
  AssertEquals('4 Existing exact match, case insensitive',1,J.IndexOfName(B,true));
  AssertEquals('5 Existing , case sensitive again',2,J.IndexOfName(C));
  AssertEquals('6 Existing case-insensitive match, case insensitive',2,J.IndexOfName(Uppercase(C),true));
end;

Procedure TTestObject.TestIfFind;
Var
  B: TJSONBoolean;
  S: TJSONString;
  N: TJSONNumber;
  D: TJSONData;
begin
  J.Add('s', 'astring');
  J.Add('b', true);
  J.Add('n', 1);
  TestJSONType(J,jtObject);
  TestIsNull(J,False);
  TestItemCount(J,3);
  AssertEquals('boolean found', true, j.Find('b', B));
  AssertEquals('string found', true, j.Find('s', S));
  AssertEquals('number found', true, j.Find('n', N));
  AssertEquals('data found', true, j.Find('s', D));
end;

procedure TTestObject.AppendA;

begin
  J.Add('A','S')
end;

procedure TTestObject.TestDuplicate;

begin
  J.Add('A',TJSONObject.Create);
  AssertException(EJSON,@AppendA);
end;


procedure TTestObject.TestCreateString;

Const
  A = 'A';
  S = 'A string';

begin
  J.Add(A,S);
  TestJSONType(J,jtObject);
  TestItemCount(J,1);
  TestJSONType(J[A],jtString);
  TestJSON(J,'{ "A" : "'+S+'" }');
  TestIsNull(J,False);
end;

procedure TTestObject.TestCreateStringUnquoted;

Const
  A = 'A';
  S = 'A string';

begin
  TJSONObject.UnquotedMemberNames:=True;
  J.Add(A,S);
  TestJSONType(J,jtObject);
  TestItemCount(J,1);
  TestJSONType(J[A],jtString);
  TestJSON(J,'{ A : "'+S+'" }');
  TestIsNull(J,False);
end;

procedure TTestObject.TestCreatePchar;

Const
  A = 'A';
  S = 'A string';

Var
  O : TJSONObject;

begin
  O:=TJSONObject.Create([A,Pchar(S)]);
  try
    TestJSONType(O,jtObject);
    TestItemCount(O,1);
    TestJSONType(O[A],jtString);
    TestJSON(O,'{ "A" : "'+S+'" }');
    TestIsNull(O,False);
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreatePcharUnquoted;

Const
  A = 'A';
  S = 'A string';

Var
  O : TJSONObject;

begin
  TJSONObject.UnQuotedMemberNames:=True;
  O:=TJSONObject.Create([A,Pchar(S)]);
  try
    TestJSONType(O,jtObject);
    TestItemCount(O,1);
    TestJSONType(O[A],jtString);
    TestJSON(O,'{ A : "'+S+'" }');
    TestIsNull(O,False);
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateStrings;

Const
  A = 'A';
  B = 'B';
  S = 'A string';
  T = 'B string';

Var
  O : TJSONObject;

begin
  O:=TJSONObject.Create([A,S,B,T]);
  try
    TestJSONType(O,jtObject);
    TestItemCount(O,2);
    TestJSONType(O[A],jtString);
    TestJSONType(O[B],jtString);
    TestJSON(O,'{ "A" : "'+S+'", "B" : "'+T+'" }');
    TestIsNull(O,False);
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateStringsCompressed;

Const
  A = 'A';
  B = 'B';
  S = 'A string';
  T = 'B string';

Var
  O : TJSONObject;

begin
  TJSONData.CompressedJSON:=True;
  O:=TJSONObject.Create([A,S,B,T]);
  try
    TestJSONType(O,jtObject);
    TestItemCount(O,2);
    TestJSONType(O[A],jtString);
    TestJSONType(O[B],jtString);
    TestJSON(O,'{"A":"'+S+'","B":"'+T+'"}');
    TestIsNull(O,False);
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateStringsCompressedUnquoted;

Const
  A = 'A';
  B = 'B';
  S = 'A string';
  T = 'B string';

Var
  O : TJSONObject;

begin
  TJSONData.CompressedJSON:=True;
  TJSONObject.UnQuotedMemberNames:=True;
  O:=TJSONObject.Create([A,S,B,T]);
  try
    TestJSONType(O,jtObject);
    TestItemCount(O,2);
    TestJSONType(O[A],jtString);
    TestJSONType(O[B],jtString);
    TestJSON(O,'{A:"'+S+'",B:"'+T+'"}');
    TestIsNull(O,False);
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateInteger;

Const
  A = 'A';
  S = 3;

Var
  O : TJSONObject;

begin
  O:=TJSONObject.Create([A,S]);
  try
    TestJSONType(O,jtObject);
    TestItemCount(O,1);
    TestJSONType(O[A],jtNumber);
    TestJSON(O,'{ "A" : 3 }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateIntegerUnquoted;
Const
  A = 'A';
  S = 3;

Var
  O : TJSONObject;

begin
  TJSONObject.UnQuotedMemberNames:=True;
  O:=TJSONObject.Create([A,S]);
  try
    TestJSONType(O,jtObject);
    TestItemCount(O,1);
    TestJSONType(O[A],jtNumber);
    TestJSON(O,'{ A : 3 }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateFloat;

Const
  A = 'A';
  S : double = 1.2;

Var
  O : TJSONObject;
  r : String;

begin
  O:=TJSONObject.Create([A,S]);
  try
    TestJSONType(O,jtObject);
    TestItemCount(O,1);
    TestJSONType(O[A],jtNumber);
    Str(S,R);
    TestJSON(O,'{ "A" :'+R+' }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateFloatUnquoted;
Const
  A = 'A';
  S : double = 1.2;

Var
  O : TJSONObject;
  r : String;

begin
  TJSONObject.UnQuotedMemberNames:=True;
  O:=TJSONObject.Create([A,S]);
  try
    TestJSONType(O,jtObject);
    TestItemCount(O,1);
    TestJSONType(O[A],jtNumber);
    Str(S,R);
    TestJSON(O,'{ A :'+R+' }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateInt64;

Const
  A = 'A';
  S : Int64 = $FFFFFFFFFFFFF;

Var
  O : TJSONObject;

begin
  O:=TJSONObject.Create([A,S]);
  try
    TestJSONType(O,jtObject);
    TestItemCount(O,1);
    TestJSONType(O[A],jtNumber);
    TestJSON(O,'{ "A" : '+IntToStr(S)+' }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateInt64Unquoted;
Const
  A = 'A';
  S : Int64 = $FFFFFFFFFFFFF;

Var
  O : TJSONObject;

begin
  TJSONObject.UnQuotedMemberNames:=True;
  O:=TJSONObject.Create([A,S]);
  try
    TestJSONType(O,jtObject);
    TestItemCount(O,1);
    TestJSONType(O[A],jtNumber);
    TestJSON(O,'{ A : '+IntToStr(S)+' }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateBoolean;

Const
  A = 'A';
  S = True;

Var
  O : TJSONObject;

begin
  O:=TJSONObject.Create([A,S]);
  try
    TestJSONType(O,jtObject);
    TestItemCount(O,1);
    TestJSONType(O[A],jtBoolean);
    TestJSON(O,'{ "A" : true }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateBooleanUnquoted;
Const
  A = 'A';
  S = True;

Var
  O : TJSONObject;

begin
  TJSONObject.UnQuotedMemberNames:=True;
  O:=TJSONObject.Create([A,S]);
  try
    TestJSONType(O,jtObject);
    TestItemCount(O,1);
    TestJSONType(O[A],jtBoolean);
    TestJSON(O,'{ A : true }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateJSONObject;

Const
  A = 'A';
  
Var
  O : TJSONObject;

begin
  O:=TJSONObject.Create([A,TJSONObject.Create]);
  try
    TestItemCount(O,1);
    TestJSONType(O[A],jtObject);
    TestJSON(O,'{ "A" : {} }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateJSONObjectUnquoted;
Const
  A = 'A';

Var
  O : TJSONObject;

begin
  TJSONObject.UnQuotedMemberNames:=True;
  O:=TJSONObject.Create([A,TJSONObject.Create]);
  try
    TestItemCount(O,1);
    TestJSONType(O[A],jtObject);
    TestJSON(O,'{ A : {} }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateJSONString;

Const
  A = 'A';
  S = 'A string';

Var
  O : TJSONObject;

begin
  O:=TJSONObject.Create([A,TJSONString.Create(S)]);
  try
    TestItemCount(O,1);
    TestJSONType(O[A],jtString);
    TestJSON(O,'{ "A" : "'+S+'" }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateJSONStringUnquoted;

Const
  A = 'A';
  S = 'A string';

Var
  O : TJSONObject;

begin
  TJSONObject.UnQuotedMemberNames:=True;
  O:=TJSONObject.Create([A,TJSONString.Create(S)]);
  try
    TestItemCount(O,1);
    TestJSONType(O[A],jtString);
    TestJSON(O,'{ A : "'+S+'" }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateObject;

Const
  A = 'A';

Var
  O : TJSONObject;
  OO : TObject;

begin
  O:=Nil;
  try
    Try
      OO:=TObject.Create;
      O:=TJSONObject.Create([A,OO]);
      Fail('Array constructor accepts only TJSONData');
    finally
      FreeAndNil(O);
      FreeAndNil(OO);
    end;
  except
    // Should be OK.
  end;
end;

procedure TTestObject.TestCreateJSONUnicodeString;
Const
  A = 'A';
  S : Unicodestring = 'A string';

Var
  O : TJSONObject;

begin
  O:=TJSONObject.Create([A,S]);
  try
    TestItemCount(O,1);
    TestJSONType(O[A],jtString);
    TestJSON(O,'{ "A" : "'+UTF8Encode(S)+'" }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateJSONWideString;
Const
  A = 'A';
  W : WideString = 'A string';

Var
  O : TJSONObject;

begin
  O:=TJSONObject.Create([A,W]);
  try
    TestItemCount(O,1);
    TestJSONType(O[A],jtString);
    TestJSON(O,'{ "A" : "'+UTF8Encode(W)+'" }');
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreateNilPointer;

Const
  A = 'A';

Var
  O : TJSONObject;
  P : Pointer;

begin
  O:=Nil;
  P:=Nil;
  Try
    O:=TJSONObject.Create([A,P]);
    TestJSONType(O[A],jtNull);
  finally
    FreeAndNil(O);
  end;
end;

procedure TTestObject.TestCreatePointer;

Const
  A = 'A';

Var
  O : TJSONObject;
  P : Pointer;

begin
  O:=Nil;
  P:=@Self;
  try
    Try
      O:=TJSONObject.Create([A,P]);
      Fail('Array constructor accepts only NIL pointers');
    finally
      FreeAndNil(O);
    end;
  except
    // Should be OK.
  end;
end;

{ TTestJSONString }

procedure TTestJSONString.TestTo(const Src, Dest: String; Strict : Boolean = False);

Var
  S : String;

begin
  S:='StringToJSONString('''+Src+''')='''+Dest+'''';
  AssertEquals(S,Dest,StringToJSONString(Src,Strict));
end;

procedure TTestJSONString.TestFrom(const Src, Dest: String);

Var
  S : String;

begin
  S:='JSONStringToString('''+Src+''')='''+Dest+'''';
  AssertEquals(S,Dest,JSONStringToString(Src));
end;

procedure TTestJSONString.TestJSONStringToString;

Const
  // Glowing star in UTF8
  GlowingStar = #$F0#$9F#$8C#$9F;
  Chinese = #$95e8#$88ab#$8111#$5b50#$6324#$574f#$4e86;
  Chinese4b = #$95e8#$d867#$de3d#$88ab#$8111#$5b50#$6324#$574f#$4e86;

begin
  TestFrom('','');
  TestFrom('A','A');
  TestFrom('AB','AB');
  TestFrom('ABC','ABC');
  TestFrom('\\','\');
  TestFrom('\/','/');
  TestFrom('\"','"');
  TestFrom('\b',#8);
  TestFrom('\t',#9);
  TestFrom('\n',#10);
  TestFrom('\f',#12);
  TestFrom('\r',#13);
  TestFrom('\bBC',#8'BC');
  TestFrom('\tBC',#9'BC');
  TestFrom('\nBC',#10'BC');
  TestFrom('\fBC',#12'BC');
  TestFrom('\rBC',#13'BC');
  TestFrom('A\b','A'#8);
  TestFrom('A\t','A'#9);
  TestFrom('A\n','A'#10);
  TestFrom('A\f','A'#12);
  TestFrom('A\r','A'#13);
  TestFrom('A\bBC','A'#8'BC');
  TestFrom('A\tBC','A'#9'BC');
  TestFrom('A\nBC','A'#10'BC');
  TestFrom('A\fBC','A'#12'BC');
  TestFrom('A\rBC','A'#13'BC');
  TestFrom('\\\\','\\');
  TestFrom('\/\/','//');
  TestFrom('\"\"','""');
  TestFrom('\b\b',#8#8);
  TestFrom('\t\t',#9#9);
  TestFrom('\n\n',#10#10);
  TestFrom('\f\f',#12#12);
  TestFrom('\r\r',#13#13);
  TestFrom('\u00f8','ø'); // this is ø
  TestFrom('\u00f8\"','ø"'); // this is ø"
  TestFrom('\ud83c\udf1f',GlowingStar);
  TestFrom('\u0041\u0042','AB');   //issue #0038622
  TestFrom('\u0041\u0042\u0043','ABC');
  TestFrom('\u0041\u0042\u0043\u0044','ABCD');
  TestFrom('\u95e8\u88ab\u8111\u5b50\u6324\u574f\u4e86',Utf8Encode(Chinese));
  TestFrom('\u95e8\ud867\ude3d\u88ab\u8111\u5b50\u6324\u574f\u4e86',Utf8Encode(Chinese4b));
end;

procedure TTestJSONString.TestStringToJSONString;
begin
  TestTo('','');
  TestTo('A','A');
  TestTo('AB','AB');
  TestTo('ABC','ABC');
  TestTo('\','\\');
  TestTo('/','/');
  TestTo('/','\/',True);
  TestTo('"','\"');
  TestTo(#8,'\b');
  TestTo(#9,'\t');
  TestTo(#10,'\n');
  TestTo(#12,'\f');
  TestTo(#13,'\r');
  TestTo(#8'BC','\bBC');
  TestTo(#9'BC','\tBC');
  TestTo(#10'BC','\nBC');
  TestTo(#12'BC','\fBC');
  TestTo(#13'BC','\rBC');
  TestTo('A'#8,'A\b');
  TestTo('A'#9,'A\t');
  TestTo('A'#10,'A\n');
  TestTo('A'#12,'A\f');
  TestTo('A'#13,'A\r');
  TestTo('A'#8'BC','A\bBC');
  TestTo('A'#9'BC','A\tBC');
  TestTo('A'#10'BC','A\nBC');
  TestTo('A'#12'BC','A\fBC');
  TestTo('A'#13'BC','A\rBC');
  TestTo('\\','\\\\');
  TestTo('//','//');
  TestTo('//','\/\/',true);
  TestTo('""','\"\"');
  TestTo(#8#8,'\b\b');
  TestTo(#9#9,'\t\t');
  TestTo(#10#10,'\n\n');
  TestTo(#12#12,'\f\f');
  TestTo(#13#13,'\r\r');
end;

initialization
  RegisterTest(TTestJSONString);
  RegisterTest(TTestNull);
  RegisterTest(TTestBoolean);
  RegisterTest(TTestInteger);
  RegisterTest(TTestInt64);
  RegisterTest(TTestQWord);
  RegisterTest(TTestFloat);
  RegisterTest(TTestString);
  RegisterTest(TTestArray);
  RegisterTest(TTestObject);
  RegisterTest(TTestJSONPath);
  RegisterTest(TTestFactory);
  RegisterTest(TTestIterator);
end.

