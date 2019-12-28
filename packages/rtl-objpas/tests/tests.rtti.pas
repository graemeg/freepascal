unit tests.rtti;

{$ifdef fpc}
{$mode objfpc}{$H+}
{$modeswitch advancedrecords}
{$endif}

interface

uses
{$IFDEF FPC}
  fpcunit,testregistry, testutils,
{$ELSE FPC}
  TestFramework,
{$ENDIF FPC}
  Classes, SysUtils, typinfo,
  Rtti;

type

  { TTestCase1 }

  TTestCase1= class(TTestCase)
  published
    //procedure GetTypes;
    procedure GetTypeInteger;
    procedure GetTypePointer;
    procedure GetClassProperties;

    procedure GetClassPropertiesValue;

    procedure TestTRttiTypeProperties;
    procedure TestPropGetValueString;
    procedure TestPropGetValueInteger;
    procedure TestPropGetValueBoolean;
    procedure TestPropGetValueShortString;
    procedure TestPropGetValueProcString;
    procedure TestPropGetValueProcInteger;
    procedure TestPropGetValueProcBoolean;
    procedure TestPropGetValueProcShortString;
    procedure TestPropGetValueObject;
    procedure TestPropGetValueInterface;
    procedure TestPropGetValueFloat;
    procedure TestPropGetValueDynArray;
    procedure TestPropGetValueEnumeration;
    procedure TestPropGetValueChars;

    procedure TestPropSetValueString;
    procedure TestPropSetValueInteger;
    procedure TestPropSetValueBoolean;
    procedure TestPropSetValueShortString;
    procedure TestPropSetValueObject;
    procedure TestPropSetValueInterface;
    procedure TestPropSetValueFloat;
    procedure TestPropSetValueDynArray;
    procedure TestPropSetValueEnumeration;
    procedure TestPropSetValueChars;

    procedure TestGetValueStringCastError;
    procedure TestGetIsReadable;
    procedure TestIsWritable;

    procedure TestIsType;

    procedure TestMakeNil;
    procedure TestMakeObject;
    procedure TestMakeArrayDynamic;
    procedure TestMakeArrayStatic;
{$ifdef fpc}
    procedure TestMakeArrayOpen;
{$endif}
    procedure TestMakeSingle;
    procedure TestMakeDouble;
    procedure TestMakeExtended;
    procedure TestMakeCurrency;
    procedure TestMakeComp;
    procedure TestMakeEnum;
    procedure TestMakeAnsiChar;
    procedure TestMakeWideChar;

    procedure TestFromOrdinal;

    procedure TestDataSize;
    procedure TestDataSizeEmpty;
    procedure TestReferenceRawData;
    procedure TestReferenceRawDataEmpty;

    procedure TestIsManaged;
{$ifdef fpc}
    procedure TestOpenArrayToDyn;
{$endif}

    procedure TestInterface;
{$ifdef fpc}
    procedure TestInterfaceRaw;
{$endif}

    procedure TestProcVar;
    procedure TestMethod;

    procedure TestRawThunk;
  private
    procedure MakeFromOrdinalTObject;
    procedure MakeFromOrdinalSet;
    procedure MakeFromOrdinalString;
    procedure MakeFromOrdinalNil;
  end;

implementation

uses
  Tests.Rtti.Util;

type

  {$M+}
  TGetClassProperties = class
  private
    FPubPropRO: integer;
    FPubPropRW: integer;
  published
    property PubPropRO: integer read FPubPropRO;
    property PubPropRW: integer read FPubPropRW write FPubPropRW;
    property PubPropSetRO: integer read FPubPropRO;
    property PubPropSetRW: integer read FPubPropRW write FPubPropRW;
  end;

  TGetClassPropertiesSub = class(TGetClassProperties)

  end;

  TTestDynArray = array of Integer;
  TTestEnumeration = (en1, en2, en3, en4);
  {$M-}

  { TTestValueClass }

  {$M+}
  TTestValueClass = class
  private
    FAArray: TTestDynArray;
    FAChar: AnsiChar;
    FAComp: Comp;
    FACurrency: Currency;
    FADouble: Double;
    FAEnumeration: TTestEnumeration;
    FAExtended: Extended;
    FAInteger: integer;
    FAObject: TObject;
    FASingle: Single;
    FAString: string;
    FABoolean: boolean;
    FAShortString: ShortString;
    FAUnknown: IUnknown;
    FAWideChar: WideChar;
    function GetAInteger: integer;
    function GetAString: string;
    function GetABoolean: boolean;
    function GetAShortString: ShortString;
    procedure SetWriteOnly(AValue: integer);
  published
    property AArray: TTestDynArray read FAArray write FAArray;
    property AEnumeration: TTestEnumeration read FAEnumeration write FAEnumeration;
    property AInteger: Integer read FAInteger write FAInteger;
    property AString: string read FAString write FAString;
    property ASingle: Single read FASingle write FASingle;
    property ADouble: Double read FADouble write FADouble;
    property AExtended: Extended read FAExtended write FAExtended;
    property ACurrency: Currency read FACurrency write FACurrency;
    property AObject: TObject read FAObject write FAObject;
    property AUnknown: IUnknown read FAUnknown write FAUnknown;
    property AComp: Comp read FAComp write FAComp;
    property ABoolean: boolean read FABoolean write FABoolean;
    property AShortString: ShortString read FAShortString write FAShortString;
    property AGetInteger: Integer read GetAInteger;
    property AGetString: string read GetAString;
    property AGetBoolean: boolean read GetABoolean;
    property AGetShortString: ShortString read GetAShortString;
    property AWriteOnly: integer write SetWriteOnly;
    property AChar: AnsiChar read FAChar write FAChar;
    property AWideChar: WideChar read FAWideChar write FAWideChar;
  end;
  {$M-}

  {$M+}
  ITestInterface = interface
    procedure Test;
    function Test2: LongInt;
    procedure Test3(aArg1: LongInt; const aArg2: AnsiString; var aArg3: Boolean; out aArg4: Word);
    function Test4(aArg1: array of LongInt; aArg2: array of const): AnsiString;
  end;
  {$M-}

  TManagedRec = record
    s: string;
  end;

{$ifdef fpc}
  TManagedRecOp = record
    class operator AddRef(var a: TManagedRecOp);
  end;
{$endif}

  TNonManagedRec = record
    i: Integer;
  end;

  TManagedObj = object
    i: IInterface;
  end;

  TNonManagedObj = object
    d: double;
  end;

  TTestEnum = (te1, te2, te3, te4, te5);
  TTestSet = set of TTestEnum;

  TTestProc = procedure;
  TTestFunc1 = function: LongInt;
  TTestFunc2 = function(aArg1: LongInt; aArg2: array of LongInt): String;
  TTestMethod = procedure of object;
  TTestMethod1 = function: LongInt of object;
  TTestMethod2 = function(aArg1: LongInt; aArg2: array of LongInt): String of object;
  TTestHelper = class helper for TObject
  end;

  TArrayOfString = array[0..0] of string;
  TArrayOfManagedRec = array[0..0] of TManagedRec;
  TArrayOfNonManagedRec = array[0..0] of TNonManagedRec;
  TArrayOfByte = array[0..0] of byte;

  TArrayOfLongintDyn = array of LongInt;
  TArrayOfLongintStatic = array[0..3] of LongInt;

  TTestRecord = record
    Value1: LongInt;
    Value2: String;
  end;
  PTestRecord = ^TTestRecord;

{$ifdef fpc}
{$PUSH}
{$INTERFACES CORBA}

  ICORBATest = interface
  end;

{$POP}
{$endif}

{$ifdef fpc}
class operator TManagedRecOp.AddRef(var  a: TManagedRecOp);
begin
end;
{$endif}

{ TTestValueClass }

function TTestValueClass.GetAInteger: integer;
begin
  result := FAInteger;
end;

function TTestValueClass.GetAString: string;
begin
  result := FAString;
end;

function TTestValueClass.GetABoolean: boolean;
begin
  result := FABoolean;
end;

function TTestValueClass.GetAShortString: ShortString;
begin
  Result := FAShortString;
end;

procedure TTestValueClass.SetWriteOnly(AValue: integer);
begin
  // Do nothing
end;

{ Note: GetTypes currently only returns those types that had been acquired using
        GetType, so GetTypes itself can't be really tested currently }
(*procedure TTestCase1.GetTypes;
var
  LContext: TRttiContext;
  LType: TRttiType;
  IsTestCaseClassFound: boolean;
begin
  LContext := TRttiContext.Create;

  { Enumerate all types declared in the application }
  for LType in LContext.GetTypes() do
    begin
    if LType.Name='TTestCase1' then
      IsTestCaseClassFound:=true;
    end;
  LContext.Free;
  CheckTrue(IsTestCaseClassFound, 'RTTI information does not contain class of testcase.');
end;*)

procedure TTestCase1.TestGetValueStringCastError;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AValue: TValue;
  i: integer;
  HadException: boolean;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.AString := '12';
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      AValue := ARttiType.GetProperty('astring').GetValue(ATestClass);
      HadException := false;
      try
        i := AValue.AsInteger;
      except
        on E: Exception do
          if E.ClassType=EInvalidCast then
            HadException := true;
      end;
      Check(HadException, 'No or invalid exception on invalid cast');
    finally
      AtestClass.Free;
    end;
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestMakeNil;
var
  value: TValue;
begin
  TValue.Make(Nil, Nil, value);
  CheckTrue(value.Kind = tkUnknown);
  CheckTrue(value.IsEmpty);
  CheckTrue(value.IsObject);
  CheckTrue(value.IsClass);
  CheckTrue(value.IsOrdinal);
  CheckFalse(value.IsArray);
  CheckTrue(value.AsObject = Nil);
  CheckTrue(value.AsClass = Nil);
  CheckTrue(value.AsInterface = Nil);
  CheckEquals(0, value.AsOrdinal);

  TValue.Make(Nil, TypeInfo(TObject), value);
  CheckTrue(value.IsEmpty);
  CheckTrue(value.IsObject);
  CheckTrue(value.IsClass);
  CheckTrue(value.IsOrdinal);
  CheckFalse(value.IsArray);
  CheckTrue(value.AsObject=Nil);
  CheckTrue(value.AsClass=Nil);
  CheckTrue(value.AsInterface=Nil);
  CheckEquals(0, value.AsOrdinal);

  TValue.Make(Nil, TypeInfo(TClass), value);
  CheckTrue(value.IsEmpty);
  CheckTrue(value.IsClass);
  CheckTrue(value.IsOrdinal);
  CheckFalse(value.IsArray);
  CheckTrue(value.AsObject=Nil);
  CheckTrue(value.AsClass=Nil);
  CheckTrue(value.AsInterface=Nil);
  CheckEquals(0, value.AsOrdinal);

  TValue.Make(Nil, TypeInfo(LongInt), value);
  CheckTrue(value.IsOrdinal);
  CheckFalse(value.IsEmpty);
  CheckFalse(value.IsClass);
  CheckFalse(value.IsObject);
  CheckFalse(value.IsArray);
  CheckEquals(0, value.AsOrdinal);
  CheckEquals(0, value.AsInteger);
  CheckEquals(0, value.AsInt64);
  CheckEquals(0, value.AsUInt64);

  TValue.Make(Nil, TypeInfo(String), value);
  CheckFalse(value.IsEmpty);
  CheckFalse(value.IsObject);
  CheckFalse(value.IsClass);
  CheckFalse(value.IsArray);
  CheckEquals('', value.AsString);
end;

procedure TTestCase1.TestMakeObject;
var
  AValue: TValue;
  ATestClass: TTestValueClass;
begin
  ATestClass := TTestValueClass.Create;
  ATestClass.AInteger := 54329;
  TValue.Make(@ATestClass, TypeInfo(TTestValueClass),AValue);
  CheckEquals(AValue.IsClass, False);
  CheckEquals(AValue.IsObject, True);
  Check(AValue.AsObject=ATestClass);
  Check(PPointer(AValue.GetReferenceToRawData)^ = Pointer(ATestClass));
  CheckEquals(TTestValueClass(AValue.AsObject).AInteger, 54329);
  ATestClass.Free;
end;

procedure TTestCase1.TestMakeArrayDynamic;
var
  arr: TArrayOfLongintDyn;
  value: TValue;
begin
  SetLength(arr, 2);
  arr[0] := 42;
  arr[1] := 21;
  TValue.Make(@arr, TypeInfo(TArrayOfLongintDyn), value);
  CheckEquals(value.IsArray, True);
  CheckEquals(value.IsObject, False);
  CheckEquals(value.IsOrdinal, False);
  CheckEquals(value.IsClass, False);
  CheckEquals(value.GetArrayLength, 2);
  CheckEquals(value.GetArrayElement(0).AsInteger, 42);
  CheckEquals(value.GetArrayElement(1).AsInteger, 21);
  Check(PPointer(value.GetReferenceToRawData)^ = Pointer(arr));
  value.SetArrayElement(0, 84);
  CheckEquals(arr[0], 84);
end;

procedure TTestCase1.TestMakeArrayStatic;
type
  TArrStat = array[0..1] of LongInt;
  TArrStat2D = array[0..1, 0..1] of LongInt;
var
  arr: TArrStat;
  arr2D: TArrStat2D;
  value: TValue;
begin
  arr[0] := 42;
  arr[1] := 21;
  TValue.Make(@arr, TypeInfo(TArrStat), value);
  CheckEquals(value.IsArray, True);
  CheckEquals(value.IsObject, False);
  CheckEquals(value.IsOrdinal, False);
  CheckEquals(value.IsClass, False);
  CheckEquals(value.GetArrayLength, 2);
  CheckEquals(value.GetArrayElement(0).AsInteger, 42);
  CheckEquals(value.GetArrayElement(1).AsInteger, 21);
  value.SetArrayElement(0, 84);
  { since this is a static array the original array isn't touched! }
  CheckEquals(arr[0], 42);

  arr2D[0, 0] := 42;
  arr2D[0, 1] := 21;
  arr2D[1, 0] := 84;
  arr2D[1, 1] := 63;

  TValue.Make(@arr2D, TypeInfo(TArrStat2D), value);
  CheckEquals(value.IsArray, True);
  CheckEquals(value.GetArrayLength, 4);
  CheckEquals(value.GetArrayElement(0).AsInteger, 42);
  CheckEquals(value.GetArrayElement(1).AsInteger, 21);
  CheckEquals(value.GetArrayElement(2).AsInteger, 84);
  CheckEquals(value.GetArrayElement(3).AsInteger, 63);
end;

{$ifdef fpc}
procedure TTestCase1.TestMakeArrayOpen;

  procedure TestOpenArrayValueCopy(aArr: array of LongInt);
  var
    value: TValue;
  begin
    TValue.MakeOpenArray(@aArr[0], Length(aArr), PTypeInfo(TypeInfo(aArr)), value);
    CheckEquals(value.IsArray, True);
    CheckEquals(value.IsOpenArray, True);
    CheckEquals(value.IsObject, False);
    CheckEquals(value.IsOrdinal, False);
    CheckEquals(value.IsClass, False);
    CheckEquals(value.GetArrayLength, 2);
    CheckEquals(value.GetArrayElement(0).AsInteger, 42);
    CheckEquals(value.GetArrayElement(1).AsInteger, 21);
    value.SetArrayElement(0, 84);
    { since this is an open array the original array is modified! }
    CheckEquals(aArr[0], 84);
  end;

  procedure TestOpenArrayValueVar(var aArr: array of LongInt);
  var
    value: TValue;
  begin
    TValue.MakeOpenArray(@aArr[0], Length(aArr), PTypeInfo(TypeInfo(aArr)), value);
    CheckEquals(value.IsArray, True);
    CheckEquals(value.IsOpenArray, True);
    CheckEquals(value.IsObject, False);
    CheckEquals(value.IsOrdinal, False);
    CheckEquals(value.IsClass, False);
    CheckEquals(value.GetArrayLength, 2);
    CheckEquals(value.GetArrayElement(0).AsInteger, 42);
    CheckEquals(value.GetArrayElement(1).AsInteger, 21);
    value.SetArrayElement(0, 84);
    { since this is an open array the original array is modified! }
    CheckEquals(aArr[0], 84);
  end;

  procedure TestOpenArrayValueOut(var aArr: array of LongInt);
  var
    value: TValue;
  begin
    TValue.MakeOpenArray(@aArr[0], Length(aArr), PTypeInfo(TypeInfo(aArr)), value);
    CheckEquals(value.IsArray, True);
    CheckEquals(value.IsOpenArray, True);
    CheckEquals(value.IsObject, False);
    CheckEquals(value.IsOrdinal, False);
    CheckEquals(value.IsClass, False);
    CheckEquals(value.GetArrayLength, 2);
    CheckEquals(value.GetArrayElement(0).AsInteger, 42);
    CheckEquals(value.GetArrayElement(1).AsInteger, 21);
    value.SetArrayElement(0, 84);
    value.SetArrayElement(1, 128);
    { since this is an open array the original array is modified! }
    CheckEquals(aArr[0], 84);
    CheckEquals(aArr[1], 128);
    CheckEquals(value.GetArrayElement(0).AsInteger, 84);
    CheckEquals(value.GetArrayElement(1).AsInteger, 128);
  end;

var
  arr: array of LongInt;
begin
  TestOpenArrayValueCopy([42, 21]);

  arr := [42, 21];
  TestOpenArrayValueVar(arr);
  CheckEquals(arr[0], 84);
  CheckEquals(arr[1], 21);

  arr := [42, 21];
  TestOpenArrayValueOut(arr);
  CheckEquals(arr[0], 84);
  CheckEquals(arr[1], 128);
end;

{$endif}

procedure TTestCase1.TestMakeSingle;
var
  fs: Single;
  v: TValue;
  hadexcept: Boolean;
begin
  fs := 3.14;

  TValue.Make(@fs, TypeInfo(fs), v);
  CheckEquals(v.IsClass, False);
  CheckEquals(v.IsObject, False);
  CheckEquals(v.IsOrdinal, False);
  Check(v.AsExtended=fs);
  Check(v.GetReferenceToRawData <> @fs);

  try
    hadexcept := False;
    v.AsInt64;
  except
    hadexcept := True;
  end;

  CheckTrue(hadexcept, 'No signed type conversion exception');

  try
    hadexcept := False;
    v.AsUInt64;
  except
    hadexcept := True;
  end;

  CheckTrue(hadexcept, 'No unsigned type conversion exception');
end;

procedure TTestCase1.TestMakeDouble;
var
  fd: Double;
  v: TValue;
  hadexcept: Boolean;
begin
  fd := 3.14;

  TValue.Make(@fd, TypeInfo(fd), v);
  CheckEquals(v.IsClass, False);
  CheckEquals(v.IsObject, False);
  CheckEquals(v.IsOrdinal, False);
  Check(v.AsExtended=fd);
  Check(v.GetReferenceToRawData <> @fd);

  try
    hadexcept := False;
    v.AsInt64;
  except
    hadexcept := True;
  end;

  CheckTrue(hadexcept, 'No signed type conversion exception');

  try
    hadexcept := False;
    v.AsUInt64;
  except
    hadexcept := True;
  end;

  CheckTrue(hadexcept, 'No unsigned type conversion exception');
end;

procedure TTestCase1.TestMakeExtended;
var
  fe: Extended;
  v: TValue;
  hadexcept: Boolean;
begin
  fe := 3.14;

  TValue.Make(@fe, TypeInfo(fe), v);
  CheckEquals(v.IsClass, False);
  CheckEquals(v.IsObject, False);
  CheckEquals(v.IsOrdinal, False);
  Check(v.AsExtended=fe);
  Check(v.GetReferenceToRawData <> @fe);

  try
    hadexcept := False;
    v.AsInt64;
  except
    hadexcept := True;
  end;

  CheckTrue(hadexcept, 'No signed type conversion exception');

  try
    hadexcept := False;
    v.AsUInt64;
  except
    hadexcept := True;
  end;

  CheckTrue(hadexcept, 'No unsigned type conversion exception');
end;

procedure TTestCase1.TestMakeCurrency;
var
  fcu: Currency;
  v: TValue;
  hadexcept: Boolean;
begin
  fcu := 3.14;

  TValue.Make(@fcu, TypeInfo(fcu), v);
  CheckEquals(v.IsClass, False);
  CheckEquals(v.IsObject, False);
  CheckEquals(v.IsOrdinal, False);
  Check(v.AsExtended=Extended(fcu));
  Check(v.AsCurrency=fcu);
  Check(v.GetReferenceToRawData <> @fcu);

  try
    hadexcept := False;
    v.AsInt64;
  except
    hadexcept := True;
  end;

  CheckTrue(hadexcept, 'No signed type conversion exception');

  try
    hadexcept := False;
    v.AsUInt64;
  except
    hadexcept := True;
  end;

  CheckTrue(hadexcept, 'No unsigned type conversion exception');
end;

procedure TTestCase1.TestMakeComp;
var
  fco: Comp;
  v: TValue;
  hadexcept: Boolean;
begin
  fco := 314;

  TValue.Make(@fco, TypeInfo(fco), v);

  if v.Kind <> tkFloat then
    Exit;

  CheckEquals(v.IsClass, False);
  CheckEquals(v.IsObject, False);
  CheckEquals(v.IsOrdinal, False);
  Check(v.AsExtended=Extended(fco));
  Check(v.GetReferenceToRawData <> @fco);

  try
    hadexcept := False;
    CheckEquals(v.AsInt64, 314);
  except
    hadexcept := True;
  end;

  CheckFalse(hadexcept, 'Had signed type conversion exception');

  try
    hadexcept := False;
    CheckEquals(v.AsUInt64, 314);
  except
    hadexcept := True;
  end;

  CheckFalse(hadexcept, 'Had unsigned type conversion exception');
end;

procedure TTestCase1.TestMakeEnum;
var
  e: TTestEnum;
  v: TValue;
begin
  e := te1;

  TValue.Make(@e, TypeInfo(e), v);
  Check(not v.IsClass);
  Check(not v.IsArray);
  Check(not v.IsEmpty);
  Check(not v.IsOpenArray);
  Check(not v.IsObject);
  Check(v.IsOrdinal);

  Check(v.GetReferenceToRawData <> @e);
  Check(TTestEnum(v.AsOrdinal) = te1);
end;

procedure TTestCase1.TestMakeAnsiChar;
var
  c: AnsiChar;
  v: TValue;
begin
  c := #20;

  TValue.Make(@c, TypeInfo(c), v);
  Check(not v.IsClass);
  Check(not v.IsArray);
  Check(not v.IsEmpty);
  Check(not v.IsOpenArray);
  Check(not v.IsObject);
  Check(v.IsOrdinal);

  Check(v.GetReferenceToRawData <> @c);
  Check(AnsiChar(v.AsOrdinal) = #20);
  Check(v.AsAnsiChar = #20);
end;

procedure TTestCase1.TestMakeWideChar;
var
  c: WideChar;
  v: TValue;
begin
  c := #$1234;

  TValue.Make(@c, TypeInfo(c), v);
  Check(not v.IsClass);
  Check(not v.IsArray);
  Check(not v.IsEmpty);
  Check(not v.IsOpenArray);
  Check(not v.IsObject);
  Check(v.IsOrdinal);

  Check(v.GetReferenceToRawData <> @c);
  Check(WideChar(v.AsOrdinal) = #$1234);
  Check(v.AsWideChar = #$1234);
end;

procedure TTestCase1.MakeFromOrdinalTObject;
begin
  TValue.FromOrdinal(TypeInfo(TObject), 42);
end;

procedure TTestCase1.MakeFromOrdinalSet;
begin
  TValue.FromOrdinal(TypeInfo(TTestSet), 42);
end;

procedure TTestCase1.MakeFromOrdinalString;
begin
  TValue.FromOrdinal(TypeInfo(AnsiString), 42);
end;

procedure TTestCase1.MakeFromOrdinalNil;
begin
  TValue.FromOrdinal(Nil, 42);
end;

procedure TTestCase1.TestFromOrdinal;
var
  v: TValue;
begin
  v := TValue.FromOrdinal(TypeInfo(LongInt), 42);
  Check(v.IsOrdinal);
  CheckEquals(v.AsOrdinal, 42);

  v := TValue.FromOrdinal(TypeInfo(Boolean), Ord(True));
  Check(v.IsOrdinal);
  CheckEquals(v.AsOrdinal, Ord(True));

  v := TValue.FromOrdinal(TypeInfo(Int64), $1234123412341234);
  Check(v.IsOrdinal);
  CheckEquals(v.AsOrdinal, $1234123412341234);

  v := TValue.FromOrdinal(TypeInfo(QWord), $1234123412341234);
  Check(v.IsOrdinal);
  CheckEquals(v.AsOrdinal, $1234123412341234);

  v := TValue.FromOrdinal(TypeInfo(LongBool), Ord(True));
  Check(v.IsOrdinal);
  CheckEquals(v.AsOrdinal, Ord(True));

  v := TValue.FromOrdinal(TypeInfo(TTestEnum), Ord(te1));
  Check(v.IsOrdinal);
  CheckEquals(v.AsOrdinal, Ord(te1));

  v := TValue.FromOrdinal(TypeInfo(AnsiChar), Ord(#20));
  Check(v.IsOrdinal);
  CheckEquals(v.AsOrdinal, Ord(#20));

  v := TValue.FromOrdinal(TypeInfo(WideChar), Ord(#$1234));
  Check(v.IsOrdinal);
  CheckEquals(v.AsOrdinal, Ord(#$1234));

  CheckException({$ifdef fpc}@{$endif}MakeFromOrdinalNil, EInvalidCast);
  CheckException({$ifdef fpc}@{$endif}MakeFromOrdinalTObject, EInvalidCast);
  CheckException({$ifdef fpc}@{$endif}MakeFromOrdinalSet, EInvalidCast);
  CheckException({$ifdef fpc}@{$endif}MakeFromOrdinalString, EInvalidCast);
end;

procedure TTestCase1.TestGetIsReadable;
var
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
begin
  c := TRttiContext.Create;
  try
    ARttiType := c.GetType(TTestValueClass);
    AProperty := ARttiType.GetProperty('aBoolean');
    CheckEquals(AProperty.IsReadable, true);
    AProperty := ARttiType.GetProperty('aGetBoolean');
    CheckEquals(AProperty.IsReadable, true);
    AProperty := ARttiType.GetProperty('aWriteOnly');
    CheckEquals(AProperty.IsReadable, False);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestIsWritable;
var
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
begin
  c := TRttiContext.Create;
  try
    ARttiType := c.GetType(TTestValueClass);
    AProperty := ARttiType.GetProperty('aBoolean');
    CheckEquals(AProperty.IsWritable, true);
    AProperty := ARttiType.GetProperty('aGetBoolean');
    CheckEquals(AProperty.IsWritable, false);
    AProperty := ARttiType.GetProperty('aWriteOnly');
    CheckEquals(AProperty.IsWritable, True);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestIsType;
type
  TMyLongInt = type LongInt;
var
  v: TValue;
  l: LongInt;
  ml: TMyLongInt;
begin
  l := 42;
  ml := 42;
  TValue.Make(@l, TypeInfo(l), v);
  Check(v.IsType(TypeInfo(l)));
  Check(not v.IsType(TypeInfo(ml)));
  Check(not v.IsType(TypeInfo(String)));
  Check(v.specialize IsType<LongInt>);
  Check(not v.specialize IsType<TMyLongInt>);
  Check(not v.specialize IsType<String>);

  TValue.Make(@ml, TypeInfo(ml), v);
  Check(v.IsType(TypeInfo(ml)));
  Check(not v.IsType(TypeInfo(l)));
  Check(not v.IsType(TypeInfo(String)));
  Check(v.specialize IsType<TMyLongInt>);
  Check(not v.specialize IsType<LongInt>);
  Check(not v.specialize IsType<String>);
end;

procedure TTestCase1.TestPropGetValueBoolean;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.ABoolean := true;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));
      AProperty := ARttiType.GetProperty('aBoolean');
      AValue := AProperty.GetValue(ATestClass);
      CheckEquals(true,AValue.AsBoolean);
      ATestClass.ABoolean := false;
      CheckEquals(true, AValue.AsBoolean);
      CheckEquals('True', AValue.ToString);
      CheckEquals(True, AValue.IsOrdinal);
      CheckEquals(1, AValue.AsOrdinal);
    finally
      AtestClass.Free;
    end;
      CheckEquals(True,AValue.AsBoolean);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropGetValueShortString;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.AShortString := 'Hello World';
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));
      AProperty := ARttiType.GetProperty('aShortString');
      AValue := AProperty.GetValue(ATestClass);
      CheckEquals('Hello World',AValue.AsString);
      ATestClass.AShortString := 'Foobar';
      CheckEquals('Hello World', AValue.AsString);
      CheckEquals(False, AValue.IsOrdinal);
      CheckEquals(False, AValue.IsObject);
      CheckEquals(False, AValue.IsArray);
      CheckEquals(False, AValue.IsClass);
    finally
      AtestClass.Free;
    end;
    CheckEquals('Hello World',AValue.AsString);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropGetValueInteger;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.AInteger := 472349;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));
      AProperty := ARttiType.GetProperty('ainteger');
      AValue := AProperty.GetValue(ATestClass);
      CheckEquals(472349,AValue.AsInteger);
      ATestClass.AInteger := 12;
      CheckEquals(472349, AValue.AsInteger);
      CheckEquals('472349', AValue.ToString);
      CheckEquals(True, AValue.IsOrdinal);
    finally
      AtestClass.Free;
    end;
      CheckEquals(472349,AValue.AsInteger);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropGetValueString;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
  i: int64;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.AString := 'Hello World';
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));
      AProperty := ARttiType.GetProperty('astring');
      AValue := AProperty.GetValue(ATestClass);
      CheckEquals('Hello World',AValue.AsString);
      ATestClass.AString := 'Goodbye World';
      CheckEquals('Hello World',AValue.AsString);
      CheckEquals('Hello World',AValue.ToString);
      Check(TypeInfo(string)=AValue.TypeInfo);
      Check(AValue.TypeData=GetTypeData(AValue.TypeInfo));
      Check(AValue.IsEmpty=false);
      Check(AValue.IsObject=false);
      Check(AValue.IsClass=false);
      CheckEquals(AValue.IsOrdinal, false);
      CheckEquals(AValue.TryAsOrdinal(i), false);
      CheckEquals(AValue.IsType(TypeInfo(string)), true);
      CheckEquals(AValue.IsType(TypeInfo(integer)), false);
      CheckEquals(AValue.IsArray, false);
    finally
      AtestClass.Free;
    end;
    CheckEquals('Hello World',AValue.AsString);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropGetValueProcBoolean;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.ABoolean := true;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));
      AProperty := ARttiType.GetProperty('aGetBoolean');
      AValue := AProperty.GetValue(ATestClass);
      CheckEquals(true,AValue.AsBoolean);
    finally
      AtestClass.Free;
    end;
      CheckEquals(True,AValue.AsBoolean);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropGetValueProcShortString;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.AShortString := 'Hello World';
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));
      AProperty := ARttiType.GetProperty('aGetShortString');
      AValue := AProperty.GetValue(ATestClass);
      CheckEquals('Hello World',AValue.AsString);
    finally
      AtestClass.Free;
    end;
    CheckEquals('Hello World',AValue.AsString);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropGetValueObject;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
  O: TObject;
begin
  c := TRttiContext.Create;
  O := TObject.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.AObject := O;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));
      AProperty := ARttiType.GetProperty('AObject');
      AValue := AProperty.GetValue(ATestClass);
      CheckEquals(O.GetHashCode, AValue.AsObject.GetHashCode);
    finally
      AtestClass.Free;
    end;
    CheckEquals(O.GetHashCode, AValue.AsObject.GetHashCode);
  finally
    c.Free;
    O.Free;
  end;
end;

procedure TTestCase1.TestPropGetValueInterface;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
  i: IInterface;
begin
  c := TRttiContext.Create;
  i := TInterfacedObject.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.AUnknown := i;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));
      AProperty := ARttiType.GetProperty('AUnknown');
      AValue := AProperty.GetValue(ATestClass);
      Check(i = AValue.AsInterface);
    finally
      AtestClass.Free;
    end;
    Check(i = AValue.AsInterface);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropGetValueFloat;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValueS, AValueD, AValueE, AValueC, AValueCm: TValue;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.ASingle := 1.1;
    ATestClass.ADouble := 2.2;
    ATestClass.AExtended := 3.3;
    ATestClass.ACurrency := 4;
    ATestClass.AComp := 5;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));

      AProperty := ARttiType.GetProperty('ASingle');
      AValueS := AProperty.GetValue(ATestClass);
      CheckEquals(1.1, AValueS.AsExtended, 0.001);

      AProperty := ARttiType.GetProperty('ADouble');
      AValueD := AProperty.GetValue(ATestClass);
      CheckEquals(2.2, AValueD.AsExtended, 0.001);

      AProperty := ARttiType.GetProperty('AExtended');
      AValueE := AProperty.GetValue(ATestClass);
      CheckEquals(3.3, AValueE.AsExtended, 0.001);

      AProperty := ARttiType.GetProperty('ACurrency');
      AValueC := AProperty.GetValue(ATestClass);
      CheckEquals(4.0, AValueC.AsExtended, 0.001);

      AProperty := ARttiType.GetProperty('AComp');
      AValueCm := AProperty.GetValue(ATestClass);
      CheckEquals(5.0, AValueCm.AsExtended, 0.001);
    finally
      AtestClass.Free;
    end;

    CheckEquals(1.1, AValueS.AsExtended, 0.001);
    CheckEquals(2.2, AValueD.AsExtended, 0.001);
    CheckEquals(3.3, AValueE.AsExtended, 0.001);
    CheckEquals(4.0, AValueC.AsExtended, 0.001);
    CheckEquals(5.0, AValueCm.AsExtended, 0.001);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropGetValueDynArray;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
  A: TTestDynArray;
begin
  c := TRttiContext.Create;
  A := [1, 2, 3, 4];
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.AArray := A;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));
      AProperty := ARttiType.GetProperty('AArray');
      AValue := AProperty.GetValue(ATestClass);

      CheckEquals(A[0], AValue.GetArrayElement(0).AsInteger);
      CheckEquals(A[1], AValue.GetArrayElement(1).AsInteger);
      CheckEquals(A[2], AValue.GetArrayElement(2).AsInteger);
      CheckEquals(A[3], AValue.GetArrayElement(3).AsInteger);
    finally
      AtestClass.Free;
    end;
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropGetValueEnumeration;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.AEnumeration := en3;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));
      AProperty := ARttiType.GetProperty('AEnumeration');
      AValue := AProperty.GetValue(ATestClass);
      CheckEquals(Ord(en3),AValue.AsOrdinal);
      ATestClass.AEnumeration := en1;
      CheckEquals(Ord(en3), AValue.AsOrdinal);
      CheckEquals('en3', AValue.ToString);
      CheckEquals(True, AValue.IsOrdinal);
    finally
      AtestClass.Free;
    end;

    CheckEquals(Ord(en3),AValue.AsOrdinal);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropGetValueChars;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValueC, AValueW: TValue;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.AChar := 'C';
    ATestClass.AWideChar := 'W';
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));

      AProperty := ARttiType.GetProperty('AChar');
      AValueC := AProperty.GetValue(ATestClass);
      CheckEquals('C',AValueC.AsAnsiChar);
      ATestClass.AChar := 'N';
      CheckEquals('C', AValueC.AsAnsiChar);
      CheckEquals('C', AValueC.ToString);
      CheckEquals(True, AValueC.IsOrdinal);

      AProperty := ARttiType.GetProperty('AWideChar');
      AValueW := AProperty.GetValue(ATestClass);
      CheckEquals('W',AValueW.AsWideChar);
      ATestClass.AWideChar := 'Z';
      CheckEquals('W', AValueW.AsWideChar);
      CheckEquals('W', AValueW.ToString);
      CheckEquals(True, AValueW.IsOrdinal);
    finally
      AtestClass.Free;
    end;

    CheckEquals('C',AValueC.AsAnsiChar);
    CheckEquals('W',AValueW.AsWideChar);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropSetValueString;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
  s: string;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      AProperty := ARttiType.GetProperty('astring');

      s := 'ipse lorem or something like that';
      TValue.Make(@s, TypeInfo(string), AValue);
      AProperty.SetValue(ATestClass, AValue);
      CheckEquals(ATestClass.AString, s);
      s := 'Another string';
      CheckEquals(ATestClass.AString, 'ipse lorem or something like that');
    finally
      AtestClass.Free;
    end;
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropSetValueInteger;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
  i: integer;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      AProperty := ARttiType.GetProperty('aInteger');

      i := -43573;
      TValue.Make(@i, TypeInfo(Integer), AValue);
      AProperty.SetValue(ATestClass, AValue);
      CheckEquals(ATestClass.AInteger, i);
      i := 1;
      CheckEquals(ATestClass.AInteger, -43573);
    finally
      AtestClass.Free;
    end;
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropSetValueBoolean;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
  b: boolean;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      AProperty := ARttiType.GetProperty('aboolean');

      b := true;
      TValue.Make(@b, TypeInfo(Boolean), AValue);
      AProperty.SetValue(ATestClass, AValue);
      CheckEquals(ATestClass.ABoolean, b);
      b := false;
      CheckEquals(ATestClass.ABoolean, true);
      TValue.Make(@b, TypeInfo(Boolean), AValue);
      AProperty.SetValue(ATestClass, AValue);
      CheckEquals(ATestClass.ABoolean, false);
    finally
      AtestClass.Free;
    end;
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropSetValueShortString;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
  s: string;
  ss: ShortString;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      AProperty := ARttiType.GetProperty('aShortString');

      s := 'ipse lorem or something like that';
      TValue.Make(@s, TypeInfo(String), AValue);
      AProperty.SetValue(ATestClass, AValue);
      CheckEquals(ATestClass.AShortString, s);
      s := 'Another string';
      CheckEquals(ATestClass.AShortString, 'ipse lorem or something like that');

      ss := 'Hello World';
      TValue.Make(@ss, TypeInfo(ShortString), AValue);
      AProperty.SetValue(ATestClass, AValue);
      CheckEquals(ATestClass.AShortString, ss);
      ss := 'Foobar';
      CheckEquals(ATestClass.AShortString, 'Hello World');

      AProperty.SetValue(ATestClass, 'Another string');
      CheckEquals(ATestClass.AShortString, 'Another string');
    finally
      AtestClass.Free;
    end;
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropSetValueObject;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
  O: TObject;
  TypeInfo: PTypeInfo;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      AProperty := ARttiType.GetProperty('AObject');
      TypeInfo := GetPropInfo(ATestClass, 'AObject')^.PropType;

      O := TPersistent.Create;
      TValue.Make(@O, TypeInfo, AValue);
      AProperty.SetValue(ATestClass, AValue);
      CheckEquals(ATestClass.AObject.GetHashCode, O.GetHashCode);
      O.Free;

      O := TPersistent.Create;
      AProperty.SetValue(ATestClass, O);
      CheckEquals(ATestClass.AObject.GetHashCode, O.GetHashCode);
      O.Free;
    finally
      AtestClass.Free;
    end;
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropSetValueInterface;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
  TypeInfo: PTypeInfo;
  i: IInterface;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      AProperty := ARttiType.GetProperty('AUnknown');
      TypeInfo := GetPropInfo(ATestClass, 'AUnknown')^.PropType;

      i := TInterfacedObject.Create;
      TValue.Make(@i, TypeInfo, AValue);
      AProperty.SetValue(ATestClass, AValue);
      Check(ATestClass.AUnknown = i);

      i := TInterfacedObject.Create;
      AProperty.SetValue(ATestClass, i);
      Check(ATestClass.AUnknown = i);
    finally
      AtestClass.Free;
    end;
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropSetValueFloat;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
  TypeInfo: PTypeInfo;
  S: Single;
  D: Double;
  E: Extended;
  Cur: Currency;
  Cmp: Comp;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);

      AProperty := ARttiType.GetProperty('ASingle');
      TypeInfo := GetPropInfo(ATestClass, 'ASingle')^.PropType;

      S := 1.1;
      TValue.Make(@S, TypeInfo, AValue);
      AProperty.SetValue(ATestClass, AValue);
      CheckEquals(S, ATestClass.ASingle, 0.001);

      S := 1.2;
      AProperty.SetValue(ATestClass, S);
      CheckEquals(S, ATestClass.ASingle, 0.001);

      AProperty := ARttiType.GetProperty('ADouble');
      TypeInfo := GetPropInfo(ATestClass, 'ADouble')^.PropType;

      D := 2.1;
      TValue.Make(@D, TypeInfo, AValue);
      AProperty.SetValue(ATestClass, AValue);
      CheckEquals(D, ATestClass.ADouble, 0.001);

      D := 2.2;
      AProperty.SetValue(ATestClass, D);
      CheckEquals(D, ATestClass.ADouble, 0.001);

      AProperty := ARttiType.GetProperty('AExtended');
      TypeInfo := GetPropInfo(ATestClass, 'AExtended')^.PropType;

      E := 3.1;
      TValue.Make(@E, TypeInfo, AValue);
      AProperty.SetValue(ATestClass, AValue);
      CheckEquals(E, ATestClass.AExtended, 0.001);

      E := 3.2;
      AProperty.SetValue(ATestClass, E);
      CheckEquals(E, ATestClass.AExtended, 0.001);

      AProperty := ARttiType.GetProperty('ACurrency');
      TypeInfo := GetPropInfo(ATestClass, 'ACurrency')^.PropType;

      Cur := 40;
      TValue.Make(@Cur, TypeInfo, AValue);
      AProperty.SetValue(ATestClass, AValue);
      CheckEquals(Cur, ATestClass.ACurrency, 0.001);

      Cur := 41;
      AProperty.SetValue(ATestClass, Cur);
      CheckEquals(Cur, ATestClass.ACurrency, 0.001);

      AProperty := ARttiType.GetProperty('AComp');
      TypeInfo := GetPropInfo(ATestClass, 'AComp')^.PropType;

      Cmp := 50;
      TValue.Make(@Cmp, TypeInfo, AValue);
      AProperty.SetValue(ATestClass, AValue);
      CheckEquals(Cmp, ATestClass.AComp, 0.001);

      Cmp := 51;
      AProperty.SetValue(ATestClass, Cmp);
      CheckEquals(Cmp, ATestClass.AComp, 0.001);
    finally
      AtestClass.Free;
    end;
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropSetValueDynArray;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
  A: TTestDynArray;
  TypeInfo: PTypeInfo;
  i: Integer;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      AProperty := ARttiType.GetProperty('AArray');
      TypeInfo := GetPropInfo(ATestClass, 'AArray')^.PropType;

      A := [1, 2, 3, 4, 5];
      TValue.Make(@A, TypeInfo, AValue);
      AProperty.SetValue(ATestClass, AValue);

      for i := 0 to High(A) do
        CheckEquals(A[i], ATestClass.AArray[i]);
    finally
      AtestClass.Free;
    end;
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropSetValueEnumeration;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
  E: TTestEnumeration;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      AProperty := ARttiType.GetProperty('AEnumeration');

      E := en2;
      TValue.Make(@E, TypeInfo(TTestEnumeration), AValue);
      AProperty.SetValue(ATestClass, AValue);
      CheckEquals(Ord(E), Ord(ATestClass.AEnumeration));
    finally
      AtestClass.Free;
    end;
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropSetValueChars;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValueC, AValueW: TValue;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.AChar := 'C';
    ATestClass.AWideChar := 'W';
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));

      AProperty := ARttiType.GetProperty('AChar');
      AValueC := AProperty.GetValue(ATestClass);
      CheckEquals('C', AValueC.AsAnsiChar);

      AProperty := ARttiType.GetProperty('AWideChar');
      AValueW := AProperty.GetValue(ATestClass);
      CheckEquals('W', AValueW.AsWideChar);
    finally
      AtestClass.Free;
    end;
      CheckEquals('C', AValueC.AsAnsiChar);
      CheckEquals('W', AValueW.AsWideChar);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropGetValueProcInteger;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.AInteger := 472349;
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));
      AProperty := ARttiType.GetProperty('agetinteger');
      AValue := AProperty.GetValue(ATestClass);
      CheckEquals(472349,AValue.AsInteger);
    finally
      AtestClass.Free;
    end;
      CheckEquals(472349,AValue.AsInteger);
  finally
    c.Free;
  end;
end;

procedure TTestCase1.TestPropGetValueProcString;
var
  ATestClass : TTestValueClass;
  c: TRttiContext;
  ARttiType: TRttiType;
  AProperty: TRttiProperty;
  AValue: TValue;
begin
  c := TRttiContext.Create;
  try
    ATestClass := TTestValueClass.Create;
    ATestClass.AString := 'Hello World';
    try
      ARttiType := c.GetType(ATestClass.ClassInfo);
      Check(assigned(ARttiType));
      AProperty := ARttiType.GetProperty('agetstring');
      AValue := AProperty.GetValue(ATestClass);
      CheckEquals('Hello World',AValue.AsString);
    finally
      AtestClass.Free;
    end;
    CheckEquals('Hello World',AValue.AsString);
  finally
    c.Free;
  end;
end;


procedure TTestCase1.TestTRttiTypeProperties;
var
  c: TRttiContext;
  ARttiType: TRttiType;

begin
  c := TRttiContext.Create;
  try
    ARttiType := c.GetType(TTestValueClass);
    Check(assigned(ARttiType));
    CheckEquals(ARttiType.Name,'TTestValueClass');
    Check(ARttiType.TypeKind=tkClass);
//    CheckEquals(ARttiType.IsPublicType,false);
    CheckEquals(ARttiType.TypeSize,SizeOf(TObject));
    CheckEquals(ARttiType.IsManaged,false);
    CheckEquals(ARttiType.BaseType.classname,'TRttiInstanceType');
    CheckEquals(ARttiType.IsInstance,True);
    CheckEquals(ARttiType.AsInstance.DeclaringUnitName,'tests.rtti');
    Check(ARttiType.BaseType.Name='TObject');
    Check(ARttiType.AsInstance.BaseType.Name='TObject');
    CheckEquals(ARttiType.IsOrdinal,False);
    CheckEquals(ARttiType.IsRecord,False);
    CheckEquals(ARttiType.IsSet,False);
  finally
    c.Free;
  end;

end;

procedure TTestCase1.GetTypeInteger;
var
  LContext: TRttiContext;
  LType: TRttiType;
begin
  LContext := TRttiContext.Create;

  LType := LContext.GetType(TypeInfo(integer));
{$ifdef fpc}
  CheckEquals(LType.Name, 'LongInt');
{$else}
  CheckEquals(LType.Name, 'Integer');
{$endif}

  LContext.Free;
end;

procedure TTestCase1.GetTypePointer;
var
  context: TRttiContext;
  t: TRttiType;
  p: TRttiPointerType absolute t;
begin
  context := TRttiContext.Create;
  try
    t := context.GetType(TypeInfo(Pointer));
    Assert(t is TRttiPointerType, 'Type of Pointer is not a TRttiPointerType');
    Assert(not Assigned(p.ReferredType), 'ReferredType of Pointer is not Nil');
    t := context.GetType(TypeInfo(PLongInt));
    Assert(t is TRttiPointerType, 'Type of Pointer is not a TRttiPointerType');
    Assert(Assigned(p.ReferredType), 'ReferredType of PLongInt is Nil');
    Assert(p.ReferredType = context.GetType(TypeInfo(LongInt)), 'ReferredType of PLongInt is not a LongInt');
    t := context.GetType(TypeInfo(PWideChar));
    Assert(t is TRttiPointerType, 'Type of Pointer is not a TRttiPointerType');
    Assert(Assigned(p.ReferredType), 'ReferredType of PWideChar is Nil');
    Assert(p.ReferredType = context.GetType(TypeInfo(WideChar)), 'ReferredType of PWideChar is not a WideChar');
  finally
    context.Free;
  end;
end;

procedure TTestCase1.GetClassProperties;
var
  LContext: TRttiContext;
  LType: TRttiType;
  PropList, PropList2: {$ifdef fpc}specialize{$endif} TArray<TRttiProperty>;
  i: LongInt;
begin
  LContext := TRttiContext.Create;

  LType := LContext.GetType(TypeInfo(TGetClassProperties));
  PropList := LType.GetProperties;

  CheckEquals(4, length(PropList));
  CheckEquals('PubPropRO', PropList[0].Name);
  CheckEquals('PubPropRW', PropList[1].Name);
  CheckEquals('PubPropSetRO', PropList[2].Name);
  CheckEquals('PubPropSetRW', PropList[3].Name);

  LType := LContext.GetType(TypeInfo(TGetClassPropertiesSub));
  PropList2 := LType.GetProperties;

  CheckEquals(Length(PropList), Length(PropList2));
  for i := 0 to High(PropList) do
    Check(PropList[i] = PropList2[i], 'Property instances are not equal');

  LContext.Free;
end;

procedure TTestCase1.GetClassPropertiesValue;
var
  AGetClassProperties: TGetClassProperties;
  LContext: TRttiContext;
  LType: TRttiType;
  AValue: TValue;
begin
  LContext := TRttiContext.Create;

  LType := LContext.GetType(TGetClassProperties);

  AGetClassProperties := TGetClassProperties.Create;
  try
    AGetClassProperties.PubPropRW:=12345;

    AValue := LType.GetProperty('PubPropRW').GetValue(AGetClassProperties);
    CheckEquals(12345, AValue.AsInteger);

  finally
    AGetClassProperties.Free;
  end;

  LContext.Free;
end;

procedure TTestCase1.TestReferenceRawData;
var
  value: TValue;
  str: String;
  intf: IInterface;
  i: LongInt;
  test: TTestRecord;
  arrdyn: TArrayOfLongintDyn;
  arrstat: TArrayOfLongintStatic;
begin
  str := 'Hello World';
  UniqueString(str);
  TValue.Make(@str, TypeInfo(String), value);
  Check(PPointer(value.GetReferenceToRawData)^ = Pointer(str), 'Reference to string data differs');

  intf := TInterfacedObject.Create;
  TValue.Make(@intf, TypeInfo(IInterface), value);
  Check(PPointer(value.GetReferenceToRawData)^ = Pointer(intf), 'Reference to interface data differs');

  i := 42;
  TValue.Make(@i, TypeInfo(LongInt), value);
  Check(value.GetReferenceToRawData <> @i, 'Reference to longint is equal');
  Check(PLongInt(value.GetReferenceToRawData)^ = PLongInt(@i)^, 'Reference to longint data differs');

  test.value1 := 42;
  test.value2 := 'Hello World';
  TValue.Make(@test, TypeInfo(TTestRecord), value);
  Check(value.GetReferenceToRawData <> @test, 'Reference to record is equal');
  Check(PTestRecord(value.GetReferenceToRawData)^.value1 = PTestRecord(@test)^.value1, 'Reference to record data value1 differs');
  Check(PTestRecord(value.GetReferenceToRawData)^.value2 = PTestRecord(@test)^.value2, 'Reference to record data value2 differs');

  SetLength(arrdyn, 3);
  arrdyn[0] := 42;
  arrdyn[1] := 23;
  arrdyn[2] := 49;
  TValue.Make(@arrdyn, TypeInfo(TArrayOfLongintDyn), value);
  Check(PPointer(value.GetReferenceToRawData)^ = Pointer(arrdyn), 'Reference to dynamic array data differs');

  arrstat[0] := 42;
  arrstat[1] := 23;
  arrstat[2] := 49;
  arrstat[3] := 59;
  TValue.Make(@arrstat, TypeInfo(TArrayOfLongintStatic), value);
  Check(value.GetReferenceToRawData <> @arrstat, 'Reference to static array is equal');
  Check(PLongInt(value.GetReferenceToRawData)^ = PLongInt(@arrstat)^, 'Reference to static array data differs');
end;

procedure TTestCase1.TestReferenceRawDataEmpty;
var
  value: TValue;
begin
  TValue.Make(Nil, TypeInfo(String), value);
  Check(Assigned(value.GetReferenceToRawData()), 'Reference to empty String is not assigned');
  Check(not Assigned(PPointer(value.GetReferenceToRawData)^), 'Empty String data is assigned');

  TValue.Make(Nil, TypeInfo(IInterface), value);
  Check(Assigned(value.GetReferenceToRawData()), 'Reference to empty interface is not assigned');
  Check(not Assigned(PPointer(value.GetReferenceToRawData)^), 'Empty interface data is assigned');

  TValue.Make(Nil, TypeInfo(LongInt), value);
  Check(Assigned(value.GetReferenceToRawData()), 'Reference to empty LongInt is not assigned');
  Check(PLongInt(value.GetReferenceToRawData)^ = 0, 'Empty longint data is not 0');

  TValue.Make(Nil, TypeInfo(TTestRecord), value);
  Check(Assigned(value.GetReferenceToRawData()), 'Reference to empty record is not assigned');
  Check(PTestRecord(value.GetReferenceToRawData)^.value1 = 0, 'Empty record data value1 is not 0');
  Check(PTestRecord(value.GetReferenceToRawData)^.value2 = '', 'Empty record data value2 is not empty');

  TValue.Make(Nil, TypeInfo(TArrayOfLongintDyn), value);
  Check(Assigned(value.GetReferenceToRawData()), 'Reference to empty dynamic array is not assigned');
  Check(not Assigned(PPointer(value.GetReferenceToRawData)^), 'Empty dynamic array data is assigned');

  TValue.Make(Nil, TypeInfo(TArrayOfLongintStatic), value);
  Check(Assigned(value.GetReferenceToRawData()), 'Reference to empty static array is not assigned');
  Check(PLongInt(value.GetReferenceToRawData)^ = 0, 'Empty static array data is not 0');
end;

procedure TTestCase1.TestDataSize;
var
  u8: UInt8;
  u16: UInt16;
  u32: UInt32;
  u64: UInt64;
  s8: Int8;
  s16: Int16;
  s32: Int32;
  s64: Int64;
  f32: Single;
  f64: Double;
{$ifdef FPC_HAS_TYPE_EXTENDED}
  f80: Extended;
{$endif}
  fco: Comp;
  fcu: Currency;
  ss: ShortString;
  sa: AnsiString;
  su: UnicodeString;
  sw: WideString;
  o: TObject;
  c: TClass;
  i: IInterface;
  ad: TArrayOfLongintDyn;
  _as: TArrayOfLongintStatic;
  b8: Boolean;
{$ifdef fpc}
  b16: Boolean16;
  b32: Boolean32;
  b64: Boolean64;
{$endif}
  bl8: ByteBool;
  bl16: WordBool;
  bl32: LongBool;
{$ifdef fpc}
  bl64: QWordBool;
{$endif}
  e: TTestEnum;
  s: TTestSet;
  t: TTestRecord;
  p: Pointer;
  proc: TTestProc;
  method: TTestMethod;

  value: TValue;
begin
  u8:=245;
  TValue.Make(@u8, TypeInfo(UInt8), value);
  CheckEquals(1, value.DataSize, 'Size of UInt8 differs');
  u16:=789;
  TValue.Make(@u16, TypeInfo(UInt16), value);
  CheckEquals(2, value.DataSize, 'Size of UInt16 differs');
  u32:=568789;
  TValue.Make(@u32, TypeInfo(UInt32), value);
  CheckEquals(4, value.DataSize, 'Size of UInt32 differs');
  u64:=$abdcefadbcef;
  TValue.Make(@u64, TypeInfo(UInt64), value);
  CheckEquals(8, value.DataSize, 'Size of UInt64 differs');
  s8:=-32;
  TValue.Make(@s8, TypeInfo(Int8), value);
  CheckEquals(1, value.DataSize, 'Size of Int8 differs');
  s16:=-5345;
  TValue.Make(@s16, TypeInfo(Int16), value);
  CheckEquals(2, value.DataSize, 'Size of Int16 differs');
  s32:=-234567;
  TValue.Make(@s32, TypeInfo(Int32), value);
  CheckEquals(4, value.DataSize, 'Size of Int32 differs');
  s64:=23456789012;
  TValue.Make(@s64, TypeInfo(Int64), value);
  CheckEquals(8, value.DataSize, 'Size of Int64 differs');
  b8:=false;
  TValue.Make(@b8, TypeInfo(Boolean), value);
  CheckEquals(1, value.DataSize, 'Size of Boolean differs');
{$ifdef fpc}
  b16:=true;
  TValue.Make(@b16, TypeInfo(Boolean16), value);
  CheckEquals(2, value.DataSize, 'Size of Boolean16 differs');
  b32:=false;
  TValue.Make(@b32, TypeInfo(Boolean32), value);
  CheckEquals(4, value.DataSize, 'Size of Boolean32 differs');
  b64:=true;
  TValue.Make(@b64, TypeInfo(Boolean64), value);
  CheckEquals(8, value.DataSize, 'Size of Boolean64 differs');
{$endif}
  bl8:=true;
  TValue.Make(@bl8, TypeInfo(ByteBool), value);
  CheckEquals(1, value.DataSize, 'Size of ByteBool differs');
  bl16:=false;
  TValue.Make(@bl16, TypeInfo(WordBool), value);
  CheckEquals(2, value.DataSize, 'Size of WordBool differs');
  bl32:=false;
  TValue.Make(@bl32, TypeInfo(LongBool), value);
  CheckEquals(4, value.DataSize, 'Size of LongBool differs');
{$ifdef fpc}
  bl64:=true;
  TValue.Make(@bl64, TypeInfo(QWordBool), value);
  CheckEquals(8, value.DataSize, 'Size of QWordBool differs');
{$endif}
  f32:=4.567;
  TValue.Make(@f32, TypeInfo(Single), value);
  CheckEquals(4, value.DataSize, 'Size of Single differs');
  f64:=-3456.678;
  TValue.Make(@f64, TypeInfo(Double), value);
  CheckEquals(8, value.DataSize, 'Size of Double differs');
{$ifdef FPC_HAS_TYPE_EXTENDED}
  f80:=-2345.678;
  TValue.Make(@f80, TypeInfo(Extended), value);
  CheckEquals(10, value.DataSize, 'Size of Extended differs');
{$endif}
  fcu:=56.78;
  TValue.Make(@fcu, TypeInfo(Currency), value);
  CheckEquals(SizeOf(Currency), value.DataSize, 'Size of Currency differs');
  fco:=456;
  TValue.Make(@fco, TypeInfo(Comp), value);
  CheckEquals(SizeOf(Comp), value.DataSize, 'Size of Comp differs');
  ss := '';
  TValue.Make(@ss, TypeInfo(ShortString), value);
  CheckEquals(254, value.DataSize, 'Size ofShortString differs');
  sa:= '';
  TValue.Make(@sa, TypeInfo(AnsiString), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of AnsiString differs');
  sw := '';
  TValue.Make(@sw, TypeInfo(WideString), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of WideString differs');
  su:='';
  TValue.Make(@su, TypeInfo(UnicodeString), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of UnicodeString differs');
  o := TTestValueClass.Create;
  TValue.Make(@o, TypeInfo(TObject), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of TObject differs');
  o.Free;
  c := TObject;
  TValue.Make(@c, TypeInfo(TClass), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of TClass differs');
  i := Nil;
  TValue.Make(@i, TypeInfo(IInterface), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of IInterface differs');
  TValue.Make(@t, TypeInfo(TTestRecord), value);
  CheckEquals(SizeOf(TTestRecord), value.DataSize, 'Size of TTestRecord differs');
  proc := Nil;
  TValue.Make(@proc, TypeInfo(TTestProc), value);
  CheckEquals(SizeOf(TTestProc), value.DataSize, 'Size of TTestProc differs');
  method := Nil;
  TValue.Make(@method, TypeInfo(TTestMethod), value);
  CheckEquals(SizeOf(TTestMethod), value.DataSize, 'Size of TTestMethod differs');
  TValue.Make(@_as, TypeInfo(TArrayOfLongintStatic), value);
  CheckEquals(SizeOf(TArrayOfLongintStatic), value.DataSize, 'Size of TArrayOfLongintStatic differs');
  TValue.Make(@ad, TypeInfo(TArrayOfLongintDyn), value);
  CheckEquals(SizeOf(TArrayOfLongintDyn), value.DataSize, 'Size of TArrayOfLongintDyn differs');
  e:=low(TTestEnum);
  TValue.Make(@e, TypeInfo(TTestEnum), value);
  CheckEquals(SizeOf(TTestEnum), value.DataSize, 'Size of TTestEnum differs');
  s:=[low(TTestEnum),high(TTestEnum)];
  TValue.Make(@s, TypeInfo(TTestSet), value);
  CheckEquals(SizeOf(TTestSet), value.DataSize, 'Size of TTestSet differs');
  p := Nil;
  TValue.Make(@p, TypeInfo(Pointer), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of Pointer differs');
end;

procedure TTestCase1.TestDataSizeEmpty;
var
  value: TValue;
begin
  TValue.Make(Nil, TypeInfo(UInt8), value);
  CheckEquals(1, value.DataSize, 'Size of UInt8 differs');
  TValue.Make(Nil, TypeInfo(UInt16), value);
  CheckEquals(2, value.DataSize, 'Size of UInt16 differs');
  TValue.Make(Nil, TypeInfo(UInt32), value);
  CheckEquals(4, value.DataSize, 'Size of UInt32 differs');
  TValue.Make(Nil, TypeInfo(UInt64), value);
  CheckEquals(8, value.DataSize, 'Size of UInt64 differs');
  TValue.Make(Nil, TypeInfo(Int8), value);
  CheckEquals(1, value.DataSize, 'Size of Int8 differs');
  TValue.Make(Nil, TypeInfo(Int16), value);
  CheckEquals(2, value.DataSize, 'Size of Int16 differs');
  TValue.Make(Nil, TypeInfo(Int32), value);
  CheckEquals(4, value.DataSize, 'Size of Int32 differs');
  TValue.Make(Nil, TypeInfo(Int64), value);
  CheckEquals(8, value.DataSize, 'Size of Int64 differs');
  TValue.Make(Nil, TypeInfo(Boolean), value);
  CheckEquals(1, value.DataSize, 'Size of Boolean differs');
{$ifdef fpc}
  TValue.Make(Nil, TypeInfo(Boolean16), value);
  CheckEquals(2, value.DataSize, 'Size of Boolean16 differs');
  TValue.Make(Nil, TypeInfo(Boolean32), value);
  CheckEquals(4, value.DataSize, 'Size of Boolean32 differs');
  TValue.Make(Nil, TypeInfo(Boolean64), value);
  CheckEquals(8, value.DataSize, 'Size of Boolean64 differs');
{$endif}
  TValue.Make(Nil, TypeInfo(ByteBool), value);
  CheckEquals(1, value.DataSize, 'Size of ByteBool differs');
  TValue.Make(Nil, TypeInfo(WordBool), value);
  CheckEquals(2, value.DataSize, 'Size of WordBool differs');
  TValue.Make(Nil, TypeInfo(LongBool), value);
  CheckEquals(4, value.DataSize, 'Size of LongBool differs');
{$ifdef fpc}
  TValue.Make(Nil, TypeInfo(QWordBool), value);
  CheckEquals(8, value.DataSize, 'Size of QWordBool differs');
{$endif}
  TValue.Make(Nil, TypeInfo(Single), value);
  CheckEquals(4, value.DataSize, 'Size of Single differs');
  TValue.Make(Nil, TypeInfo(Double), value);
  CheckEquals(8, value.DataSize, 'Size of Double differs');
{$ifdef FPC_HAS_TYPE_EXTENDED}
  TValue.Make(Nil, TypeInfo(Extended), value);
  CheckEquals(10, value.DataSize, 'Size of Extended differs');
{$endif}
  TValue.Make(Nil, TypeInfo(Currency), value);
  CheckEquals(SizeOf(Currency), value.DataSize, 'Size of Currency differs');
  TValue.Make(Nil, TypeInfo(Comp), value);
  CheckEquals(SizeOf(Comp), value.DataSize, 'Size of Comp differs');
  TValue.Make(Nil, TypeInfo(ShortString), value);
  CheckEquals(254, value.DataSize, 'Size of ShortString differs');
  TValue.Make(Nil, TypeInfo(AnsiString), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of Pointer differs');
  TValue.Make(Nil, TypeInfo(WideString), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of WideString differs');
  TValue.Make(Nil, TypeInfo(UnicodeString), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of UnicodeString differs');
  TValue.Make(Nil, TypeInfo(TObject), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of TObject differs');
  TValue.Make(Nil, TypeInfo(TClass), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of TClass differs');
  TValue.Make(Nil, TypeInfo(IInterface), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of IInterface differs');
  TValue.Make(Nil, TypeInfo(TTestRecord), value);
  CheckEquals(SizeOf(TTestRecord), value.DataSize, 'Size of TTestRecord differs');
  TValue.Make(Nil, TypeInfo(TTestProc), value);
  CheckEquals(SizeOf(TTestProc), value.DataSize, 'Size of TTestProc differs');
  TValue.Make(Nil, TypeInfo(TTestMethod), value);
  CheckEquals(SizeOf(TTestMethod), value.DataSize, 'Size of TTestMethod differs');
  TValue.Make(Nil, TypeInfo(TArrayOfLongintStatic), value);
  CheckEquals(SizeOf(TArrayOfLongintStatic), value.DataSize, 'Size of TArrayOfLongintStatic differs');
  TValue.Make(Nil, TypeInfo(TArrayOfLongintDyn), value);
  CheckEquals(SizeOf(TArrayOfLongintDyn), value.DataSize, 'Size of TArrayOfLongintDyn differs');
  TValue.Make(Nil, TypeInfo(TTestEnum), value);
  CheckEquals(SizeOf(TTestEnum), value.DataSize, 'Size of TTestEnum differs');
  TValue.Make(Nil, TypeInfo(TTestSet), value);
  CheckEquals(SizeOf(TTestSet), value.DataSize, 'Size of TTestSet differs');
  TValue.Make(Nil, TypeInfo(Pointer), value);
  CheckEquals(SizeOf(Pointer), value.DataSize, 'Size of Pointer differs');
end;

procedure TTestCase1.TestIsManaged;
begin
  CheckEquals(true, IsManaged(TypeInfo(ansistring)), 'IsManaged for tkAString');
  CheckEquals(true, IsManaged(TypeInfo(widestring)), 'IsManaged for tkWString');
  CheckEquals(true, IsManaged(TypeInfo(Variant)), 'IsManaged for tkVariant');
  CheckEquals(true, IsManaged(TypeInfo(TArrayOfManagedRec)),
    'IsManaged for tkArray (with managed ElType)');
  CheckEquals(true, IsManaged(TypeInfo(TArrayOfString)),
    'IsManaged for tkArray (with managed ElType)');
  CheckEquals(true, IsManaged(TypeInfo(TManagedRec)), 'IsManaged for tkRecord');
  {$ifdef fpc}
  CheckEquals(true, IsManaged(TypeInfo(TManagedRecOp)), 'IsManaged for tkRecord');
  {$endif}
  CheckEquals(true, IsManaged(TypeInfo(IInterface)), 'IsManaged for tkInterface');
  CheckEquals(true, IsManaged(TypeInfo(TManagedObj)), 'IsManaged for tkObject');
  {$ifdef fpc}
  CheckEquals(true, IsManaged(TypeInfo(specialize TArray<byte>)), 'IsManaged for tkDynArray');
  {$else}
  CheckEquals(true, IsManaged(TypeInfo(TArray<byte>)), 'IsManaged for tkDynArray');
  {$endif}
  CheckEquals(true, IsManaged(TypeInfo(unicodestring)), 'IsManaged for tkUString');
  CheckEquals(false, IsManaged(TypeInfo(shortstring)), 'IsManaged for tkSString');
  CheckEquals(false, IsManaged(TypeInfo(Byte)), 'IsManaged for tkInteger');
  CheckEquals(false, IsManaged(TypeInfo(Char)), 'IsManaged for tkChar');
  CheckEquals(false, IsManaged(TypeInfo(TTestEnum)), 'IsManaged for tkEnumeration');
  CheckEquals(false, IsManaged(TypeInfo(Single)), 'IsManaged for tkFloat');
  CheckEquals(false, IsManaged(TypeInfo(TTestSet)), 'IsManaged for tkSet');
  {$ifdef fpc}
  CheckEquals(false, IsManaged(TypeInfo(TTestMethod)), 'IsManaged for tkMethod');
  {$else}
  { Delphi bug (or sabotage). For some reason Delphi considers method pointers to be managed (only in newer versions, probably since XE7) :/ }
  CheckEquals({$if RTLVersion>=28}true{$else}false{$endif}, IsManaged(TypeInfo(TTestMethod)), 'IsManaged for tkMethod');
  {$endif}
  CheckEquals(false, IsManaged(TypeInfo(TArrayOfByte)),
    'IsManaged for tkArray (with non managed ElType)');
  CheckEquals(false, IsManaged(TypeInfo(TArrayOfNonManagedRec)),
    'IsManaged for tkArray (with non managed ElType)');
  CheckEquals(false, IsManaged(TypeInfo(TNonManagedRec)), 'IsManaged for tkRecord');
  CheckEquals(false, IsManaged(TypeInfo(TObject)), 'IsManaged for tkClass');
  CheckEquals(false, IsManaged(TypeInfo(TNonManagedObj)), 'IsManaged for tkObject');
  CheckEquals(false, IsManaged(TypeInfo(WideChar)), 'IsManaged for tkWChar');
  CheckEquals(false, IsManaged(TypeInfo(Boolean)), 'IsManaged for tkBool');
  CheckEquals(false, IsManaged(TypeInfo(Int64)), 'IsManaged for tkInt64');
  CheckEquals(false, IsManaged(TypeInfo(UInt64)), 'IsManaged for tkQWord');
  {$ifdef fpc}
  CheckEquals(false, IsManaged(TypeInfo(ICORBATest)), 'IsManaged for tkInterfaceRaw');
  {$endif}
  CheckEquals(false, IsManaged(TypeInfo(TTestProc)), 'IsManaged for tkProcVar');
  CheckEquals(false, IsManaged(TypeInfo(TTestHelper)), 'IsManaged for tkHelper');
  {$ifdef fpc}
  CheckEquals(false, IsManaged(TypeInfo(file)), 'IsManaged for tkFile');
  {$endif}
  CheckEquals(false, IsManaged(TypeInfo(TClass)), 'IsManaged for tkClassRef');
  CheckEquals(false, IsManaged(TypeInfo(Pointer)), 'IsManaged for tkPointer');
  CheckEquals(false, IsManaged(nil), 'IsManaged for nil');
end;

{$ifdef fpc}
procedure TTestCase1.TestOpenArrayToDyn;

  procedure OpenArrayProc(aArr: array of LongInt);
  var
    value: TValue;
  begin
{$ifndef InLazIDE}
    value := specialize OpenArrayToDynArrayValue<LongInt>(aArr);
{$endif}
    CheckEquals(value.IsArray, True);
    CheckEquals(value.IsOpenArray, False);
    CheckEquals(value.IsObject, False);
    CheckEquals(value.IsOrdinal, False);
    CheckEquals(value.IsClass, False);
    CheckEquals(value.GetArrayLength, 2);
    CheckEquals(value.GetArrayElement(0).AsInteger, 42);
    CheckEquals(value.GetArrayElement(1).AsInteger, 84);
    value.SetArrayElement(0, 21);
    { since this is a copy the original array is not modified! }
    CheckEquals(aArr[0], 42);
  end;

begin
  OpenArrayProc([42, 84]);
end;
{$endif}

procedure TTestCase1.TestInterface;
var
  context: TRttiContext;
  t: TRttiType;
  ti1, ti2: TRttiInterfaceType;
  methods: {$ifdef fpc}specialize{$endif} TArray<TRttiMethod>;
  params: {$ifdef fpc}specialize{$endif} TArray<TRttiParameter>;
  method: TRttiMethod;
  param: TRttiParameter;
  flag: TParamFlag;
begin
  context := TRttiContext.Create;
  try
    t := context.GetType(TypeInfo(IInterface));
    Check(t is TRttiInterfaceType, 'Type is not an interface type');

    Check(not Assigned(t.BaseType), 'Base type is assigned');

    ti1 := TRttiInterfaceType(t);
    Check(not Assigned(ti1.BaseType), 'Base type is assigned');

    methods := t.GetMethods;
    CheckEquals(0, Length(methods), 'Overall method count does not match');

    methods := t.GetDeclaredMethods;
    CheckEquals(0, Length(methods), 'Declared method conut does not match');

    t := context.GetType(TypeInfo(ITestInterface));
    Check(t is TRttiInterfaceType, 'Type is not an interface type');

    Check(Assigned(t.BaseType), 'Base type is not assigned');
    Check(t.BaseType = TRttiType(ti1), 'Base type does not match');

    ti2 := TRttiInterfaceType(t);
    Check(Assigned(ti2.BaseType), 'Base type is not assigned');
    Check(ti2.BaseType = ti1, 'Base type does not match');

    methods := t.GetMethods;
    CheckEquals(4, Length(methods), 'Overall method count does not match');

    methods := t.GetDeclaredMethods;
    CheckEquals(4, Length(methods), 'Declared method count does not match');

    method := methods[0];
    CheckEquals(method.Name, 'Test', 'Method name of Test does not match');
    Check(method.CallingConvention = DefaultCC, 'Calling convention of Test does not match');
    Check(method.MethodKind = mkProcedure, 'Method kind of Test does not match');
    Check(method.DispatchKind = dkInterface, 'Dispatch kind of Test does not match');
    Check(not Assigned(method.CodeAddress), 'Code address of Test is not Nil');
    CheckEquals(method.VirtualIndex, 3, 'Virtual index of Test does not match');
    Check(not Assigned(method.ReturnType), 'Return type of Test is not Nil');
    params := method.GetParameters;
    CheckEquals(0, Length(params), 'Parameter count of Test does not match');

    method := methods[1];
    CheckEquals(method.Name, 'Test2', 'Method name of Test2 does not match');
    Check(method.CallingConvention = DefaultCC, 'Calling convention of Test2 does not match');
    Check(method.MethodKind = mkFunction, 'Method kind of Test2 does not match');
    Check(method.DispatchKind = dkInterface, 'Dispatch kind of Test2 does not match');
    Check(not Assigned(method.CodeAddress), 'Code address of Test2 is not Nil');
    CheckEquals(method.VirtualIndex, 4, 'Virtual index of Test2 does not match');
    Check(Assigned(method.ReturnType), 'Return type of Test2 is Nil');
    Check(method.ReturnType.TypeKind = tkInteger, 'Return type of Test2 is not an ordinal');
    params := method.GetParameters;
    CheckEquals(0, Length(params), 'Parameter count of Test2 does not match');

    method := methods[2];
    CheckEquals(method.Name, 'Test3', 'Method name of Test3 does not match');
    Check(method.CallingConvention = DefaultCC, 'Calling convention of Test3 does not match');
    Check(method.MethodKind = mkProcedure, 'Method kind of Test3 does not match');
    Check(method.DispatchKind = dkInterface, 'Dispatch kind of Test3 does not match');
    Check(not Assigned(method.CodeAddress), 'Code address of Test3 is not Nil');
    CheckEquals(method.VirtualIndex, 5, 'Virtual index of Test3 does not match');
    Check(not Assigned(method.ReturnType), 'Return type of Test3 is not Nil');

    params := method.GetParameters;
    CheckEquals(4, Length(params), 'Parameter count of Test3 does not match');

    param := params[0];
    CheckEquals(param.Name, 'aArg1', 'Parameter name of Test3.aArg1 does not match');
    Check(param.Flags = [], 'Parameter flags of Test3.aArg1 do not match');
    Check(Assigned(param.ParamType), 'Parameter type of Test3.aArg1 is Nil');
    Check(param.ParamType.TypeKind = tkInteger, 'Parameter type of Test3.aArg1 is not an ordinal');

    param := params[1];
    CheckEquals(param.Name, 'aArg2', 'Parameter name of Test3.aArg2 does not match');
    Check(param.Flags = [pfConst], 'Parameter flags of Test3.aArg2 do not match');
    Check(Assigned(param.ParamType), 'Parameter type of Test3.aArg2 is Nil');
    Check(param.ParamType.TypeKind = tkAnsiString, 'Parameter type of Test3.aArg2 is not a string');

    param := params[2];
    CheckEquals(param.Name, 'aArg3', 'Parameter name of Test3.aArg3 does not match');
    Check(param.Flags = [pfVar], 'Parameter flags of Test3.aArg3 do not match');
    Check(Assigned(param.ParamType), 'Parameter type of Test3.aArg3 is Nil');
    Check(param.ParamType.TypeKind = {$ifdef fpc}tkBool{$else}tkEnumeration{$endif}, 'Parameter type of Test3.aArg3 is not a boolean');

    param := params[3];
    CheckEquals(param.Name, 'aArg4', 'Parameter name of Test3.aArg4 does not match');
    Check(param.Flags = [pfOut], 'Parameter flags of Test3.aArg4 do not match');
    Check(Assigned(param.ParamType), 'Parameter type of Test3.aArg4 is Nil');
    Check(param.ParamType.TypeKind = tkInteger, 'Parameter type of Test3.aArg4 is not a string');

    method := methods[3];
    CheckEquals(method.Name, 'Test4', 'Method name of Test4 does not match');
    Check(method.CallingConvention = DefaultCC, 'Calling convention of Test4 does not match');
    Check(method.MethodKind = mkFunction, 'Method kind of Test4 does not match');
    Check(method.DispatchKind = dkInterface, 'Dispatch kind of Test4 does not match');
    Check(not Assigned(method.CodeAddress), 'Code address of Test4 is not Nil');
    CheckEquals(method.VirtualIndex, 6, 'Virtual index of Test4 does not match');
    Check(Assigned(method.ReturnType), 'Return type of Test4 is not Nil');
    Check(method.ReturnType.TypeKind = tkAnsiString, 'Return type of Test4 is not a string');

    params := method.GetParameters;
    CheckEquals(2, Length(params), 'Parameter count of Test4 does not match');

    param := params[0];
    CheckEquals(param.Name, 'aArg1', 'Parameter name of Test4.aArg1 does not match');
    Check(param.Flags = [pfArray, pfReference], 'Parameter flags of Test4.aArg1 do not match');
    Check(Assigned(param.ParamType), 'Parameter type of Test4.aArg1 is Nil');
    Check(param.ParamType.TypeKind = tkInteger, 'Parameter type of Test4.aArg1 is not an ordinal');

    param := params[1];
    CheckEquals(param.Name, 'aArg2', 'Parameter name of Test4.aArg2 does not match');
    Check(param.Flags = [pfArray, pfReference], 'Parameter flags of Test4.aArg2 do not match');
    Check(Assigned(param.ParamType), 'Parameter type of Test4.aArg2 is Nil');
    Check(param.ParamType.TypeKind = tkRecord, 'Parameter type of Test4.aArg2 is not a record');
  finally
    context.Free;
  end;
end;

procedure TTestCase1.TestRawThunk;
var
  intf: IInterface;
begin
  { we test the raw thunking by instantiating a TVirtualInterface of IInterface }
  { this does not require a function call manager as the thunking is implemented
    directly inside the RTTI unit }
  try
    intf := TVirtualInterface.Create(PTypeInfo(TypeInfo(IInterface))) as IInterface;
  except
    on e: ENotImplemented do
      Ignore('RawThunk not implemented');
  end;
  { if all went well QueryInterface and _AddRef were called and now we call
    _Release as well }
  intf := Nil;
end;

{$ifdef fpc}
procedure TTestCase1.TestInterfaceRaw;
var
  context: TRttiContext;
  t: TRttiType;
  ti: TRttiInterfaceType;
begin
  context := TRttiContext.Create;
  try
    t := context.GetType(TypeInfo(ICORBATest));
    Check(t is TRttiInterfaceType, 'Type is not a raw interface type');

    Check(not Assigned(t.BaseType), 'Base type is assigned');

    ti := TRttiInterfaceType(t);
    Check(not Assigned(ti.BaseType), 'Base type is assigned');
  finally
    context.Free;
  end;
end;
{$endif}

procedure TTestCase1.TestProcVar;
var
  context: TRttiContext;
  t: TRttiType;
  p: TRttiProcedureType;
  params: {$ifdef fpc}specialize{$endif} TArray<TRttiParameter>;
begin
  context := TRttiContext.Create;
  try
    t := context.GetType(PTypeInfo(TypeInfo(TTestProc)));
    Check(Assigned(t), 'Rtti Type is Nil');
    Check(t is TRttiInvokableType, 'Rtti Type is not an invokeable');
    Check(t is TRttiProcedureType, 'Rtti Type is not a procedure type');

    p := t as TRttiProcedureType;
    Check(p.CallingConvention = DefaultCC, 'Calling convention does not match');
    Check(not Assigned(p.ReturnType), 'Return type is assigned');
    CheckEquals(0, Length(p.GetParameters), 'Procedure variable has parameters');

    t := context.GetType(PTypeInfo(TypeInfo(TTestFunc1)));
    Check(Assigned(t), 'Rtti Type is Nil');
    Check(t is TRttiInvokableType, 'Rtti Type is not an invokeable');
    Check(t is TRttiProcedureType, 'Rtti Type is not a procedure type');

    p := t as TRttiProcedureType;
    Check(p.CallingConvention = DefaultCC, 'Calling convention does not match');
    Check(Assigned(p.ReturnType), 'Return type is not assigned');
    //Check(p.ReturnType is TRttiOrdinalType, 'Return type is not an ordinal type');
    CheckEquals(0, Length(p.GetParameters), 'Procedure variable has parameters');

    t := context.GetType(PTypeInfo(TypeInfo(TTestFunc2)));
    Check(Assigned(t), 'Rtti Type is Nil');
    Check(t is TRttiInvokableType, 'Rtti Type is not an invokeable');
    Check(t is TRttiProcedureType, 'Rtti Type is not a procedure type');

    p := t as TRttiProcedureType;
    Check(p.CallingConvention = DefaultCC, 'Calling convention does not match');
    Check(Assigned(p.ReturnType), 'Return type is not assigned');
    Check(p.ReturnType is TRttiStringType, 'Return type is not a string type');

    params := p.GetParameters;
    CheckEquals(2, Length(params), 'Procedure variable has incorrect amount of parameters');

    Check(params[0].ParamType.TypeKind in [tkInteger, tkInt64], 'Parameter 1 is not an ordinal type');
    //Check(params[0].ParamType is TRttiOrdinalType, 'Parameter 1 is not an ordinal type');
    Check(pfArray in params[1].Flags, 'Parameter 2 is not an array');
    Check(params[1].ParamType.TypeKind in [tkInteger, tkInt64], 'Parameter 2 is not an ordinal array');
  finally
    context.Free;
  end;
end;

procedure TTestCase1.TestMethod;
var
  context: TRttiContext;
  t: TRttiType;
  m: TRttiMethodType;
  params: {$ifdef fpc}specialize{$endif} TArray<TRttiParameter>;
begin
  context := TRttiContext.Create;
  try
    t := context.GetType(PTypeInfo(TypeInfo(TTestMethod)));
    Check(Assigned(t), 'Rtti Type is Nil');
    Check(t is TRttiInvokableType, 'Rtti Type is not an invokeable');
    Check(t is TRttiMethodType, 'Rtti Type is not a method type');

    m := t as TRttiMethodType;
    Check(m.CallingConvention = DefaultCC, 'Calling convention does not match');
    Check(not Assigned(m.ReturnType), 'Return type is assigned');
    CheckEquals(0, Length(m.GetParameters), 'Method variable has parameters');

    t := context.GetType(PTypeInfo(TypeInfo(TTestMethod1)));
    Check(Assigned(t), 'Rtti Type is Nil');
    Check(t is TRttiInvokableType, 'Rtti Type is not an invokeable');
    Check(t is TRttiMethodType, 'Rtti Type is not a method type');

    m := t as TRttiMethodType;
    Check(m.CallingConvention = DefaultCC, 'Calling convention does not match');
    Check(Assigned(m.ReturnType), 'Return type is not assigned');
    //Check(p.ReturnType is TRttiOrdinalType, 'Return type is not an ordinal type');
    CheckEquals(0, Length(m.GetParameters), 'Method variable has parameters');

    t := context.GetType(PTypeInfo(TypeInfo(TTestMethod2)));
    Check(Assigned(t), 'Rtti Type is Nil');
    Check(t is TRttiInvokableType, 'Rtti Type is not an invokeable');
    Check(t is TRttiMethodType, 'Rtti Type is not a method type');

    m := t as TRttiMethodType;
    Check(m.CallingConvention = DefaultCC, 'Calling convention does not match');
    Check(Assigned(m.ReturnType), 'Return type is not assigned');
    Check(m.ReturnType is TRttiStringType, 'Return type is not a string type');

    params := m.GetParameters;
    CheckEquals(2, Length(params), 'Method variable has incorrect amount of parameters');

    Check(params[0].ParamType.TypeKind in [tkInteger, tkInt64], 'Parameter 1 is not an ordinal type');
    //Check(params[0].ParamType is TRttiOrdinalType, 'Parameter 1 is not an ordinal type');
    Check(pfArray in params[1].Flags, 'Parameter 2 is not an array');
    Check(params[1].ParamType.TypeKind in [tkInteger, tkInt64], 'Parameter 2 is not an ordinal array');
  finally
    context.Free;
  end;
end;

initialization
{$ifdef fpc}
  RegisterTest(TTestCase1);
{$else fpc}
  RegisterTest(TTestCase1.Suite);
{$endif fpc}
end.

