unit testjsonrtti;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry, typinfo, fpjson,
  dateutils, testcomps, testjsondata, fpjsonrtti;

type

  { TTestJSONStreamer }

  TTestJSONStreamer = class(TTestJSON)
  private
    FRJ : TJSONStreamer;
    FSR : TJSONObject;
    FToFree : TObject;
    FCalled : Boolean;
    procedure DoStreamProperty1(Sender: TObject; AObject: TObject; Info: PPropInfo; var Res: TJSONData);
  protected
    procedure SetUp; override; 
    procedure TearDown; override;
    Procedure AssertEquals(AMessage : String; Expected,Actual : TJSONType); overload;
    Procedure AssertPropCount(ACount : Integer);
    Function  AssertProperty(APropName : String; AType : TJSONType) : TJSONData;
    Procedure AssertProp(APropName : String; AValue : Boolean);
    Procedure AssertProp(APropName : String; AValue : Integer);
    procedure AssertProp(APropName : String; AValue: String);
    procedure AssertProp(APropName : String; AValue: TJSONFloat);
    procedure AssertProp(APropName : String; AValue : Array of String);
    procedure AssertProp(APropName : String; AValue : Array of Integer);
    function CreateVariantComp : TVariantComponent;
    procedure AssertNullProp(APropName : String);
    Function AssertObjectProp(APropName : String) : TJSONObject;
    Function AssertArrayProp(APropName : String) : TJSONArray;
    Function StreamObject(AObject : TObject) : TJSONObject;
    Property RJ : TJSONStreamer read FRJ;
    Property SR : TJSONObject Read FSR Write FSR;
  published
    procedure TestNil;
    procedure TestEmpty;
    procedure TestEmptyComponent;
    procedure TestWriteBoolean;
    procedure TestWriteInteger;
    procedure TestWriteString;
    procedure TestWriteFloat;
    procedure TestWriteFloat2;
    procedure TestWriteFloat3;
    procedure TestWriteFloat4;
    procedure TestWriteFloat5;
    procedure TestEnum1;
    procedure TestEnum2;
    Procedure TestSet1;
    Procedure TestSet2;
    Procedure TestSet3;
    Procedure TestSet4;
    Procedure TestObjectNil;
    Procedure TestComponentProp1;
    Procedure TestComponentProp2;
    Procedure TestCollectionProp1;
    Procedure TestCollectionProp2;
    Procedure TestPersistentProp1;
    Procedure TestStringsProp1;
    Procedure TestStringsProp2;
    procedure TestStringsProp3;
    procedure TestStringsProp4;
    procedure TestStringsArray;
    procedure TestStringsObject;
    procedure TestStringsStream1;
    procedure TestStringsStream2;
    procedure TestStringsStream3;
    procedure TestStringsStream4;
    procedure TestStringsStream5;
    procedure TestCollectionStream;
    procedure TestCollectionStream2;
    procedure TestOnStreamProperty;
    Procedure TestDateTimeProp;
    Procedure TestDateTimePropDefaultString;
    Procedure TestDateTimePropDefaultStringTime;
    Procedure TestDateTimeProp2;
    Procedure TestDateTimeProp3;
    procedure TestDateTimeProp4;
    procedure TestDateTimeProp5;
    procedure TestDateTimeProp6;
    procedure TestDateTimeProp7;
    Procedure TestVariantShortint;
    Procedure TestVariantbyte;
    Procedure TestVariantword;
    Procedure TestVariantsmallint;
    Procedure TestVariantinteger;
    Procedure TestVariantlongword;
    Procedure TestVariantint64;
    Procedure TestVariantqword;
    Procedure TestVariantsingle;
    Procedure TestVariantdouble;
    Procedure TestVariantCurrency;
    Procedure TestVariantString;
    Procedure TestVariantolestr;
    Procedure TestVariantboolean;
    Procedure TestVariantDate;
    procedure TestVariantDate2;
    Procedure TestVariantArray;
    Procedure TestMultipleProps;
    Procedure TestObjectToJSONString;
    Procedure TestStringsToJSONString;
    Procedure TestCollectionToJSONString;
    procedure TestTListToJSONString;
    Procedure TestChildren;
    Procedure TestChildren2;
    Procedure TestLowercase;
  end;

  { TTestJSONDeStreamer }

  TTestJSONDeStreamer = class(TTestJSON)
  private
    FDS : TJSONDeStreamer;
    FJD : TJSONData;
    FToFree : TObject;
    procedure DeStream(JSON: TJSONStringType; AObject: TObject);
    procedure DeStream(JSON: TJSONObject; AObject: TObject);
    procedure DoDateTimeFormat;
    Procedure DoNullError;
  protected
    procedure SetUp; override;
    procedure TearDown; override;
    Procedure AssertVarType(Msg : String; AVarType : TVarType; Const Variant : Variant);
    Property DS : TJSONDeStreamer Read FDS;
    Property JD : TJSONData Read FJD Write FJD;
    Property Tofree : TObject Read FToFree Write FToFree;
  published
    Procedure TestVariantInteger;
    Procedure TestVariantFloat;
    Procedure TestVariantInt64;
    Procedure TestVariantBoolean;
    Procedure TestVariantNull;
    Procedure TestVariantString;
    Procedure TestVariantArray;
    procedure TestEmpty;
    procedure TestNullError;
    procedure TestNull;
    procedure TestBoolean;
    procedure TestInteger;
    procedure TestIntegerCaseInsensitive;
    procedure TestIntegerCaseSensitive;
    procedure TestString;
    procedure TestFloat;
    procedure TestFloat2;
    procedure TestFloat3;
    procedure TestFloat4;
    procedure TestFloat5;
    procedure TestDateTime;
    procedure TestDateTimeFormat;
    procedure TestEnum1;
    procedure TestEnum2;
    Procedure TestSet1;
    Procedure TestSet2;
    Procedure TestSet3;
    Procedure TestSet4;
    Procedure TestVariantProp;
    procedure TestCollection;
    procedure TestCollection2;
    procedure TestCollection3;
    procedure TestCollection4;
    procedure TestCollection5;
    procedure TestCollection6;
    procedure TestCollectionProp;
    procedure TestCollectionProp2;
    procedure TestStrings;
    procedure TestStrings2;
    procedure TestStrings3;
  end;

implementation

uses variants;

{ TTestJSONDeStreamer }

procedure TTestJSONDeStreamer.SetUp;
begin
  inherited SetUp;
  FDS:=TJSONDeStreamer.Create(Nil)
end;

procedure TTestJSONDeStreamer.TearDown;
begin
  FreeAndNil(FDS);
  FreeAndNil(FJD);
  FreeAndNil(FToFree);
  inherited TearDown;
end;

procedure TTestJSONDeStreamer.AssertVarType(Msg: String; AVarType: TVarType;
  const Variant: Variant);
begin
  AssertEquals(Msg,VarTypeAsText(AVarType),VarTypeAsText(VarType(Variant)));
end;

procedure TTestJSONDeStreamer.TestVariantInteger;

Var
  V : Variant;

begin
  JD:=TJSOnIntegerNumber.Create(12);
  V:=DS.JSONToVariant(JD);
  AssertVarType('Integer data',varInteger,V);
  AssertEquals('Integer value',12,V);
end;

procedure TTestJSONDeStreamer.TestVariantFloat;
Var
  V : Variant;

begin
  JD:=TJSOnFloatNumber.Create(1.2);
  V:=DS.JSONToVariant(JD);
  AssertVarType('Double data',varDouble,V);
  AssertEquals('Float value',1.2,V);
end;

procedure TTestJSONDeStreamer.TestVariantInt64;
Var
  V : Variant;

begin
  JD:=TJSONInt64Number.Create(123);
  V:=DS.JSONToVariant(JD);
  AssertVarType('Int64 data',varInt64,V);
  AssertEquals('Int64 value',123,V);
end;

procedure TTestJSONDeStreamer.TestVariantBoolean;
Var
  V : Variant;

begin
  JD:=TJSONBoolean.Create(True);
  V:=DS.JSONToVariant(JD);
  AssertVarType('Boolean data',varBoolean,V);
  AssertEquals('Boolean value',True,V);
end;

procedure TTestJSONDeStreamer.TestVariantNull;
Var
  V : Variant;

begin
  JD:=TJSONNull.Create();
  V:=DS.JSONToVariant(JD);
  AssertVarType('Null data',varNull,V);
end;

procedure TTestJSONDeStreamer.TestVariantString;
Var
  V : Variant;

begin
  JD:=TJSONString.Create('A string');
  V:=DS.JSONToVariant(JD);
  AssertVarType('String data',varOleStr,V);
  AssertEquals('String data','A string',V);
end;

procedure TTestJSONDeStreamer.TestVariantArray;
Var
  V : Variant;
begin
  JD:=TJSONArray.Create([1,2,3]);
  V:=DS.JSONToVariant(JD);
  AssertEQuals('Variant is array',true,VarIsArray(V));
  AssertEquals('Lower bound is zero ',0,VarArrayLowBound(V,1));
  AssertEquals('Higher bound is count-1 ',2,VarArrayHighBound(V,1));
  AssertEquals('Element 0 value correct ',1,V[0]);
  AssertEquals('Element 1 value correct ',2,V[1]);
  AssertEquals('Element 2 value correct ',3,V[2]);
end;

procedure TTestJSONDeStreamer.TestEmpty;
begin
  FTofree:=TComponent.Create(Nil);
  DS.JSONToObject('{}',FTofree);
  AssertEquals('Empty name','',TComponent(FToFree).Name);
  AssertEquals('Empty Tag',0,TComponent(FToFree).Tag);
end;

procedure TTestJSONDeStreamer.TestNullError;

begin
  AssertException('Null error',EJSON, @DoNullError);
end;

procedure TTestJSONDeStreamer.TestNull;
Var
  B : TBooleanComponent;

begin
  B:=TBooleanComponent.Create(Nil);
  DS.Options:=DS.Options+[jdoIgnoreNulls];
  DeStream('{ "BooleanProp" : null }',B);
  AssertEquals('Correct boolean value',False,B.BooleanProp);
end;

procedure TTestJSONDeStreamer.DeStream(JSON : TJSONStringType; AObject : TObject);

begin
  FToFree:=AObject;
  DS.JSONToObject(JSON,FTofree);
end;

procedure TTestJSONDeStreamer.DeStream(JSON: TJSONObject; AObject: TObject);
begin
  FToFree:=AObject;
  JD:=JSON;
  DS.JSONToObject(JSON,FTofree);
end;

procedure TTestJSONDeStreamer.TestBoolean;

Var
  B : TBooleanComponent;

begin
  B:=TBooleanComponent.Create(Nil);
  DeStream('{ "BooleanProp" : true }',B);
  AssertEquals('Correct boolean value',true,B.BooleanProp);
end;

procedure TTestJSONDeStreamer.TestInteger;

Var
  B : TIntegerComponent;

begin
  B:=TIntegerComponent.Create(Nil);
  DeStream('{ "IntProp" : 22 }',B);
  AssertEquals('Correct integer value',22,B.IntProp);
end;

procedure TTestJSONDeStreamer.TestIntegerCaseInsensitive;

Var
  B : TIntegerComponent;

begin
  DS.Options:=DS.Options+[jdoCaseInsensitive];
  B:=TIntegerComponent.Create(Nil);
  DeStream('{ "intprop" : 22 }',B);
  AssertEquals('Correct integer value',22,B.IntProp);
end;

procedure TTestJSONDeStreamer.TestIntegerCaseSensitive;

Var
  B : TIntegerComponent;

begin
  DS.Options:=DS.Options;
  B:=TIntegerComponent.Create(Nil);
  B.IntProp:=0;
  DeStream('{ "intprop" : 22 }',B);
  AssertEquals('Correct integer value not reas',0,B.IntProp);
end;

procedure TTestJSONDeStreamer.TestString;

Var
  B : TStringComponent;

begin
  B:=TStringComponent.Create(Nil);
  DeStream('{ "StringProp" : "A nice string"}',B);
  AssertEquals('Correct string value','A nice string',B.StringProp);
end;

procedure TTestJSONDeStreamer.TestFloat;

Var
  B : TSingleComponent;

begin
  B:=TSingleComponent.Create(Nil);
  DeStream('{ "SingleProp" : 2.34 }',B);
  AssertEquals('Correct single value',2.34,B.SingleProp);
end;

procedure TTestJSONDeStreamer.TestFloat2;

Var
  B : TDoubleComponent;

begin
  B:=TDoubleComponent.Create(Nil);
  DeStream('{ "DoubleProp" : 3.45 }',B);
  AssertEquals('Correct Double value',3.45,B.DoubleProp);
end;

procedure TTestJSONDeStreamer.TestFloat3;
Var
  B : TExtendedComponent;

begin
  B:=TExtendedComponent.Create(Nil);
  DeStream('{ "ExtendedProp" : 4.56 }',B);
  AssertEquals('Correct extended value',4.56,B.ExtendedProp);
end;

procedure TTestJSONDeStreamer.TestFloat4;

Var
  B : TCompComponent;

begin
  B:=TCompComponent.Create(Nil);
  DeStream('{ "CompProp" : 5.67 }',B);
  AssertEquals('Correct comp value',round(5.67),B.CompProp);
end;

procedure TTestJSONDeStreamer.TestFloat5;
Var
  B : TCurrencyComponent;

begin
  B:=TCurrencyComponent.Create(Nil);
  DeStream('{ "CurrencyProp" : 5.67 }',B);
  AssertEquals('Correct string value',5.67,B.CurrencyProp);
end;

procedure TTestJSONDeStreamer.TestDateTime;

Var
  E : TDateTimeComponent;
  D : TDateTime;

begin
  E:=TDateTimeComponent.Create(Nil);
  D:= RecodeMillisecond(Now,0);
  DeStream('{"DateTimeProp" : "'+FormatDateTime(RFC3339DateTimeFormat,D)+'"}',E);
  AssertEquals('Correct value',D,E.DateTimeProp);
end;

procedure TTestJSONDeStreamer.DoDateTimeFormat;

begin
  DeStream('{"DateTimeProp" : "'+DateTimeToStr(RecodeMillisecond(Now,0))+'"}',FToFree);
end;

procedure TTestJSONDeStreamer.DoNullError;
Var
  B : TBooleanComponent;

begin
  B:=TBooleanComponent.Create(Nil);
  Destream('{ "BooleanProp" : null }',B);
end;

procedure TTestJSONDeStreamer.TestDateTimeFormat;

Const
  ISO8601 = 'yyyymmdd"T"hhnnss';

Var
  E : TDateTimeComponent;
  D : TDateTime;

begin
  E:=TDateTimeComponent.Create(Nil);
  D:=RecodeMillisecond(Now,0);
  DS.DateTimeFormat:=ISO8601;
  DeStream('{"DateTimeProp" : "'+FormatDateTime(Iso8601,D)+'"}',E);
  AssertEquals('Correct value',D,E.DateTimeProp);
  AssertException('Error if  string does not correspond to specified format',EConvertError,@DoDateTimeFormat);
end;

procedure TTestJSONDeStreamer.TestEnum1;

Var
  E : TEnumcomponent;

begin
  E:=TEnumComponent.Create(Nil);
  DeStream('{ "Dice" : 2 }',E);
  AssertEquals('Correct value',2,Ord(E.Dice));
end;

procedure TTestJSONDeStreamer.TestEnum2;

Var
  E : TEnumcomponent;

begin
  E:=TEnumComponent.Create(Nil);
  DeStream('{ "Dice" : "three" }',E);
  AssertEquals('Correct value',GetEnumName(TypeInfo(TDice),Ord(Three)),GetEnumName(TypeInfo(TDice),Ord(E.Dice)));
end;

procedure TTestJSONDeStreamer.TestSet1;

Var
  T : TSetComponent;

begin
  T:=TSetComponent.Create(Nil);
  DeStream('{ "Throw" : "one,two" }',T);
  If not (T.Throw=[one,two]) then
    Fail('Correct value for throw');
end;

procedure TTestJSONDeStreamer.TestSet2;

Var
  T : TSetComponent;

begin
  T:=TSetComponent.Create(Nil);
  DeStream('{ "Throw" : "[one,two]" }',T);
  If not (T.Throw=[one,two]) then
    Fail('Correct value for throw');
end;

procedure TTestJSONDeStreamer.TestSet3;

Var
  T : TSetComponent;

begin
  T:=TSetComponent.Create(Nil);
  DeStream('{ "Throw" : [ "one", "two"] }',T);
  If not (T.Throw=[one,two]) then
    Fail('Correct value for throw');
end;

procedure TTestJSONDeStreamer.TestSet4;

Var
  T : TSetComponent;

begin
  T:=TSetComponent.Create(Nil);
  DeStream('{ "Throw" : [ 0 , 1 ] }',T);
  If not (T.Throw=[one,two]) then
    Fail('Correct value for throw');
end;

procedure TTestJSONDeStreamer.TestVariantProp;
Var
  V : TVariantComponent;

begin
  V:=TVariantComponent.Create(Nil);
  DeStream('{ "VariantProp" : "A string" }',V);
  AssertEquals('Variant property value','A string',V.VariantProp);
end;

procedure TTestJSONDeStreamer.TestCollection;

Var
  C : TTestCollection;

begin
  C:=TTestCollection.Create;
  DeStream('[ { "StrProp" : "one" }, { "StrProp" : "two" } ]',C);
  AssertEquals('Item count',2,C.Count);
  AssertEquals('Class item 0',TTestItem,C.Items[0].ClassType);
  AssertEquals('Class item 1',TTestItem,C.Items[1].ClassType);
  AssertEquals('Class item 0','one',TTestItem(C.Items[0]).StrProp);
  AssertEquals('Class item 1','two',TTestItem(C.Items[1]).StrProp);
end;

procedure TTestJSONDeStreamer.TestCollection2;

Var
  C : TTestCollection;

begin
  C:=TTestCollection.Create;
  DeStream('{ "Items" : [ { "StrProp" : "one" }, { "StrProp" : "two" } ] }',C);
  AssertEquals('Item count',2,C.Count);
  AssertEquals('Class item 0',TTestItem,C.Items[0].ClassType);
  AssertEquals('Class item 1',TTestItem,C.Items[1].ClassType);
  AssertEquals('Class item 0','one',TTestItem(C.Items[0]).StrProp);
  AssertEquals('Class item 1','two',TTestItem(C.Items[1]).StrProp);
end;

procedure TTestJSONDeStreamer.TestCollection3;

Var
  C : TTestCollection;

begin
  C:=TTestCollection.Create;
  FTofree:=C;
  DS.JSONToCollection('{ "Items" : [ { "StrProp" : "one" }, { "StrProp" : "two" } ] }',C);
  AssertEquals('Item count',2,C.Count);
  AssertEquals('Class item 0',TTestItem,C.Items[0].ClassType);
  AssertEquals('Class item 1',TTestItem,C.Items[1].ClassType);
  AssertEquals('Class item 0','one',TTestItem(C.Items[0]).StrProp);
  AssertEquals('Class item 1','two',TTestItem(C.Items[1]).StrProp);
end;

procedure TTestJSONDeStreamer.TestCollection4;

Var
  C : TTestCollection;

begin
  C:=TTestCollection.Create;
  FTofree:=C;
  DS.JSONToCollection('[ { "StrProp" : "one" }, { "StrProp" : "two" } ]',C);
  AssertEquals('Item count',2,C.Count);
  AssertEquals('Class item 0',TTestItem,C.Items[0].ClassType);
  AssertEquals('Class item 1',TTestItem,C.Items[1].ClassType);
  AssertEquals('Class item 0','one',TTestItem(C.Items[0]).StrProp);
  AssertEquals('Class item 1','two',TTestItem(C.Items[1]).StrProp);
end;

procedure TTestJSONDeStreamer.TestCollection5;

Var
  C : TTestCollection;

begin
  C:=TTestCollection.Create;
  FTofree:=C;
  JD:=TJSONArray.Create([TJSONObject.Create(['StrProp','one']),TJSONObject.Create(['StrProp','two'])]);
  DS.JSONToCollection(JD,C);
  AssertEquals('Item count',2,C.Count);
  AssertEquals('Class item 0',TTestItem,C.Items[0].ClassType);
  AssertEquals('Class item 1',TTestItem,C.Items[1].ClassType);
  AssertEquals('Class item 0','one',TTestItem(C.Items[0]).StrProp);
  AssertEquals('Class item 1','two',TTestItem(C.Items[1]).StrProp);
end;

procedure TTestJSONDeStreamer.TestCollection6;
Var
  C : TTestCollection;

begin
  C:=TTestCollection.Create;
  FTofree:=C;
  JD:=TJSONObject.Create(['Items',TJSONArray.Create([TJSONObject.Create(['StrProp','one']),TJSONObject.Create(['StrProp','two'])])]);
  DS.JSONToCollection(JD,C);
  AssertEquals('Item count',2,C.Count);
  AssertEquals('Class item 0',TTestItem,C.Items[0].ClassType);
  AssertEquals('Class item 1',TTestItem,C.Items[1].ClassType);
  AssertEquals('Class item 0','one',TTestItem(C.Items[0]).StrProp);
  AssertEquals('Class item 1','two',TTestItem(C.Items[1]).StrProp);
end;

procedure TTestJSONDeStreamer.TestCollectionProp;

Var
  C : TCollection;

begin
  JD:=TJSONObject.Create(['Coll',TJSONArray.Create([TJSONObject.Create(['StrProp','one']),TJSONObject.Create(['StrProp','two'])])]);
  DeStream(JD as TJSONObject,TCollectionComponent.Create(Nil));
  C:=TCollectionComponent(ToFree).Coll;
  AssertEquals('Item count',2,C.Count);
  AssertEquals('Class item 0',TTestItem,C.Items[0].ClassType);
  AssertEquals('Class item 1',TTestItem,C.Items[1].ClassType);
  AssertEquals('Class item 0','one',TTestItem(C.Items[0]).StrProp);
  AssertEquals('Class item 1','two',TTestItem(C.Items[1]).StrProp);
end;

procedure TTestJSONDeStreamer.TestCollectionProp2;

Var
  C : TCollection;

begin

  DeStream('{ "Coll" : [ { "StrProp" : "one" }, { "StrProp" : "two" } ]}',TCollectionComponent.Create(Nil));
  C:=TCollectionComponent(ToFree).Coll;
  AssertEquals('Item count',2,C.Count);
  AssertEquals('Class item 0',TTestItem,C.Items[0].ClassType);
  AssertEquals('Class item 1',TTestItem,C.Items[1].ClassType);
  AssertEquals('Class item 0','one',TTestItem(C.Items[0]).StrProp);
  AssertEquals('Class item 1','two',TTestItem(C.Items[1]).StrProp);
end;

procedure TTestJSONDeStreamer.TestStrings;

Var
  S : TStrings;

begin
  S:=TStringList.Create;
  FTofree:=S;
  DS.JSONToStrings('[ "one" , "two" ]',S);
  AssertEquals('Item count',2,S.Count);
  AssertEquals('First item','one',S[0]);
  AssertEquals('First item','two',S[1]);
end;

procedure TTestJSONDeStreamer.TestStrings2;

Var
  S : TStrings;

begin
  S:=TStringList.Create;
  FTofree:=S;
  DS.JSONToStrings('{ "Strings" : [ "one" , "two" ] }',S);
  AssertEquals('Item count',2,S.Count);
  AssertEquals('First item','one',S[0]);
  AssertEquals('First item','two',S[1]);
end;

procedure TTestJSONDeStreamer.TestStrings3;
Var
  S : TStrings;

begin
  S:=TStringList.Create;
  FTofree:=S;
  DS.JSONToStrings('{ "Strings" : [ "one" , "two" ] }',S);
  AssertEquals('Item count',2,S.Count);
  AssertEquals('First item','one',S[0]);
  AssertEquals('First item','two',S[1]);
end;

{ TTestJSONStreamer }

function TTestJSONStreamer.StreamObject(AObject: TObject): TJSONObject;
begin
  FToFree:=AObject;
  FSR:=FRJ.ObjectToJSON(AObject);
  Result:=FSR;
end;

procedure TTestJSONStreamer.DoStreamProperty1(Sender: TObject; AObject: TObject;
  Info: PPropInfo; var Res: TJSONData);
begin
  If (info^.name<>'IntProp') and (info^.name<>'Name') and (info^.name<>'Tag') then
    Fail('Wrong property');
  If (info^.name='IntProp') then
    FreeAndNil(Res);
  FCalled:=true;
end;

procedure TTestJSONStreamer.SetUp;
begin
  Inherited;
  FRJ:=TJSONStreamer.Create(Nil);
end;

procedure TTestJSONStreamer.TearDown;
begin
  FreeAndNil(FSR);
  FreeAndNil(FRJ);
  FreeAndNil(FToFree);
  Inherited;
end;

procedure TTestJSONStreamer.AssertEquals(AMessage: String; Expected, Actual: TJSONType);
begin
  AssertEquals(AMessage,GetEnumName(TypeInfo(TJSONType),Ord(Expected)),
                        GetEnumName(TypeInfo(TJSONType),Ord(Actual)));
end;

procedure TTestJSONStreamer.AssertPropCount(ACount: Integer);
begin
  AssertNotNull('Result of streaming available',FSR);
  If FToFree is TComponent then
    ACount:=ACount+2; // Tag + Name
  // Writeln(FSR.ASJSON);
  AssertEquals('Property count correct',ACount,FSR.Count);
end;

function TTestJSONStreamer.AssertProperty(APropName: String; AType: TJSONType
  ): TJSONData;

Var
  i : Integer;

begin
  I:=FSR.IndexOfName(APropName);
  If (I=-1) then
    Fail('No property "'+APropName+'" available');
  Result:=FSR.Items[i];
  AssertEquals('Property "'+APropName+'" has correct type',GetEnumName(TypeInfo(TJSONType),Ord(AType)),
                                                           GetEnumName(TypeInfo(TJSONType),Ord(Result.JSONType)));
end;

procedure TTestJSONStreamer.AssertProp(APropName: String; AValue: Boolean);
begin
  AssertNotNull('Result of streaming available',FSR);
  AssertEquals('Result of streaming is TJSONObject',TJSONObject,FSR.ClassType);
  AssertEquals('Correct value',AValue,AssertProperty(APropName,jtBoolean).AsBoolean);
end;

procedure TTestJSONStreamer.AssertProp(APropName: String; AValue: Integer);
begin
  AssertNotNull('Result of streaming available',FSR);
  AssertEquals('Result of streaming is TJSONObject',TJSONObject,FSR.ClassType);
  AssertEquals('Correct value',AValue,AssertProperty(APropName,jtNumber).AsInteger);
end;

procedure TTestJSONStreamer.AssertProp(APropName: String; AValue: String);
begin
  AssertNotNull('Result of streaming available',FSR);
  AssertEquals('Result of streaming is TJSONObject',TJSONObject,FSR.ClassType);
  AssertEquals('Correct value',AValue,AssertProperty(APropName,jtString).AsString);
end;

procedure TTestJSONStreamer.AssertProp(APropName: String; AValue: TJSONFloat);
begin
  AssertNotNull('Result of streaming available',FSR);
  AssertEquals('Result of streaming is TJSONObject',TJSONObject,FSR.ClassType);
  AssertEquals('Correct value',AValue,AssertProperty(APropName,jtNumber).AsFloat);
end;

procedure TTestJSONStreamer.AssertProp(APropName: String; AValue: array of String
  );
Var
  a : TJSONArray;
  i : integer;

begin
  a:=AssertArrayProp(APropName);
  AssertEquals('Correct count ',Length(AValue),A.Count);
  For I:=Low(AValue) to High(Avalue) do
    begin
    AssertEquals('Array element type',jtString,A.Types[i]);
    AssertEquals('Array value',AValue[i],A.strings[i]);
    end;
end;

procedure TTestJSONStreamer.AssertProp(APropName: String; AValue: array of Integer
  );
Var
  a : TJSONArray;
  i : integer;

begin
  a:=AssertArrayProp(APropName);
  For I:=Low(AValue) to High(Avalue) do
    begin
    AssertEquals('Array element type',jtNumber,A.Types[i]);
    AssertEquals('Array value',AValue[i],A.Integers[i]);
    end;
end;

function TTestJSONStreamer.CreateVariantComp: TVariantComponent;
begin
  Result:=TVariantComponent.Create(Nil);
  FTofree:=Result;
end;

procedure TTestJSONStreamer.AssertNullProp(APropName: String);
begin
  AssertProperty(APropName,jtNull);
end;

function TTestJSONStreamer.AssertObjectProp(APropName: String): TJSONObject;
begin
  Result:=AssertProperty(APropName,jtObject) as TJSONObject;
end;

function TTestJSONStreamer.AssertArrayProp(APropName: String): TJSONArray;
begin
  Result:=AssertProperty(APropName,jtArray) as TJSONArray;
end;

procedure TTestJSONStreamer.TestNil;
begin
  AssertNull('Nil returns nil',StreamObject(Nil));
end;

procedure TTestJSONStreamer.TestEmpty;
begin
  StreamObject(TemptyPersistent.Create);
  AssertPropCount(0);
end;

procedure TTestJSONStreamer.TestEmptyComponent;
begin
  StreamObject(TComponent.Create(nil));
  AssertPropCount(0);
end;

procedure TTestJSONStreamer.TestWriteBoolean;

begin
  StreamObject(TBooleanComponent.Create(nil));
  AssertPropCount(1);
  AssertProp('BooleanProp',False);
end;

procedure TTestJSONStreamer.TestWriteInteger;
begin
  StreamObject(TIntegerComponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('IntProp',3);
end;

procedure TTestJSONStreamer.TestWriteString;
begin
  StreamObject(TStringComponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('StringProp','A string');
end;

procedure TTestJSONStreamer.TestWriteFloat;
begin
  StreamObject(TSingleComponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('SingleProp',1.23);
end;

procedure TTestJSONStreamer.TestWriteFloat2;
begin
  StreamObject(TDoubleComponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('DoubleProp',2.34);
end;

procedure TTestJSONStreamer.TestWriteFloat3;
begin
  StreamObject(TExtendedComponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('ExtendedProp',3.45);
end;

procedure TTestJSONStreamer.TestWriteFloat4;
begin
  StreamObject(TCompComponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('CompProp',5);
end;

procedure TTestJSONStreamer.TestWriteFloat5;
begin
  StreamObject(TCurrencyComponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('CurrencyProp',5.67);
end;

procedure TTestJSONStreamer.TestEnum1;
begin
  StreamObject(TEnumComponent3.Create(Nil));
  AssertPropCount(1);
  AssertProp('Dice',GetEnumName(TypeInfo(TDice),Ord(three)));
end;

procedure TTestJSONStreamer.TestEnum2;
begin
  RJ.Options:=[jsoEnumeratedAsInteger];
  StreamObject(TEnumComponent3.Create(Nil));
  AssertProp('Dice',Ord(three));
end;

procedure TTestJSONStreamer.TestSet1;
begin
  StreamObject(TSetComponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('Throw',['two','five']);
end;

procedure TTestJSONStreamer.TestSet2;
begin
  RJ.Options:=[jsoSetAsString];
  StreamObject(TSetComponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('Throw','two,five');
end;

procedure TTestJSONStreamer.TestSet3;
begin
  RJ.Options:=[jsoSetAsString,jsoSetBrackets];
  StreamObject(TSetComponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('Throw','[two,five]');
end;

procedure TTestJSONStreamer.TestSet4;
begin
  RJ.Options:=[jsoSetEnumeratedAsInteger];
  StreamObject(TSetComponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('Throw',[Ord(two),Ord(five)]);
end;

procedure TTestJSONStreamer.TestObjectNil;

Var
  P : TOwnedComponent;

begin
  P:=TOwnedComponent.Create(Nil);
  P.CompProp.Free;
  P.CompProp:=Nil;
  StreamObject(P);
  AssertPropCount(1);
  AssertNullProp('CompProp');
end;

procedure TTestJSONStreamer.TestComponentProp1;
begin
  StreamObject(TOwnedComponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('CompProp','SubComponent');
end;

procedure TTestJSONStreamer.TestComponentProp2;

Var
  C : TOwnedComponent;
  F : TJSONObject;

begin
  RJ.Options:=[jsoComponentsInline];
  C:=TOwnedComponent.Create(Nil);
  StreamObject(C);
  AssertPropCount(1);
  F:=SR;
  try
    SR:=AssertObjectProp('CompProp');
    AssertPropCount(1);
    AssertProp('Name','SubComponent');
    Assertprop('Tag',0);
    AssertProp('IntProp',3);
  finally
    SR:=F;
  end;
end;

procedure TTestJSONStreamer.TestCollectionProp1;

Var
  C : TCollectionComponent;
  F : TJSONObject;
  A : TJSONArray;

begin
  C:=TCollectionComponent2.Create(Nil);
  StreamObject(C);
  AssertPropCount(1);
  F:=SR;
  try
    A:=AssertArrayProp('Coll');
    AssertEquals('Collection item cound',3,A.Count);
    AssertEquals('Item 0 is object',jtObject,A.Types[0]);
    SR:=A.Objects[0];
    FToFree:=SR;
    AssertPropCount(1);
    AssertProp('StrProp','First');
    AssertEquals('Item 1 is object',jtObject,A.Types[1]);
    SR:=A.Objects[1];
    FToFree:=SR;
    AssertPropCount(1);
    AssertProp('StrProp','Second');
    AssertEquals('Item 2 is object',jtObject,A.Types[2]);
    SR:=A.Objects[2];
    FToFree:=SR;
    AssertPropCount(1);
    AssertProp('StrProp','Third');
  finally
    SR:=F;
    FToFree:=C;
  end;
end;

procedure TTestJSONStreamer.TestCollectionProp2;

Var
  C : TCollectionComponent;
  A : TJSONArray;

begin
  C:=TCollectionComponent.Create(Nil);
  StreamObject(C);
  AssertPropCount (1);
  A:=AssertArrayProp('Coll');
  AssertEquals('Collection item count',0,A.Count);
end;

procedure TTestJSONStreamer.TestPersistentProp1;

var
  P : TPersistentComponent;
  F : TJSONObject;

begin
  P:=TPersistentComponent.Create(Nil);
  StreamObject(P);
  AssertPropCount(1);
  F:=SR;
  try
    SR:=AssertObjectProp('Persist');
    FToFree:=P.Persist;
    AssertPropCount(2);
    AssertProp('AString','A persistent string');
    AssertProp('AInteger',3);
  finally
    FToFree:=P;
    SR:=F;
  end;
end;

procedure TTestJSONStreamer.TestStringsProp1;

begin
  RJ.Options:=[jsoTstringsAsArray];
  StreamObject(TStringsCOmponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('StringsProp',['One','Two','Three']);
end;

procedure TTestJSONStreamer.TestStringsProp2;

begin
  StreamObject(TStringsCOmponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('StringsProp','One'+sLineBreak+'Two'+sLineBreak+'Three'+sLineBreak);
end;

procedure TTestJSONStreamer.TestStringsProp3;

Var
  O : TJSONObject;
  S : TStringsComponent;

begin
  S:=TStringsCOmponent.Create(Nil);
  RJ.Options:=[jsoTstringsAsObject];
  StreamObject(S);
  AssertPropCount(1);
  O:=SR;
  SR:=AssertObjectprop('StringsProp');
  FTofree:=Nil;
  try
    AssertNullProp('One');
    AssertNullProp('Two');
    AssertNullProp('Three');
  finally
    SR:=o;
    FToFree:=S;
  end;
end;

procedure TTestJSONStreamer.TestStringsProp4;

Var
  O,SP : TJSONObject;
  S : TStringsComponent;

begin
  S:=TStringsCOmponent.Create(Nil);
  S.StringsProp.Objects[0]:=TEmptyPersistent.Create;
  S.StringsProp.Objects[1]:=TEmptyPersistent.Create;
  S.StringsProp.Objects[2]:=TEmptyPersistent.Create;
  try
    RJ.Options:=[jsoTstringsAsObject];
    StreamObject(S);
    AssertPropCount(1);
    O:=SR;
    SP:=AssertObjectprop('StringsProp');
    SR:=SP;
    FTofree:=Nil;
    try
      SR:=AssertObjectProp('One');
      AssertPropCount(0);
      SR:=SP;
      SR:=AssertObjectProp('Two');
      AssertPropCount(0);
      SR:=SP;
      SR:=AssertObjectProp('Three');
      AssertPropCount(0);
    finally
      SR:=o;
      FToFree:=S;
    end;
  finally
    S.StringsProp.Objects[0].Free;
    S.StringsProp.Objects[1].Free;
    S.StringsProp.Objects[2].Free;
  end;
end;

procedure TTestJSONStreamer.TestStringsArray;

Var
  O : TJSONArray;
  S : TStringList;

begin
  S:=TStringList.create;
  try
    S.Add('one');
    O:=RJ.StreamTStringsArray(S);
    try
      AssertEquals('one element',1,O.Count);
      AssertEquals('string type',jtString,O.Types[0]);
      AssertEquals('string value','one',O.Strings[0]);
    finally
      FreeAndNil(O);
    end;
  finally
    S.Free;
  end;
end;

procedure TTestJSONStreamer.TestStringsObject;

Var
  O : TJSONObject;
  S : TStringList;

begin
  S:=TStringList.create;
  try
    S.AddObject('one',TEmptyPersistent.Create);
    O:=RJ.StreamTStringsObject(S);
    try
      AssertEquals('one element',1,O.Count);
      AssertEquals('Have property',0,O.IndexOfName('one'));
      AssertEquals('string type',jtObject,O.Types['one']);
      AssertEquals('string value','one',O.Names[0]);
    finally
      FreeAndNil(O);
    end;
  finally
    S.Objects[0].FRee;
    S.Free;
  end;
end;

procedure TTestJSONStreamer.TestStringsStream1;

Var
  D: TJSONData;
  O : TJSONArray;
  S : TStringList;

begin
  S:=TStringList.create;
  try
    S.Add('one');
    RJ.Options:=[jsoTstringsAsArray];
    D:=RJ.StreamTStrings(S);
    try
      AssertEquals('Correct type',jtArray,D.JSONType);
      O:=D as TJSONArray;
      AssertEquals('one element',1,O.Count);
      AssertEquals('string type',jtString,O.Types[0]);
      AssertEquals('string value','one',O.Strings[0]);
    finally
      FreeAndNil(O);
    end;
  finally
    S.Free;
  end;
end;

procedure TTestJSONStreamer.TestStringsStream2;

Var
  D : TJSONData;
  O : TJSONObject;
  S : TStringList;

begin
  S:=TStringList.create;
  try
    S.AddObject('one',TEmptyPersistent.Create);
    RJ.Options:=[jsoTstringsAsObject];
    D:=RJ.StreamTstrings(S);
    try
      AssertEquals('Correct type',jtObject,D.JSONType);
      O:=D as TJSONObject;
      AssertEquals('one element',1,O.Count);
      AssertEquals('Have property',0,O.IndexOfName('one'));
      AssertEquals('string type',jtObject,O.Types['one']);
      AssertEquals('string value','one',O.Names[0]);
    finally
      SR:=O;
    end;
  finally
    S.Objects[0].FRee;
    S.Free;
  end;
end;

procedure TTestJSONStreamer.TestStringsStream3;
Var
  O : TJSONObject;
  S : TStringList;

begin
  S:=TStringList.create;
  try
    S.AddObject('one',TEmptyPersistent.Create);
    RJ.Options:=[jsoTstringsAsObject];
    SR:=RJ.ObjectToJSON(S);
    O:=AssertObjectProp('Strings');
    AssertEquals('one element',1,O.Count);
    AssertEquals('Have property',0,O.IndexOfName('one'));
    AssertEquals('string type',jtObject,O.Types['one']);
    AssertEquals('string value','one',O.Names[0]);
  finally
    S.Objects[0].FRee;
    S.Free;
  end;
end;

procedure TTestJSONStreamer.TestStringsStream4;
Var
  S : TStringList;

begin
  S:=TStringList.create;
  try
    S.AddObject('one',TEmptyPersistent.Create);
    SR:=RJ.ObjectToJSON(S);
    AssertProp('Strings','one'+sLinebreak);
  finally
    S.Objects[0].FRee;
    S.Free;
  end;
end;

procedure TTestJSONStreamer.TestStringsStream5;
Var
  D : TJSONData;
  S : TStringList;

begin
  S:=TStringList.create;
  try
    S.AddObject('one',TEmptyPersistent.Create);
    D:=RJ.StreamTstrings(S);
    try
      AssertEquals('String data',jtString,D.JSONType);
      AssertEquals('String value','one'+sLineBreak,D.AsString);
    finally
      D.free;
    end;
  finally
    S.Objects[0].FRee;
    S.Free;
  end;
end;

procedure TTestJSONStreamer.TestCollectionStream;

Var
  C : TTestCollection;
  A : TJSONArray;

begin
  C:=TTestCollection.Create;
  FToFree:=C;
  TTestItem(C.Add).StrProp:='One';
  TTestItem(C.Add).StrProp:='Two';
  A:=RJ.StreamCollection(C);
  try
    AssertNotNull('Have result',A);
    AssertEquals('2 items',2,A.Count);
    AssertEquals('Type item 0,',jtObject,A.Types[0]);
    AssertEquals('Type item 1,',jtObject,A.Types[1]);
    SR:=A.Objects[0];
    AssertPropCount(1);
    AssertProp('StrProp','One');
    SR:=A.Objects[1];
    AssertPropCount(1);
    AssertProp('StrProp','Two');
    SR:=Nil;
  finally
    FreeAndNil(A);
  end;
end;

procedure TTestJSONStreamer.TestCollectionStream2;

Var
  C : TTestCollection;
  A : TJSONArray;
  o : TJSONObject;

begin
  C:=TTestCollection.Create;
  TTestItem(C.Add).StrProp:='One';
  TTestItem(C.Add).StrProp:='Two';
  FToFree:=C;
  StreamObject(C);
  O:=SR;
  try
    A:=AssertProperty('Items',jtArray) as TJSONArray;
    AssertNotNull('Have result',A);
    AssertEquals('2 items',2,A.Count);
    AssertEquals('Type item 0,',jtObject,A.Types[0]);
    AssertEquals('Type item 1,',jtObject,A.Types[1]);
    SR:=A.Objects[0];
    AssertPropCount(1);
    AssertProp('StrProp','One');
    SR:=A.Objects[1];
    AssertPropCount(1);
    AssertProp('StrProp','Two');
    SR:=Nil;
  finally
    SR:=O;
  end;
end;

procedure TTestJSONStreamer.TestOnStreamProperty;
begin
  RJ.OnStreamProperty:=@DoStreamProperty1;
  StreamObject(TIntegerComponent.Create(Nil));
  AssertPropCount(0);
end;

procedure TTestJSONStreamer.TestDateTimeProp;

Var
  D : Double;
begin
  StreamObject(TDateTimeComponent.Create(Nil));
  D:=EncodeDate(1996,8,1);
  AssertPropCount(1);
  AssertProp('DateTimeProp',D);
end;

procedure TTestJSONStreamer.TestDateTimePropDefaultString;

Var
  D : Double;
begin
  RJ.Options:=[jsoDateTimeAsString];
  StreamObject(TDateTimeComponent.Create(Nil));
  D:=EncodeDate(1996,8,1);
  AssertPropCount(1);
  AssertProp('DateTimeProp',FormatDateTime(RFC3339DateTimeFormat,D));
end;

procedure TTestJSONStreamer.TestDateTimePropDefaultStringTime;
Var
  D : Double;
begin
  RJ.Options:=[jsoDateTimeAsString];
  StreamObject(TDateTimeComponent3.Create(Nil));
  D:=EncodeDate(1996,8,1)+EncodeTime(23,20,0,0);
  AssertPropCount(1);
  AssertProp('DateTimeProp',FormatDateTime(RFC3339DateTimeFormat,D));
end;

procedure TTestJSONStreamer.TestDateTimeProp2;
Var
  D : Double;
begin
  StreamObject(TDateTimeComponent2.Create(Nil));
  D:=EncodeTime(23,20,0,0);
  AssertPropCount(1);
  AssertProp('DateTimeProp',D);
end;

procedure TTestJSONStreamer.TestDateTimeProp3;
Var
  D : Double;
begin
  StreamObject(TDateTimeComponent3.Create(Nil));
  D:=EncodeDate(1996,8,1)+EncodeTime(23,20,0,0);
  AssertPropCount(1);
  AssertProp('DateTimeProp',D);
end;

procedure TTestJSONStreamer.TestDateTimeProp4;

begin
  RJ.Options:=[jsoDateTimeAsString,jsoLegacyDateTime];
  StreamObject(TDateTimeComponent.Create(Nil));
  AssertPropCount(1);
  AssertProp('DateTimeProp',DateToStr(EncodeDate(1996,8,1)));
end;

procedure TTestJSONStreamer.TestDateTimeProp5;

begin
  RJ.Options:=[jsoDateTimeAsString,jsoLegacyDateTime];
  StreamObject(TDateTimeComponent2.Create(Nil));
  AssertPropCount(1);
  AssertProp('DateTimeProp',TimeToStr(EncodeTime(23,20,0,0)));
end;

procedure TTestJSONStreamer.TestDateTimeProp6;

begin
  RJ.Options:=[jsoDateTimeAsString,jsoLegacyDateTime];
  StreamObject(TDateTimeComponent3.Create(Nil));
  AssertPropCount(1);
  AssertProp('DateTimeProp',DateTimeToStr(EncodeDate(1996,8,1)+EncodeTime(23,20,0,0)));
end;

procedure TTestJSONStreamer.TestDateTimeProp7;
begin
  RJ.Options:=[jsoDateTimeAsString];
  RJ.DateTimeFormat:='hh:nn';
  StreamObject(TDateTimeComponent3.Create(Nil));
  AssertPropCount(1);
  AssertProp('DateTimeProp',FormatDateTime('hh:nn',EncodeDate(1996,8,1)+EncodeTime(23,20,0,0)));
end;

procedure TTestJSONStreamer.TestVariantShortint;

Var
  i : ShortInt;
  C : TVariantComponent;

begin
  i:=3;
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varshortint),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',3);
end;

procedure TTestJSONStreamer.TestVariantbyte;
Var
  i : Byte;
  C : TVariantComponent;

begin
  i:=3;
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varByte),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',3);
end;

procedure TTestJSONStreamer.TestVariantword;

Var
  i : Word;
  C : TVariantComponent;

begin
  i:=3;
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varWord),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',3);
end;

procedure TTestJSONStreamer.TestVariantsmallint;

Var
  i : Smallint;
  C : TVariantComponent;

begin
  i:=3;
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varSmallint),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',3);
end;

procedure TTestJSONStreamer.TestVariantinteger;
Var
  i : Integer;
  C : TVariantComponent;

begin
  i:=3;
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varInteger),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',3);
end;

procedure TTestJSONStreamer.TestVariantlongword;

Var
  i : Cardinal;
  C : TVariantComponent;

begin
  i:=3;
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varLongword),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',3);
end;

procedure TTestJSONStreamer.TestVariantint64;
Var
  i : Int64;
  C : TVariantComponent;

begin
  i:=3;
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varInt64),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',3);
end;

procedure TTestJSONStreamer.TestVariantqword;
Var
  i : QWord;
  C : TVariantComponent;

begin
  i:=3;
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varQWord),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',3);
end;

procedure TTestJSONStreamer.TestVariantsingle;
Var
  i : Single;
  C : TVariantComponent;

begin
  i:=3.14;
  C:=CreateVariantComp;
  C.VariantProp:=VarAsType(3.14,varSingle);
  AssertEquals('Variant type',VarTypeAsText(varSingle),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',i);
end;

procedure TTestJSONStreamer.TestVariantdouble;

Var
  i : Double;
  C : TVariantComponent;

begin
  i:=3.14;
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varDouble),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',3.14);
end;

procedure TTestJSONStreamer.TestVariantCurrency;
Var
  i : Currency;
  C : TVariantComponent;

begin
  i:=3.14;
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varCurrency),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',3.14);
end;

procedure TTestJSONStreamer.TestVariantString;

Var
  i : String;
  C : TVariantComponent;

begin
  i:='3.14';
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varString),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp','3.14');
end;

procedure TTestJSONStreamer.TestVariantolestr;

Var
  i : String;
  C : TVariantComponent;

begin
  i:='3.14';
  C:=CreateVariantComp;
  C.VariantProp:=VarAsType(i,varOleStr);
  AssertEquals('Variant type',VarTypeAsText(varOleStr),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp','3.14');
end;

procedure TTestJSONStreamer.TestVariantboolean;
Var
  i : Boolean;
  C : TVariantComponent;

begin
  i:=True;
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varBoolean),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',True);
end;

procedure TTestJSONStreamer.TestVariantDate;

Var
  i : TDateTime;
  C : TVariantComponent;

begin
  i:=EncodeDate(2010,12,23);
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varDate),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',EncodeDate(2010,12,23));
end;

procedure TTestJSONStreamer.TestVariantDate2;

Var
  i : TDateTime;
  C : TVariantComponent;

begin
  RJ.Options:=[jsoDateTimeAsString,jsoLegacyDateTime];
  i:=EncodeDate(2010,12,23);
  C:=CreateVariantComp;
  C.VariantProp:=i;
  AssertEquals('Variant type',VarTypeAsText(varDate),VarTypeAsText(VarType(C.VariantProp)));
  StreamObject(FTofree);
  AssertPropCount(1);
  AssertProp('VariantProp',DateToStr(EncodeDate(2010,12,23)));
end;

procedure TTestJSONStreamer.TestVariantArray;
Var
  i : Integer;
  V : Variant;
  C : TVariantComponent;
  A : TJSONArray;

begin
  V:=VarArrayCreate([1,10],varInteger);
  For I:=1 to 10 do
    V[i]:=11-I;
  C:=CreateVariantComp;
  C.VariantProp:=V;
  StreamObject(FTofree);
  AssertPropCount(1);
  A:=AssertProperty('VariantProp',jtArray) as TJSONArray;
  AssertEquals('10 elements in array',10,A.Count);
  For I:=1 to 10 do
    begin
    assertEquals('Type of element',jtNumber,A.Types[i-1]);
    AssertEquals('Value of element',11-i,A.Integers[i-1]);
    end;
end;

procedure TTestJSONStreamer.TestMultipleProps;
begin
  StreamObject(TMultipleComponent.Create(Nil));
  AssertPropCount(5);
  AssertProp('IntProp',1);
  Assertprop('StringProp','A String');
  AssertProp('CurrencyProp',2.3);
  AssertProp('Throw',['three','four']);
  AssertProp('Dice','two');
end;

procedure TTestJSONStreamer.TestObjectToJSONString;
begin
  StreamObject(TIntegerComponent.Create(Nil));
  AssertEquals('Correct stream',SR.AsJSON,RJ.ObjectToJSONString(FToFree));
end;

procedure TTestJSONStreamer.TestStringsToJSONString;
Var
  S : TStrings;
begin
  S:=TStringList.Create;
  try
    S.Add('one');
    S.Add('two');
    S.Add('three');
    AssertEquals('StringsToJSONString','["one", "two", "three"]',RJ.StringsToJSON(S));
    AssertEquals('StringsToJSONString','{ "one" : null, "two" : null, "three" : null }',RJ.StringsToJSON(S,True));
  finally
    FreeAndNil(S);
  end;
end;

procedure TTestJSONStreamer.TestTListToJSONString ;


Var
  C : TList;
  D : TJSONData;
  P : Pointer;

  Function Add : TTestItem;

  begin
    Result:=TTestItem.Create(Nil);
    C.Add(Result);
  end;

begin
  RJ.Options:=RJ.Options + [jsoStreamTList];
  C:=TList.Create;
  try
    Add.StrProp:='one';
    Add.StrProp:='two';
    Add.StrProp:='three';
    D:=RJ.StreamTList(C);
    AssertEquals('StreamTlist','[{ "StrProp" : "one" }, { "StrProp" : "two" }, { "StrProp" : "three" }]',D.AsJSON);
  finally
    D.Free;
    For P in C do
      TObject(P).Free;
    FreeAndNil(C);
  end;
end;

procedure TTestJSONStreamer.TestCollectionToJSONString;

Var
  C : TTestCollection;

begin
  C:=TTestCollection.Create;
  try
    (C.Add as TTestItem).StrProp:='one';
    (C.Add as TTestItem).StrProp:='two';
    (C.Add as TTestItem).StrProp:='three';
    AssertEquals('CollectionToJSON','[{ "StrProp" : "one" }, { "StrProp" : "two" }, { "StrProp" : "three" }]',RJ.CollectionToJSON(C));
  finally
    FreeAndNil(C);
  end;
end;

procedure TTestJSONStreamer.TestChildren;

Var
  C : TChildrenComponent;

begin
  C:=TChildrenComponent.Create(Nil);
  TComponent.Create(C).Name:='Child1';
  TComponent.Create(C).Name:='Child2';
  StreamObject(C);
  If SR.IndexOfName('Children')<>-1 then
    Fail('Children streamed with default options');

end;

procedure TTestJSONStreamer.TestChildren2;
Var
  C : TChildrenComponent;
  A : TJSONArray;
  O : TJSONObject;

begin
  C:=TChildrenComponent.Create(Nil);
  TComponent.Create(C).Name:='Child1';
  TComponent.Create(C).Name:='Child2';
  RJ.Options:=[jsoStreamChildren];
  StreamObject(C);
  AssertPropCount(1);
  A:=AssertProperty('Children',jtArray) as TJSONArray;
  O:=SR;
  try
    AssertEquals('2 Elements in array',2,A.Count);
    AssertEquals('First in array is object',jtObject,A.Types[0]);
    AssertEquals('Second in array is object',jtObject,A.Types[1]);
    SR:=A.Objects[0];
    AssertProp('Name','Child1');
    SR:=A.Objects[1];
    AssertProp('Name','Child2');
  finally
    SR:=O;
  end;
end;

procedure TTestJSONStreamer.TestLowercase;
begin
  RJ.Options:=RJ.Options+[jsoLowerPropertyNames];
  StreamObject(TBooleanComponent.Create(nil));
  AssertPropCount(1);
  AssertProp('booleanprop',False);
end;

initialization

  RegisterTests([TTestJSONStreamer,TTestJSONDeStreamer]);
end.

