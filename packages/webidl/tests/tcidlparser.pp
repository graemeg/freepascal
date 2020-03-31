unit tcidlparser;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry, webidldefs, webidlparser, webidlscanner;

Type

  { TTestParser }

  TTestParser = Class(TTestCase)
  private
    FContext: TWebIDLContext;
    FParser: TWebIDLParser;
    FVersion: TWebIDLVersion;
    function GetList: TIDLDefinitionList;
    procedure SetVersion(AValue: TWebIDLVersion);
  Protected
    Procedure Setup; override;
    Procedure TearDown; override;
    Procedure InitSource(Const aSource: UTF8String);
    procedure AssertParserError(const Msg: String; const aSource: UTF8String);
    Class Procedure AssertEquals(Msg : String; AExpected,AActual : TConstType); overload;
    Class Procedure AssertEquals(Msg : String; AExpected,AActual : TAttributeOption); overload;
    Class Procedure AssertEquals(Msg : String; AExpected,AActual : TFunctionOption); overload;
    Class Procedure AssertEquals(Msg : String; AExpected,AActual : TAttributeOptions); overload;
    Class Procedure AssertEquals(Msg : String; AExpected,AActual : TFunctionOptions); overload;
  Public
    Property Parser : TWebIDLParser Read FParser;
    Property Context : TWebIDLContext Read FContext;
    Property Definitions : TIDLDefinitionList Read GetList;
    Property Version : TWebIDLVersion Read FVersion Write SetVersion;
  end;

  { TTestEnumParser }

  TTestEnumParser = Class(TTestParser)
  Public
    Procedure TestEnum(Const aSource,AName : UTF8String; AValues : Array of UTF8String);
  Published
    Procedure TestSingle;
    Procedure TestTwo;
    Procedure TestMissingIdent;
    Procedure TestMissingOpening;
    Procedure TestMissingClosing;
    Procedure TestMissingSemicolon;
    Procedure TestMissingComma;
  end;

  { TTestTypeDefParser }

  TTestTypeDefParser = Class(TTestParser)
  private
    function DoTestPromise(aDef: UTF8String; AReturnType: String=''): TIDLPromiseTypeDefDefinition;
    function DoTestSequence(aDef: UTF8String): TIDLSequenceTypeDefDefinition;
    function DoTestRecord(aDef: UTF8String; const aKeyTypeName,
      aValueTypeName: String): TIDLRecordDefinition;
    function DoTestUnion(aDef: String): TIDLUnionTypeDefDefinition;
  Public
    function TestTypeDef(const aSource, AName, aType: UTF8String): TIDLTypeDefDefinition;
  Published
    Procedure TestSimpleBoolean;
    Procedure TestSimpleBooleanNull;
    Procedure TestSimpleInt;
    procedure TestSimpleIntNull;
    Procedure TestSimpleLongint;
    procedure TestSimpleLongintNull;
    Procedure TestSimpleLongLongint;
    Procedure TestSimpleLongLongintNull;
    Procedure TestSimpleUnsignedShortint;
    Procedure TestSimpleUnsignedShortintNull;
    Procedure TestSimpleUnsignedLongint;
    Procedure TestSimpleUnsignedLongintNull;
    Procedure TestSimpleUnsignedLongLongint;
    Procedure TestSimpleUnsignedLongLongintNull;
    Procedure TestUnrestrictedFloat;
    Procedure TestSimpleFloat;
    Procedure TestSimpleFloatNull;
    Procedure TestSimpleDouble;
    Procedure TestSimpleDoubleNull;
    Procedure TestSimpleOctet;
    Procedure TestSimpleOctetNull;
    Procedure TestSimpleByte;
    procedure TestSimpleByteNull;
    Procedure TestSimpleIdentifier;
    Procedure TestSimpleIdentifierNull;
    Procedure TestAnyType;
    Procedure TestAnyTypeNull;
    Procedure TestUnion;
    Procedure TestUnionNull;
    Procedure TestSequence;
    Procedure TestSequenceNull;
    Procedure TestPromise;
    Procedure TestPromiseVoid;
    Procedure TestPromiseNull;
    Procedure TestPromiseReturnNull;
    Procedure TestRecord;
  end;

  { TTestInterfaceParser }

  { TTestBaseInterfaceParser }

  TTestBaseInterfaceParser = Class(TTestParser)
  private
    FCustAttributes: String;
    FisMixin: Boolean;
  Protected
    Procedure Setup; override;
  Public
    Function ParseInterface(AName,aInheritance : UTF8String; AMembers : Array of UTF8String) : TIDLInterfaceDefinition;
    Property isMixin : Boolean Read FisMixin Write FisMixin;
    Property CustAttributes : String Read FCustAttributes Write FCustAttributes;
  end;

  TTestInterfaceParser = Class(TTestBaseInterfaceParser)
  Published
    Procedure ParseEmpty;
    Procedure ParseEmptyInheritance;
    Procedure ParseMixinEmpty;
    Procedure ParseMixinEmptyInheritance;
    Procedure ParseCustomAttributes1;
  end;

  { TTestMapLikeInterfaceParser }

  TTestMapLikeInterfaceParser = Class(TTestBaseInterfaceParser)
  Public
    function ParseMapLike(const AKeyTypeName, aValueTypeName: UTF8String; IsReadOnly: Boolean): TIDLMapLikeDefinition;
  Published
    Procedure Parse;
    Procedure ParseReadOnly;
  end;

  { TTestSetLikeInterfaceParser }

  TTestSetLikeInterfaceParser = Class(TTestBaseInterfaceParser)
  Public
    Function ParseSetLike(const aElementTypeName : UTF8String; IsReadOnly : Boolean) : TIDLSetlikeDefinition;
  Published
    Procedure Parse;
    Procedure ParseReadOnly;
  end;
  { TTestConstInterfaceParser }

  TTestConstInterfaceParser = Class(TTestBaseInterfaceParser)
  Public
    Function ParseConst(AName,ATypeName,aValue : UTF8String; AType : TConstType) : TIDLConstDefinition;
  Published
    Procedure ParseConstInt;
    Procedure Parse2ConstInt;
    Procedure ParseConstIntHex;
    Procedure ParseConstLongint;
    Procedure ParseConstLongLongint;
    Procedure ParseConstUnsignedShortint;
    Procedure ParseConstUnsignedLongint;
    Procedure ParseConstUnsignedLongLongint;
    Procedure ParseConstFloat;
    Procedure ParseConstNan;
    Procedure ParseConstInfinity;
    Procedure ParseConstNegInfinity;
    Procedure ParseConstNull;
    Procedure ParseConstOctet;
    Procedure ParseConstByte;
    Procedure ParseConstBooleantrue;
    Procedure ParseConstBooleanFalse;
    Procedure ParseConstIdentifier;
  end;

  { TTestAttributeInterfaceParser }

  TTestAttributeInterfaceParser = Class(TTestBaseInterfaceParser)
  private
    Fattr: TIDLAttributeDefinition;
  Public
    Function ParseAttribute(ADef,AName,ATypeName : UTF8String; Options : TAttributeOptions = []) : TIDLAttributeDefinition;
    Property Attr : TIDLAttributeDefinition Read Fattr;
  Published
    Procedure ParseSimpleAttribute;
    Procedure ParseSimpleAttributeWithExtendedAttrs;
    Procedure ParseSimpleStaticAttribute;
    Procedure ParseSimpleStringifierAttribute;
    Procedure ParseSimpleReadonlyAttribute;
    Procedure ParseSimpleInheritedAttribute;
    Procedure ParseSimpleReadonlyInheritedAttribute;
    Procedure ParseSimpleReadonlyStaticAttribute;
    Procedure ParseSimpleReadonlyStringifierAttribute;
    Procedure ParseComplexReadonlyStaticAttribute;
    Procedure ParseIdentifierAttribute;
    Procedure Parse2IdentifierAttributes;
  end;

  { TTestSerializerInterfaceParser }

  TTestSerializerInterfaceParser = Class(TTestBaseInterfaceParser)
  private
    FSer: TIDLSerializerDefinition;
  Public
    Function ParseSerializer(ADef : UTF8String; Attrs : Array of UTF8String) : TIDLSerializerDefinition;
    Property Ser : TIDLSerializerDefinition Read FSer;
  Published
    Procedure TestSimpleIdentifier;
    Procedure TestSimpleFunction;
    Procedure TestMap;
    Procedure TestMapWithInherited;
    Procedure TestMapWithGetter;
    Procedure TestList;
    Procedure TestListWithGetter;
  end;

  { TTestOperationInterfaceParser }

  TTestOperationInterfaceParser = Class(TTestBaseInterfaceParser)
  private
    FFunc: TIDLFunctionDefinition;
  Public
    Function ParseFunction(ADef,aName,aReturnType : UTF8String; aArguments : Array of UTF8String) : TIDLFunctionDefinition;
    Property Func : TIDLFunctionDefinition Read FFunc;
  Published
    Procedure TestSimpleFunction;
    Procedure TestSimpleGetterFunction;
    Procedure TestSimpleSetterFunction;
    Procedure TestSimpleLegacyCallerFunction;
    Procedure TestSimpleDeleterFunction;
    Procedure TestAttrFunctionFunction;
    Procedure TestOptionalDefaultArgFunction;
  end;

  { TTestDictionaryParser }

  TTestDictionaryParser = Class(TTestParser)
  private
    FDict: TIDLDictionaryDefinition;
    FisPartial: Boolean;
    procedure AssertMember(aIndex: Integer; Aname, ATypeName, aDefaultValue: String; aDefaultType: TConstType=ctNull; isRequired: Boolean=False);
  Protected
    Property isPartial : Boolean Read FisPartial Write FisPartial;
  Public
    Function ParseDictionary(AName,aInheritance : UTF8String; AMembers : Array of UTF8String) : TIDLDictionaryDefinition;
    Property Dict : TIDLDictionaryDefinition read FDict;
  Published
    Procedure ParseSingleSimpleElement;
    Procedure ParseSingleSimpleElementInheritance;
    Procedure ParseSingleSimpleElementAttributes;
    Procedure ParseSingleSimpleElementAttributes2;
    Procedure ParseSingleSimpleElementRequired;
    Procedure ParseSingleSimpleElementDefaultString;
    Procedure ParseSingleSimpleElementRequiredDefaultString;
    Procedure ParseSingleSimpleElementRequiredDefaultEmptyArray;
    Procedure ParseSingleSimpleElementRequiredDefaultNull;
    Procedure ParseSingleSimpleElementUnsignedLongLong;
    Procedure ParseTwoSimpleElements;
    Procedure ParseThreeElements;
    Procedure ParsePartialSingleSimpleElement;
  end;

  { TTestFunctionCallbackParser }

  TTestFunctionCallbackParser = Class(TTestParser)
  private
    FFunction: TIDLFunctionDefinition;
  Public
    function ParseCallback(Const AName, aReturnType: UTF8String; AArguments: array of UTF8String): TIDLFunctionDefinition;
    Property Func : TIDLFunctionDefinition Read FFunction;
  Published
    Procedure ParseNoArgumentsReturnVoid;
    Procedure ParseOneArgumentReturnVoid;
    Procedure ParseOneUnsignedLongLongArgumentReturnVoid;
    Procedure ParseOneUnsignedLongLongArgumentReturnUnsignedLongLong;
    Procedure ParseOneArgumentWithAttrsReturnVoid;
    Procedure ParseOneOptionalArgumentReturnVoid;
    Procedure ParseOneOptionalArgumentWithAttrsReturnVoid;
    Procedure ParseTwoArgumentsReturnVoid;
    Procedure ParseTwoArgumentsAttrsReturnVoid;
    Procedure ParseThreeArgumentsAttrsReturnVoid;
  end;

  { TTestImplementsParser }

  TTestImplementsParser = Class(TTestParser)
  private
    FImpl: TIDLImplementsDefinition;
  Public
    Function ParseImplements(Const AName,aImplements: UTF8String) : TIDLImplementsDefinition;
    Property Impl: TIDLImplementsDefinition Read FImpl;
  Published
    Procedure ParseImplementsSimple;
  end;

  { TTestIncludesParser }

  TTestIncludesParser = Class(TTestParser)
  private
    FImpl: TIDLIncludesDefinition;
  Public
    Function ParseIncludes(Const AName,aIncludes: UTF8String) : TIDLIncludesDefinition;
    Property Impl: TIDLIncludesDefinition Read FImpl;
  Published
    Procedure ParseIncludesSimple;
  end;

  { TTestIterableInterfaceParser }

  TTestIterableInterfaceParser = Class(TTestBaseInterfaceParser)
  private
    Fiter: TIDLIterableDefinition;
  Public
    Function ParseIterable(Const AValueTypeName,AKeyTypeName : UTF8String) : TIDLIterableDefinition;
    Property Iter : TIDLIterableDefinition Read FIter;
  Published
    Procedure ParseSimpleIter;
    Procedure ParseKeyValueIter;
  end;

implementation

uses typinfo;

{ TTestSetLikeInterfaceParser }

function TTestSetLikeInterfaceParser.ParseSetLike(
  const aElementTypeName: UTF8String; IsReadOnly: Boolean
  ): TIDLSetlikeDefinition;
Var
  Id : TIDLInterfaceDefinition;
  S : UTF8String;

begin
  Version:=V2;
  S:=Format('setlike <%s>',[aElementTypeName]);
  if isReadOnly then
    S:='readonly '+S;
  Id:=ParseInterFace('IA','',[S]);
  AssertEquals('Correct class',TIDLSetLikeDefinition,Id.Members[0].ClassType);
  Result:=Id.Members[0] as TIDLSetLikeDefinition;
  AssertNotNull('Have key type',Result.ElementType);
  AssertEquals('key type',TIDLTypeDefDefinition, Result.ElementType.ClassType);
  AssertEquals('Key type Name',AElementTypeName,Result.ElementType.TypeName);
  AssertEquals('Readonly',IsReadOnly,Result.IsReadOnly);
end;

procedure TTestSetLikeInterfaceParser.Parse;
begin
  ParseSetLike('short',False);
end;

procedure TTestSetLikeInterfaceParser.ParseReadOnly;
begin
  ParseSetLike('short',True);
end;

{ TTestMapLikeInterfaceParser }

function TTestMapLikeInterfaceParser.ParseMapLike(const AKeyTypeName,
  aValueTypeName: UTF8String; IsReadOnly : Boolean): TIDLMapLikeDefinition;
Var
  Id : TIDLInterfaceDefinition;
  S : UTF8String;

begin
  Version:=V2;
  S:=Format('maplike <%s,%s>',[aKeyTypeName,aValueTypeName]);
  if isReadOnly then
    S:='readonly '+S;
  Id:=ParseInterFace('IA','',[S]);
  AssertEquals('Correct class',TIDLMapLikeDefinition,Id.Members[0].ClassType);
  Result:=Id.Members[0] as TIDLMapLikeDefinition;
  AssertNotNull('Have key type',Result.KeyType);
  AssertEquals('key type',TIDLTypeDefDefinition, Result.KeyType.ClassType);
  AssertEquals('Key type Name',AKeyTypeName,Result.KeyType.TypeName);
  AssertNotNull('Have value type',Result.ValueType);
  AssertEquals('key value',TIDLTypeDefDefinition, Result.ValueType.ClassType);
  AssertEquals('Key value Name',AValueTypeName,Result.ValueType.TypeName);
  AssertEquals('Readonly',IsReadOnly,Result.IsReadOnly);
end;

procedure TTestMapLikeInterfaceParser.Parse;
begin
  ParseMapLike('short','string',False);
end;

procedure TTestMapLikeInterfaceParser.ParseReadOnly;
begin
  ParseMapLike('short','string',True);
end;

{ TTestIncludesParser }

function TTestIncludesParser.ParseIncludes(const AName, aIncludes: UTF8String
  ): TIDLIncludesDefinition;
Var
  Src : UTF8String;
begin
  Src:=AName+' includes '+aIncludes+';'+sLineBreak;
  InitSource(Src);
  Parser.Version:=v2;
  Parser.Parse;
  AssertEquals('Correct class',TIDLIncludesDefinition,Definitions[0].ClassType);
  Result:=Definitions[0] as TIDLIncludesDefinition;
  AssertEquals('Correct name ',AName,Result.Name);
  AssertEquals('Correct implements ',aIncludes,Result.IncludedInterface);
  FImpl:=Result;
end;

procedure TTestIncludesParser.ParseIncludesSimple;
begin

end;

{ TTestOperationInterfaceParser }

function TTestOperationInterfaceParser.ParseFunction(ADef, aName,
  aReturnType: UTF8String; aArguments: array of UTF8String): TIDLFunctionDefinition;
Var
  TN,Src : UTF8String;
  P,I,Idx : integer;
  Arg : TIDLArgumentDefinition;
  ID : TIDLInterfaceDefinition;

begin
  ID:=ParseInterface('IA','',[aDef]);
  Parser.Parse;
  AssertEquals('Correct class',TIDLFunctionDefinition,ID.Members[0].ClassType);
  Result:=ID.Members[0] as TIDLFunctionDefinition;
  AssertEquals('Name',AName,Result.Name);
  AssertNotNull('Have return type',Result.ReturnType);
  AssertEquals('Return type name',aReturnType,Result.ReturnType.TypeName);
  AssertEquals('Have arguments',Length(aArguments)>0,Result.HasArguments);
  AssertEquals('Argument count',Length(aArguments) div 2,Result.Arguments.Count);
  I:=0;
  While I<Length(aArguments)-1 do
    begin
    Idx:=I div 2;
    Arg:=Result.Argument[idx];
    AssertEquals('Argument '+IntToStr(Idx)+' name',aArguments[I+1],Arg.Name);
    AssertNotNull('Argument '+IntToStr(Idx)+' have type',Arg.ArgumentType);
    TN:=aArguments[I];
    P:=Pos(']',TN);
    If P>0 then
      TN:=Trim(Copy(TN,P+1,Length(TN)-P));
    if Pos('optional',TN)=1 then
      TN:=Trim(Copy(TN,9,Length(TN)-8));
    AssertEquals('Argument '+IntToStr(I div 2)+' type name',TN,Arg.ArgumentType.TypeName);
    Inc(I,2);
    end;
  FFunc:=Result;
end;

procedure TTestOperationInterfaceParser.TestSimpleFunction;
begin
  ParseFunction('short A()','A','short',[]);
end;

procedure TTestOperationInterfaceParser.TestSimpleGetterFunction;
begin
  AssertEquals('Options',[foGetter],ParseFunction('getter short A()','A','short',[]).Options);

end;

procedure TTestOperationInterfaceParser.TestSimpleSetterFunction;
begin
  AssertEquals('Options',[foSetter],ParseFunction('setter short A()','A','short',[]).Options);
end;

procedure TTestOperationInterfaceParser.TestSimpleLegacyCallerFunction;
begin
  AssertEquals('Options',[foLegacyCaller],ParseFunction('legacycaller short A()','A','short',[]).Options);
end;

procedure TTestOperationInterfaceParser.TestSimpleDeleterFunction;
begin
  AssertEquals('Options',[foDeleter],ParseFunction('deleter short A()','A','short',[]).Options);
end;

procedure TTestOperationInterfaceParser.TestAttrFunctionFunction;
begin
  AssertTrue('HasAttribute',ParseFunction('[Me] short A()','A','short',[]).HasSimpleAttribute('Me'));
end;

procedure TTestOperationInterfaceParser.TestOptionalDefaultArgFunction;
begin
  ParseFunction('void A(optional short me = 0,optional short you = 0)','A','void',['short','me','short','you'])
end;

{ TTestSerializerInterfaceParser }

function TTestSerializerInterfaceParser.ParseSerializer(ADef: UTF8String; Attrs: array of UTF8String): TIDLSerializerDefinition;

Var
  Id : TIDLInterfaceDefinition;
  i : Integer;

begin
  Id:=ParseInterFace('IA','',['serializer '+ADef]);
  AssertEquals('Correct class',TIDLSerializerDefinition,Id.Members[0].ClassType);
  Result:=Id.Members[0] as TIDLSerializerDefinition;
  if (Length(Attrs)>0) then
    begin
    AssertTrue('Kind is object or array',Result.Kind in [skObject,skArray,skSingle]);
    AssertEquals('Identifier count',Length(Attrs),Result.Identifiers.Count);
    For I:=0 to Length(Attrs)-1 do
      AssertEquals('Identifier',Attrs[I],Result.Identifiers[i]);
    end
  else if (Result.SerializerFunction<>Nil) then
    AssertTrue('Kind is function',Result.Kind=skFunction);
  FSer:=Result;
end;

procedure TTestSerializerInterfaceParser.TestSimpleIdentifier;
begin
  ParseSerializer('= A',['A']);
end;

procedure TTestSerializerInterfaceParser.TestSimpleFunction;

Var
  D : TIDLFunctionDefinition;

begin
  AssertNotNull(ParseSerializer('string A ()',[]).SerializerFunction);
  D:=Ser.SerializerFunction;
  AssertEquals('Function name','A',D.Name);
end;

procedure TTestSerializerInterfaceParser.TestMap;
begin
  ParseSerializer('= {A, B, C}',['A','B','C']);
  AssertTrue('Map',Ser.Kind=skObject);
end;

procedure TTestSerializerInterfaceParser.TestMapWithInherited;
begin
  ParseSerializer('= {inherit, B, C}',['inherit','B','C']);
  AssertTrue('Map',Ser.Kind=skObject);
end;

procedure TTestSerializerInterfaceParser.TestMapWithGetter;
begin
  ParseSerializer('= {getter, B, C}',['getter','B','C']);
  AssertTrue('Map',Ser.Kind=skObject);
end;

procedure TTestSerializerInterfaceParser.TestList;
begin
  ParseSerializer('= [A, B, C]',['A','B','C']);
  AssertTrue('Map',Ser.Kind=skArray);
end;

procedure TTestSerializerInterfaceParser.TestListWithGetter;
begin
  ParseSerializer('= [getter, B, C]',['getter','B','C']);
  AssertTrue('Map',Ser.Kind=skArray);
end;

{ TTestIterableInterfaceParser }

function TTestIterableInterfaceParser.ParseIterable(const AValueTypeName,
  AKeyTypeName: UTF8String): TIDLIterableDefinition;

Var
  Id : TIDLInterfaceDefinition;
  Src : UTF8String;

begin
  Src:='iterable <';
  if AKeyTypeName<>'' then
    Src:=Src+aKeyTypeName+',';
  Src:=Src+aValueTypeName+'>';
  Id:=ParseInterFace('IA','',[Src]);
  AssertEquals('Correct class',TIDLIterableDefinition,Id.Members[0].ClassType);
  Result:=Id.Members[0] as TIDLIterableDefinition;
  AssertNotNull('Have value type',Result.ValueType);
  AssertEquals('Attr type',AValueTypeName,Result.ValueType.TypeName);
  if AKeyTypeName='' then
    AssertNull('No key type',Result.KeyType)
  else
    begin
    AssertNotNull('Have key type',Result.KeyType);
    AssertEquals('Attr type',AKeyTypeName,Result.KeyType.TypeName);
    end;
  Fiter:=Result;
end;

procedure TTestIterableInterfaceParser.ParseSimpleIter;
begin
  ParseIterable('short','');
end;

procedure TTestIterableInterfaceParser.ParseKeyValueIter;
begin
  ParseIterable('short','long');
end;

{ TTestAttributeInterfaceParser }


function TTestAttributeInterfaceParser.ParseAttribute(ADef, AName,
  ATypeName: UTF8String; Options: TAttributeOptions): TIDLAttributeDefinition;

Var
  Id : TIDLInterfaceDefinition;

begin
  Id:=ParseInterFace('IA','',[aDef]);
  AssertEquals('Correct class',TIDLAttributeDefinition,Id.Members[0].ClassType);
  Result:=Id.Members[0] as TIDLAttributeDefinition;
  AssertEquals('Attr name',AName,Result.Name);
  AssertNotNull('Have type',Result.AttributeType);
  AssertEquals('Attr type',ATypeName,Result.AttributeType.TypeName);
  AssertEquals('Attr options',Options,Result.Options);
  FAttr:=Result;
end;

procedure TTestAttributeInterfaceParser.ParseSimpleAttribute;
begin
  ParseAttribute('attribute short A','A','short',[]);
end;

procedure TTestAttributeInterfaceParser.ParseSimpleAttributeWithExtendedAttrs;
begin
  AssertTrue('Have attribute',ParseAttribute('[Me] attribute short A','A','short',[]).HasSimpleAttribute('Me'));
end;

procedure TTestAttributeInterfaceParser.ParseSimpleStaticAttribute;
begin
  ParseAttribute('static attribute short A','A','short',[aoStatic]);
end;

procedure TTestAttributeInterfaceParser.ParseSimpleStringifierAttribute;
begin
  ParseAttribute('stringifier attribute short A','A','short',[aoStringifier]);
end;

procedure TTestAttributeInterfaceParser.ParseSimpleReadonlyAttribute;
begin
  ParseAttribute('readonly attribute short A','A','short',[aoReadOnly]);

end;

procedure TTestAttributeInterfaceParser.ParseSimpleInheritedAttribute;
begin
  ParseAttribute('inherit attribute short A','A','short',[aoInherit]);
end;

procedure TTestAttributeInterfaceParser.ParseSimpleReadonlyInheritedAttribute;
begin
  ParseAttribute('inherit readonly attribute short A','A','short',[aoInherit,aoReadonly]);
end;

procedure TTestAttributeInterfaceParser.ParseSimpleReadonlyStaticAttribute;
begin
  ParseAttribute('static readonly attribute short A','A','short',[aoStatic,aoReadOnly]);

end;

procedure TTestAttributeInterfaceParser.ParseSimpleReadonlyStringifierAttribute;
begin
  ParseAttribute('stringifier readonly attribute short A','A','short',[aoStringifier,aoReadOnly]);
end;

procedure TTestAttributeInterfaceParser.ParseComplexReadonlyStaticAttribute;
begin
  ParseAttribute('static readonly attribute unsigned long long A','A','unsigned long long',[aoStatic,aoReadOnly]);
end;


procedure TTestAttributeInterfaceParser.ParseIdentifierAttribute;
begin
  ParseAttribute('attribute B A','A','B',[]);
end;

procedure TTestAttributeInterfaceParser.Parse2IdentifierAttributes;

Var
  Id : TIDLInterfaceDefinition;
begin
  Id:=ParseInterFace('IA','',['attribute B A','readonly attribute C D']);
  AssertEquals('Correct class',TIDLAttributeDefinition,Id.Members[0].ClassType);
  FAttr:=Id.Members[0] as TIDLAttributeDefinition;
  AssertEquals('Attr name','A',FAttr.Name);
  AssertNotNull('Have type',FAttr.AttributeType);
  AssertEquals('Attr type','B',FAttr.AttributeType.TypeName);
  AssertEquals('Attr options',[],FAttr.Options);
  FAttr:=Id.Members[1] as TIDLAttributeDefinition;
  AssertEquals('Attr name','D',FAttr.Name);
  AssertNotNull('Have type',FAttr.AttributeType);
  AssertEquals('Attr type','C',FAttr.AttributeType.TypeName);
  AssertEquals('Attr options',[aoReadonly],FAttr.Options);
end;

{ TTestImplementsParser }

function TTestImplementsParser.ParseImplements(const AName,
  aImplements: UTF8String): TIDLImplementsDefinition;
Var
  Src : UTF8String;
begin
  Src:=AName+' implements '+aImplements+';'+sLineBreak;
  InitSource(Src);
  Parser.Version:=V1;
  Parser.Parse;
  AssertEquals('Correct class',TIDLImplementsDefinition,Definitions[0].ClassType);
  Result:=Definitions[0] as TIDLImplementsDefinition;
  AssertEquals('Correct name ',AName,Result.Name);
  AssertEquals('Correct implements ',aImplements,Result.ImplementedInterface);
  FImpl:=Result;
end;

procedure TTestImplementsParser.ParseImplementsSimple;
begin
  ParseImplements('A','B');
end;

{ TTestFunctionCallbackParser }

function TTestFunctionCallbackParser.ParseCallback(const AName,
  aReturnType: UTF8String; AArguments: array of UTF8String
  ): TIDLFunctionDefinition;
Var
  TN,Src : UTF8String;
  P,I,Idx : integer;
  Arg : TIDLArgumentDefinition;

begin
  Src:='callback '+aName+' = '+AReturnType+' (';
  I:=0;
  While I<Length(aArguments) do
    begin
    if I>0 then
      Src:=Src+', ';
    Src:=Src+aArguments[I]+ ' '+aArguments[I+1];
    Inc(I,2);
    end;
  Src:=Src+');'+sLineBreak;
  InitSource(Src);
  Parser.Parse;
  AssertEquals('Correct class',TIDLFunctionDefinition,Definitions[0].ClassType);
  Result:=Definitions[0] as TIDLFunctionDefinition;
  AssertEquals('Name',AName,Result.Name);
  AssertNotNull('Have return type',Result.ReturnType);
  AssertEquals('Return type name',aReturnType,Result.ReturnType.TypeName);
  AssertEquals('Have arguments',Length(aArguments)>0,Result.HasArguments);
  AssertEquals('Argument count',Length(aArguments) div 2,Result.Arguments.Count);
  I:=0;
  While I<Length(aArguments)-1 do
    begin
    Idx:=I div 2;
    Arg:=Result.Argument[idx];
    AssertEquals('Argument '+IntToStr(Idx)+' name',aArguments[I+1],Arg.Name);
    AssertNotNull('Argument '+IntToStr(Idx)+' have type',Arg.ArgumentType);
    TN:=aArguments[I];
    P:=Pos(']',TN);
    If P>0 then
      TN:=Trim(Copy(TN,P+1,Length(TN)-P));
    if Pos('optional',TN)=1 then
      TN:=Trim(Copy(TN,9,Length(TN)-8));
    AssertEquals('Argument '+IntToStr(I div 2)+' type name',TN,Arg.ArgumentType.TypeName);
    Inc(I,2);
    end;
  FFunction:=Result;
end;

procedure TTestFunctionCallbackParser.ParseNoArgumentsReturnVoid;
begin
  ParseCallback('A','void',[]);
end;

procedure TTestFunctionCallbackParser.ParseOneArgumentReturnVoid;
begin
  ParseCallback('A','void',['short','A']);
end;

procedure TTestFunctionCallbackParser.ParseOneUnsignedLongLongArgumentReturnVoid;
begin
  ParseCallback('A','void',['unsigned long long','A']);
end;

procedure TTestFunctionCallbackParser.ParseOneUnsignedLongLongArgumentReturnUnsignedLongLong;
begin
  ParseCallback('A','unsigned long long',['unsigned long long','A']);
end;

procedure TTestFunctionCallbackParser.ParseOneArgumentWithAttrsReturnVoid;
begin
  ParseCallback('A','void',['[Me] unsigned long long','A']);
  AssertTrue('Have attribute',Func.Arguments[0].HasSimpleAttribute('Me'));
end;

procedure TTestFunctionCallbackParser.ParseOneOptionalArgumentReturnVoid;
begin
  ParseCallback('A','void',['optional unsigned long long','A']);
  AssertTrue('is optional',Func.Argument[0].IsOptional);
  AssertEquals('Type name','unsigned long long',Func.Argument[0].ArgumentType.TypeName);
end;

procedure TTestFunctionCallbackParser.ParseOneOptionalArgumentWithAttrsReturnVoid;
begin
  ParseCallback('A','void',['[Me] optional unsigned long long','A']);
  AssertTrue('is optional',Func.Argument[0].IsOptional);
  AssertEquals('Type name','unsigned long long',Func.Argument[0].ArgumentType.TypeName);
  AssertTrue('Have attribute',Func.Arguments[0].HasSimpleAttribute('Me'));
end;

procedure TTestFunctionCallbackParser.ParseTwoArgumentsReturnVoid;
begin
  ParseCallback('A','void',['short','B','short','C']);
end;

procedure TTestFunctionCallbackParser.ParseTwoArgumentsAttrsReturnVoid;
begin
  ParseCallback('A','void',['[Me] short','B','[Me] short','C']);
  AssertTrue('Have attribute',Func.Arguments[0].HasSimpleAttribute('Me'));
  AssertTrue('Have attribute',Func.Arguments[1].HasSimpleAttribute('Me'));
end;

procedure TTestFunctionCallbackParser.ParseThreeArgumentsAttrsReturnVoid;
begin
  ParseCallback('A','void',['[Me] short','B','[Me] short','C','[Me] optional unsigned long long','D']);
  AssertTrue('Have attribute',Func.Arguments[0].HasSimpleAttribute('Me'));
  AssertTrue('Have attribute',Func.Arguments[1].HasSimpleAttribute('Me'));
  AssertTrue('Have attribute',Func.Arguments[2].HasSimpleAttribute('Me'));
  AssertTrue('Have attribute',Func.Argument[2].IsOptional);
end;

{ TTestDictionaryParser }

function TTestDictionaryParser.ParseDictionary(AName, aInheritance: UTF8String;
  AMembers: array of UTF8String): TIDLDictionaryDefinition;
Var
  Src : UTF8String;
  I : integer;

begin
  Src:='dictionary '+aName+' ';
  If IsPartial then
    Src:='partial '+Src;

  if (aInheritance<>'') then
    Src:=Src+': '+aInheritance+' ';
  Src:=Src+'{'+sLineBreak;
  For I:=0 to Length(AMembers)-1 do
    Src:=Src+AMembers[I]+';'+sLineBreak;
  Src:=Src+'};'+sLineBreak;
  InitSource(Src);
  Parser.Parse;
  AssertEquals('Correct class',TIDLDictionaryDefinition,Definitions[0].ClassType);
  Result:=Definitions[0] as TIDLDictionaryDefinition;
  AssertEquals('Name',AName,Result.Name);
  AssertEquals('Inheritance : ',aInheritance,Result.ParentName);
  AssertEquals('Member count',Length(AMembers),Result.Members.Count);
  FDict:=Result;
end;

procedure TTestDictionaryParser.AssertMember(aIndex : Integer; Aname, ATypeName,aDefaultValue : String; aDefaultType  : TConstType = ctNull; isRequired : Boolean = False);

Var
  m : TIDLDictionaryMemberDefinition;
  S : string;

begin
  S:=Format('Member %d (Name %s)',[aIndex,AName]);
  AssertNotNull(S+' have dict',Dict);
  AssertTrue(S+' dict has members',Dict.HasMembers);
  AssertTrue(S+' index in range',(aIndex>=0) and (aIndex<Dict.Members.Count));
  AssertEquals(S+' element has correct class',TIDLDictionaryMemberDefinition,Dict.Members[aIndex].ClassType);
  M:=Dict[aIndex];
  AssertEquals(S+' isRequired : ',isRequired,M.IsRequired);
  AssertEquals(S+' Name : ',aName,M.Name);
  AssertNotNull(S+' Have type',M.MemberType);
  AssertEquals(S+' type name',aTypeName,M.MemberType.TypeName);
  if (aDefaultValue='') then
    AssertNull(S+' Have no default value',M.DefaultValue)
  else
    begin
    AssertNotNull(S+' Have default value',M.DefaultValue);
    AssertEquals(S+' default value',aDefaultValue,M.DefaultValue.Value);
    AssertEquals(S+' default value type',aDefaultType,M.DefaultValue.ConstType);
    end;
end;

procedure TTestDictionaryParser.ParseSingleSimpleElement;

begin
  ParseDictionary('A','',['string B']);
  AssertMember(0,'B','string','');
end;

procedure TTestDictionaryParser.ParseSingleSimpleElementInheritance;
begin
  ParseDictionary('A','C',['string B']);
  AssertMember(0,'B','string','');
end;

procedure TTestDictionaryParser.ParseSingleSimpleElementAttributes;
begin
  ParseDictionary('A','',['[Replaceable] required string B']);
  AssertMember(0,'B','string','',ctNull,True);
  AssertTrue('Has attributes',Dict[0].HasAttributes);
  AssertEquals('Attribute count',1,Dict[0].Attributes.Count);
  AssertEquals('Has attributes','Replaceable',Dict[0].Attributes[0]);
end;

procedure TTestDictionaryParser.ParseSingleSimpleElementAttributes2;
begin
  ParseDictionary('A','',['[Replaceable] octet B']);
  AssertMember(0,'B','octet','',ctNull,False);
  AssertTrue('Has attributes',Dict[0].HasAttributes);
  AssertEquals('Attribute count',1,Dict[0].Attributes.Count);
  AssertEquals('Has attributes','Replaceable',Dict[0].Attributes[0]);
end;

procedure TTestDictionaryParser.ParseSingleSimpleElementRequired;
begin
  ParseDictionary('A','',['required string B']);
  AssertMember(0,'B','string','',ctNull,True);
end;

procedure TTestDictionaryParser.ParseSingleSimpleElementDefaultString;
begin
  ParseDictionary('A','',['string B = "abc"']);
  AssertMember(0,'B','string','abc',ctString);
end;

procedure TTestDictionaryParser.ParseSingleSimpleElementRequiredDefaultString;
begin
  ParseDictionary('A','',['required string B = "abc"']);
  AssertMember(0,'B','string','abc',ctString,true);
end;

procedure TTestDictionaryParser.ParseSingleSimpleElementRequiredDefaultEmptyArray;
begin
  ParseDictionary('A','',['required string B = []']);
  AssertMember(0,'B','string','[]',ctEmptyArray,true);
end;

procedure TTestDictionaryParser.ParseSingleSimpleElementRequiredDefaultNull;
begin
  ParseDictionary('A','',['string B = null']);
  AssertMember(0,'B','string','null',ctNull,False);
end;

procedure TTestDictionaryParser.ParseSingleSimpleElementUnsignedLongLong;
begin
  ParseDictionary('A','',['required unsigned long long B']);
  AssertMember(0,'B','unsigned long long','',ctNull,True);
end;

procedure TTestDictionaryParser.ParseTwoSimpleElements;
begin
  ParseDictionary('A','',['string B','short C']);
  AssertMember(0,'B','string','');
  AssertMember(1,'C','short','');
end;

procedure TTestDictionaryParser.ParseThreeElements;
begin
  ParseDictionary('A','',['string B','short C','required unsigned long long D']);
  AssertMember(0,'B','string','');
  AssertMember(1,'C','short','');
  AssertMember(2,'D','unsigned long long','',ctNull,true);
end;

procedure TTestDictionaryParser.ParsePartialSingleSimpleElement;
begin
  isPartial:=True;
  ParseDictionary('A','',['string B']);
  AssertMember(0,'B','string','');
  AssertTrue('Partial',Dict.IsPartial);
end;

{ TTestTypeDefParser }

function TTestTypeDefParser.TestTypeDef(const aSource, AName, aType: UTF8String
  ): TIDLTypeDefDefinition;

Var
  E : TIDLTypeDefDefinition;

begin
  InitSource('typedef '+ASource+';');
  Parser.Parse;
  AssertEquals('Definition count',1,Definitions.Count);
  AssertTrue('Correct class',Definitions[0].ClassType.InheritsFrom(TIDLTypeDefDefinition));
  E:=Definitions[0] as TIDLTypeDefDefinition;
  AssertEquals('Name',AName,E.Name);
  AssertEquals('Type name',AType,E.TypeName);
  if Pos('?',aSource)=0 then
    AssertEquals('Not Null',False,E.AllowNull);
  Result:=E;
end;

procedure TTestTypeDefParser.TestSimpleBoolean;
begin
  TestTypeDef('boolean A','A','boolean');
end;

procedure TTestTypeDefParser.TestSimpleBooleanNull;
begin
  AssertTrue('AllowNull',TestTypeDef('boolean ? A','A','boolean').AllowNull);
end;

procedure TTestTypeDefParser.TestSimpleInt;
begin
  TestTypeDef('short A','A','short');
end;

procedure TTestTypeDefParser.TestSimpleIntNull;
begin
  AssertTrue('AllowNull',TestTypeDef('short ? A','A','short').AllowNull);
end;

procedure TTestTypeDefParser.TestSimpleLongint;
begin
  TestTypeDef('long A','A','long');
end;

procedure TTestTypeDefParser.TestSimpleLongintNull;
begin
  AssertTrue('AllowNull',TestTypeDef('long ? A','A','long').AllowNull);
end;

procedure TTestTypeDefParser.TestSimpleLongLongint;
begin
  TestTypeDef('long long A','A','long long');
end;

procedure TTestTypeDefParser.TestSimpleLongLongintNull;
begin
  AssertTrue('AllowNull',TestTypeDef('long long ? A','A','long long').AllowNull);
end;

procedure TTestTypeDefParser.TestSimpleUnsignedShortint;
begin
  TestTypeDef('unsigned short A','A','unsigned short');
end;

procedure TTestTypeDefParser.TestSimpleUnsignedShortintNull;

begin
  AssertTrue('AllowNull',TestTypeDef('unsigned short ? A','A','unsigned short').AllowNull);
end;

procedure TTestTypeDefParser.TestSimpleUnsignedLongint;
begin
  TestTypeDef('unsigned long A','A','unsigned long');
end;

procedure TTestTypeDefParser.TestSimpleUnsignedLongintNull;
begin
  AssertTrue('AllowNull',TestTypeDef('unsigned long ? A','A','unsigned long').AllowNull);
end;

procedure TTestTypeDefParser.TestSimpleUnsignedLongLongint;
begin
  TestTypeDef('unsigned long long A','A','unsigned long long');
end;

procedure TTestTypeDefParser.TestSimpleUnsignedLongLongintNull;
begin
  AssertTrue('AllowNull',TestTypeDef('unsigned long long ? A','A','unsigned long long').AllowNull);
end;

procedure TTestTypeDefParser.TestUnrestrictedFloat;
begin
  TestTypeDef('unrestricted float A','A','unrestricted float');
end;

procedure TTestTypeDefParser.TestSimpleFloat;
begin
  TestTypeDef('float A','A','float');
end;

procedure TTestTypeDefParser.TestSimpleFloatNull;
begin
  AssertTrue('AllowNull',TestTypeDef('float ? A','A','float').AllowNull)
end;

procedure TTestTypeDefParser.TestSimpleDouble;
begin
  TestTypeDef('double A','A','double');
end;

procedure TTestTypeDefParser.TestSimpleDoubleNull;
begin
  AssertTrue('AllowNull',TestTypeDef('double ? A','A','double').AllowNull);
end;

procedure TTestTypeDefParser.TestSimpleOctet;
begin
  TestTypeDef('octet A','A','octet');
end;

procedure TTestTypeDefParser.TestSimpleOctetNull;
begin
  AssertTrue('AllowNull',TestTypeDef('octet ? A','A','octet').AllowNull);
end;

procedure TTestTypeDefParser.TestSimpleByte;
begin
  TestTypeDef('byte A','A','byte');
end;

procedure TTestTypeDefParser.TestSimpleByteNull;
begin
  AssertTrue('AllowNull',TestTypeDef('byte ? A','A','byte').AllowNull);
end;

procedure TTestTypeDefParser.TestSimpleIdentifier;
begin
  TestTypeDef('Zaza A','A','Zaza');
end;

procedure TTestTypeDefParser.TestSimpleIdentifierNull;
begin
  AssertTrue('AllowNull',TestTypeDef('Zaza ? A','A','Zaza').AllowNull);
end;

procedure TTestTypeDefParser.TestAnyType;
begin
  TestTypeDef('any A','A','any');
end;

procedure TTestTypeDefParser.TestAnyTypeNull;
begin
  AssertTrue('AllowNull',TestTypeDef('any ? A','A','any').AllowNull);
end;

function TTestTypeDefParser.DoTestUnion(aDef: String): TIDLUnionTypeDefDefinition;

Var
  D : TIDLTypeDefDefinition;
  U : TIDLUnionTypeDefDefinition;

begin
  D:=TestTypeDef(aDef,'A','union');
  AssertEquals('Correct class',TIDLUnionTypeDefDefinition,D.ClassType);
  U:=TIDLUnionTypeDefDefinition(D);
  AssertEquals('Union types',2,U.Union.Count);
  AssertNotNull('Have type 1',U.Union[0]);
  AssertEquals('1: Correct class',TIDLTypeDefDefinition,U.Union[0].ClassType);
  D:=TIDLTypeDefDefinition(U.Union[0]);
  AssertEquals('1: Correct type name','byte',D.TypeName);
  AssertNotNull('Have type 2',U.Union[1]);
  AssertEquals('2: Correct class',TIDLTypeDefDefinition,U.Union[1].ClassType);
  D:=TIDLTypeDefDefinition(U.Union[1]);
  AssertEquals('2: Correct type name','octet',D.TypeName);
  Result:=U;
end;

procedure TTestTypeDefParser.TestUnion;

begin
  DoTestUnion('(byte or octet) A');
end;

procedure TTestTypeDefParser.TestUnionNull;
begin
  AssertTrue('Is null',DoTestUnion('(byte or octet) ? A').AllowNull);
end;

function TTestTypeDefParser.DoTestSequence(aDef: UTF8String
  ): TIDLSequenceTypeDefDefinition;

Var
  D : TIDLTypeDefDefinition;
  S : TIDLSequenceTypeDefDefinition;

begin
  D:=TestTypeDef(aDef ,'A','sequence');
  AssertEquals('Correct class',TIDLSequenceTypeDefDefinition,D.ClassType);
  S:=TIDLSequenceTypeDefDefinition(D);
  AssertNotNull('Have element type',S.ElementType);
  D:=TIDLTypeDefDefinition(S.ElementType);
  AssertEquals('1: Correct type name','byte',D.TypeName);
  Result:=S;
end;

function TTestTypeDefParser.DoTestRecord(aDef: UTF8String; const aKeyTypeName,
  aValueTypeName: String): TIDLRecordDefinition;
Var
  D : TIDLTypeDefDefinition;
  R : TIDLRecordDefinition;

begin
  Version:=v2;
  D:=TestTypeDef(aDef ,'A','record');
  AssertEquals('Correct class',TIDLRecordDefinition,D.ClassType);
  R:=TIDLRecordDefinition(D);
  AssertNotNull('Have key type',R.KeyType);
  D:=TIDLTypeDefDefinition(R.KeyType);
  AssertEquals('1: Correct type name',aKeyTypeName,D.TypeName);
  AssertNotNull('Have value type',R.ValueType);
  D:=TIDLTypeDefDefinition(R.ValueType);
  AssertEquals('1: Correct type name',aValueTypeName,D.TypeName);
  Result:=R;

end;

procedure TTestTypeDefParser.TestSequence;

begin
  DoTestSequence('sequence<byte> A');
end;

procedure TTestTypeDefParser.TestSequenceNull;

begin
  AssertTrue('Is Null ',DoTestSequence('sequence<byte> ? A').AllowNull);
end;

function TTestTypeDefParser.DoTestPromise(aDef: UTF8String; AReturnType : String = ''): TIDLPromiseTypeDefDefinition;

Var
  D : TIDLTypeDefDefinition;
  S : TIDLPromiseTypeDefDefinition;

begin
  D:=TestTypeDef(ADef,'A','Promise');
  AssertEquals('Correct class',TIDLPromiseTypeDefDefinition,D.ClassType);
  S:=TIDLPromiseTypeDefDefinition(D);
  AssertNotNull('Have element type',S.ReturnType);
  D:=TIDLTypeDefDefinition(S.ReturnType);
  if aReturnType='' then
    aReturnType:='byte';
  AssertEquals('1: Correct type name',aReturnType,D.TypeName);
  Result:=S;
end;


procedure TTestTypeDefParser.TestPromise;

begin
  DoTestPromise('Promise<byte> A');
end;

procedure TTestTypeDefParser.TestPromiseVoid;
begin
  DoTestPromise('Promise<void> A','void');
end;

procedure TTestTypeDefParser.TestPromiseNull;
begin
  AssertTrue('Is Null',DoTestPromise('Promise<byte> ? A').AllowNull);
end;

procedure TTestTypeDefParser.TestPromiseReturnNull;
begin
  AssertTrue('ReturnType Is Null',DoTestPromise('Promise<byte ?> A').ReturnType.AllowNull);
end;

procedure TTestTypeDefParser.TestRecord;
begin
  DoTestRecord('record <short,string> A','short','string');
end;

{ TTestInterfaceParser }

procedure TTestBaseInterfaceParser.Setup;
begin
  inherited Setup;
  FIsMixin:=False
end;

function TTestBaseInterfaceParser.ParseInterface(AName,aInheritance: UTF8String;
  AMembers: array of UTF8String): TIDLInterfaceDefinition;

Var
  Src : UTF8String;
  I : integer;

begin
  if IsMixin then
    Src:='interface mixin '+aName+' '
  else
    Src:='interface '+aName+' ';
  if (FCustAttributes<>'') then
    Src:=FCustAttributes+' '+Src;
  if (aInheritance<>'') then
    Src:=Src+': '+aInheritance+' ';
  Src:=Src+'{'+sLineBreak;
  For I:=0 to Length(AMembers)-1 do
    Src:=Src+'  '+AMembers[I]+';'+sLineBreak;
  Src:=Src+'};'+sLineBreak;
  InitSource(Src);
  Parser.Parse;
  AssertEquals('Correct class',TIDLInterfaceDefinition,Definitions[0].ClassType);
  Result:=Definitions[0] as TIDLInterfaceDefinition;
  AssertEquals('Name',AName,Result.Name);
  AssertEquals('Inheritance : ',aInheritance,Result.ParentName);
  AssertEquals('Member count',Length(AMembers),Result.Members.Count);
  AssertEquals('Mixin correct',IsMixin,Result.IsMixin);
end;

function TTestConstInterfaceParser.ParseConst(AName, ATypeName, aValue: UTF8String;
  AType: TConstType): TIDLConstDefinition;

Var
  Id : TIDLInterfaceDefinition;
  P : Integer;
  isNull : Boolean;

begin
  Id:=ParseInterFace('IA','',['const '+aTypeName+' '+AName+' = '+AValue]);
  AssertEquals('Correct class',TIDLConstDefinition,Id.Members[0].ClassType);
  Result:=Id.Members[0] as TIDLConstDefinition;
  AssertEquals('Const Name',AName,Result.Name);
  P:=Pos('?',ATypeName);
  isNull:=P>0;
  if IsNull then
    ATypeName:=Trim(Copy(ATypeName,1,P-1));
  AssertEquals('Const type',ATypeName,Result.TypeName);
  AssertEquals('Const consttype',AType,Result.ConstType);
  AssertEquals('Const value',AValue,Result.Value);
  AssertEquals('Const null allowed',IsNull,Result.AllowNull);
end;

procedure TTestInterfaceParser.ParseEmpty;
begin
  ParseInterface('A','',[]);
end;

procedure TTestInterfaceParser.ParseEmptyInheritance;
begin
  ParseInterface('A','B',[]);
end;

procedure TTestInterfaceParser.ParseMixinEmpty;
begin
  IsMixin:=true;
  Version:=v2;
  ParseInterface('A','',[]);
end;

procedure TTestInterfaceParser.ParseMixinEmptyInheritance;
begin
  IsMixin:=true;
  Version:=v2;
  ParseInterface('A','B',[]);
end;

procedure TTestInterfaceParser.ParseCustomAttributes1;
begin
  CustAttributes:='[Constructor(DOMString type,optional WebGLContextEventInit eventInit)]';
  AssertEquals('Attributes',CustAttributes,ParseInterface('A','B',[]).Attributes.AsString(True));
end;

procedure TTestConstInterfaceParser.ParseConstInt;
begin
  ParseConst('A','short','123',ctInteger);
end;

procedure TTestConstInterfaceParser.Parse2ConstInt;
Var
  Id : TIDLInterfaceDefinition;
  CD : TIDLConstDefinition;

begin
  Id:=ParseInterFace('IA','',['const GLenum READ_BUFFER = 0x0C02','const GLenum UNPACK_ROW_LENGTH = 0x0CF2']);
  AssertEquals('Correct class',TIDLConstDefinition,Id.Members[0].ClassType);
  CD:=Id.Members[0] as TIDLConstDefinition;
  AssertEquals('Const Name','READ_BUFFER',CD.Name);
  AssertEquals('Const type','GLenum',CD.TypeName);
  AssertEquals('Const consttype',ctInteger,CD.ConstType);
  AssertEquals('Const value','0x0C02',CD.Value);
  AssertEquals('Const null allowed',False,CD.AllowNull);
  CD:=Id.Members[1] as TIDLConstDefinition;
  AssertEquals('Const Name','UNPACK_ROW_LENGTH',CD.Name);
  AssertEquals('Const type','GLenum',CD.TypeName);
  AssertEquals('Const consttype',ctInteger,CD.ConstType);
  AssertEquals('Const value','0x0CF2',CD.Value);
  AssertEquals('Const null allowed',False,CD.AllowNull);
end;

procedure TTestConstInterfaceParser.ParseConstIntHex;
begin
  ParseConst('A','short','0xABCDEF',ctInteger);
end;

procedure TTestConstInterfaceParser.ParseConstLongint;
begin
  ParseConst('A','long','123',ctInteger);
end;

procedure TTestConstInterfaceParser.ParseConstLongLongint;
begin
  ParseConst('A','long long','123',ctInteger);
end;

procedure TTestConstInterfaceParser.ParseConstUnsignedShortint;
begin
  ParseConst('A','unsigned short','123',ctInteger);
end;

procedure TTestConstInterfaceParser.ParseConstUnsignedLongint;
begin
  ParseConst('A','unsigned long','123',ctInteger);
end;

procedure TTestConstInterfaceParser.ParseConstUnsignedLongLongint;
begin
  ParseConst('A','unsigned long long','123',ctInteger);
end;

procedure TTestConstInterfaceParser.ParseConstFloat;
begin
  ParseConst('A','float','1.23',ctFloat);
end;

procedure TTestConstInterfaceParser.ParseConstNan;
begin
  ParseConst('A','float','NaN',ctNaN);
end;

procedure TTestConstInterfaceParser.ParseConstInfinity;
begin
  ParseConst('A','float','Infinity',ctInfinity);
end;

procedure TTestConstInterfaceParser.ParseConstNegInfinity;
begin
  ParseConst('A','float','-Infinity',ctNegInfinity);
end;

procedure TTestConstInterfaceParser.ParseConstNull;
begin
  ParseConst('A','short ?','123',ctInteger);
end;

procedure TTestConstInterfaceParser.ParseConstOctet;
begin
  ParseConst('A','octet','123',ctInteger);
end;

procedure TTestConstInterfaceParser.ParseConstByte;
begin
  ParseConst('A','byte','123',ctInteger);
end;

procedure TTestConstInterfaceParser.ParseConstBooleantrue;
begin
  ParseConst('A','boolean','true',ctBoolean);
end;

procedure TTestConstInterfaceParser.ParseConstBooleanFalse;
begin
  ParseConst('A','boolean','false',ctBoolean);
end;

procedure TTestConstInterfaceParser.ParseConstIdentifier;
begin
  ParseConst('A','Zaza','false',ctBoolean);
end;


{ TTestEnumParser }

procedure TTestEnumParser.TestEnum(const aSource, AName: UTF8String;
  AValues: array of UTF8String);

Var
  E : TIDLEnumDefinition;
  i : Integer;

begin
  InitSource('enum '+ASource+';');
  Parser.Parse;
  AssertEquals('Definition count',1,Definitions.Count);
  AssertEquals('Correct class',TIDLEnumDefinition,Definitions[0].ClassType);
  E:=Definitions[0] as TIDLEnumDefinition;
  AssertEquals('Name',AName,E.Name);
  AssertEquals('Value count',Length(AValues),E.Values.Count);
  For I:=0 to E.Values.Count-1 do
    AssertEquals('Value '+IntToStr(i),AValues[i],E.Values[i]);
end;

procedure TTestEnumParser.TestSingle;
begin
  TestEnum('A { "one" }','A',['one']);
end;

procedure TTestEnumParser.TestTwo;
begin
  TestEnum('A { "one", "two" }','A',['one','two']);
end;

procedure TTestEnumParser.TestMissingIdent;
begin
  AssertParserError('No ident','enum { "one" };');
end;

procedure TTestEnumParser.TestMissingOpening;
begin
  AssertParserError('No {','enum A "one" };');
end;

procedure TTestEnumParser.TestMissingClosing;
begin
  AssertParserError('No }','enum A { "one" ;');
end;

procedure TTestEnumParser.TestMissingSemicolon;
begin
  AssertParserError('No ; ','enum A { "one" }');
end;

procedure TTestEnumParser.TestMissingComma;
begin
  AssertParserError('No ; ','enum A { "one" "two"}');
end;

{ TTestParser }

function TTestParser.GetList: TIDLDefinitionList;
begin
  Result:=Context.Definitions;
end;

procedure TTestParser.SetVersion(AValue: TWebIDLVersion);
begin
  if FVersion=AValue then Exit;
  FVersion:=AValue;
  if Assigned(FParser) then
    FParser.Version:=aValue;
end;

procedure TTestParser.Setup;
begin
  FContext:=TWebIDLContext.Create;
  FVersion:=v1;
  inherited Setup;
end;

procedure TTestParser.TearDown;
begin
  FreeAndNil(FParser);
  FreeAndNil(FContext);
  inherited TearDown;
end;

procedure TTestParser.InitSource(const aSource: UTF8String);
begin
  Writeln(TestName+' source : ');
  Writeln(aSource);
  FParser:=TWebIDLParser.Create(Context,aSource);
  FParser.Version:=Version;
end;

procedure TTestParser.AssertParserError(const Msg: String;
  const aSource: UTF8String);
begin
  InitSource(aSource);
  AssertException(Msg,EWebIDLParser,@Parser.Parse);
end;

class procedure TTestParser.AssertEquals(Msg: String; AExpected,
  AActual: TConstType);
begin
  AssertEQuals(Msg,GetEnumName(TypeInfo(TConstType),Ord(AExpected)),GetEnumName(TypeInfo(TConstType),Ord(AActual)));
end;

class procedure TTestParser.AssertEquals(Msg: String; AExpected,
  AActual: TAttributeOption);
begin
  AssertEquals(Msg,GetEnumName(TypeInfo(TAttributeOption),Ord(AExpected)),GetEnumName(TypeInfo(TAttributeOption),Ord(AActual)));
end;

class procedure TTestParser.AssertEquals(Msg: String; AExpected, AActual: TFunctionOption);
begin
  AssertEquals(Msg,GetEnumName(TypeInfo(TFunctionOption),Ord(AExpected)),GetEnumName(TypeInfo(TFunctionOption),Ord(AActual)));
end;

class procedure TTestParser.AssertEquals(Msg: String; AExpected,
  AActual: TAttributeOptions);
begin
  AssertEquals(Msg,SetToString(PTypeInfo(TypeInfo(TAttributeOptions)),Integer(AExpected),True),
                   SetToString(PTypeInfo(TypeInfo(TAttributeOptions)),Integer(AActual),True));
end;

class procedure TTestParser.AssertEquals(Msg: String; AExpected,
  AActual: TFunctionOptions);
begin
  AssertEquals(Msg,SetToString(PTypeInfo(TypeInfo(TFunctionOptions)),Integer(AExpected),True),
                   SetToString(PTypeInfo(TypeInfo(TFunctionOptions)),Integer(AActual),True));
end;

initialization
  RegisterTests([TTestEnumParser,
                 TTestInterfaceParser,
                 TTestConstInterfaceParser,
                 TTestTypeDefParser,
                 TTestDictionaryParser,
                 TTestFunctionCallbackParser,
                 TTestImplementsParser,
                 TTestIncludesParser,
                 TTestAttributeInterfaceParser,
                 TTestIterableInterfaceParser,
                 TTestSerializerInterfaceParser,
                 TTestOperationInterfaceParser,
                 TTestMapLikeInterfaceParser,
                 TTestSetLikeInterfaceParser]);
end.

