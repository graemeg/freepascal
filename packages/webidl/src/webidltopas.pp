{
    This file is part of the Free Component Library

    WEBIDL to pascal code converter
    Copyright (c) 2018 by Michael Van Canneyt michael@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit webidltopas;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, contnrs, WebIDLParser, WebIDLScanner, WebIDLDefs, pascodegen;

Type

  { TWebIDLToPas }

  { TPasData }

  TPasData = Class(TObject)
  private
    FPasName: String;
  Public
    Constructor Create(APasName : String);
    Property PasName : String read FPasName;
  end;

  TConversionOption = (coDictionaryAsClass,coUseNativeTypeAliases,coExternalConst,coExpandUnionTypeArgs,coaddOptionsToheader);
  TConversionOptions = Set of TConversionOption;

  TWebIDLToPas = Class(TPascalCodeGenerator)
  private
    FClassPrefix: String;
    FClassSuffix: String;
    FContext: TWebIDLContext;
    FDictionaryClassParent: String;
    FFieldPrefix: String;
    FIncludeImplementationCode: TStrings;
    FIncludeInterfaceCode: TStrings;
    FInputFileName: String;
    FOptions: TConversionOptions;
    FOutputFileName: String;
    FTypeAliases: TStrings;
    FVerbose: Boolean;
    FWebIDLVersion: TWebIDLVersion;
    FPasNameList : TFPObjectList;
    FAutoTypes : TStrings;
    procedure SetIncludeImplementationCode(AValue: TStrings);
    procedure SetIncludeInterfaceCode(AValue: TStrings);
    procedure SetTypeAliases(AValue: TStrings);
  Protected
    procedure AddOptionsToHeader;
    Procedure Parse; virtual;
    Procedure WritePascal; virtual;
    function CreateParser(aContext: TWebIDLContext; S: TWebIDLScanner): TWebIDLParser; virtual;
    function CreateScanner(S: TStream): TWebIDLScanner;virtual;
    Function CreateContext : TWebIDLContext; virtual;
    Function BaseUnits : String; override;
    // Auxiliary routines
    procedure Getoptions(L: TStrings); virtual;
    procedure ProcessDefinitions; virtual;
    function CreatePasName(aName: String): TPasData;virtual;
    procedure AllocatePasNames(aList: TIDLDefinitionList; ParentName: String='');virtual;
    Function AllocatePasName(D: TIDLDefinition; ParentName: String='') : TPasData;virtual;
    procedure EnsureUniqueNames(ML: TIDLDefinitionList);virtual;
    function WriteFunctionImplicitTypes(aList: TIDLDefinitionList): Integer;virtual;
    function WriteAttributeImplicitTypes(aList: TIDLDefinitionList): Integer;virtual;
    function WriteDictionaryMemberImplicitTypes(aList: TIDLDefinitionList): Integer;virtual;
    function AddSequenceDef(ST: TIDLSequenceTypeDefDefinition): Boolean; virtual;
    function GetName(ADef: TIDLDefinition): String;virtual;
    function GetTypeName(Const aTypeName: String; ForTypeDef: Boolean=False): String;virtual;
    function GetTypeName(aTypeDef: TIDLTypeDefDefinition; ForTypeDef: Boolean=False): String;virtual;
    function CheckUnionTypeDefinition(D: TIDLDefinition): TIDLUnionTypeDefDefinition;virtual;
    procedure AddArgumentToOverloads(aList: TFPObjectlist; AName, ATypeName: String);virtual;
    procedure AddUnionOverloads(aList: TFPObjectlist; AName: String;  UT: TIDLUnionTypeDefDefinition);virtual;
    procedure AddArgumentToOverloads(aList: TFPObjectlist; adef: TIDLArgumentDefinition);virtual;
    procedure AddOverloads(aList: TFPObjectlist; adef: TIDLFunctionDefinition; aIdx: Integer);virtual;
    function CloneNonPartialArgumentList(aList: TFPObjectlist; ADest: TFPObjectlist= Nil; AsPartial: Boolean=True): integer;virtual;
    function GetOverloads(aDef: TIDLFunctionDefinition): TFPObjectlist;virtual;
    function GetArguments(aList: TIDLDefinitionList; ForceBrackets: Boolean): String;virtual;
    function HaveConsts(aList: TIDLDefinitionList): Boolean;virtual;
    // Actual code generation routines
    // Lists. Return the number of actually written defs.
    function WriteCallBackDefs(aList: TIDLDefinitionList): Integer; virtual;
    Function WriteDictionaryDefs(aList: TIDLDefinitionList) : Integer;virtual;
    Function WriteForwardClassDefs(aList: TIDLDefinitionList) : Integer;virtual;
    Function WriteInterfaceDefs(aList: TIDLDefinitionList) : Integer;virtual;
    Function WriteMethodDefs(aList: TIDLDefinitionList) : Integer;virtual;
    Function WriteTypeDefs(aList: TIDLDefinitionList) : Integer;virtual;
    Function WriteEnumDefs(aList: TIDLDefinitionList) : Integer;virtual;
    function WriteConsts(aList: TIDLDefinitionList): Integer;virtual;
    function WriteProperties(aList: TIDLDefinitionList): Integer;
    function WritePlainFields(aList: TIDLDefinitionList): Integer;virtual;
    function WriteDictionaryFields(aList: TIDLDefinitionList): Integer;virtual;
    function WritePrivateReadOnlyFields(aList: TIDLDefinitionList): Integer;virtual;
    // Actual definitions. Return true if a definition was written.
    Function WriteForwardClassDef(D: TIDLStructuredDefinition) : Boolean;virtual;
    function WriteFunctionTypeDefinition(aDef: TIDLFunctionDefinition): Boolean;virtual;
    function WriteFunctionDefinition(aDef: TIDLFunctionDefinition): Boolean;virtual;
    function WriteTypeDef(aDef: TIDLTypeDefDefinition): Boolean; virtual;
    function WriteRecordDef(aDef: TIDLRecordDefinition): Boolean; virtual;
    function WriteEnumDef(aDef: TIDLEnumDefinition): Boolean; virtual;
    function WriteDictionaryField(aField: TIDLDictionaryMemberDefinition): Boolean;virtual;
    Function WritePrivateReadOnlyField(aAttr: TIDLAttributeDefinition) : Boolean;virtual;
    Function WriteField(aAttr: TIDLAttributeDefinition) : Boolean;virtual;
    Function WriteReadonlyProperty(aAttr: TIDLAttributeDefinition) : Boolean;virtual;
    Function WriteConst(aConst: TIDLConstDefinition) : Boolean ;virtual;
    function WriteInterfaceDef(Intf: TIDLInterfaceDefinition): Boolean; virtual;
    function WriteDictionaryDef(aDict: TIDLDictionaryDefinition): Boolean; virtual;
    // Additional
    procedure WriteAliasTypeDef(aDef: TIDLTypeDefDefinition);virtual;
    procedure WritePromiseDef(aDef: TIDLPromiseTypeDefDefinition);virtual;
    procedure WriteSequenceDef(aDef: TIDLSequenceTypeDefDefinition);virtual;
    procedure WriteUnionDef(aDef: TIDLUnionTypeDefDefinition);virtual;
    // Extra interface/Implementation code.
    procedure WriteImplementation; virtual;
    procedure WriteIncludeInterfaceCode; virtual;
    Property Context : TWebIDLContext Read FContext;
  Public
    Constructor Create(Aowner : TComponent); override;
    Destructor Destroy; override;
    Procedure Execute;
  Published
    Property InputFileName : String Read FInputFileName Write FInputFileName;
    Property OutputFileName : String Read FOutputFileName Write FOutputFileName;
    Property Verbose : Boolean Read FVerbose Write FVerbose;
    Property FieldPrefix : String Read FFieldPrefix Write FFieldPrefix;
    Property ClassPrefix : String Read FClassPrefix Write FClassPrefix;
    Property ClassSuffix : String Read FClassSuffix Write FClassSuffix;
    Property Options : TConversionOptions Read FOptions Write FOptions;
    Property WebIDLVersion : TWebIDLVersion Read FWebIDLVersion Write FWebIDLVersion;
    Property TypeAliases : TStrings Read FTypeAliases Write SetTypeAliases;
    Property IncludeInterfaceCode : TStrings Read FIncludeInterfaceCode Write SetIncludeInterfaceCode;
    Property IncludeImplementationCode : TStrings Read FIncludeImplementationCode Write SetIncludeImplementationCode;
    Property DictionaryClassParent : String Read FDictionaryClassParent Write FDictionaryClassParent;
  end;

implementation

uses typinfo;

{ TPasData }

constructor TPasData.Create(APasName: String);
begin
  FPasName:=APasName;
end;

{ TWebIDLToPas }

function TWebIDLToPas.CreateContext: TWebIDLContext;
begin
  Result:=TWebIDLContext.Create(True);
end;

function TWebIDLToPas.CreateScanner(S : TStream) :  TWebIDLScanner;

begin
  Result:=TWebIDLScanner.Create(S);
end;

function TWebIDLToPas.CreateParser(aContext : TWebIDLContext;S : TWebIDLScanner) :  TWebIDLParser;

begin
  Result:=TWebIDLParser.Create(aContext,S);
  Result.Version:=FWebIDLVersion;
end;

procedure TWebIDLToPas.Parse;

Var
  F : TFileStream;
  S : TWebIDLScanner;
  P : TWebIDLParser;

begin
  P:=Nil;
  F:=TFileStream.Create(InputFileName,fmOpenRead or fmShareDenyWrite);
  try
    S:=CreateScanner(F);
    P:=CreateParser(Context,S);
    P.Parse;
  finally
    P.Free;
    S.Free;
    F.Free;
  end;
end;

function TWebIDLToPas.GetName(ADef: TIDLDefinition): String;

begin
  If Assigned(ADef) and (TObject(ADef.Data) is TPasData) then
    Result:=TPasData(ADef.Data).PasName
  else
    Result:=ADef.Name;
end;

function TWebIDLToPas.HaveConsts(aList: TIDLDefinitionList): Boolean;

Var
  D : TIDLDefinition;

begin
  Result:=False;
  For D in aList do
    if D is TIDLConstDefinition then
      Exit(True);
end;

function TWebIDLToPas.WritePrivateReadOnlyFields(aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;
  A : TIDLAttributeDefinition absolute D;

begin
  Result:=0;
  For D in aList do
    if (D is TIDLAttributeDefinition) then
      if (aoReadOnly in A.Options) then
        if WritePrivateReadOnlyField(A) then
          Inc(Result);
end;

function TWebIDLToPas.WriteProperties(aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;
  A : TIDLAttributeDefinition absolute D;

begin
  Result:=0;
  For D in aList do
    if (D is TIDLAttributeDefinition) then
      if (aoReadOnly in A.Options) then
        if WriteReadOnlyProperty(A) then
          Inc(Result);
end;

function TWebIDLToPas.WriteConst(aConst: TIDLConstDefinition): Boolean;

Const
  ConstTypes : Array[TConstType] of String =
     ('Double','NativeInt','Boolean','JSValue','JSValue','JSValue','JSValue','String','JSValue','JSValue');
Var
  S : String;

begin
  Result:=True;
  // Consts cannot be strings
  if coExternalConst in Options then
    begin
    S:=ConstTypes[aConst.ConstType];
    Addln('%s : %s;',[GetName(aConst),S])
    end
  else
    begin
    S:=aConst.Value;
    if aConst.ConstType=ctInteger then
      S:=StringReplace(S,'0x','$',[]);
    Addln('%s = %s;',[GetName(aConst),S])
    end;
end;

function TWebIDLToPas.WriteConsts(aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;

begin
  EnsureSection(csConst);
  Indent;
  Result:=0;
  For D in aList do
    if D is TIDLConstDefinition then
      if WriteConst(D as TIDLConstDefinition) then
        Inc(Result);
  Undent;
end;

function TWebIDLToPas.WritePlainFields(aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;
  A : TIDLAttributeDefinition absolute D;

begin
  EnsureSection(csDeclaration);
  Indent;
  Result:=0;
  For D in aList do
    if D is TIDLAttributeDefinition then
      if Not (aoReadOnly in A.Options) then
        if WriteField(A) then
          Inc(Result);
  Undent;
end;

function TWebIDLToPas.WriteDictionaryField(
  aField: TIDLDictionaryMemberDefinition): Boolean;

Var
  Def,N,TN : String;

begin
  Result:=True;
  N:=GetName(aField);
  TN:=GetTypeName(aField.MemberType);
  if TN='record' then
    TN:='TJSObject';
  if SameText(N,TN) then
    N:='_'+N;
  Def:=Format('%s : %s;',[N,TN]);
  if (N<>aField.Name) then
    Def:=Def+Format('external name ''%s'';',[aField.Name]);
  AddLn(Def);
end;

function TWebIDLToPas.WriteDictionaryFields(aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;
  M : TIDLDictionaryMemberDefinition absolute D;

begin
  Indent;
  Result:=0;
  For D in aList do
    if D is TIDLDictionaryMemberDefinition then
      if WriteDictionaryField(M) then
        Inc(Result);
  Undent;
end;

function TWebIDLToPas.WriteMethodDefs(aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;
  FD : TIDLFunctionDefinition absolute D;

begin
  Result:=0;
  for D in aList do
    if D is TIDLFunctionDefinition then
      if Not (foCallBack in FD.Options) then
         if WriteFunctionDefinition(FD) then
           Inc(Result);
end;

function TWebIDLToPas.AddSequenceDef(ST: TIDLSequenceTypeDefDefinition
  ): Boolean;

var
  TN : String;
begin
  TN:=GetTypeName(ST);
  Result:=FAutoTypes.IndexOf(TN)=-1;
  if Result then
    begin
    FAutoTypes.Add(TN);
    DoLog('Automatically adding %s sequence definition.',[TN]);
    AddLn('%s = Array of %s;',[TN,GetTypeName(ST.ElementType)]);
    ST.Data:=CreatePasName(TN);
    end;
end;

function TWebIDLToPas.WriteFunctionImplicitTypes(aList: TIDLDefinitionList): Integer;

Var
  D,D2,D3 : TIDLDefinition;
  FD : TIDLFunctionDefinition absolute D;
  DA : TIDLArgumentDefinition absolute D2;
  UT : TIDLUnionTypeDefDefinition;

begin
  Result:=0;
  for D in aList do
    if D is TIDLFunctionDefinition then
      if Not (foCallBack in FD.Options) then
        begin
        if (FD.ReturnType is TIDLSequenceTypeDefDefinition) then
          if AddSequenceDef(FD.ReturnType as TIDLSequenceTypeDefDefinition) then
            Inc(Result);
        For D2 in FD.Arguments do
          if (DA.ArgumentType is TIDLSequenceTypeDefDefinition) then
            begin
            if AddSequenceDef(DA.ArgumentType as TIDLSequenceTypeDefDefinition) then
              Inc(Result);
            end
          else
            begin
            UT:=CheckUnionTypeDefinition(DA.ArgumentType);
            if Assigned(UT) then
              For D3 in UT.Union do
                if (D3 is TIDLSequenceTypeDefDefinition) then
                  if AddSequenceDef(D3 as TIDLSequenceTypeDefDefinition) then
                    Inc(Result);
            end;
        end;
  if Result>0 then
    AddLn('');
end;

function TWebIDLToPas.WriteAttributeImplicitTypes(aList: TIDLDefinitionList
  ): Integer;
Var
  D : TIDLDefinition;
  FA : TIDLAttributeDefinition absolute D;

begin
  Result:=0;
  for D in aList do
    if D is TIDLAttributeDefinition then
      if (FA.AttributeType is TIDLSequenceTypeDefDefinition) then
        if AddSequenceDef(FA.AttributeType as TIDLSequenceTypeDefDefinition) then
          Inc(Result);
end;

function TWebIDLToPas.WriteDictionaryMemberImplicitTypes(
  aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;
  FD : TIDLDictionaryMemberDefinition absolute D;

begin
  Result:=0;
  for D in aList do
    if D is TIDLDictionaryMemberDefinition then
      if (FD.MemberType is TIDLSequenceTypeDefDefinition) then
        if AddSequenceDef(FD.MemberType as TIDLSequenceTypeDefDefinition) then
          Inc(Result);
end;

procedure TWebIDLToPas.EnsureUniqueNames(ML : TIDLDefinitionList);

Var
  L : TFPObjectHashTable;

  Procedure CheckRename(aD : TIDLDefinition);

  var
    I : integer;
    NOrig,N,N2 : String;
    isDup : Boolean;
    D2 : TIDLDefinition;

  begin
    NOrig:=GetName(aD);
    N:=LowerCase(NOrig);
    N2:=N;
    I:=0;
    isDup:=False;
    Repeat
      D2:=TIDLDefinition(L.Items[N2]);
      if (D2<>Nil) then
        // Overloads
        begin
        isDup:=((D2 is TIDLFunctionDefinition) and (ad is TIDLFunctionDefinition));
        if IsDup then
          D2:=Nil
        else
          begin
          inc(I);
          N2:=KeywordPrefix+N+KeywordSuffix;
          Norig:=KeywordPrefix+NOrig+KeywordSuffix;
          end;
        end;
    Until (D2=Nil);
    if (N<>N2) then
      begin
      N:=GetName(aD);
      DoLog('Renaming duplicate identifier (%s) %s to %s',[aD.ClassName,N,Norig]);
      // Original TPasName is in list, will be freed automatically
      aD.Data:=CreatePasName(NOrig);
      end;
    if not IsDup then
      L.Add(N2,aD);
  end;

var
  D : TIDLDefinition;

begin
  L:=TFPObjectHashTable.Create(False);
  try
    For D in ML Do
      if not (D is TIDLConstDefinition) then
        CheckRename(D);
    For D in ML Do
      if (D is TIDLConstDefinition) then
        CheckRename(D);
  finally
    L.Free;
  end;
end;

function TWebIDLToPas.WriteInterfaceDef(Intf: TIDLInterfaceDefinition): Boolean;

Var
  CN,PN : String;
  Decl : String;
  ML : TIDLDefinitionList;

begin
  Result:=True;
  ML:=TIDLDefinitionList.Create(Nil,False);
  try
    Intf.GetFullMemberList(ML);
    EnsureUniqueNames(ML);
    CN:=GetName(Intf);
    ClassHeader(CN);
    WriteFunctionImplicitTypes(ML);
    WriteAttributeImplicitTypes(ML);
    Decl:=Format('%s = class external name %s ',[CN,MakePascalString(Intf.Name,True)]);
    if Assigned(Intf.ParentInterface) then
      PN:=GetName(Intf.ParentInterface)
    else
      PN:=GetTypeName(Intf.ParentName);
    if PN<>'' then
      Decl:=Decl+Format(' (%s)',[PN]);
    AddLn(Decl);
    AddLn('Private');
    Indent;
    WritePrivateReadOnlyFields(ML);
    Undent;
    AddLn('Public');
    if HaveConsts(ML) then
      begin
      Indent;
      PushSection(csUnknown);
      WriteConsts(ML);
      PopSection;
      Undent;
      AddLn('Public');
      end;
    Indent;
    WritePlainFields(ML);
    WriteMethodDefs(ML);
    WriteProperties(ML);
    Undent;
    AddLn('end;');
  finally
    ML.Free;
  end;
end;

function TWebIDLToPas.WriteDictionaryDef(aDict: TIDLDictionaryDefinition
  ): Boolean;

Var
  CN,CP : String;
  ML : TIDLDefinitionList;
  PD: TIDLDictionaryDefinition;

begin
  Result:=True;
  ML:=TIDLDefinitionList.Create(Nil,False);
  try
    PD:=aDict;
    While PD<>Nil do
      begin
      PD.GetFullMemberList(ML);
      PD:=PD.ParentDictionary;
      end;
    CN:=GetName(aDict);
    CP:=DictionaryClassParent;
    if CP='' then
      CP:='TJSObject';
    ClassHeader(CN);
    WriteDictionaryMemberImplicitTypes(ML);
    if (coDictionaryAsClass in Options) then
      Addln('%s = class(%s)',[CN,CP])
    else
      Addln('%s = record',[CN]);
    WriteDictionaryFields(ML);
    AddLn('end;');
  finally
    ML.Free;
  end;
end;

constructor TWebIDLToPas.Create(Aowner: TComponent);
begin
  inherited Create(Aowner);
  WebIDLVersion:=v2;
  FieldPrefix:='F';
  ClassPrefix:='T';
  ClassSuffix:='';
  Switches.Add('modeswitch externalclass');
  FTypeAliases:=TStringList.Create;
  FPasNameList:=TFPObjectList.Create(True);
  FAutoTypes:=TStringList.Create;
  FIncludeInterfaceCode:=TStringList.Create;
  FIncludeImplementationCode:=TStringList.Create;
end;


destructor TWebIDLToPas.Destroy;
begin
  FreeAndNil(FIncludeInterfaceCode);
  FreeAndNil(FIncludeImplementationCode);
  FreeAndNil(FAutoTypes);
  FreeAndNil(FTypeAliases);
  FreeAndNil(FPasNameList);
  inherited Destroy;
end;

procedure TWebIDLToPas.WriteImplementation;

Var
  S : String;

begin
  Addln('');
  For S in FIncludeImplementationCode do
    Addln(S);
  Addln('');
end;

function TWebIDLToPas.GetTypeName(aTypeDef : TIDLTypeDefDefinition; ForTypeDef : Boolean = False): String;

begin
  if ATypeDef is TIDLSequenceTypeDefDefinition then
    begin
    if Assigned(aTypeDef.Data) then
      Result:=GetName(aTypeDef)
    else
      begin
      Result:=GetTypeName(TIDLSequenceTypeDefDefinition(aTypeDef).ElementType,ForTypeDef);
      Result:='T'+Result+'DynArray';
      end
    end
  else
    Result:=GetTypeName(aTypeDef.TypeName,ForTypeDef);
end;

function TWebIDLToPas.GetTypeName(const aTypeName: String; ForTypeDef: Boolean
  ): String;


  Function UsePascalType(Const aPascalType : string) : String;

  begin
    if (coUseNativeTypeAliases in Options) and ForTypeDef then
      Result:=StringReplace(aTypeName,' ','',[rfReplaceAll])
    else
      Result:=aPascalType;
  end;

Var
  A,TN : UTF8String;
  D : TIDLDefinition;

begin
  Case aTypeName of
    'union': TN:='JSValue';
    'short': TN:=UsePascalType('Integer');
    'long': TN:=UsePascalType('Integer');
    'long long': TN:=UsePascalType('NativeInt');
    'unsigned short': TN:=UsePascalType('Cardinal');
    'unrestricted float': TN:=UsePascalType('Double');
    'unrestricted double': TN:=UsePascalType('Double');
    'unsigned long': TN:=UsePascalType('NativeInt');
    'unsigned long long': TN:=UsePascalType('NativeInt');
    'octet': TN:=UsePascalType('Byte');
    'any' : TN:=UsePascalType('JSValue');
    'float' : TN:=UsePascalType('Double');
    'double' : TN:=UsePascalType('Double');
    'DOMString',
    'USVString',
    'ByteString' : TN:=UsePascalType('String');
    'object' : TN:=UsePascalType('TJSObject');
    'Error' : TN:=UsePascalType('TJSError');
    'DOMException' : TN:=UsePascalType('TJSError');
    'ArrayBuffer',
    'DataView',
    'Int8Array',
    'Int16Array',
    'Int32Array',
    'Uint8Array',
    'Uint16Array',
    'Uint32Array',
    'Uint8ClampedArray',
    'Float32Array',
    'Float64Array' : TN:='TJS'+aTypeName;
  else
    TN:=aTypeName;
    D:=FContext.FindDefinition(TN);
    if D<>Nil then
      TN:=GetName(D)
    else
      begin
      A:=FTypeAliases.Values[TN];
      If (A<>'') then
        TN:=A;
      end;
  end;
  Result:=TN;
end;

function TWebIDLToPas.WritePrivateReadOnlyField(aAttr: TIDLAttributeDefinition
  ): Boolean;

begin
  AddLn('%s%s : %s; external name ''%s''; ',[FieldPrefix,GetName(aAttr),GetTypeName(aAttr.AttributeType),aAttr.Name]);
end;

function TWebIDLToPas.WriteField(aAttr: TIDLAttributeDefinition): Boolean;

Var
  Def,TN,N : String;

begin
  Result:=True;
  N:=GetName(aAttr);
  TN:=GetTypeName(aAttr.AttributeType);
  if TN='record' then
    TN:='TJSObject';
  if SameText(N,TN) then
    N:='_'+N;
  Def:=Format('%s : %s;',[N,TN]);
  if (N<>aAttr.Name) then
    Def:=Def+Format('external name ''%s'';',[aAttr.Name]);
  AddLn(Def);
end;

function TWebIDLToPas.WriteReadonlyProperty(aAttr: TIDLAttributeDefinition
  ): Boolean;

Var
  TN,N,PN : String;

begin
  Result:=True;
  N:=GetName(aAttr);
  PN:=N;
  TN:=GetTypeName(aAttr.AttributeType);
  if SameText(PN,TN) then
    PN:='_'+PN;
  AddLn('Property %s : %s Read %s%s; ',[PN,TN,FieldPrefix,N]);
end;


function TWebIDLToPas.WriteForwardClassDef(D: TIDLStructuredDefinition): Boolean;

begin
  Result:=not D.IsPartial;
  if Result then
    AddLn('%s = Class;',[GetName(D)]);
end;

function TWebIDLToPas.WriteForwardClassDefs(aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;

begin
  Result:=0;
  Comment('Forward class definitions');
  For D in aList do
    if D is TIDLInterfaceDefinition then
      if WriteForwardClassDef(D as TIDLInterfaceDefinition) then
        Inc(Result);
  if coDictionaryAsClass in Options then
    For D in aList do
      if D is TIDLDictionaryDefinition then
        if WriteForwardClassDef(D as TIDLDictionaryDefinition) then
          Inc(Result);
end;

procedure TWebIDLToPas.WriteSequenceDef(aDef : TIDLSequenceTypeDefDefinition);

begin
  Addln('%s = array of %s;',[GetName(aDef),GetTypeName(aDef.ElementType)])
end;


procedure TWebIDLToPas.WriteUnionDef(aDef : TIDLUnionTypeDefDefinition);

Var
  S : UTF8String;
  D : TIDLDefinition;
begin
  S:='';
  For D in adef.Union do
    begin
    if (S<>'') then
      S:=S+', ';
    S:=S+(D as TIDLTypeDefDefinition).TypeName;
    end;
  Comment('Union of '+S);
  AddLn('%s = JSValue; ',[GetName(aDef)])
end;


procedure TWebIDLToPas.WritePromiseDef(aDef : TIDLPromiseTypeDefDefinition);

begin
  AddLn('%s = TJSPromise;',[GetName(aDef)]);
end;

procedure TWebIDLToPas.WriteAliasTypeDef(aDef : TIDLTypeDefDefinition);

Var
  TN : String;

begin
  TN:=GetTypeName(aDef,True);
  AddLn('%s = %s;',[GetName(aDef),TN]);
end;

function TWebIDLToPas.WriteTypeDef(aDef: TIDLTypeDefDefinition): Boolean;

begin
  Result:=True;
  if ADef is TIDLSequenceTypeDefDefinition then
    WriteSequenceDef(aDef as TIDLSequenceTypeDefDefinition)
  else if ADef is TIDLUnionTypeDefDefinition then
    WriteUnionDef(aDef as TIDLUnionTypeDefDefinition)
  else if ADef is TIDLPromiseTypeDefDefinition then
    WritePromiseDef(aDef as TIDLPromiseTypeDefDefinition)
  else if ADef is TIDLRecordDefinition then
    WriteRecordDef(aDef as TIDLRecordDefinition)
  else
    WriteAliasTypeDef(aDef);
end;

function TWebIDLToPas.WriteRecordDef(aDef: TIDLRecordDefinition): Boolean;

Var
  KT,VT : String;

begin
  Result:=True;
  KT:=GetTypeName(aDef.KeyType);
  VT:=GetTypeName(aDef.ValueType);
  AddLn('%s = Class(TJSObject)',[GetName(aDef)]);
  AddLn('private');
  Indent;
  AddLn('function GetValue(aKey: %s): %s; external name ''[]'';',[KT,VT]);
  AddLn('procedure SetValue(aKey: %s; const AValue: %s); external name ''[]'';',[KT,VT]);
  undent;
  AddLn('public');
  Indent;
  AddLn('property Values[Name: %s]: %s read GetProperties write SetProperties; default;',[KT,VT]);
  undent;
  AddLn('end;');
end;

function TWebIDLToPas.WriteTypeDefs(aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;
  TD : TIDLTypeDefDefinition absolute D;

begin
  Result:=0;
  EnsureSection(csType);
  for D in aList do
    if D is TIDLTypeDefDefinition then
      if WriteTypeDef(TD) then
        Inc(Result);
end;

function TWebIDLToPas.WriteEnumDef(aDef: TIDLEnumDefinition): Boolean;

begin
  Result:=True;
  AddLn('%s = String;',[GetName(aDef)]);
end;

function TWebIDLToPas.WriteEnumDefs(aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;
  ED : TIDLEnumDefinition absolute D;

begin
  Result:=0;
  EnsureSection(csType);
  for D in aList do
    if D is TIDLEnumDefinition then
      if WriteEnumDef(ED) then
        Inc(Result);
end;

function TWebIDLToPas.GetArguments(aList: TIDLDefinitionList;
  ForceBrackets: Boolean): String;

Var
  I : TIDLDefinition;
  A : TIDLArgumentDefinition absolute I;
  Arg : string;

begin
  Result:='';
  For I in aList do
    begin
    Arg:=GetName(A);
    Arg:=Arg+' : '+GetTypeName(A.ArgumentType);
    if Result<>'' then
      Result:=Result+'; ';
    Result:=Result+Arg;
    end;
  if (Result<>'') or ForceBrackets then
    Result:='('+Result+')';
end;

Type
  // A partial argument list is a list which has been generated for a optional argument.
  // Additional arguments can never be added to a partial list...
  TIDLPartialDefinitionList = Class(TIDLDefinitionList);

function TWebIDLToPas.CloneNonPartialArgumentList(aList: TFPObjectlist;
  ADest: TFPObjectlist; AsPartial: Boolean): integer;

Var
  I,J : Integer;
  CD : TIDLDefinition;
  DL,CL : TIDLDefinitionList;

begin
  Result:=0;
  if ADest=Nil then
    ADest:=aList;
  I:=aList.Count-1;
  While (I>=0) do
    begin
    DL:=TIDLDefinitionList(alist[i]);
    if Not (DL is TIDLPartialDefinitionList) then
      begin
      Inc(Result);
      if AsPartial then
        CL:=TIDLPartialDefinitionList.Create(Nil,True)
      else
        CL:=TIDLDefinitionList.Create(Nil,True);
      aDest.Add(CL);
      For J:=0 to DL.Count-1 do
        begin
        CD:=(DL.Definitions[J] as TIDLArgumentDefinition).Clone(Nil);
        CL.Add(CD);
        AllocatePasName(CD);
        end;
      end;
    Dec(I);
    end;
end;

procedure TWebIDLToPas.AddArgumentToOverloads(aList: TFPObjectlist; AName,ATypeName : String);

Var
  I : Integer;
  CD : TIDLArgumentDefinition;
  DL : TIDLDefinitionList;

begin
  For I:=0 to aList.Count-1 do
    begin
    DL:=TIDLDefinitionList(alist[i]);
    if Not (DL is TIDLPartialDefinitionList) then
      begin
      CD:=TIDLArgumentDefinition.Create(Nil,aName);
      CD.ArgumentType:=TIDLTypeDefDefinition.Create(CD,'');
      CD.ArgumentType.TypeName:=aTypeName;
      DL.Add(CD);
      AllocatePasName(cd,'');
      end;
    end;
end;

procedure TWebIDLToPas.AddArgumentToOverloads(aList: TFPObjectlist; adef: TIDLArgumentDefinition);

Var
  I : Integer;
  CD : TIDLDefinition;
  DL : TIDLDefinitionList;

begin
  For I:=0 to aList.Count-1 do
    begin
    DL:=TIDLDefinitionList(alist[i]);
    if Not (DL is TIDLPartialDefinitionList) then
      begin
      CD:=aDef.Clone(Nil);
      DL.Add(CD);
      if aDef.Data<>Nil then
        CD.Data:=CreatePasName(TPasData(aDef.Data).PasName)
      else
        AllocatePasName(cd,'');
      end;
    end;
end;

procedure TWebIDLToPas.AddUnionOverloads(aList: TFPObjectlist; AName : String; UT : TIDLUnionTypeDefDefinition);

Var
  L,L2 : TFPObjectList;
  I,J : Integer;
  D : TIDLDefinitionList;
  Dups : TStringList;

begin
  L2:=Nil;
  Dups:=TStringList.Create;
  Dups.Sorted:=True;
  Dups.Duplicates:=dupIgnore;
  L:=TFPObjectList.Create(False);
  try
    L2:=TFPObjectList.Create(False);
    // Collect non partial argument lists
    for I:=0 to AList.Count-1 do
      begin
      D:=TIDLDefinitionList(alist[i]);
      if Not (D is TIDLPartialDefinitionList) then
        L.Add(D);
      end;
    // Collect unique pascal types. Note that this can reduce the list to 1 element...
    For I:=0 to UT.Union.Count-1 do
      Dups.AddObject(GetTypeName(UT.Union[I] as TIDLTypeDefDefinition),UT.Union[I]);
    // First, clone list and add argument to cloned lists
    For I:=1 to Dups.Count-1 do
      begin
      // Clone list
      CloneNonPartialArgumentList(L,L2,False);
      // Add argument to cloned list
      AddArgumentToOverloads(L2,aName,Dups[i]);
      // Add overloads to original list
      For J:=0 to L2.Count-1 do
        aList.Add(L2[J]);
      L2.Clear;
      end;
    // Add first Union to original list
    AddArgumentToOverloads(L,aName,Dups[0]);
  finally
    Dups.Free;
    L2.Free;
    L.Free;
  end;
end;

function TWebIDLToPas.CheckUnionTypeDefinition(D: TIDLDefinition
  ): TIDLUnionTypeDefDefinition;

begin
  Result:=Nil;
  If (D is TIDLUnionTypeDefDefinition) then
    Result:=D as TIDLUnionTypeDefDefinition
  else
    begin
    D:=Context.FindDefinition((D as TIDLTypeDefDefinition).TypeName);
    if (D is TIDLUnionTypeDefDefinition) then
      Result:=D as TIDLUnionTypeDefDefinition
    end
end;

procedure TWebIDLToPas.AddOverloads(aList: TFPObjectlist;
  adef: TIDLFunctionDefinition; aIdx: Integer);

Var
  Arg : TIDLArgumentDefinition;
  D : TIDLDefinition;
  UT : TIDLUnionTypeDefDefinition;

begin
 if aIdx>=ADef.Arguments.Count then
    exit;
  Arg:=ADef.Argument[aIdx];
  if Arg.IsOptional then
    CloneNonPartialArgumentList(aList);
  // Add current to list.
  D:=Arg.ArgumentType;
  UT:=Nil;
  if coExpandUnionTypeArgs in Options then
    UT:=CheckUnionTypeDefinition(D);
  if UT=Nil then
    AddArgumentToOverloads(aList,Arg)
  else
    AddUnionOverLoads(aList,Arg.Name,UT);
  AddOverloads(aList,aDef,aIdx+1);
end;

function TWebIDLToPas.GetOverloads(aDef: TIDLFunctionDefinition): TFPObjectlist;

begin
  Result:=TFPObjectList.Create;
  try
    Result.Add(TIDLDefinitionList.Create(Nil,True));
    AddOverloads(Result,adef,0);
  except
    Result.Free;
    Raise;
  end;
end;

function TWebIDLToPas.WriteFunctionTypeDefinition(aDef: TIDLFunctionDefinition): Boolean;

Var
  FN,RT,Args : String;

begin
  Result:=True;
  FN:=GetName(aDef);
  RT:=GetTypeName(aDef.ReturnType,False);
  if (RT='void') then
    RT:='';
  Args:=GetArguments(aDef.Arguments,False);
  if (RT='') then
    AddLn('%s = Procedure %s;',[FN,Args])
  else
    AddLn('%s = function %s: %s;',[FN,Args,RT])
end;

function TWebIDLToPas.WriteFunctionDefinition(aDef: TIDLFunctionDefinition): Boolean;

Var
  FN,RT,Suff,Args : String;
  Overloads : TFPObjectList;
  I : Integer;

begin
  Result:=True;
  if not (foConstructor in aDef.Options) then
    begin
    FN:=GetName(aDef);
    if FN<>aDef.Name then
      Suff:=Format('; external name ''%s''',[aDef.Name]);
    RT:=GetTypeName(aDef.ReturnType,False);
    if (RT='void') then
      RT:='';
    end
  else
    FN:='New';
  Overloads:=GetOverloads(ADef);
  try
    for I:=0 to aDef.Arguments.Count-1 do
      if aDef.Argument[i].HasEllipsis then
        Suff:='; varargs';
    if Overloads.Count>1 then
      Suff:=Suff+'; overload';
    For I:=0 to Overloads.Count-1 do
      begin
      Args:=GetArguments(TIDLDefinitionList(Overloads[i]),False);
      if (RT='') then
        begin
        if not (foConstructor in aDef.Options) then
          AddLn('Procedure %s%s%s;',[FN,Args,Suff])
        else
          AddLn('constructor %s%s%s;',[FN,Args,Suff]);
        end
      else
        AddLn('function %s%s: %s%s;',[FN,Args,RT,Suff])
      end;
  finally
    Overloads.Free;
  end;
end;

function TWebIDLToPas.WriteCallBackDefs(aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;
  FD : TIDLFunctionDefinition absolute D;

begin
  Result:=0;
  EnsureSection(csType);
  for D in aList do
    if D is TIDLFunctionDefinition then
      if (foCallBack in FD.Options) then
         if WriteFunctionTypeDefinition(FD) then
           Inc(Result);
end;

function TWebIDLToPas.WriteDictionaryDefs(aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;
  DD : TIDLDictionaryDefinition absolute D;

begin
  Result:=0;
  EnsureSection(csType);
  for D in aList do
    if D is TIDLDictionaryDefinition then
      if not TIDLDictionaryDefinition(D).IsPartial then
        if WriteDictionaryDef(DD) then
          Inc(Result);
end;

function TWebIDLToPas.WriteInterfaceDefs(aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;
  ID : TIDLInterfaceDefinition absolute D;

begin
  Result:=0;
  EnsureSection(csType);
  for D in aList do
    if D is TIDLInterfaceDefinition then
      if not TIDLInterfaceDefinition(D).IsPartial then
        if WriteInterfaceDef(ID) then
          Inc(Result);
end;

procedure TWebIDLToPas.Getoptions(L : TStrings);

Var
  S : String;
  I : Integer;

begin
  L.Add('Automatically generated file by '+ClassName+' on '+FormatDateTime('yyyy-mm-dd hh:nn:ss',Now));
  L.Add('');
  L.Add('Used command-line options : ');
  For I:=1 to ParamCount do
    L.Add(ParamStr(i));
  L.Add('');
  L.Add('Command-line options translate to: ');
  L.Add('');
  S:=SetToString(PtypeInfo(TypeInfo(TConversionOptions)),Integer(OPtions),True);
  L.Add('Options : '+S);
  L.Add('Keyword prefix : '+KeywordPrefix);
  L.Add('Keyword suffix : '+KeywordSuffix);
  L.Add('Class prefix : '+ClassPrefix);
  L.Add('Class suffix : '+ClassSuffix);
  L.Add('Field prefix : '+FieldPrefix);
  Str(WebIDLversion,S);
  L.Add('WEBIDLversion : '+S);
  if TypeAliases.Count>0 then
    begin
    L.Add('Type aliases:');
    L.AddStrings(Self.TypeAliases);
    end;
end;

procedure TWebIDLToPas.AddOptionsToHeader;

Var
  L : TStrings;
begin
  L:=TStringList.Create;
  try
    GetOptions(L);
    Comment(L);
  finally
    L.Free;
  end;
end;

procedure TWebIDLToPas.WriteIncludeInterfaceCode;

Var
  S : String;

begin
  For S in IncludeInterfaceCode do
    Addln(S);
end;

procedure TWebIDLToPas.WritePascal;

begin
  CreateUnitClause;
  CreateHeader;
  if coaddOptionsToheader in Options then
    AddOptionsToHeader;
  EnsureSection(csType);
  Indent;
  WriteForwardClassDefs(Context.Definitions);
  WriteEnumDefs(Context.Definitions);
  WriteTypeDefs(Context.Definitions);
  WriteCallbackDefs(Context.Definitions);
  WriteDictionaryDefs(Context.Definitions);
  WriteInterfaceDefs(Context.Definitions);
  Undent;
  WriteIncludeInterfaceCode;
  Addln('');
  AddLn('implementation');
  WriteImplementation;
  AddLn('end.');
  Source.SaveToFile(OutputFileName);
end;

function TWebIDLToPas.BaseUnits: String;

begin
  Result:='SysUtils, JS'
end;

function TWebIDLToPas.CreatePasName(aName: String): TPasData;

begin
  Result:=TPasData.Create(EscapeKeyWord(aName));
  FPasNameList.Add(Result);
end;

function TWebIDLToPas.AllocatePasName(D: TIDLDefinition; ParentName: String): TPasData;

Var
  CN : String;

begin
  if D Is TIDLInterfaceDefinition then
    begin
    CN:=ClassPrefix+D.Name+ClassSuffix;
    Result:=CreatePasname(CN);
    D.Data:=Result;
    AllocatePasNames((D as TIDLInterfaceDefinition).members,D.Name);
    end
  else if D Is TIDLDictionaryDefinition then
    begin
    CN:=D.Name;
    if coDictionaryAsClass in Options then
      CN:=ClassPrefix+CN+ClassSuffix;
    Result:=CreatePasname(EscapeKeyWord(CN));
    D.Data:=Result;
    AllocatePasNames((D as TIDLDictionaryDefinition).members,D.Name);
    end
  else
    begin
    Result:=CreatePasName(D.Name);
    D.Data:=Result;
    if D Is TIDLFunctionDefinition then
      AllocatePasNames((D as TIDLFunctionDefinition).Arguments,D.Name);
    end;
  if Verbose and (TPasData(D.Data).PasName<>D.Name) then
    begin
    if (ParentName<>'') then
      ParentName:=ParentName+'.';
    DoLog('Renamed %s to %s',[ParentName+D.Name,TPasData(D.Data).PasName]);
    end;
end;

procedure TWebIDLToPas.SetTypeAliases(AValue: TStrings);
begin
  if FTypeAliases=AValue then Exit;
  FTypeAliases.Assign(AValue);
end;

procedure TWebIDLToPas.SetIncludeInterfaceCode(AValue: TStrings);
begin
  if FIncludeInterfaceCode=AValue then Exit;
  FIncludeInterfaceCode.Assign(AValue);
end;

procedure TWebIDLToPas.SetIncludeImplementationCode(AValue: TStrings);
begin
  if FIncludeImplementationCode=AValue then Exit;
  FIncludeImplementationCode.Assign(AValue);
end;

procedure TWebIDLToPas.AllocatePasNames(aList : TIDLDefinitionList; ParentName: String = '');

var
  D : TIDLDefinition;

begin
  For D in aList do
    AllocatePasName(D,ParentName);
end;


procedure TWebIDLToPas.ProcessDefinitions;

begin
  FContext.AppendPartials;
  FContext.AppendIncludes;
  AllocatePasNames(FContext.Definitions);
end;

procedure TWebIDLToPas.Execute;

begin
  FContext:=CreateContext;
  try
    FContext.Aliases:=Self.TypeAliases;
    Parse;
    if Verbose then
      DoLog('Parsed %d definitions.',[Context.Definitions.Count]);
    ProcessDefinitions;
    WritePascal;
  finally
    FreeAndNil(FContext);
  end;
end;

end.

