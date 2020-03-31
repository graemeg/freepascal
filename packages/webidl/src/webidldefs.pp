{
    This file is part of the Free Component Library

    WEBIDL definition containers
    Copyright (c) 2018 by Michael Van Canneyt michael@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit webidldefs;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, contnrs;

Type

  { TAttributeList }

  TAttributeList = Class(TPersistent)
  private
    Fattrs: TStringList;
    function GetAttrs(aIndex : Integer): UTF8String;
    function GetCount: Integer;
  Public
    Constructor Create;
    Destructor destroy; override;
    Procedure Assign(aSource : TPersistent); override;
    Procedure Add(aAttribute : UTF8String);
    Function ToLine (ASep : String): UTF8String;
    Function AsString (Full : Boolean = False): UTF8String;
    Function IndexOf(Const aName : UTF8string) : Integer;
    Property Attrs[aIndex : Integer] : UTF8String Read GetAttrs; default;
    Property Count : Integer Read GetCount;
  end;

  { TIDLDefinition }
  TIDLDefinition = Class;
  TIDLTypeDefDefinition = Class;
  TIDLDefinitionClass = Class of TIDLDefinition;

  TIDLBaseObject = Class
  Public
    // The IDLBaseobject is owner of the newly created instance !
    Function Add(aClass : TIDLDefinitionClass; Const AName : UTF8String) : TIDLDefinition; virtual; abstract;
    Function AsString(Full : Boolean): UTF8String; virtual; abstract;
  end;

  TIDLDefinition = Class(TIDLBaseObject)
  private
    FAttributes: TAttributeList;
    FData: TObject;
    FName: UTF8String;
    FParent: TIDLDefinition;
    function GetAttributes: TAttributeList;
    procedure SetAttributes(AValue: TAttributeList);
  Public
    Constructor Create(aParent : TIDLDefinition;Const aName : UTF8String); virtual;
    Function Add(aClass : TIDLDefinitionClass; Const AName : UTF8String) : TIDLDefinition; override;
    Destructor Destroy; override;
    // This definition extens an existing one. It will not be in the lookup list of symbols
    Function IsExtension : Boolean; virtual;
    Function AsString(Full : Boolean): UTF8String; override;
    Function HasAttributes : Boolean;
    Function HasSimpleAttribute(Const AName : UTF8String) : Boolean;
    Property Name : UTF8String Read FName Write FName;
    Property Data : TObject Read FData Write FData;
    Property Parent : TIDLDefinition Read FParent Write FParent;
    // Attributes are owned by the definition. If you set it, your list will be freed by the definition
    Property Attributes : TAttributeList Read GetAttributes Write SetAttributes;
  end;

  { TIDLDefinitionList }
  TIDLDefinitionList = Class;

  { TIDLDefinitionEnumerator }

  TIDLDefinitionEnumerator = class
  private
    FList: TIDLDefinitionList;
    FPosition: Integer;
  public
    constructor Create(AList: TIDLDefinitionList);
    function GetCurrent: TIDLDefinition;
    function MoveNext: Boolean;
    property Current: TIDLDefinition read GetCurrent;
  end;

  TIDLDefinitionList = Class (TIDLBaseObject)
  private
    FList : TFPObjectList;
    FParent: TIDLDefinition;
    function GetCount: Integer;
    function GetD(aIndex : Integer): TIDLDefinition;
    function GetOwnsDefinitions: Boolean;
  Public
    Constructor Create(AParent : TIDLDefinition; OwnsDefinitions : Boolean = True); virtual; overload;
    Destructor Destroy; override;
    Procedure Clear;
    function AsString(const aSep, aStart, aEnd, aIndent: String; aFull,
      AppendSep: Boolean): UTF8String;
    Function AsString(Full : Boolean): UTF8String; override;
    Function Add(aClass : TIDLDefinitionClass; Const AName : UTF8String) : TIDLDefinition; override;
    Function Add(aItem : TIDLDefinition) : Integer;
    function GetEnumerator: TIDLDefinitionEnumerator;
    Property Parent : TIDLDefinition Read FParent;
    Property Definitions[aIndex : Integer] : TIDLDefinition Read GetD;default;
    Property Count : Integer Read GetCount;
    Property OwnsDefinitions : Boolean Read GetOwnsDefinitions;
  end;

  { TIDLConstDefinition }
  TConstType = (ctFloat,ctInteger,ctBoolean,ctInfinity,ctNegInfinity,ctNan,ctNull,ctString,ctEmptyArray,ctEmptyObject);
  TIDLConstDefinition = Class(TIDLDefinition)
  private
    FConstType: TConstType;
    FNull: Boolean;
    FTypeName: UTF8String;
    FValue: UTF8String;
  Public
    Function AsString(Full : Boolean): UTF8String; override;
    Property TypeName : UTF8String Read FTypeName Write FTypeName;
    Property Value : UTF8String Read FValue Write FValue;
    Property AllowNull : Boolean Read FNull Write FNull;
    Property ConstType : TConstType Read FConstType Write FConstType;
  end;

  TAttributeOption = (aoStatic,aoInherit,aoReadOnly,aoStringifier);
  TAttributeOptions = set of TAttributeOption;

  { TIDLAttributeDefinition }

  TIDLAttributeDefinition = Class(TIDLDefinition)
  private
    FOptions : TAttributeOptions;
    FType: TIDLTypeDefDefinition;
    procedure SetType(AValue: TIDLTypeDefDefinition);
  Public
    Destructor Destroy; override;
    Function AsString(Full : Boolean): UTF8String; override;
    // Owned by definition
    Property AttributeType : TIDLTypeDefDefinition Read FType Write SetType;
    Property Options : TAttributeOptions Read FOptions Write FOptions;
  end;

  { TIDLStructuredDefinition }

  TIDLStructuredDefinition = Class(TIDLDefinition)
  Private
    FIsCallBack: Boolean;
    FPartials,
    FMembers: TIDLDefinitionList;
    FParentName: String;
    FPartial: Boolean;
    function GetMember(Aindex : Integer): TIDLDefinition;
    function GetMembers: TIDLDefinitionList;
    function GetPartial(Aindex : Integer): TIDLStructuredDefinition;
    function GetPartials: TIDLDefinitionList;
  Public
    Destructor Destroy; override;
    Function IsExtension : Boolean; override;
    Function GetFullMemberList(aList : TIDLDefinitionList) : Integer;
    Function HasMembers : Boolean;
    // Members are owned by this instance.
    Property members : TIDLDefinitionList Read GetMembers;
    Property Member[Aindex : Integer] : TIDLDefinition Read GetMember; default;
    Function HasPartials : Boolean;
    // Partials are NOT owned by this instance
    Property Partials : TIDLDefinitionList Read GetPartials;
    Property Partial[Aindex : Integer] : TIDLStructuredDefinition Read GetPartial;
    Property ParentName : String Read FParentName Write FParentName;
    // is this a partial definition?
    Property IsPartial : Boolean Read FPartial Write FPartial;
    // is this a callback definition?
    Property IsCallBack : Boolean Read FIsCallBack Write FIsCallBack;
  end;


  { TIDLInterfaceDefinition }

  TIDLInterfaceDefinition = Class(TIDLStructuredDefinition)
  private
    FHasSerializer: Boolean;
    FIsInclude: Boolean;
    FIsMixin: Boolean;
    FParentInterface: TIDLInterfaceDefinition;
  Public
    Function AsString (aFull : Boolean) : UTF8String; override;
    Property ParentInterface : TIDLInterfaceDefinition Read FParentInterface Write FParentInterface;
    Property HasSerializer : Boolean Read FHasSerializer Write FHasSerializer;
    // is this a mixin definition?
    Property IsMixin : Boolean Read FIsMixin Write FIsMixin;
    Property IsInclude : Boolean Read FIsInclude Write FIsInclude;
  end;

  { TIDLArgumentDefinition }

  TIDLArgumentDefinition = Class(TIDLDefinition)
  private
    FDefaultValue: String;
    FHasDefaultValue: Boolean;
    FHasEllipsis: Boolean;
    FIsOptional: Boolean;
    FType: TIDLTypeDefDefinition;
    procedure SetType(AValue: TIDLTypeDefDefinition);
  Public
    Destructor Destroy; override;
    Function Clone(aType : TIDLTypeDefDefinition) : TIDLArgumentDefinition; // need
    Function AsString(Full : Boolean) : UTF8String; override;
    // Owned by definition
    Property ArgumentType : TIDLTypeDefDefinition Read FType Write SetType;
    Property IsOptional : Boolean Read FIsOptional Write FIsOptional;
    Property HasDefaultValue : Boolean Read FHasDefaultValue Write FHasDefaultValue;
    Property HasEllipsis : Boolean Read FHasEllipsis Write FHasEllipsis;
    Property DefaultValue : String Read FDefaultValue Write FDefaultValue;
  end;

  { TIDLFunctionDefinition }
  TFunctionOption = (foCallBack,foStatic,foStringifier,foGetter, foSetter, foDeleter, foLegacyCaller, foConstructor);
  TFunctionOptions = Set of TFunctionOption;

  TIDLFunctionDefinition = Class(TIDLDefinition)
  private
    FOptions : TFunctionOptions;
    FReturnType: TIDLTypeDefDefinition;
    Farguments: TIDLDefinitionList;
    function GetA(AIndex : Integer): TIDLArgumentDefinition;
    function GetArguments: TIDLDefinitionList;
    procedure SetReturnType(AValue: TIDLTypeDefDefinition);
  Public
    Destructor Destroy; override;
    Function HasArguments : Boolean;
    Function AsString(Full : Boolean) : UTF8String; override;
    Property Arguments : TIDLDefinitionList Read GetArguments;
    Property Argument[AIndex : Integer] : TIDLArgumentDefinition Read GetA; default;
    // Owned by function definition.
    Property ReturnType : TIDLTypeDefDefinition Read FReturnType Write SetReturnType;
    // is this a callback function definition?
    Property Options : TFunctionOptions Read FOptions Write FOptions;
  end;

  TSerializerKind = (skObject,skArray,skSingle,skFunction);

  { TIDLSerializerDefinition }

  TIDLSerializerDefinition = Class(TIDLDefinition)
  private
    FKind: TSerializerKind;
    FIdentifiers : TAttributeList;
    FSerializerFunction: TIDLFunctionDefinition;
    procedure SetIdentifierList(AValue: TAttributeList);
    Procedure SetSerializerFunction (aValue : TIDLFunctionDefinition);
    function GetIdentifierList: TAttributeList;
  Public
    Destructor Destroy; override;
    Function HasIdentifiers : Boolean;
    Property Identifiers : TAttributeList Read GetIdentifierList Write SetIDentifierList;
    Property SerializerFunction : TIDLFunctionDefinition Read FSerializerFunction Write SetSerializerFunction;
    Property Kind : TSerializerKind Read FKind Write FKind;
  end;


  { TIDLDictionaryMemberDefinition }

  TIDLDictionaryMemberDefinition = Class(TIDLDefinition)
  private
    FDefaultValue: TIDLConstDefinition;
    FMemberType: TIDLTypeDefDefinition;
    FRequired: Boolean;
  Public
    Destructor Destroy; override;
    Function AsString(Full : Boolean) : UTF8String;  override;
    Property IsRequired : Boolean Read FRequired Write FRequired;
    // Owned  by memberdefinition
    Property MemberType : TIDLTypeDefDefinition Read FMemberType Write FMemberType;
    Property DefaultValue : TIDLConstDefinition Read FDefaultValue Write FDefaultValue;
  end;

  { TIDLDictionaryDefinition }

  TIDLDictionaryDefinition = Class(TIDLStructuredDefinition)
  private
    FParentDictionary: TIDLDictionaryDefinition;
    function GetDM(AIndex : Integer): TIDLDictionaryMemberDefinition;
  Public
    Function AsString(Full : Boolean) : UTF8String; override;
    Property ParentDictionary : TIDLDictionaryDefinition Read FParentDictionary Write FParentDictionary;
    Property DictionaryMembers[AIndex : Integer] : TIDLDictionaryMemberDefinition Read GetDM; default;
  end;

  { TIDLEnumDefinition }

  TIDLEnumDefinition = Class(TIDLDefinition)
  private
    FValues: TStrings;
  Public
    Constructor Create(aParent : TIDLDefinition;Const aName : UTF8String); override;
    Destructor Destroy; override;
    Procedure AddValue(Const aValue : String);
    Property Values : TStrings Read FValues;
  end;

  { TIDLTypeDefDefinition }

  TIDLTypeDefDefinition = Class(TIDLDefinition)
  private
    FNull: Boolean;
    FTypeName: String;
  Public
    Function Clone (aParent : TIDLDefinition) : TIDLTypeDefDefinition; virtual;
    Function AsString(Full: Boolean): UTF8String; override;
    Property TypeName : String Read FTypeName Write FTypeName;
    Property AllowNull : Boolean Read FNull Write FNull;
  end;
  TIDLTypeDefDefinitionClass = Class of TIDLTypeDefDefinition;

  { TIDLPromiseTypeDefDefinition }

  TIDLPromiseTypeDefDefinition = Class(TIDLTypeDefDefinition)
  private
    FReturnType: TIDLTypeDefDefinition;
    procedure SetReturnType(AValue: TIDLTypeDefDefinition);
  Public
    Destructor Destroy; override;
    Function Clone (aParent : TIDLDefinition) : TIDLTypeDefDefinition; override;
    Function AsString(Full: Boolean): UTF8String; override;
    property ReturnType : TIDLTypeDefDefinition Read FReturnType Write SetReturnType;
  end;

  { TIDLMapLikeDefinition }

  { TIDLKeyValueDefinition }

  TIDLKeyValueDefinition = Class(TIDLTypeDefDefinition)
  private
    FKeyType: TIDLTypeDefDefinition;
    FValueType: TIDLTypeDefDefinition;
    procedure SetKeyType(AValue: TIDLTypeDefDefinition);
    procedure SetValueType(AValue: TIDLTypeDefDefinition);
  Public
    Destructor Destroy; override;
    Function Clone (aParent : TIDLDefinition) : TIDLTypeDefDefinition; override;
    property KeyType : TIDLTypeDefDefinition Read FKeyType Write SetKeyType;
    property ValueType : TIDLTypeDefDefinition Read FValueType Write SetValueType;
  end;

  TIDLMapLikeDefinition = Class(TIDLKeyValueDefinition)
  private
    FIsReadonly: Boolean;
  Public
    Function Clone (aParent : TIDLDefinition) : TIDLTypeDefDefinition; override;
    Function AsString(Full: Boolean): UTF8String; override;
    property IsReadonly: Boolean Read FIsReadonly Write FIsReadonly;
  end;

  { TIDLRecordDefinition }

  TIDLRecordDefinition = Class(TIDLKeyValueDefinition)
  Public
    Function AsString(Full: Boolean): UTF8String; override;
  end;

  { TIDLSequenceTypeDefDefinition }

  TIDLSequenceTypeDefDefinition = Class(TIDLTypeDefDefinition)
  private
    FElementType: TIDLTypeDefDefinition;
    procedure SetElementType(AValue: TIDLTypeDefDefinition);
  Public
    Function AsString(Full: Boolean): UTF8String; override;
    Function Clone (aParent : TIDLDefinition) : TIDLTypeDefDefinition; override;
    Destructor Destroy; override;
    property ElementType : TIDLTypeDefDefinition Read FElementType Write SetElementType;
  end;

  { TIDLSetlikeDefinition }

  TIDLSetlikeDefinition = Class(TIDLDefinition)
  private
    FElementType: TIDLTypeDefDefinition;
    FIsReadonly: Boolean;
    procedure SetElementType(AValue: TIDLTypeDefDefinition);
  Public
    Function AsString(Full: Boolean): UTF8String; override;
    Destructor Destroy; override;
    property ElementType : TIDLTypeDefDefinition Read FElementType Write SetElementType;
    property IsReadonly: Boolean Read FIsReadonly Write FIsReadonly;
  end;

  { TIDLUnionTypeDefDefinition }

  TIDLUnionTypeDefDefinition = Class(TIDLTypeDefDefinition)
  private
    FUnion: TIDLDefinitionList;
  Public
    Constructor Create(aParent : TIDLDefinition;Const aName : UTF8String); override;
    Destructor Destroy; override;
    Function Clone (aParent : TIDLDefinition) : TIDLTypeDefDefinition; override;
    Function AsString(Full: Boolean): UTF8String; override;
    property Union : TIDLDefinitionList Read FUnion;
  end;

  { TIDLImplementsDefinition }
  TIDLImplementsOrIncludesDefinition = Class(TIDLDefinition);

  TIDLImplementsDefinition = Class(TIDLImplementsOrIncludesDefinition)
  private
    FImplementedInterface: UTF8String;
  Public
    Function IsExtension : Boolean; override;
    Function AsString(Full: Boolean): UTF8String; override;
    Property ImplementedInterface  : UTF8String Read FImplementedInterface Write FImplementedInterface;
  end;

  { TIDLIncludesDefinition }

  TIDLIncludesDefinition = Class(TIDLImplementsOrIncludesDefinition)
  private
    FIncludedInterface : UTF8String;
  Public
    Function IsExtension : Boolean; override;
    Function AsString(Full: Boolean): UTF8String; override;
    Property IncludedInterface  : UTF8String Read FIncludedInterface Write FIncludedInterface;
  end;

  { TIDLIterableDefinition }

  TIDLIterableDefinition = Class(TIDLDefinition)
  private
    FValueType: TIDLTypeDefDefinition;
    FKeyType: TIDLTypeDefDefinition;
    procedure SetKeyType(AValue: TIDLTypeDefDefinition);
    procedure SetValueType(AValue: TIDLTypeDefDefinition);
  Public
    Destructor Destroy; override;
    property ValueType : TIDLTypeDefDefinition Read FValueType Write SetValueType;
    property KeyType : TIDLTypeDefDefinition Read FKeyType Write SetKeyType;
  end;


implementation

{ TIDLSetlikeDefinition }

procedure TIDLSetlikeDefinition.SetElementType(
  AValue: TIDLTypeDefDefinition);
begin
  if FElementType=AValue then Exit;
  FreeAndNil(FElementType);
  FElementType:=AValue;
  if Assigned(FElementType) then
    FElementType.Parent:=Self
end;

function TIDLSetlikeDefinition.AsString(Full: Boolean): UTF8String;
begin
  Result:='setlike <'+ElementType.TypeName+'>';
  if IsReadonly then
    Result:='readonly '+Result;
  if full and HasAttributes then
    Result:=Attributes.AsString(True)+' '+Result;
end;

destructor TIDLSetlikeDefinition.Destroy;
begin
  FreeAndNil(FElementType);
  inherited Destroy;
end;

{ TIDLRecordDefinition }

function TIDLRecordDefinition.AsString(Full: Boolean): UTF8String;
begin
  Result:='record <'+KeyType.AsString(False)+','+ValueType.AsString(False)+'>';
  if AllowNull then
    Result:=Result+'?';
  if Full then
      begin
      Result:='typedef '+Result;
      if HasAttributes then
        Result:=Attributes.AsString(True)+' '+Result;
      end;
end;

{ TIDLMapLikeDefinition }

procedure TIDLKeyValueDefinition.SetKeyType(AValue: TIDLTypeDefDefinition);
begin
  if FKeyType=AValue then Exit;
  FreeAndNil(FKeyType);
  FKeyType:=AValue;
  if Assigned(FKeyType) then
    FKeyType.Parent:=Self
end;

procedure TIDLKeyValueDefinition.SetValueType(AValue: TIDLTypeDefDefinition);
begin
  if FValueType=AValue then Exit;
  FreeAndNil(FValueType);
  FValueType:=AValue;
  if Assigned(FValueType) then
    FValueType.Parent:=Self
end;


destructor TIDLKeyValueDefinition.Destroy;
begin
  FreeAndNil(FKeyType);
  FreeAndNil(FValueType);
  inherited Destroy;
end;

function TIDLKeyValueDefinition.Clone(aParent: TIDLDefinition
  ): TIDLTypeDefDefinition;
begin
  Result:=inherited Clone(aParent);
  if Assigned(KeyType) then
    TIDLKeyValueDefinition(Result).KeyType:=KeyType.Clone(Result);
  if Assigned(ValueType) then
    TIDLKeyValueDefinition(Result).ValueType:=ValueType.Clone(Result);
end;

function TIDLMapLikeDefinition.Clone(aParent: TIDLDefinition
  ): TIDLTypeDefDefinition;
begin
  Result:=inherited Clone(aParent);
  TIDLMapLikeDefinition(Result).IsReadonly:=IsReadonly;
end;

function TIDLMapLikeDefinition.AsString(Full: Boolean): UTF8String;
begin
  Result:='maplike <'+KeyType.AsString(False)+','+ValueType.AsString(False)+'>';
  if IsReadonly then
    Result:='readonly '+Result;
  if Full and HasAttributes then
      Result:=Attributes.AsString(True)+' '+Result;
end;

{ TIDLIncludesDefinition }

function TIDLIncludesDefinition.IsExtension: Boolean;
begin
  Result:=True;
end;

function TIDLIncludesDefinition.AsString(Full: Boolean): UTF8String;

begin
  Result:=Name+' includes '+IncludedInterface;
  If Full and HasAttributes then
    Result:=Attributes.AsString(True)+' '+Result;
end;

{ TIDLDefinitionEnumerator }

constructor TIDLDefinitionEnumerator.Create(AList: TIDLDefinitionList);
begin
  FList:=AList;
  FPosition:=-1;
end;

function TIDLDefinitionEnumerator.GetCurrent: TIDLDefinition;
begin
  Result := FList[FPosition];
end;

function TIDLDefinitionEnumerator.MoveNext: Boolean;
begin
  Inc(FPosition);
  Result := FPosition < FList.Count;
end;

{ TIDLInterfaceDefinition }

function TIDLInterfaceDefinition.AsString(aFull: Boolean): UTF8String;
begin
  Result:='interface '+Name;
  if IsPartial then
    Result:='partial '+Result
  else if IsCallBack then
    Result:='callback '+Result;
  if ParentName<>'' then
    Result:=Result+' : '+ParentName;
  if Not HasMembers then
    Result:=Result+' {'+sLineBreak+'}'
  else
    Result:=Result+' '+members.AsString(true);

  if aFull and HasAttributes then
    Result:=Attributes.AsString(true)+' '+Result;
end;

{ TIDLTypeDefDefinition }

function TIDLTypeDefDefinition.Clone(aParent: TIDLDefinition): TIDLTypeDefDefinition;
begin
  Result:=TIDLTypeDefDefinitionClass(Self.ClassType).Create(aParent,Name);
  Result.TypeName:=Self.TypeName;
end;

function TIDLTypeDefDefinition.AsString(Full: Boolean): UTF8String;
begin
  Result:=TypeName;
  if AllowNull then
    Result:=Result+'?';
  If Full then
    begin
    Result:='typedef '+Result+' '+Name;
    if HasAttributes then
      Result:=Attributes.AsString(True)+' '+Result;
    end;
end;

{ TIDLImplementsDefinition }

function TIDLImplementsDefinition.IsExtension: Boolean;
begin
  Result:=True;
end;

function TIDLImplementsDefinition.AsString(Full: Boolean): UTF8String;
begin
  Result:=Name+' implements '+ImplementedInterface;
  If Full and HasAttributes then
    Result:=Attributes.AsString(True)+' '+Result;
end;

{ TIDLConstDefinition }

function TIDLConstDefinition.AsString(Full: Boolean): UTF8String;
begin
  Result:=TypeName+' '+Name+' '+Value;
  If Full then
    begin
    Result:='const '+Result;
    if HasAttributes then
      Result:=Attributes.AsString(True)+' '+Result;
    end;
end;

{ TIDLSerializerDefinition }

procedure TIDLSerializerDefinition.SetSerializerFunction(aValue: TIDLFunctionDefinition);
begin
  if (FSerializerFunction=aValue) then exit;
  FreeAndNil(FSerializerFunction);
  FSerializerFunction:=aValue;
  if AValue<>Nil then
    Kind:=skFunction;
end;

procedure TIDLSerializerDefinition.SetIdentifierList(AValue: TAttributeList);
begin
  If (FIdentifiers=AValue) then
    exit;
  FreeAndNil(FAttributes);
  FAttributes:=AValue;
end;

function TIDLSerializerDefinition.GetIdentifierList: TAttributeList;

begin
  If FIdentifiers=Nil then
    FIdentifiers:=TAttributeList.Create;
  Result:=FIdentifiers;
end;

destructor TIDLSerializerDefinition.Destroy;
begin
  FreeAndNil(FIdentifiers);
  FreeAndNil(FSerializerFunction);
  inherited Destroy;
end;

function TIDLSerializerDefinition.HasIdentifiers: Boolean;

begin
  Result:=Assigned(FIdentifiers) and (FIdentifiers.Count>0);
end;

{ TIDLIterableDefinition }

procedure TIDLIterableDefinition.SetKeyType(AValue: TIDLTypeDefDefinition);
begin
  if (AValue=FKeyType) then exit;
  FreeAndNil(FKeyType);
  FKeyType:=AValue;
end;

procedure TIDLIterableDefinition.SetValueType(AValue: TIDLTypeDefDefinition);
begin
  if (AValue=FValueType) then exit;
  FreeAndNil(FValueType);
  FValueType:=AValue;
end;

destructor TIDLIterableDefinition.Destroy;
begin
  ValueType:=Nil;
  KeyType:=Nil;
  inherited Destroy;
end;

{ TIDLAttributeDefinition }

procedure TIDLAttributeDefinition.SetType(AValue: TIDLTypeDefDefinition);
begin
  If (AValue=FType) then exit;
  FreeAndNil(FType);
  FType:=AValue;
end;

destructor TIDLAttributeDefinition.Destroy;
begin
  AttributeType:=Nil;
  inherited Destroy;
end;

function TIDLAttributeDefinition.AsString(Full: Boolean): UTF8String;
begin
  Result:=Name;
  if Assigned(AttributeType) then
    Result:=AttributeType.TypeName+' '+Result;
  if Full then
    begin
    Result:='attribute '+Result;
    if (aoReadonly in Options) then
      Result:='readonly '+Result;
    if (aoInherit in Options) then
      Result:='inherit '+Result;
    if (aoStatic in Options) then
      Result:='static '+Result;
    if (aoStringifier in Options) then
      Result:='stringifier '+Result;
    if HasAttributes then
      Result:=Attributes.AsString(True)+' '+Result;
    end;
end;

{ TIDLArgumentDefinition }

procedure TIDLArgumentDefinition.SetType(AValue: TIDLTypeDefDefinition);
begin
  if FType=AValue then Exit;
  FreeAndNil(FType);
  FType:=AValue;
end;

destructor TIDLArgumentDefinition.Destroy;
begin
  ArgumentType:=Nil;
  inherited Destroy;
end;

function TIDLArgumentDefinition.Clone(aType: TIDLTypeDefDefinition): TIDLArgumentDefinition;

begin
  Result:=TIDLArgumentDefinition.Create(Nil,Self.Name);
  if (AType=Nil) and Assigned(ArgumentType) then
    begin
    AType:=ArgumentType.Clone(Result);
    AType.TypeName:=Self.ArgumentType.TypeName;
    end;
  Result.ArgumentType:=aType;
end;

function TIDLArgumentDefinition.AsString(Full: Boolean): UTF8String;
begin
  Result:=ArgumentType.AsString(False)+' '+Name;
  If IsOptional then
    Result:='optional '+Result;
  if HasDefaultValue then
    if DefaultValue='' then
      Result:=Result+' = ""'
    else
      Result:=Result+' = '+DefaultValue;
  if Full and HasAttributes then
    Result:=Attributes.AsString(True)+' '+Result;
end;

{ TIDLFunctionDefinition }

procedure TIDLFunctionDefinition.SetReturnType(AValue: TIDLTypeDefDefinition);
begin
  if FReturnType=AValue then Exit;
  FreeAndNil(FReturnType);
  FReturnType:=AValue;
end;

function TIDLFunctionDefinition.GetArguments: TIDLDefinitionList;
begin
  if FArguments=nil then
    Farguments:=TIDLDefinitionList.Create(Self);
  Result:=Farguments;
end;

function TIDLFunctionDefinition.GetA(AIndex : Integer): TIDLArgumentDefinition;
begin
  Result:=Arguments[AIndex] as TIDLArgumentDefinition;
end;

destructor TIDLFunctionDefinition.Destroy;
begin
  FreeAndNil(FArguments);
  ReturnType:=nil;
  inherited Destroy;
end;

function TIDLFunctionDefinition.HasArguments: Boolean;
begin
  Result:=Assigned(FArguments) and (FArguments.Count>0);
end;

function TIDLFunctionDefinition.AsString(Full: Boolean): UTF8String;

  Procedure MaybeAdd(O : TFunctionOption; S : String);

  begin
    if O in options then
      Result:=S+' '+Result;
  end;

begin
  if foCallBack in Options then
    Result:=Name+' = '+ReturnType.AsString(False)
  else
    Result:=ReturnType.AsString(False)+' '+Name;
  Result:=Result+' '+Arguments.AsString(', ','(',')','',True,False);
  MaybeAdd(foCallback,'callback');
  MaybeAdd(foStatic,'static');
  MaybeAdd(foGetter,'getter');
  MaybeAdd(foSetter,'setter');
  MaybeAdd(foDeleter,'deleter');
  MaybeAdd(foStringifier,'stringifier');
  if Full and HasAttributes then
    Result:=Attributes.AsString(Full)+' '+Result;
end;

{ TIDLDictionaryDefinition }

function TIDLDictionaryDefinition.GetDM(AIndex : Integer
  ): TIDLDictionaryMemberDefinition;
begin
  Result:=Members[aIndex] as TIDLDictionaryMemberDefinition;
end;

function TIDLDictionaryDefinition.AsString(Full: Boolean): UTF8String;
begin
  Result:='dictionary '+Name;
  if (ParentName<>'') then
    Result:=Result+' : '+ParentName;
  if Not HasMembers then
    Result:=Result+' {'+sLineBreak+'}'
  else
    Result:=Result+' '+members.AsString(true);
  if Full and HasAttributes then
    Result:=Attributes.AsString(True)+' '+Result
end;

{ TIDLDictionaryMemberDefinition }

destructor TIDLDictionaryMemberDefinition.Destroy;
begin
  FreeAndNil(FMemberType);
  FreeAndNil(FDefaultValue);
  inherited Destroy;
end;

function TIDLDictionaryMemberDefinition.AsString(Full: Boolean): UTF8String;
begin
  Result:=MemberType.AsString(False)+' '+Name;
  if IsRequired then
    Result:='required '+Result;
  If DefaultValue<>Nil then
    Result:=Result+' = '+DefaultValue.Value;
  if Full and HasAttributes then
    Result:=Attributes.AsString(True)+' '+Result;
end;

{ TIDLSequenceTypeDefDefinition }

procedure TIDLSequenceTypeDefDefinition.SetElementType(
  AValue: TIDLTypeDefDefinition);
begin
  if FElementType=AValue then Exit;
  FreeAndNil(FElementType);
  FElementType:=AValue;
  if Assigned(FElementType) then
    FElementType.Parent:=Self
end;

function TIDLSequenceTypeDefDefinition.AsString(Full: Boolean): UTF8String;
begin
  if Full then
    Result:='typedef sequence <'+ElementType.TypeName+'> '+Name
  else
    Result:='sequence <'+ElementType.TypeName+'>';
  if full and HasAttributes then
    Result:=Attributes.AsString(True)+' '+Result;
end;

function TIDLSequenceTypeDefDefinition.Clone(aParent: TIDLDefinition
  ): TIDLTypeDefDefinition;
begin
  Result:=inherited Clone(aParent);
  if Assigned(ElementType) then
    TIDLSequenceTypeDefDefinition(Result).ElementType:=ElementType.Clone(Result);
end;

destructor TIDLSequenceTypeDefDefinition.Destroy;
begin
  ElementType:=nil;
  inherited Destroy;
end;

{ TIDLPromiseTypeDefDefinition }

procedure TIDLPromiseTypeDefDefinition.SetReturnType(AValue: TIDLTypeDefDefinition);
begin
  if FReturnType=AValue then Exit;
  FreeAndNil(FReturnType);
  FReturnType:=AValue;
end;

destructor TIDLPromiseTypeDefDefinition.Destroy;
begin
  ReturnType:=Nil;
  inherited Destroy;
end;

function TIDLPromiseTypeDefDefinition.Clone(aParent: TIDLDefinition
  ): TIDLTypeDefDefinition;
begin
  Result:=inherited Clone(aParent);
  if Assigned(ReturnType) then
    TIDLPromiseTypeDefDefinition(Result).ReturnType:=ReturnType.Clone(Result);
end;

function TIDLPromiseTypeDefDefinition.AsString(Full: Boolean): UTF8String;
begin
  Result:='promise <'+ReturnType.AsString(False)+'>';
  if AllowNull then
    Result:=Result+'?';
  if Full then
    begin
    Result:='typedef '+Result+' '+Name;
    If HasAttributes then
      Result:=Attributes.AsString(True)+' '+Result;
    end;
end;

{ TIDLUnionTypeDefDefinition }

constructor TIDLUnionTypeDefDefinition.Create(aParent: TIDLDefinition;
  const aName: UTF8String);
begin
  inherited Create(aParent, aName);
  FUnion:=TIDLDefinitionList.Create(Self,True);
end;

destructor TIDLUnionTypeDefDefinition.Destroy;
begin
  FreeAndNil(FUnion);
  inherited Destroy;
end;

function TIDLUnionTypeDefDefinition.Clone(aParent: TIDLDefinition
  ): TIDLTypeDefDefinition;

Var
  D : TIDLDefinition;

begin
  Result:=inherited Clone(aParent);
  For D in Union do
    if D is TIDLTypeDefDefinition then
      TIDLUnionTypeDefDefinition(Result).Union.Add(TIDLTypeDefDefinition(D).Clone(Result));
end;

function TIDLUnionTypeDefDefinition.AsString(Full: Boolean): UTF8String;
begin
  Result:=Union.AsString(' or ','(',')','',false,False);
  if Full  then
    begin
    Result:='typedef '+Result+' '+Name;
    if HasAttributes then
      Result:=Attributes.AsString(True)+' '+Result;
    end;
end;

{ TIDLEnumDefinition }

constructor TIDLEnumDefinition.Create(aParent: TIDLDefinition;
  const aName: UTF8String);
begin
  inherited Create(aParent, aName);
  FValues:=TStringList.Create;
end;

destructor TIDLEnumDefinition.Destroy;
begin
  FreeAndNil(FValues);
  inherited Destroy;
end;

procedure TIDLEnumDefinition.AddValue(const aValue: String);
begin
  FValues.Add(aValue);
end;

{ TIDLInterfaceDefinition }

function TIDLStructuredDefinition.GetMembers: TIDLDefinitionList;
begin
  if Not Assigned(FMembers) then
    FMembers:=TIDLDefinitionList.Create(Self);
  Result:=FMembers;
end;

function TIDLStructuredDefinition.GetMember(Aindex : Integer): TIDLDefinition;
begin
  Result:=Members[AIndex] as TIDLDefinition;
end;

function TIDLStructuredDefinition.GetPartial(Aindex : Integer
  ): TIDLStructuredDefinition;
begin
  Result:=Partials[AIndex] as TIDLStructuredDefinition;
end;

function TIDLStructuredDefinition.GetPartials: TIDLDefinitionList;
begin
  if Not Assigned(FPartials) then
    FPartials:=TIDLDefinitionList.Create(Self,False);
  Result:=FPartials;
end;

destructor TIDLStructuredDefinition.Destroy;
begin
  FreeAndNil(FMembers);
  FreeAndNil(FPartials);
  inherited Destroy;
end;

function TIDLStructuredDefinition.IsExtension: Boolean;
begin
  Result:=IsPartial;
end;

function TIDLStructuredDefinition.GetFullMemberList(aList: TIDLDefinitionList): Integer;

Var
  D : TIDLDefinition;

begin
  Result:=aList.Count;
  For D in Members do
    aList.Add(D);
  Result:=aList.Count-Result;
  For D in Partials do
    Result:=Result+(D as TIDLStructuredDefinition).GetFullMemberList(aList);
end;

function TIDLStructuredDefinition.HasMembers: Boolean;
begin
  Result:=Assigned(FMembers) and (FMembers.Count>0)
end;

function TIDLStructuredDefinition.HasPartials: Boolean;
begin
  Result:=Assigned(FPartials) and (FPartials.Count>0);
end;

{ TAttributeList }

function TAttributeList.GetAttrs(aIndex : Integer): UTF8String;
begin
  Result:=FAttrs[aIndex];
end;

function TAttributeList.GetCount: Integer;
begin
  Result:=FAttrs.Count;
end;

constructor TAttributeList.Create;
begin
  Fattrs:=TStringList.Create;
end;

destructor TAttributeList.destroy;
begin
  FreeAndNil(Fattrs);
  inherited;
end;

procedure TAttributeList.Assign(aSource: TPersistent);
begin
  If aSource is TAttributeList then
    Fattrs.Assign(TAttributeList(aSource).Fattrs);
end;

procedure TAttributeList.Add(aAttribute: UTF8String);
begin
  FAttrs.Add(aAttribute);
end;

function TAttributeList.ToLine(ASep: String): UTF8String;

Var
  I : Integer;

begin
  Result:='';
  For I:=0 to Count-1 do
    begin
    if (Result<>'') then
      Result:=Result+ASep;
    Result:=Result+Attrs[I];
    end;
end;

function TAttributeList.AsString(Full: Boolean): UTF8String;

begin
  Result:=ToLine(',');
  if Full and (Result<>'') then
    Result:='['+Result+']';
end;

function TAttributeList.IndexOf(const aName: UTF8string): Integer;
begin
  Result:=Fattrs.IndexOf(aName);
end;

{ TIDLDefinitionList }

function TIDLDefinitionList.GetD(aIndex : Integer): TIDLDefinition;
begin
  Result:= FList[aIndex] as TIDLDefinition;
end;

function TIDLDefinitionList.GetOwnsDefinitions: Boolean;
begin
  Result:=FList.OwnsObjects;
end;

function TIDLDefinitionList.GetCount: Integer;
begin
  Result:=FList.Count;
end;

constructor TIDLDefinitionList.Create(AParent: TIDLDefinition; OwnsDefinitions : Boolean = True);
begin
  FParent:=AParent;
  FList:=TFPObjectList.Create(OwnsDefinitions);
end;

destructor TIDLDefinitionList.Destroy;
begin
  FreeAndNil(FList);
  inherited Destroy;
end;

procedure TIDLDefinitionList.Clear;
begin
  FList.Clear;
end;

function TIDLDefinitionList.AsString(const aSep, aStart, aEnd, aIndent : String; aFull,AppendSep : Boolean): UTF8String;

Var
  I : Integer;

begin
  Result:=aStart;
  if Count>0 then
    begin
    Result:=Result+aIndent+Definitions[0].AsString(aFull);
    For I:=1 to Count-1 do
      begin
      Result:=Result+aSep;
      Result:=Result+aIndent+Definitions[I].AsString(aFull);
      end;
    If AppendSep and (Count>0) then
      Result:=Result+aSep;
    end;
  Result:=Result+aEnd;
end;

function TIDLDefinitionList.AsString(Full: Boolean): UTF8String;

begin
  if Full then
    Result:=AsString(';'+sLineBreak,'{'+sLineBreak,'}','  ',Full,True)
  else
    Result:=AsString(';'+sLineBreak,'','','',Full,true);
end;

function TIDLDefinitionList.Add(aClass: TIDLDefinitionClass;
  const AName: UTF8String): TIDLDefinition;
begin
  Result:=aClass.Create(FParent,aName);
  FList.Add(Result);
end;

function TIDLDefinitionList.Add(aItem: TIDLDefinition): Integer;
begin
  Result:=FList.Add(aItem);
  if (FParent<>nil) then
    aItem.Parent:=FParent;
end;

function TIDLDefinitionList.GetEnumerator: TIDLDefinitionEnumerator;
begin
  Result:=TIDLDefinitionEnumerator.Create(Self);
end;

{ TIDLDefinition }

procedure TIDLDefinition.SetAttributes(AValue: TAttributeList);
begin
  if FAttributes=AValue then Exit;
  FreeAndNil(FAttributes);
  FAttributes:=AValue;
end;

function TIDLDefinition.GetAttributes: TAttributeList;
begin
  if FAttributes=Nil then
    Fattributes:=TAttributeList.Create;
  Result:=Fattributes;
end;

constructor TIDLDefinition.Create(aParent : TIDLDefinition; const aName: UTF8String);
begin
  FName:=AName;
  FParent:=AParent;
end;

function TIDLDefinition.Add(aClass: TIDLDefinitionClass; const AName: UTF8String
  ): TIDLDefinition;
begin
  Result:=aClass.Create(Self,AName);
end;

destructor TIDLDefinition.Destroy;
begin
  FreeAndNil(FAttributes);
  Inherited;
end;

function TIDLDefinition.IsExtension: Boolean;
begin
  Result:=False;
end;

function TIDLDefinition.AsString(Full: Boolean): UTF8String;
begin
  Result:=Name;
  if Full and HasAttributes then
    Result:=Attributes.AsString(True)+' '+Result;
end;

function TIDLDefinition.HasAttributes: Boolean;
begin
  Result:=Assigned(FAttributes) and (FAttributes.Count>0)
end;

function TIDLDefinition.HasSimpleAttribute(const AName : UTF8String): Boolean;
begin
  Result:=HasAttributes and (FAttributes.IndexOf(aName)<>-1);
end;

end.

