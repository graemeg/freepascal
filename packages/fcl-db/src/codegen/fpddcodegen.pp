{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by Michael Van Canneyt, member of the
    Free Pascal development team

    Data Dictionary Code Generator Implementation.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpddcodegen;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, DB, fpDataDict;
  
Type
  TPropType = (ptAuto,
               ptBoolean,
               ptShortint, ptByte,
               ptSmallInt, ptWord,
               ptLongint, ptCardinal,
               ptInt64, ptQWord,
               ptShortString, ptAnsiString, ptWideString,
               ptSingle, ptDouble, ptExtended, ptComp, ptCurrency,
               ptDateTime,
               ptEnumerated, ptSet, ptStream, ptTStrings,
               ptCustom);
               
  TVisibility = (vPrivate,vProtected,vPublic,vPublished);
  TVisibilities = Set of TVisibility;
  TPropAccess = (paReadWrite,paReadonly,paWriteonly);
<<<<<<< HEAD
<<<<<<< HEAD
  TPropSetter = (psRead,psWrite);
  TPropSetters = set of TPropSetter;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


  TFieldPropDefs = Class;

  { TFieldPropDef }

  TFieldPropDef = Class (TCollectionItem)
  private
    FEnabled: Boolean;
    FFieldName: String;
    FFieldType: TFieldType;
    FPropAccess: TPropAccess;
    FPropDef: String;
<<<<<<< HEAD
<<<<<<< HEAD
    FPropSetters: TPropSetters;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FPropType : TPropType;
    FPRopSize: Integer;
    FPropName : String;
    FPropVis: TVisibility;
    function GetPropName: String;
    function GetPropType: TPropType;
    function GetPropTypeStored: boolean;
    procedure SetPropName(const AValue: String);
  Protected
    Procedure InitFromField(F : TField); virtual;
    Procedure InitFromDDFieldDef(F : TDDFieldDef);virtual;
  Public
    Constructor Create(ACollection : TCollection) ; override;
    Procedure Assign(ASource : TPersistent); override;
    Function FieldPropDefs : TFieldPropDefs;
<<<<<<< HEAD
<<<<<<< HEAD
    Function HasGetter : Boolean; Virtual; // Checks Propsetters for psRead
    Function HasSetter : Boolean; Virtual; // True for streams/strings or if Propsetters has pswrite
=======
    Function HasGetter : Boolean; Virtual; // Always false.
    Function HasSetter : Boolean; Virtual; // True for streams/strings
>>>>>>> graemeg/fixes_2_2
=======
    Function HasGetter : Boolean; Virtual; // Always false.
    Function HasSetter : Boolean; Virtual; // True for streams/strings
>>>>>>> origin/fixes_2_2
    Function ObjPasTypeDef : String; virtual; // Object pascal definition of type
    Function ObjPasReadDef : String; virtual; // Object pascal definition of getter
    Function ObjPasWriteDef : String; virtual; // Object pascal definition of setter
  Published
    Property Enabled : Boolean Read FEnabled Write FEnabled;
    Property FieldName : String Read FFieldName Write FFieldName;
    Property FieldType : TFieldType Read FFieldType Write FFieldType;
    Property PropertyName : String Read GetPropName Write SetPropName;
    Property PropertyType : TPropType Read GetPropType Write FPropType Stored GetPropTypeStored;
    Property PropertySize : Integer Read FPRopSize Write FPropSize;
    Property PropertyDef : String Read FPropDef Write FPropDef;
    Property PropertyVisibility : TVisibility Read FPropVis Write FPropVis;
    Property PropertyAccess : TPropAccess Read FPropAccess Write FPropAccess;
<<<<<<< HEAD
<<<<<<< HEAD
    Property PropSetters : TPropSetters Read FPropSetters Write FPropSetters;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
  
  { TFieldPropDefs }

  TFieldPropDefs = Class (TCollection)
  private
    function GetPropDef(Index : integer): TFieldPropDef;
    procedure SetPropDef(Index : integer; const AValue: TFieldPropDef);
  Public
    Function AddDef(AName : String) : TFieldPropDef;
    Procedure FromDataset(Dataset : TDataset; DoClear : Boolean = True);
    Procedure FromDDFieldDefs(Defs : TDDFieldDefs; DoClear : Boolean = True);
    Function IndexOfPropName(AName : String) : Integer;
    Function IndexOfFieldName(AName : String) : Integer;
    Function FindPropName(AName : String) : TFieldPropDef;
    Function FindFieldName(AName : String) : TFieldPropDef;
    Property PropDefs[Index : integer] : TFieldPropDef Read GetPropDef write SetPropDef; Default;
  end;

  { TFieldPropDefs }

  TCodeOption = (coInterface,coImplementation,coUnit);
  TCodeOptions = Set of TCodeOption;

  { TCodeGeneratorOptions }

  TCodeGeneratorOptions = Class(TPersistent)
  private
    FImplementationUnits: String;
    FInterfaceUnits: String;
    FOptions: TCodeOptions;
    FUnitName: String;
<<<<<<< HEAD
<<<<<<< HEAD
    FExtraSetterLine : string;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure SetImplementationUnits(const AValue: String);
    procedure SetInterfaceUnits(const AValue: String);
    procedure SetUnitname(const AValue: String);
  Protected
    procedure SetOPtions(const AValue: TCodeOptions); virtual;
  Public
    Constructor create; virtual;
    Procedure Assign(ASource : TPersistent); override;
  Published
<<<<<<< HEAD
<<<<<<< HEAD
    // Line of code that will be added to each property setter. Use %PROPNAME% to include property name in the line.
    Property ExtraSetterLine : String Read FExtraSetterLine Write FExtraSetterLine;
    // options
    Property Options : TCodeOptions Read FOptions Write SetOPtions;
    // Name of unit if a unit is generated.
    Property UnitName : String Read FUnitName Write SetUnitname;
    // Comma-separated list of  units that will be put in the interface units clause
    Property InterfaceUnits : String Read FInterfaceUnits Write SetInterfaceUnits;
    //  Comma-separated list of  units that will be put in the implementation units clause
=======
    Property Options : TCodeOptions Read FOptions Write SetOPtions;
    Property UnitName : String Read FUnitName Write SetUnitname;
    Property InterfaceUnits : String Read FInterfaceUnits Write SetInterfaceUnits;
>>>>>>> graemeg/fixes_2_2
=======
    Property Options : TCodeOptions Read FOptions Write SetOPtions;
    Property UnitName : String Read FUnitName Write SetUnitname;
    Property InterfaceUnits : String Read FInterfaceUnits Write SetInterfaceUnits;
>>>>>>> origin/fixes_2_2
    Property ImplementationUnits : String Read FImplementationUnits Write SetImplementationUnits;
  end;
  TCodeGeneratorOptionsClass = Class of TCodeGeneratorOptions;

  { TDDCustomCodeGenerator }

  TDDCustomCodeGenerator = Class(TComponent)
    FCodeOptions: TCodeGeneratorOptions;
  Private
    FIndent: Integer;
    FCurrentIndent :String;
  Protected
    // Utility routines to add lines to the code. Will prepend indent.
    procedure AddLn(Strings: TStrings); overload;
    procedure AddLn(Strings: TStrings; Line: String); overload;
    procedure AddLn(Strings: TStrings; Fmt: String; Args: array of const); overload;
    // Increase indent by defined amount
    procedure IncIndent;
    // Decrease indent by defined amount
    procedure DecIndent;
    // Start a method implementation. Writes the declaration. No Begin.
    procedure BeginMethod(STrings: TStrings; const Decl: String); Virtual;
    // End a method implementation. Writes the final end;
    procedure EndMethod(STrings: TStrings; const Decl: String);Virtual;
    // The following must be overridden by descendents
    Procedure DoGenerateInterface(Strings: TStrings); virtual;
    Procedure DoGenerateImplementation(Strings: TStrings); virtual;
    // Override this to return an instance of the proper class.
    Function CreateOptions : TCodeGeneratorOptions; virtual;
    // Override to return minimal uses clause for interface section.
    Function GetInterfaceUsesClause : String; virtual;
    // Override to return minimal uses clause for implementation section.
    Function GetImplementationUsesClause : String; virtual;
    // Must override to return real fielddefs
    function GetFieldDefs: TFieldPropDefs; virtual;
    // Must override to return real fielddefs
    procedure SetFieldDefs(const AValue: TFieldPropDefs); virtual;
    // Must override to return real SQL
    function GetSQL: TStrings; virtual;
    // Must override to set real SQL
    procedure SetSQL(const AValue: TStrings); virtual;
  Public
    Constructor Create(AOWner : TComponent); override;
    Destructor Destroy; override;
    Procedure GenerateCode(Stream : TStream);
    Procedure GenerateCode(Strings: TStrings);
    Class Function NeedsSQL : Boolean; virtual;
    Class Function NeedsFieldDefs : Boolean; virtual;
    Function ShowConfigDialog : Boolean;
    Property Fields : TFieldPropDefs Read GetFieldDefs Write SetFieldDefs;
    Property SQL : TStrings Read GetSQL Write SetSQL;
  Published
    Property CodeOptions : TCodeGeneratorOptions Read FCodeOptions Write FCodeOptions;
    Property Indent : Integer Read FIndent Write FIndent Default 2;
  end;
  
  { TClassCodeGeneratorOptions }

  TClassCodeGeneratorOptions = Class(TCodeGeneratorOptions)
  private
    FAncestorClass: String;
    FClassName: String;
    procedure SetAncestorClass(const AValue: String);
  Protected
    procedure SetClassName(const AValue: String); virtual;
    // Set to default value. Publish if needed.
    Property AncestorClass : String Read FAncestorClass Write SetAncestorClass;
  Public
    Procedure Assign(ASource : TPersistent); override;
  Published
    Property ObjectClassName : String Read FClassName Write SetClassName;
  end;

  { TDDClassCodeGenerator }

  TDDClassCodeGenerator = Class(TDDCustomCodeGenerator)
  private
    FAncestorClass : String;
    FClassName: String;
    FFieldDefs: TFieldPropDefs;
    FOptions: TCodeOptions;
    FStreamClass: String;
    FStringsClass: String;
    FUnitName: String;
    function GetOpts: TClassCodeGeneratorOptions;
    procedure SetAncestorClass(const AValue: String);
    procedure SetClassName(const AValue: String);
    procedure SetUnitname(const AValue: String);
<<<<<<< HEAD
<<<<<<< HEAD
=======
    procedure WritePropertyGetterImpl(Strings: TStrings; F: TFieldPropDef);
    procedure WritePropertySetterImpl(Strings: TStrings; F: TFieldPropDef);
>>>>>>> graemeg/fixes_2_2
=======
    procedure WritePropertyGetterImpl(Strings: TStrings; F: TFieldPropDef);
    procedure WritePropertySetterImpl(Strings: TStrings; F: TFieldPropDef);
>>>>>>> origin/fixes_2_2
  Protected
    // Overrides from base class
    Function GetFieldDefs: TFieldPropDefs; override;
    procedure SetFieldDefs(const AValue: TFieldPropDefs); override;
    Function CreateOptions : TCodeGeneratorOptions; override;
    Procedure DoGenerateInterface(Strings: TStrings); override;
    Procedure DoGenerateImplementation(Strings: TStrings); override;
    // General code things.
    // Override to create TFieldpropdefs descendent instance.
    Function CreateFieldPropDefs : TFieldPropDefs; virtual;
    // Set to default value. Publish if needed.
    //
    // Interface routines
    //
<<<<<<< HEAD
<<<<<<< HEAD
    // Write property getter implementation
    procedure WritePropertyGetterImpl(Strings: TStrings; F: TFieldPropDef); virtual;
    // Write property setter implementation
    procedure WritePropertySetterImpl(Strings: TStrings; F: TFieldPropDef); virtual;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    // Create class declaration.
    procedure CreateDeclaration(Strings: TStrings); virtual;
    // Create class head. Override to add after class start.
    procedure CreateClassHead(Strings: TStrings); virtual;
    // Create class end. Override to add before class end.
    procedure CreateClassEnd(Strings : TStrings); virtual;
    // Called right after section start is written.
    procedure WriteVisibilityStart(V: TVisibility; Strings: TStrings); virtual;
    // Should a property declaration be written ?
    function AllowPropertyDeclaration(F: TFieldPropDef; AVisibility: TVisibilities): Boolean; virtual;
    // Creates a property declaration.
    Function PropertyDeclaration(Strings: TStrings; Def: TFieldPropDef) : String; virtual;
    // Writes private fields for class.
    procedure WritePrivateFields(Strings: TStrings); virtual;
    //
    // Implementation routines
    //
    // Create class implementation
    procedure CreateImplementation(Strings: TStrings); virtual;
    // Write implementation of constructor
    procedure WriteConstructorImplementation(Strings: TStrings); Virtual;
    // Write implementation of Destructor
    procedure WriteDestructorImplementation(Strings: TStrings); Virtual;
    // Write initialization of property (in constructor)
    procedure WriteFieldCreate(Strings: TStrings; F: TFieldPropDef); Virtual;
    // Write Finalization of property (in destructor)
    procedure WriteFieldDestroy(Strings: TStrings; F: TFieldPropDef); Virtual;
    //
    // Routines used in both Implementation/Interface
    //
    // Write property getter declaration
    Function PropertyGetterDeclaration(Def: TFieldPropDef; Impl : Boolean) : String; virtual;
    // Write property setter declaration
    Function PropertySetterDeclaration(Def: TFieldPropDef; Impl : Boolean) : String; virtual;
    // Determines whether a constructor/destructor pair is written.
    // By default one is written if ptStream/ptStrings is detected.
    Function NeedsConstructor : Boolean; virtual;
    // By default, this calls NeedsConstructor.
    Function NeedsDestructor : Boolean; virtual;
    // Override this to return the constructor declaration.
    Function ConstructorDeclaration(Impl : Boolean) : String; Virtual;
    // Override this to return the destructor declaration
    Function DestructorDeclaration(Impl : Boolean) : String; Virtual;
    //
    // Properties
    //
    // Class name used to instantiate TStrings instances.
    Property StringsClass : String Read FStringsClass Write FStringsClass;
    // Class name used to instantiate TStream instances.
    Property StreamClass : String Read FStreamClass Write FStreamClass;
    // Easy access to options
    Property ClassOptions : TClassCodeGeneratorOptions Read GetOpts;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
    Procedure GenerateClass(Strings : TStrings); virtual;
    Procedure GenerateClass(Stream : TStream);
  Published
    Property Fields;
  end;

  ECodeGenerator = Class(Exception);
  
  { TExportFormatItem }

  TDDCustomCodeGeneratorClass = Class of TDDCustomCodeGenerator;
  TCodeGeneratorConfigureEvent = Function (Generator : TDDCustomCodeGenerator) : Boolean of object;

  { TCodeGeneratorItem }

  TCodeGeneratorItem = Class(TCollectionItem)
  private
    FClass: TDDCustomCodeGeneratorClass;
    FDescription: String;
    FName: String;
    FOnConfigure: TCodeGeneratorConfigureEvent;
    Procedure SetName(const AValue: String);
  Public
    Property GeneratorClass : TDDCustomCodeGeneratorClass Read FClass Write FClass;
  Published
    Property Name : String Read FName Write SetName;
    Property Description : String Read FDescription Write FDescription;
    Property OnConfigureDialog : TCodeGeneratorConfigureEvent Read FOnConfigure Write FOnConfigure;
  end;

  { TCodeGenerators }

  TCodeGenerators = Class(TCollection)
  private
    function GetGen(Index : Integer): TCodeGeneratorItem;
    procedure SetGen(Index : Integer; const AValue: TCodeGeneratorItem);
  Public
    // Registration/Unregistration
    Function RegisterCodeGenerator(Const AName, ADescription : String; AClass : TDDCustomCodeGeneratorClass) : TCodeGeneratorItem;
    Procedure UnRegisterCodeGenerator(AClass : TDDCustomCodeGeneratorClass);
    Procedure UnRegisterCodeGenerator(Const AName : String);
    // Searching
    Function IndexOfCodeGenerator(Const AName : String): Integer;
    Function IndexOfCodeGenerator(AClass : TDDCustomCodeGeneratorClass): Integer;
    Function FindCodeGenerator(Const AName : String) : TCodeGeneratorItem;
    Function FindCodeGenerator(AClass : TDDCustomCodeGeneratorClass) : TCodeGeneratorItem;
    // Shows configuration dialog, if one was configured for this class
    Function ConfigureCodeGenerator(AGenerator : TDDCustomCodeGenerator) : Boolean;
    Function GeneratorByName(Const AName : String) : TCodeGeneratorItem;
    Property Generators[Index : Integer] : TCodeGeneratorItem Read GetGen Write SetGen; default;
  end;

Function CodeGenerators : TCodeGenerators;

// Easy access functions

Function RegisterCodeGenerator(Const AName,ADescription : String; AClass : TDDCustomCodeGeneratorClass) : TCodeGeneratorItem;
Procedure UnRegisterCodeGenerator(AClass : TDDCustomCodeGeneratorClass);
Procedure UnRegisterCodeGenerator(Const AName : String);

Type
  TFieldPropTypeMap = Array[TFieldType] of TPropType;
  TPropertyVisibilityMap = Array[TPropType] of TVisibility;

Var

  FieldToPropTypeMap : TFieldPropTypeMap = (
    ptCustom, ptAnsiString, ptSmallInt, ptLongInt, ptWord,
    ptBoolean, ptDouble, ptCurrency, ptCurrency, ptDateTime, ptDateTime, ptDateTime,
    ptCustom, ptCustom, ptLongInt, ptStream, ptTStrings, ptStream, ptTStrings,
    ptCustom, ptCustom, ptCustom, ptCustom, ptAnsiString,
    ptWideString, ptInt64, ptCustom, ptCustom, ptCustom,
    ptCustom, ptCustom, ptCustom, ptCustom, ptCustom,
    ptCustom, ptAnsiString, ptDateTime, ptCurrency, ptWideString, ptWideString);
    
  PropTypeToVisibilityMap : TPropertyVisibilityMap = (
    vPrivate,
    vPublished,
    vPublished, vPublished,
    vPublished, vPublished,
    vPublished, vPublished,
    vPublished, vPublished,
    vPublished, vPublished, vPublished,
    vPublished, vPublished, vPublished, vPublished, vPublished,
    vPublished,
    vPublished, vPublished, vPublic, vPublished,
    vPrivate);

Const
  ptInteger = ptLongint;
  ptString  = ptAnsiString;
Const
  PropTypeNames : Array[TPropType] of string
     = ('',
        'Boolean',
        'ShortInt', 'Byte',
        'SmallInt', 'Word',
        'Longint', 'Cardinal',
        'Int64', 'QWord',
        'String', 'AnsiString', 'WideString',
        'Single', 'Double' , 'Extended', 'Comp', 'Currency',
        'TDateTime',
        '','', 'TStream', 'TStrings',
        '');

Resourcestring
  SErrInvalidIdentifier = '"%s" is not a valid object pascal identifier.';
  SErrGeneratorExists   = 'A code generator with name "%s" already exists';
  SUnknownGenerator     = 'Unknown code generator name : "%s"';

Function MakeIdentifier (S : String) : String;
Function CreateString(S : String) : String;
Procedure CheckIdentifier(AValue : String; AllowEmpty : Boolean = True);

implementation

Function CreateString(S : String) : String;

begin
  Result:=StringReplace(S,'''','''''',[rfReplaceAll]);
  Result:=''''+Result+'''';
end;

Procedure CheckIdentifier(AValue : String; AllowEmpty : Boolean = True);

begin
  If ((AValue='') and Not AllowEmpty) or Not IsValidIdent(AValue) then
    Raise ECodeGenerator.CreateFmt(SErrInvalidIdentifier,[AValue]);
end;

Var
  CodeGens : TCodeGenerators;

function CodeGenerators: TCodeGenerators;
begin
  If (CodeGens=Nil) then
    CodeGens:=TCodeGenerators.Create(TCodeGeneratorItem);
  Result:=CodeGens;
end;

Procedure DoneCodeGenerators;

begin
  FreeAndNil(CodeGens);
end;

function RegisterCodeGenerator(const AName, ADescription: String;
  AClass: TDDCustomCodeGeneratorClass): TCodeGeneratorItem;
begin
  CodeGenerators.RegisterCodeGenerator(AName,ADescription,AClass);
end;

procedure UnRegisterCodeGenerator(AClass: TDDCustomCodeGeneratorClass);
begin
  CodeGenerators.UnRegisterCodeGenerator(AClass);
end;

procedure UnRegisterCodeGenerator(const AName: String);
begin
  CodeGenerators.UnRegisterCodeGenerator(AName);
end;

Function MakeIdentifier (S : String) : String;

Var
  I : Integer;
  
begin
  Result:=S;
  For I:=Length(Result) downto 0 do
    If Not ((Upcase(Result[i]) in ['_','A'..'Z'])
             or ((I>0) and (Result[i] in (['0'..'9'])))) then
     Delete(Result,i,1);
end;

{ TFieldPropDef }

function TFieldPropDef.GetPropName: String;
begin
  Result:=FPropName;
  If (Result='') then
    Result:=MakeIdentifier(FFieldName);
end;

function TFieldPropDef.GetPropType: TPropType;
begin
  Result:=FPropType;
  If (Result=ptAuto) then
    Result:=FieldToPropTypeMap[FieldType];
end;

function TFieldPropDef.GetPropTypeStored: boolean;
begin
  Result:=(FPropType<>ptAuto)
end;


procedure TFieldPropDef.SetPropName(const AValue: String);

begin
  If (AValue<>FPropName) then
    begin
    CheckIdentifier(AValue);
    FPropName:=AValue;
    end;
end;

procedure TFieldPropDef.InitFromField(F: TField);
begin
  FieldType:=F.DataType;
  PropertySize:=F.Size;
end;

procedure TFieldPropDef.InitFromDDFieldDef(F: TDDFieldDef);
begin
  FieldType:=F.FieldType;
  PropertySize:=F.Size;
end;

constructor TFieldPropDef.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  FPropVis:=vPublished
end;

procedure TFieldPropDef.Assign(ASource: TPersistent);

Var
  PD : TFieldPropDef;

begin
  if (ASource is TFieldPropDef) then
    begin
    PD:=ASource as TFieldPropDef;
    FEnabled:=PD.Enabled;
    FFieldName:=PD.FFieldName;
    FFieldType:=PD.FFIeldType;
    FPropAccess:=PD.FPropAccess;
    FPropDef:=PD.FPropDef;
    FPropType:=PD.FPropType;
    FPRopSize:=PD.FPropSize;
    FPropName:=PD.FPropName;
    FPropVis:=PD.FPropVis;
    end
  else
    inherited Assign(ASource);
end;

function TFieldPropDef.FieldPropDefs: TFieldPropDefs;
begin
  Result:=Collection as TFieldPropDefs;
end;

function TFieldPropDef.HasGetter: Boolean;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=psRead in PropSetters;
=======
  Result:=False;
>>>>>>> graemeg/fixes_2_2
=======
  Result:=False;
>>>>>>> origin/fixes_2_2
end;

function TFieldPropDef.HasSetter: Boolean;
begin
  Result:=(PropertyAccess in [paReadWrite,paWriteOnly])
<<<<<<< HEAD
<<<<<<< HEAD
          and ((PropertyType in [ptStream,ptTStrings]) or (psWrite in Propsetters));
=======
          and (PropertyType in [ptStream,ptTStrings]);
>>>>>>> graemeg/fixes_2_2
=======
          and (PropertyType in [ptStream,ptTStrings]);
>>>>>>> origin/fixes_2_2
end;

function TFieldPropDef.ObjPasTypeDef: String;
begin
  If PropertyType in [ptCustom,ptSet,ptEnumerated] then
    Result:=PropertyDef
  else
    begin
    Result:=PropTypeNames[PropertyType];
    If PropertyType=ptShortString then
      Result:=Result+Format('String[%d]',[PropertySize]);
    end;
end;

function TFieldPropDef.ObjPasReadDef: String;
begin
  If HasGetter then
    Result:='Get'+PropertyName
  else
    Result:='F'+PropertyName;
end;

function TFieldPropDef.ObjPasWriteDef: String;
begin
  If HasSetter then
    Result:='Set'+PropertyName
  else
    Result:='F'+PropertyName;
end;

{ TFieldPropDefs }

function TFieldPropDefs.GetPropDef(Index : integer): TFieldPropDef;
begin
  Result:=TFieldPropDef(Items[index]);
end;

procedure TFieldPropDefs.SetPropDef(Index : integer; const AValue: TFieldPropDef);
begin
  Items[Index]:=AValue;
end;

function TFieldPropDefs.AddDef(AName: String): TFieldPropDef;
begin
  Result:=Add As TFieldPropDef;
  Result.FieldName:=AName;
end;

procedure TFieldPropDefs.FromDataset(Dataset: TDataset; DoClear: Boolean = True);

Var
  I : Integer;
  D : TFieldPropDef;
  F : TField;
  
begin
  If DoClear then
    Clear;
  For I:=0 to Dataset.Fields.Count-1 do
    begin
    F:=Dataset.Fields[I];
    D:=AddDef(F.FieldName);
    D.Enabled:=True;
    D.InitFromField(F);
    end;
end;

procedure TFieldPropDefs.FromDDFieldDefs(Defs: TDDFieldDefs; DoClear: Boolean = True);

Var
  I : Integer;
  D : TFieldPropDef;
  F : TDDFieldDef;

begin
  If DoClear then
    Clear;
  For I:=0 to Defs.Count-1 do
    begin
    F:=Defs[I];
    D:=AddDef(F.FieldName);
    D.Enabled:=True;
    D.InitFromDDFieldDef(F);
    end;
end;

function TFieldPropDefs.IndexOfPropName(AName: String): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetPropDef(Result).PropertyName,AName)<>0) do
    Dec(Result);
end;

function TFieldPropDefs.IndexOfFieldName(AName: String): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetPropDef(Result).FieldName,AName)<>0) do
    Dec(Result);
end;

function TFieldPropDefs.FindPropName(AName: String): TFieldPropDef;

Var
  I : Integer;
  
begin
  I:=IndexOfPropName(AName);
  If (I<>-1) then
    Result:=GetpropDef(I)
  else
    Result:=Nil;
end;

function TFieldPropDefs.FindFieldName(AName: String): TFieldPropDef;

Var
  I : Integer;

begin
  I:=IndexOfFieldName(AName);
  If (I<>-1) then
    Result:=GetpropDef(I)
  else
    Result:=Nil;
end;

{ TDDClassCodeGenerator }

procedure TDDClassCodeGenerator.SetClassName(const AValue: String);
begin
end;

procedure TDDClassCodeGenerator.SetAncestorClass(const AValue: String);
begin
  FAncestorClass:=AValue;
end;

function TDDClassCodeGenerator.GetOpts: TClassCodeGeneratorOptions;
begin
  Result:=CodeOptions as TClassCodeGeneratorOptions;
end;

procedure TDDClassCodeGenerator.SetFieldDefs(const AValue: TFieldPropDefs);
begin
  if FFieldDefs=AValue then exit;
  FFieldDefs:=AValue;
end;


procedure TDDClassCodeGenerator.SetUnitname(const AValue: String);
begin
  FUnitName:=AValue;
end;

function TDDClassCodeGenerator.CreateFieldPropDefs: TFieldPropDefs;
begin
  Result:=TFieldPropDefs.Create(TFieldPropDef);
end;

constructor TDDClassCodeGenerator.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FFieldDefs:=CreateFieldPropDefs;
<<<<<<< HEAD
<<<<<<< HEAD
  StringsClass:='TStringList';
  StreamClass:='TMemoryStream';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

destructor TDDClassCodeGenerator.Destroy;
begin
  FreeAndNil(FFieldDefs);
  inherited Destroy;
end;


procedure TDDClassCodeGenerator.GenerateClass(Strings: TStrings);

begin
 IncIndent;
 Try
  AddLn(Strings,'// Declaration');
  AddLn(Strings,'Type');
  AddLn(Strings);
  CreateDeclaration(Strings);
  AddLn(Strings);
  AddLn(Strings,'// Implementation');
  AddLn(Strings);
  CreateDeclaration(Strings);
  Finally
    DecIndent;
  end;
end;

Function TDDClassCodeGenerator.AllowPropertyDeclaration(F : TFieldPropDef; AVisibility : TVisibilities) : Boolean;

begin
  Result:=Assigned(f) and F.Enabled and ((AVisibility=[]) or (F.PropertyVisibility in AVisibility));
end;

Procedure TDDClassCodeGenerator.CreateDeclaration(Strings : TStrings);

Const
  VisibilityNames : Array [TVisibility] of string
                  = ('Private','Protected','Public','Published');

Var
  V : TVisibility;
  I : Integer;
  F : TFieldPropDef;

begin
  CreateClassHead(Strings);
  AddLn(Strings,VisibilityNames[vPrivate]);
  WritePrivateFields(Strings);
  For v:=Low(TVisibility) to High(TVisibility) do
    begin
    AddLn(Strings,VisibilityNames[v]);
    IncIndent;
    Try
      WriteVisibilityStart(V,Strings);
      For I:=0 to Fields.Count-1 do
        begin
        F:=Fields[i];
        if AllowPropertyDeclaration(F,[V]) then
          AddLn(Strings,PropertyDeclaration(Strings,F)+';');
        end;
    Finally
      Decindent;
    end;
    end;
  CreateClassEnd(Strings);
end;

Procedure TDDClassCodeGenerator.WritePrivateFields(Strings : TStrings);

Var
  I : Integer;
  F : TFieldPropDef;

begin
  IncIndent;
  Try
    For I:=0 to Fields.Count-1 do
      begin
      F:=Fields[i];
      if AllowPropertyDeclaration(F,[]) then
        AddLn(Strings,'F%s : %s;',[F.PropertyName,F.ObjPasTypeDef]);
      end;
  Finally
    DecIndent;
  end;
end;

Procedure TDDClassCodeGenerator.CreateImplementation(Strings : TStrings);

Var
  B : Boolean;
  I : Integer;
  F : TFieldPropDef;
  
begin
  AddLn(Strings,' { %s } ',[ClassOptions.ObjectClassName]);
  AddLn(Strings);
  If NeedsConstructor then
    begin
    Addln(Strings,' { Constructor and destructor }');
    Addln(Strings);
    WriteConstructorImplementation(Strings);
    WriteDestructorImplementation(Strings);
    end;
  B:=False;
  For I:=0 to Fields.Count-1 do
    begin
    F:=Fields[i];
    if AllowPropertyDeclaration(F,[]) and F.HasGetter then
      begin
      If not B then
        begin
        B:=True;
        Addln(Strings,' { Property Getters }');
        Addln(Strings);
        end;
      WritePropertyGetterImpl(Strings,F);
      end;
    end;
  B:=False;
  For I:=0 to Fields.Count-1 do
    begin
    F:=Fields[i];
<<<<<<< HEAD
<<<<<<< HEAD
    if AllowPropertyDeclaration(F,[]) and F.HasSetter then
=======
    if AllowPropertyDeclaration(F,[]) and F.HasGetter then
>>>>>>> graemeg/fixes_2_2
=======
    if AllowPropertyDeclaration(F,[]) and F.HasGetter then
>>>>>>> origin/fixes_2_2
      begin
      If not B then
        begin
        B:=True;
        Addln(Strings,' { Property Setters }');
        Addln(Strings);
        end;
      WritePropertySetterImpl(Strings,F);
      end;
    end;
end;

Procedure TDDClassCodeGenerator.WritePropertyGetterImpl(Strings : TStrings; F : TFieldPropDef);

Var
  S : String;

begin
  S:=PropertyGetterDeclaration(F,True);
  BeginMethod(Strings,S);
  AddLn(Strings,'begin');
  IncIndent;
  Try
<<<<<<< HEAD
<<<<<<< HEAD
    AddLn(Strings,Format('Result:=F%s;',[F.PropertyName]));
=======
    AddLn(Strings,Format('Result:=F%s',[F.PropertyName]));
>>>>>>> graemeg/fixes_2_2
=======
    AddLn(Strings,Format('Result:=F%s',[F.PropertyName]));
>>>>>>> origin/fixes_2_2
  Finally
    DecIndent;
  end;
  EndMethod(Strings,S);
end;

Procedure TDDClassCodeGenerator.WritePropertySetterImpl(Strings : TStrings; F : TFieldPropDef);

Var
  S : String;
<<<<<<< HEAD
<<<<<<< HEAD
  L : Integer;

begin
  S:=PropertySetterDeclaration(F,True);
=======

begin
  S:=PropertyGetterDeclaration(F,True);
>>>>>>> graemeg/fixes_2_2
=======

begin
  S:=PropertyGetterDeclaration(F,True);
>>>>>>> origin/fixes_2_2
  BeginMethod(Strings,S);
  AddLn(Strings,'begin');
  IncIndent;
  Try
<<<<<<< HEAD
<<<<<<< HEAD
    AddLn(Strings,Format('if (F%s=AValue) then exit;',[F.PropertyName]));
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Case F.PropertyType of
      ptTStrings :
        S:=Format('F%s.Assign(AValue);',[F.PropertyName]);
      ptStream :
        S:=Format('F%s.CopyFrom(AValue,0);',[F.PropertyName]);
    else
<<<<<<< HEAD
<<<<<<< HEAD
       S:=Format('F%s:=AValue;',[F.PropertyName]);
    end;
    AddLn(Strings,S);
    S:=CodeOptions.ExtraSetterLine;
    L:=Length(S);
    if (L>0) then
      begin
      S:=StringReplace(S,'%PROPNAME%',F.PropertyName,[rfReplaceAll,rfIgnoreCase]);
      L:=Length(S);
      if (S[L]<>';') then
        S:=S+';';
      AddLn(Strings,S);  
      end;
=======
       S:=Format('F%s:=AValue',[F.PropertyName]);
    end;
    AddLn(Strings,S);
>>>>>>> graemeg/fixes_2_2
=======
       S:=Format('F%s:=AValue',[F.PropertyName]);
    end;
    AddLn(Strings,S);
>>>>>>> origin/fixes_2_2
  Finally
    DecIndent;
  end;
  EndMethod(Strings,S);
end;

function TDDClassCodeGenerator.GetFieldDefs: TFieldPropDefs;
begin
  Result:=FFieldDefs;
end;

function TDDClassCodeGenerator.CreateOptions: TCodeGeneratorOptions;
begin
  Result:=TClassCodeGeneratorOptions.Create;
end;

procedure TDDClassCodeGenerator.DoGenerateInterface(Strings: TStrings);
begin
  AddLn(Strings,'Type');
  AddLn(Strings);
  IncIndent;
  Try
    CreateDeclaration(Strings);
  Finally
    DecIndent;
  end;
end;

procedure TDDClassCodeGenerator.DoGenerateImplementation(Strings: TStrings);
begin
  CreateImplementation(Strings);
end;


Procedure TDDClassCodeGenerator.WriteConstructorImplementation(Strings : TStrings);

Var
  I : Integer;
  F : TFieldPropDef;
  S : String;

begin
  S:=ConstructorDeclaration(True);
  BeginMethod(Strings,S);
  AddLn(Strings,'begin');
<<<<<<< HEAD
<<<<<<< HEAD
  AddLn(Strings,'  inherited;');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  IncIndent;
  Try
    For I:=0 to Fields.Count-1 do
      begin
      F:=Fields[i];
      if F.Enabled then
        WriteFieldCreate(Strings,F);
      end;
  Finally
    DecIndent;
  end;
  EndMethod(Strings,S);
end;

Procedure TDDClassCodeGenerator.WriteDestructorImplementation(Strings : TStrings);

Var
  I : Integer;
  F : TFieldPropDef;
  S : String;

begin
  S:=DestructorDeclaration(True);
  BeginMethod(Strings,S);
  AddLn(Strings,'begin');
  IncIndent;
  Try
    For I:=0 to Fields.Count-1 do
      begin
      F:=Fields[i];
      if F.Enabled then
        WriteFieldDestroy(Strings,F);
      end;
    AddLn(Strings,'Inherited;');
  Finally
    DecIndent;
  end;
  EndMethod(Strings,S);
end;



Procedure TDDClassCodeGenerator.WriteFieldCreate(Strings : TStrings;F : TFieldPropDef);

Var
  S : String;

begin
  Case F.PropertyType of
    ptTStrings :
      begin
      S:=Format('F%s:=%s.Create;',[F.PropertyName,StringsClass]);
      AddLn(Strings,S);
      end;
    ptStream :
      begin
      S:=Format('F%s:=%s.Create;',[F.PropertyName,StreamClass]);
      AddLn(Strings,S);
      end;
    ptCustom :
      begin
      AddLn(Strings,'// Add Creation for '+F.PropertyName);
      end;
  end;
end;

Procedure TDDClassCodeGenerator.WriteFieldDestroy(Strings : TStrings;F : TFieldPropDef);

Var
  S : String;

begin
  Case F.PropertyType of
    ptTStrings,
    ptStream :
      begin
      S:=Format('FreeAndNil(F%s);',[F.PropertyName]);
      AddLn(Strings,S);
      end;
    ptCustom :
      begin
      AddLn(Strings,'// Add destroy for '+F.PropertyName);
      end;
  end;
end;


Procedure TDDClassCodeGenerator.CreateClassHead(Strings : TStrings);

begin
  Addln(Strings,'{ %s }',[ClassOptions.ObjectClassName]);
  AddLn(Strings);
  AddLn(Strings,'%s = Class(%s)',[ClassOptions.ObjectClassName,ClassOptions.AncestorClass]);
end;

Procedure TDDClassCodeGenerator.CreateClassEnd(Strings : TStrings);

begin
  AddLn(Strings,'end;');
  AddLn(Strings);
end;


Procedure TDDClassCodeGenerator.WriteVisibilityStart(V : TVisibility; Strings : TStrings);

Var
  I : Integer;
  F : TFieldPropDef;
  
begin
  If (v=vPrivate) then
    begin
    For I:=0 to Fields.Count-1 do
      begin
      F:=Fields[i];
      If AllowPropertyDeclaration(F,[]) then
        begin
        if (F.Hasgetter) then
          AddLn(Strings,PropertyGetterDeclaration(F,False));
        if (F.HasSetter) then
          AddLn(Strings,PropertySetterDeclaration(F,False));
        end;
      end;
    end
  else if v=vPublic then
    begin
    If NeedsConstructor then
      begin
      AddLn(Strings,ConstructorDeclaration(False));
      Addln(Strings,DestructorDeclaration(False));
      end;
    end
  // Do nothing
end;


Function TDDClassCodeGenerator.PropertyDeclaration(Strings : TStrings; Def : TFieldPropDef) : String;

begin
  Result:='Property '+Def.PropertyName+' ';
  Result:=Result+': '+Def.ObjPasTypeDef;
  If Def.PropertyAccess in [paReadWrite,paReadOnly] then
    Result:=Result+' Read '+Def.ObjPasReadDef;
  If Def.PropertyAccess in [paReadWrite,paWriteOnly] then
    Result:=Result+' Write '+Def.ObjPasWriteDef;
end;

Function TDDClassCodeGenerator.PropertyGetterDeclaration(Def : TFieldPropDef; Impl : Boolean) : String;


begin
  Result:='Function ';
  If Impl then
    Result:=Result+Classoptions.ObjectClassName+'.';
  If Impl then
    Result:=Result+Def.ObjPasReadDef+' : '+Def.ObjPasTypeDef+';';
end;

Function TDDClassCodeGenerator.PropertySetterDeclaration(Def : TFieldPropDef; Impl : Boolean) : String;


begin
  Result:='Procedure ';
  If Impl then
    Result:=Result+ClassOptions.ObjectClassName+'.';
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=Result+Def.ObjPasWriteDef+' (AValue  : '+Def.ObjPasTypeDef+');';
=======
  Result:=Result+Def.ObjPasReadDef+' (AValue  : '+Def.ObjPasTypeDef+');';
>>>>>>> graemeg/fixes_2_2
=======
  Result:=Result+Def.ObjPasReadDef+' (AValue  : '+Def.ObjPasTypeDef+');';
>>>>>>> origin/fixes_2_2
end;

function TDDClassCodeGenerator.NeedsConstructor: Boolean;

Var
  I : Integer;
  F : TFieldPropDef;

begin
  Result:=False;
  I:=Fields.Count-1;
  While (Not Result) and (I>=0) do
    begin
    F:=Fields[i];
    Result:=F.Enabled and (F.PropertyType in [ptStream,ptTStrings]);
    Dec(I);
    end;
end;

function TDDClassCodeGenerator.NeedsDestructor: Boolean;
begin
  Result:=NeedsConstructor;
end;

Function TDDClassCodeGenerator.ConstructorDeclaration(Impl : Boolean) : String;
begin
  Result:='Constructor ';
  If Impl then
    Result:=Result+ClassOptions.ObjectClassName+'.';
  Result:=Result+'Create;';
end;

Function TDDClassCodeGenerator.DestructorDeclaration(Impl : Boolean) : String;
begin
  Result:='Destructor ';
  If Impl then
    Result:=Result+ClassOptions.ObjectClassName+'.';
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=Result+'Destroy;';
  if not Impl then
    Result:=Result+' Override;';
=======
  Result:=Result+'Destroy; Override;';
>>>>>>> graemeg/fixes_2_2
=======
  Result:=Result+'Destroy; Override;';
>>>>>>> origin/fixes_2_2
end;

procedure TDDClassCodeGenerator.GenerateClass(Stream: TStream);

Var
  L : TStringList;

begin
  L:=TStringList.Create;
  try
    GenerateClass(L);
    L.SaveToStream(Stream);
  finally
    L.Free;
  end;
end;

{ TDDCustomCodeGenerator }

procedure TDDCustomCodeGenerator.IncIndent;

begin
  FCurrentIndent:=FCurrentIndent+StringOfChar(' ',FIndent);
end;

procedure TDDCustomCodeGenerator.DecIndent;

begin
  Delete(FCurrentIndent,1,FIndent);
end;

procedure TDDCustomCodeGenerator.DoGenerateInterface(Strings: TStrings);
begin
end;

procedure TDDCustomCodeGenerator.DoGenerateImplementation(Strings: TStrings);
begin

end;

function TDDCustomCodeGenerator.GetFieldDefs: TFieldPropDefs;
begin

end;

procedure TDDCustomCodeGenerator.SetFieldDefs(const AValue: TFieldPropDefs);
begin

end;

function TDDCustomCodeGenerator.GetSQL: TStrings;
begin
  Result:=Nil;
end;

procedure TDDCustomCodeGenerator.SetSQL(const AValue: TStrings);
begin
  // Do nothing
end;

constructor TDDCustomCodeGenerator.Create(AOWner: TComponent);
begin
  inherited Create(AOWner);
  FCodeOptions:=CreateOptions;
  FIndent:=2;
end;

destructor TDDCustomCodeGenerator.Destroy;
begin
  FreeAndNil(FCodeOptions);
  inherited Destroy;
end;

procedure TDDCustomCodeGenerator.AddLn(Strings : TStrings);

begin
  Strings.Add('');
end;

procedure TDDCustomCodeGenerator.AddLn(Strings : TStrings; Line : String);

begin
  Strings.Add(FCurrentIndent+Line);
end;

procedure TDDCustomCodeGenerator.AddLn(Strings : TStrings; Fmt : String; Args : Array Of Const);

begin
  Strings.Add(FCurrentIndent+Format(Fmt,Args));
end;


function TDDCustomCodeGenerator.CreateOptions: TCodeGeneratorOptions;
begin
  Result:=TCodeGeneratorOptions.Create;
end;

function TDDCustomCodeGenerator.GetInterfaceUsesClause: String;
begin
  Result:='Classes, SysUtils';
  If (CodeOptions.InterfaceUnits<>'') then
    Result:=Result+','+CodeOptions.InterfaceUnits;
end;

function TDDCustomCodeGenerator.GetImplementationUsesClause: String;
begin
  Result:=CodeOptions.ImplementationUnits;
end;

procedure TDDCustomCodeGenerator.GenerateCode(Stream: TStream);

Var
  L : TStringList;

begin
  L:=TStringList.Create;
  try
    GenerateCode(L);
    L.SaveToStream(Stream);
  finally
    L.Free;
  end;
end;

procedure TDDCustomCodeGenerator.GenerateCode(Strings: TStrings);

  Procedure MaybeAddUsesClause(S : String);
  
  begin
    If (S<>'') then
      begin
      If S[Length(S)]<>';' then
        S:=S+';';
      AddLn(Strings,'Uses '+S);
      AddLn(Strings);
      end;
  end;

Var
  S : String;
  
begin
  FCurrentIndent:='';
  if (coUnit in CodeOptions.Options) then
    begin
    Addln(Strings,'Unit '+CodeOptions.UnitName+';');
    Addln(Strings);
<<<<<<< HEAD
<<<<<<< HEAD
    Addln(Strings, '{$mode objfpc}{$H+}');
    Addln(Strings);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Addln(Strings,'Interface');
    Addln(Strings);
    S:=GetInterfaceUsesClause;
    MaybeAddUsesClause(S);
    end;
  if coInterface in CodeOptions.Options then
    begin
    DoGenerateInterface(Strings);
    Addln(Strings);
    end;
  FCurrentIndent:='';
  if coUnit in CodeOptions.options then
    begin
    if coImplementation in CodeOptions.Options then
      begin
      Addln(Strings,'Implementation');
      S:=GetImplementationUsesClause;
      MaybeAddUsesClause(S);
      end;
    end;
  if coImplementation in CodeOptions.Options then
    begin
    Addln(Strings);
    DoGenerateImplementation(Strings);
    end;
  Addln(Strings);
  if (coUnit in CodeOptions.options) then
    Addln(Strings,'end.');
end;

class function TDDCustomCodeGenerator.NeedsSQL: Boolean;
begin
  Result:=False;
end;

class function TDDCustomCodeGenerator.NeedsFieldDefs: Boolean;
begin
  Result:=False;
end;

function TDDCustomCodeGenerator.ShowConfigDialog: Boolean;
begin

end;

Procedure TDDCustomCodeGenerator.BeginMethod(STrings : TStrings; Const Decl : String);

begin
  AddLn(Strings,Decl);
  AddLn(Strings);
end;

Procedure TDDCustomCodeGenerator.EndMethod(STrings : TStrings; Const Decl : String);

begin
  AddLn(Strings,'end;');
  Addln(Strings);
  Addln(Strings);
end;


{ TCodeGeneratorItem }

procedure TCodeGeneratorItem.SetName(const AValue: String);

Var
  G : TCodeGeneratorItem;

begin
  if (FName=AValue) then
    exit;
  If (AValue<>'') then
    begin
    G:=TCodeGenerators(Collection).FindCodeGenerator(AValue);
    If (G<>Nil) and (G<>Self) then
      Raise ECodeGenerator.CreateFmt(SErrGeneratorExists,[AValue]);
    end;
  FName:=AValue;

end;

{ TCodeGenerators }

function TCodeGenerators.GetGen(Index: Integer): TCodeGeneratorItem;
begin
  Result:=TCodeGeneratorItem(Items[Index]);
end;

procedure TCodeGenerators.SetGen(Index: Integer;
  const AValue: TCodeGeneratorItem);
begin
  Items[Index]:=AValue;
end;

function TCodeGenerators.RegisterCodeGenerator(const AName, ADescription : String;
  AClass: TDDCustomCodeGeneratorClass): TCodeGeneratorItem;
begin
  If (IndexOfCodeGenerator(AName)<>-1) then
    Raise ECodeGenerator.CreateFmt(SErrGeneratorExists,[AName]);
  Result:=Add as TCodeGeneratorItem;
  Result.Name:=AName;
  Result.Description:=ADescription;
  Result.GeneratorClass:=AClass;
end;

procedure TCodeGenerators.UnRegisterCodeGenerator(AClass: TDDCustomCodeGeneratorClass);
begin
  FindCodeGenerator(AClass).Free;
end;

procedure TCodeGenerators.UnRegisterCodeGenerator(const AName: String);
begin
  FindCodeGenerator(AName).Free;
end;

function TCodeGenerators.IndexOfCodeGenerator(const AName: String): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetGen(Result).Name,AName)<>0) do
    Dec(Result);
end;

function TCodeGenerators.IndexOfCodeGenerator(AClass: TDDCustomCodeGeneratorClass): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (GetGen(Result).GeneratorClass<>AClass) do
    Dec(Result);
end;

function TCodeGenerators.FindCodeGenerator(const AName: String): TCodeGeneratorItem;

Var
  I : Integer;

begin
  I:=IndexOfCodeGenerator(AName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetGen(I);
end;

function TCodeGenerators.FindCodeGenerator(AClass: TDDCustomCodeGeneratorClass): TCodeGeneratorItem;

Var
  I : Integer;

begin
  I:=IndexOfCodeGenerator(AClass);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetGen(I);
end;

function TCodeGenerators.ConfigureCodeGenerator(
  AGenerator: TDDCustomCodeGenerator): Boolean;

Var
  G : TCodeGeneratorItem;

begin
  Result:=True;
  G:=FindCodeGenerator(TDDCustomCodeGeneratorClass(AGenerator.ClassType));
  If Assigned(G) and Assigned(G.OnConfigureDialog) then
    Result:=G.OnConfigureDialog(AGenerator);
end;

function TCodeGenerators.GeneratorByName(const AName: String): TCodeGeneratorItem;
begin
  Result:=FindCodeGenerator(AName);
  If (Result=Nil) then
    Raise ECodegenerator.CreateFmt(SUnknownGenerator,[AName]);
end;

{ TCodeGeneratorOptions }

procedure TCodeGeneratorOptions.SetOPtions(const AValue: TCodeOptions);
begin
  FOptions:=AValue;
end;

constructor TCodeGeneratorOptions.create;
begin
  FOptions:=[coInterface,coImplementation,coUnit];
  UnitName:='Unit1';
end;

procedure TCodeGeneratorOptions.Assign(ASource: TPersistent);

Var
  CG : TCodeGeneratorOptions;
  
begin
  If ASource is TCodeGeneratorOptions then
    begin
    CG:=ASource as TCodeGeneratorOptions;
<<<<<<< HEAD
<<<<<<< HEAD
    FInterfaceUnits:=CG.InterfaceUnits;
    FImplementationUnits:=CG.ImplementationUnits;
    FOptions:=CG.FOptions;
    FUnitName:=CG.UnitName;
    FExtraSetterLine:=CG.ExtraSetterLine;
=======
    FOptions:=CG.FOptions;
    FUnitName:=CG.UnitName;
>>>>>>> graemeg/fixes_2_2
=======
    FOptions:=CG.FOptions;
    FUnitName:=CG.UnitName;
>>>>>>> origin/fixes_2_2
    end
  else
    inherited Assign(ASource);
end;

procedure TCodeGeneratorOptions.SetUnitname(const AValue: String);
begin
  if FUnitName=AValue then exit;
  CheckIdentifier(AValue,False);
  FUnitName:=AValue;
end;

procedure TCodeGeneratorOptions.SetInterfaceUnits(const AValue: String);
begin
  if FInterfaceUnits=AValue then exit;
  FInterfaceUnits:=AValue;
  // Do some checks here
end;

procedure TCodeGeneratorOptions.SetImplementationUnits(const AValue: String);
begin
  if FImplementationUnits=AValue then exit;
  FImplementationUnits:=AValue;
end;

{ TClassCodeGeneratorOptions }

procedure TClassCodeGeneratorOptions.SetClassName(const AValue: String);
begin
  if FClassName=AValue then
    exit;
  CheckIdentifier(AValue,False);
  FClassName:=AValue;
end;

procedure TClassCodeGeneratorOptions.Assign(ASource: TPersistent);

Var
  CO : TClassCodeGeneratorOptions;

begin
  If ASource is TClassCodeGeneratorOptions then
    begin
    CO:=ASource as TClassCodeGeneratorOptions;
    FClassName:=CO.FClassName;
    FAncestorClass:=CO.FAncestorClass;
    end;
  inherited Assign(ASource);
end;

procedure TClassCodeGeneratorOptions.SetAncestorClass(const AValue: String);
begin
  if (FAncestorClass=AValue) then
    Exit;
  CheckIdentifier(AValue,False);
  FAncestorClass:=AValue;
end;



Finalization
  DoneCodeGenerators;
end.

