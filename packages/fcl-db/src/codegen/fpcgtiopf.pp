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
unit fpcgtiopf;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, db, fpddcodegen;
  
TYpe
<<<<<<< HEAD
<<<<<<< HEAD
  TClassOption = (caCreateClass,caConstructor,caDestructor,caCreateList,
                  caListAddMethod,caListItemsProperty,caOverrideRead,
                  caOverrideReadThis,caOverrideSave);
  TClassOptions = Set of TClassOption;
  TVisitorOption = (voRead,voReadList,voCreate,voDelete,voUpdate,
                    voCommonSetupParams,voSingleSaveVisitor,voRegisterVisitors);
  TVisitorOptions = set of TVisitorOption;

  { TTiOPFFieldPropDef }

  TTiOPFFieldPropDef = Class(TFieldPropDef)
  Public
    Constructor Create(ACollection : TCollection); override;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
  TClassOption = (caCreateClass,caConstructor,caDestructor,caCreateList,caListAddMethod,caListItemsProperty);
  TClassOptions = Set of TClassOption;
  TVisitorOption = (voRead,voReadList,voCreate,voDelete,voUpdate,
                    voCommonSetupParams,voSingleSaveVisitor);
  TVisitorOptions = set of TVisitorOption;
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { TTiOPFCodeOptions }

  TTiOPFCodeOptions = Class (TClassCodeGeneratorOptions)
  Private
    FClassOptions: TClassOptions;
    FFinalVisitors: TVisitorOptions;
    FListAncestorName: String;
    FListClassName : String;
    FVisitorOptions: TVisitorOptions;
    FTableName : String;
    function GetListClassName: String;
    procedure SetClassOptions(const AValue: TClassOptions);
    procedure SetListAncestorName(const AValue: String);
    procedure SetListClassName(const AValue: String);
    procedure SetVisitorOptions(const AValue: TVisitorOptions);
  Public
    Constructor Create; override;
    Procedure Assign(ASource : TPersistent); override;
  Published
    Property ClassOptions : TClassOptions Read FClassOptions Write SetClassOptions;
    Property VisitorOptions : TVisitorOptions Read FVisitorOptions Write SetVisitorOptions;
    Property FinalVisitors : TVisitorOptions Read FFinalVisitors Write FFinalVisitors;
    Property ListAncestorName : String Read FListAncestorName Write SetListAncestorName;
    Property ListClassName : String Read GetListClassName Write SetListClassName;
    Property AncestorClass;
    Property TableName : String Read FTableName Write FTableName;
  end;
  
  { TTiOPFCodeGenerator }

  TTiOPFCodeGenerator = Class(TDDClassCodeGenerator)
    procedure CreateListImplementation(Strings: TStrings; const ObjectClassName, ListClassName: String);
    function BeginInit(Strings: TStrings; const AClass: String): String;
    function BeginAcceptVisitor(Strings: TStrings; const AClass, ObjectClassName: String): String;
    function BeginSetupParams(Strings: TStrings; const AClass,ObjectClassName: String; DeclareObject : Boolean): String;
    function BeginMapRowToObject(Strings: TStrings; const AClass, ObjectClassName : String): String;
    procedure DeclareObjectvariable(Strings: TStrings;
      const ObjectClassName: String);
  private
    Function CreateSQLStatement(V: TVisitorOption) : String;
    function GetOpt: TTiOPFCodeOptions;
    Function UseCommonSetupParams : Boolean;
    Function SingleSaveVisitor : Boolean;
    Function VisitorClassName(V : TVisitorOption; Const ObjectClassName : String) : String;
    // Auxiliary routines
    procedure WriteFieldAssign(Strings: TStrings; F: TFieldPropDef);
    procedure WriteAssignToParam(Strings: TStrings; F: TFieldPropDef);
<<<<<<< HEAD
<<<<<<< HEAD
    procedure WriteReadWriteOverride(Strings: TStrings; const AAMethod, AVisitorGroup: String);
    procedure WriteRegisterVisitorLine(Strings: TStrings;
      const V: TVisitorOption; const ObjectClassName: String);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure WriteSetSQL(Strings: TStrings; const ASQL: String);
    procedure WriteSQLConstants(Strings: TStrings);
    Procedure WriteTerminateVisitor(Strings : TStrings; V : TVisitorOption; const ObjectClassName: String);
    procedure WriteSetupParams(Strings: TStrings; const AClassName, ObjectClassName: String);
    // Visitors
    procedure WriteCommonSetupVisitor(Strings: TStrings; const ObjectClassName: String);
    procedure WriteSaveVisitor(Strings: TStrings; const ObjectClassName: String);
    procedure WriteCreateVisitor(Strings: TStrings; const ObjectClassName: String);
    procedure WriteDeleteVisitor(Strings: TStrings; const ObjectClassName: String);
    procedure WriteUpdateVisitor(Strings: TStrings; const ObjectClassName: String);
    procedure WriteReadListVisitor(Strings: TStrings; const ObjectClassName: String);
    procedure WriteReadVisitor(Strings: TStrings; const ObjectClassName: String );
    procedure WriteVisitorDeclaration(Strings: TStrings; V: TVisitorOption; const ObjectClassName: String);
    procedure WriteVisitorImplementation(Strings: TStrings; V: TVisitorOption; const ObjectClassName: String);
<<<<<<< HEAD
<<<<<<< HEAD
    procedure WriteVisitorRegistration(Strings: TStrings; const ObjectClassName: String);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Protected
    // Not to be overridden.
    procedure WriteListAddObject(Strings: TStrings; const ListClassName, ObjectClassName: String);
    // Overrides of parent objects
<<<<<<< HEAD
<<<<<<< HEAD
    Function CreateFieldPropDefs : TFieldPropDefs; override;
    function AllowPropertyDeclaration(F: TFieldPropDef; AVisibility: TVisibilities): Boolean; override;
    Function GetInterfaceUsesClause : string; override;
    procedure WriteVisibilityStart(V: TVisibility; Strings: TStrings); override;
    Procedure DoGenerateInterface(Strings: TStrings); override;
    Procedure DoGenerateImplementation(Strings: TStrings); override;
    procedure CreateImplementation(Strings: TStrings); override;
=======
=======
>>>>>>> origin/fixes_2_2
    function AllowPropertyDeclaration(F: TFieldPropDef; AVisibility: TVisibilities): Boolean; override;
    Function GetInterfaceUsesClause : string; override;
    Procedure DoGenerateInterface(Strings: TStrings); override;
    Procedure DoGenerateImplementation(Strings: TStrings); override;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Function NeedsConstructor : Boolean; override;
    Function NeedsDestructor : Boolean; override;
    Class Function NeedsFieldDefs : Boolean; override;
    Function CreateOptions : TCodeGeneratorOptions; override;
    //
    //  New methods
    //
    // Override to add declarations to list declaration
    procedure DoCreateListDeclaration(Strings: TStrings; const ObjectClassName, ListClassName, ListAncestorName: String); virtual;
  Public
    procedure CreateListDeclaration(Strings: TStrings; const ObjectClassName, ListClassName, ListAncestorName: String);
    Property TiOPFOptions : TTiOPFCodeOptions Read GetOpt;
  end;

Const
  SOID = 'OID';              // OID property.
  SDefTableName = 'MYTABLE'; // Default table name.
  
implementation

Function StripType(S : String) : string;

begin
  Result:=S;
  If (Result<>'') and (Result[1]='T') then
    Delete(Result,1,1);
end;

<<<<<<< HEAD
<<<<<<< HEAD
{ TTiOPFFieldPropDef }

constructor TTiOPFFieldPropDef.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  PropSetters:=[psWrite];
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ TTiOPFCodeOptions }

function TTiOPFCodeOptions.GetListClassName: String;
begin
  Result:=FListClassName;
  If (Result='') then
    Result:=ObjectClassName+'List';
end;

procedure TTiOPFCodeOptions.SetClassOptions(const AValue: TClassOptions);

Var
  B : Boolean;

begin
  If AValue=FClassOptions then
    Exit;
  B:=Not(caCreateList in FClassOptions) and (caCreateList in AValue);
  FClassOptions:=AValue;
  If B then
    Include(FVisitorOptions,voReadList);
end;

procedure TTiOPFCodeOptions.SetListAncestorName(const AValue: String);
begin
  CheckIdentifier(AValue,False);
  FListAncestorName:=AValue;
end;

procedure TTiOPFCodeOptions.SetListClassName(const AValue: String);
begin
  CheckIdentifier(AValue,True);
  FListClassName:=AValue;
end;

procedure TTiOPFCodeOptions.SetVisitorOptions(const AValue: TVisitorOptions);

Var
  V : TVisitorOption;

begin
  FVisitorOptions:=AValue;
  // Consistency check
  If voSingleSaveVisitor in FVisitorOptions then
    begin
    Exclude(FVisitorOptions,voCommonSetupParams);
    Exclude(FVisitorOptions,voCreate);
    Exclude(FVisitorOptions,voUpdate);
    Exclude(FVisitorOptions,voDelete);
    end
  else If voCommonSetupParams in FVisitorOptions then
    begin
    Include(FVisitorOptions,voCreate);
    Include(FVisitorOptions,voUpdate);
    end;
  For V:=Low(TVisitorOption) to High(TVisitorOption) do
    If Not (V in FVisitorOptions) then
      Exclude(FFinalVisitors,V);
end;

constructor TTiOPFCodeOptions.Create;
begin
  inherited Create;
  FListAncestorName:='TtiObjectList';
  AncestorClass:='TtiObject';
  ObjectClassName:='MyObject';
  TableName:=SDefTableName;
  FVisitorOptions:=[voRead,voCreate,voDelete,voUpdate];
  FClassOptions:=[caCreateClass,caCreateList,caListAddMethod,caListItemsProperty];
end;

procedure TTiOPFCodeOptions.Assign(ASource: TPersistent);

Var
  OC : TTiOPFCodeOptions;
  
begin
  If ASource is TTiOPFCodeOptions then
    begin
    OC:=ASource as TTiOPFCodeOptions;
    FListAncestorName:=OC.FListAncestorName;
    AncestorClass:=OC.AncestorClass;
    FVisitorOptions:=OC.FVisitorOptions;
    FClassOptions:=OC.FClassOptions;
    FTableName:=OC.TableName;
    FFinalVisitors:=OC.FinalVisitors;
    end;
  inherited Assign(ASource);
end;

{ TTiOPFCodeGenerator }

{ ---------------------------------------------------------------------
  General overrides
  ---------------------------------------------------------------------}

function TTiOPFCodeGenerator.NeedsConstructor: Boolean;
begin
  Result:=inherited NeedsConstructor;
  Result:=Result or (caConstructor in TiOPFOptions.ClassOptions);
end;

function TTiOPFCodeGenerator.NeedsDestructor: Boolean;
begin
  Result:=inherited NeedsDestructor;
  Result:=Result or (caDestructor in TiOPFOptions.ClassOptions);
end;

class function TTiOPFCodeGenerator.NeedsFieldDefs: Boolean;
begin
  Result:=True;
end;

function TTiOPFCodeGenerator.CreateOptions: TCodeGeneratorOptions;
begin
  Result:=TTiOPFCodeOptions.Create;
end;
function TTiOPFCodeGenerator.GetOpt: TTiOPFCodeOptions;
begin
  Result:=CodeOptions as TTiOPFCodeOptions;
end;

function TTiOPFCodeGenerator.UseCommonSetupParams: Boolean;
begin
  Result:=VoCommonSetupParams in tiOPFOptions.VisitorOptions;
end;

function TTiOPFCodeGenerator.SingleSaveVisitor: Boolean;
begin
  Result:=voSingleSaveVisitor in tiOPFOptions.VisitorOptions;
end;

function TTiOPFCodeGenerator.VisitorClassName(V: TVisitorOption;
  const ObjectClassName: String): String;

Var
  S : String;

begin
  Case V of
    voRead        : S:='Read';
    voReadList    : S:='ReadList';
    voCreate      : S:='Create';
    voDelete      : S:='Delete';
    voUpdate      : S:='Update';
    voCommonSetupParams : S:='UpdateCreate';
    voSingleSaveVisitor : S:='Save';
  else
    Result:='Unknown';
  end;
  // Real class name
  Result:=Format('T%s%sVisitor',[S,StripType(ObjectClassName)]);
end;

procedure TTiOPFCodeGenerator.WriteCommonSetupVisitor(Strings: TStrings;
  const ObjectClassName: String);


Var
  CS,C,S : String;
  I : Integer;

begin
  C:=VisitorClassName(voCommonSetupParams,ObjectClassName);
  Addln(Strings,'{ %s }',[C]);
  Addln(Strings);
  WriteSetupParams(Strings,C,ObjectClassName);
end;

procedure TTiOPFCodeGenerator.WriteSaveVisitor(Strings: TStrings; const ObjectClassName: String);

  Procedure WriteSQLCase(Const ACaselabel,ASQL : String);

  begin
    addln(Strings,ACaseLabel+':');
    incIndent;
    WriteSetSQL(Strings,ASQL);
    DecIndent;
  end;

Var
  OCN,CS,C,S : String;
  I : Integer;
  F : TFieldPropDef;

begin
  OCN:=StripType(ObjectClassName);
  C:=VisitorClassName(voSingleSaveVisitor,OCN);
  Addln(Strings,'{ %s }',[C]);
  Addln(Strings);
  // Init
  S:=BeginInit(Strings,C);
  AddLn(Strings,'Case Visited.ObjectState of');
  IncIndent;
  try
    WriteSQLCase('posCreate',Format('SQLCreate%s',[OCN]));
    WriteSQLCase('posUpdate',Format('SQLUpdate%s',[OCN]));
    WriteSQLCase('posDelete',Format('SQLDelete%s',[OCN]));
  finally
    DecIndent;
  end;
  Addln(Strings,'end;');
  DecIndent;
  EndMethod(Strings,S);
  // AcceptVisitor
  S:=BeginAcceptVisitor(Strings,C,ObjectClassName);
<<<<<<< HEAD
<<<<<<< HEAD
  AddLn(Strings,'Result:=Result and (Visited.ObjectState in [posCreate,posDelete,posUpdate]);');
=======
  AddLn(Strings,'Result:=Result and (Visited.ObjectState in [posCreate,posdelete,posUpdate]);');
>>>>>>> graemeg/fixes_2_2
=======
  AddLn(Strings,'Result:=Result and (Visited.ObjectState in [posCreate,posdelete,posUpdate]);');
>>>>>>> origin/fixes_2_2
  DecIndent;
  EndMethod(Strings,S);
  S:=BeginSetupParams(Strings,C,ObjectClassName,True);
  Addln(Strings,'With Query do',[ObjectClassName]);
  IncINdent;
  try
    Addln(Strings,'begin');
    F:=Fields.FindPropName('OID');
    If (F<>Nil) then
      WriteAssignToParam(Strings,F)
    else
      AddLn(Strings,'// No OID property found. Add delete key parameter setup code here.');
    AddLn(Strings,'If (Visited.ObjectState<>posDelete) then');
    IncIndent;
    try
      AddLn(Strings,'begin');
      For I:=0 to Fields.Count-1 do
        If Fields[i].Enabled and (CompareText(Fields[i].PropertyName,'OID')<>0) then
          WriteAssignToParam(Strings,Fields[i]);
      AddLn(Strings,'end;');
    Finally
      DecIndent;
    end;
    Addln(Strings,'end;');
  finally
    DecIndent;
  end;
  DecIndent;
  EndMethod(Strings,S);
end;

function TTiOPFCodeGenerator.GetInterfaceUsesClause: string;
begin
  Result:=inherited GetInterfaceUsesClause;
  If (Result<>'') then
    Result:=Result+', ';
  Result:=Result+'tiVisitor, tiVisitorDB, tiObject';
<<<<<<< HEAD
<<<<<<< HEAD
  If (voRegisterVisitors in tiOPFoptions.VisitorOptions)
     or ([caOverrideRead,caOverrideReadThis,caOverrideSave]*tiOPFOptions.ClassOptions<>[]) then
    Result:=Result+', tiOPFManager';
end;


=======
end;

>>>>>>> graemeg/fixes_2_2
=======
end;

>>>>>>> origin/fixes_2_2
procedure TTiOPFCodeGenerator.DoGenerateInterface(Strings: TStrings);

Var
  V : TVisitorOption;

begin
  If (caCreateClass in TiOPFOptions.ClassOptions) then
    inherited DoGenerateInterface(Strings)
  else
    begin
    Addln(Strings,'Type');
    Addln(Strings);
    end;
  With TiOPFOptions do
    begin
    IncIndent;
    try
      If caCreateList in ClassOptions then
        CreateListDeclaration(Strings,ObjectClassName,ListClassName,ListAncestorName);
      If voCommonSetupParams in VisitorOptions then
        WriteVisitorDeclaration(Strings,voCommonSetupParams,ObjectClassName);
      For V:=Low(TVisitorOption) to High(TVisitorOption) do
        If (V in VisitorOptions) and (V<>voCommonSetupParams) then
          WriteVisitorDeclaration(Strings,V,ObjectClassName);
    Finally
      DecIndent;
    end;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
  If voRegisterVisitors in tiOPFoptions.VisitorOptions then
    begin
    AddLn(Strings);
    AddLn(Strings,'Procedure Register'+tiOPFoptions.ObjectClassName+'Visitors;');
    AddLn(Strings);
    end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


procedure TTiOPFCodeGenerator.WriteVisitorDeclaration(Strings : TStrings; V : TVisitorOption; Const ObjectClassName : String);

Var
  S,T,A : string;

begin
  // Ancestor name
  // Common setup case
  If (V in [voCreate,voUpdate]) and (UseCommonSetupParams) then
    A:=Format('TUpdateCreate%sVisitor',[StripType(ObjectClassName)])
<<<<<<< HEAD
<<<<<<< HEAD
  else If (V in [voCreate,voDelete,voUpdate,voCommonSetupParams,voSingleSaveVisitor]) then
=======
  else If (V in [voCreate,voDelete,voUpdate,voCommonSetupParams]) then
>>>>>>> graemeg/fixes_2_2
=======
  else If (V in [voCreate,voDelete,voUpdate,voCommonSetupParams]) then
>>>>>>> origin/fixes_2_2
    A:='TtiVisitorUpdate'
  else
    A:='TtiVisitorSelect';
  // Real class
  S:=VisitorClassName(V,ObjectClassName);
  AddLn(Strings,'{ %s }',[S]);
  AddlN(Strings,'%s = Class(%s)',[S,A]);
  AddlN(Strings,'Protected');
  IncIndent;
  Try
    If (V<>VoCommonSetupParams) then
      begin
      AddLn(Strings,'Procedure Init; override;');
      AddLn(Strings,'Function AcceptVisitor : Boolean; override;');
      If Not ((V in [voCreate,voUpdate]) and UseCommonSetupParams) then
        AddLn(Strings,'Procedure SetupParams; override;');
      end
    else
      AddLn(Strings,'Procedure SetupParams; override;');
    If (V in [voRead,voReadList]) then
      AddLn(Strings,'Procedure MapRowToObject; override;');
    if (V in TiOPFOptions.FinalVisitors) then
      Addln(Strings,'Procedure Execute(Const AData : TtiVisited); override;');
  Finally
    DecIndent;
  end;
  AddlN(Strings,'end;');
  AddlN(Strings);
end;

Function TTiOPFCodeGenerator.CreateSQLStatement(V : TVisitorOption) : String;

  Function AddToS(Const S,Add : String) : string;
  
  begin
    Result:=S;
    If (Result<>'') then
      Result:=Result+', ';
    Result:=Result+Add;
  end;

Var
  I : integer;
  W,S,VS,TN : String;
  F : TFieldPropDef;

begin
  TN:=TiOPFOptions.TableName;
  If (TN='') then 
    TN:=SDefTableName;
  S:='';
  VS:='';
  W:='Your condition here';
  Result:='';
  Case V of
   voRead,
   voReadList : begin
                Result:='SELECT ';
                For I:=0 to Fields.Count-1 do
                  begin
                  F:=Fields[i];
                  If F.Enabled then
                    begin
                    S:=AddToS(S,F.FieldName);
                    If (V=voRead) and (F.PropertyName=SOID) then
                      W:=Format('%s = :%s',[F.FieldName,F.FieldName]);
                    end;
                  end;
                Result:=Result+S+Format(' FROM %s WHERE (%s);',[TN,W]);
                end;
   voCreate : begin
              Result:=Format('INSERT INTO %s (',[TN]);
              For I:=0 to Fields.Count-1 do
                begin
                F:=Fields[i];
                If F.Enabled then
                  begin
                  S:=AddToS(S,F.FieldName);
                  VS:=AddToS(VS,':'+F.FieldName);
                  end;
                end;
              Result:=Result+S+') VALUES ('+VS+');';
              end;
   voDelete : begin
              For I:=0 to Fields.Count-1 do
                begin
                F:=Fields[i];
                If (F.PropertyName=SOID) then
                  W:=Format('%s = :%s',[F.FieldName,F.FieldName]);
                end;
              Result:=Format('DELETE FROM %s WHERE (%s);',[TN,W]);
              end;
   voUpdate : begin
              Result:=Format('UPDATE %s SET ',[TN]);
              For I:=0 to Fields.Count-1 do
                 begin
                  F:=Fields[i];
                  If F.Enabled then
                    If (F.PropertyName=SOID) then
                      W:=Format('%s = :%s',[F.FieldName,F.FieldName])
                    else
                      S:=AddToS(S,F.FieldName+' = :'+F.FieldName);
                  end;
              Result:=Result+S+Format(' WHERE (%s);',[W]);
              end;
  end;
end;

procedure TTiOPFCodeGenerator.WriteSQLConstants(Strings : TStrings);

Const
  VisSQL : Array [TVisitorOption] of string
<<<<<<< HEAD
<<<<<<< HEAD
         = ('Read','ReadList','Create','Delete','Update','','','');
=======
         = ('Read','ReadList','Create','Delete','Update','','');
>>>>>>> graemeg/fixes_2_2
=======
         = ('Read','ReadList','Create','Delete','Update','','');
>>>>>>> origin/fixes_2_2

Var
  OCN,S : String;
  V : TVisitorOption;

begin
  AddLn(Strings,'Const');
  IncIndent;
  try
    OCN:=StripType(TiOPFOptions.ObjectClassName);
    For V:=Low(TVisitorOption) to High(TVisitorOption) do
      If ((V in TiOPFOptions.VisitorOptions) or
           (SingleSaveVisitor and (V in [voCreate,voUpdate,voDelete]))) and (VisSQL[V]<>'') then
        begin
        S:=CreateSQLStatement(V);
        S:=Format('SQL%s%s = ''%s'';',[VisSQL[V],OCN,S]);
        AddLn(Strings,S);
        end;
  finally
    DecIndent;
  end;
  AddLn(Strings,'');
end;

procedure TTiOPFCodeGenerator.WriteTerminateVisitor(Strings  : TStrings;V : TVisitorOption;
  const ObjectClassName: String);

Var
  S  : String;
begin
  S:=VisitorclassName(V,ObjectClassName);
  S:=Format('Procedure %s.Execute(Const AData : TtiVisited);',[S]);
  BeginMethod(Strings,S);
  AddLn(Strings,'begin');
  IncIndent;
  try
    AddLn(Strings,'Inherited Execute(AData);');
    Addln(Strings,'If not AcceptVisitor then');
    IncIndent;
    Try
      Addln(Strings,'Exit; // ==>');
    Finally
      DecIndent;
    end;
    AddLn(Strings,'ContinueVisiting:=False;');
  Finally
    DecIndent;
  end;
  EndMethod(Strings,S);
end;


procedure TTiOPFCodeGenerator.DoGenerateImplementation(Strings: TStrings);

Var
  V : TVisitorOption;

begin
  If (caCreateClass in TiOPFOptions.ClassOptions) then
    inherited DoGenerateImplementation(Strings);
  With TiOPFOptions do
    begin
    If (VisitorOptions<>[])   then
      WriteSQLConstants(Strings);
    If caCreateList in ClassOptions then
      CreateListImplementation(Strings,ObjectClassName,ListClassName);
    For V:=Low(TVisitorOption) to High(TVisitorOption) do
      If V in VisitorOptions then
        WriteVisitorImplementation(Strings,V,ObjectClassName);
<<<<<<< HEAD
<<<<<<< HEAD
    If (voRegisterVisitors in TiOPFOptions.VisitorOptions) then
      WriteVisitorRegistration(Strings,ObjectClassName);
    end;
end;

{ ---------------------------------------------------------------------
  Override read/write/readthis
  ---------------------------------------------------------------------}

procedure TTiOPFCodeGenerator.WriteVisibilityStart(V: TVisibility;
  Strings: TStrings);
begin
  Inherited;
  If (V=vPublic) then
    begin
    if (caOverrideSave in TiOPFOptions.ClassOptions) then
      AddLn(Strings,'Procedure Save; override;');
    If (caOverrideRead in TiOPFOptions.ClassOptions) then
      AddLn(Strings,'Procedure Read; override;');
    If (caOverrideReadThis in TiOPFOptions.ClassOptions) then
      AddLn(Strings,'Procedure ReadThis; override;');
    end;
end;

procedure TTiOPFCodeGenerator.CreateImplementation(Strings: TStrings);

begin
  inherited CreateImplementation(Strings);
  if (caOverrideSave in TiOPFOptions.ClassOptions) then
    WriteReadWriteOverride(Strings,'Save','Save');
  If (caOverrideRead in TiOPFOptions.ClassOptions) then
    WriteReadWriteOverride(Strings,'Read','Read');
  If (caOverrideReadThis in TiOPFOptions.ClassOptions) then
    WriteReadWriteOverride(Strings,'ReadThis','Read');
end;

procedure TTiOPFCodeGenerator.WriteReadWriteOverride(Strings : TStrings; Const AAMethod,AVisitorGroup : String);

Const
  SExecVisitor = 'GTIOPFManager.VisitorManager.Execute(''%s_%s'',Self);';

Var
  OCN,S: String;

begin
  OCN:=TiOPFOptions.ObjectClassName;
  S:=Format('Procedure %s.%s;',[OCN,AAMethod]);
  BeginMethod(Strings,S);
  AddLn(Strings,'begin');
  IncIndent;
  S:=Format(SExecVisitor,[OCN,AVisitorGroup]);
  AddLn(Strings,S);
  DecIndent;
  EndMethod(Strings,S);
end;

=======
    end;
end;

>>>>>>> graemeg/fixes_2_2
=======
    end;
end;

>>>>>>> origin/fixes_2_2
{ ---------------------------------------------------------------------
  Visitor helper routines
  ---------------------------------------------------------------------}

procedure TTiOPFCodeGenerator.WriteVisitorImplementation(Strings : TStrings; V : TVisitorOption; Const ObjectClassName : String);

begin
  Case V of
    voRead              : WriteReadVisitor(Strings,ObjectClassName);
    voReadList          : WriteReadListVisitor(Strings,ObjectClassName);
    voCreate            : WriteCreateVisitor(Strings,ObjectClassName);
    voDelete            : WriteDeleteVisitor(Strings,ObjectClassName);
    voUpdate            : WriteUpdateVisitor(Strings,ObjectClassName);
    voCommonSetupParams : WriteCommonSetupVisitor(Strings,ObjectClassName);
    voSingleSaveVisitor : WriteSaveVisitor(Strings,ObjectClassName);
  end;
  If v in TiOPFOptions.FinalVisitors then
    WriteTerminateVisitor(Strings,V,ObjectClassName);
end;

<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Function TTiOPFCodeGenerator.BeginInit(Strings : TStrings; const AClass : String) : String;

begin
  Result:=Format('Procedure %s.Init;',[AClass]);
  BeginMethod(Strings,Result);
  AddLn(Strings,'begin');
  IncIndent;
end;

Function TTiOPFCodeGenerator.BeginAcceptVisitor(Strings : TStrings; Const AClass, ObjectClassName: String) : String;

begin
  Result:=Format('Function %s.AcceptVisitor : Boolean;',[AClass]);
  BeginMethod(Strings,Result);
  AddLn(Strings,'begin');
  IncIndent;
  AddLn(Strings,'Result:=Visited is %s;',[ObjectClassName]);
end;

Function TTiOPFCodeGenerator.BeginSetupParams(Strings : TStrings; const AClass,ObjectClassName : String; DeclareObject : Boolean) : String;

begin
  Result:=Format('Procedure %s.SetupParams;',[AClass]);
  BeginMethod(Strings,Result);
  If DeclareObject Then
    DeclareObjectVariable(Strings,ObjectClassName);
  AddLn(Strings,'begin');
  IncIndent;
  If DeclareObject Then
    Addln(Strings,'O:=%s(Visited);',[ObjectClassName]);
end;

Procedure TTiOPFCodeGenerator.DeclareObjectvariable(Strings : TStrings; Const ObjectClassName : String);

begin
  AddLn(Strings,'var');
  IncIndent;
  try
    AddLn(Strings,'O : %s;',[ObjectClassName]);
    AddLn(Strings);
  finally
    DecIndent;
  end;
end;

Function TTiOPFCodeGenerator.BeginMapRowToObject(Strings : TStrings; Const AClass,ObjectClassName : String) : String;

begin
  Result:=Format('Procedure %s.MapRowToObject;',[AClass]);
  BeginMethod(Strings,Result);
  DeclareObjectVariable(Strings,ObjectClassName);
  AddLn(Strings,'begin');
  IncIndent;
end;

{ ---------------------------------------------------------------------
<<<<<<< HEAD
<<<<<<< HEAD
  Visitor registration
  ---------------------------------------------------------------------}

procedure TTiOPFCodeGenerator.WriteRegisterVisitorLine(Strings : TStrings; Const V: TVisitorOption; Const ObjectClassName : String);

Var
  C : String;
  S : String;

begin
  C:=VisitorClassName(v,ObjectClassName);
  Case V of
    voRead        : S:='Read';
    voReadList    : S:='ReadList';
    voCreate      : S:='Save';
    voDelete      : S:='Save';
    voUpdate      : S:='Save';
  end;
  S:=ObjectClassName+'_'+S;
  S:=Format('GTIOPFManager.RegisterVisitor(''%s'',%s);',[S,C]);
  AddLn(Strings,S);
end;

procedure TTiOPFCodeGenerator.WriteVisitorRegistration(Strings : TStrings; Const ObjectClassName : String);

Const
  RealVis = [voRead,voReadList,voCreate,voDelete,voUpdate];

Var
  v : TVisitorOption;
  S : String;

begin
  Addln(Strings);
  S:='Procedure Register'+ObjectClassName+'Visitors;';
  BeginMethod(Strings,S);
  AddLn(Strings,'begin');
  IncIndent;
  For v:=Low(TVisitorOption) to High(TVisitorOption) do
    begin
    If (V in RealVis) and (V in TiOPFOptions.VisitorOptions) then
      WriteRegisterVisitorLine(Strings,V,ObjectClassName);
    end;
  DecIndent;
  EndMethod(Strings,S);
end;

{ ---------------------------------------------------------------------
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Read Visitor
  ---------------------------------------------------------------------}

procedure TTiOPFCodeGenerator.WriteReadVisitor(Strings : TStrings; Const ObjectClassName : String);

Var
  OCN,CS,C,S : String;
  I : Integer;
  F : TFieldPropDef;

begin
  OCN:=StripType(ObjectClassName);
  CS:=Format('SQLRead%s',[OCN]);
  C:=VisitorClassName(voRead,OCN);
  Addln(Strings,'{ %s }',[C]);
  Addln(Strings);
  // Init
  S:=BeginInit(Strings,C);
  WriteSetSQL(Strings,CS);
  DecIndent;
  EndMethod(Strings,S);
  // AcceptVisitor
  S:=BeginAcceptVisitor(Strings,C,ObjectClassName);
  DecIndent;
  EndMethod(Strings,S);
  // AcceptSetupParams
  F:=Fields.FindPropName('OID');
  S:=BeginSetupParams(Strings,C,ObjectClassName,F<>Nil);
  If (F<>Nil) then
    WriteAssignToParam(Strings,F)
  else
    AddLn(Strings,'// Set up as needed');
  DecIndent;
  EndMethod(Strings,S);
  // MapRowToObject
  S:=BeginMapRowToObject(Strings,C,ObjectClassName);
  Addln(Strings,'O:=%s(Visited);',[ObjectClassName]);
  Addln(Strings,'With Query do',[ObjectClassName]);
  IncINdent;
  try
    Addln(Strings,'begin');
    For I:=0 to Fields.Count-1 do
      If Fields[i].Enabled then
        WriteFieldAssign(Strings,Fields[i]);
    Addln(Strings,'end;');
  finally
    DecIndent;
  end;
  DecIndent;
  EndMethod(Strings,S);
end;

procedure TTiOPFCodeGenerator.WriteFieldAssign(Strings : TStrings; F : TFieldPropDef);

Var
  PN,FN,SFN,R,S : String;

begin
  PN:=F.PropertyName;
  FN:=F.FieldName;
  SFN:=CreateString(FN);
  If (PN=SOID) then
    R:=Format('O.OID.AssignFromTIQuery(''%s'',Query);',[FN])
  else
    Case F.PropertyType of
      ptBoolean :
        S:='AsBoolean';
      ptShortint, ptByte,
      ptSmallInt, ptWord,
      ptLongint, ptCardinal,
      ptInt64:
        S:='AsInteger';
      ptShortString, ptAnsiString, ptWideString :
        S:='AsString';
      ptSingle, ptDouble, ptExtended, ptComp :
        S:='AsFloat';
      ptCurrency :
<<<<<<< HEAD
<<<<<<< HEAD
        S:='AsFloat';
=======
        S:='AsCurrency';
>>>>>>> graemeg/fixes_2_2
=======
        S:='AsCurrency';
>>>>>>> origin/fixes_2_2
      ptDateTime :
        S:='AsDateTime';
      ptEnumerated :
        R:=Format('Integer(O.%s):=FieldAsInteger[%s];',[PN,SFN]);
      ptSet :
        S:=Format('// Add custom set loading code here for %s from %s',[PN,FN]);
      ptStream :
        R:=Format('AssignFieldAsStream(%s,O.%s);',[SFN,PN]);
      ptTStrings :
        R:=Format('O.%s.Text:=FieldAsString[%s];',[PN,SFN]);
      ptCustom :
        R:=Format('// Add custom loading code here for %s from %s',[PN,FN]);
    end;
  If (S<>'') then
    R:=Format('O.%s:=Field%s[%s];',[PN,S,SFN]);
  AddLn(Strings,R);
end;

procedure TTiOPFCodeGenerator.WriteAssignToParam(Strings : TStrings; F : TFieldPropDef);

Var
  PN,FN,SFN,R,S : String;

begin
  PN:=F.PropertyName;
  FN:=F.FieldName;
  SFN:=CreateString(FN);
  If (PN=SOID) then
    R:=Format('O.OID.AssignToTIQuery(''%s'',Query);',[FN])
  else
    Case F.PropertyType of
      ptBoolean :
        S:='AsBoolean';
      ptShortint, ptByte,
      ptSmallInt, ptWord,
      ptLongint, ptCardinal,
      ptInt64, ptQWord :
        S:='AsInteger';
      ptShortString, ptAnsiString, ptWideString :
        S:='AsString';
      ptSingle, ptDouble, ptExtended, ptComp :
        S:='AsFloat';
      ptCurrency :
        S:='AsCurrency';
      ptDateTime :
        S:='AsDateTime';
      ptEnumerated :
        R:=Format('ParamAsInteger[%s]:=Integer(O.%s);',[SFN,PN]);
      ptSet :
        S:=Format('// Add custom set loading code here for %s from %s',[PN,FN]);
      ptStream :
        R:=Format('AssignParamFromStream(%s,O.%s);',[SFN,PN]);
      ptTStrings :
        R:=Format('ParamAsString[%s]:=O.%s.Text;',[SFN,PN]);
      ptCustom :
        R:=Format('// Add custom loading code here for %s from %s',[PN,FN]);
    end;
  If (S<>'') then
    R:=Format('Param%s[%s]:=O.%s;',[S,SFN,PN]);
  AddLn(Strings,R);
end;

{ ---------------------------------------------------------------------
  List Read Visitor
  ---------------------------------------------------------------------}

procedure TTiOPFCodeGenerator.WriteReadListVisitor(Strings : TStrings; Const ObjectClassName : String);

Var
  OCN,CS,C,S,LN : String;
  I : Integer;

begin
  LN:=tiOPFOptions.ListClassName;
  OCN:=StripType(ObjectClassName);
  CS:=Format('SQLReadList%s',[OCN]);
  C:=VisitorClassName(voReadList,OCN);
  Addln(Strings,'{ %s }',[C]);
  Addln(Strings);
  // Init
  S:=BeginInit(Strings,C);
  WriteSetSQL(Strings,CS);
  DecIndent;
  EndMethod(Strings,C);
  // AcceptVisitor
  S:=BeginAcceptVisitor(Strings,C,LN);
  DecIndent;
  EndMethod(Strings,S);
  // AcceptSetupParams
  S:=BeginSetupParams(Strings,C,'',False);
  DecIndent;
  EndMethod(Strings,S);
  // MapRowToObject
  S:=BeginMapRowToObject(Strings,C,ObjectClassName);
  Addln(Strings,'O:=%s.Create;',[ObjectClassName]);
  Addln(Strings,'With Query do',[ObjectClassName]);
  IncINdent;
  try
    Addln(Strings,'begin');
    For I:=0 to Fields.Count-1 do
      If Fields[i].Enabled then
        WriteFieldAssign(Strings,Fields[i]);
    Addln(Strings,'end;');
  finally
    DecIndent;
  end;
  Addln(Strings,'O.ObjectState:=posClean;');
  Addln(Strings,'%s(Visited).Add(O);',[LN]);
  DecIndent;
  EndMethod(Strings,S);
end;

{ ---------------------------------------------------------------------
  Create Visitor
  ---------------------------------------------------------------------}

procedure TTiOPFCodeGenerator.WriteCreateVisitor(Strings : TStrings; Const ObjectClassName : String);


Var
  OCN,CS,C,S : String;
  I : Integer;

begin
  OCN:=StripType(ObjectClassName);
  CS:=Format('SQLCreate%s',[OCN]);
  C:=VisitorClassName(voCreate,OCN);
  Addln(Strings,'{ %s }',[C]);
  Addln(Strings);
  // Init
  S:=BeginInit(Strings,C);
  WriteSetSQL(Strings,CS);
  DecIndent;
  EndMethod(Strings,S);
  // AcceptVisitor
  S:=BeginAcceptVisitor(Strings,C,ObjectClassName);
  AddLn(Strings,'Result:=Result and (Visited.ObjectState=posCreate);');
  DecIndent;
  EndMethod(Strings,S);
  If Not (UseCommonSetupParams) then
    WriteSetupParams(Strings,C,ObjectClassName);
end;

procedure TTiOPFCodeGenerator.WriteSetupParams(Strings : TStrings; Const AClassName,ObjectClassName : String);

Var
  S : String;
  I : Integer;

begin
  // SetupParams
  S:=BeginSetupParams(Strings,AClassName,ObjectClassName,True);
  Addln(Strings,'With Query do',[ObjectClassName]);
  IncINdent;
  try
    Addln(Strings,'begin');
    For I:=0 to Fields.Count-1 do
      If Fields[i].Enabled then
        WriteAssignToParam(Strings,Fields[i]);
    Addln(Strings,'end;');
  finally
    DecIndent;
  end;
  DecIndent;
  EndMethod(Strings,S);
end;

procedure TTiOPFCodeGenerator.WriteSetSQL(Strings : TStrings; Const ASQL : String);

begin
  Addln(Strings,Format('Query.SQLText:=%s;',[ASQL]));
end;

procedure TTiOPFCodeGenerator.WriteDeleteVisitor(Strings : TStrings; Const ObjectClassName : String);

Var
  OCN,CS, C,S : String;
  F : TFieldPropDef;
  
begin
  OCN:=StripType(ObjectClassName);
  CS:=Format('SQLDelete%s',[OCN]);
  C:=VisitorClassName(voDelete,OCN);
  Addln(Strings,'{ %s }',[C]);
  // Init
  S:=BeginInit(Strings,C);
  WriteSetSQL(Strings,CS);
  DecIndent;
  EndMethod(Strings,S);
  // AcceptVisitor
  S:=BeginAcceptVisitor(Strings,C,ObjectClassName);
  AddLn(Strings,'Result:=Result and (Visited.ObjectState=posDelete);');
  DecIndent;
  EndMethod(Strings,S);
  // SetupParams
  S:=BeginSetupParams(Strings,C,ObjectClassName,True);
  F:=Fields.FindPropName('OID');
  If (F<>Nil) then
    WriteAssignToParam(Strings,F)
  else
    AddLn(Strings,'// Add parameter setup code here ');
  DecIndent;
  EndMethod(Strings,S);
end;

procedure TTiOPFCodeGenerator.WriteUpdateVisitor(Strings : TStrings; Const ObjectClassName : String);

Var
  OCN,CS,C,S : String;
  I : Integer;

begin
  OCN:=StripType(ObjectClassName);
  CS:=Format('SQLUpdate%s',[OCN]);
  C:=VisitorClassName(voUpdate,OCN);
  Addln(Strings,'{ %s }',[C]);
  Addln(Strings);
  // Init
  S:=BeginInit(Strings,C);
  WriteSetSQl(Strings,CS);
  DecIndent;
  EndMethod(Strings,S);
  // AcceptVisitor
  S:=BeginAcceptVisitor(Strings,C,ObjectClassName);
  AddLn(Strings,'Result:=Result and (Visited.ObjectState=posUpdate);');
  DecIndent;
  EndMethod(Strings,S);
  If Not (UseCommonSetupParams) then
    WriteSetupParams(Strings,C,ObjectClassName);
end;


{ ---------------------------------------------------------------------
  List object commands
  ---------------------------------------------------------------------}

procedure TTiOPFCodeGenerator.DoCreateListDeclaration(Strings: TStrings;
  const ObjectClassName, ListClassName, ListAncestorName: String);
begin
  If caListItemsProperty in tiOPFOptions.ClassOptions then
    begin
    AddLn(Strings,'Private');
    IncIndent;
    Try
      AddLn(Strings,'Function GetObj(AIndex : Integer) : %s;',[ObjectClassname]);
      AddLn(Strings,'Procedure SetObj(AIndex : Integer; AValue : %s);',[ObjectClassname]);
    Finally
      DecIndent;
    end;
    end;
  If (caListAddMethod in tiOPFOptions.ClassOptions) then
    begin
    AddLn(Strings,'Public');
    IncIndent;
    Try
      Addln(Strings,'Function Add(AnItem : %s) : Integer; reintroduce;',[ObjectClassName]);
    Finally
      DecIndent;
    end;
    end;
  If (caListItemsProperty in tiOPFOptions.ClassOptions) then
    begin
    If Not (caListAddMethod in tiOPFOptions.ClassOptions) then
      AddLn(Strings,'Public');
    IncIndent;
    Try
<<<<<<< HEAD
<<<<<<< HEAD
      AddLn(Strings,'Property Items[AIndex : Integer] : %s Read GetObj Write SetObj; Default;',[ObjectClassname]);
=======
      AddLn(Strings,'Property Items[Index : Integer] : %s Read GetObj Write SetObj; Default;',[ObjectClassname]);
>>>>>>> graemeg/fixes_2_2
=======
      AddLn(Strings,'Property Items[Index : Integer] : %s Read GetObj Write SetObj; Default;',[ObjectClassname]);
>>>>>>> origin/fixes_2_2
    Finally
      DecIndent;
    end;
    end;
end;

procedure TTiOPFCodeGenerator.CreateListDeclaration(Strings: TStrings;
  const ObjectClassName, ListClassName, ListAncestorName: String);
begin
  Addln(Strings);
  Addln(Strings,'{ %s }',[ListClassName]);
  Addln(Strings);
  Addln(Strings,'%s = Class(%s)',[ListClassName,ListAncestorName]);
  DoCreateListDeclaration(Strings,ObjectClassName,ListClassName,ListAncestorName);
  AddLn(Strings,'end;');
  Addln(Strings);
end;

procedure TTiOPFCodeGenerator.WriteListAddObject(Strings: TStrings;
  const ListClassName, ObjectClassName: String);
  
Var
  S : String;
  
begin
   S:=Format('Function %s.Add(AnItem : %s) : Integer;',[ListClassName,ObjectClassName]);
   BeginMethod(Strings,S);
   Addln(Strings,'begin');
   IncIndent;
   try
     Addln(Strings,'Result:=inherited Add(AnItem);');
   finally
     DecIndent;
   end;
   EndMethod(Strings,S);
   Addln(Strings);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TTiOPFCodeGenerator.CreateFieldPropDefs: TFieldPropDefs;
begin
  Result:=TFieldPropDefs.Create(TTiOPFFieldPropDef);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TTiOPFCodeGenerator.AllowPropertyDeclaration(F: TFieldPropDef;
  AVisibility: TVisibilities): Boolean;
begin
  If F.PropertyName=SOID then
    Result:=False
  else
    Result:=inherited AllowPropertyDeclaration(F, AVisibility);
end;


procedure TTiOPFCodeGenerator.CreateListImplementation(Strings: TStrings; const ObjectClassName, ListClassName: String);

Var
  S : String;

begin
  If caListItemsProperty in tiOPFOptions.ClassOptions then
    begin
    AddLn(Strings,'{ %s }',[ListClassName]);
    AddLn(Strings);
    S:=Format('Function %s.GetObj(AIndex : Integer) : %s;',[ListClassName,ObjectClassname]);
    BeginMethod(Strings,S);
    AddLn(Strings,'begin');
    IncIndent;
    try
      AddLn(Strings,'Result:=%s(Inherited Items[AIndex]);',[ObjectClassname]);
    finally
      DecIndent;
    end;
    EndMethod(Strings,S);
    Addln(Strings);
    S:=Format('Procedure %s.SetObj(AIndex : Integer; AValue : %s);',[ListClassName,ObjectClassname]);
    BeginMethod(Strings,S);
    AddLn(Strings,'begin');
    IncIndent;
    try
      AddLn(Strings,'Inherited Items[AIndex]:=AValue;');
    finally
      DecIndent;
    end;
    EndMethod(Strings,S);
    Addln(Strings);
    end;
  If (caListAddMethod in tiOPFOptions.ClassOptions) then
    WriteListAddObject(Strings,ListClassName,ObjectClassName);
end;

Initialization
<<<<<<< HEAD
<<<<<<< HEAD
  RegisterCodeGenerator('tiOPF','tiOPF classes and hard-coded visitors for the data',TTiOPFCodeGenerator);
=======
  RegisterCodeGenerator('tiOPF','tiOPF class and visitors for the data',TTiOPFCodeGenerator);
>>>>>>> graemeg/fixes_2_2
=======
  RegisterCodeGenerator('tiOPF','tiOPF class and visitors for the data',TTiOPFCodeGenerator);
>>>>>>> origin/fixes_2_2

Finalization
  UnRegisterCodeGenerator(TTiOPFCodeGenerator);
end.

