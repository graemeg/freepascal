{
    This file is part of the Free Component Library

    Pascal tree source file writer
    Copyright (c) 2003 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$mode objfpc}
{$h+}
{$inline on}

unit PasWrite;

interface

uses StrUtils, SysUtils, Classes, PasTree;

type
  EPasWriter = Class(Exception);

  { TPasWriter }
  TPasWriterOption = (woNoImplementation, // Do not create implementation code.
                      woNoExternalClass,  // Do not create classes as external
                      woNoExternalVar,    // Do not declare external variables as external.
                      woNoExternalFunc,   // Do not declare external functions as external.
                      woAddLineNumber,    // Prefix line with generated line numbers in comment
                      woAddSourceLineNumber,    // Prefix line with original source line numbers (when available) in comment
                      woForwardClasses,   // Add forward definitions for all classes
                      woForceOverload,     // Force 'overload;' on overloads that are not marked as such.
                      woNoAsm,         // Do not allow asm block
                      woSkipPrivateExternals,  // Skip generation of external procedure declaration in implementation section
                      woAlwaysRecordHelper,     // Force use of record helper for type helper
                      woSkipHints          // Do not add identifier hints
                      );
  TPasWriterOptions = Set of TPasWriterOption;

  TOnUnitAlias = function(const UnitName : String) : String of Object;

  TPasWriter = class
  private
    FCurrentLineNumber : Integer;
    FCurrentLine : String;
    FExtraUnits: String;
    FForwardClasses: TStrings;
    FLineEnding: String;
    FLineNumberWidth: Integer;
    FOnUnitAlias: TOnUnitAlias;
    FOPtions: TPasWriterOptions;
    FStream: TStream;
    FIndentSize : Integer;
    IsStartOfLine: Boolean;
    FLineElement : TPasElement;
    FIndentStep,
    Indent,
    CurDeclSection: string;
    DeclSectionStack: TList;
    FInImplementation : Boolean;
    procedure SetForwardClasses(AValue: TStrings);
    procedure SetIndentSize(AValue: Integer);
    function CheckUnitAlias(const AUnitName : String) : String;
  protected
    procedure DisableHintsWarnings;
    procedure PrepareDeclSectionInStruct(const ADeclSection: string);
    procedure MaybeSetLineElement(AElement: TPasElement);
    function GetExpr(E: TPasExpr): String; virtual;
    Function HasOption(aOption : TPasWriterOption) : Boolean; inline;
    Function NotOption(aOption : TPasWriterOption) : Boolean; inline;
    Function PostProcessLine(S : String) : String; virtual;
    Function GetLineNumberComment : String; virtual;
    Procedure ResetIndent;
    procedure IncIndent;
    procedure DecIndent;
    procedure IncDeclSectionLevel;
    procedure DecDeclSectionLevel;
    procedure PrepareDeclSection(const ADeclSection: string);
    procedure Add(const s: string);
    procedure Add(const Fmt: string; Args : Array of const);
    procedure AddLn(const s: string);overload;
    procedure AddLn(const Fmt: string; Args : Array of const);overload;
    procedure AddLn;overload;
    procedure AddProcArgs(aList: TfpList); virtual;
  public
    constructor Create(AStream: TStream); virtual;
    destructor Destroy; override;
    procedure WriteMembers(aMembers: TFPList; aDefaultVisibility: TPasMemberVisibility=visDefault); virtual;
    procedure AddForwardClasses(aSection: TPasSection); virtual;
    procedure WriteResourceString(aStr: TPasResString); virtual;
    procedure WriteEnumType(AType: TPasEnumType); virtual;
    procedure WriteElement(AElement: TPasElement;SkipSection : Boolean = False);virtual;
    procedure WriteType(AType: TPasType; Full : Boolean = True);virtual;
    procedure WriteProgram(aModule : TPasProgram); virtual;
    Procedure WriteLibrary(aModule : TPasLibrary); virtual;
    Procedure WriteUnit(aModule : TPasModule); virtual;
    procedure WriteModule(AModule: TPasModule); virtual;
    procedure WriteSection(ASection: TPasSection); virtual;
    procedure WriteUsesList(ASection: TPasSection); virtual;
    procedure WriteClass(AClass: TPasClassType); virtual;
    procedure WriteConst(AConst: TPasConst); virtual;
    procedure WriteVariable(aVar: TPasVariable); virtual;
    procedure WriteArgument(aArg: TPasArgument); virtual;
    procedure WriteDummyExternalFunctions(aSection: TPasSection); virtual;
    procedure WriteOverloadedProc(aProc : TPasOverloadedProc; ForceBody: Boolean = False; NamePrefix : String = ''); virtual;
    Procedure WriteAliasType(AType : TPasAliasType); virtual;
    Procedure WriteRecordType(AType : TPasRecordType); virtual;
    Procedure WriteArrayType(AType : TPasArrayType; Full : Boolean = True); virtual;
    procedure WriteProcType(AProc: TPasProcedureType);  virtual;
    procedure WriteProcDecl(AProc: TPasProcedure; ForceBody: Boolean = False; NamePrefix : String = ''); virtual;
    procedure WriteProcImpl(AProc: TProcedureBody; IsAsm : Boolean = false); virtual;
    procedure WriteProcImpl(AProc: TPasProcedureImpl); virtual;
    procedure WriteProperty(AProp: TPasProperty); virtual;
    procedure WriteImplBlock(ABlock: TPasImplBlock);  virtual;
    procedure WriteImplElement(AElement: TPasImplElement; AAutoInsertBeginEnd: Boolean); virtual;
    procedure WriteImplCommand(ACommand: TPasImplCommand);virtual;
    procedure WriteImplCommands(ACommands: TPasImplCommands); virtual;
    procedure WriteImplIfElse(AIfElse: TPasImplIfElse); virtual;
    procedure WriteImplCaseOf(ACaseOf: TPasImplCaseOf); virtual;
    procedure WriteImplCaseStatement(ACaseStatement: TPasImplCaseStatement;
      AAutoInsertBeginEnd: boolean=true); virtual;
    procedure WriteImplForLoop(AForLoop: TPasImplForLoop); virtual;
    procedure WriteImplWhileDo(aWhileDo : TPasImplWhileDo); virtual;
    procedure WriteImplRepeatUntil(aRepeatUntil : TPasImplRepeatUntil); virtual;
    procedure WriteImplTryFinallyExcept(aTry: TPasImplTry); virtual;
    Procedure WriteImplRaise(aRaise : TPasImplRaise); virtual;
    Procedure WriteImplAssign(aAssign : TPasImplAssign); virtual;
    Procedure WriteImplSimple(aSimple: TPasImplSimple); virtual;
    Procedure WriteImplExceptOn(aOn : TPasImplExceptOn); virtual;
    //
    procedure wrt(const s: string); deprecated ;
    procedure wrtln(const s: string);overload; deprecated ;
    procedure wrtln;overload; deprecated ;
    property Stream: TStream read FStream;
  Published
    Property OnUnitAlias : TOnUnitAlias Read FOnUnitAlias Write FOnUnitAlias;
    Property Options : TPasWriterOptions Read FOPtions Write FOptions;
    Property IndentSize : Integer Read FIndentSize Write SetIndentSize;
    Property LineEnding : String Read FLineEnding Write FLineEnding;
    Property ExtraUnits : String Read FExtraUnits Write FExtraUnits;
    Property ForwardClasses : TStrings Read FForwardClasses Write SetForwardClasses;
    Property LineNumberWidth : Integer Read FLineNumberWidth Write FLineNumberWidth;
  end;

procedure WritePasFile(AElement: TPasElement; const AFilename: string);overload;
procedure WritePasFile(AElement: TPasElement; AStream: TStream);overload;

implementation

type
  PDeclSectionStackElement = ^TDeclSectionStackElement;
  TDeclSectionStackElement = record
    LastDeclSection, LastIndent: string;
  end;

constructor TPasWriter.Create(AStream: TStream);
begin
  FStream := AStream;
  IndentSize:=2;
  IsStartOfLine := True;
  DeclSectionStack := TList.Create;
  FForwardClasses:=TStringList.Create;
  FLineEnding:=sLineBreak;
  FLineNumberWidth:=4;
end;

destructor TPasWriter.Destroy;
var
  i: Integer;
  El: PDeclSectionStackElement;
begin
  for i := 0 to DeclSectionStack.Count - 1 do
  begin
    El := PDeclSectionStackElement(DeclSectionStack[i]);
    Dispose(El);
  end;
  DeclSectionStack.Free;
  FForwardClasses.Free;
  inherited Destroy;
end;

procedure TPasWriter.Add(const s: string);
begin
  if IsStartOfLine then // We cannot check for empty, Indent may be empty
    begin
    Inc(FCurrentLineNumber);
    IsStartOfLine := False;
    end;
  if (FCurrentLine='') and (S<>'') and (Length(Indent)>0) then
    FCurrentLine:=FCurrentLine+Indent;
  FCurrentLine:=FCurrentLine+S;
end;

procedure TPasWriter.Add(const Fmt: string; Args: array of const);
begin
  Add(Format(Fmt,Args));
end;

procedure TPasWriter.AddLn(const s: string);

Var
  L : String;
  len : Integer;

begin
  Add(s);
  L:=PostProcessLine(FCurrentLine);
  Len:=Length(L);
  if Len>0 then
    Stream.Write(L[1],Len);
  Stream.Write(FLineEnding[1],Length(FLineEnding));
  IsStartOfLine:=True;
  FCurrentLine:='';
  FLineElement:=Nil;
end;

procedure TPasWriter.AddLn(const Fmt: string; Args: array of const);
begin
  AddLn(Format(Fmt,Args));
end;

procedure TPasWriter.AddLn;
begin
  AddLn('');
end;

procedure TPasWriter.MaybeSetLineElement(AElement : TPasElement);

begin
  If FLineElement=Nil then
    FLineElement:=AElement;
end;

procedure TPasWriter.WriteElement(AElement: TPasElement;SkipSection : Boolean = False);

begin
  if not SkipSection then
    MaybeSetLineElement(AElement);
  if AElement.InheritsFrom(TPasModule) then
    WriteModule(TPasModule(AElement))
  else if AElement.InheritsFrom(TPasSection) then
    WriteSection(TPasSection(AElement))
  else if AElement.ClassType.InheritsFrom(TPasProperty) then
    WriteProperty(TPasProperty(AElement))
  else if AElement.InheritsFrom(TPasConst) then
    WriteConst(TPasConst(AElement)) // Must be before variable
  else if AElement.InheritsFrom(TPasVariable) then
    WriteVariable(TPasVariable(AElement))
  else if AElement.InheritsFrom(TPasArgument) then
    WriteArgument(TPasArgument(AElement))
  else if AElement.InheritsFrom(TPasType) then
    WriteType(TPasType(AElement))
  else if AElement.InheritsFrom(TPasOverloadedProc) then
    WriteOverloadedProc(TPasOverloadedProc(AElement))
  else if AElement.InheritsFrom(TPasProcedureImpl) then // This one must come before TProcedureBody/TPasProcedure
    WriteProcImpl(TPasProcedureImpl(AElement))
  else if AElement.InheritsFrom(TPasProcedure) then
    WriteProcDecl(TPasProcedure(AElement))
  else if AElement.InheritsFrom(TProcedureBody) then
    WriteProcImpl(TProcedureBody(AElement))
  else if AElement.InheritsFrom(TPasImplCommand) or AElement.InheritsFrom(TPasImplCommands) then
    WriteImplElement(TPasImplElement(AElement),false)
  else if AElement.InheritsFrom(TPasResString) then
    WriteResourceString(TPasResString(AElement))
 else
    raise EPasWriter.CreateFmt('Writing not implemented for %s nodes',[AElement.ElementTypeName]);
end;

procedure TPasWriter.WriteResourceString(aStr : TPasResString);

begin
  PrepareDeclSection('resourcestring');
  AddLn(Astr.GetDeclaration(True)+';');
end;

procedure TPasWriter.WriteEnumType(AType: TPasEnumType);

begin
  Add(Atype.GetDeclaration(true));
end;

procedure TPasWriter.WriteType(AType: TPasType; Full : Boolean = True);

begin
  MaybeSetLineElement(AType);
  if Full and (AType.Parent is TPasSection)  then
    PrepareDeclSection('type');
  if AType.ClassType = TPasUnresolvedTypeRef then
    Add(AType.Name)
  else if AType.ClassType.InheritsFrom(TPasClassType) then
    WriteClass(TPasClassType(AType))
  else if AType.ClassType = TPasEnumType then
    WriteEnumType(TPasEnumType(AType))
  else if AType is TPasProcedureType then
    WriteProcType(TPasProcedureType(AType))
  else if AType is TPasArrayType then
    WriteArrayType(TPasArrayType(AType),Full)
  else if AType is TPasRecordType then
    WriteRecordType(TPasRecordType(AType))
  else if AType is TPasAliasType then
    WriteAliasType(TPasAliasType(AType))
  else if AType is TPasPointerType then
    Add(AType.GetDeclaration(true))
  else if AType is TPasSetType then
    Add(AType.GetDeclaration(true))
  else if AType is TPasRangeType then
    Add(AType.GetDeclaration(true))
  else
    raise EPasWriter.CreateFmt('Writing not implemented for %s type nodes',[aType.ElementTypeName]);
  if Full then
    AddLn(';');
end;

procedure TPasWriter.DisableHintsWarnings;

begin
  Addln('{$HINTS OFF}');
  Addln('{$WARNINGS OFF}');
  Addln('{$IFDEF FPC}');
  Addln('{$NOTES OFF}');
  Addln('{$ENDIF FPC}');
end;

procedure TPasWriter.WriteProgram(aModule: TPasProgram);

Var
  S : String;

begin
  S:='';
  if aModule.Name<>'' then
    S:=Format('program %s',[aModule.SafeName]);
  if (S<>'') then
    begin
    If AModule.InputFile<>'' then
      begin
      S:=S+'('+aModule.InputFile;
      if aModule.OutPutFile<>'' then
        S:=S+','+aModule.OutPutFile;
      S:=S+')';
      end;
    AddLn(S+';');
    AddLn;
    end;
  if HasOption(woNoImplementation) then
    DisableHintsWarnings;
  if Assigned(aModule.ProgramSection) then
    WriteSection(aModule.ProgramSection);
  if Assigned(AModule.InitializationSection) then
    begin
    PrepareDeclSection('');
    AddLn;
    AddLn('begin');
    IncIndent;
    if NotOption(woNoImplementation) then
      WriteImplBlock(AModule.InitializationSection);
    DecIndent;
    end;
  Addln('end.');
end;

procedure TPasWriter.WriteLibrary(aModule: TPasLibrary);
Var
  S : String;

begin
  S:='';
  if aModule.Name<>'' then
    S:=Format('library %s',[aModule.SafeName]);
  if (S<>'') then
    begin
    If AModule.InputFile<>'' then
      begin
      S:=S+'('+aModule.InputFile;
      if aModule.OutPutFile<>'' then
        S:=S+','+aModule.OutPutFile;
      S:=S+')';
      end;
    AddLn(S+';');
    AddLn;
    end;
  if HasOption(woNoImplementation) then
    DisableHintsWarnings;
  if Assigned(AModule.InitializationSection) then
    begin
    PrepareDeclSection('');
    AddLn;
    AddLn('begin');
    IncIndent;
    if NotOption(woNoImplementation) then
      WriteImplBlock(AModule.InitializationSection);
    DecIndent;
    end;
  Addln('end.');
end;

procedure TPasWriter.WriteDummyExternalFunctions(aSection : TPasSection);

  Function IsExt(P : TPasProcedure; AllowConstructor : Boolean) : Boolean;

  begin
    Result:=Assigned(P.LibrarySymbolName) or Assigned(P.LibraryExpr);
    if (Not Result) Then
      Result:=(AllowConstructor and (P is TPasConstructor));
  end;

  Procedure DoCheckElement(E : TPasElement; Force : Boolean; Prefix: String);

  Var
    P : TPasProcedure;
    PP : TPasOverloadedProc;
    I : Integer;

  begin
    if (E is TPasProcedure) then
      begin
      P:=E as TPasProcedure;
      if Force or IsExt(P,False) then
        WriteProcDecl(P,True,Prefix)
      end
    else if (E is TPasOverloadedProc) then
      begin
      PP:=(E as TPasOverloadedProc);
      For I:=0 to PP.Overloads.Count-1 do
        begin
        P:=TPasProcedure(PP.Overloads[I]);
        if Force or IsExt(P,False) then
          WriteProcDecl(P,True,Prefix)
        end
      end;
  end;

  procedure DoCheckClass(C: TPasClassType; Force : Boolean; Prefix: String);
  var
    I: Integer;
    M : TPasElement;

  begin
    if (C.ExternalName<>'') then
      for I:=0 to C.Members.Count-1 do
      begin
        M:=TPasElement(C.members[I]);
        if (M is TPasClassType) then
          DoCheckClass(M as TPasClassType, Force, Prefix + C.SafeName + '.')
        else
          DoCheckElement(M, Force, Prefix + C.SafeName + '.');
      end;
  end;

Var
  I : Integer;
  E : TPasElement;
  C : TPasClassType;

begin
  Addln;
  Addln('// Dummy implementations for externals');
  Addln;
  For I:=0 to aSection.Declarations.Count-1 do
    begin
    E:=TPasElement(aSection.Declarations[i]);
    DoCheckElement(E,False,'');
    if (E is TPasClassType) then
      DoCheckClass(E as TPasClassType, True, '');
    end;
  Addln;
  Addln('// end of dummy implementations');
  Addln;
end;

procedure TPasWriter.AddForwardClasses(aSection : TPasSection);

Var
  I : Integer;
  CN : String;

begin
  if Not Assigned(aSection.Classes) or (aSection.Classes.Count=0) then
    exit;
  PrepareDeclSection('type');
  For I:=0 to aSection.Classes.Count-1 do
    begin
    CN:=TPasElement(aSection.Classes[i]).SafeName;
    if (FForwardClasses.Count=0) or (ForwardClasses.IndexOf(CN)<>-1) then
      Addln('%s = class;',[CN]);
    end;
end;

procedure TPasWriter.WriteUnit(aModule: TPasModule);

begin
  AddLn('unit ' + CheckUnitAlias(AModule.SafeName) + ';');
  if Assigned(AModule.GlobalDirectivesSection) then
    begin
    AddLn;
    WriteImplElement(AModule.GlobalDirectivesSection,false);
    end;
  AddLn;
  AddLn('interface');
  AddLn;
  WriteSection(AModule.InterfaceSection);
  ResetIndent;
  AddLn;
  AddLn;
  AddLn('implementation');
  FInImplementation:=True;
  if HasOption(woNoImplementation) then
    DisableHintsWarnings;
  if hasOption(woNoExternalFunc) then
    WriteDummyExternalFunctions(AModule.InterfaceSection);
  if Assigned(AModule.ImplementationSection) then
    begin
    AddLn;
    WriteSection(AModule.ImplementationSection);
    end;
  AddLn;
  if NotOption(woNoImplementation) then
    begin
    PrepareDeclSection('');
    if Assigned(AModule.InitializationSection) then
      begin
      AddLn('initialization');
      IncIndent;
      WriteImplBlock(AModule.InitializationSection);
      DecIndent;
      end;
    if Assigned(AModule.FinalizationSection) then
      begin
      AddLn('finalization');
      IncIndent;
      WriteImplBlock(AModule.FinalizationSection);
      DecIndent;
      end;
    end;
  AddLn('end.');
end;

procedure TPasWriter.WriteModule(AModule: TPasModule);

begin
  FInImplementation:=False;;
  if aModule is TPasProgram then
    WriteProgram(TPasProgram(aModule))
  else if aModule is TPasLibrary then
    WriteLibrary(TPasLibrary(aModule))
  else
    WriteUnit(aModule)
end;

procedure TPasWriter.WriteUsesList(ASection: TPasSection);

Const
  UnitSeps = [',',';',' '];

Var
  C : Integer;

  function AllowUnit(S : String) : Boolean;

  begin
    Result:=Not SameText(S,'System');
  end;

  Procedure AddUnit(Const aName : String; AUnitFile : TPasExpr);
  begin
    if c > 0 then
      Add(', ')
    else
      Add('uses ');
    Add(CheckUnitAlias(AName));
    if (AUnitFile<>Nil) then
      Add(' in '+GetExpr(AUnitFile));
    Inc(c);
  end;

Var
  I : integer;
  u : string;

begin
  C:=0;
  if ASection.UsesList.Count>0 then
    begin
    if not (aSection is TImplementationSection) then
      For I:=1 to WordCount(ExtraUnits,UnitSeps) do
        begin
        u:=Trim(ExtractWord(1,ExtraUnits,UnitSeps));
        if (U<>'') then
          AddUnit(U,Nil);
        end;
    if length(ASection.UsesClause)=ASection.UsesList.Count then
      begin
      for i := 0 to length(ASection.UsesClause)-1 do
        if AllowUnit(ASection.UsesClause[i].Name) then
          AddUnit(ASection.UsesClause[i].Name,ASection.UsesClause[i].InFilename);
      end
    else
      for i := 0 to ASection.UsesList.Count - 1 do
        if AllowUnit(TPasElement(ASection.UsesList[i]).Name) then
          AddUnit(TPasElement(ASection.UsesList[i]).Name,Nil);
    if C>0 then
      begin
      AddLn(';');
      AddLn;
      end;
    end;
end;

procedure TPasWriter.WriteSection(ASection: TPasSection);

var
  i: Integer;

begin
  WriteUsesList(aSection);
  CurDeclSection := '';
  if HasOption(woForwardClasses) then
    begin
    AddForwardClasses(ASection);
    AddLn;
    end;
  for i := 0 to ASection.Declarations.Count - 1 do
    WriteElement(TPasElement(ASection.Declarations[i]));
end;

procedure TPasWriter.WriteClass(AClass: TPasClassType);

var
  i: Integer;
  InterfacesListPrefix: string;

begin
  PrepareDeclSection('type');
  Addln;
  MaybeSetLineElement(AClass);
  Add(AClass.SafeName + ' = ');
  if AClass.IsPacked then
     Add('packed ');                      // 12/04/04 - Dave - Added
  case AClass.ObjKind of
    okObject: Add('object');
    okClass: Add('class');
    okInterface: Add('interface');
    okTypeHelper :
      if HasOption(woAlwaysRecordHelper) then
        Add('record helper')
      else
        Add('type helper');
    okRecordHelper: Add('record helper');
    okClassHelper: Add('class helper');
  end;
  if (AClass.ObjKind in [okTypeHelper,okRecordHelper,okClassHelper]) then
    begin
    if not Assigned(AClass.HelperForType) then
      Add(' for unknowntype')
    else
      Add(' for '+AClass.HelperForType.SafeName)
    end;

  if AClass.IsForward then
    exit;
  if (AClass.ObjKind=okClass) and (ACLass.ExternalName<>'') and NotOption(woNoExternalClass) then
    Add(' external name ''%s'' ',[AClass.ExternalName]);
  if Assigned(AClass.AncestorType) then
    Add('(' + AClass.AncestorType.SafeName);
  if AClass.Interfaces.Count > 0 then
  begin
    if Assigned(AClass.AncestorType) then
      InterfacesListPrefix:=', '
    else
      InterfacesListPrefix:='(';
    Add(InterfacesListPrefix + TPasType(AClass.Interfaces[0]).SafeName);
    for i := 1 to AClass.Interfaces.Count - 1 do
      Add(', ' + TPasType(AClass.Interfaces[i]).SafeName);
  end;
  if Assigned(AClass.AncestorType) or (AClass.Interfaces.Count > 0) then
    AddLn(')')
  else
    AddLn;
  if AClass.ObjKind = okInterface then
    if Assigned(AClass.GUIDExpr) then
      AddLn('['+AClass.InterfaceGUID+']');
  IncIndent;
  IncDeclSectionLevel;
  WriteMembers(AClass.Members);
  DecDeclSectionLevel;
  DecIndent;
  Add('end');
end;

procedure TPasWriter.WriteMembers(aMembers : TFPList; aDefaultVisibility : TPasMemberVisibility = visDefault);

Var
  Member, LastMember: TPasElement;
  LastVisibility, CurVisibility: TPasMemberVisibility;

  function ForceVisibility: boolean;
  begin
    Result := (LastMember <> nil) and
      // variables can't be declared directly after methods nor properties
      // (visibility section or var keyword is required)
      ((Member is TPasVariable) and not (Member is TPasProperty)) and not (LastMember is TPasVariable);
  end;

Var
  I : integer;

begin
  LastVisibility:=aDefaultVisibility;
  LastMember := nil;
  for i := 0 to aMembers.Count - 1 do
    begin
    Member := TPasElement(aMembers[i]);
    CurVisibility := Member.Visibility;
    if (CurVisibility <> LastVisibility) or ForceVisibility then
      begin
      DecIndent;
      case CurVisibility of
        visPrivate: AddLn('private');
        visProtected: AddLn('protected');
        visPublic: AddLn('public');
        visPublished: AddLn('published');
        visAutomated: AddLn('automated');
      end;
      IncIndent;
      LastVisibility := CurVisibility;
      CurDeclSection := '';
      end;
    WriteElement(Member);
    LastMember := Member;
    end;
end;

procedure TPasWriter.WriteConst(AConst: TPasConst);

Const
  Seps : Array[Boolean] of Char = ('=',':');

Var
  Vart,Decl : String;

begin
  PrepareDeclSection('const');
  Decl:='';
  With AConst do
    begin
    If Assigned(VarType) then
      begin
      If VarType.Name='' then
        Vart:=VarType.GetDeclaration(False)
      else
        Vart:=VarType.SafeName;
      Decl:=Vart+Modifiers;
      Vart:=LowerCase(Vart);
      if (Value<>'') then
         Decl:=Decl+' = '+Value
      else if (ExportName<>Nil) or ((Parent is TPasClassType) and (TPasClassType(Parent).ExternalName<>'')) then // external name
        case VarT of
          'integer',
          'byte',
          'word',
          'smallint',
          'int64',
          'nativeint',
          'shortint',
          'longint' : Decl:=Decl+' = 0';
          'double',
          'single',
          'extended' : Decl:=Decl+' = 0.0';
          'string' : Decl:=Decl+' = ''''';
        else
          if Pos('array',Vart)>0 then
            Decl:=Decl+' = []';
        end;
      end
    else
      Decl:=Value;

    Decl:=SafeName+' '+Seps[Assigned(VarType)]+' '+Decl;
    if NotOption(woSkipHints) then
      Decl:=Decl+HintsString;
    end;
  AddLn(Decl+';');
end;

procedure TPasWriter.WriteVariable(aVar: TPasVariable);

var
  LParentIsClassOrRecord: boolean;

begin
  LParentIsClassOrRecord:= (aVar.Parent.ClassType = TPasClassType) or
    (aVar.Parent.ClassType = TPasRecordType);
  if not LParentIsClassOrRecord then
    PrepareDeclSection('var')
  // handle variables in classes/records
  else if vmClass in aVar.VarModifiers then
    PrepareDeclSectionInStruct('class var')
  else if (CurDeclSection<>'') and not (aVar.Parent.ClassType = TPasRecordType) then
    PrepareDeclSectionInStruct('var');
  Add(aVar.SafeName + ': ');
  if Not Assigned(aVar.VarType) then
    Raise EWriteError.CreateFmt('No type for variable %s',[aVar.SafeName]);
  WriteType(aVar.VarType,False);
  if (aVar.AbsoluteExpr<>nil) then
    Add(' absolute %s',[aVar.AbsoluteExpr.ClassName])
  else if (aVar.LibraryName<>Nil) or Assigned (aVar.ExportName) then
    begin
    if LParentIsClassOrRecord then
      begin
      if NotOption(woNoExternalClass) then
        Add('; external name ''%s''',[aVar.ExportName.GetDeclaration(true)]);
      end
    else if NotOption(woNoExternalVar) then
      begin
      Add('; external ');
      if (aVar.LibraryName<>Nil) then
        Add('%s ',[aVar.LibraryName.GetDeclaration(true)]);
      Add('name %s',[aVar.ExportName.GetDeclaration(true)]);
      end;
    end;
  if Not LParentIsClassOrRecord then
    if Assigned(aVar.Expr) then
      Add(' = '+aVar.Expr.GetDeclaration(true));
  AddLn(';');
end;

procedure TPasWriter.WriteArgument(aArg: TPasArgument);

begin
  if (aArg.Access<>argDefault) then
    Add(AccessNames[aArg.Access]+' ');
  Add(aArg.SafeName+' : ');
  WriteType(aArg.ArgType,False);
end;

procedure TPasWriter.WriteOverloadedProc(aProc: TPasOverloadedProc; ForceBody: Boolean = False; NamePrefix : String = '');

Var
  I : integer;

begin
  For I:=0 to aProc.Overloads.Count-1 do
    begin
    if HasOption(woForceOverload) then
      TPasProcedure(aProc.Overloads[i]).AddModifier(pmOverload);
    WriteProcDecl(TPasElement(aProc.Overloads[i]) as TPasProcedure,ForceBody,NamePrefix);
    end;
end;

procedure TPasWriter.WriteAliasType(AType: TPasAliasType);

begin
  If AType.Parent is TPasSection then
    Add(AType.GetDeclaration(true))
  else
    Add(AType.Name)
end;

procedure TPasWriter.WriteRecordType(AType: TPasRecordType);

Var
  Temp : String;

begin
  Temp:='record';
  If aType.IsPacked then
    if Atype.IsBitPacked then
      Temp:='bitpacked '+Temp
    else
      Temp:='packed '+Temp;
  If (Atype.Name<>'') then
    begin
    if AType.GenericTemplateTypes.Count>0 then
      Temp:=AType.SafeName+GenericTemplateTypesAsString(AType.GenericTemplateTypes)+' = '+Temp
    else
      Temp:=AType.SafeName+' = '+Temp;
    end;
  AddLn(Temp);
  IncIndent;
  IncDeclSectionLevel;
  WriteMembers(AType.Members,visPublic);
  DecDeclSectionLevel;
  DecIndent;
  Add('end');
end;

procedure TPasWriter.WriteArrayType(AType: TPasArrayType; Full : Boolean = True);

begin
  Add(AType.GetDeclaration(Full));
end;

procedure TPasWriter.WriteProcType(AProc: TPasProcedureType);

begin
  Add(TPasProcedureType(AProc).GetDeclaration(true));
  if TPasProcedureType(AProc).CallingConvention<>ccDefault then
    Add('; '+cCallingConventions[TPasProcedureType(AProc).CallingConvention]);
end;


procedure TPasWriter.WriteProcDecl(AProc: TPasProcedure; ForceBody : Boolean = False; NamePrefix : String = '');

  Procedure EmptyBody;

  begin
    Addln('');
    Addln('begin');
    AddLn('end;');
    Addln('');
  end;
Var
  AddExternal : boolean;
  IsImpl : Boolean;

begin

  IsImpl:=AProc.Parent is TImplementationSection;
  if IsImpl then
    PrepareDeclSection('');
  if Not IsImpl then
    IsImpl:=FInImplementation;
  if FInImplementation and not forcebody and (Assigned(AProc.LibraryExpr) or Assigned(AProc.LibrarySymbolName)) and HasOption(woSkipPrivateExternals)  then
    Exit;
  Add(AProc.TypeName + ' ' + NamePrefix+AProc.SafeName);
  if Assigned(AProc.ProcType) and (AProc.ProcType.Args.Count > 0) then
    AddProcArgs(AProc.ProcType.Args) ;
  if Assigned(AProc.ProcType) and
    (AProc.ProcType.ClassType = TPasFunctionType) then
  begin
    Add(': ');
    WriteType(TPasFunctionType(AProc.ProcType).ResultEl.ResultType,False);
  end;
  Add(';');
  // delphi compatible order for example: procedure foo; reintroduce; overload; static;
  if not IsImpl and AProc.IsReintroduced then
    Add(' reintroduce;');
  // if NamePrefix is not empty, we're writing a dummy for external class methods.
  // In that case, we must not write the 'overload'.
  if AProc.IsOverload and (NamePrefix='') and not IsImpl then
    Add(' overload;');
  if not IsImpl then
    begin
    if AProc.IsVirtual then
      Add(' virtual;');
    if AProc.IsDynamic then
      Add(' dynamic;');
    if AProc.IsAbstract then
      Add(' abstract;');
    if AProc.IsOverride then
      Add(' override;');
    if AProc.IsStatic then
      Add(' static;');
    end;
  if (pmAssembler in AProc.Modifiers) and Not (woNoAsm in OPtions) then
    Add(' assembler;');
  if AProc.CallingConvention<>ccDefault then
    Add(' '+cCallingConventions[AProc.CallingConvention]+';');
  If Assigned(AProc.LibraryExpr) or Assigned(AProc.LibrarySymbolName) then
    begin
    if AProc.Parent is TPasClassType then
      AddExternal:=NotOption(woNoExternalClass)
    else
      AddExternal:=NotOption(woNoExternalFunc);
    if AddExternal then
      begin
      add('external');
      if Assigned(AProc.LibraryExpr) then
        Add(' '+GetExpr(AProc.LibraryExpr));
      if Assigned(AProc.LibrarySymbolName) then
        Add(' name '+GetExpr(AProc.LibrarySymbolName));
      Add(';');
      end;
    end;
  AddLn;

  if Assigned(AProc.Body)  then
    begin
    if (pmAssembler in AProc.Modifiers) and (woNoAsm in Options) then
      EmptyBody
    else
      WriteProcImpl(AProc.Body,pmAssembler in AProc.Modifiers)
    end
  else if ForceBody then
    EmptyBody;
end;


procedure TPasWriter.AddProcArgs(aList : TfpList);

Var
  I : Integer;
  A : TPasArgument;

begin
  Add('(');
  If Assigned(aList) then
    for i := 0 to Alist.Count - 1 do
      begin
      A:= TPasArgument(AList[i]);
      if i > 0 then
        Add('; ');
      Add(AccessNames[A.Access]+A.Name);
      if Assigned(A.ArgType) then
        begin
        Add(': ');
        WriteType(A.ArgType,False);
        end;
      if A.Value <> '' then
        Add(' = ' + A.Value);
      end;
  Add(')');
end;

// For backwards compatibility

procedure TPasWriter.WriteProcImpl(AProc: TPasProcedureImpl);

var
  i: Integer;
  E,PE  :TPasElement;

begin
  PrepareDeclSection('');
  if AProc.IsClassMethod then
    Add('class ');
  Add(AProc.TypeName + ' ');
  if AProc.Parent.ClassType = TPasClassType then
    Add(AProc.Parent.SafeName + '.');
  Add(AProc.SafeName);
  if Assigned(AProc.ProcType) and (AProc.ProcType.Args.Count > 0) then
    AddProcArgs(AProc.ProcType.Args);
  if Assigned(AProc.ProcType) and
    (AProc.ProcType.ClassType = TPasFunctionType) then
  begin
    Add(': ');
    WriteType(TPasFunctionType(AProc.ProcType).ResultEl.ResultType,False);
  end;
  AddLn(';');
  IncDeclSectionLevel;
  PE:=nil;
  for i := 0 to AProc.Locals.Count - 1 do
    begin
    E:=TPasElement(AProc.Locals[i]);
    if E.InheritsFrom(TPasProcedureImpl) then
      begin
      IncIndent;
      if (i = 0) or not PE.InheritsFrom(TPasProcedureImpl) then
        Addln;
      end;
    WriteElement(E);
    if E.InheritsFrom(TPasProcedureImpl) then
      DecIndent;
    PE:=E;
    end;
  DecDeclSectionLevel;
  AddLn('begin');
  IncIndent;
  if Assigned(AProc.Body) then
    WriteImplBlock(AProc.Body);
  DecIndent;
  AddLn('end;');
  AddLn;
end;

procedure TPasWriter.WriteProcImpl(AProc: TProcedureBody; IsAsm : Boolean = false);

var
  i: Integer;
  El,PEl : TPasElement;
begin
  PrepareDeclSection('');
  If NotOption(woNoImplementation) then
    begin
    IncDeclSectionLevel;
    PEl:=Nil;
    for i := 0 to aProc.Declarations.Count - 1 do
      begin
      El:=TPasElement(aProc.Declarations[i]);
      if El.InheritsFrom(TPasProcedureImpl) then
        begin
        IncIndent;
        if (PEL=Nil) or not PEL.InheritsFrom(TPasProcedureImpl) then
          AddLn;
        end;
      WriteElement(El);
      if El.InheritsFrom(TPasProcedureImpl) then
        DecIndent;
      Pel:=El;
      end;
    DecDeclSectionLevel;
    end;
  if IsAsm then
    AddLn('asm')
  else
    AddLn('begin');
  If NotOption(woNoImplementation) then
    begin
    IncIndent;
    if Assigned(AProc.Body) then
      WriteImplBlock(AProc.Body);
    DecIndent;
    end;
  AddLn('end;');
  AddLn;
end;

procedure TPasWriter.WriteProperty(AProp: TPasProperty);
var
  i: Integer;
begin
  if AProp.IsClass then
    Add('class ');
  Add('property ' + AProp.SafeName);
  if AProp.Args.Count > 0 then
  begin
    Add('[');
    for i := 0 to AProp.Args.Count - 1 do
      begin
      if I>0 then Add(',');
      WriteArgument(TPasArgument(AProp.Args[i]));
      end;
      // !!!: Create WriteArgument method and call it here
    Add(']');
  end;
  if Assigned(AProp.VarType) then
  begin
    Add(': ');
    WriteType(AProp.VarType,False);
  end;
  if AProp.IndexValue <> '' then
    Add(' index ' + AProp.IndexValue); 
  if AProp.ReadAccessorName <> '' then
    Add(' read ' + AProp.ReadAccessorName);
  if AProp.WriteAccessorName <> '' then
    Add(' write ' + AProp.WriteAccessorName);
  if AProp.StoredAccessorName <> '' then
    Add(' stored ' + AProp.StoredAccessorName);
  if AProp.DefaultValue <> '' then
    Add(' default ' + AProp.DefaultValue);
  if AProp.IsNodefault then
    Add(' nodefault');
  if AProp.IsDefault then
    Add('; default');
  AddLn(';');
end;

procedure TPasWriter.WriteImplBlock(ABlock: TPasImplBlock);
var
  i: Integer;
begin
  for i := 0 to ABlock.Elements.Count - 1 do
  begin
    WriteImplElement(TPasImplElement(ABlock.Elements[i]), False);
    if (TPasImplElement(ABlock.Elements[i]).ClassType = TPasImplCommand) then
    begin
      if TPasImplCommand(ABlock.Elements[i]).SemicolonAtEOL then
        AddLn(';')
      else
        AddLn;
    end;
  end;
end;

procedure TPasWriter.WriteImplElement(AElement: TPasImplElement;  AAutoInsertBeginEnd: Boolean);

begin
  if AElement.ClassType = TPasImplCommand then
    WriteImplCommand(TPasImplCommand(AElement))
  else
  if AElement.ClassType = TPasImplCommands then
    begin
    if AAutoInsertBeginEnd then
    begin
      DecIndent;
      AddLn('begin');
      IncIndent;
    end;
    WriteImplCommands(TPasImplCommands(AElement));
    if AAutoInsertBeginEnd then
    begin
      DecIndent;
      AddLn('end;');
      IncIndent;
    end;
    end
  else if (AElement.ClassType = TPasImplBlock) or (AElement.ClassType = TPasImplBeginBlock) then
    begin
    if AAutoInsertBeginEnd or (AElement.ClassType = TPasImplBeginBlock) then
    begin
      DecIndent;
      AddLn('begin');
      IncIndent;
    end;
    WriteImplBlock(TPasImplBlock(AElement));
    if AAutoInsertBeginEnd or (AElement.ClassType = TPasImplBeginBlock) then
    begin
      DecIndent;
      AddLn('end;');
      IncIndent;
    end;
    end
  else if AElement.ClassType = TPasImplIfElse then
    WriteImplIfElse(TPasImplIfElse(AElement))
  else if AElement.InheritsFrom(TPasImplCaseOf) then
    WriteImplCaseOf(TPasImplCaseOf(aElement))
  else if AElement.ClassType = TPasImplForLoop then
    WriteImplForLoop(TPasImplForLoop(AElement))
  else if AElement.InheritsFrom(TPasImplWhileDo) then
    WriteImplWhileDo(TPasImplWhileDo(AElement))
  else if AElement.InheritsFrom(TPasImplRepeatUntil) then
    WriteImplRepeatUntil(TPasImplRepeatUntil(AElement))
  else if AElement.InheritsFrom(TPasImplTry) then
    WriteImplTryFinallyExcept(TPasImplTry(aElement))
  else if AElement.InheritsFrom(TPasImplRaise) then
    WriteImplRaise(TPasImplRaise(aElement))
  else if AElement.InheritsFrom(TPasImplAssign) then
    WriteImplAssign(TPasImplAssign(aElement))
  else if AElement.InheritsFrom(TPasImplSimple) then
    WriteImplSimple(TPasImplSimple(aElement))
  else if AElement.InheritsFrom(TPasImplExceptOn) then
    WriteImplExceptOn(TPasImplExceptOn(aElement))
  else
    raise EPasWriter.CreateFmt('Writing not yet implemented for %s implementation elements',[AElement.ClassName]);
end;

procedure TPasWriter.WriteImplCommand(ACommand: TPasImplCommand);
begin
  Add(ACommand.Command);
end;

procedure TPasWriter.WriteImplCommands(ACommands: TPasImplCommands);
var
  i: Integer;
  s: string;
begin
  for i := 0 to ACommands.Commands.Count - 1 do
  begin
    s := ACommands.Commands[i];
    if Length(s) > 0 then
      if (Length(s) >= 2) and (s[1] = '/') and (s[2] = '/') then
        AddLn(s)
      else
        if ACommands.SemicolonAtEOL then
          AddLn(s + ';')
        else
          AddLn(s);
  end;
end;

procedure TPasWriter.WriteImplIfElse(AIfElse: TPasImplIfElse);

Var
  DoBeginEnd : Boolean;

begin
  Add('if ' + AIfElse.Condition + ' then');
  if Assigned(AIfElse.IfBranch) then
    begin
    AddLn;
    DoBeginEnd:=(AIfElse.IfBranch.ClassType = TPasImplCommands) or
                (AIfElse.IfBranch.ClassType = TPasImplBlock) or
                Assigned(aIfElse.ElseBranch);
    if DoBeginEnd then
      AddLn('begin');
    IncIndent;
    if AIfElse.IfBranch is TPasImplBeginBlock then
       WriteImplBlock(TPasImplBeginBlock(AIfElse.IfBranch))
     else
       WriteImplElement(AIfElse.IfBranch, False);
    DecIndent;
    if DoBeginEnd then
      begin
      if Assigned(AIfElse.ElseBranch) then
        Add('end ')
      else
        AddLn('end;')
      end
    else
      if Assigned(AIfElse.ElseBranch) then
        AddLn;
    end
  else if not Assigned(AIfElse.ElseBranch) then
    AddLn(';')
  else
    AddLn;

  if Assigned(AIfElse.ElseBranch) then
    if AIfElse.ElseBranch.ClassType = TPasImplIfElse then
    begin
      Add('else ');
      WriteImplElement(AIfElse.ElseBranch, True);
    end else
    begin
      AddLn('else');
      IncIndent;
      WriteImplElement(AIfElse.ElseBranch, True);
{      if (not Assigned(AIfElse.Parent)) or
        (AIfElse.Parent.ClassType <> TPasImplIfElse) or
        (TPasImplIfElse(AIfElse.Parent).IfBranch <> AIfElse) then
        AddLn(';');}
      DecIndent;
    end;
end;

procedure TPasWriter.WriteImplCaseStatement(ACaseStatement: TPasImplCaseStatement;AAutoInsertBeginEnd:boolean=true);
var
  i: Integer;
begin
  for i := 0 to ACaseStatement.Expressions.Count - 1 do
     begin
       if i>0 then add(', ');
       add(GetExpr(TPasExpr(ACaseStatement.Expressions[i])))
     end;
  add(': ');
  IncIndent;
  //JC: If no body is assigned, omit the whole block
  if assigned(ACaseStatement.Body) then
    begin
      if AAutoInsertBeginEnd then
        begin
          addLn('begin');
          IncIndent;
        end;
      //JC: if the body already is a begin-end-Block, the begin of that block is omitted
      if ACaseStatement.Body is TPasImplBeginBlock then
         WriteImplBlock(TPasImplBeginBlock(ACaseStatement.Body))
       else
         WriteImplElement(ACaseStatement.Body,false);
      if AAutoInsertBeginEnd then
        begin
          DecIndent;
          Add('end'); //JC: No semicolon or Linefeed here !
          // Otherwise there would be a problem with th else-statement.
        end;
    end;
  DecIndent;
end;

procedure TPasWriter.WriteImplCaseOf(ACaseOf: TPasImplCaseOf);
var
  i: Integer;

begin
  Add('case %s of', [GetExpr(ACaseOf.CaseExpr)]);
  IncIndent;
  for i := 0 to ACaseOf.Elements.Count - 1 do
  begin
    if TPasElement(ACaseOf.Elements[i]) is TPasImplCaseStatement then
      begin
        if i >0 then
          AddLn(';')
        else
          AddLn;
        WriteImplCaseStatement(TPasImplCaseStatement(ACaseOf.Elements[i]),True);
      end;
  end;
  if assigned(ACaseOf.ElseBranch) then
    begin
      AddLn;
      AddLn('else');
      IncIndent;
      WriteImplBlock(ACaseOf.ElseBranch);
      DecIndent;
    end
  else
    AddLn(';');
  DecIndent;
  AddLn('end;');
end;


procedure TPasWriter.WriteImplRepeatUntil(aRepeatUntil: TPasImplRepeatUntil);

begin
  Addln('repeat');
  with aRepeatUntil do
    begin
    IncIndent;
    WriteImplBlock(aRepeatUntil);
    DecIndent;
    AddLn('until %s;',[GetExpr(ConditionExpr)]);
    end;
end;

procedure TPasWriter.WriteImplTryFinallyExcept(aTry: TPasImplTry);
begin
  Addln('try');
  with aTry do
    begin
    IncIndent;
    WriteImplBlock(aTry);
    DecIndent;
    if aTry.FinallyExcept is TPasImplTryFinally then
      AddLn('finally')
    else
      AddLn('except');
    IncIndent;
    WriteImplBlock(aTry.FinallyExcept);
    DecIndent;
    if Assigned(aTry.ElseBranch) then
      begin
      AddLn('else');
      IncIndent;
      WriteImplBlock(aTry.ElseBranch);
      DecIndent;
      end;
    end;
  AddLn('end;')
end;

procedure TPasWriter.WriteImplRaise(aRaise: TPasImplRaise);
begin
  if assigned(aRaise.ExceptObject) then
    begin
    Add('raise %s',[GetExpr(aRaise.ExceptObject)]);
    if aRaise.ExceptAddr<>Nil then
      Add(' at %s',[GetExpr(aRaise.ExceptAddr)]);
    end
  else
    Add('raise');
  Addln(';');
end;

procedure TPasWriter.WriteImplAssign(aAssign: TPasImplAssign);

begin
  AddLn('%s %s %s;',[GetExpr(aAssign.left),AssignKindNames[aAssign.Kind],GetExpr(aAssign.right)]);
end;

procedure TPasWriter.WriteImplSimple(aSimple: TPasImplSimple);
begin
  Addln('%s;',[GetExpr(aSimple.expr)]);
end;

procedure TPasWriter.WriteImplExceptOn(aOn: TPasImplExceptOn);
begin
  Addln('On %s : %s do',[aOn.VarEl.SafeName,aOn.TypeEl.SafeName]);
  if Assigned(aOn.Body) then
    WriteImplElement(aOn.Body,True);
end;

procedure TPasWriter.wrt(const s: string);
begin
  Add(s);
end;

procedure TPasWriter.wrtln(const s: string);
begin
  AddLn(s);
end;

procedure TPasWriter.wrtln;
begin
  Addln;
end;

function TPasWriter.GetExpr(E : TPasExpr) : String;

begin
  Result:=E.GetDeclaration(True);
end;

procedure TPasWriter.WriteImplForLoop(AForLoop: TPasImplForLoop);

Const
  ToNames : Array[Boolean] of string = ('to','downto');

begin
  With aForLoop do
    begin
    If LoopType=ltIn then
      Add('for %s in %s do',[GetExpr(VariableName),GetExpr(StartExpr)])
    else
      Add('for %s:=%s %s %s do',[GetExpr(VariableName),GetExpr(StartExpr),
                                   ToNames[Down],GetExpr(EndExpr)]);
    if assigned(Body) then
      begin
        AddLn;
        IncIndent;
        WriteImplElement(Body, True);
        DecIndent;
        if (Body is TPasImplBlock) and
           (Body is TPasImplCommands) then
          AddLn(';');
      end
    else
      AddLn(';');
    end;
end;


procedure TPasWriter.WriteImplWhileDo(aWhileDo: TPasImplWhileDo);

begin
  With aWhileDo do
    begin
    Add('While %s do',[GetExpr(ConditionExpr)]);
    if assigned(Body) then
      begin
        AddLn;
        IncIndent;
        WriteImplElement(Body, True);
        DecIndent;
        if (Body.InheritsFrom(TPasImplBlock)) and
           (Body.InheritsFrom(TPasImplCommands)) then
          AddLn(';');
      end
    else
      AddLn(';');
    end;
end;

procedure TPasWriter.IncIndent;
begin
  Indent := Indent + FIndentStep;
end;

procedure TPasWriter.DecIndent;
begin
  if (Length(Indent)<FIndentSize) then
    raise EPasWriter.Create('Internal indent error');
  SetLength(Indent, Length(Indent) - FIndentSize);
end;

procedure TPasWriter.IncDeclSectionLevel;
var
  El: PDeclSectionStackElement;
begin
  New(El);
  DeclSectionStack.Add(El);
  El^.LastDeclSection := CurDeclSection;
  El^.LastIndent := Indent;
  CurDeclSection := '';
end;

procedure TPasWriter.DecDeclSectionLevel;
var
  El: PDeclSectionStackElement;
begin
  if DeclSectionStack.Count=0 then
    raise EPasWriter.Create('Internal section indent error');
  El := PDeclSectionStackElement(DeclSectionStack[DeclSectionStack.Count - 1]);
  DeclSectionStack.Delete(DeclSectionStack.Count - 1);
  CurDeclSection := El^.LastDeclSection;
  Indent := El^.LastIndent;
  Dispose(El);
end;

procedure TPasWriter.PrepareDeclSection(const ADeclSection: string);
begin
  if Not SameText(ADeclSection,CurDeclSection) then
  begin
    if CurDeclsection <> '' then
      begin
      DecIndent;
      end;
    if ADeclSection <> '' then
    begin
      AddLn(ADeclSection);
      IncIndent;
    end;
    CurDeclSection := ADeclSection;
  end;
end;

procedure TPasWriter.PrepareDeclSectionInStruct(const ADeclSection: string);

begin
  if Not SameText(ADeclSection,CurDeclSection) then
  begin
    if ADeclSection <> '' then
    begin
      DecIndent;
      AddLn(ADeclSection);
      IncIndent;
    end;
    CurDeclSection := ADeclSection;
  end;
end;

procedure TPasWriter.SetForwardClasses(AValue: TStrings);
begin
  if FForwardClasses=AValue then Exit;
  FForwardClasses.Assign(AValue);
end;

procedure TPasWriter.SetIndentSize(AValue: Integer);
begin
  if AValue=FIndentSize then exit;
  if AValue<0 then
    AValue:=0;
  FIndentSize:=AValue;
  FIndentStep:=StringOfChar(' ',aValue);
end;

function TPasWriter.CheckUnitAlias(const AUnitName: String): String;
begin
  if Assigned(FOnUnitAlias) then
    Result := FOnUnitAlias(AUnitName)
  else
    Result := AUnitName;
end;

function TPasWriter.HasOption(aOption: TPasWriterOption): Boolean;
begin
  Result:=(aOption in FOptions)
end;

function TPasWriter.NotOption(aOption: TPasWriterOption): Boolean;
begin
  Result:=Not (aOption in FOptions)
end;

function TPasWriter.PostProcessLine(S: String): String;
begin
  Result:=S;
  if HasOption(woAddLineNumber) or HasOption(woAddSourceLineNumber) then
    Result:=GetLineNumberComment+Result;
end;

function TPasWriter.GetLineNumberComment: String;

Var
  Ln,OL : string;

begin
  OL:='';
  LN:='';
  if Hasoption(woAddSourceLineNumber) then
    if Assigned(FLineElement) then
      OL:=Format('%.*d',[LineNumberWidth,FLineElement.SourceLinenumber])
    else
      ol:=StringOfChar(' ',LineNumberWidth);
  if HasOption(woAddLineNumber) then
    begin
    LN:=Format('%.*d',[LineNumberWidth,FCurrentLineNumber]);
    if OL<>'' then
      OL:=' '+OL
    end;
  Result:='{ '+LN+OL+' }';
end;

procedure TPasWriter.ResetIndent;

Var
  I : integer;
  E : PDeclSectionStackElement;

begin
  CurDeclSection:='';
  Indent:='';
  For I:=DeclSectionStack.Count-1 downto 0 do
    begin
    E:=PDeclSectionStackElement(DeclSectionStack[i]);
    Dispose(E);
    end;
  DeclSectionStack.Clear;
end;

procedure WritePasFile(AElement: TPasElement; const AFilename: string);
var
  Stream: TFileStream;
begin
  Stream := TFileStream.Create(AFilename, fmCreate or fmShareDenyNone);
  try
    WritePasFile(AElement, Stream);
  finally
    Stream.Free;
  end;
end;

procedure WritePasFile(AElement: TPasElement; AStream: TStream);
var
  Writer: TPasWriter;
begin
  Writer := TPasWriter.Create(AStream);
  try
    Writer.WriteElement(AElement);
  finally
    Writer.Free;
  end;
end;

end.
